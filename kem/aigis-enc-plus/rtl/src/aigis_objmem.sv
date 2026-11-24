// Object buffers PK (512x64), SK (1024x64), CT in 1r1w SRAM, SS in registers.  CT is two physical
// buffers: words 0..319 in CTU (512x64), words 320..447 in CTV (128x64), so writers of the two
// regions (Enc III u and v) and readers (Dec III) proceed in parallel.  Physical buffer index:
// 0 PK, 1 SK, 2 CTU, 3 CTV.
// Host port (only while idle): synchronous, host_rdata valid one cycle after a read.
// Internal clients (while busy): per-buffer fixed-priority arbitration (lower client index first),
// grant in the request cycle, SRAM inputs registered, read data (rv/rdata) two cycles after grant.
//   read clients : 0 hash ch0, 1 hash ch1, 2 hash ch2 (reject KDF, lowest priority), 3 copy engine, 4 st_pk, 5 st_sk, 6 st_ct
//   write clients: 0 copy engine, 1 st_pk, 2 st_sk, 3 st_ct, 4 hash subsystem (H(pk) -> SK)
// pkw[i] = PK word i written internally during the current command (cleared on clr).
module aigis_objmem #(
  parameter int NRC = 7,
  parameter int NWC = 4
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          busy,
  input  logic          clr,
  // host
  input  logic          host_en,
  input  logic          host_we,
  input  logic [11:0]   host_addr,
  input  logic [63:0]   host_wdata,
  output logic [63:0]   host_rdata,
  // internal read clients
  input  logic [NRC-1:0]       r_req,
  input  logic [NRC-1:0][1:0]  r_sel,
  input  logic [NRC-1:0][9:0]  r_addr,
  output logic [NRC-1:0]       r_gnt,
  output logic [NRC-1:0]       r_rv,
  output logic [NRC-1:0][63:0] r_data,
  // internal write clients
  input  logic [NWC-1:0]       w_req,
  input  logic [NWC-1:0][1:0]  w_sel,
  input  logic [NWC-1:0][9:0]  w_addr,
  input  logic [NWC-1:0][63:0] w_data,
  output logic [NWC-1:0]       w_gnt,
  // SS registers (written by the copy engine)
  input  logic          ss_we,
  input  logic [2:0]    ss_idx,
  input  logic [63:0]   ss_data,
  output logic [327:0]  pkw,
  // mirror: st_pk (write client 1) PK writes also go to SK at word mirror_off + addr
  input  logic          mirror_en,
  input  logic [9:0]    mirror_off
);
  // ---------------------------------------------------------------- arbitration
  function automatic logic [1:0] phys(input logic [1:0] sel, input logic [9:0] a);
    phys = (sel == 2'd2 && a >= 10'd320) ? 2'd3 : sel;
  endfunction
  function automatic logic [9:0] paddr(input logic [1:0] sel, input logic [9:0] a);
    paddr = (sel == 2'd2 && a >= 10'd320) ? a - 10'd320 : a;
  endfunction
  logic [NRC-1:0][1:0] r_ph;
  logic [NWC-1:0][1:0] w_ph;
  always_comb begin
    for (int c = 0; c < NRC; c++) r_ph[c] = phys(r_sel[c], r_addr[c]);
    for (int c = 0; c < NWC; c++) w_ph[c] = phys(w_sel[c], w_addr[c]);
  end
  logic [3:0]            rb_v;      // buffer read granted
  logic [3:0][9:0]       rb_a;
  logic [3:0][2:0]       rb_c;      // client index
  logic [3:0]            wb_v;
  logic [3:0][9:0]       wb_a;
  logic [3:0][63:0]      wb_d;
  always_comb begin
    r_gnt = '0; w_gnt = '0; rb_v = '0; rb_a = '0; rb_c = '0; wb_v = '0; wb_a = '0; wb_d = '0;
    if (busy) begin
      for (int b = 0; b < 4; b++) begin
        // read priority: lower client index first, except the reject-KDF channel (client 2), which
        // is served only when no other client requests the buffer (it is off the critical path)
        if (r_req[2] && r_ph[2] == 2'(b)) begin rb_v[b] = 1'b1; rb_a[b] = paddr(r_sel[2], r_addr[2]); rb_c[b] = 3'd2; end
        for (int c = NRC - 1; c >= 0; c--)
          if (c != 2 && r_req[c] && r_ph[c] == 2'(b)) begin rb_v[b] = 1'b1; rb_a[b] = paddr(r_sel[c], r_addr[c]); rb_c[b] = 3'(c); end
        for (int c = NWC - 1; c >= 0; c--)
          if (w_req[c] && w_ph[c] == 2'(b)) begin wb_v[b] = 1'b1; wb_a[b] = paddr(w_sel[c], w_addr[c]); wb_d[b] = w_data[c]; end
      end
      for (int c = 0; c < NRC; c++) r_gnt[c] = r_req[c] && rb_v[r_ph[c]] && rb_c[r_ph[c]] == 3'(c);
      for (int c = 0; c < NWC; c++) begin
        logic hi;
        hi = 1'b0;
        for (int d = 0; d < c; d++) if (w_req[d] && w_ph[d] == w_ph[c]) hi = 1'b1;
        // the mirrored st_pk write needs the SK port too (priority above st_sk / st_ct)
        if (c == 1 && mirror_en && w_req[0] && w_ph[0] == 2'd1) hi = 1'b1;
        if (c > 1 && mirror_en && w_req[1] && w_ph[1] == 2'd0 && w_ph[c] == 2'd1) hi = 1'b1;
        w_gnt[c] = w_req[c] && !hi;
      end
      if (mirror_en && w_gnt[1] && w_ph[1] == 2'd0) begin
        wb_v[1] = 1'b1; wb_a[1] = w_addr[1] + mirror_off; wb_d[1] = w_data[1];
      end
    end
  end

  // registered internal SRAM inputs
  logic [3:0]       ir_v, iw_v;
  logic [3:0][9:0]  ir_a, iw_a;
  logic [3:0][2:0]  ir_c;
  logic [3:0][63:0] iw_d;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin ir_v <= '0; iw_v <= '0; end
    else begin ir_v <= rb_v; iw_v <= wb_v; end
  end
  always_ff @(posedge clk) begin ir_a <= rb_a; ir_c <= rb_c; iw_a <= wb_a; iw_d <= wb_d; end

  // host decode (capacities PK 328, SK 728, CT 448, SS 8 words)
  logic [1:0] hobj; logic [9:0] hw; logic hin;
  assign hobj = host_addr[11:10];
  assign hw   = host_addr[9:0];
  always_comb begin
    case (hobj)
      2'd0: hin = (hw < 10'd328);
      2'd1: hin = (hw < 10'd728);
      2'd2: hin = (hw < 10'd448);
      default: hin = (hw < 10'd8);
    endcase
  end
  logic hacc, hrdy;   // host port ignored while reset is active
  always_ff @(posedge clk or negedge rst_n) if (!rst_n) hrdy <= 1'b0; else hrdy <= 1'b1;
  assign hacc = host_en && !busy && hin && hrdy;

  // SRAM port muxes
  logic [1:0] hph; logic [9:0] hpa;
  assign hph = (hobj == 2'd3) ? 2'd0 : phys(hobj, hw);
  assign hpa = paddr(hobj, hw);
  logic [3:0]       m_re, m_we;
  logic [3:0][9:0]  m_ra, m_wa;
  logic [3:0][63:0] m_d, m_q;
  always_comb begin
    for (int b = 0; b < 4; b++) begin
      if (busy) begin
        m_re[b] = ir_v[b]; m_ra[b] = ir_a[b];
        m_we[b] = iw_v[b]; m_wa[b] = iw_a[b]; m_d[b] = iw_d[b];
      end else begin
        m_re[b] = hacc && !host_we && hobj != 2'd3 && hph == 2'(b); m_ra[b] = hpa;
        m_we[b] = hacc &&  host_we && hobj != 2'd3 && hph == 2'(b); m_wa[b] = hpa; m_d[b] = host_wdata;
      end
    end
  end
  SRAM1R1W_512X64  u_pk  (.CLK(clk), .RE(m_re[0]), .RA(m_ra[0][8:0]), .Q(m_q[0]), .WE(m_we[0]), .WA(m_wa[0][8:0]), .D(m_d[0]));
  SRAM1R1W_1024X64 u_sk  (.CLK(clk), .RE(m_re[1]), .RA(m_ra[1]),      .Q(m_q[1]), .WE(m_we[1]), .WA(m_wa[1]),      .D(m_d[1]));
  SRAM1R1W_512X64  u_ct  (.CLK(clk), .RE(m_re[2]), .RA(m_ra[2][8:0]), .Q(m_q[2]), .WE(m_we[2]), .WA(m_wa[2][8:0]), .D(m_d[2]));
  SRAM1R1W_128X64  u_ctv (.CLK(clk), .RE(m_re[3]), .RA(m_ra[3][6:0]), .Q(m_q[3]), .WE(m_we[3]), .WA(m_wa[3][6:0]), .D(m_d[3]));

  // internal read data return (cycle after the registered read)
  logic [3:0]      q_v;
  logic [3:0][2:0] q_c;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) q_v <= '0; else q_v <= busy ? ir_v : 4'b0000;
  end
  always_ff @(posedge clk) q_c <= ir_c;
  always_comb begin
    r_rv = '0; r_data = '0;
    for (int b = 0; b < 4; b++)
      if (q_v[b]) begin r_rv[q_c[b]] = 1'b1; r_data[q_c[b]] = m_q[b]; end
  end

  // SS registers
  logic [7:0][63:0] ss;
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) ss <= '0; else if (ss_we) ss[ss_idx] <= ss_data;

  // host read data
  logic h_rv; logic [1:0] h_obj, h_ph; logic [2:0] h_ssi;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) h_rv <= 1'b0; else h_rv <= hacc && !host_we;
  end
  always_ff @(posedge clk) begin h_obj <= hobj; h_ph <= hph; h_ssi <= hw[2:0]; end
  always_comb begin
    if (!h_rv) host_rdata = '0;
    else if (h_obj == 2'd3) host_rdata = ss[h_ssi];
    else host_rdata = m_q[h_ph];
  end

  // PK written bitmap
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) pkw <= '0;
    else if (clr) pkw <= '0;
    else if (wb_v[0] && wb_a[0] < 10'd328) pkw[wb_a[0]] <= 1'b1;
  end
endmodule
