// nev_objmem: object memory. SKF / PK fields are 384 x 64-bit 1rw SRAMs with byte enables, the CT
// field is a 192 x 128-bit 1rw SRAM with 16 byte enables (two 8-byte words per row, word w in row w/2,
// half w%2); a unit may access a whole CT row (om_wide, 16 bytes) per cycle;
// HPK and SS are 8-word register fields. Host port (1-cycle read latency) is served only while the
// top is idle; unit ports (hash, codec A, codec B) have fixed priority per SRAM and a 2-cycle read
// latency (granted in cycle t, data in t+2).
module nev_objmem import nev_pkg::*; #(parameter int NU = 3) (
  input  logic                 clk,
  input  logic                 rst_n,
  input  logic                 busy,
  // host port
  input  logic                 mem_cs,
  input  logic                 mem_we,
  input  logic [10:0]          mem_addr,
  input  logic [7:0]           mem_be,
  input  logic [63:0]          mem_wdata,
  output logic [63:0]          mem_rdata,
  // unit ports (index 0 has highest priority)
  input  logic [NU-1:0]        om_req,
  input  logic [NU-1:0]        om_we,
  input  logic [NU-1:0][1:0]   om_sel,
  input  logic [NU-1:0][8:0]   om_addr,
  input  logic [NU-1:0][127:0] om_wdata,    // narrow accesses use [63:0]
  input  logic [NU-1:0][15:0]  om_be,       // narrow accesses use [7:0]
  input  logic [NU-1:0]        om_wide,     // CT only: whole row (addr even), 16 bytes
  output logic [NU-1:0]        om_gnt,
  output logic [NU-1:0]        om_rvalid,
  output logic [NU-1:0][127:0] om_rdata,    // narrow reads return the word in [63:0]
  // register fields
  input  logic                 hpk_we,
  input  logic [511:0]         hpk_wdata,
  output logic [511:0]         hpk_q,
  input  logic                 ss_we,
  input  logic [511:0]         ss_wdata,
  input  logic                 ss_clr,
  output logic [511:0]         ss_q
);
  // ---------------- host decode
  logic        h_on;
  logic [1:0]  h_sel;          // 0 SKF 1 PK 2 CT 3 regs/none
  logic [8:0]  h_waddr;
  logic        h_reg_hpk, h_reg_ss;
  logic [2:0]  h_regw;
  always_comb begin
    h_on = mem_cs && !busy;
    h_sel = 2'd3; h_waddr = '0; h_reg_hpk = 1'b0; h_reg_ss = 1'b0; h_regw = mem_addr[2:0];
    if (mem_addr < 11'h180)                          begin h_sel = 2'd0; h_waddr = 9'(mem_addr - BASE_SKF); end
    else if (mem_addr < 11'h300)                     begin h_sel = 2'd1; h_waddr = 9'(mem_addr - BASE_PK);  end
    else if (mem_addr >= BASE_HPK && mem_addr < BASE_SS)          h_reg_hpk = 1'b1;
    else if (mem_addr >= BASE_SS && mem_addr < 11'h310)           h_reg_ss  = 1'b1;
    else if (mem_addr >= BASE_CT && mem_addr < 11'h580) begin h_sel = 2'd2; h_waddr = 9'(mem_addr - BASE_CT); end
  end

  // ---------------- per-SRAM port selection
  logic [2:0]        s_cs, s_we;
  logic [2:0][8:0]   s_addr;             // word address (CT: row = addr[8:1])
  logic [2:0][15:0]  s_be;
  logic [2:0][127:0] s_wdata;
  logic [1:0][63:0]  s_q;                // SKF, PK
  logic [127:0]      ct_q;
  logic [2:0][NU-1:0] s_gnt;
  // CT row view of a narrow (word) or wide (row) access
  function automatic logic [15:0] ct_be(input logic wide, input logic half, input logic [15:0] be);
    ct_be = wide ? be : (half ? {be[7:0], 8'h00} : {8'h00, be[7:0]});
  endfunction
  always_comb begin
    om_gnt = '0;
    for (int s = 0; s < 3; s++) begin
      s_cs[s] = 1'b0; s_we[s] = 1'b0; s_addr[s] = '0; s_be[s] = '0; s_wdata[s] = '0; s_gnt[s] = '0;
      if (h_on && h_sel == 2'(s)) begin
        s_cs[s] = 1'b1; s_we[s] = mem_we; s_addr[s] = h_waddr;
        s_be[s] = (s == 2) ? ct_be(1'b0, h_waddr[0], {8'h00, mem_be}) : {8'h00, mem_be};
        s_wdata[s] = {mem_wdata, mem_wdata};
      end else if (busy) begin
        for (int u = NU - 1; u >= 0; u--) begin
          if (om_req[u] && om_sel[u] == 2'(s)) begin
            s_cs[s] = 1'b1; s_we[s] = om_we[u]; s_addr[s] = om_addr[u];
            s_be[s] = (s == 2) ? ct_be(om_wide[u], om_addr[u][0], om_be[u]) : om_be[u];
            s_wdata[s] = om_wide[u] ? om_wdata[u] : {om_wdata[u][63:0], om_wdata[u][63:0]};
            s_gnt[s] = '0; s_gnt[s][u] = 1'b1;
          end
        end
      end
      om_gnt = om_gnt | s_gnt[s];
    end
  end

  for (genvar s = 0; s < 2; s++) begin : g_ram
    nev_sram_1rw_be #(.DEPTH(384), .NBYTE(8), .AW(9)) u_ram (
      .clk(clk), .cs(s_cs[s]), .we(s_we[s]), .addr(s_addr[s]), .be(s_be[s][7:0]), .wdata(s_wdata[s][63:0]), .rdata(s_q[s]));
  end
  nev_sram_1rw_be #(.DEPTH(192), .NBYTE(16), .AW(8)) u_ram_ct (
    .clk(clk), .cs(s_cs[2]), .we(s_we[2]), .addr(s_addr[2][8:1]), .be(s_be[2]), .wdata(s_wdata[2]), .rdata(ct_q));

  // ---------------- unit read return (2-cycle latency)
  logic [NU-1:0]       rd1, rd2;
  logic [NU-1:0][1:0]  rsel1;
  logic [NU-1:0]       rhalf1, rwide1;
  logic [NU-1:0][127:0] rq2;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rd1 <= '0; rd2 <= '0; rsel1 <= '0; rhalf1 <= '0; rwide1 <= '0; rq2 <= '0;
    end else begin
      for (int u = 0; u < NU; u++) begin
        rd1[u]    <= om_gnt[u] && !om_we[u];
        rsel1[u]  <= om_sel[u];
        rhalf1[u] <= om_addr[u][0];
        rwide1[u] <= om_wide[u];
        rd2[u]    <= rd1[u];
        if (rsel1[u] == 2'd2) rq2[u] <= rwide1[u] ? ct_q : {64'd0, (rhalf1[u] ? ct_q[127:64] : ct_q[63:0])};
        else                  rq2[u] <= {64'd0, s_q[rsel1[u][0]]};
      end
    end
  end
  assign om_rvalid = rd2;
  assign om_rdata  = rq2;

  // ---------------- register fields
  logic [7:0][63:0] hpk_r, ss_r;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      hpk_r <= '0; ss_r <= '0;
    end else begin
      if (hpk_we) hpk_r <= hpk_wdata;
      else if (h_on && mem_we && h_reg_hpk)
        for (int i = 0; i < 8; i++) if (mem_be[i]) hpk_r[h_regw][8*i +: 8] <= mem_wdata[8*i +: 8];
      if (ss_clr) ss_r <= '0;
      else if (ss_we) ss_r <= ss_wdata;
      else if (h_on && mem_we && h_reg_ss)
        for (int i = 0; i < 8; i++) if (mem_be[i]) ss_r[h_regw][8*i +: 8] <= mem_wdata[8*i +: 8];
    end
  end
  assign hpk_q = hpk_r;
  assign ss_q  = ss_r;

  // ---------------- host read data (valid after the next edge)
  logic [1:0] hr_sel;     // 0..2 SRAM, 3 reg
  logic       hr_half;
  logic [63:0] hr_reg;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      hr_sel <= 2'd3; hr_half <= 1'b0; hr_reg <= '0;
    end else if (h_on && !mem_we) begin
      hr_sel <= h_sel;
      hr_half <= h_waddr[0];
      hr_reg <= h_reg_hpk ? hpk_r[h_regw] : (h_reg_ss ? ss_r[h_regw] : 64'd0);
    end
  end
  always_comb begin
    case (hr_sel)
      2'd0, 2'd1: mem_rdata = s_q[hr_sel[0]];
      2'd2:       mem_rdata = hr_half ? ct_q[127:64] : ct_q[63:0];
      default:    mem_rdata = hr_reg;
    endcase
  end
endmodule
