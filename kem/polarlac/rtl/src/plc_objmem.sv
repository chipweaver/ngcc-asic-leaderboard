// Object buffers PK/SK/CT (64-bit words, byte-enable writes) with host and engine access.
// Idle (busy=0): host writes (in_*) and host reads (out_*, latency exactly 2 edges).
// Busy: codec read/write ports (always granted) and the hash read port (granted when the codec
// does not read the same buffer). Internal read data are valid 2 cycles after the request.
module plc_objmem (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              busy,
  // host write
  input  logic              in_valid,
  input  logic [1:0]        in_obj,
  input  logic [9:0]        in_addr,
  input  logic [63:0]       in_data,
  // host read
  input  logic              out_req,
  input  logic [1:0]        out_obj,
  input  logic [9:0]        out_addr,
  output logic              out_valid,
  output logic [63:0]       out_data,
  input  logic [511:0]      ss_reg,
  // codec port
  input  logic [2:0]        cd_oren,
  input  logic [2:0][9:0]   cd_oraddr,
  output logic [2:0][63:0]  cd_ordata,
  input  logic [2:0]        cd_owen,
  input  logic [2:0][9:0]   cd_owaddr,
  input  logic [2:0][7:0]   cd_owbe,
  input  logic [2:0][63:0]  cd_owdata,
  // hash read port
  input  logic              hs_oreq,
  input  logic [1:0]        hs_obuf,
  input  logic [9:0]        hs_oaddr,
  output logic              hs_ognt,
  output logic [63:0]       hs_ordata
);
  localparam int DPK = 316, DSK = 836, DCT = 372;

  function automatic logic [9:0] depth(input int b);
    return (b == 0) ? 10'(DPK) : (b == 1) ? 10'(DSK) : 10'(DCT);
  endfunction

  logic [2:0]       ren, wen;
  logic [2:0][9:0]  raddr, waddr;
  logic [2:0][7:0]  wbe;
  logic [2:0][63:0] wdata, rdata;

  logic host_rd, host_wr, host_byp;
  assign host_rd  = out_req && !busy && (out_obj != 2'd3) && (out_addr < depth(int'(out_obj)));
  assign host_wr  = in_valid && !busy && (in_obj != 2'd3) && (in_addr < depth(int'(in_obj)));
  // Same-cycle host write and read of the same word: the read returns the newly written word
  // (write-first, consistent with "a write accepted on an edge is visible from that edge"); the
  // SRAM read is suppressed so the 1R1W macro never sees a same-address read/write.
  assign host_byp = host_rd && host_wr && (in_obj == out_obj) && (in_addr == out_addr);
  assign hs_ognt = busy && !cd_oren[hs_obuf];

  always_comb begin
    for (int b = 0; b < 3; b++) begin
      if (!busy) begin
        ren[b]   = host_rd && !host_byp && (out_obj == 2'(b));
        raddr[b] = out_addr;
        wen[b]   = in_valid && (in_obj == 2'(b)) && (in_addr < depth(b));
        waddr[b] = in_addr;
        wbe[b]   = 8'hFF;
        wdata[b] = in_data;
      end else begin
        ren[b]   = cd_oren[b] || (hs_oreq && hs_obuf == 2'(b));
        raddr[b] = cd_oren[b] ? cd_oraddr[b] : hs_oaddr;
        wen[b]   = cd_owen[b];
        waddr[b] = cd_owaddr[b];
        wbe[b]   = cd_owbe[b];
        wdata[b] = cd_owdata[b];
      end
    end
  end

  plc_sram_1r1w_be #(.DEPTH(DPK), .NB(8)) u_pk (
    .clk(clk), .ren(ren[0]), .raddr(raddr[0][8:0]), .rdata(rdata[0]),
    .wen(wen[0]), .waddr(waddr[0][8:0]), .wbe(wbe[0]), .wdata(wdata[0]));
  plc_sram_1r1w_be #(.DEPTH(DSK), .NB(8)) u_sk (
    .clk(clk), .ren(ren[1]), .raddr(raddr[1]), .rdata(rdata[1]),
    .wen(wen[1]), .waddr(waddr[1]), .wbe(wbe[1]), .wdata(wdata[1]));
  plc_sram_1r1w_be #(.DEPTH(DCT), .NB(8)) u_ct (
    .clk(clk), .ren(ren[2]), .raddr(raddr[2][8:0]), .rdata(rdata[2]),
    .wen(wen[2]), .waddr(waddr[2][8:0]), .wbe(wbe[2]), .wdata(wdata[2]));

  // ---------------- internal read return ----------------
  logic [2:0] ren_q;
  logic [1:0] hs_buf_q1, hs_buf_q2;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ren_q <= '0; hs_buf_q1 <= '0; hs_buf_q2 <= '0;
    end else begin
      ren_q     <= ren & {3{busy}};
      hs_buf_q1 <= hs_obuf;
      hs_buf_q2 <= hs_buf_q1;
    end
  end
  logic [2:0][63:0] rdq;
  always_ff @(posedge clk) begin
    for (int b = 0; b < 3; b++) if (ren_q[b]) rdq[b] <= rdata[b];
  end
  assign cd_ordata = rdq;
  always_comb begin
    hs_ordata = rdq[0];
    if (hs_buf_q2 == 2'd1) hs_ordata = rdq[1];
    if (hs_buf_q2 == 2'd2) hs_ordata = rdq[2];
  end

  // ---------------- host read pipeline (exactly 2 edges) ----------------
  logic       h_v1, h_in1, h_byp1;
  logic [63:0] h_bd1;
  logic [1:0] h_obj1;
  logic [2:0] h_ssw1;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      h_v1 <= 1'b0; h_in1 <= 1'b0; h_obj1 <= '0; h_ssw1 <= '0; h_byp1 <= 1'b0;
      out_valid <= 1'b0;
    end else begin
      h_v1   <= out_req && !busy;
      h_obj1 <= out_obj;
      h_in1  <= (out_obj == 2'd3) ? (out_addr < 10'd8) : (out_addr < depth(int'(out_obj)));
      h_ssw1 <= out_addr[2:0];
      h_byp1 <= host_byp;
      out_valid <= h_v1;
    end
  end
  always_ff @(posedge clk) begin
    if (host_byp) h_bd1 <= in_data;
    if (h_v1) begin
      if (!h_in1)              out_data <= '0;
      else if (h_obj1 == 2'd3) out_data <= ss_reg[64*h_ssw1 +: 64];
      else if (h_byp1)         out_data <= h_bd1;
      else                     out_data <= rdata[h_obj1];
    end
  end
endmodule
