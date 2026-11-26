// Two-port (1 read + 1 write) synchronous SRAM wrapper with per-byte write enables.
// Interface rule: read data is valid the cycle after (ren, raddr) is sampled and holds until the next
// read; a same-address read and write in one cycle is not used by the design.
// Simulation (SYNTHESIS undefined): behavioural array; a same-address read+write returns X and reports.
// Synthesis (SYNTHESIS defined): instantiates the hard-macro black box `plc_bb_1r1w_be_<DEPTH>x64`
// (ports CLK, REN, RA, Q, WEN, WA, BE, D), intentionally not defined in the RTL (black box with a
// CACTI-derived model). Shapes used: 316x64 (PK), 836x64 (SK), 372x64 (CT).
module plc_sram_1r1w_be #(
  parameter int DEPTH = 320,
  parameter int NB    = 8,                 // bytes per word
  parameter int AW    = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
  input  logic              clk,
  input  logic              ren,
  input  logic [AW-1:0]     raddr,
  output logic [8*NB-1:0]   rdata,
  input  logic              wen,
  input  logic [AW-1:0]     waddr,
  input  logic [NB-1:0]     wbe,
  input  logic [8*NB-1:0]   wdata
);
`ifdef SYNTHESIS
  if (DEPTH == 316 && NB == 8) begin : g_m
    plc_bb_1r1w_be_316x64 u_m (.CLK(clk), .REN(ren), .RA(raddr), .Q(rdata), .WEN(wen), .WA(waddr), .BE(wbe), .D(wdata));
  end else if (DEPTH == 836 && NB == 8) begin : g_m
    plc_bb_1r1w_be_836x64 u_m (.CLK(clk), .REN(ren), .RA(raddr), .Q(rdata), .WEN(wen), .WA(waddr), .BE(wbe), .D(wdata));
  end else if (DEPTH == 372 && NB == 8) begin : g_m
    plc_bb_1r1w_be_372x64 u_m (.CLK(clk), .REN(ren), .RA(raddr), .Q(rdata), .WEN(wen), .WA(waddr), .BE(wbe), .D(wdata));
  end else begin : g_m
    plc_bb_1r1w_be_unsupported_shape u_m ();
  end
`else
  logic [8*NB-1:0] mem [0:DEPTH-1];
  always_ff @(posedge clk) begin
    if (ren) rdata <= mem[raddr];
    if (wen)
      for (int b = 0; b < NB; b++)
        if (wbe[b]) mem[waddr][8*b +: 8] <= wdata[8*b +: 8];
    if (ren && wen && raddr == waddr) begin
      rdata <= 'x;
      $error("plc_sram_1r1w_be %m: same-address read/write collision at %0d", raddr);
    end
  end
`endif
endmodule
