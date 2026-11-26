// Standard two-port (1 read + 1 write) synchronous SRAM wrapper.
// Interface rule: read data is valid the cycle after (ren, raddr) is sampled and holds until the next
// read; a same-address read and write in one cycle is not used by the design.
// Simulation (SYNTHESIS undefined): behavioural array; a same-address read+write returns X and reports.
// Synthesis (SYNTHESIS defined): instantiates the hard-macro black box `plc_bb_1r1w_<DEPTH>x<WIDTH>`
// (ports CLK, REN, RA, Q, WEN, WA, D), which is intentionally not defined in the RTL: synthesis treats
// it as a black box and attaches the CACTI-derived model. Shapes used: 64x128, 128x512, 64x32.
module plc_sram_1r1w #(
  parameter int DEPTH = 64,
  parameter int WIDTH = 128,
  parameter int AW    = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
  input  logic             clk,
  input  logic             ren,
  input  logic [AW-1:0]    raddr,
  output logic [WIDTH-1:0] rdata,
  input  logic             wen,
  input  logic [AW-1:0]    waddr,
  input  logic [WIDTH-1:0] wdata
);
`ifdef SYNTHESIS
  if (DEPTH == 64 && WIDTH == 128) begin : g_m
    plc_bb_1r1w_64x128  u_m (.CLK(clk), .REN(ren), .RA(raddr), .Q(rdata), .WEN(wen), .WA(waddr), .D(wdata));
  end else if (DEPTH == 128 && WIDTH == 512) begin : g_m
    plc_bb_1r1w_128x512 u_m (.CLK(clk), .REN(ren), .RA(raddr), .Q(rdata), .WEN(wen), .WA(waddr), .D(wdata));
  end else if (DEPTH == 64 && WIDTH == 32) begin : g_m
    plc_bb_1r1w_64x32   u_m (.CLK(clk), .REN(ren), .RA(raddr), .Q(rdata), .WEN(wen), .WA(waddr), .D(wdata));
  end else begin : g_m
    plc_bb_1r1w_unsupported_shape u_m ();
  end
`else
  logic [WIDTH-1:0] mem [0:DEPTH-1];
  always_ff @(posedge clk) begin
    if (ren) rdata <= mem[raddr];
    if (wen) mem[waddr] <= wdata;
    if (ren && wen && raddr == waddr) begin
      rdata <= 'x;
      $error("plc_sram_1r1w %m: same-address read/write collision at %0d", raddr);
    end
  end
`endif
endmodule
