// nev_sram_1r1w: behavioural 1-read 1-write synchronous SRAM (replaced by a macro in synthesis).
// Write: mem[waddr] <= wdata on the clock edge when we=1.
// Read : rdata <= mem[raddr] on the clock edge when re=1 (data available the cycle after the
//        address is presented; rdata holds otherwise). Contents are never reset.
module nev_sram_1r1w #(
  parameter int DEPTH = 128,
  parameter int WIDTH = 128
) (
  input  logic                     clk,
  input  logic                     re,
  input  logic [$clog2(DEPTH)-1:0] raddr,
  output logic [WIDTH-1:0]         rdata,
  input  logic                     we,
  input  logic [$clog2(DEPTH)-1:0] waddr,
  input  logic [WIDTH-1:0]         wdata
);
`ifdef SYNTHESIS
  if (DEPTH == 128 && WIDTH == 128) begin : g_m
    SRAM1R1W_128X128 u_macro (.CLK(clk), .RE(re), .RA(raddr), .Q(rdata), .WE(we), .WA(waddr), .D(wdata));
  end else begin : g_bad
    NEV_SRAM_SHAPE_WITHOUT_MACRO u_bad ();   // unresolved on purpose: no CACTI macro for this shape
  end
`else
  logic [WIDTH-1:0] mem [0:DEPTH-1];

  always_ff @(posedge clk) begin
    if (we) mem[waddr] <= wdata;
    if (re) rdata <= mem[raddr];
  end
`endif
endmodule
