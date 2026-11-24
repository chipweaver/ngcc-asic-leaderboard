// nev_sram_1rw: single-port synchronous SRAM wrapper. One access per cycle: write when cs & we,
// read when cs & !we; read data valid after the clock edge.
// Simulation: behavioural array. Synthesis (SYNTHESIS defined): black-box macro whose Liberty model
// is derived from the CACTI 90 nm estimate of this shape.
module nev_sram_1rw #(
  parameter int DEPTH = 32,
  parameter int WIDTH = 192,
  parameter int AW    = 5
) (
  input  logic             clk,
  input  logic             cs,
  input  logic             we,
  input  logic [AW-1:0]    addr,
  input  logic [WIDTH-1:0] wdata,
  output logic [WIDTH-1:0] rdata
);
`ifdef SYNTHESIS
  if (DEPTH == 32 && WIDTH == 192) begin : g_m
    SRAM1RW_32X192 u_macro (.CLK(clk), .CS(cs), .WE(we), .A(addr), .D(wdata), .Q(rdata));
  end else begin : g_bad
    NEV_SRAM_SHAPE_WITHOUT_MACRO u_bad ();   // unresolved on purpose: no CACTI macro for this shape
  end
`else
  logic [WIDTH-1:0] mem [0:DEPTH-1];
  always_ff @(posedge clk) begin
    if (cs) begin
      if (we) mem[addr] <= wdata;
      else    rdata     <= mem[addr];
    end
  end
`endif
endmodule
