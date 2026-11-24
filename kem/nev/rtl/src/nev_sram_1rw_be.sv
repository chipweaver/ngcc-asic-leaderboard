// nev_sram_1rw_be: single-port synchronous SRAM wrapper with byte write enables.
// Simulation: behavioural array. Synthesis (SYNTHESIS defined): black-box CACTI-modelled macro.
module nev_sram_1rw_be #(
  parameter int DEPTH = 384,
  parameter int NBYTE = 8,
  parameter int AW    = 9
) (
  input  logic               clk,
  input  logic               cs,
  input  logic               we,
  input  logic [AW-1:0]      addr,
  input  logic [NBYTE-1:0]   be,
  input  logic [8*NBYTE-1:0] wdata,
  output logic [8*NBYTE-1:0] rdata
);
`ifdef SYNTHESIS
  if (DEPTH == 384 && NBYTE == 8) begin : g_m
    SRAM1RWBE_384X64 u_macro (.CLK(clk), .CS(cs), .WE(we), .A(addr), .BE(be), .D(wdata), .Q(rdata));
  end else if (DEPTH == 192 && NBYTE == 16) begin : g_m2
    SRAM1RWBE_192X128 u_macro (.CLK(clk), .CS(cs), .WE(we), .A(addr), .BE(be), .D(wdata), .Q(rdata));
  end else begin : g_bad
    NEV_SRAM_SHAPE_WITHOUT_MACRO u_bad ();   // unresolved on purpose: no CACTI macro for this shape
  end
`else
  logic [8*NBYTE-1:0] mem [0:DEPTH-1];
  always_ff @(posedge clk) begin
    if (cs) begin
      if (we) begin
        for (int i = 0; i < NBYTE; i++) if (be[i]) mem[addr][8*i +: 8] <= wdata[8*i +: 8];
      end else rdata <= mem[addr];
    end
  end
`endif
endmodule
