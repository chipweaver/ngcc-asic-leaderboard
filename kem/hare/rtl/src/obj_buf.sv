// Object buffer: 64-bit words with byte write enables, built from two 1rw macros
// (low macro 2^AW0 words, high macro 2^AW1 words starting at address 2^AW0).
// Read latency 1 cycle.
module obj_buf #(
  parameter int AW0 = 11,   // key buffer: 2048 + 1024 words; CT buffer: 4096 + 1024 words
  parameter int AW1 = 10
) (
  input  logic        clk,
  input  logic        ce,
  input  logic        we,
  input  logic [12:0] a,
  input  logic [63:0] d,
  input  logic [7:0]  be,
  output logic [63:0] q
);
  logic hi, hi_q;
  logic [63:0] q0, q1;
  assign hi = (a >= (13'd1 << AW0));
  logic [12:0] a1;
  assign a1 = a - (13'd1 << AW0);
  sram_sp #(.AW(AW0), .DW(64), .BW(8)) u_lo (.clk, .ce(ce & ~hi), .we, .a(a[AW0-1:0]), .d, .wm(be), .q(q0));
  sram_sp #(.AW(AW1), .DW(64), .BW(8)) u_hi (.clk, .ce(ce & hi), .we, .a(a1[AW1-1:0]), .d, .wm(be), .q(q1));
  always_ff @(posedge clk) if (ce & ~we) hi_q <= hi;
  assign q = hi_q ? q1 : q0;
endmodule
