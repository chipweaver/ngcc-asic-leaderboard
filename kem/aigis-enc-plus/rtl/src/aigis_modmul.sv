// Pipelined modular multiplier: r = a * b mod q (q = 3329), latency 3 cycles, one result per cycle.
// a, b canonical (< q; every caller passes canonical operands or the constant 1).  Barrett reduction
// with a 13-bit constant: t = floor(x * 5039 / 2^24) with 5039 = floor(2^24 / q) underestimates
// floor(x / q) by at most 1 for every x < 2^24 (checked exhaustively: x - t*q <= 5713 < 2q), so one
// conditional subtraction gives the canonical result; t is kept in 12 bits, exact for x < q^2.
module aigis_modmul (
  input  logic        clk,
  input  logic [11:0] a,
  input  logic [11:0] b,
  output logic [11:0] r
);
  logic [23:0] x1, x2;
  logic [11:0] t2;
  always_ff @(posedge clk) begin
    x1 <= a * b;
    x2 <= x1;
    t2 <= 12'((({13'd0, x1}) * 37'd5039) >> 24);
  end
  logic [13:0] d3;
  always_comb d3 = 14'(x2[13:0] - 14'(t2 * 14'd3329));
  always_ff @(posedge clk) r <= (d3 >= 14'd3329) ? 12'(d3 - 14'd3329) : d3[11:0];
endmodule

// Reduction of a 28-bit accumulator: r = x mod q, latency 2 cycles.
// t = floor(x * 80635 / 2^28) (80635 = floor(2^28 / q)) underestimates floor(x / q) by at most 1
// for every x < 2^28 (checked exhaustively: x - t*q <= 4869 < 2q), so one conditional subtraction.
module aigis_modred28 (
  input  logic        clk,
  input  logic [27:0] x,
  output logic [11:0] r
);
  logic [27:0] x1;
  logic [16:0] t1;
  always_ff @(posedge clk) begin
    x1 <= x;
    t1 <= 17'((({17'd0, x} * 45'd80635) >> 28));
  end
  logic [13:0] d2;
  always_comb d2 = 14'(x1[13:0] - 14'(t1 * 17'd3329));
  always_ff @(posedge clk) r <= (d2 >= 14'd3329) ? 12'(d2 - 14'd3329) : d2[11:0];
endmodule
