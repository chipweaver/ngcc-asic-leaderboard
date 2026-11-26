// plc_codec_dig — 512-Star CRT digit extraction (poly_decompress of 512-Star poly.c).
// For a 48-bit word x returns the five C digits  d_j = (x / 769^j) % 769  (j = 0..4), i.e. exactly what
// the C loop "c = tmp % 769; tmp /= 769" (uint64) produces, also for malformed x >= 769^5.
// Fully pipelined (one x per cycle); x is the caller's registered input, dg is valid LAT = 9 cycles later.
//   s1: carry-save  x[47:19] * M1,  M1 = floor(2^48/591361)
//   s2: qh  = bits [57:29] of the product                    (q - qh in {0,1,2}, q = x / 769^2)
//   s3: r0  = x - qh*591361 (mod 2^21)
//   s4: two corrections in parallel                         -> Q2 = x / 769^2 (29 b), R2 = x % 769^2
//   s5: qa = (R2 * 1363) >> 20 ; carry-save Q2 * 698141
//   s6: qb = bits [48:29] of Q2 * 698141                      (q - qh in {0,1} for both)
//   s7: (d1,d0) = divmod(R2,769), (Q3,d2) = divmod(Q2,769)  (remainder and remainder-769 in parallel)
//   s8: qc = (Q3 * 1363) >> 20
//   s9: (Q4,d3) = divmod(Q3,769), d4 = Q4 mod 769 (Q4 <= 804)
// The constant multiplications are the generated carry-save functions of plc_codec_pkg.
module plc_codec_dig
  import plc_codec_pkg::*;
(
  input  logic             clk,
  input  logic [47:0]      x,
  output logic [4:0][9:0]  dg
);
  localparam logic [20:0] D2 = 21'd591361;

  // divmod-by-769 finish: y and q (low 12 bits), true remainder y - q*769 in [0, 1538)
  function automatic logic [10:0] fin769(input logic [11:0] y, input logic [11:0] q);  // {inc, r[9:0]}
    logic [10:0] r;
    logic [11:0] t;
    r = rem769(y[10:0], q[10:0]);
    t = rem769m(y, q);
    return t[11] ? {1'b0, r[9:0]} : {1'b1, t[9:0]};
  endfunction

  // ---- s1, s2 ----
  logic [57:0] s1_ps, s1_pc;
  logic [20:0] s1_xl, s2_xl;
  logic [28:0] s2_qh;
  always_ff @(posedge clk) begin
    logic [115:0] p;
    logic [57:0]  t;
    p     = mul_m1_cs(x[47:19]);
    s1_ps <= p[57:0];
    s1_pc <= p[115:58];
    s1_xl <= x[20:0];
    t     = s1_ps + s1_pc;
    s2_qh <= t[57:29];
    s2_xl <= s1_xl;
  end

  // ---- s3 ----
  logic [28:0] s3_qh;
  logic [20:0] s3_r0;
  always_ff @(posedge clk) begin
    s3_r0 <= rem591361(s2_xl, s2_qh[20:0]);
    s3_qh <= s2_qh;
  end

  // ---- s4 ----
  logic [28:0] s4_q2;
  logic [19:0] s4_r2;
  always_ff @(posedge clk) begin
    logic ge1, ge2;
    ge1 = (s3_r0 >= D2);
    ge2 = (s3_r0 >= 21'(2 * 591361));
    s4_r2 <= 20'(ge2 ? s3_r0 - 21'(2 * 591361) : ge1 ? s3_r0 - D2 : s3_r0);
    s4_q2 <= s3_qh + {27'd0, ge2, ge1 & !ge2};
  end

  // ---- s5, s6 ----
  logic [9:0]  s5_qa, s6_qa;
  logic [48:0] s5_ps, s5_pc;
  logic [11:0] s5_ya, s5_yb, s6_ya, s6_yb;
  logic [19:0] s6_qb;
  always_ff @(posedge clk) begin
    logic [30:0] pa;
    logic [97:0] pb;
    logic [48:0] t;
    pa = mul_1363(s4_r2);
    pb = mul_698141_cs(s4_q2);
    s5_qa <= pa[29:20];          // R2 < 591361 -> quotient < 769
    s5_ps <= pb[48:0];
    s5_pc <= pb[97:49];
    s5_ya <= s4_r2[11:0];
    s5_yb <= s4_q2[11:0];
    t     = s5_ps + s5_pc;
    s6_qb <= t[48:29];
    s6_qa <= s5_qa;
    s6_ya <= s5_ya;
    s6_yb <= s5_yb;
  end

  // ---- s7 ----
  logic [9:0]  s7_d0, s7_d1, s7_d2;
  logic [19:0] s7_q3;
  always_ff @(posedge clk) begin
    logic [10:0] fa, fb;
    fa = fin769(s6_ya, {2'b00, s6_qa});
    fb = fin769(s6_yb, s6_qb[11:0]);
    s7_d0 <= fa[9:0];
    s7_d1 <= s6_qa + {9'd0, fa[10]};
    s7_d2 <= fb[9:0];
    s7_q3 <= s6_qb + {19'd0, fb[10]};
  end

  // ---- s8 ----
  logic [9:0]  s8_qc, s8_d0, s8_d1, s8_d2;
  logic [11:0] s8_yc;
  always_ff @(posedge clk) begin
    logic [30:0] pc;
    pc = mul_1363(s7_q3);
    s8_qc <= pc[29:20];          // Q3 < 618959 -> quotient <= 804
    s8_yc <= s7_q3[11:0];
    s8_d0 <= s7_d0; s8_d1 <= s7_d1; s8_d2 <= s7_d2;
  end

  // ---- s9 ----
  always_ff @(posedge clk) begin
    logic [10:0] fc;
    logic [9:0]  d4a, d4b;
    fc  = fin769(s8_yc, {2'b00, s8_qc});
    d4a = (s8_qc >= 10'd769) ? 10'(s8_qc - 10'd769) : s8_qc;               // Q4 = qc
    d4b = (s8_qc >= 10'd768) ? 10'(s8_qc - 10'd768) : 10'(s8_qc + 10'd1);  // Q4 = qc + 1
    dg[3] <= fc[9:0];
    dg[4] <= fc[10] ? d4b : d4a;
    dg[0] <= s8_d0; dg[1] <= s8_d1; dg[2] <= s8_d2;
  end
endmodule
