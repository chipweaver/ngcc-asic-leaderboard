// Arithmetic helpers for plc_ntt / plc_pwalu, bit-exact with ntt.c / ntt.h of each set
// (gcc -fwrapv semantics: every int16 store wraps modulo 2^16).
//
//   montgomery_reduce(a): u = (int16)(a*QINV); t = a - (int32)u*Q; return (int16)(t >> 16)
//     QINV = -255 (q=257) = 1 - 2^8,  QINV = -767 (q=769) = 1 - 3*2^8.
//   rl_kem_mod_q(a): q=257: t = montgomery_reduce(a)            ; t += (t>>15)&Q
//                    q=769: t = montgomery_reduce(a * 171)      ; t += (t>>15)&Q   (512-Star ntt.h)
//     For every 17-bit argument this is the canonical residue a mod Q (|t| < Q before the
//     correction; checked exhaustively), so plc_pwalu reduces canonically.
package plc_ntt_pwalu_pkg;

  // Montgomery reduction of a signed 32-bit value, short-path form (equal to the C for
  // |a| <= 2^30; checked exhaustively/randomly). With qm = Q>>8 (1 or 3)
  // and a = 2^16*hi + 256*h + l (h, l unsigned bytes):
  //   u = (int16)(a*QINV) = 256*U1 + l,  U1 = signed byte of (h - qm*l)
  //   (a - u*Q) >> 16 = hi - qm*U1 + floor((h - qm*l + 128) / 256)
  // Let D = h + 128 - qm*l (11-bit signed), dl = D[7:0], k = D[10:8] (signed), U1 = dl - 128:
  //   result = hi + v,  v = 128*qm - qm*dl + k   (v in [-384, 385])
  // Split for pipelining: mred_d -> {hi, D} (one short carry-propagate);
  // mred_f -> hi + k - dl - q*2dl + (128 + 256q) as one carry-save tree + one adder,
  // i.e. hi + k + ~dl + q*~(2dl) + (129 + 257q) (mod 2^16).
  typedef struct packed {
    logic [15:0] hi;
    logic [10:0] d;
  } mrd_t;

  // ---- explicit carry-save helpers: every multi-operand sum is a 3:2 compressor tree plus
  // ---- one 2-operand adder (keeps synthesis from building ripple chains). Results are mod 2^32.
  function automatic logic [63:0] csa32(input logic [31:0] a, input logic [31:0] b, input logic [31:0] c);
    return {a ^ b ^ c, ((a & b) | (a & c) | (b & c)) << 1};    // {sum, carry}
  endfunction
  function automatic logic [31:0] add3(input logic [31:0] a, input logic [31:0] b, input logic [31:0] c);
    logic [63:0] r;
    r = csa32(a, b, c);
    return r[63:32] + r[31:0];
  endfunction
  function automatic logic [31:0] add4(input logic [31:0] a, input logic [31:0] b, input logic [31:0] c,
                                       input logic [31:0] d);
    logic [63:0] r;
    r = csa32(a, b, c);
    return add3(r[63:32], r[31:0], d);
  endfunction
  // 16-operand 16-bit sum (Wallace order; unused operands must be 0)
  function automatic logic [15:0] wsum16(input logic [15:0] o [0:15]);
    logic [15:0] l1 [0:10], l2 [0:7], l3 [0:5], l4 [0:3], l5 [0:2];
    logic [63:0] r;
    for (int i = 0; i < 5; i++) begin
      r = csa32({16'd0, o[3*i]}, {16'd0, o[3*i+1]}, {16'd0, o[3*i+2]});
      l1[2*i] = r[47:32]; l1[2*i+1] = r[15:0];
    end
    l1[10] = o[15];
    for (int i = 0; i < 3; i++) begin
      r = csa32({16'd0, l1[3*i]}, {16'd0, l1[3*i+1]}, {16'd0, l1[3*i+2]});
      l2[2*i] = r[47:32]; l2[2*i+1] = r[15:0];
    end
    l2[6] = l1[9]; l2[7] = l1[10];
    for (int i = 0; i < 2; i++) begin
      r = csa32({16'd0, l2[3*i]}, {16'd0, l2[3*i+1]}, {16'd0, l2[3*i+2]});
      l3[2*i] = r[47:32]; l3[2*i+1] = r[15:0];
    end
    l3[4] = l2[6]; l3[5] = l2[7];
    for (int i = 0; i < 2; i++) begin
      r = csa32({16'd0, l3[3*i]}, {16'd0, l3[3*i+1]}, {16'd0, l3[3*i+2]});
      l4[2*i] = r[47:32]; l4[2*i+1] = r[15:0];
    end
    r = csa32({16'd0, l4[0]}, {16'd0, l4[1]}, {16'd0, l4[2]});
    l5[0] = r[47:32]; l5[1] = r[15:0]; l5[2] = l4[3];
    r = csa32({16'd0, l5[0]}, {16'd0, l5[1]}, {16'd0, l5[2]});
    return r[47:32] + r[15:0];
  endfunction
  // same tree, carry-save result {carry, sum}: wsum16(o) == cs[31:16] + cs[15:0] (mod 2^16)
  function automatic logic [31:0] wsum16_cs(input logic [15:0] o [0:15]);
    logic [15:0] l1 [0:10], l2 [0:7], l3 [0:5], l4 [0:3], l5 [0:2];
    logic [63:0] r;
    for (int i = 0; i < 5; i++) begin
      r = csa32({16'd0, o[3*i]}, {16'd0, o[3*i+1]}, {16'd0, o[3*i+2]});
      l1[2*i] = r[47:32]; l1[2*i+1] = r[15:0];
    end
    l1[10] = o[15];
    for (int i = 0; i < 3; i++) begin
      r = csa32({16'd0, l1[3*i]}, {16'd0, l1[3*i+1]}, {16'd0, l1[3*i+2]});
      l2[2*i] = r[47:32]; l2[2*i+1] = r[15:0];
    end
    l2[6] = l1[9]; l2[7] = l1[10];
    for (int i = 0; i < 2; i++) begin
      r = csa32({16'd0, l2[3*i]}, {16'd0, l2[3*i+1]}, {16'd0, l2[3*i+2]});
      l3[2*i] = r[47:32]; l3[2*i+1] = r[15:0];
    end
    l3[4] = l2[6]; l3[5] = l2[7];
    for (int i = 0; i < 2; i++) begin
      r = csa32({16'd0, l3[3*i]}, {16'd0, l3[3*i+1]}, {16'd0, l3[3*i+2]});
      l4[2*i] = r[47:32]; l4[2*i+1] = r[15:0];
    end
    r = csa32({16'd0, l4[0]}, {16'd0, l4[1]}, {16'd0, l4[2]});
    l5[0] = r[47:32]; l5[1] = r[15:0]; l5[2] = l4[3];
    r = csa32({16'd0, l5[0]}, {16'd0, l5[1]}, {16'd0, l5[2]});
    return {r[47:32], r[15:0]};
  endfunction

  function automatic logic [31:0] add5(input logic [31:0] a, input logic [31:0] b, input logic [31:0] c,
                                       input logic [31:0] d, input logic [31:0] e);
    logic [63:0] r1, r2;
    r1 = csa32(a, b, c);
    r2 = csa32(r1[63:32], r1[31:0], d);
    return add3(r2[63:32], r2[31:0], e);
  endfunction
  function automatic logic [31:0] add6(input logic [31:0] a, input logic [31:0] b, input logic [31:0] c,
                                       input logic [31:0] d, input logic [31:0] e, input logic [31:0] f);
    logic [63:0] r1, r2, r3;
    r1 = csa32(a, b, c);
    r2 = csa32(d, e, f);
    r3 = csa32(r1[63:32], r1[31:0], r2[63:32]);
    return add3(r3[63:32], r3[31:0], r2[31:0]);
  endfunction

  // stage 1: D = h + 128 - l - q*2l
  function automatic mrd_t mred_d(input logic signed [31:0] a, input logic q769);
    mrd_t r;
    logic [7:0]  h, l;
    logic [31:0] d;
    h    = a[15:8];
    l    = a[7:0];
    d    = add4({24'd0, h}, ~{24'd0, l}, q769 ? ~{23'd0, l, 1'b0} : 32'd0, q769 ? 32'd130 : 32'd129);
    r.d  = d[10:0];
    r.hi = a[31:16];
    return r;
  endfunction

  // stage-2 operands (4 + constant) of t = hi + v, and of -t
  function automatic logic [31:0] mk(input mrd_t r);   // sign-extended k
    return {{29{r.d[10]}}, r.d[10:8]};
  endfunction
  function automatic logic [31:0] mdl1(input mrd_t r);
    return {24'd0, r.d[7:0]};
  endfunction
  function automatic logic [31:0] mdl2(input mrd_t r, input logic q769);
    return q769 ? {23'd0, r.d[7:0], 1'b0} : 32'd0;
  endfunction

  // t = hi + k + ~dl + q*~(2dl) + 129 + 257q
  function automatic logic [15:0] mred_f(input mrd_t r, input logic q769);
    logic [31:0] t;
    t = add5({16'd0, r.hi}, mk(r), ~mdl1(r), q769 ? ~mdl2(r, 1'b1) : 32'd0, q769 ? 32'd386 : 32'd129);
    return t[15:0];
  endfunction
  // p + t
  function automatic logic [15:0] mred_f_add(input logic [15:0] p, input mrd_t r, input logic q769);
    logic [31:0] t;
    t = add6({16'd0, p}, {16'd0, r.hi}, mk(r), ~mdl1(r), q769 ? ~mdl2(r, 1'b1) : 32'd0,
             q769 ? 32'd386 : 32'd129);
    return t[15:0];
  endfunction
  // p - t = p + ~hi + ~k + dl + q*2dl + (3 + q - 129 - 257q)
  function automatic logic [15:0] mred_f_sub(input logic [15:0] p, input mrd_t r, input logic q769);
    logic [31:0] t;
    t = add6({16'd0, p}, ~{16'd0, r.hi}, ~mk(r), mdl1(r), mdl2(r, q769),
             q769 ? -32'sd382 : -32'sd126);
    return t[15:0];
  endfunction

  function automatic logic [15:0] mred(input logic signed [31:0] a, input logic q769);
    return mred_f(mred_d(a, q769), q769);
  endfunction

  function automatic logic [15:0] qv(input logic q769);
    return q769 ? 16'd769 : 16'd257;
  endfunction

  // t += (t >> 15) & Q   (int16)
  function automatic logic [15:0] addq_if_neg(input logic [15:0] t, input logic q769);
    return t + (t[15] ? qv(q769) : 16'd0);
  endfunction

  // fn[127] scaling products: 655*x (512-Star) / 255*x (q=257 sets), as CSA + one adder
  function automatic logic [31:0] mul_fn127(input logic [15:0] v, input logic q769);
    logic [31:0] x, a, b;
    logic [63:0] r1, r2;
    x  = {{16{v[15]}}, v};
    r1 = csa32(x << 9, x << 7, x << 4);               // 655x = 512x+128x+16x - x
    r2 = csa32(r1[63:32], r1[31:0], ~x);
    a  = q769 ? r2[63:32] : ((x << 8) ^ ~x);          // 255x = 256x - x
    b  = q769 ? r2[31:0]  : (((x << 8) & ~x) << 1);
    return a + (b | 32'd1);                            // + 1 completes the two's complement of x
  endfunction

endpackage
