// Job descriptors and the per-operation job tables (in dependency order).
package aigis_ctrl_pkg;
  import aigis_pkg::*;
  localparam int NJ = 25;
  // units
  localparam logic [3:0] U_RX = 4'd0, U_HA = 4'd1, U_HB = 4'd2, U_CP = 4'd3, U_GA = 4'd4, U_NX = 4'd5,
                         U_NTT = 4'd6, U_MAC = 4'd7, U_WP = 4'd8, U_SPK = 4'd9, U_SSK = 4'd10, U_SCT = 4'd11;
  // regfile words
  localparam logic [9:0] R_MSG = 10'd0, R_H2O = 10'd16, R_Z = 10'd64, R_KB = 10'd72, R_SEED = 10'd80;
  // stream modes: {dir, fmt[1:0], dec, dual}
  localparam logic [4:0] ST_L10D  = 5'b0_00_1_0, ST_L10DD = 5'b0_00_1_1, ST_L12 = 5'b0_01_0_0,
                         ST_LC2R  = 5'b0_10_0_0, ST_S10 = 5'b1_00_0_0, ST_S10M = 5'b1_00_0_1, ST_S12 = 5'b1_01_0_0, ST_SC2 = 5'b1_10_0_0,
                         ST_S10C = 5'b1_00_1_0, ST_SC2C = 5'b1_10_1_0;   // store-compare
  // copy kinds
  localparam logic [2:0] K_REG = 3'd0, K_PK = 3'd1, K_SK = 3'd2, K_CT = 3'd3, K_SS = 3'd4;

  typedef struct packed {
    logic        v;
    logic [3:0]  unit;
    logic [NJ-1:0] deps;
    logic [NJ-1:0] sdeps;      // jobs that must have started (follow-mode consumers)
    logic [4:0]  mode;
    logic [5:0][2:0] s;
    logic [9:0]  a1, a2, cnt;
    logic [2:0]  k1, k2;
  } job_t;

  function automatic job_t J(input logic [3:0] unit, input logic [NJ-1:0] deps, input logic [4:0] mode,
                             input logic [2:0] s0, input logic [2:0] s1, input logic [2:0] s2,
                             input logic [2:0] s3, input logic [2:0] s4, input logic [2:0] s5,
                             input logic [9:0] a1, input logic [9:0] a2, input logic [9:0] cnt,
                             input logic [2:0] k1, input logic [2:0] k2);
    J.v = 1'b1; J.unit = unit; J.deps = deps; J.sdeps = '0; J.mode = mode;
    J.s[0] = s0; J.s[1] = s1; J.s[2] = s2; J.s[3] = s3; J.s[4] = s4; J.s[5] = s5;
    J.a1 = a1; J.a2 = a2; J.cnt = cnt; J.k1 = k1; J.k2 = k2;
  endfunction
  function automatic logic [NJ-1:0] D(input int a, input int b, input int c, input int d);
    D = '0;
    if (a >= 0) D[a] = 1'b1;
    if (b >= 0) D[b] = 1'b1;
    if (c >= 0) D[c] = 1'b1;
    if (d >= 0) D[d] = 1'b1;
  endfunction

  function automatic job_t [NJ-1:0] job_table(input logic [1:0] op, input logic [1:0] p, input logic fail);
    job_t [NJ-1:0] t;
    logic [9:0] bw, pkw, pkseed, skpk, skpkseed, skhpk, skz, ctv;
    logic [2:0] es, ee;
    t = '0;
    bw       = 10'(bbytes(p) >> 3);
    pkw      = 10'(pk_bytes(p) >> 3);
    pkseed   = 10'(polyc10_bytes(p) >> 3);
    skpk     = 10'(poly12_bytes(p) >> 3);
    skpkseed = skpk + pkseed;
    skhpk    = 10'((sk_bytes(p) - 13'(2 * bbytes(p))) >> 3);
    skz      = 10'((sk_bytes(p) - 13'(bbytes(p))) >> 3);
    ctv      = pkseed;
    es = eta_s(p); ee = eta_e(p);
    case (op)
      2'd0: begin  // KeyGen
        t[0]  = J(U_RX,  D(-1, -1, -1, -1),      5'd0, 0,0,0,0,0,0, R_MSG, 10'd0, bw, 3'd0, 3'd0);
        t[1]  = J(U_RX,  D(0, -1, -1, -1),     5'd0, 0,0,0,0,0,0, R_Z,   10'd0, bw, 3'd0, 3'd0);
        t[2]  = J(U_HA,  D(-1, -1, -1, -1),    5'd0, 0,0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);          // Hash2(d): reads d as it arrives
        t[3]  = J(U_NX,  D(2, -1, -1, -1),     5'd0, 3'd0,0,0,0,0,0, R_H2O, 10'd0, 10'd0, 3'd0, es);          // s
        t[4]  = J(U_GA,  D(2, -1, -1, -1),     5'd0, 3'd1,0,0,0,0,0, R_H2O + bw, 10'd0, 10'd0, 3'd0, 3'd0);   // a
        t[5]  = J(U_NTT, D(3, -1, -1, -1),     5'd0, 3'd0,0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);
        t[6]  = J(U_MAC, D(5, -1, -1, -1),   5'd1, 3'd1,3'd0,3'd3,3'd5,3'd6,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);  // a o s (passes) -> slot 3
        t[6].sdeps = D(4, -1, -1, -1);
        t[7]  = J(U_NX,  D(3, -1, -1, -1),     5'd0, 3'd2,0,0,0,0,0, R_H2O, 10'd0, 10'd0, 3'd1, ee);          // e
        t[8]  = J(U_NTT, D(-1, -1, -1, -1),    5'd7, 3'd3,3'd4,3'd6,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);   // INTT follows MAC: 3 -> 4, final pass -> 6
        t[8].sdeps = D(6, -1, -1, -1);
        t[9]  = J(U_SPK, D(7, -1, -1, -1),   ST_S10M, 3'd6,3'd2,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);  // pk poly, follows the INTT final pass
        t[9].sdeps = D(8, -1, -1, -1);
        t[10] = J(U_CP,  D(2, -1, -1, -1),     5'd0, 0,0,0,0,0,0, R_H2O + bw, pkseed, bw, K_REG, K_PK);       // publicseed -> PK
        t[11] = J(U_HA,  D(2, -1, -1, -1),     5'd3, 0,0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);           // H(pk), wait
        t[12] = J(U_SSK, D(6, -1, -1, -1),     ST_S12, 3'd0,3'd0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);  // sk poly
        t[13] = J(U_CP,  D(2, -1, -1, -1),     5'd0, 0,0,0,0,0,0, R_H2O + bw, skpkseed, bw, K_REG, K_SK);   // publicseed -> SK (pk poly mirrored by SPK)
        // H(pk) -> SK is copied by the hash subsystem at the end of job 11
        t[15] = J(U_CP,  D(1, -1, -1, -1),     5'd0, 0,0,0,0,0,0, R_Z, skz, bw, K_REG, K_SK);                 // z
      end
      2'd1: begin  // Enc
        t[0]  = J(U_RX,  D(-1, -1, -1, -1),      5'd0, 0,0,0,0,0,0, R_MSG, 10'd0, bw, 3'd0, 3'd0);
        t[1]  = J(U_CP,  D(-1, -1, -1, -1),      5'd0, 0,0,0,0,0,0, pkseed, R_SEED, bw, K_PK, K_REG);
        t[2]  = J(U_GA,  D(1, -1, -1, -1),     5'd0, 3'd1,0,0,0,0,0, R_SEED, 10'd0, 10'd0, 3'd0, 3'd0);
        t[3]  = J(U_HA,  D(-1, -1, -1, -1),      5'd1, 0,0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);           // H(pk)
        t[4]  = J(U_SPK, D(-1, -1, -1, -1),      ST_L10D, 3'd3,3'd3,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);
        t[5]  = J(U_NTT, D(4, -1, -1, -1),     5'd0, 3'd3,0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);
        t[6]  = J(U_HA,  D(0, 3, -1, -1),   5'd4, 0,0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);           // Hash2(m||H(pk))
        t[7]  = J(U_NX,  D(6, 2, -1, -1),   5'd0, 3'd0,0,0,0,0,0, R_H2O + bw, 10'd0, 10'd0, 3'd0, es);     // r
        t[8]  = J(U_NTT, D(7, -1, -1, -1),     5'd4, 3'd0,3'd0,3'd5,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);   // NTT r, final pass -> slot 5
        t[9]  = J(U_NX,  D(7, -1, -1, -1),     5'd0, 3'd2,0,0,0,0,0, R_H2O + bw, 10'd0, 10'd0, 3'd1, ee);     // e1
        t[10] = J(U_NX,  D(9, -1, -1, -1),     5'd0, 3'd4,0,0,0,0,0, R_H2O + bw, 10'd0, 10'd0, 3'd2, ee);     // e2
        t[11] = J(U_MAC, D(2, -1, -1, -1), 5'd6, 3'd1,3'd5,3'd6,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);   // A o R follows NTT r -> slot 6
        t[11].sdeps = D(8, -1, -1, -1);
        t[12] = J(U_NTT, D(-1, -1, -1, -1),    5'd3, 3'd6,3'd0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);   // INTT U follows A o R: 6 -> 0
        t[12].sdeps = D(11, -1, -1, -1);
        t[13] = J(U_NTT, D(-1, -1, -1, -1),    5'd3, 3'd1,3'd6,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);   // INTT V follows T o R: 1 -> 6
        t[13].sdeps = D(17, -1, -1, -1);
        t[17] = J(U_MAC, D(5, 8, 11, -1), 5'd4, 3'd3,3'd5,3'd1,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);   // T o R -> slot 1
        t[14] = J(U_SPK, D(12, 9, -1, -1),  ST_S10, 3'd0,3'd2,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd2, 3'd0);   // u pack into CT on u_spk
        t[15] = J(U_SCT, D(13, 10, -1, -1), ST_SC2, 3'd6,3'd4,0,0,0,0, ctv, 10'd0, 10'd0, 3'd0, 3'd0);
        t[16] = J(U_CP,  D(6, -1, -1, -1),     5'd0, 0,0,0,0,0,0, R_H2O, 10'd0, bw, K_REG, K_SS);             // ss = K
      end
      default: begin  // Dec
        t[0]  = J(U_SCT, D(-1, -1, -1, -1),      ST_L10D, 3'd2,3'd2,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);  // U
        t[1]  = J(U_SSK, D(-1, -1, -1, -1),      ST_L12, 3'd0,3'd0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);   // s^
        t[2]  = J(U_NTT, D(0, -1, -1, -1),     5'd0, 3'd2,0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);
        t[3]  = J(U_MAC, D(1, 2, -1, -1),   5'd0, 3'd2,3'd0,3'd2,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);
        t[4]  = J(U_NTT, D(3, -1, -1, -1),     5'd1, 3'd2,0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);
        t[5]  = J(U_SCT, D(0, -1, -1, -1),     ST_LC2R, 3'd6,3'd6,0,0,0,0, ctv, 10'd0, 10'd0, 3'd0, 3'd0);    // Vd
        t[6]  = J(U_WP,  D(4, 5, -1, -1),   5'd0, 3'd2,3'd6,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);     // tomsg
        t[7]  = J(U_CP,  D(-1, -1, -1, -1),      5'd0, 0,0,0,0,0,0, skhpk, R_MSG + bw, bw, K_SK, K_REG);
        t[8]  = J(U_CP,  D(-1, -1, -1, -1),      5'd0, 0,0,0,0,0,0, skpkseed, R_SEED, bw, K_SK, K_REG);
        t[9]  = J(U_HA,  D(6, 7, -1, -1),   5'd4, 0,0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);           // Hash2(m'||H(pk))
        t[10] = J(U_GA,  D(8, -1, -1, -1),     5'd0, 3'd1,0,0,0,0,0, R_SEED, 10'd0, 10'd0, 3'd0, 3'd0);
        t[11] = J(U_SSK, D(1, -1, -1, -1),     ST_L10D, 3'd3,3'd3,0,0,0,0, skpk, 10'd0, 10'd0, 3'd0, 3'd0);   // t
        t[12] = J(U_NTT, D(11, 4, -1, -1),     5'd0, 3'd3,0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);
        t[13] = J(U_NX,  D(9, 10, 3, -1), 5'd0, 3'd0,0,0,0,0,0, R_H2O + bw, 10'd0, 10'd0, 3'd0, es);    // r
        t[14] = J(U_NTT, D(13, -1, -1, -1),    5'd0, 3'd0,0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);
        t[15] = J(U_NX,  D(13, 6, -1, -1),  5'd0, 3'd2,0,0,0,0,0, R_H2O + bw, 10'd0, 10'd0, 3'd1, ee);     // e1
        t[16] = J(U_NX,  D(15, -1, -1, -1),    5'd0, 3'd4,0,0,0,0,0, R_H2O + bw, 10'd0, 10'd0, 3'd2, ee);     // e2
        t[17] = J(U_MAC, D(10, 14, -1, -1), 5'd0, 3'd1,3'd0,3'd1,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);   // A o R (fast)
        t[23] = J(U_MAC, D(12, 14, 17, -1), 5'd0, 3'd3,3'd0,3'd3,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);  // T o R (fast)
        t[18] = J(U_NTT, D(17, -1, -1, -1),    5'd1, 3'd1,0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);
        t[19] = J(U_NTT, D(23, -1, -1, -1),    5'd1, 3'd3,0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);
        t[20] = J(U_SPK, D(18, 15, -1, -1), ST_S10C, 3'd1,3'd2,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd2, 3'd0);  // compare u' = compress10(U'+E1) with ct (CTU)
        t[24] = J(U_SCT, D(19, 16, -1, -1), ST_SC2C, 3'd3,3'd4,0,0,0,0, ctv, 10'd0, 10'd0, 3'd0, 3'd0);    // compare v' = compress_c2(V'+E2-M) with ct
        t[21] = J(U_HB,  D(-1, -1, -1, -1),      5'd0, 0,0,0,0,0,0, 10'd0, 10'd0, 10'd0, 3'd0, 3'd0);           // KDF(z||ct)
        t[22] = J(U_CP,  D(20, 21, 9, 24), 5'd0, 0,0,0,0,0,0, fail ? R_KB : R_H2O, 10'd0, bw, K_REG, K_SS);
      end
    endcase
    job_table = t;
  endfunction
endpackage
