// plc_pwalu - pointwise ALU, one 128-bit word (8 coefficients) per cycle.
// Bit-exact with the reference C (int16 wrap, Montgomery rl_kem_mod_q of each set):
//   mode 0 MULACC: d = mod_q((acc_zero ? 0 : c) + pw(a,b)[t])   pw = mq_poly_pointwise_mul (ntt.c)
//   mode 1 ADD   : d = mod_q(c + a)
//   mode 2 MSG   : x = c + RATIO*cw[t] - Q ; d = x + ((x>>15)&Q)                  (pke.c Encrypt)
//   mode 3 DSUB  : q=257: su = c + ((c>>15)&Q) ; q=769: su = mod_q(c)
//                  h = a - su ; d = h + ((h>>15)&Q)                               (pke.c Decrypt)
// mod_q: q=257 mred(x)+Q if <0 ; q=769 mred(x*171)+Q if <0 (512-Star ntt.h); for all 17-bit
// arguments both equal the canonical residue x mod Q, which is what the tail computes.
//
// pw (ntt.c base_mul): word w holds degree-D blocks, D = 2/4/8 for N = 256/512/1024.
//   level 1: c_i = fqmul(a_i,b_i), m_ij = fqmul(a_i+a_j, b_i+b_j) (i<j in a block),
//            s_ij = m_ij - c_i - c_j; for D=2 instead s_{01} = fqmul(a0,b1) + fqmul(a1,b0).
//   lane t (local t' in its block): r_t = L_t + fqmul(K_t, zeta_t) with
//     L_t = sum_{i<j, i+j=t'} s_ij + (t' even ? c_{t'/2} : 0)
//     K_t = sum_{i<j, i+j=t'+D} s_ij + c_{(t'+D)/2} (t'+D even)     (K_t = 0 -> fqmul = 0)
//   zetas: N=1024 +-f[64+w/2] (minus for odd w); N=512 +f[64+w] (lanes 0-3), -f[64+w] (4-7);
//          N=256 +f[64+2w], -f[64+2w], +f[64+2w+1], -f[64+2w+1] for lane pairs 01,23,45,67.
//   All additions are modulo 2^16 exactly as the C int16 stores (fqmul arguments are int16).
//
// Schedule (word issued at cycle e): A,B read at e (mode 0); C (and A for modes 1,3) read at
// e+8; data 2 cycles after ren; D written at e+13. With slot_d == slot_c the C read of word
// w+5 and the write of word w share a cycle -> never the same bank address.
// start -> done = NW + 13 cycles. cw must be held stable while busy (not latched).
// Pipeline (cycle of the word): e+2 S0 bank select + operand pre-add (registered),
// e+3 S1 16x16 multiply, e+4 S2 Montgomery first half, e+5 S3 second half, e+6 S4a K/L
// carry-save trees, e+7 S4b K/L final add + select, zetas, e+8 S5 K*zeta, e+9 S6 Montgomery
// first half, e+10 S7 r = L + fqmul, x = c + r (late C/A data), e+11 S8 canonical mod q,
// e+12 S9 DSUB (q=769) final, e+13 write.
module plc_pwalu
  import plc_pkg::*;
  import plc_rom_pkg::*;
  import plc_ntt_pwalu_pkg::*;
  import plc_pwalu_rom_pkg::*;
(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              start,
  input  logic [1:0]        mode,
  input  logic              acc_zero,
  input  logic [2:0]        pcode,
  input  logic [3:0]        slot_a,
  input  logic [3:0]        slot_b,
  input  logic [3:0]        slot_c,
  input  logic [3:0]        slot_d,
  input  logic [1023:0]     cw,
  output logic              busy,
  output logic              done,
  // read port A
  output logic [3:0]        a_rslot,
  output logic [1:0]        a_ren,
  output logic [1:0][5:0]   a_raddr,
  input  logic [1:0][127:0] a_rdata,
  // read port B
  output logic [3:0]        b_rslot,
  output logic [1:0]        b_ren,
  output logic [1:0][5:0]   b_raddr,
  input  logic [1:0][127:0] b_rdata,
  // read port C
  output logic [3:0]        c_rslot,
  output logic [1:0]        c_ren,
  output logic [1:0][5:0]   c_raddr,
  input  logic [1:0][127:0] c_rdata,
  // write port D
  output logic [3:0]        d_wslot,
  output logic [1:0]        d_wen,
  output logic [1:0][5:0]   d_waddr,
  output logic [1:0][127:0] d_wdata
);
  localparam int LATE = 8;    // late read offset (C, and A in modes 1 and 3)
  localparam int LAT  = 13;   // issue -> write

  // ---------------- configuration ----------------
  logic [1:0] mode_r, nc_r;
  logic       az_r, q_r;
  logic [3:0] sa_r, sb_r, sc_r, sd_r;
  assign a_rslot = sa_r;
  assign b_rslot = sb_r;
  assign c_rslot = sc_r;
  assign d_wslot = sd_r;

  // ---------------- issue counter ----------------
  logic       run;
  logic [6:0] w;
  logic [6:0] wmax;
  assign wmax = (nc_r == 2'd0) ? 7'd31 : (nc_r == 2'd1) ? 7'd63 : 7'd127;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run <= 1'b0; w <= '0; busy <= 1'b0;
      mode_r <= '0; nc_r <= '0; az_r <= 1'b0; q_r <= 1'b0;
      sa_r <= '0; sb_r <= '0; sc_r <= '0; sd_r <= '0;
    end else begin
      if (start && !busy) begin
        busy <= 1'b1; run <= 1'b1; w <= '0;
        mode_r <= mode; nc_r <= p_ncode(pcode); az_r <= acc_zero; q_r <= p_q769(pcode);
        sa_r <= slot_a; sb_r <= slot_b; sc_r <= slot_c; sd_r <= slot_d;
      end else begin
        if (run) begin
          w <= w + 7'd1;
          if (w == wmax) run <= 1'b0;
        end
        if (done) busy <= 1'b0;
      end
    end
  end

  // per-word metadata pipeline: vld[n]/wq[n]/bq[n] visible in cycle e+n+1 (bq = bank = ^w)
  logic [LAT-1:0] vld;
  logic [LAT-1:0] lst;
  logic [LAT-1:0] bq;
  logic [6:0]     wq [0:LAT-1];
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      vld <= '0; lst <= '0;
    end else begin
      vld <= {vld[LAT-2:0], run};
      lst <= {lst[LAT-2:0], run & (w == wmax)};
    end
  end
  always_ff @(posedge clk) begin
    wq[0] <= w;
    bq    <= {bq[LAT-2:0], ^w};
    for (int i = 1; i < LAT; i++) wq[i] <= wq[i-1];
  end

  // ---------------- read issue ----------------
  logic       early_v, late_v, wb, lb;
  logic [6:0] wl;
  assign early_v = run & (mode_r == 2'd0);
  assign late_v  = vld[LATE-1];          // word issued at e, now cycle e+LATE
  assign wl      = wq[LATE-1];
  assign wb      = ^w;
  assign lb      = bq[LATE-1];
  always_comb begin
    logic use_a_late, use_c;
    use_a_late = mode_r[0];                    // ADD (1) and DSUB (3) read A late; MSG reads no A
    use_c      = (mode_r != 2'd0) | ~az_r;
    a_ren = '0; b_ren = '0; c_ren = '0;
    a_raddr[0] = (mode_r != 2'd0) ? wl[6:1] : w[6:1];
    a_raddr[1] = a_raddr[0];
    b_raddr[0] = w[6:1];  b_raddr[1] = w[6:1];
    c_raddr[0] = wl[6:1]; c_raddr[1] = wl[6:1];
    if (early_v) begin
      a_ren[wb] = 1'b1;
      b_ren[wb] = 1'b1;
    end
    if (late_v) begin
      if (use_a_late) a_ren[lb] = 1'b1;
      if (use_c)      c_ren[lb] = 1'b1;
    end
  end

  // =============== MULACC datapath ===============
  // multiplier index: 0..7 -> c_i ; pair (i<j) -> 8 + pidx(i,j)
  function automatic int pidx(input int i, input int j);
    // i<j ; row-major over the upper triangle
    return i * 7 - (i * (i - 1)) / 2 + (j - i - 1);
  endfunction
  localparam int NM = 36;

  // S0 (e+2): select words, level-1 operands (registered)
  logic [15:0] ae [0:7], be [0:7];
  logic [15:0] opa [0:NM-1], opb [0:NM-1];
  always_comb begin
    logic [127:0] aw, bw;
    aw = bq[1] ? a_rdata[1] : a_rdata[0];
    bw = bq[1] ? b_rdata[1] : b_rdata[0];
    for (int k = 0; k < 8; k++) begin
      ae[k] = aw[16*k +: 16];
      be[k] = bw[16*k +: 16];
    end
    for (int i = 0; i < 8; i++) begin
      opa[i] = ae[i];
      opb[i] = be[i];
    end
    for (int i = 0; i < 8; i++)
      for (int j = i + 1; j < 8; j++) begin
        if (nc_r == 2'd0 && j == i + 1 && i % 2 == 0) begin
          // D=2: fqmul(a[2p], b[2p+1]) on multiplier (2p,2p+1)
          opa[8 + pidx(i, j)] = ae[i];
          opb[8 + pidx(i, j)] = be[j];
        end else if (nc_r == 2'd0 && i % 2 == 0 && (j == i + 2 || (i == 0 && j == 6))) begin
          // D=2: fqmul(a[2p+1], b[2p]) on spare multipliers (0,2)->p0 (2,4)->p1 (4,6)->p2 (0,6)->p3
          if (i == 0 && j == 6) begin
            opa[8 + pidx(i, j)] = ae[7];
            opb[8 + pidx(i, j)] = be[6];
          end else begin
            opa[8 + pidx(i, j)] = ae[i + 1];
            opb[8 + pidx(i, j)] = be[i];
          end
        end else begin
          opa[8 + pidx(i, j)] = ae[i] + ae[j];
          opb[8 + pidx(i, j)] = be[i] + be[j];
        end
      end
  end

  logic [15:0] opa_q [0:NM-1], opb_q [0:NM-1];
  always_ff @(posedge clk) begin
    for (int m = 0; m < NM; m++) begin
      opa_q[m] <= opa[m];
      opb_q[m] <= opb[m];
    end
  end

  // S1 (e+3): multiply
  logic signed [31:0] prd [0:NM-1];
  always_ff @(posedge clk) begin
    for (int m = 0; m < NM; m++) prd[m] <= $signed(opa_q[m]) * $signed(opb_q[m]);
  end

  // S2 (e+4) / S3 (e+5): Montgomery reduction in two halves
  mrd_t        prd_d [0:NM-1];
  logic [15:0] mr [0:NM-1];
  always_ff @(posedge clk) begin
    for (int m = 0; m < NM; m++) prd_d[m] <= mred_d(prd[m], q_r);
    for (int m = 0; m < NM; m++) mr[m]    <= mred_f(prd_d[m], q_r);
  end

  // S4a (e+6): K_t, L_t carry-save trees over m_ij, ~c_i, ~c_j, c_i and a constant
  // s_ij = m_ij - c_i - c_j = m_ij + ~c_i + ~c_j + 2 (mod 2^16)
  logic [31:0] K8 [0:7], L8 [0:7], K4 [0:7], L4 [0:7];   // {carry, sum}
  logic [15:0] K2 [0:7], L2 [0:7];
  always_comb begin
    logic [15:0] oK [0:15], oL [0:15];
    logic [15:0] cK, cL;
    int nK, nL, bb, tl, xs;
    for (int t = 0; t < 8; t++) begin
      // ---- D = 8 ----
      for (int z = 0; z < 16; z++) begin oK[z] = '0; oL[z] = '0; end
      nK = 0; nL = 0; cK = '0; cL = '0;
      for (int i = 0; i < 8; i++)
        for (int j = i + 1; j < 8; j++) begin
          if (i + j == t) begin
            oL[nL] = mr[8 + pidx(i, j)]; oL[nL+1] = ~mr[i]; oL[nL+2] = ~mr[j]; nL = nL + 3; cL = cL + 16'd2;
          end
          if (i + j == t + 8) begin
            oK[nK] = mr[8 + pidx(i, j)]; oK[nK+1] = ~mr[i]; oK[nK+2] = ~mr[j]; nK = nK + 3; cK = cK + 16'd2;
          end
        end
      if (t % 2 == 0) begin
        oL[nL] = mr[t / 2];       nL = nL + 1;
        oK[nK] = mr[(t + 8) / 2]; nK = nK + 1;
      end
      oL[nL] = cL; oK[nK] = cK;
      L8[t] = wsum16_cs(oL);
      K8[t] = wsum16_cs(oK);
      // ---- D = 4 (blocks at lanes 0 and 4) ----
      bb = t & 4; tl = t & 3;
      for (int z = 0; z < 16; z++) begin oK[z] = '0; oL[z] = '0; end
      nK = 0; nL = 0; cK = '0; cL = '0;
      for (int i = 0; i < 4; i++)
        for (int j = i + 1; j < 4; j++) begin
          if (i + j == tl) begin
            oL[nL] = mr[8 + pidx(bb + i, bb + j)]; oL[nL+1] = ~mr[bb + i]; oL[nL+2] = ~mr[bb + j];
            nL = nL + 3; cL = cL + 16'd2;
          end
          if (i + j == tl + 4) begin
            oK[nK] = mr[8 + pidx(bb + i, bb + j)]; oK[nK+1] = ~mr[bb + i]; oK[nK+2] = ~mr[bb + j];
            nK = nK + 3; cK = cK + 16'd2;
          end
        end
      if (tl % 2 == 0) begin
        oL[nL] = mr[bb + tl / 2];       nL = nL + 1;
        oK[nK] = mr[bb + (tl + 4) / 2]; nK = nK + 1;
      end
      oL[nL] = cL; oK[nK] = cK;
      L4[t] = wsum16_cs(oL);
      K4[t] = wsum16_cs(oK);
    end
    // ---- D = 2 (blocks at lanes 0,2,4,6): r0 = c0 + fqmul(c1,z), r1 = x01 + x10 ----
    for (int p = 0; p < 4; p++) begin
      xs = (p == 3) ? (8 + pidx(0, 6)) : (8 + pidx(2 * p, 2 * p + 2));
      L2[2*p]   = mr[2*p];
      K2[2*p]   = mr[2*p + 1];
      L2[2*p+1] = mr[8 + pidx(2*p, 2*p + 1)] + mr[xs];
      K2[2*p+1] = '0;
    end
  end

  // S4a register: carry-save pair of the selected D (K2/L2 carried as {value, 0})
  logic [31:0] k_cs [0:7], l_cs [0:7];
  always_ff @(posedge clk) begin
    for (int t = 0; t < 8; t++) begin
      k_cs[t] <= (nc_r == 2'd2) ? K8[t] : (nc_r == 2'd1) ? K4[t] : {K2[t], 16'd0};
      l_cs[t] <= (nc_r == 2'd2) ? L8[t] : (nc_r == 2'd1) ? L4[t] : {L2[t], 16'd0};
    end
  end

  // S4b (e+7): final add
  logic [15:0] Kt [0:7], Lt [0:7];
  always_comb begin
    for (int t = 0; t < 8; t++) begin
      Kt[t] = k_cs[t][31:16] + k_cs[t][15:0];
      Lt[t] = l_cs[t][31:16] + l_cs[t][15:0];
    end
  end

  // zetas (index from wq[5] registered at e+6, ROM read at e+7)
  logic [6:0] zi0, zi1;
  logic       zodd;
  always_ff @(posedge clk) begin
    case (nc_r)
      2'd0:    begin zi0 <= 7'd64 + {wq[5][5:0], 1'b0}; zi1 <= 7'd64 + {wq[5][5:0], 1'b1}; end
      2'd1:    begin zi0 <= 7'd64 + wq[5][5:0];         zi1 <= 7'd64 + wq[5][5:0]; end
      default: begin zi0 <= 7'd64 + {1'b0, wq[5][6:1]}; zi1 <= 7'd64 + {1'b0, wq[5][6:1]}; end
    endcase
    zodd <= wq[5][0];
  end
  logic signed [15:0] zf0, zf1;
  assign zf0 = q_r ? zeta769_f(zi0) : zeta257_f(zi0);
  assign zf1 = q_r ? zeta769_f(zi1) : zeta257_f(zi1);

  logic [15:0]        s4_K [0:6], s4_L [0:7];
  logic signed [15:0] s4_z [0:6];
  always_ff @(posedge clk) begin
    for (int t = 0; t < 7; t++) begin
      logic signed [15:0] z;
      logic neg;
      z = (nc_r == 2'd0 && t >= 4) ? zf1 : zf0;
      case (nc_r)
        2'd0:    neg = (t % 4 >= 2);
        2'd1:    neg = (t >= 4);
        default: neg = zodd;
      endcase
      s4_z[t] <= neg ? -z : z;
      s4_K[t] <= Kt[t];
    end
    for (int t = 0; t < 8; t++) s4_L[t] <= Lt[t];
  end

  // S5 (e+8): K*zeta (lane 7 always has K = 0)
  logic signed [31:0] s5_m [0:6];
  logic [15:0]        s5_L [0:7];
  always_ff @(posedge clk) begin
    for (int t = 0; t < 7; t++) s5_m[t] <= $signed(s4_K[t]) * s4_z[t];
    for (int t = 0; t < 8; t++) s5_L[t] <= s4_L[t];
  end

  // S6 (e+9): Montgomery first half; codeword bits for the word
  mrd_t        s6_d [0:6];
  logic [15:0] s6_L [0:7];
  logic [7:0]  s6_cw;
  always_ff @(posedge clk) begin
    for (int t = 0; t < 7; t++) s6_d[t] <= mred_d(s5_m[t], q_r);
    for (int t = 0; t < 8; t++) s6_L[t] <= s5_L[t];
    s6_cw <= cw[8*wq[8] +: 8];
  end

  // =============== common tail ===============
  // S7 (e+10): late read data (C, A) valid
  logic [15:0] cl [0:7], al [0:7];
  always_comb begin
    logic [127:0] aw, cwd;
    aw  = bq[9] ? a_rdata[1] : a_rdata[0];
    cwd = bq[9] ? c_rdata[1] : c_rdata[0];
    for (int k = 0; k < 8; k++) begin
      al[k] = aw[16*k +: 16];
      cl[k] = cwd[16*k +: 16];
    end
  end

  // S7 (e+10): r = L + fqmul(K,zeta); x = 17-bit rl_kem_mod_q argument (modes 0, 1, 3/769)
  //           or a 16-bit value (MSG: c + RATIO*cw - Q ; DSUB/257: a - (c + ((c>>15)&Q)))
  logic [16:0] s7_x [0:7];
  logic [15:0] s7_a [0:7];
  always_ff @(posedge clk) begin
    for (int t = 0; t < 8; t++) begin
      logic [31:0] r, m, h;
      logic [15:0] cz;
      r  = (t < 7) ? {16'd0, mred_f_add(s6_L[t], s6_d[t > 6 ? 6 : t], q_r)} : {16'd0, s6_L[t]};
      cz = az_r ? 16'd0 : cl[t];
      m  = add3({16'd0, cl[t]}, s6_cw[t] ? (q_r ? 32'd385 : 32'd129) : 32'd0, q_r ? -32'sd769 : -32'sd257);
      h  = add3({16'd0, al[t]}, {16'd0, ~cl[t]}, cl[t][15] ? (32'd1 - 32'd257) : 32'd1);
      case (mode_r)
        2'd0:    s7_x[t] <= {r[15], r[15:0]} + {cz[15], cz};
        2'd1:    s7_x[t] <= {al[t][15], al[t]} + {cl[t][15], cl[t]};
        2'd2:    s7_x[t] <= {1'b0, m[15:0]};
        default: s7_x[t] <= q_r ? {cl[t][15], cl[t]} : {1'b0, h[15:0]};
      endcase
      s7_a[t] <= al[t];
    end
  end

  // S8 (e+11): canonical reduction. rl_kem_mod_q(x) == x mod Q in [0,Q) for every 17-bit x
  // (both q, see header of plc_ntt_pwalu_pkg): x = 256*xh + xl, T = (256*xh) mod Q
  // (q=257: x == xl - xh, corrected by +-257 ; q=769: T from ROM, t = T + xl < 2Q, minus Q if >= Q).
  // MSG / DSUB-257: v + ((v>>15)&Q).
  logic [15:0] s8_v [0:7], s8_a [0:7];
  always_ff @(posedge clk) begin
    for (int t = 0; t < 8; t++) begin
      logic [8:0]  xh;
      logic [7:0]  xl;
      logic [9:0]  tt;
      logic [11:0] t0, tp, tm, u0, u1;
      logic [15:0] v16, cr;
      xh = s7_x[t][16:8];
      xl = s7_x[t][7:0];
      // q=257: x == xl - xh (mod 257), xl - xh in [-255, 511]
      t0 = {4'd0, xl} - {{3{xh[8]}}, xh};
      tp = {4'd0, xl} - {{3{xh[8]}}, xh} + 12'd257;
      tm = {4'd0, xl} - {{3{xh[8]}}, xh} - 12'd257;
      // q=769: T = (256*xh) mod 769 (ROM), T + xl in [0, 1023]
      tt = t769_256(xh);
      u0 = {2'd0, tt} + {4'd0, xl};
      u1 = {2'd0, tt} + {4'd0, xl} - 12'd769;
      if (q_r) cr = u1[11] ? {4'd0, u0} : {4'd0, u1};
      else     cr = t0[11] ? {4'd0, tp} : (tm[11] ? {4'd0, t0} : {4'd0, tm});
      v16 = s7_x[t][15:0];
      if (mode_r == 2'd2 || (mode_r == 2'd3 && !q_r)) s8_v[t] <= addq_if_neg(v16, q_r);
      else                                          s8_v[t] <= cr;
      s8_a[t] <= s7_a[t];
    end
  end

  // S9 (e+12): DSUB/769: h = a - su ; d = h + ((h>>15)&Q)  (both candidates, select on sign)
  logic [127:0] s9_d;
  always_ff @(posedge clk) begin
    for (int t = 0; t < 8; t++) begin
      logic [31:0] h0, h1;
      h0 = add3({16'd0, s8_a[t]}, {16'd0, ~s8_v[t]}, 32'd1);
      h1 = add3({16'd0, s8_a[t]}, {16'd0, ~s8_v[t]}, 32'd770);
      if (mode_r == 2'd3 && q_r) s9_d[16*t +: 16] <= h0[15] ? h1[15:0] : h0[15:0];
      else                       s9_d[16*t +: 16] <= s8_v[t];
    end
  end

  // write (e+13)
  logic [6:0] ww;
  assign ww = wq[LAT-1];
  always_comb begin
    d_wen             = '0;
    d_wen[bq[LAT-1]]  = vld[LAT-1];
    d_waddr[0]        = ww[6:1];
    d_waddr[1]        = ww[6:1];
    d_wdata[0]        = s9_d;
    d_wdata[1]        = s9_d;
  end
  assign done = vld[LAT-1] & lst[LAT-1];

endmodule
