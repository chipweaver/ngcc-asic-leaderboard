// nev_seq: operation sequencer of nev_kem_top (KeyGen / Enc / Dec phase FSM).
// Issues commands to the hash unit, sampler, poly ALU (direct or through the inversion
// sub-sequencer) and the two codecs, tracks their completion, and owns busy/done/fail.
// A second small FSM (h preparation) builds the scaled copies of h for the (re-)encryption
// product while the hash unit runs H(pk) (Enc) or Hash2 (Dec).
// Speculation: in KeyGen the next candidate (nonce+1, g distribution) is generated while the
// current f candidate is inverted/judged; it is used only if f is accepted, otherwise discarded
// (the f loop then restarts with nonce+1 once the speculative KDF/sampling has finished).
module nev_seq import nev_pkg::*; import nev_palu_pkg::*; (
  input  logic        clk,
  input  logic        rst_n,
  // command / status
  input  logic        start,
  input  logic [1:0]  op,
  input  logic [3:0]  param,
  input  logic [1:0]  tst_fail_f,
  input  logic [1:0]  tst_fail_g,
  output logic        busy,
  output logic        done,
  output logic        fail,
  output pset_t       ps,             // latched parameter set
  // hash unit
  output logic        h_valid,
  output logic [2:0]  h_op,
  output logic        h_seed_sel,
  output logic [7:0]  h_nonce,
  output logic [6:0]  h_nblocks,
  output logic [7:0]  h_nonce1,
  output logic [6:0]  h_nblocks1,
  output logic [11:0] h_nmlen,
  output logic        h_pkall,        // pk_avail = all words (else from codec B progress)
  output logic        h_rnd_go,       // start the random-byte capture
  input  logic        h_rnd_fin,      // random bytes captured
  input  logic        h_busy,
  input  logic        h_done,
  input  logic        h_done1,        // KDF stream 1 complete (noisem input final)
  // sampler
  output logic        s_valid,
  output logic [3:0]  s_dist,
  output logic        s_region,
  output logic [2:0]  s_dst,
  output logic        s_rinv,
  output logic        s_vinv,
  input  logic        s_busy,
  input  logic        s_done,
  input  logic        s_need,
  // noisem sampler (KDF stream 1)
  output logic        s1_valid,
  output logic [2:0]  s1_dst,
  input  logic        s1_done,
  // poly ALU (direct)
  output logic        p_valid,
  output palu_cmd_t   p_cmd,
  input  logic        p_done,
  input  logic        p_nz,           // PROD compare: re-encryption differs from the ciphertext coefficients
  // inversion sub-sequencer
  output logic        q_start,
  output logic        q_judge,
  output logic        q_inj,
  output logic [2:0]  q_bA, q_bYA0, q_bCA0, q_bT1, q_bT2, q_bYCH, q_bCCH,
  output logic [11:0] q_cval,
  input  logic        q_done,
  input  logic        q_noninv,
  // codecs
  output logic        ca_valid, cb_valid,
  output logic [2:0]  ca_mode, cb_mode,
  output logic [1:0]  ca_field, cb_field,
  output logic [2:0]  ca_buf, cb_buf,
  input  logic        ca_done, cb_done,
  input  logic        ca_mismatch,
  input  logic        cb_ncanon,      // CT unpack: non-canonical ciphertext encoding
  output logic        ca_strm, cb_strm,   // codec reads stream behind the poly ALU result (palu watermark)
  // object memory register fields
  output logic        hpk_we,
  output logic        ss_we,
  output logic        ss_clr
);
  // hash ops
  localparam logic [2:0] H_RND = 3'd0, H_SEED = 3'd1, H_HPK = 3'd2, H_LOADHK = 3'd3, H_HASH2 = 3'd4, H_KDF = 3'd5, H_MORE = 3'd6;
  // codec modes
  localparam logic [2:0] C_UNPACK = 3'd0, C_PACK = 3'd1, C_PACKCMP = 3'd2, C_DECOMP = 3'd3, C_COMP = 3'd4, C_COMPCMP = 3'd5, C_TOMSG = 3'd6;
  localparam logic [1:0] F_SKF = 2'd0, F_PK = 2'd1, F_CT = 2'd2;
  localparam logic [2:0] B0 = 3'd0, B1 = 3'd1, B2 = 3'd2, B3 = 3'd3, B4 = 3'd4, B5 = 3'd5, B6 = 3'd6, B7 = 3'd7;

  typedef enum logic [5:0] {
    S_IDLE,
    K_RND, K_SEED, K_FKDF, K_FNTT, K_FINV, K_FCHK, K_FWAIT, K_GKDF, K_GNTT, K_GINV, K_GCHK,
    K_PSC, K_PROD, K_FIN,
    E_RND, E_H2,
    D_START, D_VNTT, D_FSC, D_PROD, D_INTT, D_H2,
    R_KDF, R_NTT, R_PROD, R_INTT,
    S_FIN, S_DONE
  } st_t;
  st_t st, st_q;

  typedef enum logic [2:0] {HP_IDLE, HP_WAIT, HP_S1, HP_W1, HP_S2, HP_W2, HP_DONE} hp_t;
  hp_t hp;

  logic [1:0]  opq;
  logic [1:0]  tf, tg;          // remaining forced failures
  logic [7:0]  nonce;
  logic        pend_h, pend_s, pend_s1, pend_p, pend_q, pend_a, pend_b;
  logic        noninv_q;
  logic        ncq;           // Dec: ciphertext encoding non-canonical

  // ---------------- derived per-set values
  logic [2:0]  klog;
  logic [11:0] rq, kd, kci;
  always_comb begin
    klog = (ps.d64 ? 3'd3 : 3'd2) + {1'b0, ps.nlog};
    case (ps.qsel)
      Q769:    begin rq = 12'd171;  kd = 12'd763;  kci = 12'd715;  end
      Q1409:   begin rq = 12'd722;  kd = 12'd1387; kci = 12'd39;   end
      default: begin rq = 12'd2285; kd = 12'd3303; kci = 12'd2264; end
    endcase
  end
  // KDF block counts (bias3: initial estimate plus two spare blocks; more are requested on demand).
  // Simulation-only test hook: +NEV_NO_SPARE removes the spare blocks so that rare inputs exercise the
  // KDF extension paths (kdf_need while the job runs, MORE after it ended).
  logic [6:0] spare;
`ifndef SYNTHESIS
  initial spare = $test$plusargs("NEV_NO_SPARE") ? 7'd0 : 7'd2;
`else
  assign spare = 7'd2;
`endif
  function automatic logic [6:0] nblk(input logic [3:0] eta, input logic [1:0] nlog);
    logic [6:0] n128;  // N/128
    n128 = 7'd4 << nlog;
    case (eta)
      4'd1: nblk = n128;                    // N/4 bytes
      4'd2: nblk = n128 << 1;               // N/2
      4'd3: nblk = 7'(n128 * 3);            // 3N/4
      4'd4: nblk = n128 << 2;               // N
      4'd7: nblk = 7'(n128 * 7);            // 7N/4
      4'd8: nblk = 7'((n128 * 3) >> 1);     // 3N/8
      default: nblk = ((nlog == 2'd0) ? 7'd5 : (nlog == 2'd1) ? 7'd8 : 7'd14) + spare;   // bias3 initial + spare
    endcase
  endfunction
  logic [11:0] nmlen;      // (8*eta_e - 1) * S
  logic [6:0]  nmblk;
  always_comb begin
    nmlen = 12'((({4'd0, ps.eta_e} << 3) - 8'd1) * ({5'd0, sbytes(ps.slog)}));
    nmblk = 7'((nmlen + 12'd31) >> 5);
  end

  // NTT result buffers: 6 levels (NTT_DIM 64) end in the source buffer, 7 levels in the other
  logic [2:0] Fb, Fo, Gb, Gx, Rb, Eb, Ib, Ob, Vc, hU, Pout;
  always_comb begin
    Fb = ps.d64 ? B0 : B1; Fo = ps.d64 ? B1 : B0;    // f candidate: sampled into B0, NTT B0 <-> B1
    Gb = ps.d64 ? B7 : B6; Gx = ps.d64 ? B6 : B7;    // g candidate: sampled into B7, NTT B7 <-> B6
    // r: B1 <-> B3 (Enc) / B0 <-> B3 (Dec, B1 keeps the ciphertext coefficients), e: B2 <-> B4
    Rb = ps.d64 ? ((opq == OP_DEC) ? B0 : B1) : B3; Eb = ps.d64 ? B2 : B4;
    Ib = ps.d64 ? B3 : B4; Ob = ps.d64 ? B4 : B3;    // Dec INTT B3 <-> B4, scaled result in Ob
    Vc = B2;                                          // compressed sets (NTT_DIM 128): NTT(v) B1 -> B2
    hU = ps.comp ? B7 : B6;                           // h (uncompressed) / kci*h (compressed); Y-scaled copy in B5
                                                      // Dec: kd*f in B7, kd*Y*f in B5 before the h preparation
    Pout = ps.comp ? B1 : B7;                         // (re-)encryption product
  end

  // command helpers
  palu_cmd_t pc;
  logic dfail;
  assign dfail = ps.comp ? ca_mismatch : (p_nz | ncq);

  logic hp_go, hp_issue;
  logic ca_strm_n, cb_strm_n;
  // a codec command reads a poly buffer behind the poly ALU when it is launched together with the
  // producing ALU command: PROD (KeyGen PK, Enc ct), INTT (compressed ct)
  always_comb begin
    ca_strm_n = (st_q == R_PROD) || (st_q == R_INTT);   // st_q: state that issued the strobe
    cb_strm_n = (st_q == K_PROD);
  end
  palu_cmd_t hp_cmd;

  // ---------------- main FSM
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE; st_q <= S_IDLE; busy <= 1'b0; done <= 1'b0; fail <= 1'b0; ps <= '0; opq <= '0; tf <= '0; tg <= '0;
      nonce <= '0; noninv_q <= 1'b0; ncq <= 1'b0;
      pend_h <= 1'b0; pend_s <= 1'b0; pend_s1 <= 1'b0; s1_valid <= 1'b0; s1_dst <= '0; pend_p <= 1'b0; pend_q <= 1'b0; pend_a <= 1'b0; pend_b <= 1'b0;
      h_valid <= 1'b0; h_op <= '0; h_seed_sel <= 1'b0; h_nonce <= '0; h_nblocks <= '0; h_nonce1 <= '0; h_nblocks1 <= '0;
      h_nmlen <= '0; h_pkall <= 1'b1;
      s_valid <= 1'b0; s_dist <= '0; s_region <= 1'b0; s_dst <= '0; s_rinv <= 1'b0; s_vinv <= 1'b0;
      p_valid <= 1'b0; p_cmd <= '0;
      q_start <= 1'b0; q_judge <= 1'b0; q_inj <= 1'b0; q_bA <= '0; q_bYA0 <= '0; q_bCA0 <= '0; q_bT1 <= '0; q_bT2 <= '0;
      q_bYCH <= '0; q_bCCH <= '0; q_cval <= '0;
      ca_strm <= 1'b0; cb_strm <= 1'b0;
      ca_valid <= 1'b0; cb_valid <= 1'b0; ca_mode <= '0; cb_mode <= '0; ca_field <= '0; cb_field <= '0; ca_buf <= '0; cb_buf <= '0;
      hpk_we <= 1'b0; ss_we <= 1'b0; ss_clr <= 1'b0; hp_go <= 1'b0; h_rnd_go <= 1'b0;
    end else begin
      h_rnd_go <= 1'b0;
      st_q <= st;
      // single-cycle strobes
      h_valid <= 1'b0; s_valid <= 1'b0; s1_valid <= 1'b0; p_valid <= 1'b0; q_start <= 1'b0; ca_valid <= 1'b0; cb_valid <= 1'b0;
      hpk_we <= 1'b0; ss_we <= 1'b0; ss_clr <= 1'b0; hp_go <= 1'b0;
      // completion tracking
      if (h_done) pend_h <= 1'b0;
      if (s_done) pend_s <= 1'b0;
      if (s1_done) pend_s1 <= 1'b0;
      if (p_done) pend_p <= 1'b0;
      if (q_done) begin pend_q <= 1'b0; noninv_q <= q_noninv; end
      if (ca_done) pend_a <= 1'b0;
      if (cb_done) pend_b <= 1'b0;
      // streaming lasts until the producing ALU command completes (later commands restart the watermark)
      if (p_done) begin ca_strm <= 1'b0; cb_strm <= 1'b0; end
      if (ca_valid) ca_strm <= ca_strm_n;
      if (cb_valid) cb_strm <= cb_strm_n;
      // h-preparation commands share the poly ALU port
      if (hp_issue) begin p_valid <= 1'b1; p_cmd <= hp_cmd; pend_p <= 1'b1; end
`ifndef SYNTHESIS
      if (hp_issue && (st == R_NTT || st == R_PROD || st == R_INTT || st == D_FSC || st == D_PROD || st == D_INTT ||
                       st == D_VNTT || st == K_FNTT || st == K_GNTT || st == K_PSC || st == K_PROD))
        $error("nev_seq: h-preparation command while the main FSM may issue a poly ALU command at %t", $time);
`endif
      // rejection sampler needs more KDF blocks after the KDF job ended: extend stream 0
      if (pend_s && s_need && !pend_h && !h_busy && !h_valid) begin
        h_valid <= 1'b1; h_op <= H_MORE; pend_h <= 1'b1;
      end

      case (st)
        S_IDLE: begin
          done <= done;
          if (start && op != 2'd3 && param < 4'd12) begin
            ps <= pset(param); opq <= op; tf <= tst_fail_f; tg <= tst_fail_g; nonce <= '0;
            busy <= 1'b1; done <= 1'b0; fail <= 1'b0;
            case (op)
              OP_KG:  st <= K_RND;
              OP_ENC: st <= E_RND;
              default: begin st <= D_START; ss_clr <= 1'b1; end
            endcase
          end
        end

        // ======================= KeyGen
        K_RND: begin h_rnd_go <= 1'b1; st <= K_SEED; end
        K_SEED: if (!h_rnd_go && h_rnd_fin) begin h_valid <= 1'b1; h_op <= H_SEED; pend_h <= 1'b1; st <= K_FKDF; end
        K_FKDF: if (!pend_h && !pend_s && !h_valid) begin
          h_valid <= 1'b1; h_op <= H_KDF; h_seed_sel <= 1'b0; h_nonce <= nonce; h_nblocks <= nblk(ps.eta_f, ps.nlog);
          h_nblocks1 <= '0; h_nmlen <= '0; pend_h <= 1'b1;
          s_valid <= 1'b1; s_dist <= ps.eta_f; s_region <= 1'b0; s_dst <= B0; s_rinv <= 1'b0; s_vinv <= 1'b1; pend_s <= 1'b1;
          st <= K_FNTT;
        end
        K_FNTT: if (!pend_s && !pend_h && !s_valid && !h_valid && !pend_p) begin
          pc = '0; pc.op = PA_NTT; pc.ba = B0; pc.bb = B1; pc.split = 1'b1; pc.bc = B2; pc.bd = B3; p_valid <= 1'b1; p_cmd <= pc; pend_p <= 1'b1; st <= K_FINV;
        end
        K_FINV: if (!pend_p && !p_valid) begin
          q_start <= 1'b1; pend_q <= 1'b1; q_judge <= ps.comp; q_inj <= (tf != 2'd0); q_cval <= rq;
          q_bA <= Fb; q_bYA0 <= B2; q_bCA0 <= B3; q_bT1 <= B4; q_bT2 <= B5;
          if (!ps.comp) begin q_bYCH <= B6; q_bCCH <= Fo; end
          else          begin q_bYCH <= B5; q_bCCH <= Fo; end
          // speculative next candidate: g with nonce+1 into B7
          h_valid <= 1'b1; h_op <= H_KDF; h_seed_sel <= 1'b0; h_nonce <= nonce + 8'd1; h_nblocks <= nblk(ps.eta_g, ps.nlog);
          h_nblocks1 <= '0; h_nmlen <= '0; pend_h <= 1'b1;
          s_valid <= 1'b1; s_dist <= ps.eta_g; s_region <= 1'b0; s_dst <= B7; s_rinv <= 1'b0; s_vinv <= 1'b0; pend_s <= 1'b1;
          st <= K_FCHK;
        end
        K_FCHK: if (!pend_q && !q_start) begin
          nonce <= nonce + 8'd1;
          if (noninv_q) begin
            if (tf != 2'd0) tf <= tf - 2'd1;
            st <= K_FWAIT;                       // discard the speculative candidate
          end else st <= K_GNTT;
        end
        K_FWAIT: if (!pend_h && !pend_s && !h_valid && !s_valid) st <= K_FKDF;
        K_GKDF: if (!pend_h && !pend_s && !h_valid) begin
          h_valid <= 1'b1; h_op <= H_KDF; h_seed_sel <= 1'b0; h_nonce <= nonce; h_nblocks <= nblk(ps.eta_g, ps.nlog);
          h_nblocks1 <= '0; h_nmlen <= '0; pend_h <= 1'b1;
          s_valid <= 1'b1; s_dist <= ps.eta_g; s_region <= 1'b0; s_dst <= B7; s_rinv <= 1'b0; s_vinv <= 1'b0; pend_s <= 1'b1;
          st <= K_GNTT;
        end
        K_GNTT: if (!pend_s && !pend_h && !s_valid && !h_valid && !pend_p && !pend_q) begin
          pc = '0; pc.op = PA_NTT; pc.ba = B7; pc.bb = B6; pc.split = 1'b1; pc.bc = B2; pc.bd = B3; p_valid <= 1'b1; p_cmd <= pc; pend_p <= 1'b1; st <= K_GINV;
        end
        K_GINV: if (!pend_p && !p_valid) begin
          q_start <= 1'b1; pend_q <= 1'b1; q_judge <= !ps.comp; q_inj <= (tg != 2'd0); q_cval <= rq;
          q_bA <= Gb; q_bYA0 <= B2; q_bCA0 <= B3; q_bT1 <= B4;
          if (!ps.comp) begin q_bT2 <= Gx; q_bYCH <= Fo; q_bCCH <= Gx; end   // judge: T2 unused; f^-1 kept in B5
          else          begin q_bT2 <= B5; q_bYCH <= Fo; q_bCCH <= Gx; end   // inverse -> B5
          st <= K_GCHK;
        end
        K_GCHK: if (!pend_q && !q_start) begin
          nonce <= nonce + 8'd1;
          if (noninv_q) begin
            if (tg != 2'd0) tg <= tg - 2'd1;
            st <= K_GKDF;
          end else st <= K_PSC;
        end
        K_PSC: begin   // compressed: Y*F into B2 (uncompressed: Y*G already in B2 from the judge)
          if (ps.comp) begin
            pc = '0; pc.op = PA_SCALE; pc.ba = Fb; pc.nrows = 6'(6'd1 << klog); pc.bd = B2; pc.cmode = CM_Y;
            p_valid <= 1'b1; p_cmd <= pc; pend_p <= 1'b1;
          end
          st <= K_PROD;
        end
        K_PROD: if (!pend_p && !p_valid) begin
          // h = f^-1 g (g^-1 f) in natural row order; codec B packs PK behind it and H(pk) follows codec B
          pc = '0; pc.op = PA_PROD; pc.pmode = PM_NORM; pc.mlog = klog; pc.ba = ps.comp ? Fb : Gb; pc.bb = B2; pc.bc = B5; pc.bd = B3;
          p_valid <= 1'b1; p_cmd <= pc; pend_p <= 1'b1;
          ca_valid <= 1'b1; ca_mode <= C_PACK; ca_field <= F_SKF; ca_buf <= ps.comp ? Gb : Fb; pend_a <= 1'b1;
          cb_valid <= 1'b1; cb_mode <= C_PACK; cb_field <= F_PK;  cb_buf <= B3; pend_b <= 1'b1;
          h_valid <= 1'b1; h_op <= H_HPK; h_pkall <= 1'b0; pend_h <= 1'b1;
          st <= K_FIN;
        end
        K_FIN: if (!pend_a && !pend_b && !pend_h && !pend_p && !ca_valid) begin
          hpk_we <= 1'b1; st <= S_FIN;
        end

        // ======================= Enc
        E_RND: begin
          // random bytes captured beside H(pk); h unpacked and prepared in parallel
          h_rnd_go <= 1'b1;
          h_valid <= 1'b1; h_op <= H_HPK; h_pkall <= 1'b1; pend_h <= 1'b1;
          ca_valid <= 1'b1; ca_mode <= C_UNPACK; ca_field <= F_PK; ca_buf <= B6; pend_a <= 1'b1;
          hp_go <= 1'b1;
          st <= E_H2;
        end
        E_H2:  if (!pend_h && !h_rnd_go && h_rnd_fin) begin h_valid <= 1'b1; h_op <= H_HASH2; pend_h <= 1'b1; st <= R_KDF; end

        // ======================= Dec
        D_START: begin
          ca_valid <= 1'b1; ca_mode <= C_UNPACK; ca_field <= F_SKF; ca_buf <= B0; pend_a <= 1'b1;
          cb_valid <= 1'b1; cb_mode <= ps.comp ? C_DECOMP : C_UNPACK; cb_field <= F_CT; cb_buf <= B1; pend_b <= 1'b1;
          h_valid <= 1'b1; h_op <= H_LOADHK; pend_h <= 1'b1;
          if (!ps.comp) begin
            // B7 = kd*f, B5 = kd*Y*f, streamed behind the f unpack (codec A)
            pc = '0; pc.op = PA_SCALE; pc.ba = B0; pc.nrows = 6'(6'd1 << klog); pc.cmode = CM_KY; pc.cval = kd; pc.ks = 1'b0;
            pc.be = B7; pc.bd = B5; pc.strm = 1'b1;
            p_valid <= 1'b1; p_cmd <= pc; pend_p <= 1'b1;
          end
          st <= D_VNTT;
        end
        D_VNTT: if (!pend_b && !cb_valid) begin
          ncq <= !ps.comp && cb_ncanon;
          if (ps.comp) begin pc = '0; pc.op = PA_NTT; pc.ba = B1; pc.bb = B2; pc.split = 1'b1; pc.bc = B3; pc.bd = B4; p_valid <= 1'b1; p_cmd <= pc; pend_p <= 1'b1; end
          st <= ps.comp ? D_FSC : D_PROD;
        end
        D_FSC: if (!pend_a && !pend_p && !ca_valid && !p_valid) begin
          pc = '0; pc.op = PA_SCALE; pc.ba = B0; pc.nrows = 6'(6'd1 << klog); pc.cmode = CM_KY; pc.cval = kd; pc.ks = 1'b0;
          pc.be = B7; pc.bd = B5;
          p_valid <= 1'b1; p_cmd <= pc; pend_p <= 1'b1; st <= D_PROD;
        end
        D_PROD: if (!pend_p && !p_valid && !pend_a && !ca_valid) begin
          // h (pk copy) for the re-encryption, unpacked while the product runs
          ca_valid <= 1'b1; ca_mode <= C_UNPACK; ca_field <= F_PK; ca_buf <= B6; pend_a <= 1'b1;
          pc = '0; pc.op = PA_PROD; pc.pmode = PM_NORM; pc.mlog = klog; pc.ba = B7; pc.bb = B5; pc.bc = ps.comp ? Vc : B1; pc.bd = B3;
          p_valid <= 1'b1; p_cmd <= pc; pend_p <= 1'b1;
          st <= D_INTT;
        end
        D_INTT: if (!pend_p && !p_valid) begin
          // t = kd * INTT(f*v); TOMSG streams behind the last INTT level
          pc = '0; pc.op = PA_INTT; pc.ba = B3; pc.bb = B4; pc.split = 1'b1; pc.tmsg = 1'b1;
          pc.bc = ps.comp ? B1 : B0; pc.bd = B2;
          p_valid <= 1'b1; p_cmd <= pc; pend_p <= 1'b1; hp_go <= 1'b1;
          st <= D_H2;
        end
        D_H2: if (!pend_p && !p_valid && !pend_h && hp != HP_S1 && hp != HP_W1 && hp != HP_S2 && hp != HP_W2) begin
          h_valid <= 1'b1; h_op <= H_HASH2; pend_h <= 1'b1; st <= R_KDF;
        end

        // ======================= (re-)encryption core, shared by Enc and Dec
        R_KDF: if (!pend_h && !h_valid) begin
          // stream 0: r (uncompressed sets), stream 1: noisem bytes for e
          h_valid <= 1'b1; h_op <= H_KDF; h_seed_sel <= 1'b1;
          h_nonce <= 8'd0; h_nblocks <= ps.comp ? 7'd0 : nblk(ps.eta_r, ps.nlog);
          h_nonce1 <= 8'd1; h_nblocks1 <= nmblk; h_nmlen <= nmlen; pend_h <= 1'b1;
          if (!ps.comp) begin
            s_valid <= 1'b1; s_dist <= ps.eta_r; s_region <= 1'b0; s_dst <= (opq == OP_DEC) ? B0 : B1; s_rinv <= 1'b1; s_vinv <= 1'b0; pend_s <= 1'b1;
          end
          // noisem e on the second sampler, concurrently (it follows the stream-1 watermark)
          s1_valid <= 1'b1; s1_dst <= B2; pend_s1 <= 1'b1;
          st <= R_NTT;
        end
        R_NTT: if (!pend_s && !s_valid && !pend_s1 && !s1_valid && !pend_p && !p_valid && hp == HP_DONE) begin
          pc = '0; pc.op = PA_NTT; pc.ba = B2; pc.bb = B4; pc.split = 1'b1; pc.bc = B1; pc.bd = B3;
          if (!ps.comp) begin pc.ba = (opq == OP_DEC) ? B0 : B1; pc.bb = B3; pc.bc = B2; pc.bd = B4; pc.dual = 1'b1; pc.split = 1'b0; end
          p_valid <= 1'b1; p_cmd <= pc; pend_p <= 1'b1; st <= R_PROD;
        end
        R_PROD: if (!pend_p && !p_valid) begin
          pc = '0; pc.op = PA_PROD; pc.pmode = PM_NORM; pc.mlog = klog; pc.ba = hU; pc.bb = B5; pc.bd = Pout;
          if (!ps.comp) begin pc.bc = Rb; pc.extra = 1'b1; pc.be = Eb; end
          else          begin pc.bc = Eb; end
          if (!ps.comp && opq == OP_DEC) begin pc.cmpc = 1'b1; pc.bd = B1; end   // compare with the ciphertext coefficients
          p_valid <= 1'b1; p_cmd <= pc; pend_p <= 1'b1;
          if (!ps.comp && opq == OP_ENC) begin   // ct packed behind the product
            ca_valid <= 1'b1; ca_field <= F_CT; ca_buf <= B7; pend_a <= 1'b1; ca_mode <= C_PACK;
          end
          st <= ps.comp ? R_INTT : S_FIN;
        end
        R_INTT: if (!pend_p && !p_valid) begin
          // compressed sets (NTT_DIM 128, 7 levels): INTT B1 <-> B3, result in B3 (kci folded into h);
          // ct compressed / compared behind the last INTT level
          pc = '0; pc.op = PA_INTT; pc.ba = B1; pc.bb = B3; pc.split = 1'b1; pc.bc = B2; pc.bd = B4; p_valid <= 1'b1; p_cmd <= pc; pend_p <= 1'b1;
          ca_valid <= 1'b1; ca_field <= F_CT; ca_buf <= B3; pend_a <= 1'b1;
          ca_mode <= (opq == OP_ENC) ? C_COMP : C_COMPCMP;
          st <= S_FIN;
        end

        // all units idle: write the SS register field now so that it is committed on the
        // completion edge (S_DONE), never after it
        S_FIN: if (!pend_a && !pend_b && !pend_h && !pend_s && !pend_s1 && !pend_p && !pend_q &&
                   !ca_valid && !cb_valid && !h_valid && !s_valid && !p_valid && !q_start && !hpk_we &&
                   (hp == HP_IDLE || hp == HP_DONE)) begin
          if (opq == OP_ENC) ss_we <= 1'b1;
          if (opq == OP_DEC && !dfail) ss_we <= 1'b1;
          st <= S_DONE;
        end
        S_DONE: begin
          if (opq == OP_DEC) fail <= dfail;
          busy <= 1'b0; done <= 1'b1; st <= S_IDLE;
        end
        default: st <= S_IDLE;
      endcase
    end
  end

  // ---------------- h preparation (Enc during H(pk), Dec during TOMSG/Hash2)
  //   uncompressed: B5 = Y*h            (U operand: h in B6)
  //   compressed  : B7 = kci*h ; B5 = Y*B7   (U operand: B7)
  always_comb begin
    hp_issue = 1'b0; hp_cmd = '0;
    if (hp == HP_S1) begin
      hp_issue = 1'b1; hp_cmd.op = PA_SCALE; hp_cmd.ba = B6; hp_cmd.nrows = 6'(6'd1 << klog);
      if (ps.comp) begin hp_cmd.be = B7; hp_cmd.bd = B5; hp_cmd.cmode = CM_KY; hp_cmd.cval = kci; hp_cmd.ks = 1'b1; end
      else         begin hp_cmd.bd = B5; hp_cmd.cmode = CM_Y; end
    end else if (hp == HP_S2) begin
      hp_issue = 1'b1; hp_cmd.op = PA_SCALE; hp_cmd.ba = B7; hp_cmd.nrows = 6'(6'd1 << klog);
      hp_cmd.bd = B5; hp_cmd.cmode = CM_Y;
    end
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) hp <= HP_IDLE;
    else begin
      case (hp)
        HP_IDLE: if (hp_go) hp <= HP_WAIT;
        HP_WAIT: if (!pend_a && !ca_valid && !pend_p && !p_valid) hp <= HP_S1;
        HP_S1:   hp <= HP_W1;
        HP_W1:   if (!pend_p && !p_valid) hp <= HP_DONE;
        HP_S2:   hp <= HP_W2;
        HP_W2:   if (!pend_p && !p_valid) hp <= HP_DONE;
        HP_DONE: if (st == S_DONE) hp <= HP_IDLE;
        default: hp <= HP_IDLE;
      endcase
    end
  end
endmodule
