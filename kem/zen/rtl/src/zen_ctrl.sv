// Command controller: accepts commands, receives random bytes, sequences the units for
// KeyGen / Encaps / Decaps and drives the polynomial-slot crossbar.
// Read masters : 0 NTT, 1 BRE A, 2 BRE B, 3 PASS R1, 4 PASS R2, 5 PKU
// Write masters: 0 SAMPLER, 1 NTT, 2 BRE C, 3 PASS W, 4 PKU
module zen_ctrl (
  input  logic          clk,
  input  logic          rst_n,
  // command / status
  input  logic          cmd_valid,
  output logic          cmd_ready,
  input  logic [1:0]    cmd_op,
  input  logic [1:0]    cmd_param,
  output logic          busy,
  output logic          done,
  output logic          fail,
  output logic          result_valid,
  output logic          accept,          // command accepted this cycle
  // random stream
  input  logic          rnd_valid,
  output logic          rnd_ready,
  input  logic [63:0]   rnd_data,
  output logic          rnd_we,
  output logic [1:0]    rnd_reg,
  output logic [2:0]    rnd_idx,
  output logic [63:0]   rnd_wdata,
  // latched configuration
  output logic [1:0]    param,
  output logic [7:0]    nonce,
  output logic          clr,
  // crossbar
  output logic [5:0][1:0] rsel,
  output logic [4:0][1:0] wsel,
  // sampler
  output logic          samp_start,
  output logic [1:0]    samp_poly,
  input  logic          samp_done,
  input  logic          samp_t0_odd,
  // ntt
  output logic          ntt_start,
  output logic [1:0]    ntt_mode,
  input  logic          ntt_done,
  input  logic          ntt_zq_fail,
  // bre
  output logic          bre_start,
  output logic [1:0]    bre_kind,
  input  logic          bre_done,
  // pass
  output logic          pass_start,
  output logic [1:0]    pass_mode,
  input  logic          pass_done,
  // ntt fused combine / FO compare
  output logic          ntt_cmp,
  input  logic          ntt_mismatch,
  input  logic          ntt_iss_done,    // NTT: all reads issued (overlap NTT -> basemul)
  input  logic          bre_wb_last,     // BRE: final write-back begins (overlap basemul -> INTT)
  input  logic          ntt_act_q,       // NTT busy (a new NTT operation needs an idle engine)
  input  logic          bre_act_q,       // BRE busy
  input  logic [6:0]    samp_wpairs,     // sampler: word pairs written in the current operation
  output logic          pku_lim_bre,     // PKU PK_PACK reads limited to words committed by the BRE (KeyGen)
  // pku
  output logic          pku_start,
  output logic [2:0]    pku_op,
  input  logic          pku_done,
  // gf2
  output logic          gf2_start,
  output logic          gf2_op,
  input  logic          gf2_done,
  output logic          m_load,
  // hash
  output logic          t_hpk_start, t_rej_start, t_kr_start, t_smid_start, t_skld_start, t_skst_start,
  output logic          m_ok,
  input  logic          t_hpk_done, t_rej_done, t_kr_done, t_smid_done, t_skld_done, t_skst_done,
  output logic          pk_stream,       // KeyGen: limit H(pk) reads to packed words
  output logic          ss_load,
  output logic          ss_sel_rej
);
  localparam logic [1:0] OP_KG = 2'd0, OP_EN = 2'd1, OP_DE = 2'd2;
  localparam logic [1:0] SL0 = 2'd0, SL1 = 2'd1, SL2 = 2'd2;

  typedef enum logic [5:0] {
    S_IDLE, S_ILLEGAL,
    // KeyGen
    K_SEED, K_SMID, K_F, K_FW, K_FN, K_G, K_GW, K_GN, K_BM, K_BMW, K_F2, K_F2W, K_PACK, K_HPKW, K_SKST,
    // Encaps
    E_START, E_KR, E_SMID, E_S, E_SW, E_BM, E_INTT, E_COMB, E_END,
    // Decaps
    D_START, D_NTT, D_BM, D_INTT, D_FOLD, D_GF2, D_KR, D_SMID, D_S, D_SW, D_BM2, D_INTT2, D_COMB, D_END,
    S_FINISH
  } st_t;
  st_t st;

  logic [1:0] op;
  logic       legal;
  assign legal = (cmd_op != 2'd3) && (cmd_param != 2'd3);

  // sticky completion flags (set by done, cleared by the matching start)
  logic f_samp, f_ntt, f_bre, f_pass, f_pku, f_gf2, f_hpk, f_rej, f_kr, f_smid, f_skld, f_skst;
  logic zq_q, odd_q;
  logic [1:0] pku_seq;
  logic       act_rej_q, bre_busy_q, bre_wbl_q, ntt_go_q;
  logic [1:0] kseq;       // KeyGen PKU side sequence: SKF_PACK -> F2_STORE -> free
  logic       ntt_early;
  // NTT(s) early start: once the sampler has written the first word pair of its last chunk (pairs written
  // >= 8*(chunks-1)+1). The remaining 7 pairs are written on consecutive cycles before the NTT reads them
  // (first NTT read >= 3 cycles later; pass-0 words of the last chunk are read no earlier than pair
  // positions that follow their writes), and NTT writes start after the sampler has finished.
  assign ntt_early = (param == 2'd0) ? (samp_wpairs >= 7'd9) : (param == 2'd1) ? (samp_wpairs >= 7'd25) : (samp_wpairs >= 7'd57);

  // random receiver
  logic [3:0] rcnt;          // beats received in current phase
  logic [1:0] rphase;        // 0 idle, 1 seed (KeyGen), 2 z (KeyGen), 3 m (Enc)
  logic       seed_rdy, z_rdy, m_rdy;
  logic [3:0] nbeats_ss;
  assign nbeats_ss = (param == 2'd0) ? 4'd2 : (param == 2'd1) ? 4'd4 : 4'd8;

  // ------------------------------------------------------------------ F4 Zq-check injection hook
  // Simulation only (FV-F4H): the testbench may set bit n of inj_f_mask / inj_g_mask to force the
  // committed Zq invertibility check of the f (g) attempt sampled with nonce n to "fail". The forced
  // result takes exactly the path of a real failure (nonce increment, resampling). Excluded from
  // synthesis: inj_f / inj_g are constant 0 under SYNTHESIS. inj_fired_f/g count forced checks.
  logic inj_f, inj_g;
`ifndef SYNTHESIS
  logic [255:0] inj_f_mask = '0;
  logic [255:0] inj_g_mask = '0;
  int           inj_fired_f = 0;
  int           inj_fired_g = 0;
  assign inj_f = (op == OP_KG) && inj_f_mask[nonce - 8'd1];   // nonce was incremented after sampling
  assign inj_g = (op == OP_KG) && inj_g_mask[nonce - 8'd1];
  always @(posedge clk) begin
    if (st == K_FN && f_ntt && !zq_q && inj_f) inj_fired_f <= inj_fired_f + 1;
    if (st == K_GN && f_ntt && !zq_q && inj_g) inj_fired_g <= inj_fired_g + 1;
  end
`else
  assign inj_f = 1'b0;
  assign inj_g = 1'b0;
`endif

  // ready only after reset release (cleared asynchronously by reset)
  logic rdy_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) rdy_q <= 1'b0;
    else        rdy_q <= 1'b1;
  end
  assign cmd_ready = (st == S_IDLE) && rdy_q;
  assign accept    = cmd_valid && cmd_ready;
  assign busy      = (st != S_IDLE);
  assign rnd_ready = (rphase != 2'd0);
  assign rnd_we    = rnd_valid && rnd_ready;
  assign rnd_idx   = rcnt[2:0];
  assign rnd_wdata = rnd_data;
  assign rnd_reg   = (rphase == 2'd1) ? 2'd0 : (rphase == 2'd2) ? 2'd1 : 2'd2;
  assign pk_stream = (op == OP_KG);
  assign m_ok      = (op == OP_EN) ? m_rdy : 1'b1;
  // SS register update in the completion cycle: the SS write commits at the same edge that sets done
  // (Encaps: kr[0:ss]; Decaps: kr or the rejection-hash value according to the fused FO compare result)
  assign ss_load    = ((st == E_END) || (st == D_END)) && (f_ntt || ntt_done) && !ntt_start;
  assign ss_sel_rej = (st == D_END) && ntt_mismatch;

  // single-cycle start helpers
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE; op <= '0; param <= '0; nonce <= '0;
      done <= 1'b0; fail <= 1'b0; result_valid <= 1'b0; clr <= 1'b0;
      rphase <= '0; rcnt <= '0; seed_rdy <= 1'b0; z_rdy <= 1'b0; m_rdy <= 1'b0;
      {samp_start, ntt_start, bre_start, pass_start, pku_start, gf2_start, m_load} <= '0;
      {t_hpk_start, t_rej_start, t_kr_start, t_smid_start, t_skld_start, t_skst_start} <= '0;
      samp_poly <= '0; ntt_mode <= '0; ntt_cmp <= 1'b0; bre_kind <= '0; pass_mode <= '0; pku_op <= '0; gf2_op <= 1'b0;
      rsel <= '0; wsel <= '0;
      {f_samp, f_ntt, f_bre, f_pass, f_pku, f_gf2, f_hpk, f_rej, f_kr, f_smid, f_skld, f_skst} <= '0;
      zq_q <= 1'b0; odd_q <= 1'b0;
      pku_seq <= 2'd0; act_rej_q <= 1'b0; bre_busy_q <= 1'b0; bre_wbl_q <= 1'b0; ntt_go_q <= 1'b0;
      kseq <= 2'd0; pku_lim_bre <= 1'b0;
    end else begin
      if (accept || samp_start) ntt_go_q <= 1'b0;
      if (bre_wb_last) bre_wbl_q <= 1'b1;
      if (bre_start)   bre_wbl_q <= 1'b0;
      // defaults: pulses low
      done <= 1'b0; clr <= 1'b0;
      {samp_start, ntt_start, bre_start, pass_start, pku_start, gf2_start, m_load} <= '0;
      {t_hpk_start, t_rej_start, t_kr_start, t_smid_start, t_skld_start, t_skst_start} <= '0;
      // completion flags
      if (samp_done) begin f_samp <= 1'b1; odd_q <= samp_t0_odd; end
      if (ntt_done)  begin f_ntt  <= 1'b1; zq_q  <= ntt_zq_fail; end
      if (bre_done)  f_bre  <= 1'b1;
      if (pass_done) f_pass <= 1'b1;
      if (pku_done)  f_pku  <= 1'b1;
      if (gf2_done)  f_gf2  <= 1'b1;
      if (t_hpk_done)  f_hpk  <= 1'b1;
      if (t_rej_done)  f_rej  <= 1'b1;
      if (t_kr_done)   f_kr   <= 1'b1;
      if (t_smid_done) f_smid <= 1'b1;
      if (t_skld_done) f_skld <= 1'b1;
      if (t_skst_done) f_skst <= 1'b1;
      // random receiver
      if (rnd_we) begin
        if (rphase == 2'd1) begin
          if (rcnt == 4'd7) begin rcnt <= '0; rphase <= 2'd2; seed_rdy <= 1'b1; end
          else rcnt <= rcnt + 4'd1;
        end else if (rphase == 2'd2) begin
          if (rcnt == nbeats_ss - 4'd1) begin rcnt <= '0; rphase <= 2'd0; z_rdy <= 1'b1; end
          else rcnt <= rcnt + 4'd1;
        end else begin
          if (rcnt == nbeats_ss - 4'd1) begin rcnt <= '0; rphase <= 2'd0; m_rdy <= 1'b1; end
          else rcnt <= rcnt + 4'd1;
        end
      end

      case (st)
        S_IDLE: if (cmd_valid && rdy_q) begin
          op <= cmd_op; param <= cmd_param;
          result_valid <= 1'b0; fail <= 1'b0; clr <= 1'b1;
          nonce <= 8'd0;
          seed_rdy <= 1'b0; z_rdy <= 1'b0; m_rdy <= 1'b0; rcnt <= '0;
          {f_samp, f_ntt, f_bre, f_pass, f_pku, f_gf2, f_hpk, f_rej, f_kr, f_smid, f_skld, f_skst} <= '0;
          if (!legal) st <= S_ILLEGAL;
          else begin
            case (cmd_op)
              OP_KG:   begin st <= K_SEED;  rphase <= 2'd1; end
              OP_EN:   begin
                // Encaps: H(pk), kr (+ seed midstate) and the sampler for s start in the acceptance cycle
                st <= E_SW; rphase <= 2'd3;
                wsel[4] <= SL0; pku_op <= 3'd0; pku_start <= 1'b1; f_pku <= 1'b0;
                t_hpk_start <= 1'b1; f_hpk <= 1'b0;
                t_kr_start <= 1'b1; f_kr <= 1'b0;
                nonce <= 8'd0; wsel[0] <= SL1; samp_poly <= 2'd2; samp_start <= 1'b1; f_samp <= 1'b0;
              end
              default: begin st <= D_START; rphase <= 2'd0; end
            endcase
          end
        end
        S_ILLEGAL: begin st <= S_IDLE; done <= 1'b1; fail <= 1'b1; end

        // =================================================== KeyGen
        // the sampler is started together with the seed midstate; its XOF requests are accepted once the
        // midstate is valid (zen_hash_sys smid_ok)
        K_SEED: if (seed_rdy) begin
          t_smid_start <= 1'b1; f_smid <= 1'b0;
          wsel[0] <= SL0; samp_poly <= 2'd0; samp_start <= 1'b1; f_samp <= 1'b0; st <= K_FW;
        end
        K_SMID: st <= K_F;
        K_F: begin
          wsel[0] <= SL0; samp_poly <= 2'd0; samp_start <= 1'b1; f_samp <= 1'b0; st <= K_FW;
        end
        K_FW: if (f_samp) begin
          nonce <= nonce + 8'd1;
          if (!odd_q) st <= K_F;
          else begin
            rsel[0] <= SL0; wsel[1] <= SL0; ntt_mode <= 2'd1; ntt_start <= 1'b1; f_ntt <= 1'b0; st <= K_FN;
          end
        end
        K_FN: if (f_ntt) begin
          if (zq_q || inj_f) st <= K_F;
          else begin
            // baseinv P0 -> P1, GF(2) inverse, sample g -> P2; SKF_PACK of f (P0) runs alongside baseinv
            // (shared P0 read port, baseinv has priority)
            rsel[1] <= SL0; rsel[2] <= SL1; wsel[2] <= SL1; bre_kind <= 2'd2; bre_start <= 1'b1; f_bre <= 1'b0;
            gf2_op <= 1'b0; gf2_start <= 1'b1; f_gf2 <= 1'b0;
            rsel[5] <= SL0; pku_op <= 3'd4; pku_start <= 1'b1; f_pku <= 1'b0; kseq <= 2'd1;
            st <= K_G;
          end
        end
        K_G: begin
          wsel[0] <= SL2; samp_poly <= 2'd1; samp_start <= 1'b1; f_samp <= 1'b0; st <= K_GW;
        end
        K_GW: if (f_samp) begin
          nonce <= nonce + 8'd1;
          rsel[0] <= SL2; wsel[1] <= SL2; ntt_mode <= 2'd1; ntt_start <= 1'b1; f_ntt <= 1'b0; st <= K_GN;
        end
        K_GN: if (f_ntt) begin
          if (zq_q || inj_g) st <= K_G;
          else st <= K_BM;
        end
        // h = basemul_mq(g, f^-1): A = P2 (g), B = P1 (finv), C = P1. PK_PACK reads the words already
        // written back by basemul_mq (P1 read port shared, BRE priority) and H(pk) follows the packed pk words.
        K_BM: if (f_bre && kseq == 2'd3 && !pku_start) begin
          rsel[1] <= SL2; rsel[2] <= SL1; wsel[2] <= SL1; bre_kind <= 2'd1; bre_start <= 1'b1; f_bre <= 1'b0;
          rsel[5] <= SL1; pku_op <= 3'd2; pku_start <= 1'b1; f_pku <= 1'b0; pku_lim_bre <= 1'b1;
          t_hpk_start <= 1'b1; f_hpk <= 1'b0;
          st <= K_HPKW;
        end
        K_BMW: st <= K_HPKW;
        K_F2W: st <= K_HPKW;
        K_HPKW: if (f_hpk && f_pku && f_bre && z_rdy && !pku_start && !bre_start) begin
          pku_lim_bre <= 1'b0;
          t_skst_start <= 1'b1; f_skst <= 1'b0; st <= K_SKST;
        end
        K_SKST: if (f_skst) st <= S_FINISH;

        // =================================================== Encaps
        // H(pk), kr (+ XOF seed midstate) and the sampler for s are all started at once; kr waits for
        // hpk and m inside zen_hash_sys, the sampler for the seed midstate
        E_START: begin
          wsel[4] <= SL0; pku_op <= 3'd0; pku_start <= 1'b1; f_pku <= 1'b0;
          t_hpk_start <= 1'b1; f_hpk <= 1'b0;
          t_kr_start <= 1'b1; f_kr <= 1'b0;
          nonce <= 8'd0; wsel[0] <= SL1; samp_poly <= 2'd2; samp_start <= 1'b1; f_samp <= 1'b0;
          st <= E_SW;
        end
        E_KR: st <= E_SW;
        E_SMID: st <= E_SW;
        E_S: st <= E_SW;
        // NTT(s) on P1 may start while the sampler is still writing s (from the first pair of the last chunk on,
        // after which every word is written before the NTT reads it); e is
        // sampled into P2 after s
        E_SW: begin
          if (!ntt_go_q && !samp_start && (f_samp || samp_done || ntt_early)) begin
            rsel[0] <= SL1; wsel[1] <= SL1; ntt_mode <= 2'd0; ntt_start <= 1'b1; f_ntt <= 1'b0; ntt_go_q <= 1'b1;
          end
          if ((f_samp || samp_done) && !samp_start && (ntt_go_q || ntt_start)) begin
            nonce <= 8'd1; wsel[0] <= SL2; samp_poly <= 2'd3; samp_start <= 1'b1; f_samp <= 1'b0;
            st <= E_BM;
          end
        end
        // basemul starts when NTT(s) has issued all reads: the remaining NTT writes complete before the
        // engine reads those words (CT last-pass word order)
        E_BM: if ((f_ntt || ntt_iss_done) && !ntt_start && f_pku && !bre_act_q) begin
          rsel[1] <= SL0; rsel[2] <= SL1; wsel[2] <= SL1; bre_kind <= 2'd0; bre_start <= 1'b1; f_bre <= 1'b0;
          st <= E_INTT;
        end
        // INTT_ENC starts when basemul's final write-back begins (its words are read after they are written)
        E_INTT: if ((f_bre || (bre_wbl_q && !bre_start)) && f_samp && !ntt_act_q) begin
          // INTT with the fused combine/compress: t in P1, e in P2 -> CT object
          rsel[0] <= SL1; wsel[1] <= SL1; rsel[4] <= SL2; ntt_mode <= 2'd3; ntt_cmp <= 1'b0;
          ntt_start <= 1'b1; f_ntt <= 1'b0; st <= E_END;
        end
        E_COMB: st <= E_END;
        E_END: if ((f_ntt || ntt_done) && !ntt_start) begin
          st <= S_IDLE; done <= 1'b1; result_valid <= 1'b1;    // SS loaded in this cycle (ss_load)
        end

        // =================================================== Decaps
        D_START: begin
          t_skld_start <= 1'b1; f_skld <= 1'b0;
          wsel[4] <= SL0; pku_op <= 3'd3; pku_start <= 1'b1; f_pku <= 1'b0;
          wsel[3] <= SL1; pass_mode <= 2'd0; pass_start <= 1'b1; f_pass <= 1'b0;
          st <= D_NTT;
        end
        D_NTT: begin
          if (f_skld && !f_rej && !act_rej_q) begin t_rej_start <= 1'b1; act_rej_q <= 1'b1; end
          if (f_pass) begin
            rsel[0] <= SL1; wsel[1] <= SL1; ntt_mode <= 2'd0; ntt_start <= 1'b1; f_ntt <= 1'b0; f_pass <= 1'b0;
            st <= D_BM;
          end
        end
        D_BM: begin
          if (f_skld && !act_rej_q) begin t_rej_start <= 1'b1; act_rej_q <= 1'b1; end
          if ((f_ntt || ntt_iss_done) && !ntt_start && (pku_seq != 2'd0 || f_pku)) begin
            rsel[1] <= SL1; rsel[2] <= SL0; wsel[2] <= SL1; bre_kind <= 2'd0; bre_start <= 1'b1; f_bre <= 1'b0;
            st <= D_INTT;
          end
        end
        D_INTT: begin
          if (f_skld && !act_rej_q) begin t_rej_start <= 1'b1; act_rej_q <= 1'b1; end
          if ((f_bre || (bre_wbl_q && !bre_start)) && !ntt_act_q) begin
            rsel[0] <= SL1; wsel[1] <= SL1; ntt_mode <= 2'd2; ntt_start <= 1'b1; f_ntt <= 1'b0; st <= D_FOLD;
          end
        end
        D_FOLD: if (f_ntt) begin
          rsel[3] <= SL1; pass_mode <= 2'd2; pass_start <= 1'b1; f_pass <= 1'b0; st <= D_GF2;
        end
        D_GF2: if (f_pass && pku_seq >= 2'd2) begin
          gf2_op <= 1'b1; gf2_start <= 1'b1; f_gf2 <= 1'b0; f_pass <= 1'b0; st <= D_KR;
        end
        D_KR: if (f_gf2) begin
          m_load <= 1'b1; st <= D_SMID;
        end
        D_SMID: begin
          t_kr_start <= 1'b1; f_kr <= 1'b0;
          nonce <= 8'd0; wsel[0] <= SL0; samp_poly <= 2'd2; samp_start <= 1'b1; f_samp <= 1'b0;
          st <= D_BM2;
        end
        D_S: st <= D_BM2;
        D_SW: st <= D_BM2;
        D_BM2: begin
          if (!ntt_go_q && !samp_start && (f_samp || samp_done || ntt_early)) begin
            rsel[0] <= SL0; wsel[1] <= SL0; ntt_mode <= 2'd0; ntt_start <= 1'b1; f_ntt <= 1'b0; ntt_go_q <= 1'b1;
          end
          if ((f_samp || samp_done) && !samp_start && (ntt_go_q || ntt_start)) begin
            nonce <= 8'd1; wsel[0] <= SL1; samp_poly <= 2'd3; samp_start <= 1'b1; f_samp <= 1'b0;
            st <= D_INTT2;
          end
        end
        D_INTT2: begin
          if ((f_ntt || ntt_iss_done) && !ntt_start && pku_seq == 2'd3 && !bre_busy_q && !bre_act_q) begin
            rsel[1] <= SL2; rsel[2] <= SL0; wsel[2] <= SL0; bre_kind <= 2'd0; bre_start <= 1'b1; f_bre <= 1'b0;
            bre_busy_q <= 1'b1;
          end
          // re-encryption INTT with the fused FO compare (CT ports A/B); interlock: the rejection hash
          // (CT port B reader) has completed (f_rej) before the compare starts
          if (bre_busy_q && (f_bre || bre_wbl_q) && !bre_start && f_samp && f_rej && !ntt_act_q) begin
            rsel[0] <= SL0; wsel[1] <= SL0; rsel[4] <= SL1; ntt_mode <= 2'd3; ntt_cmp <= 1'b1;
            ntt_start <= 1'b1; f_ntt <= 1'b0;
            bre_busy_q <= 1'b0;
            st <= D_END;
          end
        end
        D_COMB: st <= D_END;
        D_END: if ((f_ntt || ntt_done) && !ntt_start) begin
          st <= S_IDLE; done <= 1'b1; result_valid <= 1'b1;    // SS loaded in this cycle (ss_load)
        end

        S_FINISH: begin st <= S_IDLE; done <= 1'b1; result_valid <= 1'b1; end
        default: st <= S_IDLE;
      endcase

      // KeyGen PKU side sequence: SKF_PACK (started with baseinv) -> F2_STORE (after the GF(2) inverse)
      if (op == OP_KG && st != S_IDLE) begin
        case (kseq)
          2'd1: if (f_pku && f_gf2 && !pku_start) begin pku_op <= 3'd6; pku_start <= 1'b1; f_pku <= 1'b0; kseq <= 2'd2; end
          2'd2: if (f_pku && !pku_start) kseq <= 2'd3;
          default: ;
        endcase
      end
      // Decaps PKU side sequence: SKF_UNPACK (started in D_START) -> F2_LOAD -> PK_UNPACK_SK
      if (op == OP_DE && st != S_IDLE && st != D_START) begin
        case (pku_seq)
          2'd0: if (f_pku && !pku_start) begin pku_op <= 3'd5; pku_start <= 1'b1; f_pku <= 1'b0; pku_seq <= 2'd1; end
          2'd1: if (f_pku && !pku_start) begin wsel[4] <= SL2; pku_op <= 3'd1; pku_start <= 1'b1; f_pku <= 1'b0; pku_seq <= 2'd2; end
          2'd2: if (f_pku && !pku_start) pku_seq <= 2'd3;
          default: ;
        endcase
      end
      // Decaps rejection hash: started as soon as hpk/z are loaded from SK, in whatever state the sequence is in
      if (op == OP_DE && st != S_IDLE && st != S_FINISH && f_skld && !act_rej_q && !t_rej_start) begin t_rej_start <= 1'b1; act_rej_q <= 1'b1; end
      if (accept) begin pku_seq <= 2'd0; act_rej_q <= 1'b0; bre_busy_q <= 1'b0; kseq <= 2'd0; pku_lim_bre <= 1'b0; bre_wbl_q <= 1'b0; end
    end
  end

endmodule
