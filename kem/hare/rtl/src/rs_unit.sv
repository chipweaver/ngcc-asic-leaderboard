// Reed-Solomon unit over GF(2^8)/0x11D, 88-lane vector datapath (lane i = coefficient i).
// Decode reproduces reed_solomon_decode of the reference (erasure-aware, masked BM):
//   * per arriving symbol (pos ascending): S_i += r * alpha^{(i+1)pos}; on erasure
//     gamma_j += alpha^pos * gamma_{j-1} (j = 1..P); erasure count.
//   * finish: T = gamma*S mod x^P; bm[mu] = T[ecs+mu]; P BM iterations; Lambda = sigma*gamma mod
//     x^(P+1); Omega = S*Lambda mod x^P; Chien/Forney over every position, correcting the K
//     message bytes (positions P..P+K-1) when the decode is valid (erasures <= P).
//   The decode latency is fixed per level (two cycles per symbol, the ecs shift padded to P
//   steps), independent of the erasure pattern and of the received data.
// Encode: systematic LFSR (reed_solomon_encode), codeword = parity[0..P-1] || msg[0..K-1].
module rs_unit import hare_pkg::*; (
  input  logic         clk,
  input  logic         rst_n,
  input  logic [1:0]   lvl,
  // decode
  input  logic         dec_init,
  input  logic         sym_valid,
  input  logic [7:0]   sym_byte,
  input  logic         sym_era,
  input  logic         dec_finish,
  // encode
  input  logic         enc_start,
  input  logic [511:0] enc_msg,      // byte j = msg[j]
  input  logic         enc_use_msg,  // 1: take enc_msg, 0: encode the decoded message in place
  // outputs
  output logic [511:0] msg_out,      // decoded message (byte j)
  input  logic [7:0]   cw_idx,
  output logic [7:0]   cw_byte,      // codeword byte cw_idx (after encode)
  input  logic [7:0]   cw_idx2,
  output logic [7:0]   cw_byte2,
  output logic         busy
);
  localparam int NL = 88;
  logic [7:0] S   [NL];
  logic [7:0] PW  [NL];
  logic [7:0] GAM [NL];
  logic [7:0] A1  [NL];
  logic [7:0] A2  [NL];
  logic [7:0] SIG [NL];
  logic [7:0] XSP [NL];
  logic [7:0] TMP [NL];
  logic [7:0] GSH [NL];
  logic [7:0] MSG [64];
  logic [7:0] gm_a [NL];
  logic [7:0] gm_b [NL];
  logic [7:0] gm   [NL];

  logic [6:0] P;
  logic [6:0] K;
  logic [7:0] N1;
  assign P  = p_par(lvl);
  assign K  = p_s(lvl);
  assign N1 = p_n1(lvl);
  // level-derived lane masks, registered (lvl is stable for a whole operation)
  logic [NL-1:0] mlt, mle;     // lane i < P, lane i <= P
  logic [6:0]    act_it;       // P - ecs (BM active iterations)
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin mlt <= '0; mle <= '0; end
    else for (int i = 0; i < NL; i = i + 1) begin mlt[i] <= (i < P); mle[i] <= (i <= P); end

  typedef enum logic [4:0] {R_IDLE, R_ABS2, R_TI, R_T, R_SH, R_BI, R_B1, R_B2, R_B3,
                            R_LI, R_L, R_EI, R_E, R_C, R_ENC} st_t;
  st_t st;
  logic [7:0]  pos;          // symbol position (decode accumulation) / Chien position
  logic [7:0]  cnt;          // erasure count
  logic [7:0]  eta;          // alpha^pos
  logic        era_q;
  logic [7:0]  step;         // loop counter
  logic [6:0]  ecs;
  logic        valid_dec;
  // BM scalars
  logic [7:0]  disc_q, factor_q, prev_disc;
  logic [15:0] degree, prev_degree, pp;
  logic [6:0]  mu;
  logic        act_q, nz_q, upd_q;
  // Chien pipeline
  logic        c1v, c2v;
  logic [7:0]  c1loc, c1num, c1dodd, c1apos, c1pos;
  logic [7:0]  c2loc, c2num, c2den, c2pos;
  logic [7:0]  apos;

  genvar gi;
  generate
    for (gi = 0; gi < NL; gi = gi + 1) begin : g_gm
      assign gm[gi] = gmul(gm_a[gi], gm_b[gi]);
    end
  endgenerate

  // XOR reductions
  logic [7:0] x_sigr, x_a1, x_a2, x_a1odd;
  always_comb begin
    x_sigr = '0; x_a1 = '0; x_a2 = '0; x_a1odd = '0;
    for (int i = 1; i < NL; i = i + 1) x_sigr = x_sigr ^ gm[i];
    for (int i = 0; i < NL; i = i + 1) begin
      x_a1 = x_a1 ^ A1[i];
      x_a2 = x_a2 ^ A2[i];
      if (i % 2 == 1) x_a1odd = x_a1odd ^ A1[i];
    end
  end

  // encoder feedback
  logic [7:0] enc_gate, a2_top;
  always_comb begin
    case (lvl)
      2'd0: a2_top = A2[15];
      2'd1: a2_top = A2[48];
      2'd2: a2_top = A2[42];
      default: a2_top = A2[86];
    endcase
    enc_gate = MSG[6'(K - 7'd1 - step[6:0])] ^ a2_top;
  end

  // GM operand selection
  always_comb begin
    for (int i = 0; i < NL; i = i + 1) begin
      gm_a[i] = 8'd0; gm_b[i] = 8'd0;
      case (st)
        R_IDLE: begin gm_a[i] = sym_byte; gm_b[i] = PW[i]; end
        R_ABS2: begin gm_a[i] = eta; gm_b[i] = (i == 0) ? 8'd0 : GAM[i-1]; end
        R_T, R_L, R_E: begin gm_a[i] = GSH[0]; gm_b[i] = TMP[i]; end
        R_B1:   begin gm_a[i] = SIG[i]; gm_b[i] = A2[i]; end       // A2 holds R
        R_B3:   begin gm_a[i] = factor_q; gm_b[i] = XSP[i]; end
        R_ENC:  begin gm_a[i] = enc_gate; gm_b[i] = p_g(lvl, i[6:0]); end
        default: ;
      endcase
    end
  end

  // outputs
  always_comb begin
    for (int i = 0; i < 64; i = i + 1) msg_out[8*i +: 8] = MSG[i];
    if (cw_idx < {1'b0, P}) cw_byte = A2[cw_idx[6:0]];
    else if (cw_idx - {1'b0, P} < 8'd64) cw_byte = MSG[6'(cw_idx - {1'b0, P})];
    else                    cw_byte = 8'd0;   // beyond the codeword (masked by the caller)
    if (cw_idx2 < {1'b0, P}) cw_byte2 = A2[cw_idx2[6:0]];
    else if (cw_idx2 - {1'b0, P} < 8'd64) cw_byte2 = MSG[6'(cw_idx2 - {1'b0, P})];
    else                     cw_byte2 = 8'd0;
  end
  assign busy = (st != R_IDLE);

  logic [15:0] cand;
  assign cand = (({9'd0, mu}) - pp) + prev_degree;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= R_IDLE; pos <= '0; cnt <= '0; eta <= 8'd1; era_q <= 1'b0; step <= '0; ecs <= '0;
      valid_dec <= 1'b0; act_it <= '0; disc_q <= '0; factor_q <= '0; prev_disc <= 8'd1; degree <= '0;
      prev_degree <= '0; pp <= '0; mu <= '0; act_q <= 1'b0; nz_q <= 1'b0; upd_q <= 1'b0;
      c1v <= 1'b0; c2v <= 1'b0; c1loc <= '0; c1num <= '0; c1dodd <= '0; c1apos <= '0; c1pos <= '0;
      c2loc <= '0; c2num <= '0; c2den <= '0; c2pos <= '0; apos <= 8'd1;
      for (int i = 0; i < NL; i = i + 1) begin
        S[i] <= '0; PW[i] <= 8'd1; GAM[i] <= '0; A1[i] <= '0; A2[i] <= '0; SIG[i] <= '0;
        XSP[i] <= '0; TMP[i] <= '0; GSH[i] <= '0;
      end
      for (int i = 0; i < 64; i = i + 1) MSG[i] <= '0;
    end else begin
      case (st)
        R_IDLE: begin
          if (dec_init) begin
            pos <= '0; cnt <= '0; eta <= 8'd1;
            for (int i = 0; i < NL; i = i + 1) begin S[i] <= '0; PW[i] <= 8'd1; GAM[i] <= (i == 0) ? 8'd1 : 8'd0; end
          end else if (sym_valid) begin
            for (int i = 0; i < NL; i = i + 1) begin
              if (mlt[i]) S[i] <= S[i] ^ gm[i];
              PW[i] <= gmul(PW[i], p_alog(i + 1));
            end
            if (pos >= {1'b0, P} && pos < {1'b0, P} + {1'b0, K}) MSG[pos - {1'b0, P}] <= sym_byte;
            // every symbol takes two cycles (erasure-locator update in R_ABS2 only when erased),
            // so the decode latency does not depend on the erasure pattern
            era_q <= sym_era;
            st <= R_ABS2;
            if (sym_era) cnt <= cnt + 8'd1;
          end else if (dec_finish) begin
            ecs <= (cnt <= {1'b0, P}) ? cnt[6:0] : P;
            valid_dec <= (cnt <= {1'b0, P});
            st <= R_TI;
          end else if (enc_start) begin
            step <= '0;
            for (int i = 0; i < NL; i = i + 1) A2[i] <= '0;
            if (enc_use_msg) for (int i = 0; i < 64; i = i + 1) MSG[i] <= enc_msg[8*i +: 8];
            st <= R_ENC;
          end
        end
        R_ABS2: begin
          if (era_q) for (int i = 1; i < NL; i = i + 1) if (mle[i]) GAM[i] <= GAM[i] ^ gm[i];
          pos <= pos + 8'd1; eta <= gmul(eta, 8'd2);
          st <= R_IDLE;
        end
        // ---------------- T = gamma * S mod x^P  (acc A1)
        R_TI: begin
          for (int i = 0; i < NL; i = i + 1) begin A1[i] <= '0; TMP[i] <= S[i]; GSH[i] <= GAM[i]; end
          step <= '0; st <= R_T;
        end
        R_T: begin
          for (int i = 0; i < NL; i = i + 1) begin
            if (mlt[i]) A1[i] <= A1[i] ^ gm[i];
            TMP[i] <= (i == 0) ? 8'd0 : TMP[i-1];
            GSH[i] <= (i == NL - 1) ? 8'd0 : GSH[i+1];
          end
          step <= step + 8'd1;
          if (step == {1'b0, P} - 8'd1) begin step <= '0; st <= R_SH; end
        end
        // shift by ecs, padded to P steps so the decode latency does not depend on the erasure count
        R_SH: begin
          if (step < {1'b0, ecs}) for (int i = 0; i < NL; i = i + 1) A1[i] <= (i == NL - 1) ? 8'd0 : A1[i+1];
          step <= step + 8'd1;
          if (step == {1'b0, P} - 8'd1) begin step <= '0; st <= R_BI; end
        end
        // ---------------- BM
        R_BI: begin
          for (int i = 0; i < NL; i = i + 1) begin
            SIG[i] <= (i == 0) ? 8'd1 : 8'd0; XSP[i] <= (i == 1) ? 8'd1 : 8'd0; A2[i] <= '0;
          end
          degree <= '0; prev_degree <= '0; pp <= 16'hFFFF; prev_disc <= 8'd1; mu <= '0; act_it <= P - ecs;
          st <= R_B1;
        end
        R_B1: begin
          act_q  <= (mu < act_it);
          disc_q <= (mu < act_it) ? (A1[0] ^ x_sigr) : 8'd0;
          nz_q   <= (mu < act_it) && ((A1[0] ^ x_sigr) != 8'd0);
          st <= R_B2;
        end
        R_B2: begin
          factor_q <= gmul(disc_q, p_ginv(prev_disc));
          upd_q <= act_q && nz_q && (cand > degree);
          st <= R_B3;
        end
        R_B3: begin
          for (int i = 0; i < NL; i = i + 1) begin
            if (act_q && nz_q && i >= 1 && mle[i]) SIG[i] <= SIG[i] ^ gm[i];
            if (i == 0 || !mle[i]) XSP[i] <= 8'd0;
            else XSP[i] <= upd_q ? SIG[i-1] : XSP[i-1];
            // R (in A2): R[i] <= R[i-1], R[1] <= bm[mu] = A1[0]
            if (i == 0) A2[i] <= 8'd0;
            else if (i == 1) A2[i] <= A1[0];
            else A2[i] <= A2[i-1];
            A1[i] <= (i == NL - 1) ? 8'd0 : A1[i+1];
          end
          if (upd_q) begin degree <= cand; pp <= {9'd0, mu}; prev_disc <= disc_q; prev_degree <= degree; end
          mu <= mu + 7'd1;
          st <= (mu == P - 7'd1) ? R_LI : R_B1;
        end
        // ---------------- Lambda = sigma * gamma mod x^(P+1)  (acc A1)
        R_LI: begin
          for (int i = 0; i < NL; i = i + 1) begin A1[i] <= '0; TMP[i] <= GAM[i]; GSH[i] <= SIG[i]; end
          step <= '0; st <= R_L;
        end
        R_L: begin
          for (int i = 0; i < NL; i = i + 1) begin
            if (mle[i]) A1[i] <= A1[i] ^ gm[i];
            TMP[i] <= (i == 0) ? 8'd0 : TMP[i-1];
            GSH[i] <= (i == NL - 1) ? 8'd0 : GSH[i+1];
          end
          step <= step + 8'd1;
          if (step == {1'b0, P}) begin step <= '0; st <= R_EI; end
        end
        // ---------------- Omega = S * Lambda mod x^P  (acc A2)
        R_EI: begin
          for (int i = 0; i < NL; i = i + 1) begin A2[i] <= '0; TMP[i] <= A1[i]; GSH[i] <= S[i]; end
          step <= '0; st <= R_E;
        end
        R_E: begin
          for (int i = 0; i < NL; i = i + 1) begin
            if (mlt[i]) A2[i] <= A2[i] ^ gm[i];
            TMP[i] <= (i == 0) ? 8'd0 : TMP[i-1];
            GSH[i] <= (i == NL - 1) ? 8'd0 : GSH[i+1];
          end
          step <= step + 8'd1;
          if (step == {1'b0, P} - 8'd1) begin step <= '0; st <= R_C; pos <= '0; apos <= 8'd1; c1v <= 1'b0; c2v <= 1'b0; end
        end
        // ---------------- Chien / Forney (3-stage pipeline over positions)
        R_C: begin
          // stage 1
          if (pos < N1) begin
            c1v <= 1'b1; c1loc <= x_a1; c1num <= x_a2; c1dodd <= x_a1odd; c1apos <= apos; c1pos <= pos;
            for (int i = 0; i < NL; i = i + 1) begin
              A1[i] <= gmul(A1[i], p_alog(255 - i));
              A2[i] <= gmul(A2[i], p_alog(255 - i));
            end
            apos <= gmul(apos, 8'd2);
            pos <= pos + 8'd1;
          end else c1v <= 1'b0;
          // stage 2
          c2v <= c1v; c2loc <= c1loc; c2num <= c1num; c2pos <= c1pos;
          c2den <= gmul(c1dodd, c1apos);
          // stage 3
          if (c2v && valid_dec && c2loc == 8'd0 && c2den != 8'd0 &&
              c2pos >= {1'b0, P} && c2pos < {1'b0, P} + {1'b0, K})
            MSG[c2pos - {1'b0, P}] <= MSG[c2pos - {1'b0, P}] ^ gmul(c2num, p_ginv(c2den));
          if (pos == N1 && !c1v && !c2v) st <= R_IDLE;
        end
        // ---------------- encode
        R_ENC: begin
          for (int i = 0; i < NL; i = i + 1) begin
            if (i == 0) A2[i] <= gm[0];
            else if (mlt[i]) A2[i] <= A2[i-1] ^ gm[i];
          end
          step <= step + 8'd1;
          if (step == {1'b0, K} - 8'd1) st <= R_IDLE;
        end
        default: st <= R_IDLE;
      endcase
    end
  end
endmodule
