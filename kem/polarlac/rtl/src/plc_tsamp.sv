// plc_tsamp - screened ternary sampling, hash client 0, with plc_fft and plc_tern inside.
// Bit-exact with pke.c: sample_screened_polyvec x2 (+ poly_generate_tenary for e2):
//   candidates n = nonce0, nonce0+1, ... (uint8); candidate = poly_generate_tenary(RG_SD, n);
//   accepted iff fft_within_bound_int16(cand, RL_KEM_T); roles 0..3 filled in acceptance order;
//   role 4 (only with want_e2) = the candidate right after the 4th accepted one, not screened.
// Scheduling: candidates are sampled (plc_tern) and checked (plc_fft) strictly in nonce order.  The sampler
//   runs at most one candidate ahead of the FFT (candidate n+1 is sampled while n is checked), which is
//   enough because the FFT is the slower unit for every set.  Pool slots 8..14 are allocated lowest-free
//   first; a rejected candidate's slot is freed at once.  After the 4th acceptance the speculative candidate
//   becomes e2 (want_e2) or is drained: its hash job is completed with slot writes suppressed, and busy
//   stays high until then.  A new start (only when !busy) frees all pool slots.
// Outputs: role_ok[k] (level) rises when role k is accepted (k<4) / fully written (k=4); role_slot[k];
//   nonce_next = last accepted screened nonce + 1 (valid at done); done pulses one cycle after the last
//   requested role_ok rises.
module plc_tsamp (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              start,
  input  logic [2:0]        pcode,
  input  logic [7:0]        nonce0,
  input  logic              want_e2,
  output logic              busy,
  output logic              done,
  output logic [4:0]        role_ok,
  output logic [4:0][3:0]   role_slot,
  output logic [7:0]        nonce_next,
  // hash job client 0
  output logic              hj_req,
  input  logic              hj_gnt,
  output logic [1:0]        hj_kind0,
  output logic [2:0]        hj_reg0,
  output logic [1:0]        hj_buf0,
  output logic [12:0]       hj_off0,
  output logic [12:0]       hj_len0,
  output logic [1:0]        hj_kind1,
  output logic [2:0]        hj_reg1,
  output logic [1:0]        hj_buf1,
  output logic [12:0]       hj_off1,
  output logic [12:0]       hj_len1,
  output logic [55:0]       hj_sfx,
  output logic [2:0]        hj_sfxlen,
  output logic [6:0]        hj_count,
  output logic [1:0]        hj_cache,
  input  logic              xo_valid,
  output logic              xo_ready,
  input  logic [255:0]      xo_data,
  input  logic              xo_last,
  // slot write port (sampler)
  output logic [3:0]        ts_wslot,
  output logic [1:0]        ts_wen,
  output logic [1:0][5:0]   ts_waddr,
  output logic [1:0][127:0] ts_wdata,
  // slot read port (FFT)
  output logic [3:0]        ts_rslot,
  output logic [1:0]        ts_ren,
  output logic [1:0][5:0]   ts_raddr,
  input  logic [1:0][127:0] ts_rdata
);
  import plc_pkg::*;

  // constant hash descriptor fields: seg0 = RG_SD, 64 bytes, midstate cacheable; no seg1; 5-byte suffix
  assign hj_kind0  = 2'd1;
  assign hj_reg0   = RG_SD;
  assign hj_buf0   = 2'd0;
  assign hj_off0   = 13'd0;
  assign hj_len0   = 13'd64;
  assign hj_kind1  = 2'd0;
  assign hj_reg1   = 3'd0;
  assign hj_buf1   = 2'd0;
  assign hj_off1   = 13'd0;
  assign hj_len1   = 13'd0;
  assign hj_sfxlen = 3'd5;
  assign hj_cache  = 2'd1;

  // ---------------------------------------------------------------- state
  logic        run, done_sent, done_q;
  logic [2:0]  pc;
  logic        e2_r;
  logic [6:0]  used;           // pool slot allocated (slot 8+i)
  logic [2:0]  acc;            // accepted screened candidates (0..4)
  logic [7:0]  s_nonce;        // next nonce to sample
  logic [7:0]  s_ncur;         // nonce being sampled
  logic [2:0]  s_slot;         // pool index being sampled
  logic        w_valid;        // sampled candidate waiting for the FFT
  logic [2:0]  w_slot;
  logic [7:0]  w_nonce;
  logic        f_act;          // FFT running
  logic [2:0]  f_slot;
  logic [7:0]  f_nonce;

  // sub-blocks
  logic t_start, t_busy, t_done, t_discard;
  logic f_start, f_busy, f_done, f_pass;

  // lowest free pool slot
  logic       free_ok;
  logic [2:0] free_idx;
  always_comb begin
    free_ok = 1'b0; free_idx = '0;
    for (int i = NPOOL - 1; i >= 0; i--)
      if (!used[i]) begin free_ok = 1'b1; free_idx = 3'(i); end
  end

  wire acc4      = (acc == 3'd4);
  wire last_acc  = f_done && f_pass && (acc == 3'd3);
  assign t_start = run && !done_sent && !t_busy && !w_valid && !acc4 && !last_acc && free_ok;
  assign f_start = run && w_valid && !acc4 && !f_act;
  assign t_discard = (acc4 && !e2_r) || done_sent;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run <= 1'b0; done_sent <= 1'b0; done_q <= 1'b0; pc <= '0; e2_r <= 1'b0; used <= '0; acc <= '0;
      s_nonce <= '0; s_ncur <= '0; s_slot <= '0; w_valid <= 1'b0; w_slot <= '0; w_nonce <= '0;
      f_act <= 1'b0; f_slot <= '0; f_nonce <= '0; role_ok <= '0; role_slot <= '0; nonce_next <= '0;
    end else begin
      done_q <= 1'b0;
      if (start && !run) begin
        run       <= 1'b1;
        done_sent <= 1'b0;
        pc        <= pcode;
        e2_r      <= want_e2;
        used      <= '0;
        acc       <= '0;
        s_nonce   <= nonce0;
        w_valid   <= 1'b0;
        f_act     <= 1'b0;
        role_ok   <= '0;
        nonce_next <= nonce0;
      end else if (run) begin
        logic [6:0] used_n;
        used_n = used;
        // sampler start
        if (t_start) begin
          used_n[free_idx] = 1'b1;
          s_slot  <= free_idx;
          s_ncur  <= s_nonce;
          s_nonce <= s_nonce + 8'd1;
        end
        // sampler finished: candidate waits for the FFT (or is dropped while draining)
        if (t_done) begin
          if (t_discard) used_n[s_slot] = 1'b0;
          else begin
            w_valid <= 1'b1;
            w_slot  <= s_slot;
            w_nonce <= s_ncur;
          end
        end
        // waiting candidate: to the FFT, or e2 / drop once 4 are accepted
        if (w_valid && acc4) begin
          w_valid <= 1'b0;
          if (e2_r && !role_ok[4]) begin
            role_ok[4]   <= 1'b1;
            role_slot[4] <= SL_POOL0 + 4'(w_slot);
          end else begin
            used_n[w_slot] = 1'b0;
          end
        end else if (f_start) begin
          w_valid <= 1'b0;
          f_act   <= 1'b1;
          f_slot  <= w_slot;
          f_nonce <= w_nonce;
        end
        // FFT verdict
        if (f_done) begin
          f_act <= 1'b0;
          if (f_pass) begin
            role_ok[acc[1:0]]   <= 1'b1;
            role_slot[acc[1:0]] <= SL_POOL0 + 4'(f_slot);
            acc        <= acc + 3'd1;
            nonce_next <= f_nonce + 8'd1;
          end else begin
            used_n[f_slot] = 1'b0;
          end
        end
        used <= used_n;
        // completion and drain
        if (!done_sent && (&role_ok[3:0]) && (!e2_r || role_ok[4])) begin
          done_q    <= 1'b1;
          done_sent <= 1'b1;
        end
        if (done_sent && !t_busy && !t_start && !f_act && !w_valid) run <= 1'b0;
      end
    end
  end

  assign busy = run;
  assign done = done_q;

  // ---------------------------------------------------------------- sampler
  plc_tern u_tern (
    .clk      (clk),
    .rst_n    (rst_n),
    .start    (t_start),
    .pcode    (pc),
    .nonce    (s_nonce),
    .slot     (SL_POOL0 + 4'(free_idx)),
    .discard  (t_discard),
    .busy     (t_busy),
    .done     (t_done),
    .hj_req   (hj_req),
    .hj_gnt   (hj_gnt),
    .hj_sfx   (hj_sfx),
    .hj_count (hj_count),
    .xo_valid (xo_valid),
    .xo_ready (xo_ready),
    .xo_data  (xo_data),
    .wslot    (ts_wslot),
    .wen      (ts_wen),
    .waddr    (ts_waddr),
    .wdata    (ts_wdata)
  );

  // ---------------------------------------------------------------- FFT checker
  plc_fft u_fft (
    .clk      (clk),
    .rst_n    (rst_n),
    .start    (f_start),
    .pcode    (pc),
    .slot     (SL_POOL0 + 4'(w_slot)),
    .busy     (f_busy),
    .done     (f_done),
    .pass     (f_pass),
    .ff_rslot (ts_rslot),
    .ff_ren   (ts_ren),
    .ff_raddr (ts_raddr),
    .ff_rdata (ts_rdata)
  );

  // xo_last / f_busy are not needed for control (digests are counted; f_act tracks the FFT)
  logic unused_ok;
  assign unused_ok = xo_last ^ f_busy;
endmodule
