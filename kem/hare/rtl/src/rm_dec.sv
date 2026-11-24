// Duplicated RM(1,7) soft decoder (reed_muller_decode of the reference).
// Accepts the n1*M 128-bit chunks of v - Truncate(u*y) in order (M = n2/128 copies per symbol).
// Per symbol: sum the M copies bitwise, 7-pass Hadamard (two passes per cycle), t[0] -= 64*M, then
// the first peak and the largest other magnitude (8 values per cycle through a registered merge
// tree, sequential semantics: first index wins ties);
// symbol = (|peak| - second > alpha) ? (peak_idx | (peak > 0) << 7) : 0xFF with erasure flag.
// About 22 cycles per symbol. A symbol completing while the previous one is processed waits in a
// one-entry pending register. rdy (combinational) tells the producer that the pending entry is
// free in the next cycle, so a chunk presented then is always accepted; overflow flags a completion
// with the pending entry still full (cannot happen while the producer obeys rdy).
module rm_dec import hare_pkg::*; (
  input  logic         clk,
  input  logic         rst_n,
  input  logic [1:0]   lvl,
  input  logic         init,
  input  logic         chunk_valid,
  input  logic [127:0] chunk,
  output logic         sym_valid,
  output logic [7:0]   sym_byte,
  output logic         sym_era,
  output logic         overflow,
  output logic         rdy,         // a chunk may be presented in the next cycle
  output logic         act          // working: a chunk is accumulated or a symbol is in the datapath (phase trace)
);
  logic [3:0]  M;
  logic [4:0]  alpha;
  assign M = p_m(lvl);
  assign alpha = p_alpha(lvl);

  logic [3:0]  cnt  [128];         // running copy sums of the symbol being received
  logic [3:0]  psum [128];         // completed symbol waiting for the Hadamard datapath
  logic        pend;
  logic [3:0]  cis;
  logic signed [11:0] had [128];
  typedef enum logic [2:0] {H_IDLE, H_PASS, H_PEAK, H_FLUSH, H_OUT} hst_t;
  hst_t hs;
  assign act = (hs != H_IDLE) || chunk_valid;   // copy accumulation or symbol datapath
  logic [1:0]  pass;
  logic [3:0]  grp;

  typedef struct packed { logic [10:0] mx; logic [10:0] sc; logic [6:0] idx; logic pos; } t2_t;
  function automatic t2_t merge(input t2_t a, input t2_t b);   // a holds earlier indices
    t2_t o;
    if (b.mx > a.mx) begin o = b; o.sc = (a.mx > b.sc) ? a.mx : b.sc; end
    else begin o = a; o.sc = (a.sc > b.mx) ? a.sc : b.mx; end
    return o;
  endfunction
  function automatic t2_t leaf(input logic signed [11:0] v, input [6:0] idx);
    t2_t o;
    o.mx = v[11] ? 11'(-v) : v[10:0];
    o.sc = '0; o.idx = idx; o.pos = (v > 0);
    return o;
  endfunction

  // one Hadamard pass
  typedef logic signed [11:0] hv_t [128];
  function automatic hv_t hpass(input hv_t x);
    hv_t y;
    for (int i = 0; i < 64; i = i + 1) begin
      y[i]      = x[2*i] + x[2*i+1];
      y[i + 64] = x[2*i] - x[2*i+1];
    end
    return y;
  endfunction
  hv_t h1, h2;
  always_comb begin
    h1 = hpass(had);
    h2 = hpass(h1);
  end

  // 8-leaf merge tree over had[0..7] (leaf 0 carries t[0] - 64M in the first group)
  t2_t lf [8];
  t2_t tree, mg_q, run, run_m;
  logic mg_v;
  always_comb begin
    lf[0] = leaf((grp == 4'd0) ? (had[0] - $signed({2'b00, M, 6'd0})) : had[0], {grp, 3'd0});
    for (int i = 1; i < 8; i = i + 1) lf[i] = leaf(had[i], {grp, 3'(i)});
    tree = merge(merge(merge(lf[0], lf[1]), merge(lf[2], lf[3])),
                 merge(merge(lf[4], lf[5]), merge(lf[6], lf[7])));
    run_m = merge(run, mg_q);
  end

  // completion of a received symbol this cycle
  logic sym_done;
  assign sym_done = chunk_valid && (cis == M - 4'd1);
  logic [3:0] sum_now [128];
  always_comb
    for (int i = 0; i < 128; i = i + 1)
      sum_now[i] = ((cis == 4'd0) ? 4'd0 : cnt[i]) + {3'd0, chunk[i]};

  // pending-entry state of the next cycle (same terms as the sequential block below)
  logic free_c, from_pend_c, start_new_c, pend_n;
  always_comb begin
    free_c      = (hs == H_IDLE) || (hs == H_OUT);
    from_pend_c = free_c && pend;
    start_new_c = free_c && !pend && sym_done;
    if (sym_done && !start_new_c) pend_n = 1'b1;
    else if (from_pend_c)         pend_n = 1'b0;
    else                          pend_n = pend;
  end
  assign rdy = !pend_n;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < 128; i = i + 1) begin cnt[i] <= '0; psum[i] <= '0; had[i] <= '0; end
      pend <= 1'b0; cis <= '0; hs <= H_IDLE; pass <= '0; grp <= '0;
      run <= '0; mg_q <= '0; mg_v <= 1'b0;
      sym_valid <= 1'b0; sym_byte <= '0; sym_era <= 1'b0; overflow <= 1'b0;
    end else begin
      sym_valid <= 1'b0;
      if (init) begin
        cis <= '0; hs <= H_IDLE; pend <= 1'b0; overflow <= 1'b0;
      end else begin : body
        logic start_new, from_pend, free_dp;
        // the datapath is free in this cycle if idle or finishing (H_OUT)
        free_dp   = (hs == H_IDLE) || (hs == H_OUT);
        from_pend = free_dp && pend;                  // pending symbol enters first
        start_new = free_dp && !pend && sym_done;     // received symbol enters directly
        // ---------------- accumulation
        if (chunk_valid) begin
          if (cis == M - 4'd1) cis <= '0;
          else begin
            cis <= cis + 4'd1;
            for (int i = 0; i < 128; i = i + 1) cnt[i] <= sum_now[i];
          end
        end
        // ---------------- pending register
        if (sym_done && !start_new) begin
          if (pend && !from_pend) overflow <= 1'b1;
          for (int i = 0; i < 128; i = i + 1) psum[i] <= sum_now[i];
          pend <= 1'b1;
        end else if (from_pend) pend <= 1'b0;
        // ---------------- symbol processing
        case (hs)
          H_PASS: begin
            if (pass == 2'd3) begin
              for (int i = 0; i < 128; i = i + 1) had[i] <= h1[i];        // 7th pass
              hs <= H_PEAK; grp <= '0; run <= '0; mg_v <= 1'b0;
            end else begin
              for (int i = 0; i < 128; i = i + 1) had[i] <= h2[i];        // passes 2p+1, 2p+2
            end
            pass <= pass + 2'd1;
          end
          H_PEAK: begin
            mg_q <= tree; mg_v <= 1'b1;
            if (mg_v) run <= run_m;
            for (int i = 0; i < 120; i = i + 1) had[i] <= had[i+8];
            grp <= grp + 4'd1;
            if (grp == 4'd15) hs <= H_FLUSH;
          end
          H_FLUSH: begin
            run <= run_m; mg_v <= 1'b0; hs <= H_OUT;
          end
          H_OUT: begin
            sym_valid <= 1'b1;
            if ((run.mx - run.sc) > {6'd0, alpha}) begin sym_byte <= {run.pos, run.idx}; sym_era <= 1'b0; end
            else begin sym_byte <= 8'hFF; sym_era <= 1'b1; end
            hs <= H_IDLE;
          end
          default: ;
        endcase
        // ---------------- start of the next symbol (overrides H_OUT -> H_IDLE)
        if (from_pend) begin
          for (int i = 0; i < 128; i = i + 1) had[i] <= $signed({8'd0, psum[i]});
          hs <= H_PASS; pass <= '0;
        end else if (start_new) begin
          for (int i = 0; i < 128; i = i + 1) had[i] <= $signed({8'd0, sum_now[i]});
          hs <= H_PASS; pass <= '0;
        end
      end
    end
  end
endmodule
