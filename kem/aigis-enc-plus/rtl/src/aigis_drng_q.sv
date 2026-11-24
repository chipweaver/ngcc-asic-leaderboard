// gen_a quarter q: SM3 DRNG (kat_test/drng.c) seeded with publicseed || q, rejection sampling
// (gen_a.c rej_uniform / poly_uniform_seed) of N = n/4 coefficients into lanes 4q..4q+3 of slot A.
//
// Three SM3 cores per quarter.
//   Core 2 (Y): SM3_df(seed||q, c=2), then the V-update chain H_k = SM3(0x03 || V_k),
//               V_{k+1} = V_k + H_k + C + k (440-bit big-endian), one 2-block job per generate.
//   Cores 0, 1: SM3_df(seed||q, c=1) (core 0), SM3_df(0x00||V_1, c=1/2) (cores 0/1) for C, then
//               the output blocks O(k,i) = SM3(V_k + i), i = 0..2, issued in order on whichever
//               output core is free.  All three blocks of every generate are computed
//               speculatively: a 32-byte generate (decided by the sampler after buffer k-1)
//               consumes O(k,0) only and the others are discarded.
// V_k is kept in a two-entry ring (k mod 2); the update producing V_{k+1} is committed only after
// every output block of generate k-1 has been issued (a core latches its block at start).
// An output core keeps its digest (core vout) until the sampler has copied it into its window or
// the block is dead (its generate is behind the sampler); only then is it reused.
// Sampler: one triple per cycle from a 34-byte window, two 12-bit candidates, accepted if < q
// while cur < N (gen_a.c rej_uniform).  The quarter completes when cur reaches N; cores still
// running speculative jobs finish unobserved.
module aigis_drng_q #(parameter logic [1:0] QI = 2'd0) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          start,
  input  logic [1:0]    param,
  input  logic [511:0]  seed,         // seed byte j at bits [8j+7:8j]
  output logic          busy,
  output logic          done,
  output logic [2:0]          cs, cc,
  output logic [2:0][511:0]   cblk,
  input  logic [2:0]          cbusy, cdone, clast,
  input  logic [2:0][255:0]   cv,
  output logic [9:0]       wprog,       // coefficients written (registered)
  output logic [7:0]       we,          // local lane l (0..3) bank k at index 2l+k
  output logic [7:0][5:0]  wa,
  output logic [7:0][11:0] wd
);
  import aigis_pkg::*;

  logic [1:0] p;
  logic [6:0] b;
  logic [9:0] N;
  logic [2:0] lw;
  assign b  = bbytes(p);
  assign N  = 10'd128 << (p - 2'd1);
  assign lw = 3'd5 + {1'b0, p - 2'd1};

  logic [7:0] sbt [0:63];
  always_comb for (int j = 0; j < 64; j++) sbt[j] = seed[8*j +: 8];

  function automatic logic [511:0] put(input logic [511:0] blk, input int pos, input logic [7:0] v);
    put = blk; put[511 - 8*pos -: 8] = v;
  endfunction
  // SM3_df(seed||q, c) message [c, 00 00 01 B8, seed(b), q], b+6 bytes
  function automatic logic [511:0] df1(input logic [7:0] c, input logic blk2, input logic [6:0] bb);
    logic [511:0] r;
    r = '0;
    if (!blk2) begin
      r = put(r, 0, c); r = put(r, 3, 8'h01); r = put(r, 4, 8'hB8);
      for (int j = 0; j < 59; j++) if (j < int'(bb)) r = put(r, 5 + j, sbt[j]);
      if (bb != 7'd64) begin
        r = put(r, 5 + int'(bb), {6'd0, QI}); r = put(r, 6 + int'(bb), 8'h80);
        r[63:0] = 64'(8 * (int'(bb) + 6));
      end
    end else begin
      for (int j = 0; j < 5; j++) r = put(r, j, sbt[59 + j]);
      r = put(r, 5, {6'd0, QI}); r = put(r, 6, 8'h80); r[63:0] = 64'd560;
    end
    df1 = r;
  endfunction
  // SM3_df(0x00 || V, c) message [c, 00 00 01 B8, 00, V(55)], 61 bytes
  function automatic logic [511:0] df2(input logic [7:0] c, input logic blk2, input logic [439:0] v);
    logic [511:0] r;
    r = '0;
    if (!blk2) begin
      r = put(r, 0, c); r = put(r, 3, 8'h01); r = put(r, 4, 8'hB8);
      r[511 - 48 -: 440] = v;
      r = put(r, 61, 8'h80);
    end else r[63:0] = 64'd488;
    df2 = r;
  endfunction
  function automatic logic [511:0] outb(input logic [439:0] v);
    logic [511:0] r;
    r = '0; r[511 -: 440] = v; r = put(r, 55, 8'h80); r[63:0] = 64'd440;
    outb = r;
  endfunction
  function automatic logic [511:0] updb(input logic blk2, input logic [439:0] v);
    logic [511:0] r;
    r = '0;
    if (!blk2) begin r = put(r, 0, 8'h03); r[511 - 8 -: 440] = v; r = put(r, 56, 8'h80); end
    else r[63:0] = 64'd448;
    updb = r;
  endfunction

  // ------------------------------------------------------------- control state
  logic        run;                  // quarter active (start .. complete)
  logic        s_complete;           // N coefficients sampled
  logic [439:0] vr [0:1];            // V_k at index k mod 2
  logic [5:0]  kv;                   // highest k with V_k available (0: none)
  logic [439:0] C;
  logic        c_ok;
  logic        d1x_ok, d1y_ok, d2x_ok, d2y_ok;

  // sampler position (generate / block it consumes next)
  logic [5:0]  sk;                   // generate being sampled (1-based)
  logic [1:0]  si;                   // block within it

  // sampler state (declared early: used by the control block)
  logic        pend_v;
  logic        pend_flush;
  // output issue pointer
  logic [5:0]  ik;
  logic [1:0]  ii;

  // ------------------------------------------------------------- core Y (index 2)
  typedef enum logic [2:0] {Y_IDLE, Y_DF1, Y_WAITV, Y_UPD, Y_ADD, Y_STOP} yst_t;
  yst_t yst;
  logic [5:0]  yk;                   // generate whose update runs
  logic        yb2;                  // second block of the running 2-block job
  logic        ysk;                  // ignore the done pulse of the chained first block
  logic        vs_ok;
  logic [439:0] vsum;
  logic        y_start;
  logic [511:0] y_blk;
  logic        y_two;
  always_comb begin
    y_two = 1'b1;
    if (yst == Y_DF1) begin y_two = (b == 7'd64); y_blk = df1(8'h02, yb2, b); end
    else              y_blk = updb(yb2, vr[yk[0]]);
  end
  // the next update starts in the cycle the current one finishes (V_{k+1} computed from the core
  // output) when C is known and every output block of generate k-1 has been issued
  logic         y_go;
  logic [439:0] vnext;
  assign vnext = vsum + {184'd0, cv[2]} + 440'(yk);
  assign y_go  = run && !s_complete && (yst == Y_UPD) && cdone[2] && yb2 && !ysk && vs_ok && (ik >= yk);
  assign cs[2]   = y_start || y_go;
  // cont only for this quarter's live job (a speculative job left running at completion ends after its block)
  assign cc[2]   = run && !s_complete && !y_start && (yst == Y_DF1 || yst == Y_UPD) && y_two && !yb2 && clast[2];
  assign cblk[2] = y_go ? updb(1'b0, vnext) :
                   (cc[2] || yb2) ? ((yst == Y_DF1) ? df1(8'h02, 1'b1, b) : updb(1'b1, vr[yk[0]])) : y_blk;

  // ------------------------------------------------------------- output cores 0, 1
  // job kinds
  localparam logic [1:0] J_DF1 = 2'd0, J_DF2 = 2'd1, J_OUT = 2'd2;
  logic [1:0]       o_run, o_hold, o_b2, o_sk;
  logic [1:0][1:0]  o_kind;
  logic [1:0][5:0]  o_k;
  logic [1:0][1:0]  o_i;
  logic [1:0]       o_start;
  logic             df1_go;
  logic [1:0][511:0] o_blk;
  logic [1:0]       o_two;
  logic [1:0]       o_free;
  logic             df2_issued;
  // a held block is dead when its generate is behind the sampler
  always_comb begin
    for (int j = 0; j < 2; j++) begin
      o_free[j] = !o_run[j] && !cbusy[j] && (!o_hold[j] || (o_k[j] < sk));
      o_two[j]  = (o_kind[j] == J_DF2) || (o_kind[j] == J_DF1 && b == 7'd64);
      cc[j]     = run && !s_complete && o_run[j] && o_two[j] && !o_b2[j] && clast[j] && !o_start[j] && !(j == 0 && df1_go);
    end
  end

  // issue decision for this cycle
  logic       iss_df1, iss_df2, iss_out;
  logic [1:0] iss_core;          // one-hot
  logic [439:0] vout_in;
  always_comb begin
    iss_df1 = 1'b0; iss_df2 = 1'b0; iss_out = 1'b0; iss_core = 2'b00;
    vout_in = vr[ik[0]] + 440'(ii);
    if (run && !s_complete) begin
      if (!df2_issued && kv != 6'd0) begin
        if (o_free == 2'b11) begin iss_df2 = 1'b1; iss_core = 2'b11; end
      end else if (df2_issued && c_ok && kv >= ik && ik != 6'd0) begin
        if (o_free[0])      begin iss_out = 1'b1; iss_core = 2'b01; end
        else if (o_free[1]) begin iss_out = 1'b1; iss_core = 2'b10; end
      end
    end
    for (int j = 0; j < 2; j++) begin
      o_start[j] = 1'b0; o_blk[j] = outb(vout_in);
      if (iss_core[j]) begin
        o_start[j] = 1'b1;
        o_blk[j] = iss_df2 ? df2(j == 0 ? 8'h01 : 8'h02, 1'b0, vr[1]) : outb(vout_in);
      end else if (cc[j] || o_b2[j]) begin
        o_blk[j] = (o_kind[j] == J_DF1) ? df1(8'h01, 1'b1, b) : df2(8'h00, 1'b1, '0);
      end
    end
    if (iss_df1) begin o_start[0] = 1'b1; o_blk[0] = df1(8'h01, 1'b0, b); end
  end
  // DF1 on core 0 is started by the start pulse itself (all cores idle at start)
  assign cs[0] = o_start[0] || df1_go;
  assign cs[1] = o_start[1];
  assign cblk[0] = df1_go ? df1(8'h01, 1'b0, b) : o_blk[0];
  assign cblk[1] = o_blk[1];

  // ------------------------------------------------------------- sampler interface
  logic        want;                 // sampler wants block (sk, si)
  logic [1:0]  have;                 // core j holds (sk, si) and is done
  logic        take_blk;
  logic [255:0] tblk;
  always_comb begin
    for (int j = 0; j < 2; j++)
      have[j] = o_hold[j] && (o_k[j] == sk) && (o_i[j] == si);
    tblk = have[0] ? cv[0] : cv[1];
  end
  assign take_blk = want && (|have);

  // ------------------------------------------------------------- sequential control
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run <= 1'b0; busy <= 1'b0; done <= 1'b0; p <= 2'd1; kv <= '0; c_ok <= 1'b0;
      d1x_ok <= 1'b0; d1y_ok <= 1'b0; d2x_ok <= 1'b0; d2y_ok <= 1'b0;
      yst <= Y_IDLE; yk <= '0; yb2 <= 1'b0; ysk <= 1'b0; vs_ok <= 1'b0; y_start <= 1'b0; df1_go <= 1'b0;
      o_run <= '0; o_hold <= '0; o_b2 <= '0; o_sk <= '0; o_kind <= '0; o_k <= '0; o_i <= '0;
      ik <= '0; ii <= '0; df2_issued <= 1'b0;
    end else begin
      done <= 1'b0; y_start <= 1'b0; df1_go <= 1'b0;
      if (start && !run) begin
        run <= 1'b1; busy <= 1'b1; p <= param; kv <= '0; c_ok <= 1'b0;
        d1x_ok <= 1'b0; d1y_ok <= 1'b0; d2x_ok <= 1'b0; d2y_ok <= 1'b0;
        yst <= Y_DF1; yb2 <= 1'b0; ysk <= 1'b0; vs_ok <= 1'b0; y_start <= 1'b1; yk <= 6'd1;
        df1_go <= 1'b1;
        o_run <= 2'b01; o_hold <= '0; o_b2 <= '0; o_sk <= '0; o_kind[0] <= J_DF1; o_k <= '0; o_i <= '0;
        ik <= 6'd1; ii <= 2'd0; df2_issued <= 1'b0;
      end else if (run) begin
        // ---- core Y
        case (yst)
          Y_DF1: begin
            if (cc[2]) begin yb2 <= 1'b1; ysk <= 1'b1; end
            if (cdone[2] && ysk) ysk <= 1'b0;
            else if (cdone[2] && (!y_two || yb2)) begin d1y_ok <= 1'b1; yb2 <= 1'b0; yst <= Y_WAITV; end
          end
          Y_WAITV: if (s_complete) yst <= Y_STOP;
            else if (kv >= yk) begin yst <= Y_UPD; y_start <= 1'b1; yb2 <= 1'b0; ysk <= 1'b0; vs_ok <= 1'b0; end
          Y_UPD: begin
            if (cc[2]) begin yb2 <= 1'b1; ysk <= 1'b1; end
            if (cdone[2] && ysk) ysk <= 1'b0;
            else if (y_go) begin
              vr[~yk[0]] <= vnext; kv <= yk + 6'd1; yk <= yk + 6'd1; yb2 <= 1'b0; vs_ok <= 1'b0;
            end
            else if (cdone[2] && yb2) begin yb2 <= 1'b0; yst <= Y_ADD; end
            if (c_ok && !y_go) begin vsum <= vr[yk[0]] + C; vs_ok <= 1'b1; end
          end
          Y_ADD: begin
            // V_{k+1} overwrites V_{k-1}: every output block of generate k-1 must be issued
            if (s_complete) yst <= Y_STOP;
            else if (!vs_ok) begin if (c_ok) begin vsum <= vr[yk[0]] + C; vs_ok <= 1'b1; end end
            else if (ik >= yk) begin
              vr[~yk[0]] <= vsum + {184'd0, cv[2]} + 440'(yk);
              kv <= yk + 6'd1; yk <= yk + 6'd1; yst <= Y_WAITV;
            end
          end
          default: ;
        endcase
        // V_1 / C straight from the core outputs: the two SM3_df jobs of a pair start in the same
        // cycle, have equal length and their cores are not restarted before the capture
        // (core 2 waits for kv >= 1, cores 0/1 for kv / c_ok), so vout is still valid here.
        if (kv == 6'd0 && d1x_ok && d1y_ok) begin vr[1] <= {cv[0], cv[2][255:72]}; kv <= 6'd1; end
        if (!c_ok && d2x_ok && d2y_ok) begin C <= {cv[0], cv[1][255:72]}; c_ok <= 1'b1; end
        // ---- output cores
        for (int j = 0; j < 2; j++) begin
          if (cc[j]) begin o_b2[j] <= 1'b1; o_sk[j] <= 1'b1; end
          if (o_run[j] && cdone[j] && o_sk[j]) o_sk[j] <= 1'b0;
          else if (o_run[j] && cdone[j] && (!o_two[j] || o_b2[j])) begin
            o_run[j] <= 1'b0; o_b2[j] <= 1'b0;
            case (o_kind[j])
              J_DF1: d1x_ok <= 1'b1;
              J_DF2: begin if (j == 0) d2x_ok <= 1'b1; else d2y_ok <= 1'b1; end
              default: o_hold[j] <= 1'b1;
            endcase
          end
          if (have[j] && take_blk) o_hold[j] <= 1'b0;
          if (o_start[j]) begin
            o_run[j] <= 1'b1; o_hold[j] <= 1'b0; o_b2[j] <= 1'b0; o_sk[j] <= 1'b0;
            o_kind[j] <= iss_df2 ? J_DF2 : J_OUT; o_k[j] <= ik; o_i[j] <= ii;
          end
        end
        if (iss_df2) df2_issued <= 1'b1;
        if (iss_out) begin
          if (ii == 2'd2) begin ik <= ik + 6'd1; ii <= 2'd0; end else ii <= ii + 2'd1;
        end
        // issue pointer never lags the sampler (skipped blocks of a 32-byte generate)
        if (!iss_out && (ik < sk)) begin ik <= sk; ii <= 2'd0; end
        // ---- completion
        if (s_complete && !pend_flush) begin run <= 1'b0; busy <= 1'b0; done <= 1'b0; end
        if (s_complete && !pend_flush) done <= 1'b1;
      end
    end
  end

  // ------------------------------------------------------------- sampler
  logic [33:0][7:0] win;
  logic [5:0]  wn;                  // bytes in window
  logic [9:0]  cur;
  logic [5:0]  tleft;               // triples left in this buffer
  logic [1:0]  bleft;               // blocks of this buffer not yet loaded
  logic [9:0]  pend_i;
  logic [11:0] pend_d;
  logic        take;
  logic [11:0] v0, v1;
  logic        a0, a1;
  assign pend_flush = pend_v;
  assign want = run && !s_complete && (bleft != 2'd0) && (wn < 6'd3);
  assign v0 = {win[1][3:0], win[0]};
  assign v1 = {win[2], win[1][7:4]};
  assign take = run && !s_complete && !pend_v && (wn >= 6'd3) && (tleft != 6'd0);
  assign a0 = take && (v0 < 12'd3329) && (cur < N);
  assign a1 = take && (v1 < 12'd3329) && ((cur + {9'd0, a0}) < N);

  function automatic logic [1:0] lanel(input logic [9:0] i, input logic [2:0] l);
    lanel = 2'(i >> l);
  endfunction
  function automatic logic [6:0] wrd(input logic [9:0] i, input logic [2:0] l);
    wrd = 7'(i & ((10'd1 << l) - 10'd1));
  endfunction
  logic        w0v, w1v, conflict;
  logic [9:0]  w0i, w1i;
  logic [11:0] w0d, w1d;
  always_comb begin
    w0v = 1'b0; w1v = 1'b0; w0i = cur; w1i = cur + 10'd1; w0d = v0; w1d = v1;
    if (pend_v) begin w0v = 1'b1; w0i = pend_i; w0d = pend_d; end
    else if (a0) begin w0v = 1'b1; w1v = a1; end
    else if (a1) begin w0v = 1'b1; w0d = v1; end
    conflict = w0v && w1v && (lanel(w0i, lw) == lanel(w1i, lw)) && (wbank(wrd(w0i, lw)) == wbank(wrd(w1i, lw)));
    we = '0; wa = '0; wd = '0;
    if (w0v) begin
      we[2*lanel(w0i, lw) + 32'(wbank(wrd(w0i, lw)))] = 1'b1;
      wa[2*lanel(w0i, lw) + 32'(wbank(wrd(w0i, lw)))] = 6'(wrd(w0i, lw) >> 1);
      wd[2*lanel(w0i, lw) + 32'(wbank(wrd(w0i, lw)))] = w0d;
    end
    if (w1v && !conflict) begin
      we[2*lanel(w1i, lw) + 32'(wbank(wrd(w1i, lw)))] = 1'b1;
      wa[2*lanel(w1i, lw) + 32'(wbank(wrd(w1i, lw)))] = 6'(wrd(w1i, lw) >> 1);
      wd[2*lanel(w1i, lw) + 32'(wbank(wrd(w1i, lw)))] = w1d;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wn <= '0; cur <= '0; tleft <= '0; bleft <= '0; pend_v <= 1'b0; s_complete <= 1'b0; wprog <= '0;
      sk <= '0; si <= '0; win <= '0; pend_i <= '0; pend_d <= '0;
    end else if (start && !run) begin
      wn <= '0; cur <= '0; tleft <= 6'd32; bleft <= 2'd3; pend_v <= 1'b0; s_complete <= 1'b0; wprog <= '0;
      sk <= 6'd1; si <= 2'd0;
    end else if (run) begin
      logic [33:0][7:0] w_n;
      logic [5:0] wn_n;
      logic [9:0] cur_n;
      w_n = win; wn_n = wn;
      if (take) begin
        for (int j = 0; j < 31; j++) w_n[j] = win[j + 3];
        wn_n = wn_n - 6'd3;
      end
      if (take_blk) begin
        for (int j = 0; j < 32; j++) begin
          if (wn_n == 6'd0) w_n[j] = tblk[255 - 8*j -: 8];
          else if (wn_n == 6'd1) w_n[j + 1] = tblk[255 - 8*j -: 8];
          else w_n[j + 2] = tblk[255 - 8*j -: 8];
        end
        wn_n = wn_n + 6'd32;
      end
      win <= w_n; wn <= wn_n;
      if (take_blk) begin bleft <= bleft - 2'd1; si <= si + 2'd1; end
      cur_n = cur + {9'd0, a0} + {9'd0, a1};
      cur <= cur_n;
      wprog <= conflict ? cur_n - 10'd1 : cur_n;
      if (pend_v) pend_v <= 1'b0;
      else if (conflict) begin pend_v <= 1'b1; pend_i <= w1i; pend_d <= w1d; end
      if (take) tleft <= tleft - 6'd1;
      if (!s_complete) begin
        if (cur_n >= N) s_complete <= 1'b1;
        else if (!take && tleft == 6'd0 && bleft == 2'd0) begin
          // buffer sk finished (a 32-byte buffer drops its last 2 bytes): decide generate sk+1
          sk <= sk + 6'd1; si <= 2'd0; wn <= '0;
          tleft <= (cur + 10'd21 < N) ? 6'd32 : 6'd10;
          bleft <= (cur + 10'd21 < N) ? 2'd3 : 2'd1;
        end
      end
    end
  end
`ifndef SYNTHESIS
  // invariant: every core of the quarter is idle when the quarter starts, so the
  // first done pulse after start belongs to this quarter's DF1 jobs
  always @(posedge clk) if (start && !run && |cbusy)
    $error("drng_q%0d: started while a core is busy (%b)", QI, cbusy);
`endif
endmodule
