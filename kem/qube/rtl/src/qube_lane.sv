// Ring lane, W = 1024-bit words. Two dense slots (each 2 x SRAM1RW_256X512, word q = {hi, lo} at
// address q), a modular accumulator ACC split into an even and an odd word bank (each bank
// 2 x SRAM1RW_128X512, word j in bank j[0] at address j[7:1]) and a support list SUP (1R1W).
//
// A program computes ACC = sum_{s in A} rot(slot0, s) + sum_{s in B} rot(slot1, s) + sum_{s in I} x^s
// mod (X^n - 1). Stream A takes its supports from SUP at a_base, a_base + a_step, ... (a_cnt of them),
// stream B likewise from b_base (b_cnt), the injected sparse terms I from i_base (i_cnt, step 1).
// Each pass takes the next support of A and the next support of B (paired pass; a stream that is
// exhausted contributes nothing) and up to NINJ injected terms, and sweeps the destination words
// j = 0 .. NW-1 in ascending order; the first pass overwrites ACC. When A and B are exhausted and
// injected terms remain, sweep passes without streams absorb them.
//
// Per stream with support s (jx = s / W, bx = s mod W, t = n - s, tw = t / W, tb = t mod W) the
// destination word j receives
//   j <  jx : {src[j+tw+1], src[j+tw]} >> tb                      (bits p < s, source p + n - s)
//   j == jx : the same for bits < bx, plus {src[0], 0} >> (W - bx) for bits >= bx
//   j >  jx : {src[j-jx], src[j-jx-1]} >> (W - bx)                 (bits p >= s, source p - s)
// with src[q] = 0 for q >= NW (source bits >= n are zero in the slots). The first token of a pass
// reads src[tw] (window priming); a junction word with bx != 0 takes a second token (sub 1) that
// reloads the window with {src[0], 0}; the other stream idles in that token. Destination bits >= n of
// word NW-1 are cleared. A pass therefore issues 1 + NW + (number of distinct split junction words)
// tokens without any stall.
//
// Token pipeline: s0 drives the slot read pins, s1 receives Q and updates the per-stream window
// {cur, prev}, s2 shifts by the 64-bit multiple of the offset (0 .. 1024), s3 shifts by the remaining
// 0..63 bits, XORs both streams, the injected bits and the partial of a split word, and reads the ACC
// word of an emitting token; s4 writes ACC[j] = Q ^ contribution (or the contribution on the first
// pass). Consecutive emitting tokens have consecutive destination words, so the ACC read of word j+1
// and the write of word j in one cycle always address different banks; across a pass boundary the
// priming token separates word NW-1 from word 0.
module qube_lane
  import qube_pkg::*;
#(parameter int NINJ = 4)
(
  input  logic          clk,
  input  logic          rst_n,
  input  logic [2:0]    ps,
  // program
  input  logic          start,
  input  logic [9:0]    a_base,
  input  logic          a_step2,       // stream A takes every second support
  input  logic [8:0]    a_cnt,
  input  logic [9:0]    b_base,
  input  logic          b_step2,
  input  logic [8:0]    b_cnt,
  input  logic [9:0]    i_base,
  input  logic [8:0]    i_cnt,
  input  logic [1:0]    slot_rdy,
  output logic          busy,
  output logic          done,
  // supports
  input  logic          sup_rst,
  input  logic          sup_we,
  input  logic [17:0]   sup_wdata,
  // slot write ports (loaders), one per slot
  input  logic [1:0]    sw_en,
  input  logic [8:0]    sw_addr [0:1],
  input  logic [511:0]  sw_data [0:1],
  // ACC external read port (lane idle), 512-bit half-word h: data on ar_data in the next cycle
  input  logic          ar_en,
  input  logic [8:0]    ar_addr,
  output logic [511:0]  ar_data
);
  localparam logic [10:0] WFULL = 11'd1024;

  // ---------------- parameters (latched at program start: local copies keep the controller's
  // parameter register off the lane's datapath timing)
  logic [17:0] n_p;
  logic [7:0]  nw_p;
  logic [2:0]  ps_l;
  // valid bits of word NW-1 (p_lastk: 821 / 781 / 691 / 837 / 683), as constants per set
  logic [1023:0] lastmask;
  always_comb begin
    case (ps_l)
      3'd1: lastmask = {{(1024 - 781){1'b0}}, {781{1'b1}}};
      3'd2: lastmask = {{(1024 - 691){1'b0}}, {691{1'b1}}};
      3'd3: lastmask = {{(1024 - 837){1'b0}}, {837{1'b1}}};
      3'd4: lastmask = {{(1024 - 683){1'b0}}, {683{1'b1}}};
      default: lastmask = {{(1024 - 821){1'b0}}, {821{1'b1}}};
    endcase
  end

  // ---------------- SUP memory and support fetch
  logic [9:0]  wptr;
  logic        sup_we_r;
  logic [17:0] sup_wd_r, sup_q;
  logic        sup_re;
  logic [9:0]  sup_ra;
  SRAM1R1W_1024X18 u_sup (.CLK(clk), .RE(sup_re), .RA(sup_ra), .Q(sup_q), .WE(sup_we_r), .WA(wptr), .D(sup_wd_r));

  logic        run, firstp;
  logic        a_s2, b_s2;
  logic [9:0]  a_ptr, b_ptr, i_ptr;
  logic [8:0]  a_left, b_left, i_left;
  logic        an_v, bn_v;                // next support of A / B captured
  logic [17:0] an_s, bn_s;
  logic [17:0] iq [0:NINJ-1];             // injected terms fetched, not yet assigned to a pass
  logic [2:0]  iq_n;
  logic        rd_v;                      // SUP read issued last cycle
  logic [1:0]  rd_tag;                    // 0 A, 1 B, 2 I
  logic        pendA, pendB, pendI;
  assign pendA = rd_v && rd_tag == 2'd0;
  assign pendB = rd_v && rd_tag == 2'd1;
  assign pendI = rd_v && rd_tag == 2'd2;

  logic canA, canB, canI;
  always_comb begin
    canA = run && !an_v && !pendA && (a_left != 9'd0) && (a_ptr < wptr);
    canB = run && !bn_v && !pendB && (b_left != 9'd0) && (b_ptr < wptr);
    canI = run && ({1'b0, iq_n} + (pendI ? 4'd1 : 4'd0) < 4'(NINJ)) && (i_left != 9'd0) && (i_ptr < wptr);
    sup_re = canA || canB || canI;
    sup_ra = canA ? a_ptr : canB ? b_ptr : i_ptr;
  end
  logic aex, bex;                         // stream exhausted (no support left to take)
  assign aex = (a_left == 9'd0) && !an_v && !pendA;
  assign bex = (b_left == 9'd0) && !bn_v && !pendB;
  logic irem;                             // injected terms remain
  assign irem = (i_left != 9'd0) || (iq_n != 3'd0) || pendI;

  // ---------------- pass descriptors
  typedef struct packed {
    logic        act;
    logic [7:0]  jx;
    logic [9:0]  bx;
    logic [7:0]  tw;
    logic [9:0]  tb;
  } strm_t;
  typedef struct packed {
    strm_t       a, b;
    logic        sweep;      // no stream: injection-only pass
    logic        ovw;
    logic [NINJ-1:0] iv;
    logic [NINJ-1:0][7:0] ij;
    logic [NINJ-1:0][9:0] ib;
  } pass_t;

  function automatic strm_t mk_strm(input logic act, input logic [17:0] s, input logic [17:0] n);
    logic [17:0] t;
    t = n - s;
    mk_strm.act = act; mk_strm.jx = s[17:10]; mk_strm.bx = s[9:0]; mk_strm.tw = t[17:10]; mk_strm.tb = t[9:0];
  endfunction

  logic  nx_v, cp_v;
  pass_t nx, cp;
  logic  cp_prime, cp_sub;
  logic [7:0] cp_j;

  // next-pass load
  logic stream_rdy, sweep_rdy, nx_load;
  always_comb begin
    stream_rdy = (an_v || aex) && (bn_v || bex) && (an_v || bn_v) &&
                 (!an_v || slot_rdy[0]) && (!bn_v || slot_rdy[1]);
    sweep_rdy  = aex && bex && (iq_n != 3'd0) && ((iq_n == 3'(NINJ)) || ((i_left == 9'd0) && !pendI));
    nx_load    = run && !nx_v && (stream_rdy || sweep_rdy);
  end
  pass_t nx_new;
  always_comb begin
    nx_new = '0;
    nx_new.a = mk_strm(an_v, an_s, n_p);
    nx_new.b = mk_strm(bn_v, bn_s, n_p);
    nx_new.sweep = !(an_v || bn_v);
    nx_new.ovw = firstp && (an_v || bn_v);
    for (int k = 0; k < NINJ; k++) begin
      nx_new.iv[k] = (3'(k) < iq_n);
      nx_new.ij[k] = iq[k][17:10];
      nx_new.ib[k] = iq[k][9:0];
    end
  end

  // ---------------- token generation
  typedef struct packed {
    logic        upd;       // window update (slot read or zero)
    logic        rd;        // read src[q] (else zero data)
    logic        zp;        // prev := 0 (phase-1 window reload)
    logic        en;        // contributes to this token
    logic [8:0]  q;
    logic [10:0] off;       // window offset 0 .. 1024
  } sx_t;
  typedef struct packed {
    logic        v;
    logic        emit;      // last token of destination word j
    logic        second;    // XOR the partial of the previous token (split word)
    logic        ovw;
    logic        mlast;
    logic [7:0]  j;
    sx_t         a, b;
    logic [NINJ-1:0] ih;
    logic [NINJ-1:0][9:0] ib;
  } tok_t;

  function automatic logic split_at(input strm_t x, input logic [7:0] j);
    split_at = x.act && (x.jx == j) && (x.bx != 10'd0);
  endfunction

  function automatic sx_t tok_strm(input strm_t x, input logic prime, input logic sub, input logic [7:0] j,
                                   input logic [7:0] nw);
    sx_t r;
    logic [8:0] q2;
    r = '0;
    q2 = {1'b0, j} + {1'b0, x.tw} + 9'd1;
    if (!x.act) r = '0;
    else if (prime) begin
      r.upd = 1'b1; r.rd = 1'b1; r.q = {1'b0, x.tw};
    end else if (!sub) begin
      r.upd = 1'b1; r.en = 1'b1;
      if (j < x.jx || (j == x.jx && x.bx != 10'd0)) begin
        r.q = q2; r.rd = (q2 < {1'b0, nw}); r.off = {1'b0, x.tb};
      end else if (j == x.jx) begin
        r.q = 9'd0; r.rd = 1'b1; r.zp = 1'b1; r.off = WFULL;
      end else begin
        r.q = {1'b0, j - x.jx}; r.rd = 1'b1;
        r.off = (x.bx == 10'd0) ? WFULL : (WFULL - {1'b0, x.bx});
      end
    end else if (split_at(x, j)) begin
      r.upd = 1'b1; r.en = 1'b1; r.q = 9'd0; r.rd = 1'b1; r.zp = 1'b1; r.off = WFULL - {1'b0, x.bx};
    end
    tok_strm = r;
  endfunction

  tok_t tk;
  logic tsplit, tend, tissue;
  always_comb begin
    tk = '0;
    tsplit = !cp_prime && (split_at(cp.a, cp_j) || split_at(cp.b, cp_j));
    tk.v = 1'b1;
    tk.a = tok_strm(cp.a, cp_prime, cp_sub, cp_j, nw_p);
    tk.b = tok_strm(cp.b, cp_prime, cp_sub, cp_j, nw_p);
    tk.emit = !cp_prime && (cp_sub || !tsplit);
    tk.second = cp_sub;
    tk.ovw = cp.ovw;
    tk.j = cp_j;
    tk.mlast = (cp_j == nw_p - 8'd1);
    for (int k = 0; k < NINJ; k++) begin
      tk.ih[k] = tk.emit && cp.iv[k] && (cp.ij[k] == cp_j);
      tk.ib[k] = cp.ib[k];
    end
    tissue = run && cp_v;
    tend = !cp_prime && (cp_sub || !tsplit) && (cp_j == nw_p - 8'd1);
  end

  // ---------------- slot SRAMs (word = {hi macro, lo macro})
  tok_t s0, s1, s2, s3;
  logic         d_ce [0:1][0:1], d_we [0:1][0:1];
  logic [7:0]   d_a  [0:1];
  logic [511:0] d_q  [0:1][0:1];
  always_comb for (int x = 0; x < 2; x++) begin
    sx_t sx;
    sx = (x == 0) ? s0.a : s0.b;
    if (sw_en[x]) begin
      for (int h = 0; h < 2; h++) begin
        d_ce[x][h] = (sw_addr[x][0] == 1'(h)); d_we[x][h] = 1'b1;
      end
      d_a[x] = sw_addr[x][8:1];
    end else begin
      for (int h = 0; h < 2; h++) begin
        d_ce[x][h] = s0.v && sx.upd && sx.rd; d_we[x][h] = 1'b0;
      end
      d_a[x] = sx.q[7:0];
    end
  end
  genvar gx, gh, gk;
  generate for (gx = 0; gx < 2; gx++) begin : g_slot
    for (gh = 0; gh < 2; gh++) begin : g_half
      SRAM1RW_256X512 u_d (.CLK(clk), .CE(d_ce[gx][gh]), .WE(d_we[gx][gh]), .A(d_a[gx]), .D(sw_data[gx]),
                           .Q(d_q[gx][gh]));
    end
  end endgenerate

  // ---------------- ACC SRAMs: bank k (word parity), half h
  logic          cw_v, cw_ovw;
  logic [7:0]    cw_j;
  logic [1023:0] cw;
  logic          a_ce [0:1], a_we [0:1];
  logic [6:0]    a_a  [0:1];
  logic [511:0]  a_q  [0:1][0:1];
  logic [511:0]  a_d  [0:1][0:1];
  logic          a_rd [0:1], a_wr [0:1];
  always_comb for (int k = 0; k < 2; k++) begin
    a_wr[k] = cw_v && (cw_j[0] == 1'(k));
    if (run) a_rd[k] = s3.v && s3.emit && !s3.ovw && (s3.j[0] == 1'(k));
    else     a_rd[k] = ar_en && (ar_addr[1] == 1'(k));
    a_ce[k] = a_wr[k] || a_rd[k];
    a_we[k] = a_wr[k];
    a_a[k]  = a_wr[k] ? cw_j[7:1] : (run ? s3.j[7:1] : ar_addr[8:2]);
    for (int h = 0; h < 2; h++) a_d[k][h] = cw[512 * h +: 512] ^ (cw_ovw ? 512'd0 : a_q[k][h]);
  end
  generate for (gk = 0; gk < 2; gk++) begin : g_bank
    for (gh = 0; gh < 2; gh++) begin : g_acc
      SRAM1RW_128X512 u_acc (.CLK(clk), .CE(a_ce[gk]), .WE(a_we[gk]), .A(a_a[gk]), .D(a_d[gk][gh]),
                             .Q(a_q[gk][gh]));
    end
  end endgenerate
  logic ar_bk, ar_hf;
  assign ar_data = a_q[ar_bk][ar_hf];

`ifndef SYNTHESIS
  // the read of word j+1 and the write of word j never share a bank
  always @(posedge clk) if (rst_n && run)
    for (int k = 0; k < 2; k++)
      if (a_wr[k] && a_rd[k]) $error("qube_lane: ACC bank %0d read and write in one cycle", k);
`endif

  // ---------------- stream windows, shifts, combine
  logic [1023:0] cur_a, prev_a, cur_b, prev_b;
  logic [1086:0] crs_a, crs_b;
  logic [1023:0] pacc;
  logic [2047:0] fa, fb;
  logic [1023:0] ca, cb, inj, part;
  always_comb begin
    fa = {cur_a, prev_a} >> {s2.a.off[10:6], 6'd0};
    fb = {cur_b, prev_b} >> {s2.b.off[10:6], 6'd0};
    ca = s3.a.en ? 1024'(crs_a >> s3.a.off[5:0]) : 1024'd0;
    cb = s3.b.en ? 1024'(crs_b >> s3.b.off[5:0]) : 1024'd0;
    inj = '0;
    for (int k = 0; k < NINJ; k++) if (s3.ih[k]) inj = inj ^ (1024'd1 << s3.ib[k]);
    part = ca ^ cb ^ inj ^ (s3.second ? pacc : 1024'd0);
    if (s3.mlast) part = part & lastmask;
  end

  // ---------------- probes
  // coverage (verification plan CH-PAR): unloaded observation nets
  logic cov_pass_pair, cov_pass_single, cov_sweep, cov_split_a, cov_split_b, cov_split_both, cov_zero_rd,
        cov_jx0_bx0, cov_bx0, cov_split_last, cov_inj, cov_inj_multi, cov_bank_rw, cov_ovw_inj;
  logic pass_start;
  assign pass_start = run && nx_v && (!cp_v || (tissue && tend));
  assign cov_pass_pair   = pass_start && nx.a.act && nx.b.act;
  assign cov_pass_single = pass_start && (nx.a.act ^ nx.b.act);
  assign cov_sweep       = pass_start && nx.sweep;
  assign cov_split_a     = tissue && !cp_prime && !cp_sub && split_at(cp.a, cp_j) && !split_at(cp.b, cp_j);
  assign cov_split_b     = tissue && !cp_prime && !cp_sub && split_at(cp.b, cp_j) && !split_at(cp.a, cp_j);
  assign cov_split_both  = tissue && !cp_prime && !cp_sub && split_at(cp.a, cp_j) && split_at(cp.b, cp_j);
  assign cov_zero_rd     = tissue && ((tk.a.upd && !tk.a.rd) || (tk.b.upd && !tk.b.rd));
  assign cov_jx0_bx0     = pass_start && ((nx.a.act && nx.a.jx == 8'd0 && nx.a.bx == 10'd0) ||
                                          (nx.b.act && nx.b.jx == 8'd0 && nx.b.bx == 10'd0));
  assign cov_bx0         = pass_start && ((nx.a.act && nx.a.bx == 10'd0) || (nx.b.act && nx.b.bx == 10'd0));
  assign cov_split_last  = tsplit && tissue && !cp_sub && (cp_j == nw_p - 8'd1);
  assign cov_inj         = tissue && (|tk.ih);
  assign cov_inj_multi   = tissue && ((tk.ih & (tk.ih - 1'b1)) != '0);
  assign cov_bank_rw     = (a_wr[0] && a_rd[1]) || (a_wr[1] && a_rd[0]);
  assign cov_ovw_inj     = tissue && tk.ovw && (|tk.ih);

  // phase probe: first token of the program (prb_grp_new, group 0), last token of the last paired /
  // single pass (prb_grp_end, group 0), first and last token of the injection sweeps (group 1)
  logic       prb_grp_new, prb_grp_end;
  logic [1:0] prb_grp, prb_grp_e;
  logic       prb_first, prb_sweeping;
  assign prb_grp_new = tissue && (prb_first || (cp.sweep && cp_prime && !prb_sweeping));
  assign prb_grp     = prb_first ? 2'd0 : 2'd1;
  assign prb_grp_end = tissue && tend && (cp.sweep ? (!irem && !nx_v) : (aex && bex && !(nx_v && !nx.sweep)));
  assign prb_grp_e   = cp.sweep ? 2'd1 : 2'd0;

  logic pipe_empty;
  assign pipe_empty = !s0.v && !s1.v && !s2.v && !s3.v && !cw_v;
  assign busy = run;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wptr <= '0; sup_we_r <= 1'b0; sup_wd_r <= '0; n_p <= '0; nw_p <= '0; ps_l <= '0;
      run <= 1'b0; done <= 1'b0; firstp <= 1'b0; a_s2 <= 1'b0; b_s2 <= 1'b0;
      a_ptr <= '0; b_ptr <= '0; i_ptr <= '0; a_left <= '0; b_left <= '0; i_left <= '0;
      an_v <= 1'b0; bn_v <= 1'b0; an_s <= '0; bn_s <= '0;
      for (int k = 0; k < NINJ; k++) iq[k] <= '0;
      iq_n <= '0; rd_v <= 1'b0; rd_tag <= '0;
      nx_v <= 1'b0; cp_v <= 1'b0; nx <= '0; cp <= '0; cp_prime <= 1'b0; cp_sub <= 1'b0; cp_j <= '0;
      s0 <= '0; s1 <= '0; s2 <= '0; s3 <= '0;
      cur_a <= '0; prev_a <= '0; cur_b <= '0; prev_b <= '0; crs_a <= '0; crs_b <= '0; pacc <= '0;
      cw_v <= 1'b0; cw_ovw <= 1'b0; cw_j <= '0; cw <= '0; ar_bk <= 1'b0; ar_hf <= 1'b0;
      prb_first <= 1'b0; prb_sweeping <= 1'b0;
    end else begin
      done <= 1'b0;
      // support writes (sampler) and pointer reset
      sup_we_r <= sup_we; sup_wd_r <= sup_wdata;
      if (sup_rst) begin wptr <= '0; sup_we_r <= 1'b0; end
      else if (sup_we_r) wptr <= wptr + 10'd1;

      if (start) begin
        n_p <= p_n(ps); nw_p <= p_nwk(ps); ps_l <= ps;
        run <= 1'b1; firstp <= 1'b1; prb_first <= 1'b1; prb_sweeping <= 1'b0;
        a_ptr <= a_base; a_left <= a_cnt; a_s2 <= a_step2;
        b_ptr <= b_base; b_left <= b_cnt; b_s2 <= b_step2;
        i_ptr <= i_base; i_left <= i_cnt;
        an_v <= 1'b0; bn_v <= 1'b0; iq_n <= '0; rd_v <= 1'b0;
        nx_v <= 1'b0; cp_v <= 1'b0;
      end else if (run) begin
        // ---- support fetch: issue
        rd_v <= sup_re;
        if (canA) begin rd_tag <= 2'd0; a_ptr <= a_ptr + (a_s2 ? 10'd2 : 10'd1); a_left <= a_left - 9'd1; end
        else if (canB) begin rd_tag <= 2'd1; b_ptr <= b_ptr + (b_s2 ? 10'd2 : 10'd1); b_left <= b_left - 9'd1; end
        else if (canI) begin rd_tag <= 2'd2; i_ptr <= i_ptr + 10'd1; i_left <= i_left - 9'd1; end
        // ---- next pass load (takes A, B and all fetched injected terms)
        begin
          logic [2:0] qn;
          qn = iq_n;
          if (nx_load) begin
            nx_v <= 1'b1; nx <= nx_new;
            if (an_v || bn_v) firstp <= 1'b0;
            an_v <= 1'b0; bn_v <= 1'b0; qn = 3'd0;
          end
          // ---- support fetch: capture
          if (pendA) begin an_v <= 1'b1; an_s <= sup_q; end
          if (pendB) begin bn_v <= 1'b1; bn_s <= sup_q; end
          if (pendI) begin iq[qn] <= sup_q; qn = qn + 3'd1; end
          iq_n <= qn;
        end
        // ---- token issue
        if (tissue) begin
          prb_first <= 1'b0;
          if (cp.sweep && cp_prime) prb_sweeping <= 1'b1;
          if (cp_prime) cp_prime <= 1'b0;
          else if (!cp_sub && tsplit) cp_sub <= 1'b1;
          else begin
            cp_sub <= 1'b0;
            if (tend) cp_v <= 1'b0;
            else cp_j <= cp_j + 8'd1;
          end
        end
        if (nx_v && (!cp_v || (tissue && tend))) begin
          cp_v <= 1'b1; cp <= nx; cp_sub <= 1'b0; cp_j <= '0;
          cp_prime <= 1'b1;   // also for sweeps: separates word NW-1 from word 0 (ACC banks)
          if (!nx_load) nx_v <= 1'b0;
        end
        // ---- completion
        if (aex && bex && !irem && !nx_v && !cp_v && pipe_empty) begin
          run <= 1'b0; done <= 1'b1;
        end
      end
      // ---- pipeline
      if (!run && ar_en) begin ar_bk <= ar_addr[1]; ar_hf <= ar_addr[0]; end
      s0 <= tissue ? tk : '0;
      s1 <= s0;
      s2 <= s1;
      s3 <= s2;
      if (s1.v && s1.a.upd) begin
        prev_a <= s1.a.zp ? 1024'd0 : cur_a;
        cur_a  <= s1.a.rd ? {d_q[0][1], d_q[0][0]} : 1024'd0;
      end
      if (s1.v && s1.b.upd) begin
        prev_b <= s1.b.zp ? 1024'd0 : cur_b;
        cur_b  <= s1.b.rd ? {d_q[1][1], d_q[1][0]} : 1024'd0;
      end
      if (s2.v) begin
        crs_a <= fa[1086:0];
        crs_b <= fb[1086:0];
      end
      if (s3.v && !s3.emit) pacc <= part;
      cw_v <= s3.v && s3.emit;
      if (s3.v && s3.emit) begin cw <= part; cw_j <= s3.j; cw_ovw <= s3.ovw; end
    end
  end
endmodule
