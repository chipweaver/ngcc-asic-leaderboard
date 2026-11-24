// Hash jobs PRG / H / G / J (reference hash_bytes with domain tags 1..4) on two SM3 chain engines.
// Each engine pairs an SM3 core with a byte packer that assembles 64-byte blocks from a call's
// segment list (constant bytes, k-byte registers, 32-byte digest slots, object-buffer streams),
// appends SM3 padding (total length includes 512 bits when the call starts from IV1/IV2) and chains
// the compressions. Pseudo-hash variants: E0 runs k1-inner = SM3(IV1; V 00 || M) while E1 runs
// h1 = SM3(M || V 00); then E0 runs the tail (k1, h2, k2.., h3.., k3, k4, h4). Plain SM3: E1 only.
// Output out = h1 || h2 || h3 || h4 (little-endian bytes, byte 0 = first digest byte).
module qube_hash
  import qube_pkg::*;
(
  input  logic          clk,
  input  logic          rst_n,
  input  logic          start,
  input  logic [1:0]    job,          // 0 PRG, 1 H, 2 G, 3 J
  input  logic [2:0]    ps,
  input  logic          hsrc_sk,      // H: 0 = PK buffer at 0, 1 = SK buffer at byte k
  input  logic [511:0]  r_rnda,       // PRG: rho
  input  logic [511:0]  r_gm,         // G: m
  input  logic [511:0]  r_gs,         // G: sigma / salt
  input  logic [511:0]  r_hpk,        // G, J: H(pk)
  input  logic [511:0]  r_rhok,       // J: rho_k
  input  logic          hpk_ok,       // r_hpk holds H(pk) (a packer reaching the H(pk) segment waits)
  output logic          busy,
  output logic          done,
  output logic [1023:0] out,
  // object buffer reads, one port per engine: request until gnt; data with rv
  output logic [1:0]    mrq,
  output logic [1:0]    mbuf [0:1],   // 0 PK, 1 SK, 2 CT
  output logic [9:0]    maddr [0:1],
  input  logic [1:0]    mgnt,
  input  logic [1:0]    mrv,
  input  logic [511:0]  mrdata [0:1]
);
  // ---------------- segment / call descriptors
  localparam logic [2:0] S_NONE = 3'd0, S_CONST = 3'd1, S_REG = 3'd2, S_DIG = 3'd3, S_MEM = 3'd4;
  // registers: 0 rnda, 1 gm, 2 gs, 3 hpk, 4 rhok
  // digest slots: 0 tmp(inner), 1 k1, 2 h1, 3 h2, 4 h3, 5 h4, 6 k2, 7 k3, 8 k4
  typedef struct packed {
    logic [2:0]  typ;
    logic [3:0]  id;        // reg id / slot id / buffer id
    logic [23:0] cval;      // const bytes (byte 0 in [7:0])
    logic [15:0] off;       // MEM byte offset (multiple of 8)
    logic [15:0] len;       // bytes
  } seg_t;
  typedef struct packed {
    logic [1:0]  iv;        // 0 IV0, 1 IV1, 2 IV2
    logic [3:0]  dst;
    seg_t [4:0]  seg;
  } call_t;

  logic [2:0]  var_r;

  function automatic seg_t mk_seg(input logic [2:0] t, input logic [3:0] id, input logic [23:0] cv,
                                  input logic [15:0] off, input logic [15:0] len);
    mk_seg.typ = t; mk_seg.id = id; mk_seg.cval = cv; mk_seg.off = off; mk_seg.len = len;
  endfunction

  // message M of the job as up to 4 segments followed by the tag segment (domain [, V, 00])
  // mode 0: M only (+ domain), 1: V 00 || M (k1-inner), 2: M || V 00 (h1 of a pseudo-hash)
  function automatic call_t msg_call(input logic [1:0] jb, input logic [1:0] mode, input logic [2:0] vv,
                                     input logic [6:0] k, input logic [15:0] pkl, input logic [15:0] ctl,
                                     input logic hsk);
    seg_t m [0:3];
    logic [7:0] dom;
    logic [15:0] kk;
    call_t c;
    kk = {9'd0, k};
    for (int i = 0; i < 4; i++) m[i] = mk_seg(S_NONE, 4'd0, 24'd0, 16'd0, 16'd0);
    case (jb)
      2'd0: begin m[0] = mk_seg(S_REG, 4'd0, 24'd0, 16'd0, kk); dom = 8'h01; end
      2'd1: begin m[0] = mk_seg(S_MEM, hsk ? 4'd1 : 4'd0, 24'd0, hsk ? kk : 16'd0, pkl); dom = 8'h02; end
      2'd2: begin m[0] = mk_seg(S_REG, 4'd1, 24'd0, 16'd0, kk); m[1] = mk_seg(S_REG, 4'd2, 24'd0, 16'd0, kk);
                  m[2] = mk_seg(S_REG, 4'd3, 24'd0, 16'd0, kk); dom = 8'h03; end
      default: begin m[0] = mk_seg(S_REG, 4'd4, 24'd0, 16'd0, kk); m[1] = mk_seg(S_MEM, 4'd2, 24'd0, 16'd0, ctl);
                  m[2] = mk_seg(S_REG, 4'd3, 24'd0, 16'd0, kk); dom = 8'h04; end
    endcase
    c.dst = (mode == 2'd1) ? 4'd0 : 4'd2;
    c.iv  = (mode == 2'd1) ? 2'd1 : 2'd0;
    if (mode == 2'd1) begin
      c.seg[0] = mk_seg(S_CONST, 4'd0, {8'd0, 8'd0, 5'd0, vv}, 16'd0, 16'd2);
      c.seg[1] = m[0]; c.seg[2] = m[1]; c.seg[3] = m[2];
      c.seg[4] = mk_seg(S_CONST, 4'd0, {16'd0, dom}, 16'd0, 16'd1);
    end else begin
      c.seg[0] = m[0]; c.seg[1] = m[1]; c.seg[2] = m[2];
      c.seg[3] = (mode == 2'd2) ? mk_seg(S_CONST, 4'd0, {8'd0, 5'd0, vv, dom}, 16'd0, 16'd3)
                                : mk_seg(S_CONST, 4'd0, {16'd0, dom}, 16'd0, 16'd1);
      c.seg[4] = mk_seg(S_NONE, 4'd0, 24'd0, 16'd0, 16'd0);
    end
    msg_call = c;
  endfunction

  function automatic call_t dig_call(input logic [1:0] iv, input logic [3:0] dst, input logic [1:0] n,
                                     input logic [3:0] a, input logic [3:0] b, input logic [3:0] c3);
    call_t c;
    c.iv = iv; c.dst = dst;
    for (int i = 0; i < 5; i++) c.seg[i] = mk_seg(S_NONE, 4'd0, 24'd0, 16'd0, 16'd0);
    c.seg[0] = mk_seg(S_DIG, a, 24'd0, 16'd0, 16'd32);
    if (n >= 2'd2) c.seg[1] = mk_seg(S_DIG, b, 24'd0, 16'd0, 16'd32);
    if (n == 2'd3) c.seg[2] = mk_seg(S_DIG, c3, 24'd0, 16'd0, 16'd32);
    dig_call = c;
  endfunction

  // tail step t (0-based after the parallel phase)
  function automatic call_t tail_call(input logic [3:0] t);
    case (t)
      4'd0: tail_call = dig_call(2'd2, 4'd1, 2'd1, 4'd0, 4'd0, 4'd0);   // k1 = SM3(IV2; k1i)
      4'd1: tail_call = dig_call(2'd0, 4'd3, 2'd2, 4'd1, 4'd2, 4'd0);   // h2 = SM3(k1 || h1)
      4'd2: tail_call = dig_call(2'd1, 4'd0, 2'd1, 4'd2, 4'd0, 4'd0);   // k2i = SM3(IV1; h1)
      4'd3: tail_call = dig_call(2'd2, 4'd6, 2'd1, 4'd0, 4'd0, 4'd0);   // k2
      4'd4: tail_call = dig_call(2'd0, 4'd4, 2'd2, 4'd3, 4'd6, 4'd0);   // h3 = SM3(h2 || k2)
      4'd5: tail_call = dig_call(2'd1, 4'd0, 2'd1, 4'd3, 4'd0, 4'd0);   // k3i = SM3(IV1; h2)
      4'd6: tail_call = dig_call(2'd2, 4'd7, 2'd1, 4'd0, 4'd0, 4'd0);   // k3
      4'd7: tail_call = dig_call(2'd1, 4'd0, 2'd1, 4'd4, 4'd0, 4'd0);   // k4i = SM3(IV1; h3)
      4'd8: tail_call = dig_call(2'd2, 4'd8, 2'd1, 4'd0, 4'd0, 4'd0);   // k4
      default: tail_call = dig_call(2'd0, 4'd5, 2'd3, 4'd7, 4'd4, 4'd8); // h4 = SM3(k3 || h3 || k4)
    endcase
  endfunction

  // ---------------- sequencer
  logic        run;
  logic        par;           // parallel phase active
  logic [3:0]  tstep, tlast;  // tail step index / last tail step
  logic [1:0]  e_start;
  call_t       e_call [0:1];
  logic [1:0]  e_done;
  logic [1:0]  pdone;
  logic [255:0] dig [0:8];
  logic [255:0] e_dout [0:1];
  logic [3:0]  e_dst [0:1];
  logic        tstep_go, tail_wait;

  always_comb begin
    case (var_r)
      3'd2: tlast = 4'd1;
      3'd3: tlast = 4'd4;
      default: tlast = 4'd9;
    endcase
  end

  // ---------------- engines
  genvar ge;
  generate for (ge = 0; ge < 2; ge++) begin : g_eng
    // call registers
    call_t       c;
    logic        act, first, pad, padlen, fin_wait;
    logic [2:0]  si;
    logic [15:0] sp;
    logic [6:0]  f;
    logic [511:0] blk;
    logic [63:0] lenbits;
    logic [255:0] st;
    logic        core_pend, core_start, core_busy, core_done;
    logic [255:0] core_dout;
    logic [511:0] core_blk;
    logic [255:0] core_iv;
    // memory word cache
    logic [511:0] mw;
    logic [9:0]  mwa;
    logic        mwv, mwp;
    logic        rq;
    logic [9:0]  rqa;
    logic [9:0]  rqa_hold;

    qube_sm3_core u_core (.clk(clk), .rst_n(rst_n), .start(core_start), .abort(1'b0), .iv(core_iv),
      .blk(core_blk), .busy(core_busy), .done(core_done), .dout(core_dout));

    seg_t s;
    logic [15:0] rem, abs;
    logic [511:0] srcv;          // source value (REG / DIG / CONST / MEM word)
    logic [5:0]  bpos;           // byte position in srcv
    logic        srcok, segend, take, blkfull, hand;
    logic [63:0] b8;             // srcv bytes bpos .. bpos + 7
    logic [3:0]  ntk;            // bytes taken this cycle: min(8, rem, 64 - f, 64 - bpos)
    logic [63:0] bmask;          // block bytes written this cycle
    logic [511:0] bdat, bexp;
    always_comb begin
      s = c.seg[si];
      rem = s.len - sp;
      abs = s.off + sp;
      segend = act && !pad && (rem == 16'd0);
      case (s.typ)
        S_CONST: begin srcv = {488'd0, s.cval}; bpos = sp[5:0]; srcok = 1'b1; end
        S_REG: begin
          case (s.id)
            4'd0: srcv = r_rnda; 4'd1: srcv = r_gm; 4'd2: srcv = r_gs; 4'd3: srcv = r_hpk;
            default: srcv = r_rhok;
          endcase
          bpos = sp[5:0]; srcok = (s.id == 4'd3) ? hpk_ok : 1'b1;
        end
        S_DIG: begin srcv = {256'd0, dig[s.id]}; bpos = sp[5:0]; srcok = 1'b1; end
        default: begin srcv = mw; bpos = abs[5:0]; srcok = mwv && (mwa == abs[15:6]); end
      endcase
      b8 = 64'(srcv >> {bpos, 3'd0});
      ntk = 4'd8;
      if (rem < 16'd8) ntk = rem[3:0];
      if ((7'd64 - f) < {3'd0, ntk}) ntk = 4'(7'd64 - f);
      if ((7'd64 - {1'b0, bpos}) < {3'd0, ntk}) ntk = 4'(7'd64 - {1'b0, bpos});
      bmask = ((64'd1 << ntk) - 64'd1) << f[5:0];
      bdat  = {448'd0, b8} << {f[5:0], 3'd0};
      for (int i = 0; i < 64; i++) bexp[8 * i +: 8] = {8{bmask[i]}};
      blkfull = (f == 7'd64);
      take = act && !pad && !segend && !blkfull && srcok;
      // the next block may start in the cycle the core reports done (chaining value from dout)
      hand = act && blkfull && (!core_pend || core_done) && !core_busy;
      core_start = hand;
      core_iv = first ? ((c.iv == 2'd1) ? SM3_IV1 : (c.iv == 2'd2) ? SM3_IV2 : SM3_IV0) :
                (core_done ? core_dout : st);
      for (int i = 0; i < 64; i++) core_blk[511 - 8 * i -: 8] = blk[8 * i +: 8];
      rq = act && !pad && !segend && (s.typ == S_MEM) && !srcok && !mwp;
      rqa = abs[15:6];
    end
    // coverage probes (CH-PAR (a)): unaligned take, hand-off in the core's done cycle
    logic cov_misalign, cov_hand_done, cov_hpk_wait, prb_hpk_take;
    assign cov_misalign  = take && (f[2:0] != bpos[2:0]);
    assign cov_hand_done = hand && core_done && !first;
    // packer at the H(pk) register segment: waiting for hpk_ok / taking H(pk) bytes (CH-NOPRE (c))
    assign cov_hpk_wait  = act && !pad && !segend && !blkfull && (s.typ == S_REG) && (s.id == 4'd3) && !hpk_ok;
    assign prb_hpk_take  = take && (s.typ == S_REG) && (s.id == 4'd3);
    assign mrq[ge] = rq;
    assign maddr[ge] = rqa;
    assign mbuf[ge] = (s.typ == S_MEM && s.id[1:0] != 2'd3) ? s.id[1:0] : 2'd0;   // valid index outside MEM segments
    assign e_dout[ge] = core_dout;

    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        c <= '0; act <= 1'b0; first <= 1'b0; pad <= 1'b0; padlen <= 1'b0; fin_wait <= 1'b0; si <= '0;
        sp <= '0; f <= '0; blk <= '0; lenbits <= '0; st <= '0; core_pend <= 1'b0;
        mw <= '0; mwa <= '0; mwv <= 1'b0; mwp <= 1'b0; e_done[ge] <= 1'b0;
      end else begin
        e_done[ge] <= 1'b0;
        if (e_start[ge]) begin
          c <= e_call[ge]; act <= 1'b1; first <= 1'b1; pad <= 1'b0; padlen <= 1'b0; fin_wait <= 1'b0;
          si <= '0; sp <= '0; f <= '0; blk <= '0; mwv <= 1'b0;
          lenbits <= {45'd0, (e_call[ge].seg[0].len + e_call[ge].seg[1].len + e_call[ge].seg[2].len +
                       e_call[ge].seg[3].len + e_call[ge].seg[4].len), 3'd0} +
                     ((e_call[ge].iv != 2'd0) ? 64'd512 : 64'd0);
        end else if (act) begin
          // memory word fetch
          if (rq && mgnt[ge]) mwp <= 1'b1;
          if (mrv[ge]) begin mw <= mrdata[ge]; mwa <= rqa_hold; mwv <= 1'b1; mwp <= 1'b0; end
          // data bytes
          if (take) begin
            blk <= (blk & ~bexp) | (bdat & bexp); f <= f + {3'd0, ntk}; sp <= sp + {12'd0, ntk};
          end
          if (segend) begin
            sp <= '0;
            if (si == 3'd4) pad <= 1'b1; else si <= si + 3'd1;
          end
          // padding
          if (pad && !padlen && !blkfull && !fin_wait) begin
            // 0x80 then zeros; length if it fits
            for (int i = 0; i < 64; i++) begin
              if (i == f) blk[8 * i +: 8] <= 8'h80;
              else if (i > f) blk[8 * i +: 8] <= 8'h00;
            end
            if (f <= 7'd55) begin
              for (int i = 0; i < 8; i++) blk[8 * (56 + i) +: 8] <= lenbits[63 - 8 * i -: 8];
              padlen <= 1'b1;
            end else fin_wait <= 1'b1;   // 0x80 completes this block; length goes in the next
            f <= 7'd64;
          end
          if (pad && fin_wait && !blkfull) begin
            blk <= '0;
            for (int i = 0; i < 8; i++) blk[8 * (56 + i) +: 8] <= lenbits[63 - 8 * i -: 8];
            padlen <= 1'b1; fin_wait <= 1'b0; f <= 7'd64;
          end
        end
        if (core_done) begin
          core_pend <= 1'b0; st <= core_dout;
        end
        // block hand-off (after the done clear: a hand-off in the done cycle keeps core_pend set)
        if (act && !e_start[ge] && hand) begin
          core_pend <= 1'b1; first <= 1'b0; f <= '0;
          if (padlen) act <= 1'b0;      // final block in flight
        end
        if (core_done && !act && core_pend) e_done[ge] <= 1'b1;
      end
    end
    // address of the outstanding memory request
    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) rqa_hold <= '0;
      else if (rq && mgnt[ge]) rqa_hold <= rqa;
    end
    assign e_dst[ge] = c.dst;
  end endgenerate

  function automatic logic [255:0] be2le(input logic [255:0] x);
    for (int i = 0; i < 32; i++) be2le[8 * i +: 8] = x[255 - 8 * i -: 8];
  endfunction

  // ---------------- job sequencing
  always_comb begin
    e_start = 2'b00;
    e_call[0] = msg_call(job, 2'd1, p_var2k(ps), p_k(ps), p_pkl(ps), p_ctl(ps), hsrc_sk);
    e_call[1] = msg_call(job, 2'd0, 3'd1, p_k(ps), p_pkl(ps), p_ctl(ps), hsrc_sk);
    if (start) begin
      // variant of this job
      if (((job == 2'd0 || job == 2'd2) ? p_var2k(ps) : p_vark(ps)) == 3'd1) begin
        e_call[1] = msg_call(job, 2'd0, 3'd1, p_k(ps), p_pkl(ps), p_ctl(ps), hsrc_sk);
        e_start = 2'b10;
      end else begin
        e_call[0] = msg_call(job, 2'd1, (job == 2'd0 || job == 2'd2) ? p_var2k(ps) : p_vark(ps),
                             p_k(ps), p_pkl(ps), p_ctl(ps), hsrc_sk);
        e_call[1] = msg_call(job, 2'd2, (job == 2'd0 || job == 2'd2) ? p_var2k(ps) : p_vark(ps),
                             p_k(ps), p_pkl(ps), p_ctl(ps), hsrc_sk);
        e_start = 2'b11;
      end
    end else if (run && !par && tstep_go) begin
      e_call[0] = tail_call(tstep);
      e_start = 2'b01;
    end
  end

  assign tstep_go = !tail_wait;
  assign busy = run;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run <= 1'b0; par <= 1'b0; tstep <= '0; tail_wait <= 1'b0; done <= 1'b0; pdone <= '0;
      var_r <= '0;
      for (int i = 0; i < 9; i++) dig[i] <= '0;
    end else begin
      done <= 1'b0;
      for (int e = 0; e < 2; e++) if (e_done[e]) dig[e_dst[e]] <= be2le(e_dout[e]);
      if (start) begin
        run <= 1'b1; par <= 1'b1; tstep <= '0; tail_wait <= 1'b0;
        var_r <= (job == 2'd0 || job == 2'd2) ? p_var2k(ps) : p_vark(ps);
        pdone <= ((((job == 2'd0 || job == 2'd2) ? p_var2k(ps) : p_vark(ps)) == 3'd1) ? 2'b01 : 2'b00);
      end else if (run) begin
        if (par) begin
          if ((pdone | e_done) == 2'b11) begin
            par <= 1'b0;
            if (var_r == 3'd1) begin run <= 1'b0; done <= 1'b1; end
          end
          pdone <= pdone | e_done;
        end else begin
          if (e_start[0]) tail_wait <= 1'b1;
          if (e_done[0]) begin
            tail_wait <= 1'b0;
            if (tstep == tlast) begin run <= 1'b0; done <= 1'b1; end
            else tstep <= tstep + 4'd1;
          end
        end
      end
    end
  end

  assign out = {dig[5], dig[4], dig[3], dig[2]};
endmodule
