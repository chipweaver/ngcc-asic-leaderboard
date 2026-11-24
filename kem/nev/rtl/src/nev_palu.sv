// nev_palu: dual-issue 32-lane polynomial ALU (address generation, lane permutation, write-back).
// Two 16-lane halves are issued in the same cycle: half A uses read ports 0/1 and write ports 0/1,
// half B read ports 2/3 and write ports 2/3. NTT/INTT: half A processes butterfly pair pp and half B
// pair pp+1 of the same row and level; SCALE/PROD/EXP/ZCHK: half A slot group g, half B group g+1.
// The four words touched in one cycle lie in four different banks of nev_pbuf.
// Timing contract with nev_pbuf: a read issued in cycle t returns data in t+2; lane latency 7
// (4 for NTT/INTT butterflies); write port registered at t+9 (t+6) and committed at the end of t+10
// (t+7). A dependent read may be issued at t+11 (t+8) or later, so consecutive NTT levels of one polynomial and consecutive commands are
// separated by idle cycles (or, for two interleaved polynomials, by the other polynomial's level);
// done is pulsed only after the last write has been committed.
module nev_palu import nev_pkg::*; import nev_palu_pkg::*; (
  input  logic          clk,
  input  logic          rst_n,
  input  logic [1:0]    qsel,
  input  logic          d64,
  input  logic [1:0]    nlog,
  input  logic          cmd_valid,
  input  palu_cmd_t     cmd,
  input  logic [7:0]    slim,          // SCALE strm: source words written by the codec
  input  logic          sblk,          // SCALE strm: the codec writes the source in the next cycle
  output logic          busy,
  output logic          done,
  output logic          zflag,
  output logic          nzflag,        // PROD cmpc: some output nonzero
  output logic          m_we,          // INTT tmsg: message bits valid (pulse with done)
  output logic [511:0]  m_wdata,
  output logic [7:0]    wm,            // words of the command's result written so far, in natural order
                                       // (PROD, SCALE destination bd, last level of a single/split NTT/INTT)
  // poly buffer ports
  output logic [3:0]          r_v,
  output logic [3:0][2:0]     r_buf,
  output logic [3:0][6:0]     r_addr,
  input  logic [3:0][WW-1:0]  r_data,
  output logic [3:0]          w_v,
  output logic [3:0][2:0]     w_buf,
  output logic [3:0][6:0]     w_addr,
  output logic [3:0][WW-1:0]  w_data
);
  localparam int DR = 9;                       // idle cycles before a dependent read
  localparam int DRN = 6;                      // same for NTT/INTT (fast butterfly path)
  localparam logic [2:0] M_CT = 3'd0, M_GS = 3'd1, M_MAC = 3'd2, M_MULC = 3'd3, M_MUL2 = 3'd5;
  localparam logic [1:0] CS_TW = 2'd0, CS_Y = 2'd1, CS_CONST = 2'd2, CS_YK = 2'd3;

  // control bundle travelling with each issued request
  typedef struct packed {
    logic        v;        // lane op valid
    logic        ld;       // EXP load capture
    logic        zc;       // zero check on port-0/2 data
    logic [2:0]  mode;
    logic        first, last, neg, nb, one;
    logic        bz, clr, ek;      // PROD: V absent, term-set start, E step index
    logic        cn;               // PROD: C step (rows subtracted)
    logic        cmp;              // PROD: compare instead of write
    logic [1:0]  rl;               // PROD: log2 rows of the block
    logic [5:0]  wtop;             // PROD (last step): output row of accumulator 0
    logic        wst2;             // PROD: output row stride 2 (UP)
    logic        wmv;              // PROD: block complete after this group pair (NORM)
    logic [7:0]  wmval;            // PROD: natural-order words complete after this block
    logic [2:0]  pm;       // 0 straight, 1..4 in-word len 8/4/2/1
    logic [1:0]  csel;
    logic        inv;
    logic [2:0]  li;
    logic [1:0]  pi;       // half A pair (half B: pi+1)
    logic [2:0]  g;        // half A group (half B: g+1)
    logic        mask;
    logic        bc0;              // half B takes read port 0 data (SCALE CM_KY)
    logic        ks;
    logic [3:0]  wmi;      // watermark increment when this issue's writes are presented
    logic [3:0]  wv;
    logic [3:0][2:0] wbuf;
    logic [3:0][6:0] waddr;
  } cb_t;

  // ---------------- derived parameters (quasi-static, registered locally)
  logic [1:0] glog;          // log2 words per block (G)
  logic [2:0] klog;          // log2 blocks (k)
  logic [2:0] nlev;          // NTT levels
  logic [1:0] var_;          // ROM variant
  logic [1:0] qsel_l;
  logic [5:0] lvl_issue;     // issue cycles of one polynomial level = k*G/4
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin glog <= 2'd3; klog <= 3'd2; nlev <= 3'd7; var_ <= 2'd0; qsel_l <= 2'd0; lvl_issue <= 6'd8; end
    else begin
      glog <= d64 ? 2'd2 : 2'd3;
      klog <= (d64 ? 3'd3 : 3'd2) + {1'b0, nlog};
      nlev <= d64 ? 3'd6 : 3'd7;
      var_ <= d64 ? 2'd2 : ((qsel == Q3329) ? 2'd1 : 2'd0);
      qsel_l <= qsel;
      // k*G/4 = 2^(klog+glog-2): d64 -> 2^(3+nlog+0), D128 -> 2^(2+nlog+1)
      lvl_issue <= 6'd8 << nlog;
    end
  end
  logic [5:0] kk;            // k
  logic [5:0] blk_end;       // last row of the current pass
  logic [5:0] ph_issue;      // issue cycles of the other half-pass (dual: full level, split: half level)
  logic [3:0] gg_last;       // G-2
  logic [2:0] pp_last;       // G/2-2
  always_comb begin
    kk      = 6'd1 << klog;
    gg_last = (glog == 2'd3) ? 4'd6 : 4'd2;
    pp_last = (glog == 2'd3) ? 3'd2 : 3'd0;
  end

  // ---------------- command latch and FSM
  typedef enum logic [2:0] {S_IDLE, S_RUN, S_GAP, S_DRAIN, S_XSTEP, S_XSTORE} st_t;
  st_t        st;
  palu_cmd_t  c;
  logic [2:0] li;            // level
  logic       ph;            // NTT: polynomial (dual)
  logic [5:0] blk;           // block / row counter
  logic [5:0] rr;            // SCALE row
  logic [2:0] gg;            // group (even)
  logic [1:0] pp;            // pair (even)
  logic [5:0] oo, ii;        // PROD block base / step
  logic       pts, ppar, pephase;   // PROD term set (DOWN), output parity (UP), E-step phase
  logic [1:0] pek;           // PROD E/C step
  logic [3:0] dcnt;
  logic [3:0] xn;            // EXP step
  logic [3:0] xw;            // EXP period counter
  always_comb begin
    blk_end  = (c.split && !ph) ? ((kk >> 1) - 6'd1) : (kk - 6'd1);
    ph_issue = c.split ? (lvl_issue >> 1) : lvl_issue;
  end

  // helpers
  function automatic logic [6:0] waddr_of(input logic [5:0] row, input logic [2:0] g, input logic [1:0] gl);
    waddr_of = (gl == 2'd3) ? {row[3:0], g} : {row[4:0], g[1:0]};
  endfunction
  function automatic logic [2:0] ins0(input logic [2:0] x, input logic [1:0] b);
    // insert a zero bit at position b into a 2-bit value (result 3 bits)
    case (b)
      2'd0: ins0 = {x[1:0], 1'b0};
      2'd1: ins0 = {x[1], 1'b0, x[0]};
      default: ins0 = {1'b0, x[1:0]};
    endcase
  endfunction
  function automatic logic [3:0] ins0_4(input logic [2:0] u, input logic [1:0] lb);
    case (lb)
      2'd0: ins0_4 = {u, 1'b0};
      2'd1: ins0_4 = {u[2:1], 1'b0, u[0]};
      2'd2: ins0_4 = {u[2], 1'b0, u[1:0]};
      default: ins0_4 = {1'b0, u};
    endcase
  endfunction

  // EXP schedule (right-to-left binary): exponent e = q-2, L bits, bit 0 and bit L-1 are 1.
  // step 0     : x <- x^2 ; acc <- x*cval
  // step n<L-1 : x <- x^2 ; if bit n: acc <- acc*x
  // step L-1   :             acc <- acc*x
  logic [3:0]  xns;          // L
  logic [11:0] xbits;
  always_comb begin
    case (qsel_l)
      Q769:    begin xns = 4'd10; xbits = 12'b001011111111; end   // 767
      Q1409:   begin xns = 4'd11; xbits = 12'b010101111111; end   // 1407
      default: begin xns = 4'd12; xbits = 12'b110011111111; end   // 3327
    endcase
  end
  logic x_sqr, x_mul;
  always_comb begin
    x_sqr = (xn != xns - 4'd1);
    x_mul = xbits[xn];
  end

  // issue registers
  logic [3:0]       rq_v, rq_v_n;
  logic [3:0][2:0]  rq_buf, rq_buf_n;
  logic [3:0][6:0]  rq_addr, rq_addr_n;
  cb_t              cb_n;
  cb_t              cbp [0:10];

  assign r_v = rq_v; assign r_buf = rq_buf; assign r_addr = rq_addr;

  // level geometry
  logic [2:0] lenlog;        // log2 len of current level
  logic       xmode;
  always_comb begin
    if (c.op == PA_NTT) lenlog = nlev - 3'd1 - li;   // D/2 >> li
    else                lenlog = li;                  // 1 << li
    xmode = (lenlog >= 3'd4);
  end

  // PROD index arithmetic (row blocks of R = 2^rl output rows; R = min(4, B)).
  // Step ss of term set st (0 .. B+R-2): U index u = base + R-1 - ss - st (wrapped -> Y copy in bb),
  // V term ss (absent for ss >= B); the lane delays V so that accumulator d (output row base+R-1-d)
  // pairs U with V term ss-d. E steps (NORM extra) add E rows to the accumulators.
  logic [5:0] bb_;           // B
  logic [1:0] prl;           // log2 R
  logic [5:0] prr;           // R
  logic [5:0] pst_last;      // last step index B+R-2
  logic       pdown, pup, pnorm;
  logic       ptsl, pe_n;
  logic [1:0] pne;           // E steps
  logic [2:0] pnx;           // E + C steps
  logic       pcs;           // current extra step is a C step
  logic       pk0;           // row pair index of the current extra step
  logic [6:0] uu;            // signed U index
  logic       wrap;
  logic [5:0] uidx;
  logic [5:0] urow, vrow, erow0, erow1;
  logic       vval;
  logic       plast;         // final step of this group pair's block
  logic [5:0] wtop_n;
  always_comb begin
    bb_   = 6'd1 << c.mlog;
    prl   = (c.mlog >= 3'd2) ? 2'd2 : c.mlog[1:0];
    prr   = 6'd1 << prl;
    pst_last = bb_ + prr - 6'd2;
    pdown = (c.pmode == PM_DOWN);
    pup   = (c.pmode == PM_UP);
    pnorm = (c.pmode == PM_NORM);
    pne   = (pnorm && c.extra) ? ((prl == 2'd2) ? 2'd2 : 2'd1) : 2'd0;
    pnx   = {1'b0, pne} + ((pnorm && c.cmpc) ? ((prl == 2'd2) ? 3'd2 : 3'd1) : 3'd0);
    pcs   = ({1'b0, pek} >= {1'b0, pne});
    pk0   = pcs ? (pek[0] ^ pne[0]) : pek[0];
    ptsl = !pdown || pts;
    pe_n  = pnorm && c.extra;
    uu    = {1'b0, oo} + {1'b0, prr} - 7'd1 - {1'b0, ii} - {6'd0, pdown & pts};
    wrap  = uu[6];
    uidx  = wrap ? 6'(uu + {1'b0, bb_}) : uu[5:0];
    vval  = (ii < bb_);
    if (pnorm)      begin urow = {1'b0, c.ra} + uidx;                         vrow = {1'b0, c.rc} + ii; end
    else if (pdown) begin urow = {1'b0, c.ra} + {uidx[4:0], 1'b0} + {5'd0, pts}; vrow = {1'b0, c.rc} + {ii[4:0], 1'b0} + {5'd0, pts}; end
    else            begin urow = {1'b0, c.ra} + {uidx[4:0], 1'b0} + {5'd0, ppar}; vrow = {1'b0, c.rc} + ii; end
    erow0 = (pcs ? {1'b0, c.rd} : {1'b0, c.re}) + oo + prr - 6'd1 - {4'd0, pk0, 1'b0};
    erow1 = erow0 - 6'd1;
    plast = pephase ? ({1'b0, pek} == pnx - 3'd1) : (ptsl && ii == pst_last && pnx == 3'd0);
    wtop_n = pup ? ({1'b0, c.rd} + {oo[4:0], 1'b0} + {prr[4:0], 1'b0} - 6'd2 + {5'd0, ppar})
                 : ({1'b0, c.rd} + oo + prr - 6'd1);
  end

  // SCALE CM_KY issue condition (streaming behind a codec writing the source)
  logic       sc_ok;
  logic [7:0] sc_word;
  always_comb begin
    sc_word = 8'(({1'b0, c.ra} + rr) << glog) + {5'd0, gg};
    sc_ok   = !c.strm || ((sc_word < slim) && !sblk);
  end

  // next issue (combinational) ------------------------------------------------
  logic [2:0] gwa0, gwb0, gwa1, gwb1;
  logic [2:0] nsrc, ndst;
  logic [5:0] srow;
  always_comb begin
    rq_v_n = '0; rq_buf_n = '0; rq_addr_n = '0;
    cb_n = '0; gwa0 = '0; gwb0 = '0; gwa1 = '0; gwb1 = '0; nsrc = '0; ndst = '0; srow = '0;
    if (st == S_RUN) begin
      case (c.op)
        PA_NTT, PA_INTT: begin
          if (xmode) begin
            gwa0 = ins0({1'b0, pp}, 2'(lenlog - 3'd4));
            gwb0 = gwa0 + (3'd1 << (lenlog - 3'd4));
            gwa1 = ins0({1'b0, pp + 2'd1}, 2'(lenlog - 3'd4));
            gwb1 = gwa1 + (3'd1 << (lenlog - 3'd4));
          end else begin
            gwa0 = {pp, 1'b0}; gwb0 = {pp, 1'b1};
            gwa1 = {pp + 2'd1, 1'b0}; gwb1 = {pp + 2'd1, 1'b1};
          end
          if (!ph) begin nsrc = li[0] ? c.bb : c.ba; ndst = li[0] ? c.ba : c.bb; end
          else if (!c.split) begin nsrc = li[0] ? c.bd : c.bc; ndst = li[0] ? c.bc : c.bd; end
          else begin
            // upper half of a split transform
            nsrc = (li == 3'd0) ? c.ba : (li[0] ? c.bc : c.bd);
            ndst = li[0] ? c.bd : c.bc;
            if (li == nlev - 3'd1) ndst = li[0] ? c.ba : c.bb;
          end
          rq_v_n = 4'b1111;
          rq_buf_n = {nsrc, nsrc, nsrc, nsrc};
          rq_addr_n[0] = waddr_of(blk, gwa0, glog); rq_addr_n[1] = waddr_of(blk, gwb0, glog);
          rq_addr_n[2] = waddr_of(blk, gwa1, glog); rq_addr_n[3] = waddr_of(blk, gwb1, glog);
          cb_n.v = 1'b1; cb_n.mode = (c.op == PA_NTT) ? M_CT : M_GS;
          cb_n.pm = xmode ? 3'd0 : (3'd4 - lenlog);
          cb_n.csel = CS_TW; cb_n.inv = (c.op == PA_INTT); cb_n.li = li; cb_n.pi = pp;
          cb_n.wv = 4'b1111;
          cb_n.wmi = (li == nlev - 3'd1 && !c.dual && pp == pp_last[1:0]) ? ((glog == 2'd3) ? 4'd8 : 4'd4) : 4'd0;
          cb_n.wbuf = {ndst, ndst, ndst, ndst};
          cb_n.waddr = rq_addr_n;
        end
        PA_SCALE: if (c.cmode == CM_KY) begin
          srow = {1'b0, c.ra} + rr;
          if (sc_ok) begin
            rq_v_n[0] = 1'b1; rq_buf_n[0] = c.ba; rq_addr_n[0] = waddr_of(srow, gg, glog);
            cb_n.v = 1'b1; cb_n.mode = M_MULC; cb_n.g = gg; cb_n.bc0 = 1'b1; cb_n.ks = c.ks;
            cb_n.csel = CS_YK;
            cb_n.wv = 4'b0101;
            cb_n.wmi = 4'd1;
            cb_n.wbuf = {3'd0, c.bd, 3'd0, c.be};
            cb_n.waddr[0] = waddr_of({1'b0, c.re} + rr, gg, glog);
            cb_n.waddr[2] = waddr_of({1'b0, c.rd} + rr, gg, glog);
          end
        end else begin
          srow = {1'b0, c.ra} + rr;
          rq_v_n[0] = 1'b1; rq_buf_n[0] = c.ba; rq_addr_n[0] = waddr_of(srow, gg, glog);
          rq_v_n[2] = 1'b1; rq_buf_n[2] = c.ba; rq_addr_n[2] = waddr_of(srow, gg + 3'd1, glog);
          cb_n.v = 1'b1; cb_n.mode = M_MULC; cb_n.g = gg; cb_n.mask = c.mask;
          cb_n.csel = (c.cmode == CM_Y) ? CS_Y : CS_CONST;
          cb_n.wv = {c.copy, 1'b1, c.copy, 1'b1};
          cb_n.wmi = 4'd2;
          cb_n.wbuf = {c.be, c.bd, c.be, c.bd};
          cb_n.waddr[0] = waddr_of({1'b0, c.rd} + rr, gg, glog);
          cb_n.waddr[1] = waddr_of({1'b0, c.re} + rr, gg, glog);
          cb_n.waddr[2] = waddr_of({1'b0, c.rd} + rr, gg + 3'd1, glog);
          cb_n.waddr[3] = waddr_of({1'b0, c.re} + rr, gg + 3'd1, glog);
        end
        PA_PROD: begin
          if (pephase) begin
            rq_v_n[0] = 1'b1; rq_buf_n[0] = pcs ? c.bd : c.be; rq_addr_n[0] = waddr_of(erow0, gg, glog);
            rq_v_n[2] = 1'b1; rq_buf_n[2] = pcs ? c.bd : c.be; rq_addr_n[2] = waddr_of(erow0, gg + 3'd1, glog);
            if ({pk0, 1'b1} < prr[1:0] || prr[2]) begin
              rq_v_n[1] = 1'b1; rq_buf_n[1] = pcs ? c.bd : c.be; rq_addr_n[1] = waddr_of(erow1, gg, glog);
              rq_v_n[3] = 1'b1; rq_buf_n[3] = pcs ? c.bd : c.be; rq_addr_n[3] = waddr_of(erow1, gg + 3'd1, glog);
            end else cb_n.bz = 1'b1;
            cb_n.one = 1'b1; cb_n.ek = pk0; cb_n.cn = pcs;
          end else begin
            rq_v_n[0] = 1'b1; rq_buf_n[0] = wrap ? c.bb : c.ba; rq_addr_n[0] = waddr_of(urow, gg, glog);
            rq_v_n[2] = 1'b1; rq_buf_n[2] = wrap ? c.bb : c.ba; rq_addr_n[2] = waddr_of(urow, gg + 3'd1, glog);
            if (vval) begin
              rq_v_n[1] = 1'b1; rq_buf_n[1] = c.bc; rq_addr_n[1] = waddr_of(vrow, gg, glog);
              rq_v_n[3] = 1'b1; rq_buf_n[3] = c.bc; rq_addr_n[3] = waddr_of(vrow, gg + 3'd1, glog);
            end else cb_n.bz = 1'b1;
            cb_n.clr = (ii == 6'd0);
            cb_n.nb  = pdown && pts;
          end
          cb_n.v = 1'b1; cb_n.mode = M_MAC; cb_n.g = gg;
          cb_n.neg = pnorm ? c.neg : (pup & ppar);
          cb_n.first = !pephase && !pts && (ii == 6'd0);
          cb_n.last  = plast;
          cb_n.rl    = prl;
          cb_n.wtop  = wtop_n;
          cb_n.wst2  = pup;
          cb_n.wmv   = pnorm && ({1'b0, gg} == gg_last);
          cb_n.cmp   = c.cmpc;
          cb_n.wmval = 8'((oo + prr) << glog);
          cb_n.wv    = '0;
          cb_n.wmi   = 4'd0;
          cb_n.wbuf  = {3'd0, c.bd, 3'd0, c.bd};
        end
        PA_EXP, PA_ZCHK: begin
          rq_v_n[0] = 1'b1; rq_buf_n[0] = c.ba; rq_addr_n[0] = waddr_of({1'b0, c.ra}, gg, glog);
          rq_v_n[2] = 1'b1; rq_buf_n[2] = c.ba; rq_addr_n[2] = waddr_of({1'b0, c.ra}, gg + 3'd1, glog);
          cb_n.zc = 1'b1; cb_n.ld = (c.op == PA_EXP); cb_n.g = gg;
        end
        default: ;
      endcase
    end
  end

  // FSM / counters
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE; c <= '0; li <= '0; ph <= 1'b0; blk <= '0; rr <= '0; gg <= '0; pp <= '0; oo <= '0; ii <= '0;
      pts <= 1'b0; ppar <= 1'b0; pek <= '0; pephase <= 1'b0;
      dcnt <= '0; xn <= '0; xw <= '0; done <= 1'b0;
    end else begin
      done <= 1'b0;
      case (st)
        S_IDLE: if (cmd_valid) begin
          c <= cmd; li <= '0; ph <= 1'b0; blk <= '0; rr <= '0; gg <= '0; pp <= '0; oo <= '0; ii <= '0;
          pts <= 1'b0; ppar <= 1'b0; pek <= '0; pephase <= 1'b0;
          st <= S_RUN;
        end
        S_RUN: begin
          case (c.op)
            PA_NTT, PA_INTT: begin
              if (pp == pp_last[1:0]) begin
                pp <= '0;
                if (blk == blk_end) begin
                  blk <= (c.split && !ph) ? (kk >> 1) : 6'd0;
                  if ((c.dual || c.split) && !ph) ph <= 1'b1;
                  else if (li == nlev - 3'd1) begin dcnt <= 4'(DRN); st <= S_DRAIN; end
                  else if ((c.dual || c.split) && ph_issue >= 6'(DRN + 1)) begin ph <= 1'b0; blk <= 6'd0; li <= li + 3'd1; end
                  else begin
                    dcnt <= (c.dual || c.split) ? 4'(6'(DRN) - ph_issue) : 4'(DRN);
                    blk <= 6'd0;
                    st <= S_GAP;
                  end
                end else blk <= blk + 6'd1;
              end else pp <= pp + 2'd2;
            end
            PA_SCALE: begin
              if (c.cmode == CM_KY) begin
                if (sc_ok) begin
                  if ({1'b0, gg} == gg_last + 4'd1) begin
                    gg <= '0;
                    if (rr == c.nrows - 6'd1) begin dcnt <= 4'(DR); st <= S_DRAIN; end
                    else rr <= rr + 6'd1;
                  end else gg <= gg + 3'd1;
                end
              end else if ({1'b0, gg} == gg_last) begin
                gg <= '0;
                if (rr == c.nrows - 6'd1) begin dcnt <= 4'(DR); st <= S_DRAIN; end
                else rr <= rr + 6'd1;
              end else gg <= gg + 3'd2;
            end
            PA_PROD: begin
              // block (parity, base) outer, group pair, term set, step, E steps innermost
              if (plast) begin
                ii <= '0; pts <= 1'b0; pek <= '0; pephase <= 1'b0;
                if ({1'b0, gg} == gg_last) begin
                  gg <= '0;
                  if (oo + prr == bb_) begin
                    oo <= '0;
                    if (pup && !ppar) ppar <= 1'b1;
                    else begin dcnt <= 4'(DR) + 4'(prr) - 4'd1; st <= S_DRAIN; end
                  end else oo <= oo + prr;
                end else gg <= gg + 3'd2;
              end else if (pephase) pek <= pek + 2'd1;
              else if (ii == pst_last) begin
                ii <= '0;
                if (!ptsl) pts <= 1'b1;
                else pephase <= 1'b1;
              end else ii <= ii + 6'd1;
            end
            PA_ZCHK: begin
              if ({1'b0, gg} == gg_last) begin gg <= '0; dcnt <= 4'd4; st <= S_DRAIN; end
              else gg <= gg + 3'd2;
            end
            PA_EXP: begin
              if ({1'b0, gg} == gg_last) begin gg <= '0; dcnt <= 4'd4; st <= S_GAP; end
              else gg <= gg + 3'd2;
            end
            default: st <= S_DRAIN;
          endcase
        end
        S_GAP: begin
          if (dcnt == 4'd0) begin
            if (c.op == PA_EXP) begin st <= S_XSTEP; xn <= '0; xw <= '0; gg <= '0; end
            else begin li <= li + 3'd1; ph <= 1'b0; st <= S_RUN; end
          end else dcnt <= dcnt - 4'd1;
        end
        S_XSTEP: begin
          // issue word xw of step xn when xw < G; period 8 (lane latency 7 + register file write)
          if (xw == 4'd7) begin
            xw <= '0;
            if (xn == xns - 4'd1) begin st <= S_XSTORE; gg <= '0; end
            else xn <= xn + 4'd1;
          end else xw <= xw + 4'd1;
        end
        S_XSTORE: begin
          // wait for the last step results before storing: xw counts
          if (xw < 4'd8) xw <= xw + 4'd1;
          else if ({1'b0, gg} == gg_last) begin gg <= '0; dcnt <= 4'd2; st <= S_DRAIN; end
          else gg <= gg + 3'd2;
        end
        S_DRAIN: begin
          if (dcnt == 4'd0) begin st <= S_IDLE; done <= 1'b1; end
          else dcnt <= dcnt - 4'd1;
        end
        default: st <= S_IDLE;
      endcase
    end
  end
  assign busy = (st != S_IDLE);
`ifndef SYNTHESIS
  always_ff @(posedge clk) if (rst_n && cmd_valid && st != S_IDLE)
    $error("nev_palu: command issued while busy (dropped) at %t", $time);
`endif

  // issue registers and control pipeline
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rq_v <= '0; rq_buf <= '0; rq_addr <= '0;
      for (int k = 0; k < 11; k++) cbp[k] <= '0;
    end else begin
      rq_v <= rq_v_n; rq_buf <= rq_buf_n; rq_addr <= rq_addr_n;
      cbp[0] <= cb_n;
      for (int k = 1; k < 11; k++) cbp[k] <= cbp[k-1];
    end
  end

  // ---------------- lane inputs (cycle t+2 for read-path ops, or EXP direct)
  logic [1:0][191:0] tw_row, y_row;
  nev_rom_tw u_tw0 (.v(var_), .inv(cbp[2].inv), .li(cbp[2].li), .pi(cbp[2].pi),        .tw(tw_row[0]));
  nev_rom_tw u_tw1 (.v(var_), .inv(cbp[2].inv), .li(cbp[2].li), .pi(cbp[2].pi + 2'd1), .tw(tw_row[1]));
  logic [1:0][255:0] twq_row;
  nev_rom_twq u_twq0 (.v(var_), .inv(cbp[2].inv), .li(cbp[2].li), .pi(cbp[2].pi),        .twq(twq_row[0]));
  nev_rom_twq u_twq1 (.v(var_), .inv(cbp[2].inv), .li(cbp[2].li), .pi(cbp[2].pi + 2'd1), .twq(twq_row[1]));
  nev_rom_y  u_y0  (.v(var_), .g(cbp[2].g),        .y(y_row[0]));
  nev_rom_y  u_y1  (.v(var_), .g(cbp[2].g + 3'd1), .y(y_row[1]));
  logic [191:0] yk_row;
  nev_rom_yk u_yk  (.v(var_), .ks(cbp[2].ks), .g(cbp[2].g), .y(yk_row));

  // EXP register files
  logic [7:0][WW-1:0] xrf, arf;
  logic               xin_v, xin_va, xin_vb;
  logic [2:0]         xin_g;
  logic [1:0][WW-1:0] xin_a, xin_b;

  logic [31:0]       ln_v;
  logic [31:0][11:0] ln_a, ln_b, ln_c, ln_y0, ln_y1;
  logic [31:0][15:0] ln_cq;
  logic [31:0]       ln_ov;
  logic [1:0]        plb;     // log2 len for in-word permutation
  always_comb begin
    plb = 2'd3 - 2'(cbp[2].pm - 3'd1);   // pm 1..4 -> len 8,4,2,1 -> lb 3,2,1,0
    for (int h = 0; h < 2; h++) begin
      for (int l = 0; l < 16; l++) begin
        logic [3:0] pa, pb;
        logic [WW-1:0] da, db;
        int L;
        L = 16 * h + l;
        da = (cbp[2].bc0 && h == 1) ? r_data[0] : r_data[2*h]; db = r_data[2*h+1];
        ln_v[L] = 1'b0; ln_a[L] = '0; ln_b[L] = '0; ln_c[L] = '0;
        ln_cq[L] = twq_row[h][16*l +: 16];
        if (xin_v) begin
          ln_v[L] = h ? xin_vb : xin_va;
          ln_a[L] = xin_a[h][12*l +: 12]; ln_b[L] = xin_b[h][12*l +: 12];
        end else begin
          ln_v[L] = cbp[2].v;
          if (cbp[2].pm == 3'd0) begin
            ln_a[L] = da[12*l +: 12]; ln_b[L] = db[12*l +: 12];
          end else begin
            pa = ins0_4(3'(l & 7), plb);
            pb = pa + (4'd1 << plb);
            if (l < 8) begin ln_a[L] = da[12*pa +: 12]; ln_b[L] = da[12*pb +: 12]; end
            else       begin ln_a[L] = db[12*pa +: 12]; ln_b[L] = db[12*pb +: 12]; end
          end
          if (cbp[2].mask && h == 0 && l == 0 && cbp[2].g == 3'd0) ln_a[L] = '0;
          case (cbp[2].csel)
            CS_TW:    ln_c[L] = tw_row[h][12*l +: 12];
            CS_Y:     ln_c[L] = y_row[h][12*l +: 12];
            CS_YK:    ln_c[L] = h ? yk_row[12*l +: 12] : c.cval;
            default:  ln_c[L] = c.cval;
          endcase
        end
      end
    end
  end

  logic [2:0] ln_mode;
  logic       ln_first, ln_last, ln_neg, ln_one, ln_nb, ln_bz, ln_clr, ln_ek, ln_cn;
  logic [1:0] ln_rl;
  always_comb begin
    ln_bz = cbp[2].bz; ln_clr = cbp[2].clr; ln_ek = cbp[2].ek; ln_rl = cbp[2].rl; ln_cn = cbp[2].cn;
    if (xin_v) begin ln_mode = M_MUL2; ln_first = 1'b0; ln_last = 1'b0; ln_neg = 1'b0; ln_one = 1'b0; ln_nb = 1'b0; end
    else begin
      ln_mode = cbp[2].mode; ln_first = cbp[2].first; ln_last = cbp[2].last; ln_neg = cbp[2].neg;
      ln_one = cbp[2].one; ln_nb = cbp[2].nb;
    end
  end

  for (genvar l = 0; l < 32; l++) begin : g_lane
    nev_lane u_lane (
      .clk(clk), .rst_n(rst_n), .qsel(qsel_l),
      .in_v(ln_v[l]), .in_mode(ln_mode), .in_first(ln_first), .in_last(ln_last), .in_neg(ln_neg), .in_one(ln_one),
      .in_nb(ln_nb), .in_cn(ln_cn), .in_bz(ln_bz), .in_clr(ln_clr), .in_ek(ln_ek), .in_rl(ln_rl), .in_a(ln_a[l]), .in_b(ln_b[l]), .in_c(ln_c[l]), .in_cq(ln_cq[l]),
      .out_v(ln_ov[l]), .out_y0(ln_y0[l]), .out_y1(ln_y1[l]));
  end

  // ---------------- EXP direct issue, register files, zero flag
  always_comb begin
    xin_v = 1'b0; xin_va = 1'b0; xin_vb = 1'b0; xin_g = 3'(xw); xin_a = '0; xin_b = '0;
    if (st == S_XSTEP && xw < (4'd1 << glog)) begin
      xin_v  = 1'b1;
      xin_va = x_sqr;
      xin_vb = x_mul;
      xin_a[0] = xrf[xin_g]; xin_b[0] = xrf[xin_g];
      if (xn == 4'd0) begin
        xin_a[1] = xrf[xin_g];
        for (int l = 0; l < 16; l++) xin_b[1][12*l +: 12] = c.cval;
      end else begin
        xin_a[1] = arf[xin_g]; xin_b[1] = xrf[xin_g];
      end
    end
  end

  logic [31:0] zl;
  always_comb for (int l = 0; l < 16; l++) begin
    zl[l]      = (r_data[0][12*l +: 12] == 12'd0);
    zl[16 + l] = (r_data[2][12*l +: 12] == 12'd0);
  end

  logic [4:0] xg_d [0:6];      // EXP direct issue: {valid A, valid B, g} delay line
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      zflag <= 1'b0;
      for (int k = 0; k < 7; k++) xg_d[k] <= '0;
    end else begin
      if (st == S_IDLE && cmd_valid) zflag <= 1'b0;
      else if (cbp[2].zc && (|zl)) zflag <= 1'b1;
      xg_d[0] <= {xin_v & xin_va, xin_v & xin_vb, xin_g};
      for (int k = 1; k < 7; k++) xg_d[k] <= xg_d[k-1];
    end
  end
  always_ff @(posedge clk) begin
    if (cbp[2].ld) begin
      xrf[cbp[2].g] <= r_data[0]; xrf[cbp[2].g + 3'd1] <= r_data[2];
    end else begin
      if (xg_d[6][4]) for (int l = 0; l < 16; l++) xrf[xg_d[6][2:0]][12*l +: 12] <= ln_y0[l];
      if (xg_d[6][3]) for (int l = 0; l < 16; l++) arf[xg_d[6][2:0]][12*l +: 12] <= ln_y0[16 + l];
    end
  end

  // ---------------- write-back, registered write ports (control bundle of the issue whose lane
  // outputs are present: cbp[9], NTT/INTT cbp[6])
  cb_t cbw;
  always_comb cbw = (c.op == PA_NTT || c.op == PA_INTT) ? cbp[6] : cbp[9];
  logic [3:0][WW-1:0] wd_n;
  logic [1:0]         olb;
  always_comb begin
    olb = 2'd3 - 2'(cbw.pm - 3'd1);
    wd_n = '0;
    for (int h = 0; h < 2; h++) begin
      if (cbw.pm == 3'd0) begin
        for (int l = 0; l < 16; l++) begin
          wd_n[2*h][12*l +: 12] = ln_y0[16*h + l]; wd_n[2*h+1][12*l +: 12] = ln_y1[16*h + l];
        end
      end else begin
        for (int u = 0; u < 8; u++) begin
          logic [3:0] pa, pb;
          pa = ins0_4(3'(u), olb); pb = pa + (4'd1 << olb);
          wd_n[2*h][12*pa +: 12]   = ln_y0[16*h + u];     wd_n[2*h][12*pb +: 12]   = ln_y1[16*h + u];
          wd_n[2*h+1][12*pa +: 12] = ln_y0[16*h + u + 8]; wd_n[2*h+1][12*pb +: 12] = ln_y1[16*h + u + 8];
        end
      end
    end
  end
  // INTT tmsg: message decoding at the last level (reference tomsg: bit = sum of flipabs over the four
  // coefficients 16*(2a+b)+l, a = 0..3, of one row is < q; 128-dim: slot {row, b}, 64-dim: slot row)
  function automatic logic [11:0] flipabs(input logic [11:0] x, input logic [10:0] qh);
    logic [12:0] r;
    r = {1'b0, x} - {2'b00, qh};
    return r[12] ? 12'(-r) : r[11:0];          // |r| (reference (r + m) ^ m with m = r >> 15)
  endfunction
  logic [511:0]       msg_r;
  logic [15:0][13:0]  tac0, tac1;
  logic [11:0]        qv;
  logic [10:0]        qhv;
  always_comb begin
    case (qsel_l)
      Q769:    qv = 12'd769;
      Q1409:   qv = 12'd1409;
      default: qv = 12'd3329;
    endcase
    qhv = qv[11:1];
  end
  logic               tm_now;
  always_comb tm_now = cbw.v && cbw.mode == M_GS && c.tmsg && (cbw.li == nlev - 3'd1);
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      msg_r <= '0; tac0 <= '0; tac1 <= '0; m_we <= 1'b0;
    end else begin
      m_we <= 1'b0;
      if (st == S_IDLE && cmd_valid) msg_r <= '0;
      if (st == S_DRAIN && dcnt == 4'd0 && c.op == PA_INTT && c.tmsg) m_we <= 1'b1;
      if (tm_now) begin
        for (int l = 0; l < 16; l++) begin
          logic [13:0] p0, p1, s4;
          // last INTT level (len D/2): ports 0/1 carry words g, g+4 (b = g[0] = 0 for pair pp even),
          // ports 2/3 words g+1, g+5 (b = 1); 64-dim: the four words of the row
          p0 = 14'(flipabs(wd_n[0][12*l +: 12], qhv)) + 14'(flipabs(wd_n[1][12*l +: 12], qhv));
          p1 = 14'(flipabs(wd_n[2][12*l +: 12], qhv)) + 14'(flipabs(wd_n[3][12*l +: 12], qhv));
          if (glog == 2'd2) begin
            s4 = p0 + p1;
            msg_r[16*cbw.waddr[0][6:2] + l] <= (s4 < {2'b00, qv});
          end else if (cbw.pi == 2'd0) begin
            tac0[l] <= p0; tac1[l] <= p1;
          end else begin
            msg_r[16*{cbw.waddr[0][6:3], 1'b0} + l] <= ((tac0[l] + p0) < {2'b00, qv});
            msg_r[16*{cbw.waddr[0][6:3], 1'b1} + l] <= ((tac1[l] + p1) < {2'b00, qv});
          end
        end
      end
    end
  end
  assign m_wdata = msg_r;

  // PROD write-back: accumulator 0 of a block arrives with the block's last step (cbp[9]), rows
  // 1 .. R-1 in the following cycles (row wtop - d*stride)
  logic       pw_now, pw_on;
  logic [1:0] pw_d, pw_last;
  logic [5:0] pw_row, pw_top;
  logic       pw_st2, pw_wmv, pw_fin, pw_cmp;
  logic [7:0] pw_wmval;
  logic [2:0] pw_g;
  logic [2:0] pw_buf;
  always_comb begin
    pw_now = 1'b0; pw_row = '0; pw_fin = 1'b0;
    if (cbp[9].v && cbp[9].mode == M_MAC && cbp[9].last) begin
      pw_now = 1'b1; pw_row = cbp[9].wtop; pw_fin = (cbp[9].rl == 2'd0);
    end else if (pw_on) begin
      pw_now = 1'b1; pw_row = pw_top - (pw_st2 ? {3'd0, pw_d, 1'b0} : {4'd0, pw_d}); pw_fin = (pw_d == pw_last);
    end
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pw_on <= 1'b0; pw_d <= '0; pw_last <= '0; pw_top <= '0; pw_st2 <= 1'b0; pw_wmv <= 1'b0; pw_wmval <= '0; pw_g <= '0; pw_buf <= '0;
      pw_cmp <= 1'b0;
    end else if (cbp[9].v && cbp[9].mode == M_MAC && cbp[9].last) begin
      pw_on <= (cbp[9].rl != 2'd0); pw_d <= 2'd1; pw_last <= 2'((3'd1 << cbp[9].rl) - 3'd1);
      pw_top <= cbp[9].wtop; pw_st2 <= cbp[9].wst2; pw_wmv <= cbp[9].wmv; pw_wmval <= cbp[9].wmval;
      pw_g <= cbp[9].g; pw_buf <= cbp[9].wbuf[0]; pw_cmp <= cbp[9].cmp;
    end else if (pw_on) begin
      pw_d <= pw_d + 2'd1;
      if (pw_d == pw_last) pw_on <= 1'b0;
    end
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) nzflag <= 1'b0;
    else if (st == S_IDLE && cmd_valid) nzflag <= 1'b0;
    else if (pw_now && (pw_on ? pw_cmp : cbp[9].cmp) && (|wd_n[0] || |wd_n[2])) nzflag <= 1'b1;
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) wm <= '0;
    else if (st == S_IDLE && cmd_valid) wm <= '0;
    else if (pw_now && pw_fin && (pw_on ? pw_wmv : cbp[9].wmv)) wm <= pw_on ? pw_wmval : cbp[9].wmval;
    else if (cbw.wmi != 4'd0 && cbw.mode != M_MAC) wm <= wm + {4'd0, cbw.wmi};
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      w_v <= '0; w_buf <= '0; w_addr <= '0; w_data <= '0;
    end else begin
      if (st == S_XSTORE && xw == 4'd8) begin
        w_v    <= 4'b0101;
        w_buf  <= {3'd0, c.bd, 3'd0, c.bd};
        w_addr <= {7'd0, waddr_of({1'b0, c.rd}, gg + 3'd1, glog), 7'd0, waddr_of({1'b0, c.rd}, gg, glog)};
        w_data <= {{WW{1'b0}}, arf[gg + 3'd1], {WW{1'b0}}, arf[gg]};
      end else if (pw_now) begin
        w_v    <= (pw_on ? pw_cmp : cbp[9].cmp) ? 4'b0000 : 4'b0101;
        w_buf  <= pw_on ? {3'd0, pw_buf, 3'd0, pw_buf} : cbp[9].wbuf;
        w_addr <= {7'd0, waddr_of(pw_row, (pw_on ? pw_g : cbp[9].g) + 3'd1, glog), 7'd0, waddr_of(pw_row, pw_on ? pw_g : cbp[9].g, glog)};
        w_data <= wd_n;
      end else begin
        w_v[0] <= cbw.wv[0] && ln_ov[0];
        w_v[1] <= cbw.wv[1] && ln_ov[0];
        w_v[2] <= cbw.wv[2] && ln_ov[16];
        w_v[3] <= cbw.wv[3] && ln_ov[16];
        w_buf  <= cbw.wbuf;
        w_addr <= cbw.waddr;
        w_data <= wd_n;
      end
    end
  end
endmodule
