// plc_hash -- SM3 hash cluster (4 x plc_sm3_core) executing pseudoXOF jobs.
//
// A job computes, for ct = 1..count, SM3(msg || be32(ct)) (auxfunc.c pseudoXOF, whole bytes),
// msg = seg0 || seg1 || sfx (L bytes). Precondition (asserted): (L mod 64) + 4 <= 55, so each
// output = floor(L/64) shared prefix blocks + exactly one final block. Prefix blocks run
// serially on core 0 (chaining value bypassed core->core so blocks issue back to back); final
// blocks share one template (tail bytes, 0x80, bit length) and differ only in the ct byte
// (ct <= 64 -> be32(ct) = 00 00 00 ct), they are issued round-robin to cores 0..3 and the
// digests are delivered in ct order through a 1-entry output register (xo_*).
//
// Message assembly: a fetch unit walks the segments (register: rg, object: hs_o* port with
// grant stalls and 2-cycle latency, sfx) one 8-byte source word per cycle, a 32-byte byte FIFO
// realigns arbitrary offsets/lengths, and a packer pops 8 bytes/cycle into the 64-byte block
// register bb (lane = 8 message bytes, so no shifting on the block side). Assembly of the next
// block overlaps the compression of the current one.
//
// Midstate cache: 2 entries (hj_cache=1: RG_SD 64 B -> entry 0, hj_cache=2: RG_PS 64 B ->
// entry 1) holding the chaining value after the first 64-byte block. A hit skips seg0.
// cache_clr clears both, cache_inv[e] clears entry e (and suppresses a pending write to it).
//
// Clients: 0 TSAMP, 1 GENA, 2 CTRL; priority 0 > 2 > 1; one job at a time; a job ends when its
// last digest is accepted. Output hs_busy: job in progress.
module plc_hash
  import plc_pkg::*;
(
  input  logic              clk,
  input  logic              rst_n,
  // job request ports (index = client)
  input  logic [2:0]        hj_req,
  output logic [2:0]        hj_gnt,
  input  logic [2:0][1:0]   hj_kind0,
  input  logic [2:0][2:0]   hj_reg0,
  input  logic [2:0][1:0]   hj_buf0,
  input  logic [2:0][12:0]  hj_off0,
  input  logic [2:0][12:0]  hj_len0,
  input  logic [2:0][1:0]   hj_kind1,
  input  logic [2:0][2:0]   hj_reg1,
  input  logic [2:0][1:0]   hj_buf1,
  input  logic [2:0][12:0]  hj_off1,
  input  logic [2:0][12:0]  hj_len1,
  input  logic [2:0][55:0]  hj_sfx,
  input  logic [2:0][2:0]   hj_sfxlen,
  input  logic [2:0][6:0]   hj_count,
  input  logic [2:0][1:0]   hj_cache,
  input  logic              cache_clr,
  input  logic [1:0]        cache_inv,
  // digest stream
  output logic [2:0]        xo_valid,
  input  logic [2:0]        xo_ready,
  output logic [255:0]      xo_data,
  output logic              xo_last,
  // register file (byte i of register r at rg[r][8i+:8])
  input  logic [6:0][511:0] rg,
  // hash object read port
  output logic              hs_oreq,
  output logic [1:0]        hs_obuf,
  output logic [9:0]        hs_oaddr,
  input  logic              hs_ognt,
  input  logic [63:0]       hs_ordata,
  output logic              hs_busy
);
  localparam logic [255:0] SM3_IV =
    256'h7380166F_4914B2B9_172442D7_DA8A0600_A96F30BC_163138AA_E38DEE4D_B0FB0E4E;

  typedef enum logic [1:0] {S_IDLE, S_SETUP, S_RUN} st_t;
  st_t st;

  // ---------------- job descriptor (latched at grant) ----------------
  logic [1:0]  jcl;
  logic [1:0]  jk0, jk1, jb0, jb1, jcache;
  logic [2:0]  jr0, jr1, jsfl;
  logic [12:0] jo0, jo1, jl0, jl1;
  logic [55:0] jsfx;
  logic [6:0]  jcnt;
  logic [13:0] jL;
  logic        jhit;

  // ---------------- arbitration ----------------
  logic [1:0] wsel;
  always_comb begin
    if (hj_req[0])      wsel = 2'd0;
    else if (hj_req[2]) wsel = 2'd2;
    else                wsel = 2'd1;
    hj_gnt = '0;
    if (st == S_IDLE && |hj_req) hj_gnt[wsel] = 1'b1;
  end
  logic grant;
  assign grant = (st == S_IDLE) && |hj_req;

  // ---------------- cache ----------------
  logic [1:0]       cval;
  logic [1:0][255:0] cmid;
  logic             poison;
  logic             ce;        // entry index of the current job (jcache-1)
  logic             jcv;       // job uses the cache (jcache 1 or 2)
  assign ce  = (jcache == 2'd2);
  assign jcv = (jcache == 2'd1) || (jcache == 2'd2);

  // setup-cycle derived values
  logic [13:0] su_L;
  logic        su_hit;
  always_comb begin
    su_L   = ((jk0 != 2'd0) ? {1'b0, jl0} : 14'd0) + ((jk1 != 2'd0) ? {1'b0, jl1} : 14'd0)
           + {11'd0, jsfl};
    su_hit = jcv && cval[ce] && !cache_clr && !cache_inv[ce];
  end

  // ---------------- SM3 cores ----------------
  logic [3:0]        c_start, c_busy, c_done;
  logic [3:0][255:0] c_cv;
  logic [255:0]      cvr, cv_bus;
  logic [511:0]      bb;        // block register, byte i at bb[8i+:8]
  logic [63:0]       ctm;       // byte mask of the ct byte in the final template
  logic [6:0]        ict;       // next ct to issue
  logic [1:0]        icore;
  logic [3:0]        ioh;       // one-hot of icore
  logic              ict_ok;    // ict <= jcnt
  logic [511:0]      blk_in;
  logic              pre0;      // core 0 runs a prefix block
  logic [3:0]        fr;        // core runs a final block
  logic [3:0]        rv;        // core holds an undelivered digest

  always_comb begin
    for (int i = 0; i < 64; i++)
      blk_in[511 - 8*i -: 8] = bb[8*i +: 8] | (ctm[i] ? {1'b0, ict} : 8'h00);
  end
  assign cv_bus = (c_done[0] && pre0) ? c_cv[0] : cvr;

  for (genvar k = 0; k < 4; k++) begin : g_core
    plc_sm3_core u_core (
      .clk    (clk),
      .rst_n  (rst_n),
      .start  (c_start[k]),
      .cv_in  (cv_bus),
      .blk_in (blk_in),
      .busy   (c_busy[k]),
      .done   (c_done[k]),
      .cv_out (c_cv[k])
    );
  end

  // ---------------- fetch unit ----------------
  logic [1:0]  fseg;          // 0 seg0, 1 seg1, 2 sfx, 3 done
  logic [12:0] fa;            // byte address within the current source
  logic [13:0] frem;          // bytes left in the current segment
  logic        s_obj, s_sfx;
  logic [2:0]  s_reg;
  logic [1:0]  s_buf;
  always_comb begin
    s_obj = 1'b0; s_sfx = 1'b0; s_reg = jr0; s_buf = jb0;
    case (fseg)
      2'd0:    begin s_obj = (jk0 == 2'd2); s_reg = jr0; s_buf = jb0; end
      2'd1:    begin s_obj = (jk1 == 2'd2); s_reg = jr1; s_buf = jb1; end
      default: begin s_sfx = 1'b1; end
    endcase
  end

  logic [3:0]  f_nmax, fn;
  logic [5:0]  fcnt;
  logic        p1v, p2v, p1obj, p2obj;
  logic [2:0]  p1sh, p2sh;
  logic [3:0]  p1n_r, p2n_r;
  logic [63:0] p1d, p2d;
  logic [6:0]  pend;          // bytes issued but not yet popped (FIFO + in flight)
  logic [2:0]  p2rot;         // (p2sh - fcnt) mod 8 at insertion time
  logic        f_can, f_iss, f_last, f_adv;
  assign f_nmax = 4'd8 - {1'b0, fa[2:0]};
  assign fn     = (frem < {10'd0, f_nmax}) ? frem[3:0] : f_nmax;
  assign f_can  = (st == S_RUN) && (fseg != 2'd3) && (frem != 14'd0) && (pend <= 7'd24);
  assign hs_oreq  = f_can && s_obj;
  assign hs_obuf  = s_buf;
  assign hs_oaddr = fa[12:3];
  assign f_iss  = f_can && (!s_obj || hs_ognt);
  assign f_last = (frem <= {10'd0, f_nmax});
  assign f_adv  = (st == S_RUN) && (fseg != 2'd3) && ((frem == 14'd0) || (f_iss && f_last));

  logic [63:0] f_rword;
  always_comb begin
    logic [511:0] r;
    case (s_reg)
      3'd0: r = rg[0];
      3'd1: r = rg[1];
      3'd2: r = rg[2];
      3'd3: r = rg[3];
      3'd4: r = rg[4];
      3'd5: r = rg[5];
      3'd6: r = rg[6];
      default: r = '0;
    endcase
    f_rword = r[64*fa[5:3] +: 64];
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      p1v <= 1'b0; p2v <= 1'b0;
    end else begin
      p1v <= f_iss; p2v <= p1v;
    end
  end
  always_ff @(posedge clk) begin
    if (f_iss) begin
      p1obj <= s_obj; p1sh <= fa[2:0]; p1n_r <= fn;
      p1d   <= s_sfx ? {8'd0, jsfx} : f_rword;
    end
    p2obj <= p1obj; p2sh <= p1sh; p2n_r <= p1n_r; p2d <= p1d;
    // fcnt mod 8 only changes by appends (pops are 8 bytes, or the final clear with nothing in flight)
    p2rot <= p1sh - (fcnt[2:0] + (p2v ? p2n_r[2:0] : 3'd0));
  end

  // ---------------- byte FIFO + packer ----------------
  logic [255:0] fq;
  logic [13:0]  prem;          // message bytes still to be popped into blocks
  logic [2:0]   lp;            // next lane of bb
  logic         bfull;         // bb holds a complete prefix block
  logic [7:0]   pleft;         // prefix blocks still to be assembled
  logic [7:0]   pcl;           // prefix blocks still to be completed
  logic         first_pre;
  logic         tdone;         // final template is in bb
  logic [3:0]   take;
  logic         pop;
  logic [63:0]  d2;
  logic [255:0] fq_n;
  assign take = (prem >= 14'd8) ? 4'd8 : prem[3:0];
  assign pop  = (st == S_RUN) && !bfull && (prem != 14'd0) && ({2'b0, fcnt} >= {4'd0, take});
  assign d2   = p2obj ? hs_ordata : p2d;
  // insert at the pre-pop count (fits: credit check at issue), then apply the pop shift
  always_comb begin
    logic [255:0] ex;
    logic [5:0]   o;
    logic [2:0]   j;
    ex = fq;
    for (int i = 0; i < 32; i++) begin
      o = 6'(i) - fcnt;
      j = 3'(i) + p2rot;
      if (p2v && (6'(i) >= fcnt) && (o < {2'b0, p2n_r})) ex[8*i +: 8] = d2[8*j +: 8];
    end
    fq_n = (pop && take[3]) ? {64'd0, ex[255:64]} : ex;
  end

  // final template
  logic [5:0]   tr;
  logic [16:0]  tbits;
  logic         tgo;
  logic [511:0] tmpl;
  logic [63:0]  tctm;
  assign tr    = jL[5:0];
  assign tbits = {jL + 14'd4, 3'b000};
  assign tgo   = (st == S_RUN) && !tdone && !bfull && (pleft == 8'd0) && (prem == 14'd0);
  always_comb begin
    for (int i = 0; i < 64; i++) begin
      if (6'(i) < tr)              tmpl[8*i +: 8] = bb[8*i +: 8];
      else if (6'(i) == tr + 6'd4) tmpl[8*i +: 8] = 8'h80;
      else                         tmpl[8*i +: 8] = 8'h00;
      tctm[i] = (6'(i) == tr + 6'd3);
    end
    tmpl[8*61 +: 8] = {7'd0, tbits[16]};
    tmpl[8*62 +: 8] = tbits[15:8];
    tmpl[8*63 +: 8] = tbits[7:0];
  end

  // ---------------- core scheduling ----------------
  logic [3:0] cfree;
  logic       pre_go, pre_last_now, fin_go;
  always_comb begin
    for (int k = 0; k < 4; k++) cfree[k] = !c_busy[k] && !rv[k] && !(c_done[k] && fr[k]);
  end
  assign pre_go       = (st == S_RUN) && bfull && !c_busy[0];
  assign pre_last_now = c_done[0] && pre0 && (pcl == 8'd1);
  logic       fin_base;
  assign fin_base     = (st == S_RUN) && tdone && ((pcl == 8'd0) || pre_last_now) && ict_ok;
  assign fin_go       = fin_base && |(cfree & ioh);
  always_comb begin
    c_start = {4{fin_base}} & cfree & ioh;
    if (pre_go) c_start[0] = 1'b1;
  end

  // ---------------- output stage ----------------
  logic [6:0]   dct;
  logic [1:0]   dcore;
  logic         ovalid, olast;
  logic [1:0]   ocl;
  logic [255:0] oreg;
  logic         avail, ocons, move;
  logic [255:0] dsel;
  assign avail = rv[dcore] || (c_done[dcore] && fr[dcore]);
  assign ocons = ovalid && xo_ready[ocl];
  assign move  = (st == S_RUN) && avail && (!ovalid || ocons);
  always_comb begin
    for (int i = 0; i < 32; i++) dsel[8*i +: 8] = c_cv[dcore][255 - 8*i -: 8];
  end
  always_comb begin
    xo_valid = '0;
    xo_valid[ocl] = ovalid;
  end
  assign xo_data = oreg;
  assign xo_last = olast;
  assign hs_busy = (st != S_IDLE);

  logic cwr;
  assign cwr = c_done[0] && pre0 && first_pre && jcv && !jhit && !poison && !cache_clr && !cache_inv[ce];

  // ---------------- sequential ----------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE;
      jcl <= '0; jk0 <= '0; jk1 <= '0; jcache <= '0; jhit <= 1'b0;
      cval <= '0; poison <= 1'b0;
      fseg <= 2'd3; frem <= '0; fa <= '0;
      fcnt <= '0; prem <= '0; lp <= '0; bfull <= 1'b0; pleft <= '0; pcl <= '0;
      first_pre <= 1'b0; tdone <= 1'b0; ctm <= '0;
      ict <= '0; icore <= '0; ioh <= 4'b0001; ict_ok <= 1'b0; pend <= '0; pre0 <= 1'b0; fr <= '0; rv <= '0;
      dct <= '0; dcore <= '0; ovalid <= 1'b0; olast <= 1'b0; ocl <= '0;
    end else begin
      // cache maintenance
      if (cache_clr) cval <= '0;
      else           cval <= cval & ~cache_inv;
      if (cwr) cval[ce] <= 1'b1;
      if (st == S_RUN && (cache_clr || (jcv && cache_inv[ce]))) poison <= 1'b1;

      case (st)
        S_IDLE: if (grant) begin
          st     <= S_SETUP;
          jcl    <= wsel;
          jk0    <= hj_kind0[wsel]; jk1 <= hj_kind1[wsel];
          jcache <= hj_cache[wsel];
        end
        S_SETUP: begin
          st      <= S_RUN;
          jhit    <= su_hit;
          poison  <= 1'b0;
          if (su_hit) begin
            fseg <= 2'd1; fa <= jo1; frem <= (jk1 != 2'd0) ? {1'b0, jl1} : 14'd0;
          end else begin
            fseg <= 2'd0; fa <= jo0; frem <= (jk0 != 2'd0) ? {1'b0, jl0} : 14'd0;
          end
          fcnt  <= '0;
          prem  <= su_L - (su_hit ? 14'd64 : 14'd0);
          pleft <= su_L[13:6] - {7'd0, su_hit};
          pcl   <= su_L[13:6] - {7'd0, su_hit};
          lp <= '0; bfull <= 1'b0; tdone <= 1'b0; ctm <= '0;
          first_pre <= 1'b1;
          ict <= 7'd1; icore <= '0; ioh <= 4'b0001; ict_ok <= 1'b1; dct <= 7'd1; dcore <= '0;
          pend <= '0;
        end
        default: ;
      endcase

      if (st == S_RUN) begin
        // fetch sequencing
        if (f_adv) begin
          fseg <= fseg + 2'd1;
          case (fseg)
            2'd0:    begin fa <= jo1; frem <= (jk1 != 2'd0) ? {1'b0, jl1} : 14'd0; end
            2'd1:    begin fa <= '0;  frem <= {11'd0, jsfl}; end
            default: begin frem <= '0; end
          endcase
        end else if (f_iss) begin        // not the last chunk: fn == f_nmax
          fa   <= {fa[12:3] + 10'd1, 3'b000};
          frem <= frem - {10'd0, f_nmax};
        end
        // FIFO
        fcnt <= fcnt - (pop ? {2'b0, take} : 6'd0) + (p2v ? {2'b0, p2n_r} : 6'd0);
        // packer
        if (pop) begin
          prem <= prem - {10'd0, take};
          lp   <= lp + 3'd1;
          if (lp == 3'd7 && pleft != 8'd0) begin
            bfull <= 1'b1;
            pleft <= pleft - 8'd1;
          end
        end
        if (pre_go) bfull <= 1'b0;
        if (tgo) begin
          tdone <= 1'b1;
          ctm   <= tctm;
        end
        // prefix chain
        if (pre_go) pre0 <= 1'b1;
        else if (c_done[0] && pre0) pre0 <= 1'b0;
        if (c_done[0] && pre0) begin
          pcl       <= pcl - 8'd1;
          first_pre <= 1'b0;
        end
        // final issue
        if (fin_go) begin
          ict    <= ict + 7'd1;
          icore  <= icore + 2'd1;
          ioh    <= {ioh[2:0], ioh[3]};
          ict_ok <= (ict < jcnt);
        end
        pend <= pend - (pop ? {3'd0, take} : 7'd0) + (f_iss ? {3'd0, fn} : 7'd0);
        // delivery
        if (move) begin
          dct   <= dct + 7'd1;
          dcore <= dcore + 2'd1;
          ovalid <= 1'b1;
          olast  <= (dct == jcnt);
          ocl    <= jcl;
        end else if (ocons) begin
          ovalid <= 1'b0;
        end
        if (ocons && olast) st <= S_IDLE;
      end

      for (int k = 0; k < 4; k++) begin
        if (c_done[k] && fr[k]) begin
          fr[k] <= 1'b0;
          if (!(move && dcore == 2'(k))) rv[k] <= 1'b1;
        end else if (move && dcore == 2'(k)) begin
          rv[k] <= 1'b0;
        end
        if (fin_go && ioh[k]) fr[k] <= 1'b1;
      end
    end
  end

  // wide datapath registers (no reset)
  always_ff @(posedge clk) begin
    if (st == S_IDLE && grant) begin
      jr0 <= hj_reg0[wsel]; jb0 <= hj_buf0[wsel]; jo0 <= hj_off0[wsel]; jl0 <= hj_len0[wsel];
      jr1 <= hj_reg1[wsel]; jb1 <= hj_buf1[wsel]; jo1 <= hj_off1[wsel]; jl1 <= hj_len1[wsel];
      jsfx <= hj_sfx[wsel]; jsfl <= hj_sfxlen[wsel]; jcnt <= hj_count[wsel];
    end
    if (st == S_SETUP) begin
      jL  <= su_L;
      cvr <= su_hit ? cmid[ce] : SM3_IV;
    end
    if (st == S_RUN && c_done[0] && pre0) cvr <= c_cv[0];
    if (cwr) cmid[ce] <= c_cv[0];
    if (st == S_RUN) fq <= fq_n;
    if (pop) bb[64*lp +: 64] <= fq[63:0];
    if (tgo) bb <= tmpl;
    if (move) oreg <= dsel;
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin
    if (rst_n && st == S_SETUP) begin
      if ({8'd0, su_L[5:0]} + 14'd4 > 14'd55)
        $error("plc_hash: job length %0d violates (L mod 64)+4 <= 55", su_L);
      if (jcnt == 7'd0 || jcnt > 7'd64)
        $error("plc_hash: job count %0d out of range 1..64", jcnt);
      if (jcv && !(jk0 == 2'd1 && jl0 == 13'd64 && jo0 == 13'd0))
        $error("plc_hash: hj_cache set but seg0 is not a 64-byte register");
    end
    if (rst_n && pop && lp == 3'd7 && pleft == 8'd0)
      $error("plc_hash: final block overflow");
  end
`endif
endmodule
