// NTT / INTT unit: radix-4 (two levels per pass), 64 butterflies per cycle on one poly slot,
// physical four-bank port.
//
// The poly (n = 128*m, W = n/16 words per lane) is m independent 128-point blocks; lane g, word w
// holds coefficient c = g*W + w, block j = c/128, block-local index i = (g mod 16/m)*W + w.
// Forward (CT, levels L = 6..0): x' = x + z*y, y' = x - z*y, z = TW[2^(6-L) + (i >> (L+1))].
// Inverse (GS, levels L = 0..6): x' = x + y, y' = z*(x - y), z = TWI[128 - 2^(7-L) + (i >> (L+1))],
// and at L = 6 additionally x' = (x + y) * 128^-1 (exact division by 2^7 modulo q, q = 1 mod 128).
//
// Memory: lane g has four banks, physical bank of word w = {po, pe} with pe = w0^w2^w4^w6,
// po = w1^w3^w5, address w >> 2 (aigis_polymem).  Port index 4g + {po, pe}.
//
// A transform is four passes of W/4 operations; one operation per cycle reads four words in every
// lane (four distinct banks), runs up to two butterfly levels on them and writes them back.
//   inter pair (La, Lb), La, Lb < lw (= log2 W), opposite parity:
//       words q0 = a, q1 = a^2^La, q2 = a^2^Lb, q3 = a^2^La^2^Lb;
//       stage 1 (level La): (q0, q1), (q2, q3); stage 2 (level Lb): (q0, q2), (q1, q3).
//   inter single Lb with partner La: same words, stage 1 bypassed.
//   intra (level >= lw, lanes (g, g + 2^(L-lw)) of one block): words 4a .. 4a+3, eight lane-pair
//       butterflies per word; intra pair (I only: levels 5, 6) uses both stages, intra single
//       stage 2 only.
// Pass lists (forward | inverse):
//   III: (6,5) (4,3) (2,1) 0/1      | (0,1) (2,3) (4,5) 6/5
//   II : 6i (5,4) (3,2) (1,0)        | (0,1) (2,3) (4,5) 6i
//   I  : (6,5)i (4,3) (2,1) 0/1      | (0,1) (2,3) 4/3 (5,6)i
// Every pass reads the destination slot and writes it in place, except that the first pass reads
// the source slot and the final pass writes s_fin (out-of-place final pass, so a consumer may read
// the result while the final pass is still running: wfin[w] marks words already final).
// A scoreboard stalls an operation while any of its four words is still in flight (issued in the
// last LAT cycles), so dependent passes follow each other without a drain.
//
// FOLLOW mode (inverse only): the INTT runs behind the base multiplication that writes its source.
// Per 16-word chunk c (source words < fprog, or the followed job finished): pass (0,1) reads the
// source chunk, pass (2,3) the destination chunk; for W = 128 after every four chunks pass (4,5) on
// that 64-word half; then the remaining global passes.
module aigis_ntt (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              start,
  input  logic              inv,
  input  logic              follow,
  input  logic [7:0]        fprog,       // words 0..fprog-1 of the source slot are final
  input  logic              fdone,       // the followed job has finished (source complete)
  input  logic [2:0]        s_src,
  input  logic [2:0]        s_dst,
  input  logic [2:0]        s_fin,
  output logic [2:0]        rsel,
  output logic [2:0]        wsel,
  input  logic [1:0]        param,
  output logic              busy,
  output logic              done,
  output logic [127:0]      wfin,        // word w of the final slot written by the final pass
  output logic [63:0]       re,
  output logic [63:0][4:0]  ra,
  input  logic [63:0][11:0] rd,
  output logic [63:0]       we,
  output logic [63:0][4:0]  wa,
  output logic [63:0][11:0] wd
);
  import aigis_pkg::*;
  localparam int LAT = 9;

  // pass kinds
  localparam logic [1:0] K_IP = 2'd0, K_IS = 2'd1, K_XS = 2'd2, K_XP = 2'd3;
  typedef struct packed {
    logic [1:0] kind;
    logic [2:0] la, lb;
  } pass_t;
  function automatic pass_t PS(input logic [1:0] k, input int a, input int b);
    PS.kind = k; PS.la = 3'(a); PS.lb = 3'(b);
  endfunction
  function automatic pass_t plist(input logic [1:0] pp, input logic iv, input logic [1:0] k);
    pass_t r;
    r = PS(K_IP, 0, 1);
    case ({pp, iv})
      3'b11_0: case (k) 2'd0: r = PS(K_IP, 6, 5); 2'd1: r = PS(K_IP, 4, 3); 2'd2: r = PS(K_IP, 2, 1); default: r = PS(K_IS, 1, 0); endcase
      3'b11_1: case (k) 2'd0: r = PS(K_IP, 0, 1); 2'd1: r = PS(K_IP, 2, 3); 2'd2: r = PS(K_IP, 4, 5); default: r = PS(K_IS, 5, 6); endcase
      3'b10_0: case (k) 2'd0: r = PS(K_XS, 0, 6); 2'd1: r = PS(K_IP, 5, 4); 2'd2: r = PS(K_IP, 3, 2); default: r = PS(K_IP, 1, 0); endcase
      3'b10_1: case (k) 2'd0: r = PS(K_IP, 0, 1); 2'd1: r = PS(K_IP, 2, 3); 2'd2: r = PS(K_IP, 4, 5); default: r = PS(K_XS, 0, 6); endcase
      3'b01_0: case (k) 2'd0: r = PS(K_XP, 6, 5); 2'd1: r = PS(K_IP, 4, 3); 2'd2: r = PS(K_IP, 2, 1); default: r = PS(K_IS, 1, 0); endcase
      default: case (k) 2'd0: r = PS(K_IP, 0, 1); 2'd1: r = PS(K_IP, 2, 3); 2'd2: r = PS(K_IS, 3, 4); default: r = PS(K_XP, 5, 6); endcase
    endcase
    plist = r;
  endfunction

  logic        mode_inv, fol;
  logic [1:0]  p;
  logic [2:0]  lw;
  assign lw = 3'd5 + {1'b0, p - 2'd1};
  logic [5:0]  nq;            // W/4
  assign nq = 6'd8 << (p - 2'd1);
  logic [2:0]  s_src_q, s_dst_q, s_fin_q;

  function automatic logic pe_of(input logic [6:0] w); pe_of = w[0] ^ w[2] ^ w[4] ^ w[6]; endfunction
  function automatic logic po_of(input logic [6:0] w); po_of = w[1] ^ w[3] ^ w[5]; endfunction
  function automatic logic [1:0] pb_of(input logic [6:0] w); pb_of = {po_of(w), pe_of(w)}; endfunction
  function automatic logic [6:0] ins0(input logic [6:0] v, input logic [2:0] b);
    ins0 = ((v >> b) << (b + 3'd1)) | (v & ((7'd1 << b) - 7'd1));
  endfunction

  // ------------------------------------------------------------------ sequencer
  // scopes: 0 chunk (16 words, 4 ops), 1 half (64 words, 16 ops), 2 global (W/4 ops)
  // Follow mode: chunk pass A(c) = (0,1) reads the source chunk c, B(c) = (2,3) the destination
  // chunk c.  A pass is chosen when its first op issues: A(na) while it is at most two chunks ahead
  // of B and its source words are ready, otherwise B(nb); for W = 128 the half pass H(h) follows
  // B(4h+3); the global passes follow the last B (and H(1)).
  typedef enum logic [1:0] {Q_IDLE, Q_RUN, Q_DRAIN} qst_t;
  qst_t        qst;
  logic [1:0]  sc_r, pk_r;    // latched scope / pass of the running pass
  logic [2:0]  fc_r;
  logic        fh_r;
  logic [3:0]  na, nb;        // follow: next A / next B chunk
  logic        h0, h1;        // half passes issued
  logic        pend;          // no op of the current pass issued yet (choice still open)
  logic [4:0]  idx;           // op index within the scope
  logic [3:0]  dcnt;
  logic [3:0]  nch;           // chunks W/16
  assign nch = 4'(nq[5:2]);
  // choice for a pass that has not started
  logic [1:0]  sc_c, pk_c;
  logic [2:0]  fc_c;
  logic        fh_c, gate_a;
  always_comb begin
    logic a_ok;
    gate_a = !fdone && ({1'b0, fprog} < ({5'd0, na} + 9'd1) * 9'd16);
    a_ok = (na < nch) && ((na - nb) < 4'd3) && !gate_a;
    sc_c = 2'd0; pk_c = 2'd1; fc_c = nb[2:0]; fh_c = 1'b0;
    if (!fol) begin sc_c = 2'd2; pk_c = pk_r; end
    else if (lw == 3'd7 && nb == 4'd4 && !h0) begin sc_c = 2'd1; pk_c = 2'd2; fh_c = 1'b0; end
    else if (lw == 3'd7 && nb == 4'd8 && !h1) begin sc_c = 2'd1; pk_c = 2'd2; fh_c = 1'b1; end
    else if (nb == nch) begin sc_c = 2'd2; pk_c = (pk_r < 2'd2) ? ((lw == 3'd7) ? 2'd3 : 2'd2) : pk_r; end
    else if (a_ok || (nb == na)) begin sc_c = 2'd0; pk_c = 2'd0; fc_c = na[2:0]; end
  end
  logic [1:0]  sc, pk;
  logic [2:0]  fc;
  logic        fh;
  assign sc = pend ? sc_c : sc_r;
  assign pk = pend ? pk_c : pk_r;
  assign fc = pend ? fc_c : fc_r;
  assign fh = pend ? fh_c : fh_r;
  pass_t       cp;            // current pass
  logic        cfirst, cfinal;  // current pass reads the source / writes the final slot
  logic [6:0]  cbase;
  logic [4:0]  nops_m1;
  always_comb begin
    cp = plist(p, mode_inv, pk);
    cbase = '0;
    case (sc)
      2'd0: begin nops_m1 = 5'd3;  cbase = {fc, 4'd0}; end
      2'd1: begin nops_m1 = 5'd15; cbase = {fh, 6'd0}; end
      default: nops_m1 = 5'(nq - 6'd1);
    endcase
    cfirst = (pk == 2'd0) && (!fol || sc == 2'd0);
    cfinal = (pk == 2'd3);
  end

  logic [3:0][6:0] qw;
  always_comb begin
    logic [2:0] lo, hi;
    logic [6:0] a;
    lo = (cp.la < cp.lb) ? cp.la : cp.lb;
    hi = (cp.la < cp.lb) ? cp.lb : cp.la;
    if (cp.kind == K_XS || cp.kind == K_XP) begin
      for (int k = 0; k < 4; k++) qw[k] = {idx, 2'(k)};
    end else begin
      a = ins0(ins0({2'd0, idx}, lo), hi) | cbase;
      qw[0] = a;
      qw[1] = a ^ (7'd1 << cp.la);
      qw[2] = a ^ (7'd1 << cp.lb);
      qw[3] = a ^ (7'd1 << cp.la) ^ (7'd1 << cp.lb);
    end
  end

  // pipeline control c1..c9
  typedef struct packed {
    logic       v;
    logic [3:0][6:0] w;
    pass_t      ps;
    logic       fin;          // final pass
    logic [2:0] ws;           // write slot
  } ctl_t;
  ctl_t [LAT:1] c;
  logic hazard, gate, go;
  always_comb begin
    hazard = 1'b0;
    for (int s = 1; s <= LAT; s++)
      for (int k = 0; k < 4; k++)
        for (int j = 0; j < 4; j++)
          if (c[s].v && c[s].w[j] == qw[k]) hazard = 1'b1;
    gate = fol && sc == 2'd0 && pk == 2'd0 && gate_a;
    go = (qst == Q_RUN) && !hazard && !gate;
  end
  assign rsel = cfirst ? s_src_q : s_dst_q;
  assign wsel = c[LAT].ws;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      qst <= Q_IDLE; busy <= 1'b0; done <= 1'b0; sc_r <= '0; pk_r <= '0; fc_r <= '0; fh_r <= 1'b0; idx <= '0; dcnt <= '0;
      na <= '0; nb <= '0; h0 <= 1'b0; h1 <= 1'b0; pend <= 1'b0;
      mode_inv <= 1'b0; fol <= 1'b0; p <= 2'd1; s_src_q <= '0; s_dst_q <= '0; s_fin_q <= '0;
    end else begin
      done <= 1'b0;
      case (qst)
        Q_IDLE: if (start) begin
          busy <= 1'b1; qst <= Q_RUN; mode_inv <= inv; fol <= follow && inv; p <= param;
          s_src_q <= s_src; s_dst_q <= (follow && inv) ? s_dst : s_src; s_fin_q <= s_fin;
          pk_r <= '0; idx <= '0; na <= '0; nb <= '0; h0 <= 1'b0; h1 <= 1'b0; pend <= 1'b1;
        end
        Q_RUN: if (go) begin
          if (pend) begin pend <= 1'b0; sc_r <= sc_c; pk_r <= pk_c; fc_r <= fc_c; fh_r <= fh_c; end
          if (idx != nops_m1) idx <= idx + 5'd1;
          else begin
            idx <= '0; pend <= 1'b1;
            case (sc)
              2'd0: if (pk == 2'd0) na <= na + 4'd1; else nb <= nb + 4'd1;
              2'd1: if (!fh) h0 <= 1'b1; else h1 <= 1'b1;
              default: begin
                if (pk == 2'd3) begin qst <= Q_DRAIN; dcnt <= 4'(LAT); pend <= 1'b0; end
                else pk_r <= pk + 2'd1;
              end
            endcase
            if (sc != 2'd2) pk_r <= 2'd0;  // keep the global-pass pointer below 2 until the global phase
          end
        end
        Q_DRAIN: begin
          if (dcnt == 4'd0) begin qst <= Q_IDLE; busy <= 1'b0; done <= 1'b1; end
          else dcnt <= dcnt - 4'd1;
        end
        default: qst <= Q_IDLE;
      endcase
    end
  end

  // read requests: word qw[k] of every lane on its physical bank
  always_comb begin
    re = '0; ra = '0;
    if (go)
      for (int g = 0; g < 16; g++)
        for (int k = 0; k < 4; k++) begin
          re[4*g + pb_of(qw[k])] = 1'b1;
          ra[4*g + pb_of(qw[k])] = qw[k][6:2];
        end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) c <= '0;
    else begin
      c[1] <= '{v: go, w: qw, ps: cp, fin: cfinal, ws: cfinal ? s_fin_q : s_dst_q};
      for (int s = 2; s <= LAT; s++) c[s] <= c[s-1];
    end
  end

  // final-word bitmap
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) wfin <= '0;
    else if (start && qst == Q_IDLE) wfin <= '0;
    else if (c[LAT].v && c[LAT].fin)
      for (int k = 0; k < 4; k++) wfin[c[LAT].w[k]] <= 1'b1;
  end

  // ------------------------------------------------------------------ datapath
  logic [3:0] mlog;
  assign mlog = 4'(3'd3 - {1'b0, p});
  logic [3:0] gmask;                       // lane group mask: 16/m - 1 = 3, 1, 0
  assign gmask = (4'd1 << mlog) - 4'd1;

  // twiddle of a butterfly whose x operand is word w of lane group gm at level L
  function automatic logic [11:0] twid(input logic iv, input logic [1:0] pp, input logic [1:0] gm, input logic [6:0] w,
                                       input logic [2:0] L);
    logic [6:0] i, k;
    i = 7'((7'(gm) << (3'd5 + {1'b0, pp - 2'd1})) + w);
    if (!iv) k = 7'((7'd1 << (3'd6 - L)) + (i >> (L + 3'd1)));
    else     k = 7'(7'd0 - (7'd1 << (3'd7 - L)) + (i >> (L + 3'd1)));
    // one ROM: TWI[k] = q - TW[127 - k] for k < 126; TWI[126] carries the 1/128 factor
    if (!iv) twid = tw_fwd(k);
    else if (k == 7'd126) twid = 12'd1677;
    else twid = 12'(13'd3329 - {1'b0, tw_fwd(7'd127 - k)});
  endfunction

  // word-slot view of the read data (c1)
  logic [15:0][3:0][11:0] d0;
  always_comb for (int g = 0; g < 16; g++) for (int k = 0; k < 4; k++) d0[g][k] = rd[4*g + pb_of(c[1].w[k])];

  // operand selection for one stage: values v[g][k] (lane g, word slot k) of words w, pass ps, stage st
  // (st = 0: level la, pairs (q0,q1),(q2,q3) / intra lanes by la; st = 1: level lb, pairs (q0,q2),(q1,q3))
  // butterfly u: inter u = 2g + h; intra u = 8k + pi
  logic [31:0][11:0] ax, ay, az, bx, by, bz;
  logic [31:0]       a_byp, b_sc;
  logic [15:0][3:0][11:0] v1;              // stage-1 output (c5), word-slot view
  logic [3:0][1:0][11:0] tz1, tz2;         // twiddles per (lane group, pair h)
  always_comb begin
    pass_t q1p, q2p;
    logic intra1, intra2, dl1, dl2;
    q1p = c[1].ps; q2p = c[5].ps;
    intra1 = q1p.kind[1]; intra2 = q2p.kind[1];
    dl1 = (q1p.la != lw); dl2 = (q2p.lb != lw);
    // inter: stage 1 x words q0 / q2, stage 2 x words q0 / q1; intra: the twiddle depends on the
    // lane group only (level >= lw), word q0 is used
    for (int gm = 0; gm < 4; gm++)
      for (int h = 0; h < 2; h++) begin
        tz1[gm][h] = twid(mode_inv, p, 2'(gm), (intra1 || h == 0) ? c[1].w[0] : c[1].w[2], q1p.la);
        tz2[gm][h] = twid(mode_inv, p, 2'(gm), (intra2 || h == 0) ? c[5].w[0] : c[5].w[1], q2p.lb);
      end
    for (int u = 0; u < 32; u++) begin
      int g, h, kq, pi, gx0, gy0, gx1, gy1, gx, gy;
      g = u >> 1; h = u & 1;
      kq = u >> 3; pi = u % 8;
      gx0 = 2*pi; gy0 = 2*pi + 1;
      gx1 = ((pi >> 1) << 2) | (pi & 1); gy1 = gx1 + 2;
      // stage 1
      gx = dl1 ? gx1 : gx0; gy = dl1 ? gy1 : gy0;
      if (!intra1) begin
        ax[u] = d0[g][2*h]; ay[u] = d0[g][2*h + 1]; az[u] = tz1[4'(g) & gmask][h];
      end else begin
        ax[u] = d0[gx][kq]; ay[u] = d0[gy][kq]; az[u] = tz1[4'(gx) & gmask][0];
      end
      a_byp[u] = (q1p.kind == K_IS) || (q1p.kind == K_XS);
      // stage 2
      gx = dl2 ? gx1 : gx0; gy = dl2 ? gy1 : gy0;
      if (!intra2) begin
        bx[u] = v1[g][h]; by[u] = v1[g][h + 2]; bz[u] = tz2[4'(g) & gmask][h];
      end else begin
        bx[u] = v1[gx][kq]; by[u] = v1[gy][kq]; bz[u] = tz2[4'(gx) & gmask][0];
      end
      b_sc[u] = mode_inv && (q2p.lb == 3'd6);
    end
  end

  // butterfly stage: inputs at t, outputs registered at t+4
  logic [31:0][11:0] aox, aoy, box, boy;
  aigis_bfly u_s1 (.clk(clk), .inv(mode_inv), .x(ax), .y(ay), .z(az), .byp(a_byp), .sc('0), .ox(aox), .oy(aoy));
  aigis_bfly u_s2 (.clk(clk), .inv(mode_inv), .x(bx), .y(by), .z(bz), .byp('0), .sc(b_sc), .ox(box), .oy(boy));

  // stage-1 outputs back to the word-slot view (c5)
  always_comb begin
    pass_t q;
    logic dl;
    q = c[5].ps;
    dl = (q.la != lw);
    for (int g = 0; g < 16; g++)
      for (int k = 0; k < 4; k++) begin
        int r;
        if (!q.kind[1]) v1[g][k] = (k % 2 == 0) ? aox[2*g + k/2] : aoy[2*g + k/2];
        else if (!dl) v1[g][k] = (g % 2 == 0) ? aox[8*k + g/2] : aoy[8*k + (g-1)/2];
        else begin
          r = g % 4;
          v1[g][k] = (r < 2) ? aox[8*k + 2*(g/4) + r] : aoy[8*k + 2*(g/4) + r - 2];
        end
      end
  end

  // write stage (c9): lane g, bank pb(q_k) <- stage-2 output of word slot k
  always_comb begin
    pass_t q;
    logic dl;
    q = c[LAT].ps;
    dl = (q.lb != lw);
    we = '0; wa = '0; wd = '0;
    for (int g = 0; g < 16; g++)
      for (int k = 0; k < 4; k++) begin
        logic [11:0] v;
        int r;
        if (!q.kind[1]) v = (k < 2) ? box[2*g + k] : boy[2*g + k - 2];
        else if (!dl) v = (g % 2 == 0) ? box[8*k + g/2] : boy[8*k + (g-1)/2];
        else begin
          r = g % 4;
          v = (r < 2) ? box[8*k + 2*(g/4) + r] : boy[8*k + 2*(g/4) + r - 2];
        end
        for (int b = 0; b < 4; b++)
          if (pb_of(c[LAT].w[k]) == 2'(b)) begin
            we[4*g + b] = c[LAT].v; wa[4*g + b] = c[LAT].w[k][6:2]; wd[4*g + b] = v;
          end
      end
  end
endmodule

// 32 butterflies, latency 4 (inputs at t, registered outputs at t+4).
// forward: x' = x + z*y, y' = x - z*y; inverse: x' = x + y (/128 when sc), y' = z*(x - y);
// byp: x' = x, y' = y.
module aigis_bfly (
  input  logic              clk,
  input  logic              inv,
  input  logic [31:0][11:0] x, y, z,
  input  logic [31:0]       byp,
  input  logic [31:0]       sc,
  output logic [31:0][11:0] ox, oy
);
  import aigis_pkg::*;
  function automatic logic [11:0] div128(input logic [11:0] v);
    logic [6:0]  kk;
    logic [18:0] t;
    kk = 7'(7'd0 - v[6:0]);
    t  = {7'd0, v} + 19'(kk) * 19'd3329;
    div128 = t[18:7];
  endfunction
  logic [31:0][11:0] ma, mb, mr, xs1, xs2, xs3, xs4;
  logic [31:0] b1, b2, b3, b4, s2;
  always_comb begin
    for (int u = 0; u < 32; u++) begin
      if (byp[u])    begin ma[u] = y[u]; mb[u] = 12'd1; xs1[u] = x[u]; end
      else if (!inv) begin ma[u] = y[u]; mb[u] = z[u]; xs1[u] = x[u]; end
      else           begin ma[u] = subq(x[u], y[u]); mb[u] = z[u]; xs1[u] = addq(x[u], y[u]); end
    end
  end
  for (genvar u = 0; u < 32; u++) begin : g_m
    aigis_modmul u_m (.clk(clk), .a(ma[u]), .b(mb[u]), .r(mr[u]));
  end
  always_ff @(posedge clk) begin
    xs2 <= xs1; b2 <= byp; s2 <= sc;
    for (int u = 0; u < 32; u++) xs3[u] <= s2[u] ? div128(xs2[u]) : xs2[u];
    b3 <= b2;
    xs4 <= xs3; b4 <= b3;
  end
  always_ff @(posedge clk) begin
    for (int u = 0; u < 32; u++) begin
      if (b4[u] || inv) begin ox[u] <= xs4[u]; oy[u] <= mr[u]; end
      else begin ox[u] <= addq(xs4[u], mr[u]); oy[u] <= subq(xs4[u], mr[u]); end
    end
  end
endmodule
