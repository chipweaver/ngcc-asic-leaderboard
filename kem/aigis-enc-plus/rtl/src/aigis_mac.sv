// Slot-wise base multiplication in the NTT domain: c = a o b, slot i holds the degree-(m-1)
// polynomial sum_j x^j * coeff[128j + i] and x^m = zeta_i (SLOTZ[i]).
// Word w of a slot memory holds 16/m complete slots: lane g -> slot group t = g mod (16/m)
// (slot index i = t*W + w), coefficient index k = g div (16/m).
//
// c_k = sum_j a_j * e(k - j),  e(d) = b_d (d >= 0), zeta_i * b_{d+m} (d < 0).
// Two arrays of 16 lanes x 4 multipliers work on a word pair (2u, 2u+1) issued together: array 1
// on the even word, array 2 on the odd word (32 zeta multipliers).  Step s of a word covers
// j = s + S*q, q = 0..3 (S = m/4), i.e. block s of every gen_a quarter.
// Normal mode: all S steps per word (S cycles per word pair); result to s_out (oop) or in place.
// Pass mode (KeyGen a o s): S passes over all words, pass s performs step s only (one word pair
// per cycle) and starts when block s of every gen_a quarter is written (qprog); pass s > 0 adds the
// partial result of pass s-1 read back from a temporary slot.  Temporary slots alternate between
// s_x and s_y so that the final pass reads s_x and writes s_out.
// fol (normal mode): word pair u is issued only once both words are final in the NTT output
// (nwfin) or the followed NTT job has finished (fdone).
// prog counts the words written by the final pass (in order).
module aigis_mac (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              start,
  input  logic              pass_mode,
  input  logic              oop,
  input  logic              fol,
  input  logic [1:0]        param,
  input  logic [2:0]        s_a, s_b, s_out, s_x, s_y,
  input  logic [127:0]      nwfin,
  input  logic              fdone,
  input  logic [3:0][9:0]   qprog,        // gen_a: coefficients written per quarter
  input  logic              qall,         // gen_a job finished
  output logic              busy,
  output logic              done,
  output logic [7:0]        prog,
  output logic [2:0]        sel_a, sel_b, sel_p, sel_c,
  // read roles a, b, p (partial); write role c
  output logic [31:0]       re_a, re_b, re_p,
  output logic [31:0][5:0]  ra_a, ra_b, ra_p,
  input  logic [31:0][11:0] rd_a, rd_b, rd_p,
  output logic [31:0]       we_c,
  output logic [31:0][5:0]  wa_c,
  output logic [31:0][11:0] wd_c
);
  import aigis_pkg::*;

  logic [1:0] p;
  logic       pm, op_q, fol_q;
  logic [2:0] sa_q, sb_q, so_q, sx_q, sy_q;
  logic [7:0] W;
  logic [2:0] S;             // steps per word: 1, 2, 4
  logic [2:0] Se;            // steps per word in this job: S (normal) or 1 (pass)
  logic [1:0] np_m1;         // passes - 1
  logic [4:0] m;
  logic [1:0] mlog;          // log2(16/m): 2, 1, 0
  assign W = wcount(p);
  assign S = 3'd1 << (p - 2'd1);
  assign Se = pm ? 3'd1 : S;
  assign np_m1 = pm ? 2'(S - 3'd1) : 2'd0;
  assign m = 5'd4 << (p - 2'd1);
  assign mlog = 2'd3 - p;

  // temporary slot written by pass s (s < S-1): s_x when S-2-s is even
  function automatic logic [2:0] tslot(input logic [1:0] s, input logic [2:0] ss, input logic [2:0] x, input logic [2:0] y);
    logic [2:0] d;
    d = 3'(ss - 3'd2 - {1'b0, s});
    tslot = d[0] ? y : x;
  endfunction

  // ---------------------------------------------------------------- issue
  logic       issuing;
  logic [1:0] ps;            // pass
  logic [6:0] u;             // word pair
  logic [2:0] scnt;
  logic       gate, iss;
  always_comb begin
    logic [9:0] need;
    need = 10'({ps, 7'd0}) + 10'd128;
    gate = 1'b0;
    if (pm && !qall)
      for (int q = 0; q < 4; q++) if (qprog[q] < need) gate = 1'b1;
    if (!pm && fol_q && !fdone && !(nwfin[{u, 1'b0}] && nwfin[{u, 1'b1}])) gate = 1'b1;
    iss = issuing && (scnt == 3'd0) && !gate;
  end
  logic wr_v, wr_last;
  logic [6:0] wr_u;
  logic [2:0] wr_s;
  logic       wr_fin;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy <= 1'b0; done <= 1'b0; issuing <= 1'b0; ps <= '0; u <= '0; scnt <= '0; p <= 2'd1; prog <= '0;
      pm <= 1'b0; op_q <= 1'b0; fol_q <= 1'b0; sa_q <= '0; sb_q <= '0; so_q <= '0; sx_q <= '0; sy_q <= '0;
    end else begin
      done <= 1'b0;
      if (start && !busy) begin
        busy <= 1'b1; issuing <= 1'b1; ps <= '0; u <= '0; scnt <= '0; p <= param; prog <= '0;
        pm <= pass_mode; op_q <= oop; fol_q <= fol;
        sa_q <= s_a; sb_q <= s_b; so_q <= s_out; sx_q <= s_x; sy_q <= s_y;
      end else if (busy) begin
        if (issuing) begin
          if (scnt != 3'd0) scnt <= (scnt == Se - 3'd1) ? 3'd0 : scnt + 3'd1;
          else if (iss) begin
            scnt <= (Se == 3'd1) ? 3'd0 : 3'd1;
            if (u == 7'(W[7:1] - 8'd1)) begin
              u <= '0;
              if (ps == np_m1) issuing <= 1'b0; else ps <= ps + 2'd1;
            end else u <= u + 7'd1;
          end
        end
        if (wr_v && wr_fin) prog <= prog + 8'd2;
        if (wr_v && wr_last) begin busy <= 1'b0; done <= 1'b1; end
      end
    end
  end

  // read issue (t0): both words of the pair, both logical banks, address u
  // pass mode: read only the a-lanes of block s of every quarter (lane g holds block g >> mlog;
  // pass s uses blocks j with j mod S = s), so no a-slot lane that gen_a may still be writing is read
  always_comb begin
    for (int i = 0; i < 32; i++) begin
      logic [3:0] g, jb;
      g = 4'(i >> 1);
      jb = g >> mlog;
      re_a[i] = iss && (!pm || ((jb & 4'(S - 3'd1)) == 4'(ps)));
      re_b[i] = iss; ra_a[i] = 6'(u); ra_b[i] = 6'(u);
    end
  end
  assign sel_a = sa_q;
  assign sel_b = sb_q;

  // ---------------------------------------------------------------- t1: data, zeta, zb multiply
  typedef struct packed {
    logic       v;
    logic [6:0] u;
    logic [1:0] ps;
  } ctl_t;
  ctl_t k1, k2, k3, k4;
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin k1 <= '0; k2 <= '0; k3 <= '0; k4 <= '0; end
    else begin k1 <= '{v: iss, u: u, ps: ps}; k2 <= k1; k3 <= k2; k4 <= k3; end

  logic [6:0] w0, w1;
  assign w0 = {k1.u[5:0], 1'b0};
  assign w1 = {k1.u[5:0], 1'b1};
  logic [15:0][11:0] a1d, a2d, b1d, b2d, z1, z2;
  always_comb begin
    for (int g = 0; g < 16; g++) begin
      logic [3:0] t;
      logic [6:0] si0, si1;
      a1d[g] = rd_a[2*g + wbank(w0)];
      a2d[g] = rd_a[2*g + wbank(w1)];
      b1d[g] = rd_b[2*g + wbank(w0)];
      b2d[g] = rd_b[2*g + wbank(w1)];
      t  = 4'(g) & ((4'd1 << mlog) - 4'd1);
      si0 = 7'((({3'd0, t}) << (3'd5 + {1'b0, p - 2'd1})) + {1'b0, w0});
      si1 = 7'((({3'd0, t}) << (3'd5 + {1'b0, p - 2'd1})) + {1'b0, w1});
      z1[g] = slot_zeta(si0);
      z2[g] = slot_zeta(si1);
    end
  end
  logic [15:0][11:0] zb1, zb2;
  for (genvar g = 0; g < 16; g++) begin : g_zb
    aigis_modmul u_zb1 (.clk(clk), .a(b1d[g]), .b(z1[g]), .r(zb1[g]));
    aigis_modmul u_zb2 (.clk(clk), .a(b2d[g]), .b(z2[g]), .r(zb2[g]));
  end
  logic [15:0][11:0] a1_2, a1_3, a1_4, a2_2, a2_3, a2_4, b1_2, b1_3, b1_4, b2_2, b2_3, b2_4;
  always_ff @(posedge clk) begin
    a1_2 <= a1d; a1_3 <= a1_2; a1_4 <= a1_3;
    a2_2 <= a2d; a2_3 <= a2_2; a2_4 <= a2_3;
    b1_2 <= b1d; b1_3 <= b1_2; b1_4 <= b1_3;
    b2_2 <= b2d; b2_3 <= b2_2; b2_4 <= b2_3;
  end

  // ---------------------------------------------------------------- t4: load E / A registers
  // E[t*2m + x] (x < m: zb_t[x], x >= m: b_t[x-m]); A[t*m + y] = a_t[y]; coefficient j of slot
  // group t sits in lane t + j*(16/m).  Loaded shifted by the pass (pass mode).
  function automatic logic [3:0] lane_of(input logic [3:0] t, input logic [4:0] j, input logic [1:0] ml);
    lane_of = 4'(t + (j << ml));
  endfunction
  function automatic logic [31:0][11:0] ebuild(input logic [15:0][11:0] zb, input logic [15:0][11:0] bb,
                                               input logic [1:0] pp, input logic [1:0] s0);
    logic [4:0] mm; logic [1:0] ml;
    mm = 5'd4 << (pp - 2'd1); ml = 2'd3 - pp;
    for (int x = 0; x < 32; x++) begin
      logic [3:0] t; logic [4:0] xx, xs;
      case (pp)
        2'd1: begin t = 4'(x / 8);  xx = 5'(x % 8);  end
        2'd2: begin t = 4'(x / 16); xx = 5'(x % 16); end
        default: begin t = 4'd0; xx = 5'(x); end
      endcase
      xs = xx - {3'd0, s0};
      if (xx < {3'd0, s0}) ebuild[x] = 12'd0;
      else ebuild[x] = (xs < mm) ? zb[lane_of(t, xs, ml)] : bb[lane_of(t, xs - mm, ml)];
    end
  endfunction
  function automatic logic [15:0][11:0] abuild(input logic [15:0][11:0] av, input logic [1:0] pp, input logic [1:0] s0);
    logic [4:0] mm; logic [1:0] ml;
    mm = 5'd4 << (pp - 2'd1); ml = 2'd3 - pp;
    for (int y = 0; y < 16; y++) begin
      logic [3:0] t; logic [4:0] yy, ys;
      case (pp)
        2'd1: begin t = 4'(y / 4); yy = 5'(y % 4); end
        2'd2: begin t = 4'(y / 8); yy = 5'(y % 8); end
        default: begin t = 4'd0; yy = 5'(y); end
      endcase
      ys = yy + {3'd0, s0};
      abuild[y] = (ys < mm) ? av[lane_of(t, ys, ml)] : 12'd0;
    end
  endfunction
  function automatic logic [31:0][11:0] eshift(input logic [31:0][11:0] e, input logic [1:0] pp);
    for (int x = 0; x < 32; x++) begin
      logic [4:0] xx;
      case (pp)
        2'd1: xx = 5'(x % 8);
        2'd2: xx = 5'(x % 16);
        default: xx = 5'(x);
      endcase
      eshift[x] = (xx >= 5'd1) ? e[x-1] : 12'd0;
    end
  endfunction
  function automatic logic [15:0][11:0] ashift(input logic [15:0][11:0] av, input logic [1:0] pp, input logic [4:0] mm);
    for (int y = 0; y < 16; y++) begin
      logic [4:0] yy;
      case (pp)
        2'd1: yy = 5'(y % 4);
        2'd2: yy = 5'(y % 8);
        default: yy = 5'(y);
      endcase
      ashift[y] = (yy + 5'd1 < mm) ? av[y == 15 ? 15 : y+1] : 12'd0;
    end
  endfunction

  logic [31:0][11:0] E1, E2, E1in, E2in;
  logic [15:0][11:0] A1, A2, A1in, A2in;
  logic [1:0] s0;
  assign s0 = pm ? k4.ps : 2'd0;
  always_comb begin
    E1in = ebuild(zb1, b1_4, p, s0);
    E2in = ebuild(zb2, b2_4, p, s0);
    A1in = abuild(a1_4, p, s0);
    A2in = abuild(a2_4, p, s0);
  end
  logic [2:0] st;
  logic       stv;
  logic [6:0] stu;
  logic [1:0] stps;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin stv <= 1'b0; st <= '0; end
    else begin
      if (k4.v) begin stv <= 1'b1; st <= '0; end
      else if (stv) begin if (st == Se - 3'd1) stv <= 1'b0; st <= st + 3'd1; end
    end
  end
  always_ff @(posedge clk) begin
    if (k4.v) begin stu <= k4.u; stps <= k4.ps; E1 <= E1in; A1 <= A1in; E2 <= E2in; A2 <= A2in; end
    else if (stv) begin E1 <= eshift(E1, p); A1 <= ashift(A1, p, m); E2 <= eshift(E2, p); A2 <= ashift(A2, p, m); end
  end
  logic stlast;
  assign stlast = stv && (st == Se - 3'd1);

  // partial read (pass mode, pass > 0): issued in the first step cycle, data with the first sum
  logic pr_en;
  assign pr_en = pm && stv && (st == 3'd0) && (stps != 2'd0);
  always_comb begin
    for (int i = 0; i < 32; i++) begin re_p[i] = pr_en; ra_p[i] = 6'(stu); end
  end
  assign sel_p = tslot(stps - 2'd1, S, sx_q, sy_q);

  // ---------------------------------------------------------------- multiply (step stage)
  logic [15:0][3:0][23:0] pr1, pr2;
  always_ff @(posedge clk) begin
    for (int g = 0; g < 16; g++) begin
      logic [3:0] t, k;
      t = 4'(g) & ((4'd1 << mlog) - 4'd1);
      k = 4'(g) >> mlog;
      for (int q = 0; q < 4; q++) begin
        logic [4:0] ai, ei;
        ai = 5'((t * m) + (S * q));
        ei = 5'((t * 2 * m) + k + m - (S * q));
        pr1[g][q] <= A1[ai[3:0]] * E1[ei];
        pr2[g][q] <= A2[ai[3:0]] * E2[ei];
      end
    end
  end
  logic pv, plast, pfirst, ppart;
  logic [6:0] pu; logic [1:0] pps;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin pv <= 1'b0; plast <= 1'b0; pfirst <= 1'b0; ppart <= 1'b0; end
    else begin pv <= stv; plast <= stlast; pfirst <= stv && (st == 3'd0); ppart <= pr_en; end
  end
  always_ff @(posedge clk) begin pu <= stu; pps <= stps; end
  logic [6:0] pw0, pw1;
  assign pw0 = {pu[5:0], 1'b0};
  assign pw1 = {pu[5:0], 1'b1};

  // ---------------------------------------------------------------- accumulate
  logic [15:0][27:0] acc1, acc2, sum1, sum2;
  always_comb begin
    for (int g = 0; g < 16; g++) begin
      logic [27:0] i1, i2;
      i1 = ppart ? {16'd0, rd_p[2*g + wbank(pw0)]} : 28'd0;
      i2 = ppart ? {16'd0, rd_p[2*g + wbank(pw1)]} : 28'd0;
      sum1[g] = (pfirst ? i1 : acc1[g]) + pr1[g][0] + pr1[g][1] + pr1[g][2] + pr1[g][3];
      sum2[g] = (pfirst ? i2 : acc2[g]) + pr2[g][0] + pr2[g][1] + pr2[g][2] + pr2[g][3];
    end
  end
  always_ff @(posedge clk) begin if (pv) begin acc1 <= sum1; acc2 <= sum2; end end
  logic [15:0][11:0] r1, r2;
  for (genvar g = 0; g < 16; g++) begin : g_red
    aigis_modred28 u_r1 (.clk(clk), .x(sum1[g]), .r(r1[g]));
    aigis_modred28 u_r2 (.clk(clk), .x(sum2[g]), .r(r2[g]));
  end
  logic rva, rvb; logic [6:0] rua, rub; logic [1:0] rpa, rpb;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin rva <= 1'b0; rvb <= 1'b0; end
    else begin rva <= pv && plast; rvb <= rva; end
  end
  always_ff @(posedge clk) begin rua <= pu; rub <= rua; rpa <= pps; rpb <= rpa; end

  // ---------------------------------------------------------------- write: both words of the pair
  assign wr_v    = rvb;
  assign wr_u    = rub;
  assign wr_fin  = (rpb == np_m1);
  assign wr_last = wr_fin && (rub == 7'(W[7:1] - 8'd1));
  assign wr_s    = pm ? (wr_fin ? so_q : tslot(rpb, S, sx_q, sy_q)) : (op_q ? so_q : sa_q);
  assign sel_c   = wr_s;
  always_comb begin
    logic b0;
    b0 = wbank({rub[5:0], 1'b0});
    for (int g = 0; g < 16; g++) begin
      we_c[2*g] = rvb; we_c[2*g + 1] = rvb; wa_c[2*g] = 6'(rub); wa_c[2*g + 1] = 6'(rub);
      wd_c[2*g]     = b0 ? r2[g] : r1[g];
      wd_c[2*g + 1] = b0 ? r1[g] : r2[g];
    end
  end
endmodule
