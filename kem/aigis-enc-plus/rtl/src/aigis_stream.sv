// Natural-order stream unit between one object buffer and poly slots, 16 lanes in parallel.
// Lane g owns segment g: coefficients g*W .. g*W+W-1, i.e. buffer words base + g*SW .. +SW-1
// (SW = W*F/64 words for field width F; every segment starts on a 64-bit word).
// LOAD : buffer words -> lane bit buffer -> 2 fields/cycle -> slot words (2a, 2a+1) of lane g.
//        F10: d1 = decompress10 (or raw), d2 = raw (dual); F12: d1 = value mod q; FC2: d1 = raw.
// STORE: slot words (2a, 2a+1) of X (and Y) -> 2 fields/cycle -> lane bit buffer -> buffer words.
// COMPARE (store with dec = 1): the packed words are compared with the buffer words at the same
//        addresses (prefetched, one per engine) instead of written; any difference sets cmp_fail.
//        F10: compress10(X+Y); FC2: compress_c2(X+Y-M), M = 1665*msgbit(c); F12: X.
// One buffer access per cycle, granted to the lowest engine that can use it.
// NE lane engines (NE = 4, 8 or 16): engine e serves lanes e, e+NE, e+2NE, .. one after another
// (round r: lane e + r*NE); a store therefore produces segments 0..NE-1 first, segment 0 with
// priority, which keeps the pk stream that feeds H(pk) ahead of the hash.
module aigis_stream #(parameter int NE = 16) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          start,
  input  logic          dir,          // 0 load, 1 store
  input  logic [1:0]    fmt,          // 0 F10, 1 F12, 2 FC2
  input  logic          dec,          // load: d1 decompressed
  input  logic          dual,         // load: d2 raw copy
  input  logic [9:0]    base,
  input  logic [1:0]    param,
  input  logic [511:0]  msg,          // message bits (byte j at [8j+7:8j])
  input  logic          xfol,         // store: X words read only once final (xfin) or xdone
  input  logic [127:0]  xfin,
  input  logic          xdone,
  output logic          busy,
  output logic          done,
  output logic          cmp_fail,     // compare job: some packed word differed from the buffer
  // slot read roles X, Y
  output logic [31:0]       re_x, re_y,
  output logic [31:0][5:0]  ra_x, ra_y,
  input  logic [31:0][11:0] rd_x, rd_y,
  // slot write roles D1, D2
  output logic [31:0]       we_1, we_2,
  output logic [31:0][5:0]  wa_1, wa_2,
  output logic [31:0][11:0] wd_1, wd_2,
  // buffer read client
  output logic          br_req,
  output logic [9:0]    br_addr,
  input  logic          br_gnt,
  input  logic          br_rv,
  input  logic [63:0]   br_data,
  // buffer write client
  output logic          bw_req,
  output logic [9:0]    bw_addr,
  output logic [63:0]   bw_data,
  input  logic          bw_gnt
);
  import aigis_pkg::*;
  logic        dir_q, dec_q, dual_q, xfol_q, cmp_q;
  logic [NE-1:0] cwv, cwi;          // compare: buffer word valid / read in flight
  logic [NE-1:0][63:0] cw;
  logic [1:0]  fmt_q, p;
  logic [9:0]  base_q;
  logic [4:0]  F;            // field width
  logic [5:0]  SW;           // segment words
  logic [6:0]  WP;           // W/2 pairs per lane
  always_comb begin
    case (fmt_q)
      2'd0: F = 5'd10;
      2'd1: F = 5'd12;
      default: F = (p == 2'd2) ? 5'd3 : 5'd4;
    endcase
    case (fmt_q)
      2'd0: SW = 6'd5 << (p - 2'd1);
      2'd1: SW = 6'd6 << (p - 2'd1);
      default: SW = (p == 2'd1) ? 6'd2 : (p == 2'd2) ? 6'd3 : 6'd8;
    endcase
    WP = 7'd16 << (p - 2'd1);
  end

  localparam int NRND = 16 / NE;
  localparam int RW = (NRND > 1) ? $clog2(NRND) : 1;
  logic [NE-1:0][127:0] bb;
  logic [NE-1:0][7:0]   fill;
  logic [NE-1:0][6:0]   pa;        // pair index (load: written; store: issued)
  logic [NE-1:0][5:0]   wc;        // buffer words requested (load) / written (store)
  logic [NE-1:0]        infl;      // load: read in flight
  logic [NE-1:0]        s1v;       // store: pair read issued last cycle
  logic [NE-1:0][6:0]   s1a;
  logic [NE-1:0][3:0]   s1l;       // store: lane of that read
  logic [NE-1:0]        s2v;       // store: fields staged
  logic [NE-1:0][23:0]  s2f;       // 2 fields, field 0 in the low bits
  logic [NE-1:0][RW-1:0] rnd;      // round of engine e (lane e + rnd*NE)
  logic [NE-1:0]        fin;       // engine finished all its rounds
  logic [NE-1:0][3:0]   lane;
  always_comb for (int e = 0; e < NE; e++) lane[e] = 4'(e + int'(rnd[e]) * NE);

  // ------------------------------------------------------------------ buffer arbitration
  logic [NE-1:0] lreq;
  logic [3:0]  esel;              // granted engine
  logic        lany;
  logic [NE-1:0] creq, cons;
  always_comb begin
    for (int e = 0; e < NE; e++) begin
      if (!dir_q) lreq[e] = busy && !fin[e] && (wc[e] < SW) && !infl[e] && (fill[e] <= 8'd64);
      else        lreq[e] = busy && !fin[e] && (fill[e] >= 8'd64) && !cmp_q;
      creq[e] = busy && cmp_q && !fin[e] && (wc[e] < SW) && !cwv[e] && !cwi[e];
      cons[e] = busy && cmp_q && !fin[e] && (fill[e] >= 8'd64) && cwv[e];
    end
    esel = '0; lany = 1'b0;
    for (int e = NE - 1; e >= 0; e--) if (lreq[e] || creq[e]) begin esel = 4'(e); lany = 1'b1; end
    br_req  = (!dir_q || cmp_q) && lany;
    br_addr = base_q + 10'(lane[esel]) * 10'(SW) + 10'(wc[esel]);
    bw_req  = dir_q && !cmp_q && lany;
    bw_addr = base_q + 10'(lane[esel]) * 10'(SW) + 10'(wc[esel]);
    bw_data = bb[esel][63:0];
  end
  logic [3:0]  g1l, g2l;          // engine of the read granted one / two cycles ago
  always_ff @(posedge clk) begin g1l <= esel; g2l <= g1l; end

  // ------------------------------------------------------------------ per-lane datapath
  logic [NE-1:0] consume;        // load: two fields leave the bit buffer this cycle
  logic [NE-1:0] issue;          // store: pair read this cycle
  logic [NE-1:0][11:0] f0v, f1v; // load: extracted fields
  always_comb begin
    re_x = '0; re_y = '0; ra_x = '0; ra_y = '0;
    we_1 = '0; we_2 = '0; wa_1 = '0; wa_2 = '0; wd_1 = '0; wd_2 = '0;
    for (int e = 0; e < NE; e++) begin
      f0v[e] = 12'(bb[e] & ((128'd1 << F) - 128'd1));
      f1v[e] = 12'((bb[e] >> F) & ((128'd1 << F) - 128'd1));
      consume[e] = busy && !fin[e] && !dir_q && (pa[e] < WP) && (fill[e] >= {2'd0, F, 1'b0});
      issue[e]   = busy && !fin[e] &&  dir_q && (pa[e] < WP) &&
                   (!xfol_q || xdone || (xfin[{pa[e][5:0], 1'b0}] && xfin[{pa[e][5:0], 1'b1}])) &&
                   (16'(fill[e]) + 16'({F, 1'b0}) * (16'(s1v[e]) + 16'(s2v[e]) + 16'd1) <= 16'd128);
    end
    for (int l = 0; l < 16; l++) begin
      logic [6:0] w0;
      logic       b0, on;
      int         e;
      e  = l % NE;
      on = (int'(rnd[e]) == l / NE);
      w0 = {pa[e][5:0], 1'b0};
      b0 = wbank(w0);
      if (on && consume[e]) begin
        logic [11:0] d1a, d1b;
        case (fmt_q)
          2'd0: begin
            d1a = dec_q ? decompress10(f0v[e][9:0]) : f0v[e];
            d1b = dec_q ? decompress10(f1v[e][9:0]) : f1v[e];
          end
          2'd1: begin
            d1a = (f0v[e] >= 12'd3329) ? f0v[e] - 12'd3329 : f0v[e];
            d1b = (f1v[e] >= 12'd3329) ? f1v[e] - 12'd3329 : f1v[e];
          end
          default: begin d1a = f0v[e]; d1b = f1v[e]; end
        endcase
        we_1[2*l + b0] = 1'b1;  wa_1[2*l + b0] = pa[e][5:0];  wd_1[2*l + b0] = d1a;
        we_1[2*l + !b0] = 1'b1; wa_1[2*l + !b0] = pa[e][5:0]; wd_1[2*l + !b0] = d1b;
        if (dual_q) begin
          we_2[2*l + b0] = 1'b1;  wa_2[2*l + b0] = pa[e][5:0];  wd_2[2*l + b0] = f0v[e];
          we_2[2*l + !b0] = 1'b1; wa_2[2*l + !b0] = pa[e][5:0]; wd_2[2*l + !b0] = f1v[e];
        end
      end
      if (on && issue[e]) begin
        re_x[2*l] = 1'b1; re_x[2*l+1] = 1'b1; ra_x[2*l] = pa[e][5:0]; ra_x[2*l+1] = pa[e][5:0];
        if (fmt_q != 2'd1) begin re_y[2*l] = 1'b1; re_y[2*l+1] = 1'b1; end
        ra_y[2*l] = pa[e][5:0]; ra_y[2*l+1] = pa[e][5:0];
      end
    end
  end

  // store field computation (stage 1 -> stage 2)
  logic [NE-1:0][23:0] sf;
  always_comb begin
    for (int e = 0; e < NE; e++) begin
      logic [6:0]  w0;
      logic        b0;
      logic [11:0] x0, x1, y0, y1, v0, v1;
      logic [10:0] c0, c1;
      logic [8:0]  mi0, mi1;
      logic [3:0]  k0, k1, l;
      logic [31:0][11:0] rx, ry;
      l  = s1l[e];
      rx = rd_x; ry = rd_y;
      w0 = {s1a[e][5:0], 1'b0};
      b0 = wbank(w0);
      x0 = rx[2*l + b0]; x1 = rx[2*l + !b0];
      y0 = ry[2*l + b0]; y1 = ry[2*l + !b0];
      c0 = 11'((11'(l) << (3'd5 + {1'b0, p - 2'd1})) + {4'd0, w0});
      c1 = c0 + 11'd1;
      mi0 = 9'({c0[10:7], 5'd0} + {4'd0, c0[4:0]});
      mi1 = 9'({c1[10:7], 5'd0} + {4'd0, c1[4:0]});
      sf[e] = '0;
      case (fmt_q)
        2'd0: begin v0 = addq(x0, y0); v1 = addq(x1, y1); sf[e] = {4'd0, compress10(v1), compress10(v0)}; end
        2'd1: sf[e] = {x1, x0};
        default: begin
          v0 = subq(addq(x0, y0), msg[mi0] ? 12'd1665 : 12'd0);
          v1 = subq(addq(x1, y1), msg[mi1] ? 12'd1665 : 12'd0);
          k0 = compress_c2(v0, p); k1 = compress_c2(v1, p);
          if (p == 2'd2) sf[e] = {18'd0, k1[2:0], k0[2:0]};
          else           sf[e] = {16'd0, k1, k0};
        end
      endcase
    end
  end

  // ------------------------------------------------------------------ sequential
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy <= 1'b0; done <= 1'b0; fill <= '0; pa <= '0; wc <= '0; infl <= '0; s1v <= '0; s2v <= '0;
      dir_q <= 1'b0; xfol_q <= 1'b0; cmp_q <= 1'b0; cwv <= '0; cwi <= '0; cw <= '0; cmp_fail <= 1'b0; fmt_q <= '0; dec_q <= 1'b0; dual_q <= 1'b0; p <= 2'd1; base_q <= '0; bb <= '0; s1a <= '0; s2f <= '0;
      s1l <= '0; rnd <= '0; fin <= '0;
    end else begin
      done <= 1'b0;
      if (start && !busy) begin
        busy <= 1'b1; xfol_q <= xfol; cmp_q <= dir && dec; cwv <= '0; cwi <= '0; cmp_fail <= 1'b0; dir_q <= dir; fmt_q <= fmt; dec_q <= dec; dual_q <= dual; base_q <= base; p <= param;
        fill <= '0; pa <= '0; wc <= '0; infl <= '0; s1v <= '0; s2v <= '0; bb <= '0; rnd <= '0; fin <= '0;
      end else if (busy) begin
        for (int e = 0; e < NE; e++) begin
          logic [127:0] b_n;
          logic [7:0]   f_n;
          logic         lane_end;
          b_n = bb[e]; f_n = fill[e];
          if (!dir_q) begin
            if (consume[e]) begin b_n = b_n >> {F, 1'b0}; f_n = f_n - {2'd0, F, 1'b0}; pa[e] <= pa[e] + 7'd1; end
            if (br_rv && g2l == 4'(e)) begin b_n = b_n | ({64'd0, br_data} << f_n); f_n = f_n + 8'd64; infl[e] <= 1'b0; end
            if (br_req && br_gnt && esel == 4'(e)) begin infl[e] <= 1'b1; wc[e] <= wc[e] + 6'd1; end
            lane_end = !fin[e] && consume[e] && (pa[e] + 7'd1 == WP);
          end else begin
            s1v[e] <= issue[e];
            if (issue[e]) begin s1a[e] <= pa[e]; s1l[e] <= lane[e]; pa[e] <= pa[e] + 7'd1; end
            s2v[e] <= s1v[e];
            if (s1v[e]) s2f[e] <= sf[e];
            if ((bw_req && bw_gnt && esel == 4'(e)) || cons[e]) begin b_n = b_n >> 64; f_n = f_n - 8'd64; wc[e] <= wc[e] + 6'd1; end
            if (cons[e]) begin cwv[e] <= 1'b0; if (cw[e] != bb[e][63:0]) cmp_fail <= 1'b1; end
            if (br_req && br_gnt && esel == 4'(e)) cwi[e] <= 1'b1;
            if (br_rv && g2l == 4'(e)) begin cw[e] <= br_data; cwv[e] <= 1'b1; cwi[e] <= 1'b0; end
            if (s2v[e]) begin b_n = b_n | ({104'd0, s2f[e]} << f_n); f_n = f_n + {2'd0, F, 1'b0}; end
            lane_end = !fin[e] && (((bw_req && bw_gnt && esel == 4'(e)) || cons[e]) && (wc[e] + 6'd1 == SW));
          end
          bb[e] <= b_n; fill[e] <= f_n;
          if (lane_end) begin
            // next lane of this engine (all its data of the current lane is in / out)
            bb[e] <= '0; fill[e] <= '0; pa[e] <= '0; wc[e] <= '0; infl[e] <= 1'b0;
            if (int'(rnd[e]) == NRND - 1) fin[e] <= 1'b1; else rnd[e] <= rnd[e] + 1'b1;
          end
        end
        if (&fin) begin busy <= 1'b0; done <= 1'b1; end
      end
    end
  end
endmodule
