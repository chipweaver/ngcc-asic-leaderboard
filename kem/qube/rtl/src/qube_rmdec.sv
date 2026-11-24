// Reed-Muller RM(1,7) duplicated decoder (reference reed_muller_decode, model qube.rm_decode_symbol).
// Input: 512-bit words of the received vector em (4 chunks of 128 bits per word); symbol i uses
// chunks i*mult .. i*mult+mult-1; exactly ceil(N1*mult/4) words are consumed, chunks after the last
// symbol are ignored. Pipeline per symbol:
//   A: bit counts over the mult copies, one chunk per cycle (cnt[b] = number of ones at bit b).
//   B: fast Hadamard transform, one 64-butterfly pass per cycle (7 passes, the first one taken
//      directly from the counts); the last pass subtracts 64*mult from element 0.
//   T1/T2/T3: peak search over |value| as a balanced tree (lower index wins ties, equal to the
//      reference strict > scan from 0), 128 -> 32 -> 4 -> 1, then +128 if the peak value is > 0.
// Steady state is one symbol per 7 cycles; symbols are written in order on sym_we/sym_idx/sym_data.
module qube_rmdec
  import qube_pkg::*;
(
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,
  input  logic [2:0]   ps,
  input  logic         in_valid,
  output logic         in_ready,
  input  logic [511:0] in_data,
  output logic         busy,
  output logic         done,
  output logic         sym_we,
  output logic [7:0]   sym_idx,
  output logic [7:0]   sym_data
);
  logic         run;
  logic [2:0]   mult_r;
  logic [7:0]   n1_r;
  logic [8:0]   nw_r, wrcv;

  // word buffer
  logic [511:0] wbuf;
  logic         wv;
  logic [1:0]   cp;

  // stage A
  logic [2:0]   cnt [0:127];
  logic [2:0]   acnt;
  logic         afull;
  logic [7:0]   asym;

  // stage B
  logic signed [10:0] h [0:127];
  logic         bact;
  logic [2:0]   bph;           // passes done (7 = final values in h)

  // peak tree
  logic [9:0]   a1 [0:31];
  logic         s1 [0:31];
  logic [1:0]   i1 [0:31];
  logic         t1v;
  logic [9:0]   a2 [0:3];
  logic         s2 [0:3];
  logic [4:0]   i2 [0:3];
  logic         t2v;
  logic [7:0]   osym;

  // ---------------- control
  logic         bload, take, take_last_sym, last_chunk;
  logic [127:0] chunk;
  always_comb begin
    bload = afull && (!bact || bph == 3'd7);
    take  = run && wv && (asym != n1_r) && (!afull || bload);
    take_last_sym = (acnt == mult_r - 3'd1);
    last_chunk = take_last_sym && (asym == n1_r - 8'd1);
    chunk = wbuf[128*cp +: 128];
  end
  assign in_ready = run && (wrcv != nw_r) && (!wv || (take && cp == 2'd3));
  assign busy = run;

  // ---------------- Hadamard pass
  logic signed [10:0] src [0:127];
  logic signed [10:0] hn  [0:127];
  always_comb begin
    for (int i = 0; i < 128; i++) src[i] = bload ? $signed({8'd0, cnt[i]}) : h[i];
    for (int i = 0; i < 64; i++) begin
      hn[i]      = src[2*i] + src[2*i+1];
      hn[i + 64] = src[2*i] - src[2*i+1];
    end
    if (!bload && bph == 3'd6) hn[0] = src[0] + src[1] - $signed({2'd0, mult_r, 6'd0});
  end

  // ---------------- peak tree level 1: |h| and 128 -> 32
  logic [9:0] ab [0:127];
  logic       sb [0:127];
  logic [9:0] a1n [0:31];
  logic       s1n [0:31];
  logic [1:0] i1n [0:31];
  logic [10:0] ht [0:127];
  logic [9:0]  xa1 [0:31], ya1 [0:31];
  logic        xs1 [0:31], ys1 [0:31], xi1 [0:31], yi1 [0:31];
  always_comb begin
    for (int i = 0; i < 128; i++) begin
      ht[i] = h[i][10] ? 11'(-h[i]) : h[i];
      ab[i] = ht[i][9:0];
      sb[i] = !h[i][10] && (h[i] != 11'sd0);
    end
    for (int g = 0; g < 32; g++) begin
      xi1[g] = (ab[4*g+1] > ab[4*g]);
      xa1[g] = xi1[g] ? ab[4*g+1] : ab[4*g];
      xs1[g] = xi1[g] ? sb[4*g+1] : sb[4*g];
      yi1[g] = (ab[4*g+3] > ab[4*g+2]);
      ya1[g] = yi1[g] ? ab[4*g+3] : ab[4*g+2];
      ys1[g] = yi1[g] ? sb[4*g+3] : sb[4*g+2];
      if (ya1[g] > xa1[g]) begin a1n[g] = ya1[g]; s1n[g] = ys1[g]; i1n[g] = {1'b1, yi1[g]}; end
      else                 begin a1n[g] = xa1[g]; s1n[g] = xs1[g]; i1n[g] = {1'b0, xi1[g]}; end
    end
  end

  // ---------------- level 2: 32 -> 4 (groups of 8)
  logic [9:0] a2n [0:3];
  logic       s2n [0:3];
  logic [4:0] i2n [0:3];
  logic [9:0] la [0:3][0:7];
  logic       ls [0:3][0:7];
  logic [2:0] li [0:3][0:7];
  always_comb begin
    for (int q = 0; q < 4; q++) begin
      for (int m = 0; m < 8; m++) begin la[q][m] = a1[8*q+m]; ls[q][m] = s1[8*q+m]; li[q][m] = 3'(m); end
      for (int lv = 0; lv < 3; lv++)
        for (int m = 0; m < (4 >> lv); m++)
          if (la[q][2*m+1] > la[q][2*m]) begin
            la[q][m] = la[q][2*m+1]; ls[q][m] = ls[q][2*m+1]; li[q][m] = li[q][2*m+1];
          end else begin
            la[q][m] = la[q][2*m];   ls[q][m] = ls[q][2*m];   li[q][m] = li[q][2*m];
          end
      a2n[q] = la[q][0]; s2n[q] = ls[q][0]; i2n[q] = {li[q][0], i1[8*q + li[q][0]]};
    end
  end

  // ---------------- level 3: 4 -> 1
  logic [7:0] symn;
  logic       x3, y3, z3;
  logic [9:0] xa3, ya3;
  always_comb begin
    x3  = (a2[1] > a2[0]);
    xa3 = x3 ? a2[1] : a2[0];
    y3  = (a2[3] > a2[2]);
    ya3 = y3 ? a2[3] : a2[2];
    z3  = (ya3 > xa3);
    case ({z3, z3 ? y3 : x3})
      2'b00:   symn = {s2[0], 2'd0, i2[0]};
      2'b01:   symn = {s2[1], 2'd1, i2[1]};
      2'b10:   symn = {s2[2], 2'd2, i2[2]};
      default: symn = {s2[3], 2'd3, i2[3]};
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run <= 1'b0; done <= 1'b0; mult_r <= '0; n1_r <= '0; nw_r <= '0; wrcv <= '0;
      wbuf <= '0; wv <= 1'b0; cp <= '0; acnt <= '0; afull <= 1'b0; asym <= '0;
      bact <= 1'b0; bph <= '0; t1v <= 1'b0; t2v <= 1'b0; osym <= '0;
      sym_we <= 1'b0; sym_idx <= '0; sym_data <= '0;
      for (int i = 0; i < 128; i++) begin cnt[i] <= '0; h[i] <= '0; end
      for (int g = 0; g < 32; g++) begin a1[g] <= '0; s1[g] <= 1'b0; i1[g] <= '0; end
      for (int q = 0; q < 4; q++) begin a2[q] <= '0; s2[q] <= 1'b0; i2[q] <= '0; end
    end else begin
      done   <= 1'b0;
      sym_we <= 1'b0;
      if (start) begin
        run <= 1'b1; mult_r <= p_mult(ps); n1_r <= p_n1(ps); nw_r <= p_nwv(ps); wrcv <= '0;
        wv <= 1'b0; cp <= '0; acnt <= '0; afull <= 1'b0; asym <= '0;
        bact <= 1'b0; bph <= '0; t1v <= 1'b0; t2v <= 1'b0; osym <= '0;
      end else if (run) begin
        // ---- word buffer
        if (take) begin
          cp <= cp + 2'd1;
          if (cp == 2'd3 || last_chunk) wv <= 1'b0;
        end
        if (in_valid && in_ready) begin
          wbuf <= in_data; wv <= 1'b1; cp <= '0; wrcv <= wrcv + 9'd1;
        end
        // ---- stage A
        if (take) begin
          for (int b = 0; b < 128; b++) cnt[b] <= ((acnt == 3'd0) ? 3'd0 : cnt[b]) + {2'd0, chunk[b]};
          if (take_last_sym) begin acnt <= '0; asym <= asym + 8'd1; end
          else acnt <= acnt + 3'd1;
        end
        if (take && take_last_sym) afull <= 1'b1;
        else if (bload) afull <= 1'b0;
        // ---- stage B
        if (bload) begin
          for (int i = 0; i < 128; i++) h[i] <= hn[i];
          bact <= 1'b1; bph <= 3'd1;
        end else if (bact && bph != 3'd7) begin
          for (int i = 0; i < 128; i++) h[i] <= hn[i];
          bph <= bph + 3'd1;
        end else if (bact) begin
          bact <= 1'b0;
        end
        // ---- peak tree
        t1v <= bact && (bph == 3'd7);
        if (bact && bph == 3'd7)
          for (int g = 0; g < 32; g++) begin a1[g] <= a1n[g]; s1[g] <= s1n[g]; i1[g] <= i1n[g]; end
        t2v <= t1v;
        if (t1v)
          for (int q = 0; q < 4; q++) begin a2[q] <= a2n[q]; s2[q] <= s2n[q]; i2[q] <= i2n[q]; end
        if (t2v) begin
          sym_we <= 1'b1; sym_idx <= osym; sym_data <= symn; osym <= osym + 8'd1;
        end
        if (sym_we && sym_idx == n1_r - 8'd1) begin run <= 1'b0; done <= 1'b1; end
      end
    end
  end
endmodule
