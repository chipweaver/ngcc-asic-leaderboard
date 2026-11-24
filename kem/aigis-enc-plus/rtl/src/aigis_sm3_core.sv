// SM3 compression core (GB/T 32905-2016), two rounds per clock cycle.
//
// Interface
//   start : load iv (V0 in [255:224]) and blk (W0 in [511:480]); 32 round-pair cycles follow.
//   done  : one-cycle pulse in the cycle after the last round pair; vout = iv ^ state is valid from
//           that cycle until the next start/cont.
//   lastc : high in the cycle of the last round pair.  If cont is asserted in that cycle the next
//           block on blk is chained with no bubble: its first round pair runs in the done cycle
//           with chaining value vout (32 cycles per chained block).
//   A plain start in the done cycle (iv = vout or any other value) costs 33 cycles per block.
// Timing restructure: TT1 = SS2 + (FF + D + W'_j), TT2 = SS1 + (GG + H + W_j).  D and H of round j
// equal rotl(B,9) / rotl(F,19) two rounds earlier, so (D + W'_j) and (H + W_j) are registered one
// cycle ahead; each round keeps one 3-input add (SS1) and one 2-input add on the A/E path.
module aigis_sm3_core (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,
  input  logic         cont,
  input  logic [255:0] iv,
  input  logic [511:0] blk,
  output logic         busy,
  output logic         done,
  output logic         lastc,
  output logic [255:0] vout
);
  logic [31:0] A, B, C, D, E, F, G, H;
  logic [255:0] v_q;
  logic [15:0][31:0] w;
  logic [4:0] pr;
  logic run, fx, lo;
  logic [31:0] t0q, t1q, dw0, hw0, dw1, hw1;

  function automatic logic [31:0] rotl(input logic [31:0] x, input int n);
    rotl = (n == 0) ? x : ((x << n) | (x >> (32 - n)));
  endfunction
  function automatic logic [31:0] p0f(input logic [31:0] x);
    p0f = x ^ rotl(x, 9) ^ rotl(x, 17);
  endfunction
  function automatic logic [31:0] p1f(input logic [31:0] x);
    p1f = x ^ rotl(x, 15) ^ rotl(x, 23);
  endfunction
  // T_j <<< (j mod 32)
  function automatic logic [31:0] tconst(input logic [5:0] j);
    logic [31:0] t;
    t = (j < 6'd16) ? 32'h79CC4519 : 32'h7A879D8A;
    tconst = (j[4:0] == 5'd0) ? t : ((t << j[4:0]) | (t >> (6'd32 - {1'b0, j[4:0]})));
  endfunction

  logic [31:0] vA, vB, vC, vD, vE, vF, vG, vH;
  assign {vA, vB, vC, vD, vE, vF, vG, vH} = v_q;

  logic [31:0] Ai, Bi, Ci, Ei, Fi, Gi;
  logic [31:0] a12, ss1, ss2, ff, gg, k1, k2, tt1, tt2, A1, E1;
  logic [31:0] a12b, ss1b, ss2b, ffb, ggb, k1b, k2b, tt1b, tt2b, B1, F1;
  logic [31:0] w16n, w17n;
  always_comb begin
    // first round pair of a chained block: inputs are v ^ state (feed-forward folded in)
    Ai = fx ? (A ^ vA) : A;  Bi = fx ? (B ^ vB) : B;  Ci = fx ? (C ^ vC) : C;
    Ei = fx ? (E ^ vE) : E;  Fi = fx ? (F ^ vF) : F;  Gi = fx ? (G ^ vG) : G;
    // round j
    a12 = rotl(Ai, 12);
    ss1 = rotl(a12 + Ei + t0q, 7);
    ss2 = ss1 ^ a12;
    ff  = lo ? (Ai ^ Bi ^ Ci) : ((Ai & Bi) | (Ai & Ci) | (Bi & Ci));
    gg  = lo ? (Ei ^ Fi ^ Gi) : ((Ei & Fi) | (~Ei & Gi));
    k1  = ff + dw0;
    k2  = gg + hw0;
    tt1 = ss2 + k1;
    tt2 = ss1 + k2;
    A1  = tt1;
    E1  = p0f(tt2);
    B1  = rotl(Bi, 9);    // C after round j
    F1  = rotl(Fi, 19);   // G after round j
    // round j+1: (A1, Ai, B1) as (A, B, C); (E1, Ei, F1) as (E, F, G)
    a12b = rotl(A1, 12);
    ss1b = rotl(a12b + E1 + t1q, 7);
    ss2b = ss1b ^ a12b;
    ffb  = lo ? (A1 ^ Ai ^ B1) : ((A1 & Ai) | (A1 & B1) | (Ai & B1));
    ggb  = lo ? (E1 ^ Ei ^ F1) : ((E1 & Ei) | (~E1 & F1));
    k1b  = ffb + dw1;
    k2b  = ggb + hw1;
    tt1b = ss2b + k1b;
    tt2b = ss1b + k2b;
    // message expansion for the next pair
    w16n = p1f(w[0] ^ w[7] ^ rotl(w[13], 15)) ^ rotl(w[3], 7) ^ w[10];
    w17n = p1f(w[1] ^ w[8] ^ rotl(w[14], 15)) ^ rotl(w[4], 7) ^ w[11];
  end

  assign busy  = run;
  assign lastc = run && (pr == 5'd31);
  logic chain;
  assign chain = lastc && cont;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run <= 1'b0; done <= 1'b0; pr <= '0; fx <= 1'b0;
    end else begin
      done <= 1'b0;
      fx   <= 1'b0;
      if (start) begin
        run <= 1'b1; pr <= '0;
      end else if (run) begin
        pr <= pr + 5'd1;
        if (pr == 5'd31) begin
          done <= 1'b1; run <= chain; fx <= chain;
        end
      end
    end
  end

  logic [5:0] jn;
  assign jn = {pr, 1'b0} + 6'd2;   // first round index of the next pair
  integer k;
  always_ff @(posedge clk) begin
    if (start) begin
      v_q <= iv;
      {A, B, C, D, E, F, G, H} <= iv;
      for (k = 0; k < 16; k = k + 1) w[k] <= blk[511 - 32*k -: 32];
      t0q <= tconst(6'd0); t1q <= tconst(6'd1); lo <= 1'b1;
      dw0 <= iv[159:128] + (blk[511:480] ^ blk[383:352]);   // D + W'_0
      hw0 <= iv[31:0]    + blk[511:480];                    // H + W_0
      dw1 <= iv[191:160] + (blk[479:448] ^ blk[351:320]);   // C (= D of round 1) + W'_1
      hw1 <= iv[63:32]   + blk[479:448];                    // G (= H of round 1) + W_1
    end else if (run) begin
      A <= tt1b; B <= A1; C <= rotl(Ai, 9); D <= B1;
      E <= p0f(tt2b); F <= E1; G <= rotl(Ei, 19); H <= F1;
      if (fx) v_q <= {vA ^ A, vB ^ B, vC ^ C, vD ^ D, vE ^ E, vF ^ F, vG ^ G, vH ^ H};
      if (chain) begin
        for (k = 0; k < 16; k = k + 1) w[k] <= blk[511 - 32*k -: 32];
        t0q <= tconst(6'd0); t1q <= tconst(6'd1); lo <= 1'b1;
        // next block's round 0/1 inputs are v' = v ^ state_64 (state after this pair):
        // D' = vD ^ B1, H' = vH ^ F1, C' = vC ^ rotl(Ai,9), G' = vG ^ rotl(Ei,19)
        dw0 <= (vD ^ B1) + (blk[511:480] ^ blk[383:352]);
        hw0 <= (vH ^ F1) + blk[511:480];
        dw1 <= (vC ^ rotl(Ai, 9))  + (blk[479:448] ^ blk[351:320]);
        hw1 <= (vG ^ rotl(Ei, 19)) + blk[479:448];
      end else begin
        for (k = 0; k < 14; k = k + 1) w[k] <= w[k+2];
        w[14] <= w16n; w[15] <= w17n;
        t0q <= tconst(jn); t1q <= tconst(jn + 6'd1); lo <= (jn < 6'd16);
        // D, H of rounds j+2, j+3 = rotl(B,9)/rotl(F,19) after round j+1 / round j
        dw0 <= B1 + (w[2] ^ w[6]);              // D_{j+2} = C_{j+1} = rotl(B_j... see note
        hw0 <= F1 + w[2];
        dw1 <= rotl(Ai, 9) + (w[3] ^ w[7]);
        hw1 <= rotl(Ei, 19) + w[3];
      end
    end
  end
  assign vout = v_q ^ {A, B, C, D, E, F, G, H};
endmodule
