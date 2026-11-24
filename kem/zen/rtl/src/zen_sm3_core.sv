// SM3 compression core, two rounds per cycle (GB/T 32905-2016).
// start (1 cycle): iv = chaining value (V0 at [255:224]), blk = message block (W0 at [511:480]).
//   32 cycles of two rounds follow; done pulses 33 cycles after start; vout = iv ^ state is valid
//   in the done cycle (and until the next start / chained block overwrites the state).
// Chained continuation (32 cycles per block): in the last round cycle (lastc = 1) the owner may
// assert cont with the next message block on blk. The next block's first round pair then runs in
// the done cycle with A..H = vout (feed-forward XOR at the round inputs); the following done pulses
// 32 cycles later. A new start may still be issued in the done cycle instead (33-cycle path).
// Round restructuring: TT1 = SS2 + (FF + D + W'), TT2 = SS1 + (GG + H + W); the FF/GG sums are added
// in parallel with SS1, so each round has one carry-save level fewer on the A/E path.
module zen_sm3_core (
  input  logic clk, input logic rst_n, input logic start, input logic cont,
  input  logic [255:0] iv, input logic [511:0] blk,
  output logic busy, output logic done, output logic lastc, output logic [255:0] vout);
  logic [31:0] A,B,C,D,E,F,G,H;
  logic [255:0] v_q;
  logic [15:0][31:0] w;
  logic [4:0] pr;
  logic run, fx;    // fx: this cycle is the first round pair of a chained block (inputs = v ^ state)
  function automatic logic [31:0] rotl(input logic [31:0] x, input int n);
    rotl = (x << n) | (x >> (32 - n));
  endfunction
  function automatic logic [31:0] p0f(input logic [31:0] x); p0f = x ^ rotl(x,9) ^ rotl(x,17); endfunction
  function automatic logic [31:0] p1f(input logic [31:0] x); p1f = x ^ rotl(x,15) ^ rotl(x,23); endfunction
  function automatic logic [31:0] tconst(input logic [5:0] j);
    logic [31:0] t; logic [4:0] s;
    t = (j < 6'd16) ? 32'h79CC4519 : 32'h7A879D8A; s = j[4:0];
    tconst = (s == 5'd0) ? t : ((t << s) | (t >> (6'd32 - {1'b0, s})));
  endfunction
  logic [31:0] t0q, t1q, dw0, hw0, dw1, hw1;
  logic lo;
  logic [31:0] Ai,Bi,Ci,Ei,Fi,Gi;
  logic [31:0] a12,ss1,ss2,ff,gg,k1,k2,tt1,tt2, A1,E1, a12b,ss1b,ss2b,ffb,ggb,k1b,k2b,tt1b,tt2b, w16n, w17n;
  logic [31:0] vA,vB,vC,vD,vE,vF,vG,vH;
  assign {vA,vB,vC,vD,vE,vF,vG,vH} = v_q;
  always_comb begin
    Ai = fx ? (A ^ vA) : A;  Bi = fx ? (B ^ vB) : B;  Ci = fx ? (C ^ vC) : C;
    Ei = fx ? (E ^ vE) : E;  Fi = fx ? (F ^ vF) : F;  Gi = fx ? (G ^ vG) : G;
    a12 = rotl(Ai,12);
    ss1 = rotl(a12 + Ei + t0q, 7);
    ss2 = ss1 ^ a12;
    ff = lo ? (Ai^Bi^Ci) : ((Ai&Bi)|(Ai&Ci)|(Bi&Ci));
    gg = lo ? (Ei^Fi^Gi) : ((Ei&Fi)|(~Ei&Gi));
    k1 = ff + dw0;  k2 = gg + hw0;
    tt1 = ss2 + k1;
    tt2 = ss1 + k2;
    A1 = tt1; E1 = p0f(tt2);
    a12b = rotl(A1,12);
    ss1b = rotl(a12b + E1 + t1q, 7);
    ss2b = ss1b ^ a12b;
    ffb = lo ? (A1^Ai^rotl(Bi,9)) : ((A1&Ai)|(A1&rotl(Bi,9))|(Ai&rotl(Bi,9)));
    ggb = lo ? (E1^Ei^rotl(Fi,19)) : ((E1&Ei)|(~E1&rotl(Fi,19)));
    k1b = ffb + dw1; k2b = ggb + hw1;
    tt1b = ss2b + k1b;
    tt2b = ss1b + k2b;
    w16n = p1f(w[0]^w[7]^rotl(w[13],15)) ^ rotl(w[3],7) ^ w[10];
    w17n = p1f(w[1]^w[8]^rotl(w[14],15)) ^ rotl(w[4],7) ^ w[11];
  end
  assign busy  = run;
  assign lastc = run && (pr == 5'd31);
  logic chain;     // continuation accepted in the last round cycle
  assign chain = lastc && cont;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin run<=0; done<=0; pr<=0; fx<=0; end
    else begin
      done<=0; fx<=0;
      if (start) begin run<=1; pr<=0; end
      else if (run) begin
        pr<=pr+1;
        if (pr==5'd31) begin done<=1; run<=chain; fx<=chain; end
      end
    end
  end
  integer k;
  logic [5:0] jn;
  always_comb jn = {pr,1'b0} + 6'd2;
  always_ff @(posedge clk) begin
    if (start) begin
      v_q <= iv; {A,B,C,D,E,F,G,H} <= iv;
      for (k=0;k<16;k=k+1) w[k] <= blk[511-32*k -: 32];
      t0q <= tconst(6'd0); t1q <= tconst(6'd1); lo <= 1'b1;
      dw0 <= iv[159:128] + (blk[511:480]^blk[383:352]);
      hw0 <= iv[31:0] + blk[511:480];
      dw1 <= iv[191:160] + (blk[479:448]^blk[351:320]);
      hw1 <= iv[63:32] + blk[479:448];
    end else if (run) begin
      // state after this round pair
      A<=tt1b; B<=tt1; C<=rotl(Ai,9); D<=rotl(Bi,9);
      E<=p0f(tt2b); F<=E1; G<=rotl(Ei,19); H<=rotl(Fi,19);
      if (fx) v_q <= {vA^A, vB^B, vC^C, vD^D, vE^E, vF^F, vG^G, vH^H};
      if (chain) begin
        // next block: rounds 0/1 run in the done cycle with inputs v' = v ^ state_63
        for (k=0;k<16;k=k+1) w[k] <= blk[511-32*k -: 32];
        t0q <= tconst(6'd0); t1q <= tconst(6'd1); lo <= 1'b1;
        // D' = vD ^ D_63 = vD ^ rotl9(B), H' = vH ^ rotl19(F), C' = vC ^ rotl9(A), G' = vG ^ rotl19(E)
        dw0 <= (vD ^ rotl(B,9))  + (blk[511:480]^blk[383:352]);
        hw0 <= (vH ^ rotl(F,19)) + blk[511:480];
        dw1 <= (vC ^ rotl(A,9))  + (blk[479:448]^blk[351:320]);
        hw1 <= (vG ^ rotl(E,19)) + blk[479:448];
      end else begin
        for (k=0;k<14;k=k+1) w[k] <= w[k+2];
        w[14] <= w16n; w[15] <= w17n;
        t0q <= tconst(jn); t1q <= tconst(jn+6'd1); lo <= (jn < 6'd16);
        dw0 <= rotl(Bi,9) + (w[2]^w[6]);
        hw0 <= rotl(Fi,19) + w[2];
        dw1 <= rotl(Ai,9) + (w[3]^w[7]);
        hw1 <= rotl(Ei,19) + w[3];
      end
    end
  end
  assign vout = v_q ^ {A,B,C,D,E,F,G,H};
endmodule
