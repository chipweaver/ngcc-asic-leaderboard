// SM3 compression function (GB/T 32905), two rounds per cycle.
// start (when !busy) loads V and the 512-bit block (blk[511:480] = W0, big-endian words).
// 32 step edges later done pulses for one cycle and vout = V xor ABCDEFGH is held until the
// next start (combinational from the held state registers). A new start may be applied in the
// done cycle (33 cycles per compression).
// Timing: the D + W'_j and H + W_j terms of the first round of a step are added one step ahead
// (D and H of the next step are rotl(B,9) and rotl(F,19) of the current one); those of the
// second round use C and G of the current step and are added beside the first round.
module sm3_core2 (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,
  input  logic [255:0] vin,     // vin[255:224] = V0
  input  logic [511:0] blk,
  input  logic         abort,
  output logic         busy,
  output logic         done,
  output logic [255:0] vout
);
  logic [31:0] A, B, C, D, E, F, G, H;
  logic [31:0] W [0:15];
  logic [255:0] V;
  logic [4:0]  st;               // step: rounds 2st, 2st+1
  logic [31:0] t0, t1;           // T_j <<< j for the two rounds of the step
  logic [31:0] k1, k2;           // D + W'_j, H + W_j of the first round

  function automatic [31:0] rl(input [31:0] x, input integer n);
    rl = (x << n) | (x >> (32 - n));
  endfunction
  // three-operand modular add through one 3:2 carry-save level and a single carry-propagate add
  function automatic [31:0] add3(input [31:0] x, input [31:0] y, input [31:0] z);
    logic [31:0] sv, cv;
    sv = x ^ y ^ z;
    cv = ((x & y) | (x & z) | (y & z)) << 1;
    add3 = sv + cv;
  endfunction
  function automatic [31:0] p0(input [31:0] x); p0 = x ^ rl(x, 9) ^ rl(x, 17); endfunction
  function automatic [31:0] p1(input [31:0] x); p1 = x ^ rl(x, 15) ^ rl(x, 23); endfunction
  function automatic [31:0] tj_of(input [5:0] r);
    logic [31:0] t;
    logic [4:0]  s;
    t = (r < 6'd16) ? 32'h79CC4519 : 32'h7A879D8A;
    s = r[4:0];
    tj_of = (s == 5'd0) ? t : ((t << s) | (t >> (6'd32 - {1'b0, s})));
  endfunction

  logic        lo;               // rounds < 16 (both rounds of the step)
  assign lo = (st < 5'd8);
  logic [31:0] a12, ss1, ss2, ff, gg, tt1, tt2, e1;
  logic [31:0] b12, ss1b, ss2b, ffb, ggb, tt1b, tt2b, e2;
  logic [31:0] k1b, k2b, w16, w17;
  always_comb begin
    // ---- round 2st
    a12 = rl(A, 12);
    ss1 = rl(add3(a12, E, t0), 7);
    ss2 = ss1 ^ a12;
    ff  = lo ? (A ^ B ^ C) : ((A & B) | (A & C) | (B & C));
    gg  = lo ? (E ^ F ^ G) : ((E & F) | (~E & G));
    tt1 = add3(ff, ss2, k1);
    tt2 = add3(gg, ss1, k2);
    e1  = p0(tt2);
    // ---- round 2st+1: A1 = tt1, B1 = A, C1 = rotl(B,9), D1 = C, E1 = e1, F1 = E, G1 = rotl(F,19), H1 = G
    k1b  = C + (W[1] ^ W[5]);
    k2b  = G + W[1];
    b12  = rl(tt1, 12);
    ss1b = rl(add3(b12, e1, t1), 7);
    ss2b = ss1b ^ b12;
    ffb  = lo ? (tt1 ^ A ^ rl(B, 9)) : ((tt1 & A) | (tt1 & rl(B, 9)) | (A & rl(B, 9)));
    ggb  = lo ? (e1 ^ E ^ rl(F, 19)) : ((e1 & E) | (~e1 & rl(F, 19)));
    tt1b = add3(ffb, ss2b, k1b);
    tt2b = add3(ggb, ss1b, k2b);
    e2   = p0(tt2b);
    // ---- message expansion: W16, W17 from the current window
    w16 = p1(W[0] ^ W[7]  ^ rl(W[13], 15)) ^ rl(W[3], 7) ^ W[10];
    w17 = p1(W[1] ^ W[8]  ^ rl(W[14], 15)) ^ rl(W[4], 7) ^ W[11];
  end

  // the working registers hold the final state from the done cycle until the next start
  assign vout = V ^ {A, B, C, D, E, F, G, H};

  integer i;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy <= 1'b0; done <= 1'b0; st <= '0; t0 <= '0; t1 <= '0; k1 <= '0; k2 <= '0;
      A <= '0; B <= '0; C <= '0; D <= '0; E <= '0; F <= '0; G <= '0; H <= '0;
      V <= '0;
      for (i = 0; i < 16; i = i + 1) W[i] <= '0;
    end else begin
      done <= 1'b0;
      if (abort) begin
        busy <= 1'b0;
      end else if (start && !busy) begin
        busy <= 1'b1; st <= '0; t0 <= tj_of(6'd0); t1 <= tj_of(6'd1);
        V <= vin;
        {A, B, C, D, E, F, G, H} <= vin;
        for (i = 0; i < 16; i = i + 1) W[i] <= blk[511 - 32*i -: 32];
        k1 <= vin[159:128] + (blk[511:480] ^ blk[383:352]);   // D + (W0 ^ W4)
        k2 <= vin[31:0] + blk[511:480];                       // H + W0
      end else if (busy) begin
        A <= tt1b; B <= tt1; C <= rl(A, 9); D <= rl(B, 9);
        E <= e2;   F <= e1;  G <= rl(E, 19); H <= rl(F, 19);
        for (i = 0; i < 14; i = i + 1) W[i] <= W[i+2];
        W[14] <= w16; W[15] <= w17;
        // next step's first round: D' = rotl(B,9), H' = rotl(F,19), W_{j+2} = W[2], W'_{j+2} = W[2]^W[6]
        k1 <= rl(B, 9) + (W[2] ^ W[6]);
        k2 <= rl(F, 19) + W[2];
        st <= st + 5'd1;
        t0 <= tj_of({st + 5'd1, 1'b0});
        t1 <= tj_of({st + 5'd1, 1'b1});
        if (st == 5'd31) begin
          busy <= 1'b0; done <= 1'b1;
        end
      end
    end
  end
endmodule
