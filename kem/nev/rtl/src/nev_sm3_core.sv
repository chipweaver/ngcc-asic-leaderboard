// nev_sm3_core: one SM3 compression, two rounds per clock.
//
// start (accepted when busy=0): loads the message block and the initial state
//   (use_ext=1: v_ext, use_ext=0: the previous digest held in dig -> chaining).
//   Rounds run on the next 32 edges (two rounds per edge); the final feed-forward
//   XOR is folded into the last edge.  done pulses (and busy is already 0) in the
//   cycle after the last edge, with dig valid.  A new start may be issued in that
//   cycle, so chained blocks of one message take 33 cycles each.
// v_ext must be held stable for the whole compression when use_ext=1.
// dig is only written at the end of a compression; it keeps the last digest
// while a following compression with use_ext=1 runs.
// All vectors use byte order: byte i at bits [8i+7:8i]; SM3 words are big-endian.
module nev_sm3_core (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,
  input  logic         use_ext,
  input  logic [255:0] v_ext,
  input  logic [511:0] blk,
  output logic         busy,
  output logic         done,
  output logic [255:0] dig
);

  function automatic logic [31:0] rotl32(input logic [31:0] x, input int n);
    rotl32 = (x << n) | (x >> (32 - n));
  endfunction
  function automatic logic [31:0] p0f(input logic [31:0] x);
    p0f = x ^ rotl32(x, 9) ^ rotl32(x, 17);
  endfunction
  function automatic logic [31:0] p1f(input logic [31:0] x);
    p1f = x ^ rotl32(x, 15) ^ rotl32(x, 23);
  endfunction
  function automatic logic [31:0] bw(input logic [255:0] v, input int k);
    bw = {v[32*k +: 8], v[32*k+8 +: 8], v[32*k+16 +: 8], v[32*k+24 +: 8]};
  endfunction

  localparam logic [31:0] TJ0 = 32'h79cc4519;
  localparam logic [31:0] TJ1 = 32'h9d8a7a87;   // rotl(0x7a879d8a, 16)

  logic [31:0] st [8];
  logic [31:0] w  [16];
  logic [31:0] tj;          // T_j <<< j for the first round of the pair (second: <<< 1)
  logic [4:0]  rnd;         // round pair index
  logic        use_ext_q;
  logic [31:0] vin [8];
  logic [31:0] dw  [8];
  logic [31:0] hw0, hw1, dw0, dw1;      // H+W and D+W' of the two rounds of this edge (precomputed)
  logic [31:0] ldst [8];
  logic [31:0] ldw  [16];

  always_comb begin
    for (int k = 0; k < 8; k++) begin
      dw[k]  = bw(dig, k);
      vin[k] = use_ext_q ? bw(v_ext, k) : dw[k];
    end
  end

  // two rounds
  logic        lo;          // rounds < 16
  logic [31:0] a12_0, ss1_0, ss2_0, ff_0, gg_0, tt1_0, tt2_0;
  logic [31:0] a12_1, ss1_1, ss2_1, ff_1, gg_1, tt1_1, tt2_1;
  logic [31:0] s1 [8];      // state after the first round
  logic [31:0] nx [8];      // state after the second round
  logic [31:0] tjb;
  logic [31:0] wn0, wn1;
  always_comb begin
    lo  = (rnd < 5'd8);
    tjb = rotl32(tj, 1);
    // round 2k
    a12_0 = rotl32(st[0], 12);
    ss1_0 = rotl32(a12_0 + st[4] + tj, 7);
    ss2_0 = ss1_0 ^ a12_0;
    if (lo) begin
      ff_0 = st[0] ^ st[1] ^ st[2];
      gg_0 = st[4] ^ st[5] ^ st[6];
    end else begin
      ff_0 = (st[0] & st[1]) | (st[0] & st[2]) | (st[1] & st[2]);
      gg_0 = (st[4] & st[5]) | (~st[4] & st[6]);
    end
    tt1_0 = ff_0 + ss2_0 + dw0;
    tt2_0 = gg_0 + ss1_0 + hw0;
    s1[0] = tt1_0;
    s1[1] = st[0];
    s1[2] = rotl32(st[1], 9);
    s1[3] = st[2];
    s1[4] = p0f(tt2_0);
    s1[5] = st[4];
    s1[6] = rotl32(st[5], 19);
    s1[7] = st[6];
    // round 2k+1
    a12_1 = rotl32(s1[0], 12);
    ss1_1 = rotl32(a12_1 + s1[4] + tjb, 7);
    ss2_1 = ss1_1 ^ a12_1;
    if (lo) begin
      ff_1 = s1[0] ^ s1[1] ^ s1[2];
      gg_1 = s1[4] ^ s1[5] ^ s1[6];
    end else begin
      ff_1 = (s1[0] & s1[1]) | (s1[0] & s1[2]) | (s1[1] & s1[2]);
      gg_1 = (s1[4] & s1[5]) | (~s1[4] & s1[6]);
    end
    tt1_1 = ff_1 + ss2_1 + dw1;
    tt2_1 = gg_1 + ss1_1 + hw1;
    nx[0] = tt1_1;
    nx[1] = s1[0];
    nx[2] = rotl32(s1[1], 9);
    nx[3] = s1[2];
    nx[4] = p0f(tt2_1);
    nx[5] = s1[4];
    nx[6] = rotl32(s1[5], 19);
    nx[7] = s1[6];
    wn0 = p1f(w[0] ^ w[7] ^ rotl32(w[13], 15)) ^ rotl32(w[3], 7) ^ w[10];
    wn1 = p1f(w[1] ^ w[8] ^ rotl32(w[14], 15)) ^ rotl32(w[4], 7) ^ w[11];
  end

  always_comb begin
    for (int k = 0; k < 8; k++)  ldst[k] = use_ext ? bw(v_ext, k) : dw[k];
    for (int k = 0; k < 16; k++) ldw[k]  = {blk[32*k +: 8], blk[32*k+8 +: 8], blk[32*k+16 +: 8], blk[32*k+24 +: 8]};
  end

  logic [255:0] dig_nx;
  always_comb begin
    for (int k = 0; k < 8; k++) begin
      dig_nx[32*k +: 8]    = (vin[k][31:24] ^ nx[k][31:24]);
      dig_nx[32*k+8 +: 8]  = (vin[k][23:16] ^ nx[k][23:16]);
      dig_nx[32*k+16 +: 8] = (vin[k][15:8]  ^ nx[k][15:8]);
      dig_nx[32*k+24 +: 8] = (vin[k][7:0]   ^ nx[k][7:0]);
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy      <= 1'b0;
      done      <= 1'b0;
      rnd       <= 5'd0;
      use_ext_q <= 1'b0;
      tj        <= TJ0;
      dig       <= '0;
      hw0 <= '0; hw1 <= '0; dw0 <= '0; dw1 <= '0;
      for (int k = 0; k < 8; k++)  st[k] <= '0;
      for (int k = 0; k < 16; k++) w[k]  <= '0;
    end else begin
      done <= 1'b0;
      if (!busy) begin
        if (start) begin
          busy      <= 1'b1;
          rnd       <= 5'd0;
          use_ext_q <= use_ext;
          tj        <= TJ0;
          for (int k = 0; k < 8; k++)  st[k] <= ldst[k];
          for (int k = 0; k < 16; k++) w[k]  <= ldw[k];
          hw0 <= ldst[7] + ldw[0];
          hw1 <= ldst[6] + ldw[1];
          dw0 <= ldst[3] + (ldw[0] ^ ldw[4]);
          dw1 <= ldst[2] + (ldw[1] ^ ldw[5]);
        end
      end else begin
        for (int k = 0; k < 8; k++) st[k] <= nx[k];
        for (int k = 0; k < 14; k++) w[k] <= w[k+2];
        // next edge: H = rotl(F,19) / rotl(E,19), D = rotl(B,9) / rotl(A,9) of the current state
        hw0 <= rotl32(st[5], 19) + w[2];
        hw1 <= rotl32(st[4], 19) + w[3];
        dw0 <= rotl32(st[1], 9) + (w[2] ^ w[6]);
        dw1 <= rotl32(st[0], 9) + (w[3] ^ w[7]);
        w[14] <= wn0;
        w[15] <= wn1;
        rnd   <= rnd + 5'd1;
        tj    <= (rnd == 5'd7) ? TJ1 : rotl32(tj, 2);
        if (rnd == 5'd31) begin
          busy <= 1'b0;
          done <= 1'b1;
          dig  <= dig_nx;
        end
      end
    end
  end
endmodule
