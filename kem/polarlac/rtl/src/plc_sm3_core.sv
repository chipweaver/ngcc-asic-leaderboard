// SM3 compression function core (GB/T 32905-2016), one round per clock.
// start (when !busy) loads cv_in/blk_in; done pulses 66 cycles after start with
// cv_out = cv_in ^ final(A..H). cv_out holds until the next start.
module plc_sm3_core (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,
  input  logic [255:0] cv_in,    // {V0,...,V7}, V0 in [255:224]
  input  logic [511:0] blk_in,   // {W0,...,W15}, W0 in [511:480] (big-endian bytes)
  output logic         busy,
  output logic         done,
  output logic [255:0] cv_out
);
  function automatic logic [31:0] rotl(input logic [31:0] x, input int n);
    rotl = (x << n) | (x >> (32 - n));
  endfunction
  function automatic logic [31:0] p0(input logic [31:0] x);
    p0 = x ^ rotl(x, 9) ^ rotl(x, 17);
  endfunction
  function automatic logic [31:0] p1(input logic [31:0] x);
    p1 = x ^ rotl(x, 15) ^ rotl(x, 23);
  endfunction
  // 3:2 carry-save compress then one carry-propagate add
  function automatic logic [31:0] add3(input logic [31:0] x, input logic [31:0] y, input logic [31:0] z);
    logic [31:0] s, cy;
    s  = x ^ y ^ z;
    cy = ((x & y) | (x & z) | (y & z)) << 1;
    add3 = s + cy;
  endfunction

  logic [31:0] a, b, c, d, e, f, g, h;
  logic [31:0] w [0:15];
  logic [31:0] tr;        // T_j rotated left by j
  logic [31:0] hw, dw;    // precomputed H+W[j], D+W'[j]
  logic [5:0]  rnd;
  logic        fin;
  logic [255:0] cv;

  // round j combinational
  logic [31:0] a12, ss1, ss2, ff, gg, tt1, tt2, wnew, a_n_tr;
  always_comb begin
    a12 = rotl(a, 12);
    ss1 = rotl(add3(a12, e, tr), 7);
    ss2 = ss1 ^ a12;
    if (rnd < 6'd16) begin
      ff = a ^ b ^ c;
      gg = e ^ f ^ g;
    end else begin
      ff = (a & b) | (a & c) | (b & c);
      gg = ((f ^ g) & e) ^ g;
    end
    tt1 = add3(ff, ss2, dw);
    tt2 = add3(gg, ss1, hw);
    wnew = p1(w[0] ^ w[7] ^ rotl(w[13], 15)) ^ rotl(w[3], 7) ^ w[10];
    a_n_tr = (rnd == 6'd15) ? 32'h9d8a7a87 : rotl(tr, 1); // rotl(7a879d8a,16) for j=16
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy <= 1'b0;
      done <= 1'b0;
      fin  <= 1'b0;
      rnd  <= '0;
    end else begin
      done <= fin;
      fin  <= 1'b0;
      if (start && !busy) begin
        busy <= 1'b1;
        rnd  <= '0;
      end else if (busy && !fin) begin
        rnd <= rnd + 6'd1;
        if (rnd == 6'd63) fin <= 1'b1;
      end else if (fin) begin
        busy <= 1'b0;
      end
    end
  end

  always_ff @(posedge clk) begin
    if (start && !busy) begin
      cv <= cv_in;
      {a, b, c, d, e, f, g, h} <= cv_in;
      for (int i = 0; i < 16; i++) w[i] <= blk_in[511 - 32*i -: 32];
      tr <= 32'h79cc4519;
      hw <= cv_in[31:0]    + blk_in[511 -: 32];
      dw <= cv_in[159:128] + (blk_in[511 -: 32] ^ blk_in[511 - 128 -: 32]);
    end else if (busy && !fin) begin
      d <= c;
      c <= rotl(b, 9);
      b <= a;
      a <= tt1;
      h <= g;
      g <= rotl(f, 19);
      f <= e;
      e <= p0(tt2);
      for (int i = 0; i < 15; i++) w[i] <= w[i+1];
      w[15] <= wnew;
      tr <= a_n_tr;
      // next round: H' = G, D' = C, W[j+1] = w[1], W'[j+1] = w[1]^w[5]
      hw <= g + w[1];
      dw <= c + (w[1] ^ w[5]);
    end else if (fin) begin
      cv_out <= cv ^ {a, b, c, d, e, f, g, h};
    end
  end
endmodule
