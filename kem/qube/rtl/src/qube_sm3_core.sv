// SM3 compression function, two rounds per cycle.
// start (with busy = 0) loads iv and blk; done is a one-cycle pulse 32 cycles after the start edge
// with dout = compress(iv, blk). blk byte 0 is blk[511:504] (big-endian message words); iv/dout word
// A is [255:224]. D+W'_j and H+W_j of the first round of a pair are pre-added one cycle ahead; the
// second round's D+W' and H+W come from C and G of the cycle start (adders off the A/E path).
// abort returns the core to idle without a done pulse.
module qube_sm3_core (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,
  input  logic         abort,
  input  logic [255:0] iv,
  input  logic [511:0] blk,
  output logic         busy,
  output logic         done,
  output logic [255:0] dout
);
  function automatic logic [31:0] rl(input logic [31:0] x, input int n);
    rl = (x << n) | (x >> (32 - n));
  endfunction
  function automatic logic [31:0] p0f(input logic [31:0] x);
    p0f = x ^ rl(x, 9) ^ rl(x, 17);
  endfunction
  function automatic logic [31:0] p1f(input logic [31:0] x);
    p1f = x ^ rl(x, 15) ^ rl(x, 23);
  endfunction
  // a + b + c through one carry-save level and one carry-propagate adder
  function automatic logic [31:0] add3(input logic [31:0] a, input logic [31:0] b, input logic [31:0] c);
    logic [31:0] sv, cv;
    sv = a ^ b ^ c;
    cv = ((a & b) | (a & c) | (b & c)) << 1;
    add3 = sv + cv;
  endfunction
  function automatic logic [31:0] wexp(input logic [31:0] w0, input logic [31:0] w3,
                                       input logic [31:0] w7, input logic [31:0] w10,
                                       input logic [31:0] w13);
    wexp = p1f(w0 ^ w7 ^ rl(w13, 15)) ^ rl(w3, 7) ^ w10;
  endfunction

  logic [31:0] ra, rb, rc, re, rf, rg;      // working registers (D and H only via the pre-adds)
  logic [31:0] dw, hw;                       // D + W'_j, H + W_j for the first round of the pair
  logic [31:0] tj;                           // rotl(T_j, j)
  logic [31:0] w [0:15];                     // W_j .. W_{j+15}
  logic [255:0] v;                           // chaining input
  logic [4:0]  jp;                           // pair index (j = 2 jp)
  logic        run, fin;

  // round j (first of the pair)
  logic        lo;
  logic [31:0] a12, ss1, ss2, ff, gg, tt1, tt2, a1, e1;
  // round j+1
  logic [31:0] dw1, hw1, tj1;
  logic [31:0] a12b, ss1b, ss2b, ffb, ggb, tt1b, tt2b;
  logic [31:0] wn0, wn1;
  always_comb begin
    lo   = (jp < 5'd8);
    a12  = rl(ra, 12);
    ss1  = rl(add3(a12, re, tj), 7);
    ss2  = ss1 ^ a12;
    ff   = lo ? (ra ^ rb ^ rc) : ((ra & rb) | (ra & rc) | (rb & rc));
    gg   = lo ? (re ^ rf ^ rg) : (((rf ^ rg) & re) ^ rg);
    tt1  = (ff + dw) + ss2;           // late operand last: one adder after SS1
    tt2  = (gg + hw) + ss1;
    a1   = tt1;
    e1   = p0f(tt2);
    // round j+1: A=a1 B=ra C=rl(rb,9) D=rc E=e1 F=re G=rl(rf,19) H=rg
    dw1  = rc + (w[1] ^ w[5]);
    hw1  = rg + w[1];
    tj1  = rl(tj, 1);
    a12b = rl(a1, 12);
    ss1b = rl(add3(a12b, e1, tj1), 7);
    ss2b = ss1b ^ a12b;
    ffb  = lo ? (a1 ^ ra ^ rl(rb, 9)) : ((a1 & ra) | (a1 & rl(rb, 9)) | (ra & rl(rb, 9)));
    ggb  = lo ? (e1 ^ re ^ rl(rf, 19)) : (((re ^ rl(rf, 19)) & e1) ^ rl(rf, 19));
    tt1b = (ffb + dw1) + ss2b;
    tt2b = (ggb + hw1) + ss1b;
    wn0  = wexp(w[0], w[3], w[7], w[10], w[13]);
    wn1  = wexp(w[1], w[4], w[8], w[11], w[14]);
  end

  // load values
  logic [31:0] bw [0:15];
  always_comb for (int i = 0; i < 16; i++) bw[i] = blk[511 - 32 * i -: 32];

  logic [31:0] fd, fh;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run <= 1'b0; fin <= 1'b0; jp <= 5'd0;
      ra <= '0; rb <= '0; rc <= '0; re <= '0; rf <= '0; rg <= '0;
      dw <= '0; hw <= '0; tj <= '0; v <= '0; fd <= '0; fh <= '0;
      for (int i = 0; i < 16; i++) w[i] <= '0;
    end else begin
      fin <= 1'b0;
      if (abort) begin
        run <= 1'b0;
      end else if (start && !run) begin
        run <= 1'b1; jp <= 5'd0; v <= iv;
        ra <= iv[255:224]; rb <= iv[223:192]; rc <= iv[191:160];
        re <= iv[127:96];  rf <= iv[95:64];   rg <= iv[63:32];
        dw <= iv[159:128] + (bw[0] ^ bw[4]);
        hw <= iv[31:0] + bw[0];
        tj <= 32'h79CC4519;
        for (int i = 0; i < 16; i++) w[i] <= bw[i];
      end else if (run) begin
        ra <= tt1b; rb <= a1; rc <= rl(ra, 9);
        re <= p0f(tt2b); rf <= e1; rg <= rl(re, 19);
        dw <= rl(rb, 9) + (w[2] ^ w[6]);     // D_{j+2} = C_{j+1} = rotl(B_j, 9)
        hw <= rl(rf, 19) + w[2];             // H_{j+2} = G_{j+1} = rotl(F_j, 19)
        tj <= (jp == 5'd7) ? 32'h9D8A7A87 : rl(tj, 2);
        fd <= rl(rb, 9); fh <= rl(rf, 19);
        for (int i = 0; i < 14; i++) w[i] <= w[i + 2];
        w[14] <= wn0; w[15] <= wn1;
        jp <= jp + 5'd1;
        if (jp == 5'd31) begin run <= 1'b0; fin <= 1'b1; end
      end
    end
  end

  assign busy = run;
  assign done = fin;
  assign dout = v ^ {ra, rb, rc, fd, re, rf, rg, fh};
endmodule
