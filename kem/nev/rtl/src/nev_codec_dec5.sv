// nev_codec_dec5: pipelined decode5 (q = 769), one 6-byte group per cycle, no stall.
//
// Group bytes b0..b5 (in_b[8i+7:8i] = b_i): wl = {b1,b0}, V = {wl[0], b5,b4,b3,b2} (33 bits),
// lo = wl[15:1]. Coefficient t = (digit_t << 3) | lo[3t+2:3t], digit_t = base-97 digit t of V for
// t < 4, digit_4 = floor(V / 97^4) (not reduced, as the reference decode5). Outputs are
// canonicalised by one conditional subtraction of 769 (all decoded values are < 2q).
//
// Each base-97 step takes three stages:
//   a: z = V folded mod 97 to < 4844 (2^12 = 22, 2^24 = 96 mod 97)
//   b: r = z mod 97 by reciprocal multiplication (exact for z < 2^14)
//   c: V' = (V - r) * 97^-1 mod 2^w  (exact division; w = width of floor(V/97))
// Latency: 13 cycles (in_v -> out_v).
module nev_codec_dec5 (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        in_v,
  input  logic [47:0] in_b,
  output logic        out_v,
  output logic [59:0] out_x,   // coefficient t at [12t+11:12t]
  output logic        out_nc,  // some decoded coefficient >= 769 before canonicalisation (non-canonical group)
  output logic        busy     // any stage holds a group
);

  function automatic logic [12:0] fold97(input logic [32:0] v);
    logic [17:0] z1;
    logic [12:0] z2;
    z1 = {6'd0, v[11:0]} + 18'd22 * {6'd0, v[23:12]} + 18'd96 * {9'd0, v[32:24]};
    z2 = {1'b0, z1[11:0]} + 13'd22 * {7'd0, z1[17:12]};
    return z2;
  endfunction

  function automatic logic [6:0] mod97(input logic [12:0] z);
    logic [27:0] p;
    logic [6:0]  qq;
    logic [12:0] r;
    p  = {15'd0, z} * 28'd21621;
    qq = p[27:21];
    r  = z - 13'd97 * {6'd0, qq};
    return r[6:0];
  endfunction

  function automatic logic [11:0] canon769(input logic [9:0] x);
    logic [11:0] e;
    e = {2'b00, x};
    return (e >= 12'd769) ? (e - 12'd769) : e;
  endfunction

  // stage registers
  logic        s0_v;  logic [32:0] s0_V;  logic [14:0] s0_lo;
  // digit 0
  logic        a0_v;  logic [32:0] a0_V;  logic [12:0] a0_z;  logic [14:0] a0_lo;
  logic        b0_v;  logic [26:0] b0_V;  logic [6:0]  b0_r;  logic [14:0] b0_lo;
  logic        c0_v;  logic [26:0] c0_V;  logic [6:0]  c0_d0; logic [14:0] c0_lo;
  // digit 1
  logic        a1_v;  logic [26:0] a1_V;  logic [12:0] a1_z;  logic [6:0] a1_d0; logic [14:0] a1_lo;
  logic        b1_v;  logic [19:0] b1_V;  logic [6:0]  b1_r;  logic [6:0] b1_d0; logic [14:0] b1_lo;
  logic        c1_v;  logic [19:0] c1_V;  logic [13:0] c1_d;  logic [14:0] c1_lo;
  // digit 2
  logic        a2_v;  logic [19:0] a2_V;  logic [12:0] a2_z;  logic [13:0] a2_d; logic [14:0] a2_lo;
  logic        b2_v;  logic [13:0] b2_V;  logic [6:0]  b2_r;  logic [13:0] b2_d; logic [14:0] b2_lo;
  logic        c2_v;  logic [13:0] c2_V;  logic [20:0] c2_d;  logic [14:0] c2_lo;
  // digits 3 and 4
  logic        f1_v;  logic [13:0] f1_V;  logic [6:0]  f1_q;  logic [20:0] f1_d; logic [14:0] f1_lo;

  logic [26:0] c0_n;
  logic [19:0] c1_n;
  logic [13:0] c2_n;
  logic [6:0]  f1_qn;
  logic [59:0] out_n;
  logic        nc_n;

  always_comb begin : dec_comb
    logic [26:0] t27;
    logic [19:0] t20;
    logic [13:0] t14;
    logic [28:0] p29;
    logic [13:0] r3;
    t27   = b0_V - {20'd0, b0_r};
    c0_n  = t27 * 27'h702a3a1;
    t20   = b1_V - {13'd0, b1_r};
    c1_n  = t20 * 20'h2a3a1;
    t14   = b2_V - {7'd0, b2_r};
    c2_n  = t14 * 14'h23a1;
    p29   = {15'd0, c2_V} * 29'd21621;
    f1_qn = p29[27:21];
    r3    = f1_V - 14'd97 * {7'd0, f1_q};
    out_n[11:0]  = canon769({f1_d[6:0],   f1_lo[2:0]});
    out_n[23:12] = canon769({f1_d[13:7],  f1_lo[5:3]});
    out_n[35:24] = canon769({f1_d[20:14], f1_lo[8:6]});
    out_n[47:36] = canon769({r3[6:0],     f1_lo[11:9]});
    out_n[59:48] = canon769({f1_q,        f1_lo[14:12]});
    nc_n = ({f1_d[6:0], f1_lo[2:0]} >= 10'd769) || ({f1_d[13:7], f1_lo[5:3]} >= 10'd769) ||
           ({f1_d[20:14], f1_lo[8:6]} >= 10'd769) || ({r3[6:0], f1_lo[11:9]} >= 10'd769) ||
           ({f1_q, f1_lo[14:12]} >= 10'd769);
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s0_v <= 1'b0; a0_v <= 1'b0; b0_v <= 1'b0; c0_v <= 1'b0;
      a1_v <= 1'b0; b1_v <= 1'b0; c1_v <= 1'b0;
      a2_v <= 1'b0; b2_v <= 1'b0; c2_v <= 1'b0;
      f1_v <= 1'b0; out_v <= 1'b0;
      s0_V <= '0; s0_lo <= '0;
      a0_V <= '0; a0_z <= '0; a0_lo <= '0;
      b0_V <= '0; b0_r <= '0; b0_lo <= '0;
      c0_V <= '0; c0_d0 <= '0; c0_lo <= '0;
      a1_V <= '0; a1_z <= '0; a1_d0 <= '0; a1_lo <= '0;
      b1_V <= '0; b1_r <= '0; b1_d0 <= '0; b1_lo <= '0;
      c1_V <= '0; c1_d <= '0; c1_lo <= '0;
      a2_V <= '0; a2_z <= '0; a2_d <= '0; a2_lo <= '0;
      b2_V <= '0; b2_r <= '0; b2_d <= '0; b2_lo <= '0;
      c2_V <= '0; c2_d <= '0; c2_lo <= '0;
      f1_V <= '0; f1_q <= '0; f1_d <= '0; f1_lo <= '0;
      out_x <= '0; out_nc <= 1'b0;
    end else begin
      // input
      s0_v  <= in_v;
      s0_V  <= {in_b[0], in_b[47:16]};
      s0_lo <= in_b[15:1];
      // digit 0
      a0_v <= s0_v; a0_V <= s0_V; a0_z <= fold97(s0_V); a0_lo <= s0_lo;
      b0_v <= a0_v; b0_V <= a0_V[26:0]; b0_r <= mod97(a0_z); b0_lo <= a0_lo;
      c0_v <= b0_v; c0_V <= c0_n; c0_d0 <= b0_r; c0_lo <= b0_lo;
      // digit 1
      a1_v <= c0_v; a1_V <= c0_V; a1_z <= fold97({6'd0, c0_V}); a1_d0 <= c0_d0; a1_lo <= c0_lo;
      b1_v <= a1_v; b1_V <= a1_V[19:0]; b1_r <= mod97(a1_z); b1_d0 <= a1_d0; b1_lo <= a1_lo;
      c1_v <= b1_v; c1_V <= c1_n; c1_d <= {b1_r, b1_d0}; c1_lo <= b1_lo;
      // digit 2
      a2_v <= c1_v; a2_V <= c1_V; a2_z <= fold97({13'd0, c1_V}); a2_d <= c1_d; a2_lo <= c1_lo;
      b2_v <= a2_v; b2_V <= a2_V[13:0]; b2_r <= mod97(a2_z); b2_d <= a2_d; b2_lo <= a2_lo;
      c2_v <= b2_v; c2_V <= c2_n; c2_d <= {b2_r, b2_d}; c2_lo <= b2_lo;
      // digit 4 quotient (V3 < 9413, quotient <= 97), digit 3 remainder in the output stage
      f1_v <= c2_v; f1_V <= c2_V; f1_q <= f1_qn; f1_d <= c2_d; f1_lo <= c2_lo;
      out_v <= f1_v;
      out_x <= out_n;
      out_nc <= nc_n;
    end
  end

  assign busy = s0_v | a0_v | b0_v | c0_v | a1_v | b1_v | c1_v | a2_v | b2_v | c2_v | f1_v | out_v;

endmodule
