// Two-lane output-stationary sparse x dense multiplier modulo x^n - 1, 1024 bits per lane per cycle.
// Output chunk c (bits 1024c..1024c+1023) = XOR over support p of the 1024-bit window of D starting
// at a = (1024c - p) mod n. D is stored extended (bits n..n+1023 = bits 0..1023) as 256-bit words
// over an eight-bank memory (word i -> bank i[2:0], address i>>3): the window touches words
// a>>8 .. (a>>8)+4, always five different banks, and equals {q7,..,q0} rotated right by a[10:0]
// (low 1024 bits).
// DUAL  mode: lane A and lane B compute chunk g of DA*P and DB*P (g = 0..ceil(nwords/4)-1).
// SPLIT mode: lane A computes chunk 2g, lane B chunk 2g+1 (DA and DB hold the same operand).
// One support entry per cycle from the list SRAM (shared by both lanes).
// A finished group (two 1024-bit chunks) is held in a one-group output buffer and leaves as up to
// four 256-bit word groups, each taken by the consumer with r_pop (r_valid = head present):
//   DUAL : (r_ka = r_kb = 4g+i, A word i, B word i), i = 0..3
//   SPLIT: (8g, A w0; 8g+1, A w1), (8g+2, A w2; 8g+3, A w3), (8g+4, B w0; 8g+5, B w1), (8g+6, B w2; 8g+7, B w3)
// so the consumer sees the same word-group stream as a 256-bit two-lane multiplier.
// Flow control: the last term of a group is issued only while the output buffer is empty and no
// other last term is in flight, so a finished group always finds the buffer free (the issue stage
// waits otherwise; the pipeline behind it never stalls).
module mul_engine import hare_pkg::*; (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,
  input  logic         abort,
  input  logic         split,
  input  logic [17:0]  n,
  input  logic [8:0]   w,          // support entries in list[0..w-1]
  input  logic [9:0]   nwords,     // 256-bit output words to produce
  // list SRAM read port
  output logic         l_ce,
  output logic [9:0]   l_a,
  input  logic [17:0]  l_q,
  // dense memory read ports (lane A / lane B)
  output dm_req_t      ma,
  input  dm_q_t        ma_q,
  output dm_req_t      mb,
  input  dm_q_t        mb_q,
  // results: word-group head of the output buffer
  output logic         r_valid,
  output logic [9:0]   r_ka,
  output logic [255:0] r_a,
  output logic         r_vb,       // second word of the group valid (r_kb < nwords)
  output logic [9:0]   r_kb,
  output logic [255:0] r_b,
  input  logic         r_pop,
  output logic         stall,      // issue stage waiting for the output buffer (trace)
  output logic         ovf,        // sticky: a group finished while the buffer was full (verification)
  output logic         busy
);
  // ---------------- issue counters (stage 0)
  logic [8:0]  j;
  logic [7:0]  g;
  logic [7:0]  nchunks, ngroups;
  logic        issuing;
  assign nchunks = 8'((nwords + 10'd3) >> 2);
  assign ngroups = split ? 8'((nwords + 10'd7) >> 3) : nchunks;

  // stage valid / tags
  logic        v1, v2, v3, v4;
  logic        f1, f2, f3, f4;     // first term of a group
  logic        e1, e2, e3, e4;     // last term of a group
  logic [7:0]  ca1, cb1, ca2, cb2, ca3, ca4;   // chunk indices

  // output buffer
  logic          o_full;
  logic [1:0]    o_rp;             // next word group to hand out
  logic [1023:0] o_a, o_b;
  logic [7:0]    o_ca;

  logic last_term, issue;
  assign last_term = (j == w - 9'd1);
  assign stall     = issuing && last_term && (o_full || (v1 && e1) || (v2 && e2) || (v3 && e3) || (v4 && e4));
  assign issue     = issuing && !stall;
  assign l_ce = issue;
  assign l_a  = {1'b0, j};

  // ---------------- stage 1: window start a = (1024c - p) mod n
  logic signed [19:0] qa, qb;
  logic [17:0] aa, ab;
  always_comb begin
    qa = $signed({2'b00, ca1, 10'd0}) - $signed({2'b00, l_q});
    qb = $signed({2'b00, cb1, 10'd0}) - $signed({2'b00, l_q});
    aa = qa[19] ? (qa[17:0] + n) : qa[17:0];
    ab = qb[19] ? (qb[17:0] + n) : qb[17:0];
  end
  logic [17:0] aa2, ab2;

  // ---------------- stage 2: bank addresses
  // bank b holds the word of {wa, .., wa+7} that is congruent to b mod 8:
  // address (wa >> 3) + (b < wa[2:0]); only the five banks of words wa..wa+4 are enabled, and lane B
  // is not read for a SPLIT chunk that starts at or beyond nwords (last group, never handed out)
  logic [9:0] wa, wb;
  logic       vb2;
  assign wa  = aa2[17:8];
  assign wb  = ab2[17:8];
  assign vb2 = !split || ({cb2, 2'b00} < nwords);
  always_comb begin
    ma = '0; mb = '0;
    if (v2) begin
      for (int b = 0; b < 8; b = b + 1) begin
        ma.ce[b] = ((3'(b) - wa[2:0]) <= 3'd4);
        ma.a[b]  = wa[9:3] + ((3'(b) < wa[2:0]) ? 7'd1 : 7'd0);
        mb.ce[b] = vb2 && ((3'(b) - wb[2:0]) <= 3'd4);
        mb.a[b]  = wb[9:3] + ((3'(b) < wb[2:0]) ? 7'd1 : 7'd0);
      end
    end
  end
  logic [10:0] ra3, rb3;           // rotation amounts a mod 2048

  // ---------------- stage 3: 2048 -> 1024 rotation
  logic [2047:0] cata, catb;
  logic [3070:0] dbla, dblb;       // highest selected bit: 2047 + 1023
  logic [1023:0] ta, tb;
  always_comb begin
    cata = ma_q;
    catb = mb_q;
    dbla = {cata[1022:0], cata};
    dblb = {catb[1022:0], catb};
    ta = dbla[ra3 +: 1024];
    tb = dblb[rb3 +: 1024];
  end
  logic [1023:0] ta4, tb4;

  // ---------------- stage 4: accumulate
  logic [1023:0] acca, accb;
  logic [1023:0] fin_a, fin_b;
  assign fin_a = f4 ? ta4 : (acca ^ ta4);
  assign fin_b = f4 ? tb4 : (accb ^ tb4);

  // ---------------- output word groups
  logic [9:0] ka, kb;
  logic [1:0] nwg_m1;              // word groups of the buffered group minus one
  logic [9:0] base_d, base_s, left_d, left_s;
  always_comb begin
    base_d = {o_ca, 2'b00};        // DUAL: first word of chunk o_ca
    base_s = {o_ca, 2'b00};        // SPLIT: first word of the group (o_ca = 2g, word 8g)
    left_d = nwords - base_d;
    left_s = nwords - base_s;
    if (split) begin
      ka = base_s + {7'd0, o_rp, 1'b0};
      kb = ka + 10'd1;
      r_a = o_rp[1] ? o_b[512*o_rp[0] +: 256]       : o_a[512*o_rp[0] +: 256];
      r_b = o_rp[1] ? o_b[512*o_rp[0] + 256 +: 256] : o_a[512*o_rp[0] + 256 +: 256];
      r_vb = (kb < nwords);
      nwg_m1 = (left_s >= 10'd8) ? 2'd3 : 2'(((left_s + 10'd1) >> 1) - 10'd1);
    end else begin
      ka = base_d + {8'd0, o_rp};
      kb = ka;
      r_a = o_a[256*o_rp +: 256];
      r_b = o_b[256*o_rp +: 256];
      r_vb = 1'b1;
      nwg_m1 = (left_d >= 10'd4) ? 2'd3 : 2'(left_d - 10'd1);
    end
  end
  assign r_valid = o_full;
  assign r_ka = ka;
  assign r_kb = kb;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      j <= '0; g <= '0; issuing <= 1'b0; busy <= 1'b0;
      {v1, v2, v3, v4} <= '0; {f1, f2, f3, f4} <= '0; {e1, e2, e3, e4} <= '0;
      {ca1, cb1, ca2, cb2, ca3, ca4} <= '0;
      aa2 <= '0; ab2 <= '0; ra3 <= '0; rb3 <= '0;
      ta4 <= '0; tb4 <= '0; acca <= '0; accb <= '0;
      o_full <= 1'b0; o_rp <= '0; o_a <= '0; o_b <= '0; o_ca <= '0; ovf <= 1'b0;
    end else if (abort) begin
      issuing <= 1'b0; busy <= 1'b0; {v1, v2, v3, v4} <= '0; o_full <= 1'b0;
    end else begin
      // stage 0
      if (start) begin
        j <= '0; g <= '0; issuing <= 1'b1; busy <= 1'b1; o_full <= 1'b0; ovf <= 1'b0;
      end else if (issue) begin
        if (last_term) begin
          j <= '0; g <= g + 8'd1;
          if (g == ngroups - 8'd1) issuing <= 1'b0;
        end else j <= j + 9'd1;
      end
      v1 <= issue; f1 <= (j == 9'd0); e1 <= last_term;
      ca1 <= split ? {g[6:0], 1'b0} : g;
      cb1 <= split ? {g[6:0], 1'b1} : g;
      // stage 1 -> 2
      v2 <= v1; f2 <= f1; e2 <= e1; ca2 <= ca1; cb2 <= cb1; aa2 <= aa; ab2 <= ab;
      // stage 2 -> 3
      v3 <= v2; f3 <= f2; e3 <= e2; ca3 <= ca2;
      ra3 <= aa2[10:0]; rb3 <= ab2[10:0];
      // stage 3 -> 4
      v4 <= v3; f4 <= f3; e4 <= e3; ca4 <= ca3;
      if (v3) begin ta4 <= ta; tb4 <= tb; end
      // stage 4
      if (v4) begin
        acca <= fin_a;
        accb <= fin_b;
      end
      // output buffer: filled by a group's last term (always empty then), drained by r_pop
      if (v4 && e4) begin
        if (o_full) ovf <= 1'b1;
        o_full <= 1'b1; o_rp <= '0;
        o_a <= fin_a; o_b <= fin_b; o_ca <= ca4;
      end else if (o_full && r_pop) begin
        o_rp <= o_rp + 2'd1;
        if (o_rp == nwg_m1) o_full <= 1'b0;
      end
      // busy until the last word group has left
      if (busy && !start && !issuing && !v1 && !v2 && !v3 && !v4 && !o_full) busy <= 1'b0;
    end
  end
endmodule
