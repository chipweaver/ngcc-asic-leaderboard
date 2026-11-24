// nev_codec_enc5: pipelined encode5 (q = 769), one group of 5 canonical coefficients per cycle.
//
// V = sum_t (x_t >> 3) * 97^t (33 bits for canonical inputs), evaluated by Horner in two stages
// of two steps each. Output bytes (out_b[8i+7:8i] = byte i): bytes 0-1 = wl = {lo bits of
// x4..x0 (3 bits each), V[32]}, bytes 2-5 = V[31:0] little endian (reference encode5).
// Latency: 3 cycles (in_v -> out_v).
module nev_codec_enc5 (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        in_v,
  input  logic [59:0] in_x,    // coefficient t at [12t+11:12t], canonical (< 769)
  output logic        out_v,
  output logic [47:0] out_b,
  output logic        busy
);

  logic        e0_v; logic [6:0] e0_h4, e0_h3, e0_h2, e0_h1, e0_h0; logic [14:0] e0_lo;
  logic        e1_v; logic [19:0] e1_h; logic [6:0] e1_h1, e1_h0;     logic [14:0] e1_lo;
  logic [19:0] e1_n;
  logic [32:0] e2_n;

  always_comb begin : enc_comb
    e1_n = 20'd9409 * {13'd0, e0_h4} + 20'd97 * {13'd0, e0_h3} + {13'd0, e0_h2};
    e2_n = 33'd9409 * {13'd0, e1_h} + 33'd97 * {26'd0, e1_h1} + {26'd0, e1_h0};
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      e0_v <= 1'b0; e1_v <= 1'b0; out_v <= 1'b0;
      e0_h4 <= '0; e0_h3 <= '0; e0_h2 <= '0; e0_h1 <= '0; e0_h0 <= '0; e0_lo <= '0;
      e1_h <= '0; e1_h1 <= '0; e1_h0 <= '0; e1_lo <= '0;
      out_b <= '0;
    end else begin
      e0_v  <= in_v;
      e0_h0 <= in_x[9:3];
      e0_h1 <= in_x[21:15];
      e0_h2 <= in_x[33:27];
      e0_h3 <= in_x[45:39];
      e0_h4 <= in_x[57:51];
      e0_lo <= {in_x[50:48], in_x[38:36], in_x[26:24], in_x[14:12], in_x[2:0]};
      e1_v  <= e0_v;
      e1_h  <= e1_n;
      e1_h1 <= e0_h1;
      e1_h0 <= e0_h0;
      e1_lo <= e0_lo;
      out_v <= e1_v;
      out_b <= {e2_n[31:0], e1_lo, e2_n[32]};
    end
  end

  assign busy = e0_v | e1_v | out_v;

endmodule
