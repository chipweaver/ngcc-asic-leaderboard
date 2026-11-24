// zen_bre_lane: one SIMD lane of the base-ring engine.
//   * NR x 16-bit register file (flip-flops, NR = 68 in this configuration); r0..r35 is the
//     input window (a: r0..15, b: r16..31, zetas r32..35), written by the batch load port
//   * operand-fetch stage: the 13 operands of the NEXT instruction (ALU0..2 a/b/c, MUL0 a/b, MUL1 a/b)
//     are read one cycle ahead into operand registers through per-lane registered read selects (fsel);
//     results written at the end of the fetch cycle are forwarded from the result buses (byp; the MUL
//     buses are forced off by zen_bre when MFWD = 0),
//     immediates come pre-decoded (nisimm/nimm). Operand registers hold while the engine stalls/idles.
//   * 3 ALUs: LIN = w16(+-a +-b +-c) (each term may be zero; ALU2 has no third term if K2), CANON(a); written at the end
//     of the issue cycle t. ALU0 operand a also feeds the shared QINV server (q_rd).
//   * 2 fqmul pipelines (written at the end of t+2; consumable at t+3 with MUL forwarding, else t+4):
//                                 t  two 16x8 partial products
//                                 t+1 combine -> p[31:16] and u = w16(p * -767)
//                                 t+2 r = p[31:16] - (u*769)[26:16] -> RF write (readable t+3)
//   * 16 x 16-bit output buffer obuf: program outputs are captured when produced
module zen_bre_lane #(
  parameter int NR = 68,        // registers
  parameter bit K2 = 1'b1       // ALU2 two-operand (third term forced to zero)
) (
  input  logic                 clk,
  input  logic                 adv,
  input  logic                 fetch_en,
  input  logic                 fsel_ld,
  input  logic [12:0][6:0]     fsel_d,
  input  logic [12:0]          nisimm,
  input  logic [12:0][15:0]    nimm,
  input  logic [4:0][12:0]     byp,          // [bus][operand]: buses ALU0, ALU1, ALU2, MUL0, MUL1
  input  logic [2:0]           alu_canon,
  input  logic [2:0][2:0]      alu_neg,
  input  logic [2:0][2:0]      alu_zero,
  input  logic [2:0][1:0]      alu_cin,
  input  logic [2:0][NR-1:0]     alu_woh,
  input  logic [2:0]           alu_oe,
  input  logic [2:0][3:0]      alu_oi,
  input  logic [1:0][NR-1:0]     mul_woh,
  input  logic [1:0]           mul_oe,
  input  logic [1:0][3:0]      mul_oi,
  input  logic [NR-1:0]          q_woh,
  input  logic [9:0]           q_data,
  output logic [15:0]          q_rd,
  input  logic [35:0]          ld_we,
  input  logic [35:0][15:0]    ld_data,
  output logic [15:0][15:0]    obuf
);

  logic [15:0] rf [0:NR-1];
  logic [12:0][6:0]  fsel;
  logic [12:0][15:0] opr;
  logic [2:0][15:0]  alu_res;
  logic [1:0][15:0]  mul_res;

  always_ff @(posedge clk) begin
    if (fsel_ld) fsel <= fsel_d;
  end

  always_ff @(posedge clk) begin
    if (fetch_en) begin
      for (int k = 0; k < 13; k++) begin
        if (nisimm[k])      opr[k] <= nimm[k];
        else if (byp[0][k]) opr[k] <= alu_res[0];
        else if (byp[1][k]) opr[k] <= alu_res[1];
        else if (byp[2][k]) opr[k] <= alu_res[2];
        else if (byp[3][k]) opr[k] <= mul_res[0];
        else if (byp[4][k]) opr[k] <= mul_res[1];
        else                opr[k] <= rf[(fsel[k] < NR) ? fsel[k] : 7'd0];
      end
    end
  end

  assign q_rd = opr[0];

  // ALUs: sum of up to three signed terms modulo 2^16 (-x = ~x + 1)
  logic [2:0][15:0] t0, t1, t2;
  always_comb begin
    for (int k = 0; k < 3; k++) begin
      t0[k] = (opr[3*k]   ^ {16{alu_neg[k][0]}}) & {16{~alu_zero[k][0]}};
      t1[k] = (opr[3*k+1] ^ {16{alu_neg[k][1]}}) & {16{~alu_zero[k][1]}};
      t2[k] = (K2 && k == 2) ? 16'd0 : ((opr[3*k+2] ^ {16{alu_neg[k][2]}}) & {16{~alu_zero[k][2]}});
      if (alu_canon[k]) alu_res[k] = opr[3*k] + (opr[3*k][15] ? 16'd769 : 16'd0);
      else              alu_res[k] = t0[k] + t1[k] + t2[k] + {14'd0, alu_cin[k]};
    end
  end

  // fqmul pipelines
  genvar gu;
  generate
    for (gu = 0; gu < 2; gu++) begin : g_mul
      logic signed [24:0] m_pl;
      logic signed [23:0] m_ph;
      logic signed [31:0] m_p;
      logic [15:0] mb_hi, mb_u, m_ulo;
      logic signed [26:0] m_uq;
      always_comb begin
        m_p   = $signed({m_ph, 8'd0}) + $signed({{7{m_pl[24]}}, m_pl});
        m_ulo = m_p[15:0];
        m_uq  = $signed(mb_u) * 27'sd769;
        mul_res[gu] = mb_hi - {{5{m_uq[26]}}, m_uq[26:16]};
      end
      always_ff @(posedge clk) begin
        if (adv) begin
          m_pl  <= $signed(opr[9+2*gu]) * $signed({1'b0, opr[10+2*gu][7:0]});
          m_ph  <= $signed(opr[9+2*gu]) * $signed(opr[10+2*gu][15:8]);
          mb_hi <= m_p[31:16];
          mb_u  <= m_ulo + {m_ulo[7:0], 8'd0} - {m_ulo[5:0], 10'd0};   // w16(p * -767)
        end
      end
    end
  endgenerate

  always_ff @(posedge clk) begin
    for (int i = 0; i < 36; i++) begin
      if (ld_we[i])            rf[i] <= ld_data[i];
      else if (alu_woh[0][i])  rf[i] <= alu_res[0];
      else if (alu_woh[1][i])  rf[i] <= alu_res[1];
      else if (alu_woh[2][i])  rf[i] <= alu_res[2];
      else if (mul_woh[0][i])  rf[i] <= mul_res[0];
      else if (mul_woh[1][i])  rf[i] <= mul_res[1];
      else if (q_woh[i])       rf[i] <= {6'd0, q_data};
    end
    for (int i = 36; i < NR; i++) begin
      if (alu_woh[0][i])       rf[i] <= alu_res[0];
      else if (alu_woh[1][i])  rf[i] <= alu_res[1];
      else if (alu_woh[2][i])  rf[i] <= alu_res[2];
      else if (mul_woh[0][i])  rf[i] <= mul_res[0];
      else if (mul_woh[1][i])  rf[i] <= mul_res[1];
      else if (q_woh[i])       rf[i] <= {6'd0, q_data};
    end
  end

  always_ff @(posedge clk) begin
    for (int e = 0; e < 16; e++) begin
      if (alu_oe[0] && alu_oi[0] == e[3:0])      obuf[e] <= alu_res[0];
      else if (alu_oe[1] && alu_oi[1] == e[3:0]) obuf[e] <= alu_res[1];
      else if (alu_oe[2] && alu_oi[2] == e[3:0]) obuf[e] <= alu_res[2];
      else if (mul_oe[0] && mul_oi[0] == e[3:0]) obuf[e] <= mul_res[0];
      else if (mul_oe[1] && mul_oi[1] == e[3:0]) obuf[e] <= mul_res[1];
    end
  end

endmodule
