// zen_pku_div769 : pipelined exact division by the constant 769 (constant reciprocal multiply).
//   q = floor(x * M / 2^S) == floor(x / 769) for every x < 2^W (M, S chosen offline and checked
//   against the Granlund-Montgomery bound), r = x - 769*q (0..768).
//   Stage 1: x split into 3 segments of SEG bits, three constant products registered.
//   Stage 2: aligned sum of the products, q = sum[S +: WQ] registered     (q valid 2 cycles after v_in)
//   Stage 3: r = (x[9:0] - (769*q)[9:0]) mod 1024 registered             (r valid 3 cycles after v_in)
// Datapath registers load only when their stage is valid (no reset needed); valid bits are reset.
module zen_pku_div769 #(
  parameter W   = 48,                         // input width
  parameter WQ  = 39,                         // quotient width (caller guarantees it fits)
  parameter S   = 58,                         // reciprocal shift
  parameter MW  = 49,                         // reciprocal width
  parameter [63:0] M = 64'd374811932576999,   // ceil(2^S / 769)
  parameter SEG = 16                          // segment width, 3*SEG >= W, 3*SEG >= 10
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          v_in,
  input  logic [W-1:0]  x,
  output logic          v_q,
  output logic [WQ-1:0] q,
  output logic          v_r,
  output logic [9:0]    r
);
  localparam PW = SEG + MW;          // segment product width
  localparam SW = 2 * SEG + PW;      // aligned sum width

  logic [3*SEG-1:0] xe;
  logic [SEG-1:0]   x0, x1, x2;
  logic [PW-1:0]    p0_d, p1_d, p2_d;
  logic [PW-1:0]    p0_q, p1_q, p2_q;
  logic [9:0]       xl1_q, xl2_q;
  logic             v1_q, v2_q, v3_q;
  logic [SW-1:0]    sum;
  logic [WQ-1:0]    q_d;
  logic [9:0]       q769_lo;

  always_comb begin
    xe = '0;
    xe[W-1:0] = x;
    x0 = xe[SEG-1:0];
    x1 = xe[2*SEG-1:SEG];
    x2 = xe[3*SEG-1:2*SEG];
    p0_d = x0 * M[MW-1:0];
    p1_d = x1 * M[MW-1:0];
    p2_d = x2 * M[MW-1:0];
  end

  // stage 1
  always_ff @(posedge clk) begin
    if (v_in) begin
      p0_q  <= p0_d;
      p1_q  <= p1_d;
      p2_q  <= p2_d;
      xl1_q <= xe[9:0];
    end
  end

  // stage 2
  always_comb begin
    sum = {{(2*SEG){1'b0}}, p0_q} + {{SEG{1'b0}}, p1_q, {SEG{1'b0}}} + {p2_q, {(2*SEG){1'b0}}};
    q_d = sum[S +: WQ];
  end

  always_ff @(posedge clk) begin
    if (v1_q) begin
      q     <= q_d;
      xl2_q <= xl1_q;
    end
  end

  // stage 3: 769*q mod 1024 = q + (q << 8) + (q << 9) (low 10 bits)
  always_comb begin
    q769_lo = q[9:0] + {q[1:0], 8'd0} + {q[0], 9'd0};
  end

  always_ff @(posedge clk) begin
    if (v2_q) r <= xl2_q - q769_lo;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v1_q <= 1'b0;
      v2_q <= 1'b0;
      v3_q <= 1'b0;
    end else begin
      v1_q <= v_in;
      v2_q <= v1_q;
      v3_q <= v2_q;
    end
  end

  assign v_q = v2_q;
  assign v_r = v3_q;
endmodule
