// zen_gf2 -- GF(2) polynomial unit
//
//   op = 0 (INV): res = gf2_inv(f, N4)       ring x^N4 - 1, N4 = 128 << param
//                 res = f; s = f; repeat log2(N4)-1 times { s = s^2; res = res * s }
//   op = 1 (DEC): res = gf2_mul_cyc(A, B, N2, N4)   (low N4 bits of A*B mod x^N2 - 1)
//
// Micro-architecture
//   * rot_q  [1023:0] : rotating multiplicand register with run-time ring size
//                       n = N4 (INV) or N2 (DEC). In DEC it is the A register (a_clr / a_wr_*
//                       XOR-accumulate, idle only). Bits >= n are kept at zero.
//   * b_q    [511:0]  : second operand. DEC: B written through b_wr_* (idle only).
//                       INV: holds the running square s.
//   * acc_q  [511:0]  : product accumulator = res output (only bits < N4 are ever non-zero).
//   * One multiply = N4/4 cycles; each cycle consumes 4 bits b[4c+3:4c] of the second operand:
//       acc[i] ^= XOR_k b[4c+k] & rot[(i-k) mod n],  rot <= rot4(rot) (rotate left by 4 in ring n).
//   * INV: squaring of s is a single combinational step, merged into the start cycle (first
//     square of f) and into the last multiply cycle of each iteration (rot <= new res,
//     acc <= 0, s <= s^2), so an iteration takes exactly N4/4 cycles.
//   * Cycle counts start->done: INV 6*32+1 / 7*64+1 / 8*128+1, DEC 32+1 / 64+1 / 128+1.
//   * res (acc_q) holds its value after done until the next start.

module zen_gf2 (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,
  input  logic [1:0]   param,
  input  logic         op,          // 0 INV, 1 DEC
  input  logic [511:0] f_in,
  input  logic         b_wr_en,
  input  logic [2:0]   b_wr_idx,
  input  logic [63:0]  b_wr_data,
  input  logic         a_clr,
  input  logic         a_wr_en,
  input  logic [5:0]   a_wr_idx,
  input  logic [15:0]  a_wr_data,
  output logic         done,
  output logic         active,
  output logic [511:0] res
);

  localparam logic [1:0] S_IDLE = 2'd0;
  localparam logic [1:0] S_MUL  = 2'd1;
  localparam logic [1:0] S_DONE = 2'd2;

  // ---------------------------------------------------------------- control state
  logic [1:0] state_q;
  logic [6:0] cnt_q;      // 4-bit group index within one multiply (0 .. N4/4-1)
  logic [3:0] iter_q;     // INV iteration index

  // ---------------------------------------------------------------- datapath registers
  logic [1023:0] rot_q;
  logic [511:0]  b_q;
  logic [511:0]  acc_q;

  // ---------------------------------------------------------------- configuration decode
  logic [1:0] rs;         // ring size code: n = 128 << rs
  logic [6:0] cnt_last;   // N4/4 - 1
  logic [3:0] iter_last;  // log2(N4) - 2
  logic       busy;

  always_comb begin
    rs = op ? (param + 2'd1) : param;
    case (param)
      2'd0:    begin cnt_last = 7'd31;  iter_last = 4'd5; end
      2'd1:    begin cnt_last = 7'd63;  iter_last = 4'd6; end
      default: begin cnt_last = 7'd127; iter_last = 4'd7; end
    endcase
  end

  assign busy = (state_q != S_IDLE);

  // ---------------------------------------------------------------- per-bit validity masks
  // nmask[i]  : i < n   (ring size of the rotating register)
  // n4mask[i] : i < N4  (output width)
  logic [1023:0] nmask;
  logic [511:0]  n4mask;

  always_comb begin
    for (int i = 0; i < 1024; i++) begin
      if (i < 128)       nmask[i] = 1'b1;
      else if (i < 256)  nmask[i] = (rs != 2'd0);
      else if (i < 512)  nmask[i] = rs[1];
      else               nmask[i] = (rs == 2'd3);
    end
    for (int i = 0; i < 512; i++) begin
      if (i < 128)       n4mask[i] = 1'b1;
      else if (i < 256)  n4mask[i] = (param != 2'd0);
      else               n4mask[i] = param[1];
    end
  end

  // ---------------------------------------------------------------- wrap-around bits
  // wrap[d] = rot[n - d], d = 1..4  (wrap[0] unused)
  logic [4:0] wrap;

  always_comb begin
    wrap[0] = 1'b0;
    for (int d = 1; d <= 4; d++) begin
      case (rs)
        2'd0:    wrap[d] = rot_q[128  - d];
        2'd1:    wrap[d] = rot_q[256  - d];
        2'd2:    wrap[d] = rot_q[512  - d];
        default: wrap[d] = rot_q[1024 - d];
      endcase
    end
  end

  // ---------------------------------------------------------------- rotate left by 4 in ring n
  logic [1023:0] rot_r4;

  always_comb begin
    for (int i = 0; i < 4; i++)
      rot_r4[i] = wrap[4 - i];
    for (int i = 4; i < 1024; i++)
      rot_r4[i] = rot_q[i - 4] & nmask[i];
  end

  // ---------------------------------------------------------------- multiply step
  logic [3:0]   bsel;
  logic [511:0] acc_nxt;

  assign bsel = b_q[{cnt_q, 2'b00} +: 4];

  logic acc_t;

  always_comb begin
    acc_t = 1'b0;
    for (int i = 0; i < 512; i++) begin
      acc_t = acc_q[i];
      for (int k = 0; k < 4; k++) begin
        if (i >= k) acc_t = acc_t ^ (bsel[k] & rot_q[i - k]);
        else        acc_t = acc_t ^ (bsel[k] & wrap[k - i]);
      end
      acc_nxt[i] = acc_t & n4mask[i];
    end
  end

  // ---------------------------------------------------------------- squaring (ring N4)
  // sq[2t] = s[t] ^ s[t + N4/2], sq[odd] = 0
  function automatic logic [511:0] gf2_sq(input logic [511:0] s, input logic [1:0] pr);
    logic [511:0] r;
    r = '0;
    for (int t = 0; t < 256; t++) begin
      case (pr)
        2'd0:    if (t < 64)  r[2*t] = s[t] ^ s[t + 64];
        2'd1:    if (t < 128) r[2*t] = s[t] ^ s[t + 128];
        default:              r[2*t] = s[t] ^ s[t + 256];
      endcase
    end
    return r;
  endfunction

  logic [511:0] f_m;
  assign f_m = f_in & n4mask;

  logic       mul_last;
  logic       inv_final;
  assign mul_last  = (state_q == S_MUL) && (cnt_q == cnt_last);
  assign inv_final = op || (iter_q == iter_last);

  // ---------------------------------------------------------------- control FSM
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_q <= S_IDLE;
      cnt_q   <= '0;
      iter_q  <= '0;
    end else begin
      case (state_q)
        S_IDLE: begin
          if (start) begin
            state_q <= S_MUL;
            cnt_q   <= '0;
            iter_q  <= '0;
          end
        end
        S_MUL: begin
          if (cnt_q == cnt_last) begin
            cnt_q <= '0;
            if (inv_final) state_q <= S_DONE;
            else           iter_q  <= iter_q + 4'd1;
          end else begin
            cnt_q <= cnt_q + 7'd1;
          end
        end
        default: begin   // S_DONE
          state_q <= S_IDLE;
        end
      endcase
    end
  end

  assign done   = (state_q == S_DONE);
  assign active = busy;
  assign res    = acc_q;

  // ---------------------------------------------------------------- rotating / A register
  logic [63:0] a_sel;
  assign a_sel = a_wr_en ? (64'd1 << a_wr_idx) : 64'd0;

  always_ff @(posedge clk) begin
    if (!busy) begin
      if (start && !op) begin
        rot_q <= {512'b0, f_m};
      end else if (a_clr || a_wr_en) begin
        for (int c = 0; c < 64; c++) begin
          if (a_sel[c])
            rot_q[16*c +: 16] <= (a_clr ? 16'h0000 : rot_q[16*c +: 16]) ^ a_wr_data;
          else if (a_clr)
            rot_q[16*c +: 16] <= 16'h0000;
        end
      end
    end else if (state_q == S_MUL) begin
      if (mul_last && !inv_final) rot_q <= {512'b0, acc_nxt};
      else if (!mul_last)         rot_q <= rot_r4;
    end
  end

  // ---------------------------------------------------------------- second operand / square
  always_ff @(posedge clk) begin
    if (!busy) begin
      if (start && !op) begin
        b_q <= gf2_sq(f_m, param);
      end else if (b_wr_en) begin
        b_q[64*b_wr_idx +: 64] <= b_wr_data;
      end
    end else if (mul_last && !inv_final) begin
      b_q <= gf2_sq(b_q, param);
    end
  end

  // ---------------------------------------------------------------- accumulator / result
  always_ff @(posedge clk) begin
    if (!busy) begin
      if (start) acc_q <= '0;
    end else if (state_q == S_MUL) begin
      if (mul_last && !inv_final) acc_q <= '0;
      else                        acc_q <= acc_nxt;
    end
  end

endmodule
