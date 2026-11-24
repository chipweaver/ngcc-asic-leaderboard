// nev_lane: one arithmetic lane of the poly ALU. Fixed latency per mode: inputs presented in cycle c
// produce out_v/out_y0/out_y1 in cycle c+4 for CT/GS (twiddle multiplication with the precomputed
// Shoup quotient in_cq = floor(c*2^16/q): t = b*c - floor(b*in_cq/2^16)*q in [0, 2q)) and in cycle c+7
// for every other mode (Barrett reduction). All values canonical in [0, q).
//   mode CT  : t = b*c          ; y0 = a + t ; y1 = a - t        (NTT butterfly)
//   mode GS  : t = (b - a)*c    ; y0 = a + b ; y1 = t            (INTT butterfly)
//   mode MAC : multi-row slot MAC, up to four output rows per pass (row block). A V delay line
//              vl[0..3] holds the V operand of the current step (vl[0] = nb ? -b : b, 0 if bz) and of
//              the three previous steps (cleared by clr at the start of a term set); accumulator
//              acc[d] += a*vl[d]. E step (one): acc[2*ek] += a, acc[2*ek+1] += b (b ignored if bz).
//              On last, the rl-selected accumulators acc[0..2^rl-1] are reduced one per cycle
//              (acc[0] in the cycle after last, as a single-row MAC): y0 = neg ? -acc : acc (mod q);
//              out_v marks each reduced row
//   mode MULC: t = a*c          ; y0 = t     ; y1 = a            (scale + copy)
//   mode COMB: t = b*c          ; y0 = a - t                      (norm combine)
//   mode MUL2: t = a*b          ; y0 = t                          (exponentiation steps)
module nev_lane import nev_pkg::*; (
  input  logic        clk,
  input  logic        rst_n,
  input  logic [1:0]  qsel,
  input  logic        in_v,
  input  logic [2:0]  in_mode,
  input  logic        in_first,
  input  logic        in_last,
  input  logic        in_neg,
  input  logic        in_one,
  input  logic        in_nb,
  input  logic        in_cn,      // MAC E step: subtract (multiply by q-1)
  input  logic        in_bz,      // MAC: V operand absent (0)
  input  logic        in_clr,     // MAC: first step of a term set (clear delayed V)
  input  logic        in_ek,      // MAC E step index
  input  logic [1:0]  in_rl,      // MAC: log2 rows of the block
  input  logic [11:0] in_a,
  input  logic [11:0] in_b,
  input  logic [11:0] in_c,
  input  logic [15:0] in_cq,      // CT/GS: Shoup quotient of in_c
  output logic        out_v,
  output logic [11:0] out_y0,
  output logic [11:0] out_y1
);
  localparam logic [2:0] M_CT = 3'd0, M_GS = 3'd1, M_MAC = 3'd2, M_MULC = 3'd3, M_COMB = 3'd4, M_MUL2 = 3'd5;

  // modulus constants, registered locally (qsel is quasi-static: it changes only at command
  // acceptance, several cycles before the first lane operation)
  logic [11:0] q;
  logic [20:0] bm;     // floor(2^30 / q)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin q <= 12'd769; bm <= 21'd1396283; end
    else begin
      case (qsel)
        Q769:    begin q <= 12'd769;  bm <= 21'd1396283; end
        Q1409:   begin q <= 12'd1409; bm <= 21'd762059;  end
        default: begin q <= 12'd3329; bm <= 21'd322541;  end
      endcase
    end
  end

  function automatic logic [11:0] addq(input logic [11:0] x, input logic [11:0] y, input logic [11:0] qq);
    logic [12:0] s;
    s = {1'b0, x} + {1'b0, y};
    addq = (s >= {1'b0, qq}) ? 12'(s - {1'b0, qq}) : s[11:0];
  endfunction
  function automatic logic [11:0] subq(input logic [11:0] x, input logic [11:0] y, input logic [11:0] qq);
    logic [12:0] s;
    s = {1'b0, x} - {1'b0, y};
    subq = s[12] ? 12'(s + {1'b0, qq}) : s[11:0];
  endfunction

  // ---------------- S0: inputs (combinational; the poly ALU registers them in its read path)
  logic        s0_v, s0_first, s0_last, s0_neg, s0_one, s0_nb, s0_bz, s0_clr, s0_ek, s0_cn;
  logic [1:0]  s0_rl;
  logic [2:0]  s0_mode;
  logic [11:0] s0_a, s0_b, s0_c;
  logic [15:0] s0_cq;
  always_comb begin
    s0_v = in_v; s0_mode = in_mode; s0_first = in_first; s0_last = in_last; s0_neg = in_neg; s0_one = in_one; s0_nb = in_nb;
    s0_a = in_a; s0_b = in_b; s0_c = in_c; s0_cq = in_cq; s0_bz = in_bz; s0_cn = in_cn; s0_clr = in_clr; s0_ek = in_ek; s0_rl = in_rl;
  end

  logic        fast0;
  assign fast0 = (s0_mode == M_CT) || (s0_mode == M_GS);

  // ---------------- S1: operand selection (GS pre-op)
  logic        s1_v, s1_first, s1_last, s1_neg;
  logic [2:0]  s1_mode;
  logic [11:0] s1_m1, s1_m2, s1_pa;
  logic [11:0] m1_n, m2_n, pa_n;
  logic [3:0][11:0] vl;            // V delay line (MAC)
  logic [3:0][11:0] s1_x1, s1_x2;  // MAC multiplier operands of rows 1..3 (index 0 unused)
  logic [1:0]  s1_rl;
  logic [11:0] vin_n;
  always_comb vin_n = s0_bz ? 12'd0 : (s0_nb ? subq(12'd0, s0_b, q) : s0_b);
  always_comb begin
    m1_n = s0_a; m2_n = s0_c; pa_n = s0_a;
    case (s0_mode)
      M_CT:   begin m1_n = s0_b; m2_n = s0_c; pa_n = s0_a; end
      M_GS:   begin m1_n = subq(s0_b, s0_a, q); m2_n = s0_c; pa_n = addq(s0_a, s0_b, q); end
      M_MAC:  begin m1_n = s0_a; m2_n = s0_one ? (s0_cn ? 12'(q - 12'd1) : 12'd1) : vin_n; pa_n = s0_a; end
      M_MULC: begin m1_n = s0_a; m2_n = s0_c; pa_n = s0_a; end
      M_COMB: begin m1_n = s0_b; m2_n = s0_c; pa_n = s0_a; end
      default:begin m1_n = s0_a; m2_n = s0_b; pa_n = s0_a; end
    endcase
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s1_v <= 1'b0; s1_mode <= '0; s1_first <= 1'b0; s1_last <= 1'b0; s1_neg <= 1'b0;
      s1_m1 <= '0; s1_m2 <= '0; s1_pa <= '0; vl <= '0; s1_x1 <= '0; s1_x2 <= '0; s1_rl <= '0;
    end else begin
      s1_v <= s0_v & ~fast0; s1_mode <= s0_mode; s1_first <= s0_first; s1_last <= s0_last; s1_neg <= s0_neg;
      s1_m1 <= m1_n; s1_m2 <= m2_n; s1_pa <= pa_n; s1_rl <= s0_rl;
      if (s0_v && s0_mode == M_MAC) begin
        if (s0_one) begin
          // E step: rows 2ek and 2ek+1 add a and b
          for (int d = 1; d < 4; d++) begin
            s1_x2[d] <= s0_cn ? 12'(q - 12'd1) : 12'd1;
            if (d == {s0_ek, 1'b0})      s1_x1[d] <= s0_a;
            else if (d == {s0_ek, 1'b1}) s1_x1[d] <= s0_bz ? 12'd0 : s0_b;
            else                         s1_x1[d] <= 12'd0;
          end
          if (s0_ek) begin s1_m1 <= 12'd0; s1_m2 <= 12'd0; end
        end else begin
          vl[0] <= vin_n;
          for (int d = 1; d < 4; d++) vl[d] <= s0_clr ? 12'd0 : vl[d-1];
          for (int d = 1; d < 4; d++) begin
            s1_x1[d] <= s0_a;
            s1_x2[d] <= s0_clr ? 12'd0 : vl[d-1];
          end
        end
      end
    end
  end

  // ---------------- fast path (CT / GS): F1 operands, F2 products, F3 remainder, then the output stage
  logic        f1_v, f1_gs, f2_v, f2_gs, f3_v, f3_gs;
  logic [11:0] f1_m, f1_w, f1_pa, f2_pa, f3_pa;
  logic [15:0] f1_wq;
  logic [13:0] f2_pl, f3_t;
  logic [11:0] f2_qe;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      f1_v <= 1'b0; f1_gs <= 1'b0; f1_m <= '0; f1_w <= '0; f1_wq <= '0; f1_pa <= '0;
      f2_v <= 1'b0; f2_gs <= 1'b0; f2_pl <= '0; f2_qe <= '0; f2_pa <= '0;
      f3_v <= 1'b0; f3_gs <= 1'b0; f3_t <= '0; f3_pa <= '0;
    end else begin
      f1_v <= s0_v & fast0; f1_gs <= (s0_mode == M_GS);
      if (s0_v & fast0) begin f1_m <= m1_n; f1_w <= m2_n; f1_wq <= s0_cq; f1_pa <= pa_n; end
      f2_v <= f1_v; f2_gs <= f1_gs; f2_pa <= f1_pa;
      if (f1_v) begin
        logic [23:0] p1;
        logic [27:0] p2;
        p1 = f1_m * f1_w;
        p2 = f1_m * f1_wq;
        f2_pl <= p1[13:0];
        f2_qe <= p2[27:16];
      end
      f3_v <= f2_v; f3_gs <= f2_gs; f3_pa <= f2_pa;
      if (f2_v) f3_t <= f2_pl - 14'(f2_qe * q);
    end
  end

  // ---------------- S2: multiply
  logic        s2_v, s2_first, s2_last, s2_neg;
  logic [2:0]  s2_mode;
  logic [23:0] s2_p;
  logic [3:0][23:0] s2_px;         // MAC rows 1..3
  logic [1:0]  s2_rl;
  logic [11:0] s2_pa;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s2_v <= 1'b0; s2_mode <= '0; s2_first <= 1'b0; s2_last <= 1'b0; s2_neg <= 1'b0; s2_p <= '0; s2_pa <= '0;
      s2_px <= '0; s2_rl <= '0;
    end else begin
      s2_v <= s1_v; s2_mode <= s1_mode; s2_first <= s1_first; s2_last <= s1_last; s2_neg <= s1_neg;
      s2_p <= s1_m1 * s1_m2; s2_pa <= s1_pa; s2_rl <= s1_rl;
      for (int d = 1; d < 4; d++) s2_px[d] <= (s1_mode == M_MAC) ? s1_x1[d] * s1_x2[d] : 24'd0;
    end
  end

  // ---------------- S3: accumulate / pass
  logic        s3_v, s3_neg;
  logic [2:0]  s3_mode;
  logic [27:0] s3_x;
  logic [3:0][27:0] acc, sum_n, sr;
  logic [1:0]  scnt, slast;        // serializer: next row index, last row index
  logic        son, sneg;
  logic [11:0] s3_pa;
  always_comb begin
    sum_n[0] = (s2_first ? 28'd0 : acc[0]) + {4'd0, s2_p};
    for (int d = 1; d < 4; d++) sum_n[d] = (s2_first ? 28'd0 : acc[d]) + {4'd0, s2_px[d]};
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s3_v <= 1'b0; s3_mode <= '0; s3_neg <= 1'b0; s3_x <= '0; acc <= '0; s3_pa <= '0;
      sr <= '0; scnt <= '0; slast <= '0; son <= 1'b0; sneg <= 1'b0;
    end else begin
      s3_mode <= s2_mode; s3_neg <= s2_neg; s3_pa <= s2_pa;
      if (s2_v && s2_mode == M_MAC) acc <= sum_n;
      if (s2_v && s2_mode == M_MAC && s2_last) begin
        // row 0 now, rows 1 .. 2^rl-1 in the following cycles
        s3_v  <= 1'b1;
        s3_x  <= sum_n[0];
        sr    <= sum_n;
        scnt  <= 2'd1;
        slast <= 2'((3'd1 << s2_rl) - 3'd1);
        son   <= (s2_rl != 2'd0);
        sneg  <= s2_neg;
      end else if (son) begin
        s3_v   <= 1'b1;
        s3_x   <= sr[scnt];
        s3_neg <= sneg;
        s3_mode <= M_MAC;
        scnt   <= scnt + 2'd1;
        if (scnt == slast) son <= 1'b0;
      end else if (s2_mode == M_MAC) begin
        s3_v <= 1'b0;
      end else begin
        s3_v <= s2_v;
        s3_x <= {4'd0, s2_p};
      end
    end
  end

  // ---------------- S4a: Barrett partial products x_hi*m, x_lo*m (m = floor(2^30/q))
  logic        s4a_v, s4a_neg;
  logic [2:0]  s4a_mode;
  logic [34:0] s4a_ph, s4a_pl;
  logic [13:0] s4a_xl;
  logic [11:0] s4a_pa;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s4a_v <= 1'b0; s4a_mode <= '0; s4a_neg <= 1'b0; s4a_ph <= '0; s4a_pl <= '0; s4a_xl <= '0; s4a_pa <= '0;
    end else begin
      s4a_v <= s3_v; s4a_mode <= s3_mode; s4a_neg <= s3_neg; s4a_pa <= s3_pa;
      s4a_ph <= s3_x[27:14] * bm; s4a_pl <= s3_x[13:0] * bm; s4a_xl <= s3_x[13:0];
    end
  end

  // ---------------- S4b: quotient estimate qt = floor(x * m / 2^30)
  logic        s4_v, s4_neg;
  logic [2:0]  s4_mode;
  logic [18:0] s4_qt;
  logic [13:0] s4_xl;
  logic [11:0] s4_pa;
  logic [48:0] bprod;
  always_comb bprod = {s4a_ph, 14'd0} + {14'd0, s4a_pl};
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s4_v <= 1'b0; s4_mode <= '0; s4_neg <= 1'b0; s4_qt <= '0; s4_xl <= '0; s4_pa <= '0;
    end else begin
      s4_v <= s4a_v; s4_mode <= s4a_mode; s4_neg <= s4a_neg; s4_pa <= s4a_pa;
      s4_qt <= bprod[48:30]; s4_xl <= s4a_xl;
    end
  end

  // ---------------- S5: remainder r = x - qt*q in [0, 2q)  (low 14 bits suffice)
  logic        s5_v, s5_neg;
  logic [2:0]  s5_mode;
  logic [13:0] s5_r;
  logic [11:0] s5_pa;
  logic [13:0] qtq;
  always_comb qtq = 14'(s4_qt[13:0] * q);
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s5_v <= 1'b0; s5_mode <= '0; s5_neg <= 1'b0; s5_r <= '0; s5_pa <= '0;
    end else begin
      s5_v <= s4_v; s5_mode <= s4_mode; s5_neg <= s4_neg; s5_pa <= s4_pa;
      s5_r <= s4_xl - qtq;
    end
  end

  // ---------------- S6: canonical t and post operation
  logic [11:0] t, y0_n, y1_n;
  always_comb begin
    t = (s5_r >= {2'b00, q}) ? 12'(s5_r - {2'b00, q}) : s5_r[11:0];
    y0_n = t; y1_n = s5_pa;
    case (s5_mode)
      M_CT:   begin y0_n = addq(s5_pa, t, q); y1_n = subq(s5_pa, t, q); end
      M_GS:   begin y0_n = s5_pa; y1_n = t; end
      M_MAC:  begin y0_n = s5_neg ? subq(12'd0, t, q) : t; y1_n = '0; end
      M_MULC: begin y0_n = t; y1_n = s5_pa; end
      M_COMB: begin y0_n = subq(s5_pa, t, q); y1_n = '0; end
      default:begin y0_n = t; y1_n = '0; end
    endcase
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      out_v <= 1'b0; out_y0 <= '0; out_y1 <= '0;
    end else begin
      if (f3_v) begin
        logic [11:0] tf;
        tf = (f3_t >= {2'b00, q}) ? 12'(f3_t - {2'b00, q}) : f3_t[11:0];
        out_v  <= 1'b1;
        out_y0 <= f3_gs ? f3_pa : addq(f3_pa, tf, q);
        out_y1 <= f3_gs ? tf    : subq(f3_pa, tf, q);
      end else begin
        out_v <= s5_v; out_y0 <= y0_n; out_y1 <= y1_n;
      end
    end
  end
endmodule
