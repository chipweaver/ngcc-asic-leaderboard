// Word-wise passes over 16 lanes.
// TOMSG (poly_tomsg on M - decompress_c2(Vd)):  message bit 32*(c div 128) + (c mod 32) is set when
//   sum over its 4 coefficients c = 128i + 8j + k + {0,32,64,96} of |x - 1664| is < q.  In the
//   segment layout those coefficients are words u + 32r (r < W/32) of lanes in groups of 128/W, so
//   words are visited in the order u, r and the sums close per u.  Result -> regfile MSG (b bytes).
// (The re-encryption comparison runs in the stream units, compare mode.)
module aigis_wpass (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          start,
  input  logic [1:0]    param,
  input  logic [511:0]  msg,
  output logic          busy,
  output logic          done,
  // two read roles (0 = M, 1 = Vd)
  output logic [1:0][31:0]       re,
  output logic [1:0][31:0][5:0]  ra,
  input  logic [1:0][31:0][11:0] rd,
  // regfile write (TOMSG result)
  output logic          t_we,
  output logic [6:0]    t_waddr,
  output logic [63:0]   t_wdata
);
  import aigis_pkg::*;
  logic [1:0] p;
  logic [7:0] W;
  logic [2:0] R;             // W/32
  assign W = wcount(p);
  assign R = 3'd1 << (p - 2'd1);

  logic       iss;
  logic [4:0] u;
  logic [2:0] r;
  logic [6:0] w;
  always_comb w = 7'({2'd0, u} + {r, 5'd0});

  typedef enum logic [1:0] {W_IDLE, W_RUN, W_DRAIN, W_WB} wst_t;
  wst_t st;
  logic [3:0] wb;
  always_comb begin
    for (int k = 0; k < 2; k++)
      for (int i = 0; i < 32; i++) begin
        re[k][i] = iss && (i[0] == wbank(w));
        ra[k][i] = w[6:1];
      end
  end

  // data stage
  logic       v1; logic [6:0] w1; logic [4:0] u1; logic [2:0] r1;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) v1 <= 1'b0;
    else v1 <= iss;
  end
  always_ff @(posedge clk) begin w1 <= w; u1 <= u; r1 <= r; end

  logic v2; logic [2:0] r2; logic [4:0] u2;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) v2 <= 1'b0; else v2 <= v1;
  end
  always_ff @(posedge clk) begin r2 <= r1; u2 <= u1; end
  logic [15:0][12:0] acc;
  logic [511:0] mbits;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= W_IDLE; busy <= 1'b0; done <= 1'b0; iss <= 1'b0; t_we <= 1'b0;
      u <= '0; r <= '0; wb <= '0; p <= 2'd1; acc <= '0; mbits <= '0; t_waddr <= '0; t_wdata <= '0;
    end else begin
      done <= 1'b0; t_we <= 1'b0;
      case (st)
        W_IDLE: if (start) begin
          busy <= 1'b1; p <= param; u <= '0; r <= '0; iss <= 1'b1; st <= W_RUN;
        end
        W_RUN: begin
          if (r == R - 3'd1) begin r <= '0; u <= u + 5'd1; end else r <= r + 3'd1;
          if (u == 5'd31 && r == R - 3'd1) begin iss <= 1'b0; st <= W_DRAIN; end
        end
        W_DRAIN: if (!v1) begin st <= W_WB; wb <= '0; end
        W_WB: begin
          t_we <= 1'b1; t_waddr <= 7'(wb); t_wdata <= mbits[64*wb +: 64];
          wb <= wb + 4'd1;
          if (wb + 4'd1 == 4'(bbytes(p) >> 3)) begin st <= W_IDLE; busy <= 1'b0; done <= 1'b1; end
        end
        default: st <= W_IDLE;
      endcase
      // data stage
      if (v1) begin
        begin
          for (int l = 0; l < 16; l++) begin
            logic [11:0] x, vdec, d;
            logic [12:0] f;
            vdec = decompress_c2(rd[1][2*l + wbank(w1)][3:0], p);
            x = subq(rd[0][2*l + wbank(w1)], vdec);
            f = (x >= 12'd1664) ? 13'(x - 12'd1664) : 13'(12'd1664 - x);
            acc[l] <= ((r1 == 3'd0) ? 13'd0 : acc[l]) + f;
          end
        end
      end
      // close message bits one cycle later (acc complete after the r = R-1 word)
      if (v2 && r2 == R - 3'd1) begin
        for (int gr = 0; gr < 16; gr++) begin
          logic [15:0] s;
          logic [3:0]  gsz;
          gsz = 4'd4 >> (p - 2'd1);            // lanes per group: 4, 2, 1
          s = '0;
          for (int l = 0; l < 16; l++) if ((4'(l) / gsz) == 4'(gr)) s = s + {3'd0, acc[l]};
          if (gr < (16 / int'(gsz))) mbits[32*gr + int'(u2)] <= (s < 16'd3329);
        end
      end
    end
  end
endmodule
