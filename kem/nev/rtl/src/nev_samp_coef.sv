// nev_samp_coef: small signed sample value v in [-7, 7] -> canonical residue mod q,
// optionally multiplied by R^-1 mod q (R = 2^16). Purely combinational.
module nev_samp_coef (
  input  logic [1:0]  qsel,
  input  logic        scale,
  input  logic [3:0]  v,      // two's complement
  output logic [11:0] y
);
  import nev_pkg::*;
  logic [2:0]  m;
  logic [11:0] t, q;

  always_comb begin
    m = v[3] ? (3'd0 - v[2:0]) : v[2:0];
    q = qval(qsel);
    t = {9'd0, m};
    if (scale) begin
      case (qsel)
        Q769: case (m)        // R^-1 = 9
          3'd1: t = 12'd9;    3'd2: t = 12'd18;   3'd3: t = 12'd27;   3'd4: t = 12'd36;
          3'd5: t = 12'd45;   3'd6: t = 12'd54;   3'd7: t = 12'd63;   default: t = 12'd0;
        endcase
        Q1409: case (m)       // R^-1 = 1087
          3'd1: t = 12'd1087; 3'd2: t = 12'd765;  3'd3: t = 12'd443;  3'd4: t = 12'd121;
          3'd5: t = 12'd1208; 3'd6: t = 12'd886;  3'd7: t = 12'd564;  default: t = 12'd0;
        endcase
        default: case (m)     // q = 3329, R^-1 = 169
          3'd1: t = 12'd169;  3'd2: t = 12'd338;  3'd3: t = 12'd507;  3'd4: t = 12'd676;
          3'd5: t = 12'd845;  3'd6: t = 12'd1014; 3'd7: t = 12'd1183; default: t = 12'd0;
        endcase
      endcase
    end
    // negative values: q - t (t != 0 since m != 0 for v < 0)
    y = v[3] ? (q - t) : t;
  end
endmodule
