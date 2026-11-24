// zen_fci : fused combine / compress / FO compare behind the last INTT_ENC pass.
// For every coefficient i of the two output words (x' from the fused last GS layer, |x'| < q):
//   v = x' + L[e_i, m_i] reduced to [0, q),  L[e, m] = (e + 385 m) * MONT * 2^-16 = (e + 385 m) mod q
//   ct_i = ((v * 10908 + 16362) >> 15) & 255      (= poly_compress: (((v<<8)+384)*10908) >> 23)
// e_i from the e polynomial word read in step with the stage-4 output, m_i = mbits[i mod N4].
// cmp = 0: the 16 bytes of each word are written to the CT object (word aa through port A, word ab
// through port B). cmp = 1: the CT words are read (ports A/B) and compared; mismatch accumulates.
// Pipeline: in_v (s) -> bytes registered (s+1) -> CT write (s+1) / compare (read issued at s, data s+1).
module zen_fci (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              start,
  input  logic [1:0]        param,
  input  logic              cmp,
  input  logic [511:0]      mbits,
  input  logic              in_v,
  input  logic [255:0]      in_wa,
  input  logic [255:0]      in_wb,
  input  logic [6:0]        in_aa,
  input  logic [6:0]        in_ab,
  input  logic [1:0][255:0] e_data,
  output logic              cta_en, cta_we,
  output logic [6:0]        cta_addr,
  output logic [127:0]      cta_wdata,
  input  logic [127:0]      cta_rdata,
  output logic              ctb_en, ctb_we,
  output logic [6:0]        ctb_addr,
  output logic [127:0]      ctb_wdata,
  input  logic [127:0]      ctb_rdata,
  output logic              pair_done,
  output logic              mismatch,
  output logic              idle
);
  function automatic logic [7:0] fcomp(input logic [15:0] x, input logic [15:0] e, input logic m);
    logic signed [11:0] v, lv;
    logic [9:0]  vc;
    logic [23:0] pr;
    lv = $signed(e[11:0]) + (m ? 12'sd385 : 12'sd0);      // L[e,m] in [0, q)
    if (lv < 0) lv = lv + 12'sd769;
    v = $signed(x[11:0]) + lv;
    if (v < 0)                 v = v + 12'sd769;
    else if (v >= 12'sd769)    v = v - 12'sd769;
    vc = v[9:0];
    pr = {14'd0, vc} * 24'd10908 + 24'd16362;
    return pr[22:15];
  endfunction

  logic [255:0] ewa, ewb;
  assign ewa = (^in_aa) ? e_data[1] : e_data[0];
  assign ewb = (^in_aa) ? e_data[0] : e_data[1];

  logic [15:0] mwa, mwb;
  always_comb begin
    case (param)
      2'd0:    begin mwa = mbits[16*in_aa[2:0] +: 16]; mwb = mbits[16*in_ab[2:0] +: 16]; end
      2'd1:    begin mwa = mbits[16*in_aa[3:0] +: 16]; mwb = mbits[16*in_ab[3:0] +: 16]; end
      default: begin mwa = mbits[16*in_aa[4:0] +: 16]; mwb = mbits[16*in_ab[4:0] +: 16]; end
    endcase
  end

  logic [127:0] ba, bb;
  always_comb begin
    for (int k = 0; k < 16; k++) begin
      ba[8*k +: 8] = fcomp(in_wa[16*k +: 16], ewa[16*k +: 16], mwa[k]);
      bb[8*k +: 8] = fcomp(in_wb[16*k +: 16], ewb[16*k +: 16], mwb[k]);
    end
  end

  logic         v1;
  logic [127:0] ba1, bb1;
  logic [6:0]   aa1, ab1;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) v1 <= 1'b0;
    else        v1 <= in_v;
  end
  always_ff @(posedge clk) begin
    ba1 <= ba; bb1 <= bb; aa1 <= in_aa; ab1 <= in_ab;
  end

  always_comb begin
    if (cmp) begin
      cta_en = in_v; cta_we = 1'b0; cta_addr = in_aa; cta_wdata = '0;
      ctb_en = in_v; ctb_we = 1'b0; ctb_addr = in_ab; ctb_wdata = '0;
    end else begin
      cta_en = v1; cta_we = v1; cta_addr = aa1; cta_wdata = ba1;
      ctb_en = v1; ctb_we = v1; ctb_addr = ab1; ctb_wdata = bb1;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)     mismatch <= 1'b0;
    else if (start) mismatch <= 1'b0;
    else if (v1 && cmp && ((cta_rdata != ba1) || (ctb_rdata != bb1))) mismatch <= 1'b1;
  end
  assign pair_done = v1;
  assign idle      = !in_v && !v1;
endmodule
