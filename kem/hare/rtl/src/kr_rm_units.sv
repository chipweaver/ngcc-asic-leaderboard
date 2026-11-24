// Combinational coding units of the output stage, kept as separate instances so that area and
// timing can be attributed to KR compression, KR decompression and RM encoding.
//   kr_comp   : 51-bit KR block -> 41-bit message (syndrome + coset leader, reference kr_decode)
//   kr_decomp : 41-bit message -> 51-bit systematic KR codeword (reference kr_encode)
//   rm_encoder: one RS symbol byte -> 128-bit RM(1,7) codeword
module kr_comp import hare_pkg::*; (input logic [50:0] blk, output logic [40:0] msg);
  always_comb msg = kr_dec(blk);
endmodule
module kr_decomp import hare_pkg::*; (input logic [40:0] msg, output logic [50:0] blk);
  always_comb blk = kr_enc(msg);
endmodule
module rm_encoder import hare_pkg::*; (input logic [7:0] sym, output logic [127:0] cw);
  always_comb cw = rm_enc(sym);
endmodule
