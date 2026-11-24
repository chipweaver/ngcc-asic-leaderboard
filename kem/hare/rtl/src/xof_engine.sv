// Seed XOF engine: pseudoXOF(tag || seed) with 4 SM3 cores (two rounds per cycle, 33 cycles per block).
// Block i (ct = i+1) = SM3(tag || seed[0..S-1] || BE32(ct)); core c computes blocks c, c+4, ...
// For S = 64 (HARE-512) the first 64-byte block (tag || seed[0..62]) is common to every ct and
// is compressed once by core 0 before the tail blocks.
// Blocks are delivered in order on out_* (valid/ready). nblk = 0 means unlimited (until stop).
module xof_engine import hare_pkg::*; (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,
  input  logic         stop,
  input  logic [7:0]   tag,
  input  logic [511:0] seed,     // byte j = seed[8j+7:8j]
  input  logic [1:0]   lvl,
  input  logic [10:0]  nblk,
  output logic         out_valid,
  output logic [255:0] out_data, // byte j of the block = out_data[8j+7:8j]
  input  logic         out_ready,
  output logic         busy
);
  localparam logic [255:0] IV = 256'h7380166F4914B2B9172442D7DA8A0600A96F30BC163138AAE38DEE4DB0FB0E4E;

  logic [7:0]   tag_q;
  logic [511:0] seed_q;
  logic [1:0]   lvl_q;
  logic [10:0]  nblk_q;
  logic [255:0] vp;
  logic         pre_done;
  logic         pre_run;
  logic         pre_started;

  logic [3:0]   c_start, c_busy, c_done;
  logic [255:0] c_vout [4];
  logic [511:0] c_blk  [4];
  logic [10:0]  bi     [4];
  logic [3:0]   pend, ofull;
  logic [3:0]   c_start_q;
  logic [255:0] oreg   [4];
  logic [1:0]   rd_ptr;
  logic [10:0]  ndeliv;

  // ------------------------------------------------------------------ block builder
  function automatic [511:0] build(input [1:0] l, input [7:0] t, input [511:0] sd,
                                   input [31:0] ct, input pre);
    logic [7:0] by [64];
    logic [6:0] s;
    logic [63:0] bitlen;
    integer b;
    s = p_s(l);
    for (b = 0; b < 64; b = b + 1) by[b] = 8'h00;
    if (s == 7'd64) begin
      if (pre) begin
        by[0] = t;
        for (b = 1; b < 64; b = b + 1) by[b] = sd[8*(b-1) +: 8];
      end else begin
        bitlen = 64'd552;  // (1+64+4)*8
        by[0] = sd[8*63 +: 8];
        by[1] = ct[31:24]; by[2] = ct[23:16]; by[3] = ct[15:8]; by[4] = ct[7:0];
        by[5] = 8'h80;
        for (b = 0; b < 8; b = b + 1) by[56+b] = bitlen[63-8*b -: 8];
      end
    end else begin
      bitlen = ({57'd0, s} + 64'd5) << 3;
      by[0] = t;
      for (b = 1; b < 64; b = b + 1) begin
        if (b <= s)            by[b] = sd[8*(b-1) +: 8];
        else if (b == s + 1)   by[b] = ct[31:24];
        else if (b == s + 2)   by[b] = ct[23:16];
        else if (b == s + 3)   by[b] = ct[15:8];
        else if (b == s + 4)   by[b] = ct[7:0];
        else if (b == s + 5)   by[b] = 8'h80;
      end
      for (b = 0; b < 8; b = b + 1) by[56+b] = bitlen[63-8*b -: 8];
    end
    for (b = 0; b < 64; b = b + 1) build[511-8*b -: 8] = by[b];
  endfunction

  function automatic [255:0] digest_bytes(input [255:0] v);
    // SM3 digest byte j = V[j/4] >> (24 - 8*(j%4)); return little-endian byte vector
    integer b;
    for (b = 0; b < 32; b = b + 1) digest_bytes[8*b +: 8] = v[255-8*b -: 8];
  endfunction

  logic [3:0] avail;
  genvar g;
  generate
    for (g = 0; g < 4; g = g + 1) begin : g_core
      assign avail[g] = pre_done && !pend[g] && !c_busy[g] && !c_start_q[g] && !c_done[g] &&
                        ((nblk_q == 11'd0) || (bi[g] < nblk_q));
      sm3_core2 u_core (.clk, .rst_n, .start(c_start[g]),
                       .vin((g == 0 && pre_run) ? IV : vp), .blk(c_blk[g]), .abort(stop),
                       .busy(c_busy[g]), .done(c_done[g]), .vout(c_vout[g]));
    end
  endgenerate
  integer k;
  integer kc;
  always_comb begin
    for (kc = 0; kc < 4; kc = kc + 1) begin
      c_blk[kc]   = build(lvl_q, tag_q, seed_q, {21'd0, bi[kc]} + 32'd1, (kc == 0) && pre_run);
      c_start[kc] = busy && ((kc == 0 && pre_run && !pre_started) ? 1'b1 :
                            (!pre_run && avail[kc]));
    end
  end

  assign out_valid = busy && ofull[rd_ptr];
  assign out_data  = oreg[rd_ptr];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy <= 1'b0; tag_q <= '0; seed_q <= '0; lvl_q <= '0; nblk_q <= '0; vp <= IV;
      pre_done <= 1'b0; pre_run <= 1'b0; pre_started <= 1'b0; rd_ptr <= '0; ndeliv <= '0; c_start_q <= '0;
      pend <= '0; ofull <= '0;
      for (k = 0; k < 4; k = k + 1) begin bi[k] <= '0; oreg[k] <= '0; end
    end else begin
      c_start_q <= c_start;
      if (start) begin
        busy <= 1'b1; tag_q <= tag; seed_q <= seed; lvl_q <= lvl; nblk_q <= nblk;
        rd_ptr <= '0; ndeliv <= '0; pend <= '0; ofull <= '0; vp <= IV;
        pre_run  <= (p_s(lvl) == 7'd64);
        pre_started <= 1'b0;
        pre_done <= (p_s(lvl) != 7'd64);
        for (k = 0; k < 4; k = k + 1) bi[k] <= k[10:0];
      end else if (stop) begin
        busy <= 1'b0; pend <= '0; ofull <= '0; pre_run <= 1'b0;
      end else if (busy) begin
        if (c_start[0] && pre_run) pre_started <= 1'b1;
        // prefix completion
        if (pre_run && c_done[0]) begin
          vp <= c_vout[0]; pre_run <= 1'b0; pre_done <= 1'b1;
        end
        for (k = 0; k < 4; k = k + 1) begin
          if (c_start[k] && !pre_run) bi[k] <= bi[k] + 11'd4;
          // a finished tail block goes to its output register, or waits in the core
          if (!(k == 0 && pre_run) && (c_done[k] || pend[k])) begin
            if (!ofull[k] || (out_ready && out_valid && rd_ptr == k[1:0])) begin
              oreg[k]  <= digest_bytes(c_vout[k]);
              ofull[k] <= 1'b1;
              pend[k]  <= 1'b0;
            end else begin
              pend[k] <= 1'b1;
            end
          end
        end
        if (out_valid && out_ready) begin
          // clear unless refilled in the same cycle (handled above by priority of the set)
          if (!((c_done[rd_ptr] || pend[rd_ptr]) && !(rd_ptr == 2'd0 && pre_run)))
            ofull[rd_ptr] <= 1'b0;
          rd_ptr <= rd_ptr + 2'd1;
          ndeliv <= ndeliv + 11'd1;
          if (nblk_q != 11'd0 && ndeliv + 11'd1 == nblk_q) busy <= 1'b0;
        end
      end
    end
  end
endmodule
