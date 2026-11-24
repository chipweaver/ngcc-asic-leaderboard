// nev_hash_ce: SM3 chain engine.
//
// Assembles the padded SM3 message  prefix || body || suffix || 80 00.. || bitlen
// eight bytes per clock into a 64-byte block buffer (double buffer in front of
// the core's W register) and feeds the blocks of one message back to back into
// its SM3 core.  The body is either up to three register segments (fetched
// through the rq_* port, stalled while the source is not yet valid) or the PK
// byte stream (pk_* FIFO port).
//   pre: 0 none, 1 = 02 00, 2 = 04 00
//   suf: 0 none, 1 = 02 00, 2 = 04 00, 3 = 00 00 00 01
//   p64: 64 key bytes precede the message (bit length + 512; initial state
//        is a key midstate supplied by the parent through the core's v_ext)
// Fill rate: one 8-byte word per clock in general; one 32-byte half block per
// clock for 32-byte aligned register body halves (no prefix) and for halves
// that hold only padding / length.  A block is handed to the core in the cycle
// its last word is written when the core is free.
// job_done pulses in the cycle the core's last done is seen (core dig valid).
module nev_hash_ce (
  input  logic         clk,
  input  logic         rst_n,
  // job descriptor (sampled at job_start; accepted when idle or in the job_done cycle)
  input  logic         job_start,
  input  logic         d_p64,
  input  logic [1:0]   d_pre,
  input  logic [1:0]   d_suf,
  input  logic         d_pk,
  input  logic [3:0]   d_src0,
  input  logic [3:0]   d_nw0,
  input  logic [3:0]   d_src1,
  input  logic [3:0]   d_nw1,
  input  logic [3:0]   d_src2,
  input  logic [3:0]   d_nw2,
  input  logic [11:0]  pb,
  output logic         idle,
  output logic         job_done,
  // register body source
  output logic [3:0]   rq_src,
  output logic [2:0]   rq_off,
  input  logic [63:0]  rq_word,
  input  logic [255:0] rq_half,     // 4 words of the source starting at word (rq_off & ~3)
  input  logic         rq_ok,
  // PK stream body source
  input  logic         pk_rdy,
  input  logic [63:0]  pk_word,
  output logic         pk_pop,
  // SM3 core
  output logic         core_start,
  output logic         core_use_ext,
  output logic [511:0] core_blk,
  input  logic         core_busy,
  input  logic         core_done
);

  logic         run;
  logic [1:0]   pre_q, suf_q;
  logic         pk_q;
  logic [3:0]   src_q [3];
  logic [4:0]   cum1_q, cum2_q;
  logic [12:0]  prebl_q, len_q;
  logic [9:0]   nbw_q;
  logic [9:0]   nwl_q;        // index of the last message word (NW-1)
  logic [5:0]   nblk_q;
  logic [15:0]  bitlen_q;
  logic [9:0]   wi;
  logic [15:0]  prev_hi;
  logic [511:0] buf_q;
  logic         buf_full;
  logic         first;
  logic [5:0]   nstarted;

  // ---------------------------------------------------------------- job setup
  logic [12:0] s_bl, s_prebl, s_len, s_tot;
  logic [9:0]  s_nbw;
  logic [5:0]  s_nblk;
  logic [4:0]  s_nwsum;
  always_comb begin
    s_nwsum = {1'b0, d_nw0} + {1'b0, d_nw1} + {1'b0, d_nw2};
    if (d_pk) begin
      s_bl  = {1'b0, pb};
      s_nbw = 10'(({1'b0, pb} + 13'd7) >> 3);
    end else begin
      s_bl  = {5'd0, s_nwsum, 3'b000};
      s_nbw = {5'd0, s_nwsum};
    end
    s_prebl = s_bl + ((d_pre != 2'd0) ? 13'd2 : 13'd0);
    case (d_suf)
      2'd0:    s_len = s_prebl;
      2'd3:    s_len = s_prebl + 13'd4;
      default: s_len = s_prebl + 13'd2;
    endcase
    s_tot  = s_len + 13'd72;
    s_nblk = s_tot[11:6];
  end

  // ---------------------------------------------------------------- body fetch
  logic [5:0]  wi6, segbase, segend, off6;
  logic        need_body, body_ok;
  logic [63:0] cur;
  always_comb begin
    wi6 = wi[5:0];
    if (wi6 < {1'b0, cum1_q}) begin
      rq_src  = src_q[0];
      segbase = 6'd0;
      segend  = {1'b0, cum1_q};
    end else if (wi6 < {1'b0, cum2_q}) begin
      rq_src  = src_q[1];
      segbase = {1'b0, cum1_q};
      segend  = {1'b0, cum2_q};
    end else begin
      rq_src  = src_q[2];
      segbase = {1'b0, cum2_q};
      segend  = nbw_q[5:0];
    end
    off6   = wi6 - segbase;
    rq_off = off6[2:0];
    need_body = (wi < nbw_q);
    if (!need_body)  body_ok = 1'b1;
    else if (pk_q)   body_ok = pk_rdy;
    else             body_ok = rq_ok;
    if (!need_body)  cur = 64'd0;
    else if (pk_q)   cur = pk_word;
    else             cur = rq_word;
  end

  // ---------------------------------------------------------------- message word
  logic [63:0] mword;
  logic [63:0] bodyw;
  logic [12:0] pbase;
  logic [12:0] p;
  logic [12:0] sidx;
  logic        lastw;
  always_comb begin
    pbase = {wi, 3'b000};
    lastw = (wi == nwl_q);
    if (pre_q != 2'd0) bodyw = {cur[47:0], prev_hi};
    else               bodyw = cur;
    for (int i = 0; i < 8; i++) begin
      p    = pbase + 13'(i);
      sidx = p - prebl_q;
      if (pre_q != 2'd0 && p < 13'd2) begin
        if (p == 13'd0) mword[8*i +: 8] = (pre_q == 2'd2) ? 8'h04 : 8'h02;
        else            mword[8*i +: 8] = 8'h00;
      end else if (p < prebl_q) begin
        mword[8*i +: 8] = bodyw[8*i +: 8];
      end else if (p < len_q) begin
        case (suf_q)
          2'd1:    mword[8*i +: 8] = (sidx[1:0] == 2'd0) ? 8'h02 : 8'h00;
          2'd2:    mword[8*i +: 8] = (sidx[1:0] == 2'd0) ? 8'h04 : 8'h00;
          default: mword[8*i +: 8] = (sidx[1:0] == 2'd3) ? 8'h01 : 8'h00;
        endcase
      end else if (p == len_q) begin
        mword[8*i +: 8] = 8'h80;
      end else if (lastw && i == 6) begin
        mword[8*i +: 8] = bitlen_q[15:8];
      end else if (lastw && i == 7) begin
        mword[8*i +: 8] = bitlen_q[7:0];
      end else begin
        mword[8*i +: 8] = 8'h00;
      end
    end
  end

  // ---------------------------------------------------------------- half-block fill
  // A: 4 body words of one register segment, 32-byte aligned, no prefix
  // B: 4 words at or after the 0x80 byte position (padding and bit length only)
  logic         hf_a, hf_b, hf;
  logic [255:0] padh;
  always_comb begin
    hf_a = !pk_q && (pre_q == 2'd0) && (wi[1:0] == 2'b00) && (wi + 10'd4 <= nbw_q) &&
           (off6[1:0] == 2'b00) && (wi6 + 6'd4 <= segend) && rq_ok;
    hf_b = (wi[1:0] == 2'b00) && (pbase >= len_q) && (wi <= nwl_q);
    hf   = hf_a || hf_b;
    padh = '0;
    if (pbase == len_q)        padh[7:0]     = 8'h80;
    if (wi + 10'd3 == nwl_q)   padh[255:240] = {bitlen_q[7:0], bitlen_q[15:8]};
  end

  // ---------------------------------------------------------------- control
  logic         fill, fill1, cs_full, blk_now;
  logic [511:0] buf_nx;
  always_comb begin
    cs_full  = run && buf_full && !core_busy;
    fill     = run && (wi <= nwl_q) && (hf || body_ok) && (!buf_full || cs_full);
    fill1    = fill && !hf;
    blk_now  = fill && (hf ? (wi[2] == 1'b1) : (wi[2:0] == 3'd7));
    buf_nx   = buf_q;
    if (fill) begin
      if (hf) buf_nx[256*wi[2] +: 256] = hf_a ? rq_half : padh;
      else    buf_nx[64*wi[2:0] +: 64] = mword;
    end
    core_start   = cs_full || (run && !buf_full && !core_busy && blk_now);
    core_use_ext = first;
    core_blk     = buf_full ? buf_q : buf_nx;
    pk_pop       = fill1 && need_body && pk_q;
    job_done     = run && core_done && (nstarted == nblk_q);
    idle         = !run;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run      <= 1'b0;
      pre_q    <= 2'd0;
      suf_q    <= 2'd0;
      pk_q     <= 1'b0;
      for (int k = 0; k < 3; k++) src_q[k] <= 4'd0;
      cum1_q   <= 5'd0;
      cum2_q   <= 5'd0;
      prebl_q  <= 13'd0;
      len_q    <= 13'd0;
      nbw_q    <= 10'd0;
      nwl_q    <= 10'd0;
      nblk_q   <= 6'd0;
      bitlen_q <= 16'd0;
      wi       <= 10'd0;
      prev_hi  <= 16'd0;
      buf_q    <= '0;
      buf_full <= 1'b0;
      first    <= 1'b0;
      nstarted <= 6'd0;
    end else begin
      if (run) begin
        if (core_start) begin
          first    <= 1'b0;
          nstarted <= nstarted + 6'd1;
        end
        if (fill) begin
          buf_q <= buf_nx;
          wi    <= wi + (hf ? 10'd4 : 10'd1);
          if (fill1 && need_body) prev_hi <= cur[63:48];
        end
        if (blk_now && !core_start) buf_full <= 1'b1;
        else if (core_start)        buf_full <= 1'b0;
        if (job_done) run <= 1'b0;
      end
      // a new job may be accepted in the cycle the previous one completes
      if (job_start && (!run || job_done)) begin
        run      <= 1'b1;
        pre_q    <= d_pre;
        suf_q    <= d_suf;
        pk_q     <= d_pk;
        src_q[0] <= d_src0;
        src_q[1] <= d_src1;
        src_q[2] <= d_src2;
        cum1_q   <= {1'b0, d_nw0};
        cum2_q   <= {1'b0, d_nw0} + {1'b0, d_nw1};
        prebl_q  <= s_prebl;
        len_q    <= s_len;
        nbw_q    <= s_nbw;
        nwl_q    <= {1'b0, s_nblk, 3'b000} - 10'd1;
        nblk_q   <= s_nblk;
        bitlen_q <= 16'({s_len, 3'b000}) + (d_p64 ? 16'd512 : 16'd0);
        wi       <= 10'd0;
        buf_full <= 1'b0;
        first    <= 1'b1;
        nstarted <= 6'd0;
      end
    end
  end

endmodule
