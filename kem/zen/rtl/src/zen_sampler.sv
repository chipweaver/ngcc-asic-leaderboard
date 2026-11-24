// zen_sampler -- polynomial sampler from pseudoXOF digests
//
// Micro-architecture
//   * 16 digest buffers (dbuf), one per xreq/xrsp tag. Per (param,poly) a
//     256-coefficient chunk needs k digests (k = 2,3,4,5,7). The buffers are split into aligned chunk
//     slots: k = 2: 8 slots at 2s; k = 3,4: 4 slots at 4s; k = 5,7: 2 slots at 8s. Chunk q uses slot
//     q mod S, so that at least 8 digests (all SM3 cores) can be in flight, and plane j of a slot is one of
//     8/8/4/4/2/2/2 buffers (small plane multiplexers).
//   * Request engine: chunks are requested in order, one digest per accepted xreq; a chunk's
//     requests start as soon as its slot is free, so the next round's requests overlap with the
//     sampling/writing of the current chunks (sliding window of S chunks in flight).
//   * Responses are accepted every cycle in any order (tag = buffer index).
//   * Sample stage: when every digest of the next chunk (in order) has arrived and the coefficient
//     register is free (or being drained in this cycle), all 256 coefficients of the chunk are
//     computed in one cycle into a 256 x 3-bit coefficient register, and the chunk's slot is freed
//     immediately (so its next digests can be requested while the chunk is written out).
//   * Write stage: 2 polynomial words (32 coefficients) per cycle, one per bank, 8 cycles/chunk.
//     Words 2m and 2m+1 (m = 8q + c) share local address m and are in opposite banks
//     (bank(2m) = ^m).
//   * t0 / t0_odd (poly f): folded XOR of the coefficient LSBs, accumulated per chunk at sample
//     time; t0_odd = parity of all LSBs (= parity of t0).
module zen_sampler (
  input  logic                clk,
  input  logic                rst_n,
  input  logic                start,
  input  logic [1:0]          param,
  input  logic [1:0]          poly,      // 0 f, 1 g, 2 s, 3 e
  input  logic [7:0]          nonce,     // broadcast to zen_hash_sys at top level (unused here)
  output logic                done,
  output logic                active,
  output logic [6:0]          wpairs,    // word pairs written so far in this operation
  // polynomial write master
  output logic [1:0]          wr_en,
  output logic [1:0][5:0]     wr_addr,
  output logic [1:0][255:0]   wr_data,
  // XOF request / response port (zen_hash_sys)
  output logic                xreq_valid,
  input  logic                xreq_ready,
  output logic [15:0]         xreq_ctr,
  output logic [3:0]          xreq_tag,
  input  logic                xrsp_valid,
  input  logic [3:0]          xrsp_tag,
  input  logic [255:0]        xrsp_digest,
  // GF(2) check (poly f)
  output logic [511:0]        t0,
  output logic                t0_odd
);

  // ---------------------------------------------------------------- mode decode
  localparam logic [2:0] M_CBD1   = 3'd0;  // cbd1                 k=2
  localparam logic [2:0] M_C1T18  = 3'd1;  // cbd1 + tenary1_8     k=5
  localparam logic [2:0] M_C1T332 = 3'd2;  // cbd1 + tenary3_32    k=7
  localparam logic [2:0] M_CBD2   = 3'd3;  // cbd2                 k=4
  localparam logic [2:0] M_T18    = 3'd4;  // tenary1_8            k=3
  localparam logic [2:0] M_T316   = 3'd5;  // tenary3_16           k=4
  localparam logic [2:0] M_T332   = 3'd6;  // tenary3_32           k=5

  // decoded from param/poly at start and held in registers for the whole operation
  logic [2:0] mode_c, mode;
  logic [2:0] kmax_c, kmax;   // k-1
  logic [3:0] nchunk;         // N/256 = 2,4,8
  logic [1:0] prm;            // registered param
  logic       isf;            // registered (poly == f)

  always_comb begin
    case (param)
      2'd0: begin
        case (poly)
          2'd0:    mode_c = M_CBD1;
          2'd1:    mode_c = M_C1T18;
          2'd2:    mode_c = M_C1T332;
          default: mode_c = M_CBD2;
        endcase
      end
      2'd1: begin
        case (poly)
          2'd0:    mode_c = M_T18;
          2'd1:    mode_c = M_T316;
          default: mode_c = M_CBD1;
        endcase
      end
      default: begin
        case (poly)
          2'd0, 2'd1: mode_c = M_T332;
          default:    mode_c = M_T18;
        endcase
      end
    endcase
  end

  always_comb begin
    case (mode_c)
      M_CBD1:   kmax_c = 3'd1;
      M_C1T18:  kmax_c = 3'd4;
      M_C1T332: kmax_c = 3'd6;
      M_CBD2:   kmax_c = 3'd3;
      M_T18:    kmax_c = 3'd2;
      M_T316:   kmax_c = 3'd3;
      default:  kmax_c = 3'd4;
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mode   <= M_CBD1;
      kmax   <= 3'd1;
      nchunk <= 4'd2;
      prm    <= 2'd0;
      isf    <= 1'b0;
    end else if (start) begin
      mode   <= mode_c;
      kmax   <= kmax_c;
      nchunk <= (param == 2'd0) ? 4'd2 : ((param == 2'd1) ? 4'd4 : 4'd8);
      prm    <= param;
      isf    <= (poly == 2'd0);
    end
  end

  // slot index of a chunk (q mod S) and first buffer of a slot (aligned: 2s / 4s / 8s)
  function automatic logic [2:0] slot_of(input logic [2:0] q, input logic [2:0] km);
    case (km)
      3'd1:        slot_of = q;                   // k=2, S=8
      3'd2, 3'd3:  slot_of = {1'b0, q[1:0]};      // k=3,4, S=4
      default:     slot_of = {2'b0, q[0]};        // k=5,7, S=2
    endcase
  endfunction

  function automatic logic [3:0] base_of(input logic [2:0] sl, input logic [2:0] km);
    case (km)
      3'd1:        base_of = {sl, 1'b0};
      3'd2, 3'd3:  base_of = {sl[1:0], 2'b00};
      default:     base_of = {sl[0], 3'b000};
    endcase
  endfunction

  // ---------------------------------------------------------------- control state
  logic        run;
  logic [3:0]  rq;          // next chunk to request (0..nchunk)
  logic [2:0]  rj;          // next plane of chunk rq
  logic [7:0]  slot_busy;   // chunk slot holds requested / arriving digests
  logic [15:0] dvalid;      // digest buffer filled
  logic [3:0]  sq;          // next chunk to sample (0..nchunk)
  logic        wbusy;       // coefficient register holds a chunk being written
  logic [2:0]  wq;          // chunk index in the coefficient register
  logic [2:0]  wc;          // word pair being written (0..7)

  assign active = run;

  // ---------------------------------------------------------------- request engine
  logic [2:0]  rslot;
  logic        req_fire;
  logic [15:0] ctr_c;

  assign rslot      = slot_of(rq[2:0], kmax);
  assign xreq_valid = run && (rq < nchunk) && ((rj != 3'd0) || !slot_busy[rslot]);
  assign req_fire   = xreq_valid && xreq_ready;
  assign xreq_tag   = base_of(rslot, kmax) + {1'b0, rj};

  // counter (1-based) of plane rj of chunk rq
  always_comb begin
    case (mode)
      M_CBD1:   ctr_c = {11'd0, rq[2:0], 1'b0} + {13'd0, rj} + 16'd1;          // 2q+j+1
      M_C1T18,
      M_C1T332: begin
        if (rj < 3'd2) ctr_c = {11'd0, rq[2:0], 1'b0} + {13'd0, rj} + 16'd1;   // 2q+j+1
        else           ctr_c = {12'd0, rj, 1'b0} + {13'd0, rq[2:0]} + 16'd1;   // 2j+q+1
      end
      M_CBD2:   ctr_c = {11'd0, rq[2:0], 2'b00} + {13'd0, rj} + 16'd1;         // 4q+j+1
      default: begin                                                           // j*NC+q+1
        case (prm)
          2'd0:    ctr_c = {12'd0, rj, 1'b0}   + {13'd0, rq[2:0]} + 16'd1;
          2'd1:    ctr_c = {11'd0, rj, 2'b00}  + {13'd0, rq[2:0]} + 16'd1;
          default: ctr_c = {10'd0, rj, 3'b000} + {13'd0, rq[2:0]} + 16'd1;
        endcase
      end
    endcase
  end
  assign xreq_ctr = ctr_c;

  // ---------------------------------------------------------------- digest buffers
  // Stored in "bit order" of the model: L[8b+r] = bit r of digest byte b,
  // digest byte b = xrsp_digest[255-8b -: 8].
  logic [15:0][255:0] dbuf;
  logic [255:0]      dig_l;

  always_comb begin
    for (int b = 0; b < 32; b++) begin
      dig_l[8*b +: 8] = xrsp_digest[255 - 8*b -: 8];
    end
  end

  always_ff @(posedge clk) begin
    if (xrsp_valid) dbuf[xrsp_tag] <= dig_l;
  end

  // ---------------------------------------------------------------- sample stage
  logic [2:0]   sslot;
  logic [3:0]   sbase;
  logic [15:0]  smask;
  logic         sready;
  logic         wfree;
  logic         load;
  logic [255:0] B0, B1, B2, B3, B4, B5, B6;

  assign sslot  = slot_of(sq[2:0], kmax);
  assign sbase  = base_of(sslot, kmax);
  assign smask  = ((16'd2 << kmax) - 16'd1) << sbase;
  assign sready = run && (sq < nchunk) && ((dvalid & smask) == smask);
  assign wfree  = !wbusy || (wc == 3'd7);
  assign load   = sready && wfree;

  // plane select: B_j = dbuf[sbase + j]
  // sbase is a multiple of 2 (k=2), 4 (k=3,4) or 8 (k=5,7); planes j>=2 exist only for k>=3, j>=4 for k>=5
  always_comb begin
    B0 = dbuf[{sbase[3:1], 1'b0}];
    B1 = dbuf[{sbase[3:1], 1'b1}];
    B2 = dbuf[{sbase[3:2], 2'b10}];
    B3 = dbuf[{sbase[3:2], 2'b11}];
    B4 = dbuf[{sbase[3], 3'b100}];
    B5 = dbuf[{sbase[3], 3'b101}];
    B6 = dbuf[{sbase[3], 3'b110}];
  end

  // per-coefficient sampling: value = p0 + p1 - n0 - n1 (range -2..2)
  logic [255:0][2:0] cval;
  logic [255:0]      clsb;

  genvar gi;
  generate
    for (gi = 0; gi < 256; gi++) begin : g_coef
      // cbd1: bits 2i, 2i+1 of d0 || d1
      localparam int C1I = 2 * (gi % 128);
      // cbd2: bits 4i..4i+3 of d0 || d1 || d2 || d3
      localparam int C2D = gi / 64;
      localparam int C2I = 4 * (gi % 64);
      logic c1p, c1n;
      logic [3:0] c2;
      logic p0, n0, p1, n1;
      logic [1:0] pos, neg;

      assign c1p = (gi < 128) ? B0[C1I] : B1[C1I];
      assign c1n = (gi < 128) ? B0[C1I + 1] : B1[C1I + 1];
      if (C2D == 0) begin : g_d0
        assign c2 = B0[C2I +: 4];
      end else if (C2D == 1) begin : g_d1
        assign c2 = B1[C2I +: 4];
      end else if (C2D == 2) begin : g_d2
        assign c2 = B2[C2I +: 4];
      end else begin : g_d3
        assign c2 = B3[C2I +: 4];
      end

      always_comb begin
        p0 = 1'b0; n0 = 1'b0; p1 = 1'b0; n1 = 1'b0;
        case (mode)
          M_CBD1: begin
            p0 = c1p; n0 = c1n;
          end
          M_C1T18: begin
            p0 = c1p; n0 = c1n;
            p1 = B2[gi] & B4[gi];
            n1 = B3[gi] & B4[gi];
          end
          M_C1T332: begin
            p0 = c1p; n0 = c1n;
            p1 = B2[gi] & B3[gi] & B6[gi];
            n1 = B4[gi] & B5[gi] & B6[gi];
          end
          M_CBD2: begin
            p0 = c2[0]; p1 = c2[1]; n0 = c2[2]; n1 = c2[3];
          end
          M_T18: begin
            p0 = B0[gi] & B2[gi];
            n0 = B1[gi] & B2[gi];
          end
          M_T316: begin
            p0 = B0[gi] & B1[gi];
            n0 = B2[gi] & B3[gi];
          end
          default: begin  // M_T332
            p0 = B0[gi] & B1[gi] & B4[gi];
            n0 = B2[gi] & B3[gi] & B4[gi];
          end
        endcase
      end

      assign pos = {1'b0, p0} + {1'b0, p1};
      assign neg = {1'b0, n0} + {1'b0, n1};
      assign cval[gi] = {1'b0, pos} - {1'b0, neg};
      assign clsb[gi] = p0 ^ p1 ^ n0 ^ n1;
    end
  endgenerate

  // t0 contribution of the chunk being sampled
  logic [255:0] lsb_fold;
  logic         lsb_par;
  assign lsb_fold = (prm == 2'd0) ? {128'd0, clsb[127:0] ^ clsb[255:128]} : clsb;
  assign lsb_par  = ^clsb;

  // ---------------------------------------------------------------- coefficient register / write
  logic [255:0][2:0] creg;
  logic [5:0]        wm;
  logic [31:0][2:0]  pair;
  logic [255:0]      word_lo, word_hi;   // words 2m, 2m+1

  always_ff @(posedge clk) begin
    if (load) creg <= cval;
  end

  assign wm = {wq, wc};

  always_comb begin
    for (int j = 0; j < 32; j++) pair[j] = creg[{wc, 5'd0} + j];
    for (int k = 0; k < 16; k++) begin
      word_lo[16*k +: 16] = {{13{pair[k][2]}}, pair[k]};
      word_hi[16*k +: 16] = {{13{pair[16+k][2]}}, pair[16+k]};
    end
  end

  assign wr_en      = {2{wbusy}};
  assign wr_addr[0] = wm;
  assign wr_addr[1] = wm;
  assign wr_data[0] = (^wm) ? word_hi : word_lo;
  assign wr_data[1] = (^wm) ? word_lo : word_hi;

  // operation completes in the cycle of the last write pair
  logic finish;
  assign finish = wbusy && (wc == 3'd7) && (sq == nchunk) && run;
  assign done   = finish;

  // ---------------------------------------------------------------- sequential control
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run       <= 1'b0;
      rq        <= 4'd0;
      rj        <= 3'd0;
      slot_busy <= 8'd0;
      dvalid    <= 16'd0;
      sq        <= 4'd0;
      wbusy     <= 1'b0;
      wq        <= 3'd0;
      wc        <= 3'd0;
      t0_odd    <= 1'b0;
      wpairs    <= 7'd0;
    end else if (start) begin
      wpairs    <= 7'd0;
      run       <= 1'b1;
      rq        <= 4'd0;
      rj        <= 3'd0;
      slot_busy <= 8'd0;
      dvalid    <= 16'd0;
      sq        <= 4'd0;
      wbusy     <= 1'b0;
      wq        <= 3'd0;
      wc        <= 3'd0;
      t0_odd    <= 1'b0;
    end else begin
      if (finish) run <= 1'b0;
      if (wbusy) wpairs <= wpairs + 7'd1;

      // request engine
      if (req_fire) begin
        if (rj == kmax) begin
          rj <= 3'd0;
          rq <= rq + 4'd1;
        end else begin
          rj <= rj + 3'd1;
        end
      end

      // slot occupancy (set on first request of a chunk, cleared at sampling)
      for (int s = 0; s < 8; s++) begin
        if (req_fire && (rj == 3'd0) && (rslot == s))
          slot_busy[s] <= 1'b1;
        else if (load && (sslot == s))
          slot_busy[s] <= 1'b0;
      end

      // digest buffer valid flags
      for (int b = 0; b < 16; b++) begin
        if (xrsp_valid && (xrsp_tag == b))
          dvalid[b] <= 1'b1;
        else if (load && smask[b])
          dvalid[b] <= 1'b0;
      end

      // sample / write
      if (load) begin
        sq    <= sq + 4'd1;
        wq    <= sq[2:0];
        wc    <= 3'd0;
        wbusy <= 1'b1;
        if (isf) t0_odd <= t0_odd ^ lsb_par;
      end else if (wbusy) begin
        wc <= wc + 3'd1;
        if (wc == 3'd7) wbusy <= 1'b0;
      end
    end
  end

  // t0 accumulation (datapath, cleared at start)
  always_ff @(posedge clk) begin
    if (start) begin
      t0 <= 512'd0;
    end else if (load && isf) begin
      if ((prm == 2'd2) && sq[0])
        t0[511:256] <= t0[511:256] ^ lsb_fold;
      else
        t0[255:0]   <= t0[255:0] ^ lsb_fold;
    end
  end

endmodule
