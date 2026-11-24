// Fixed-weight sampler over an in-order XOF block stream.
//   mode 0 (M1, vect_generate_random_support1): 24-bit big-endian candidates, reject >= thr,
//          reduce mod n, reject duplicates (bitmap test), accept w values; optionally write them to
//          the list; optionally skip candidates to the next multiple of w (3w-byte chunk boundary).
//   mode 1 (M2 generate, vect_generate_random_support2): w little-endian 32-bit words,
//          sup[i] = i + ((rand * (n - i)) >> 32) written to list[base + i].
//   mode 2 (M2 backward pass): for i = w-1 .. 0, if bitmap[sup[i]] is set then sup[i] = i
//          (written back to the list when wback); set bitmap[sup[i]].
// The bitmap is an eight-bank dense memory: bit p lives in word p>>8 = bank (p>>8)&7, addr p>>11.
// flush discards buffered stream bytes (new XOF stream).
module sampler import hare_pkg::*; (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         flush,
  input  logic         start,
  input  logic [1:0]   mode,
  input  logic [8:0]   w,
  input  logic [9:0]   base,
  input  logic         wlist,     // M1: write accepted values to the list
  input  logic         skip,      // M1: skip to the chunk boundary afterwards
  input  logic         wback,     // M2 backward: write replaced values back to the list
  input  logic [1:0]   lvl,
  // XOF stream
  input  logic         in_valid,
  input  logic [255:0] in_data,
  output logic         in_ready,
  // list SRAM
  output logic         l_ce,
  output logic         l_we,
  output logic [9:0]   l_a,
  output logic [17:0]  l_d,
  input  logic [17:0]  l_q,
  // bitmap memory
  output dm_req_t      bm,
  input  dm_q_t        bm_q,
  output logic         busy
);
  logic [17:0] n;
  logic [23:0] thr;
  logic [17:0] mu;
  assign n = p_n(lvl); assign thr = p_thr(lvl); assign mu = p_mu(lvl);

  // ------------------------------------------------------------ byte window
  logic [255:0] b0, b1;
  logic         v0, v1;
  logic [5:0]   ptr;
  logic [6:0]   avail;
  logic [511:0] win;
  logic [31:0]  peek;              // bytes ptr..ptr+3, byte k at [8k+7:8k]
  assign avail = (v0 ? (7'd32 - {1'b0, ptr}) : 7'd0) + (v1 ? 7'd32 : 7'd0);
  assign win   = {b1, b0};
  assign peek  = win[{ptr[4:0], 3'b000} +: 32];
  logic       take;
  logic [2:0] take_n;
  assign in_ready = !v1 && !flush;

  // ------------------------------------------------------------ FSM
  typedef enum logic [3:0] {S_IDLE, S_TAKE, S_MOD1, S_MOD1B, S_MOD2, S_MOD3, S_RD, S_SKIP,
                            S_GEN, S_BLR, S_BRD, S_BTEST, S_BDUP, S_BW2} st_t;
  st_t st;
  logic [8:0]  w_q;
  logic [9:0]  base_q;
  logic        wlist_q, skip_q, wback_q;
  logic [8:0]  cnt;                // accepted (M1) / issued (M2 gen) / remaining index (M2 back)
  logic [8:0]  cpos;               // candidate position within the chunk (M1)
  logic [23:0] cand;
  logic [41:0] prod;
  logic [29:0] bp_lo, bp_hi;       // Barrett partial products (12 x 18)
  logic [18:0] rem;
  logic [17:0] red;   // reduced candidate (combinational, S_MOD3)
  assign red = (rem >= {1'b0, n}) ? rem[17:0] - n : rem[17:0];
  logic [255:0] bword_now;
  logic [17:0] pv;                 // value being inserted
  logic [7:0]  boff;               // bit offset in word
  logic [2:0]  bbank;
  assign bword_now = bm_q[bbank];

  // M2 generate pipeline: g1 (bytes, n-i) -> g2 (16x18 partial products) -> g3 (sum, write)
  logic        g1v, g2v, g3v;
  logic [31:0] g1r;
  logic [17:0] g1nmi;
  logic [8:0]  g1i, g2i, g3i;
  logic [33:0] g2lo, g2hi;
  logic [49:0] g3p;
  logic [8:0]  gissued;

  // candidate value from the window
  logic [23:0] c24;
  logic [31:0] r32;
  assign c24 = {peek[7:0], peek[15:8], peek[23:16]};
  assign r32 = peek;

  always_comb begin
    take = 1'b0; take_n = 3'd0;
    case (st)
      S_TAKE, S_SKIP: if (avail >= 7'd3 && !(st == S_SKIP && cpos == 9'd0)) begin take = 1'b1; take_n = 3'd3; end
      S_GEN:  if (avail >= 7'd4 && gissued != w_q) begin take = 1'b1; take_n = 3'd4; end
      default: ;
    endcase
  end

  // list / bitmap port drive
  logic [17:0] lq_p;
  assign lq_p = l_q;
  always_comb begin
    l_ce = 1'b0; l_we = 1'b0; l_a = '0; l_d = '0;
    bm = '0;
    case (st)
      S_MOD3: begin
        bm.ce[red[10:8]] = 1'b1; bm.a[red[10:8]] = red[17:11];
      end
      // test-and-set: the word is always written back with the bit set (unchanged for a duplicate)
      // and the value always written to list slot cnt (overwritten by the next accepted value when
      // it was a duplicate), so the bit test only steers the FSM, not the memory ports
      S_RD: begin
        bm.ce[bbank] = 1'b1; bm.we[bbank] = 1'b1; bm.a[bbank] = pv[17:11]; bm.d[bbank] = bword_now | (256'd1 << boff);
        if (wlist_q) begin l_ce = 1'b1; l_we = 1'b1; l_a = base_q + {1'b0, cnt}; l_d = pv; end
      end
      S_GEN: if (g3v) begin
        l_ce = 1'b1; l_we = 1'b1; l_a = base_q + {1'b0, g3i}; l_d = {9'd0, g3i} + g3p[49:32];
      end
      S_BLR: begin l_ce = 1'b1; l_a = base_q + {1'b0, cnt}; end
      S_BRD: begin
        bm.ce[lq_p[10:8]] = 1'b1; bm.a[lq_p[10:8]] = lq_p[17:11];
      end
      // set bit sup[i] (unchanged for a duplicate) and read the next element, unconditionally
      S_BTEST: begin
        bm.ce[bbank] = 1'b1; bm.we[bbank] = 1'b1; bm.a[bbank] = pv[17:11]; bm.d[bbank] = bword_now | (256'd1 << boff);
        if (cnt != 9'd0) begin l_ce = 1'b1; l_a = base_q + {1'b0, cnt} - 10'd1; end
      end
      // duplicate: replace by i, read the word holding bit i
      S_BDUP: begin
        if (wback_q) begin l_ce = 1'b1; l_we = 1'b1; l_a = base_q + {1'b0, cnt}; l_d = {9'd0, cnt}; end
        bm.ce[bbank] = 1'b1; bm.a[bbank] = pv[17:11];
      end
      S_BW2: begin
        bm.ce[bbank] = 1'b1; bm.we[bbank] = 1'b1; bm.a[bbank] = pv[17:11]; bm.d[bbank] = bword_now | (256'd1 << boff);
        if (cnt != 9'd0) begin l_ce = 1'b1; l_a = base_q + {1'b0, cnt} - 10'd1; end
      end
      default: ;
    endcase
  end


  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      b0 <= '0; b1 <= '0; v0 <= 1'b0; v1 <= 1'b0; ptr <= '0;
      st <= S_IDLE; busy <= 1'b0; w_q <= '0; base_q <= '0;
      wlist_q <= 1'b0; skip_q <= 1'b0; wback_q <= 1'b0; cnt <= '0; cpos <= '0;
      cand <= '0; prod <= '0; rem <= '0; pv <= '0; boff <= '0; bbank <= 3'b000;
      g1v <= 1'b0; g2v <= 1'b0; g3v <= 1'b0; g1r <= '0; g1i <= '0; g2i <= '0; g3i <= '0; g1nmi <= '0;
      g2lo <= '0; g2hi <= '0; g3p <= '0; gissued <= '0; bp_lo <= '0; bp_hi <= '0;
    end else begin
      // ---------------- byte window maintenance
      if (flush) begin
        v0 <= 1'b0; v1 <= 1'b0; ptr <= '0;
      end else begin : win_upd
        logic [5:0] np;
        logic       nv0, nv1;
        logic [255:0] nb0, nb1;
        np = ptr; nv0 = v0; nv1 = v1; nb0 = b0; nb1 = b1;
        if (take) np = ptr + {3'd0, take_n};
        if (np >= 6'd32) begin nb0 = b1; nv0 = v1; nv1 = 1'b0; np = np - 6'd32; end
        if (in_valid && in_ready) begin
          if (!nv0) begin nb0 = in_data; nv0 = 1'b1; end
          else      begin nb1 = in_data; nv1 = 1'b1; end
        end
        // ptr only meaningful while v0
        b0 <= nb0; b1 <= nb1; v0 <= nv0; v1 <= nv1; ptr <= np;
      end
      // ---------------- control
      case (st)
        S_IDLE: if (start) begin
          busy <= 1'b1; w_q <= w; base_q <= base; wlist_q <= wlist; skip_q <= skip;
          wback_q <= wback; cpos <= '0;
          case (mode)
            2'd0: begin st <= S_TAKE; cnt <= '0; end
            2'd1: begin st <= S_GEN; gissued <= '0; g1v <= 1'b0; g2v <= 1'b0; g3v <= 1'b0; end
            default: begin st <= S_BLR; cnt <= w - 9'd1; end
          endcase
        end
        S_TAKE: if (take) begin
          cpos <= (cpos == w_q - 9'd1) ? 9'd0 : cpos + 9'd1;
          if (c24 < thr) begin cand <= c24; st <= S_MOD1; end
        end
        S_MOD1: begin bp_lo <= cand[11:0] * mu; bp_hi <= cand[23:12] * mu; st <= S_MOD1B; end
        S_MOD1B: begin prod <= ({12'd0, bp_hi} << 12) + {12'd0, bp_lo}; st <= S_MOD2; end
        S_MOD2: begin rem <= cand[18:0] - prod[41:32] * n; st <= S_MOD3; end
        S_MOD3: begin pv <= red; boff <= red[7:0]; bbank <= red[10:8]; st <= S_RD; end
        S_RD: begin
          if (bword_now[boff]) st <= S_TAKE;          // duplicate
          else begin
            cnt <= cnt + 9'd1;
            if (cnt + 9'd1 == w_q) begin
              if (skip_q && cpos != 9'd0) st <= S_SKIP;
              else begin st <= S_IDLE; busy <= 1'b0; end
            end else st <= S_TAKE;
          end
        end
        S_SKIP: begin
          if (cpos == 9'd0) begin st <= S_IDLE; busy <= 1'b0; end
          else if (take) cpos <= (cpos == w_q - 9'd1) ? 9'd0 : cpos + 9'd1;
        end
        S_GEN: begin
          g1v <= take; g1r <= r32; g1i <= gissued; g1nmi <= n - {9'd0, gissued};
          if (take) gissued <= gissued + 9'd1;
          g2v <= g1v; g2i <= g1i; g2lo <= g1r[15:0] * g1nmi; g2hi <= g1r[31:16] * g1nmi;
          g3v <= g2v; g3i <= g2i; g3p <= ({16'd0, g2hi} << 16) + {16'd0, g2lo};
          if (gissued == w_q && !g1v && !g2v && !take && g3v) begin st <= S_IDLE; busy <= 1'b0; end
        end
        S_BLR: st <= S_BRD;
        S_BRD: begin pv <= lq_p; boff <= lq_p[7:0]; bbank <= lq_p[10:8]; st <= S_BTEST; end
        S_BTEST: begin
          if (!(bword_now[boff]) || cnt == w_q - 9'd1) begin
            if (cnt == 9'd0) begin st <= S_IDLE; busy <= 1'b0; end
            else begin cnt <= cnt - 9'd1; st <= S_BRD; end
          end else begin
            pv <= {9'd0, cnt}; boff <= cnt[7:0]; bbank <= {2'b00, cnt[8]}; st <= S_BDUP;
          end
        end
        S_BDUP: st <= S_BW2;
        S_BW2: begin
          if (cnt == 9'd0) begin st <= S_IDLE; busy <= 1'b0; end
          else begin cnt <= cnt - 9'd1; st <= S_BRD; end
        end
        default: st <= S_IDLE;
      endcase
    end
  end
endmodule
