// zen_pku : pack / unpack between the object memories (MEM_PK, MEM_SK) and a polynomial slot,
// plus F2 load/store. Formats are bit-exact to the ZEN pk / sk encodings and the F2 little-endian
// byte layout.
//
// op: 0 PK_UNPACK_PK  MEM_PK[0..]        -> slot (canonical 0..768)
//     1 PK_UNPACK_SK  MEM_SK[pkoff..]    -> slot
//     2 PK_PACK       slot -> MEM_PK[0..] and MEM_SK[pkoff..] (byte-masked last SK word)
//     3 SKF_UNPACK    MEM_SK[0:fpack)    -> slot (10-bit unsigned)
//     4 SKF_PACK      slot -> MEM_SK[0:fpack)
//     5 F2_LOAD       MEM_SK[fpack..]    -> zen_gf2 B (b_wr_*)
//     6 F2_STORE      f2_in              -> MEM_SK[fpack..]
//
// Micro-architecture summary
//  * Unpack: words are read at 3 per 4 cycles; one 48-bit base-769 block value T per cycle enters
//    an unrolled chain of four pipelined constant dividers (48->39->29->20->10 bit), giving the 5
//    digits of one block per cycle (5 coefficients/cycle). Digits are aligned, written into a
//    32-coefficient assembly buffer and every completed 16-coefficient half is written to the slot.
//  * PK_PACK: slot words (16 coefficients) feed a 4-word FIFO; a 5-coefficient window is extracted
//    per cycle, a 4-stage Horner pipeline forms T, groups of 4 T become 3 64-bit words in a small
//    output queue that writes MEM_PK and MEM_SK in the same cycle (1 word/cycle).
//  * SKF: 2 slot words (both banks, same local address) <-> 5 MEM_SK words (bit-plane transpose).
module zen_pku (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              start,
  input  logic [1:0]        param,
  input  logic [2:0]        op,
  output logic              done,
  output logic              active,
  // polynomial slot read master
  output logic [1:0]        rd_en,
  input  logic              rd_gnt,       // read granted this cycle (shared slot)
  input  logic [7:0]        rd_limit,     // PK_PACK: words of the slot already written (streaming from basemul_mq)
  output logic [1:0][5:0]   rd_addr,
  input  logic [1:0][255:0] rd_data,
  // polynomial slot write master
  output logic [1:0]        wr_en,
  output logic [1:0][5:0]   wr_addr,
  output logic [1:0][255:0] wr_data,
  // MEM_PK object port
  output logic              pk_en,
  output logic              pk_we,
  output logic [9:0]        pk_addr,
  output logic [63:0]       pk_wdata,
  output logic [7:0]        pk_wmask,
  input  logic [63:0]       pk_rdata,
  // MEM_SK object port
  output logic              sk_en,
  output logic              sk_we,
  output logic [9:0]        sk_addr,
  output logic [63:0]       sk_wdata,
  output logic [7:0]        sk_wmask,
  input  logic [63:0]       sk_rdata,
  // F2
  input  logic [511:0]      f2_in,
  output logic              b_wr_en,
  output logic [2:0]        b_wr_idx,
  output logic [63:0]       b_wr_data,
  // streaming hash support
  output logic [9:0]        pk_words_done
);

  localparam [2:0] OP_UNPK = 3'd0, OP_UNSK = 3'd1, OP_PACK = 3'd2, OP_SKFU = 3'd3,
                   OP_SKFP = 3'd4, OP_F2L  = 3'd5, OP_F2S  = 3'd6;

  // unpack read tags
  localparam [2:0] TG_NONE = 3'd0, TG_LO0 = 3'd1, TG_LO1 = 3'd2, TG_LO2 = 3'd3,
                   TG_HI   = 3'd4, TG_S128 = 3'd5, TG_S256 = 3'd6, TG_S512 = 3'd7;

  // ------------------------------------------------------------------ control / config
  logic       run_q, done_q;
  logic [2:0] op_q;
  logic [1:0] p_q;
  logic       fin;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run_q <= 1'b0;
      done_q <= 1'b0;
      op_q <= 3'd0;
      p_q <= 2'd0;
    end else begin
      done_q <= run_q & fin & ~done_q;
      if (start) begin
        run_q <= 1'b1;
        op_q <= op;
        p_q <= param;
      end else if (done_q) begin
        run_q <= 1'b0;
      end
    end
  end

  assign done   = done_q;
  assign active = run_q;

  // per-parameter constants
  logic [7:0] c_w;         // slot words W
  logic [6:0] c_ngrp;      // full pk groups (4 blocks / 3 words)
  logic [8:0] c_nblk;      // pack blocks incl. tail S block
  logic [9:0] c_pkw;       // MEM_PK words
  logic [9:0] c_pkoff;     // pkoff / 8
  logic [9:0] c_fpk;       // fpack / 8
  logic [6:0] c_nchk;      // SKF chunks of 32 coefficients
  logic [9:0] c_skfw;      // SKF MEM_SK words = 5 * chunks
  logic [3:0] c_f2w;       // f2 words
  logic [7:0] c_lmask;     // byte mask of last pk word in MEM_SK

  always_comb begin
    case (p_q)
      2'd0: begin
        c_w = 8'd32;   c_ngrp = 7'd25;  c_nblk = 9'd103; c_pkw = 10'd77;  c_pkoff = 10'd82;
        c_fpk = 10'd80;  c_nchk = 7'd16; c_skfw = 10'd80;  c_f2w = 4'd2; c_lmask = 8'h7F;
      end
      2'd1: begin
        c_w = 8'd64;   c_ngrp = 7'd51;  c_nblk = 9'd205; c_pkw = 10'd154; c_pkoff = 10'd164;
        c_fpk = 10'd160; c_nchk = 7'd32; c_skfw = 10'd160; c_f2w = 4'd4; c_lmask = 8'h1F;
      end
      default: begin
        c_w = 8'd128;  c_ngrp = 7'd102; c_nblk = 9'd410; c_pkw = 10'd308; c_pkoff = 10'd328;
        c_fpk = 10'd320; c_nchk = 7'd64; c_skfw = 10'd320; c_f2w = 4'd8; c_lmask = 8'h03;
      end
    endcase
  end

  logic op_un, op_pack, op_skfu, op_skfp, op_f2l, op_f2s;
  assign op_un   = run_q & ((op_q == OP_UNPK) | (op_q == OP_UNSK));
  assign op_pack = run_q & (op_q == OP_PACK);
  assign op_skfu = run_q & (op_q == OP_SKFU);
  assign op_skfp = run_q & (op_q == OP_SKFP);
  assign op_f2l  = run_q & (op_q == OP_F2L);
  assign op_f2s  = run_q & (op_q == OP_F2S);

  // ================================================================== PK unpack
  // ---- read issue sequencer
  logic [6:0] ug_q;
  logic [1:0] uph_q;
  logic       utail_q, uend_q;
  logic [9:0] uwi_q;
  logic       u_rd, u_last;
  logic [2:0] u_tag, utag_q;

  always_comb begin
    u_rd = 1'b0;
    u_tag = TG_NONE;
    u_last = 1'b0;
    if (op_un && !uend_q) begin
      if (!utail_q) begin
        case (uph_q)
          2'd0: begin u_rd = 1'b1; u_tag = TG_LO0; end
          2'd1: begin u_rd = 1'b1; u_tag = TG_LO1; end
          2'd2: begin u_rd = 1'b1; u_tag = TG_LO2; end
          default: u_tag = TG_HI;
        endcase
      end else begin
        case (p_q)
          2'd0: begin
            if (uph_q == 2'd0) begin u_rd = 1'b1; u_tag = TG_LO0; end
            else if (uph_q == 2'd1) begin u_rd = 1'b1; u_tag = TG_LO1; end
            else begin u_tag = TG_S128; u_last = 1'b1; end
          end
          2'd1: begin u_rd = 1'b1; u_tag = TG_S256; u_last = 1'b1; end
          default: begin
            if (uph_q == 2'd0) begin u_rd = 1'b1; u_tag = TG_LO0; end
            else begin u_rd = 1'b1; u_tag = TG_S512; u_last = 1'b1; end
          end
        endcase
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ug_q <= '0; uph_q <= '0; utail_q <= 1'b0; uend_q <= 1'b0; uwi_q <= '0; utag_q <= TG_NONE;
    end else if (start) begin
      ug_q <= '0; uph_q <= '0; utail_q <= 1'b0; uend_q <= 1'b0; uwi_q <= '0; utag_q <= TG_NONE;
    end else begin
      utag_q <= u_tag;
      if (u_rd) uwi_q <= uwi_q + 10'd1;
      if (u_last) uend_q <= 1'b1;
      if (op_un && !uend_q) begin
        if (!utail_q && uph_q == 2'd3) begin
          uph_q <= 2'd0;
          ug_q <= ug_q + 7'd1;
          if (ug_q == c_ngrp - 7'd1) utail_q <= 1'b1;
        end else begin
          uph_q <= uph_q + 2'd1;
        end
      end
    end
  end

  // ---- block value formation (cycle after issue, read data valid)
  logic [63:0] urd;
  logic [15:0] hi0_q, hi1_q, hi2_q;
  logic [47:0] t_d, t_q;
  logic        tv_q;

  assign urd = (op_q == OP_UNSK) ? sk_rdata : pk_rdata;

  always_comb begin
    case (utag_q)
      TG_LO0, TG_LO1, TG_LO2: t_d = urd[47:0];
      TG_HI:   t_d = {hi2_q, hi1_q, hi0_q};
      TG_S128: t_d = {28'd0, hi1_q[3:0], hi0_q};
      TG_S256: t_d = {9'd0, urd[38:0]};
      TG_S512: t_d = {19'd0, urd[12:0], hi0_q};
      default: t_d = 48'd0;
    endcase
  end

  always_ff @(posedge clk) begin
    if (utag_q == TG_LO0) hi0_q <= urd[63:48];
    if (utag_q == TG_LO1) hi1_q <= urd[63:48];
    if (utag_q == TG_LO2) hi2_q <= urd[63:48];
    if (utag_q != TG_NONE) t_q <= t_d;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) tv_q <= 1'b0;
    else        tv_q <= (utag_q != TG_NONE) & ~start;
  end

  // ---- divider chain: T -> d0..d4
  logic        v1q, v1r, v2q, v2r, v3q, v3r, v4q, v4r;
  logic [38:0] q1;
  logic [28:0] q2;
  logic [19:0] q3;
  logic [9:0]  q4;
  logic [9:0]  r1, r2, r3, r4;

  zen_pku_div769 #(.W(48), .WQ(39), .S(58), .MW(49), .M(64'd374811932576999), .SEG(16)) u_div1 (
    .clk(clk), .rst_n(rst_n), .v_in(tv_q), .x(t_q), .v_q(v1q), .q(q1), .v_r(v1r), .r(r1));
  zen_pku_div769 #(.W(39), .WQ(29), .S(49), .MW(40), .M(64'd732054555815), .SEG(13)) u_div2 (
    .clk(clk), .rst_n(rst_n), .v_in(v1q), .x(q1), .v_q(v2q), .q(q2), .v_r(v2r), .r(r2));
  zen_pku_div769 #(.W(29), .WQ(20), .S(37), .MW(28), .M(64'd178724257), .SEG(10)) u_div3 (
    .clk(clk), .rst_n(rst_n), .v_in(v2q), .x(q2), .v_q(v3q), .q(q3), .v_r(v3r), .r(r3));
  zen_pku_div769 #(.W(20), .WQ(10), .S(28), .MW(19), .M(64'd349071), .SEG(7)) u_div4 (
    .clk(clk), .rst_n(rst_n), .v_in(v3q), .x(q3), .v_q(v4q), .q(q4), .v_r(v4r), .r(r4));

  // digit alignment: d0 (r1) +6, d1 (r2) +4, d2 (r3) +2, d3 (r4) +0, d4 = q4 mod 769 (+1 reg)
  logic [5:0][9:0] d0_sr;
  logic [3:0][9:0] d1_sr;
  logic [1:0][9:0] d2_sr;
  logic [9:0]      d4_q;
  logic [4:0][9:0] dig;

  always_ff @(posedge clk) begin
    d0_sr <= {d0_sr[4:0], r1};
    d1_sr <= {d1_sr[2:0], r2};
    d2_sr <= {d2_sr[0], r3};
    if (v4q) d4_q <= (q4 >= 10'd769) ? (q4 - 10'd769) : q4;
  end

  assign dig[0] = d0_sr[5];
  assign dig[1] = d1_sr[3];
  assign dig[2] = d2_sr[1];
  assign dig[3] = r4;
  assign dig[4] = d4_q;

  // ---- coefficient assembly and slot write
  logic [31:0][9:0] ub_q;
  logic [11:0]      upos_q;
  logic             uem_v_q, uem_h_q;
  logic [6:0]       uem_w_q;
  logic             uv;

  assign uv = v4r & op_un;

  logic [31:0][9:0] ub_d;
  logic [4:0]       ub_j, ub_k;
  always_comb begin
    ub_d = ub_q;
    ub_j = 5'd0;
    ub_k = 5'd0;
    for (int j = 0; j < 32; j++) begin
      ub_j = j;
      ub_k = ub_j - upos_q[4:0];
      if (ub_k < 5'd5) ub_d[j] = dig[ub_k[2:0]];
    end
  end

  always_ff @(posedge clk) begin
    if (uv) ub_q <= ub_d;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      upos_q <= '0; uem_v_q <= 1'b0; uem_h_q <= 1'b0; uem_w_q <= '0;
    end else if (start) begin
      upos_q <= '0; uem_v_q <= 1'b0;
    end else begin
      uem_v_q <= 1'b0;
      if (uv) begin
        upos_q <= upos_q + 12'd5;
        if (upos_q[3:0] >= 4'd11) begin
          uem_v_q <= 1'b1;
          uem_w_q <= upos_q[10:4];
          uem_h_q <= upos_q[4];
        end
      end
    end
  end

  logic [255:0] u_wdata;
  always_comb begin
    for (int k = 0; k < 16; k++)
      u_wdata[16*k +: 16] = {6'd0, (uem_h_q ? ub_q[16+k] : ub_q[k])};
  end

  logic fin_u;
  assign fin_u = uem_v_q & (uem_w_q == c_w[6:0] - 7'd1);

  // ================================================================== SKF unpack
  logic [9:0]  sa_q;                 // MEM_SK read address
  logic [2:0]  sj_q;                 // issue phase within chunk (0..4)
  logic        s_rd;
  logic        srv_q;
  logic [2:0]  srj_q;
  logic [4:0][63:0] sw_q;
  logic        swp_q;                // chunk write pending
  logic [5:0]  sci_q;                // chunk being written

  assign s_rd = op_skfu & (sa_q < c_skfw);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sa_q <= '0; sj_q <= '0; srv_q <= 1'b0; srj_q <= '0; swp_q <= 1'b0; sci_q <= '0;
    end else if (start) begin
      sa_q <= '0; sj_q <= '0; srv_q <= 1'b0; swp_q <= 1'b0; sci_q <= '0;
    end else begin
      srv_q <= s_rd;
      srj_q <= sj_q;
      if (s_rd) begin
        sa_q <= sa_q + 10'd1;
        sj_q <= (sj_q == 3'd4) ? 3'd0 : sj_q + 3'd1;
      end
      swp_q <= srv_q & (srj_q == 3'd4);
      if (swp_q) sci_q <= sci_q + 6'd1;
    end
  end

  always_ff @(posedge clk) begin
    if (srv_q) sw_q[srj_q] <= sk_rdata;
  end

  // transpose 10 planes x 32 bits -> 32 coefficients x 10 bits
  logic [511:0] s_wdata;             // [255:0] coefficients 0..15, [511:256] 16..31
  always_comb begin
    s_wdata = '0;
    for (int j = 0; j < 32; j++)
      for (int k = 0; k < 10; k++)
        s_wdata[16*j + k] = sw_q[k/2][32*(k%2) + j];
  end

  logic fin_s;
  assign fin_s = swp_q & (sci_q == c_nchk[5:0] - 6'd1);

  // ================================================================== PK pack
  // ---- slot read + word FIFO (16 x 10-bit coefficients per entry)
  logic [7:0]        prw_q;          // words issued
  logic              prv_q, prb_q;
  logic [3:0][159:0] pf_q;
  logic [1:0]        pf_wp_q, pf_rp_q;
  logic [2:0]        pf_cnt_q;
  logic              p_rd;
  logic [255:0]      p_rdw;
  logic [159:0]      p_rdc;

  logic p_req;
  assign p_req = op_pack & (prw_q < c_w) & (prw_q < rd_limit) & ((pf_cnt_q + {2'd0, prv_q}) < 3'd4);
  assign p_rd  = p_req & rd_gnt;
  assign p_rdw = prb_q ? rd_data[1] : rd_data[0];

  always_comb begin
    for (int k = 0; k < 16; k++) p_rdc[10*k +: 10] = p_rdw[16*k +: 10];
  end

  // ---- 5-coefficient window
  logic [3:0]       po_q;            // coefficient offset in head word
  logic [8:0]       pblk_q;          // blocks fired
  logic [31:0][9:0] pwin;
  logic             p_fire, p_pop, p_allarr;
  logic [4:0][9:0]  pc;

  assign p_allarr = (prw_q == c_w) & ~prv_q;
  assign p_fire = op_pack & (pblk_q < c_nblk) & (pf_cnt_q != 3'd0) &
                  ((po_q <= 4'd11) | (pf_cnt_q >= 3'd2) | p_allarr);
  assign p_pop  = p_fire & (po_q >= 4'd11);

  always_comb begin
    pwin[15:0]  = pf_q[pf_rp_q];
    pwin[31:16] = (pf_cnt_q >= 3'd2) ? pf_q[pf_rp_q + 2'd1] : 160'd0;
    for (int k = 0; k < 5; k++) pc[k] = pwin[{1'b0, po_q} + k];
  end

  always_ff @(posedge clk) begin
    if (prv_q) pf_q[pf_wp_q] <= p_rdc;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      prw_q <= '0; prv_q <= 1'b0; prb_q <= 1'b0; pf_wp_q <= '0; pf_rp_q <= '0; pf_cnt_q <= '0;
      po_q <= '0; pblk_q <= '0;
    end else if (start) begin
      prw_q <= '0; prv_q <= 1'b0; pf_wp_q <= '0; pf_rp_q <= '0; pf_cnt_q <= '0;
      po_q <= '0; pblk_q <= '0;
    end else begin
      prv_q <= p_rd;
      if (p_rd) begin
        prb_q <= ^prw_q[6:0];
        prw_q <= prw_q + 8'd1;
      end
      if (prv_q) pf_wp_q <= pf_wp_q + 2'd1;
      if (p_pop) pf_rp_q <= pf_rp_q + 2'd1;
      pf_cnt_q <= pf_cnt_q + {2'd0, prv_q} - {2'd0, p_pop};
      if (p_fire) begin
        po_q <= po_q + 4'd5;
        pblk_q <= pblk_q + 9'd1;
      end
    end
  end

  // ---- Horner pipeline T = c0 + 769*(c1 + 769*(c2 + 769*(c3 + 769*c4)))
  logic        hv1_q, hv2_q, hv3_q, hv4_q;
  logic [1:0]  hi1x_q, hi2x_q, hi3x_q, hi4x_q;      // block index within group
  logic        hl1_q, hl2_q, hl3_q, hl4_q;          // tail S block
  logic [19:0] a1_q;
  logic [29:0] a2_q;
  logic [39:0] a3_q;
  logic [47:0] a4_q;
  logic [9:0]  c0_1, c1_1, c2_1, c0_2, c1_2, c0_3;
  logic        p_last;

  assign p_last = (pblk_q == c_nblk - 9'd1);

  always_ff @(posedge clk) begin
    if (p_fire) begin
      a1_q <= {pc[4], 9'd0} + {1'b0, pc[4], 8'd0} + {10'd0, pc[4]} + {10'd0, pc[3]};
      c2_1 <= pc[2]; c1_1 <= pc[1]; c0_1 <= pc[0];
      hi1x_q <= pblk_q[1:0]; hl1_q <= p_last;
    end
    if (hv1_q) begin
      a2_q <= {a1_q, 9'd0} + {1'b0, a1_q, 8'd0} + {10'd0, a1_q} + {20'd0, c2_1};
      c1_2 <= c1_1; c0_2 <= c0_1;
      hi2x_q <= hi1x_q; hl2_q <= hl1_q;
    end
    if (hv2_q) begin
      a3_q <= {a2_q, 9'd0} + {1'b0, a2_q, 8'd0} + {10'd0, a2_q} + {30'd0, c1_2};
      c0_3 <= c0_2;
      hi3x_q <= hi2x_q; hl3_q <= hl2_q;
    end
    if (hv3_q) begin
      a4_q <= {a3_q[38:0], 9'd0} + {a3_q[39:0], 8'd0} + {8'd0, a3_q} + {38'd0, c0_3};
      hi4x_q <= hi3x_q; hl4_q <= hl3_q;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      hv1_q <= 1'b0; hv2_q <= 1'b0; hv3_q <= 1'b0; hv4_q <= 1'b0;
    end else begin
      hv1_q <= p_fire & ~start;
      hv2_q <= hv1_q & ~start;
      hv3_q <= hv2_q & ~start;
      hv4_q <= hv3_q & ~start;
    end
  end

  // ---- group assembly into 64-bit words
  logic [2:0][47:0] tb_q;
  logic [1:0]       npush;
  logic [2:0][63:0] pw;

  always_comb begin
    npush = 2'd0;
    pw = '0;
    if (hv4_q) begin
      if (!hl4_q) begin
        if (hi4x_q == 2'd3) begin
          npush = 2'd3;
          pw[0] = {a4_q[15:0],  tb_q[0]};
          pw[1] = {a4_q[31:16], tb_q[1]};
          pw[2] = {a4_q[47:32], tb_q[2]};
        end
      end else begin
        case (p_q)
          2'd0: begin
            npush = 2'd2;
            pw[0] = {a4_q[15:0], tb_q[0]};
            pw[1] = {12'd0, a4_q[19:16], tb_q[1]};
          end
          2'd1: begin
            npush = 2'd1;
            pw[0] = {24'd0, a4_q[39:0]};
          end
          default: begin
            npush = 2'd2;
            pw[0] = {a4_q[15:0], tb_q[0]};
            pw[1] = {51'd0, a4_q[28:16]};
          end
        endcase
      end
    end
  end

  always_ff @(posedge clk) begin
    if (hv4_q && !hl4_q && hi4x_q != 2'd3) tb_q[hi4x_q] <= a4_q;
  end

  // ---- output queue (4 x 64), one MEM_PK + MEM_SK write per cycle
  logic [3:0][63:0] oq_q, oq_d;
  logic [2:0]       oq_cnt_q, oq_cnt_d, oq_base;
  logic [9:0]       oa_q;
  logic             o_wr;

  assign o_wr = op_pack & (oq_cnt_q != 3'd0);

  always_comb begin
    oq_d = oq_q;
    oq_base = oq_cnt_q - {2'd0, o_wr};
    if (o_wr) begin
      oq_d[0] = oq_q[1];
      oq_d[1] = oq_q[2];
      oq_d[2] = oq_q[3];
    end
    for (int j = 0; j < 4; j++)
      for (int i = 0; i < 3; i++)
        if ((i < npush) && (j == oq_base + i)) oq_d[j] = pw[i];
    oq_cnt_d = oq_base + {1'b0, npush};
  end

  always_ff @(posedge clk) begin
    if (o_wr || npush != 2'd0) oq_q <= oq_d;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      oq_cnt_q <= '0; oa_q <= '0; pk_words_done <= '0;
    end else if (start) begin
      oq_cnt_q <= '0; oa_q <= '0;
      if (op == OP_PACK) pk_words_done <= '0;
    end else begin
      oq_cnt_q <= oq_cnt_d;
      if (o_wr) begin
        oa_q <= oa_q + 10'd1;
        pk_words_done <= pk_words_done + 10'd1;
      end
    end
  end

  logic fin_p;
  assign fin_p = o_wr & (oa_q == c_pkw - 10'd1);

  // ================================================================== SKF pack
  logic [6:0]       kci_q;           // chunk read index
  logic [2:0]       kph_q;           // 5-cycle phase
  logic             k_rd, krv_q, kcb_q;
  logic [9:0][31:0] kpl_q;           // bit planes
  logic             kem_q;
  logic [2:0]       kej_q;
  logic [9:0]       ksa_q;
  logic [511:0]     k_rdc;           // coefficients 0..31 of the chunk

  logic k_req;
  assign k_req = op_skfp & (kci_q < c_nchk) & (kph_q == 3'd0);
  assign k_rd  = k_req & rd_gnt;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      kci_q <= '0; kph_q <= '0; krv_q <= 1'b0; kcb_q <= 1'b0; kem_q <= 1'b0; kej_q <= '0; ksa_q <= '0;
    end else if (start) begin
      kci_q <= '0; kph_q <= '0; krv_q <= 1'b0; kem_q <= 1'b0; kej_q <= '0; ksa_q <= '0;
    end else begin
      krv_q <= k_rd;
      if (op_skfp && !(k_req && !rd_gnt)) kph_q <= (kph_q == 3'd4) ? 3'd0 : kph_q + 3'd1;
      if (k_rd) begin
        kci_q <= kci_q + 7'd1;
        kcb_q <= ^kci_q[5:0];
      end
      if (kem_q) ksa_q <= ksa_q + 10'd1;
      if (krv_q) begin
        kem_q <= 1'b1;
        kej_q <= 3'd0;
      end else if (kem_q) begin
        if (kej_q == 3'd4) kem_q <= 1'b0;
        else kej_q <= kej_q + 3'd1;
      end
    end
  end

  assign k_rdc = kcb_q ? {rd_data[0], rd_data[1]} : {rd_data[1], rd_data[0]};

  always_ff @(posedge clk) begin
    if (krv_q) begin
      for (int k = 0; k < 10; k++)
        for (int j = 0; j < 32; j++)
          kpl_q[k][j] <= k_rdc[16*j + k];
    end
  end

  logic [63:0] k_wdata;
  always_comb begin
    case (kej_q)
      3'd0: k_wdata = {kpl_q[1], kpl_q[0]};
      3'd1: k_wdata = {kpl_q[3], kpl_q[2]};
      3'd2: k_wdata = {kpl_q[5], kpl_q[4]};
      3'd3: k_wdata = {kpl_q[7], kpl_q[6]};
      default: k_wdata = {kpl_q[9], kpl_q[8]};
    endcase
  end

  logic fin_k;
  assign fin_k = kem_q & (ksa_q == c_skfw - 10'd1);

  // ================================================================== F2 load / store
  logic [3:0] fi_q;
  logic       f_rd, f_wr, frv_q;
  logic [2:0] fri_q;

  assign f_rd = op_f2l & (fi_q < c_f2w);
  assign f_wr = op_f2s & (fi_q < c_f2w);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fi_q <= '0; frv_q <= 1'b0; fri_q <= '0; b_wr_en <= 1'b0; b_wr_idx <= '0;
    end else if (start) begin
      fi_q <= '0; frv_q <= 1'b0; b_wr_en <= 1'b0;
    end else begin
      if (f_rd || f_wr) fi_q <= fi_q + 4'd1;
      frv_q <= f_rd;
      fri_q <= fi_q[2:0];
      b_wr_en <= frv_q;
      if (frv_q) b_wr_idx <= fri_q;
    end
  end

  always_ff @(posedge clk) begin
    if (frv_q) b_wr_data <= sk_rdata;
  end

  logic fin_f;
  assign fin_f = (b_wr_en & (b_wr_idx == c_f2w[2:0] - 3'd1)) |
                 (f_wr & (fi_q == c_f2w - 4'd1));

  // ================================================================== output muxing
  always_comb begin
    case (op_q)
      OP_UNPK, OP_UNSK: fin = fin_u;
      OP_PACK:          fin = fin_p;
      OP_SKFU:          fin = fin_s;
      OP_SKFP:          fin = fin_k;
      OP_F2L, OP_F2S:   fin = fin_f;
      default:          fin = 1'b1;
    endcase
  end

  // slot read master
  always_comb begin
    rd_en = 2'b00;
    rd_addr[0] = 6'd0;
    rd_addr[1] = 6'd0;
    if (op_q == OP_SKFP) begin
      rd_en = {k_req, k_req};
      rd_addr[0] = kci_q[5:0];
      rd_addr[1] = kci_q[5:0];
    end else begin
      rd_en[0] = p_req & ~(^prw_q[6:0]);
      rd_en[1] = p_req &  (^prw_q[6:0]);
      rd_addr[0] = prw_q[6:1];
      rd_addr[1] = prw_q[6:1];
    end
  end

  // slot write master
  logic sb;   // bank of chunk word 2*sci
  assign sb = ^sci_q;

  always_comb begin
    wr_en = 2'b00;
    wr_addr[0] = 6'd0;
    wr_addr[1] = 6'd0;
    wr_data[0] = u_wdata;
    wr_data[1] = u_wdata;
    if (op_q == OP_SKFU) begin
      wr_en = {swp_q, swp_q};
      wr_addr[0] = sci_q;
      wr_addr[1] = sci_q;
      wr_data[0] = sb ? s_wdata[511:256] : s_wdata[255:0];
      wr_data[1] = sb ? s_wdata[255:0]   : s_wdata[511:256];
    end else begin
      wr_en[0] = uem_v_q & op_un & ~(^uem_w_q);
      wr_en[1] = uem_v_q & op_un &  (^uem_w_q);
      wr_addr[0] = uem_w_q[6:1];
      wr_addr[1] = uem_w_q[6:1];
    end
  end

  // object ports
  always_comb begin
    pk_en = 1'b0; pk_we = 1'b0; pk_addr = 10'd0; pk_wdata = 64'd0; pk_wmask = 8'h00;
    sk_en = 1'b0; sk_we = 1'b0; sk_addr = 10'd0; sk_wdata = 64'd0; sk_wmask = 8'h00;
    case (op_q)
      OP_UNPK: begin
        pk_en = u_rd; pk_addr = uwi_q;
      end
      OP_UNSK: begin
        sk_en = u_rd; sk_addr = c_pkoff + uwi_q;
      end
      OP_PACK: begin
        pk_en = o_wr; pk_we = o_wr; pk_addr = oa_q; pk_wdata = oq_q[0]; pk_wmask = 8'hFF;
        sk_en = o_wr; sk_we = o_wr; sk_addr = c_pkoff + oa_q; sk_wdata = oq_q[0];
        sk_wmask = (oa_q == c_pkw - 10'd1) ? c_lmask : 8'hFF;
      end
      OP_SKFU: begin
        sk_en = s_rd; sk_addr = sa_q;
      end
      OP_SKFP: begin
        sk_en = kem_q; sk_we = kem_q; sk_addr = ksa_q; sk_wdata = k_wdata; sk_wmask = 8'hFF;
      end
      OP_F2L: begin
        sk_en = f_rd; sk_addr = c_fpk + {6'd0, fi_q};
      end
      OP_F2S: begin
        sk_en = f_wr; sk_we = f_wr; sk_addr = c_fpk + {6'd0, fi_q};
        sk_wdata = f2_in[64*fi_q[2:0] +: 64]; sk_wmask = 8'hFF;
      end
      default: ;
    endcase
  end

endmodule
