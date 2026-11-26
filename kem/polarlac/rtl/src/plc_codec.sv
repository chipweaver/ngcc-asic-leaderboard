// plc_codec — object <-> polynomial-slot conversions and ciphertext compare.
// Bit-exact with the per-set reference C:
//   pke.c serialize/deserialize_polyvec_ntt (raw int16 LE), poly.c poly_compress/poly_decompress
//   (q=257 zero selector incl. Light's missing capacity guard; 512-Star CRT base-769 packing),
//   byte_compress/byte_decompress, poly_compress_c2/poly_decompress_c2 (RECIP_Q), pack/unpack_c2_dbit
//   (d=4; d=3 for Light), KEM layout sk = s_ntt || pk || z, pk = pk_seed || enc(b0) || enc(b1), ct = c1 || c2.
//
// Structure (one op at a time):
//   encode ops  : slot pair reads -> coefficient queue CQ (4-coeff groups, 16 deep)
//                 -> F: window register W (<= 20 coefficients) -> X: conversion (carry-save for 512-Star)
//                 -> E: final add -> byte aligner (byte offset, byte enables) -> object write port(s),
//                 or CT read + compare (cmp=1, sticky ne)
//   decode ops  : object word reads -> byte queue BQ (32 B) -> decode converter -> group queue OQ (8 deep)
//                 -> slot pair writes (16 coefficients per write)
//                 512-Star: one plc_codec_dig digit pipeline, 4 steps per 24-byte block (tmp0..tmp3)
//                 RD_PKSEED: pk_seed bytes collected in OQ -> rg_wdata, 1-cycle rg_we
//   PK_DEC q=257: the selector window at poly byte N (Light 32 B = no capacity guard, 128: 1 B,
//                 256/512: 2 B) is prefetched first.
// Throughput: 1 object word/cycle for raw, q=257 byte and selector codecs, q=257 C2 and register ops;
// 512-Star CRT encode 3 words / 3 cycles, CRT decode 3 words / 4 cycles, 512-Star C2 encode 4 B/cycle.
// Ops: 0 SER_S 1 DES_S 2 PK_ENC 3 PK_DEC 4 C1_ENC 5 C1_DEC 6 C2_ENC 7 C2_DEC 8 WR_PKSEED 9 WR_Z 10 RD_PKSEED.
// rg[] (RG_PS / RG_Z) is read during WR_PKSEED / WR_Z and must stay stable while busy.
// Constant multiplications: plc_codec_pkg.sv (generated functions).
module plc_codec
  import plc_pkg::*;
  import plc_codec_pkg::*;
(
  input  logic                clk,
  input  logic                rst_n,
  input  logic                start,
  input  logic [3:0]          op,
  input  logic                k,
  input  logic [2:0]          pcode,
  input  logic [3:0]          slot,
  input  logic                cmp,
  input  logic                src_sk,
  input  logic                cmp_clr,
  output logic                busy,
  output logic                done,
  output logic                ovf,
  output logic                ne,
  // register file (read: RG_PS, RG_Z) and pk_seed write-back (RD_PKSEED)
  input  logic [6:0][511:0]   rg,
  output logic                rg_we,
  output logic [511:0]        rg_wdata,
  // slot read port (engine master; rdata valid 2 cycles after ren)
  output logic [3:0]          rslot,
  output logic [1:0]          ren,
  output logic [1:0][5:0]     raddr,
  input  logic [1:0][127:0]   rdata,
  // slot write port
  output logic [3:0]          wslot,
  output logic [1:0]          wen,
  output logic [1:0][5:0]     waddr,
  output logic [1:0][127:0]   wdata,
  // codec object port (PK=0, SK=1, CT=2; ordata valid 2 cycles after oren)
  output logic [2:0]          cd_oren,
  output logic [2:0][9:0]     cd_oraddr,
  input  logic [2:0][63:0]    cd_ordata,
  output logic [2:0]          cd_owen,
  output logic [2:0][9:0]     cd_owaddr,
  output logic [2:0][7:0]     cd_owbe,
  output logic [2:0][63:0]    cd_owdata
);
  localparam logic [3:0] OP_SER_S = 4'd0, OP_DES_S = 4'd1, OP_PK_ENC = 4'd2, OP_PK_DEC = 4'd3,
                         OP_C1_ENC = 4'd4, OP_C1_DEC = 4'd5, OP_C2_ENC = 4'd6, OP_C2_DEC = 4'd7,
                         OP_WR_PKSEED = 4'd8, OP_WR_Z = 4'd9, OP_RD_PKSEED = 4'd10;
  localparam logic [2:0] EM_SER = 3'd0, EM_LOW = 3'd1, EM_C2 = 3'd2, EM_CRT = 3'd3, EM_RG = 3'd4;
  localparam logic [2:0] DM_RAW = 3'd0, DM_SEL = 3'd1, DM_LOW = 3'd2, DM_C2 = 3'd3, DM_CRT = 3'd4, DM_RG = 3'd5;
  localparam int CQ_D   = 4;    // coefficient queue: slot pairs (16 coefficients each)
  localparam int BQ_D   = 4;    // byte queue: object words
  localparam int OQ_CAP = 8;    // output queue: groups (4 coefficients)

  typedef enum logic [2:0] {S_IDLE, S_SETUP, S_INIT, S_PRE, S_PSW, S_RUN, S_DONE} st_t;
  st_t st;

  // ---------------- latched command ----------------
  logic [3:0] r_op;
  logic       r_k, r_cmp, r_srcsk;
  logic [2:0] r_pc;
  logic [3:0] r_slot;

  // ---------------- configuration (registered in S_SETUP) ----------------
  logic        c_enc;          // encode direction
  logic [2:0]  c_em, c_dm;
  logic [1:0]  c_buf;          // primary object buffer
  logic        c_dual;         // also write SK at +N/2 words (PK_ENC, WR_PKSEED)
  logic [9:0]  c_skoff;        // N/2 words
  logic        c_pkenc;        // PK_ENC q=257: count zeros, emit selector
  logic        c_q769, c_d3;
  logic [4:0]  c_selbits;      // 8 or 16
  logic [1:0]  c_senc;         // selector bytes written (1/2)
  logic [8:0]  c_nfire;        // converter steps
  logic [6:0]  c_npair;        // slot pairs (N/16)
  logic        c_rgz;          // EM_RG source: 0 RG_PS, 1 RG_Z
  logic [12:0] c_base;         // data region byte offset
  logic [12:0] c_len;          // data region bytes (decode reader)
  logic [12:0] c_selbase;      // PK_DEC q=257 selector window
  logic [5:0]  c_sellen;

  // comb config from the latched command
  logic [11:0] f_n;
  logic [12:0] f_4n, f_pkoff, f_base, f_len, f_selbase;
  logic [5:0]  f_sellen;
  logic        f_enc, f_dual, f_q769, f_pkenc;
  logic [2:0]  f_em, f_dm;
  logic [1:0]  f_buf;
  logic [8:0]  f_nfire;
  always_comb begin
    f_n      = p_n(r_pc);
    f_q769   = p_q769(r_pc);
    f_4n     = 13'(f_n) << 2;
    f_pkoff  = 13'(p_pkseed(r_pc)) + (r_k ? 13'(p_pkpb(r_pc)) : 13'd0);
    f_enc    = 1'b0; f_dual = 1'b0; f_pkenc = 1'b0;
    f_em     = EM_SER; f_dm = DM_RAW; f_buf = OB_SK;
    f_base   = '0; f_len = '0;
    f_nfire  = '0;
    f_sellen = (r_pc == 3'd0) ? 6'd32 : (r_pc == 3'd1) ? 6'd1 : 6'd2;
    case (r_op)
      OP_SER_S: begin
        f_enc = 1'b1; f_em = EM_SER; f_buf = OB_SK;
        f_base = r_k ? 13'(f_n) << 1 : 13'd0; f_nfire = 9'(f_n >> 2);
      end
      OP_DES_S: begin
        f_dm = DM_RAW; f_buf = OB_SK;
        f_base = r_k ? 13'(f_n) << 1 : 13'd0; f_len = 13'(f_n) << 1; f_nfire = 9'(f_n >> 2);
      end
      OP_PK_ENC: begin
        f_enc = 1'b1; f_em = f_q769 ? EM_CRT : EM_LOW; f_buf = OB_PK; f_dual = 1'b1; f_pkenc = !f_q769;
        f_base = f_pkoff; f_nfire = f_q769 ? 9'd154 : 9'(f_n >> 3) + 9'd1;
      end
      OP_PK_DEC: begin
        f_dm = f_q769 ? DM_CRT : DM_SEL; f_buf = r_srcsk ? OB_SK : OB_PK;
        f_base = (r_srcsk ? f_4n : 13'd0) + f_pkoff;
        f_len = f_q769 ? 13'd1229 : 13'(f_n); f_nfire = f_q769 ? 9'd205 : 9'(f_n >> 3);
      end
      OP_C1_ENC: begin
        f_enc = 1'b1; f_em = f_q769 ? EM_CRT : EM_LOW; f_buf = OB_CT;
        f_base = r_k ? 13'(p_c1pb(r_pc)) : 13'd0; f_nfire = f_q769 ? 9'd154 : 9'(f_n >> 3);
      end
      OP_C1_DEC: begin
        f_dm = f_q769 ? DM_CRT : DM_LOW; f_buf = OB_CT;
        f_base = r_k ? 13'(p_c1pb(r_pc)) : 13'd0;
        f_len = 13'(p_c1pb(r_pc)); f_nfire = f_q769 ? 9'd205 : 9'(f_n >> 3);
      end
      OP_C2_ENC: begin
        f_enc = 1'b1; f_em = EM_C2; f_buf = OB_CT;
        f_base = p_c1len(r_pc); f_nfire = f_q769 ? 9'(f_n >> 3) : 9'(f_n >> 4);
      end
      OP_C2_DEC: begin
        f_dm = DM_C2; f_buf = OB_CT;
        f_base = p_c1len(r_pc); f_len = p_c2len(r_pc); f_nfire = 9'(f_n >> 4);
      end
      OP_WR_PKSEED: begin
        f_enc = 1'b1; f_em = EM_RG; f_buf = OB_PK; f_dual = 1'b1;
        f_base = 13'd0; f_nfire = 9'(p_pkseed(r_pc) >> 3);
      end
      OP_WR_Z: begin
        f_enc = 1'b1; f_em = EM_RG; f_buf = OB_SK;
        f_base = f_4n + p_pklen(r_pc); f_nfire = 9'(p_msg(r_pc) >> 3);
      end
      default: begin // OP_RD_PKSEED
        f_dm = DM_RG; f_buf = r_srcsk ? OB_SK : OB_PK;
        f_base = r_srcsk ? f_4n : 13'd0; f_len = 13'(p_pkseed(r_pc)); f_nfire = 9'(p_pkseed(r_pc) >> 3);
      end
    endcase
    f_selbase = f_base + 13'(f_n);
  end

  // ---------------- state / handshake ----------------
  logic fin;          // all work of the current op completed (decided in S_RUN)
  logic pre_end;      // selector prefetch consumed
  logic psw_ok;       // reader idle in S_PSW

  assign busy = (st != S_IDLE);

  // ---------------- object reader (decode) ----------------
  logic        rd_on;              // reader enabled (S_PRE or S_RUN, decode)
  logic [9:0]  rd_addr;
  logic [10:0] rd_left;
  logic        rd_v1, rd_v2;
  logic        rd_iss;

  // byte queue: word FIFO + byte offset of the head (initialised to the byte offset of the pass start)
  logic [BQ_D-1:0][63:0] bqm;
  logic [1:0]  bq_wp, bq_rp;
  logic [2:0]  bq_nw;               // words stored
  logic [2:0]  bq_off;              // consumed bytes of the head word
  logic [5:0]  bq_av;               // bytes available
  logic [3:0]  bq_pop;
  logic        bq_wc;               // head word fully consumed this cycle

  // ---------------- decode converter ----------------
  logic [8:0]  dc_idx;
  logic [1:0]  dc_ph;
  logic        dc_fire;
  logic [5:0]  sel_rem;
  logic [1:0]  sel_pos;
  logic [255:0] sel;
  logic [15:0] dc_h0, dc_h1, dc_h2;
  logic        rg_we_q;
  // digit pipelines
  localparam int DLAT = 9;          // plc_codec_dig latency
  logic [47:0] dg_x;
  logic [4:0][9:0] dg;
  logic [DLAT-1:0] dg_v;            // valid pipeline along the digit unit
  logic [DLAT-1:0][2:0] dg_tag;
  logic        dgx_v;
  logic [2:0]  dgx_tag;             // 0..3: tmp0..tmp3 of a block, 4: last word
  logic [14:0][15:0] blk;           // coeffs 0..14 of the current CRT block

  // group output queue
  logic [OQ_CAP-1:0][63:0] oq;
  logic [3:0]  oq_cnt;
  logic        oq_pop;
  logic        oq_push;
  logic [2:0]  oq_png;
  logic [4:0][63:0] oq_pd;
  logic [5:0]  w_pair;

  // ---------------- slot reader (encode) ----------------
  logic        sr_iss;
  logic [6:0]  sr_left;
  logic [5:0]  sr_pair;
  logic        sr_v1, sr_v2, sr_p1, sr_p2;

  // coefficient queue: pair FIFO + group offset of the head pair
  logic [CQ_D-1:0][255:0] cqm;
  logic [1:0]  cq_wp, cq_rp;
  logic [2:0]  cq_np;               // pairs stored
  logic [1:0]  cq_off;              // consumed groups of the head pair
  logic [4:0]  cq_av;               // groups available
  logic [2:0]  cq_pop;
  logic [1:0]  cq_pc;               // pairs fully consumed this cycle

  // ---------------- encode converter ----------------
  logic [8:0]  ec_idx;
  logic [1:0]  ec_ph;
  logic        ec_fire;
  logic [31:0] t3h;
  logic [10:0] zc;
  logic [15:0] selr;
  logic        e_v, e_last;
  logic [3:0]  e_nb;
  logic [63:0] e_dat;

  // aligner
  logic [2:0]  al_f;
  logic [63:0] al_acc;
  logic [7:0]  al_m;
  logic [9:0]  al_addr;
  logic        al_fl, al_done;
  // output stage
  logic        o_v;
  logic [9:0]  o_addr;
  logic [7:0]  o_be;
  logic [63:0] o_dat;
  // compare pipe
  logic        cp1_v, cp2_v;
  logic [7:0]  cp1_be, cp2_be;
  logic [63:0] cp1_d, cp2_d;

  // =====================================================================================
  // FSM
  // =====================================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE; done <= 1'b0;
      r_op <= '0; r_k <= 1'b0; r_cmp <= 1'b0; r_srcsk <= 1'b0; r_pc <= '0; r_slot <= '0;
      c_enc <= 1'b0; c_em <= '0; c_dm <= '0; c_buf <= '0; c_dual <= 1'b0; c_skoff <= '0;
      c_pkenc <= 1'b0; c_q769 <= 1'b0; c_d3 <= 1'b0; c_selbits <= '0; c_senc <= '0;
      c_nfire <= '0; c_npair <= '0; c_rgz <= 1'b0; c_base <= '0; c_len <= '0; c_selbase <= '0; c_sellen <= '0;
    end else begin
      done <= 1'b0;
      case (st)
        S_IDLE: if (start) begin
          r_op <= op; r_k <= k; r_cmp <= cmp; r_srcsk <= src_sk; r_pc <= pcode; r_slot <= slot;
          st <= S_SETUP;
        end
        S_SETUP: begin
          c_enc <= f_enc; c_em <= f_em; c_dm <= f_dm; c_buf <= f_buf; c_dual <= f_dual;
          c_skoff <= 10'(f_n >> 1); c_pkenc <= f_pkenc; c_q769 <= f_q769; c_d3 <= (r_pc == 3'd0);
          c_selbits <= (r_pc <= 3'd1) ? 5'd8 : 5'd16; c_senc <= (r_pc <= 3'd1) ? 2'd1 : 2'd2;
          c_nfire <= f_nfire; c_npair <= 7'(f_n >> 4); c_rgz <= (r_op == OP_WR_Z);
          c_base <= f_base; c_len <= f_len; c_selbase <= f_selbase; c_sellen <= f_sellen;
          st <= S_INIT;
        end
        S_INIT: st <= (!c_enc && c_dm == DM_SEL) ? S_PRE : S_RUN;   // datapath init from c_*
        S_PRE: if (pre_end) st <= S_PSW;
        S_PSW: if (psw_ok) st <= S_RUN;
        S_RUN: if (fin) begin st <= S_DONE; done <= 1'b1; end
        default: st <= S_IDLE;   // S_DONE (done pulse is high in this cycle)
      endcase
    end
  end

  // =====================================================================================
  // DECODE: object reader + byte queue
  // =====================================================================================
  assign rd_on = !c_enc && (st == S_PRE || st == S_RUN);
  // credit: words stored after this cycle's consumption + words in flight + this word <= BQ_D
  always_comb begin
    logic [3:0] room;
    room   = 4'(bq_nw) - 4'(bq_wc) + 4'(rd_v1) + 4'(rd_v2) + 4'd1;
    rd_iss = rd_on && (rd_left != 0) && (room <= 4'(BQ_D));
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rd_addr <= '0; rd_left <= '0; rd_v1 <= 1'b0; rd_v2 <= 1'b0;
    end else begin
      rd_v1 <= rd_iss; rd_v2 <= rd_v1;
      if (st == S_INIT || (st == S_PSW && psw_ok)) begin
        logic [12:0] b, l;
        if (st == S_INIT && !c_enc && c_dm == DM_SEL) begin b = c_selbase; l = 13'(c_sellen); end
        else begin b = c_base; l = c_len; end
        rd_addr <= b[12:3];
        rd_left <= 11'((14'(b[2:0]) + 14'(l) + 14'd7) >> 3);
      end else if (rd_iss) begin
        rd_addr <= rd_addr + 10'd1;
        rd_left <= rd_left - 11'd1;
      end
    end
  end

  // byte queue update
  assign bq_av = (bq_nw == 3'd0) ? 6'd0 : {bq_nw, 3'b000} - 6'(bq_off);   // off may be > 0 before the 1st word
  always_comb begin
    logic [3:0] sum;
    sum   = 4'(bq_off) + bq_pop;
    bq_wc = sum[3];
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bq_nw <= '0; bq_off <= '0; bq_wp <= '0; bq_rp <= '0;
    end else if (st == S_INIT || (st == S_PSW && psw_ok)) begin
      logic [12:0] b;
      b = (st == S_INIT && !c_enc && c_dm == DM_SEL) ? c_selbase : c_base;
      bq_nw <= '0; bq_off <= b[2:0]; bq_wp <= '0; bq_rp <= '0;
    end else begin
      logic [3:0] sum;
      sum    = 4'(bq_off) + bq_pop;
      bq_off <= sum[2:0];
      bq_rp  <= bq_rp + 2'(bq_wc);
      bq_nw  <= bq_nw - 3'(bq_wc) + 3'(rd_v2);
      if (rd_v2) bq_wp <= bq_wp + 2'd1;
    end
  end
  always_ff @(posedge clk) if (rd_v2) bqm[bq_wp] <= cd_ordata[c_buf];

  // =====================================================================================
  // DECODE converter
  // =====================================================================================
  logic [63:0] bq8;                  // byte window: next 8 bytes of the stream
  always_comb begin
    logic [127:0] w2;
    w2  = {bqm[bq_rp + 2'd1], bqm[bq_rp]};
    bq8 = 64'(w2 >> {bq_off, 3'b000});
  end

  logic [3:0] dc_need;
  logic       dc_last;
  always_comb begin
    dc_last = (dc_idx == c_nfire - 9'd1);
    if (st == S_PRE)                         dc_need = (sel_rem >= 6'd8) ? 4'd8 : 4'(sel_rem);
    else if (c_dm == DM_C2 && c_d3)          dc_need = 4'd6;
    else if (c_dm == DM_CRT && dc_last)      dc_need = 4'd5;
    else if (c_dm == DM_CRT && dc_ph == 2'd3) dc_need = 4'd0;   // tmp3 step: no bytes
    else                                     dc_need = 4'd8;
    if (st == S_PRE)      dc_fire = (sel_rem != 0) && (bq_av >= 6'(dc_need));
    else if (st == S_RUN) dc_fire = !c_enc && (dc_idx != c_nfire) && (bq_av >= 6'(dc_need));
    else                  dc_fire = 1'b0;
    bq_pop  = dc_fire ? dc_need : 4'd0;
  end
  assign pre_end = (st == S_PRE) && (sel_rem == 0);
  assign psw_ok  = (st == S_PSW) && !rd_v1 && !rd_v2;

  // stage D1: the consumed bytes are registered (dw_*) before conversion (non-CRT modes)
  logic        dw_v, dw_pre, dw_last;
  logic [1:0]  dw_pos;
  logic [63:0] dw_b;
  logic [7:0][7:0] dwb;
  assign dwb = dw_b;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dw_v <= 1'b0; dw_pre <= 1'b0; dw_last <= 1'b0; dw_pos <= '0;
    end else begin
      dw_v    <= dc_fire && (st == S_RUN) && (c_dm != DM_CRT);
      dw_pre  <= dc_fire && (st == S_PRE);
      dw_last <= dc_last;
      dw_pos  <= sel_pos;
    end
  end
  always_ff @(posedge clk) begin
    if (dc_fire) begin
      logic [63:0] mk;
      mk = (st != S_PRE || dc_need[3]) ? 64'hFFFF_FFFF_FFFF_FFFF
                                       : ~(64'hFFFF_FFFF_FFFF_FFFF << {dc_need[2:0], 3'b000});
      dw_b <= bq8 & mk;
    end
  end

  // stage D2: byte-level decode functions on dw_b
  logic [7:0]       zmask;
  logic [7:0][2:0]  zpre;
  logic [3:0]       zcnt;
  always_comb begin
    logic [3:0] c;
    c = 4'd0;
    for (int i = 0; i < 8; i++) begin
      zmask[i] = (dwb[i] == 8'h00);
      zpre[i]  = c[2:0];
      c = c + {3'b000, zmask[i]};
    end
    zcnt = c;
  end

  logic [255:0] dc_out;   // up to 16 coefficients
  always_comb begin
    dc_out = '0;
    case (c_dm)
      DM_RAW, DM_RG: dc_out[63:0] = dw_b;
      DM_SEL: for (int i = 0; i < 8; i++)
                dc_out[16*i +: 16] = zmask[i] ? {7'd0, sel[zpre[i]], 8'h00} : {8'h00, dwb[i]};
      DM_LOW: for (int i = 0; i < 8; i++) dc_out[16*i +: 16] = {8'h00, dwb[i]};
      default: begin // DM_C2
        for (int i = 0; i < 16; i++) begin
          logic [3:0]  cd;
          logic [13:0] v;
          if (c_d3) begin
            cd = {1'b0, dw_b[3*i +: 3]};
            v  = ({cd, 8'h00} + 14'(cd) + 14'd4) >> 3;
          end else begin
            cd = dw_b[4*i +: 4];
            v  = c_q769 ? ({cd, 9'h000} + {cd, 8'h00} + 14'(cd) + 14'd8) >> 4
                        : ({cd, 8'h00} + 14'(cd) + 14'd8) >> 4;
          end
          dc_out[16*i +: 16] = {2'b00, v};
        end
      end
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dc_idx <= '0; dc_ph <= '0; sel_rem <= '0; sel_pos <= '0;
      rg_we_q <= 1'b0; dgx_v <= 1'b0; dgx_tag <= '0;
    end else begin
      rg_we_q <= dw_v && dw_last && (c_dm == DM_RG);
      dgx_v   <= 1'b0;
      if (st == S_INIT) begin
        dc_idx <= '0; dc_ph <= '0; sel_rem <= c_sellen; sel_pos <= '0;
      end
      if (dc_fire) begin
        if (st == S_PRE) begin
          sel_rem <= sel_rem - 6'(dc_need);
          sel_pos <= sel_pos + 2'd1;
        end else begin
          dc_idx <= dc_idx + 9'd1;
          case (c_dm)
            DM_CRT: begin
              dgx_v   <= 1'b1;
              dgx_tag <= dc_last ? 3'd4 : {1'b0, dc_ph};
              dc_ph   <= dc_ph + 2'd1;
            end
            default: ;   // via D1 (dw_*) into OQ
          endcase
        end
      end
    end
  end

  always_ff @(posedge clk) begin
    if (st == S_SETUP) begin
      sel <= '0;
    end else if (dw_pre) begin
      sel[64*dw_pos +: 64] <= dw_b;
    end else if (dw_v && c_dm == DM_SEL) begin
      sel <= sel >> zcnt;
    end
    if (dc_fire && st == S_RUN && c_dm == DM_CRT) begin
      // block steps: 0..2 pop words w0..w2 (tmp0..2 = w[47:0]), 3 pops nothing (tmp3 = {h2,h1,h0})
      if (dc_last)               dg_x <= {9'd0, bq8[38:0]};
      else if (dc_ph == 2'd3)    dg_x <= {dc_h2, dc_h1, dc_h0};
      else                       dg_x <= bq8[47:0];
      if (dc_ph == 2'd0) dc_h0 <= bq8[63:48];
      if (dc_ph == 2'd1) dc_h1 <= bq8[63:48];
      if (dc_ph == 2'd2) dc_h2 <= bq8[63:48];
    end
  end
  assign rg_we    = rg_we_q;
  assign rg_wdata = oq;      // RD_PKSEED collects pk_seed in OQ (no pops; groups above the count are zero)

  plc_codec_dig u_dig (.clk(clk), .x(dg_x), .dg(dg));

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dg_v <= '0; dg_tag <= '0;
    end else begin
      dg_v   <= {dg_v[DLAT-2:0], dgx_v};
      dg_tag <= {dg_tag[DLAT-2:0], dgx_tag};
    end
  end
  always_ff @(posedge clk) begin
    if (dg_v[DLAT-1] && dg_tag[DLAT-1] <= 3'd2)
      for (int i = 0; i < 5; i++) blk[5 * dg_tag[DLAT-1][1:0] + i] <= {6'd0, dg[i]};
  end

  // OQ push selection
  always_comb begin
    oq_push = 1'b0; oq_png = 3'd0; oq_pd = '0;
    if (c_dm == DM_CRT) begin
      if (dg_v[DLAT-1] && dg_tag[DLAT-1] == 3'd3) begin
        logic [19:0][15:0] b;
        for (int i = 0; i < 15; i++) b[i] = blk[i];
        for (int i = 0; i < 5; i++) b[15 + i] = {6'd0, dg[i]};
        oq_push = 1'b1; oq_png = 3'd5; oq_pd = b;
      end else if (dg_v[DLAT-1] && dg_tag[DLAT-1] == 3'd4) begin
        oq_push = 1'b1; oq_png = 3'd1;
        oq_pd[0] = {6'd0, dg[3], 6'd0, dg[2], 6'd0, dg[1], 6'd0, dg[0]};
      end
    end else begin
      oq_push = dw_v;
      oq_png  = (c_dm == DM_SEL || c_dm == DM_LOW) ? 3'd2 : (c_dm == DM_C2) ? 3'd4 : 3'd1;
      oq_pd   = {64'd0, dc_out};
    end
  end

  assign oq_pop = (oq_cnt >= 4'd4) && (c_dm != DM_RG);
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) oq_cnt <= '0;
    else if (st == S_SETUP) oq_cnt <= '0;
    else oq_cnt <= oq_cnt - (oq_pop ? 4'd4 : 4'd0) + (oq_push ? 4'(oq_png) : 4'd0);
  end
  always_ff @(posedge clk) begin
    logic [3:0] m;
    logic [64*OQ_CAP-1:0] keep;
    m    = oq_cnt - (oq_pop ? 4'd4 : 4'd0);
    keep = ~({(64*OQ_CAP){1'b1}} << {m, 6'b000000});
    oq  <= ((oq_pop ? (oq >> 256) : oq) & keep) | (oq_push ? ((64*OQ_CAP)'(oq_pd) << {m, 6'b000000}) : '0);
`ifndef SYNTHESIS
    if (oq_push && (m + 4'(oq_png) > 4'(OQ_CAP))) $error("plc_codec: OQ overflow");
`endif
  end

  // slot pair writer: OQ head (16 coefficients) -> pair w_pair (word 2p in bank ^p, word 2p+1 in the
  // other bank, both at address p); the write lands at the end of the cycle in which wen is high.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) w_pair <= '0;
    else if (st == S_SETUP) w_pair <= '0;
    else if (oq_pop) w_pair <= w_pair + 6'd1;
  end
  always_comb begin
    wdata[^w_pair]  = {oq[1], oq[0]};
    wdata[!(^w_pair)] = {oq[3], oq[2]};
  end
  assign wslot    = r_slot;
  assign wen      = {2{oq_pop}};
  assign waddr[0] = w_pair;
  assign waddr[1] = w_pair;

  // =====================================================================================
  // ENCODE: slot reader + coefficient queue
  // =====================================================================================
  always_comb begin
    logic [5:0] room;
    room   = 6'(cq_np) - 6'(cq_pc) + 6'(sr_v1) + 6'(sr_v2) + 6'd1;
    sr_iss = (st == S_RUN) && c_enc && (c_em != EM_RG) && (sr_left != 0) && (room <= 6'(CQ_D));
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sr_left <= '0; sr_pair <= '0; sr_v1 <= 1'b0; sr_v2 <= 1'b0; sr_p1 <= 1'b0; sr_p2 <= 1'b0;
    end else begin
      sr_v1 <= sr_iss; sr_v2 <= sr_v1; sr_p1 <= ^sr_pair; sr_p2 <= sr_p1;
      if (st == S_INIT) begin
        sr_left <= c_npair; sr_pair <= '0;
      end else if (sr_iss) begin
        sr_left <= sr_left - 7'd1; sr_pair <= sr_pair + 6'd1;
      end
    end
  end
  assign rslot    = r_slot;
  assign ren      = {2{sr_iss}};
  assign raddr[0] = sr_pair;
  assign raddr[1] = sr_pair;

  assign cq_av = (cq_np == 3'd0) ? 5'd0 : {cq_np, 2'b00} - 5'(cq_off);
  always_comb begin
    logic [3:0] sum;
    sum   = 4'(cq_off) + 4'(cq_pop);
    cq_pc = sum[3:2];
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cq_np <= '0; cq_off <= '0; cq_wp <= '0; cq_rp <= '0;
    end else if (st == S_SETUP) begin
      cq_np <= '0; cq_off <= '0; cq_wp <= '0; cq_rp <= '0;
    end else begin
      logic [3:0] sum;
      sum    = 4'(cq_off) + 4'(cq_pop);
      cq_off <= sum[1:0];
      cq_rp  <= cq_rp + cq_pc;
      cq_np  <= cq_np - 3'(cq_pc) + 3'(sr_v2);
      if (sr_v2) cq_wp <= cq_wp + 2'd1;
    end
  end
  // pair = {word 2p+1, word 2p}; word 2p is in bank ^p
  always_ff @(posedge clk) if (sr_v2) cqm[cq_wp] <= {rdata[!sr_p2], rdata[sr_p2]};

  // coefficient window: 5 groups starting at the head offset
  logic [4:0][63:0] cq5;
  always_comb begin
    logic [7:0][63:0] w8;
    w8 = {cqm[cq_rp + 2'd1], cqm[cq_rp]};
    for (int g = 0; g < 5; g++) cq5[g] = w8[cq_off + 3'(g)];
  end

  // =====================================================================================
  // ENCODE converter
  // =====================================================================================
  // ---- stage F (fire): consume CQ groups into the window register W ----
  logic       ec_lastq, ec_doneq;      // ec_idx == c_nfire-1 / == c_nfire
  logic       ec_selfire;
  logic [2:0] ec_need;
  always_comb begin
    ec_selfire = c_pkenc && ec_lastq;
    case (c_em)
      EM_SER:  ec_need = 3'd1;
      EM_LOW:  ec_need = ec_selfire ? 3'd0 : 3'd2;
      EM_C2:   ec_need = c_q769 ? 3'd2 : 3'd4;     // 512-Star: 8 lanes (one slot word) per step
      EM_CRT:  ec_need = (ec_ph != 2'd0) ? 3'd0 : ec_lastq ? 3'd1 : 3'd5;   // block loaded in phase 0
      default: ec_need = 3'd0;
    endcase
    ec_fire = (st == S_RUN) && c_enc && !ec_doneq && (cq_av >= 5'(ec_need));
    cq_pop  = ec_fire ? ec_need : 3'd0;
  end

  logic             w_v, w_last, w_self;
  logic [1:0]       w_ph;
  logic [2:0]       w_ridx;
  logic [19:0][15:0] cf;               // W: coefficient window (20 = one 512-Star block)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ec_idx <= '0; ec_ph <= '0; ec_lastq <= 1'b0; ec_doneq <= 1'b0;
      w_v <= 1'b0; w_last <= 1'b0; w_self <= 1'b0; w_ph <= '0; w_ridx <= '0;
    end else begin
      w_v <= ec_fire;
      if (st == S_INIT) begin
        ec_idx <= '0; ec_ph <= '0; ec_lastq <= (c_nfire == 9'd1); ec_doneq <= 1'b0;
      end else if (ec_fire) begin
        ec_idx   <= ec_idx + 9'd1;
        ec_lastq <= (ec_idx + 9'd2 == c_nfire);
        ec_doneq <= ec_lastq;
        if (c_em == EM_CRT) ec_ph <= (ec_ph == 2'd2) ? 2'd0 : ec_ph + 2'd1;
        w_last <= ec_lastq;
        w_self <= ec_selfire;
        w_ph   <= (c_em == EM_CRT) ? ec_ph : 2'd0;
        w_ridx <= ec_idx[2:0];
      end
    end
  end
  always_ff @(posedge clk) if (ec_fire && ec_need != 3'd0) cf <= cq5;

  // ---- stage X: conversion from W ----
  // q=257 zero-selector bookkeeping
  logic [15:0] sel_or;
  logic [3:0]  ezc;
  always_comb begin
    logic [3:0] c;
    c = 4'd0;
    sel_or = '0;
    for (int i = 0; i < 8; i++) begin
      logic [10:0] p;
      logic [15:0] v;
      p = zc + 11'(c);
      if (cf[i][7:0] == 8'h00) begin
        if (p < 11'(c_selbits)) begin
          v = {8'h00, 8'(cf[i][15:8] << p[2:0])};
          sel_or = sel_or | (p[3] ? {v[7:0], 8'h00} : v);
        end
        c = c + 4'd1;
      end
    end
    ezc = c;
  end

  // C2 compression lanes: code = (((x << d) + q/2) * RECIP_Q >> 24) & mask; only the product bits < 28
  // matter: ((x << d) + q/2) * RECIP_Q mod 2^28 = 2^d * (x * RECIP_Q + (q/2) * RECIP_Q / 2^d) (exact).
  // q=257: 16 lanes (2 slot words per step); q=769: 8 lanes.
  logic [15:0][3:0] c2code;
  always_comb begin
    for (int i = 0; i < 16; i++) begin
      logic [24:0] u2;
      u2 = c2u257(cf[i], c_d3);
      c2code[i] = c_d3 ? {1'b0, u2[23:21]} : u2[23:20];
    end
    if (c_q769)
      for (int i = 0; i < 8; i++) begin
        logic [23:0] u7;
        u7 = c2u769(cf[i]);
        c2code[i] = u7[23:20];
      end
  end

  // ---- stage X1 (from W): byte formats, C2 codes, 512-Star partial sums ----
  // 512-Star word of block phase p (tmp(i) = sum_j sext(c_{5i+j}) * 769^j, all mod 2^64):
  //   p0: tmp0 + (tmp3 << 48)   p1: tmp1 + (tmp3[31:16] << 48)   p2: tmp2 + (tmp3[47:32] << 48)
  //   last word (coefficients 1020..1023): tmp204, 5 bytes.
  // X1: A = c0 + 769 c1, B = c2 + 769 c3 (27-bit, carry-propagated), e = c4, for tmp(p) and (p0) tmp3.
  // X2: tmp = A + 769^2 B + 769^4 e in carry-save form (x_s, x_c), tmp3 likewise (x3_s, x3_c).
  // E : e_dat = x_s + x_c + (H << 48), H = tmp3 low 16 bits (p0, from x3_s/x3_c), t3h fields (p1, p2);
  //     in p0 E also resolves t3h = tmp3[47:16] for p1 / p2 (which reach E one / two cycles later).
  logic [63:0] y_d;
  logic [3:0]  y_nb;
  logic [26:0] ya_a, ya_b, y3_a, y3_b;
  logic [15:0] ya_e, y3_e;
  always_comb begin
    logic [4:0][15:0] ca;
    case (w_ph)
      2'd1:    ca = {cf[9], cf[8], cf[7], cf[6], cf[5]};
      2'd2:    ca = {cf[14], cf[13], cf[12], cf[11], cf[10]};
      default: ca = {w_last ? 16'd0 : cf[4], cf[3], cf[2], cf[1], cf[0]};
    endcase
    ya_a = crt_ab(ca[0], ca[1]);
    ya_b = crt_ab(ca[2], ca[3]);
    ya_e = ca[4];
    y3_a = crt_ab(cf[15], cf[16]);
    y3_b = crt_ab(cf[17], cf[18]);
    y3_e = cf[19];
    y_d = '0; y_nb = 4'd8;
    case (c_em)
      EM_SER: y_d = {cf[3], cf[2], cf[1], cf[0]};
      EM_LOW: begin
        if (w_self) begin y_d = {48'd0, selr}; y_nb = 4'(c_senc); end
        else for (int i = 0; i < 8; i++) y_d[8*i +: 8] = cf[i][7:0];
      end
      EM_C2: begin
        if (c_d3) begin
          for (int i = 0; i < 16; i++) y_d[3*i +: 3] = c2code[i][2:0];
          y_nb = 4'd6;
        end else if (c_q769) begin
          for (int i = 0; i < 8; i++) y_d[4*i +: 4] = c2code[i];
          y_nb = 4'd4;
        end else
          for (int i = 0; i < 16; i++) y_d[4*i +: 4] = c2code[i];
      end
      EM_CRT:  if (w_last) y_nb = 4'd5;
      default: y_d = c_rgz ? rg[RG_Z][64*w_ridx +: 64] : rg[RG_PS][64*w_ridx +: 64];
    endcase
  end

  logic        y_v, y_last, y_crt, y_t3;
  logic [1:0]  y_hsel;
  logic [3:0]  y_nbq;
  logic [63:0] y_dq;
  logic [26:0] yq_a, yq_b, yq3_a, yq3_b;
  logic [15:0] yq_e, yq3_e;
  logic        x_v, x_last, x_t3;
  logic [1:0]  x_hsel;
  logic [3:0]  x_nb;
  logic [63:0] x_s, x_c;
  logic [47:0] x3_s, x3_c;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      zc <= '0; selr <= '0;
      y_v <= 1'b0; y_last <= 1'b0; y_crt <= 1'b0; y_t3 <= 1'b0; y_hsel <= '0; y_nbq <= '0;
      x_v <= 1'b0; x_last <= 1'b0; x_t3 <= 1'b0; x_hsel <= '0; x_nb <= '0;
      e_v <= 1'b0; e_last <= 1'b0; e_nb <= '0;
    end else begin
      y_v    <= w_v;
      y_last <= w_v && w_last;
      x_v    <= y_v;
      x_last <= y_last;
      e_v    <= x_v;
      e_last <= x_last;
      e_nb   <= x_nb;
      if (st == S_SETUP) begin
        zc <= '0; selr <= '0;
      end else if (w_v) begin
        y_nbq  <= y_nb;
        y_crt  <= (c_em == EM_CRT);
        y_t3   <= (c_em == EM_CRT) && (w_ph == 2'd0) && !w_last;
        y_hsel <= (c_em == EM_CRT) ? w_ph : 2'd0;
        if (c_em == EM_LOW && !w_self) begin
          zc   <= zc + 11'(ezc);
          selr <= selr | sel_or;
        end
      end
      if (y_v) begin
        x_nb   <= y_nbq;
        x_t3   <= y_t3;
        x_hsel <= y_t3 ? 2'd3 : y_hsel;
      end
    end
  end
  always_ff @(posedge clk) begin
    if (w_v) begin
      y_dq <= y_d;
      yq_a <= ya_a; yq_b <= ya_b; yq_e <= ya_e;
      yq3_a <= y3_a; yq3_b <= y3_b; yq3_e <= y3_e;
    end
    // ---- stage X2 ----
    if (y_v) begin
      logic [127:0] t;
      logic [95:0]  t3;
      t  = crt_x2_cs(yq_a, yq_b, yq_e);
      t3 = crt_x3_cs(yq3_a, yq3_b, yq3_e);
      x_s  <= y_crt ? t[63:0] : y_dq;
      x_c  <= y_crt ? t[127:64] : 64'd0;
      x3_s <= t3[47:0];
      x3_c <= t3[95:48];
    end
    // ---- stage E ----
    if (x_v) begin
      logic [15:0] hs, hc;
      logic [63:0] o1, o2, o3, o4, s1, c1, s2, c2;
      logic [47:0] t3;
      case (x_hsel)
        2'd1:    begin hs = t3h[15:0];  hc = 16'd0;      end
        2'd2:    begin hs = t3h[31:16]; hc = 16'd0;      end
        2'd3:    begin hs = x3_s[15:0]; hc = x3_c[15:0]; end   // p0: + (tmp3 << 48)
        default: begin hs = 16'd0;      hc = 16'd0;      end
      endcase
      // 4:2 compression of x_s, x_c, hs<<48, hc<<48, then one carry-propagate add
      o1 = x_s; o2 = x_c; o3 = {hs, 48'd0}; o4 = {hc, 48'd0};
      s1 = o1 ^ o2 ^ o3;
      c1 = ((o1 & o2) | (o1 & o3) | (o2 & o3)) << 1;
      s2 = s1 ^ c1 ^ o4;
      c2 = ((s1 & c1) | (s1 & o4) | (c1 & o4)) << 1;
      e_dat <= s2 + c2;
      t3 = x3_s + x3_c;
      if (x_t3) t3h <= t3[47:16];
    end
  end

  // ---------------- byte aligner ----------------
  logic        al_emit;
  logic [63:0] al_wd;
  logic [7:0]  al_wbe;
  always_comb begin
    logic [127:0] win;
    logic [15:0]  wm;
    logic [4:0]   nf;
    win = {64'd0, al_acc} | ({64'd0, e_dat} << {al_f, 3'b000});
    wm  = {8'd0, al_m} | (((16'd1 << e_nb) - 16'd1) << al_f);
    nf  = 5'(al_f) + 5'(e_nb);
    al_emit = 1'b0; al_wd = win[63:0]; al_wbe = wm[7:0];
    if (e_v) al_emit = nf[3] | nf[4];
    else if (al_fl) begin al_emit = (al_m != 8'h00); al_wd = al_acc; al_wbe = al_m; end
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      al_f <= '0; al_m <= '0; al_addr <= '0; al_fl <= 1'b0; al_done <= 1'b0;
    end else if (st == S_INIT) begin
      al_f <= c_base[2:0]; al_m <= '0; al_addr <= c_base[12:3]; al_fl <= 1'b0; al_done <= 1'b0;
    end else if (e_v) begin
      logic [15:0] wm;
      logic [4:0]  nf;
      wm = {8'd0, al_m} | (((16'd1 << e_nb) - 16'd1) << al_f);
      nf = 5'(al_f) + 5'(e_nb);
      al_f <= nf[2:0];
      if (nf >= 5'd8) begin al_m <= wm[15:8]; al_addr <= al_addr + 10'd1; end
      else            al_m <= wm[7:0];
      al_fl <= e_last;
    end else if (al_fl) begin
      al_fl <= 1'b0; al_done <= 1'b1;
    end
  end
  always_ff @(posedge clk) begin
    if (st == S_SETUP) al_acc <= '0;
    else if (e_v) begin
      logic [127:0] win;
      logic [4:0]   nf;
      win = {64'd0, al_acc} | ({64'd0, e_dat} << {al_f, 3'b000});
      nf  = 5'(al_f) + 5'(e_nb);
      al_acc <= (nf >= 5'd8) ? win[127:64] : win[63:0];
    end
  end

  // ---------------- output stage: write or compare ----------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      o_v <= 1'b0; cp1_v <= 1'b0; cp2_v <= 1'b0;
    end else begin
      o_v   <= al_emit;
      cp1_v <= o_v && r_cmp;
      cp2_v <= cp1_v;
    end
  end
  always_ff @(posedge clk) begin
    if (al_emit) begin o_addr <= al_addr; o_be <= al_wbe; o_dat <= al_wd; end
    cp1_be <= o_be; cp1_d <= o_dat;
    cp2_be <= cp1_be; cp2_d <= cp1_d;
  end

  always_comb begin
    cd_owen = '0; cd_oren = '0;
    for (int b = 0; b < 3; b++) begin
      cd_owaddr[b] = o_addr; cd_owbe[b] = o_be; cd_owdata[b] = o_dat; cd_oraddr[b] = rd_addr;
    end
    cd_owaddr[OB_SK] = (c_dual && c_buf == OB_PK) ? o_addr + c_skoff : o_addr;
    if (o_v && !r_cmp) begin
      cd_owen[c_buf] = 1'b1;
      if (c_dual) cd_owen[OB_SK] = 1'b1;
    end
    if (o_v && r_cmp) begin
      cd_oren[OB_CT]   = 1'b1;
      cd_oraddr[OB_CT] = o_addr;
    end
    if (rd_iss) cd_oren[c_buf] = 1'b1;
  end

  // ---------------- flags ----------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ne <= 1'b0; ovf <= 1'b0;
    end else begin
      logic mism;
      mism = 1'b0;
      for (int b = 0; b < 8; b++)
        if (cp2_be[b] && (cp2_d[8*b +: 8] != cd_ordata[OB_CT][8*b +: 8])) mism = 1'b1;
      ne <= (ne && !cmp_clr) || (cp2_v && mism);
      if (st == S_SETUP) ovf <= 1'b0;
      else if (st == S_RUN && fin && c_pkenc) ovf <= (zc > 11'(c_selbits));
    end
  end

  // ---------------- completion ----------------
  always_comb begin
    if (c_enc)
      fin = ec_doneq && !w_v && !y_v && !x_v && !e_v && al_done && !o_v && !cp1_v && !cp2_v;
    else
      fin = (dc_idx == c_nfire) && !dw_v && !dgx_v && (dg_v == '0) && (oq_cnt == 4'd0 || c_dm == DM_RG) &&
            !rd_v1 && !rd_v2 && !rg_we_q;
  end

endmodule
