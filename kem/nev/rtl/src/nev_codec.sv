// nev_codec: polynomial <-> object-field codec.
//
// Modes: 0 UNPACK, 1 PACK, 2 PACKCMP, 3 DECOMP, 4 COMP, 5 COMPCMP, 6 TOMSG.
//
// Structure
//  * Byte window `win` (WBY bytes, byte 0 = oldest = bits [7:0]): a shift register holding the
//    field byte stream between the object memory and the coefficient side.
//      field -> poly  (UNPACK, DECOMP): object words (via a 4-deep read FIFO) are appended 8 bytes
//        at a time; whole units are drained from the front: 24 bytes (q=3329, 16 coefficients),
//        21 bytes (q=1409, 16 coefficients), 16 bytes (DECOMP, 16 coefficients), 6 bytes (q=769
//        group, 5 coefficients) and the q=769 tail (3/5/4 bytes, 2/4/3 coefficients).
//      poly -> field  (PACK, PACKCMP, COMP, COMPCMP): encoded units are appended; 8 bytes are
//        drained per object word written (or compared with the object word read back).
//        `wres` counts window bytes plus bytes of units already started (reservation).
//  * Coefficient queue `cq` (CQ entries of 12 bits) for q=769 UNPACK/PACK/PACKCMP: re-blocks
//    groups of 5 coefficients (and the tail) to/from 16-coefficient buffer words.
//  * nev_codec_dec5 / nev_codec_enc5: pipelined decode5 / encode5.
//  * TOMSG: one buffer word per cycle into 2x16 flipabs accumulators; message bits are written
//    into the (otherwise idle) window register, which drives m_wdata.
module nev_codec
  import nev_pkg::*;
(
  input  logic         clk,
  input  logic         rst_n,
  // parameter set (latched by the top for the whole operation)
  input  logic [1:0]   qsel,
  input  logic [1:0]   nlog,
  input  logic         d64,
  input  logic [1:0]   slog,
  input  logic         comp,
  input  logic [11:0]  poly_bytes,
  // command
  input  logic         cmd_valid,
  input  logic [2:0]   cmd_mode,
  input  logic [1:0]   cmd_field,
  input  logic [2:0]   cmd_buf,
  output logic         busy,
  output logic         done,
  output logic         mismatch,
  output logic         ncanon,          // UNPACK: the field bytes are not a canonical encoding
  output logic [7:0]   pw_cnt,          // poly words written (natural order)
  output logic         pw_nx,           // a poly word is written in the next cycle
  output logic [8:0]   wr_words,
  output logic         m_we,
  output logic [511:0] m_wdata,
  // object memory port
  output logic         om_req,
  output logic         om_we,
  output logic [1:0]   om_sel,
  output logic [8:0]   om_addr,
  output logic [127:0] om_wdata,
  output logic [15:0]  om_be,
  output logic         om_wide,         // COMP / COMPCMP: whole 16-byte CT row per access
  input  logic         om_gnt,
  input  logic         om_rvalid,
  input  logic [127:0] om_rdata,
  // poly buffer read port
  output logic         pr_valid,
  output logic [2:0]   pr_buf,
  output logic [6:0]   pr_addr,
  input  logic [191:0] pr_rdata,
  input  logic         pr_gnt,          // read request accepted this cycle (else it is held and retried)
  input  logic [7:0]   pr_lim,          // buffer words available for reading (streaming behind a producer)
  // poly buffer write port
  output logic         pw_valid,
  output logic [2:0]   pw_buf,
  output logic [6:0]   pw_addr,
  output logic [191:0] pw_data
);

  localparam int WBY = 64;         // byte window (bytes); 64 also holds the 512-bit TOMSG message
  localparam int CQ  = 40;         // q=769 coefficient queue entries (32 costs ~40% PACK throughput)
  localparam int RFD = 4;          // object read FIFO depth (words)
  localparam int WB  = WBY * 8;    // window bits
  localparam int CQB = CQ * 12;    // queue bits

  // ------------------------------------------------------------------ helper functions
  function automatic logic [11:0] canon(input logic [11:0] x, input logic [11:0] q);
    return (x >= q) ? (x - q) : x;
  endfunction

  // flipabs of the reference: r = x - (q>>1); |r|
  function automatic logic [11:0] flipabs(input logic [11:0] x, input logic [10:0] qh);
    logic [12:0] r;
    r = {1'b0, x} - {2'b00, qh};
    return r[12] ? 12'(-r) : r[11:0];          // |r| (reference (r + m) ^ m with m = r >> 15)
  endfunction

  // left shift of a queue vector by k 12-bit entries
  function automatic logic [CQB-1:0] cq_shl(input logic [CQB-1:0] v, input logic [5:0] k);
    logic [CQB-1:0] t;
    t = v;
    if (k[0]) t = t << 12;
    if (k[1]) t = t << 24;
    if (k[2]) t = t << 48;
    if (k[3]) t = t << 96;
    if (k[4]) t = t << 192;
    if (k[5]) t = t << 384;
    return t;
  endfunction

  // ------------------------------------------------------------------ registers
  logic         busy_r, done_r, mism_r, m_we_r;
  logic [2:0]   mode_r;
  logic [1:0]   field_r;
  logic [2:0]   buf_r;
  logic         q769_r, q1409_r, d64_r;
  logic [11:0]  q_r;
  logic [10:0]  qh_r;
  logic [11:0]  len_r;
  logic [8:0]   nwom_r;
  logic [7:0]   nbw_r;
  logic [8:0]   ngrp_r;
  logic [2:0]   kt_r, tb_r;

  // object port
  logic         om_req_r, om_we_r;
  logic [8:0]   om_addr_r;
  logic [127:0] om_wdata_r;
  logic [15:0]  om_be_r;
  logic [8:0]   rd_iss;          // reads granted
  logic [2:0]   rf_occ;          // FIFO slots reserved (granted, not popped)
  logic [127:0] rf_mem [RFD];
  logic [1:0]   rf_wp, rf_rp;
  logic [2:0]   rf_cnt;
  logic [8:0]   ow_cnt;          // object words drained from the window (written or compared)
  logic [8:0]   wr_words_r;      // writes granted

  // window
  logic [WB-1:0] win;
  logic [6:0]    wcnt;
  logic [6:0]    wres;

  // unit / group bookkeeping
  logic [8:0]   u_cnt;           // UNPACK/DECOMP units drained (non-group modes)
  logic [8:0]   grp_cnt;         // q=769 groups drained / popped
  logic         tail_drn;        // UNPACK tail drained
  logic         tail_pend;       // UNPACK tail decoded, waiting for the dec5 pipeline to empty
  logic [47:0]  tail_x;
  logic         tail_done;       // PACK tail popped

  // coefficient queue
  logic [CQB-1:0] cq;
  logic [5:0]     cq_cnt;
  logic [2:0]     cq_infl;       // poly reads in flight towards cq

  // poly read
  logic         pr_valid_r;
  logic [6:0]   pr_addr_r;
  logic [7:0]   pr_iss;
  logic         prv1, prv2, prd_v;
  logic [191:0] prd_r;

  // poly write
  logic         pw_valid_r;
  logic [6:0]   pw_addr_r;
  logic [191:0] pw_data_r;
  logic [7:0]   pw_iss;

  // q=1409 unpack pipeline
  logic         u1_v, u2_v;
  logic [167:0] u1_t;
  logic [87:0]  u2_q0;           // quotient estimate (true quotient is u2_q0 + 0..2)
  logic [103:0] u2_r0;           // t - 1409*u2_q0, in [0, 4227)

  // TOMSG
  logic [6:0]   mwi;
  logic [12:0]  acc [2][16];

  // ------------------------------------------------------------------ command decode
  logic [11:0] c_len;
  logic [8:0]  c_ngrp;
  logic [2:0]  c_kt, c_tb;
  logic [11:0] c_lenp7;
  always_comb begin : cfg_comb
    case (nlog)
      2'd0:    begin c_ngrp = 9'd102; c_kt = 3'd2; c_tb = 3'd3; end
      2'd1:    begin c_ngrp = 9'd204; c_kt = 3'd4; c_tb = 3'd5; end
      default: begin c_ngrp = 9'd409; c_kt = 3'd3; c_tb = 3'd4; end
    endcase
    if (cmd_mode == 3'd3 || cmd_mode == 3'd4 || cmd_mode == 3'd5)
      c_len = 12'd512 << nlog;
    else
      c_len = poly_bytes;
    c_lenp7 = c_len + 12'd7;
  end

  logic m_unp, m_dec, m_msg, f_in, f_out, f_wr, f_cc, f_rd, f_grp, f_cpz, f_msg;
  assign m_unp = (mode_r == 3'd0);
  assign m_dec = (mode_r == 3'd3);
  assign m_msg = (mode_r == 3'd6);
  assign f_in  = busy_r & (m_unp | m_dec);
  assign f_wr  = busy_r & (mode_r == 3'd1 || mode_r == 3'd4);
  assign f_cc  = busy_r & (mode_r == 3'd2 || mode_r == 3'd5);
  assign f_out = f_wr | f_cc;
  assign f_rd  = f_in | f_cc;
  assign f_cpz = (mode_r == 3'd3 || mode_r == 3'd4 || mode_r == 3'd5);
  assign f_grp = q769_r & ~f_cpz & (mode_r != 3'd6);
  assign f_msg = busy_r & m_msg;

  // ------------------------------------------------------------------ dec5 / enc5
  logic        dec_iv, dec_ov, dec_busy, dec_nc;
  logic        ncan_r, nc_n;
  logic [47:0] dec_ib;
  logic [59:0] dec_ox;
  logic        enc_iv, enc_ov, enc_busy;
  logic [47:0] enc_ob;

  nev_codec_dec5 u_dec5 (
    .clk(clk), .rst_n(rst_n), .in_v(dec_iv), .in_b(dec_ib),
    .out_v(dec_ov), .out_x(dec_ox), .out_nc(dec_nc), .busy(dec_busy)
  );

  nev_codec_enc5 u_enc5 (
    .clk(clk), .rst_n(rst_n), .in_v(enc_iv), .in_x(cq[59:0]),
    .out_v(enc_ov), .out_b(enc_ob), .busy(enc_busy)
  );

  // ------------------------------------------------------------------ main datapath / control
  logic          rd_gnt, wr_gnt;
  logic [127:0]  rf_head;
  logic          wide;           // COMP / COMPCMP: 16-byte object accesses
  logic [8:0]    winc;           // object words per access
  logic [11:0]   rem;
  logic [3:0]    need;
  logic [7:0]    bemask;
  logic [127:0]  bemask128;
  logic          word_rdy, wr_load, cmp_go, dr_out;
  logic [6:0]    dbytes_out;
  logic          grp_left, unit_left, in_drain;
  logic [6:0]    dlen_in, damt, cnt_ad;
  logic [WB-1:0] win_ad, win_n;
  logic          in_app, rf_pop;
  logic [191:0]  adata;
  logic [6:0]    alen;
  logic [6:0]    wcnt_n, wres_ad, wres_n;
  logic [4:0]    csz;
  logic          gen_iss, cq_iss, msg_iss, pr_go;
  logic          pr_hold;
  assign pr_hold = pr_valid_r & ~pr_gnt;
  // chunk to append in poly -> field modes
  logic          chunk_v;
  logic [191:0]  chunk;
  logic [4:0]    clen;
  // queue
  logic          cq_pop16, cq_pop5, cq_popt;
  logic [5:0]    popn, cq_cnt_ad, cq_cnt_n;
  logic [CQB-1:0] cq_ad, cq_n;
  logic [191:0]  cq_pushd;
  logic [4:0]    pushn;
  logic          prd_push;
  logic          tail_push;
  logic [7:0]    cq_tot;
  // poly write
  logic          pw_go;
  logic [191:0]  pw_d;
  // misc next state
  logic          fin_in, fin_wr, fin_cc, fin_msg, fin;
  logic [47:0]   tail_dec;
  logic          tail_nc;
  logic [12:0]   acc_n [2][16];
  logic          msg_wr;
  logic [4:0]    msg_slot;
  logic [15:0]   msg_bits;

  assign rd_gnt  = om_req_r & ~om_we_r & om_gnt;
  assign wr_gnt  = om_req_r &  om_we_r & om_gnt;
  assign rf_head = rf_mem[rf_rp];
  assign wide    = busy_r & (mode_r == 3'd3 || mode_r == 3'd4 || mode_r == 3'd5);   // DECOMP / COMP / COMPCMP: 16-byte CT rows
  assign winc    = wide ? 9'd2 : 9'd1;

  always_comb begin : main_comb
    integer l;
    logic [11:0] t0, t1;
    logic [20:0] tp;
    logic [20:0] cb;
    logic [17:0] db;
    logic [39:0] tv;
    logic        first, last, hsel;
    logic [12:0] s;
    logic [12:0] r13;

    // ---------------- poly -> field: object word ready in the window
    rem      = len_r - {ow_cnt, 3'b000};
    // compressed ciphertexts are N bytes (multiple of 16): whole 16-byte rows in wide mode
    need     = wide ? 4'd8 : ((rem >= 12'd8) ? 4'd8 : rem[3:0]);
    bemask   = (rem >= 12'd8) ? 8'hff : ~(8'hff << rem[2:0]);
    for (l = 0; l < 16; l = l + 1) bemask128[8*l +: 8] = (wide || (l < 8 && bemask[l])) ? 8'hff : 8'h00;
    word_rdy = f_out & (ow_cnt < nwom_r) & (wcnt >= (wide ? 7'd16 : {3'b000, need}));
    wr_load  = f_wr & word_rdy & (~om_req_r | om_gnt);
    cmp_go   = f_cc & word_rdy & (rf_cnt != 3'd0);
    dr_out   = wr_load | cmp_go;
    dbytes_out = wide ? 7'd16 : ((wcnt >= 7'd8) ? 7'd8 : wcnt);

    // ---------------- field -> poly: unit drain
    grp_left  = (grp_cnt < ngrp_r);
    unit_left = f_grp ? (grp_left | ~tail_drn) : (u_cnt < {1'b0, nbw_r});
    if (f_grp)        dlen_in = grp_left ? 7'd6 : {4'd0, tb_r};
    else if (f_cpz)   dlen_in = 7'd16;
    else if (q1409_r) dlen_in = 7'd21;
    else              dlen_in = 7'd24;
    in_drain = f_in & unit_left & (wcnt >= dlen_in);

    // ---------------- window drain
    if (in_drain)    damt = dlen_in;
    else if (dr_out) damt = dbytes_out;
    else             damt = 7'd0;
    cnt_ad = wcnt - damt;
    win_ad = win >> {damt, 3'b000};

    // ---------------- reservation for poly -> field units
    wres_ad = wres - (dr_out ? dbytes_out : 7'd0);
    if (f_cpz)        csz = 5'd16;
    else if (q1409_r) csz = 5'd21;
    else              csz = 5'd24;

    // ---------------- coefficient queue (q=769)
    cq_pop16 = f_in & f_grp & (cq_cnt >= 6'd16);
    cq_pop5  = f_out & f_grp & grp_left & (cq_cnt >= 6'd5) & ({1'b0, wres} + 8'd6 <= WBY);
    cq_popt  = f_out & f_grp & ~grp_left & ~tail_done & (cq_cnt >= {3'd0, kt_r}) & ~enc_busy &
               ({1'b0, wres} + {5'd0, tb_r} <= WBY);
    if (cq_pop16)     popn = 6'd16;
    else if (cq_pop5) popn = 6'd5;
    else if (cq_popt) popn = {3'd0, kt_r};
    else              popn = 6'd0;
    case (popn)
      6'd16:   cq_ad = cq >> 192;
      6'd5:    cq_ad = cq >> 60;
      6'd4:    cq_ad = cq >> 48;
      6'd3:    cq_ad = cq >> 36;
      6'd2:    cq_ad = cq >> 24;
      default: cq_ad = cq;
    endcase
    cq_cnt_ad = cq_cnt - popn;

    tail_push = f_in & f_grp & tail_pend & ~dec_busy;
    prd_push  = prd_v & f_out & f_grp;
    cq_pushd  = '0;
    pushn     = 5'd0;
    if (prd_push) begin
      cq_pushd = prd_r;            pushn = 5'd16;
    end else if (dec_ov) begin
      cq_pushd[59:0] = dec_ox;     pushn = 5'd5;
    end else if (tail_push) begin
      cq_pushd[47:0] = tail_x;     pushn = {2'd0, kt_r};
    end
    cq_n     = cq_ad | cq_shl({{(CQB-192){1'b0}}, cq_pushd}, cq_cnt_ad);
    cq_cnt_n = cq_cnt_ad + {1'b0, pushn};
    cq_tot   = {2'b00, cq_cnt_ad} + {1'b0, cq_infl, 4'b0000};
    cq_iss   = f_out & f_grp & (pr_iss < nbw_r) & (pr_iss < pr_lim) & ~pr_hold & (cq_tot + 8'd16 <= CQ);

    // ---------------- poly read issue
    gen_iss = f_out & ~f_grp & (pr_iss < nbw_r) & (pr_iss < pr_lim) & ~pr_hold & ({1'b0, wres_ad} + {3'b000, csz} <= WBY);
    msg_iss = f_msg & (pr_iss < nbw_r) & (pr_iss < pr_lim) & ~pr_hold;
    pr_go   = gen_iss | cq_iss | msg_iss;

    wres_n = wres_ad + (gen_iss ? {2'b00, csz} : 7'd0) + (cq_pop5 ? 7'd6 : 7'd0) +
             (cq_popt ? {4'd0, tb_r} : 7'd0);

    // ---------------- encoded chunk (poly -> field)
    chunk_v = 1'b0;
    chunk   = '0;
    clen    = 5'd0;
    if (prd_v & f_out & ~f_grp) begin
      chunk_v = 1'b1;
      if (f_cpz) begin
        clen = 5'd16;
        for (l = 0; l < 16; l = l + 1) begin
          cb = {9'd0, prd_r[12*l +: 12]} * 21'd341 + 21'd469;
          chunk[8*l +: 8] = cb[17:10];
        end
      end else if (q1409_r) begin
        clen = 5'd21;
        for (l = 0; l < 8; l = l + 1) begin
          t0 = prd_r[24*l +: 12];
          t1 = prd_r[24*l+12 +: 12];
          tp = {9'd0, t1} * 21'd1409 + {9'd0, t0};
          chunk[21*l +: 21] = tp;
        end
      end else begin
        clen  = 5'd24;
        chunk = prd_r;
      end
    end else if (enc_ov) begin
      chunk_v     = 1'b1;
      clen        = 5'd6;
      chunk[47:0] = enc_ob;
    end else if (cq_popt) begin
      chunk_v = 1'b1;
      clen    = {2'b00, tb_r};
      tv      = '0;
      for (l = 0; l < 4; l = l + 1)
        if (l < kt_r) tv[10*l +: 10] = cq[12*l +: 10];
      chunk[39:0] = tv;
    end

    // ---------------- window append
    in_app = f_in & (rf_cnt != 3'd0) & (cnt_ad <= (wide ? WBY - 16 : WBY - 8));
    rf_pop = in_app | cmp_go;
    if (in_app) begin
      adata = wide ? {64'd0, rf_head} : {128'd0, rf_head[63:0]};
      alen  = wide ? 7'd16 : 7'd8;
    end else if (chunk_v) begin
      adata = chunk;
      alen  = {2'b00, clen};
    end else begin
      adata = '0;
      alen  = 7'd0;
    end
    win_n  = win_ad | ({{(WB-192){1'b0}}, adata} << {cnt_ad, 3'b000});
    wcnt_n = cnt_ad + alen;

    // ---------------- UNPACK tail decode (10-bit stream, canonicalised)
    tail_dec = '0;
    tv = win[39:0];
    tail_nc = 1'b0;
    for (l = 0; l < 40; l = l + 1)
      if ((l >= 10 * kt_r) && (l < 8 * tb_r) && tv[l]) tail_nc = 1'b1;     // nonzero padding bits
    for (l = 0; l < 4; l = l + 1)
      if ((l < kt_r) && (tv[10*l +: 10] >= 10'd769)) tail_nc = 1'b1;
    for (l = 0; l < 4; l = l + 1)
      if (l < kt_r) tail_dec[12*l +: 12] = canon({2'b00, tv[10*l +: 10]}, 12'd769);

    // ---------------- poly write data
    pw_go = 1'b0;
    pw_d  = '0;
    nc_n  = 1'b0;
    if (dec_ov & dec_nc) nc_n = 1'b1;
    if (in_drain & ~f_grp & m_dec) begin
      pw_go = 1'b1;
      for (l = 0; l < 16; l = l + 1) begin
        db = {10'd0, win[8*l +: 8]} * 18'd769 + 18'd128;
        pw_d[12*l +: 12] = {2'b00, db[17:8]};
      end
    end else if (in_drain & ~f_grp & ~q1409_r) begin
      pw_go = 1'b1;
      for (l = 0; l < 16; l = l + 1) begin
        pw_d[12*l +: 12] = canon(win[12*l +: 12], 12'd3329);
        if (win[12*l +: 12] >= 12'd3329) nc_n = 1'b1;
      end
    end else if (u2_v) begin
      pw_go = 1'b1;
      for (l = 0; l < 8; l = l + 1) begin
        r13 = u2_r0[13*l +: 13];
        t1  = {1'b0, u2_q0[11*l +: 11]};
        if (r13 >= 13'd2818)      begin t0 = r13[11:0] - 12'd2818; t1 = t1 + 12'd2; end
        else if (r13 >= 13'd1409) begin t0 = r13[11:0] - 12'd1409; t1 = t1 + 12'd1; end
        else                            t0 = r13[11:0];
        if (t1 >= 12'd1409) nc_n = 1'b1;
        pw_d[24*l +: 12]    = t0;
        pw_d[24*l+12 +: 12] = canon(t1, 12'd1409);
      end
    end else if (cq_pop16) begin
      pw_go = 1'b1;
      pw_d  = cq[191:0];
    end

    // ---------------- TOMSG
    for (l = 0; l < 16; l = l + 1) begin
      acc_n[0][l] = acc[0][l];
      acc_n[1][l] = acc[1][l];
    end
    msg_wr   = 1'b0;
    msg_bits = '0;
    hsel     = ~d64_r & mwi[0];
    first    = d64_r ? (mwi[1:0] == 2'd0) : (mwi[2:1] == 2'd0);
    last     = d64_r ? (mwi[1:0] == 2'd3) : (mwi[2:1] == 2'd3);
    msg_slot = d64_r ? mwi[6:2] : {mwi[6:3], mwi[0]};
    if (prd_v & f_msg) begin
      for (l = 0; l < 16; l = l + 1) begin
        s = (first ? 13'd0 : acc[hsel][l]) + {1'b0, flipabs(prd_r[12*l +: 12], qh_r)};
        acc_n[hsel][l] = s;
        msg_bits[l] = (s < {1'b0, q_r});
      end
      msg_wr = last;
    end
    if (msg_wr) win_n[16*msg_slot +: 16] = msg_bits;

    // ---------------- completion
    fin_in  = f_in & pw_valid_r & (pw_iss == nbw_r);
    fin_wr  = f_wr & wr_gnt & (wr_words_r + winc == nwom_r);
    fin_cc  = cmp_go & (ow_cnt + winc == nwom_r);
    fin_msg = f_msg & prd_v & (mwi == nbw_r[6:0] - 7'd1);
    fin     = fin_in | fin_wr | fin_cc | fin_msg;
  end

  // q=1409 divmod, first stage: q0 = floor(floor(t/2^11) * 1488 / 2^10) <= floor(t/1409) <= q0 + 2,
  // r0 = t - 1409*q0 < 4227 (13 bits); the second stage corrects (pw data mux above).
  logic [87:0]  u2_q0_n;
  logic [103:0] u2_r0_n;
  always_comb begin : div1409_comb
    integer l;
    logic [16:0] p;
    logic [10:0] q0;
    logic [12:0] m;
    for (l = 0; l < 8; l = l + 1) begin
      p  = {7'd0, u1_t[21*l+11 +: 10]} * 17'd93;
      q0 = p[16:6];
      m  = {2'd0, q0} * 13'd1409;
      u2_q0_n[11*l +: 11] = q0;
      u2_r0_n[13*l +: 13] = u1_t[21*l +: 13] - m;
    end
  end

  assign dec_iv = in_drain & f_grp & grp_left;
  assign dec_ib = win[47:0];
  assign enc_iv = cq_pop5;

  // ------------------------------------------------------------------ registers
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy_r <= 1'b0; done_r <= 1'b0; mism_r <= 1'b0; m_we_r <= 1'b0;
      mode_r <= '0; field_r <= '0; buf_r <= '0;
      q769_r <= 1'b0; q1409_r <= 1'b0; d64_r <= 1'b0; q_r <= '0; qh_r <= '0;
      len_r <= '0; nwom_r <= '0; nbw_r <= '0; ngrp_r <= '0; kt_r <= '0; tb_r <= '0;
      om_req_r <= 1'b0; om_we_r <= 1'b0; om_addr_r <= '0; om_wdata_r <= '0; om_be_r <= '0;
      rd_iss <= '0; rf_occ <= '0; rf_wp <= '0; rf_rp <= '0; rf_cnt <= '0;
      for (int i = 0; i < RFD; i++) rf_mem[i] <= '0;
      ow_cnt <= '0; wr_words_r <= '0;
      win <= '0; wcnt <= '0; wres <= '0;
      u_cnt <= '0; grp_cnt <= '0; tail_drn <= 1'b0; tail_pend <= 1'b0; tail_x <= '0; ncan_r <= 1'b0;
      tail_done <= 1'b0;
      cq <= '0; cq_cnt <= '0; cq_infl <= '0;
      pr_valid_r <= 1'b0; pr_addr_r <= '0; pr_iss <= '0; prv1 <= 1'b0; prv2 <= 1'b0;
      prd_v <= 1'b0; prd_r <= '0;
      pw_valid_r <= 1'b0; pw_addr_r <= '0; pw_data_r <= '0; pw_iss <= '0;
      u1_v <= 1'b0; u2_v <= 1'b0; u1_t <= '0; u2_q0 <= '0; u2_r0 <= '0;
      mwi <= '0;
      for (int i = 0; i < 16; i++) begin acc[0][i] <= '0; acc[1][i] <= '0; end
    end else begin
      done_r <= busy_r & fin;
      m_we_r <= fin_msg;
      // poly read pipeline (data appears 2 cycles after pr_valid)
      prv1  <= pr_valid_r & pr_gnt;
      prv2  <= prv1;
      prd_v <= prv2;
      if (prv2) prd_r <= pr_rdata;
      // object read data FIFO
      if (om_rvalid & f_rd) begin
        rf_mem[rf_wp] <= om_rdata;
        rf_wp <= rf_wp + 2'd1;
      end
      // q=1409 unpack pipeline
      u1_v <= in_drain & ~f_grp & ~m_dec & q1409_r;
      if (in_drain) u1_t <= win[167:0];
      u2_v <= u1_v;
      if (u1_v) begin u2_q0 <= u2_q0_n; u2_r0 <= u2_r0_n; end

      if (cmd_valid & ~busy_r) begin
        busy_r  <= 1'b1;
        mism_r  <= 1'b0;
        mode_r  <= cmd_mode;
        field_r <= cmd_field;
        buf_r   <= cmd_buf;
        q769_r  <= (qsel == Q769);
        q1409_r <= (qsel == Q1409);
        d64_r   <= d64;
        q_r     <= qval(qsel);
        qh_r    <= qval(qsel) >> 1;
        len_r   <= c_len;
        nwom_r  <= c_lenp7[11:3];
        nbw_r   <= nwords(nlog);
        ngrp_r  <= c_ngrp;
        kt_r    <= c_kt;
        tb_r    <= c_tb;
        om_req_r <= 1'b0; om_we_r <= 1'b0;
        rd_iss <= '0; rf_occ <= '0; rf_wp <= '0; rf_rp <= '0; rf_cnt <= '0;
        ow_cnt <= '0; wr_words_r <= '0;
        win <= '0; wcnt <= '0; wres <= '0;
        u_cnt <= '0; grp_cnt <= '0; tail_drn <= 1'b0; tail_pend <= 1'b0; tail_done <= 1'b0; ncan_r <= 1'b0;
        cq <= '0; cq_cnt <= '0; cq_infl <= '0;
        pr_valid_r <= 1'b0; pr_iss <= '0;
        pw_valid_r <= 1'b0; pw_iss <= '0;
        mwi <= '0;
      end else begin
        if (fin) busy_r <= 1'b0;
        if (cmp_go & |((rf_head ^ win[127:0]) & bemask128)) mism_r <= 1'b1;

        // object port
        if (f_rd) begin
          om_we_r   <= 1'b0;
          om_req_r  <= ((rd_iss + (rd_gnt ? winc : 9'd0)) < nwom_r) &&
                       ((rf_occ + {2'd0, rd_gnt} - {2'd0, rf_pop}) < RFD);
          om_addr_r <= rd_iss + (rd_gnt ? winc : 9'd0);
        end else if (f_wr) begin
          om_we_r <= 1'b1;
          if (wr_load) begin
            om_req_r   <= 1'b1;
            om_addr_r  <= ow_cnt;
            om_wdata_r <= win[127:0];
            om_be_r    <= wide ? 16'hffff : {8'h00, bemask};
          end else if (om_gnt) begin
            om_req_r <= 1'b0;
          end
        end else begin
          om_req_r <= 1'b0;
        end
        rd_iss <= rd_iss + (rd_gnt ? winc : 9'd0);
        rf_occ <= rf_occ + {2'd0, rd_gnt} - {2'd0, rf_pop};
        rf_cnt <= rf_cnt + {2'd0, om_rvalid & f_rd} - {2'd0, rf_pop};
        if (rf_pop) rf_rp <= rf_rp + 2'd1;
        if (dr_out) ow_cnt <= ow_cnt + winc;
        if (wr_gnt) wr_words_r <= wr_words_r + winc;

        // window
        win  <= win_n;
        wcnt <= wcnt_n;
        wres <= wres_n;

        // units
        if (in_drain & ~f_grp) u_cnt <= u_cnt + 9'd1;
        if ((in_drain & f_grp & grp_left) | cq_pop5) grp_cnt <= grp_cnt + 9'd1;
        if (nc_n || (in_drain & f_grp & ~grp_left & tail_nc)) ncan_r <= 1'b1;
        if (in_drain & f_grp & ~grp_left) begin
          tail_drn  <= 1'b1;
          tail_pend <= 1'b1;
          tail_x    <= tail_dec;
        end
        if (tail_push) tail_pend <= 1'b0;
        if (cq_popt) tail_done <= 1'b1;

        // queue
        cq      <= cq_n;
        cq_cnt  <= cq_cnt_n;
        cq_infl <= cq_infl + {2'b00, cq_iss} - {2'b00, prd_push};

        // poly read issue
        if (!pr_hold) begin
          pr_valid_r <= pr_go;
          if (pr_go) pr_addr_r <= pr_iss[6:0];
          pr_iss <= pr_iss + {7'd0, pr_go};
        end

        // poly write
        pw_valid_r <= pw_go;
        if (pw_go) begin
          pw_addr_r <= pw_iss[6:0];
          pw_data_r <= pw_d;
        end
        pw_iss <= pw_iss + {7'd0, pw_go};

        // TOMSG
        if (prd_v & f_msg) mwi <= mwi + 7'd1;
        for (int i = 0; i < 16; i++) begin
          acc[0][i] <= acc_n[0][i];
          acc[1][i] <= acc_n[1][i];
        end
      end
    end
  end

  // ------------------------------------------------------------------ outputs
  assign busy     = busy_r;
  assign done     = done_r;
  assign mismatch = mism_r;
  assign ncanon   = ncan_r;
  assign pw_cnt   = pw_iss;
  assign pw_nx    = pw_go;
  assign wr_words = wr_words_r;
  assign m_we     = m_we_r;
  assign m_wdata  = win;
  assign om_req   = om_req_r;
  assign om_we    = om_we_r;
  assign om_sel   = field_r;
  assign om_addr  = om_addr_r;
  assign om_wdata = om_wdata_r;
  assign om_be    = om_be_r;
  assign om_wide  = wide;
  assign pr_valid = pr_valid_r;
  assign pr_buf   = buf_r;
  assign pr_addr  = pr_addr_r;
  assign pw_valid = pw_valid_r;
  assign pw_buf   = buf_r;
  assign pw_addr  = pw_addr_r;
  assign pw_data  = pw_data_r;

endmodule
