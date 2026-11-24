// HARE KEM accelerator, performance version (KeyGen / Enc / Dec, HARE-128/256/384/512 KR, Mode-B).
module hare_kem_top import hare_pkg::*; (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        cmd_valid,
  output logic        cmd_ready,
  input  logic [1:0]  cmd_op,
  input  logic [1:0]  cmd_lvl,
  output logic        busy,
  output logic        done,
  output logic        res_valid,
  output logic        dec_fail,
  input  logic        rnd_valid,
  output logic        rnd_ready,
  input  logic [63:0] rnd_data,
  input  logic        hst_en,
  input  logic        hst_we,
  input  logic [1:0]  hst_sel,
  input  logic [12:0] hst_addr,
  input  logic [63:0] hst_wdata,
  input  logic [7:0]  hst_be,
  output logic [63:0] hst_rdata
);
  // ======================================================================== reset synchroniser
  logic rs1, rs2, rst_i;
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin rs1 <= 1'b0; rs2 <= 1'b0; end
    else        begin rs1 <= 1'b1; rs2 <= rs1; end
  assign rst_i = rs2;   // active-low, asynchronous assertion, synchronous release

  // ======================================================================== command
  localparam logic [1:0] OP_KG = 2'd0, OP_ENC = 2'd1, OP_DEC = 2'd2;
  logic [1:0] op, lvl;
  logic       accept;
  assign cmd_ready = !busy;
  assign accept    = cmd_valid && !busy && (cmd_op != 2'd3);
  // Command acceptance and status flops are reset by rst_n directly (asynchronous assertion and
  // release; rst_n is timed as an input of clk), so cmd_ready = !busy is truthful on every edge
  // after release. The sequencer runs on the synchronised reset rst_i and picks the accepted
  // command up through acc_pend.
  logic acc_pend, acc_take, fin, fin_fail;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy <= 1'b0; done <= 1'b0; res_valid <= 1'b0; dec_fail <= 1'b0; op <= '0; lvl <= '0; acc_pend <= 1'b0;
    end else begin
      done <= 1'b0;
      if (accept) begin
        busy <= 1'b1; op <= cmd_op; lvl <= cmd_lvl; res_valid <= 1'b0; dec_fail <= 1'b0; acc_pend <= 1'b1;
      end else begin
        if (acc_take) acc_pend <= 1'b0;
        if (fin) begin busy <= 1'b0; done <= 1'b1; res_valid <= 1'b1; dec_fail <= fin_fail; end
      end
    end
  end

  // level parameters
  logic [17:0] n;   logic [8:0] w, we;  logic [6:0] S;  logic [14:0] PK, nbytes;  logic [15:0] CT;
  logic [9:0]  nw, nwd;  logic [7:0] N1;
  assign n = p_n(lvl); assign w = p_w(lvl); assign we = p_we(lvl); assign S = p_s(lvl);
  assign PK = p_pk(lvl); assign CT = p_ct(lvl); assign nbytes = p_nbytes(lvl);
  assign nw = p_nw(lvl); assign nwd = p_nwd(lvl); assign N1 = p_n1(lvl);
  logic [2:0] nb32;   // S/32 output blocks (1 or 2)
  assign nb32 = (S > 7'd32) ? 3'd2 : 3'd1;
  logic [2:0] nout2;
  assign nout2 = (S == 7'd16) ? 3'd1 : (S == 7'd32) ? 3'd2 : (S == 7'd48) ? 3'd3 : 3'd4;  // 2S/32


  // ======================================================================== byte registers
  // byte j of a 64-byte register = r[8j+7:8j]
  logic [511:0] r_seedkem, r_m, r_salt, r_sigma, r_seeddk, r_seedek, r_hek, r_theta, r_k, r_kbar;

  // ======================================================================== object buffers
  localparam int NKR = 4, NCR = 5;
  ob_req_t [NKR-1:0] kr;
  ob_req_t [NCR-1:0] cr;
  logic [NKR-1:0] kg;
  logic [NCR-1:0] cg;
  logic [63:0] kq, cq;
  buf_port #(.N(NKR), .AW0(11), .AW1(10)) u_kbuf (.clk, .busy, .h_en(hst_en && hst_sel == 2'd0 && hst_addr < 13'd2751),
    .h_we(hst_we), .h_a(hst_addr), .h_d(hst_wdata), .h_be(hst_be), .r(kr), .gnt(kg), .q(kq));
  buf_port #(.N(NCR), .AW0(12), .AW1(10)) u_cbuf (.clk, .busy, .h_en(hst_en && hst_sel == 2'd1 && hst_addr < 13'd4912),
    .h_we(hst_we), .h_a(hst_addr), .h_d(hst_wdata), .h_be(hst_be), .r(cr), .gnt(cg), .q(cq));

  // host read data
  logic [1:0]  hsel_q;
  logic [12:0] haddr_q;
  always_ff @(posedge clk or negedge rst_i)
    if (!rst_i) begin hsel_q <= '0; haddr_q <= '0; end
    else if (hst_en && !hst_we && !busy) begin hsel_q <= hst_sel; haddr_q <= hst_addr; end
  always_comb begin
    case (hsel_q)
      2'd0: hst_rdata = kq;
      2'd1: hst_rdata = cq;
      2'd2: begin
        hst_rdata = '0;
        for (int b = 0; b < 8; b = b + 1)
          if ({haddr_q[2:0], 3'(b)} < {3'd0, S} && haddr_q[12:3] == 10'd0) hst_rdata[8*b +: 8] = r_k[64*haddr_q[2:0] + 8*b +: 8];
      end
      default: hst_rdata = '0;
    endcase
  end

  // ======================================================================== dense memories
  localparam logic [3:0] CL_NONE = 4'd0, CL_MULA = 4'd1, CL_MULB = 4'd2, CL_DWA = 4'd3, CL_DWB = 4'd4,
                         CL_SMP = 4'd5, CL_CLR = 4'd6, CL_OSR = 4'd7, CL_OSE = 4'd8;
  logic [3:0]   own [4];
  dm_req_t      mreq [4];
  dm_q_t        mq [4];
  dm_req_t      q_mula, q_mulb, q_dwa, q_dwb, q_smp, q_clr, q_osr, q_ose;
  genvar gm;
  generate
    for (gm = 0; gm < 4; gm = gm + 1) begin : g_mem
      always_comb begin
        case (own[gm])
          CL_MULA: mreq[gm] = q_mula;
          CL_MULB: mreq[gm] = q_mulb;
          CL_DWA:  mreq[gm] = q_dwa;
          CL_DWB:  mreq[gm] = q_dwb;
          CL_SMP:  mreq[gm] = q_smp;
          CL_CLR:  mreq[gm] = q_clr;
          CL_OSR:  mreq[gm] = q_osr;
          CL_OSE:  mreq[gm] = q_ose;
          default: mreq[gm] = '0;
        endcase
      end
      // eight 128x256 banks: 256-bit word i in bank i[2:0], address i>>3
      for (genvar gb = 0; gb < 8; gb = gb + 1) begin : g_bank
        sram_sp #(.AW(7), .DW(256), .BW(0)) u_b (.clk, .ce(mreq[gm].ce[gb]), .we(mreq[gm].we[gb]),
          .a(mreq[gm].a[gb]), .d(mreq[gm].d[gb]), .wm(1'b0), .q(mq[gm][gb]));
      end
    end
  endgenerate
  // read-data routing: a client reads the memory it owns (first match)
  function automatic [1:0] find_own(input [3:0] cl, input [3:0] o0, input [3:0] o1, input [3:0] o2, input [3:0] o3);
    if (o0 == cl) find_own = 2'd0;
    else if (o1 == cl) find_own = 2'd1;
    else if (o2 == cl) find_own = 2'd2;
    else find_own = 2'd3;
  endfunction
  // registered one cycle behind own: every reading client's first read data arrives at least two
  // cycles after its role is assigned (multiplier stage 3, sampler/output-stage read states)
  logic [1:0] m_mula, m_mulb, m_smp, m_osr, m_ose;
  always_ff @(posedge clk or negedge rst_i)
    if (!rst_i) begin m_mula <= '0; m_mulb <= '0; m_smp <= '0; m_osr <= '0; m_ose <= '0; end
    else begin
      m_mula <= find_own(CL_MULA, own[0], own[1], own[2], own[3]);
      m_mulb <= find_own(CL_MULB, own[0], own[1], own[2], own[3]);
      m_smp  <= find_own(CL_SMP,  own[0], own[1], own[2], own[3]);
      m_osr  <= find_own(CL_OSR,  own[0], own[1], own[2], own[3]);
      m_ose  <= find_own(CL_OSE,  own[0], own[1], own[2], own[3]);
    end

  // ======================================================================== list SRAM
  logic        l_ce, l_we;  logic [9:0] l_a;  logic [17:0] l_d, l_q;
  logic        ml_ce;       logic [9:0] ml_a;
  logic        sl_ce, sl_we; logic [9:0] sl_a; logic [17:0] sl_d;
  logic        mul_busy;
  always_comb begin
    if (mul_busy) begin l_ce = ml_ce; l_we = 1'b0; l_a = ml_a; l_d = '0; end
    else          begin l_ce = sl_ce; l_we = sl_we; l_a = sl_a; l_d = sl_d; end
  end
  sram_sp #(.AW(10), .DW(18), .BW(0)) u_list (.clk, .ce(l_ce), .we(l_we), .a(l_a), .d(l_d), .wm(1'b0), .q(l_q));

  // ======================================================================== streaming hashers
  logic        hc0_start, hc1_start, hc0_busy, hc1_busy;
  logic [7:0]  hc0_tag, hc1_tag;
  logic [1:0]  hc0_src [3], hc1_src [3];
  logic [15:0] hc0_base [3], hc1_base [3], hc0_len [3], hc1_len [3];
  logic [2:0]  hc0_nout, hc1_nout;
  logic [7:0]  hc0_ridx, hc1_ridx, hc0_rbyte, hc1_rbyte, hc0_rbyte2, hc1_rbyte2;
  logic        hc0_ov, hc1_ov;  logic [1:0] hc0_oi, hc1_oi;  logic [255:0] hc0_od, hc1_od;
  ob_req_t     hc0_kreq, hc0_creq, hc1_kreq, hc1_creq;
  function automatic [7:0] regbyte(input [7:0] idx, input [511:0] a, input [511:0] b, input [511:0] c, input [511:0] d);
    case (idx[7:6])
      2'd0: regbyte = a[8*idx[5:0] +: 8];
      2'd1: regbyte = b[8*idx[5:0] +: 8];
      2'd2: regbyte = c[8*idx[5:0] +: 8];
      default: regbyte = d[8*idx[5:0] +: 8];
    endcase
  endfunction
  always_comb begin
    hc0_rbyte  = regbyte(hc0_ridx, r_hek, r_m, r_salt, r_sigma);
    hc1_rbyte  = regbyte(hc1_ridx, r_hek, r_m, r_salt, r_sigma);
    hc0_rbyte2 = regbyte(hc0_ridx + 8'd1, r_hek, r_m, r_salt, r_sigma);
    hc1_rbyte2 = regbyte(hc1_ridx + 8'd1, r_hek, r_m, r_salt, r_sigma);
  end
  hc_stream u_hc0 (.clk, .rst_n(rst_i), .start(hc0_start), .abort(1'b0), .tag(hc0_tag), .seg_src(hc0_src),
    .seg_base(hc0_base), .seg_len(hc0_len), .nout(hc0_nout), .reg_idx(hc0_ridx), .reg_byte(hc0_rbyte), .reg_byte2(hc0_rbyte2),
    .kreq(hc0_kreq), .kgnt(kg[1]), .kq, .creq(hc0_creq), .cgnt(cg[2]), .cq,
    .out_valid(hc0_ov), .out_idx(hc0_oi), .out_data(hc0_od), .busy(hc0_busy));
  hc_stream u_hc1 (.clk, .rst_n(rst_i), .start(hc1_start), .abort(1'b0), .tag(hc1_tag), .seg_src(hc1_src),
    .seg_base(hc1_base), .seg_len(hc1_len), .nout(hc1_nout), .reg_idx(hc1_ridx), .reg_byte(hc1_rbyte), .reg_byte2(hc1_rbyte2),
    .kreq(hc1_kreq), .kgnt(1'b0), .kq, .creq(hc1_creq), .cgnt(cg[1]), .cq,
    .out_valid(hc1_ov), .out_idx(hc1_oi), .out_data(hc1_od), .busy(hc1_busy));

  // ======================================================================== XOF engine
  logic        x_start, x_stop, x_busy, x_ov, x_or;
  logic [7:0]  x_tag;  logic [1:0] x_seedsel;  logic [10:0] x_nblk;  logic [255:0] x_od;
  logic [511:0] x_seed;
  logic [1:0]  x_dst;          // 0 sampler, 1 h writer, 2 capture
  always_comb begin
    case (x_seedsel)
      2'd0: x_seed = r_seedkem;
      2'd1: x_seed = r_theta;
      2'd2: x_seed = r_seeddk;
      default: x_seed = r_seedek;
    endcase
  end
  logic smp_ir;
  assign x_or = (x_dst == 2'd0) ? smp_ir : 1'b1;
  xof_engine u_xof (.clk, .rst_n(rst_i), .start(x_start), .stop(x_stop), .tag(x_tag), .seed(x_seed), .lvl,
    .nblk(x_nblk), .out_valid(x_ov), .out_data(x_od), .out_ready(x_or), .busy(x_busy));

  // capture of short XOF / hash outputs (up to 4 blocks = 128 bytes)
  logic [1023:0] cap;
  logic [1:0]    cap_i;

  // ======================================================================== sampler
  logic        s_flush, s_start, s_busy, s_wlist, s_skip, s_wback;
  logic [1:0]  s_mode;  logic [8:0] s_w;  logic [9:0] s_base;
  sampler u_smp (.clk, .rst_n(rst_i), .flush(s_flush), .start(s_start), .mode(s_mode), .w(s_w), .base(s_base),
    .wlist(s_wlist), .skip(s_skip), .wback(s_wback), .lvl, .in_valid(x_ov && x_dst == 2'd0), .in_data(x_od),
    .in_ready(smp_ir), .l_ce(sl_ce), .l_we(sl_we), .l_a(sl_a), .l_d(sl_d), .l_q,
    .bm(q_smp), .bm_q(mq[m_smp]), .busy(s_busy));

  // ======================================================================== dense writers / copy
  logic        dwa_start, dwa_done, dwb_start, dwb_done;
  dwriter u_dwa (.clk, .rst_n(rst_i), .start(dwa_start), .lvl, .in_valid(x_ov && x_dst == 2'd1),
    .in_data(x_od), .mreq(q_dwa), .done(dwa_done));
  logic        cp_start, cp_busy, cp_v, cp_src;   // cp_src 0: key buffer (s), 1: CT buffer (u)
  logic [255:0] cp_d;
  ob_req_t     cp_req;
  logic        cp_gnt;
  logic [63:0] cp_q;
  assign cp_gnt = cp_src ? cg[4] : kg[3];
  assign cp_q   = cp_src ? cq : kq;
  bcopy u_cpy (.clk, .rst_n(rst_i), .start(cp_start), .base(cp_src ? 13'd0 : {9'd0, S[6:3]}), .nw,
    .breq(cp_req), .bgnt(cp_gnt), .bq(cp_q), .out_valid(cp_v), .out_data(cp_d), .busy(cp_busy));
  dwriter u_dwb (.clk, .rst_n(rst_i), .start(dwb_start), .lvl, .in_valid(cp_v), .in_data(cp_d),
    .mreq(q_dwb), .done(dwb_done));

  // ======================================================================== clear engine
  // clears rows 0 .. ceil((nw+4)/8)-1 of all eight banks (words 0 .. nw+3)
  logic       clr_start, clr_busy;
  logic [6:0] clr_a;
  logic [6:0] clr_n;
  assign clr_n = 7'((nw + 10'd11) >> 3);
  always_comb begin
    q_clr = '0;
    if (clr_busy) begin
      q_clr.ce = 8'hFF; q_clr.we = 8'hFF;
      for (int b = 0; b < 8; b = b + 1) q_clr.a[b] = clr_a;
    end
  end
  always_ff @(posedge clk or negedge rst_i)
    if (!rst_i) begin clr_busy <= 1'b0; clr_a <= '0; end
    else if (clr_start) begin clr_busy <= 1'b1; clr_a <= '0; end
    else if (clr_busy) begin
      clr_a <= clr_a + 7'd1;
      if (clr_a == clr_n - 7'd1) clr_busy <= 1'b0;
    end

  // ======================================================================== multiplier / output stage
  logic        mul_start, mul_split;  logic [9:0] mul_nw;
  logic        r_valid, r_vb, r_pop, mul_stall, mul_ovf;  logic [9:0] r_ka, r_kb;  logic [255:0] r_a, r_b;
  mul_engine u_mul (.clk, .rst_n(rst_i), .start(mul_start), .abort(1'b0), .split(mul_split), .n, .w,
    .nwords(mul_nw), .l_ce(ml_ce), .l_a(ml_a), .l_q,
    .ma(q_mula), .ma_q(mq[m_mula]), .mb(q_mulb), .mb_q(mq[m_mulb]),
    .r_valid, .r_ka, .r_a, .r_vb, .r_kb, .r_b, .r_pop, .stall(mul_stall), .ovf(mul_ovf), .busy(mul_busy));
  logic        os_start, os_done, os_mis;  logic [3:0] os_act;  logic [2:0] os_mode;
  logic [7:0]  cw_idx, cw_byte, cw_idx2, cw_byte2;
  logic        ch_valid;  logic [127:0] ch_data;
  ob_req_t     os_creq, os_kreq;
  logic        rm_rdy;
  ostage u_os (.clk, .rst_n(rst_i), .lvl, .mode(os_mode), .start(os_start), .r_valid, .r_ka, .r_a, .r_vb, .r_kb, .r_b, .r_pop,
    .bmr(q_osr), .bmr_q(mq[m_osr]), .bme(q_ose), .bme_q(mq[m_ose]),
    .creq(os_creq), .cgnt(cg[0]), .cq, .kreq(os_kreq), .kgnt(kg[0]),
    .cw_idx, .cw_byte, .cw_idx2, .cw_byte2, .ch_valid, .ch_data, .rm_rdy, .done(os_done), .mismatch(os_mis), .act(os_act));

  // ======================================================================== RM / RS
  logic        rm_init, sym_v, sym_e, rm_ovf, rm_act;  logic [7:0] sym_b;
  rm_dec u_rmd (.clk, .rst_n(rst_i), .lvl, .init(rm_init), .chunk_valid(ch_valid), .chunk(ch_data),
    .sym_valid(sym_v), .sym_byte(sym_b), .sym_era(sym_e), .overflow(rm_ovf), .rdy(rm_rdy), .act(rm_act));
  logic        rs_init, rs_finish, rs_enc, rs_enc_use, rs_busy;  logic [511:0] rs_msg;
  rs_unit u_rsu (.clk, .rst_n(rst_i), .lvl, .dec_init(rs_init), .sym_valid(sym_v), .sym_byte(sym_b), .sym_era(sym_e),
    .dec_finish(rs_finish), .enc_start(rs_enc), .enc_msg(r_m), .enc_use_msg(rs_enc_use), .msg_out(rs_msg),
    .cw_idx, .cw_byte, .cw_idx2, .cw_byte2, .busy(rs_busy));
  logic [7:0]  nsym;   // symbols delivered by the RM decoder

  // ======================================================================== byte mover
  // op: dir 0 = buffer -> register, 1 = register -> buffer; one byte per cycle
  logic        bmv_start, bmv_busy, bmv_dir, bmv_buf;   // buf 0 key, 1 CT
  logic [15:0] bmv_addr;  logic [6:0] bmv_len;  logic [3:0] bmv_reg;
  logic [6:0]  bmv_i;     logic bmv_pend;  logic [6:0] bmv_pi;  logic [2:0] bmv_plane;
  ob_req_t     bmv_req;
  logic        bmv_gnt;
  logic [15:0] bmv_ba;
  assign bmv_ba  = bmv_addr + {9'd0, bmv_i};
  assign bmv_gnt = bmv_buf ? cg[3] : kg[2];
  localparam logic [3:0] RG_SEEDKEM = 4'd0, RG_M = 4'd1, RG_SALT = 4'd2, RG_SIGMA = 4'd3, RG_SEEDDK = 4'd4,
                         RG_SEEDEK = 4'd5;
  logic [7:0] bmv_rb;
  always_comb begin
    case (bmv_reg)
      RG_SEEDKEM: bmv_rb = r_seedkem[8*bmv_i[5:0] +: 8];
      RG_M:       bmv_rb = r_m[8*bmv_i[5:0] +: 8];
      RG_SALT:    bmv_rb = r_salt[8*bmv_i[5:0] +: 8];
      RG_SIGMA:   bmv_rb = r_sigma[8*bmv_i[5:0] +: 8];
      RG_SEEDDK:  bmv_rb = r_seeddk[8*bmv_i[5:0] +: 8];
      default:    bmv_rb = r_seedek[8*bmv_i[5:0] +: 8];
    endcase
    bmv_req = '0;
    if (bmv_busy && bmv_i != bmv_len) begin
      bmv_req.req = 1'b1; bmv_req.we = bmv_dir; bmv_req.a = bmv_ba[15:3];
      bmv_req.d = {8{bmv_rb}}; bmv_req.be = 8'd1 << bmv_ba[2:0];
    end
  end

  // ======================================================================== buffer requester wiring
  always_comb begin
    kr[0] = os_kreq;
    kr[1] = hc0_kreq;
    kr[2] = bmv_buf ? '0 : bmv_req;
    kr[3] = cp_src ? '0 : cp_req;
    cr[0] = os_creq;
    cr[1] = hc1_creq;
    cr[2] = hc0_creq;
    cr[3] = bmv_buf ? bmv_req : '0;
    cr[4] = cp_src ? cp_req : '0;
  end

  // ======================================================================== phase trace (for verification)
  // ph_active bit: 0 H(ek) 1 G 2 J 3 seed-XOF/I 4 h-expansion 5 sample-y 6 sample-x 7 sample-r2
  // 8 sample-e 9 sample-r1 10 product-1 11 product-2 12 RM-decode 13 RS-decode 14 RS-encode
  // 15 compress/compare/write-back 16 u/s copy
  // ph_stream (combinational, working cycles of the units streaming behind a product):
  // 0 u/s write-back (KG/ENC) or u compare (DEC re-encryption), 1 KR compression + payload
  // packing (ENC) or payload compare (DEC), 2 RM encoding, 3 KR decompression (DEC product 1),
  // 4 RM decoder processing a symbol, 5 multiplier issue stage waiting for its output buffer
  logic [16:0] ph_active;
  logic [5:0]  ph_stream;
  assign ph_stream = {mul_stall, rm_act, os_act};

  // ======================================================================== operation sequencer
  typedef enum logic [5:0] {
    M_IDLE, M_DONE,
    K_RND, K_XOF1, K_XOF1W, K_XOF2, K_XOF2W, K_Y, K_YW, K_CLR, K_X, K_XW, K_H, K_HW, K_MUL, K_MULW,
    E_PRE, E_SAMP, E_R2G, E_R2B, E_CLR, E_EG, E_EB, E_R1G, E_R1B, E_MUL, E_MULW,
    D_PRE, D_Y, D_YW, D_H, D_P1, D_P1W, D_RSF, D_RSW, D_SAMP, D_R2G, D_R2B, D_CLR, D_EG, D_EB, D_R1G, D_R1B,
    D_P2, D_P2W, D_WG, M_FIN
  } mst_t;
  mst_t ms;

  // concurrent-thread flags
  logic f_rnd, f_hek, f_g, f_j, f_seedek, f_seeddk, f_sigma, f_salt, f_copy, f_h, f_enc, f_m, f_clr1, f_clr2;
  logic f_bmv;           // byte-mover program finished
  logic [3:0] rnd_cnt;
  logic [2:0] salt_idx;
  assign salt_idx = rnd_cnt[2:0] - S[5:3];
  logic g_started, j_started, enc_started, h_started, cp_started, cp2_started, clr_d1, clr_d2;
  logic [2:0] bmv_step;

  assign rnd_ready = busy && (op != OP_DEC) && !f_rnd && (ms != M_IDLE);

  // hasher job programming (combinational from op / phase)
  always_comb begin
    hc0_tag = 8'hB2; hc0_nout = nb32;
    for (int i = 0; i < 3; i = i + 1) begin hc0_src[i] = 2'd0; hc0_base[i] = '0; hc0_len[i] = '0; end
    if (!f_hek) begin
      hc0_tag = 8'hB2; hc0_src[0] = 2'd2; hc0_base[0] = 16'd0; hc0_len[0] = {1'b0, PK}; hc0_nout = nb32;
    end else begin
      hc0_tag = 8'hB4; hc0_nout = nout2;
      hc0_src[0] = 2'd1; hc0_base[0] = 16'd0;   hc0_len[0] = {9'd0, S};
      hc0_src[1] = 2'd1; hc0_base[1] = 16'd64;  hc0_len[1] = {9'd0, S};
      hc0_src[2] = 2'd1; hc0_base[2] = 16'd128; hc0_len[2] = {9'd0, S};
    end
    hc1_tag = 8'hB5; hc1_nout = nb32;
    hc1_src[0] = 2'd1; hc1_base[0] = 16'd0;   hc1_len[0] = {9'd0, S};
    hc1_src[1] = 2'd1; hc1_base[1] = 16'd192; hc1_len[1] = {9'd0, S};
    hc1_src[2] = 2'd3; hc1_base[2] = 16'd0;   hc1_len[2] = CT;
  end

  // split helpers: first S bytes / next S bytes of the capture register
  function automatic [511:0] lo_s(input [1023:0] c, input [6:0] s);
    for (int b = 0; b < 64; b = b + 1) lo_s[8*b +: 8] = (b < s) ? c[8*b +: 8] : 8'h00;
  endfunction
  function automatic [511:0] hi_s(input [1023:0] c, input [6:0] s);
    for (int b = 0; b < 64; b = b + 1) begin
      case (s)
        7'd16: hi_s[8*b +: 8] = (b < 16) ? c[8*(b+16) +: 8] : 8'h00;
        7'd32: hi_s[8*b +: 8] = (b < 32) ? c[8*(b+32) +: 8] : 8'h00;
        7'd48: hi_s[8*b +: 8] = (b < 48) ? c[8*(b+48) +: 8] : 8'h00;
        default: hi_s[8*b +: 8] = c[8*(b+64) +: 8];
      endcase
    end
  endfunction

  assign acc_take = rst_i && (ms == M_IDLE) && acc_pend;
  assign fin      = rst_i && (ms == M_FIN) &&
                    ((op == OP_KG && f_bmv) || (op == OP_ENC && f_bmv) || (op == OP_DEC && f_j));
  assign fin_fail = (op == OP_DEC) && os_mis;

  always_ff @(posedge clk or negedge rst_i) begin
    if (!rst_i) begin
      ms <= M_IDLE;
      r_seedkem <= '0; r_m <= '0; r_salt <= '0; r_sigma <= '0; r_seeddk <= '0; r_seedek <= '0; r_hek <= '0;
      r_theta <= '0; r_k <= '0; r_kbar <= '0; cap <= '0; cap_i <= '0;
      for (int i = 0; i < 4; i = i + 1) own[i] <= CL_NONE;
      {f_rnd, f_hek, f_g, f_j, f_seedek, f_seeddk, f_sigma, f_salt, f_copy, f_h, f_enc, f_m, f_clr1, f_clr2, f_bmv} <= '0;
      {g_started, j_started, enc_started, h_started, cp_started, cp2_started, clr_d1, clr_d2} <= '0;
      rnd_cnt <= '0; bmv_step <= '0; nsym <= '0; x_dst <= '0;
      hc0_start <= 1'b0; hc1_start <= 1'b0; x_start <= 1'b0; x_stop <= 1'b0; x_tag <= 8'hB1; x_seedsel <= '0; x_nblk <= '0;
      s_flush <= 1'b0; s_start <= 1'b0; s_mode <= '0; s_w <= '0; s_base <= '0; s_wlist <= 1'b0; s_skip <= 1'b0; s_wback <= 1'b0;
      dwa_start <= 1'b0; dwb_start <= 1'b0; cp_start <= 1'b0; cp_src <= 1'b0; clr_start <= 1'b0;
      mul_start <= 1'b0; mul_split <= 1'b0; mul_nw <= '0; os_start <= 1'b0; os_mode <= '0;
      rm_init <= 1'b0; rs_init <= 1'b0; rs_finish <= 1'b0; rs_enc <= 1'b0; rs_enc_use <= 1'b1;
      bmv_start <= 1'b0; bmv_busy <= 1'b0; bmv_dir <= 1'b0; bmv_buf <= 1'b0; bmv_addr <= '0; bmv_len <= '0;
      bmv_reg <= '0; bmv_i <= '0; bmv_pend <= 1'b0; bmv_pi <= '0; bmv_plane <= '0;
      ph_active <= '0;
    end else begin
      // default pulses
      hc0_start <= 1'b0; hc1_start <= 1'b0; x_start <= 1'b0; x_stop <= 1'b0; s_flush <= 1'b0;
      s_start <= 1'b0; dwa_start <= 1'b0; dwb_start <= 1'b0; cp_start <= 1'b0; clr_start <= 1'b0;
      mul_start <= 1'b0; os_start <= 1'b0; rm_init <= 1'b0; rs_init <= 1'b0; rs_finish <= 1'b0; rs_enc <= 1'b0;
      bmv_start <= 1'b0;

      // ---------------------------------------------------------------- random bytes
      if (rnd_ready && rnd_valid) begin
        if (op == OP_KG) begin
          r_seedkem[64*rnd_cnt[2:0] +: 64] <= rnd_data;
          if ({1'b0, rnd_cnt, 3'b000} + 8'd8 == {1'b0, S}) begin f_rnd <= 1'b1; end
          rnd_cnt <= rnd_cnt + 4'd1;
        end else begin
          if (rnd_cnt < S[6:3]) r_m[64*rnd_cnt[2:0] +: 64] <= rnd_data;
          else                  r_salt[64*salt_idx +: 64] <= rnd_data;
          if ({1'b0, rnd_cnt, 3'b000} + 8'd8 == {S, 1'b0}) f_rnd <= 1'b1;
          rnd_cnt <= rnd_cnt + 4'd1;
        end
      end

      // ---------------------------------------------------------------- byte mover engine
      if (bmv_start) begin bmv_busy <= 1'b1; bmv_i <= '0; bmv_pend <= 1'b0; end
      else if (bmv_busy) begin
        bmv_pend <= 1'b0;
        if (bmv_req.req && bmv_gnt) begin
          bmv_i <= bmv_i + 7'd1;
          if (!bmv_dir) begin bmv_pend <= 1'b1; bmv_pi <= bmv_i; bmv_plane <= bmv_ba[2:0]; end
        end
        if (bmv_pend) begin
          case (bmv_reg)
            RG_SALT:   r_salt[8*bmv_pi[5:0] +: 8]   <= (bmv_buf ? cq : kq) >> (8*bmv_plane);
            RG_SIGMA:  r_sigma[8*bmv_pi[5:0] +: 8]  <= (bmv_buf ? cq : kq) >> (8*bmv_plane);
            RG_SEEDDK: r_seeddk[8*bmv_pi[5:0] +: 8] <= (bmv_buf ? cq : kq) >> (8*bmv_plane);
            RG_SEEDEK: r_seedek[8*bmv_pi[5:0] +: 8] <= (bmv_buf ? cq : kq) >> (8*bmv_plane);
            default: ;
          endcase
        end
        if (bmv_i == bmv_len && !bmv_pend && !(bmv_req.req && bmv_gnt)) bmv_busy <= 1'b0;
      end

      // ---------------------------------------------------------------- hasher outputs
      if (hc0_ov) begin
        if (!f_hek) r_hek[256*hc0_oi[0] +: 256] <= hc0_od;
        else        cap[256*hc0_oi +: 256] <= hc0_od;
      end
      if (hc1_ov) r_kbar[256*hc1_oi[0] +: 256] <= hc1_od;
      if (x_ov && x_dst == 2'd2) begin cap[256*cap_i +: 256] <= x_od; cap_i <= cap_i + 2'd1; end

      // ---------------------------------------------------------------- command accept
      if (ms == M_IDLE) begin
        if (acc_take) begin
          {f_rnd, f_hek, f_g, f_j, f_seedek, f_seeddk, f_sigma, f_salt, f_copy, f_h, f_enc, f_m, f_clr1, f_clr2, f_bmv} <= '0;
          {g_started, j_started, enc_started, h_started, cp_started, cp2_started, clr_d1, clr_d2} <= '0;
          rnd_cnt <= '0; bmv_step <= '0; nsym <= '0; cap_i <= '0;
          for (int i = 0; i < 4; i = i + 1) own[i] <= CL_NONE;
          case (op)
            OP_KG:  begin ms <= K_RND; own[2] <= CL_CLR; clr_start <= 1'b1; ph_active <= 17'd0; end
            OP_ENC: begin ms <= E_PRE; ph_active <= 17'd1; hc0_start <= 1'b1;
                          own[1] <= CL_CLR; own[2] <= CL_CLR; clr_start <= 1'b1; end
            default: begin ms <= D_PRE; ph_active <= 17'd1; hc0_start <= 1'b1; own[2] <= CL_CLR; clr_start <= 1'b1; end
          endcase
        end
      end else begin
        // ================================================================ background threads
        // clear completion (first clear issued at accept)
        if (!f_clr1 && !clr_busy && !clr_start && clr_d1) begin
          f_clr1 <= 1'b1;
          for (int i = 0; i < 4; i = i + 1) if (own[i] == CL_CLR) own[i] <= CL_NONE;
        end
        clr_d1 <= 1'b1;

        // H(ek) done -> r_hek valid
        if (!f_hek && hc0_ov && hc0_oi == nb32[1:0] - 2'd1 && op != OP_KG) begin f_hek <= 1'b1; ph_active[0] <= 1'b0; end
        // G done
        if (f_hek && g_started && !f_g && !hc0_busy && !hc0_start) begin
          f_g <= 1'b1; ph_active[1] <= 1'b0;
          r_k <= lo_s(cap, S); r_theta <= hi_s(cap, S);
        end
        // J done
        if (j_started && !f_j && !hc1_busy && !hc1_start) begin f_j <= 1'b1; ph_active[2] <= 1'b0; end

        // byte mover program per operation
        if (!bmv_busy && !bmv_start && !f_bmv) begin
          case (op)
            OP_KG: if (f_seedek) begin   // after I: write seed_ek, seed_dk, sigma, seed_kem into sk
              bmv_start <= 1'b1; bmv_dir <= 1'b1; bmv_buf <= 1'b0; bmv_len <= S;
              case (bmv_step)
                3'd0: begin bmv_reg <= RG_SEEDEK; bmv_addr <= 16'd0; end
                3'd1: begin bmv_reg <= RG_SEEDDK; bmv_addr <= {1'b0, PK}; end
                3'd2: begin bmv_reg <= RG_SIGMA;  bmv_addr <= {1'b0, PK} + {9'd0, S}; end
                3'd3: begin bmv_reg <= RG_SEEDKEM; bmv_addr <= {1'b0, PK} + {8'd0, S, 1'b0}; end
                default: begin bmv_start <= 1'b0; f_bmv <= 1'b1; end
              endcase
              bmv_step <= bmv_step + 3'd1;
            end
            OP_ENC: begin
              case (bmv_step)
                3'd0: begin bmv_start <= 1'b1; bmv_dir <= 1'b0; bmv_buf <= 1'b0; bmv_len <= S; bmv_reg <= RG_SEEDEK; bmv_addr <= 16'd0; bmv_step <= 3'd1; end
                3'd1: begin f_seedek <= 1'b1; bmv_step <= 3'd2; end
                3'd2: if (f_rnd) begin bmv_start <= 1'b1; bmv_dir <= 1'b1; bmv_buf <= 1'b1; bmv_len <= S; bmv_reg <= RG_SALT;
                                       bmv_addr <= CT - {9'd0, S}; bmv_step <= 3'd3; end
                default: f_bmv <= 1'b1;
              endcase
            end
            default: begin // DEC: seed_dk, sigma, seed_ek, salt
              bmv_dir <= 1'b0; bmv_len <= S;
              case (bmv_step)
                3'd0: begin bmv_start <= 1'b1; bmv_buf <= 1'b0; bmv_reg <= RG_SEEDDK; bmv_addr <= {1'b0, PK}; end
                3'd1: begin f_seeddk <= 1'b1; bmv_start <= 1'b1; bmv_buf <= 1'b0; bmv_reg <= RG_SIGMA; bmv_addr <= {1'b0, PK} + {9'd0, S}; end
                3'd2: begin f_sigma <= 1'b1; bmv_start <= 1'b1; bmv_buf <= 1'b0; bmv_reg <= RG_SEEDEK; bmv_addr <= 16'd0; end
                3'd3: begin f_seedek <= 1'b1; bmv_start <= 1'b1; bmv_buf <= 1'b1; bmv_reg <= RG_SALT; bmv_addr <= CT - {9'd0, S}; end
                default: begin f_salt <= 1'b1; f_bmv <= 1'b1; end
              endcase
              bmv_step <= bmv_step + 3'd1;
            end
          endcase
        end

        // copy engine: ENC s at start (after seed_ek read, to give H the port first);
        // DEC u at start, s after product 1
        if (op == OP_ENC && !cp_started) begin
          cp_started <= 1'b1; cp_src <= 1'b0; cp_start <= 1'b1; dwb_start <= 1'b1; own[0] <= CL_DWB; ph_active[16] <= 1'b1;
        end
        if (op == OP_DEC && !cp_started) begin
          cp_started <= 1'b1; cp_src <= 1'b1; cp_start <= 1'b1; dwb_start <= 1'b1; own[0] <= CL_DWB; own[1] <= CL_DWB; ph_active[16] <= 1'b1;
        end
        if (dwb_done) begin f_copy <= 1'b1; ph_active[16] <= 1'b0; if (own[0] == CL_DWB) own[0] <= CL_NONE; if (own[1] == CL_DWB) own[1] <= CL_NONE; end

        // ENC: G after H(ek) and the random bytes; RS encode after m
        if (op == OP_ENC) begin
          if (f_hek && f_rnd && !g_started && !hc0_busy) begin g_started <= 1'b1; hc0_start <= 1'b1; ph_active[1] <= 1'b1; end
          if (f_rnd && !enc_started) begin enc_started <= 1'b1; rs_enc <= 1'b1; rs_enc_use <= 1'b1; ph_active[14] <= 1'b1; end
        end
        if (enc_started && !f_enc && !rs_busy && !rs_enc) begin f_enc <= 1'b1; ph_active[14] <= 1'b0; end
        // DEC: J after H(ek) and sigma
        if (op == OP_DEC && f_hek && f_sigma && !j_started) begin j_started <= 1'b1; hc1_start <= 1'b1; ph_active[2] <= 1'b1; end
        // DEC: G and RS encode after m'
        if (op == OP_DEC && f_m && f_hek && f_salt && !g_started && !hc0_busy) begin g_started <= 1'b1; hc0_start <= 1'b1; ph_active[1] <= 1'b1; end
        if (op == OP_DEC && f_m && !enc_started) begin enc_started <= 1'b1; rs_enc <= 1'b1; rs_enc_use <= 1'b0; ph_active[14] <= 1'b1; end

        // h writer completion
        if (dwa_done) begin f_h <= 1'b1; ph_active[4] <= 1'b0; for (int i = 0; i < 4; i = i + 1) if (own[i] == CL_DWA) own[i] <= CL_NONE; end

        // RM symbols count
        if (sym_v) nsym <= nsym + 8'd1;

        // ================================================================ main sequencer
        case (ms)
          // ---------------------------- KeyGen
          K_RND: if (f_rnd) begin
            x_start <= 1'b1; x_tag <= 8'hB1; x_seedsel <= 2'd0; x_nblk <= {8'd0, nout2}; x_dst <= 2'd2; cap_i <= '0;
            ph_active[3] <= 1'b1; ms <= K_XOF1;
          end
          K_XOF1: ms <= K_XOF1W;
          K_XOF1W: if (!x_busy) begin
            r_theta <= lo_s(cap, S); r_sigma <= hi_s(cap, S);
            ms <= K_XOF2;
          end
          K_XOF2: begin
            x_start <= 1'b1; x_tag <= 8'hB3; x_seedsel <= 2'd1; x_nblk <= {8'd0, nout2}; x_dst <= 2'd2; cap_i <= '0;
            ms <= K_XOF2W;
          end
          K_XOF2W: if (!x_busy && !x_start) begin
            r_seeddk <= lo_s(cap, S); r_seedek <= hi_s(cap, S); f_seedek <= 1'b1;
            ph_active[3] <= 1'b0; ms <= K_Y;
          end
          K_Y: if (f_clr1) begin
            own[2] <= CL_SMP;
            x_start <= 1'b1; x_tag <= 8'hB1; x_seedsel <= 2'd2; x_nblk <= '0; x_dst <= 2'd0; s_flush <= 1'b1;
            s_start <= 1'b1; s_mode <= 2'd0; s_w <= w; s_base <= 10'd0; s_wlist <= 1'b1; s_skip <= 1'b1;
            ph_active[5] <= 1'b1; ms <= K_YW;
          end
          K_YW: if (!s_busy && !s_start) begin
            own[2] <= CL_CLR; clr_start <= 1'b1; ph_active[5] <= 1'b0; ms <= K_CLR;
          end
          K_CLR: if (!clr_busy && !clr_start) begin
            own[2] <= CL_SMP; s_start <= 1'b1; s_mode <= 2'd0; s_wlist <= 1'b0; s_skip <= 1'b0;
            ph_active[6] <= 1'b1; ms <= K_XW;
          end
          K_XW: if (!s_busy && !s_start) begin
            x_stop <= 1'b1; ph_active[6] <= 1'b0; ms <= K_H;
          end
          K_H: begin
            own[0] <= CL_DWA; own[1] <= CL_DWA; own[2] <= CL_NONE;
            x_start <= 1'b1; x_tag <= 8'hB1; x_seedsel <= 2'd3; x_nblk <= {1'b0, nw}; x_dst <= 2'd1; dwa_start <= 1'b1;
            ph_active[4] <= 1'b1; ms <= K_HW;
          end
          K_HW: if (f_h) begin
            own[0] <= CL_MULA; own[1] <= CL_MULB; own[2] <= CL_OSR;
            mul_start <= 1'b1; mul_split <= 1'b1; mul_nw <= nw; os_start <= 1'b1; os_mode <= 3'd1;
            ph_active[10] <= 1'b1; ph_active[15] <= 1'b1; ms <= K_MULW;
          end
          K_MULW: if (os_done) begin ph_active[10] <= 1'b0; ph_active[15] <= 1'b0; ms <= M_FIN; end

          // ---------------------------- Enc
          E_PRE: begin
            if (f_seedek && !h_started) begin
              h_started <= 1'b1; own[3] <= CL_DWA;
              x_start <= 1'b1; x_tag <= 8'hB1; x_seedsel <= 2'd3; x_nblk <= {1'b0, nw}; x_dst <= 2'd1; dwa_start <= 1'b1;
              ph_active[4] <= 1'b1;
            end
            if (f_g && f_h && f_clr1) ms <= E_SAMP;
          end
          E_SAMP, D_SAMP: begin
            own[1] <= CL_SMP;
            x_start <= 1'b1; x_tag <= 8'hB1; x_seedsel <= 2'd1; x_nblk <= '0; x_dst <= 2'd0; s_flush <= 1'b1;
            s_start <= 1'b1; s_mode <= 2'd1; s_w <= w; s_base <= 10'd0; ph_active[7] <= 1'b1;
            ms <= (ms == E_SAMP) ? E_R2G : D_R2G;
          end
          E_R2G, D_R2G: if (!s_busy && !s_start) begin
            s_start <= 1'b1; s_mode <= 2'd2; s_w <= w; s_base <= 10'd0; s_wback <= 1'b1;
            ms <= (ms == E_R2G) ? E_R2B : D_R2B;
          end
          E_R2B, D_R2B: if (!s_busy && !s_start) begin
            own[1] <= CL_CLR; clr_start <= 1'b1; ph_active[7] <= 1'b0; ms <= (ms == E_R2B) ? E_CLR : D_CLR;
          end
          E_CLR, D_CLR: if (!clr_busy && !clr_start) begin
            own[1] <= CL_SMP; s_start <= 1'b1; s_mode <= 2'd1; s_w <= we; s_base <= 10'd512; s_wback <= 1'b0;
            ph_active[8] <= 1'b1; ms <= (ms == E_CLR) ? E_EG : D_EG;
          end
          E_EG, D_EG: if (!s_busy && !s_start) begin
            s_start <= 1'b1; s_mode <= 2'd2; s_w <= we; s_base <= 10'd512; ms <= (ms == E_EG) ? E_EB : D_EB;
          end
          E_EB, D_EB: if (!s_busy && !s_start) begin
            own[1] <= CL_NONE; own[2] <= CL_SMP;
            s_start <= 1'b1; s_mode <= 2'd1; s_w <= w; s_base <= 10'd512;
            ph_active[8] <= 1'b0; ph_active[9] <= 1'b1; ms <= (ms == E_EB) ? E_R1G : D_R1G;
          end
          E_R1G, D_R1G: if (!s_busy && !s_start) begin
            s_start <= 1'b1; s_mode <= 2'd2; s_w <= w; s_base <= 10'd512; ms <= (ms == E_R1G) ? E_R1B : D_R1B;
          end
          E_R1B, D_R1B: if (!s_busy && !s_start) begin
            x_stop <= 1'b1; own[2] <= CL_NONE; ph_active[9] <= 1'b0; ms <= (ms == E_R1B) ? E_MUL : D_P2;
          end
          E_MUL, D_P2: if (f_copy && f_enc) begin
            own[3] <= CL_MULA; own[0] <= CL_MULB; own[2] <= CL_OSR; own[1] <= CL_OSE;
            mul_start <= 1'b1; mul_split <= 1'b0; mul_nw <= nw; os_start <= 1'b1;
            os_mode <= (op == OP_ENC) ? 3'd2 : 3'd3;
            ph_active[(op == OP_ENC) ? 10 : 11] <= 1'b1; ph_active[15] <= 1'b1;
            ms <= (ms == E_MUL) ? E_MULW : D_P2W;
          end
          E_MULW: if (os_done) begin ph_active[10] <= 1'b0; ph_active[15] <= 1'b0; ms <= M_FIN; end

          // ---------------------------- Dec
          D_PRE: if (f_seeddk && f_clr1) begin
            own[2] <= CL_SMP;
            x_start <= 1'b1; x_tag <= 8'hB1; x_seedsel <= 2'd2; x_nblk <= '0; x_dst <= 2'd0; s_flush <= 1'b1;
            s_start <= 1'b1; s_mode <= 2'd0; s_w <= w; s_base <= 10'd0; s_wlist <= 1'b1; s_skip <= 1'b0;
            ph_active[5] <= 1'b1; ms <= D_YW;
          end
          D_YW: if (!s_busy && !s_start) begin
            x_stop <= 1'b1; own[2] <= CL_CLR; clr_start <= 1'b1; ph_active[5] <= 1'b0; ms <= D_H;
          end
          D_H: if (!clr_busy && !clr_start && f_seedek && f_copy) begin
            own[2] <= CL_NONE; own[3] <= CL_DWA;
            x_start <= 1'b1; x_tag <= 8'hB1; x_seedsel <= 2'd3; x_nblk <= {1'b0, nw}; x_dst <= 2'd1; dwa_start <= 1'b1;
            ph_active[4] <= 1'b1;
            // product 1 (u*y) in SPLIT mode, streaming into the RM decoder
            own[0] <= CL_MULA; own[1] <= CL_MULB;
            mul_start <= 1'b1; mul_split <= 1'b1; mul_nw <= nwd; os_start <= 1'b1; os_mode <= 3'd4;
            rm_init <= 1'b1; rs_init <= 1'b1;
            ph_active[10] <= 1'b1; ph_active[12] <= 1'b1; ms <= D_P1W;
          end
          D_P1W: if (os_done) begin
            ph_active[10] <= 1'b0;
            // memories M0/M1 free: clear M1, copy s into M0
            own[1] <= CL_CLR; clr_start <= 1'b1;
            own[0] <= CL_DWB; cp_src <= 1'b0; cp_start <= 1'b1; dwb_start <= 1'b1; f_copy <= 1'b0; ph_active[16] <= 1'b1;
            ms <= D_RSF;
          end
          D_RSF: if (nsym == N1 && !rs_busy) begin
            ph_active[12] <= 1'b0; ph_active[13] <= 1'b1; rs_finish <= 1'b1; ms <= D_RSW;
          end
          D_RSW: if (!rs_busy && !rs_finish) begin
            r_m <= rs_msg; f_m <= 1'b1; ph_active[13] <= 1'b0; ms <= D_WG;
          end
          M_FIN: begin
            if (fin) begin
              if (op == OP_DEC && os_mis) r_k <= r_kbar;
              ms <= M_IDLE; ph_active <= '0;
              for (int i = 0; i < 4; i = i + 1) own[i] <= CL_NONE;
            end
          end
          default: ;
        endcase
        // DEC: after m' -> wait for G, h expansion and the M1 clear, then sampling
        if (ms == D_WG) begin
          if (!clr_busy && !clr_start && f_g && f_h && own[1] == CL_CLR) begin
            own[1] <= CL_NONE; ms <= D_SAMP;
          end
        end
        if (ms == D_P2W && os_done) begin ph_active[11] <= 1'b0; ph_active[15] <= 1'b0; ms <= M_FIN; end
      end
    end
  end
endmodule
