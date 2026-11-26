// polarlac_kem_perf — PolarLAC KEM performance-version top (KeyGen / Enc / Dec, 5 parameter sets).
// Structure: plc_ctrl (protocol, register file, microcoded sequencer) drives the engines
// (plc_hash + plc_gena + plc_tsamp(plc_fft) + plc_ntt + plc_pwalu + plc_codec + plc_polar),
// which share the polynomial slot memory (plc_slotmem) and the object buffers (plc_objmem).
module polarlac_kem_perf
  import plc_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        cmd_valid,
  output logic        cmd_ready,
  input  logic [1:0]  cmd_op,
  input  logic [2:0]  cmd_param,
  output logic        busy,
  output logic        done,
  output logic        result_valid,
  output logic        cmd_error,
  input  logic        in_valid,
  output logic        in_ready,
  input  logic [1:0]  in_obj,
  input  logic [9:0]  in_addr,
  input  logic [63:0] in_data,
  input  logic        rnd_valid,
  output logic        rnd_ready,
  input  logic [63:0] rnd_data,
  input  logic        out_req,
  input  logic [1:0]  out_obj,
  input  logic [9:0]  out_addr,
  output logic        out_valid,
  output logic [63:0] out_data
);
  // ---------------- reset ----------------
  // rst_n belongs to the clk domain: asserted asynchronously, released by the host
  // relative to clk, so release timing is a recovery/removal check on the port (no synchronizer;
  // a synchronizer would leave cmd_ready=1 while commands are still ignored after release).
  logic rst_n_s;
  assign rst_n_s = rst_n;

  assign in_ready = !busy;

  // ---------------- shared state ----------------
  pcode_t            pcode;
  logic [6:0][511:0] rg;
  logic [511:0]      ss_reg;
  logic              cmd_start;

  // ---------------- slot crossbar ports ----------------
  localparam int NR = 7, NW = 5;
  logic [NR-1:0][3:0]        r_slot;
  logic [NR-1:0][1:0]        r_en;
  logic [NR-1:0][1:0][5:0]   r_addr;
  logic [NR-1:0][1:0][127:0] r_data;
  logic [NW-1:0][3:0]        w_slot;
  logic [NW-1:0][1:0]        w_en;
  logic [NW-1:0][1:0][5:0]   w_addr;
  logic [NW-1:0][1:0][127:0] w_data;

  plc_slotmem #(.NR(NR), .NW(NW)) u_slots (
    .clk(clk), .rst_n(rst_n_s),
    .r_slot(r_slot), .r_en(r_en), .r_addr(r_addr), .r_data(r_data),
    .w_slot(w_slot), .w_en(w_en), .w_addr(w_addr), .w_data(w_data));

  // ---------------- object buffers ----------------
  logic [2:0]       cd_oren, cd_owen;
  logic [2:0][9:0]  cd_oraddr, cd_owaddr;
  logic [2:0][63:0] cd_ordata, cd_owdata;
  logic [2:0][7:0]  cd_owbe;
  logic             hs_oreq, hs_ognt;
  logic [1:0]       hs_obuf;
  logic [9:0]       hs_oaddr;
  logic [63:0]      hs_ordata;

  plc_objmem u_obj (
    .clk(clk), .rst_n(rst_n_s), .busy(busy),
    .in_valid(in_valid), .in_obj(in_obj), .in_addr(in_addr), .in_data(in_data),
    .out_req(out_req), .out_obj(out_obj), .out_addr(out_addr), .out_valid(out_valid), .out_data(out_data),
    .ss_reg(ss_reg),
    .cd_oren(cd_oren), .cd_oraddr(cd_oraddr), .cd_ordata(cd_ordata),
    .cd_owen(cd_owen), .cd_owaddr(cd_owaddr), .cd_owbe(cd_owbe), .cd_owdata(cd_owdata),
    .hs_oreq(hs_oreq), .hs_obuf(hs_obuf), .hs_oaddr(hs_oaddr), .hs_ognt(hs_ognt), .hs_ordata(hs_ordata));

  // ---------------- hash cluster (clients: 0 TSAMP, 1 GENA, 2 CTRL) ----------------
  logic [2:0]        hj_req, hj_gnt, xo_valid, xo_ready;
  logic [2:0][1:0]   hj_kind0, hj_kind1, hj_buf0, hj_buf1, hj_cache;
  logic [2:0][2:0]   hj_reg0, hj_reg1, hj_sfxlen;
  logic [2:0][12:0]  hj_off0, hj_off1, hj_len0, hj_len1;
  logic [2:0][55:0]  hj_sfx;
  logic [2:0][6:0]   hj_count;
  logic [255:0]      xo_data;
  logic              xo_last, hs_busy;
  logic              rg_inv_sd, rg_inv_ps;

  plc_hash u_hash (
    .clk(clk), .rst_n(rst_n_s),
    .hj_req(hj_req), .hj_gnt(hj_gnt),
    .hj_kind0(hj_kind0), .hj_reg0(hj_reg0), .hj_buf0(hj_buf0), .hj_off0(hj_off0), .hj_len0(hj_len0),
    .hj_kind1(hj_kind1), .hj_reg1(hj_reg1), .hj_buf1(hj_buf1), .hj_off1(hj_off1), .hj_len1(hj_len1),
    .hj_sfx(hj_sfx), .hj_sfxlen(hj_sfxlen), .hj_count(hj_count), .hj_cache(hj_cache),
    .cache_clr(cmd_start), .cache_inv({rg_inv_ps, rg_inv_sd}),
    .xo_valid(xo_valid), .xo_ready(xo_ready), .xo_data(xo_data), .xo_last(xo_last),
    .rg(rg),
    .hs_oreq(hs_oreq), .hs_obuf(hs_obuf), .hs_oaddr(hs_oaddr), .hs_ognt(hs_ognt), .hs_ordata(hs_ordata),
    .hs_busy(hs_busy));

  // ---------------- engine control wires ----------------
  logic ga_start, ga_tr, ga_busy, ga_done;
  logic ts_start, ts_e2, ts_busy, ts_done;
  logic [7:0] ts_nonce0, ts_nonce_next;
  logic [4:0] ts_role_ok;
  logic [4:0][3:0] ts_role_slot;
  logic nt_start, nt_inv, nt_busy, nt_done;
  logic [3:0] nt_slot;
  logic pw_start, pw_acc_zero, pw_busy, pw_done;
  logic [1:0] pw_mode;
  logic [3:0] pw_sa, pw_sb, pw_sc, pw_sd;
  logic cd_start, cd_k, cd_cmp, cd_src_sk, cd_busy, cd_done, cd_ovf, cd_ne, cd_rg_we;
  logic [3:0] cd_op, cd_slot;
  logic [511:0] cd_rg_wdata;
  logic pe_start, pd_start, pe_busy, pd_busy, pe_done, pd_done, pd_m_we;
  logic [511:0] pd_m_data;
  logic [1023:0] cw;

  plc_ctrl u_ctrl (
    .clk(clk), .rst_n(rst_n_s),
    .cmd_valid(cmd_valid), .cmd_ready(cmd_ready), .cmd_op(cmd_op), .cmd_param(cmd_param),
    .busy(busy), .done(done), .result_valid(result_valid), .cmd_error(cmd_error),
    .rnd_valid(rnd_valid), .rnd_ready(rnd_ready), .rnd_data(rnd_data),
    .pcode(pcode), .rg(rg), .ss_reg(ss_reg), .cmd_start(cmd_start),
    .hj_req(hj_req[2]), .hj_gnt(hj_gnt[2]),
    .hj_kind0(hj_kind0[2]), .hj_kind1(hj_kind1[2]), .hj_reg0(hj_reg0[2]), .hj_reg1(hj_reg1[2]),
    .hj_buf0(hj_buf0[2]), .hj_buf1(hj_buf1[2]), .hj_off0(hj_off0[2]), .hj_off1(hj_off1[2]),
    .hj_len0(hj_len0[2]), .hj_len1(hj_len1[2]), .hj_sfx(hj_sfx[2]), .hj_sfxlen(hj_sfxlen[2]),
    .hj_count(hj_count[2]), .hj_cache(hj_cache[2]),
    .xo_valid(xo_valid[2]), .xo_ready(xo_ready[2]), .xo_data(xo_data), .xo_last(xo_last),
    .rg_inv_sd(rg_inv_sd), .rg_inv_ps(rg_inv_ps),
    .ga_start(ga_start), .ga_tr(ga_tr), .ga_busy(ga_busy),
    .ts_start(ts_start), .ts_nonce0(ts_nonce0), .ts_e2(ts_e2), .ts_busy(ts_busy),
    .ts_role_ok(ts_role_ok), .ts_role_slot(ts_role_slot), .ts_nonce_next(ts_nonce_next),
    .nt_start(nt_start), .nt_inv(nt_inv), .nt_slot(nt_slot), .nt_busy(nt_busy),
    .pw_start(pw_start), .pw_mode(pw_mode), .pw_acc_zero(pw_acc_zero),
    .pw_sa(pw_sa), .pw_sb(pw_sb), .pw_sc(pw_sc), .pw_sd(pw_sd), .pw_busy(pw_busy),
    .cd_start(cd_start), .cd_op(cd_op), .cd_k(cd_k), .cd_slot(cd_slot), .cd_cmp(cd_cmp),
    .cd_src_sk(cd_src_sk), .cd_busy(cd_busy), .cd_ovf(cd_ovf), .cd_ne(cd_ne),
    .cd_rg_we(cd_rg_we), .cd_rg_wdata(cd_rg_wdata),
    .pe_start(pe_start), .pd_start(pd_start), .pol_busy(pe_busy | pd_busy),
    .pd_m_we(pd_m_we), .pd_m_data(pd_m_data));

  // ---------------- GENA (hash client 1, write port 3) ----------------
  plc_gena u_gena (
    .clk(clk), .rst_n(rst_n_s), .start(ga_start), .pcode(pcode), .transposed(ga_tr),
    .busy(ga_busy), .done(ga_done),
    .hj_req(hj_req[1]), .hj_gnt(hj_gnt[1]),
    .hj_kind0(hj_kind0[1]), .hj_reg0(hj_reg0[1]), .hj_buf0(hj_buf0[1]), .hj_off0(hj_off0[1]), .hj_len0(hj_len0[1]),
    .hj_kind1(hj_kind1[1]), .hj_reg1(hj_reg1[1]), .hj_buf1(hj_buf1[1]), .hj_off1(hj_off1[1]), .hj_len1(hj_len1[1]),
    .hj_sfx(hj_sfx[1]), .hj_sfxlen(hj_sfxlen[1]), .hj_count(hj_count[1]), .hj_cache(hj_cache[1]),
    .xo_valid(xo_valid[1]), .xo_ready(xo_ready[1]), .xo_data(xo_data), .xo_last(xo_last),
    .ga_wslot(w_slot[3]), .ga_wen(w_en[3]), .ga_waddr(w_addr[3]), .ga_wdata(w_data[3]));

  // ---------------- TSAMP (hash client 0, read port 4, write port 2) ----------------
  plc_tsamp u_tsamp (
    .clk(clk), .rst_n(rst_n_s), .start(ts_start), .pcode(pcode), .nonce0(ts_nonce0), .want_e2(ts_e2),
    .busy(ts_busy), .done(ts_done), .role_ok(ts_role_ok), .role_slot(ts_role_slot), .nonce_next(ts_nonce_next),
    .hj_req(hj_req[0]), .hj_gnt(hj_gnt[0]),
    .hj_kind0(hj_kind0[0]), .hj_reg0(hj_reg0[0]), .hj_buf0(hj_buf0[0]), .hj_off0(hj_off0[0]), .hj_len0(hj_len0[0]),
    .hj_kind1(hj_kind1[0]), .hj_reg1(hj_reg1[0]), .hj_buf1(hj_buf1[0]), .hj_off1(hj_off1[0]), .hj_len1(hj_len1[0]),
    .hj_sfx(hj_sfx[0]), .hj_sfxlen(hj_sfxlen[0]), .hj_count(hj_count[0]), .hj_cache(hj_cache[0]),
    .xo_valid(xo_valid[0]), .xo_ready(xo_ready[0]), .xo_data(xo_data), .xo_last(xo_last),
    .ts_wslot(w_slot[2]), .ts_wen(w_en[2]), .ts_waddr(w_addr[2]), .ts_wdata(w_data[2]),
    .ts_rslot(r_slot[4]), .ts_ren(r_en[4]), .ts_raddr(r_addr[4]), .ts_rdata(r_data[4]));

  // ---------------- NTT (read port 0, write port 0) ----------------
  plc_ntt u_ntt (
    .clk(clk), .rst_n(rst_n_s), .start(nt_start), .inv(nt_inv), .pcode(pcode), .slot(nt_slot),
    .busy(nt_busy), .done(nt_done),
    .rslot(r_slot[0]), .ren(r_en[0]), .raddr(r_addr[0]), .rdata(r_data[0]),
    .wslot(w_slot[0]), .wen(w_en[0]), .waddr(w_addr[0]), .wdata(w_data[0]));

  // ---------------- PWALU (read ports 1..3, write port 1) ----------------
  plc_pwalu u_pw (
    .clk(clk), .rst_n(rst_n_s), .start(pw_start), .mode(pw_mode), .acc_zero(pw_acc_zero), .pcode(pcode),
    .slot_a(pw_sa), .slot_b(pw_sb), .slot_c(pw_sc), .slot_d(pw_sd), .cw(cw),
    .busy(pw_busy), .done(pw_done),
    .a_rslot(r_slot[1]), .a_ren(r_en[1]), .a_raddr(r_addr[1]), .a_rdata(r_data[1]),
    .b_rslot(r_slot[2]), .b_ren(r_en[2]), .b_raddr(r_addr[2]), .b_rdata(r_data[2]),
    .c_rslot(r_slot[3]), .c_ren(r_en[3]), .c_raddr(r_addr[3]), .c_rdata(r_data[3]),
    .d_wslot(w_slot[1]), .d_wen(w_en[1]), .d_waddr(w_addr[1]), .d_wdata(w_data[1]));

  // ---------------- CODEC (read port 5, write port 4, object ports) ----------------
  plc_codec u_codec (
    .clk(clk), .rst_n(rst_n_s), .start(cd_start), .op(cd_op), .k(cd_k), .pcode(pcode), .slot(cd_slot),
    .cmp(cd_cmp), .src_sk(cd_src_sk), .cmp_clr(cmd_start),
    .busy(cd_busy), .done(cd_done), .ovf(cd_ovf), .ne(cd_ne),
    .rg(rg), .rg_we(cd_rg_we), .rg_wdata(cd_rg_wdata),
    .rslot(r_slot[5]), .ren(r_en[5]), .raddr(r_addr[5]), .rdata(r_data[5]),
    .wslot(w_slot[4]), .wen(w_en[4]), .waddr(w_addr[4]), .wdata(w_data[4]),
    .cd_oren(cd_oren), .cd_oraddr(cd_oraddr), .cd_ordata(cd_ordata),
    .cd_owen(cd_owen), .cd_owaddr(cd_owaddr), .cd_owbe(cd_owbe), .cd_owdata(cd_owdata));

  // ---------------- POLAR (read port 6) ----------------
  plc_polar u_polar (
    .clk(clk), .rst_n(rst_n_s), .pcode(pcode),
    .enc_start(pe_start), .rg_m(rg[RG_M]), .enc_busy(pe_busy), .enc_done(pe_done), .cw(cw),
    .dec_start(pd_start), .dec_busy(pd_busy), .dec_done(pd_done),
    .pl_rslot(r_slot[6]), .pl_ren(r_en[6]), .pl_raddr(r_addr[6]), .pl_rdata(r_data[6]),
    .pd_m_we(pd_m_we), .pd_m_data(pd_m_data));
endmodule
