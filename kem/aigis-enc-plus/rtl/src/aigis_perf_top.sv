// Aigis-Enc+ KEM accelerator, performance version: KeyGen / Enc / Dec for Aigis-Enc+-I/II/III with
// the ICCS-SM3 backend, one clock (clk), asynchronous active-low reset (rst_n).
module aigis_perf_top (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        cmd_valid,
  output logic        cmd_ready,
  input  logic [1:0]  cmd_op,
  input  logic [1:0]  cmd_param,
  output logic        busy,
  output logic        done,
  output logic        err,
  input  logic        rnd_valid,
  output logic        rnd_ready,
  input  logic [63:0] rnd_data,
  input  logic        host_en,
  input  logic        host_we,
  input  logic [11:0] host_addr,
  input  logic [63:0] host_wdata,
  output logic [63:0] host_rdata
);
  import aigis_pkg::*;
  import aigis_ctrl_pkg::*;

  logic rn;
  aigis_rst_sync u_rst (.clk(clk), .rst_n(rst_n), .rst_n_out(rn));

  // ---------------------------------------------------------------- controller
  logic [1:0]  param;
  logic        accept, fail;
  logic [11:0] u_done, u_start, u_sdone;
  job_t [11:0] cur;
  aigis_ctrl u_ctrl (.clk, .rst_n(rn), .cmd_valid, .cmd_ready, .cmd_op, .cmd_param, .busy, .done, .err,
                     .param, .accept, .fail, .u_done, .u_start, .cur, .u_sdone);

  // ---------------------------------------------------------------- poly memory roles
  localparam int NR = 13, NW = 9;
  logic [2:0] ntt_rsel, ntt_wsel; logic [7:0] mac_prog; logic [127:0] ntt_wfin;
  logic [2:0] mac_sa, mac_sb, mac_sp, mac_sc;
  logic [63:0] n_re, n_we; logic [63:0][4:0] n_ra, n_wa; logic [63:0][11:0] n_rd, n_wd;
  logic [NR-1:0]            ract;
  logic [NR-1:0][2:0]       rsel;
  logic [NR-1:0][31:0]      re;
  logic [NR-1:0][31:0][5:0] ra;
  logic [NR-1:0][31:0][11:0] rd;
  logic [NW-1:0]            wact;
  logic [NW-1:0][2:0]       wsel;
  logic [NW-1:0][31:0]      we;
  logic [NW-1:0][31:0][5:0] wa;
  logic [NW-1:0][31:0][11:0] wd;
  assign ract = '1;
  assign wact = '1;
  always_comb begin
    rsel[0]  = cur[U_NTT].s[0];
    rsel[1]  = mac_sa; rsel[2] = mac_sp; rsel[3] = mac_sb;
    for (int k = 0; k < 2; k++) rsel[4 + k] = cur[U_WP].s[k];
    rsel[6]  = cur[U_SPK].s[0]; rsel[7]  = cur[U_SPK].s[1];
    rsel[8]  = cur[U_SSK].s[0]; rsel[9]  = cur[U_SSK].s[1];
    rsel[10] = cur[U_SCT].s[0]; rsel[11] = cur[U_SCT].s[1];
    rsel[12] = cur[U_NX].s[0];
    wsel[0] = cur[U_NTT].s[0];
    wsel[1] = mac_sc; wsel[2] = 3'd0;
    wsel[3] = cur[U_SPK].s[0]; wsel[4] = cur[U_SSK].s[0];
    wsel[5] = cur[U_SCT].s[0]; wsel[6] = cur[U_SCT].s[1];
    wsel[7] = cur[U_GA].s[0];  wsel[8] = cur[U_NX].s[0];
  end
  aigis_polymem #(.NR(NR), .NW(NW)) u_pm (.clk, .ract, .rsel, .re, .ra, .rd, .wact, .wsel, .we, .wa, .wd,
    .n_rsel(ntt_rsel), .n_re, .n_ra, .n_rd, .n_wsel(ntt_wsel), .n_we, .n_wa, .n_wd);
  // logical NTT role slots unused (role 0)
  assign re[0] = '0; assign ra[0] = '0; assign we[0] = '0; assign wa[0] = '0; assign wd[0] = '0;

  // ---------------------------------------------------------------- object memory
  logic [6:0]       or_req, or_gnt, or_rv;
  logic [6:0][1:0]  or_sel;
  logic [6:0][9:0]  or_addr;
  logic [6:0][63:0] or_data;
  logic [4:0]       ow_req, ow_gnt;
  logic [4:0][1:0]  ow_sel;
  logic [4:0][9:0]  ow_addr;
  logic [4:0][63:0] ow_data;
  logic ss_we; logic [2:0] ss_idx; logic [63:0] ss_data;
  logic [2:0] st_busy;
  logic spk_fail, ssk_fail, sct_fail;
  assign fail = spk_fail | sct_fail;   // Dec re-encryption mismatch (compare jobs on u_spk / u_sct)
  logic [327:0] pkw;
  aigis_objmem #(.NRC(7), .NWC(5)) u_om (.clk, .rst_n(rn), .busy, .clr(accept), .host_en, .host_we, .host_addr, .host_wdata, .host_rdata,
                     .r_req(or_req), .r_sel(or_sel), .r_addr(or_addr), .r_gnt(or_gnt), .r_rv(or_rv), .r_data(or_data),
                     .w_req(ow_req), .w_sel(ow_sel), .w_addr(ow_addr), .w_data(ow_data), .w_gnt(ow_gnt),
                     .ss_we, .ss_idx, .ss_data, .pkw,
                     .mirror_en(st_busy[0] && cur[U_SPK].mode == ST_S10M), .mirror_off(10'(poly12_bytes(param) >> 3)));

  // ---------------------------------------------------------------- hash subsystem + regfile
  logic [87:0][63:0] rf;
  logic cp_rf_we, wp_t_we; logic [6:0] cp_rf_waddr, wp_t_waddr; logic [63:0] cp_rf_wdata, wp_t_wdata;
  logic ha_busy, hb_busy, rx_busy;
  aigis_hsu u_hsu (.clk, .rst_n(rn), .param, .clr(accept),
    .a_start(u_start[U_HA]), .a_job(cur[U_HA].mode[0]), .a_dbl(cur[U_HA].mode[2]), .a_waitpk(cur[U_HA].mode[1]),
    .a_busy(ha_busy), .a_done(u_done[U_HA]),
    .b_start(u_start[U_HB]), .b_busy(hb_busy), .b_done(u_done[U_HB]),
    .x_start(u_start[U_RX]), .x_dst(cur[U_RX].a1[6:0]), .x_nwords(cur[U_RX].cnt[3:0]), .x_busy(rx_busy), .x_done(u_done[U_RX]),
    .rnd_valid, .rnd_ready, .rnd_data,
    .e_we(cp_rf_we), .e_waddr(cp_rf_waddr), .e_wdata(cp_rf_wdata),
    .t_we(wp_t_we), .t_waddr(wp_t_waddr), .t_wdata(wp_t_wdata), .rf,
    .ob_req(or_req[2:0]), .ob_sel(or_sel[2:0]), .ob_addr(or_addr[2:0]), .ob_gnt(or_gnt[2:0]), .ob_rv(or_rv[2:0]),
    .ob_rdata(or_data[2:0]), .pkw,
    .sk_req(ow_req[4]), .sk_addr(ow_addr[4]), .sk_data(ow_data[4]), .sk_gnt(ow_gnt[4]));
  assign ow_sel[4] = 2'd1;

  logic [511:0] msgbits;
  always_comb for (int i = 0; i < 8; i++) msgbits[64*i +: 64] = rf[i];
  function automatic logic [511:0] rf_seed(input logic [87:0][63:0] r, input logic [9:0] base);
    for (int i = 0; i < 8; i++) rf_seed[64*i +: 64] = r[7'(base) + 7'(i)];
  endfunction
  logic [511:0] ga_seed, nx_seed;
  always_comb begin ga_seed = rf_seed(rf, cur[U_GA].a1); nx_seed = rf_seed(rf, cur[U_NX].a1); end

  // ---------------------------------------------------------------- SM3 pool: gen_a + noise
  logic ga_busy, nx_busy;
  logic [3:0] ga_qdone; logic [3:0][9:0] ga_qprog;
  aigis_pool u_pool (.clk, .rst_n(rn), .param,
    .ga_start(u_start[U_GA]), .ga_seed(ga_seed), .ga_busy(ga_busy), .ga_done(u_done[U_GA]), .ga_qdone(ga_qdone), .ga_qprog(ga_qprog),
    .ga_we(we[7]), .ga_wa(wa[7]), .ga_wd(wd[7]),
    .nx_start(u_start[U_NX]), .nx_eta(cur[U_NX].k2), .nx_nonce({5'd0, cur[U_NX].k1}), .nx_seed(nx_seed),
    .nx_busy(nx_busy), .nx_done(u_done[U_NX]),
    .nx_re(re[12]), .nx_ra(ra[12]), .nx_rd(rd[12]), .nx_we(we[8]), .nx_wa(wa[8]), .nx_wd(wd[8]));

  // ---------------------------------------------------------------- NTT / MAC / word passes
  logic ntt_busy, mac_busy, wp_busy;
  aigis_ntt u_ntt (.clk, .rst_n(rn), .start(u_start[U_NTT]), .inv(cur[U_NTT].mode[0]), .param,
    .follow(cur[U_NTT].mode[1]), .fprog(mac_prog), .fdone(u_sdone[U_NTT]), .s_src(cur[U_NTT].s[0]), .s_dst(cur[U_NTT].s[1]),
    .s_fin(cur[U_NTT].mode[2] ? cur[U_NTT].s[2] : (cur[U_NTT].mode[1] ? cur[U_NTT].s[1] : cur[U_NTT].s[0])), .wfin(ntt_wfin),
    .rsel(ntt_rsel), .wsel(ntt_wsel),
    .busy(ntt_busy), .done(u_done[U_NTT]), .re(n_re), .ra(n_ra), .rd(n_rd), .we(n_we), .wa(n_wa), .wd(n_wd));
  aigis_mac u_mac (.clk, .rst_n(rn), .start(u_start[U_MAC]), .pass_mode(cur[U_MAC].mode[0]), .fol(cur[U_MAC].mode[1]),
    .oop(cur[U_MAC].mode[2]), .param,
    .s_a(cur[U_MAC].s[0]), .s_b(cur[U_MAC].s[1]), .s_out(cur[U_MAC].s[2]), .s_x(cur[U_MAC].s[3]), .s_y(cur[U_MAC].s[4]),
    .nwfin(ntt_wfin), .fdone(u_sdone[U_MAC]), .qprog(ga_qprog), .qall(ga_qdone == 4'hf && !ga_busy),
    .busy(mac_busy), .done(u_done[U_MAC]), .prog(mac_prog), .sel_a(mac_sa), .sel_b(mac_sb), .sel_p(mac_sp), .sel_c(mac_sc),
    .re_a(re[1]), .re_p(re[2]), .re_b(re[3]), .ra_a(ra[1]), .ra_p(ra[2]), .ra_b(ra[3]),
    .rd_a(rd[1]), .rd_p(rd[2]), .rd_b(rd[3]),
    .we_c(we[1]), .wa_c(wa[1]), .wd_c(wd[1]));
  assign we[2] = '0; assign wa[2] = '0; assign wd[2] = '0;
  logic [1:0][31:0] wp_re; logic [1:0][31:0][5:0] wp_ra; logic [1:0][31:0][11:0] wp_rd;
  always_comb for (int k = 0; k < 2; k++) begin re[4 + k] = wp_re[k]; ra[4 + k] = wp_ra[k]; wp_rd[k] = rd[4 + k]; end
  aigis_wpass u_wp (.clk, .rst_n(rn), .start(u_start[U_WP]), .param, .msg(msgbits),
    .busy(wp_busy), .done(u_done[U_WP]), .re(wp_re), .ra(wp_ra), .rd(wp_rd),
    .t_we(wp_t_we), .t_waddr(wp_t_waddr), .t_wdata(wp_t_wdata));

  // ---------------------------------------------------------------- stream units (PK, SK, CT)
  logic [31:0] st_we2_unused [0:1];
  logic [1:0][31:0] nul_we; logic [1:0][31:0][5:0] nul_wa; logic [1:0][31:0][11:0] nul_wd;
  aigis_stream #(.NE(4)) u_spk (.clk, .rst_n(rn), .start(u_start[U_SPK]), .dir(cur[U_SPK].mode[4]), .fmt(cur[U_SPK].mode[3:2]),
    .dec(cur[U_SPK].mode[1]), .dual(cur[U_SPK].mode[0]), .base(cur[U_SPK].a1), .param, .msg(msgbits),
    .xfol(cur[U_SPK].sdeps != '0), .xfin(ntt_wfin), .xdone(u_sdone[U_SPK]),
    .busy(st_busy[0]), .done(u_done[U_SPK]), .cmp_fail(spk_fail),
    .re_x(re[6]), .re_y(re[7]), .ra_x(ra[6]), .ra_y(ra[7]), .rd_x(rd[6]), .rd_y(rd[7]),
    .we_1(we[3]), .we_2(nul_we[0]), .wa_1(wa[3]), .wa_2(nul_wa[0]), .wd_1(wd[3]), .wd_2(nul_wd[0]),
    .br_req(or_req[4]), .br_addr(or_addr[4]), .br_gnt(or_gnt[4]), .br_rv(or_rv[4]), .br_data(or_data[4]),
    .bw_req(ow_req[1]), .bw_addr(ow_addr[1]), .bw_data(ow_data[1]), .bw_gnt(ow_gnt[1]));
  aigis_stream #(.NE(4)) u_ssk (.clk, .rst_n(rn), .start(u_start[U_SSK]), .dir(cur[U_SSK].mode[4]), .fmt(cur[U_SSK].mode[3:2]),
    .dec(cur[U_SSK].mode[1]), .dual(cur[U_SSK].mode[0]), .base(cur[U_SSK].a1), .param, .msg(msgbits),
    .xfol(1'b0), .xfin(ntt_wfin), .xdone(1'b1),
    .busy(st_busy[1]), .done(u_done[U_SSK]), .cmp_fail(ssk_fail),
    .re_x(re[8]), .re_y(re[9]), .ra_x(ra[8]), .ra_y(ra[9]), .rd_x(rd[8]), .rd_y(rd[9]),
    .we_1(we[4]), .we_2(nul_we[1]), .wa_1(wa[4]), .wa_2(nul_wa[1]), .wd_1(wd[4]), .wd_2(nul_wd[1]),
    .br_req(or_req[5]), .br_addr(or_addr[5]), .br_gnt(or_gnt[5]), .br_rv(or_rv[5]), .br_data(or_data[5]),
    .bw_req(ow_req[2]), .bw_addr(ow_addr[2]), .bw_data(ow_data[2]), .bw_gnt(ow_gnt[2]));
  aigis_stream #(.NE(8)) u_sct (.clk, .rst_n(rn), .start(u_start[U_SCT]), .dir(cur[U_SCT].mode[4]), .fmt(cur[U_SCT].mode[3:2]),
    .dec(cur[U_SCT].mode[1]), .dual(cur[U_SCT].mode[0]), .base(cur[U_SCT].a1), .param, .msg(msgbits),
    .xfol(1'b0), .xfin(ntt_wfin), .xdone(1'b1),
    .busy(st_busy[2]), .done(u_done[U_SCT]), .cmp_fail(sct_fail),
    .re_x(re[10]), .re_y(re[11]), .ra_x(ra[10]), .ra_y(ra[11]), .rd_x(rd[10]), .rd_y(rd[11]),
    .we_1(we[5]), .we_2(we[6]), .wa_1(wa[5]), .wa_2(wa[6]), .wd_1(wd[5]), .wd_2(wd[6]),
    .br_req(or_req[6]), .br_addr(or_addr[6]), .br_gnt(or_gnt[6]), .br_rv(or_rv[6]), .br_data(or_data[6]),
    .bw_req(ow_req[3]), .bw_addr(ow_addr[3]), .bw_data(ow_data[3]), .bw_gnt(ow_gnt[3]));
  // u_spk serves PK, or CT when its job names it (k1 = 2: Enc u pack)
  assign or_sel[4] = cur[U_SPK].k1[1:0]; assign or_sel[5] = 2'd1; assign or_sel[6] = 2'd2;
  assign ow_sel[1] = cur[U_SPK].k1[1:0]; assign ow_sel[2] = 2'd1; assign ow_sel[3] = 2'd2;

  // ---------------------------------------------------------------- copy engine
  logic cp_busy;
  aigis_cp u_cp (.clk, .rst_n(rn), .start(u_start[U_CP]), .sk(cur[U_CP].k1), .sa(cur[U_CP].a1), .dk(cur[U_CP].k2),
    .da(cur[U_CP].a2), .n(cur[U_CP].cnt), .busy(cp_busy), .done(u_done[U_CP]), .rf,
    .rf_we(cp_rf_we), .rf_waddr(cp_rf_waddr), .rf_wdata(cp_rf_wdata),
    .br_req(or_req[3]), .br_sel(or_sel[3]), .br_addr(or_addr[3]), .br_gnt(or_gnt[3]), .br_rv(or_rv[3]), .br_data(or_data[3]),
    .bw_req(ow_req[0]), .bw_sel(ow_sel[0]), .bw_addr(ow_addr[0]), .bw_data(ow_data[0]), .bw_gnt(ow_gnt[0]),
    .ss_we, .ss_idx, .ss_data);
endmodule
