// ZEN KEM performance-version top level (ZEN_128/256/512 x KeyGen/Encaps/Decaps).
// Single clock clk; rst_n asynchronous active-low (asserted asynchronously, released through a
// 2-flop synchronizer).
module zen_kem_top (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         cmd_valid,
  output logic         cmd_ready,
  input  logic [1:0]   cmd_op,
  input  logic [1:0]   cmd_param,
  output logic         busy,
  output logic         done,
  output logic         fail,
  output logic         result_valid,
  input  logic         rnd_valid,
  output logic         rnd_ready,
  input  logic [63:0]  rnd_data,
  input  logic         host_en,
  input  logic         host_we,
  input  logic [1:0]   host_obj,
  input  logic [9:0]   host_addr,
  input  logic [63:0]  host_wdata,
  output logic [63:0]  host_rdata
);
  // ------------------------------------------------------------ reset synchronizer
  logic rst_n_s;
  zen_rst_sync u_rst_sync (.clk(clk), .rst_n_in(rst_n), .rst_n_out(rst_n_s));

  // ------------------------------------------------------------ wires
  logic [1:0]  param;
  logic [7:0]  nonce;
  logic        clr, accept;
  logic [5:0][1:0] rsel;
  logic [4:0][1:0] wsel;
  logic [5:0][1:0]        rd_en;
  logic [5:0][1:0][5:0]   rd_addr;
  logic [5:0][1:0][255:0] rd_data;
  logic [4:0][1:0]        wr_en;
  logic [4:0][1:0][5:0]   wr_addr;
  logic [4:0][1:0][255:0] wr_data;

  logic samp_start, samp_done, samp_act, samp_t0_odd; logic [6:0] samp_wpairs; logic [1:0] samp_poly; logic [511:0] samp_t0;
  logic ntt_start, ntt_done, ntt_act, ntt_zq, ntt_cmp, ntt_mism, fci_act, ntt_iss, bre_wbl; logic [1:0] ntt_mode;
  logic na_en, na_we, nb_en, nb_we; logic [6:0] na_addr, nb_addr; logic [127:0] na_wdata, nb_wdata, na_rdata, nb_rdata;
  logic bre_start, bre_done, bre_act; logic [1:0] bre_kind; logic [7:0] bre_wr_cnt;
  logic pass_start, pass_done, pass_act; logic [1:0] pass_mode;
  logic pku_start, pku_done, pku_act, pku_lim_bre, pm_gnt; logic [2:0] pku_op; logic [9:0] pk_words_done;
  logic gf2_start, gf2_done, gf2_act, gf2_op; logic [511:0] gf2_res;
  logic b_wr_en; logic [2:0] b_wr_idx; logic [63:0] b_wr_data;
  logic a_clr, a_wr_en; logic [5:0] a_wr_idx; logic [15:0] a_wr_data;
  logic m_load;

  logic xreq_valid, xreq_ready, xrsp_valid; logic [15:0] xreq_ctr; logic [3:0] xreq_tag, xrsp_tag; logic [255:0] xrsp_digest;
  logic t_hpk_start, t_rej_start, t_kr_start, t_smid_start, t_skld_start, t_skst_start, m_ok;
  logic [7:0] core_start, core_done;
  logic t_hpk_done, t_rej_done, t_kr_done, t_smid_done, t_skld_done, t_skst_done;
  logic pk_stream, ss_load, ss_sel_rej;
  logic rnd_we; logic [1:0] rnd_reg; logic [2:0] rnd_idx; logic [63:0] rnd_wdata;
  logic [511:0] m_reg, ss_reg;
  logic [7:0] core_busy; logic act_hpk, act_rej, act_kr, act_smid, act_xof, act_sk;

  // object ports
  logic        pk_en, pk_we, sk_en, sk_we, ct_en, ct_we;
  logic [9:0]  pk_addr, sk_addr; logic [6:0] ct_addr;
  logic [63:0] pk_wdata, sk_wdata, pk_rdata, sk_rdata;
  logic [7:0]  pk_wmask, sk_wmask;
  logic [127:0] ct_wdata, ct_rdata; logic [15:0] ct_wmask;
  logic        pkb_en, skb_en, skb_we, ctb_en;
  logic [9:0]  pkb_addr, skb_addr, ctb_addr;
  logic [63:0] pkb_rdata, skb_rdata, skb_wdata, ctb_rdata;
  logic [7:0]  skb_wmask;
  logic [63:0] om_h_rdata;

  // ------------------------------------------------------------ controller
  zen_ctrl u_ctrl (
    .clk(clk), .rst_n(rst_n_s),
    .cmd_valid(cmd_valid), .cmd_ready(cmd_ready), .cmd_op(cmd_op), .cmd_param(cmd_param),
    .busy(busy), .done(done), .fail(fail), .result_valid(result_valid), .accept(accept),
    .rnd_valid(rnd_valid), .rnd_ready(rnd_ready), .rnd_data(rnd_data),
    .rnd_we(rnd_we), .rnd_reg(rnd_reg), .rnd_idx(rnd_idx), .rnd_wdata(rnd_wdata),
    .param(param), .nonce(nonce), .clr(clr), .rsel(rsel), .wsel(wsel),
    .samp_start(samp_start), .samp_poly(samp_poly), .samp_done(samp_done), .samp_t0_odd(samp_t0_odd),
    .ntt_start(ntt_start), .ntt_mode(ntt_mode), .ntt_done(ntt_done), .ntt_zq_fail(ntt_zq),
    .bre_start(bre_start), .bre_kind(bre_kind), .bre_done(bre_done),
    .pass_start(pass_start), .pass_mode(pass_mode), .pass_done(pass_done),
    .ntt_cmp(ntt_cmp), .ntt_mismatch(ntt_mism), .ntt_iss_done(ntt_iss), .bre_wb_last(bre_wbl), .ntt_act_q(ntt_act), .bre_act_q(bre_act), .samp_wpairs(samp_wpairs), .pku_lim_bre(pku_lim_bre),
    .pku_start(pku_start), .pku_op(pku_op), .pku_done(pku_done),
    .gf2_start(gf2_start), .gf2_op(gf2_op), .gf2_done(gf2_done), .m_load(m_load),
    .t_hpk_start(t_hpk_start), .t_rej_start(t_rej_start), .t_kr_start(t_kr_start), .t_smid_start(t_smid_start),
    .t_skld_start(t_skld_start), .t_skst_start(t_skst_start), .m_ok(m_ok),
    .t_hpk_done(t_hpk_done), .t_rej_done(t_rej_done), .t_kr_done(t_kr_done), .t_smid_done(t_smid_done),
    .t_skld_done(t_skld_done), .t_skst_done(t_skst_done),
    .pk_stream(pk_stream), .ss_load(ss_load), .ss_sel_rej(ss_sel_rej));

  // ------------------------------------------------------------ polynomial memory
  zen_polymem u_polymem (.clk(clk), .rsel(rsel), .wsel(wsel),
    .rd_en(rd_en), .rd_addr(rd_addr), .rd_data(rd_data),
    .wr_en(wr_en), .wr_addr(wr_addr), .wr_data(wr_data), .rd_gnt_last(pm_gnt));

  // ------------------------------------------------------------ units
  zen_sampler u_samp (.clk(clk), .rst_n(rst_n_s), .start(samp_start), .param(param), .poly(samp_poly), .nonce(nonce),
    .done(samp_done), .active(samp_act), .wpairs(samp_wpairs),
    .wr_en(wr_en[0]), .wr_addr(wr_addr[0]), .wr_data(wr_data[0]),
    .xreq_valid(xreq_valid), .xreq_ready(xreq_ready), .xreq_ctr(xreq_ctr), .xreq_tag(xreq_tag),
    .xrsp_valid(xrsp_valid), .xrsp_tag(xrsp_tag), .xrsp_digest(xrsp_digest),
    .t0(samp_t0), .t0_odd(samp_t0_odd));

  zen_ntt u_ntt (.clk(clk), .rst_n(rst_n_s), .start(ntt_start), .param(param), .mode(ntt_mode),
    .cmp(ntt_cmp), .mbits(m_reg),
    .done(ntt_done), .active(ntt_act), .zq_fail(ntt_zq), .mismatch(ntt_mism), .fci_act(fci_act), .iss_done(ntt_iss),
    .rd_en(rd_en[0]), .rd_addr(rd_addr[0]), .rd_data(rd_data[0]),
    .wr_en(wr_en[1]), .wr_addr(wr_addr[1]), .wr_data(wr_data[1]),
    .re_rd_en(rd_en[4]), .re_rd_addr(rd_addr[4]), .re_rd_data(rd_data[4]),
    .cta_en(na_en), .cta_we(na_we), .cta_addr(na_addr), .cta_wdata(na_wdata), .cta_rdata(na_rdata),
    .ctb_en(nb_en), .ctb_we(nb_we), .ctb_addr(nb_addr), .ctb_wdata(nb_wdata), .ctb_rdata(nb_rdata));

  zen_bre u_bre (.clk(clk), .rst_n(rst_n_s), .start(bre_start), .param(param), .kind(bre_kind),
    .done(bre_done), .active(bre_act), .wr_cnt(bre_wr_cnt), .wb_last(bre_wbl),
    .ra_rd_en(rd_en[1]), .ra_rd_addr(rd_addr[1]), .ra_rd_data(rd_data[1]),
    .rb_rd_en(rd_en[2]), .rb_rd_addr(rd_addr[2]), .rb_rd_data(rd_data[2]),
    .wc_wr_en(wr_en[2]), .wc_wr_addr(wr_addr[2]), .wc_wr_data(wr_data[2]));

  // PASS only reads MEM_CT (CT_DECOMP) and never compares (FO compare fused into zen_ntt)
  assign ct_we     = 1'b0;
  assign ct_wdata  = '0;
  assign ct_wmask  = '0;
  zen_pass u_pass (.clk(clk), .rst_n(rst_n_s), .start(pass_start), .param(param), .mode(pass_mode),
    .done(pass_done), .active(pass_act),
    .r1_rd_en(rd_en[3]), .r1_rd_addr(rd_addr[3]), .r1_rd_data(rd_data[3]),
    .w_wr_en(wr_en[3]), .w_wr_addr(wr_addr[3]), .w_wr_data(wr_data[3]),
    .ct_en(ct_en), .ct_addr(ct_addr), .ct_rdata(ct_rdata),
    .a_clr(a_clr), .a_wr_en(a_wr_en), .a_wr_idx(a_wr_idx), .a_wr_data(a_wr_data));

  zen_pku u_pku (.clk(clk), .rst_n(rst_n_s), .start(pku_start), .param(param), .op(pku_op),
    .done(pku_done), .active(pku_act),
    .rd_en(rd_en[5]), .rd_gnt(pm_gnt), .rd_limit(pku_lim_bre ? bre_wr_cnt : 8'd128), .rd_addr(rd_addr[5]), .rd_data(rd_data[5]),
    .wr_en(wr_en[4]), .wr_addr(wr_addr[4]), .wr_data(wr_data[4]),
    .pk_en(pk_en), .pk_we(pk_we), .pk_addr(pk_addr), .pk_wdata(pk_wdata), .pk_wmask(pk_wmask), .pk_rdata(pk_rdata),
    .sk_en(sk_en), .sk_we(sk_we), .sk_addr(sk_addr), .sk_wdata(sk_wdata), .sk_wmask(sk_wmask), .sk_rdata(sk_rdata),
    .f2_in(gf2_res), .b_wr_en(b_wr_en), .b_wr_idx(b_wr_idx), .b_wr_data(b_wr_data),
    .pk_words_done(pk_words_done));

  zen_gf2 u_gf2 (.clk(clk), .rst_n(rst_n_s), .start(gf2_start), .param(param), .op(gf2_op), .f_in(samp_t0),
    .b_wr_en(b_wr_en), .b_wr_idx(b_wr_idx), .b_wr_data(b_wr_data),
    .a_clr(a_clr), .a_wr_en(a_wr_en), .a_wr_idx(a_wr_idx), .a_wr_data(a_wr_data),
    .done(gf2_done), .active(gf2_act), .res(gf2_res));

  zen_hash_sys u_hash (.clk(clk), .rst_n(rst_n_s), .param(param), .clr(clr),
    .t_hpk_start(t_hpk_start), .t_rej_start(t_rej_start), .t_kr_start(t_kr_start), .t_smid_start(t_smid_start),
    .t_skld_start(t_skld_start), .t_skst_start(t_skst_start), .m_ok(m_ok),
    .t_hpk_done(t_hpk_done), .t_rej_done(t_rej_done), .t_kr_done(t_kr_done), .t_smid_done(t_smid_done),
    .t_skld_done(t_skld_done), .t_skst_done(t_skst_done),
    .rnd_we(rnd_we), .rnd_reg(rnd_reg), .rnd_idx(rnd_idx), .rnd_data(rnd_wdata),
    .m_load(m_load), .mload(gf2_res), .ss_load(ss_load), .ss_sel_rej(ss_sel_rej),
    .m_reg(m_reg), .ss_reg(ss_reg),
    .xof_nonce(nonce), .xreq_valid(xreq_valid), .xreq_ready(xreq_ready), .xreq_ctr(xreq_ctr), .xreq_tag(xreq_tag),
    .xrsp_valid(xrsp_valid), .xrsp_tag(xrsp_tag), .xrsp_digest(xrsp_digest),
    .pk_limit(pk_stream ? {1'b0, pk_words_done} : 11'd1024),
    .pkb_en(pkb_en), .pkb_addr(pkb_addr), .pkb_rdata(pkb_rdata),
    .skb_en(skb_en), .skb_we(skb_we), .skb_addr(skb_addr), .skb_wdata(skb_wdata), .skb_wmask(skb_wmask), .skb_rdata(skb_rdata),
    .ctb_en(ctb_en), .ctb_addr(ctb_addr), .ctb_rdata(ctb_rdata),
    .core_busy(core_busy), .core_start(core_start), .core_done(core_done), .act_hpk(act_hpk), .act_rej(act_rej), .act_kr(act_kr), .act_smid(act_smid),
    .act_xof(act_xof), .act_sk(act_sk));

  // ------------------------------------------------------------ object memories + host port
  logic h_ok;
  assign h_ok = host_en && !busy && !accept;
  zen_objmem u_objmem (.clk(clk), .dp_sel(busy),
    .h_en(h_ok), .h_we(host_we), .h_obj(host_obj), .h_addr(host_addr), .h_wdata(host_wdata), .h_rdata(om_h_rdata),
    .pk_en(pk_en), .pk_we(pk_we), .pk_addr(pk_addr), .pk_wdata(pk_wdata), .pk_wmask(pk_wmask), .pk_rdata(pk_rdata),
    .sk_en(sk_en), .sk_we(sk_we), .sk_addr(sk_addr), .sk_wdata(sk_wdata), .sk_wmask(sk_wmask), .sk_rdata(sk_rdata),
    .ct_en(ct_en), .ct_we(ct_we), .ct_addr(ct_addr), .ct_wdata(ct_wdata), .ct_wmask(ct_wmask), .ct_rdata(ct_rdata),
    .pkb_en(pkb_en), .pkb_addr(pkb_addr), .pkb_rdata(pkb_rdata),
    .skb_en(skb_en), .skb_we(skb_we), .skb_addr(skb_addr), .skb_wdata(skb_wdata), .skb_wmask(skb_wmask), .skb_rdata(skb_rdata),
    .ctb_en(ctb_en), .ctb_addr(ctb_addr), .ctb_rdata(ctb_rdata),
    .na_en(na_en), .na_we(na_we), .na_addr(na_addr), .na_wdata(na_wdata), .na_rdata(na_rdata),
    .nb_en(nb_en), .nb_we(nb_we), .nb_addr(nb_addr), .nb_wdata(nb_wdata), .nb_rdata(nb_rdata));

  // host read: cycle 1 = memory access, cycle 2 = output register
  logic       hrd_q, hss_q;
  logic [2:0] hss_idx_q;
  always_ff @(posedge clk or negedge rst_n_s) begin
    if (!rst_n_s) begin hrd_q <= 1'b0; hss_q <= 1'b0; hss_idx_q <= '0; host_rdata <= '0; end
    else begin
      hrd_q     <= h_ok && !host_we;
      hss_q     <= (host_obj == 2'd3);
      hss_idx_q <= host_addr[2:0];
      if (hrd_q) host_rdata <= hss_q ? ss_reg[64*hss_idx_q +: 64] : om_h_rdata;
    end
  end
endmodule

// 2-flop reset synchronizer: asynchronous assertion, synchronous release.
module zen_rst_sync (
  input  logic clk,
  input  logic rst_n_in,
  output logic rst_n_out
);
  logic r1;
  always_ff @(posedge clk or negedge rst_n_in) begin
    if (!rst_n_in) begin r1 <= 1'b0; rst_n_out <= 1'b0; end
    else begin r1 <= 1'b1; rst_n_out <= r1; end
  end
endmodule
