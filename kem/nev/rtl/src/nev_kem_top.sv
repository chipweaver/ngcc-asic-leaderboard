// nev_kem_top: NEV KEM (ICCS backend) performance version - KeyGen / Enc / Dec for 12 parameter sets.
module nev_kem_top import nev_pkg::*; import nev_palu_pkg::*; (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        start,
  input  logic [1:0]  op,
  input  logic [3:0]  param,
  input  logic [1:0]  tst_fail_f,
  input  logic [1:0]  tst_fail_g,
  output logic        busy,
  output logic        done,
  output logic        fail,
  output logic        rnd_req,
  input  logic        rnd_valid,
  input  logic [63:0] rnd_data,
  input  logic        mem_cs,
  input  logic        mem_we,
  input  logic [10:0] mem_addr,
  input  logic [7:0]  mem_be,
  input  logic [63:0] mem_wdata,
  output logic [63:0] mem_rdata
);
  logic rst_i;
  nev_rst_sync u_rst (.clk(clk), .rst_n(rst_n), .rst_sync_n(rst_i));

  pset_t ps;
  // per-unit registered copies of the latched parameter set (quasi-static; every unit receives its
  // first command at least one cycle after acceptance) - keeps the design-wide fanout off the
  // datapath timing paths
  pset_t ps_h, ps_s, ps_p, ps_a, ps_b;
  always_ff @(posedge clk or negedge rst_i) begin
    if (!rst_i) begin ps_h <= '0; ps_s <= '0; ps_p <= '0; ps_a <= '0; ps_b <= '0; end
    else begin ps_h <= ps; ps_s <= ps; ps_p <= ps; ps_a <= ps; ps_b <= ps; end
  end
  logic [2:0] klog;
  assign klog = (ps_p.d64 ? 3'd3 : 3'd2) + {1'b0, ps_p.nlog};

  // ---------------- sequencer
  logic        h_valid, h_seed_sel, h_pkall, h_busy, h_done, h_done1;
  logic [2:0]  h_op;
  logic [7:0]  h_nonce, h_nonce1;
  logic [6:0]  h_nblocks, h_nblocks1;
  logic [11:0] h_nmlen;
  logic        s_valid, s_region, s_rinv, s_vinv, s_busy, s_done, s_need;
  logic        s1_valid, s1_done;
  logic [2:0]  s1_dst;
  logic [3:0]  s_dist;
  logic [2:0]  s_dst;
  logic        sp_valid, p_busy, p_done, p_zflag;
  palu_cmd_t   sp_cmd;
  logic        q_start, q_judge, q_inj, q_busy, q_done, q_noninv, q_avalid;
  logic [2:0]  q_bA, q_bYA0, q_bCA0, q_bT1, q_bT2, q_bYCH, q_bCCH;
  logic [11:0] q_cval;
  palu_cmd_t   q_acmd;
  logic        ca_valid, cb_valid, ca_busy, cb_busy, ca_done, cb_done, ca_mm, cb_mm, ca_mwe, cb_mwe;
  logic [2:0]  ca_mode, cb_mode, ca_buf, cb_buf;
  logic [1:0]  ca_field, cb_field;
  logic [8:0]  ca_ww, cb_ww;
  logic [511:0] ca_mdata, cb_mdata;
  logic        hpk_we, ss_we, ss_clr;
  logic        h_rnd_go, h_rnd_fin;
  logic        ca_strm, cb_strm;
  logic [7:0]  p_wm;
  logic        p_nz, ca_nc, cb_nc, ca_pwnx, cb_pwnx, p_mwe;
  logic [511:0] p_mdata;
  logic [7:0]  ca_pwc, cb_pwc;
  logic [5:0]  pr_gnt;

  nev_seq u_seq (
    .clk(clk), .rst_n(rst_i), .start(start), .op(op), .param(param), .tst_fail_f(tst_fail_f), .tst_fail_g(tst_fail_g),
    .busy(busy), .done(done), .fail(fail), .ps(ps),
    .h_valid(h_valid), .h_op(h_op), .h_seed_sel(h_seed_sel), .h_nonce(h_nonce), .h_nblocks(h_nblocks),
    .h_nonce1(h_nonce1), .h_nblocks1(h_nblocks1), .h_nmlen(h_nmlen), .h_pkall(h_pkall), .h_rnd_go(h_rnd_go), .h_rnd_fin(h_rnd_fin), .h_busy(h_busy), .h_done(h_done),
    .h_done1(h_done1),
    .s_valid(s_valid), .s_dist(s_dist), .s_region(s_region), .s_dst(s_dst), .s_rinv(s_rinv), .s_vinv(s_vinv),
    .s_busy(s_busy), .s_done(s_done), .s_need(s_need),
    .s1_valid(s1_valid), .s1_dst(s1_dst), .s1_done(s1_done),
    .p_valid(sp_valid), .p_cmd(sp_cmd), .p_done(p_done), .p_nz(p_nz),
    .q_start(q_start), .q_judge(q_judge), .q_inj(q_inj), .q_bA(q_bA), .q_bYA0(q_bYA0), .q_bCA0(q_bCA0), .q_bT1(q_bT1),
    .q_bT2(q_bT2), .q_bYCH(q_bYCH), .q_bCCH(q_bCCH), .q_cval(q_cval), .q_done(q_done), .q_noninv(q_noninv),
    .ca_valid(ca_valid), .cb_valid(cb_valid), .ca_mode(ca_mode), .cb_mode(cb_mode), .ca_field(ca_field), .cb_field(cb_field),
    .ca_buf(ca_buf), .cb_buf(cb_buf), .ca_done(ca_done), .cb_done(cb_done), .ca_mismatch(ca_mm), .cb_ncanon(cb_nc), .ca_strm(ca_strm), .cb_strm(cb_strm),
    .hpk_we(hpk_we), .ss_we(ss_we), .ss_clr(ss_clr));

  // ---------------- object memory
  logic [2:0]        om_req, om_we, om_gnt, om_rvalid;
  logic [2:0][1:0]   om_sel;
  logic [2:0][8:0]   om_addr;
  logic [2:0][127:0] om_wdata, om_rdata;
  logic [2:0][15:0]  om_be;
  logic [2:0]        om_wide;
  logic [63:0]       h_om_wdata;
  logic [7:0]        h_om_be;
  assign om_wdata[0] = {64'd0, h_om_wdata};   // hash port: narrow, read only
  assign om_be[0]    = {8'd0, h_om_be};
  assign om_wide[0]  = 1'b0;
  logic [511:0]      hpk_q, ss_q, kr_lo, hk;
  logic [1023:0]     kr;
  logic [511:0]      ss_wdata;
  always_comb begin
    for (int i = 0; i < 64; i++) ss_wdata[8*i +: 8] = (i < int'(sbytes(ps.slog))) ? kr[8*i +: 8] : 8'd0;
  end
  nev_objmem #(.NU(3)) u_om (
    .clk(clk), .rst_n(rst_i), .busy(busy),
    .mem_cs(mem_cs & rst_i), .mem_we(mem_we), .mem_addr(mem_addr), .mem_be(mem_be), .mem_wdata(mem_wdata), .mem_rdata(mem_rdata),
    .om_req(om_req), .om_we(om_we), .om_sel(om_sel), .om_addr(om_addr), .om_wdata(om_wdata), .om_be(om_be),
    .om_wide(om_wide), .om_gnt(om_gnt), .om_rvalid(om_rvalid), .om_rdata(om_rdata),
    .hpk_we(hpk_we), .hpk_wdata(hk), .hpk_q(hpk_q), .ss_we(ss_we), .ss_wdata(ss_wdata), .ss_clr(ss_clr), .ss_q(ss_q));

  // ---------------- hash unit
  logic        kb_we;
  logic [7:0]  kb_waddr;
  logic [127:0] kb_wdata;
  logic [7:0]  kb_wm0, kb_wm1;
  logic [511:0] c0, m_reg;
  nev_hash u_hash (
    .clk(clk), .rst_n(rst_i), .slog(ps_h.slog), .poly_bytes(ps_h.poly_bytes),
    .cmd_valid(h_valid), .cmd_op(h_op), .kdf_seed_sel(h_seed_sel), .kdf_nonce(h_nonce),
    .kdf_nblocks(h_nblocks), .kdf_nmlen(h_nmlen), .kdf_nonce1(h_nonce1), .kdf_nblocks1(h_nblocks1), .kdf_need(s_need),
    .busy(h_busy), .done(h_done),
    .rnd_req(rnd_req), .rnd_go(h_rnd_go), .rnd_fin(h_rnd_fin), .rnd_valid(rnd_valid), .rnd_data(rnd_data),
    .pk_avail(h_pkall ? 9'd384 : cb_ww),
    .om_req(om_req[0]), .om_we(om_we[0]), .om_sel(om_sel[0]), .om_addr(om_addr[0]), .om_wdata(h_om_wdata), .om_be(h_om_be),
    .om_gnt(om_gnt[0]), .om_rvalid(om_rvalid[0]), .om_rdata(om_rdata[0][63:0]),
    .hpk_field(hpk_q), .m_we(ca_mwe | cb_mwe | p_mwe), .m_wdata(p_mwe ? p_mdata : (cb_mwe ? cb_mdata : ca_mdata)),
    .kb_we(kb_we), .kb_waddr(kb_waddr), .kb_wdata(kb_wdata), .kb_wm0(kb_wm0), .kb_wm1(kb_wm1), .kdf_done1(h_done1),
    .kr(kr), .hk(hk), .c0(c0), .m_reg(m_reg));

  // ---------------- poly buffers
  // read ports: 0-3 poly ALU, 4 codec A, 5 codec B; write ports: 0-3 poly ALU, 4 sampler A, 5 codec A, 6 codec B, 7 sampler B
  logic [5:0]        pr_v;
  logic [5:0][2:0]   pr_buf;
  logic [5:0][6:0]   pr_addr;
  logic [5:0][WW-1:0] pr_data;
  logic [7:0]        pw_v;
  logic [7:0][2:0]   pw_buf;
  logic [7:0][6:0]   pw_addr;
  logic [7:0][WW-1:0] pw_data;
  nev_pbuf #(.NR(6), .NW(8)) u_pbuf (
    .clk(clk), .rst_n(rst_i), .rv(pr_v), .rbuf(pr_buf), .raddr(pr_addr), .rdata(pr_data), .rgnt(pr_gnt),
    .wv(pw_v), .wbuf(pw_buf), .waddr(pw_addr), .wdata(pw_data));

  // ---------------- samplers: A reads KDF stream 0 (region 0: f/g/r), B reads stream 1 (region 1: noisem e)
  nev_sampler u_samp (
    .clk(clk), .rst_n(rst_i), .qsel(ps_s.qsel), .nlog(ps_s.nlog), .d64(ps_s.d64), .slog(ps_s.slog),
    .cmd_valid(s_valid), .dist_code(s_dist), .region(1'b0), .dst_buf(s_dst), .scale_rinv(s_rinv), .add_vinv(s_vinv),
    .eta_e(ps_s.eta_e), .c0(c0), .c0_ok(1'b0), .busy(s_busy), .done(s_done), .kdf_need(s_need),
    .kb_we(kb_we & ~kb_waddr[7]), .kb_waddr(kb_waddr), .kb_wdata(kb_wdata), .kb_blocks(kb_wm0[6:0]), .kb_region(1'b0),
    .pw_valid(pw_v[4]), .pw_buf(pw_buf[4]), .pw_addr(pw_addr[4]), .pw_data(pw_data[4]));
  logic s1_busy, s1_need;
  nev_sampler u_samp_e (
    .clk(clk), .rst_n(rst_i), .qsel(ps_s.qsel), .nlog(ps_s.nlog), .d64(ps_s.d64), .slog(ps_s.slog),
    .cmd_valid(s1_valid), .dist_code(4'd15), .region(1'b1), .dst_buf(s1_dst), .scale_rinv(1'b0), .add_vinv(1'b0),
    .eta_e(ps_s.eta_e), .c0(c0), .c0_ok(h_done1), .busy(s1_busy), .done(s1_done), .kdf_need(s1_need),
    .kb_we(kb_we & kb_waddr[7]), .kb_waddr(kb_waddr), .kb_wdata(kb_wdata), .kb_blocks(kb_wm1[6:0]), .kb_region(1'b1),
    .pw_valid(pw_v[7]), .pw_buf(pw_buf[7]), .pw_addr(pw_addr[7]), .pw_data(pw_data[7]));

  // ---------------- poly ALU and inversion sub-sequencer
  palu_cmd_t a_cmd;
  logic      a_valid;
  assign a_valid = sp_valid | q_avalid;
  assign a_cmd   = q_avalid ? q_acmd : sp_cmd;
`ifndef SYNTHESIS
  always_ff @(posedge clk) if (rst_i && sp_valid && q_avalid)
    $error("nev_kem_top: sequencer and inversion sub-sequencer issue poly ALU commands together at %t", $time);
`endif
  nev_palu u_palu (
    .clk(clk), .rst_n(rst_i), .qsel(ps_p.qsel), .d64(ps_p.d64), .nlog(ps_p.nlog),
    .cmd_valid(a_valid), .cmd(a_cmd), .slim(ca_pwc), .sblk(ca_pwnx), .busy(p_busy), .done(p_done), .zflag(p_zflag), .nzflag(p_nz), .m_we(p_mwe), .m_wdata(p_mdata), .wm(p_wm),
    .r_v(pr_v[3:0]), .r_buf(pr_buf[3:0]), .r_addr(pr_addr[3:0]), .r_data(pr_data[3:0]),
    .w_v(pw_v[3:0]), .w_buf(pw_buf[3:0]), .w_addr(pw_addr[3:0]), .w_data(pw_data[3:0]));
  nev_pseq u_pseq (
    .clk(clk), .rst_n(rst_i), .klog(klog), .start(q_start), .judge(q_judge), .inj(q_inj),
    .bA(q_bA), .bYA0(q_bYA0), .bCA0(q_bCA0), .bT1(q_bT1), .bT2(q_bT2), .bYCH(q_bYCH), .bCCH(q_bCCH),
    .cval(q_cval), .busy(q_busy), .done(q_done), .noninv(q_noninv),
    .a_valid(q_avalid), .a_cmd(q_acmd), .a_done(p_done), .a_zflag(p_zflag));

  // ---------------- codecs
  nev_codec u_codec_a (
    .clk(clk), .rst_n(rst_i), .qsel(ps_a.qsel), .nlog(ps_a.nlog), .d64(ps_a.d64), .slog(ps_a.slog), .comp(ps_a.comp), .poly_bytes(ps_a.poly_bytes),
    .cmd_valid(ca_valid), .cmd_mode(ca_mode), .cmd_field(ca_field), .cmd_buf(ca_buf),
    .busy(ca_busy), .done(ca_done), .mismatch(ca_mm), .ncanon(ca_nc), .pw_cnt(ca_pwc), .pw_nx(ca_pwnx), .wr_words(ca_ww), .m_we(ca_mwe), .m_wdata(ca_mdata),
    .om_req(om_req[1]), .om_we(om_we[1]), .om_sel(om_sel[1]), .om_addr(om_addr[1]), .om_wdata(om_wdata[1]), .om_be(om_be[1]), .om_wide(om_wide[1]),
    .om_gnt(om_gnt[1]), .om_rvalid(om_rvalid[1]), .om_rdata(om_rdata[1]),
    .pr_valid(pr_v[4]), .pr_buf(pr_buf[4]), .pr_addr(pr_addr[4]), .pr_rdata(pr_data[4]),
    .pr_gnt(pr_gnt[4]), .pr_lim(ca_strm ? p_wm : 8'd128),
    .pw_valid(pw_v[5]), .pw_buf(pw_buf[5]), .pw_addr(pw_addr[5]), .pw_data(pw_data[5]));
  nev_codec u_codec_b (
    .clk(clk), .rst_n(rst_i), .qsel(ps_b.qsel), .nlog(ps_b.nlog), .d64(ps_b.d64), .slog(ps_b.slog), .comp(ps_b.comp), .poly_bytes(ps_b.poly_bytes),
    .cmd_valid(cb_valid), .cmd_mode(cb_mode), .cmd_field(cb_field), .cmd_buf(cb_buf),
    .busy(cb_busy), .done(cb_done), .mismatch(cb_mm), .ncanon(cb_nc), .pw_cnt(cb_pwc), .pw_nx(cb_pwnx), .wr_words(cb_ww), .m_we(cb_mwe), .m_wdata(cb_mdata),
    .om_req(om_req[2]), .om_we(om_we[2]), .om_sel(om_sel[2]), .om_addr(om_addr[2]), .om_wdata(om_wdata[2]), .om_be(om_be[2]), .om_wide(om_wide[2]),
    .om_gnt(om_gnt[2]), .om_rvalid(om_rvalid[2]), .om_rdata(om_rdata[2]),
    .pr_valid(pr_v[5]), .pr_buf(pr_buf[5]), .pr_addr(pr_addr[5]), .pr_rdata(pr_data[5]),
    .pr_gnt(pr_gnt[5]), .pr_lim(cb_strm ? p_wm : 8'd128),
    .pw_valid(pw_v[6]), .pw_buf(pw_buf[6]), .pw_addr(pw_addr[6]), .pw_data(pw_data[6]));
endmodule
