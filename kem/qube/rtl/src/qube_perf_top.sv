// QUBE-SM3 KEM accelerator, performance version: KeyGen / Enc / Dec for QUBE-128/192/256/384/512.
module qube_perf_top
  import qube_pkg::*;
(
  input  logic         clk,
  input  logic         rst_n,
  input  logic         cmd_valid,
  output logic         cmd_ready,
  input  logic [1:0]   cmd_op,
  input  logic [2:0]   cmd_param,
  output logic         busy,
  output logic         done,
  output logic         err,
  output logic         res_valid,
  input  logic         rnd_valid,
  output logic         rnd_ready,
  input  logic [63:0]  rnd_data,
  input  logic         host_en,
  input  logic         host_we,
  input  logic [1:0]   host_sel,
  input  logic [12:0]  host_addr,
  input  logic [63:0]  host_wdata,
  output logic [63:0]  host_rdata,
  output logic         host_rvalid
);
  logic rsn;
  qube_rst_sync u_rst (.clk(clk), .rst_n(rst_n), .rst_sync_n(rsn));

  // ---------------- controller
  logic [2:0]  ps;
  logic [1:0]  opc;
  logic [511:0] r_rnda, r_rho0, r_rho1, r_rhok, r_sigma, r_hpk, r_gm, r_rhoc, r_ss, x_seed1;
  logic        h_start, h_sk, h_done;
  logic [1:0]  h_job;
  logic [1023:0] h_out;
  logic [1:0]  x_start, x_stop;
  logic [511:0] x_seed;
  logic [15:0] x_b0, x_nblk;
  logic        s_start, s_done;
  logic [1:0]  s_prog;
  logic [1:0]  l_suprst, l_start, l_done;
  logic [9:0]  l_abase [0:1], l_bbase [0:1], l_ibase [0:1];
  logic [1:0]  l_astep2, l_bstep2;
  logic [8:0]  l_acnt [0:1], l_bcnt [0:1], l_icnt [0:1];
  logic        hpk_ok, j_start, j_done;
  logic [1023:0] j_out;
  logic [1:0]  l_slotrdy [0:1];
  logic        d_start, d_done;
  logic [1:0]  d_slot;
  logic [5:0]  d_skip;
  logic [1:0]  d_lanes;
  logic        m_start, m_done, m_eq, m_extwe;
  logic [1:0]  m_slot;
  logic [2:0]  m_op;
  logic [1:0]  m_buf, m_lanes;
  logic [15:0] m_off;
  logic [511:0] m_ext;
  logic        rse_start, rse_done, rmd_start, rmd_done, rsd_start, rsd_done;
  logic [511:0] rsd_msg;
  logic [31:0] task_start, task_done;   // phase probe: one-cycle task start / done strobes (u_ctrl)
  logic        ss_we;                    // probe: SS register write strobe
  logic [2:0]  obuf_we;                  // probe: engine writes to the PK / SK / CT buffer macros
  logic        host_wr;

  assign host_wr = host_en && host_we && !busy;

  qube_ctrl u_ctrl (
    .clk(clk), .rst_n(rsn), .cmd_valid(cmd_valid), .cmd_ready(cmd_ready), .cmd_op(cmd_op), .cmd_param(cmd_param),
    .busy(busy), .done(done), .err(err), .res_valid(res_valid), .host_wr(host_wr),
    .rnd_valid(rnd_valid), .rnd_ready(rnd_ready), .rnd_data(rnd_data), .ps(ps), .opc(opc),
    .r_rnda(r_rnda), .r_rho0(r_rho0), .r_rho1(r_rho1), .r_rhok(r_rhok), .r_sigma(r_sigma), .r_hpk(r_hpk),
    .r_gm(r_gm), .r_rhoc(r_rhoc), .r_ss(r_ss), .x_seed1(x_seed1),
    .h_start(h_start), .h_job(h_job), .h_sk(h_sk), .h_done(h_done), .h_out(h_out), .hpk_ok(hpk_ok),
    .j_start(j_start), .j_done(j_done), .j_out(j_out),
    .x_start(x_start), .x_stop(x_stop), .x_seed(x_seed), .x_b0(x_b0), .x_nblk(x_nblk),
    .s_start(s_start), .s_prog(s_prog), .s_done(s_done),
    .l_suprst(l_suprst), .l_start(l_start), .l_abase(l_abase), .l_astep2(l_astep2), .l_acnt(l_acnt),
    .l_bbase(l_bbase), .l_bstep2(l_bstep2), .l_bcnt(l_bcnt), .l_ibase(l_ibase), .l_icnt(l_icnt),
    .l_slotrdy(l_slotrdy), .l_done(l_done),
    .d_start(d_start), .d_skip(d_skip), .d_lanes(d_lanes), .d_slot(d_slot), .d_done(d_done),
    .m_start(m_start), .m_op(m_op), .m_buf(m_buf), .m_off(m_off), .m_lanes(m_lanes), .m_slot(m_slot),
    .m_done(m_done), .m_eq(m_eq), .m_extwe(m_extwe), .m_ext(m_ext),
    .rse_start(rse_start), .rse_done(rse_done), .rmd_start(rmd_start), .rmd_done(rmd_done),
    .rsd_start(rsd_start), .rsd_done(rsd_done), .rsd_msg(rsd_msg),
    .task_start(task_start), .task_done(task_done), .ss_we(ss_we));

  // ---------------- object buffers (0 PK, 1 SK, 2 CT)
  logic [511:0] ob_q [0:2];
  logic [2:0]   ob_lane [0:2];
  logic [2:0]   ob_wrq, ob_wgnt;
  logic [9:0]   ob_waddr [0:2];
  logic [511:0] ob_wdata [0:2];
  logic [4:0]   ob_rrq [0:2];
  logic [9:0]   ob_raddr [0:2][0:4];
  logic [4:0]   ob_rgnt [0:2];
  logic [4:0]   ob_rv [0:2];

  // hash engine memory ports: 0/1 hash unit E0/E1, 2/3 J-hash unit E0/E1
  logic [3:0]   hm_rq, hm_gnt, hm_rv;
  logic [1:0]   hm_buf [0:3];
  logic [9:0]   hm_addr [0:3];
  logic [511:0] hm_data [0:3];
  // mover memory ports
  logic [2:0]   mv_rq, mv_gnt, mv_rv, mv_wrq;
  logic [9:0]   mv_raddr;
  logic [511:0] mv_rdata [0:2];
  logic [9:0]   mv_waddr [0:2];
  logic [511:0] mv_wdata [0:2];

  genvar gb;
  generate for (gb = 0; gb < 3; gb++) begin : g_ob
    assign ob_rrq[gb] = {hm_rq[3] && hm_buf[3] == 2'(gb), hm_rq[2] && hm_buf[2] == 2'(gb),
                         hm_rq[1] && hm_buf[1] == 2'(gb), hm_rq[0] && hm_buf[0] == 2'(gb), mv_rq[gb]};
    assign ob_raddr[gb][0] = mv_raddr;
    for (genvar ge = 0; ge < 4; ge++) begin : g_ra
      assign ob_raddr[gb][ge + 1] = hm_addr[ge];
    end
    qube_objbuf u_ob (.clk(clk), .rst_n(rsn), .eng(busy),
      .h_en(host_en && host_sel == 2'(gb) && !busy), .h_we(host_we), .h_addr(host_addr), .h_wdata(host_wdata),
      .rd_q(ob_q[gb]), .h_lane(ob_lane[gb]),
      .w_rq(mv_wrq[gb]), .w_addr(mv_waddr[gb]), .w_data(mv_wdata[gb]), .w_gnt(ob_wgnt[gb]),
      .r_rq(ob_rrq[gb]), .r_addr(ob_raddr[gb]), .r_gnt(ob_rgnt[gb]), .rv(ob_rv[gb]));
    assign mv_gnt[gb] = ob_rgnt[gb][0];
    assign obuf_we[gb] = ob_wgnt[gb];
    assign mv_rv[gb] = ob_rv[gb][0];
    assign mv_rdata[gb] = ob_q[gb];
  end endgenerate
  always_comb for (int e = 0; e < 4; e++) begin
    hm_gnt[e]  = ob_rgnt[hm_buf[e]][e + 1];
    hm_rv[e]   = ob_rv[hm_buf[e]][e + 1];
    hm_data[e] = ob_q[hm_buf[e]];
  end

  // host read data path (latency 2 from the request edge)
  logic       hr1, hr2;
  logic [1:0] hs1, hs2;
  logic [2:0] ssl1, ssl2;
  always_ff @(posedge clk or negedge rsn) begin
    if (!rsn) begin hr1 <= 1'b0; hr2 <= 1'b0; host_rvalid <= 1'b0; hs1 <= '0; hs2 <= '0; ssl1 <= '0; ssl2 <= '0; end
    else begin
      hr1 <= host_en && !host_we && !busy; hs1 <= host_sel; ssl1 <= host_addr[2:0];
      hr2 <= hr1; hs2 <= hs1; ssl2 <= ssl1;
      host_rvalid <= hr2;
    end
  end
  logic [1:0] hs3; logic [63:0] ss3;
  always_ff @(posedge clk or negedge rsn) begin
    if (!rsn) begin hs3 <= '0; ss3 <= '0; end
    else begin hs3 <= hs2; ss3 <= r_ss[64 * ssl2 +: 64]; end
  end
  always_comb begin
    case (hs3)
      2'd0: host_rdata = ob_q[0][64 * ob_lane[0] +: 64];
      2'd1: host_rdata = ob_q[1][64 * ob_lane[1] +: 64];
      2'd2: host_rdata = ob_q[2][64 * ob_lane[2] +: 64];
      default: host_rdata = ss3;
    endcase
  end

  // ---------------- hash
  logic hash_busy, hashj_busy;
  qube_hash u_hash (.clk(clk), .rst_n(rsn), .start(h_start), .job(h_job), .ps(ps), .hsrc_sk(h_sk),
    .r_rnda(r_rnda), .r_gm(r_gm), .r_gs(r_sigma), .r_hpk(r_hpk), .r_rhok(r_rhok), .hpk_ok(1'b1),
    .busy(hash_busy), .done(h_done), .out(h_out),
    .mrq(hm_rq[1:0]), .mbuf(hm_buf[0:1]), .maddr(hm_addr[0:1]), .mgnt(hm_gnt[1:0]), .mrv(hm_rv[1:0]),
    .mrdata(hm_data[0:1]));
  // J(rho_k, ct, H(pk)) of Dec on its own unit: started when rho_k is read, its packers process
  // rho_k || ct concurrently with H(pk) and wait at the H(pk) segment until H(pk) is known
  qube_hash u_hashj (.clk(clk), .rst_n(rsn), .start(j_start), .job(2'd3), .ps(ps), .hsrc_sk(1'b1),
    .r_rnda(r_rnda), .r_gm(r_gm), .r_gs(r_sigma), .r_hpk(r_hpk), .r_rhok(r_rhok), .hpk_ok(hpk_ok),
    .busy(hashj_busy), .done(j_done), .out(j_out),
    .mrq(hm_rq[3:2]), .mbuf(hm_buf[2:3]), .maddr(hm_addr[2:3]), .mgnt(hm_gnt[3:2]), .mrv(hm_rv[3:2]),
    .mrdata(hm_data[2:3]));

  // coverage probe (CH-PAR (a)): the J-hash unit runs while the hash unit computes H(pk)
  // (hash unit busy before H(pk) of this command is done)
  logic cov_j_par;
  assign cov_j_par = hashj_busy && hash_busy && !hpk_ok;

  // ---------------- XOF farm, sampler, dense loader
  logic [1:0]   xf_v, xf_r;
  logic [255:0] xf_d [0:1];
  qube_xof_farm u_farm (.clk(clk), .rst_n(rsn), .ctx_start(x_start), .ctx_stop(x_stop), .seed0(x_seed),
    .seed1(x_seed1), .ps(ps), .b0(x_b0), .nblk(x_nblk), .ovalid(xf_v), .oready(xf_r), .odata(xf_d));
  logic [1:0]  sup_we;
  logic [17:0] sup_wd;
  qube_sampler u_smp (.clk(clk), .rst_n(rsn), .start(s_start), .ps(ps), .prog(s_prog), .busy(), .done(s_done),
    .svalid(xf_v[0]), .sready(xf_r[0]), .sdata(xf_d[0]), .sup_we(sup_we), .sup_wdata(sup_wd));
  logic [1:0]   dl_en;
  logic [1:0]   dl_slot;
  logic [8:0]   dl_addr;
  logic [511:0] dl_data;
  qube_hload u_hload (.clk(clk), .rst_n(rsn), .start(d_start), .ps(ps), .skip(d_skip), .lanes(d_lanes),
    .slot(d_slot), .busy(), .done(d_done), .ivalid(xf_v[1]), .iready(xf_r[1]), .idata(xf_d[1]),
    .sw_en(dl_en), .sw_slot(dl_slot), .sw_addr(dl_addr), .sw_data(dl_data));

  // ---------------- mover
  logic [7:0]   cdw [0:N1MAX-1];
  logic [1:0]   mv_aren;
  logic [8:0]   mv_araddr;
  logic [511:0] ar_q [0:1];
  logic [1:0]   mv_swen;
  logic [1:0]   mv_swslot;
  logic [8:0]   mv_swaddr;
  logic [511:0] mv_swdata;
  logic         em_v, em_r;
  logic [511:0] em_d;
  qube_mover u_mov (.clk(clk), .rst_n(rsn), .ps(ps), .start(m_start), .op(m_op), .sbuf(m_buf), .soff(m_off),
    .dlanes(m_lanes), .dslot(m_slot), .busy(), .done(m_done), .cmp_eq(m_eq), .ext_we(m_extwe), .ext_data(m_ext),
    .r_rho0(r_rho0), .r_rho1(r_rho1), .r_rhok(r_rhok), .r_sigma(r_sigma), .cdw(cdw),
    .r_rq(mv_rq), .r_addr(mv_raddr), .r_gnt(mv_gnt), .r_rv(mv_rv), .r_data(mv_rdata),
    .w_rq(mv_wrq), .w_addr(mv_waddr), .w_data(mv_wdata),
    .ar_en(mv_aren), .ar_addr(mv_araddr), .ar_data(ar_q),
    .sw_en(mv_swen), .sw_slot(mv_swslot), .sw_addr(mv_swaddr), .sw_data(mv_swdata),
    .em_valid(em_v), .em_ready(em_r), .em_data(em_d));

  // ---------------- lanes
  genvar gl;
  generate for (gl = 0; gl < 2; gl++) begin : g_lane
    logic [1:0]   swen;
    logic [8:0]   swa [0:1];
    logic [511:0] swd [0:1];
    always_comb for (int x = 0; x < 2; x++) begin
      if (dl_en[gl] && dl_slot[x]) begin swen[x] = 1'b1; swa[x] = dl_addr; swd[x] = dl_data; end
      else begin swen[x] = mv_swen[gl] && mv_swslot[x]; swa[x] = mv_swaddr; swd[x] = mv_swdata; end
    end
    qube_lane u_lane (.clk(clk), .rst_n(rsn), .ps(ps), .start(l_start[gl]),
      .a_base(l_abase[gl]), .a_step2(l_astep2[gl]), .a_cnt(l_acnt[gl]), .b_base(l_bbase[gl]),
      .b_step2(l_bstep2[gl]), .b_cnt(l_bcnt[gl]), .i_base(l_ibase[gl]), .i_cnt(l_icnt[gl]), .slot_rdy(l_slotrdy[gl]),
      .busy(), .done(l_done[gl]), .sup_rst(l_suprst[gl]), .sup_we(sup_we[gl]), .sup_wdata(sup_wd),
      .sw_en(swen), .sw_addr(swa), .sw_data(swd), .ar_en(mv_aren[gl]), .ar_addr(mv_araddr), .ar_data(ar_q[gl]));
  end endgenerate

  // ---------------- codec
  qube_rsenc u_rse (.clk(clk), .rst_n(rsn), .start(rse_start), .ps(ps), .msg(r_gm), .busy(), .done(rse_done), .cdw(cdw));
  logic        sym_we;
  logic [7:0]  sym_idx, sym_data;
  qube_rmdec u_rmd (.clk(clk), .rst_n(rsn), .start(rmd_start), .ps(ps), .in_valid(em_v), .in_ready(em_r),
    .in_data(em_d), .busy(), .done(rmd_done), .sym_we(sym_we), .sym_idx(sym_idx), .sym_data(sym_data));
  qube_rsdec u_rsd (.clk(clk), .rst_n(rsn), .start(rsd_start), .ps(ps), .sym_we(sym_we), .sym_idx(sym_idx),
    .sym_data(sym_data), .busy(), .done(rsd_done), .msg(rsd_msg));
endmodule
