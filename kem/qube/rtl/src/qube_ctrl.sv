// Command controller: accepts a command, latches op/param, and runs the operation as a task
// scoreboard. Each task starts when its dependencies are done and its unit is free, and is marked
// done on the unit's done pulse. task_start / task_done are one-cycle strobes per task (phase probe).
// Holds the seed/secret registers, the random-byte intake, SS and the status outputs.
module qube_ctrl
  import qube_pkg::*;
(
  input  logic          clk,
  input  logic          rst_n,
  // command / status
  input  logic          cmd_valid,
  output logic          cmd_ready,
  input  logic [1:0]    cmd_op,
  input  logic [2:0]    cmd_param,
  output logic          busy,
  output logic          done,
  output logic          err,
  output logic          res_valid,
  input  logic          host_wr,       // host write accepted (clears res_valid)
  // random stream
  input  logic          rnd_valid,
  output logic          rnd_ready,
  input  logic [63:0]   rnd_data,
  // latched parameter set
  output logic [2:0]    ps,
  output logic [1:0]    opc,
  // registers
  output logic [511:0]  r_rnda, r_rho0, r_rho1, r_rhok, r_sigma, r_hpk, r_gm, r_rhoc, r_ss,
  output logic [511:0]  x_seed1,
  // hash unit
  output logic          h_start,
  output logic [1:0]    h_job,
  output logic          h_sk,
  input  logic          h_done,
  input  logic [1023:0] h_out,
  output logic          hpk_ok,        // r_hpk holds H(pk) of this command
  // J hash unit (Dec)
  output logic          j_start,
  input  logic          j_done,
  input  logic [1023:0] j_out,
  // XOF farm
  output logic [1:0]    x_start,
  output logic [1:0]    x_stop,
  output logic [511:0]  x_seed,
  output logic [15:0]   x_b0,
  output logic [15:0]   x_nblk,
  // sampler
  output logic          s_start,
  output logic [1:0]    s_prog,
  input  logic          s_done,
  // lanes
  output logic [1:0]    l_suprst,
  output logic [1:0]    l_start,
  output logic [9:0]    l_abase [0:1],
  output logic [1:0]    l_astep2,
  output logic [8:0]    l_acnt [0:1],
  output logic [9:0]    l_bbase [0:1],
  output logic [1:0]    l_bstep2,
  output logic [8:0]    l_bcnt [0:1],
  output logic [9:0]    l_ibase [0:1],
  output logic [8:0]    l_icnt [0:1],
  output logic [1:0]    l_slotrdy [0:1],
  input  logic [1:0]    l_done,
  // dense loader
  output logic          d_start,
  output logic [5:0]    d_skip,
  output logic [1:0]    d_lanes,
  output logic [1:0]    d_slot,        // slot mask
  input  logic          d_done,
  // mover
  output logic          m_start,
  output logic [2:0]    m_op,
  output logic [1:0]    m_buf,
  output logic [15:0]   m_off,
  output logic [1:0]    m_lanes,
  output logic [1:0]    m_slot,        // slot mask (LOAD)
  input  logic          m_done,
  input  logic          m_eq,
  input  logic          m_extwe,
  input  logic [511:0]  m_ext,
  // codec
  output logic          rse_start,
  input  logic          rse_done,
  output logic          rmd_start,
  input  logic          rmd_done,
  output logic          rsd_start,
  input  logic          rsd_done,
  input  logic [511:0]  rsd_msg,
  // phase probe
  output logic [31:0]   task_start,
  output logic [31:0]   task_done,
  output logic          ss_we          // SS register written this cycle (probe)
);
  // ---------------- tasks
  localparam int T_RND1 = 0, T_RND2 = 1, T_PRG = 2, T_SMP = 3, T_H1 = 4, T_H2 = 5, T_L0 = 6, T_L1 = 7,
                 T_PACK = 8, T_X0 = 9, T_HPK = 10, T_G = 11, T_RSE = 12, T_S1 = 13, T_S2 = 14, T_SEL = 15,
                 T_X1 = 16, T_XK = 17, T_XS = 18, T_U = 19, T_LY0 = 20, T_LY1 = 21, T_EM = 22, T_RSD = 23,
                 T_J = 24, T_SMPR = 25, T_CMP = 26, T_SMPY = 27, T_RMD = 28;
  localparam logic [1:0] OPK = 2'd0, OPE = 2'd1, OPD = 2'd2;

  logic [31:0] td, ts;            // done / started
  logic [31:0] need;              // tasks of the current operation
  logic [31:0] go;                // tasks starting this cycle
  logic        run, fin, errp;
  logic [2:0]  ps_r;
  logic [1:0]  op_r;
  assign ps = ps_r;
  assign opc = op_r;
  logic [511:0] gm_q;
  logic [511:0] r_j;              // J(rho_k, ct, H(pk)) (Dec implicit-rejection key)
  assign r_gm = (op_r == OPE) ? r_rnda : gm_q;
  assign x_seed1 = r_rho0;

  // parameters
  logic [6:0]  kk;
  logic [8:0]  nw, wy, wx, wr, we, wg;
  logic [14:0] nb, nv;
  logic [15:0] pkl;
  always_comb begin
    kk = p_k(ps_r); nw = p_nw(ps_r); wy = p_wy(ps_r); wx = p_wx(ps_r); wr = p_wr(ps_r); we = p_we(ps_r);
    wg = p_wg(ps_r); nb = p_nb(ps_r); nv = p_nv(ps_r); pkl = p_pkl(ps_r);
  end

  // ---------------- unit busy tracking
  logic hash_b, hashj_b, mov_b, dl_b, smp_b, rmd_b;
  logic [1:0] lane_b;
  logic [4:0] h_task, m_task, d_task, s_task;   // task owning the unit
  logic [4:0] l_task [0:1];

  // ---------------- dependency rules
  function automatic logic dn(input logic [31:0] d, input int t);
    dn = d[t];
  endfunction

  logic [31:0] rdy;
  always_comb begin
    rdy = '0;
    case (op_r)
      OPK: begin
        rdy[T_RND1] = 1'b1;
        rdy[T_RND2] = td[T_RND1];
        rdy[T_PRG]  = td[T_RND1] && !hash_b;
        rdy[T_SMP]  = td[T_PRG] && !smp_b;
        rdy[T_H1]   = td[T_PRG] && !dl_b;
        rdy[T_H2]   = td[T_H1] && !dl_b;
        rdy[T_L0]   = ts[T_SMP] && !lane_b[0];
        rdy[T_L1]   = ts[T_SMP] && !lane_b[1];
        rdy[T_PACK] = td[T_L0] && td[T_L1] && td[T_RND2] && !mov_b;
      end
      OPE: begin
        rdy[T_RND1] = 1'b1;
        rdy[T_RND2] = td[T_RND1];
        rdy[T_X0]   = !mov_b;
        rdy[T_HPK]  = !hash_b;
        rdy[T_G]    = td[T_HPK] && td[T_RND1] && td[T_RND2] && !hash_b;
        rdy[T_RSE]  = td[T_RND1];
        rdy[T_H1]   = td[T_X0] && !dl_b;
        rdy[T_H2]   = td[T_H1] && !dl_b;
        rdy[T_S1]   = td[T_X0] && !mov_b;
        rdy[T_S2]   = td[T_S1] && !mov_b;
        rdy[T_SMP]  = td[T_G] && !smp_b;
        rdy[T_L0]   = ts[T_SMP] && !lane_b[0];
        rdy[T_L1]   = ts[T_SMP] && !lane_b[1];
        rdy[T_PACK] = td[T_L0] && td[T_L1] && td[T_RSE] && td[T_S2] && !mov_b;
        rdy[T_SEL]  = td[T_G] && td[T_PACK];
      end
      OPD: begin
        rdy[T_X1]   = !mov_b;
        rdy[T_X0]   = td[T_X1] && !mov_b;
        rdy[T_U]    = td[T_X0] && !mov_b;
        rdy[T_XK]   = td[T_U] && !mov_b;
        rdy[T_XS]   = td[T_XK] && !mov_b;
        rdy[T_HPK]  = !hash_b;
        rdy[T_J]    = td[T_XK] && !hashj_b;       // J unit; its packer waits at H(pk) (hpk_ok)
        rdy[T_SMPY] = td[T_X1] && !smp_b;
        rdy[T_H1]   = td[T_X0] && !dl_b;
        rdy[T_H2]   = td[T_H1] && !dl_b;
        rdy[T_LY1]  = ts[T_SMPY] && !lane_b[1];
        rdy[T_EM]   = td[T_LY1] && !mov_b && !rmd_b;
        rdy[T_RSD]  = td[T_EM] && td[T_RMD];
        rdy[T_S1]   = td[T_EM] && !mov_b;           // lane 1 slots held u until u*y finished
        rdy[T_S2]   = td[T_S1] && !mov_b;
        rdy[T_RSE]  = td[T_RSD];
        rdy[T_G]    = td[T_RSD] && td[T_HPK] && td[T_XS] && !hash_b;
        rdy[T_SMPR] = td[T_G] && td[T_EM] && !smp_b;
        rdy[T_L0]   = ts[T_SMPR] && !lane_b[0];
        rdy[T_L1]   = ts[T_SMPR] && ts[T_S2] && !lane_b[1];
        rdy[T_CMP]  = td[T_L0] && td[T_L1] && td[T_RSE] && td[T_S2] && !mov_b;
        rdy[T_SEL]  = td[T_CMP] && td[T_J];
      end
      default: rdy = '0;
    endcase
  end
  always_comb begin
    case (op_r)
      OPK: need = (32'd1 << T_RND1) | (32'd1 << T_RND2) | (32'd1 << T_PRG) | (32'd1 << T_SMP) | (32'd1 << T_H1) |
                  (32'd1 << T_H2) | (32'd1 << T_L0) | (32'd1 << T_L1) | (32'd1 << T_PACK);
      OPE: need = (32'd1 << T_RND1) | (32'd1 << T_RND2) | (32'd1 << T_X0) | (32'd1 << T_HPK) | (32'd1 << T_G) |
                  (32'd1 << T_RSE) | (32'd1 << T_H1) | (32'd1 << T_H2) | (32'd1 << T_S1) | (32'd1 << T_S2) |
                  (32'd1 << T_SMP) | (32'd1 << T_L0) | (32'd1 << T_L1) | (32'd1 << T_PACK) | (32'd1 << T_SEL);
      default: need = (32'd1 << T_X1) | (32'd1 << T_X0) | (32'd1 << T_U) | (32'd1 << T_XK) | (32'd1 << T_XS) |
                  (32'd1 << T_S1) | (32'd1 << T_HPK) | (32'd1 << T_SMPY) | (32'd1 << T_H1) |
                  (32'd1 << T_LY1) | (32'd1 << T_EM) | (32'd1 << T_RMD) | (32'd1 << T_RSD) | (32'd1 << T_H2) |
                  (32'd1 << T_S2) | (32'd1 << T_RSE) | (32'd1 << T_G) | (32'd1 << T_J) | (32'd1 << T_SMPR) |
                  (32'd1 << T_L0) | (32'd1 << T_L1) | (32'd1 << T_CMP) | (32'd1 << T_SEL);
    endcase
  end

  // one start per unit per cycle: hash, mover, dense loader, sampler (fixed priority by task order)
  function automatic logic [31:0] pick(input logic [31:0] cand, input logic [31:0] group);
    logic [31:0] c; logic taken;
    c = cand & group; taken = 1'b0; pick = '0;
    for (int i = 0; i < 32; i++) if (c[i] && !taken) begin pick[i] = 1'b1; taken = 1'b1; end
  endfunction
  localparam logic [31:0] G_HASH = (32'd1 << T_PRG) | (32'd1 << T_HPK) | (32'd1 << T_G);
  localparam logic [31:0] G_MOV  = (32'd1 << T_PACK) | (32'd1 << T_X0) | (32'd1 << T_S1) | (32'd1 << T_S2) |
                                   (32'd1 << T_X1) | (32'd1 << T_XK) | (32'd1 << T_XS) | (32'd1 << T_U) |
                                   (32'd1 << T_EM) | (32'd1 << T_CMP);
  localparam logic [31:0] G_DL   = (32'd1 << T_H1) | (32'd1 << T_H2);
  localparam logic [31:0] G_SMP  = (32'd1 << T_SMP) | (32'd1 << T_SMPY) | (32'd1 << T_SMPR);
  logic [31:0] cand;
  always_comb begin
    cand = run ? (rdy & need & ~ts) : '0;
    go = (cand & ~(G_HASH | G_MOV | G_DL | G_SMP)) | pick(cand, G_HASH) | pick(cand, G_MOV) |
         pick(cand, G_DL) | pick(cand, G_SMP);
    // RND tasks complete by beats; RMD starts together with EM
    go[T_RND1] = 1'b0; go[T_RND2] = 1'b0; go[T_RMD] = 1'b0; go[T_SEL] = cand[T_SEL];
  end
  assign task_start = go;
  assign ss_we = run && ((h_done && h_task == 5'(T_G)) || (go[T_SEL] && op_r == OPD && !m_eq));
  assign hpk_ok = td[T_HPK];
  assign j_start = go[T_J];

  // ---------------- random intake
  logic [3:0] beat;          // beats of the current group
  logic       grp2;
  assign rnd_ready = run && (op_r == OPK || op_r == OPE) && !td[T_RND2] && (beat < kk[6:3]);

  // ---------------- unit starts and arguments
  always_comb begin
    h_start = |(go & G_HASH);
    h_job = go[T_PRG] ? 2'd0 : go[T_HPK] ? 2'd1 : 2'd2;
    h_sk = (op_r == OPD);
    // farm / loader
    x_start = 2'b00; x_seed = (op_r == OPK || go[T_SMPY]) ? r_rho1 : r_rhoc; x_b0 = 16'd0; x_nblk = 16'd0;
    if (go[T_SMP] || go[T_SMPR] || go[T_SMPY]) x_start[0] = 1'b1;
    d_start = go[T_H1] || go[T_H2];
    d_skip = go[T_H2] ? {1'b0, nb[4:0]} : 6'd0;
    if (go[T_H1] || go[T_H2]) begin
      x_start[1] = 1'b1;
      x_b0 = go[T_H2] ? {6'd0, nb[14:5]} : 16'd0;
      x_nblk = {6'd0, nw, 1'b0} + ((go[T_H2] && nb[4:0] != 5'd0) ? 16'd2 : 16'd0);
    end
    s_start = |(go & G_SMP);
    s_prog = (op_r == OPK) ? 2'd0 : go[T_SMPY] ? 2'd2 : 2'd1;
    // dense loader targets
    // KeyGen: h1 -> both slots of lane 0, h2 -> both slots of lane 1 (self-paired y passes);
    // Enc / Dec: h1 -> lane 0 slot 0, h2 -> lane 0 slot 1
    d_lanes = 2'b01; d_slot = 2'b01;
    case (op_r)
      OPK: begin d_lanes = go[T_H2] ? 2'b10 : 2'b01; d_slot = 2'b11; end
      default: begin d_lanes = 2'b01; d_slot = go[T_H2] ? 2'b10 : 2'b01; end
    endcase
    // mover
    m_start = |(go & G_MOV);
    m_op = 3'd0; m_buf = 2'd0; m_off = 16'd0; m_lanes = 2'b10; m_slot = 2'b01;
    if (go[T_PACK]) m_op = (op_r == OPK) ? 3'd4 : 3'd5;
    if (go[T_X0]) begin m_op = 3'd1; m_buf = (op_r == OPD) ? 2'd1 : 2'd0; m_off = (op_r == OPD) ? {9'd0, kk} : 16'd0; end
    if (go[T_X1]) begin m_op = 3'd1; m_buf = 2'd1; m_off = 16'd0; end
    if (go[T_XK]) begin m_op = 3'd1; m_buf = 2'd1; m_off = {9'd0, kk} + pkl; end
    if (go[T_XS]) begin m_op = 3'd1; m_buf = 2'd2; m_off = {1'b0, nb} + {1'b0, nv}; end
    // Dec: u -> both slots of lane 1 (self-paired u*y passes); s1 / s2 -> lane 1 slot 0 / 1
    if (go[T_U])  begin m_op = 3'd0; m_buf = 2'd2; m_off = 16'd0; m_lanes = 2'b10; m_slot = 2'b11; end
    if (go[T_S1]) begin m_op = 3'd0; m_buf = (op_r == OPD) ? 2'd1 : 2'd0;
                        m_off = (op_r == OPD) ? {8'd0, kk, 1'b0} : {9'd0, kk};
                        m_lanes = 2'b10; m_slot = 2'b01; end
    if (go[T_S2]) begin m_op = 3'd0; m_buf = (op_r == OPD) ? 2'd1 : 2'd0;
                        m_off = ((op_r == OPD) ? {8'd0, kk, 1'b0} : {9'd0, kk}) + {1'b0, nb};
                        m_lanes = 2'b10; m_slot = 2'b10; end
    if (go[T_EM])  m_op = 3'd2;
    if (go[T_CMP]) m_op = 3'd3;
    rse_start = go[T_RSE];
    rmd_start = go[T_EM];
    rsd_start = go[T_RSD];
    // lanes
    l_start = {go[T_L1] || go[T_LY1], go[T_L0]};
    l_suprst = {2{go[T_SMP] || go[T_SMPY] || go[T_SMPR]}};
    // lane programs (SUP layout = sampler write order of the lane):
    //   self-paired y (KeyGen y*h, Dec u*y): A = y[0], y[2], ..; B = y[1], y[3], ..; I = x1 / x2 after y
    //   cross-paired (Enc, re-encryption): A = f1 (slot 0), B = f2 (slot 1), I = g (lane 0) / e (lane 1)
    for (int l = 0; l < 2; l++) begin
      l_abase[l] = 10'd0; l_bbase[l] = 10'd1; l_astep2[l] = 1'b1; l_bstep2[l] = 1'b1;
      l_acnt[l] = (wy + 9'd1) >> 1; l_bcnt[l] = wy >> 1; l_ibase[l] = {1'b0, wy}; l_icnt[l] = wx;
      if (op_r == OPE || (op_r == OPD && !go[T_LY1])) begin
        l_abase[l] = 10'd0; l_bbase[l] = {1'b0, wr}; l_astep2[l] = 1'b0; l_bstep2[l] = 1'b0;
        l_acnt[l] = wr; l_bcnt[l] = wr; l_ibase[l] = {wr, 1'b0}; l_icnt[l] = (l == 0) ? wg : we;
      end else if (op_r == OPD) l_icnt[l] = 9'd0;
    end
  end

  // ---------------- slot ready flags
  logic [1:0] srdy [0:1];
  assign l_slotrdy = srdy;
  logic [1:0] d_tgt_l, d_tgt_s, m_tgt_l, m_tgt_s;

  // ---------------- status
  assign cmd_ready = !run && rst_n;
  assign busy = run;
  assign done = fin;
  assign err = fin && errp;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run <= 1'b0; fin <= 1'b0; errp <= 1'b0; res_valid <= 1'b0; ps_r <= '0; op_r <= '0;
      td <= '0; ts <= '0; beat <= '0; grp2 <= 1'b0;
      r_rnda <= '0; r_rho0 <= '0; r_rho1 <= '0; r_rhok <= '0; r_sigma <= '0; r_hpk <= '0; gm_q <= '0;
      r_rhoc <= '0; r_ss <= '0; r_j <= '0;
      hash_b <= 1'b0; hashj_b <= 1'b0; mov_b <= 1'b0; dl_b <= 1'b0; smp_b <= 1'b0; rmd_b <= 1'b0; lane_b <= '0;
      h_task <= '0; m_task <= '0; d_task <= '0; s_task <= '0; l_task[0] <= '0; l_task[1] <= '0;
      srdy[0] <= '0; srdy[1] <= '0; d_tgt_l <= '0; d_tgt_s <= '0; m_tgt_l <= '0; m_tgt_s <= '0;
      x_stop <= '0; task_done <= '0;
    end else begin
      fin <= 1'b0; x_stop <= '0; task_done <= '0;
      if (host_wr) res_valid <= 1'b0;
      if (!run && cmd_valid) begin
        // accept
        res_valid <= 1'b0;
        td <= '0; ts <= '0;              // also on a reserved code: hpk_ok (td[T_HPK]) falls at every accept
        if (cmd_op == 2'd3 || cmd_param > 3'd4) begin
          fin <= 1'b1; errp <= 1'b1;
        end else begin
          run <= 1'b1; errp <= 1'b0; ps_r <= cmd_param; op_r <= cmd_op;
          td <= '0; ts <= '0; beat <= '0; grp2 <= 1'b0;
          srdy[0] <= '0; srdy[1] <= '0;
          if (cmd_op != 2'd2) begin r_rnda <= '0; r_rhok <= '0; r_sigma <= '0; end
          hash_b <= 1'b0; hashj_b <= 1'b0; mov_b <= 1'b0; dl_b <= 1'b0; smp_b <= 1'b0; rmd_b <= 1'b0; lane_b <= '0;
        end
      end else if (run) begin
        ts <= ts | go;
        // ---- random intake (registers cleared at accept)
        if (rnd_valid && rnd_ready) begin
          for (int b = 0; b < 8; b++) begin
            if (!grp2) r_rnda[64 * beat + 8 * b +: 8] <= rnd_data[8 * b +: 8];
            else if (op_r == OPK) r_rhok[64 * beat + 8 * b +: 8] <= rnd_data[8 * b +: 8];
            else r_sigma[64 * beat + 8 * b +: 8] <= rnd_data[8 * b +: 8];
          end
          if (beat == kk[6:3] - 4'd1) begin
            beat <= '0;
            if (!grp2) begin grp2 <= 1'b1; td[T_RND1] <= 1'b1; task_done[T_RND1] <= 1'b1; end
            else begin td[T_RND2] <= 1'b1; task_done[T_RND2] <= 1'b1; end
          end else beat <= beat + 4'd1;
        end
        // ---- unit busy bookkeeping
        if (|(go & G_HASH)) hash_b <= 1'b1;
        if (|(go & G_MOV))  mov_b <= 1'b1;
        if (|(go & G_DL))   dl_b <= 1'b1;
        if (|(go & G_SMP))  smp_b <= 1'b1;
        if (go[T_EM])       rmd_b <= 1'b1;
        for (int t = 0; t < 32; t++) begin
          if (go[t] && G_HASH[t]) h_task <= 5'(t);
          if (go[t] && G_MOV[t])  m_task <= 5'(t);
          if (go[t] && G_DL[t])   d_task <= 5'(t);
          if (go[t] && G_SMP[t])  s_task <= 5'(t);
        end
        if (go[T_J])        hashj_b <= 1'b1;
        if (go[T_L0]) begin lane_b[0] <= 1'b1; l_task[0] <= 5'(T_L0); end
        if (go[T_L1] || go[T_LY1]) begin lane_b[1] <= 1'b1; l_task[1] <= go[T_L1] ? 5'(T_L1) : 5'(T_LY1); end
        // dense loader target bookkeeping (slot not ready while it is being written)
        if (d_start) begin d_tgt_l <= d_lanes; d_tgt_s <= d_slot;
          for (int l = 0; l < 2; l++) for (int x = 0; x < 2; x++) if (d_lanes[l] && d_slot[x]) srdy[l][x] <= 1'b0; end
        if (m_start && m_op == 3'd0) begin m_tgt_l <= m_lanes; m_tgt_s <= m_slot;
          for (int l = 0; l < 2; l++) for (int x = 0; x < 2; x++) if (m_lanes[l] && m_slot[x]) srdy[l][x] <= 1'b0; end
        // ---- completions
        if (h_done) begin
          hash_b <= 1'b0; td[h_task] <= 1'b1; task_done[h_task] <= 1'b1;
          case (h_task)
            5'(T_PRG): begin
              r_rho0 <= '0; r_rho1 <= '0;
              for (int b = 0; b < 64; b++) begin
                if (7'(b) < kk) r_rho0[8 * b +: 8] <= h_out[8 * b +: 8];
                if (7'(b) < kk) r_rho1[8 * b +: 8] <= h_out[8 * (b + 64) - 8 * (64 - kk) +: 8];
              end
            end
            5'(T_HPK): begin r_hpk <= '0; for (int b = 0; b < 64; b++) if (7'(b) < kk) r_hpk[8 * b +: 8] <= h_out[8 * b +: 8]; end
            5'(T_G): begin
              r_ss <= '0; r_rhoc <= '0;
              for (int b = 0; b < 64; b++) if (7'(b) < kk) begin
                r_ss[8 * b +: 8] <= h_out[8 * b +: 8];
                r_rhoc[8 * b +: 8] <= h_out[8 * (b + 64) - 8 * (64 - kk) +: 8];
              end
            end
            default: ;
          endcase
        end
        if (j_done) begin
          hashj_b <= 1'b0; td[T_J] <= 1'b1; task_done[T_J] <= 1'b1;
          r_j <= '0; for (int b = 0; b < 64; b++) if (7'(b) < kk) r_j[8 * b +: 8] <= j_out[8 * b +: 8];
        end
        if (m_done) begin
          mov_b <= 1'b0; td[m_task] <= 1'b1; task_done[m_task] <= 1'b1;
          if (m_task == 5'(T_U) || m_task == 5'(T_S1) || m_task == 5'(T_S2))
            for (int l = 0; l < 2; l++) for (int x = 0; x < 2; x++) if (m_tgt_l[l] && m_tgt_s[x]) srdy[l][x] <= 1'b1;
        end
        if (m_extwe) begin
          case (m_task)
            5'(T_X0): r_rho0 <= m_ext;
            5'(T_X1): r_rho1 <= m_ext;
            5'(T_XK): r_rhok <= m_ext;
            default:  r_sigma <= m_ext;
          endcase
        end
        if (d_done) begin
          dl_b <= 1'b0; td[d_task] <= 1'b1; task_done[d_task] <= 1'b1; x_stop[1] <= 1'b1;
          for (int l = 0; l < 2; l++) for (int x = 0; x < 2; x++) if (d_tgt_l[l] && d_tgt_s[x]) srdy[l][x] <= 1'b1;
        end
        if (s_done) begin smp_b <= 1'b0; td[s_task] <= 1'b1; task_done[s_task] <= 1'b1; x_stop[0] <= 1'b1; end
        for (int l = 0; l < 2; l++) if (l_done[l]) begin
          lane_b[l] <= 1'b0; td[l_task[l]] <= 1'b1; task_done[l_task[l]] <= 1'b1;
        end
        if (rmd_done) begin rmd_b <= 1'b0; td[T_RMD] <= 1'b1; task_done[T_RMD] <= 1'b1; end
        if (rsd_done) begin
          td[T_RSD] <= 1'b1; task_done[T_RSD] <= 1'b1;
          gm_q <= rsd_msg;
        end
        if (rse_done) begin td[T_RSE] <= 1'b1; task_done[T_RSE] <= 1'b1; end
        if (go[T_SEL]) begin
          td[T_SEL] <= 1'b1; task_done[T_SEL] <= 1'b1;
          // Enc: ss = G output; Dec: G output if the re-encryption matched, else J
          if (op_r == OPD && !m_eq) r_ss <= r_j;
        end
        // ---- operation complete
        if ((td & need) == need && ((op_r != OPK) || !mov_b)) begin
          run <= 1'b0; fin <= 1'b1; res_valid <= 1'b1;
        end
      end
    end
  end
endmodule
