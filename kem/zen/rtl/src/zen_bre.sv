// zen_bre: base-ring SIMD microcoded engine.
//   kind 0 = basemul (bm), 1 = basemul_mq (bmq), 2 = baseinv (inv); param 0/1/2 = ZEN_128/256/512.
//   Result = baseop(p, kind, a, b) over 128 blocks of m = 4 << param coefficients.
//
// Lane organisation: every lane holds ONE polynomial word (16 coefficients) per operand, i.e.
// k = 16/m base-ring blocks (multi-block programs: ZEN_128 k=4, ZEN_256 k=2, ZEN_512 k=1).
// Batch b (16 lanes) covers polynomial words 16b .. 16b+15 (2/4/8 batches for ZEN_128/256/512).
// ZEN_512 baseinv runs as two passes: inv1 (A -> C, 8 intermediate coefficients g per block) and
// inv2 (A, B = g -> C); the controller maps B to the C slot.
//
// Per batch:
//   LOAD  : 8 cycles, read word pair j = (16b+2j, 16b+2j+1) of A and B (both banks) -> lanes 2j, 2j+1
//           (a -> r0..r15, b -> r16..r31); zetas of the lane's k blocks -> r32..r35. The loads of batch
//           b+1 start when the program of batch b executes PC ld_pc (no input-window access from then on,
//           guaranteed by the microcode compiler).
//   EXEC  : microprogram (zen_bre_rom), 1 microinstruction per cycle; ROM -> NIR (+ per-lane read
//           selects, immediates) | operand fetch with forwarding | execute (IR) and RF write.
//           The first instruction of batch b+1 follows the last of batch b without a gap when its loads
//           are complete. An instruction holding a QINV (ALU slot 0) first stalls 5 cycles while 4 shared
//           qinv ROM copies serve the 16 lanes, 4 lanes per cycle.
//   WRITE : 8 cycles after the last instruction, word pair j from the lanes' output buffers (overlapped
//           with the next batch; the generator asserts no output capture before instruction WBCYC-1 = 7; all current programs capture first at >= 9).
// Each batch reads and writes only its own words, so C may map to the same slot as A or B.
// wr_cnt: number of polynomial words of the current pass already committed by the write port, in order.
module zen_bre (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              start,
  input  logic [1:0]        param,
  input  logic [1:0]        kind,
  output logic              done,
  output logic              active,
  output logic [7:0]        wr_cnt,
  output logic              wb_last,       // pulse: write-back of the final batch begins next cycle
  // read master A
  output logic [1:0]        ra_rd_en,
  output logic [1:0][5:0]   ra_rd_addr,
  input  logic [1:0][255:0] ra_rd_data,
  // read master B
  output logic [1:0]        rb_rd_en,
  output logic [1:0][5:0]   rb_rd_addr,
  input  logic [1:0][255:0] rb_rd_data,
  // write master C
  output logic [1:0]        wc_wr_en,
  output logic [1:0][5:0]   wc_wr_addr,
  output logic [1:0][255:0] wc_wr_data
);

  localparam int IW = 180;
  localparam logic [2:0] S_IDLE = 3'd0, S_LOAD = 3'd1, S_FETCH = 3'd2, S_EXEC = 3'd3, S_FIN = 3'd4;

  // lane configuration: must match the generated zen_bre_rom / zen_bre_tab (R, LM, K2)
  localparam int NR   = 68;     // registers per lane
  localparam bit MFWD = 1'b0;   // 1: MUL results forwarded to the operand fetch (LM = 3), 0: LM = 4
  localparam bit K2   = 1'b1;   // ALU2 two-operand

  function automatic logic [NR-1:0] onehot_r(input logic [6:0] a);
    logic [NR-1:0] r;
    r = '0;
    for (int i = 0; i < NR; i++) if (a == i[6:0]) r[i] = 1'b1;
    return r;
  endfunction
  function automatic logic alu_valid(input logic [2:0] op);
    return (op == 3'd1) || (op == 3'd3);
  endfunction

  // ------------------------------------------------------------------ configuration
  logic [1:0]  prm, knd, kc;        // latched param / kind, ROM kind code of the current pass
  logic        pas, npas2, use_b;   // pass index, two-pass operation, B operand used
  logic [2:0]  nbl;                 // number of batches - 1
  logic        zneg;                // zeta sign swap (baseinv, param != 0)
  always_comb begin
    case (prm)
      2'd0:    nbl = 3'd1;
      2'd1:    nbl = 3'd3;
      default: nbl = 3'd7;
    endcase
    npas2 = (prm == 2'd2) && (knd == 2'd2);
    kc    = (npas2 && pas) ? 2'd3 : knd;
    use_b = (knd != 2'd2) || (npas2 && pas);
    zneg  = (knd == 2'd2) && (prm != 2'd0);
  end

  logic [IW-1:0] rom_instr;
  logic [7:0]    last_pc, ld_pc;
  logic [7:0]    fpc, npc, ir_pc;
  zen_bre_rom u_rom (.param(prm), .kind(kc), .pc(fpc), .instr(rom_instr), .last_pc(last_pc), .ld_pc(ld_pc));

  // ------------------------------------------------------------------ ROM output pre-decode
  logic [12:0][6:0]  rsel_d;
  logic [12:0]       isimm_d;
  logic [12:0][15:0] immv_d;
  logic              hasq_d;
  always_comb begin
    for (int k = 0; k < 3; k++) begin
      rsel_d[3*k]   = rom_instr[42*k+10 +: 7];
      rsel_d[3*k+1] = rom_instr[42*k+17 +: 7];
      rsel_d[3*k+2] = rom_instr[42*k+24 +: 7];
    end
    for (int u = 0; u < 2; u++) begin
      rsel_d[9+2*u]  = rom_instr[126+27*u+8 +: 7];
      rsel_d[10+2*u] = rom_instr[126+27*u+15 +: 7];
    end
    for (int k = 0; k < 13; k++) isimm_d[k] = (rsel_d[k][6:5] == 2'b11);
    hasq_d = (rom_instr[2:0] == 3'd4);
  end
  genvar gi;
  generate
    for (gi = 0; gi < 13; gi++) begin : g_imm
      zen_bre_imm u_imm (.idx(rsel_d[gi][4:0]), .val(immv_d[gi]));
    end
  endgenerate

  // ------------------------------------------------------------------ control state
  logic [2:0]    state;
  logic [2:0]    batch;              // batch of the instruction in IR
  logic          ir_last;
  logic          adv_q;              // IR executes this cycle
  logic [4:0]    qcnt;
  logic [IW-1:0] ir, nir;
  logic          nir_hasq;
  logic [12:0]       nisimm;
  logic [12:0][15:0] nimm;
  logic [2:0][NR-1:0]  a_woh_r;
  logic [1:0]        m1_v, m2_v, m1_oe, m2_oe;
  logic [1:0][6:0]   m1_dst, m2_dst;
  logic [1:0][3:0]   m1_oi, m2_oi;
  logic [1:0][NR-1:0]  m1_woh, m2_woh;
  logic          qv;
  logic [1:0]    qgrp;
  logic [3:0][9:0] qin;
  logic [15:0][15:0]        lane_qrd;
  logic [15:0][15:0][15:0]  lane_obuf;
  logic [3:0][15:0]         qrd_sel;
  logic [3:0][9:0]          q_data;

  // load engine
  logic          ld_act;             // issuing reads
  logic [2:0]    ld_j, ld_b;         // read pair / batch being loaded
  logic          ld_v;               // read data valid this cycle
  logic [2:0]    ld_jq, ld_bq;
  logic          ld_swap;
  logic          ld_cmpl;            // loads of batch ld_b fully written (fetch of its instr 0 allowed)
  logic          ld_req;             // start loading batch batch+1 now
  // write-back engine
  logic          wb_act;
  logic [2:0]    wb_j, wb_b;

  // ------------------------------------------------------------------ IR decode
  logic [2:0][2:0] a_op;
  logic [2:0][6:0] a_dst;
  logic [2:0]      a_oe, a_canon;
  logic [2:0][3:0] a_oi;
  logic [2:0][2:0] a_neg, a_zero;
  logic [2:0][1:0] a_cin;
  logic [2:0][6:0] n_dst;
  logic [2:0]      n_aval;
  logic [12:0][6:0] n_sel;
  logic [6:0]      q_dst;
  always_comb begin
    for (int k = 0; k < 3; k++) begin
      a_op[k]    = ir[42*k +: 3];
      a_dst[k]   = ir[42*k+3 +: 7];
      a_oe[k]    = ir[42*k+37];
      a_oi[k]    = ir[42*k+38 +: 4];
      a_canon[k] = (a_op[k] == 3'd3);
      for (int i = 0; i < 3; i++) begin
        a_neg[k][i]  = (ir[42*k+31+2*i +: 2] == 2'd2);
        a_zero[k][i] = (ir[42*k+31+2*i +: 2] == 2'd0);
      end
      a_cin[k]  = {1'b0, a_neg[k][0]} + {1'b0, a_neg[k][1]} + {1'b0, a_neg[k][2]};
      n_dst[k]  = nir[42*k+3 +: 7];
      n_aval[k] = alu_valid(nir[42*k +: 3]);
      n_sel[3*k]   = nir[42*k+10 +: 7];
      n_sel[3*k+1] = nir[42*k+17 +: 7];
      n_sel[3*k+2] = nir[42*k+24 +: 7];
    end
    for (int u = 0; u < 2; u++) begin
      n_sel[9+2*u]  = nir[126+27*u+8 +: 7];
      n_sel[10+2*u] = nir[126+27*u+15 +: 7];
    end
    q_dst = a_dst[0];
  end

  logic stall, prog_end, fetch_en, nir_ld, nb_wait;
  assign stall    = (state == S_EXEC) && !adv_q;
  assign prog_end = adv_q && ir_last;
  // last instruction of a non-final batch: the next batch's instr 0 is fetched in this cycle only if its
  // loads are complete; otherwise the pipeline waits in S_FETCH
  assign nb_wait  = prog_end && (batch != nbl) && !ld_cmpl;
  assign fetch_en = (adv_q && !nb_wait && !(prog_end && batch == nbl)) || ((state == S_FETCH) && ld_cmpl);
  assign nir_ld   = fetch_en || (state == S_LOAD);

  // execute-cycle write enables / captures
  logic [2:0][NR-1:0] alu_woh;
  logic [2:0]       alu_oe;
  logic [1:0][NR-1:0] mul_woh;
  logic [1:0]       mul_oe;
  logic [4:0][12:0] byp;
  logic [NR-1:0]      q_woh_sh;
  always_comb begin
    for (int k = 0; k < 3; k++) begin
      alu_woh[k] = adv_q ? a_woh_r[k] : '0;
      alu_oe[k]  = adv_q && alu_valid(a_op[k]) && a_oe[k];
    end
    for (int u = 0; u < 2; u++) begin
      mul_woh[u] = adv_q ? m2_woh[u] : '0;
      mul_oe[u]  = adv_q && m2_v[u] && m2_oe[u];
    end
    q_woh_sh = onehot_r(q_dst);
    for (int k = 0; k < 13; k++) begin
      for (int b = 0; b < 3; b++)
        byp[b][k] = adv_q && !nisimm[k] && alu_valid(a_op[b]) && (a_dst[b] == n_sel[k]);
      for (int u = 0; u < 2; u++)
        byp[3+u][k] = MFWD && adv_q && !nisimm[k] && m2_v[u] && (m2_dst[u] == n_sel[k]);
    end
  end

  // ------------------------------------------------------------------ FSM
  assign ld_req = (state == S_EXEC) && adv_q && (ir_pc == ld_pc) && (batch != nbl);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= S_IDLE; prm <= 2'd0; knd <= 2'd0; pas <= 1'b0;
      batch <= 3'd0; fpc <= 8'd0; npc <= 8'd0; ir_pc <= 8'd0; ir_last <= 1'b0; adv_q <= 1'b0;
      qcnt <= 5'd0; ir <= '0; nir <= '0; nir_hasq <= 1'b0; nisimm <= '0; nimm <= '0; a_woh_r <= '0;
      m1_v <= '0; m2_v <= '0; m1_oe <= '0; m2_oe <= '0; m1_dst <= '0; m2_dst <= '0; m1_oi <= '0; m2_oi <= '0;
      m1_woh <= '0; m2_woh <= '0; qv <= 1'b0; qgrp <= 2'd0; qin <= '0;
      ld_act <= 1'b0; ld_j <= 3'd0; ld_b <= 3'd0; ld_v <= 1'b0; ld_jq <= 3'd0; ld_bq <= 3'd0; ld_swap <= 1'b0;
      ld_cmpl <= 1'b0; wb_act <= 1'b0; wb_j <= 3'd0; wb_b <= 3'd0; wr_cnt <= 8'd0;
    end else begin
      // ---------------- load engine
      ld_v    <= ld_act;
      ld_jq   <= ld_j;
      ld_bq   <= ld_b;
      ld_swap <= ^{ld_b, ld_j};
      if (ld_act) begin
        ld_j <= ld_j + 3'd1;
        if (ld_j == 3'd7) ld_act <= 1'b0;
      end
      if (ld_v && ld_jq == 3'd7) ld_cmpl <= 1'b1;
      if (ld_req) begin ld_act <= 1'b1; ld_j <= 3'd0; ld_b <= batch + 3'd1; ld_cmpl <= 1'b0; end

      // ---------------- QINV server: 4 ROM copies, lane group qcnt (lanes 4g..4g+3) per cycle
      qcnt <= stall ? qcnt + 5'd1 : 5'd0;
      qv   <= stall && (qcnt < 5'd4);
      qgrp <= qcnt[1:0];
      for (int c = 0; c < 4; c++) qin[c] <= qrd_sel[c][9:0];

      // ---------------- next-instruction register (ROM output + pre-decode)
      // NIR <- ROM[fpc]; fpc runs ahead of NIR by one and wraps to 0 after last_pc
      if (nir_ld) begin
        nir <= rom_instr; nir_hasq <= hasq_d; nisimm <= isimm_d; nimm <= immv_d;
        npc <= fpc;
        fpc <= (fpc == last_pc) ? 8'd0 : fpc + 8'd1;
      end

      // ---------------- instruction register advance
      if (fetch_en) begin
        ir <= nir;
        ir_pc <= npc;
        for (int k = 0; k < 3; k++) a_woh_r[k] <= n_aval[k] ? onehot_r(n_dst[k]) : '0;
        ir_last <= (npc == last_pc);
      end

      // ---------------- multiplier control pipelines
      if (adv_q) begin
        for (int u = 0; u < 2; u++) begin
          m1_v[u]   <= ir[126+27*u];
          m1_dst[u] <= ir[126+27*u+1 +: 7];
          m1_oe[u]  <= ir[126+27*u+22];
          m1_oi[u]  <= ir[126+27*u+23 +: 4];
          m1_woh[u] <= ir[126+27*u] ? onehot_r(ir[126+27*u+1 +: 7]) : '0;
        end
        m2_v <= m1_v; m2_dst <= m1_dst; m2_oe <= m1_oe; m2_oi <= m1_oi; m2_woh <= m1_woh;
      end

      // ---------------- adv for the next cycle
      if (state == S_EXEC && adv_q)            adv_q <= !prog_end ? !nir_hasq : ((batch != nbl) && ld_cmpl && !nir_hasq);
      else if (state == S_EXEC)                adv_q <= (qcnt == 5'd4);
      else if (state == S_FETCH && ld_cmpl)    adv_q <= !nir_hasq;
      else                                     adv_q <= 1'b0;

      // ---------------- write-back sequencer
      if (prog_end) begin
        wb_act <= 1'b1; wb_j <= 3'd0; wb_b <= batch;
      end else if (wb_act) begin
        if (wb_j == 3'd7) wb_act <= 1'b0;
        wb_j <= wb_j + 3'd1;
      end
      if (wb_act) wr_cnt <= {1'b0, wb_b, wb_j, 1'b0} + 8'd2;

      // ---------------- main FSM
      case (state)
        S_IDLE: if (start) begin
          prm <= param; knd <= kind; pas <= 1'b0; batch <= 3'd0; wr_cnt <= 8'd0;
          ld_act <= 1'b1; ld_j <= 3'd0; ld_b <= 3'd0; ld_cmpl <= 1'b0;
          fpc <= 8'd0; state <= S_LOAD;
        end
        S_LOAD:  state <= S_FETCH;                       // NIR <- instruction 0
        S_FETCH: if (ld_cmpl) state <= S_EXEC;           // operands of instruction 0 fetched
        S_EXEC: begin
          if (prog_end) begin
            if (batch != nbl) begin
              batch <= batch + 3'd1;
              if (!ld_cmpl) state <= S_FETCH;            // next batch's loads not complete yet
            end else if (npas2 && !pas) begin
              pas <= 1'b1; batch <= 3'd0; fpc <= 8'd0;
              ld_act <= 1'b1; ld_j <= 3'd0; ld_b <= 3'd0; ld_cmpl <= 1'b0;
              state <= S_LOAD;
            end else begin
              state <= S_FIN;
            end
          end
        end
        S_FIN: if (done) state <= S_IDLE;
        default: state <= S_IDLE;
      endcase
    end
  end

  assign active = (state != S_IDLE);
  assign wb_last = prog_end && (batch == nbl) && !(npas2 && !pas);
  assign done   = (state == S_FIN) && wb_act && (wb_j == 3'd7) && !(npas2 && !pas);

  // ------------------------------------------------------------------ memory masters
  always_comb begin
    ra_rd_en   = ld_act ? 2'b11 : 2'b00;
    rb_rd_en   = (ld_act && use_b) ? 2'b11 : 2'b00;
    ra_rd_addr = {{ld_b, ld_j}, {ld_b, ld_j}};
    rb_rd_addr = {{ld_b, ld_j}, {ld_b, ld_j}};
  end

  // ------------------------------------------------------------------ lane input load
  logic [255:0] aev, aod, bev, bod;
  logic [15:0][35:0] ld_we;
  logic [15:0][35:0][15:0] ld_data;

  assign aev = ld_swap ? ra_rd_data[1] : ra_rd_data[0];
  assign aod = ld_swap ? ra_rd_data[0] : ra_rd_data[1];
  assign bev = ld_swap ? rb_rd_data[1] : rb_rd_data[0];
  assign bod = ld_swap ? rb_rd_data[0] : rb_rd_data[1];

  // zeta of block blk = (16*batch + lane)*k + j : +-f[64 + blk/2], '+' for even blk (swapped by zneg).
  // Zetas are written together with the lane's operands (lanes 2*ld_jq, 2*ld_jq+1): 8 table lookups per cycle.
  logic [1:0][3:0][5:0]  zidx;
  logic [1:0][3:0]       zsgn;
  logic [1:0][3:0][15:0] zmag, zeta;
  always_comb begin
    for (int h = 0; h < 2; h++) begin
      for (int j = 0; j < 4; j++) begin
        logic [8:0] blk;
        logic [3:0] ln;
        ln = {ld_jq, h[0]};
        case (prm)
          2'd0:    blk = {2'b0, ld_bq[0], ln, j[1:0]};      // k = 4
          2'd1:    blk = {2'b0, ld_bq[1:0], ln, j[0]};      // k = 2
          default: blk = {2'b0, ld_bq, ln};                 // k = 1
        endcase
        zidx[h][j] = blk[6:1];
        zsgn[h][j] = blk[0] ^ zneg;
      end
    end
  end
  generate
    for (gi = 0; gi < 8; gi++) begin : g_zeta
      zen_bre_zeta u_zeta (.i(zidx[gi/4][gi%4]), .fz(zmag[gi/4][gi%4]));
    end
  endgenerate
  always_comb begin
    for (int h = 0; h < 2; h++)
      for (int j = 0; j < 4; j++)
        zeta[h][j] = zsgn[h][j] ? (16'd0 - zmag[h][j]) : zmag[h][j];
  end

  always_comb begin
    ld_we   = '0;
    ld_data = '0;
    for (int l = 0; l < 16; l++) begin
      if (ld_v && (ld_jq == l[3:1])) begin
        for (int i = 0; i < 16; i++) begin
          ld_we[l][i]      = 1'b1;
          ld_data[l][i]    = l[0] ? aod[16*i +: 16] : aev[16*i +: 16];
          ld_we[l][16+i]   = use_b;
          ld_data[l][16+i] = l[0] ? bod[16*i +: 16] : bev[16*i +: 16];
        end
      end
      if (ld_v && (ld_jq == l[3:1])) begin
        for (int j = 0; j < 4; j++) begin
          ld_we[l][32+j]   = 1'b1;
          ld_data[l][32+j] = zeta[l%2][j];
        end
      end
    end
  end

  // ------------------------------------------------------------------ lanes
  always_comb begin
    for (int c = 0; c < 4; c++) qrd_sel[c] = lane_qrd[{qcnt[1:0], c[1:0]}];
  end
  generate
    for (gi = 0; gi < 4; gi++) begin : g_qinv
      zen_bre_qinv u_qinv (.x(qin[gi]), .y(q_data[gi]));
    end
  endgenerate

  generate
    for (gi = 0; gi < 16; gi++) begin : g_lane
      logic [NR-1:0] q_woh_l;
      assign q_woh_l = (qv && (qgrp == gi / 4)) ? q_woh_sh : '0;
      zen_bre_lane #(.NR(NR), .K2(K2)) u_lane (
        .clk(clk), .adv(adv_q), .fetch_en(fetch_en), .fsel_ld(nir_ld), .fsel_d(rsel_d),
        .nisimm(nisimm), .nimm(nimm), .byp(byp),
        .alu_canon(a_canon), .alu_neg(a_neg), .alu_zero(a_zero), .alu_cin(a_cin),
        .alu_woh(alu_woh), .alu_oe(alu_oe), .alu_oi(a_oi),
        .mul_woh(mul_woh), .mul_oe(mul_oe), .mul_oi(m2_oi),
        .q_woh(q_woh_l), .q_data(q_data[gi % 4]), .q_rd(lane_qrd[gi]),
        .ld_we(ld_we[gi]), .ld_data(ld_data[gi]), .obuf(lane_obuf[gi]));
    end
  endgenerate

  // ------------------------------------------------------------------ write-back data
  logic [255:0] wev, wod;
  logic [5:0]   wr_addr6;
  assign wr_addr6 = {wb_b, wb_j};
  always_comb begin
    for (int i = 0; i < 16; i++) begin
      wev[16*i +: 16] = lane_obuf[{wb_j, 1'b0}][i];
      wod[16*i +: 16] = lane_obuf[{wb_j, 1'b1}][i];
    end
    wc_wr_en      = wb_act ? 2'b11 : 2'b00;
    wc_wr_addr    = {wr_addr6, wr_addr6};
    wc_wr_data[0] = (^wr_addr6) ? wod : wev;
    wc_wr_data[1] = (^wr_addr6) ? wev : wod;
  end

endmodule
