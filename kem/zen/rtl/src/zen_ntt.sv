// zen_ntt : NTT / NTT_MQ / INTT_RAW / INTT_ENC engine
//
// Two memory passes over a streaming pipeline of four 16-butterfly stages (zen_ntt_bf) joined by
// multipath delay commutators (zen_ntt_comm, delays 4/2/1 elements). The polynomial slot still moves
// 2 words in + 2 words out per cycle.
//   mode 0 NTT      : 7 CT layers                                 = ntt(p, a, mq=False)
//   mode 1 NTT_MQ   : 7 CT layers + scaling canon(fqmul(x,171)), zq_fail on an all-zero m-block
//   mode 2 INTT_RAW : 7 GS layers                                 = intt_raw(p, a)
//   mode 3 INTT_ENC : 6 GS layers + fused last layer; the outputs are not written back but
//                     handed to zen_fci (combine/compress -> CT object, or FO compare when cmp = 1)
// Layer l: CT e = 6+param-l, GS e = param+l; e >= 2: cross-word layer with word stride 2^(e-2),
// e = 1: within-word len 8 (W8), e = 0: within-word len 4 (W4). Twiddle f[2^l + blk] (CT),
// fn[128 - 128/2^l + blk] (GS).
// Pass split: CT pass 0 = layers 0..3, pass 1 = layers 4..6 + stage 4 SC (NTT_MQ) / bypass;
//             GS pass 0 = layers 0..2 + stage 4 bypass, pass 1 = layers 3..6.
// A pass processes groups of 2^c words (c = number of cross-word layers in the pass); a group is
// issued as 2^(c-1) word pairs on consecutive cycles (pair j = local indices j and j + 2^(c-1)); the
// i-th cross-word stage pairs local bit c-i, whose physical stride is the layer's word stride.
// Read-after-write between the passes: a 128-bit scoreboard marks words between read issue and write;
// a pass-1 group is issued only when none of its words is marked and at least 8 cycles after the last
// pass-0 issue (commutator delays of the two passes never mix).
// Pipeline: issue (c) -> read request register (c+1) -> read data (c+2) -> stage 1 (2 regs) ->
// commutator -> stage 2 -> commutator -> stage 3 -> commutator -> stage 4 -> write.
module zen_ntt (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              start,
  input  logic [1:0]        param,
  input  logic [1:0]        mode,
  input  logic              cmp,           // INTT_ENC: compare with the CT object instead of writing
  input  logic [511:0]      mbits,         // INTT_ENC: message bits (low N4 bits)
  output logic              done,
  output logic              active,
  output logic              zq_fail,
  output logic              mismatch,      // INTT_ENC with cmp: valid at done
  output logic              fci_act,       // INTT_ENC fused combine/compress active (observation)
  output logic              iss_done,      // all reads of the operation issued (writes may still be in flight)
  output logic [1:0]        rd_en,
  output logic [1:0][5:0]   rd_addr,
  input  logic [1:0][255:0] rd_data,
  output logic [1:0]        wr_en,
  output logic [1:0][5:0]   wr_addr,
  output logic [1:0][255:0] wr_data,
  // INTT_ENC: e polynomial read master (other slot)
  output logic [1:0]        re_rd_en,
  output logic [1:0][5:0]   re_rd_addr,
  input  logic [1:0][255:0] re_rd_data,
  // INTT_ENC: CT object ports A and B (16-byte words)
  output logic              cta_en, cta_we,
  output logic [6:0]        cta_addr,
  output logic [127:0]      cta_wdata,
  input  logic [127:0]      cta_rdata,
  output logic              ctb_en, ctb_we,
  output logic [6:0]        ctb_addr,
  output logic [127:0]      ctb_wdata,
  input  logic [127:0]      ctb_rdata
);
  localparam logic [2:0] OP_CT = 3'd0, OP_GS = 3'd1, OP_SC = 3'd2, OP_BYP = 3'd3, OP_GF = 3'd4;
  localparam logic [1:0] LT_X = 2'd0, LT_W8 = 2'd1, LT_W4 = 2'd2;
  localparam int EW = 269;     // element {pass, v, L[3:0], addr[6:0], word[255:0]}

  // ---------------------------------------------------------------- configuration
  typedef struct packed {
    logic [2:0] op;
    logic [1:0] lt;
    logic [2:0] ls;
    logic [6:0] base;
    logic       rom;
    logic       x;        // cross-word stage
    logic [2:0] d;        // commutator delay in front of this stage (0,1,2,4)
  } scfg_t;

  logic [1:0] prm, md;
  logic       cmpq;

  function automatic scfg_t layer(input logic [1:0] p, input logic gs, input logic [2:0] l);
    scfg_t r;
    logic [3:0] e;
    logic [7:0] gsb;
    gsb = 8'd128 - (8'd128 >> l);
    r = '0;
    if (gs) begin r.op = OP_GS; r.rom = 1'b1; r.base = gsb[6:0]; e = {2'd0, p} + {1'b0, l}; end
    else    begin r.op = OP_CT; r.rom = 1'b0; r.base = 7'd1 << l;  e = 4'd6 + {2'd0, p} - {1'b0, l}; end
    if (e >= 4'd2)      begin r.lt = LT_X;  r.ls = e[2:0] - 3'd2; r.x = 1'b1; end
    else if (e == 4'd1) begin r.lt = LT_W8; r.ls = 3'd0; r.x = 1'b0; end
    else                begin r.lt = LT_W4; r.ls = 3'd0; r.x = 1'b0; end
    return r;
  endfunction

  // stage configuration for (param, mode, pass, stage 0..3); d filled in by cfg_all
  function automatic scfg_t stage_raw(input logic [1:0] p, input logic [1:0] m, input logic ps, input int k);
    scfg_t r;
    logic gs;
    gs = m[1];
    r = '0;
    if (!gs) begin
      if (!ps)        r = layer(p, 1'b0, k[2:0]);
      else if (k < 3) r = layer(p, 1'b0, 3'd4 + k[2:0]);
      else begin r.op = (m == 2'd1) ? OP_SC : OP_BYP; r.lt = LT_X; end
    end else begin
      if (!ps) begin
        if (k < 3) r = layer(p, 1'b1, k[2:0]);
        else begin r.op = OP_BYP; r.lt = LT_X; end
      end else begin
        r = layer(p, 1'b1, 3'd3 + k[2:0]);
        if (k == 3 && m == 2'd3) r.op = OP_GF;
      end
    end
    return r;
  endfunction

  typedef struct packed {
    scfg_t [3:0] st;
    logic  [2:0] c;            // number of cross-word stages
    logic  [3:0][2:0] lsx;     // stride exponents of cross stage i = 1..4 (index i-1)
  } pcfg_t;

  function automatic pcfg_t cfg_all(input logic [1:0] p, input logic [1:0] m, input logic ps);
    pcfg_t r;
    int ci;
    r = '0; ci = 0;
    for (int k = 0; k < 4; k++) begin
      r.st[k] = stage_raw(p, m, ps, k);
      if (r.st[k].x) begin r.lsx[ci] = r.st[k].ls; ci = ci + 1; end
    end
    r.c = ci[2:0];
    ci = 0;
    for (int k = 0; k < 4; k++) begin
      if (r.st[k].x) begin
        ci = ci + 1;
        if (ci >= 2) r.st[k].d = 3'd1 << (r.c - ci[2:0]);
      end
    end
    return r;
  endfunction

  pcfg_t c0, c1, ci;           // pass 0, pass 1, issue pass
  logic  pas;
  always_comb begin
    c0 = cfg_all(prm, md, 1'b0);
    c1 = cfg_all(prm, md, 1'b1);
    ci = pas ? c1 : c0;
  end

  // ---------------------------------------------------------------- address generation
  logic [2:0] aw;              // address bits: 5/6/7
  always_comb aw = 3'd5 + {1'b0, prm};

  // physical address of group g, local index L under configuration cf
  function automatic logic [6:0] gaddr(input pcfg_t cf, input logic [2:0] abits, input logic [6:0] g, input logic [3:0] L);
    logic [6:0] used, a;
    int gb;
    used = '0; a = '0;
    for (int i = 0; i < 4; i++) if (i < cf.c) used[cf.lsx[i]] = 1'b1;
    gb = 0;
    for (int b = 0; b < 7; b++) begin
      if (b < abits && !used[b]) begin a[b] = g[gb]; gb = gb + 1; end
    end
    for (int i = 0; i < 4; i++)
      if (i < cf.c && L[cf.c - 1 - i]) a[cf.lsx[i]] = 1'b1;
    return a;
  endfunction

  // ---------------------------------------------------------------- issue control
  logic        busy, issuing;
  logic [6:0]  g, glast;
  logic [2:0]  j, jlast;
  logic        ingrp;          // inside a group (pairs being issued)
  logic [3:0]  gap;            // cycles since the last pass-0 issue (saturating)
  logic [127:0] sb;
  logic [8:0]  inflight;
  logic        issue;
  logic [6:0]  ia, ib;
  logic [3:0]  il_u, il_l;
  logic        grp_free;
  logic        out_v4;         // stage-4 output valid this cycle

  always_comb begin
    glast = (7'd1 << (aw - ci.c)) - 7'd1;
    jlast = (3'd1 << (ci.c - 3'd1)) - 3'd1;
    ia    = gaddr(ci, aw, g, {1'b0, j});
    ib    = gaddr(ci, aw, g, {1'b0, j} + (4'd1 << (ci.c - 3'd1)));
    il_u  = {1'b0, j};
    il_l  = {1'b0, j} + (4'd1 << (ci.c - 3'd1));
    grp_free = 1'b1;
    for (int L = 0; L < 16; L++)
      if (L < (1 << ci.c) && sb[gaddr(ci, aw, g, L[3:0])]) grp_free = 1'b0;
    issue = busy && issuing && (ingrp || (!pas) || (grp_free && gap >= 4'd8));
  end

  // ---------------------------------------------------------------- pipeline elements
  // read request (c+1), read data (c+2)
  logic        r_v, d_v;
  logic [6:0]  r_a, r_b, d_a, d_b;
  logic [3:0]  r_lu, r_ll, d_lu, d_ll;
  logic        r_p, d_p;
  logic [1:0][5:0] rd_addr_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin r_v <= 1'b0; d_v <= 1'b0; end
    else begin r_v <= issue; d_v <= r_v; end
  end
  always_ff @(posedge clk) begin
    r_a <= ia; r_b <= ib; r_lu <= il_u; r_ll <= il_l; r_p <= pas;
    d_a <= r_a; d_b <= r_b; d_lu <= r_lu; d_ll <= r_ll; d_p <= r_p;
    if (^ia) begin rd_addr_q[1] <= ia[6:1]; rd_addr_q[0] <= ib[6:1]; end
    else     begin rd_addr_q[0] <= ia[6:1]; rd_addr_q[1] <= ib[6:1]; end
  end
  assign rd_en   = {2{r_v}};
  assign rd_addr = rd_addr_q;

  logic [EW-1:0] s_in_u [0:3], s_in_l [0:3];    // stage inputs (after commutators)
  logic [EW-1:0] s_out_u [0:3], s_out_l [0:3];  // stage outputs
  logic [3:0][7:0] s_zf;
  logic [3:0][2:0] s_oop;

  assign s_in_u[0] = {d_p, d_v, d_lu, d_a, (^d_a) ? rd_data[1] : rd_data[0]};
  assign s_in_l[0] = {d_p, d_v, d_ll, d_b, (^d_a) ? rd_data[0] : rd_data[1]};

  // element layout: [268] pass, [267] valid, [266:263] local index L, [262:256] word address, [255:0] word
  genvar gk;
  generate
    for (gk = 0; gk < 4; gk++) begin : g_st
      scfg_t kc;
      logic [EW-1:0] eu, el;
      logic          ov;
      logic [255:0]  owa, owb;
      logic [22:0]   mi, mo;            // {pass, L_u, addr_u, L_l, addr_l}
      assign eu = s_in_u[gk];
      assign el = s_in_l[gk];
      assign kc = eu[268] ? c1.st[gk] : c0.st[gk];
      assign mi = {eu[268], eu[266:263], eu[262:256], el[266:263], el[262:256]};
      zen_ntt_bf #(.HAS_SC(gk == 3), .NTW((gk == 3) ? 1 : (gk == 1) ? 2 : 4), .TW(23)) u_bf (
        .clk(clk), .rst_n(rst_n), .en(1'b1), .in_v(eu[267]), .in_wa(eu[255:0]), .in_wb(el[255:0]),
        .in_addr(eu[262:256]), .in_addr_b(el[262:256]),
        .in_op(kc.op), .in_lt(kc.lt), .in_ls(kc.ls), .in_base(kc.base), .in_rom(kc.rom),
        .in_tag(mi), .out_v(ov), .out_wa(owa), .out_wb(owb), .out_op(s_oop[gk]),
        .out_tag(mo), .out_zf(s_zf[gk]));
      assign s_out_u[gk] = {mo[22], ov, mo[21:18], mo[17:11], owa};
      assign s_out_l[gk] = {mo[22], ov, mo[10:7],  mo[6:0],   owb};
    end
  endgenerate

  // commutators in front of stages 2..4 (max delays 4, 2, 1)
  logic [2:0] cd1, cd2, cd3;
  assign cd1 = s_out_u[0][268] ? c1.st[1].d : c0.st[1].d;
  assign cd2 = s_out_u[1][268] ? c1.st[2].d : c0.st[2].d;
  assign cd3 = s_out_u[2][268] ? c1.st[3].d : c0.st[3].d;
  zen_ntt_comm #(.DMAX(4), .EW(EW)) u_c1 (.clk(clk), .rst_n(rst_n), .d_in(cd1), .u(s_out_u[0]), .l(s_out_l[0]), .ou(s_in_u[1]), .ol(s_in_l[1]));
  zen_ntt_comm #(.DMAX(2), .EW(EW)) u_c2 (.clk(clk), .rst_n(rst_n), .d_in(cd2), .u(s_out_u[1]), .l(s_out_l[1]), .ou(s_in_u[2]), .ol(s_in_l[2]));
  zen_ntt_comm #(.DMAX(1), .EW(EW)) u_c3 (.clk(clk), .rst_n(rst_n), .d_in(cd3), .u(s_out_u[2]), .l(s_out_l[2]), .ou(s_in_u[3]), .ol(s_in_l[3]));

  // ---------------------------------------------------------------- stage 4 output: write / fused combine
  logic [EW-1:0] ou, ol;
  logic          enc_last;        // output belongs to the fused last pass of INTT_ENC
  assign ou = s_out_u[3];
  assign ol = s_out_l[3];
  assign out_v4   = ou[267];
  assign enc_last = (md == 2'd3) && ou[268];

  always_comb begin
    wr_en = {2{out_v4 && !enc_last}};
    if (^ou[262:256]) begin
      wr_addr[1] = ou[262:257]; wr_data[1] = ou[255:0];
      wr_addr[0] = ol[262:257]; wr_data[0] = ol[255:0];
    end else begin
      wr_addr[0] = ou[262:257]; wr_data[0] = ou[255:0];
      wr_addr[1] = ol[262:257]; wr_data[1] = ol[255:0];
    end
  end

  // e read for the fused pass: issued when the pair enters stage 4 (data aligned with the stage-4 output)
  logic        e_req;
  logic [6:0]  e_a, e_b;
  logic [1:0][5:0] re_addr_q;
  logic        re_v;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) re_v <= 1'b0;
    else        re_v <= (md == 2'd3) && s_in_u[3][267] && s_in_u[3][268];
  end
  always_ff @(posedge clk) begin
    if (^s_in_u[3][262:256]) begin re_addr_q[1] <= s_in_u[3][262:257]; re_addr_q[0] <= s_in_l[3][262:257]; end
    else                     begin re_addr_q[0] <= s_in_u[3][262:257]; re_addr_q[1] <= s_in_l[3][262:257]; end
  end
  assign re_rd_en   = {2{re_v}};
  assign re_rd_addr = re_addr_q;

  logic fci_done_v;   // a pair finished by zen_fci this cycle
  logic fci_idle;
  zen_fci u_fci (
    .clk(clk), .rst_n(rst_n), .start(start && !busy), .param(prm), .cmp(cmpq), .mbits(mbits),
    .in_v(out_v4 && enc_last), .in_wa(ou[255:0]), .in_wb(ol[255:0]), .in_aa(ou[262:256]), .in_ab(ol[262:256]),
    .e_data(re_rd_data),
    .cta_en(cta_en), .cta_we(cta_we), .cta_addr(cta_addr), .cta_wdata(cta_wdata), .cta_rdata(cta_rdata),
    .ctb_en(ctb_en), .ctb_we(ctb_we), .ctb_addr(ctb_addr), .ctb_wdata(ctb_wdata), .ctb_rdata(ctb_rdata),
    .pair_done(fci_done_v), .mismatch(mismatch), .idle(fci_idle));

  // ---------------------------------------------------------------- control
  logic fin_issue;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy <= 1'b0; issuing <= 1'b0; fin_issue <= 1'b0; prm <= 2'd0; md <= 2'd0; cmpq <= 1'b0; pas <= 1'b0;
      g <= 7'd0; j <= 3'd0; ingrp <= 1'b0; gap <= 4'd0; inflight <= 9'd0; sb <= '0;
    end else begin
      if (start && !busy) begin
        busy <= 1'b1; issuing <= 1'b1; fin_issue <= 1'b0; prm <= param; md <= mode; cmpq <= cmp; pas <= 1'b0;
        g <= 7'd0; j <= 3'd0; ingrp <= 1'b0; gap <= 4'd0; inflight <= 9'd0; sb <= '0;
      end else if (busy) begin
        if (pas && gap != 4'd15) gap <= gap + 4'd1;
        if (issue) begin
          if (j == jlast) begin
            j <= 3'd0; ingrp <= 1'b0;
            if (g == glast) begin
              g <= 7'd0;
              if (pas) begin issuing <= 1'b0; fin_issue <= 1'b1; end
              else begin pas <= 1'b1; gap <= 4'd0; end
            end else g <= g + 7'd1;
          end else begin
            j <= j + 3'd1; ingrp <= 1'b1;
          end
        end
        inflight <= inflight + {8'd0, issue} - {8'd0, (out_v4 && !enc_last) || fci_done_v};
        for (int w = 0; w < 128; w++) begin
          if (out_v4 && (ou[262:256] == w[6:0] || ol[262:256] == w[6:0])) sb[w] <= 1'b0;
          if (issue && (ia == w[6:0] || ib == w[6:0])) sb[w] <= 1'b1;
        end
        if (fin_issue && inflight == 9'd0 && !issue && fci_idle) busy <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------- status
  logic zq_now, zq_acc, zblk;
  logic [7:0] zf4;
  assign zf4 = s_zf[3];
  always_comb begin
    case (prm)
      2'd0:    zblk = |zf4;
      2'd1:    zblk = (zf4[0] & zf4[1]) | (zf4[2] & zf4[3]) | (zf4[4] & zf4[5]) | (zf4[6] & zf4[7]);
      default: zblk = (&zf4[3:0]) | (&zf4[7:4]);
    endcase
    zq_now = out_v4 && (s_oop[3] == OP_SC) && zblk;
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)              zq_acc <= 1'b0;
    else if (start && !busy) zq_acc <= 1'b0;
    else if (zq_now)         zq_acc <= 1'b1;
  end
  assign zq_fail = zq_acc;
  assign done    = busy && fin_issue && inflight == 9'd0 && !issue && fci_idle;
  assign active  = busy;
  assign fci_act = busy && (md == 2'd3) && pas;
  assign iss_done = busy && fin_issue;
endmodule
