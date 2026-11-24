// nev_hash: Hash / Hash2 / H(pk) / KDF unit of nev_kem_top (ICCS SM3 backend).
//
// Sixteen SM3 cores (one round per clock).  SEED / HPK / HASH2 run as static
// job programs on two chain engines (cores 0 and 1) that exchange intermediate
// digests through value registers with valid flags (an engine's fill stalls on
// a value that is not yet produced).  KDF runs up to sixteen independent counter
// blocks of two streams in parallel on all cores, with a cached seed midstate for S = 64.
// The random-byte capture into m runs beside any job (rnd_go / rnd_fin).
module nev_hash #(
  parameter int NC = 16                  // SM3 cores
) (
  input  logic          clk,
  input  logic          rst_n,
  // latched configuration
  input  logic [1:0]    slog,
  input  logic [11:0]   poly_bytes,
  // command
  input  logic          cmd_valid,
  input  logic [2:0]    cmd_op,
  input  logic          kdf_seed_sel,
  input  logic [7:0]    kdf_nonce,
  input  logic [6:0]    kdf_nblocks,
  input  logic [11:0]   kdf_nmlen,
  input  logic [7:0]    kdf_nonce1,
  input  logic [6:0]    kdf_nblocks1,
  input  logic          kdf_need,
  output logic          busy,
  output logic          done,
  // random byte stream
  output logic          rnd_req,
  input  logic          rnd_go,         // start capturing SEED_BYTES random bytes into m (runs beside any job)
  output logic          rnd_fin,        // capture complete
  input  logic          rnd_valid,
  input  logic [63:0]   rnd_data,
  // PK field (object memory port, read only, sel = 1)
  input  logic [8:0]    pk_avail,
  output logic          om_req,
  output logic          om_we,
  output logic [1:0]    om_sel,
  output logic [8:0]    om_addr,
  output logic [63:0]   om_wdata,
  output logic [7:0]    om_be,
  input  logic          om_gnt,
  input  logic          om_rvalid,
  input  logic [63:0]   om_rdata,
  // H(pk) load and message load
  input  logic [511:0]  hpk_field,
  input  logic          m_we,
  input  logic [511:0]  m_wdata,
  // KDF buffer write port and watermark
  output logic          kb_we,
  output logic [7:0]    kb_waddr,
  output logic [127:0]  kb_wdata,
  output logic [7:0]    kb_wm0,
  output logic [7:0]    kb_wm1,
  output logic          kdf_done1,
  // results
  output logic [1023:0] kr,
  output logic [511:0]  hk,
  output logic [511:0]  c0,
  output logic [511:0]  m_reg
);

  // ---------------------------------------------------------------- constants
  localparam logic [2:0] OP_RND    = 3'd0;
  localparam logic [2:0] OP_SEED   = 3'd1;
  localparam logic [2:0] OP_HPK    = 3'd2;
  localparam logic [2:0] OP_LOADHK = 3'd3;
  localparam logic [2:0] OP_HASH2  = 3'd4;
  localparam logic [2:0] OP_KDF    = 3'd5;
  localparam logic [2:0] OP_MORE   = 3'd6;

  // SM3 IV and HMAC key midstates compress(IV, KEY^36..), compress(IV, KEY^5c..)
  // (byte order: byte i at bits [8i+7:8i])
  localparam logic [255:0] IVB = 256'h4e0efbb04dee8de3aa383116bc306fa900068adad7422417b9b214496f168073;
  localparam logic [255:0] K1M = 256'h79dda8d028d13e0799a771835408209fab6b4bd48309e536d5f2be48a4b63e31;
  localparam logic [255:0] K2M = 256'h1a417fb9c2f7ff056530c3848d56d51d3f5cab620fc04aa27a1266e95dc77c74;

  // value ids (chain job sources / destinations)
  localparam logic [3:0] V_M    = 4'd0;
  localparam logic [3:0] V_HK   = 4'd1;
  localparam logic [3:0] V_SELF = 4'd2;
  localparam logic [3:0] V_K1   = 4'd3;   // register T0
  localparam logic [3:0] V_K2   = 4'd4;   // register T1
  localparam logic [3:0] V_K3   = 4'd5;   // register T0
  localparam logic [3:0] V_K4   = 4'd6;   // register T1
  localparam logic [3:0] V_H1   = 4'd7;   // OUT quarter 0 (KS / HK / KR by command)
  localparam logic [3:0] V_H2   = 4'd8;   // OUT quarter 1
  localparam logic [3:0] V_H3   = 4'd9;   // OUT quarter 2
  localparam logic [3:0] V_H4   = 4'd10;  // OUT quarter 3

  localparam logic [1:0] VS_IV = 2'd0;
  localparam logic [1:0] VS_K1 = 2'd1;
  localparam logic [1:0] VS_K2 = 2'd2;

  typedef struct packed {
    logic       v;
    logic [1:0] vsel;
    logic       p64;
    logic [1:0] pre;
    logic [1:0] suf;
    logic       pk;
    logic [3:0] s0;
    logic [3:0] n0;
    logic [3:0] s1;
    logic [3:0] n1;
    logic [3:0] s2;
    logic [3:0] n2;
    logic [3:0] dst;
  } jd_t;

  // job program ROM: op, slog, engine, job index
  function automatic jd_t jdesc(input logic [2:0] op, input logic [1:0] sl, input logic e,
                                input logic [2:0] ji);
    jd_t j;
    logic       ph512, ph1024;
    logic       bpk;
    logic [3:0] bs0, bn0, bs1, bn1;
    logic [1:0] code;
    j = '0;
    ph512 = 1'b0; ph1024 = 1'b0; bpk = 1'b0;
    bs0 = V_M; bn0 = 4'd0; bs1 = V_HK; bn1 = 4'd0; code = 2'd1;
    // single-block / single-chain cases on engine 0, job 0
    case (op)
      OP_SEED: begin
        if (sl == 2'd2) begin ph512 = 1'b1; bn0 = 4'd8; end
        else if (!e && ji == 3'd0) begin
          j.v = 1'b1; j.s0 = V_M; j.n0 = (sl == 2'd0) ? 4'd2 : 4'd4;
          j.suf = (sl == 2'd0) ? 2'd3 : 2'd0; j.dst = V_H1;
        end
      end
      OP_HPK: begin
        if (sl == 2'd2) begin ph512 = 1'b1; bpk = 1'b1; end
        else if (!e && ji == 3'd0) begin
          j.v = 1'b1; j.pk = 1'b1; j.suf = (sl == 2'd0) ? 2'd3 : 2'd0; j.dst = V_H1;
        end
      end
      OP_HASH2: begin
        if (sl == 2'd2) begin ph1024 = 1'b1; bn0 = 4'd8; bn1 = 4'd8; code = 2'd2; end
        else if (sl == 2'd1) begin ph512 = 1'b1; bn0 = 4'd4; bn1 = 4'd4; end
        else if (!e && ji == 3'd0) begin
          j.v = 1'b1; j.s0 = V_M; j.n0 = 4'd2; j.s1 = V_HK; j.n1 = 4'd2; j.dst = V_H1;
        end
      end
      default: ;
    endcase
    if (ph512 || ph1024) begin
      // job 0 of both engines: the two long chains over the message
      if (ji == 3'd0) begin
        j.v = 1'b1; j.pk = bpk; j.s0 = bs0; j.n0 = bn0; j.s1 = bs1; j.n1 = bn1;
        if (!e) begin j.vsel = VS_K1; j.p64 = 1'b1; j.pre = code; j.dst = V_SELF; end
        else    begin j.vsel = VS_IV; j.suf = code; j.dst = V_H1; end
      end
    end
    if (ph512 && ji != 3'd0) begin
      if (!e) begin
        if (ji == 3'd1) begin j.v = 1'b1; j.vsel = VS_K2; j.p64 = 1'b1; j.s0 = V_SELF; j.n0 = 4'd4; j.dst = V_K1; end
      end else begin
        if (ji == 3'd1) begin j.v = 1'b1; j.s0 = V_K1; j.n0 = 4'd4; j.s1 = V_H1; j.n1 = 4'd4; j.dst = V_H2; end
      end
    end
    if (ph1024 && ji != 3'd0) begin
      if (!e) begin
        case (ji)
          3'd1: begin j.v = 1'b1; j.vsel = VS_K2; j.p64 = 1'b1; j.s0 = V_SELF; j.n0 = 4'd4; j.dst = V_K1; end
          3'd2: begin j.v = 1'b1; j.s0 = V_K1; j.n0 = 4'd4; j.s1 = V_H1; j.n1 = 4'd4; j.dst = V_H2; end
          3'd3: begin j.v = 1'b1; j.s0 = V_H2; j.n0 = 4'd4; j.s1 = V_K2; j.n1 = 4'd4; j.dst = V_H3; end
          3'd4: begin j.v = 1'b1; j.vsel = VS_K1; j.p64 = 1'b1; j.s0 = V_H3; j.n0 = 4'd4; j.dst = V_SELF; end
          3'd5: begin j.v = 1'b1; j.vsel = VS_K2; j.p64 = 1'b1; j.s0 = V_SELF; j.n0 = 4'd4; j.dst = V_K4; end
          default: ;
        endcase
      end else begin
        case (ji)
          3'd1: begin j.v = 1'b1; j.vsel = VS_K1; j.p64 = 1'b1; j.s0 = V_H1; j.n0 = 4'd4; j.dst = V_SELF; end
          3'd2: begin j.v = 1'b1; j.vsel = VS_K2; j.p64 = 1'b1; j.s0 = V_SELF; j.n0 = 4'd4; j.dst = V_K2; end
          3'd3: begin j.v = 1'b1; j.vsel = VS_K1; j.p64 = 1'b1; j.s0 = V_H2; j.n0 = 4'd4; j.dst = V_SELF; end
          3'd4: begin j.v = 1'b1; j.vsel = VS_K2; j.p64 = 1'b1; j.s0 = V_SELF; j.n0 = 4'd4; j.dst = V_K3; end
          3'd5: begin j.v = 1'b1; j.s0 = V_K3; j.n0 = 4'd4; j.s1 = V_H3; j.n1 = 4'd4; j.s2 = V_K4; j.n2 = 4'd4;
                      j.dst = V_H4; end
          default: ;
        endcase
      end
    end
    return j;
  endfunction

  function automatic logic [255:0] vconst(input logic [1:0] vs);
    case (vs)
      VS_K1:   vconst = K1M;
      VS_K2:   vconst = K2M;
      default: vconst = IVB;
    endcase
  endfunction

  // KDF message block: seed || nonce || ctr_be32 || padding (S = 16, 32) or
  // the second block nonce || ctr_be32 || padding after the 64-byte seed (S = 64)
  function automatic logic [511:0] kdfblk(input logic [1:0] sl, input logic [511:0] seed,
                                          input logic [7:0] nonce, input logic [7:0] ctr);
    logic [511:0] b;
    b = '0;
    case (sl)
      2'd0: begin
        b[127:0] = seed[127:0]; b[8*16 +: 8] = nonce; b[8*20 +: 8] = ctr;
        b[8*21 +: 8] = 8'h80; b[8*63 +: 8] = 8'ha8;
      end
      2'd1: begin
        b[255:0] = seed[255:0]; b[8*32 +: 8] = nonce; b[8*36 +: 8] = ctr;
        b[8*37 +: 8] = 8'h80; b[8*62 +: 8] = 8'h01; b[8*63 +: 8] = 8'h28;
      end
      default: begin
        b[7:0] = nonce; b[8*4 +: 8] = ctr;
        b[8*5 +: 8] = 8'h80; b[8*62 +: 8] = 8'h02; b[8*63 +: 8] = 8'h28;
      end
    endcase
    return b;
  endfunction

  // ---------------------------------------------------------------- registers
  logic [511:0]  m_q, ks_q, hk_q;
  logic [1023:0] kr_q;
  logic [255:0]  t0_q, t1_q;
  logic [10:0]   vflag;       // value valid flags (index = value id)
  logic          busy_q, done_q;
  logic [2:0]    op_q;
  logic [3:0]    rcnt;
  logic          rnd_on;

  logic          accept;
  assign accept = cmd_valid && !busy_q;

  logic is_chain;
  assign is_chain = busy_q && (op_q == OP_SEED || op_q == OP_HPK || op_q == OP_HASH2);

  // ---------------------------------------------------------------- SM3 cores
  logic         c_start   [NC];
  logic         c_use_ext [NC];
  logic [255:0] c_vext    [NC];
  logic [511:0] c_blk     [NC];
  logic         c_busy    [NC];
  logic         c_done    [NC];
  logic [255:0] c_dig     [NC];

  genvar gi;
  generate
    for (gi = 0; gi < NC; gi++) begin : g_core
      nev_sm3_core u_core (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (c_start[gi]),
        .use_ext (c_use_ext[gi]),
        .v_ext   (c_vext[gi]),
        .blk     (c_blk[gi]),
        .busy    (c_busy[gi]),
        .done    (c_done[gi]),
        .dig     (c_dig[gi])
      );
    end
  endgenerate

  // ---------------------------------------------------------------- chain engines
  logic [2:0]   ji      [2];
  logic         fin     [2];
  jd_t          jd_cur  [2];
  jd_t          jd_nxt  [2];
  jd_t          jd_go   [2];
  logic         e_start [2];
  logic         e_idle  [2];
  logic         e_jdone [2];
  logic [3:0]   e_rqsrc [2];
  logic [2:0]   e_rqoff [2];
  logic [63:0]  e_rqword[2];
  logic [255:0] e_rqhalf[2];
  logic         e_rqok  [2];
  logic         e_pkrdy [2];
  logic [63:0]  e_pkword[2];
  logic         e_pkpop [2];
  logic         e_cstart[2];
  logic         e_cuse  [2];
  logic [511:0] e_cblk  [2];
  logic         fin_nx  [2];
  logic [255:0] e_q     [2];

  // OUT register of the running command (KS / HK / KR)
  logic [1023:0] out_cur;
  always_comb begin
    case (op_q)
      OP_SEED: out_cur = {512'd0, ks_q};
      OP_HPK:  out_cur = {512'd0, hk_q};
      default: out_cur = kr_q;
    endcase
  end

  // PK fetch FIFO (shared storage, one read pointer per engine)
  logic [63:0] pf_mem [4];
  logic [2:0]  pf_wp;
  logic [2:0]  pf_rp [2];

  jd_t  jd_acc [2];
  logic acc_chain;
  always_comb begin
    acc_chain = accept && (cmd_op == OP_SEED || cmd_op == OP_HPK || cmd_op == OP_HASH2);
    for (int e = 0; e < 2; e++) begin
      jd_cur[e] = jdesc(op_q, slog, e[0], ji[e]);
      jd_nxt[e] = jdesc(op_q, slog, e[0], ji[e] + 3'd1);
      jd_acc[e] = jdesc(cmd_op, slog, e[0], 3'd0);
    end
  end

  generate
    for (gi = 0; gi < 2; gi++) begin : g_eng
      // source word fetch
      always_comb begin
        e_q[gi]    = '0;
        e_rqok[gi] = 1'b1;
        case (e_rqsrc[gi])
          V_M: begin
            e_rqword[gi] = m_q[64*e_rqoff[gi] +: 64];
            e_rqhalf[gi] = m_q[256*e_rqoff[gi][2] +: 256];
          end
          V_HK: begin
            e_rqword[gi] = hk_q[64*e_rqoff[gi] +: 64];
            e_rqhalf[gi] = hk_q[256*e_rqoff[gi][2] +: 256];
          end
          default: begin
            case (e_rqsrc[gi])
              V_SELF:      e_q[gi] = c_dig[gi];
              V_K1, V_K3:  e_q[gi] = t0_q;
              V_K2, V_K4:  e_q[gi] = t1_q;
              V_H1:        e_q[gi] = out_cur[255:0];
              V_H2:        e_q[gi] = out_cur[511:256];
              V_H3:        e_q[gi] = out_cur[767:512];
              default:     e_q[gi] = out_cur[1023:768];
            endcase
            e_rqword[gi] = e_q[gi][64*e_rqoff[gi][1:0] +: 64];
            e_rqhalf[gi] = e_q[gi];
            if (e_rqsrc[gi] != V_SELF) e_rqok[gi] = vflag[e_rqsrc[gi]];
          end
        endcase
      end

      assign e_pkrdy[gi]  = (pf_wp != pf_rp[gi]);
      assign e_pkword[gi] = pf_mem[pf_rp[gi][1:0]];

      always_comb begin
        e_start[gi] = 1'b0;
        jd_go[gi]   = jd_cur[gi];
        if (acc_chain) begin
          // job 0 is loaded on the accept edge
          jd_go[gi]   = jd_acc[gi];
          e_start[gi] = jd_acc[gi].v;
        end else if (is_chain && !fin[gi]) begin
          if (e_jdone[gi]) begin
            jd_go[gi]   = jd_nxt[gi];
            e_start[gi] = jd_nxt[gi].v;
          end else if (e_idle[gi]) begin
            e_start[gi] = jd_cur[gi].v;
          end
        end
        fin_nx[gi] = fin[gi] || (e_jdone[gi] && !jd_nxt[gi].v) || (e_idle[gi] && !jd_cur[gi].v);
      end

      nev_hash_ce u_ce (
        .clk          (clk),
        .rst_n        (rst_n),
        .job_start    (e_start[gi]),
        .d_p64        (jd_go[gi].p64),
        .d_pre        (jd_go[gi].pre),
        .d_suf        (jd_go[gi].suf),
        .d_pk         (jd_go[gi].pk),
        .d_src0       (jd_go[gi].s0),
        .d_nw0        (jd_go[gi].n0),
        .d_src1       (jd_go[gi].s1),
        .d_nw1        (jd_go[gi].n1),
        .d_src2       (jd_go[gi].s2),
        .d_nw2        (jd_go[gi].n2),
        .pb           (poly_bytes),
        .idle         (e_idle[gi]),
        .job_done     (e_jdone[gi]),
        .rq_src       (e_rqsrc[gi]),
        .rq_off       (e_rqoff[gi]),
        .rq_word      (e_rqword[gi]),
        .rq_half      (e_rqhalf[gi]),
        .rq_ok        (e_rqok[gi]),
        .pk_rdy       (e_pkrdy[gi]),
        .pk_word      (e_pkword[gi]),
        .pk_pop       (e_pkpop[gi]),
        .core_start   (e_cstart[gi]),
        .core_use_ext (e_cuse[gi]),
        .core_blk     (e_cblk[gi]),
        .core_busy    (c_busy[gi]),
        .core_done    (c_done[gi])
      );
    end
  endgenerate

  // ---------------------------------------------------------------- PK fetcher
  logic [8:0]  pf_addr;
  logic [2:0]  pf_out;
  logic [9:0]  pf_nbw;
  logic [2:0]  pf_occ0, pf_occ1, pf_occ;
  always_comb begin
    pf_nbw  = 10'(({1'b0, poly_bytes} + 13'd7) >> 3);
    pf_occ0 = pf_wp - pf_rp[0];
    pf_occ1 = pf_wp - pf_rp[1];
    pf_occ  = (slog == 2'd2 && pf_occ1 > pf_occ0) ? pf_occ1 : pf_occ0;
    om_req  = busy_q && (op_q == OP_HPK) && ({1'b0, pf_addr} < pf_nbw) && (pf_addr < pk_avail) &&
              ((pf_out + pf_occ) < 3'd4);
    om_we    = 1'b0;
    om_sel   = 2'd1;
    om_addr  = pf_addr;
    om_wdata = 64'd0;
    om_be    = 8'd0;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pf_addr  <= 9'd0;
      pf_out   <= 3'd0;
      pf_wp    <= 3'd0;
      pf_rp[0] <= 3'd0;
      pf_rp[1] <= 3'd0;
      for (int k = 0; k < 4; k++) pf_mem[k] <= 64'd0;
    end else begin
      if (accept && cmd_op == OP_HPK) begin
        pf_addr  <= 9'd0;
        pf_wp    <= 3'd0;
        pf_rp[0] <= 3'd0;
        pf_rp[1] <= 3'd0;
      end else begin
        if (om_req && om_gnt) pf_addr <= pf_addr + 9'd1;
        if (om_rvalid) begin
          pf_mem[pf_wp[1:0]] <= om_rdata;
          pf_wp <= pf_wp + 3'd1;
        end
        if (e_pkpop[0]) pf_rp[0] <= pf_rp[0] + 3'd1;
        if (e_pkpop[1]) pf_rp[1] <= pf_rp[1] + 3'd1;
      end
      pf_out <= pf_out + ((om_req && om_gnt) ? 3'd1 : 3'd0) - (om_rvalid ? 3'd1 : 3'd0);
    end
  end

  // ---------------------------------------------------------------- KDF control
  // Two counter streams share the NC-core farm: stream 0 (KDF-buffer region 0, nonce kdf_nonce,
  // kdf_nblocks blocks) and stream 1 (region 1, nonce kdf_nonce1, kdf_nblocks1 blocks, noisem length
  // kdf_nmlen accumulated into c0). Batches of up to NC blocks are launched together (stream 0 blocks
  // first); the digests of a finished batch are written two rows per block while the next batch runs.
  // Per-stream watermarks count written blocks. MORE (or kdf_need while running) extends stream 0.
  localparam logic [1:0] KD_IDLE = 2'd0;
  localparam logic [1:0] KD_CHK  = 2'd1;
  localparam logic [1:0] KD_MID  = 2'd2;
  localparam logic [1:0] KD_RUN  = 2'd3;

  logic [1:0]   kd_st;
  logic         kd_sel;
  logic [7:0]   kd_nonce0, kd_nonce1;
  logic [11:0]  kd_nmlen;
  logic [7:0]   kd_tgt0, kd_tgt1;     // last ctr of each stream (0: none)
  logic [7:0]   kd_nx0, kd_nx1;       // next ctr to launch
  logic         kd_infl;
  logic [4:0]   kd_bn;                // blocks in the batch in flight
  logic         kd_bs  [NC];          // stream of core i in the batch in flight
  logic [6:0]   kd_bc  [NC];          // ctr of core i
  logic [7:0]   kbcnt0, kbcnt1;
  logic         more_pend;
  logic [255:0] mid_q;
  logic         mid_valid, mid_sel;
  // writer
  logic         w_act;
  logic [4:0]   w_n;
  logic [3:0]   w_i;
  logic         w_h;
  logic         w_bs   [NC];
  logic [6:0]   w_bc   [NC];
  logic         kb_we_q;
  logic [7:0]   kb_waddr_q;
  logic [127:0] kb_wdata_q;
  logic         inc0_q, inc1_q;
  logic         acc_en_q;
  logic [1:0]   acc_seg_q;
  logic [15:0]  acc_mask_q;
  logic [511:0] acc_q;

  logic         kdf_mode;
  assign kdf_mode = busy_q && (op_q == OP_KDF || op_q == OP_MORE);

  logic [511:0] kd_seed;
  always_comb begin
    case (slog)
      2'd0:    kd_seed = kd_sel ? {384'd0, kr_q[255:128]}  : {384'd0, ks_q[127:0]};
      2'd1:    kd_seed = kd_sel ? {256'd0, kr_q[511:256]}  : {256'd0, ks_q[255:0]};
      default: kd_seed = kd_sel ? kr_q[1023:512]           : ks_q;
    endcase
  end

  logic       kd_need_mid, kd_launch, kd_pend;
  logic [7:0] kd_rem0, kd_rem1;
  logic [4:0] kd_n0, kd_n;
  logic       ls   [NC];
  logic [7:0] lc   [NC];
  always_comb begin
    kd_need_mid = (slog == 2'd2) && !(mid_valid && mid_sel == kd_sel);
    kd_rem0     = (kd_nx0 <= kd_tgt0) ? (kd_tgt0 - kd_nx0 + 8'd1) : 8'd0;
    kd_rem1     = (kd_nx1 <= kd_tgt1) ? (kd_tgt1 - kd_nx1 + 8'd1) : 8'd0;
    kd_n0       = (kd_rem0 > 8'(NC)) ? 5'(NC) : kd_rem0[4:0];
    kd_n        = ((kd_rem0 + kd_rem1) > 8'(NC)) ? 5'(NC) : 5'(kd_rem0 + kd_rem1);
    kd_pend     = (kd_rem0 != 8'd0) || (kd_rem1 != 8'd0);
    kd_launch   = kdf_mode && kd_pend &&
                  ((kd_st == KD_RUN && (!kd_infl || c_done[0])) ||
                   (kd_st == KD_CHK && !kd_need_mid) ||
                   (kd_st == KD_MID && c_done[0]));
    for (int i = 0; i < NC; i++) begin
      ls[i] = (5'(i) >= kd_n0);
      lc[i] = ls[i] ? (kd_nx1 + 8'(i) - {3'd0, kd_n0}) : (kd_nx0 + 8'(i));
    end
  end

  // writer row
  logic [7:0]   w_ridx;
  logic [127:0] w_row;
  logic [255:0] w_dig;
  logic [15:0]  w_mask;
  logic [11:0]  w_x0;
  logic         w_s;
  always_comb begin
    w_s    = w_bs[w_i];
    w_ridx = {w_bc[w_i] - 7'd1, w_h};
    w_dig  = c_dig[w_i];
    w_row  = w_h ? w_dig[255:128] : w_dig[127:0];
    w_x0   = {w_ridx, 4'd0};
    for (int l = 0; l < 16; l++)
      w_mask[l] = w_s && (({1'b0, w_x0} + 13'(l)) < {1'b0, kd_nmlen});
  end

  // core input muxing
  always_comb begin
    for (int i = 0; i < NC; i++) begin
      c_start[i]   = 1'b0;
      c_use_ext[i] = 1'b1;
      // midstate: register while running, straight from core 0 on the cycle it completes
      if (slog == 2'd2 && (kd_st == KD_RUN || kd_st == KD_CHK)) c_vext[i] = mid_q;
      else if (slog == 2'd2 && kd_st == KD_MID && c_done[0]) c_vext[i] = c_dig[0];
      else                                               c_vext[i] = IVB;
      c_blk[i]     = kdfblk(slog, kd_seed, ls[i] ? kd_nonce1 : kd_nonce0, lc[i]);
      if (kdf_mode) begin
        if (kd_launch && (5'(i) < kd_n)) c_start[i] = 1'b1;
      end
    end
    if (kdf_mode) begin
      if (kd_st == KD_CHK && kd_need_mid) begin
        c_start[0] = 1'b1;
        c_vext[0]  = IVB;
        c_blk[0]   = kd_seed;
      end
    end else begin
      for (int i = 0; i < 2; i++) begin
        c_start[i]   = e_cstart[i];
        c_use_ext[i] = e_cuse[i];
        c_vext[i]    = vconst(jd_cur[i].vsel);
        c_blk[i]     = e_cblk[i];
      end
    end
  end

  logic more_go;
  // extend stream 0 only when the sampler is starved after every stream-0 block has been written
  assign more_go = kdf_mode && kdf_need && !more_pend && (kd_rem0 == 8'd0) && (kd_tgt0 != 8'd0) && (kbcnt0 == kd_tgt0);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      kd_st       <= KD_IDLE;
      kd_sel      <= 1'b0;
      kd_nonce0   <= 8'd0;
      kd_nonce1   <= 8'd0;
      kd_nmlen    <= 12'd0;
      kd_tgt0     <= 8'd0;
      kd_tgt1     <= 8'd0;
      kd_nx0      <= 8'd1;
      kd_nx1      <= 8'd1;
      kd_infl     <= 1'b0;
      kd_bn       <= 5'd0;
      for (int i = 0; i < NC; i++) begin kd_bs[i] <= 1'b0; kd_bc[i] <= 7'd0; w_bs[i] <= 1'b0; w_bc[i] <= 7'd0; end
      kbcnt0      <= 8'd0;
      kbcnt1      <= 8'd0;
      more_pend   <= 1'b0;
      mid_q       <= '0;
      mid_valid   <= 1'b0;
      mid_sel     <= 1'b0;
      w_act       <= 1'b0;
      w_n         <= 5'd0;
      w_i         <= 4'd0;
      w_h         <= 1'b0;
      kb_we_q     <= 1'b0;
      kb_waddr_q  <= 8'd0;
      kb_wdata_q  <= '0;
      inc0_q      <= 1'b0;
      inc1_q      <= 1'b0;
      acc_en_q    <= 1'b0;
      acc_seg_q   <= 2'd0;
      acc_mask_q  <= 16'd0;
      acc_q       <= '0;
    end else begin
      if (accept && (cmd_op == OP_SEED || cmd_op == OP_HASH2)) mid_valid <= 1'b0;
      if (accept && cmd_op == OP_KDF) begin
        kd_st       <= KD_CHK;
        kd_sel      <= kdf_seed_sel;
        kd_nonce0   <= kdf_nonce;
        kd_nonce1   <= kdf_nonce1;
        kd_nmlen    <= kdf_nmlen;
        kd_tgt0     <= {1'b0, kdf_nblocks};
        kd_tgt1     <= {1'b0, kdf_nblocks1};
        kd_nx0      <= 8'd1;
        kd_nx1      <= 8'd1;
        kbcnt0      <= 8'd0;
        kbcnt1      <= 8'd0;
        more_pend   <= 1'b0;
        acc_q       <= '0;
      end else if (accept && cmd_op == OP_MORE) begin
        kd_st   <= KD_CHK;
        kd_tgt0 <= kd_tgt0 + 8'd1;
      end else if (kdf_mode) begin
        case (kd_st)
          KD_CHK: begin
            kd_st     <= kd_need_mid ? KD_MID : KD_RUN;
          end
          KD_MID: begin
            if (c_done[0]) begin
              mid_q     <= c_dig[0];
              mid_valid <= 1'b1;
              mid_sel   <= kd_sel;
              kd_st     <= KD_RUN;
            end
          end
          default: ;
        endcase
        if (more_go) begin kd_tgt0 <= kd_tgt0 + 8'd1; more_pend <= 1'b1; end
      end
      if (!kdf_mode && !(accept && (cmd_op == OP_KDF || cmd_op == OP_MORE))) kd_st <= KD_IDLE;
      if (more_pend && kbcnt0 == kd_tgt0) more_pend <= 1'b0;

      // batch launch / completion
      if (kd_launch) begin
        kd_infl  <= 1'b1;
        kd_bn    <= kd_n;
        for (int i = 0; i < NC; i++) begin kd_bs[i] <= ls[i]; kd_bc[i] <= lc[i][6:0]; end
        kd_nx0   <= kd_nx0 + {3'd0, kd_n0};
        if (kd_n > kd_n0) kd_nx1 <= kd_nx1 + {3'd0, kd_n - kd_n0};
      end else if (kd_infl && c_done[0]) begin
        kd_infl <= 1'b0;
      end
      if (kd_infl && c_done[0]) begin
        w_act  <= 1'b1;
        w_n    <= kd_bn;
        w_i    <= 4'd0;
        w_h    <= 1'b0;
        for (int i = 0; i < NC; i++) begin w_bs[i] <= kd_bs[i]; w_bc[i] <= kd_bc[i]; end
      end else if (w_act) begin
        w_h <= ~w_h;
        if (w_h) begin
          if ({1'b0, w_i} == w_n - 5'd1) w_act <= 1'b0;
          else                            w_i   <= w_i + 4'd1;
        end
      end

      // KDF buffer write (registered outputs) and accumulator / watermarks
      kb_we_q  <= w_act;
      inc0_q   <= w_act && w_h && !w_s;
      inc1_q   <= w_act && w_h && w_s;
      acc_en_q <= w_act;
      if (w_act) begin
        kb_waddr_q <= {w_s, w_ridx[6:0]};
        kb_wdata_q <= w_row;
        acc_mask_q <= w_mask;
        case (slog)
          2'd0:    acc_seg_q <= 2'd0;
          2'd1:    acc_seg_q <= {1'b0, w_ridx[0]};
          default: acc_seg_q <= w_ridx[1:0];
        endcase
      end
      if (acc_en_q) begin
        for (int l = 0; l < 16; l++)
          if (acc_mask_q[l]) acc_q[128*acc_seg_q + 8*l +: 8] <= acc_q[128*acc_seg_q + 8*l +: 8] ^ kb_wdata_q[8*l +: 8];
      end
      if (!(accept && cmd_op == OP_KDF)) begin
        if (inc0_q) kbcnt0 <= kbcnt0 + 8'd1;
        if (inc1_q) kbcnt1 <= kbcnt1 + 8'd1;
      end
    end
  end

  // ---------------------------------------------------------------- command FSM and registers
  logic       chain_fin;
  logic [6:0] sb;
  assign sb        = 7'd16 << slog;
  assign chain_fin = fin_nx[0] && fin_nx[1];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy_q <= 1'b0;
      done_q <= 1'b0;
      op_q   <= 3'd0;
      rcnt   <= 4'd0;
      rnd_on <= 1'b0;
      rnd_fin <= 1'b0;
      m_q    <= '0;
      ks_q   <= '0;
      hk_q   <= '0;
      kr_q   <= '0;
      t0_q   <= '0;
      t1_q   <= '0;
      vflag  <= '0;
      ji[0]  <= 3'd0;
      ji[1]  <= 3'd0;
      fin[0] <= 1'b0;
      fin[1] <= 1'b0;
    end else begin
      done_q <= 1'b0;
      if (m_we) m_q <= m_wdata;
      // random-byte capture (independent of the job engine)
      if (rnd_go) begin
        rnd_on <= 1'b1; rnd_fin <= 1'b0; rcnt <= 4'd0;
      end else if (rnd_req && rnd_valid) begin
        m_q[64*rcnt[2:0] +: 64] <= rnd_data;
        rcnt <= rcnt + 4'd1;
        if ({rcnt + 4'd1, 3'b000} == sb) begin rnd_on <= 1'b0; rnd_fin <= 1'b1; end
      end
      if (accept) begin
        busy_q <= 1'b1;
        op_q   <= cmd_op;
        vflag  <= '0;
        ji[0]  <= 3'd0;
        ji[1]  <= 3'd0;
        fin[0] <= 1'b0;
        fin[1] <= 1'b0;
      end else if (busy_q) begin
        case (op_q)
          OP_LOADHK: begin
            hk_q   <= hpk_field;
            busy_q <= 1'b0;
            done_q <= 1'b1;
          end
          OP_SEED, OP_HPK, OP_HASH2: begin
            for (int e = 0; e < 2; e++) begin
              if (e_jdone[e]) begin
                ji[e] <= ji[e] + 3'd1;
                if (jd_cur[e].dst != V_SELF) vflag[jd_cur[e].dst] <= 1'b1;
                case (jd_cur[e].dst)
                  V_K1, V_K3: t0_q <= c_dig[e];
                  V_K2, V_K4: t1_q <= c_dig[e];
                  V_H1, V_H2, V_H3, V_H4: begin
                    case (op_q)
                      OP_SEED: begin
                        if (slog == 2'd0)              ks_q[127:0] <= c_dig[e][127:0];
                        else if (jd_cur[e].dst == V_H1) ks_q[255:0] <= c_dig[e];
                        else                           ks_q[511:256] <= c_dig[e];
                      end
                      OP_HPK: begin
                        if (slog == 2'd0)              hk_q[127:0] <= c_dig[e][127:0];
                        else if (jd_cur[e].dst == V_H1) hk_q[255:0] <= c_dig[e];
                        else                           hk_q[511:256] <= c_dig[e];
                      end
                      default: kr_q[256*(jd_cur[e].dst - V_H1) +: 256] <= c_dig[e];
                    endcase
                  end
                  default: ;
                endcase
              end
              fin[e] <= fin_nx[e];
            end
            if (chain_fin) begin
              busy_q <= 1'b0;
              done_q <= 1'b1;
            end
          end
          OP_KDF, OP_MORE: begin
            if (kd_st == KD_RUN && !kd_pend && !kd_infl && !w_act && !inc0_q && !inc1_q &&
                !more_pend && !more_go) begin
              busy_q <= 1'b0;
              done_q <= 1'b1;
            end
          end
          default: begin
            busy_q <= 1'b0;
            done_q <= 1'b1;
          end
        endcase
      end
    end
  end

  // ---------------------------------------------------------------- outputs
  always_comb begin
    busy      = busy_q;
    done      = done_q;
    rnd_req   = rnd_on && ({rcnt, 3'b000} < sb);
    kb_we     = kb_we_q;
    kb_waddr  = kb_waddr_q;
    kb_wdata  = kb_wdata_q;
    kb_wm0    = kbcnt0;
    kb_wm1    = kbcnt1;
    kdf_done1 = (kd_tgt1 != 8'd0) && (kbcnt1 == kd_tgt1);
    kr        = kr_q;
    hk        = hk_q;
    m_reg     = m_q;
    for (int b = 0; b < 64; b++)
      c0[8*b +: 8] = (b < sb) ? (m_q[8*b +: 8] ^ acc_q[8*b +: 8]) : 8'd0;
  end

endmodule
