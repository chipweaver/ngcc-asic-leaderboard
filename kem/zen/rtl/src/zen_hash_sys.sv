// Hash subsystem: 8 SM3 cores (2 rounds/cycle, chained blocks 32 cycles, zen_sm3_core),
// pseudoXOF digest dispatch (sampler port, 16 tags), two job-chain executors (A: cores 0/1 bulk jobs H(pk)
// and implicit-rejection hash; B: cores 2/3 pseudohash kr chain + XOF seed midstate, or the KeyGen seed
// midstate), SK hpk/z copier, byte registers and SS.
// Scheduling:
//  * blocks of a multi-block job are chained (next block presented in the core's last round cycle);
//  * a dependent job starts in the cycle its last dependency completes (digest bypass tmp_eff);
//  * the kr chain may be armed before H(pk) / m are available: its blocks wait for hpk_ok / m_ok; for
//    ZEN_512 the m-only first blocks of K1IN and H1 run during H(pk); h4's first block (k3||h3) does not
//    wait for k4;
//  * the XOF seed midstate is the last job of the kr chain; XOF requests are accepted once the
//    midstate of the current command is valid (smid_ok), so the sampler may be started early.
// Byte strings are stored little-endian packed (byte j at [8j+7:8j]); SM3 blocks/digests use
// the SM3 big-endian word packing (word 0 at the MSBs).
module zen_hash_sys (
  input  logic          clk,
  input  logic          rst_n,
  input  logic [1:0]    param,
  input  logic          clr,               // command accepted: clear task state
  // ---- tasks (1-cycle start pulses; done pulses)
  input  logic          t_hpk_start,       // H(pk) of MEM_PK          -> hpk
  input  logic          t_rej_start,       // implicit-rejection hash  -> ssp
  input  logic          t_kr_start,        // kr = pseudohash(m || hpk[:sym]) followed by the XOF seed midstate
  input  logic          t_smid_start,      // KeyGen: XOF seed midstate of the rnd seed register
  input  logic          t_skld_start,      // hpk,z  <- MEM_SK
  input  logic          t_skst_start,      // MEM_SK <- hpk,z
  input  logic          m_ok,              // m register complete (Enc: random bytes received)
  output logic          t_hpk_done, t_rej_done, t_kr_done, t_smid_done, t_skld_done, t_skst_done,
  // ---- register writes
  input  logic          rnd_we,
  input  logic [1:0]    rnd_reg,           // 0 seed, 1 z, 2 m
  input  logic [2:0]    rnd_idx,           // 8-byte beat index
  input  logic [63:0]   rnd_data,
  input  logic          m_load,            // m <- mload (Dec m')
  input  logic [511:0]  mload,
  input  logic          ss_load,
  input  logic          ss_sel_rej,        // 0: kr[0:ss], 1: ssp
  // ---- outputs
  output logic [511:0]  m_reg,
  output logic [511:0]  ss_reg,
  // ---- pseudoXOF port (sampler)
  input  logic [7:0]    xof_nonce,
  input  logic          xreq_valid,
  output logic          xreq_ready,
  input  logic [15:0]   xreq_ctr,
  input  logic [3:0]    xreq_tag,
  output logic          xrsp_valid,
  output logic [3:0]    xrsp_tag,
  output logic [255:0]  xrsp_digest,       // byte 0 at [255:248]
  // ---- object memory port B
  input  logic [10:0]   pk_limit,          // MEM_PK words readable by the bulk reader
  output logic          pkb_en,
  output logic [9:0]    pkb_addr,
  input  logic [63:0]   pkb_rdata,
  output logic          skb_en, skb_we,
  output logic [9:0]    skb_addr,
  output logic [63:0]   skb_wdata,
  output logic [7:0]    skb_wmask,
  input  logic [63:0]   skb_rdata,
  output logic          ctb_en,
  output logic [9:0]    ctb_addr,
  input  logic [63:0]   ctb_rdata,
  // ---- phase observation
  output logic [7:0]    core_busy,
  output logic [7:0]    core_start,
  output logic [7:0]    core_done,
  output logic          act_hpk, act_rej, act_kr, act_smid, act_xof, act_sk
);
  // ------------------------------------------------------------ constants
  localparam logic [255:0] SM3_IV = 256'h7380166F4914B2B9172442D7DA8A0600A96F30BC163138AAE38DEE4DB0FB0E4E;
  `include "zen_hash_consts.svh"

  logic [6:0]  sym;
  logic [12:0] pk_len, ct_len;
  always_comb begin
    case (param)
      2'd0:    begin sym = 7'd16; pk_len = 13'd615;  ct_len = 13'd512;  end
      2'd1:    begin sym = 7'd32; pk_len = 13'd1229; ct_len = 13'd1024; end
      default: begin sym = 7'd64; pk_len = 13'd2458; ct_len = 13'd2048; end
    endcase
  end

  function automatic logic [511:0] le2blk(input logic [511:0] le);
    for (int j = 0; j < 64; j++) le2blk[511 - 8*j -: 8] = le[8*j +: 8];
  endfunction
  function automatic logic [255:0] dig2le(input logic [255:0] d);
    for (int j = 0; j < 32; j++) dig2le[8*j +: 8] = d[255 - 8*j -: 8];
  endfunction

  // ------------------------------------------------------------ SM3 cores
  logic [7:0]          c_start, c_busy, c_done, c_cont, c_lastc;
  logic [7:0]          x_own, x_pend, x_wt;     // XOF: core running / response pending / claimed, waiting for the seed midstate
  logic [7:0][255:0]   c_iv, c_vout;
  logic [7:0][511:0]   c_blk;
  genvar gc;
  generate
    for (gc = 0; gc < 8; gc++) begin : g_core
      zen_sm3_core u_core (.clk(clk), .rst_n(rst_n), .start(c_start[gc]), .cont(c_cont[gc]), .lastc(c_lastc[gc]),
                           .iv(c_iv[gc]), .blk(c_blk[gc]), .busy(c_busy[gc]), .done(c_done[gc]), .vout(c_vout[gc]));
    end
  endgenerate
  assign core_busy  = c_busy;
  // observation: block start events; a chained block starts (first round pair) in its predecessor's done cycle
  logic [7:0] cont_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) cont_q <= '0;
    else        cont_q <= c_cont & c_lastc;
  end
  assign core_start = c_start | cont_q;
  assign core_done  = c_done;

  // ------------------------------------------------------------ job tables
  // fmt: 0 BULK, 1 K1IN, 2 H1, 3 D32, 4 CAT2, 5 CAT3, 6 SEED (rnd seed), 7 KSEED (kr seed -> smid)
  // iv : 0 IV, 1 K1MID, 2 K2MID
  typedef struct packed {
    logic        v;
    logic        core;     // local core 0/1
    logic [1:0]  iv;
    logic [2:0]  fmt;
    logic [3:0]  a, b, c;
    logic [12:0] deps;     // dependencies (job indices) for the first block
    logic [12:0] dep1;     // additional dependencies for blocks >= 1 (CAT3 second block)
    logic [3:0]  out;      // temp index
    logic [1:0]  bcfg;     // bulk config 0..3
  } job_t;

  function automatic job_t mkj(input logic cr, input logic [1:0] iv, input logic [2:0] fmt, input logic [3:0] a,
                               input logic [3:0] b, input logic [3:0] c, input logic [12:0] deps,
                               input logic [12:0] dep1, input logic [3:0] out, input logic [1:0] bcfg);
    job_t r;
    r.v = 1'b1; r.core = cr; r.iv = iv; r.fmt = fmt; r.a = a; r.b = b; r.c = c; r.deps = deps; r.dep1 = dep1;
    r.out = out; r.bcfg = bcfg;
    return r;
  endfunction

  // executor A chains: 0 HPK_SM3, 1 HPK_PH512, 2 REJ_SM3, 3 REJ_XOF512
  function automatic job_t jobA(input logic [1:0] ch, input logic [3:0] j);
    job_t r; r = '0;
    case (ch)
      2'd0: if (j == 0) r = mkj(1'b0, 2'd0, 3'd0, 4'd0, 4'd0, 4'd0, 13'h0, 13'h0, 4'd12, 2'd0);
      2'd1: case (j)
              4'd0: r = mkj(1'b0, 2'd1, 3'd0, 4'd0,  4'd0,  4'd0, 13'h000, 13'h0, 4'd12, 2'd1);
              4'd1: r = mkj(1'b1, 2'd0, 3'd0, 4'd0,  4'd0,  4'd0, 13'h000, 13'h0, 4'd13, 2'd2);
              4'd2: r = mkj(1'b0, 2'd2, 3'd3, 4'd12, 4'd0,  4'd0, 13'h001, 13'h0, 4'd14, 2'd0);
              4'd3: r = mkj(1'b0, 2'd0, 3'd4, 4'd14, 4'd13, 4'd0, 13'h006, 13'h0, 4'd15, 2'd0);
              default: r = '0;
            endcase
      2'd2: if (j == 0) r = mkj(1'b0, 2'd0, 3'd0, 4'd0, 4'd0, 4'd0, 13'h0, 13'h0, 4'd12, 2'd3);
      default: case (j)
              4'd0: r = mkj(1'b0, 2'd0, 3'd0, 4'd0, 4'd0, 4'd0, 13'h0, 13'h0, 4'd12, 2'd3);
              4'd1: r = mkj(1'b1, 2'd0, 3'd0, 4'd0, 4'd0, 4'd0, 13'h0, 13'h0, 4'd13, 2'd3);
              default: r = '0;
            endcase
    endcase
    return r;
  endfunction

  // executor B chains: 0 KR768 + seed midstate, 1 KR1024 + seed midstate, 2 SEEDMID (KeyGen)
  function automatic job_t jobB(input logic [1:0] ch, input logic [3:0] j);
    job_t r; r = '0;
    case (ch)
      2'd0: case (j)
              4'd0: r = mkj(1'b0, 2'd1, 3'd1, 4'd0, 4'd0, 4'd0, 13'h000, 13'h0, 4'd0, 2'd0);   // k1 inner
              4'd1: r = mkj(1'b1, 2'd0, 3'd2, 4'd0, 4'd0, 4'd0, 13'h000, 13'h0, 4'd1, 2'd0);   // h1
              4'd2: r = mkj(1'b0, 2'd2, 3'd3, 4'd0, 4'd0, 4'd0, 13'h001, 13'h0, 4'd2, 2'd0);   // k1
              4'd3: r = mkj(1'b1, 2'd1, 3'd3, 4'd1, 4'd0, 4'd0, 13'h002, 13'h0, 4'd3, 2'd0);   // k2 inner
              4'd4: r = mkj(1'b0, 2'd0, 3'd4, 4'd2, 4'd1, 4'd0, 13'h006, 13'h0, 4'd4, 2'd0);   // h2
              4'd5: r = mkj(1'b1, 2'd2, 3'd3, 4'd3, 4'd0, 4'd0, 13'h008, 13'h0, 4'd5, 2'd0);   // k2
              4'd6: r = mkj(1'b0, 2'd0, 3'd4, 4'd4, 4'd5, 4'd0, 13'h030, 13'h0, 4'd6, 2'd0);   // h3
              4'd7: r = mkj(1'b1, 2'd0, 3'd7, 4'd0, 4'd0, 4'd0, 13'h052, 13'h0, 4'd0, 2'd0);   // seed midstate
              default: r = '0;
            endcase
      2'd1: case (j)
              4'd0:  r = mkj(1'b0, 2'd1, 3'd1, 4'd0, 4'd0, 4'd0,  13'h000, 13'h0,   4'd0,  2'd0);  // k1 inner
              4'd1:  r = mkj(1'b1, 2'd0, 3'd2, 4'd0, 4'd0, 4'd0,  13'h000, 13'h0,   4'd1,  2'd0);  // h1
              4'd2:  r = mkj(1'b0, 2'd2, 3'd3, 4'd0, 4'd0, 4'd0,  13'h001, 13'h0,   4'd2,  2'd0);  // k1
              4'd3:  r = mkj(1'b1, 2'd1, 3'd3, 4'd1, 4'd0, 4'd0,  13'h002, 13'h0,   4'd3,  2'd0);  // k2 inner
              4'd4:  r = mkj(1'b0, 2'd0, 3'd4, 4'd2, 4'd1, 4'd0,  13'h006, 13'h0,   4'd4,  2'd0);  // h2
              4'd5:  r = mkj(1'b1, 2'd2, 3'd3, 4'd3, 4'd0, 4'd0,  13'h008, 13'h0,   4'd5,  2'd0);  // k2
              4'd6:  r = mkj(1'b0, 2'd1, 3'd3, 4'd4, 4'd0, 4'd0,  13'h010, 13'h0,   4'd6,  2'd0);  // k3 inner
              4'd7:  r = mkj(1'b1, 2'd0, 3'd4, 4'd4, 4'd5, 4'd0,  13'h030, 13'h0,   4'd7,  2'd0);  // h3
              4'd8:  r = mkj(1'b0, 2'd2, 3'd3, 4'd6, 4'd0, 4'd0,  13'h040, 13'h0,   4'd8,  2'd0);  // k3
              4'd9:  r = mkj(1'b1, 2'd1, 3'd3, 4'd7, 4'd0, 4'd0,  13'h080, 13'h0,   4'd9,  2'd0);  // k4 inner
              4'd10: r = mkj(1'b1, 2'd2, 3'd3, 4'd9, 4'd0, 4'd0,  13'h200, 13'h0,   4'd10, 2'd0);  // k4
              4'd11: r = mkj(1'b0, 2'd0, 3'd5, 4'd8, 4'd7, 4'd10, 13'h180, 13'h400, 4'd11, 2'd0);  // h4 (k3||h3 | k4)
              4'd12: r = mkj(1'b1, 2'd0, 3'd7, 4'd0, 4'd0, 4'd0,  13'h880, 13'h0,   4'd0,  2'd0);  // seed midstate
              default: r = '0;
            endcase
      default: if (j == 0) r = mkj(1'b0, 2'd0, 3'd6, 4'd0, 4'd0, 4'd0, 13'h0, 13'h0, 4'd0, 2'd0);
    endcase
    return r;
  endfunction

  function automatic logic [5:0] fmt_nb(input logic [2:0] fmt, input logic [1:0] p);
    case (fmt)
      3'd1, 3'd2: fmt_nb = (p == 2'd0) ? 6'd1 : (p == 2'd1) ? 6'd2 : 6'd3;
      3'd4, 3'd5: fmt_nb = 6'd2;
      default:    fmt_nb = 6'd1;
    endcase
  endfunction

  function automatic logic [255:0] ivsel(input logic [1:0] s);
    case (s)
      2'd1:    ivsel = K1MID_C;
      2'd2:    ivsel = K2MID_C;
      default: ivsel = SM3_IV;
    endcase
  endfunction

  // ------------------------------------------------------------ registers
  logic [511:0] seed_reg, z_reg, hpk_reg;
  logic [255:0] smid_reg;
  logic         hpk_valid, smid_valid;
  logic [15:0][255:0] tmp;     // T0..T11 executor B, T12..T15 executor A
  logic [15:0][255:0] tmp_eff; // tmp with this cycle's completing digests bypassed
  logic [15:0]        tmp_wr;
  logic [15:0][255:0] tmp_nv;
  logic sk_ld_commit;
  logic [511:0] sk_hpk, sk_z;
  logic task_hpk, task_rej, task_kr, task_smid;
  assign act_hpk = task_hpk; assign act_rej = task_rej; assign act_kr = task_kr;
  logic smid_run;            // seed midstate job active (KeyGen task, or the last job of the kr chain)
  assign act_smid = task_smid || smid_run;

  always_comb begin
    for (int i = 0; i < 16; i++) tmp_eff[i] = tmp_wr[i] ? tmp_nv[i] : tmp[i];
  end

  // hpk as soon as the final H(pk) job completes (bypass)
  logic         hpk_now;
  logic [511:0] hpk_new, hpk_eff;
  logic         hpk_ok;
  assign hpk_new = (param == 2'd2) ? {tmp_eff[15], tmp_eff[13]} : {256'd0, tmp_eff[12]};
  assign hpk_eff = hpk_now ? hpk_new : hpk_reg;
  assign hpk_ok  = hpk_valid || hpk_now;

  // kr = h1||h2||h3(||h4), from the bypassed temps
  logic [1023:0] kr, kr_r;
  logic [511:0]  kr_seed, kr_ss, ssp;
  always_comb begin
    if (param == 2'd2) kr = {tmp_eff[11], tmp_eff[7], tmp_eff[4], tmp_eff[1]};
    else               kr = {256'd0, tmp_eff[6], tmp_eff[4], tmp_eff[1]};
    if (param == 2'd2) kr_r = {tmp[11], tmp[7], tmp[4], tmp[1]};
    else               kr_r = {256'd0, tmp[6], tmp[4], tmp[1]};
    case (param)
      2'd0:    begin kr_seed = kr[128 +: 512]; kr_ss = {384'd0, kr_r[127:0]}; end
      2'd1:    begin kr_seed = kr[256 +: 512]; kr_ss = {256'd0, kr_r[255:0]}; end
      default: begin kr_seed = kr[512 +: 512]; kr_ss = kr_r[511:0]; end
    endcase
    if (param == 2'd2) ssp = {tmp[13], tmp[12]};
    else if (param == 2'd1) ssp = {256'd0, tmp[12]};
    else ssp = {384'd0, tmp[12][127:0]};
  end

  // kr message = m[0:sym] || hpk[0:sym]
  logic [1023:0] msgx;
  always_comb begin
    case (param)
      2'd0:    msgx = {768'd0, hpk_eff[127:0], m_reg[127:0]};
      2'd1:    msgx = {512'd0, hpk_eff[255:0], m_reg[255:0]};
      default: msgx = {hpk_eff, m_reg};
    endcase
  end

  // ------------------------------------------------------------ formatters (LE byte streams)
  // K1IN: {tag,0,msgx[0:L],80,0..,len} with total bits (64+2+L)*8 ; H1: {msgx[0:L],tag,0,80,...,len}
  logic [7:0]    tag;
  logic [1535:0] s_k1in, s_h1;
  always_comb begin
    logic [12:0] L, tb1, tb2;
    logic [1:0]  nbk;
    tag = (param == 2'd2) ? 8'h04 : 8'h03;
    L   = {5'd0, sym, 1'b0};
    nbk = (param == 2'd0) ? 2'd1 : (param == 2'd1) ? 2'd2 : 2'd3;
    tb1 = (13'd66 + L);         // bytes incl. 64-byte HMAC key block
    tb2 = (13'd2 + L);
    s_k1in = '0; s_h1 = '0;
    for (int i = 0; i < 192; i++) begin
      // K1IN
      if (i == 0)                          s_k1in[8*i +: 8] = tag;
      else if (i == 1)                     s_k1in[8*i +: 8] = 8'h00;
      else if (i < 2 + L)                  s_k1in[8*i +: 8] = msgx[8*((i >= 2 && i < 130) ? (i-2) : 0) +: 8];  // clamp (L <= 128)
      else if (i == 2 + L)                 s_k1in[8*i +: 8] = 8'h80;
      // H1
      if (i < L)                           s_h1[8*i +: 8] = msgx[8*((i < 128) ? i : 0) +: 8];            // clamp (L <= 128)
      else if (i == L)                     s_h1[8*i +: 8] = tag;
      else if (i == L + 1)                 s_h1[8*i +: 8] = 8'h00;
      else if (i == L + 2)                 s_h1[8*i +: 8] = 8'h80;
    end
    // big-endian bit lengths in the last 8 bytes of the final block
    for (int k = 0; k < 8; k++) begin
      s_k1in[8*(64*nbk - 8 + k) +: 8] = (k >= 6) ? (({tb1, 3'd0} >> (8*(7-k))) & 8'hFF) : 8'h00;
      s_h1  [8*(64*nbk - 8 + k) +: 8] = (k >= 6) ? (({tb2, 3'd0} >> (8*(7-k))) & 8'hFF) : 8'h00;
    end
  end

  function automatic logic [511:0] fmt_d32(input logic [255:0] a);   // a || pad, total 96 bytes
    logic [511:0] s;
    s = '0; s[255:0] = a; s[263:256] = 8'h80;
    s[8*62 +: 8] = 8'h03; s[8*63 +: 8] = 8'h00;   // 768 bits = 0x0300
    return le2blk(s);
  endfunction
  function automatic logic [511:0] pad64only(input logic dummy);    // 80 .. len=512
    logic [511:0] s;
    s = '0; s[7:0] = 8'h80; s[8*62 +: 8] = 8'h02; s[8*63 +: 8] = 8'h00;
    return le2blk(s);
  endfunction

  // XOF block 2: nonce || ctr(BE32) || 80 || 0 || len 552
  function automatic logic [511:0] xof_blk(input logic [7:0] nonce, input logic [15:0] ctr);
    logic [511:0] b;
    b = '0;
    b[511:480] = {nonce, 8'h00, ctr};
    b[479:448] = 32'h80000000;
    b[31:0]    = 32'd552;
    // ctr is 32-bit BE: bytes 1..4 = {0,0,ctr_hi,ctr_lo}; byte 5 = 0x80
    b[511:480] = {nonce, 8'h00, 8'h00, ctr[15:8]};
    b[479:448] = {ctr[7:0], 8'h80, 16'h0000};
    return b;
  endfunction


  // ------------------------------------------------------------ executors
  logic        exA_run, exB_run;
  logic [1:0]  exA_ch, exB_ch;
  logic [12:0] exA_started, exA_done, exB_started, exB_done;
  logic [12:0] exA_all, exB_all, exA_dnow, exB_dnow;
  logic [1:0]  exA_cj_v, exB_cj_v, exA_wait, exB_wait, exA_chn, exB_chn;
  logic [1:0][3:0] exA_cj, exB_cj;
  logic [1:0][5:0] exA_blk, exB_blk;

  always_comb begin
    exA_all = '0; exB_all = '0;
    for (int j = 0; j < 13; j++) begin
      job_t ta, tb;
      ta = jobA(exA_ch, j[3:0]); tb = jobB(exB_ch, j[3:0]);
      exA_all[j] = ta.v; exB_all[j] = tb.v;
    end
  end

  // next unstarted job per local core
  logic [1:0][3:0] exA_nj, exB_nj;
  logic [1:0]      exA_nj_v, exB_nj_v;
  always_comb begin
    for (int c = 0; c < 2; c++) begin
      exA_nj_v[c] = 1'b0; exA_nj[c] = '0; exB_nj_v[c] = 1'b0; exB_nj[c] = '0;
      for (int j = 12; j >= 0; j--) begin
        job_t ta, tb;
        ta = jobA(exA_ch, j[3:0]); tb = jobB(exB_ch, j[3:0]);
        if (exA_all[j] && !exA_started[j] && ta.core == c[0]) begin exA_nj_v[c] = 1'b1; exA_nj[c] = j[3:0]; end
        if (exB_all[j] && !exB_started[j] && tb.core == c[0]) begin exB_nj_v[c] = 1'b1; exB_nj[c] = j[3:0]; end
      end
    end
  end

  // bulk assemblers (cores 0/1)
  logic [1:0]        asm_start, asm_mreq, asm_mgnt, asm_bv, asm_last, asm_take, asm_busy;
  logic [1:0][9:0]   asm_maddr;
  logic [1:0][511:0] asm_blk;
  logic [1:0][1:0]   asm_pre;
  logic [1:0][6:0]   asm_zl;
  logic [1:0][12:0]  asm_mlen;
  logic [1:0][2:0]   asm_suf;
  logic [1:0][31:0]  asm_ctr;
  logic [1:0]        asm_pfx;
  logic [1:0]        asm_last_q;
  logic              mem_is_ct;
  logic [63:0]       asm_rdata;
  logic [10:0]       asm_limit;
  generate
    for (gc = 0; gc < 2; gc++) begin : g_asm
      zen_hash_asm u_asm (.clk(clk), .rst_n(rst_n),
        .cfg_start(asm_start[gc]), .cfg_pre(asm_pre[gc]), .cfg_zl(asm_zl[gc]), .cfg_mlen(asm_mlen[gc]),
        .cfg_suf(asm_suf[gc]), .cfg_ctr(asm_ctr[gc]), .cfg_pfx64(asm_pfx[gc]), .zreg(z_reg),
        .mreq(asm_mreq[gc]), .maddr(asm_maddr[gc]), .mgnt(asm_mgnt[gc]), .mrdata(asm_rdata),
        .mlimit(asm_limit),
        .blk_valid(asm_bv[gc]), .blk(asm_blk[gc]), .blk_last(asm_last[gc]), .blk_take(asm_take[gc]),
        .busy(asm_busy[gc]));
    end
  endgenerate

  // bulk configurations: 0 pk plain, 1 {02,00}||pk (K1 prefix), 2 pk||{02,00}, 3 z||ct(||ctr)
  always_comb begin
    for (int c = 0; c < 2; c++) begin
      job_t tj;
      asm_pre[c] = 2'd0; asm_zl[c] = 7'd0; asm_mlen[c] = pk_len; asm_suf[c] = 3'd0;
      asm_ctr[c] = 32'd0; asm_pfx[c] = 1'b0;
      tj = jobA(exA_ch, exA_nj[c]);
      case (tj.bcfg)
        2'd1: begin asm_pre[c] = 2'd2; asm_pfx[c] = 1'b1; end
        2'd2: begin asm_suf[c] = 3'd2; end
        2'd3: begin
          asm_zl[c] = sym; asm_mlen[c] = ct_len;
          if (exA_ch == 2'd3) begin asm_suf[c] = 3'd4; asm_ctr[c] = (c == 0) ? 32'd1 : 32'd2; end
        end
        default: ;
      endcase
    end
  end
  assign mem_is_ct = exA_ch[1];
  assign asm_limit = mem_is_ct ? 11'd1024 : pk_limit;

  logic rr;
  always_comb begin
    asm_mgnt = 2'b00;
    if (asm_mreq[0] && asm_mreq[1]) asm_mgnt = rr ? 2'b10 : 2'b01;
    else asm_mgnt = asm_mreq;
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) rr <= 1'b0;
    else if (clr) rr <= 1'b0;                       // deterministic per command (FV-F9)
    else if (asm_mreq[0] && asm_mreq[1]) rr <= ~rr;
  end
  logic [9:0] arb_addr;
  assign arb_addr  = asm_mgnt[1] ? asm_maddr[1] : asm_maddr[0];
  assign pkb_en    = |asm_mgnt && !mem_is_ct;
  assign pkb_addr  = arb_addr;
  assign ctb_en    = |asm_mgnt && mem_is_ct;
  assign ctb_addr  = arb_addr;
  assign asm_rdata = mem_is_ct ? ctb_rdata : pkb_rdata;

  // ---- per-core executor signals
  // A: cores 0/1 ; B: cores 2/3
  logic [1:0] exA_bdone, exA_lastb, exA_fin, exA_free, exA_go, exA_st, exA_cnt;   // cnt: continuation start (done cycle)
  logic [1:0] exB_bdone, exB_lastb, exB_fin, exB_free, exB_go, exB_st, exB_cnt;
  logic [1:0] exA_chq, exB_chq;          // chained continuation accepted in the last round cycle
  logic [1:0][511:0] exA_bv, exB_bv;     // block for the next start of the local core
  logic [1:0]        exA_bok, exB_bok;
  logic [1:0][511:0] exA_cv, exB_cv;     // chained (next) block
  logic [1:0]        exA_cok, exB_cok;

  function automatic logic [511:0] fmtB(input job_t jb, input logic [5:0] bb, input logic [511:0] seedv);
    case (jb.fmt)
      3'd1:    fmtB = le2blk(s_k1in[512*((bb > 6'd2) ? 6'd2 : bb) +: 512]);   // index clamp (unused beyond the last block)
      3'd2:    fmtB = le2blk(s_h1[512*((bb > 6'd2) ? 6'd2 : bb) +: 512]);
      3'd3:    fmtB = fmt_d32(tmp_eff[jb.a]);
      3'd4:    fmtB = (bb == 6'd0) ? le2blk({tmp_eff[jb.b], tmp_eff[jb.a]}) : pad64only(1'b0);
      3'd5:    fmtB = (bb == 6'd0) ? le2blk({tmp_eff[jb.b], tmp_eff[jb.a]}) : fmt_d32(tmp_eff[jb.c]);
      3'd7:    fmtB = le2blk(kr_seed);
      default: fmtB = le2blk(seedv);
    endcase
  endfunction
  // block availability for chain B: K1IN/H1 need m (and hpk except ZEN_512 block 0); CAT3 block 1 needs dep1
  function automatic logic okB(input job_t jb, input logic [5:0] bb, input logic [12:0] dn);
    case (jb.fmt)
      3'd1, 3'd2: okB = m_ok && (hpk_ok || (param == 2'd2 && bb == 6'd0));
      3'd5:       okB = (bb == 6'd0) || ((jb.dep1 & ~dn) == 13'd0);
      default:    okB = 1'b1;
    endcase
  endfunction

  always_comb begin
    tmp_wr = '0; tmp_nv = '0;
    for (int c = 0; c < 2; c++) begin
      job_t ja, jb, jan, jbn;
      // ---------------- executor A
      ja  = jobA(exA_ch, exA_cj[c]);
      jan = jobA(exA_ch, exA_nj[c]);
      exA_bdone[c] = exA_cj_v[c] && exA_wait[c] && c_done[c];
      exA_lastb[c] = (ja.fmt == 3'd0) ? asm_last_q[c] : (exA_blk[c] == fmt_nb(ja.fmt, param) - 6'd1);
      exA_fin[c]   = exA_bdone[c] && exA_lastb[c] && !exA_chn[c];
      exA_free[c]  = !exA_cj_v[c] || exA_fin[c];
      if (exA_fin[c]) begin tmp_wr[ja.out] = 1'b1; tmp_nv[ja.out] = dig2le(c_vout[c]); end
      // ---------------- executor B
      jb  = jobB(exB_ch, exB_cj[c]);
      jbn = jobB(exB_ch, exB_nj[c]);
      exB_bdone[c] = exB_cj_v[c] && exB_wait[c] && c_done[c+2];
      exB_lastb[c] = (exB_blk[c] == fmt_nb(jb.fmt, param) - 6'd1);
      exB_fin[c]   = exB_bdone[c] && exB_lastb[c] && !exB_chn[c];
      exB_free[c]  = !exB_cj_v[c] || exB_fin[c];
      if (exB_fin[c] && jb.fmt != 3'd6 && jb.fmt != 3'd7) begin tmp_wr[jb.out] = 1'b1; tmp_nv[jb.out] = dig2le(c_vout[c+2]); end
    end
    exA_dnow = '0; exB_dnow = '0;
    for (int c = 0; c < 2; c++) begin
      if (exA_fin[c]) exA_dnow[exA_cj[c]] = 1'b1;
      if (exB_fin[c]) exB_dnow[exB_cj[c]] = 1'b1;
    end
  end

  always_comb begin
    smid_run = 1'b0;
    for (int c = 0; c < 2; c++) begin
      job_t jb;
      jb = jobB(exB_ch, exB_cj[c]);
      if (exB_run && exB_cj_v[c] && jb.fmt == 3'd7) smid_run = 1'b1;
    end
  end

  // H(pk) final job completes this cycle
  always_comb begin
    hpk_now = 1'b0;
    if (task_hpk) begin
      if (exA_ch == 2'd0) hpk_now = exA_dnow[0];
      else if (exA_ch == 2'd1) hpk_now = exA_dnow[3];
    end
  end

  always_comb begin
    for (int c = 0; c < 2; c++) begin
      job_t ja, jb, jan, jbn;
      logic [12:0] dA, dB;
      ja  = jobA(exA_ch, exA_cj[c]);
      jan = jobA(exA_ch, exA_nj[c]);
      jb  = jobB(exB_ch, exB_cj[c]);
      jbn = jobB(exB_ch, exB_nj[c]);
      dA = exA_done | exA_dnow;
      dB = exB_done | exB_dnow;
      // ---------------- A: new job claim
      exA_go[c] = exA_run && exA_free[c] && exA_nj_v[c] && ((jan.deps & ~dA) == 13'd0) && !x_own[c] && !x_wt[c] && !x_pend[c];
      // One block per core and cycle: the chained continuation (last round cycle of a non-final block), the
      // next block of the running job, or block 0 of the job claimed now (mutually exclusive per cycle).
      begin
        logic cA, rA, cB, rB;
        logic [3:0] jA, jB;
        logic [5:0] bA, bB;
        job_t js;
        cA = exA_cj_v[c] && exA_wait[c] && c_lastc[c] &&
             ((ja.fmt == 3'd0) ? !asm_last_q[c] : (exA_blk[c] != fmt_nb(ja.fmt, param) - 6'd1));
        rA = exA_cj_v[c] && !exA_fin[c];
        jA = (cA || rA) ? exA_cj[c] : exA_nj[c];
        bA = cA ? exA_blk[c] + 6'd1 : rA ? (exA_bdone[c] ? exA_blk[c] + 6'd1 : exA_blk[c]) : 6'd0;
        js = jobA(exA_ch, jA);
        case (js.fmt)
          3'd0:    begin exA_bv[c] = asm_blk[c]; exA_bok[c] = asm_bv[c] && (cA || rA); end   // bulk: new job starts its assembler
          3'd3:    begin exA_bv[c] = fmt_d32(tmp_eff[js.a]); exA_bok[c] = 1'b1; end
          default: begin exA_bv[c] = (bA == 6'd0) ? le2blk({tmp_eff[js.b], tmp_eff[js.a]}) : pad64only(1'b0); exA_bok[c] = 1'b1; end
        endcase
        exA_cv[c]  = exA_bv[c];
        exA_cok[c] = cA && exA_bok[c];
        // ---------------- B
        exB_go[c] = exB_run && exB_free[c] && exB_nj_v[c] && ((jbn.deps & ~dB) == 13'd0) && !x_own[c+2] && !x_wt[c+2] && !x_pend[c+2];
        cB = exB_cj_v[c] && exB_wait[c] && c_lastc[c+2] && (exB_blk[c] != fmt_nb(jb.fmt, param) - 6'd1);
        rB = exB_cj_v[c] && !exB_fin[c];
        jB = (cB || rB) ? exB_cj[c] : exB_nj[c];
        bB = cB ? exB_blk[c] + 6'd1 : rB ? (exB_bdone[c] ? exB_blk[c] + 6'd1 : exB_blk[c]) : 6'd0;
        js = jobB(exB_ch, jB);
        exB_bv[c]  = fmtB(js, bB, seed_reg);
        exB_bok[c] = okB(js, bB, dB);
        exB_cv[c]  = exB_bv[c];
        exB_cok[c] = cB && exB_bok[c];
      end
    end
  end

  // ------------------------------------------------------------ XOF dispatch
  logic [7:0][3:0]  x_tag;
  logic [7:0][15:0] x_ctr;
  logic             smid_now;
  logic [255:0]     smid_eff;
  logic [7:0]       resA, resB;
  assign resA = exA_run ? 8'b0000_0011 : 8'b0;
  assign resB = exB_run ? 8'b0000_1100 : 8'b0;

  logic [7:0] free_core;
  logic [2:0] xsel;
  logic       xsel_v;
  always_comb begin
    free_core = ~(c_busy | x_own | x_pend | x_wt | resA | resB);
    xsel_v = 1'b0; xsel = 3'd0;
    for (int c = 7; c >= 0; c--) if (free_core[c]) begin xsel_v = 1'b1; xsel = c[2:0]; end
  end
  // requests are accepted on any free core; a core claimed before the seed midstate of this command is
  // valid starts in the cycle the midstate completes (all waiting cores start together)
  assign xreq_ready = xsel_v;
  always_comb begin
    smid_now = 1'b0; smid_eff = smid_reg;
    for (int c = 0; c < 2; c++) begin
      job_t jb;
      jb = jobB(exB_ch, exB_cj[c]);
      if (exB_fin[c] && (jb.fmt == 3'd6 || jb.fmt == 3'd7)) begin smid_now = 1'b1; smid_eff = c_vout[c+2]; end
    end
  end

  // response arbiter: the pending digest with the smallest tag first (the sampler consumes chunks in tag order)
  logic [7:0] x_pick;
  logic [2:0] xp;
  logic       xpv;
  always_comb begin
    xpv = 1'b0; xp = 3'd0;
    for (int c = 0; c < 8; c++)
      if (x_pend[c] && (!xpv || x_tag[c] < x_tag[xp])) begin xpv = 1'b1; xp = c[2:0]; end
    x_pick = xpv ? (8'd1 << xp) : 8'd0;
    xrsp_valid  = xpv;
    xrsp_tag    = x_tag[xp];
    xrsp_digest = c_vout[xp];
  end

  // ------------------------------------------------------------ core start / continuation muxes
  always_comb begin
    c_start = '0; c_cont = '0; asm_take = '0; asm_start = '0;
    exA_st = '0; exB_st = '0; exA_cnt = '0; exB_cnt = '0; exA_chq = '0; exB_chq = '0;
    for (int c = 0; c < 8; c++) begin c_iv[c] = smid_reg; c_blk[c] = xof_blk(xof_nonce, xreq_ctr); end
    for (int c = 0; c < 2; c++) begin
      job_t ja, jan, jb, jbn;
      ja  = jobA(exA_ch, exA_cj[c]);
      jan = jobA(exA_ch, exA_nj[c]);
      jb  = jobB(exB_ch, exB_cj[c]);
      jbn = jobB(exB_ch, exB_nj[c]);
      // ---------------- A
      c_blk[c] = exA_bv[c];
      if (exA_cok[c]) begin
        // chained continuation: next block of the running job
        c_cont[c] = 1'b1; c_blk[c] = exA_cv[c]; exA_chq[c] = 1'b1;
        if (ja.fmt == 3'd0) asm_take[c] = 1'b1;
      end else if (exA_cj_v[c] && !exA_fin[c] && !exA_chn[c] && (!exA_wait[c] || exA_bdone[c]) && !(c_busy[c] && !exA_bdone[c]) && exA_bok[c]) begin
        // next block of the running job, started in the done cycle of its predecessor or later
        c_start[c] = 1'b1; exA_st[c] = 1'b1; exA_cnt[c] = exA_bdone[c];
        c_iv[c] = (exA_blk[c] == 6'd0 && !exA_wait[c] && !exA_bdone[c]) ? ivsel(ja.iv) : c_vout[c];
        if (ja.fmt == 3'd0) asm_take[c] = 1'b1;
      end else if (exA_go[c]) begin
        if (jan.fmt == 3'd0) asm_start[c] = 1'b1;
        else begin c_start[c] = 1'b1; exA_st[c] = 1'b1; c_iv[c] = ivsel(jan.iv); end
      end
      // ---------------- B
      c_blk[c+2] = exB_bv[c];
      if (exB_cok[c]) begin
        c_cont[c+2] = 1'b1; c_blk[c+2] = exB_cv[c]; exB_chq[c] = 1'b1;
      end else if (exB_cj_v[c] && !exB_fin[c] && !exB_chn[c] && (!exB_wait[c] || exB_bdone[c]) && !(c_busy[c+2] && !exB_bdone[c]) && exB_bok[c]) begin
        c_start[c+2] = 1'b1; exB_st[c] = 1'b1; exB_cnt[c] = exB_bdone[c];
        c_iv[c+2] = (exB_blk[c] == 6'd0 && !exB_wait[c] && !exB_bdone[c]) ? ivsel(jb.iv) : c_vout[c+2];
      end else if (exB_go[c] && exB_bok[c]) begin
        c_start[c+2] = 1'b1; exB_st[c] = 1'b1; c_iv[c+2] = ivsel(jbn.iv);
      end
    end
    for (int c = 0; c < 8; c++) begin
      if (x_wt[c] && (smid_valid || smid_now)) begin
        c_start[c] = 1'b1;
        c_iv[c]    = smid_eff;
        c_blk[c]   = xof_blk(xof_nonce, x_ctr[c]);
      end
    end
    if (xreq_valid && xreq_ready && smid_valid) begin
      c_start[xsel] = 1'b1;
      c_iv[xsel]    = smid_reg;
      c_blk[xsel]   = xof_blk(xof_nonce, xreq_ctr);
    end
  end

  // ------------------------------------------------------------ sequential control
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      exA_run <= 1'b0; exB_run <= 1'b0; exA_ch <= '0; exB_ch <= '0;
      exA_started <= '0; exA_done <= '0; exB_started <= '0; exB_done <= '0;
      exA_cj_v <= '0; exB_cj_v <= '0; exA_wait <= '0; exB_wait <= '0; exA_chn <= '0; exB_chn <= '0;
      exA_cj <= '0; exB_cj <= '0; exA_blk <= '0; exB_blk <= '0;
      x_own <= '0; x_pend <= '0; x_wt <= '0;
      t_hpk_done <= 1'b0; t_rej_done <= 1'b0; t_kr_done <= 1'b0; t_smid_done <= 1'b0;
      task_hpk <= 1'b0; task_rej <= 1'b0; task_kr <= 1'b0; task_smid <= 1'b0;
      hpk_valid <= 1'b0; smid_valid <= 1'b0;
    end else begin
      t_hpk_done <= 1'b0; t_rej_done <= 1'b0; t_kr_done <= 1'b0; t_smid_done <= 1'b0;
      if (clr) begin
        exA_run <= 1'b0; exB_run <= 1'b0; exA_cj_v <= '0; exB_cj_v <= '0; exA_wait <= '0; exB_wait <= '0;
        exA_chn <= '0; exB_chn <= '0;
        task_hpk <= 1'b0; task_rej <= 1'b0; task_kr <= 1'b0; task_smid <= 1'b0;
        hpk_valid <= 1'b0; smid_valid <= 1'b0;
      end
      // task starts (may coincide with clr: command accepted and its first tasks started together)
        if (t_hpk_start) begin
        exA_run <= 1'b1; exA_ch <= (param == 2'd2) ? 2'd1 : 2'd0; exA_started <= '0; exA_done <= '0; task_hpk <= 1'b1;
      end
      if (t_rej_start) begin
        exA_run <= 1'b1; exA_ch <= (param == 2'd2) ? 2'd3 : 2'd2; exA_started <= '0; exA_done <= '0; task_rej <= 1'b1;
      end
      if (t_kr_start) begin
        exB_run <= 1'b1; exB_ch <= (param == 2'd2) ? 2'd1 : 2'd0; exB_started <= '0; exB_done <= '0; task_kr <= 1'b1;
        smid_valid <= 1'b0;
      end
      if (t_smid_start) begin
        exB_run <= 1'b1; exB_ch <= 2'd2; exB_started <= '0; exB_done <= '0; task_smid <= 1'b1;
        smid_valid <= 1'b0;
      end
      if (!clr) begin
        if (hpk_now || sk_ld_commit) hpk_valid <= 1'b1;
        // ---- executors
        for (int c = 0; c < 2; c++) begin
          // A
          if (exA_bdone[c]) begin
            if (exA_chn[c]) begin exA_blk[c] <= exA_blk[c] + 6'd1; exA_chn[c] <= 1'b0; end   // chained: keep running
            else if (exA_lastb[c]) begin exA_cj_v[c] <= 1'b0; exA_wait[c] <= 1'b0; exA_done[exA_cj[c]] <= 1'b1; end
            else begin exA_blk[c] <= exA_blk[c] + 6'd1; exA_wait[c] <= exA_st[c]; end
          end else if (exA_st[c] && exA_cj_v[c]) exA_wait[c] <= 1'b1;
          if (exA_chq[c]) exA_chn[c] <= 1'b1;
          if (exA_go[c] && !(exA_cj_v[c] && !exA_fin[c] && exA_st[c])) begin
            exA_cj_v[c] <= 1'b1; exA_cj[c] <= exA_nj[c]; exA_blk[c] <= 6'd0; exA_started[exA_nj[c]] <= 1'b1;
            exA_wait[c] <= exA_st[c];
          end
          // B
          if (exB_bdone[c]) begin
            if (exB_chn[c]) begin exB_blk[c] <= exB_blk[c] + 6'd1; exB_chn[c] <= 1'b0; end
            else if (exB_lastb[c]) begin exB_cj_v[c] <= 1'b0; exB_wait[c] <= 1'b0; exB_done[exB_cj[c]] <= 1'b1; end
            else begin exB_blk[c] <= exB_blk[c] + 6'd1; exB_wait[c] <= exB_st[c]; end
          end else if (exB_st[c] && exB_cj_v[c]) exB_wait[c] <= 1'b1;
          if (exB_chq[c]) exB_chn[c] <= 1'b1;
          if (exB_go[c] && !(exB_cj_v[c] && !exB_fin[c] && exB_st[c])) begin
            exB_cj_v[c] <= 1'b1; exB_cj[c] <= exB_nj[c]; exB_blk[c] <= 6'd0; exB_started[exB_nj[c]] <= 1'b1;
            exB_wait[c] <= exB_st[c];
          end
        end
        if (exA_run && exA_all != 13'd0 && ((exA_done | exA_dnow) | ~exA_all) == 13'h1FFF && !t_hpk_start && !t_rej_start) begin
          exA_run <= 1'b0;
          if (task_hpk) t_hpk_done <= 1'b1;
          if (task_rej) t_rej_done <= 1'b1;
          task_hpk <= 1'b0; task_rej <= 1'b0;
        end
        if (exB_run && exB_all != 13'd0 && ((exB_done | exB_dnow) | ~exB_all) == 13'h1FFF && !t_kr_start && !t_smid_start) begin
          exB_run <= 1'b0;
          if (task_kr) t_kr_done <= 1'b1;
          if (task_smid) t_smid_done <= 1'b1;
          task_kr <= 1'b0; task_smid <= 1'b0;
        end
        // seed midstate job completes -> XOF requests start directly (smid_now) and from the next cycle
        if (smid_now) smid_valid <= 1'b1;
      end
      // ---- XOF bookkeeping
      for (int c = 0; c < 8; c++) begin
        if (xreq_valid && xreq_ready && xsel == c[2:0]) begin
          if (smid_valid) x_own[c] <= 1'b1;
          else            x_wt[c]  <= 1'b1;
        end
        if (x_wt[c] && (smid_valid || smid_now)) begin x_wt[c] <= 1'b0; x_own[c] <= 1'b1; end
        if (c_done[c] && x_own[c]) begin x_own[c] <= 1'b0; x_pend[c] <= 1'b1; end
        if (x_pick[c]) x_pend[c] <= 1'b0;
      end
      if (clr) begin x_own <= '0; x_pend <= '0; x_wt <= '0; end
    end
  end
  assign act_xof = |(x_own | x_pend | x_wt);
  always_ff @(posedge clk) begin
    for (int c = 0; c < 8; c++) if (xreq_valid && xreq_ready && xsel == c[2:0]) begin
      x_tag[c] <= xreq_tag; x_ctr[c] <= xreq_ctr;
    end
  end

  // capture "last block" flag of bulk blocks when taken
  always_ff @(posedge clk) begin
    for (int c = 0; c < 2; c++) if (asm_take[c]) asm_last_q[c] <= asm_last[c];
  end

  // results into temps / seed midstate
  always_ff @(posedge clk) begin
    for (int i = 0; i < 16; i++) if (tmp_wr[i]) tmp[i] <= tmp_nv[i];
    if (smid_now) smid_reg <= smid_eff;
  end

  // ------------------------------------------------------------ registers
  always_ff @(posedge clk) begin
    if (rnd_we) begin
      case (rnd_reg)
        2'd0: seed_reg[64*rnd_idx +: 64] <= rnd_data;
        2'd1: z_reg[64*rnd_idx +: 64]    <= rnd_data;
        default: m_reg[64*rnd_idx +: 64] <= rnd_data;
      endcase
    end
    if (m_load) m_reg <= mload;
    if (hpk_now) hpk_reg <= hpk_new;
    if (ss_load) ss_reg <= ss_sel_rej ? ssp : kr_ss;
    if (sk_ld_commit) begin hpk_reg <= sk_hpk; z_reg <= sk_z; end
  end

  // ------------------------------------------------------------ SK copier (hpk || z at hpkoff)
  logic [9:0]  sk_w0;
  logic [2:0]  sk_d;
  logic [4:0]  sk_nw;          // words to transfer
  always_comb begin
    case (param)
      2'd0:    begin sk_w0 = 10'd158; sk_d = 3'd7; sk_nw = 5'd5;  end
      2'd1:    begin sk_w0 = 10'd317; sk_d = 3'd5; sk_nw = 5'd9;  end
      default: begin sk_w0 = 10'd635; sk_d = 3'd2; sk_nw = 5'd17; end
    endcase
  end
  // region bytes R = hpk[0:sym] || z[0:ss]   (Lr = 2*sym bytes)
  logic [1023:0] sk_region;
  always_comb begin
    case (param)
      2'd0:    sk_region = {768'd0, z_reg[127:0], hpk_reg[127:0]};
      2'd1:    sk_region = {512'd0, z_reg[255:0], hpk_reg[255:0]};
      default: sk_region = {z_reg, hpk_reg};
    endcase
  end
  logic [1087:0] sk_wbuf;      // region shifted by d bytes (17 words)
  assign sk_wbuf = {64'd0, sk_region} << (8 * sk_d);
  logic [1087:0] sk_rbuf;
  logic [1023:0] sk_rd_region;
  assign sk_rd_region = sk_rbuf[8*sk_d +: 1024];
  always_comb begin
    case (param)
      2'd0:    begin sk_hpk = {384'd0, sk_rd_region[127:0]}; sk_z = {384'd0, sk_rd_region[255:128]}; end
      2'd1:    begin sk_hpk = {256'd0, sk_rd_region[255:0]}; sk_z = {256'd0, sk_rd_region[511:256]}; end
      default: begin sk_hpk = sk_rd_region[511:0];           sk_z = sk_rd_region[1023:512]; end
    endcase
  end
  // byte mask of region bytes within the shifted window
  logic [135:0] sk_bmask;
  assign sk_bmask = ({8'd0, {128{1'b1}}} >> (128 - {sym, 1'b0})) << sk_d;

  typedef enum logic [1:0] {K_IDLE, K_LD, K_ST} kst_t;
  kst_t kst;
  logic [4:0] kcnt;
  logic       krd_v;
  logic [4:0] krd_i;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      kst <= K_IDLE; kcnt <= '0; krd_v <= 1'b0; krd_i <= '0; t_skld_done <= 1'b0; t_skst_done <= 1'b0; sk_ld_commit <= 1'b0;
      act_sk <= 1'b0;
    end else begin
      t_skld_done <= 1'b0; t_skst_done <= 1'b0; sk_ld_commit <= 1'b0;
      krd_v <= 1'b0;
      if (clr) begin kst <= K_IDLE; act_sk <= 1'b0; end
      else case (kst)
        K_IDLE: begin
          kcnt <= '0;
          if (t_skld_start) begin kst <= K_LD; act_sk <= 1'b1; end
          else if (t_skst_start) begin kst <= K_ST; act_sk <= 1'b1; end
        end
        K_LD: begin
          if (kcnt < sk_nw) begin krd_v <= 1'b1; krd_i <= kcnt; kcnt <= kcnt + 5'd1; end
          else if (!krd_v) begin kst <= K_IDLE; sk_ld_commit <= 1'b1; t_skld_done <= 1'b1; act_sk <= 1'b0; end
        end
        K_ST: begin
          if (kcnt < sk_nw) kcnt <= kcnt + 5'd1;
          else begin kst <= K_IDLE; t_skst_done <= 1'b1; act_sk <= 1'b0; end
        end
        default: kst <= K_IDLE;
      endcase
    end
  end
  always_ff @(posedge clk) begin
    if (krd_v) sk_rbuf[64*krd_i +: 64] <= skb_rdata;
  end
  assign skb_en    = ((kst == K_LD) || (kst == K_ST)) && (kcnt < sk_nw);
  assign skb_we    = (kst == K_ST);
  assign skb_addr  = sk_w0 + {5'd0, kcnt};
  assign skb_wdata = sk_wbuf[64*kcnt +: 64];
  assign skb_wmask = sk_bmask[8*kcnt +: 8];
endmodule
