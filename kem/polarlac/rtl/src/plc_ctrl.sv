// Command protocol, register file, random intake, CTRL hash jobs and the microcoded sequencer.
// The sequencer walks a per-operation program (prog()) of engine starts and waits:
// - a start instruction stalls until its engine is idle (and referenced TSAMP roles are ready),
//   then issues a 1-cycle start pulse with operands; engines run concurrently;
// - WAIT stalls until all engines in its mask are idle; END waits for all engines, then commits.
// Conditions: cond=1 only for q=257 sets, cond=2 only for 512-Star.
module plc_ctrl
  import plc_pkg::*;
(
  input  logic              clk,
  input  logic              rst_n,
  // host control
  input  logic              cmd_valid,
  output logic              cmd_ready,
  input  logic [1:0]        cmd_op,
  input  logic [2:0]        cmd_param,
  output logic              busy,
  output logic              done,
  output logic              result_valid,
  output logic              cmd_error,
  // random stream
  input  logic              rnd_valid,
  output logic              rnd_ready,
  input  logic [63:0]       rnd_data,
  // shared state
  output pcode_t            pcode,
  output logic [6:0][511:0] rg,
  output logic [511:0]      ss_reg,
  output logic              cmd_start,     // pulse on valid-command acceptance (cache clear, cmp clear)
  // hash client 2 (CTRL)
  output logic              hj_req,
  input  logic              hj_gnt,
  output logic [1:0]        hj_kind0, hj_kind1,
  output logic [2:0]        hj_reg0,  hj_reg1,
  output logic [1:0]        hj_buf0,  hj_buf1,
  output logic [12:0]       hj_off0,  hj_off1,
  output logic [12:0]       hj_len0,  hj_len1,
  output logic [55:0]       hj_sfx,
  output logic [2:0]        hj_sfxlen,
  output logic [6:0]        hj_count,
  output logic [1:0]        hj_cache,
  input  logic              xo_valid,
  output logic              xo_ready,
  input  logic [255:0]      xo_data,
  input  logic              xo_last,
  output logic              rg_inv_sd,     // RG_SD rewritten
  output logic              rg_inv_ps,     // RG_PS rewritten
  // GENA
  output logic              ga_start,
  output logic              ga_tr,
  input  logic              ga_busy,
  // TSAMP
  output logic              ts_start,
  output logic [7:0]        ts_nonce0,
  output logic              ts_e2,
  input  logic              ts_busy,
  input  logic [4:0]        ts_role_ok,
  input  logic [4:0][3:0]   ts_role_slot,
  input  logic [7:0]        ts_nonce_next,
  // NTT
  output logic              nt_start,
  output logic              nt_inv,
  output logic [3:0]        nt_slot,
  input  logic              nt_busy,
  // PWALU
  output logic              pw_start,
  output logic [1:0]        pw_mode,
  output logic              pw_acc_zero,
  output logic [3:0]        pw_sa, pw_sb, pw_sc, pw_sd,
  input  logic              pw_busy,
  // CODEC
  output logic              cd_start,
  output logic [3:0]        cd_op,
  output logic              cd_k,
  output logic [3:0]        cd_slot,
  output logic              cd_cmp,
  output logic              cd_src_sk,
  input  logic              cd_busy,
  input  logic              cd_ovf,
  input  logic              cd_ne,
  input  logic              cd_rg_we,
  input  logic [511:0]      cd_rg_wdata,
  // POLAR
  output logic              pe_start,
  output logic              pd_start,
  input  logic              pol_busy,
  input  logic              pd_m_we,
  input  logic [511:0]      pd_m_data
);
  // ---------------- micro-op format ----------------
  localparam logic [3:0] K_END = 4'd0, K_WAIT = 4'd1, K_RNDI = 4'd2, K_HJ = 4'd3, K_GENA = 4'd4,
                         K_TS = 4'd5, K_NTT = 4'd6, K_PW = 4'd7, K_CD = 4'd8, K_PENC = 4'd9,
                         K_PDEC = 4'd10, K_BROVF = 4'd11, K_JMP = 4'd12;
  localparam int E_RNDI = 0, E_HJ = 1, E_GENA = 2, E_TS = 3, E_NTT = 4, E_PW = 5, E_CD = 6, E_POL = 7;
  // codec ops
  localparam logic [3:0] C_SER_S = 4'd0, C_DES_S = 4'd1, C_PK_ENC = 4'd2, C_PK_DEC = 4'd3,
                         C_C1_ENC = 4'd4, C_C1_DEC = 4'd5, C_C2_ENC = 4'd6, C_C2_DEC = 4'd7,
                         C_WR_PKSEED = 4'd8, C_WR_Z = 4'd9, C_RD_PKSEED = 4'd10;
  // pw modes
  localparam logic [3:0] P_MULACC = 4'd0, P_ADD = 4'd1, P_MSG = 4'd2, P_DSUB = 4'd3;
  // hash job types
  localparam logic [3:0] H_EXPAND = 4'd0, H_GENC = 4'd1, H_GDEC = 4'd2, H_REJECT = 4'd3;
  // rnd targets
  localparam logic [3:0] R_KG = 4'd0, R_Z = 4'd1, R_M = 4'd2;

  typedef struct packed {
    logic [3:0] kind;
    logic [1:0] cond;
    logic [3:0] sub;
    logic       f;
    logic       k;
    logic [4:0] sa, sb, sc, sd;
    logic [7:0] mask;
    logic [5:0] tgt;
  } uop_t;

  // slot specs: physical 0..15, role k = 16+k
  localparam logic [4:0] A00 = 5'd0, A01 = 5'd1, A10 = 5'd2, A11 = 5'd3, B0 = 5'd4, B1 = 5'd5,
                         U0 = 5'd6, U1 = 5'd7, DS0 = 5'd8, DS1 = 5'd9, SU = 5'd10, HM = 5'd11,
                         VV = 5'd15, R0 = 5'd16, R1 = 5'd17, R2 = 5'd18, R3 = 5'd19, R4 = 5'd20;

  function automatic uop_t u(input logic [3:0] kind, input logic [1:0] cond, input logic [3:0] sub,
                             input logic f, input logic k,
                             input logic [4:0] sa, input logic [4:0] sb,
                             input logic [4:0] sc, input logic [4:0] sd,
                             input logic [7:0] mask, input logic [5:0] tgt);
    uop_t x;
    x.kind = kind; x.cond = cond; x.sub = sub; x.f = f; x.k = k;
    x.sa = sa; x.sb = sb; x.sc = sc; x.sd = sd; x.mask = mask; x.tgt = tgt;
    return x;
  endfunction
  function automatic logic [7:0] m(input int e0, input int e1, input int e2, input int e3);
    logic [7:0] r;
    r = '0;
    r[e0] = 1'b1;
    if (e1 >= 0) r[e1] = 1'b1;
    if (e2 >= 0) r[e2] = 1'b1;
    if (e3 >= 0) r[e3] = 1'b1;
    return r;
  endfunction

  // Encrypt core (shared by Enc and Dec re-encryption); i = index from 0, cmp = compare mode.
  function automatic uop_t enc_core(input int i, input logic cmp);
    case (i)
      0:  return u(K_TS, 0, 0, 1'b1, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);                                           // r0,r1,e1_0,e1_1 + e2
      1:  return u(K_NTT, 0, 0, 1'b0, 0, R0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
      2:  return u(K_NTT, 0, 0, 1'b0, 0, R1, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
      3:  return u(K_NTT, 2, 0, 1'b0, 0, R2, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);                                   // Star: NTT(e1)
      4:  return u(K_NTT, 2, 0, 1'b0, 0, R3, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
      5:  return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_NTT, E_GENA, E_CD, E_POL), 6'd0);
      6:  return u(K_PW, 0, P_MULACC, 1'b1, 0, A00, R0, 0, U0, 8'd0, 6'd0);
      7:  return u(K_PW, 0, P_MULACC, 1'b0, 0, A01, R1, U0, U0, 8'd0, 6'd0);
      8:  return u(K_PW, 0, P_MULACC, 1'b1, 0, A10, R0, 0, U1, 8'd0, 6'd0);
      9:  return u(K_NTT, 1, 4'd1, 1'b0, 0, U0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);                                // q257: INTT u0
      10: return u(K_PW, 0, P_MULACC, 1'b0, 0, A11, R1, U1, U1, 8'd0, 6'd0);
      11: return u(K_PW, 0, P_MULACC, 1'b1, 0, B0, R0, 0, VV, 8'd0, 6'd0);
      12: return u(K_NTT, 1, 4'd1, 1'b0, 0, U1, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
      13: return u(K_PW, 0, P_MULACC, 1'b0, 0, B1, R1, VV, VV, 8'd0, 6'd0);
      14: return u(K_PW, 0, P_ADD, 1'b0, 0, R2, 0, U0, U0, 8'd0, 6'd0);
      15: return u(K_NTT, 0, 4'd1, 1'b0, 0, VV, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
      16: return u(K_PW, 0, P_ADD, 1'b0, 0, R3, 0, U1, U1, 8'd0, 6'd0);
      17: return u(K_CD, 0, C_C1_ENC, cmp, 1'b0, U0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
      18: return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_NTT, E_PW, -1, -1), 6'd0);
      19: return u(K_PW, 0, P_ADD, 1'b0, 0, R4, 0, VV, VV, 8'd0, 6'd0);
      20: return u(K_PW, 0, P_MSG, 1'b0, 0, 0, 0, VV, VV, 8'd0, 6'd0);
      21: return u(K_CD, 0, C_C1_ENC, cmp, 1'b1, U1, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
      22: return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_PW, -1, -1, -1), 6'd0);
      23: return u(K_CD, 0, C_C2_ENC, cmp, 1'b0, VV, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
      default: return u(K_END, 2'd0, 4'd0, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
    endcase
  endfunction

  function automatic uop_t prog(input logic [1:0] op, input logic [5:0] pc);
    if (op == 2'd0) begin : keygen
      case (pc)
        0:  return u(K_RNDI, 0, R_KG, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        1:  return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_RNDI, -1, -1, -1), 6'd0);
        2:  return u(K_HJ, 0, H_EXPAND, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        3:  return u(K_RNDI, 0, R_Z, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        4:  return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_HJ, -1, -1, -1), 6'd0);
        5:  return u(K_TS, 0, 0, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);                                   // s0,s1,e0,e1 from nonce 0
        6:  return u(K_GENA, 0, 0, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        7:  return u(K_CD, 0, C_WR_PKSEED, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        8:  return u(K_NTT, 0, 0, 0, 0, R0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        9:  return u(K_NTT, 0, 0, 0, 0, R1, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        10: return u(K_NTT, 0, 0, 0, 0, R2, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        11: return u(K_NTT, 0, 0, 0, 0, R3, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        12: return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_NTT, E_GENA, -1, -1), 6'd0);
        13: return u(K_PW, 0, P_MULACC, 1'b1, 0, A00, R0, 0, B0, 8'd0, 6'd0);
        14: return u(K_PW, 0, P_MULACC, 1'b0, 0, A01, R1, B0, B0, 8'd0, 6'd0);
        15: return u(K_PW, 0, P_ADD, 1'b0, 0, R2, 0, B0, B0, 8'd0, 6'd0);
        16: return u(K_PW, 0, P_MULACC, 1'b1, 0, A10, R0, 0, B1, 8'd0, 6'd0);
        17: return u(K_CD, 0, C_PK_ENC, 1'b0, 1'b0, B0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        18: return u(K_PW, 0, P_MULACC, 1'b0, 0, A11, R1, B1, B1, 8'd0, 6'd0);
        19: return u(K_PW, 0, P_ADD, 1'b0, 0, R3, 0, B1, B1, 8'd0, 6'd0);
        20: return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_PW, E_CD, -1, -1), 6'd0);
        21: return u(K_BROVF, 1, 0, 0, 0, 0, 0, 0, 0, 0, 6'd30);
        22: return u(K_CD, 0, C_PK_ENC, 1'b0, 1'b1, B1, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        23: return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_CD, -1, -1, -1), 6'd0);
        24: return u(K_BROVF, 1, 0, 0, 0, 0, 0, 0, 0, 0, 6'd30);
        25: return u(K_CD, 0, C_SER_S, 1'b0, 1'b0, R0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        26: return u(K_CD, 0, C_SER_S, 1'b0, 1'b1, R1, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        27: return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_RNDI, -1, -1, -1), 6'd0);
        28: return u(K_CD, 0, C_WR_Z, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        29: return u(K_END, 2'd0, 4'd0, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        30: return u(K_TS, 0, 0, 1'b0, 1'b1, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);                                   // retry: continue nonce
        31: return u(K_JMP, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6'd8);
        default: return u(K_END, 2'd0, 4'd0, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
      endcase
    end else if (op == 2'd1) begin : enc
      case (pc)
        0:  return u(K_RNDI, 0, R_M, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        1:  return u(K_CD, 0, C_RD_PKSEED, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        2:  return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_CD, -1, -1, -1), 6'd0);
        3:  return u(K_GENA, 0, 0, 1'b1, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        4:  return u(K_CD, 0, C_PK_DEC, 1'b0, 1'b0, B0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        5:  return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_RNDI, -1, -1, -1), 6'd0);
        6:  return u(K_HJ, 0, H_GENC, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        7:  return u(K_PENC, 2'd0, 4'd0, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        8:  return u(K_CD, 0, C_PK_DEC, 1'b0, 1'b1, B1, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        9:  return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_HJ, -1, -1, -1), 6'd0);
        default: return enc_core(int'(pc) - 10, 1'b0);
      endcase
    end else begin : dec
      case (pc)
        0:  return u(K_HJ, 0, H_REJECT, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        1:  return u(K_CD, 0, C_RD_PKSEED, 1'b1, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        2:  return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_CD, -1, -1, -1), 6'd0);
        3:  return u(K_GENA, 0, 0, 1'b1, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        4:  return u(K_CD, 0, C_C1_DEC, 1'b0, 1'b0, U0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        5:  return u(K_CD, 0, C_DES_S, 1'b0, 1'b0, DS0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        6:  return u(K_NTT, 1, 0, 0, 0, U0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        7:  return u(K_CD, 0, C_C1_DEC, 1'b0, 1'b1, U1, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        8:  return u(K_CD, 0, C_DES_S, 1'b0, 1'b1, DS1, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        9:  return u(K_NTT, 1, 0, 0, 0, U1, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        10: return u(K_CD, 0, C_C2_DEC, 1'b0, 1'b0, VV, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        11: return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_NTT, E_CD, -1, -1), 6'd0);
        12: return u(K_PW, 0, P_MULACC, 1'b1, 0, U0, DS0, 0, SU, 8'd0, 6'd0);
        13: return u(K_PW, 0, P_MULACC, 1'b0, 0, U1, DS1, SU, SU, 8'd0, 6'd0);
        14: return u(K_CD, 0, C_PK_DEC, 1'b1, 1'b0, B0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        15: return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_PW, -1, -1, -1), 6'd0);
        16: return u(K_NTT, 0, 4'd1, 0, 0, SU, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        17: return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_NTT, -1, -1, -1), 6'd0);
        18: return u(K_PW, 0, P_DSUB, 1'b0, 0, VV, 0, SU, HM, 8'd0, 6'd0);
        19: return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_PW, -1, -1, -1), 6'd0);
        20: return u(K_PDEC, 2'd0, 4'd0, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        21: return u(K_CD, 0, C_PK_DEC, 1'b1, 1'b1, B1, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        22: return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_POL, -1, -1, -1), 6'd0);
        23: return u(K_HJ, 0, H_GDEC, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        24: return u(K_PENC, 2'd0, 4'd0, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0, 5'd0, 8'd0, 6'd0);
        25: return u(K_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, m(E_HJ, -1, -1, -1), 6'd0);
        default: return enc_core(int'(pc) - 26, 1'b1);
      endcase
    end
  endfunction

  // ---------------- command state ----------------
  logic [1:0] op_q;
  logic [5:0] pc;
  logic       q769, fin_q;
  assign q769      = (pcode == 3'd4);
  assign cmd_ready = !busy;

  logic accept, valid_cmd;
  assign accept    = cmd_valid && !busy;
  assign valid_cmd = (cmd_op != 2'd3) && (cmd_param <= 3'd4);

  // ---------------- engine idle tracking ----------------
  logic [7:0] pend;           // start issued last cycle
  logic       rn_busy, hj_busy;
  logic [7:0] ebusy, idle;
  assign ebusy = {pol_busy, cd_busy, pw_busy, nt_busy, ts_busy, ga_busy, hj_busy, rn_busy};
  assign idle  = ~(ebusy | pend);

  // ---------------- sequencer ----------------
  // micro-op fetched into a register (one bubble cycle after every pc change, for timing)
  uop_t uo;
  logic uo_vld;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) uo <= '0;
    else        uo <= prog(op_q, pc);
  end

  logic cond_ok;
  assign cond_ok = (uo.cond == 2'd0) || (uo.cond == 2'd1 && !q769) || (uo.cond == 2'd2 && q769);

  function automatic logic role_ready(input logic [4:0] s, input logic [4:0] ok);
    return !s[4] || ok[s[2:0]];
  endfunction
  function automatic logic [3:0] phys(input logic [4:0] s, input logic [4:0][3:0] rs);
    return s[4] ? rs[s[2:0]] : s[3:0];
  endfunction

  logic [3:0] eng;
  always_comb begin
    case (uo.kind)
      K_RNDI:          eng = 4'(E_RNDI);
      K_HJ:            eng = 4'(E_HJ);
      K_GENA:          eng = 4'(E_GENA);
      K_TS:            eng = 4'(E_TS);
      K_NTT:           eng = 4'(E_NTT);
      K_PW:            eng = 4'(E_PW);
      K_CD:            eng = 4'(E_CD);
      K_PENC, K_PDEC:  eng = 4'(E_POL);
      default:         eng = 4'd8;
    endcase
  end

  logic roles_ok;
  assign roles_ok = role_ready(uo.sa, ts_role_ok) && role_ready(uo.sb, ts_role_ok) &&
                    role_ready(uo.sc, ts_role_ok) && role_ready(uo.sd, ts_role_ok);

  logic go, finish;
  always_comb begin
    go = 1'b0; finish = 1'b0;
    if (busy && uo_vld && cond_ok) begin
      case (uo.kind)
        K_END:   finish = (idle == 8'hFF) && !fin_q;
        K_WAIT:  go = ((idle & uo.mask) == uo.mask);
        K_BROVF, K_JMP: go = idle[E_CD];
        default: go = idle[eng[2:0]] && roles_ok;
      endcase
    end
  end

  logic rn_start, hj_start;
  logic [3:0] rn_tgt_n, hj_type_n;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy <= 1'b0; done <= 1'b0; result_valid <= 1'b0; cmd_error <= 1'b0;
      op_q <= '0; pcode <= '0; pc <= '0; pend <= '0; cmd_start <= 1'b0; uo_vld <= 1'b0; fin_q <= 1'b0;
      ga_start <= 1'b0; ts_start <= 1'b0; nt_start <= 1'b0; pw_start <= 1'b0; cd_start <= 1'b0;
      pe_start <= 1'b0; pd_start <= 1'b0; rn_start <= 1'b0; hj_start <= 1'b0;
      ga_tr <= 1'b0; ts_nonce0 <= '0; ts_e2 <= 1'b0; nt_inv <= 1'b0; nt_slot <= '0;
      pw_mode <= '0; pw_acc_zero <= 1'b0; pw_sa <= '0; pw_sb <= '0; pw_sc <= '0; pw_sd <= '0;
      cd_op <= '0; cd_k <= 1'b0; cd_slot <= '0; cd_cmp <= 1'b0; cd_src_sk <= 1'b0;
      rn_tgt_n <= '0; hj_type_n <= '0;
    end else begin
      done <= 1'b0; cmd_start <= 1'b0;
      ga_start <= 1'b0; ts_start <= 1'b0; nt_start <= 1'b0; pw_start <= 1'b0; cd_start <= 1'b0;
      pe_start <= 1'b0; pd_start <= 1'b0; rn_start <= 1'b0; hj_start <= 1'b0;
      pend <= '0;
      if (accept) begin
        result_valid <= 1'b0;
        if (valid_cmd) begin
          busy <= 1'b1; cmd_error <= 1'b0; op_q <= cmd_op; pcode <= cmd_param; pc <= '0; uo_vld <= 1'b0;
          cmd_start <= 1'b1;
        end else begin
          done <= 1'b1; cmd_error <= 1'b1;
        end
      end else if (fin_q) begin
        fin_q <= 1'b0; busy <= 1'b0; done <= 1'b1; result_valid <= 1'b1;
      end else if (busy && !uo_vld) begin
        uo_vld <= 1'b1;
      end else if (busy && uo_vld) begin
        if (!cond_ok) begin
          pc <= pc + 6'd1; uo_vld <= 1'b0;
        end else if (finish) begin
          fin_q <= 1'b1;
        end else if (go) begin
          pc <= pc + 6'd1; uo_vld <= 1'b0;
          if (eng < 4'd8) pend[eng[2:0]] <= 1'b1;
          case (uo.kind)
            K_RNDI: begin rn_start <= 1'b1; rn_tgt_n <= uo.sub; end
            K_HJ:   begin hj_start <= 1'b1; hj_type_n <= uo.sub; end
            K_GENA: begin ga_start <= 1'b1; ga_tr <= uo.f; end
            K_TS:   begin ts_start <= 1'b1; ts_e2 <= uo.f; ts_nonce0 <= uo.k ? ts_nonce_next : 8'd0; end
            K_NTT:  begin nt_start <= 1'b1; nt_inv <= uo.sub[0]; nt_slot <= phys(uo.sa, ts_role_slot); end
            K_PW: begin
              pw_start <= 1'b1; pw_mode <= uo.sub[1:0]; pw_acc_zero <= uo.f;
              pw_sa <= phys(uo.sa, ts_role_slot); pw_sb <= phys(uo.sb, ts_role_slot);
              pw_sc <= phys(uo.sc, ts_role_slot); pw_sd <= phys(uo.sd, ts_role_slot);
            end
            K_CD: begin
              cd_start <= 1'b1; cd_op <= uo.sub; cd_k <= uo.k; cd_slot <= phys(uo.sa, ts_role_slot);
              cd_cmp <= (uo.sub == C_C1_ENC || uo.sub == C_C2_ENC) ? uo.f : 1'b0;
              cd_src_sk <= uo.f;
            end
            K_PENC: pe_start <= 1'b1;
            K_PDEC: pd_start <= 1'b1;
            K_BROVF: if (cd_ovf) pc <= uo.tgt;
            K_JMP:   pc <= uo.tgt;
            default: ;
          endcase
        end
      end
    end
  end

  // ---------------- register file ----------------
  logic [2:0]  rn_reg;
  logic [3:0]  rn_cnt, rn_left;
  logic        hj_wr;
  logic [3:0]  hj_type;
  logic [1023:0] hout;
  logic [1:0]  hcnt;

  // hash output byte offsets
  logic [6:0] first_len;     // bytes of the first destination
  always_comb begin
    first_len = (hj_type == H_EXPAND) ? p_pkseed(pcode) : p_msg(pcode);
  end
  function automatic logic [511:0] lowbytes(input logic [511:0] x, input logic [6:0] n);
    logic [511:0] mk;
    mk = (n >= 7'd64) ? '1 : ((512'd1 << (8 * n)) - 512'd1);
    return x & mk;
  endfunction
  logic [511:0] h_first, h_second;
  always_comb begin
    h_first  = lowbytes(hout[511:0], first_len);
    case (first_len)
      7'd16:   h_second = hout[128 +: 512];
      7'd32:   h_second = hout[256 +: 512];
      default: h_second = hout[512 +: 512];
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rg <= '0; ss_reg <= '0; rg_inv_sd <= 1'b0; rg_inv_ps <= 1'b0;
    end else begin
      rg_inv_sd <= 1'b0; rg_inv_ps <= 1'b0;
      if (rn_start) rg[rn_reg] <= '0;
      if (rnd_valid && rnd_ready) rg[rn_reg][64*rn_cnt[2:0] +: 64] <= rnd_data;
      if (hj_wr) begin
        case (hj_type)
          H_EXPAND: begin
            rg[RG_PS] <= h_first; rg[RG_SD] <= h_second; rg_inv_sd <= 1'b1; rg_inv_ps <= 1'b1;
          end
          H_GENC, H_GDEC: begin
            rg[RG_SC] <= h_first; rg[RG_SD] <= h_second; rg_inv_sd <= 1'b1;
          end
          default: rg[RG_RK] <= h_first;
        endcase
      end
      if (cd_rg_we) begin rg[RG_PS] <= cd_rg_wdata; rg_inv_ps <= 1'b1; end
      if (pd_m_we)  rg[RG_M] <= pd_m_data;
      if (fin_q) begin
        if (op_q == 2'd1)      ss_reg <= rg[RG_SC];
        else if (op_q == 2'd2) ss_reg <= cd_ne ? rg[RG_RK] : rg[RG_SC];
      end
    end
  end

  // ---------------- random intake ----------------
  always_comb begin
    case (rn_tgt_n)
      R_KG:    rn_reg = RG_KG;
      R_Z:     rn_reg = RG_Z;
      default: rn_reg = RG_M;
    endcase
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rn_busy <= 1'b0; rn_cnt <= '0; rn_left <= '0;
    end else if (rn_start) begin
      rn_busy <= 1'b1; rn_cnt <= '0;
      rn_left <= (rn_tgt_n == R_KG) ? 4'd8 : 4'(p_msg(pcode) >> 3);
    end else if (rnd_valid && rnd_ready) begin
      rn_cnt  <= rn_cnt + 4'd1;
      rn_left <= rn_left - 4'd1;
      if (rn_left == 4'd1) rn_busy <= 1'b0;
    end
  end
  assign rnd_ready = rn_busy;

  // ---------------- CTRL hash jobs ----------------
  typedef enum logic [1:0] {HS_IDLE, HS_REQ, HS_COL, HS_WR} hs_t;
  hs_t hs;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      hs <= HS_IDLE; hj_type <= '0; hcnt <= '0; hj_wr <= 1'b0;
    end else begin
      hj_wr <= 1'b0;
      case (hs)
        HS_IDLE: if (hj_start) begin hs <= HS_REQ; hj_type <= hj_type_n; hcnt <= '0; end
        HS_REQ:  if (hj_gnt) hs <= HS_COL;
        HS_COL:  if (xo_valid) begin
                   hcnt <= hcnt + 2'd1;
                   if (xo_last) begin hs <= HS_WR; hj_wr <= 1'b1; end
                 end
        HS_WR:   hs <= HS_IDLE;
        default: hs <= HS_IDLE;
      endcase
    end
  end
  always_ff @(posedge clk) begin
    if (hs == HS_COL && xo_valid) hout[256*hcnt +: 256] <= xo_data;
  end
  assign hj_busy  = (hs != HS_IDLE);
  assign hj_req   = (hs == HS_REQ);
  assign xo_ready = (hs == HS_COL);

  logic [12:0] n4, pklen, ctlen;
  assign n4    = 13'(4 * p_n(pcode));
  assign pklen = p_pklen(pcode);
  assign ctlen = p_ctlen(pcode);
  always_comb begin
    hj_kind0 = 2'd1; hj_reg0 = RG_KG; hj_buf0 = OB_PK; hj_off0 = '0; hj_len0 = 13'd64;
    hj_kind1 = 2'd0; hj_reg1 = '0;    hj_buf1 = OB_PK; hj_off1 = '0; hj_len1 = '0;
    hj_sfx = '0; hj_sfxlen = '0; hj_cache = 2'd0;
    hj_count = 7'((8'(p_pkseed(pcode)) + 8'd64 + 8'd31) >> 5);
    case (hj_type)
      H_GENC, H_GDEC: begin
        hj_reg0 = RG_M; hj_len0 = 13'(p_msg(pcode));
        hj_kind1 = 2'd2; hj_buf1 = (hj_type == H_GDEC) ? OB_SK : OB_PK;
        hj_off1 = (hj_type == H_GDEC) ? n4 : 13'd0; hj_len1 = pklen;
        hj_count = 7'((8'(p_msg(pcode)) + 8'd64 + 8'd31) >> 5);
      end
      H_REJECT: begin
        hj_kind0 = 2'd2; hj_buf0 = OB_SK; hj_off0 = n4 + pklen; hj_len0 = 13'(p_msg(pcode));
        hj_kind1 = 2'd2; hj_buf1 = OB_CT; hj_off1 = '0; hj_len1 = ctlen;
        hj_count = 7'((p_msg(pcode) + 7'd31) >> 5);
      end
      default: ;
    endcase
  end
endmodule
