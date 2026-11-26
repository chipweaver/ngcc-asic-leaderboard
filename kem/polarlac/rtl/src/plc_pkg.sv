// Shared parameter-set constants and system-wide encodings for polarlac_kem_perf.
// pcode: 0 Light, 1 128, 2 256, 3 512, 4 512-Star (same encoding as cmd_param).
package plc_pkg;
  typedef logic [2:0] pcode_t;

  // ---------------- physical polynomial slots ----------------
  localparam int NSLOT = 16;
  localparam logic [3:0] SL_A00 = 4'd0, SL_A01 = 4'd1, SL_A10 = 4'd2, SL_A11 = 4'd3;
  localparam logic [3:0] SL_B0  = 4'd4, SL_B1  = 4'd5;
  localparam logic [3:0] SL_U0  = 4'd6, SL_U1  = 4'd7;
  localparam logic [3:0] SL_POOL0 = 4'd8;           // 8..14: sample pool (7 slots)
  localparam int         NPOOL    = 7;
  localparam logic [3:0] SL_V   = 4'd15;
  // decrypt-phase aliases inside the pool (pool is idle until TSAMP starts)
  localparam logic [3:0] SL_DS0 = 4'd8, SL_DS1 = 4'd9, SL_SU = 4'd10, SL_HM = 4'd11;

  // ---------------- object buffers ----------------
  localparam logic [1:0] OB_PK = 2'd0, OB_SK = 2'd1, OB_CT = 2'd2;

  // ---------------- 64-byte registers (register file in the top) ----------------
  localparam logic [2:0] RG_KG = 3'd0;  // seed_kg (KeyGen)
  localparam logic [2:0] RG_PS = 3'd1;  // pk_seed (16/32/64 B, low bytes)
  localparam logic [2:0] RG_SD = 3'd2;  // seed_se (KeyGen) / seed_enc (Enc/Dec)
  localparam logic [2:0] RG_M  = 3'd3;  // message m
  localparam logic [2:0] RG_SC = 3'd4;  // ss candidate (G output)
  localparam logic [2:0] RG_RK = 3'd5;  // reject key
  localparam logic [2:0] RG_Z  = 3'd6;  // z (KeyGen, from rnd)

  // ---------------- per-set constants ----------------
  function automatic logic [3:0] p_logn(input pcode_t p);
    return (p <= 3'd1) ? 4'd8 : (p == 3'd2) ? 4'd9 : 4'd10;
  endfunction
  function automatic logic [1:0] p_ncode(input pcode_t p);   // 0:N=256 1:512 2:1024
    return (p <= 3'd1) ? 2'd0 : (p == 3'd2) ? 2'd1 : 2'd2;
  endfunction
  function automatic logic p_q769(input pcode_t p);
    return p == 3'd4;
  endfunction
  function automatic logic [11:0] p_n(input pcode_t p);
    return 12'd1 << p_logn(p);
  endfunction
  function automatic logic [7:0] p_nw(input pcode_t p);      // 128-bit words per poly
    return 8'(p_n(p) >> 3);
  endfunction
  function automatic logic [6:0] p_pkseed(input pcode_t p);  // pk_seed bytes
    return (p <= 3'd1) ? 7'd16 : (p == 3'd2) ? 7'd32 : 7'd64;
  endfunction
  function automatic logic [6:0] p_msg(input pcode_t p);     // m bytes = ss bytes
    return (p <= 3'd1) ? 7'd16 : (p == 3'd2) ? 7'd32 : 7'd64;
  endfunction
  function automatic logic [10:0] p_pkpb(input pcode_t p);   // bytes per encoded pk / c1(Star) poly
    case (p)
      3'd0, 3'd1: return 11'd257;
      3'd2:       return 11'd514;
      3'd3:       return 11'd1026;
      default:    return 11'd1229;
    endcase
  endfunction
  function automatic logic [12:0] p_pklen(input pcode_t p);
    return 13'(p_pkseed(p)) + 13'(2 * p_pkpb(p));
  endfunction
  function automatic logic [12:0] p_sklen(input pcode_t p);
    return 13'(4 * p_n(p)) + p_pklen(p) + 13'(p_msg(p));   // s_ntt || pk || z (z = msg bytes)
  endfunction
  function automatic logic [11:0] p_c1pb(input pcode_t p);   // bytes per c1 poly
    return (p == 3'd4) ? 12'd1229 : p_n(p);
  endfunction
  function automatic logic [12:0] p_c1len(input pcode_t p);
    return 13'(2 * p_c1pb(p));
  endfunction
  function automatic logic [2:0] p_dc2(input pcode_t p);
    return (p == 3'd0) ? 3'd3 : 3'd4;
  endfunction
  function automatic logic [12:0] p_c2len(input pcode_t p);
    return (p == 3'd0) ? 13'd96 : 13'(p_n(p) >> 1);
  endfunction
  function automatic logic [12:0] p_ctlen(input pcode_t p);
    return p_c1len(p) + p_c2len(p);
  endfunction
  function automatic logic [12:0] p_thr(input pcode_t p);    // RL_KEM_T (strict <)
    case (p)
      3'd0:    return 13'd576;
      3'd1:    return 13'd900;
      3'd2:    return 13'd1296;
      3'd3:    return 13'd1849;
      default: return 13'd4096;
    endcase
  endfunction
  function automatic logic [9:0] p_ratio(input pcode_t p);
    return (p == 3'd4) ? 10'd385 : 10'd129;
  endfunction
  // ternary sampler kind: 0: 3 bit-planes (Light, 256); 1: 4 planes (128, Star); 2: 7-byte (512)
  function automatic logic [1:0] p_tkind(input pcode_t p);
    case (p)
      3'd0, 3'd2: return 2'd0;
      3'd3:       return 2'd2;
      default:    return 2'd1;
    endcase
  endfunction
  function automatic logic [10:0] p_tbytes(input pcode_t p); // ternary squeeze bytes per poly
    case (p_tkind(p))
      2'd0:    return 11'(3 * (p_n(p) >> 3));
      2'd1:    return 11'(4 * (p_n(p) >> 3));
      default: return 11'(7 * (p_n(p) >> 3));
    endcase
  endfunction
endpackage
