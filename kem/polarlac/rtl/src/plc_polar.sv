// plc_polar: polar encoder (poly.c Encode_m) and table-driven Fast-SSC decoder (poly.c Decode_m /
// polar.c decode_polar) of PolarLAC, bit-exact for all five parameter sets (pcode 0 Light, 1 128,
// 2 256, 3 512, 4 512-Star). Tables: plc_polar_rom_pkg (generated from
// the reference C).
//
// Encoder: enc_start (1-cycle pulse) samples pcode and rg_m (= RG_M); u[info pos k] = m bit k, x = u*F^{(x)n}
// (natural-order butterfly, identical to encode_polar_opt); cw[i] = code_m[i] (bits >= N are 0) is
// valid from the enc_done cycle and held until the next enc_start. enc_start at cycle t: enc_busy at t+1..t+2,
// enc_done at t+2 (cw holds the u vector during t+1).
//
// Decoder: dec_start samples pcode; dec_busy high from the next cycle up to and including the dec_done cycle.
//  LOAD : hatm words are read from slot SL_HM (pl_r* port, 2-cycle latency, word w in bank ^w at addr w>>1),
//         centred exactly like Decode_m (int16 arithmetic) and converted through llr_table (half table +
//         antisymmetry correction, 2 lanes/cycle) into the channel layer of the LLR SRAM.
//  NODES: node_type_matrix is walked in order. Each node: one g stage at layer ctz(start) (f from layer n-1
//         for node 0), then f stages down to the node layer, then a 1-cycle decision. R1 nodes are expanded
//         into their M rate-1 leaves (exact SC = R1_SC_arr), Rep/Type-I sums are accumulated in int64.
//         The decoded message bits are taken in the u domain (Rep: u[M-1]=r; Type-I: u[M-2]=e^o, u[M-1]=o;
//         leaf: decision), which equals encode(x_cap) at data_pos_sorted; the partial sums C[.][0] of each
//         layer are kept in flops and the bit recursion of polar.c is one combinational merge per node.
//  LLR storage: layers 4..n (64-bit LLRs, 8 per word) in two plc_sram_1r1w #(128,512) banks (read data is
//         registered in opa/opb before any logic, so the SRAM access time is not in a logic path); layer l word j
//         lives in bank ^j at address 128-2^(l-3)+(j>>1). Layers 0..3 are flops. A same-address read+write
//         never happens (reads and writes of a cycle are always in different layers).
//  pd_m_we/pd_m_data (byte i at [8i+:8], only p_msg bytes non-zero) are asserted in the dec_done cycle.
// int64 arithmetic: |llr_table| * N < 2^63 for every set, so no intermediate value wraps; f/g are the exact
// polar.c macros (f = sign(a)*sign(b)*min(|a|,|b|), g = (1-2u)*a + b), decisions are (x < 0).
// hatm outside the C-defined index range of llr_table (only possible for hatm outside [0,q)) gives LLR 0.
module plc_polar
  import plc_pkg::*;
  import plc_polar_rom_pkg::*;
(
  input  logic              clk,
  input  logic              rst_n,
  input  logic [2:0]        pcode,
  // encoder
  input  logic              enc_start,
  input  logic [511:0]      rg_m,
  output logic              enc_busy,
  output logic              enc_done,
  output logic [1023:0]     cw,
  // decoder
  input  logic              dec_start,
  output logic              dec_busy,
  output logic              dec_done,
  output logic [3:0]        pl_rslot,
  output logic [1:0]        pl_ren,
  output logic [1:0][5:0]   pl_raddr,
  input  logic [1:0][127:0] pl_rdata,
  output logic              pd_m_we,
  output logic [511:0]      pd_m_data
);

  // =====================================================================================
  // Encoder
  // =====================================================================================
  function automatic logic [1023:0] polar_xform(input logic [1023:0] u);
    logic [1023:0] x;
    x = u;
    for (int s = 0; s < 10; s++)
      for (int i = 0; i < 1024; i++)
        if (((i >> s) & 1) == 0) x[i] = x[i] ^ x[i | (1 << s)];
    return x;
  endfunction

  // start edge: cw <= u (info bits placed); next edge: cw <= u*F (in place); done in the cycle after.
  logic enc_s1;
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin
      enc_s1   <= 1'b0;
      enc_done <= 1'b0;
    end else begin
      enc_s1   <= enc_start;
      enc_done <= enc_s1;
    end
  assign enc_busy = enc_s1 | enc_done;

  always_ff @(posedge clk)
    if (enc_start)   cw <= enc_u(pcode, rg_m);
    else if (enc_s1) cw <= polar_xform(cw);

  // =====================================================================================
  // Decoder helpers
  // =====================================================================================
  function automatic logic [63:0] pe_f(input logic [63:0] a, input logic [63:0] b);
    logic sa, sb, lt, sel;
    logic [63:0] s;
    sa  = a[63];
    sb  = b[63];
    lt  = ($signed(a) < $signed(b));
    s   = a + b;
    // |a| < |b| (ties pick either operand: equal magnitude gives the same result)
    sel = sa ^ ((sa == sb) ? lt : s[63]);
    return sel ? (sb ? (64'd0 - a) : a) : (sa ? (64'd0 - b) : b);
  endfunction

  // g = (1-2u)*a + b where (a, b) = (x0, x1) if !swp else (x1, x0); computed as one adder with
  // conditional operand inversion: n0/n1 select which operand is subtracted.
  function automatic logic [63:0] pe_g(input logic n0, input logic n1, input logic [63:0] x0,
                                       input logic [63:0] x1);
    logic [64:0] s;
    s = {x0 ^ {64{n0}}, n0 | n1} + {x1 ^ {64{n1}}, n0 | n1};     // + carry-in (n0|n1)
    return s[64:1];
  endfunction

  // 3:2 carry-save compressor
  function automatic logic [127:0] csa3(input logic [63:0] a, input logic [63:0] b, input logic [63:0] c);
    return {((a & b) | (a & c) | (b & c)) << 1, a ^ b ^ c};
  endfunction

  function automatic logic [3:0] ctz11(input logic [10:0] v);
    logic [3:0] r;
    r = 4'd0;
    for (int i = 10; i >= 0; i--) if (v[i]) r = 4'(i);
    return r;
  endfunction

  // base address of LLR layer l (4..10) in each bank
  function automatic logic [6:0] lay_base(input logic [3:0] l);
    return 7'(8'd128 - (8'd1 << (l - 4'd3)));
  endfunction

  // =====================================================================================
  // Decoder state
  // =====================================================================================
  typedef enum logic [2:0] {D_IDLE, D_LOAD, D_STG, D_ACC, D_DEC, D_FIN} dst_t;
  dst_t dst;

  logic [2:0]  pc;           // latched pcode
  logic [3:0]  nlog;         // log2 N
  logic        q769;
  assign nlog = p_logn(pc);
  assign q769 = (pc == 3'd4);

  // node walk
  logic [7:0]  ni;           // node index
  logic [2:0]  nm;           // log2 size of node ni
  logic [1:0]  nty;          // type of node ni (0 R0, 1 R1, 2 Rep, 3 Type-I)
  logic [4:0]  rom_q;        // node_rom(pc, ni+1)
  logic [9:0]  cst;          // current (sub-)node start
  logic [2:0]  cm;           // current log2 size (0 for R1 leaves)
  logic [1:0]  cty;          // 0 R0, 1 leaf, 2 Rep, 3 Type-I

  // stage engine
  logic [3:0]  sl;           // layer being computed
  logic        sg;           // g (1) / f (0)
  logic        sissue;       // SRAM read issue phase
  logic [6:0]  sj;           // issue word counter
  // pipeline: p0 = SRAM data cycle (read data registered into opa/opb, no logic), p1 = PE cycle,
  // p2 = SRAM write cycle
  logic        p0_v, p0_g, p0_last, p0_acc;
  logic [3:0]  p0_l;
  logic [6:0]  p0_j;
  logic [7:0]  p0_n0, p0_n1;
  logic        p1_v, p1_g, p1_last, p1_acc;
  logic [3:0]  p1_l;
  logic [6:0]  p1_j;
  logic [7:0]  p1_n0, p1_n1;
  logic [511:0] opa, opb;     // registered SRAM read data (bank 0 / bank 1)
  logic        p2_v, p2_last, p2_acc;
  logic [3:0]  p2_l;
  logic [6:0]  p2_j;

  // LLR flops (layers 0..3) and the SRAM write staging register
  logic [7:0][63:0] L3;
  logic [3:0][63:0] L2;
  logic [1:0][63:0] L1;
  logic [63:0]      L0;
  logic [7:0][63:0] obuf;
  // Rep / Type-I accumulators
  logic [63:0] acc_e, acc_o, acc_t;
  // partial sums C[.][0] of layers 0..9 (layer l at offset 2^l-1), message shift register
  logic [1022:0] C0;
  logic [511:0]  msr;

  // load
  logic [11:0] lc;
  logic [7:0][9:0] hidx;     // llr_table index (for the correction table)
  logic [7:0][8:0] hjj;      // half-table index
  logic [7:0]      hok, hneg;
  logic [1:0][55:0] rom_h;
  logic [1:0]       rom_neg, rom_ok;
  logic [1:0][3:0]  rom_cor;

  // SRAM ports
  logic [1:0]       m_ren, m_wen;
  logic [1:0][6:0]  m_raddr, m_waddr;
  logic [1:0][511:0] m_rdata;

  plc_sram_1r1w #(.DEPTH(128), .WIDTH(512)) u_llr0 (
    .clk(clk), .ren(m_ren[0]), .raddr(m_raddr[0]), .rdata(m_rdata[0]),
    .wen(m_wen[0]), .waddr(m_waddr[0]), .wdata(obuf));
  plc_sram_1r1w #(.DEPTH(128), .WIDTH(512)) u_llr1 (
    .clk(clk), .ren(m_ren[1]), .raddr(m_raddr[1]), .rdata(m_rdata[1]),
    .wen(m_wen[1]), .waddr(m_waddr[1]), .wdata(obuf));

  // =====================================================================================
  // LOAD schedule (lc = cycle in LOAD): word w read at lc=4w, raw word captured at 4w+2, index computed at
  // 4w+3, ROM groups at 4w+4..4w+7, correction adder at 4w+5..4w+8, SRAM write at 4w+9. Ends at lc = N/2+5.
  // =====================================================================================
  logic [11:0] nh;
  assign nh = 12'd1 << (nlog - 4'd1);       // N/2 = 4*NW
  logic [11:0] lcm4, lcm5, lcm9;
  assign lcm4 = lc - 12'd4;
  assign lcm5 = lc - 12'd5;
  assign lcm9 = lc - 12'd9;
  logic ld, ld_iss, ld_rom, ld_add, ld_wr, ld_end;
  logic iss_d1, iss_d2, iss_d3, bank_d1, bank_d2;   // issue delayed: d2 = data arrival, d3 = index stage
  assign ld     = (dst == D_LOAD);
  assign ld_iss = ld && (lc[1:0] == 2'd0) && (lc < nh);
  assign ld_rom = ld && (lc >= 12'd4) && (lcm4 < nh);
  assign ld_add = ld && (lc >= 12'd5) && (lcm5 < nh);
  assign ld_wr  = ld && (lc >= 12'd9) && (lcm9[1:0] == 2'd0) && (lcm9 < nh);
  assign ld_end = ld && (lc == nh + 12'd5);

  logic [6:0] w_iss, w_wr;
  assign w_iss = lc[8:2];
  assign w_wr  = lcm9[8:2];
  logic [127:0] hraw;

  assign pl_rslot = SL_HM;
  always_comb begin
    pl_ren   = '0;
    pl_raddr = '0;
    if (ld_iss) begin
      pl_ren[^w_iss]   = 1'b1;
      pl_raddr[^w_iss] = w_iss[6:1];
    end
  end

  // centring of Decode_m: c = (int16)(hatm - q/4); wrap by q into [-RATIO, RATIO-1]; idx = c + RATIO - 1
  function automatic logic [10:0] hm_idx(input logic [15:0] h, input logic big);
    logic [15:0] h2, qv, rm1, c1, c;
    logic [16:0] ix;
    h2  = big ? 16'd193 : 16'd65;
    qv  = big ? 16'd769 : 16'd257;
    rm1 = big ? 16'd384 : 16'd128;
    c1  = h - h2;
    if ($signed(c1) > $signed(rm1))             c = c1 - qv;
    else if ($signed(c1) < -$signed(rm1 + 16'd1)) c = c1 + qv;
    else                                         c = c1;
    ix = {c[15], c} + {1'b0, rm1};
    // valid iff 0 <= ix <= 2*RATIO-2
    return {(!ix[16] && (ix[15:0] <= {rm1[14:0], 1'b0})), ix[9:0]};
  endfunction

  logic [7:0][10:0] cap_n;
  logic [7:0][9:0]  cap_jj;
  logic [9:0]       zc;
  logic [1:0][55:0] add_v;
  assign zc = q769 ? 10'd384 : 10'd128;
  always_comb begin
    for (int k = 0; k < 8; k++) begin
      cap_n[k]  = hm_idx(hraw[16*k +: 16], q769);
      cap_jj[k] = (cap_n[k][9:0] > zc) ? ({zc[8:0], 1'b0} - cap_n[k][9:0]) : cap_n[k][9:0];
    end
    for (int i = 0; i < 2; i++) begin
      logic [4:0] k1;
      k1 = {rom_cor[i][3], rom_cor[i]} + 5'd1;          // 1 + corr (signed)
      add_v[i] = rom_neg[i] ? (~rom_h[i] + {{51{k1[4]}}, k1}) : rom_h[i];
    end
  end

  always_ff @(posedge clk) begin
    if (iss_d2) hraw <= pl_rdata[bank_d2];
    // index stage; each ROM cycle consumes lanes 0,1 and shifts the rest down by two
    if (iss_d3) begin
      for (int k = 0; k < 8; k++) begin
        hidx[k] <= cap_n[k][9:0];
        hok[k]  <= cap_n[k][10];
        hneg[k] <= (cap_n[k][9:0] > zc);
        hjj[k]  <= cap_jj[k][8:0];
      end
    end else if (ld_rom) begin
      for (int k = 0; k < 6; k++) begin
        hidx[k] <= hidx[k+2];
        hok[k]  <= hok[k+2];
        hneg[k] <= hneg[k+2];
        hjj[k]  <= hjj[k+2];
      end
    end
    if (ld_rom) begin
      for (int i = 0; i < 2; i++) begin
        rom_h[i]   <= llr_half(pc, hjj[i]);
        rom_neg[i] <= hneg[i];
        rom_cor[i] <= llr_corr(pc, hidx[i]);
        rom_ok[i]  <= hok[i];
      end
    end
  end

  // =====================================================================================
  // Processing elements (8 lanes)
  // =====================================================================================
  logic [7:0][63:0] pa, pb, py;   // pa/pb: x0/x1 operands (bank 0 / bank 1 for SRAM stages)
  logic [7:0]       pn0, pn1;      // per-lane g operand inversion (u=1: subtract x0 / x1)
  logic             pop_g;
  logic             fstg;          // flop stage (layers 0..2) active this cycle
  assign fstg = (dst == D_STG) && (sl <= 4'd2);

  always_comb begin
    pa = '0;
    pb = '0;
    pn0 = '0;
    pn1 = '0;
    pop_g = sg;
    if (p1_v) begin
      pop_g = p1_g;
      pn0   = p1_n0;               // computed at issue (first-half word in bank 1: operands swapped)
      pn1   = p1_n1;
      for (int k = 0; k < 8; k++) begin
        pa[k] = opa[64*k +: 64];
        pb[k] = opb[64*k +: 64];
      end
    end else begin
      case (sl[1:0])
        2'd2: for (int k = 0; k < 4; k++) begin pa[k] = L3[k]; pb[k] = L3[k+4]; pn0[k] = C0[3+k]; end
        2'd1: for (int k = 0; k < 2; k++) begin pa[k] = L2[k]; pb[k] = L2[k+2]; pn0[k] = C0[1+k]; end
        default: begin pa[0] = L1[0]; pb[0] = L1[1]; pn0[0] = C0[0]; end
      endcase
    end
    for (int k = 0; k < 8; k++)
      py[k] = pop_g ? pe_g(pn0[k], pn1[k], pa[k], pb[k]) : pe_f(pa[k], pb[k]);   // f is symmetric
  end

  // C0 bits for the g stage at layer sl (>= 3), word sj
  logic [7:0] c0w;
  always_comb begin
    c0w = '0;
    for (int l = 3; l <= 9; l++)
      if (sl == 4'(l)) c0w = C0[((1 << l) - 1) + 8 * sj +: 8];
  end

  // SRAM read issue (stage reads layer sl+1, words sj and sj+Wo)
  logic [3:0] li;
  logic [6:0] wo;
  logic [6:0] ja, jb;
  logic       iss, pa_bank;
  assign li  = sl + 4'd1;
  assign wo  = 7'd1 << (sl - 4'd3);
  assign ja  = sj;
  assign jb  = sj | wo;
  assign iss = (dst == D_STG) && sissue;
  assign pa_bank = ^ja;

  always_comb begin
    m_ren   = '0;
    m_raddr = '0;
    if (iss) begin
      m_ren = 2'b11;
      m_raddr[pa_bank]  = lay_base(li) + 7'(ja >> 1);
      m_raddr[!pa_bank] = lay_base(li) + 7'(jb >> 1);
    end
  end

  always_comb begin
    m_wen   = '0;
    m_waddr = '0;
    if (ld_wr) begin
      m_wen[^w_wr]   = 1'b1;
      m_waddr[^w_wr] = lay_base(nlog) + 7'(w_wr >> 1);
    end else if (p2_v) begin
      m_wen[^p2_j]   = 1'b1;
      m_waddr[^p2_j] = lay_base(p2_l) + 7'(p2_j >> 1);
    end
  end

  // =====================================================================================
  // Accumulators (Rep / Type-I)
  // =====================================================================================
  // obuf holds the last stage output (unused lanes of flop stages are 0), so it is the only source.
  logic             acc_en;
  logic [63:0]      acc_e_n, acc_o_n, acc_t_n;
  assign acc_en = (dst == D_ACC) || (p2_v && p2_acc);
  always_comb begin
    logic [127:0] e1, e2, e3, o1, o2, o3, t1, t2, t3, t4, t5, t6, t7;
    e1 = csa3(obuf[0], obuf[2], obuf[4]);
    e2 = csa3(obuf[6], acc_e, e1[63:0]);
    e3 = csa3(e1[127:64], e2[63:0], e2[127:64]);
    acc_e_n = e3[63:0] + e3[127:64];
    o1 = csa3(obuf[1], obuf[3], obuf[5]);
    o2 = csa3(obuf[7], acc_o, o1[63:0]);
    o3 = csa3(o1[127:64], o2[63:0], o2[127:64]);
    acc_o_n = o3[63:0] + o3[127:64];
    t1 = csa3(obuf[0], obuf[1], obuf[2]);
    t2 = csa3(obuf[3], obuf[4], obuf[5]);
    t3 = csa3(obuf[6], obuf[7], acc_t);
    t4 = csa3(t1[63:0], t1[127:64], t2[63:0]);
    t5 = csa3(t2[127:64], t3[63:0], t3[127:64]);
    t6 = csa3(t4[63:0], t4[127:64], t5[63:0]);
    t7 = csa3(t6[63:0], t6[127:64], t5[127:64]);
    acc_t_n = t7[63:0] + t7[127:64];
  end

  // =====================================================================================
  // Decision, bit recursion (merge) and next-node logic (D_DEC)
  // =====================================================================================
  logic xe, xo;                 // codeword bit of even / odd local positions
  always_comb begin
    case (cty)
      2'd1:    begin xe = L0[63];    xo = L0[63];    end   // leaf: P0 < 0
      2'd2:    begin xe = acc_t[63]; xo = acc_t[63]; end   // Rep
      2'd3:    begin xe = acc_e[63]; xo = acc_o[63]; end   // Type-I
      default: begin xe = 1'b0;      xo = 1'b0;      end   // R0
    endcase
  end

  logic [10:0] cend;            // cst + M
  logic [3:0]  lt;              // target layer of the merge
  logic        psi, fin, r1_more;
  assign cend    = {1'b0, cst} + (11'd1 << cm);
  assign lt      = ctz11(cend);
  assign psi     = cst[cm];
  assign fin     = (cend == (11'd1 << nlog));
  assign r1_more = (nty == 2'd1) && ((cend[9:0] & ((10'd1 << nm) - 10'd1)) != 10'd0);

  // merge network: mg[L][p] = new C0 of layer L when the right child at layer cm completes. The bit
  // recursion of polar.c (layers cm..lt-1, C[.][1] chain) unrolls to
  //   mg[L][p] = x[p&1] ^ XOR_{z = cm..L-1, bit z of p == 0} C0_z[p mod 2^z]
  // (for cm = 0 the node is a leaf and xe == xo).
  logic [511:0] mg [10];
  logic [9:0]   zen;          // zen[z] = (z >= cm)
  always_comb begin
    for (int z = 0; z < 10; z++) zen[z] = (z >= int'(cm));
    for (int L = 0; L < 10; L++) mg[L] = '0;
    for (int L = 1; L < 10; L++) begin
      for (int p = 0; p < (1 << L); p++) begin
        logic v;
        v = (p % 2 == 0) ? xe : xo;
        for (int z = 0; z < L; z++)
          if (((p >> z) & 1) == 0) v = v ^ (zen[z] & C0[(1 << z) - 1 + (p & ((1 << z) - 1))]);
        mg[L][p] = v;
      end
    end
  end

  logic [9:0] cst_n;
  assign cst_n = cend[9:0];

  // =====================================================================================
  // Control FSM
  // =====================================================================================
  logic [4:0] e0;
  assign e0 = node_rom(pc, 8'd0);
  logic adv;   // current stage completes this cycle
  assign adv = (dst == D_STG) && (fstg || (p1_v && (p1_l == 4'd3)) || (p2_v && p2_last));

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dst    <= D_IDLE;
      pc     <= 3'd0;
      lc     <= '0;
      ni     <= '0;
      nm     <= '0;
      nty    <= '0;
      cst    <= '0;
      cm     <= '0;
      cty    <= '0;
      sl     <= '0;
      sg     <= 1'b0;
      sissue <= 1'b0;
      sj     <= '0;
      p0_v   <= 1'b0;
      p1_v   <= 1'b0;
      p2_v   <= 1'b0;
      iss_d1 <= 1'b0;
      iss_d2 <= 1'b0;
      iss_d3 <= 1'b0;
      bank_d1 <= 1'b0;
      bank_d2 <= 1'b0;
    end else begin
      p0_v <= iss;
      p1_v <= p0_v;
      iss_d1  <= ld_iss;
      iss_d2  <= iss_d1;
      iss_d3  <= iss_d2;
      bank_d1 <= ^w_iss;
      bank_d2 <= bank_d1;
      p2_v <= p1_v && (p1_l != 4'd3);
      case (dst)
        D_IDLE: if (dec_start) begin
          dst <= D_LOAD;
          pc  <= pcode;
          lc  <= '0;
        end
        D_LOAD: begin
          lc <= lc + 12'd1;
          if (ld_end) begin
            ni     <= 8'd0;
            nm     <= e0[4:2];
            nty    <= e0[1:0];
            cst    <= '0;
            cm     <= (e0[1:0] == 2'd1) ? 3'd0 : e0[4:2];
            cty    <= e0[1:0];
            sl     <= nlog - 4'd1;
            sg     <= 1'b0;
            sissue <= 1'b1;
            sj     <= '0;
            dst    <= D_STG;
          end
        end
        D_STG: begin
          if (iss) begin
            sj <= sj + 7'd1;
            if (sj == wo - 7'd1) sissue <= 1'b0;
          end
          if (adv) begin
            if (sl == {1'b0, cm}) begin
              dst <= (cty[1] && (cm <= 3'd3)) ? D_ACC : D_DEC;
            end else begin
              sl     <= sl - 4'd1;
              sg     <= 1'b0;
              sissue <= (sl - 4'd1) >= 4'd3;
              sj     <= '0;
            end
          end
        end
        D_ACC: dst <= D_DEC;
        D_DEC: begin
          if (fin) begin
            dst <= D_FIN;
          end else begin
            cst <= cst_n;
            if (r1_more) begin
              cm  <= 3'd0;
              cty <= 2'd1;
            end else begin
              ni  <= ni + 8'd1;
              nm  <= rom_q[4:2];
              nty <= rom_q[1:0];
              cm  <= (rom_q[1:0] == 2'd1) ? 3'd0 : rom_q[4:2];
              cty <= rom_q[1:0];
            end
            sl     <= ctz11({1'b0, cst_n});
            sg     <= 1'b1;
            sissue <= ctz11({1'b0, cst_n}) >= 4'd3;
            sj     <= '0;
            dst    <= D_STG;
          end
        end
        D_FIN: dst <= D_IDLE;
        default: dst <= D_IDLE;
      endcase
    end
  end

  assign dec_busy  = (dst != D_IDLE);
  assign dec_done  = (dst == D_FIN);
  assign pd_m_we   = (dst == D_FIN);
  always_comb begin
    case (pc)
      3'd0, 3'd1: pd_m_data = {384'd0, msr[511:384]};
      3'd2:       pd_m_data = {256'd0, msr[511:256]};
      default:    pd_m_data = msr;
    endcase
  end

  // =====================================================================================
  // Datapath registers (no reset)
  // =====================================================================================
  always_ff @(posedge clk) begin
    rom_q <= node_rom(pc, ni + 8'd1);

    // pipeline payload
    if (iss) begin
      p0_l    <= sl;
      p0_g    <= sg;
      p0_j    <= sj;
      p0_n0   <= c0w & {8{sg & !pa_bank}};
      p0_n1   <= c0w & {8{sg &  pa_bank}};
      p0_last <= (sj == wo - 7'd1);
      p0_acc  <= (sl == {1'b0, cm}) && cty[1];
    end
    if (p0_v) begin
      p1_l    <= p0_l;
      p1_g    <= p0_g;
      p1_j    <= p0_j;
      p1_n0   <= p0_n0;
      p1_n1   <= p0_n1;
      p1_last <= p0_last;
      p1_acc  <= p0_acc;
      opa     <= m_rdata[0];
      opb     <= m_rdata[1];
    end
    if (p1_v) begin
      p2_l    <= p1_l;
      p2_j    <= p1_j;
      p2_last <= p1_last;
      p2_acc  <= p1_acc;
    end

    // PE results
    if (p1_v || fstg) obuf <= py;
    if (p1_v) begin
      if (p1_l == 4'd3) L3 <= py;
    end else if (fstg) begin
      case (sl[1:0])
        2'd2:    L2 <= py[3:0];
        2'd1:    L1 <= py[1:0];
        default: L0 <= py[0];
      endcase
    end

    // load: correction adder into the staging register
    // (lanes enter at the top and shift down by two: after 4 groups lane k holds coefficient k)
    if (ld_add) begin
      obuf[5:0] <= obuf[7:2];
      for (int i = 0; i < 2; i++)
        obuf[6 + i] <= rom_ok[i] ? {{8{add_v[i][55]}}, add_v[i]} : 64'd0;
    end

    // accumulators
    if (dst == D_DEC || ld_end) begin
      acc_e <= '0;
      acc_o <= '0;
      acc_t <= '0;
    end else if (acc_en) begin
      acc_e <= acc_e_n;
      acc_o <= acc_o_n;
      acc_t <= acc_t_n;
    end

    // decision: message bits and partial sums
    if (dst == D_DEC) begin
      case (cty)
        2'd1, 2'd2: msr <= {xe, msr[511:1]};
        2'd3:       msr <= {xo, xe ^ xo, msr[511:2]};
        default: ;
      endcase
      for (int L = 0; L < 10; L++) begin
        if (!psi && ({1'b0, cm} == 4'(L))) begin
          for (int p = 0; p < (1 << L); p++) C0[(1 << L) - 1 + p] <= (p % 2 == 0) ? xe : xo;
        end
        if (psi && !fin && (lt == 4'(L)) && (L >= 1)) begin
          for (int p = 0; p < (1 << L); p++) C0[(1 << L) - 1 + p] <= mg[L][p];
        end
      end
    end
  end

endmodule
