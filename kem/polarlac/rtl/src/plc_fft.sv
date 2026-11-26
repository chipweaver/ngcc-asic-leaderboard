// plc_fft - spectral-rejection check, bit-exact with fft.c fft_within_bound_int16(a, RL_KEM_T)
// of every parameter set (int16 fixed-point "unique half" real FFT, ci16_mul_tw with rsh32(.,8),
// final rsh32(.,8) and re^2+im^2 < T for all N/2 outputs).
//
// Algorithm (derived from the macro-generated REAL_FFT_FN recursion):
//   value X_{n,r}[j] (sub-transform of length n, offset r, n/2 outputs) is kept at index j*(N/n)+r of
//   an N/2-entry complex int16 buffer.  Then every recursion level is an in-place radix-2 pass:
//     base (n=2):  idx r          <- (int16)(a[r]<<8), (int16)(a[r+N/2]<<8)        (n2 incl. tw[1] = j*256)
//     level n:     S = N/n = 2^b, for idx_E with bit b = 0, idx_O = idx_E + S, k = idx_E >> (b+1)
//                  t = mul_tw(X[idx_O], tw[n/2 + k]);  X[idx_E] = X_E + t;  X[idx_O] = X_E - t   (int16 wrap)
//   for b = m-1 .. 0 (m = log2(N/2)); after b = 0 the buffer holds out[] in natural order and every
//   output is checked (order irrelevant).
// Datapath: 4 butterflies per cycle on a 3-bit index cube {b,1,0} (b>=2), {1,0,2} (b=1), {0,1,2} (b=0);
//   8 x plc_sram_1r1w #(64,32) banks, bank(idx) = {^idx[m-1:2], idx[1:0]}, addr = idx[m-1:3], so every
//   cube hits all 8 banks once (conflict free for read and in-place write).
//   Twiddles: plc_tsamp_fft_pkg::tw4 (tw[4a], tw[4a+2]); odd entries tw[2i+1] = j*tw[2i] = (-im, re).
// Passes: load (N/16 cycles, slot words w and w+NW/2 per cycle) + m butterfly passes of N/16 cycles,
//   5 idle cycles between passes (pipeline drain, in-place RAW safety).  start->done: 172 / 337 / 694
//   cycles for N = 256 / 512 / 1024.
// Pipeline (cycle after issue; register names carry the index of the cycle that consumes them):
//   c0 issue: cube addresses, lane selects, twiddle index -> s0 regs
//   c1 read requests on the buffer banks / slot port; twiddle ROM -> tw1
//   c2 buffer read data, bank->lane routing -> E2/O2; Booth digits of the twiddles -> bre2/bim2/bnim2
//   c3 radix-4 Booth partial products + explicit 3:2 tree -> pre3/pim3 (carry-save); slot data (load) -> wl3
//   c4 final add + rsh32(.,8) -> tre4/tim4
//   c5 E +/- t, lane->bank routing -> wd5
//   c6 in-place buffer write (last pass instead: |rsh32(.,8)| -> mag6), c7 squares/compare -> f7, c8 accumulate
// Arbitrary int16 inputs are handled exactly like the C ((int16)(a<<8), int16 wrap of every add).
module plc_fft (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              start,
  input  logic [2:0]        pcode,
  input  logic [3:0]        slot,
  output logic              busy,
  output logic              done,
  output logic              pass,       // valid with done, held until the next start
  // slot read port (2-cycle read latency)
  output logic [3:0]        ff_rslot,
  output logic [1:0]        ff_ren,
  output logic [1:0][5:0]   ff_raddr,
  input  logic [1:0][127:0] ff_rdata
);
  import plc_pkg::*;
  import plc_tsamp_fft_pkg::*;

  // ---------------------------------------------------------------- control
  logic        act, issuing, is_load;
  logic [3:0]  m_r;            // log2(N/2): 7, 8, 9
  logic [3:0]  b_r;            // partner bit of the current butterfly pass
  logic [5:0]  cc;             // cube / load-word counter
  logic [5:0]  cc_last;        // N/16 - 1
  logic [2:0]  gap;
  logic [12:0] thr_r;
  logic [3:0]  slot_r;
  logic        fail_acc, done_q, pass_q;
  logic        chk_fail;       // stage-7 check result (registered fail of the last check stage)

  wire last_issue = issuing && (cc == cc_last);
  wire fin_pass   = !is_load && (b_r == 4'd0);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      act <= 1'b0; issuing <= 1'b0; is_load <= 1'b0; m_r <= 4'd7; b_r <= '0; cc <= '0; cc_last <= '0;
      gap <= '0; thr_r <= '0; slot_r <= '0; fail_acc <= 1'b0; done_q <= 1'b0; pass_q <= 1'b0;
    end else begin
      done_q <= 1'b0;
      if (start && !act) begin
        act      <= 1'b1;
        issuing  <= 1'b1;
        is_load  <= 1'b1;
        cc       <= '0;
        m_r      <= p_logn(pcode) - 4'd1;
        cc_last  <= 6'((p_n(pcode) >> 4) - 12'd1);
        thr_r    <= p_thr(pcode);
        slot_r   <= slot;
        fail_acc <= 1'b0;
      end else if (act) begin
        if (chk_fail) fail_acc <= 1'b1;
        if (issuing) begin
          cc <= cc + 6'd1;
          if (last_issue) begin
            issuing <= 1'b0;
            gap     <= fin_pass ? 3'd7 : 3'd4;
          end
        end else if (gap != 3'd0) begin
          gap <= gap - 3'd1;
        end else if (fin_pass) begin
          // last check result is in stage 5 this cycle
          act    <= 1'b0;
          done_q <= 1'b1;
          pass_q <= !(fail_acc | chk_fail);
        end else begin
          issuing <= 1'b1;
          cc      <= '0;
          if (is_load) begin
            is_load <= 1'b0;
            b_r     <= m_r - 4'd1;
          end else begin
            b_r     <= b_r - 4'd1;
          end
        end
      end
    end
  end

  assign busy = act | done_q;
  assign done = done_q;
  assign pass = pass_q;

  // ---------------------------------------------------------------- issue (combinational)
  // butterfly cube geometry
  logic [1:0] bt_i;            // 2: b>=2, 1: b==1, 0: b==0
  logic [8:0] base_i, idxo_i;
  logic [5:0] addrE_i, addrO_i;
  logic       par_i;
  logic [9:0] tidx_i;          // twiddle index of lane 0

  always_comb begin
    logic [8:0] u9, lo_mask;
    logic [3:0] bm2;
    bt_i   = (b_r >= 4'd2) ? 2'd2 : 2'(b_r);
    u9     = {3'b000, cc};
    bm2    = b_r - 4'd2;
    lo_mask = (9'd1 << bm2) - 9'd1;
    if (bt_i == 2'd2) base_i = ((u9 >> bm2) << (b_r + 4'd1)) | ((u9 & lo_mask) << 2);
    else              base_i = u9 << 3;
    idxo_i  = base_i | (9'd1 << b_r);
    addrE_i = base_i[8:3];
    addrO_i = idxo_i[8:3];
    par_i   = ^base_i[8:2];
    case (bt_i)
      2'd2:    tidx_i = (10'd1 << (m_r - b_r)) + 10'(u9 >> bm2);
      2'd1:    tidx_i = (10'd1 << (m_r - 4'd1)) + 10'(u9 << 1);
      default: tidx_i = (10'd1 << m_r) + 10'(u9 << 2);
    endcase
  end

  // ---------------------------------------------------------------- stage 0 (registered requests)
  function automatic logic [6:0] p_nw_half(input logic [3:0] m);   // NW/2 = N/16 = 2^(m-3)
    return 7'd1 << (m - 4'd3);
  endfunction
  logic             v0, ld0, fin0;
  logic [7:0][5:0]  ba0;       // buffer bank read addresses (= in-place write addresses)
  logic [5:0]       w0;        // load word
  logic             sb0;       // slot bank holding word w
  logic [1:0]       bt0;
  logic [9:0]       tidx0;
  logic [3:0][2:0]  ke0, ko0;  // per-lane bank of E / O (decoded at issue)
  logic [7:0][2:0]  ws0;       // per-bank write-back source {odd, lane}
  logic [1:0]       sren0;
  logic [1:0][5:0]  sraddr0;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v0 <= 1'b0; ld0 <= 1'b0; fin0 <= 1'b0; sren0 <= '0;
    end else begin
      v0    <= issuing;
      ld0   <= is_load;
      fin0  <= issuing && fin_pass;
      sren0 <= (issuing && is_load) ? 2'b11 : 2'b00;
    end
  end

  always_ff @(posedge clk) begin
    logic sb;
    logic [6:0] whi;
    bt0   <= bt_i;
    tidx0 <= tidx_i;
    w0    <= cc;
    sb    = ^cc;                        // bank of slot word w (w < NW/2: bit 6 is 0)
    sb0   <= sb;
    whi   = {1'b0, cc} | 7'(p_nw_half(m_r));
    sraddr0[sb]  <= cc >> 1;
    sraddr0[~sb] <= whi[6:1];
    for (int k = 0; k < 8; k++) begin
      if (bt_i == 2'd2) ba0[k] <= (k[2] == par_i) ? addrE_i : addrO_i;
      else              ba0[k] <= cc;
    end
    for (int j = 0; j < 4; j++) begin
      case (bt_i)
        2'd2:    begin ke0[j] <= {par_i, 2'(j)};             ko0[j] <= {~par_i, 2'(j)}; end
        2'd1:    begin ke0[j] <= {par_i ^ j[1], 1'b0, j[0]}; ko0[j] <= {par_i ^ j[1], 1'b1, j[0]}; end
        default: begin ke0[j] <= {par_i ^ j[1], j[0], 1'b0}; ko0[j] <= {par_i ^ j[1], j[0], 1'b1}; end
      endcase
    end
    for (int k = 0; k < 8; k++) begin
      logic [2:0] kk;
      kk = 3'(k);
      case (bt_i)
        2'd2:    ws0[k] <= {kk[2] != par_i, kk[1:0]};
        2'd1:    ws0[k] <= {kk[1], kk[2] ^ par_i, kk[0]};
        default: ws0[k] <= {kk[0], kk[2] ^ par_i, kk[1]};
      endcase
    end
  end

  assign ff_rslot = slot_r;
  assign ff_ren   = sren0;
  assign ff_raddr = sraddr0;

  // twiddle ROM (stage 0 -> tw1): lane twiddles from tw[4a], tw[4a+2] and tw[2i+1] = j*tw[2i]
  function automatic logic [19:0] mulj(input logic [19:0] t);  // j*(re,im) = (-im, re)
    return {10'(-$signed(t[9:0])), t[19:10]};
  endfunction
  // radix-4 Booth digits of a 10-bit signed twiddle component: 5 digits {neg, two, one}
  function automatic logic [14:0] booth10(input logic [9:0] y);
    logic [10:0] ye;
    ye = {y, 1'b0};
    for (int i = 0; i < 5; i++) begin
      logic [2:0] t;
      t = ye[2*i +: 3];                       // {y[2i+1], y[2i], y[2i-1]}
      booth10[3*i +: 3] = {t[2], (t == 3'b011) || (t == 3'b100), t[1] ^ t[0]};
    end
  endfunction
  logic [3:0][19:0] tw1;       // per butterfly lane {re[9:0], im[9:0]}
  logic [3:0][14:0] bre2, bim2, bnim2;   // Booth digits of tw.re, tw.im, -tw.im
  always_ff @(posedge clk) begin
    logic [39:0] rom;
    logic [19:0] ee, eo, bsel;
    rom  = tw4(tidx0[9:2]);
    ee   = rom[39:20];
    eo   = rom[19:0];
    bsel = tidx0[1] ? eo : ee;
    case (bt0)
      2'd2:    for (int j = 0; j < 4; j++) tw1[j] <= tidx0[0] ? mulj(bsel) : bsel;
      2'd1:    begin tw1[0] <= bsel; tw1[1] <= bsel; tw1[2] <= mulj(bsel); tw1[3] <= mulj(bsel); end
      default: begin tw1[0] <= ee; tw1[1] <= mulj(ee); tw1[2] <= eo; tw1[3] <= mulj(eo); end
    endcase
    for (int j = 0; j < 4; j++) begin
      bre2[j]  <= booth10(tw1[j][19:10]);
      bim2[j]  <= booth10(tw1[j][9:0]);
      bnim2[j] <= booth10(10'(-$signed(tw1[j][9:0])));
    end
  end

  // ---------------------------------------------------------------- work buffer
  logic [7:0]       bren, bwen;
  logic [7:0][5:0]  bwaddr;
  logic [7:0][31:0] brdata, bwdata;
  assign bren = (v0 && !ld0) ? 8'hFF : 8'h00;

  for (genvar k = 0; k < 8; k++) begin : g_bank
    plc_sram_1r1w #(.DEPTH(64), .WIDTH(32)) u_bank (
      .clk   (clk),
      .ren   (bren[k]),
      .raddr (ba0[k]),
      .rdata (brdata[k]),
      .wen   (bwen[k]),
      .waddr (bwaddr[k]),
      .wdata (bwdata[k])
    );
  end

  // ---------------------------------------------------------------- pipeline control
  logic             v1, v2, v3, ld1, ld2, ld3, fin1, fin2, fin3;
  logic [7:0][5:0]  ba1, ba2, ba3;
  logic [5:0]       w1, w2, w3;
  logic             sb1, sb2;
  logic [3:0][2:0]  ke1, ko1;
  logic [7:0][2:0]  ws1, ws2, ws3;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0; ld1 <= 1'b0; ld2 <= 1'b0; ld3 <= 1'b0;
      fin1 <= 1'b0; fin2 <= 1'b0; fin3 <= 1'b0;
    end else begin
      v1 <= v0; v2 <= v1; v3 <= v2;
      ld1 <= ld0; ld2 <= ld1; ld3 <= ld2;
      fin1 <= fin0; fin2 <= fin1; fin3 <= fin2;
    end
  end
  always_ff @(posedge clk) begin
    ba1 <= ba0; ba2 <= ba1; ba3 <= ba2;
    w1 <= w0; w2 <= w1; w3 <= w2;
    sb1 <= sb0; sb2 <= sb1;
    ke1 <= ke0; ko1 <= ko0;
    ws1 <= ws0; ws2 <= ws1; ws3 <= ws2;
  end

  // ---------------------------------------------------------------- stage 1: bank -> lane routing
  logic [3:0][31:0] E2, O2, E3;
  always_ff @(posedge clk) begin
    for (int j = 0; j < 4; j++) begin
      E2[j] <= brdata[ke1[j]];
      O2[j] <= brdata[ko1[j]];
    end
  end

  // ---------------------------------------------------------------- stage 2: twiddle products
  // re = O.re*tw.re - O.im*tw.im, im = O.im*tw.re + O.re*tw.im (exact in int32, = ci16_mul_tw before
  // rounding), each as one 10-term Booth partial-product sum (explicit, so the final adder is a single CPA).
  // Booth partial product of x (int16) for digit d = {neg, two, one}, shifted by sh; a negated product is
  // the inverted magnitude, its '+1' is injected at bit sh of the next partial product (or the extra word).
  function automatic logic [25:0] bpp(input logic [15:0] x, input logic [2:0] d, input int sh);
    logic [17:0] m;
    logic [25:0] e;
    m = d[0] ? {{2{x[15]}}, x} : d[1] ? {x[15], x, 1'b0} : 18'd0;
    if (d[2]) m = ~m;
    e = {{8{m[17]}}, m};
    return e << sh;
  endfunction
  function automatic logic [51:0] csa3(input logic [25:0] a, input logic [25:0] b, input logic [25:0] c);
    return {a ^ b ^ c, ((a & b) | (a & c) | (b & c)) << 1};          // {sum, carry}
  endfunction
  // xa*ya + xb*yb (ya, yb as Booth digits) mod 2^26: 11 operands, explicit 3:2 tree -> {sum, carry}
  function automatic logic [51:0] bsum(input logic [15:0] xa, input logic [14:0] da,
                                       input logic [15:0] xb, input logic [14:0] db);
    logic [10:0][25:0] op;
    logic [7:0][25:0]  l1;
    logic [5:0][25:0]  l2;
    logic [3:0][25:0]  l3;
    logic [2:0][25:0]  l4;
    logic [1:0][25:0]  l5;
    for (int i = 0; i < 5; i++) begin
      op[i]   = bpp(xa, da[3*i +: 3], 2*i);
      op[5+i] = bpp(xb, db[3*i +: 3], 2*i);
      if (i > 0) begin
        op[i][2*i-2]   = da[3*(i-1)+2];       // low bits of a shifted product are zero
        op[5+i][2*i-2] = db[3*(i-1)+2];
      end
    end
    op[10] = '0;
    op[10][8] = da[14] ^ db[14];
    op[10][9] = da[14] & db[14];
    {l1[0], l1[1]} = csa3(op[0], op[1], op[2]);
    {l1[2], l1[3]} = csa3(op[3], op[4], op[5]);
    {l1[4], l1[5]} = csa3(op[6], op[7], op[8]);
    l1[6] = op[9]; l1[7] = op[10];
    {l2[0], l2[1]} = csa3(l1[0], l1[1], l1[2]);
    {l2[2], l2[3]} = csa3(l1[3], l1[4], l1[5]);
    l2[4] = l1[6]; l2[5] = l1[7];
    {l3[0], l3[1]} = csa3(l2[0], l2[1], l2[2]);
    {l3[2], l3[3]} = csa3(l2[3], l2[4], l2[5]);
    {l4[0], l4[1]} = csa3(l3[0], l3[1], l3[2]);
    l4[2] = l3[3];
    {l5[0], l5[1]} = csa3(l4[0], l4[1], l4[2]);
    return {l5[0], l5[1]};
  endfunction
  logic [3:0][51:0] pre3, pim3;          // carry-save {sum, carry}
  always_ff @(posedge clk) begin
    for (int j = 0; j < 4; j++) begin
      pre3[j] <= bsum(O2[j][15:0],  bre2[j], O2[j][31:16], bnim2[j]);
      pim3[j] <= bsum(O2[j][31:16], bre2[j], O2[j][15:0],  bim2[j]);
    end
    E3 <= E2;
  end

  // load: slot read data arrives in stage 2; lane l = idx[2:0] of word w2 -> bank {l2 ^ ^w2, l1, l0}
  logic [7:0][31:0] wl3;
  always_ff @(posedge clk) begin
    for (int k = 0; k < 8; k++) begin
      logic [2:0] l;
      logic [127:0] lo, hi;
      l  = {k[2] ^ (^w2), k[1], k[0]};
      lo = ff_rdata[sb2];
      hi = ff_rdata[~sb2];
      wl3[k] <= {hi[16*l +: 8], 8'h00, lo[16*l +: 8], 8'h00};
    end
  end

  // ---------------------------------------------------------------- stage 3: final add, rsh32 rounding
  // rsh32(x, 8) truncated to int16 (x fits 26 bits)
  function automatic logic [15:0] rnd8(input logic signed [25:0] x);
    logic signed [25:0] s;
    s = x + $signed({18'd0, ~x[25], {7{x[25]}}});
    return s[23:8];
  endfunction
  logic [3:0][15:0] tre4, tim4;
  logic [3:0][31:0] E4;
  logic [7:0][31:0] wl4;
  always_ff @(posedge clk) begin
    for (int j = 0; j < 4; j++) begin
      tre4[j] <= rnd8(pre3[j][51:26] + pre3[j][25:0]);
      tim4[j] <= rnd8(pim3[j][51:26] + pim3[j][25:0]);
    end
    E4  <= E3;
    wl4 <= wl3;
  end

  // ---------------------------------------------------------------- stage 4: add/sub, lane -> bank routing
  logic             v4, ld4, fin4;
  logic [7:0][5:0]  ba4;
  logic [5:0]       w4;
  logic [7:0][2:0]  ws4;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin v4 <= 1'b0; ld4 <= 1'b0; fin4 <= 1'b0; end
    else begin v4 <= v3; ld4 <= ld3; fin4 <= fin3; end
  end
  always_ff @(posedge clk) begin
    ba4 <= ba3; w4 <= w3; ws4 <= ws3;
  end
  logic [7:0][31:0] wd_bf;
  always_comb begin
    logic [3:0][31:0] X0, X1;
    for (int j = 0; j < 4; j++) begin
      X0[j] = {E4[j][31:16] + tim4[j], E4[j][15:0] + tre4[j]};
      X1[j] = {E4[j][31:16] - tim4[j], E4[j][15:0] - tre4[j]};
    end
    for (int k = 0; k < 8; k++) wd_bf[k] = ws4[k][2] ? X1[ws4[k][1:0]] : X0[ws4[k][1:0]];
  end

  // ---------------------------------------------------------------- stage 5: write / check
  logic             we5, chk5;
  logic [7:0][5:0]  wa5;
  logic [7:0][31:0] wd5;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin we5 <= 1'b0; chk5 <= 1'b0; end
    else begin we5 <= v4 && !fin4; chk5 <= v4 && fin4; end
  end
  always_ff @(posedge clk) begin
    for (int k = 0; k < 8; k++) wa5[k] <= ld4 ? w4 : ba4[k];
    wd5 <= ld4 ? wl4 : wd_bf;
  end
  assign bwen   = {8{we5}};
  assign bwaddr = wa5;
  assign bwdata = wd5;

  // final check: comp = rsh32(re,8)^2 + rsh32(im,8)^2 >= T -> reject
  function automatic logic [8:0] rs8(input logic [15:0] x);   // rsh32(int16 x, 8), in [-128, 128]
    logic [16:0] s;
    s = {x[15], x} + {9'd0, ~x[15], {7{x[15]}}};
    return s[16:8];
  endfunction
  logic             chk6, chk7;
  logic [15:0][7:0] mag6;      // |rsh32(.,8)| of the 16 re/im values
  logic             f7;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin chk6 <= 1'b0; chk7 <= 1'b0; f7 <= 1'b0; end
    else begin
      chk6 <= chk5;
      chk7 <= chk6;
      f7   <= 1'b0;
      if (chk6) begin
        for (int k = 0; k < 8; k++) begin
          logic [15:0] comp;
          comp = 16'(mag6[2*k] * mag6[2*k]) + 16'(mag6[2*k+1] * mag6[2*k+1]);
          if (comp >= 16'(thr_r)) f7 <= 1'b1;
        end
      end
    end
  end
  always_ff @(posedge clk) begin
    for (int k = 0; k < 8; k++) begin
      logic [8:0] r0, r1;
      r0 = rs8(wd5[k][15:0]);
      r1 = rs8(wd5[k][31:16]);
      mag6[2*k]   <= r0[8] ? 8'(-r0) : r0[7:0];
      mag6[2*k+1] <= r1[8] ? 8'(-r1) : r1[7:0];
    end
  end
  assign chk_fail = chk7 & f7;
endmodule
