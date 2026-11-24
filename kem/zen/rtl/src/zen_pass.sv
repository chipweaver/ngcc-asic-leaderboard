// -----------------------------------------------------------------------------
// zen_pass : elementwise passes of the ZEN KEM accelerator
//
//   mode 0 CT_DECOMP   : MEM_CT bytes -> c = (b*769+128)>>8 -> slot (write master W)
//   mode 2 DEC_FOLD    : t (R1, INTT_RAW output of decryption) ->
//                        t1 = fqmul(t,655); t0 (fold by x^(N/2)+1, *171 Montgomery);
//                        A = a_bits XOR t2 (SimpleDecoding) -> zen_gf2 (a_wr_*)
//   (mode 1 ENC_COMBINE is not used: the Encaps combine/compress and the FO compare are
//    fused into the last INTT pass, zen_ntt / zen_fci.)
//
// 16 DEC lanes process half a word pair per cycle and the two Montgomery
// multiplications take one cycle each (multiply and reduce in the same stage).
//   DEC issue order per group g: (A,C) half 0, (B,D) half 0, (A,C) half 1, (B,D) half 1
//   (cnt[0] = pair, cnt[1] = half, cnt[6:2] = g); every issue reads both words of its pair.
//
// Pipeline (stage register k is loaded in cycle k+1 after the issue cycle 0, controlled by m[k]):
//   c0  issue R1 read (2 words, both banks) / MEM_CT read (DECOMP)
//   c1  t_q  <- R1 read data half              (DECOMP: dr_q <- ct_rdata)
//   c2  x_q  <- fqmul(t, 655)                  (DECOMP: dw_q <- decompress(dr_q))
//   c3  s_q  <- w16(C-A), w16(A+C) per pair    (DECOMP: slot write)
//   c4  v_q  <- fqmul(s, 171)
//   c5  a-bit / min(cs) per pair
//   c6  (A,C): hold ; (B,D): combine with the held pair -> 8+8 output bits (half 0 kept)
//   c7  a_wr chunk g (half 1 completes it), c8: chunk g+Q
// -----------------------------------------------------------------------------
module zen_pass (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              start,
  input  logic [1:0]        param,
  input  logic [1:0]        mode,
  output logic              done,
  output logic              active,
  // read master R1
  output logic [1:0]        r1_rd_en,
  output logic [1:0][5:0]   r1_rd_addr,
  input  logic [1:0][255:0] r1_rd_data,
  // write master W
  output logic [1:0]        w_wr_en,
  output logic [1:0][5:0]   w_wr_addr,
  output logic [1:0][255:0] w_wr_data,
  // object port to MEM_CT (128-bit words, read only)
  output logic              ct_en,
  output logic [6:0]        ct_addr,
  input  logic [127:0]      ct_rdata,
  // to zen_gf2 (A operand, XOR-accumulate)
  output logic              a_clr,
  output logic              a_wr_en,
  output logic [5:0]        a_wr_idx,
  output logic [15:0]       a_wr_data
);

  localparam logic [1:0] MD_DECOMP = 2'd0;
  localparam logic [1:0] MD_DEC    = 2'd2;
  localparam int         ND        = 6;     // meta pipeline depth m[0..ND-1]
  localparam int         NL        = 16;    // DEC lanes (8 pairs)

  // ---------------------------------------------------------------------------
  // arithmetic helpers (all mod 2^32 / mod 2^16, explicit shift-add constants)
  // ---------------------------------------------------------------------------
  // p = t*655 (signed, |p| < 2^25 -> 26 bits)
  function automatic logic [25:0] f_mul655(input logic [15:0] t);
    logic [31:0] s;
    logic [31:0] r;
    s = {{16{t[15]}}, t};
    r = (s << 9) + (s << 7) + (s << 4) - s;              // 512+128+16-1
    f_mul655 = r[25:0];
  endfunction

  // u = w16(t*655*-767) = t*21903 mod 2^16  (Montgomery quotient of t*655)
  function automatic logic [15:0] f_u655(input logic [15:0] t);
    f_u655 = (t << 14) + (t << 12) + (t << 10) + (t << 8) + (t << 7) + (t << 4) - t;
  endfunction

  // a = s*171 (s signed 16 bits, |a| < 2^25 -> 26 bits)
  function automatic logic [25:0] f_mul171(input logic [15:0] s);
    logic [31:0] x;
    logic [31:0] r;
    x = {{16{s[15]}}, s};
    r = (x << 7) + (x << 5) + (x << 3) + (x << 1) + x;  // 128+32+8+2+1
    f_mul171 = r[25:0];
  endfunction

  // u = w16(s*171*-767) = -(s*85) mod 2^16
  function automatic logic [15:0] f_u171(input logic [15:0] y);
    f_u171 = 16'd0 - ((y << 6) + (y << 4) + (y << 2) + y);
  endfunction

  // Montgomery final step: w16((a - u*769) >> 16), a signed 26 bits, int32 semantics
  function automatic logic [15:0] f_mred(input logic [25:0] a, input logic [15:0] u);
    logic [31:0] as;
    logic [31:0] us;
    logic [31:0] d;
    as = {{6{a[25]}}, a};
    us = {{16{u[15]}}, u};
    d  = as - ((us << 9) + (us << 8) + us);              // a - u*769
    f_mred = d[31:16];
  endfunction

  // decompress: (b*769 + 128) >> 8
  function automatic logic [15:0] f_decomp(input logic [7:0] b);
    logic [17:0] y;
    y = {1'b0, b, 9'd0} + {2'b0, b, 8'd0} + {10'd0, b} + 18'd128;
    f_decomp = {6'd0, y[17:8]};
  endfunction

  // SimpleDecoding distance: cv>=0 ? w16(384-cv) : w16(384+cv)
  function automatic logic [15:0] f_cs(input logic [15:0] cv);
    f_cs = cv[15] ? (16'd384 + cv) : (16'd384 - cv);
  endfunction

  // ---------------------------------------------------------------------------
  // configuration latch and issue control
  // ---------------------------------------------------------------------------
  logic [1:0] mode_q, param_q;
  logic       issuing;
  logic [6:0] cnt;
  logic [6:0] last_cnt;
  logic       active_q;
  logic       a_clr_q;

  always_comb begin
    case ({mode_q == MD_DEC, param_q})
      3'b000:  last_cnt = 7'd31;
      3'b001:  last_cnt = 7'd63;
      3'b010:  last_cnt = 7'd127;
      3'b100:  last_cnt = 7'd31;
      3'b101:  last_cnt = 7'd63;
      3'b110:  last_cnt = 7'd127;
      default: last_cnt = 7'd31;
    endcase
  end

  // Q = N4/16 = words per quarter polynomial
  logic [6:0] q_words;
  always_comb begin
    case (param_q)
      2'd1:    q_words = 7'd16;
      2'd2:    q_words = 7'd32;
      default: q_words = 7'd8;
    endcase
  end

  logic is_decomp, is_dec;
  assign is_decomp = (mode_q == MD_DECOMP);
  assign is_dec    = (mode_q == MD_DEC);

  logic issue_last;
  assign issue_last = (cnt == last_cnt);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mode_q  <= 2'd0;
      param_q <= 2'd0;
      issuing <= 1'b0;
      cnt     <= 7'd0;
    end else if (start) begin
      mode_q  <= mode;
      param_q <= param;
      issuing <= 1'b1;
      cnt     <= 7'd0;
    end else if (issuing) begin
      cnt <= cnt + 7'd1;
      if (issue_last) issuing <= 1'b0;
    end
  end

  // issue-cycle operation descriptor
  logic [6:0] i_idx;     // DECOMP: word index ; DEC: group g
  logic       i_ph;      // DEC: 0 = (A,C) pair, 1 = (B,D) pair
  logic       i_hf;      // DEC: half word (coefficients 8*hf .. 8*hf+7)
  logic [6:0] i_lo, i_hi;
  logic       i_lsel;    // bank of the "lo" word
  always_comb begin
    if (is_dec) begin
      i_idx = {2'b0, cnt[6:2]};
      i_ph  = cnt[0];
      i_hf  = cnt[1];
      i_lo  = cnt[0] ? (i_idx + q_words) : i_idx;
      i_hi  = cnt[0] ? (i_idx + q_words + (q_words << 1)) : (i_idx + (q_words << 1));
    end else begin
      i_idx = cnt;
      i_ph  = 1'b0;
      i_hf  = 1'b0;
      i_lo  = cnt;
      i_hi  = cnt;
    end
    i_lsel = ^i_lo;
  end

  // meta pipeline: m[k] describes the op whose stage register k is loaded at the end of cycle k+1
  logic [ND-1:0]      m_v, m_ph, m_hf, m_last, m_lsel;
  logic [ND-1:0][6:0] m_idx;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      m_v    <= '0;
      m_ph   <= '0;
      m_hf   <= '0;
      m_last <= '0;
      m_lsel <= '0;
      m_idx  <= '0;
    end else begin
      // start flushes ops of the previous pass still draining through unused stages
      m_v    <= start ? '0 : {m_v[ND-2:0], issuing};
      if (issuing || (|m_v)) begin
        m_ph   <= {m_ph[ND-2:0], i_ph};
        m_hf   <= {m_hf[ND-2:0], i_hf};
        m_last <= {m_last[ND-2:0], issue_last};
        m_lsel <= {m_lsel[ND-2:0], i_lsel};
        m_idx  <= {m_idx[ND-2:0], i_idx};
      end
    end
  end

  // ---------------------------------------------------------------------------
  // memory read requests
  // ---------------------------------------------------------------------------
  always_comb begin
    r1_rd_en   = 2'b00;
    r1_rd_addr = {i_hi[6:1], i_lo[6:1]};
    if (issuing && is_dec) r1_rd_en = 2'b11;
    if (i_lsel) r1_rd_addr = {i_lo[6:1], i_hi[6:1]};
  end

  // ---------------------------------------------------------------------------
  // DEC lane datapath: lane k < 8 holds A/B coefficient 8*hf+k, lane k+8 the C/D coefficient
  // ---------------------------------------------------------------------------
  logic [NL-1:0][15:0] t_q;
  logic [NL-1:0][15:0] x_q;
  logic [NL-1:0][15:0] s_q;
  logic [NL-1:0][15:0] v_q;
  logic [7:0]          dab_q, hab_q;
  logic [7:0][15:0]    dcm_q, hcm_q;

  logic [ND-1:0] en;
  assign en = m_v & {ND{is_dec}};

  genvar k;
  generate
    for (k = 0; k < NL; k = k + 1) begin : g_lane
      localparam int KP = (k + 8) % 16;
      localparam int KL = k % 8;
      localparam bit HI = (k >= 8);

      // c1: capture R1 data (lo lanes from bank m_lsel, hi lanes from the other bank), half m_hf
      logic [15:0] rd_w;
      assign rd_w = r1_rd_data[m_lsel[0] ^ HI][16*KL + 128*m_hf[0] +: 16];
      always_ff @(posedge clk) if (en[0]) t_q[k] <= rd_w;

      // c2: fqmul(.,655)
      always_ff @(posedge clk) if (en[1]) x_q[k] <= f_mred(f_mul655(t_q[k]), f_u655(t_q[k]));

      // c3: fold. lo lane: w16(t1[i+N2] - t1[i])  (C - A, or D - B); hi lane: w16(t1[i] + t1[i+N2])
      always_ff @(posedge clk) begin
        if (en[2]) s_q[k] <= HI ? (x_q[KP] + x_q[k]) : (x_q[KP] - x_q[k]);
      end

      // c4: fqmul(.,171)
      always_ff @(posedge clk) if (en[3]) v_q[k] <= f_mred(f_mul171(s_q[k]), f_u171(s_q[k]));

      if (k < 8) begin : g_post
        // c5: lo = t0[i] (or t0[i+N4]), hi = t0[i+N2] (or t0[i+3N4])
        logic [15:0] cs_lo, cs_hi;
        assign cs_lo = f_cs(v_q[k]);
        assign cs_hi = f_cs(v_q[k+8]);
        always_ff @(posedge clk) begin
          if (en[4]) begin
            dab_q[k] <= v_q[k][0] ^ v_q[k+8][0];
            dcm_q[k] <= ($signed(cs_lo) <= $signed(cs_hi)) ? cs_lo : cs_hi;
          end
        end
        // c6: hold the (A,C) pair result
        always_ff @(posedge clk) begin
          if (en[5] && !m_ph[5]) begin
            hab_q[k] <= dab_q[k];
            hcm_q[k] <= dcm_q[k];
          end
        end
      end
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // DEC_FOLD: combine (B,D) pair with held (A,C) pair, emit A chunks
  // ---------------------------------------------------------------------------
  logic       fin;
  logic [7:0] fin_lo, fin_hi;
  assign fin = en[5] & m_ph[5];
  // hab = a_bits[i], dab = a_bits[i+N4], par = hab ^ dab,
  // sel = (c0 <= c1): t2 bit (= par) goes to i, else to i+N4.
  logic [7:0] fin_par, fin_sel;
  always_comb begin
    for (int j = 0; j < 8; j = j + 1)
      fin_sel[j] = ($signed(hcm_q[j]) <= $signed(dcm_q[j]));
    fin_par = hab_q ^ dab_q;
    fin_lo  = hab_q ^ (fin_par & fin_sel);
    fin_hi  = dab_q ^ (fin_par & ~fin_sel);
  end

  logic        aw_en_q, aw_last_q, pend_v_q, pend_last_q;
  logic [5:0]  aw_idx_q, pend_idx_q;
  logic [15:0] aw_data_q, pend_data_q;
  logic [7:0]  h0_lo_q, h0_hi_q;   // half-0 bits of the current group
  logic [6:0]  hi_idx;
  assign hi_idx = m_idx[5] + q_words;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      aw_en_q     <= 1'b0;
      aw_last_q   <= 1'b0;
      aw_idx_q    <= 6'd0;
      aw_data_q   <= 16'd0;
      pend_v_q    <= 1'b0;
      pend_last_q <= 1'b0;
      pend_idx_q  <= 6'd0;
      pend_data_q <= 16'd0;
    end else if (fin && m_hf[5]) begin
      aw_en_q     <= 1'b1;
      aw_last_q   <= 1'b0;
      aw_idx_q    <= m_idx[5][5:0];
      aw_data_q   <= {fin_lo, h0_lo_q};
      pend_v_q    <= 1'b1;
      pend_last_q <= m_last[5];
      pend_idx_q  <= hi_idx[5:0];
      pend_data_q <= {fin_hi, h0_hi_q};
    end else if (pend_v_q) begin
      aw_en_q     <= 1'b1;
      aw_last_q   <= pend_last_q;
      aw_idx_q    <= pend_idx_q;
      aw_data_q   <= pend_data_q;
      pend_v_q    <= 1'b0;
    end else begin
      aw_en_q     <= 1'b0;
      aw_last_q   <= 1'b0;
    end
  end

  // half-0 bits wait for half 1 of the same group (two cycles later; a half-1 fin and its pending
  // second chunk never coincide with a half-0 fin)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      h0_lo_q <= 8'd0;
      h0_hi_q <= 8'd0;
    end else if (fin && !m_hf[5]) begin
      h0_lo_q <= fin_lo;
      h0_hi_q <= fin_hi;
    end
  end

  assign a_wr_en   = aw_en_q;
  assign a_wr_idx  = aw_idx_q;
  assign a_wr_data = aw_data_q;
  assign a_clr     = a_clr_q;

  // ---------------------------------------------------------------------------
  // CT_DECOMP datapath
  // ---------------------------------------------------------------------------
  logic [127:0]      dr_q;
  logic [15:0][15:0] dw_q;
  always_ff @(posedge clk) if (m_v[0] && is_decomp) dr_q <= ct_rdata;
  always_ff @(posedge clk) begin
    if (m_v[1] && is_decomp) begin
      for (int j = 0; j < 16; j = j + 1) dw_q[j] <= f_decomp(dr_q[8*j +: 8]);
    end
  end

  always_comb begin
    w_wr_en       = 2'b00;
    w_wr_addr[0]  = m_idx[2][6:1];
    w_wr_addr[1]  = m_idx[2][6:1];
    w_wr_data[0]  = dw_q;
    w_wr_data[1]  = dw_q;
    if (m_v[2] && is_decomp) w_wr_en[m_lsel[2]] = 1'b1;
  end

  // MEM_CT port (CT_DECOMP reads)
  assign ct_en   = issuing & is_decomp;
  assign ct_addr = cnt;

  // ---------------------------------------------------------------------------
  // status
  // ---------------------------------------------------------------------------
  logic done_d;
  always_comb begin
    case (mode_q)
      MD_DECOMP: done_d = m_v[2] & m_last[2];
      MD_DEC:    done_d = aw_en_q & aw_last_q;
      default:   done_d = 1'b0;
    endcase
  end
  assign done   = done_d & active_q;
  assign active = active_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      active_q <= 1'b0;
      a_clr_q  <= 1'b0;
    end else begin
      a_clr_q <= start & (mode == MD_DEC);
      if (start)     active_q <= 1'b1;
      else if (done) active_q <= 1'b0;
    end
  end

endmodule
