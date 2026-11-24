// Hash subsystem: regfile, random-byte receiver, sequencer A (two channels: Hash2 and H(pk)) and
// sequencer B (one channel: implicit-rejection KDF(z || ct)).
//
// Regfile (64-bit words, object byte order: byte k of a word at bits [8k+7:8k]):
//   MSG  0..15  Hash2 input: KeyGen d (b bytes); Enc/Dec m || H(pk)  (H(pk) at byte b)
//   H2O 16..31  Hash2 output (I: 32 B; II: h1||h2; III: h1..h4)
//   TMP 32..63  pseudo-hash temporaries T0..T7 (4 words each)
//   Z   64..71  KeyGen z
//   KBAR 72..79 KDF(z||ct)
//   SEED 80..87 gen_a public seed copied from PK/SK (Enc/Dec)
module aigis_hsu (
  input  logic         clk,
  input  logic         rst_n,
  input  logic [1:0]   param,
  input  logic         clr,        // command accepted: message words 0..15 become invalid
  // sequencer A job: 0 HASH2, 1 HPK
  input  logic         a_start,
  input  logic         a_job,
  input  logic         a_dbl,      // HASH2 input length 2b (else b)
  input  logic         a_waitpk,   // HPK: wait for PK words written in this command
  output logic         a_busy,
  output logic         a_done,
  // sequencer B job: REJ
  input  logic         b_start,
  output logic         b_busy,
  output logic         b_done,
  // random receiver job: nwords beats into regfile word dst
  input  logic         x_start,
  input  logic [6:0]   x_dst,
  input  logic [3:0]   x_nwords,
  output logic         x_busy,
  output logic         x_done,
  input  logic         rnd_valid,
  output logic         rnd_ready,
  input  logic [63:0]  rnd_data,
  // external regfile write ports (copy engine, tomsg)
  input  logic         e_we,
  input  logic [6:0]   e_waddr,
  input  logic [63:0]  e_wdata,
  input  logic         t_we,
  input  logic [6:0]   t_waddr,
  input  logic [63:0]  t_wdata,
  output logic [87:0][63:0] rf,
  // object buffer read clients (ch0, ch1, ch2)
  output logic [2:0]       ob_req,
  output logic [2:0][1:0]  ob_sel,
  output logic [2:0][9:0]  ob_addr,
  input  logic [2:0]       ob_gnt,
  input  logic [2:0]       ob_rv,
  input  logic [2:0][63:0] ob_rdata,
  input  logic [327:0]     pkw,
  // KeyGen H(pk): copy of H(pk) from the regfile into SK (object-buffer write client)
  output logic             sk_req,
  output logic [9:0]       sk_addr,
  output logic [63:0]      sk_data,
  input  logic             sk_gnt
);
  import aigis_pkg::*;
  logic [15:0] mv;   // message word valid (see regfile)
  localparam logic [6:0] MSG = 7'd0, H2O = 7'd16, TMP = 7'd32, ZR = 7'd64, KB = 7'd72;
  localparam logic [1:0] K_REG = 2'd0, K_OBJ = 2'd1, K_IMM = 2'd2;
  localparam logic [1:0] O_PK = 2'd0, O_SK = 2'd1, O_CT = 2'd2;
  localparam logic [1:0] I_IV = 2'd0, I_MK1 = 2'd1, I_MK2 = 2'd2, I_MID = 2'd3;

  typedef struct packed {
    logic        v;
    logic [1:0]  init;
    logic [12:0] prefix;
    logic [1:0]  nseg;
    logic [2:0][58:0] seg;
    logic        nofinal;
    logic [6:0]  dst;
    logic [2:0]  dstw;
  } call_t;

  function automatic logic [58:0] sreg(input logic [6:0] w, input logic [12:0] len);
    sreg = {K_REG, 2'd0, {3'd0, w}, len, 32'd0};
  endfunction
  function automatic logic [58:0] sobj(input logic [1:0] o, input logic [9:0] w, input logic [12:0] len);
    sobj = {K_OBJ, o, w, len, 32'd0};
  endfunction
  function automatic logic [58:0] sown();
    sown = {2'd3, 2'd0, 10'd0, 13'd32, 32'd0};
  endfunction
  function automatic logic [58:0] simm(input logic [31:0] v, input logic [12:0] len);
    simm = {K_IMM, 2'd0, 10'd0, len, v};
  endfunction
  function automatic call_t mk(input logic [1:0] init, input logic [12:0] prefix, input logic [1:0] nseg,
                               input logic [58:0] s0, input logic [58:0] s1, input logic [58:0] s2,
                               input logic nofinal, input logic [6:0] dst, input logic [2:0] dstw);
    mk.v = 1'b1; mk.init = init; mk.prefix = prefix; mk.nseg = nseg;
    mk.seg[0] = s0; mk.seg[1] = s1; mk.seg[2] = s2;
    mk.nofinal = nofinal; mk.dst = dst; mk.dstw = dstw;
  endfunction
  function automatic logic [6:0] tw(input int i);   // temporary register word index
    tw = TMP + 7'(4 * i);
  endfunction

  // ----------------------------------------------------------------- sequencer A programs
  logic [2:0] a_step, a_stp;    // a_stp: step whose calls are presented (issued now)
  logic       a_job_q, a_dbl_q, a_waitpk_q;
  call_t a_c0, a_c1;
  logic  a_last, a_cur_last;
  always_comb begin
    logic [12:0] L, PKB;
    logic [6:0]  bw;
    logic [31:0] tag;
    a_c0 = '0; a_c1 = '0; a_last = 1'b1;
    bw  = 7'(bbytes(param) >> 3);
    L   = a_dbl_q ? 13'(2 * bbytes(param)) : 13'(bbytes(param));
    PKB = pk_bytes(param);
    tag = (param == 2'd3) ? 32'h0400_0000 : 32'h0200_0000;
    if (a_job_q == 1'b0) begin
      // Hash2
      case (param)
        2'd1: begin a_c0 = mk(I_IV, 13'd0, 2'd1, sreg(MSG, L), '0, '0, 1'b0, H2O, 3'd4); a_last = 1'b1; end
        2'd2: case (a_stp)
          3'd0: begin a_c0 = mk(I_MK1, 13'd64, 2'd2, simm(tag, 13'd2), sreg(MSG, L), '0, 1'b0, tw(0), 3'd4);
                      a_c1 = mk(I_IV, 13'd0, 2'd2, sreg(MSG, L), simm(tag, 13'd2), '0, 1'b0, H2O, 3'd4); a_last = 1'b0; end
          3'd1: begin a_c0 = mk(I_MK2, 13'd64, 2'd1, sown(), '0, '0, 1'b0, tw(1), 3'd4); a_last = 1'b0; end
          default: begin a_c0 = mk(I_IV, 13'd0, 2'd2, sown(), sreg(H2O, 13'd32), '0, 1'b0, H2O + 7'd4, 3'd4); a_last = 1'b1; end
        endcase
        default: case (a_stp)
          // pseudohash-1024: k1 = HMAC(tag||m), h1 = SM3(m||tag), k2 = HMAC(h1), h2 = SM3(k1||h1),
          // h3 = SM3(h2||k2), k3 = HMAC(h2), k4 = HMAC(h3), h4 = SM3(k3||h3||k4); the first block of
          // h4 (k3||h3) is compressed (nofinal, ch1 midstate) while k4 is still being computed.
          // T0 k1 inner, T1 k1, T2 k2 inner, T3 k2, T4 k3 inner, T5 k3, T6 k4 inner, T7 k4.
          3'd0: begin a_c0 = mk(I_MK1, 13'd64, 2'd2, simm(tag, 13'd2), sreg(MSG, L), '0, 1'b0, tw(0), 3'd4);
                      a_c1 = mk(I_IV, 13'd0, 2'd2, sreg(MSG, L), simm(tag, 13'd2), '0, 1'b0, H2O, 3'd4); a_last = 1'b0; end
          3'd1: begin a_c0 = mk(I_MK2, 13'd64, 2'd1, sown(), '0, '0, 1'b0, tw(1), 3'd4);
                      a_c1 = mk(I_MK1, 13'd64, 2'd1, sown(), '0, '0, 1'b0, tw(2), 3'd4); a_last = 1'b0; end
          3'd2: begin a_c0 = mk(I_IV, 13'd0, 2'd2, sown(), sreg(H2O, 13'd32), '0, 1'b0, H2O + 7'd4, 3'd4);
                      a_c1 = mk(I_MK2, 13'd64, 2'd1, sown(), '0, '0, 1'b0, tw(3), 3'd4); a_last = 1'b0; end
          3'd3: begin a_c0 = mk(I_IV, 13'd0, 2'd2, sown(), sreg(tw(3), 13'd32), '0, 1'b0, H2O + 7'd8, 3'd4);
                      a_c1 = mk(I_MK1, 13'd64, 2'd1, sreg(H2O + 7'd4, 13'd32), '0, '0, 1'b0, tw(4), 3'd4); a_last = 1'b0; end
          3'd4: begin a_c0 = mk(I_MK1, 13'd64, 2'd1, sown(), '0, '0, 1'b0, tw(6), 3'd4);
                      a_c1 = mk(I_MK2, 13'd64, 2'd1, sown(), '0, '0, 1'b0, tw(5), 3'd4); a_last = 1'b0; end
          3'd5: begin a_c0 = mk(I_MK2, 13'd64, 2'd1, sown(), '0, '0, 1'b0, tw(7), 3'd4);
                      a_c1 = mk(I_IV, 13'd0, 2'd2, sown(), sreg(H2O + 7'd8, 13'd32), '0, 1'b1, H2O + 7'd12, 3'd0); a_last = 1'b0; end
          default: begin a_c1 = mk(I_MID, 13'd64, 2'd1, sreg(tw(7), 13'd32), '0, '0, 1'b0, H2O + 7'd12, 3'd4); a_last = 1'b1; end
        endcase
      endcase
    end else begin
      // H(pk) -> MSG + b/8
      case (param)
        2'd1: begin a_c0 = mk(I_IV, 13'd0, 2'd2, sobj(O_PK, 10'd0, PKB), simm(32'h0000_0001, 13'd4), '0, 1'b0, MSG + bw, 3'd2); a_last = 1'b1; end
        2'd2: begin a_c0 = mk(I_IV, 13'd0, 2'd1, sobj(O_PK, 10'd0, PKB), '0, '0, 1'b0, MSG + bw, 3'd4); a_last = 1'b1; end
        default: case (a_stp)
          3'd0: begin a_c0 = mk(I_MK1, 13'd64, 2'd2, simm(32'h0200_0000, 13'd2), sobj(O_PK, 10'd0, PKB), '0, 1'b0, tw(0), 3'd4);
                      a_c1 = mk(I_IV, 13'd0, 2'd2, sobj(O_PK, 10'd0, PKB), simm(32'h0200_0000, 13'd2), '0, 1'b0, MSG + bw, 3'd4); a_last = 1'b0; end
          3'd1: begin a_c0 = mk(I_MK2, 13'd64, 2'd1, sown(), '0, '0, 1'b0, tw(1), 3'd4); a_last = 1'b0; end
          default: begin a_c0 = mk(I_IV, 13'd0, 2'd2, sown(), sreg(MSG + bw, 13'd32), '0, 1'b0, MSG + bw + 7'd4, 3'd4); a_last = 1'b1; end
        endcase
      endcase
    end
  end

  // ----------------------------------------------------------------- sequencer B program (REJ)
  logic [1:0] b_step;
  call_t b_c;
  logic  b_last;
  always_comb begin
    logic [9:0]  zw;
    logic [12:0] B, CB;
    B  = 13'(bbytes(param));
    CB = ct_bytes(param);
    zw = 10'((sk_bytes(param) - B) >> 3);
    b_c = '0; b_last = 1'b1;
    if (param != 2'd3) begin
      b_c = mk(I_IV, 13'd0, 2'd3, sobj(O_SK, zw, B), sobj(O_CT, 10'd0, CB), simm(32'h0000_0001, 13'd4), 1'b0, KB, 3'(B >> 3));
    end else case (b_step)
      2'd0: begin b_c = mk(I_IV, 13'd0, 2'd2, sobj(O_SK, zw, B), sobj(O_CT, 10'd0, CB), '0, 1'b1, KB, 3'd0); b_last = 1'b0; end
      2'd1: begin b_c = mk(I_MID, 13'd3648, 2'd1, simm(32'h0000_0001, 13'd4), '0, '0, 1'b0, KB, 3'd4); b_last = 1'b0; end
      default: begin b_c = mk(I_MID, 13'd3648, 2'd1, simm(32'h0000_0002, 13'd4), '0, '0, 1'b0, KB + 7'd4, 3'd4); b_last = 1'b1; end
    endcase
  end

  // ----------------------------------------------------------------- channels
  logic [2:0] ch_start, ch_busy, ch_done, ch_we, ch_we4;
  logic [2:0][6:0] ch_raddr, ch_waddr;
  logic [2:0][63:0] ch_wdata;
  logic [2:0][4:0] ch_waddr4;
  logic [2:0][3:0][63:0] ch_wdata4;
  call_t [2:0] ch_call;
  assign ch_call[0] = a_c0;
  assign ch_call[1] = a_c1;
  assign ch_call[2] = b_c;
  for (genvar c = 0; c < 3; c++) begin : g_ch
    aigis_sm3_chan u_ch (
      .clk(clk), .rst_n(rst_n), .start(ch_start[c]),
      .d_init(ch_call[c].init), .d_prefix(ch_call[c].prefix), .d_nseg(ch_call[c].nseg), .d_seg(ch_call[c].seg),
      .d_nofinal(ch_call[c].nofinal), .d_dst(ch_call[c].dst), .d_dstw(ch_call[c].dstw), .d_waitpk(c < 2 ? a_waitpk_q : 1'b0),
      .busy(ch_busy[c]), .done(ch_done[c]),
      .rf_raddr(ch_raddr[c]), .rf_rdata(rf[ch_raddr[c]]), .rf_rvalid((ch_raddr[c] >= 7'd16) || mv[ch_raddr[c][3:0]]),
      .rf_we(ch_we[c]), .rf_waddr(ch_waddr[c]), .rf_wdata(ch_wdata[c]),
      .rf_we4(ch_we4[c]), .rf_waddr4(ch_waddr4[c]), .rf_wdata4(ch_wdata4[c]),
      .ob_req(ob_req[c]), .ob_sel(ob_sel[c]), .ob_addr(ob_addr[c]), .ob_gnt(ob_gnt[c]), .ob_rv(ob_rv[c]),
      .ob_rdata(ob_rdata[c]), .pkw(pkw));
  end

  // ----------------------------------------------------------------- sequencer A / B control
  // A step's channels are started in the cycle the previous step's last channel finishes.
  typedef enum logic [1:0] {S_IDLE, S_ISSUE, S_WAIT, S_COPY} sst_t;
  sst_t a_st, b_st;
  logic [1:0] a_pend;   // channels of the current step still running
  logic [1:0] a_np;
  logic       a_adv;    // current step complete and another follows: issue it now
  logic       a_lastw;  // the waiting step is the last one
  logic [3:0] a_cpi;    // SK copy word index
  always_comb begin
    a_np  = a_pend & ~ch_done[1:0];
    a_adv = (a_st == S_WAIT) && (a_np == 2'b00) && !a_lastw;
    a_stp = a_adv ? a_step + 3'd1 : a_step;
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      a_st <= S_IDLE; a_busy <= 1'b0; a_done <= 1'b0; a_step <= '0; a_pend <= '0; a_lastw <= 1'b0; a_cpi <= '0;
      a_job_q <= 1'b0; a_dbl_q <= 1'b0; a_waitpk_q <= 1'b0;
    end else begin
      a_done <= 1'b0;
      case (a_st)
        S_IDLE: if (a_start) begin
          a_busy <= 1'b1; a_step <= '0; a_job_q <= a_job; a_dbl_q <= a_dbl; a_waitpk_q <= a_waitpk; a_st <= S_ISSUE;
        end
        S_ISSUE: begin a_pend <= {a_c1.v, a_c0.v}; a_lastw <= a_last; a_st <= S_WAIT; end
        S_WAIT: begin
          a_pend <= a_np;
          if (a_np == 2'b00) begin
            if (a_lastw) begin
              // KeyGen H(pk) (waits on pk words): copy H(pk) into SK before completing
              if (a_job_q && a_waitpk_q) begin a_st <= S_COPY; a_cpi <= '0; end
              else begin a_st <= S_IDLE; a_busy <= 1'b0; a_done <= 1'b1; end
            end
            else begin a_step <= a_step + 3'd1; a_pend <= {a_c1.v, a_c0.v}; a_lastw <= a_last; end
          end
        end
        S_COPY: if (sk_gnt) begin
          a_cpi <= a_cpi + 4'd1;
          if (a_cpi + 4'd1 == 4'(bbytes(param) >> 3)) begin a_st <= S_IDLE; a_busy <= 1'b0; a_done <= 1'b1; end
        end
        default: a_st <= S_IDLE;
      endcase
    end
  end
  assign sk_req  = (a_st == S_COPY);
  assign sk_addr = 10'((sk_bytes(param) - 13'(2 * bbytes(param))) >> 3) + 10'(a_cpi);
  assign sk_data = rf[7'(bbytes(param) >> 3) + 7'(a_cpi)];
  assign ch_start[0] = ((a_st == S_ISSUE) || a_adv) && a_c0.v;
  assign ch_start[1] = ((a_st == S_ISSUE) || a_adv) && a_c1.v;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin b_st <= S_IDLE; b_busy <= 1'b0; b_done <= 1'b0; b_step <= '0; end
    else begin
      b_done <= 1'b0;
      case (b_st)
        S_IDLE: if (b_start) begin b_busy <= 1'b1; b_step <= '0; b_st <= S_ISSUE; end
        S_ISSUE: b_st <= S_WAIT;
        S_WAIT: if (ch_done[2]) begin
          if (b_last) begin b_st <= S_IDLE; b_busy <= 1'b0; b_done <= 1'b1; end
          else begin b_step <= b_step + 2'd1; b_st <= S_ISSUE; end
        end
        default: b_st <= S_IDLE;
      endcase
    end
  end
  assign ch_start[2] = (b_st == S_ISSUE);

  // ----------------------------------------------------------------- random receiver
  logic [6:0] x_ptr;
  logic [3:0] x_left;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin x_busy <= 1'b0; x_done <= 1'b0; x_left <= '0; x_ptr <= '0; end
    else begin
      x_done <= 1'b0;
      if (x_start && !x_busy) begin x_busy <= 1'b1; x_ptr <= x_dst; x_left <= x_nwords; end
      else if (x_busy && rnd_valid && rnd_ready) begin
        x_ptr <= x_ptr + 7'd1; x_left <= x_left - 4'd1;
        if (x_left == 4'd1) begin x_busy <= 1'b0; x_done <= 1'b1; end
      end
    end
  end
  assign rnd_ready = x_busy && (x_left != 4'd0);

  // ----------------------------------------------------------------- regfile
  // mv[w]: message word w written during the current command (a hash call reading it waits)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) mv <= '0;
    else if (clr) mv <= '0;
    else begin
      for (int c = 0; c < 3; c++) begin
        if (ch_we[c] && ch_waddr[c] < 7'd16) mv[ch_waddr[c][3:0]] <= 1'b1;
        if (ch_we4[c] && ch_waddr4[c] < 5'd4) for (int w = 0; w < 4; w++) mv[{ch_waddr4[c][1:0], 2'(w)}] <= 1'b1;
      end
      if (x_busy && rnd_valid && rnd_ready && x_ptr < 7'd16) mv[x_ptr[3:0]] <= 1'b1;
      if (e_we && e_waddr < 7'd16) mv[e_waddr[3:0]] <= 1'b1;
      if (t_we && t_waddr < 7'd16) mv[t_waddr[3:0]] <= 1'b1;
    end
  end
  always_ff @(posedge clk) begin
    for (int c = 0; c < 3; c++) if (ch_we[c]) rf[ch_waddr[c]] <= ch_wdata[c];
    for (int c = 0; c < 3; c++) if (ch_we4[c]) for (int w = 0; w < 4; w++) rf[{ch_waddr4[c], 2'(w)}] <= ch_wdata4[c][w];
    if (x_busy && rnd_valid && rnd_ready) rf[x_ptr] <= rnd_data;
    if (e_we) rf[e_waddr] <= e_wdata;
    if (t_we) rf[t_waddr] <= t_wdata;
  end
endmodule
