// plc_gena -- uniform matrix A generation (poly_generate_uniformQ of sample.c), hash client 1.
//
// For each (i,j) in C loop order, exseed = pk_seed (RG_PS, p_pkseed bytes) || x || y with
// (x,y) = (j,i) (transposed=0) or (i,j) (transposed=1); stream nonce 0, block counter from 0.
// One squeeze of B bytes = one plc_hash job: seg0 = RG_PS[0..pkseed), sfx = {x, y, 0, be32(block)},
// count = ceil(B/32) (last digest truncated). A[i][j] is written to slot 2i+j.
//
// q = 257 sets: one squeeze of N bytes, coefficient t = byte t (zero extended); each 8-byte chunk
// of a digest is one slot word (1 word/cycle, 4 cycles per digest).
// 512-Star (sample.c sample_uniformQ_packed + poly_unpack, pack/unpack is the identity on the
// 205 values): Q5 stage: squeezes of 1368 bytes, 6-byte little-endian values accepted if
// < 769^5 until 204 are accepted (next squeeze at block+1 if exhausted); Q4 stage: fresh 30-byte
// squeeze at the next block, 5-byte values masked to 39 bits, first one < 769^4 accepted (else
// next block). Coefficients: a[5k+d] = base-769 digit d (least significant first) of Q5 value k,
// a[1020+d] = digit d of the Q4 value. Digits are produced by a 4-stage pipelined divider by 769
// (reciprocal multiply, estimate q in {q-1,q}, one correction on 11 low bits), 1 value/cycle.
//
// Handshake: start (when !busy) latches pcode/transposed; busy from the cycle after start, low
// in the done cycle; done pulses one cycle after the last slot write is on ga_w*.
// Slot write port: word w -> bank ^w[6:0], address w>>1; at most one word per cycle.
module plc_gena
  import plc_pkg::*;
(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              start,
  input  logic [2:0]        pcode,
  input  logic              transposed,
  output logic              busy,
  output logic              done,
  // hash client (client index 1 of plc_hash)
  output logic              hj_req,
  input  logic              hj_gnt,
  output logic [1:0]        hj_kind0,
  output logic [2:0]        hj_reg0,
  output logic [1:0]        hj_buf0,
  output logic [12:0]       hj_off0,
  output logic [12:0]       hj_len0,
  output logic [1:0]        hj_kind1,
  output logic [2:0]        hj_reg1,
  output logic [1:0]        hj_buf1,
  output logic [12:0]       hj_off1,
  output logic [12:0]       hj_len1,
  output logic [55:0]       hj_sfx,
  output logic [2:0]        hj_sfxlen,
  output logic [6:0]        hj_count,
  output logic [1:0]        hj_cache,
  input  logic              xo_valid,
  output logic              xo_ready,
  input  logic [255:0]      xo_data,
  input  logic              xo_last,
  // slot write port
  output logic [3:0]        ga_wslot,
  output logic [1:0]        ga_wen,
  output logic [1:0][5:0]   ga_waddr,
  output logic [1:0][127:0] ga_wdata
);
  localparam logic [47:0] Q5C = 48'd268925323054849;  // 769^5
  localparam logic [47:0] Q4C = 48'd349707832321;     // 769^4
  localparam logic [10:0] Q5BYTES = 11'd1368;
  localparam logic [10:0] Q4BYTES = 11'd30;

  // ---------------- configuration ----------------
  logic [2:0] gp;
  logic       gtr, star;
  logic [6:0] nw;              // words per poly - 1
  assign star = p_q769(gp);
  always_comb nw = 7'(p_nw(gp) - 8'd1);

  // ---------------- request side ----------------
  typedef enum logic [1:0] {R_IDLE, R_REQ, R_RECV, R_FIN} rst_t;
  rst_t       rs;
  logic [1:0] pi;              // poly index {i,j}
  logic [7:0] blk;             // stream block counter (upper 24 bits are 0)
  logic       q4m;             // Star: Q4 stage
  logic       rcv_done;        // last digest of the current job accepted
  logic [7:0] acc;             // Star: accepted Q5 values of the current poly
  logic       q4got;

  logic gx, gy;
  assign gx = gtr ? pi[1] : pi[0];
  assign gy = gtr ? pi[0] : pi[1];

  assign hj_req    = (rs == R_REQ);
  assign hj_kind0  = 2'd1;
  assign hj_reg0   = RG_PS;
  assign hj_buf0   = OB_PK;
  assign hj_off0   = '0;
  assign hj_len0   = 13'(p_pkseed(gp));
  assign hj_kind1  = 2'd0;
  assign hj_reg1   = '0;
  assign hj_buf1   = OB_PK;
  assign hj_off1   = '0;
  assign hj_len1   = '0;
  assign hj_sfx    = {blk, 8'd0, 8'd0, 8'd0, 8'd0, 7'd0, gy, 7'd0, gx};
  assign hj_sfxlen = 3'd7;
  assign hj_count  = !star ? 7'(p_n(gp) >> 5) : (q4m ? 7'd1 : 7'd43);
  assign hj_cache  = (p_pkseed(gp) == 7'd64) ? 2'd2 : 2'd0;

  // ---------------- digest chunking ----------------
  logic [1:0]  wi;             // 8-byte chunk of the current digest
  logic [63:0] ch;
  logic        ch_v, ch_take;
  logic [10:0] sqb;            // Star: squeeze bytes still to be used
  logic        drop;           // Star: target reached, discard the rest of the job
  assign ch   = xo_data[64*wi +: 64];
  logic [63:0] ch1;            // q=257: second chunk of the pair (wi is 0 or 2)
  assign ch1  = xo_data[128*wi[1] + 64 +: 64];
  assign ch_v = (rs == R_RECV) && xo_valid;

  // ---------------- Star byte FIFO / value extraction ----------------
  logic [191:0] bf;            // 24 bytes, byte i at bf[8i+:8]
  logic [4:0]   bc;
  logic [2:0]   vlen;
  logic         xpop, app;
  logic [3:0]   an;            // bytes appended
  logic [191:0] bf_n;
  assign vlen   = q4m ? 3'd5 : 3'd6;
  assign xpop   = star && (rs == R_RECV) && !drop && (bc >= {2'b0, vlen});
  // chunk accepted this cycle (star: appended or discarded; 257: written). The append space
  // check uses the count before this cycle's pop (bc <= 16), which still sustains 6 B/cycle.
  assign ch_take = ch_v && (!star || drop || (sqb == 11'd0) || (bc <= 5'd16));
  assign app     = ch_take && star && !drop && (sqb != 11'd0);
  assign an      = (sqb >= 11'd8) ? 4'd8 : sqb[3:0];
  assign xo_ready = ch_take && (star ? ((wi == 2'd3) || drop || (sqb == 11'd0)) : (wi == 2'd2));
  always_comb begin
    logic [255:0] ex;          // insert at the pre-pop count, then pop-shift
    logic [4:0]   o;
    ex = {64'd0, bf};
    for (int i = 0; i < 32; i++) begin
      o = 5'(i) - bc;
      if (app && (5'(i) >= bc) && (o < {1'b0, an})) ex[8*i +: 8] = ch[8*o[2:0] +: 8];
    end
    bf_n = !xpop ? ex[191:0] : q4m ? ex[231:40] : ex[239:48];
  end

  // compare stage
  logic        cmpv;
  logic [47:0] cmpx;
  logic        acc_ok;         // value accepted
  assign acc_ok = cmpv && (q4m ? (cmpx < Q4C && !q4got) : (cmpx < Q5C && acc < 8'd204));

  // decision point of a Star squeeze
  logic dec_ok;
  assign dec_ok = star && (rs == R_RECV) && rcv_done && (drop || bc < {2'b0, vlen}) && !cmpv;

  // ---------------- divide-by-769 pipeline ----------------
  // stage s: A regs (x, digits so far) -> B regs (q estimate, x[10:0]) -> next A regs
  localparam logic [38:0] M0 = 39'((64'd1 << 48) / 64'd769);   // floor(2^48/769)
  localparam logic [29:0] M1 = 30'((64'd1 << 39) / 64'd769);   // floor(2^39/769)
  localparam logic [19:0] M2 = 20'((64'd1 << 29) / 64'd769);   // floor(2^29/769)
  localparam logic [10:0] M3 = 11'((64'd1 << 20) / 64'd769);   // floor(2^20/769)

  // correction: t = x - qe*769 (exact in 11 bits since it lies in [0,1538)); returns {inc, digit}
  function automatic logic [10:0] corr(input logic [10:0] xl, input logic [10:0] qel);
    logic [10:0] t, p;
    p = (qel << 9) + (qel << 8) + qel;
    t = xl - p;
    if (t >= 11'd769) corr = {1'b1, 10'(t - 11'd769)};
    else              corr = {1'b0, t[9:0]};
  endfunction

  logic        a0v, b0v, a1v, b1v, a2v, b2v, a3v, b3v, ov;
  logic        a0t, b0t, a1t, b1t, a2t, b2t, a3t, b3t, ot;   // tag: Q4 value
  logic [47:0] a0x;
  logic [38:0] b0q, a1x;
  logic [28:0] b1q, a2x;
  logic [19:0] b2q, a3x;
  logic [9:0]  b3q;
  logic [10:0] b0l, b1l, b2l, b3l;
  logic [9:0]  a1d0, b1d0, a2d0, b2d0, a3d0, b3d0;
  logic [9:0]  a2d1, b2d1, a3d1, b3d1;
  logic [9:0]  a3d2, b3d2;
  logic [4:0][9:0] od;
  logic [86:0] pr0;
  logic [68:0] pr1;
  logic [48:0] pr2;
  logic [30:0] pr3;
  logic [10:0] c0, c1, c2, c3;
  assign pr0 = a0x * M0;
  assign pr1 = a1x * M1;
  assign pr2 = a2x * M2;
  assign pr3 = a3x * M3;
  assign c0  = corr(b0l, b0q[10:0]);
  assign c1  = corr(b1l, b1q[10:0]);
  assign c2  = corr(b2l, b2q[10:0]);
  assign c3  = corr(b3l, {1'b0, b3q});

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      a0v <= 1'b0; b0v <= 1'b0; a1v <= 1'b0; b1v <= 1'b0;
      a2v <= 1'b0; b2v <= 1'b0; a3v <= 1'b0; b3v <= 1'b0; ov <= 1'b0;
    end else begin
      a0v <= acc_ok; b0v <= a0v; a1v <= b0v; b1v <= a1v;
      a2v <= b1v; b2v <= a2v; a3v <= b2v; b3v <= a3v; ov <= b3v;
    end
  end
  always_ff @(posedge clk) begin
    a0x <= cmpx; a0t <= q4m;
    b0q <= pr0[86:48]; b0l <= a0x[10:0]; b0t <= a0t;
    a1x <= b0q + {38'd0, c0[10]}; a1d0 <= c0[9:0]; a1t <= b0t;
    b1q <= pr1[67:39]; b1l <= a1x[10:0]; b1d0 <= a1d0; b1t <= a1t;
    a2x <= b1q + {28'd0, c1[10]}; a2d0 <= b1d0; a2d1 <= c1[9:0]; a2t <= b1t;
    b2q <= pr2[48:29]; b2l <= a2x[10:0]; b2d0 <= a2d0; b2d1 <= a2d1; b2t <= a2t;
    a3x <= b2q + {19'd0, c2[10]}; a3d0 <= b2d0; a3d1 <= b2d1; a3d2 <= c2[9:0]; a3t <= b2t;
    b3q <= pr3[29:20]; b3l <= a3x[10:0]; b3d0 <= a3d0; b3d1 <= a3d1; b3d2 <= a3d2; b3t <= a3t;
    od[0] <= b3d0; od[1] <= b3d1; od[2] <= b3d2; od[3] <= c3[9:0];
    od[4] <= b3q + {9'd0, c3[10]};
    ot <= b3t;
  end

  // ---------------- coefficient packer (Star) ----------------
  logic [15:0][9:0] pk;
  logic [3:0]       pc;
  logic [15:0][9:0] pk_a;      // after append
  logic [4:0]       pc_a;
  always_comb begin
    logic [3:0] o;
    pk_a = pk;
    for (int i = 0; i < 16; i++) begin
      o = 4'(i) - pc;
      if (ov && 4'(i) >= pc && o < (ot ? 4'd4 : 4'd5)) pk_a[i] = od[o[2:0]];
    end
    pc_a = {1'b0, pc} + (ov ? (ot ? 5'd4 : 5'd5) : 5'd0);
  end

  // ---------------- write side ----------------
  logic         wv;            // a word is produced this cycle
  logic [127:0] wd, wd1;
  logic [1:0]   wpi;
  logic [6:0]   wword;
  logic         ow_v, lastw;
  logic [6:0]   ow_word;
  logic [3:0]   ow_slot;
  logic [1:0]   ow_en;
  logic [1:0][127:0] ow_d;
  logic         wbank, wlast;
  assign wbank = ^wword;
  assign wlast = star ? (wword == nw) : (wword == nw - 7'd1);
  always_comb begin
    if (!star) begin
      wv = ch_take;
      for (int k = 0; k < 8; k++) wd[16*k +: 16]  = {8'd0, ch[8*k +: 8]};
      for (int k = 0; k < 8; k++) wd1[16*k +: 16] = {8'd0, ch1[8*k +: 8]};
    end else begin
      wv = pc_a[3];
      for (int k = 0; k < 8; k++) wd[16*k +: 16] = {6'd0, pk_a[k]};
      wd1 = wd;
    end
  end
  assign ga_wslot = ow_slot;
  assign ga_wen      = ow_v ? ow_en : 2'b00;
  assign ga_waddr[0] = ow_word[6:1];
  assign ga_waddr[1] = ow_word[6:1];
  assign ga_wdata    = ow_d;

  // ---------------- sequential control ----------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      gp <= '0; gtr <= 1'b0; busy <= 1'b0; done <= 1'b0;
      rs <= R_IDLE; pi <= '0; blk <= '0; q4m <= 1'b0; rcv_done <= 1'b0; acc <= '0; q4got <= 1'b0;
      wi <= '0; sqb <= '0; drop <= 1'b0; bc <= '0; cmpv <= 1'b0;
      pc <= '0; wpi <= '0; wword <= '0; ow_v <= 1'b0; lastw <= 1'b0;
    end else begin
      done  <= 1'b0;
      ow_v  <= wv;
      lastw <= 1'b0;
      if (lastw) begin
        done <= 1'b1;
        busy <= 1'b0;
      end
      // request / receive
      case (rs)
        R_REQ: if (hj_gnt) begin
          rs <= R_RECV; rcv_done <= 1'b0; wi <= '0; drop <= 1'b0; bc <= '0;
          sqb <= q4m ? Q4BYTES : Q5BYTES;
        end
        R_RECV: begin
          if (xo_ready && xo_last) rcv_done <= 1'b1;
          if (!star && xo_ready && xo_last) begin
            pi <= pi + 2'd1;
            rs <= (pi == 2'd3) ? R_FIN : R_REQ;
          end
          if (dec_ok) begin
            blk <= blk + 8'd1;
            rs  <= R_REQ;
            if (q4m) begin
              if (q4got) begin
                blk <= '0; q4m <= 1'b0; acc <= '0; q4got <= 1'b0;
                pi  <= pi + 2'd1;
                rs  <= (pi == 2'd3) ? R_FIN : R_REQ;
              end
            end else if (acc == 8'd204) begin
              q4m <= 1'b1;
            end
          end
        end
        default: ;
      endcase

      // chunk pointer
      if (ch_take) wi <= xo_ready ? 2'd0 : wi + (star ? 2'd1 : 2'd2);
      if (app) sqb <= sqb - {7'd0, an};

      // Star value path
      if (rs == R_RECV) begin
        bc   <= drop ? 5'd0 : bc - (xpop ? {2'b0, vlen} : 5'd0) + (app ? {1'b0, an} : 5'd0);
        cmpv <= xpop;
        if (acc_ok) begin
          if (q4m) q4got <= 1'b1;
          else     acc   <= acc + 8'd1;
        end
        if (acc_ok && (q4m || acc == 8'd203)) drop <= 1'b1;
      end else begin
        cmpv <= 1'b0;
      end

      // packer + write counters
      if (star) pc <= wv ? 4'(pc_a - 5'd8) : pc_a[3:0];
      if (wv) begin
        if (wlast) begin
          wword <= '0;
          wpi   <= wpi + 2'd1;
          if (wpi == 2'd3) lastw <= 1'b1;
        end else begin
          wword <= wword + (star ? 7'd1 : 7'd2);
        end
      end

      if (lastw) rs <= R_IDLE;
      if (start && !busy) begin
        gp <= pcode; gtr <= transposed; busy <= 1'b1;
        rs <= R_REQ; pi <= '0; blk <= '0; q4m <= 1'b0; acc <= '0; q4got <= 1'b0;
        pc <= '0; wpi <= '0; wword <= '0;
      end
    end
  end

  always_ff @(posedge clk) begin
    bf <= bf_n;
    if (xpop) cmpx <= q4m ? {9'd0, bf[38:0]} : bf[47:0];
    if (star) pk <= wv ? {80'd0, pk_a[15:8]} : pk_a;
    if (wv) begin
      ow_word <= wword;
      ow_slot <= {2'b00, wpi};
      // q=257: words wword (even) and wword+1 share address wword>>1 in opposite banks
      ow_en         <= star ? (wbank ? 2'b10 : 2'b01) : 2'b11;
      ow_d[wbank]   <= wd;
      ow_d[!wbank]  <= star ? wd : wd1;
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin
    if (rst_n && start && busy) $error("plc_gena: start while busy");
    if (rst_n && star && blk == 8'hff && rs == R_REQ) $error("plc_gena: block counter overflow");
  end
`endif
endmodule
