// SM3 hash channel: one 2-round SM3 core fed by an 8-byte/cycle message assembler.
//
// A call hashes init_state || segments (|| padding), where init_state is the SM3 IV, the fixed-key
// HMAC midstate MK1 (after K xor ipad) or MK2 (after K xor opad), or the channel's saved midstate
// MID; `prefix` is the number of bytes already absorbed by init_state (for the length field).
// Segments (up to 3): REG (regfile words), OBJ (object-buffer words, 8-byte aligned, optional wait
// on the PK written bitmap), IMM (1..4 immediate bytes, first byte in imm[31:24]).  All REG/OBJ
// segment lengths are multiples of 8.  OWN: the 32-byte digest of this channel's previous call
// (still on the core output), appended in one cycle at buffer offset 0 or 32.  Padding takes one
// cycle when the length field fits the current block (the buffer beyond the fill is zero).
// A 4-word digest write-back to a 4-aligned regfile address is one wide write.  nofinal: no padding, the final chaining value is saved to MID
// (the total length is then a multiple of 64).  Otherwise the first dstw words of the digest are
// written to the regfile at dst (digest byte 8i+k -> word dst+i bits [8k+7:8k]).
module aigis_sm3_chan (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,
  input  logic [1:0]   d_init,      // 0 IV, 1 MK1, 2 MK2, 3 MID
  input  logic [12:0]  d_prefix,
  input  logic [1:0]   d_nseg,      // 1..3
  input  logic [2:0][58:0] d_seg,   // {kind[1:0], obj[1:0], addr[9:0], len[12:0], imm[31:0]}
  input  logic         d_nofinal,
  input  logic [6:0]   d_dst,
  input  logic [2:0]   d_dstw,
  input  logic         d_waitpk,
  output logic         busy,
  output logic         done,         // combinational in the final cycle of a wide/midstate call
  // regfile
  output logic [6:0]   rf_raddr,
  input  logic [63:0]  rf_rdata,
  input  logic         rf_rvalid,    // the addressed regfile word is valid for this command
  output logic         rf_we,
  output logic [6:0]   rf_waddr,
  output logic [63:0]  rf_wdata,
  output logic         rf_we4,
  output logic [4:0]   rf_waddr4,
  output logic [3:0][63:0] rf_wdata4,
  // object buffer read client (data 2 cycles after grant)
  output logic         ob_req,
  output logic [1:0]   ob_sel,
  output logic [9:0]   ob_addr,
  input  logic         ob_gnt,
  input  logic         ob_rv,
  input  logic [63:0]  ob_rdata,
  input  logic [327:0] pkw
);
  localparam logic [255:0] IV  = 256'h7380166f4914b2b9172442d7da8a0600a96f30bc163138aae38dee4db0fb0e4e;
  localparam logic [255:0] MK1 = 256'h313eb6a448bef2d536e50983d44b6bab9f2008548371a799073ed128d0a8dd79;
  localparam logic [255:0] MK2 = 256'h747cc75de966127aa24ac00f62ab5c3f1dd5568d84c3306505fff7c2b97f411a;
  localparam logic [1:0] K_REG = 2'd0, K_OBJ = 2'd1, K_IMM = 2'd2, K_OWN = 2'd3;
  typedef enum logic [2:0] {P_IDLE, P_SEG, P_80, P_ZER, P_LEN, P_FIN, P_WB} ph_t;

  ph_t ph;
  logic [1:0]  init_q, nseg_q, si;
  logic [2:0][58:0] seg_q;
  logic        nofinal_q, waitpk_q;
  logic [6:0]  dst_q;
  logic [2:0]  dstw_q, wbi;
  logic [63:0] totbits;
  logic [255:0] mid;

  logic [1:0]  skind, sobj;
  logic [9:0]  saddr;
  logic [12:0] slen;
  logic [31:0] simm;
  assign {skind, sobj, saddr, slen, simm} = seg_q[si];

  logic [12:0] sdone;       // bytes of the current segment appended
  logic [9:0]  sissued;     // OBJ words requested
  // OBJ prefetch FIFO
  logic [3:0][63:0] fq;
  logic [2:0]  fcnt;        // entries in fifo
  logic [2:0]  fout;        // outstanding reads
  logic [1:0]  fhead, ftail;

  // assembly buffer
  logic [71:0][7:0] bb;
  logic [6:0]  f;

  // core
  logic        c_start, c_cont, c_busy, c_done, c_lastc;
  logic [255:0] c_iv, c_vout;
  logic [511:0] c_blk;
  aigis_sm3_core u_core (.clk(clk), .rst_n(rst_n), .start(c_start), .cont(c_cont), .iv(c_iv), .blk(c_blk),
                         .busy(c_busy), .done(c_done), .lastc(c_lastc), .vout(c_vout));
  logic [7:0]  nsent, ndone;
  logic        last_sent, first_pending;

  // ------------------------------------------------------------------ chunk selection
  logic [7:0][7:0] chunk;
  logic [3:0]  k;           // bytes appended this cycle
  logic        app;
  logic        seg_end;     // this append finishes the current segment
  logic        pop;
  logic        app_own, app_pad, fjump;
  logic [6:0]  fj;
  always_comb begin
    chunk = '0; k = 4'd0; app = 1'b0; seg_end = 1'b0; pop = 1'b0;
    app_own = 1'b0; app_pad = 1'b0; fjump = 1'b0; fj = f;
    rf_raddr = saddr + sdone[9:3];
    if (f <= 7'd64) begin
      case (ph)
        P_SEG: begin
          if (skind == K_REG) begin
            if (rf_rvalid) begin
              for (int i = 0; i < 8; i++) chunk[i] = rf_rdata[8*i +: 8];
              k = 4'd8; app = 1'b1;
              seg_end = (sdone + 13'd8 >= slen);
            end
          end else if (skind == K_OBJ) begin
            if (fcnt != 3'd0) begin
              for (int i = 0; i < 8; i++) chunk[i] = fq[fhead][8*i +: 8];
              k = 4'd8; app = 1'b1; pop = 1'b1;
              seg_end = (sdone + 13'd8 >= slen);
            end
          end else if (skind == K_OWN) begin
            if (f <= 7'd32) begin app_own = 1'b1; seg_end = 1'b1; end
          end else begin
            for (int i = 0; i < 4; i++) chunk[i] = simm[31 - 8*i -: 8];
            k = 4'(slen); app = 1'b1; seg_end = 1'b1;
          end
        end
        P_80: begin
          if (f <= 7'd55) app_pad = 1'b1;
          else begin chunk[0] = 8'h80; k = 4'd1; app = 1'b1; end
        end
        P_ZER: begin fjump = 1'b1; fj = (f <= 7'd56) ? 7'd56 : 7'd64; end
        P_LEN: begin
          if (f == 7'd56) begin
            for (int i = 0; i < 8; i++) chunk[i] = totbits[63 - 8*i -: 8];
            k = 4'd8; app = 1'b1;
          end
        end
        default: ;
      endcase
    end
  end

  // ------------------------------------------------------------------ block handoff to the core
  logic blk_ready, hand, hand_start, hand_cont, hand_last;
  assign blk_ready = (f >= 7'd64) && (ph != P_IDLE) && (ph != P_WB) && !last_sent;
  assign hand_cont  = blk_ready && c_lastc && !first_pending;
  assign hand_start = blk_ready && !c_busy && (first_pending || ((ndone + {7'd0, c_done}) == nsent));
  assign hand = hand_cont || hand_start;
  assign hand_last = (ph == P_FIN) && (f == 7'd64);
  always_comb begin
    for (int i = 0; i < 64; i++) c_blk[511 - 8*i -: 8] = bb[i];
    c_start = hand_start;
    c_cont  = hand_cont;
    case (init_q)
      2'd0: c_iv = first_pending ? IV  : c_vout;
      2'd1: c_iv = first_pending ? MK1 : c_vout;
      2'd2: c_iv = first_pending ? MK2 : c_vout;
      default: c_iv = first_pending ? mid : c_vout;
    endcase
  end

  // ------------------------------------------------------------------ OBJ prefetch
  logic [9:0] swords;
  assign swords = slen[12:3];
  always_comb begin
    ob_req  = (ph == P_SEG) && (skind == K_OBJ) && (sissued < swords) && ({1'b0, fcnt} + {1'b0, fout} < 4'd4)
              && (!(waitpk_q && sobj == 2'd0) || pkw[saddr + sissued]);
    ob_sel  = sobj;
    ob_addr = saddr + sissued;
  end

  // ------------------------------------------------------------------ sequential
  logic [71:0][7:0] bbA;
  logic [7:0] fA;
  logic pad_now;
  always_comb begin
    bbA = bb;
    for (int i = 0; i < 8; i++) if (app && i < k) bbA[f + i] = chunk[i];
    fA = {1'b0, f} + (app ? {4'd0, k} : 8'd0);
    if (app_own) begin
      for (int i = 0; i < 32; i++) begin
        if (f[5]) bbA[32 + i] = c_vout[255 - 8*i -: 8];
        else      bbA[i]      = c_vout[255 - 8*i -: 8];
      end
      fA = {1'b0, f} + 8'd32;
    end
    if (app_pad) begin
      bbA[f] = 8'h80;
      for (int i = 0; i < 8; i++) bbA[56 + i] = totbits[63 - 8*i -: 8];
      fA = 8'd64;
    end
    if (fjump) fA = {1'b0, fj};
    // padding merged into the final segment append when the length field fits this block
    pad_now = (ph == P_SEG) && seg_end && (app || app_own) && (si + 2'd1 == nseg_q) && !nofinal_q && (fA <= 8'd55);
    if (pad_now) begin
      bbA[fA[6:0]] = 8'h80;
      for (int i = 0; i < 8; i++) bbA[56 + i] = totbits[63 - 8*i -: 8];
      fA = 8'd64;
    end
  end

  // the final block completes this cycle: digest written (registered) and done, a new call may start
  logic fin_now, fin_wide;
  assign fin_now  = (ph == P_FIN) && last_sent && c_done && (ndone + 8'd1 == nsent);
  assign fin_wide = fin_now && (nofinal_q || (dstw_q == 3'd4 && dst_q[1:0] == 2'b00));
  logic done_wb;
  assign done = fin_wide || done_wb;
  // 4-word digest write-back in the final cycle (the regfile captures it at the end of that cycle)
  always_comb begin
    rf_we4 = fin_wide && !nofinal_q;
    rf_waddr4 = dst_q[6:2];
    for (int w = 0; w < 4; w++) for (int kk = 0; kk < 8; kk++) rf_wdata4[w][8*kk +: 8] = c_vout[255 - 64*w - 8*kk -: 8];
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ph <= P_IDLE; busy <= 1'b0; done_wb <= 1'b0; f <= '0; fcnt <= '0; fout <= '0; fhead <= '0; ftail <= '0;
      nsent <= '0; ndone <= '0; last_sent <= 1'b0; first_pending <= 1'b0; si <= '0; sdone <= '0;
      sissued <= '0; wbi <= '0; rf_we <= 1'b0;
      init_q <= '0; nseg_q <= '0; seg_q <= '0; nofinal_q <= 1'b0; waitpk_q <= 1'b0; dst_q <= '0; dstw_q <= '0;
      totbits <= '0; fq <= '0; bb <= '0; mid <= '0; rf_waddr <= '0; rf_wdata <= '0;
    end else begin
      done_wb <= 1'b0;
      rf_we <= 1'b0;
      if (fin_wide) begin
        // final digest of the running call (also in the cycle a new call is accepted)
        if (nofinal_q) mid <= c_vout;
        ph <= P_IDLE; busy <= 1'b0;
      end
      if (start && (!busy || fin_wide)) begin
        busy <= 1'b1; ph <= P_SEG; f <= '0; si <= '0; sdone <= '0; sissued <= '0; bb <= '0;
        fcnt <= '0; fout <= '0; fhead <= '0; ftail <= '0;
        nsent <= '0; ndone <= '0; last_sent <= 1'b0; first_pending <= 1'b1;
        init_q <= d_init; nseg_q <= d_nseg; seg_q <= d_seg; nofinal_q <= d_nofinal;
        dst_q <= d_dst; dstw_q <= d_dstw; waitpk_q <= d_waitpk;
        totbits <= {48'd0, (d_prefix + (d_nseg > 2'd0 ? d_seg[0][44:32] : 13'd0)
                               + (d_nseg > 2'd1 ? d_seg[1][44:32] : 13'd0)
                               + (d_nseg > 2'd2 ? d_seg[2][44:32] : 13'd0)), 3'd0};
      end else if (busy && !fin_wide) begin
        // fifo bookkeeping
        if (ob_req && ob_gnt) begin sissued <= sissued + 10'd1; end
        if (ob_rv) begin fq[ftail] <= ob_rdata; ftail <= ftail + 2'd1; end
        fcnt <= fcnt + (ob_rv ? 3'd1 : 3'd0) - (pop ? 3'd1 : 3'd0);
        fout <= fout + ((ob_req && ob_gnt) ? 3'd1 : 3'd0) - (ob_rv ? 3'd1 : 3'd0);
        if (pop) fhead <= fhead + 2'd1;
        // append + handoff
        if (hand) begin
          for (int i = 0; i < 72; i++) bb[i] <= (i < 8) ? bbA[64 + i] : 8'h00;
          f <= 7'(fA - 8'd64);
          nsent <= nsent + 8'd1; first_pending <= 1'b0;
          if (hand_last) last_sent <= 1'b1;
        end else begin
          bb <= bbA; f <= fA[6:0];
        end
        if (c_done) ndone <= ndone + 8'd1;
        // phase transitions caused by the append
        case (ph)
          P_SEG: if (app || app_own) begin
            sdone <= (skind == K_IMM) ? 13'd0 : sdone + 13'd8;
            if (seg_end) begin
              sdone <= '0; sissued <= '0;
              if (si + 2'd1 == nseg_q) ph <= (nofinal_q || pad_now) ? P_FIN : P_80;
              else si <= si + 2'd1;
            end
          end
          P_80: if (app_pad) ph <= P_FIN; else if (app) ph <= P_ZER;
          P_ZER: if (fA == 8'd56) ph <= P_LEN;
          P_LEN: if (app) ph <= P_FIN;
          P_FIN: if (fin_now && !fin_wide) begin ph <= P_WB; wbi <= '0; end
          P_WB: begin
            rf_we <= 1'b1;
            rf_waddr <= dst_q + 7'(wbi);
            for (int kk = 0; kk < 8; kk++) rf_wdata[8*kk +: 8] <= c_vout[255 - 64*wbi - 8*kk -: 8];
            wbi <= wbi + 3'd1;
            if (wbi + 3'd1 == dstw_q) begin ph <= P_IDLE; busy <= 1'b0; done_wb <= 1'b1; end
          end
          default: ;
        endcase
      end
    end
  end
endmodule
