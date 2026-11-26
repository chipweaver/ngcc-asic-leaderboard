// plc_tern - ternary candidate sampler, bit-exact with sample.c poly_generate_tenary(a, seed, nonce)
// of every parameter set.  One hash job per candidate (issued by this block as client 0 of plc_hash):
//   msg = RG_SD[0..64) || nonce || be32(0)  (bit_xof128_stream_squeeze, block 0), count = p_tbytes/32
// The descriptor constants (seg0 = RG_SD len 64, hj_cache = 1, sfx_len = 5) are driven by plc_tsamp;
// this block drives hj_req, hj_sfx and hj_count.
//
// Kinds (plc_pkg::p_tkind), buf = squeezed bytes, L = N/8:
//  0 (Light N=256, 256 N=512): t[x] = bit (x div 3L) of buf[x mod 3L];  a[c] = (t[c]-t[c+N])*t[c+2N].
//    All 3L = 96/192 bytes are buffered in a ring of R = 3L/8 = 12/24 64-bit chunks (digests shift in at the
//    tail).  Word w (coeffs 8w..8w+7) needs, for each role r in {c, c+N, c+2N}, the 8 consecutive bytes of
//    chunk (x_r mod 3L)/8 at bit i_r = x_r div 3L.  Words are emitted in order w = 0..NW-1 while the ring
//    rotates by one chunk per word, so the three chunks sit at fixed ring taps (0, N/8 mod R, 2N/8 mod R)
//    and only the bit indices i_r advance (when the role's chunk index wraps).
//  1 (128 N=256, 512-Star N=1024): byte j -> a[j] = (b0&b2)-(b4&b6), a[j+N/2] = (b1&b3)-(b5&b7).
//    8 bytes -> word g (low half) and word g+NW/2 (other bank), both written in one cycle.
//  2 (512 N=1024): 7 bytes x0..x6 -> 8 coeffs, k = x_i & 127 (i<7), k7 = {x0[7],...,x6[7]};
//    coeff = +1 if k > 116, -1 if k < 11, else 0.  One word per 7 bytes.
//  Kinds 1/2 use a 40-byte byte queue (digests appended when fewer than one group is queued).
// Slot write: word w -> bank ^w[6:0], address w>>1.  Write port outputs are registered;
// done pulses in the cycle the last word write is on the port (it lands at the end of that cycle).
module plc_tern (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              start,      // only when !busy
  input  logic [2:0]        pcode,
  input  logic [7:0]        nonce,
  input  logic [3:0]        slot,
  input  logic              discard,    // level: suppress slot writes (speculative work being drained)
  output logic              busy,
  output logic              done,
  // hash job client (descriptor constants driven by the parent)
  output logic              hj_req,
  input  logic              hj_gnt,
  output logic [55:0]       hj_sfx,
  output logic [6:0]        hj_count,
  input  logic              xo_valid,
  output logic              xo_ready,
  input  logic [255:0]      xo_data,
  // slot write port
  output logic [3:0]        wslot,
  output logic [1:0]        wen,
  output logic [1:0][5:0]   waddr,
  output logic [1:0][127:0] wdata
);
  import plc_pkg::*;

  typedef enum logic [1:0] {S_IDLE, S_REQ, S_RUN} st_t;
  st_t        st;
  logic [1:0] kind;
  logic       r12;            // kind 0 with N = 256 (ring of 12 chunks)
  logic [6:0] nw_m1;          // last emitted word/pair index
  logic [6:0] nwh;            // NW/2
  logic [7:0] nonce_r;
  logic [3:0] slot_r;
  logic [4:0] dleft;          // digests still to accept (<= 28)
  logic [6:0] ne;             // emitted word (kind 0/2) / pair (kind 1) counter
  logic       done_q;

  assign hj_req   = (st == S_REQ);
  assign hj_sfx   = {48'd0, nonce_r};            // nonce || be32(block = 0)
  assign hj_count = 7'(dleft);                   // stable while requesting
  assign busy     = (st != S_IDLE) | done_q;
  assign done     = done_q;
  assign wslot    = slot_r;

  // ---------------------------------------------------------------- kind 0 ring
  logic [23:0][63:0] ring;
  logic [4:0]        cp, cn, cm;     // chunk index of each role (mod R)
  logic [2:0]        ip, in_, im;    // bit index of each role
  wire  [4:0]        rlast = r12 ? 5'd11 : 5'd23;

  // ---------------------------------------------------------------- kind 1/2 byte queue
  logic [39:0][7:0]  q;
  logic [5:0]        qcnt;
  wire  [5:0]        grp = (kind == 2'd1) ? 6'd8 : 6'd7;

  wire loading0 = (st == S_RUN) && (kind == 2'd0) && (dleft != 5'd0);
  wire emit0    = (st == S_RUN) && (kind == 2'd0) && (dleft == 5'd0);
  wire emitq    = (st == S_RUN) && (kind != 2'd0) && (qcnt >= grp);
  wire emit     = emit0 | emitq;
  assign xo_ready = loading0 || ((st == S_RUN) && (kind != 2'd0) && (dleft != 5'd0) && (qcnt < grp));
  wire acc_d    = xo_valid && xo_ready;
  wire last_emit = emit && (ne == nw_m1);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE; kind <= '0; r12 <= 1'b0; nw_m1 <= '0; nwh <= '0; nonce_r <= '0; slot_r <= '0;
      dleft <= '0; ne <= '0; qcnt <= '0; done_q <= 1'b0;
      cp <= '0; cn <= '0; cm <= '0; ip <= '0; in_ <= '0; im <= '0;
    end else begin
      done_q <= 1'b0;
      case (st)
        S_IDLE: if (start) begin
          st      <= S_REQ;
          kind    <= p_tkind(pcode);
          r12     <= (p_tkind(pcode) == 2'd0) && (p_ncode(pcode) == 2'd0);
          nwh     <= 7'(p_nw(pcode) >> 1);
          nw_m1   <= (p_tkind(pcode) == 2'd1) ? 7'((p_nw(pcode) >> 1) - 8'd1) : 7'(p_nw(pcode) - 8'd1);
          dleft   <= 5'(p_tbytes(pcode) >> 5);
          nonce_r <= nonce;
          slot_r  <= slot;
          ne      <= '0;
          qcnt    <= '0;
          cp      <= 5'd0;
          cn      <= (p_ncode(pcode) == 2'd0) ? 5'd8 : 5'd16;
          cm      <= (p_ncode(pcode) == 2'd0) ? 5'd4 : 5'd8;
          ip      <= 3'd0; in_ <= 3'd2; im <= 3'd5;
        end
        S_REQ: if (hj_gnt) st <= S_RUN;
        default: begin
          if (acc_d) dleft <= dleft - 5'd1;
          if (acc_d && kind != 2'd0) qcnt <= qcnt + 6'd32;
          if (emitq) qcnt <= qcnt - grp;
          if (emit) begin
            ne <= ne + 7'd1;
            if (last_emit) begin
              st     <= S_IDLE;
              done_q <= 1'b1;
            end
          end
          if (emit0) begin
            if (cp == rlast) begin cp <= '0; ip <= ip + 3'd1; end else cp <= cp + 5'd1;
            if (cn == rlast) begin cn <= '0; in_ <= in_ + 3'd1; end else cn <= cn + 5'd1;
            if (cm == rlast) begin cm <= '0; im <= im + 3'd1; end else cm <= cm + 5'd1;
          end
        end
      endcase
    end
  end

  // ring and queue data (no reset needed)
  always_ff @(posedge clk) begin
    if (loading0 && xo_valid) begin
      for (int k = 0; k < 20; k++) ring[k] <= ring[k+4];
      for (int c = 0; c < 4; c++) begin
        ring[20+c] <= xo_data[64*c +: 64];
        if (r12) ring[8+c] <= xo_data[64*c +: 64];
      end
    end else if (emit0) begin
      for (int k = 0; k < 23; k++) ring[k] <= ring[k+1];
      ring[23] <= ring[0];
      if (r12) ring[11] <= ring[0];
    end
    if (acc_d && kind != 2'd0) begin
      // append at offset qcnt (< grp <= 8 whenever a digest is accepted): 8:1 byte mux
      for (int p = 0; p < 40; p++)
        for (int o = 0; o < 8; o++)
          if (qcnt[2:0] == 3'(o) && p >= o && p < o + 32) q[p] <= xo_data[8*(p - o) +: 8];
    end else if (emitq) begin
      for (int p = 0; p < 40; p++)
        if (p + grp < 40) q[p] <= q[p + grp];
    end
  end

  // ---------------------------------------------------------------- coefficient generation
  function automatic logic [15:0] tern16(input logic pos, input logic neg);
    return (pos && !neg) ? 16'h0001 : (!pos && neg) ? 16'hFFFF : 16'h0000;
  endfunction
  function automatic logic [15:0] thr16(input logic [6:0] k);
    return (k > 7'd116) ? 16'h0001 : (k < 7'd11) ? 16'hFFFF : 16'h0000;
  endfunction

  logic [127:0] wd_lo, wd_hi;      // word for kind 0/2 in wd_lo; kind 1: lo/hi half words
  always_comb begin
    logic [63:0] tp, tn, tm;
    tp = ring[0];
    tn = r12 ? ring[8] : ring[16];
    tm = r12 ? ring[4] : ring[8];
    wd_lo = '0; wd_hi = '0;
    case (kind)
      2'd0: for (int l = 0; l < 8; l++)
              wd_lo[16*l +: 16] = tm[8*l + im] ? tern16(tp[8*l + ip], tn[8*l + in_]) : 16'h0000;
      2'd1: for (int l = 0; l < 8; l++) begin
              wd_lo[16*l +: 16] = tern16(q[l][0] & q[l][2], q[l][4] & q[l][6]);
              wd_hi[16*l +: 16] = tern16(q[l][1] & q[l][3], q[l][5] & q[l][7]);
            end
      default: begin
              for (int l = 0; l < 7; l++) wd_lo[16*l +: 16] = thr16(q[l][6:0]);
              wd_lo[127:112] = thr16({q[0][7], q[1][7], q[2][7], q[3][7], q[4][7], q[5][7], q[6][7]});
            end
    endcase
  end

  // ---------------------------------------------------------------- registered write port
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) wen <= '0;
    else begin
      wen <= '0;
      if (emit && !discard) begin
        if (kind == 2'd1) wen <= 2'b11;
        else              wen[^ne] <= 1'b1;
      end
    end
  end
  always_ff @(posedge clk) begin
    if (emit) begin
      logic       bk;
      logic [6:0] whi;
      bk  = ^ne;                       // bank of word ne (kind 1: low-half word g = ne)
      whi = ne | nwh;                  // kind 1: word g + NW/2 (g < NW/2)
      waddr[bk] <= ne[6:1];
      wdata[bk] <= wd_lo;
      if (kind == 2'd1) begin
        waddr[~bk] <= whi[6:1];
        wdata[~bk] <= wd_hi;
      end
    end
  end
endmodule
