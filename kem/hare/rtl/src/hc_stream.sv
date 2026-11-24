// Streaming SM3 hasher for pseudoXOF over long or register-sourced messages:
//   out = SM3(M || BE32(1)) || SM3(M || BE32(2)) || ... (nout 32-byte blocks)
//   M = tag || seg0 || seg1 || seg2, each segment from the register file (REG), the key buffer
//   (KEY) or the CT buffer (CT), byte-addressed.
// The message is fed up to two bytes per cycle into a 64-byte fill buffer (two bytes whenever both
// lie in the current segment and, for buffer segments, in the head word); full blocks are
// compressed by a two-round-per-cycle SM3 core (33 cycles) while the next block fills. The zero
// padding and the 64-bit length field are written in one cycle each. The chaining value after the
// last full block of M and the remaining tail bytes are kept, so each output block costs only its
// 1-2 tail compressions.
module hc_stream import hare_pkg::*; (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,
  input  logic         abort,
  input  logic [7:0]   tag,
  input  logic [1:0]   seg_src  [3],   // 0 none, 1 REG, 2 KEY buffer, 3 CT buffer
  input  logic [15:0]  seg_base [3],
  input  logic [15:0]  seg_len  [3],
  input  logic [2:0]   nout,
  output logic [7:0]   reg_idx,
  input  logic [7:0]   reg_byte,       // register byte reg_idx
  input  logic [7:0]   reg_byte2,      // register byte reg_idx + 1
  output ob_req_t      kreq,
  input  logic         kgnt,
  input  logic [63:0]  kq,
  output ob_req_t      creq,
  input  logic         cgnt,
  input  logic [63:0]  cq,
  output logic         out_valid,
  output logic [1:0]   out_idx,
  output logic [255:0] out_data,       // byte j = out_data[8j+7:8j]
  output logic         busy
);
  localparam logic [255:0] IV = 256'h7380166F4914B2B9172442D7DA8A0600A96F30BC163138AAE38DEE4DB0FB0E4E;
  typedef enum logic [3:0] {S_IDLE, S_TAG, S_SEG, S_ENDM, S_CT, S_P80, S_ZERO, S_LEN, S_FIN, S_NEXT} st_t;
  st_t st;

  // job registers
  logic [1:0]  src_q [3];
  logic [15:0] base_q [3];
  logic [15:0] len_q [3];
  logic [2:0]  nout_q;
  logic [7:0]  tag_q;
  logic [15:0] mlen;            // total message bytes L
  logic [1:0]  seg;             // current segment
  logic [15:0] scnt;            // bytes of current segment emitted
  logic [2:0]  oidx;            // output block index
  logic [2:0]  bcnt;            // byte counter for CT / LEN fields

  // fill buffer and saved tail
  logic [7:0]  fill [64];
  logic [6:0]  pos;
  logic [7:0]  tail [64];
  logic [6:0]  tpos;
  logic [255:0] vchain, vp;

  // SM3 core
  logic         c_start, c_busy, c_done;
  logic [255:0] c_vout;
  logic [511:0] c_blk;
  logic [255:0] c_vin;
  integer b;
  always_comb for (b = 0; b < 64; b = b + 1) c_blk[511-8*b -: 8] = fill[b];
  assign c_vin = c_done ? c_vout : vchain;
  sm3_core2 u_core (.clk, .rst_n, .start(c_start), .vin(c_vin), .blk(c_blk), .abort,
                   .busy(c_busy), .done(c_done), .vout(c_vout));
  logic c_start_q;
  logic core_free;
  assign core_free = !c_busy && !c_start_q;

  // ---------------------------------------------------------------- buffer byte source
  logic        cur_buf;                 // 0 key, 1 ct (for the current segment)
  logic [12:0] waddr;                   // next word to request
  logic [63:0] wq [2];
  logic [1:0]  wv;                      // valid entries (wq[0] is head)
  logic        infl;                    // a read is in flight (data next cycle)
  logic [2:0]  bo;                      // byte offset in head word
  logic [15:0] words_left;              // words still to request for this segment
  logic        is_buf;
  assign is_buf = (st == S_SEG) && (src_q[seg] >= 2'd2);
  logic        want_req;
  assign want_req = is_buf && (words_left != 16'd0) &&
                    ({1'b0, wv[0]} + {1'b0, wv[1]} + {1'b0, infl} < 2'd2);
  logic        gnt;
  assign gnt = cur_buf ? cgnt : kgnt;
  always_comb begin
    kreq = '0; creq = '0;
    if (want_req) begin
      if (cur_buf) begin creq.req = 1'b1; creq.a = waddr; end
      else         begin kreq.req = 1'b1; kreq.a = waddr; end
    end
  end
  logic [7:0] buf_byte;
  assign buf_byte = wq[0][8*bo +: 8];
  logic [7:0] buf_byte2;
  logic [2:0] bo1;
  assign bo1 = bo + 3'd1;                     // used only when bo != 7
  assign buf_byte2 = wq[0][8*bo1 +: 8];
  logic [7:0] seg_byte, seg_byte2;
  logic       byte_avail, two_avail;
  logic [15:0] seg_left;
  assign seg_left = len_q[seg] - scnt;
  always_comb begin
    reg_idx = base_q[seg][7:0] + scnt[7:0];
    if (src_q[seg] == 2'd1) begin
      seg_byte = reg_byte; seg_byte2 = reg_byte2; byte_avail = 1'b1; two_avail = 1'b1;
    end else begin
      seg_byte = buf_byte; seg_byte2 = buf_byte2; byte_avail = wv[0]; two_avail = wv[0] && (bo != 3'd7);
    end
  end

  // fill-buffer room
  logic room, room2;
  assign room  = (pos != 7'd64);
  assign room2 = (pos <= 7'd62);
  logic [63:0] bitlen;
  assign bitlen = ({48'd0, mlen} + 64'd4) << 3;

  // emitted bytes this cycle (emit: at least one byte at pos; emit2: a second one at pos+1)
  logic       emit, emit2;
  logic [7:0] ebyte, ebyte2;
  logic       zfill, lenw;          // S_ZERO bulk zero fill / S_LEN 8-byte write
  logic [6:0] zlim;
  always_comb begin
    emit = 1'b0; emit2 = 1'b0; ebyte = 8'h00; ebyte2 = 8'h00; zfill = 1'b0; lenw = 1'b0;
    zlim = (pos <= 7'd56) ? 7'd56 : 7'd64;
    case (st)
      S_TAG:  begin emit = room; ebyte = tag_q; end
      S_SEG:  begin
        emit  = room && byte_avail && (src_q[seg] != 2'd0) && (len_q[seg] != 16'd0);
        emit2 = emit && room2 && two_avail && (seg_left >= 16'd2);
        ebyte = seg_byte; ebyte2 = seg_byte2;
      end
      S_CT:   begin
        emit  = room; emit2 = room2 && (bcnt <= 3'd2);
        ebyte = 8'h00; ebyte2 = (bcnt == 3'd2) ? {5'd0, oidx} + 8'd1 : 8'h00;
        if (bcnt == 3'd3) ebyte = {5'd0, oidx} + 8'd1;
      end
      S_P80:  begin emit = room; ebyte = 8'h80; end
      S_ZERO: zfill = room && (pos != 7'd56);
      S_LEN:  lenw = room;
      default: ;
    endcase
  end
  logic [6:0] nemit;
  assign nemit = zfill ? (zlim - pos) : lenw ? 7'd8 : emit2 ? 7'd2 : emit ? 7'd1 : 7'd0;

  // compress when the fill buffer is full and the core is free
  assign c_start = busy && (pos == 7'd64) && core_free;

  integer i;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE; busy <= 1'b0; out_valid <= 1'b0; out_idx <= '0; out_data <= '0;
      for (i = 0; i < 3; i = i + 1) begin src_q[i] <= '0; base_q[i] <= '0; len_q[i] <= '0; end
      nout_q <= '0; tag_q <= '0; mlen <= '0; seg <= '0; scnt <= '0; oidx <= '0; bcnt <= '0;
      for (i = 0; i < 64; i = i + 1) begin fill[i] <= '0; tail[i] <= '0; end
      pos <= '0; tpos <= '0; vchain <= IV; vp <= IV; c_start_q <= 1'b0;
      cur_buf <= 1'b0; waddr <= '0; wq[0] <= '0; wq[1] <= '0; wv <= '0; infl <= 1'b0; bo <= '0;
      words_left <= '0;
    end else begin
      out_valid <= 1'b0;
      c_start_q <= c_start;
      if (c_done) vchain <= c_vout;
      if (c_start) pos <= 7'd0;
      if (abort) begin
        st <= S_IDLE; busy <= 1'b0; wv <= '0; infl <= 1'b0;
      end else begin
        // buffer read pipeline
        if (want_req && gnt) begin waddr <= waddr + 13'd1; words_left <= words_left - 16'd1; end
        infl <= want_req && gnt;
        for (i = 0; i < 64; i = i + 1) begin
          if (zfill && (7'(i) >= pos) && (7'(i) < zlim))   fill[i] <= 8'h00;
          else if (lenw && i >= 56)                          fill[i] <= bitlen[63 - 8*(i-56) -: 8];
          else if (emit && 7'(i) == pos)                     fill[i] <= ebyte;
          else if (emit2 && 7'(i) == pos + 7'd1)             fill[i] <= ebyte2;
        end
        if (nemit != 7'd0) pos <= pos + nemit;
        // word queue: pop head when its last byte is used, push returning data
        begin : wqueue
          logic pop;
          logic [3:0] nbo;
          nbo = {1'b0, bo} + (emit2 ? 4'd2 : 4'd1);
          pop = (st == S_SEG) && emit && is_buf && (nbo == 4'd8 || seg_left == (emit2 ? 16'd2 : 16'd1));
          if (pop) bo <= 3'd0;
          else if ((st == S_SEG) && emit && is_buf) bo <= nbo[2:0];
          case ({pop, infl})
            2'b10: begin wq[0] <= wq[1]; wv <= {1'b0, wv[1]}; end
            2'b01: begin
              if (!wv[0]) begin wq[0] <= cur_buf ? cq : kq; wv[0] <= 1'b1; end
              else        begin wq[1] <= cur_buf ? cq : kq; wv[1] <= 1'b1; end
            end
            2'b11: begin
              if (wv[1]) begin wq[0] <= wq[1]; wq[1] <= cur_buf ? cq : kq; wv <= 2'b11; end
              else       begin wq[0] <= cur_buf ? cq : kq; wv <= 2'b01; end
            end
            default: ;
          endcase
        end
        case (st)
          S_IDLE: begin
            if (start) begin
              busy <= 1'b1; st <= S_TAG;
              for (i = 0; i < 3; i = i + 1) begin src_q[i] <= seg_src[i]; base_q[i] <= seg_base[i]; len_q[i] <= seg_len[i]; end
              nout_q <= nout; tag_q <= tag; oidx <= '0; pos <= '0; vchain <= IV;
              mlen <= 16'd1 + (seg_src[0] != 2'd0 ? seg_len[0] : 16'd0)
                            + (seg_src[1] != 2'd0 ? seg_len[1] : 16'd0)
                            + (seg_src[2] != 2'd0 ? seg_len[2] : 16'd0);
            end
          end
          S_TAG: if (emit) begin st <= S_SEG; seg <= 2'd0; scnt <= '0; end
          S_SEG: begin
            if (src_q[seg] == 2'd0 || len_q[seg] == 16'd0) begin
              if (seg == 2'd2) st <= S_ENDM; else seg <= seg + 2'd1;
              scnt <= '0;
            end else if (scnt == 16'd0 && !emit && wv == 2'b00 && !infl && words_left == 16'd0 && src_q[seg] >= 2'd2) begin
              // (re)arm the word fetcher for this buffer segment
              cur_buf <= (src_q[seg] == 2'd3);
              waddr <= base_q[seg][15:3];
              bo <= base_q[seg][2:0];
              words_left <= ({13'd0, base_q[seg][2:0]} + len_q[seg] + 16'd7) >> 3;
            end
            if (emit) begin
              scnt <= scnt + (emit2 ? 16'd2 : 16'd1);
              if (seg_left == (emit2 ? 16'd2 : 16'd1)) begin
                scnt <= '0;
                if (seg == 2'd2) st <= S_ENDM; else seg <= seg + 2'd1;
              end
            end
          end
          S_ENDM: begin
            // message complete: wait until every full block is compressed, then save state
            if (pos != 7'd64 && !c_busy && !c_start_q && !c_done) begin
              vp <= vchain; tpos <= pos;
              for (i = 0; i < 64; i = i + 1) tail[i] <= fill[i];
              st <= S_CT; bcnt <= '0;
            end
          end
          S_CT:   if (emit) begin
                    bcnt <= bcnt + (emit2 ? 3'd2 : 3'd1);
                    if (bcnt == 3'd3 || (emit2 && bcnt == 3'd2)) st <= S_P80;
                  end
          S_P80:  if (emit) st <= S_ZERO;
          S_ZERO: if (room && pos == 7'd56) begin st <= S_LEN; bcnt <= '0; end
          S_LEN:  if (lenw) st <= S_FIN;
          S_FIN: begin
            // the final block is full (pos == 64); wait for its compression to finish
            if (c_done && pos == 7'd0 && !c_start_q) begin
              out_valid <= 1'b1; out_idx <= oidx[1:0];
              for (i = 0; i < 32; i = i + 1) out_data[8*i +: 8] <= c_vout[255-8*i -: 8];
              st <= S_NEXT;
            end
          end
          S_NEXT: begin
            if (oidx + 3'd1 == nout_q) begin
              st <= S_IDLE; busy <= 1'b0;
            end else begin
              oidx <= oidx + 3'd1; vchain <= vp; pos <= tpos;
              for (i = 0; i < 64; i = i + 1) fill[i] <= tail[i];
              st <= S_CT; bcnt <= '0;
            end
          end
          default: st <= S_IDLE;
        endcase
      end
    end
  end
endmodule
