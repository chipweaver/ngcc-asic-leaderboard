// Bulk SM3 message assembler: builds 64-byte SM3 blocks (W0 at [511:480]) from the stream
//   pre (0 or 2 bytes {0x02,0x00}) || Z[0:zl] || MEM[0:mlen] || suffix (0, 2 {0x02,0x00} or 4 bytes BE counter)
// followed by standard SM3 padding with total length (64*pfx64 + msglen) * 8 bits.
// Memory words (64-bit, byte k at [8k+7:8k]) are fetched through a request/grant port with
// 1-cycle read latency. Pipelined: one 8-byte chunk per cycle (request chunk i while chunk
// i-1 is assembled from the returning data); a block is ready ~9 cycles after it is started.
module zen_hash_asm (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         cfg_start,        // 1-cycle: latch config, begin block 0
  input  logic [1:0]   cfg_pre,          // 0 or 2
  input  logic [6:0]   cfg_zl,           // 0, 16, 32, 64
  input  logic [12:0]  cfg_mlen,         // memory bytes
  input  logic [2:0]   cfg_suf,          // 0 none, 2 {02,00}, 4 counter
  input  logic [31:0]  cfg_ctr,
  input  logic         cfg_pfx64,
  input  logic [511:0] zreg,             // Z bytes, byte j at [8j+7:8j]
  // memory read port
  output logic         mreq,
  output logic [9:0]   maddr,
  input  logic         mgnt,
  input  logic [63:0]  mrdata,           // valid the cycle after mgnt
  input  logic [10:0]  mlimit,           // words readable: maddr < mlimit
  // block output
  output logic         blk_valid,
  output logic [511:0] blk,
  output logic         blk_last,
  input  logic         blk_take,         // core consumed blk (only when blk_valid)
  output logic         busy
);
  typedef enum logic [1:0] {S_IDLE, S_RUN, S_HOLD} st_t;
  st_t st;
  logic [1:0]  pre;
  logic [6:0]  zl;
  logic [12:0] mlen, msglen;
  logic [2:0]  suf;
  logic [31:0] ctr;
  logic        pfx64;
  logic [5:0]  nb;            // blocks
  logic [5:0]  bi;            // current block
  logic [3:0]  rq;            // next chunk to request (0..8)
  logic        pv;            // a chunk is pending assembly this cycle
  logic [2:0]  pc;            // pending chunk index
  logic        prd;           // pending chunk read memory
  logic [63:0] prevw;
  logic [511:0] buf_le;       // block bytes, byte j at [8j+7:8j]
  logic [63:0] len64;

  assign busy = (st != S_IDLE);

  // request side: chunk rq of block bi
  logic [12:0] rbase;
  logic [9:0]  wa;
  logic        need_rd, rq_ok, rq_mv;
  always_comb begin
    rbase   = {bi, 6'd0} + {6'd0, rq[2:0], 3'd0};
    wa      = (rbase[12:3] - {6'd0, zl[6:3]});
    need_rd = (rbase >= {6'd0, zl}) && ({wa, 3'd0} < mlen);
  end
  assign maddr = wa;
  assign mreq  = (st == S_RUN) && !rq[3] && need_rd && ({1'b0, wa} < mlimit);
  assign rq_mv = (st == S_RUN) && !rq[3] && (!need_rd || (mreq && mgnt));

  // assembly side: chunk pc of block bi, memory data (if prd) on mrdata
  logic [12:0] base;
  logic [13:0] mstart, mend;
  always_comb begin
    base   = {bi, 6'd0} + {6'd0, pc, 3'd0};
    mstart = {12'd0, pre} + {7'd0, zl};
    mend   = mstart + {1'b0, mlen};
  end

  logic [63:0] chunk;
  always_comb begin
    logic [13:0] idx;
    logic [13:0] off;
    logic [7:0]  b;
    for (int k = 0; k < 8; k++) begin
      idx = {1'b0, base} + k[13:0];
      if (idx < {12'd0, pre}) begin
        b = (idx[0] == 1'b0) ? 8'h02 : 8'h00;
      end else if (idx < mstart) begin
        b = zreg[8*(idx - {12'd0, pre}) +: 8];
      end else if (idx < mend) begin
        off = idx - mstart;
        if (pre == 2'd2 && k < 2) b = prevw[8*((k < 2) ? (k+6) : 0) +: 8];   // index clamp: branch only reached for k<2
        else if (pre == 2'd2)     b = mrdata[8*((k >= 2) ? (k-2) : 0) +: 8]; // index clamp: branch only reached for k>=2
        else                      b = mrdata[8*k +: 8];
      end else if (idx < {1'b0, msglen}) begin
        off = idx - mend;
        if (suf == 3'd2) b = (off[0] == 1'b0) ? 8'h02 : 8'h00;
        else begin
          case (off[1:0])
            2'd0: b = ctr[31:24];
            2'd1: b = ctr[23:16];
            2'd2: b = ctr[15:8];
            default: b = ctr[7:0];
          endcase
        end
      end else if (idx == {1'b0, msglen}) begin
        b = 8'h80;
      end else if ((bi == nb - 6'd1) && (pc == 3'd7)) begin
        b = len64[8*(7-k) +: 8];
      end else begin
        b = 8'h00;
      end
      chunk[8*k +: 8] = b;
    end
  end

  function automatic logic [511:0] le2blk(input logic [511:0] le);
    for (int j = 0; j < 64; j++) le2blk[511 - 8*j -: 8] = le[8*j +: 8];
  endfunction
  assign blk = le2blk(buf_le);
  assign blk_valid = (st == S_HOLD);
  assign blk_last  = (bi == nb - 6'd1);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE; bi <= '0; rq <= '0; pv <= 1'b0; pc <= '0; prd <= 1'b0; nb <= '0;
    end else begin
      pv <= rq_mv; pc <= rq[2:0]; prd <= rq_mv && need_rd;
      case (st)
        S_IDLE: if (cfg_start) begin
          st <= S_RUN; bi <= '0; rq <= '0;
          nb <= ((({1'b0, cfg_pre} + {6'd0, cfg_zl} + cfg_mlen + {10'd0, cfg_suf}) + 13'd9 + 13'd63) >> 6);
        end
        S_RUN: begin
          if (rq_mv) rq <= rq + 4'd1;
          if (pv && pc == 3'd7) st <= S_HOLD;
        end
        S_HOLD: if (blk_take) begin
          if (bi == nb - 6'd1) st <= S_IDLE;
          else begin st <= S_RUN; bi <= bi + 6'd1; rq <= '0; end
        end
        default: st <= S_IDLE;
      endcase
    end
  end

  always_ff @(posedge clk) begin
    if (cfg_start && st == S_IDLE) begin
      pre <= cfg_pre; zl <= cfg_zl; mlen <= cfg_mlen; suf <= cfg_suf; ctr <= cfg_ctr; pfx64 <= cfg_pfx64;
      msglen <= {11'd0, cfg_pre} + {6'd0, cfg_zl} + cfg_mlen + {10'd0, cfg_suf};
      len64  <= {35'd0, ({11'd0, cfg_pre} + {6'd0, cfg_zl} + cfg_mlen + {10'd0, cfg_suf} + (cfg_pfx64 ? 13'd64 : 13'd0)), 3'd0};
    end
    if (pv) begin
      buf_le[64*pc +: 64] <= chunk;
      if (prd) prevw <= mrdata;
    end
  end
endmodule
