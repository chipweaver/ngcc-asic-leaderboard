// Output stage behind the multiplier.
//   OS_KG   : s_k = (y*h)_k ^ x_k (x bitmap via port R) -> key buffer bytes [S, PK)
//   OS_ENC  : u_k = (h*r2)_k ^ r1_k (port R) -> CT bytes [0, ceil(n/8));
//             v_k = (s*r2)_k ^ e_k (port E) ^ RMenc(RS codeword)_k, first L1 bits ->
//             KR compression (51 -> 41 per block, Q blocks) + raw tail v2 -> payload packed
//             LSB-first at CT byte ceil(n/8)
//   OS_DECC : as OS_ENC but every CT byte is read and compared (sticky mismatch)
//   OS_DECRM: x_k = (u*y)_k ^ v_k with v from the KR decompressor (payload read from CT);
//             the first n1*M 128-bit chunks go to the RM decoder.
// Results are taken word group by word group from the multiplier's output buffer (r_valid head,
// r_pop); the multiplier holds back the last term of its next group until the buffer is drained, so
// no result can be lost. In OS_DECRM a chunk is presented to the RM decoder only when rm_rdy.
module ostage import hare_pkg::*; (
  input  logic         clk,
  input  logic         rst_n,
  input  logic [1:0]   lvl,
  input  logic [2:0]   mode,
  input  logic         start,
  // multiplier results
  input  logic         r_valid,
  input  logic [9:0]   r_ka,
  input  logic [255:0] r_a,
  input  logic         r_vb,
  input  logic [9:0]   r_kb,
  input  logic [255:0] r_b,
  output logic         r_pop,
  // bitmap read ports
  output dm_req_t      bmr,
  input  dm_q_t        bmr_q,
  output dm_req_t      bme,
  input  dm_q_t        bme_q,
  // object buffers
  output ob_req_t      creq,
  input  logic         cgnt,
  input  logic [63:0]  cq,
  output ob_req_t      kreq,
  input  logic         kgnt,
  // RS codeword
  output logic [7:0]   cw_idx,
  input  logic [7:0]   cw_byte,
  output logic [7:0]   cw_idx2,
  input  logic [7:0]   cw_byte2,
  // RM decoder chunks
  output logic         ch_valid,
  output logic [127:0] ch_data,
  input  logic         rm_rdy,
  output logic         done,
  output logic         mismatch,
  // phase trace, asserted only in working cycles: [0] u/s word write-back (KG/ENC) or compare
  // (DECC), [1] KR compression + payload packing / payload compare, [2] RM encoding of the
  // codeword chunks, [3] KR decompression (a block or the raw tail decoded)
  output logic [3:0]   act
);
  localparam logic [2:0] OS_KG = 3'd1, OS_ENC = 3'd2, OS_DECC = 3'd3, OS_DECRM = 3'd4;

  logic [9:0]  nw, nwd;  logic [7:0] nr;  logic [6:0] S;
  logic [14:0] nbytes, paybytes, PK;  logic [11:0] Q;  logic [5:0] NV2;  logic [17:0] L1;
  logic [3:0]  M;   logic [10:0] nchunk;
  assign nw = p_nw(lvl); assign nwd = p_nwd(lvl); assign nr = p_nr(lvl);
  assign S = p_s(lvl); assign nbytes = p_nbytes(lvl); assign paybytes = p_paybytes(lvl);
  assign PK = p_pk(lvl); assign Q = p_q(lvl); assign NV2 = p_nv2(lvl); assign L1 = p_l1(lvl);
  assign M = p_m(lvl); assign nchunk = p_nchunk(lvl);
  logic [15:0] payend;
  assign payend = {1'b0, nbytes} + {1'b0, paybytes};


  // ------------------------------------------------------------------ main FSM
  typedef enum logic [3:0] {O_IDLE, O_WAIT, O_RD, O_CALC, O_U, O_KR, O_TAIL, O_FLUSH, O_DONE,
                            O_KW, O_RW, O_RW2} st_t;
  st_t st;
  logic         lane;          // 0: lane A result, 1: lane B
  logic [9:0]   k;             // current output word index
  logic [255:0] res, res2;     // current results (A / B)
  logic [9:0]   kb_q;
  logic         vb_q;
  logic [255:0] uw, vw;        // computed words
  logic [1:0]   j;             // sub-word index
  logic         vdone;         // v word of the current pair pushed (first O_U cycle done)
  logic [9:0]   groups_done;
  logic [9:0]   ngroups;
  logic [17:0]  vbits;         // v bits pushed into the KR FIFO
  logic [11:0]  blocks;        // KR blocks compressed
  logic         tail_done;
  // KR FIFO and packer
  logic [306:0] kf;  logic [8:0] kf_cnt;
  logic [127:0] pk;  logic [7:0] pk_cnt;  logic [12:0] pw;
  // chunk -> symbol tracking for RM encoding
  logic [7:0]   sym;  logic [3:0] cis;
  // DECC compare pipeline
  logic         cmp_pend;  logic [63:0] cmp_exp;  logic [7:0] cmp_be;

  always_comb begin
    case (mode)
      OS_ENC, OS_DECC: ngroups = nw;
      OS_KG:           ngroups = (nw + 10'd1) >> 1;
      default:         ngroups = (nwd + 10'd1) >> 1;
    endcase
  end

  // RM encoding of the two chunks of word k
  logic [7:0] sym1;    // symbol of the second chunk of word k (registered with sym/cis)
  assign cw_idx  = sym;
  assign cw_idx2 = sym1;
  logic [255:0] rmw, rmw_q;      // RM encoding registered in O_RD (sym/cis stable until O_U)
  logic [127:0] rm_cw0, rm_cw1;
  rm_encoder u_rme0 (.sym(cw_byte),  .cw(rm_cw0));
  rm_encoder u_rme1 (.sym(cw_byte2), .cw(rm_cw1));
  assign rmw = {rm_cw1, rm_cw0};
  // KR compression of the oldest 51 FIFO bits / decompression of the oldest 41 payload bits
  logic [40:0] krc_msg;
  logic [50:0] krd_blk;
  logic [306:0] kf_w;
  logic [127:0] pf_w;
  kr_comp   u_krc (.blk(kf_w[50:0]), .msg(krc_msg));
  kr_decomp u_krd (.msg(pf_w[40:0]), .blk(krd_blk));

  // masks
  logic [255:0] nmask, l1mask;
  always_comb begin
    nmask  = (k == nw - 10'd1) ? (~(256'd0) >> (9'd256 - {1'b0, nr})) : ~(256'd0);
    l1mask = ((L1 - vbits) >= 18'd256) ? ~(256'd0) : (~(256'd0) >> (9'd256 - (L1 - vbits)));
  end
  logic [8:0] vpush;
  assign vpush = ((L1 - vbits) >= 18'd256) ? 9'd256 : (L1 - vbits);

  // bitmap read requests
  always_comb begin
    bmr = '0; bme = '0;
    if (st == O_RD) begin
      bmr.ce[k[2:0]] = 1'b1; bmr.a[k[2:0]] = k[9:3];
      bme.ce[k[2:0]] = 1'b1; bme.a[k[2:0]] = k[9:3];
    end
  end
  logic [255:0] bmr_w, bme_w;
  assign bmr_w = bmr_q[k[2:0]];
  assign bme_w = bme_q[k[2:0]];

  // ------------------------------------------------------------------ KR decompressor (OS_DECRM)
  ob_req_t      d_creq;
  logic         busy_d;
  logic [12:0]  d_addr;
  logic         d_infl, d_first;
  logic [127:0] pf;  logic [7:0] pf_cnt;
  logic [11:0]  d_blocks;
  logic         d_tail, d_padded;
  logic [306:0] vf;  logic [8:0] vf_cnt;
  logic [255:0] vq [2];  logic [1:0] vq_cnt;  logic vq_rp, vq_wp;
  logic [15:0]  d_words_left;
  logic         d_pop;            // consumer pops a word
  logic [5:0]   off0;
  assign off0 = {nbytes[2:0], 3'b000};
  always_comb begin
    d_creq = '0;
    d_creq.req = (mode == OS_DECRM) && busy_d && !d_infl && (pf_cnt <= 8'd64) && (d_words_left != 16'd0);
    d_creq.a   = d_addr;
  end

  // buffer transactions
  logic [12:0] uwi;   // u/s word address
  logic [14:0] ub;    // first byte of that word
  logic [7:0]  ube;
  logic [7:0]  pbe;
  logic        u_has;
  logic [15:0] pwb;
  always_comb begin
    if (mode == OS_KG) uwi = {6'd0, S[6:3]} + {1'b0, k, 2'b00} + {11'd0, j};
    else               uwi = {1'b0, k, 2'b00} + {11'd0, j};
    ub = {uwi[11:0], 3'b000};
    for (int b = 0; b < 8; b = b + 1) begin
      if (mode == OS_KG) ube[b] = ({1'b0, ub} + b) < {1'b0, PK} && ({1'b0, ub} + b) >= {9'd0, S};
      else               ube[b] = ({1'b0, ub} + b) < {1'b0, nbytes};
      pwb = {pw, 3'b000} + b;
      pbe[b] = (pwb >= {1'b0, nbytes}) && (pwb < payend);
    end
    u_has = (ube != 8'd0);
  end

  logic decc;
  assign decc = (mode == OS_DECC);
  // O_KR: a granted 64-bit packer write and one 51 -> 41 block compression in the same cycle
  logic         kr_wr, kr_add, kr_fin;
  logic [7:0]   kr_pc1, kr_pc2;
  logic [127:0] kr_pk1, kr_pk2;
  logic [8:0]   kr_kc2;
  logic [11:0]  kr_bl2;
  always_comb begin
    kr_wr  = (st == O_KR) && (pk_cnt >= 8'd64) && cgnt;
    kr_pc1 = kr_wr ? pk_cnt - 8'd64 : pk_cnt;
    kr_pk1 = kr_wr ? (pk >> 64) : pk;
    kr_add = (st == O_KR) && (blocks != Q) && (kf_cnt >= 9'd51) && (kr_pc1 <= 8'd87);
    kr_pk2 = kr_add ? (kr_pk1 | ({87'd0, krc_msg} << kr_pc1)) : kr_pk1;
    kr_pc2 = kr_add ? kr_pc1 + 8'd41 : kr_pc1;
    kr_kc2 = kr_add ? kf_cnt - 9'd51 : kf_cnt;
    kr_bl2 = kr_add ? blocks + 12'd1 : blocks;
    kr_fin = !((kr_bl2 != Q) && (kr_kc2 >= 9'd51)) && (kr_pc2 < 8'd64);
  end
  // OS_DECRM: chunk hand-off to the RM decoder
  logic c0_need, c1_need, rw_go;
  assign c0_need = ({k, 1'b0} < {1'b0, nchunk});
  assign c1_need = ({k, 1'b1} < {1'b0, nchunk});
  assign rw_go   = (st == O_RW) && (vq_cnt != 2'd0) && (rm_rdy || !c0_need);
  always_comb begin
    creq = '0; kreq = '0;
    case (st)
      O_KW: begin
        kreq.req = u_has; kreq.we = 1'b1; kreq.a = uwi; kreq.d = uw[64*j +: 64]; kreq.be = ube;
      end
      O_U: begin
        creq.req = u_has; creq.we = !decc; creq.a = uwi; creq.d = uw[64*j +: 64]; creq.be = ube;
      end
      O_KR, O_TAIL, O_FLUSH: if (((st == O_KR || st == O_TAIL) && pk_cnt >= 8'd64) || (st == O_FLUSH && pk_cnt != 8'd0)) begin
        creq.req = 1'b1; creq.we = !decc; creq.a = pw; creq.d = pk[63:0]; creq.be = pbe;
      end
      default: ;
    endcase
    if (mode == OS_DECRM) creq = d_creq;
  end

  // ------------------------------------------------------------------ sequential
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < 2; i = i + 1) vq[i] <= '0;
      st <= O_IDLE; lane <= 1'b0; k <= '0; rmw_q <= '0; res <= '0; res2 <= '0; kb_q <= '0; vb_q <= 1'b0; uw <= '0; vw <= '0; j <= '0;
      groups_done <= '0; vbits <= '0; blocks <= '0; tail_done <= 1'b0; vdone <= 1'b0;
      kf <= '0; kf_cnt <= '0; pk <= '0; pk_cnt <= '0; pw <= '0; sym <= '0; sym1 <= '0; cis <= '0;
      cmp_pend <= 1'b0; cmp_exp <= '0; cmp_be <= '0;
      done <= 1'b0; mismatch <= 1'b0; ch_valid <= 1'b0; ch_data <= '0;
      d_addr <= '0; d_infl <= 1'b0; d_first <= 1'b0; pf <= '0; pf_cnt <= '0; d_blocks <= '0;
      d_tail <= 1'b0; d_padded <= 1'b0; vf <= '0; vf_cnt <= '0; vq_cnt <= '0;
      vq_rp <= 1'b0; vq_wp <= 1'b0; d_words_left <= '0; busy_d <= 1'b0;
    end else begin
      done <= 1'b0; ch_valid <= 1'b0;
      // ---------------- DECC compare pipeline
      if (cmp_pend) begin
        for (int b = 0; b < 8; b = b + 1)
          if (cmp_be[b] && (cq[8*b +: 8] != cmp_exp[8*b +: 8])) mismatch <= 1'b1;
      end
      cmp_pend <= 1'b0;
      if (decc && creq.req && cgnt) begin cmp_pend <= 1'b1; cmp_exp <= creq.d; cmp_be <= creq.be; end

      // ---------------- main FSM
      case (st)
        O_IDLE: if (start) begin
          st <= O_WAIT; groups_done <= '0; vbits <= '0; blocks <= '0; tail_done <= 1'b0;
          kf <= '0; kf_cnt <= '0; pk <= '0; pk_cnt <= {2'b00, off0}; pw <= nbytes[14:3];
          sym <= '0; sym1 <= '0; cis <= '0; mismatch <= 1'b0;
          // decompressor
          busy_d <= (mode == OS_DECRM); d_addr <= nbytes[14:3]; d_infl <= 1'b0; d_first <= 1'b1;
          pf <= '0; pf_cnt <= '0; d_blocks <= '0; d_tail <= 1'b0; d_padded <= 1'b0;
          vf <= '0; vf_cnt <= '0; vq_cnt <= '0; vq_rp <= 1'b0; vq_wp <= 1'b0;
          d_words_left <= 16'(({14'd0, off0} + {8'd0, Q} * 20'd41 + {14'd0, NV2} + 20'd63) >> 6);
        end
        O_WAIT: begin
          if (groups_done == ngroups) begin
            if (mode == OS_ENC || mode == OS_DECC) st <= O_TAIL;
            else st <= O_DONE;
          end else if (r_valid) begin
            res <= r_a; res2 <= r_b; k <= r_ka; kb_q <= r_kb; vb_q <= r_vb;
            lane <= 1'b0;
            st <= (mode == OS_DECRM) ? O_RW : O_RD;
          end
        end
        O_RD: begin rmw_q <= rmw; st <= O_CALC; end
        O_CALC: begin
          uw <= (res ^ bmr_w) & nmask;
          vw <= (res2 ^ bme_w ^ rmw_q) & l1mask;
          j <= '0; vdone <= 1'b0;
          st <= (mode == OS_KG) ? O_KW : O_U;
        end
        O_KW: if (!u_has || kgnt) begin
          j <= j + 2'd1;
          if (j == 2'd3) begin
            if (lane == 1'b0 && vb_q) begin
              lane <= 1'b1; k <= kb_q; res <= res2; st <= O_RD;
            end else begin
              groups_done <= groups_done + 10'd1; st <= O_WAIT;
            end
          end
        end
        O_U: begin
          // first cycle: v word into the KR FIFO and advance the RM symbol tracking
          if (j == 2'd0 && !vdone) begin
            vdone <= 1'b1;
            if (vbits != L1) begin
              kf <= kf | ({51'd0, vw} << kf_cnt);
              kf_cnt <= kf_cnt + vpush;
              vbits <= vbits + {9'd0, vpush};
            end
            if (cis == M - 4'd1)      begin cis <= 4'd1; sym <= sym + 8'd1; sym1 <= sym + 8'd1; end
            else if (cis == M - 4'd2) begin cis <= 4'd0; sym <= sym + 8'd1; sym1 <= sym + 8'd1; end
            else begin
              cis <= cis + 4'd2;
              sym1 <= (cis + 4'd2 == M - 4'd1) ? sym + 8'd1 : sym;
            end
          end
          if (!u_has || cgnt) begin
            j <= j + 2'd1;
            if (j == 2'd3) st <= O_KR;
          end
        end
        O_KR: begin
          pk <= kr_pk2; pk_cnt <= kr_pc2;
          if (kr_wr) pw <= pw + 13'd1;
          if (kr_add) begin kf <= kf >> 51; kf_cnt <= kr_kc2; blocks <= kr_bl2; end
          if (kr_fin) begin groups_done <= groups_done + 10'd1; st <= O_WAIT; end
        end
        O_TAIL: begin
          if (pk_cnt >= 8'd64) begin
            if (cgnt) begin pk <= pk >> 64; pk_cnt <= pk_cnt - 8'd64; pw <= pw + 13'd1; end
          end else if (!tail_done) begin
            pk <= pk | (({64'd0, kf[63:0]} & ((128'd1 << NV2) - 128'd1)) << pk_cnt);
            pk_cnt <= pk_cnt + {2'b00, NV2};
            tail_done <= 1'b1;
          end else begin
            pk_cnt <= (pk_cnt + 8'd7) & 8'hF8;   // pad to a byte (bits above pk_cnt are zero)
            st <= O_FLUSH;
          end
        end
        O_FLUSH: begin
          if (pk_cnt == 8'd0) st <= O_DONE;
          else if (cgnt) begin
            pk <= pk >> 64; pw <= pw + 13'd1;
            pk_cnt <= (pk_cnt >= 8'd64) ? pk_cnt - 8'd64 : 8'd0;
          end
        end
        O_DONE: begin
          if (!cmp_pend) begin done <= 1'b1; st <= O_IDLE; busy_d <= 1'b0; end
        end
        // OS_DECRM: combine each lane result with one decompressed v word, two chunks
        O_RW: begin
          if (rw_go) begin
            uw <= res ^ vq[vq_rp];
            if (c0_need) begin ch_valid <= 1'b1; ch_data <= res[127:0] ^ vq[vq_rp][127:0]; end
            st <= O_RW2;
          end
        end
        O_RW2: if (rm_rdy || !c1_need) begin
          if (c1_need) begin ch_valid <= 1'b1; ch_data <= uw[255:128]; end
          if (lane == 1'b0 && vb_q) begin
            lane <= 1'b1; k <= kb_q; res <= res2; st <= O_RW;
          end else begin
            groups_done <= groups_done + 10'd1; st <= O_WAIT;
          end
        end
        default: st <= O_IDLE;
      endcase

      // ---------------- decompressor datapath
      if (busy_d) begin : dcmp
        logic [127:0] npf;  logic [7:0] npc;
        logic [306:0] nvf;  logic [8:0] nvc;
        npf = pf; npc = pf_cnt; nvf = vf; nvc = vf_cnt;
        d_infl <= d_creq.req && cgnt;
        if (d_creq.req && cgnt) begin d_addr <= d_addr + 13'd1; d_words_left <= d_words_left - 16'd1; end
        if (d_infl) begin
          if (d_first) begin npf = npf | ({64'd0, cq >> off0} << npc); npc = npc + 8'd64 - {2'b00, off0}; end
          else         begin npf = npf | ({64'd0, cq} << npc);         npc = npc + 8'd64; end
          d_first <= 1'b0;
        end
        // one block (or the raw tail) per cycle into the v FIFO
        if (nvc <= 9'd255) begin
          if (d_blocks != Q && pf_cnt >= 8'd41) begin
            nvf = nvf | ({256'd0, krd_blk} << nvc); nvc = nvc + 9'd51;
            npf = npf >> 41; npc = npc - 8'd41;
            d_blocks <= d_blocks + 12'd1;
          end else if (d_blocks == Q && !d_tail && pf_cnt >= {2'b00, NV2}) begin
            nvf = nvf | (({243'd0, pf[63:0]} & ((307'd1 << NV2) - 307'd1)) << nvc); nvc = nvc + {3'd0, NV2};
            npf = npf >> NV2; npc = npc - {2'b00, NV2};
            d_tail <= 1'b1;
          end else if (d_tail && !d_padded && nvc != 9'd0) begin
            nvc = 9'd256; d_padded <= 1'b1;   // zero-pad the last word (bits above nvc are zero)
          end
        end
        // output queue
        begin
          logic push_w;
          push_w = (vf_cnt >= 9'd256) && (vq_cnt != 2'd2 || d_pop);
          if (push_w) begin
            vq[vq_wp] <= vf[255:0]; vq_wp <= ~vq_wp;
            nvf = nvf >> 256; nvc = nvc - 9'd256;
          end
          if (d_pop) vq_rp <= ~vq_rp;
          vq_cnt <= vq_cnt + (push_w ? 2'd1 : 2'd0) - (d_pop ? 2'd1 : 2'd0);
        end
        pf <= npf; pf_cnt <= npc; vf <= nvf; vf_cnt <= nvc;
      end
    end
  end
  assign d_pop = (mode == OS_DECRM) && rw_go;
  assign r_pop = (st == O_WAIT) && (groups_done != ngroups) && r_valid;
  assign act[0] = (st == O_KW) || (st == O_U);
  assign act[1] = ((st == O_U) && (j == 2'd0) && !vdone && (vbits != L1)) || kr_wr || kr_add ||
                  (st == O_TAIL) || (st == O_FLUSH);
  assign act[2] = (st == O_RD) && (mode == OS_ENC || mode == OS_DECC) && (vbits != L1);
  assign act[3] = busy_d && (vf_cnt <= 9'd255) &&
                  ((d_blocks != Q && pf_cnt >= 8'd41) || (d_blocks == Q && !d_tail && pf_cnt >= {2'b00, NV2}));
  assign kf_w = kf;
  assign pf_w = pf;
endmodule
