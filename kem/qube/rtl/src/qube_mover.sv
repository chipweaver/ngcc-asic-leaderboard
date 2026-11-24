// Object-buffer mover. One operation at a time:
//   LOAD    : n-bit vector at byte offset soff of buffer sbuf -> NW words of the selected lanes / slot mask
//   EXTRACT : 64 bytes at soff of sbuf -> ext_data (bytes >= k cleared), ext_we pulse
//   EMFEED  : v (CT at nb) xor ACC1 (u*y), NWv words -> RM decoder stream
//   CMP     : CT u-part vs ACC0 and CT v-part vs trunc(ACC1) xor RM(RS(m)), raw bytes -> cmp_eq
//   PACK_KG : pk = rho0||s1||s2 -> PK, sk = rho1||rho0||s1||s2||rho_k -> SK (s1 = ACC0, s2 = ACC1)
//   PACK_CT : ct = u||v||sigma -> CT, u = ACC0, v = trunc(ACC1) xor RM(RS(m))
// Buffer reads are realigned by the byte offset (out_j = bytes [soff + 64j, soff + 64j + 64)).
module qube_mover
  import qube_pkg::*;
(
  input  logic          clk,
  input  logic          rst_n,
  input  logic [2:0]    ps,
  input  logic          start,
  input  logic [2:0]    op,
  input  logic [1:0]    sbuf,
  input  logic [15:0]   soff,
  input  logic [1:0]    dlanes,
  input  logic [1:0]    dslot,        // slot mask of LOAD
  output logic          busy,
  output logic          done,
  output logic          cmp_eq,
  output logic          ext_we,
  output logic [511:0]  ext_data,
  // registers for packing
  input  logic [511:0]  r_rho0,
  input  logic [511:0]  r_rho1,
  input  logic [511:0]  r_rhok,
  input  logic [511:0]  r_sigma,
  input  logic [7:0]    cdw [0:N1MAX-1],
  // object buffer reads (mover is reader 0 of each buffer)
  output logic [2:0]    r_rq,
  output logic [9:0]    r_addr,
  input  logic [2:0]    r_gnt,
  input  logic [2:0]    r_rv,
  input  logic [511:0]  r_data [0:2],
  // object buffer writes
  output logic [2:0]    w_rq,
  output logic [9:0]    w_addr [0:2],
  output logic [511:0]  w_data [0:2],
  // lane ACC reads (data in the next cycle)
  output logic [1:0]    ar_en,
  output logic [8:0]    ar_addr,
  input  logic [511:0]  ar_data [0:1],
  // lane slot writes
  output logic [1:0]    sw_en,
  output logic [1:0]    sw_slot,
  output logic [8:0]    sw_addr,
  output logic [511:0]  sw_data,
  // decoder stream
  output logic          em_valid,
  input  logic          em_ready,
  output logic [511:0]  em_data
);
  localparam logic [2:0] OP_LOAD = 3'd0, OP_EXTRACT = 3'd1, OP_EMFEED = 3'd2, OP_CMP = 3'd3,
                         OP_PACK_KG = 3'd4, OP_PACK_CT = 3'd5;

  function automatic logic [127:0] rm_enc(input logic [7:0] m);
    logic [31:0] fw;
    logic [127:0] r;
    fw = {32{m[7]}};
    fw = fw ^ ({32{m[0]}} & 32'hAAAAAAAA);
    fw = fw ^ ({32{m[1]}} & 32'hCCCCCCCC);
    fw = fw ^ ({32{m[2]}} & 32'hF0F0F0F0);
    fw = fw ^ ({32{m[3]}} & 32'hFF00FF00);
    fw = fw ^ ({32{m[4]}} & 32'hFFFF0000);
    r[31:0] = fw;
    fw = fw ^ {32{m[5]}};
    r[63:32] = fw;
    fw = fw ^ {32{m[6]}};
    r[127:96] = fw;
    fw = fw ^ {32{m[5]}};
    r[95:64] = fw;
    rm_enc = r;
  endfunction

  function automatic logic [511:0] bytemask(input logic [6:0] nbytes);  // nbytes 0..64
    for (int i = 0; i < 64; i++) bytemask[8 * i +: 8] = (7'(i) < nbytes) ? 8'hFF : 8'h00;
  endfunction

  // ---------------- latched parameters
  logic [2:0]  op_r;
  logic [1:0]  buf_r;
  logic [1:0]  lanes_r;
  logic [1:0]  slot_r;
  logic [6:0]  k_r;
  logic [8:0]  nw_r, nwv_r, lastbits_r;
  logic [14:0] nb_r, nv_r;
  logic [7:0]  n1_r;
  logic [2:0]  mult_r;
  logic        run;
  logic        cph;        // CMP phase: 0 u, 1 v
  logic        eq_r;

  // ---------------- read stream
  logic        rs_on;
  logic [9:0]  rs_w;       // next word address to request
  logic [9:0]  rs_left;    // words left to request
  logic [5:0]  rs_bs;
  logic [2:0]  rs_out;     // requests in flight
  logic [511:0] fq [0:3];
  logic [1:0]  fq_rd, fq_wr;
  logic [2:0]  fq_n;
  logic        rs_pv;
  logic [511:0] rs_prev;
  logic        rs_get;     // pop the FIFO head this cycle
  logic        rs_avail;   // realigned word available at the head
  logic [511:0] rs_word;
  logic [2:0]  rgnt_any;
  logic        req_ok;
  always_comb begin
    req_ok = rs_on && (rs_left != 10'd0) && ({1'b0, rs_out} + {1'b0, fq_n} < 4'd4);
    r_rq = req_ok ? (3'b001 << buf_r) : 3'b000;
    r_addr = rs_w;
  end
  logic got;
  assign got = |(r_rv);
  logic [1023:0] rfun;
  always_comb begin
    rfun = {fq[fq_rd], rs_prev} >> {rs_bs, 3'd0};
    rs_avail = (fq_n != 3'd0) && ((rs_bs == 6'd0) || rs_pv);
    rs_word = (rs_bs == 6'd0) ? fq[fq_rd] : rfun[511:0];
  end

  // ---------------- item stage A
  logic        a_v;
  logic [511:0] a_w;           // realigned buffer word or register chunk
  logic [8:0]  a_j;
  logic [1:0]  a_kind;         // PACK: 0 reg chunk, 1 ACC0, 2 ACC1 (+cw)
  logic [6:0]  a_len;          // PACK chunk bytes
  logic        a_skonly;
  logic        consume, issue;

  // output word counter for streamed ops
  logic [8:0]  oj, onum;

  // PACK item sequencer
  logic [2:0]  pk_seg;          // KG: 0 rho1, 1 rho0, 2 ACC0, 3 ACC1, 4 rho_k ; CT: 2 ACC0, 3 ACC1, 5 sigma
  logic [8:0]  pk_j;
  logic        pk_on;

  // cw generator state (chunk index of word a_j's first chunk)
  logic [7:0]  cw_sym;
  logic [2:0]  cw_cp;
  logic [511:0] cw_word;
  always_comb begin
    logic [7:0] sy; logic [2:0] cp;
    sy = cw_sym; cp = cw_cp;
    for (int t = 0; t < 4; t++) begin
      cw_word[128 * t +: 128] = (sy < n1_r) ? rm_enc(cdw[sy]) : 128'd0;
      if (cp == mult_r - 3'd1) begin cp = 3'd0; sy = sy + 8'd1; end else cp = cp + 3'd1;
    end
  end
  logic [7:0] cw_sym_n; logic [2:0] cw_cp_n;
  always_comb begin
    logic [7:0] sy; logic [2:0] cp;
    sy = cw_sym; cp = cw_cp;
    for (int t = 0; t < 4; t++) begin
      if (cp == mult_r - 3'd1) begin cp = 3'd0; sy = sy + 8'd1; end else cp = cp + 3'd1;
    end
    cw_sym_n = sy; cw_cp_n = cp;
  end

  // source of the next item
  logic        src_v;
  logic [511:0] src_w;
  logic [1:0]  src_kind;
  logic [6:0]  src_len;
  logic        src_skonly;
  logic        need_acc;
  always_comb begin
    src_v = 1'b0; src_w = rs_word; src_kind = 2'd0; src_len = 7'd64; src_skonly = 1'b0; need_acc = 1'b0;
    if (op_r == OP_PACK_KG || op_r == OP_PACK_CT) begin
      src_v = pk_on;
      case (pk_seg)
        3'd0: begin src_w = r_rho1; src_len = k_r; src_skonly = 1'b1; end
        3'd1: begin src_w = r_rho0; src_len = k_r; end
        3'd2: begin src_kind = 2'd1; need_acc = 1'b1;
                    src_len = (pk_j == nw_r - 9'd1) ? 7'(nb_r - {pk_j[8:0], 6'd0}) : 7'd64; end
        3'd3: begin src_kind = 2'd2; need_acc = 1'b1;
                    if (op_r == OP_PACK_KG) src_len = (pk_j == nw_r - 9'd1) ? 7'(nb_r - {pk_j[8:0], 6'd0}) : 7'd64;
                    else src_len = (pk_j == nwv_r - 9'd1) ? 7'(nv_r - {pk_j[8:0], 6'd0}) : 7'd64; end
        3'd4: begin src_w = r_rhok; src_len = k_r; src_skonly = 1'b1; end
        default: begin src_w = r_sigma; src_len = k_r; end
      endcase
    end else begin
      src_v = run && rs_avail && (oj != onum);
      need_acc = (op_r == OP_EMFEED) || (op_r == OP_CMP);
    end
  end

  // destination readiness of the item in A
  logic dst_rdy;
  assign dst_rdy = (op_r == OP_EMFEED) ? em_ready : 1'b1;
  assign consume = a_v && dst_rdy;
  assign issue = src_v && (!a_v || consume);
  assign rs_get = issue && !(op_r == OP_PACK_KG || op_r == OP_PACK_CT);

  // ACC reads for the issued item
  always_comb begin
    ar_en = 2'b00; ar_addr = (op_r == OP_PACK_KG || op_r == OP_PACK_CT) ? pk_j : oj;
    if (issue && need_acc) begin
      if (op_r == OP_EMFEED) ar_en = 2'b10;
      else if (op_r == OP_CMP) ar_en = cph ? 2'b10 : 2'b01;
      else ar_en = (src_kind == 2'd1) ? 2'b01 : 2'b10;
    end
  end

  // stage-A data
  logic [511:0] a_acc, a_val, vlast_mask, lmask;
  always_comb for (int i = 0; i < 512; i++) lmask[i] = (9'(i) < lastbits_r);
  always_comb begin
    a_acc = 512'd0;
    case (op_r)
      OP_EMFEED: a_acc = ar_data[1];
      OP_CMP:    a_acc = cph ? (ar_data[1] ^ cw_word) : ar_data[0];
      default:   a_acc = (a_kind == 2'd1) ? ar_data[0] : (op_r == OP_PACK_CT) ? (ar_data[1] ^ cw_word) : ar_data[1];
    endcase
    a_val = (op_r == OP_PACK_KG || op_r == OP_PACK_CT) ? ((a_kind == 2'd0) ? a_w : a_acc) : a_w;
  end

  // compare mask of the item in A
  logic [6:0]  cmp_bytes;
  always_comb begin
    if (!cph) cmp_bytes = (a_j == nw_r - 9'd1) ? 7'(nb_r - {a_j, 6'd0}) : 7'd64;
    else      cmp_bytes = (a_j == nwv_r - 9'd1) ? 7'(nv_r - {a_j, 6'd0}) : 7'd64;
  end

  // ---------------- packers (0: PK or CT target, 1: SK)
  logic [6:0]   pf [0:1];
  logic [511:0] pb [0:1];
  logic [9:0]   pa [0:1];
  logic         pflush;
  logic [1023:0] pcomb [0:1];
  logic [511:0] chunk;
  always_comb begin
    chunk = a_val & bytemask(a_len);
    for (int x = 0; x < 2; x++) pcomb[x] = {512'd0, pb[x]} | ({512'd0, chunk} << {pf[x], 3'd0});
  end

  // ---------------- outputs (registered)
  logic [2:0]   w_rq_r;
  logic [9:0]   w_addr_r [0:2];
  logic [511:0] w_data_r [0:2];
  assign w_rq = w_rq_r;
  assign w_addr = w_addr_r;
  assign w_data = w_data_r;
  assign busy = run;
  assign cmp_eq = eq_r;

  logic [511:0] em_word;
  always_comb em_word = a_w ^ a_acc;
  assign em_valid = a_v && (op_r == OP_EMFEED);
  assign em_data = em_word;

  logic stream_done;
  assign stream_done = !(op_r == OP_PACK_KG || op_r == OP_PACK_CT) && (oj == onum) && !a_v;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      op_r <= '0; buf_r <= '0; lanes_r <= '0; slot_r <= '0; k_r <= '0; nw_r <= '0; nwv_r <= '0;
      lastbits_r <= '0; nb_r <= '0; nv_r <= '0; n1_r <= '0; mult_r <= '0; run <= 1'b0; cph <= 1'b0;
      eq_r <= 1'b0; done <= 1'b0;
      rs_on <= 1'b0; rs_w <= '0; rs_left <= '0; rs_bs <= '0; rs_out <= '0; fq_rd <= '0; fq_wr <= '0;
      fq_n <= '0; rs_pv <= 1'b0; rs_prev <= '0;
      for (int i = 0; i < 4; i++) fq[i] <= '0;
      a_v <= 1'b0; a_w <= '0; a_j <= '0; a_kind <= '0; a_len <= '0; a_skonly <= 1'b0;
      oj <= '0; onum <= '0; pk_seg <= '0; pk_j <= '0; pk_on <= 1'b0; cw_sym <= '0; cw_cp <= '0;
      for (int x = 0; x < 2; x++) begin pf[x] <= '0; pb[x] <= '0; pa[x] <= '0; end
      pflush <= 1'b0;
      w_rq_r <= '0;
      for (int b = 0; b < 3; b++) begin w_addr_r[b] <= '0; w_data_r[b] <= '0; end
      ext_we <= 1'b0; ext_data <= '0;
      sw_en <= '0; sw_slot <= '0; sw_addr <= '0; sw_data <= '0;
    end else begin
      done <= 1'b0; ext_we <= 1'b0; sw_en <= '0; w_rq_r <= '0;
      if (start) begin
        op_r <= op; buf_r <= (op == OP_EMFEED || op == OP_CMP) ? 2'd2 : sbuf; lanes_r <= dlanes; slot_r <= dslot;
        k_r <= p_k(ps); nw_r <= p_nw(ps); nwv_r <= p_nwv(ps); lastbits_r <= p_lastbits(ps);
        nb_r <= p_nb(ps); nv_r <= p_nv(ps); n1_r <= p_n1(ps); mult_r <= p_mult(ps);
        run <= 1'b1; cph <= 1'b0; eq_r <= 1'b1; a_v <= 1'b0; oj <= '0;
        cw_sym <= '0; cw_cp <= '0;
        for (int x = 0; x < 2; x++) begin pf[x] <= '0; pb[x] <= '0; pa[x] <= '0; end
        pflush <= 1'b0;
        pk_on <= (op == OP_PACK_KG || op == OP_PACK_CT);
        pk_seg <= (op == OP_PACK_KG) ? 3'd0 : 3'd2; pk_j <= '0;
        // read stream setup
        begin
          logic [15:0] off;
          logic [8:0]  no;
          case (op)
            OP_EMFEED: begin off = {1'b0, p_nb(ps)}; no = p_nwv(ps); end
            OP_CMP:    begin off = 16'd0; no = p_nw(ps); end
            OP_EXTRACT: begin off = soff; no = 9'd1; end
            default:   begin off = soff; no = p_nw(ps); end
          endcase
          rs_on <= !(op == OP_PACK_KG || op == OP_PACK_CT);
          rs_w <= off[15:6]; rs_bs <= off[5:0];
          rs_left <= {1'b0, no} + ((off[5:0] != 6'd0) ? 10'd1 : 10'd0);
          onum <= no;
        end
        rs_out <= '0; fq_rd <= '0; fq_wr <= '0; fq_n <= '0; rs_pv <= 1'b0;
      end else if (run) begin
        // ---- read requests and returns
        if (|(r_rq & r_gnt)) begin rs_w <= rs_w + 10'd1; rs_left <= rs_left - 10'd1; end
        begin
          logic [2:0] o_n; logic [2:0] q_n;
          o_n = rs_out + ((|(r_rq & r_gnt)) ? 3'd1 : 3'd0) - (got ? 3'd1 : 3'd0);
          rs_out <= o_n;
          q_n = fq_n;
          if (got) begin fq[fq_wr] <= r_data[buf_r]; fq_wr <= fq_wr + 2'd1; q_n = q_n + 3'd1; end
          // realign: with a byte shift the first word only primes prev
          if (fq_n != 3'd0 && rs_bs != 6'd0 && !rs_pv) begin
            rs_prev <= fq[fq_rd]; rs_pv <= 1'b1; fq_rd <= fq_rd + 2'd1; q_n = q_n - 3'd1;
          end else if (rs_get) begin
            rs_prev <= fq[fq_rd]; fq_rd <= fq_rd + 2'd1; q_n = q_n - 3'd1;
          end
          fq_n <= q_n;
        end
        // ---- consume item in A
        if (consume) begin
          a_v <= 1'b0;
          case (op_r)
            OP_LOAD: begin
              sw_en <= lanes_r; sw_slot <= slot_r; sw_addr <= a_j;
              sw_data <= (a_j == nw_r - 9'd1) ? (a_w & lmask) : a_w;
            end
            OP_EXTRACT: begin ext_we <= 1'b1; ext_data <= a_w & bytemask(k_r); end
            OP_CMP: begin
              if (((a_w ^ a_acc) & bytemask(cmp_bytes)) != 512'd0) eq_r <= 1'b0;
            end
            OP_PACK_KG, OP_PACK_CT: begin
              for (int x = 0; x < 2; x++) begin
                if (!(x == 0 && a_skonly) && !(x == 1 && op_r == OP_PACK_CT)) begin
                  if ({1'b0, pf[x]} + {1'b0, a_len} >= 8'd64) begin
                    w_rq_r[(x == 1) ? 2 - 1 : ((op_r == OP_PACK_CT) ? 2 : 0)] <= 1'b1;
                    w_addr_r[(x == 1) ? 1 : ((op_r == OP_PACK_CT) ? 2 : 0)] <= pa[x];
                    w_data_r[(x == 1) ? 1 : ((op_r == OP_PACK_CT) ? 2 : 0)] <= pcomb[x][511:0];
                    pa[x] <= pa[x] + 10'd1;
                    pb[x] <= pcomb[x][1023:512];
                    pf[x] <= pf[x] + a_len - 7'd64;
                  end else begin
                    pb[x] <= pcomb[x][511:0];
                    pf[x] <= pf[x] + a_len;
                  end
                end
              end
            end
            default: ;
          endcase
          if (op_r == OP_CMP && cph || op_r == OP_PACK_CT && a_kind == 2'd2) begin
            cw_sym <= cw_sym_n; cw_cp <= cw_cp_n;
          end
        end
        // ---- issue next item
        if (issue) begin
          a_v <= 1'b1; a_w <= src_w; a_kind <= src_kind; a_len <= src_len; a_skonly <= src_skonly;
          a_j <= (op_r == OP_PACK_KG || op_r == OP_PACK_CT) ? pk_j : oj;
          if (!(op_r == OP_PACK_KG || op_r == OP_PACK_CT)) oj <= oj + 9'd1;
          else begin
            // advance the pack sequencer
            case (pk_seg)
              3'd0: pk_seg <= 3'd1;
              3'd1: pk_seg <= 3'd2;
              3'd2: if (pk_j == nw_r - 9'd1) begin pk_j <= '0; pk_seg <= 3'd3; end else pk_j <= pk_j + 9'd1;
              3'd3: if (pk_j == ((op_r == OP_PACK_KG) ? nw_r : nwv_r) - 9'd1) begin
                      pk_j <= '0; pk_seg <= (op_r == OP_PACK_KG) ? 3'd4 : 3'd5;
                    end else pk_j <= pk_j + 9'd1;
              default: pk_on <= 1'b0;
            endcase
          end
        end
        // ---- completion
        if (op_r == OP_PACK_KG || op_r == OP_PACK_CT) begin
          if (!pk_on && !a_v && !issue && !pflush) begin
            // flush partial words
            pflush <= 1'b1;
            for (int x = 0; x < 2; x++) begin
              if (pf[x] != 7'd0 && !(x == 1 && op_r == OP_PACK_CT)) begin
                w_rq_r[(x == 1) ? 1 : ((op_r == OP_PACK_CT) ? 2 : 0)] <= 1'b1;
                w_addr_r[(x == 1) ? 1 : ((op_r == OP_PACK_CT) ? 2 : 0)] <= pa[x];
                w_data_r[(x == 1) ? 1 : ((op_r == OP_PACK_CT) ? 2 : 0)] <= pb[x];
              end
            end
          end
          if (pflush) begin run <= 1'b0; done <= 1'b1; end
        end else if (stream_done && !issue) begin
          if (op_r == OP_CMP && !cph) begin
            // start the v phase
            cph <= 1'b1; oj <= '0; onum <= nwv_r;
            rs_w <= nb_r[14:6]; rs_bs <= nb_r[5:0];
            rs_left <= {1'b0, nwv_r} + ((nb_r[5:0] != 6'd0) ? 10'd1 : 10'd0);
            rs_pv <= 1'b0; fq_rd <= '0; fq_wr <= '0; fq_n <= '0;
          end else if (rs_out == 3'd0 && !got) begin
            run <= 1'b0; done <= 1'b1; rs_on <= 1'b0;
          end
        end
      end
    end
  end
endmodule
