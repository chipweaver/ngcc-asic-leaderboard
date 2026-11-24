// nev_sampler: KDF buffer (128 x 128 1r1w SRAM, one KDF stream region) and polynomial samplers.
//
// dist_code 1/2/3/4/7/8 (cbd1, cbd2, cbd3, cbd4, cbd7, bias8): "slot" pipeline. Each output word w
//   issues one slot per KDF row it needs (row kinds R1 = cbd1, R2 = cbd2, RM = bias8 mask,
//   R4 = cbd4; a row is shared by 4/2/8/1 words) or a single no-fetch slot; the last slot of the
//   word carries "emit". Rows land in per-kind staging registers; the emit slot assembles the
//   16 signed values from the staging registers.
// dist_code 9 (bias3): sequential rows, 2 bytes per cycle through two base-3 decoders into a
//   25-entry trit buffer; one word emitted whenever 16 trits are buffered.
// dist_code 15 (noisem): for row position p and group t, the 2*eta rows (chunk c = 2*eta*t + u, row
//   (c-1)*S/16 + p; chunk 0 from c0) are accumulated; groups are taken in the order t = 1, 2, 3, 0 so
//   that only the last groups need c0 (final once the whole KDF stream is written, c0_ok), and the
//   other groups proceed while the stream is still being written; their rows into 128 signed lanes; the result is moved
//   into an output buffer (shared with the cbd staging registers) and emitted as 8 words while
//   the next group accumulates.
// All paths feed one back end: signed value -> canonical mod q (optional x R^-1, add_vinv).
//
// Pipeline: S0 decide (registered re/raddr) | S1 SRAM read | S2 rdata -> rreg | S3 staging /
//   accumulate / bias3 decode | S4 word assembly -> be | S5 mod q -> pw_* (registered).
module nev_sampler
  import nev_pkg::*;
(
  input  logic         clk,
  input  logic         rst_n,
  // configuration (latched by the top)
  input  logic [1:0]   qsel,
  input  logic [1:0]   nlog,
  input  logic         d64,
  input  logic [1:0]   slog,
  // command
  input  logic         cmd_valid,
  input  logic [3:0]   dist_code,   // "dist" field (reserved word in SystemVerilog)
  input  logic         region,
  input  logic [2:0]   dst_buf,
  input  logic         scale_rinv,
  input  logic         add_vinv,
  input  logic [3:0]   eta_e,
  input  logic [511:0] c0,
  input  logic         c0_ok,       // c0 final (noisem)
  output logic         busy,
  output logic         done,
  output logic         kdf_need,
  // KDF buffer write port and watermark (from nev_hash)
  input  logic         kb_we,
  input  logic [7:0]   kb_waddr,
  input  logic [127:0] kb_wdata,
  input  logic [6:0]   kb_blocks,
  input  logic         kb_region,
  // poly buffer write port
  output logic         pw_valid,
  output logic [2:0]   pw_buf,
  output logic [6:0]   pw_addr,
  output logic [191:0] pw_data
);

  typedef struct packed {
    logic       valid;   // slot present
    logic       fetch;   // SRAM row read issued
    logic [1:0] kind;    // cbd: 0 R1, 1 R2, 2 RM, 3 R4
    logic       emit;    // cbd: last slot of word w
    logic [6:0] w;       // cbd: word index
    logic       first;   // noisem: u == 0 (clear accumulator)
    logic       last;    // noisem: u == 2*eta-1 (move to output buffer)
    logic       neg;     // noisem: u >= eta (subtract)
    logic       c0;      // noisem: chunk 0 (data from c0, no fetch)
    logic [1:0] p;       // noisem: row position
    logic [1:0] t;       // noisem: group
  } tag_t;

  // ---------------------------------------------------------------- command state
  logic         busy_r, done_r, fin1_r, fin2_r;
  logic         md_cbd, md_b3, md_nm;
  logic         region_r, scale_r, vinv_r;
  logic [2:0]   dst_r;
  logic [3:0]   eta_r;
  logic         use1, use2, use4, usem;
  logic [511:0] c0_r;

  logic [7:0]   nw;       // N/16
  logic [11:0]  ncoef;    // N
  logic [2:0]   s16;      // S/16
  logic [7:0]   n64;      // N/64
  logic [7:0]   avail;    // rows of the command region covered by the watermark
  logic         reg_match;

  always_comb begin
    nw        = nwords(nlog);
    ncoef     = {nw, 4'd0};
    s16       = 3'd1 << slog;
    n64       = 8'd8 << nlog;
    reg_match = (kb_region == region_r);
    avail     = reg_match ? {kb_blocks, 1'b0} : 8'd0;
  end

  // ---------------------------------------------------------------- KDF buffer SRAM
  logic         sr_re;
  logic [6:0]   sr_raddr;
  logic [127:0] sr_rdata;

  nev_sram_1r1w #(.DEPTH(128), .WIDTH(128)) u_kb (
    .clk   (clk),
    .re    (sr_re),
    .raddr (sr_raddr),
    .rdata (sr_rdata),
    .we    (kb_we),
    .waddr (kb_waddr[6:0]),
    .wdata (kb_wdata)
  );

  // ---------------------------------------------------------------- S0 state
  // cbd
  logic [7:0]   w_r;
  logic [3:0]   issued_r;            // bit k: kind k already fetched for word w_r
  logic [7:0]   p1_r, p2_r, pm_r, p4_r;
  // noisem
  logic [2:0]   np_r;
  logic [1:0]   nt_r;
  logic [3:0]   nu_r;
  logic [7:0]   nrow_r;
  logic [3:0]   gcnt_r;              // cycles since the last "last" slot (saturates at 8)
  // bias3
  logic [7:0]   rowf_r;
  logic         cur_v;
  logic [2:0]   pi_r;
  logic [11:0]  ccount_r;

  tag_t         a_r, b_r, c_r;       // S1, S2, S3 slot tags
  logic         d_emit;              // S4 (cbd)
  logic [6:0]   d_w;
  logic [127:0] rreg;
  logic [511:0] stg;                 // cbd staging rows / noisem output buffer
  logic [511:0] acc;                 // noisem accumulator (128 x 4-bit signed)

  // S0 combinational decision
  tag_t         s0_tag;
  logic         s0_go;
  logic [7:0]   s0_row;
  logic [3:0]   need, rem, pick;
  logic [1:0]   pick_k;
  logic [7:0]   prow;
  logic [3:0]   twoeta;
  logic [5:0]   nm_chunk;
  logic [7:0]   nm_row;
  logic         nm_last, nm_c0;
  logic         b3_inflight, b3_want, b3_more;

  always_comb begin
    s0_tag = '0;
    s0_go  = 1'b0;
    s0_row = 8'd0;
    // cbd slot selection
    need = {use4, usem & (w_r[2:0] == 3'd0), use2 & ~w_r[0], use1 & (w_r[1:0] == 2'd0)};
    rem  = need & ~issued_r;
    if (rem[0])      begin pick = 4'b0001; pick_k = 2'd0; end
    else if (rem[1]) begin pick = 4'b0010; pick_k = 2'd1; end
    else if (rem[2]) begin pick = 4'b0100; pick_k = 2'd2; end
    else             begin pick = 4'b1000; pick_k = 2'd3; end
    case (pick_k)
      2'd0:    prow = p1_r;
      2'd1:    prow = p2_r;
      2'd2:    prow = pm_r;
      default: prow = p4_r;
    endcase
    // noisem
    twoeta  = {eta_r[2:0], 1'b0};
    nm_last = (nu_r == (twoeta - 4'd1));
    nm_c0   = (nt_r == 2'd0) && (nu_r == 4'd0);
    nm_chunk = {1'b0, eta_r, 1'b0} * {4'd0, nt_r} + {2'd0, nu_r};     // c = 2*eta*t + u
    nm_row   = (8'(nm_chunk - 6'd1) << slog) + {5'd0, np_r};
    // bias3
    b3_inflight = a_r.fetch | b_r.fetch;
    b3_more     = (ccount_r < ncoef);
    b3_want     = b3_more & ~b3_inflight & (~cur_v | (pi_r >= 3'd5));

    if (busy_r && md_cbd && (w_r < nw)) begin
      s0_tag.w = w_r[6:0];
      if (rem == 4'd0) begin
        s0_go = 1'b1; s0_tag.valid = 1'b1; s0_tag.emit = 1'b1;
      end else if (prow < avail) begin
        s0_go = 1'b1; s0_tag.valid = 1'b1; s0_tag.fetch = 1'b1; s0_tag.kind = pick_k;
        s0_tag.emit = ((rem & ~pick) == 4'd0);
        s0_row = prow;
      end
    end
    if (busy_r && md_nm && (np_r < s16)) begin
      s0_tag.p     = np_r[1:0];
      s0_tag.t     = nt_r;
      s0_tag.first = (nu_r == 4'd0);
      s0_tag.last  = nm_last;
      s0_tag.neg   = (nu_r >= eta_r);
      if (nm_c0) begin
        if (c0_ok) begin s0_go = 1'b1; s0_tag.valid = 1'b1; s0_tag.c0 = 1'b1; end
      end else if ((nm_row < avail) && (!nm_last || gcnt_r[3])) begin
        s0_go = 1'b1; s0_tag.valid = 1'b1; s0_tag.fetch = 1'b1;
        s0_row = nm_row;
      end
    end
    if (busy_r && md_b3 && b3_want && (rowf_r < avail)) begin
      s0_go = 1'b1; s0_tag.valid = 1'b1; s0_tag.fetch = 1'b1;
      s0_row = rowf_r;
    end
  end

  // bias3: request more KDF blocks only when the decoder is starved (every fetched byte has been
  // examined, fewer than N coefficients found) and the next row is beyond the watermark.
  always_comb begin
    kdf_need = busy_r & md_b3 & reg_match & b3_more & ~cur_v & ~b3_inflight & ~(rowf_r < avail);
  end

  // ---------------------------------------------------------------- command latch, S0 state
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      md_cbd <= 1'b0; md_b3 <= 1'b0; md_nm <= 1'b0;
      region_r <= 1'b0; scale_r <= 1'b0; vinv_r <= 1'b0; dst_r <= 3'd0; eta_r <= 4'd0;
      use1 <= 1'b0; use2 <= 1'b0; use4 <= 1'b0; usem <= 1'b0;
      w_r <= 8'd0; issued_r <= 4'd0;
      p1_r <= 8'd0; p2_r <= 8'd0; pm_r <= 8'd0; p4_r <= 8'd0;
      np_r <= 3'd0; nt_r <= 2'd0; nu_r <= 4'd0; nrow_r <= 8'd0; gcnt_r <= 4'd8;
      rowf_r <= 8'd0;
    end else if (cmd_valid) begin
      md_cbd   <= (dist_code != 4'd9) && (dist_code != 4'd15);
      md_b3    <= (dist_code == 4'd9);
      md_nm    <= (dist_code == 4'd15);
      region_r <= region;
      scale_r  <= scale_rinv;
      vinv_r   <= add_vinv;
      dst_r    <= dst_buf;
      eta_r    <= eta_e;
      use1     <= (dist_code == 4'd1) || (dist_code == 4'd3) || (dist_code == 4'd7) || (dist_code == 4'd8);
      use2     <= (dist_code == 4'd2) || (dist_code == 4'd3) || (dist_code == 4'd7);
      use4     <= (dist_code == 4'd4) || (dist_code == 4'd7);
      usem     <= (dist_code == 4'd8);
      w_r      <= 8'd0;
      issued_r <= 4'd0;
      // stream row bases: cbd1 [0 | N (cbd7)], cbd2 [0 | N/4 (cbd3) | 5N/4 (cbd7)], mask N/4
      p1_r     <= (dist_code == 4'd7) ? nw : 8'd0;
      p2_r     <= (dist_code == 4'd3) ? n64 : ((dist_code == 4'd7) ? (n64 + nw) : 8'd0);
      pm_r     <= n64;
      p4_r     <= 8'd0;
      np_r <= 3'd0; nt_r <= 2'd1; nu_r <= 4'd0; nrow_r <= 8'd0; gcnt_r <= 4'd8;
      rowf_r   <= 8'd0;
    end else begin
      if (s0_go && md_cbd) begin
        if (s0_tag.emit) begin
          w_r <= w_r + 8'd1;
          issued_r <= 4'd0;
        end else begin
          issued_r <= issued_r | pick;
        end
        if (s0_tag.fetch) begin
          case (pick_k)
            2'd0:    p1_r <= p1_r + 8'd1;
            2'd1:    p2_r <= p2_r + 8'd1;
            2'd2:    pm_r <= pm_r + 8'd1;
            default: p4_r <= p4_r + 8'd1;
          endcase
        end
      end
      if (s0_go && md_nm) begin
        if (nm_last) begin
          // group order t = 1, 2, 3, 0 (outer), row position p (inner): only the c0 groups wait for
          // the end of the KDF stream
          nu_r <= 4'd0;
          if (np_r == s16 - 3'd1) begin
            if (nt_r == 2'd0) np_r <= s16;     // finished
            else begin np_r <= 3'd0; nt_r <= nt_r + 2'd1; end
          end else np_r <= np_r + 3'd1;
        end else begin
          nu_r <= nu_r + 4'd1;
        end
      end
      if (s0_go && md_nm && nm_last) gcnt_r <= 4'd1;
      else if (!gcnt_r[3])          gcnt_r <= gcnt_r + 4'd1;
      if (s0_go && md_b3) rowf_r <= rowf_r + 8'd1;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)              c0_r <= '0;
    else if (s0_go && md_nm && nm_c0) c0_r <= c0;   // c0 is final when group 0 starts
  end

  // ---------------------------------------------------------------- S1/S2: SRAM read, rreg
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sr_re <= 1'b0; sr_raddr <= 7'd0;
      a_r <= '0; b_r <= '0; c_r <= '0;
    end else if (cmd_valid) begin
      sr_re <= 1'b0;
      a_r <= '0; b_r <= '0; c_r <= '0;
    end else begin
      sr_re    <= s0_go & s0_tag.fetch;
      sr_raddr <= s0_row[6:0];
      a_r      <= s0_go ? s0_tag : '0;
      b_r      <= a_r;
      c_r      <= b_r;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)        rreg <= '0;
    else if (b_r.fetch) rreg <= sr_rdata;
  end

  // ---------------------------------------------------------------- S3: cbd staging, noisem accumulate
  logic [511:0] acc_n;
  logic [127:0] nm_src;
  logic         nm_load;
  logic [3:0]   nm_base;
  logic         em_active;
  logic [2:0]   em_k;
  logic [1:0]   em_p, em_t;

  always_comb begin
    nm_src = c_r.c0 ? c0_r[128*c_r.p +: 128] : rreg;
    for (int l = 0; l < 128; l++) begin
      nm_base = c_r.first ? 4'd0 : acc[4*l +: 4];
      if (nm_src[l]) acc_n[4*l +: 4] = c_r.neg ? (nm_base - 4'd1) : (nm_base + 4'd1);
      else           acc_n[4*l +: 4] = nm_base;
    end
    nm_load = md_nm & c_r.valid & c_r.last;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      stg <= '0;
      acc <= '0;
    end else begin
      if (md_cbd && c_r.fetch) begin
        case (c_r.kind)
          2'd0:    stg[127:0]   <= rreg;
          2'd1:    stg[255:128] <= rreg;
          2'd2:    stg[383:256] <= rreg;
          default: stg[511:384] <= rreg;
        endcase
      end
      if (md_nm && c_r.valid) acc <= acc_n;
      if (nm_load) stg <= acc_n;
    end
  end

  // noisem output-buffer emitter (8 words per group)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      em_active <= 1'b0; em_k <= 3'd0; em_p <= 2'd0; em_t <= 2'd0;
    end else if (cmd_valid) begin
      em_active <= 1'b0; em_k <= 3'd0;
    end else if (nm_load) begin
      em_active <= 1'b1; em_k <= 3'd0; em_p <= c_r.p; em_t <= c_r.t;
    end else if (em_active) begin
      em_k <= em_k + 3'd1;
      if (em_k == 3'd7) em_active <= 1'b0;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      d_emit <= 1'b0; d_w <= 7'd0;
    end else if (cmd_valid) begin
      d_emit <= 1'b0;
    end else begin
      d_emit <= md_cbd & c_r.emit;
      d_w    <= c_r.w;
    end
  end

  // ---------------------------------------------------------------- S3: bias3 decode
  logic [15:0]  b3_pair;
  logic         b3_ok0, b3_ok1, b3_cons;
  logic [9:0]   b3_d0, b3_d1;
  logic         ba_v, ba_ok0, ba_ok1;
  logic [9:0]   ba_d0, ba_d1;

  always_comb begin
    b3_pair = rreg[16*pi_r +: 16];
    b3_cons = md_b3 & cur_v & b3_more;
  end

  nev_samp_tern u_tern0 (.x(b3_pair[7:0]),  .ok(b3_ok0), .d(b3_d0));
  nev_samp_tern u_tern1 (.x(b3_pair[15:8]), .ok(b3_ok1), .d(b3_d1));

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cur_v <= 1'b0; pi_r <= 3'd0; ccount_r <= 12'd0;
      ba_v <= 1'b0; ba_ok0 <= 1'b0; ba_ok1 <= 1'b0; ba_d0 <= 10'd0; ba_d1 <= 10'd0;
    end else if (cmd_valid) begin
      cur_v <= 1'b0; pi_r <= 3'd0; ccount_r <= 12'd0; ba_v <= 1'b0;
    end else begin
      ba_v   <= b3_cons;
      ba_ok0 <= b3_ok0;
      ba_ok1 <= b3_ok1;
      ba_d0  <= b3_d0;
      ba_d1  <= b3_d1;
      if (b3_cons) begin
        ccount_r <= ccount_r + (b3_ok0 ? 12'd5 : 12'd0) + (b3_ok1 ? 12'd5 : 12'd0);
        pi_r     <= pi_r + 3'd1;
        if (pi_r == 3'd7) cur_v <= 1'b0;
      end
      if (md_b3 && b_r.fetch) begin     // next row arrives in rreg
        cur_v <= 1'b1;
        pi_r  <= 3'd0;
      end
    end
  end

  // ---------------------------------------------------------------- S4: trit buffer (bias3)
  logic [49:0]  cb;          // trit k at [2k+1:2k], entries >= cb_cnt are zero
  logic [4:0]   cb_cnt;
  logic [49:0]  cb_n;
  logic [4:0]   cb_base;
  logic [19:0]  cb_new;
  logic [4:0]   cb_nn;
  logic         cb_emit;
  logic [7:0]   be_cnt;

  always_comb begin
    cb_emit = (cb_cnt >= 5'd16);
    cb_base = cb_emit ? (cb_cnt - 5'd16) : cb_cnt;
    if (ba_v && ba_ok0) cb_new = {(ba_ok1 ? ba_d1 : 10'd0), ba_d0};
    else if (ba_v && ba_ok1) cb_new = {10'd0, ba_d1};
    else cb_new = 20'd0;
    cb_nn = (ba_v && ba_ok0 ? 5'd5 : 5'd0) + (ba_v && ba_ok1 ? 5'd5 : 5'd0);
    cb_n  = (cb_emit ? {32'd0, cb[49:32]} : cb) | ({30'd0, cb_new} << {cb_base, 1'b0});
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cb <= '0; cb_cnt <= 5'd0;
    end else if (cmd_valid) begin
      cb <= '0; cb_cnt <= 5'd0;
    end else if (md_b3) begin
      cb     <= cb_n;
      cb_cnt <= cb_base + cb_nn;
    end
  end

  // ---------------------------------------------------------------- S4: word assembly -> be
  logic         be_valid_n;
  logic [6:0]   be_addr_n;
  logic [63:0]  be_vals_n;
  logic         be_valid;
  logic [6:0]   be_addr;
  logic [63:0]  be_vals;
  logic [31:0]  r1s;
  logic [63:0]  r2s;
  logic [15:0]  rms;
  logic [127:0] r4s;
  logic [3:0]   c1, c2, c4, cv;

  always_comb begin
    r1s = stg[32*d_w[1:0] +: 32];
    r2s = stg[128 + 64*d_w[0] +: 64];
    rms = stg[256 + 16*d_w[2:0] +: 16];
    r4s = stg[511:384];
    be_valid_n = 1'b0;
    be_addr_n  = 7'd0;
    be_vals_n  = 64'd0;
    if (md_cbd) begin
      be_valid_n = d_emit;
      be_addr_n  = d_w;
      for (int l = 0; l < 16; l++) begin
        c1 = {3'd0, r1s[2*l]} - {3'd0, r1s[2*l+1]};
        c2 = {3'd0, r2s[4*l]} + {3'd0, r2s[4*l+1]} - {3'd0, r2s[4*l+2]} - {3'd0, r2s[4*l+3]};
        c4 = {3'd0, r4s[8*l]} + {3'd0, r4s[8*l+1]} + {3'd0, r4s[8*l+2]} + {3'd0, r4s[8*l+3]}
           - {3'd0, r4s[8*l+4]} - {3'd0, r4s[8*l+5]} - {3'd0, r4s[8*l+6]} - {3'd0, r4s[8*l+7]};
        cv = (use4 ? c4 : 4'd0) + (use1 ? c1 : 4'd0) + (use2 ? c2 : 4'd0);
        if (usem) cv = rms[l] ? c1 : 4'd0;
        be_vals_n[4*l +: 4] = cv;
      end
    end else if (md_nm) begin
      be_valid_n = em_active;
      be_addr_n  = d64 ? {em_p, em_k, em_t}                           // 32p + 4k + t
                       : {em_p, em_k[2:1], em_t, em_k[0]};             // 32p + 8(k>>1) + 2t + (k&1)
      be_vals_n  = stg[64*em_k +: 64];
    end else if (md_b3) begin
      be_valid_n = cb_emit && (be_cnt < nw);
      be_addr_n  = be_cnt[6:0];
      for (int l = 0; l < 16; l++) begin
        // coefficient = 1 - digit : 0 -> +1, 1 -> 0, 2 -> -1
        case (cb[2*l +: 2])
          2'd0:    be_vals_n[4*l +: 4] = 4'd1;
          2'd2:    be_vals_n[4*l +: 4] = 4'hf;
          default: be_vals_n[4*l +: 4] = 4'd0;
        endcase
      end
    end
    be_valid_n = be_valid_n & busy_r;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      be_valid <= 1'b0; be_addr <= 7'd0; be_vals <= 64'd0; be_cnt <= 8'd0;
    end else if (cmd_valid) begin
      be_valid <= 1'b0; be_cnt <= 8'd0;
    end else begin
      be_valid <= be_valid_n;
      be_addr  <= be_addr_n;
      be_vals  <= be_vals_n;
      if (be_valid_n) be_cnt <= be_cnt + 8'd1;
    end
  end

  // ---------------------------------------------------------------- S5: mod q / scale -> pw
  logic [191:0] pw_data_n;
  logic [11:0]  lane_y [0:15];
  logic         vinv_hit;
  logic [11:0]  vinv_h;

  genvar gl;
  generate
    for (gl = 0; gl < 16; gl++) begin : g_lane
      nev_samp_coef u_coef (.qsel(qsel), .scale(scale_r), .v(be_vals[4*gl +: 4]), .y(lane_y[gl]));
    end
  endgenerate

  always_comb begin
    // add_vinv: coefficients 0, D/4, D/2, 3D/4 = lane 0 of words {0,1,2,3} (D=64) / {0,2,4,6}
    vinv_hit = vinv_r & (d64 ? (be_addr < 7'd4) : ((be_addr < 7'd8) & ~be_addr[0]));
    case (qsel)
      Q769:    vinv_h = 12'd385;
      Q1409:   vinv_h = 12'd705;
      default: vinv_h = 12'd1665;
    endcase
    for (int l = 0; l < 16; l++) pw_data_n[12*l +: 12] = lane_y[l];
    // |v| <= 7 < q - h: (v mod q + h) mod q = h + v
    if (vinv_hit) pw_data_n[11:0] = vinv_h + {{8{be_vals[3]}}, be_vals[3:0]};
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pw_valid <= 1'b0; pw_buf <= 3'd0; pw_addr <= 7'd0; pw_data <= '0;
    end else begin
      pw_valid <= be_valid & ~cmd_valid;
      pw_buf   <= dst_r;
      pw_addr  <= be_addr;
      pw_data  <= pw_data_n;
    end
  end

  // ---------------------------------------------------------------- busy / done
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy_r <= 1'b0; done_r <= 1'b0; fin1_r <= 1'b0; fin2_r <= 1'b0;
    end else begin
      fin1_r <= be_valid_n & (be_cnt == nw - 8'd1) & ~cmd_valid;   // aligned with be_valid
      fin2_r <= fin1_r;                                             // aligned with pw_valid
      done_r <= fin2_r;
      if (cmd_valid)   busy_r <= 1'b1;
      else if (fin2_r) busy_r <= 1'b0;
    end
  end

  always_comb begin
    busy = busy_r;
    done = done_r;
  end

endmodule
