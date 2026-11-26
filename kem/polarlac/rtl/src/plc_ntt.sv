// plc_ntt - in-place NTT / INTT engine, 8 butterflies per cycle.
// Bit-exact with mq_poly_ntt / mq_poly_intt of ntt.c (all five sets; q=257 sets share
// f/fn, 512-Star uses its own q=769 tables; int16 wrap, Montgomery reduction).
//
//  forward: k=1; for len=N/2 .. N/128 (7 layers): zeta=f[k++] per group;
//           t=fqmul(zeta,a[j+len]); a[j+len]=a[j]-t; a[j]=a[j]+t
//  inverse: k=0; for len=N/128 .. N/2 (7 layers): zeta=fn[k++] per group;
//           t=a[j]; a[j]=t+a[j+len]; a[j+len]=fqmul(zeta,t-a[j+len]);
//           then a[j]=fqmul(a[j],fn[127]) for all j (fused into the last layer).
//
// Storage: word w (lane k = coeff 8w+k) is in bank ^w at address w>>1.
// Each cycle one "pair" of words (w0,w1) is read (always one per bank) and written back
// LAT=9 cycles later. Layer l pair p:
//   logL>=3 (len>=8): w0 = p with a 0 inserted at bit s=logL-3, w1 = w0|1<<s, lanes (W0[k],W1[k])
//   logL==2 / ==1   : w0 = 2p, w1 = 2p+1, butterflies inside each word.
// Read-after-write between layers: with P = N/16 pairs per layer, word x is read in layer
// l+1 at T+P+G+p'(x) and written in layer l at T+p(x)+LAT; max(p(x)-p'(x)) = P/2, so a gap
// G >= LAT+1-P/2 is needed: none for N=512/1024, two idle cycles per layer for N=256.
// Read data is valid 2 cycles after ren (crossbar latency).
module plc_ntt
  import plc_pkg::*;
  import plc_rom_pkg::*;
  import plc_ntt_pwalu_pkg::*;
(
  input  logic              clk,
  input  logic              rst_n,
  input  logic              start,
  input  logic              inv,        // 0 forward, 1 inverse (with fn[127] scaling)
  input  logic [2:0]        pcode,
  input  logic [3:0]        slot,
  output logic              busy,
  output logic              done,
  // slot read port
  output logic [3:0]        rslot,
  output logic [1:0]        ren,
  output logic [1:0][5:0]   raddr,
  input  logic [1:0][127:0] rdata,
  // slot write port
  output logic [3:0]        wslot,
  output logic [1:0]        wen,
  output logic [1:0][5:0]   waddr,
  output logic [1:0][127:0] wdata
);
  localparam int LAT = 9;   // read issue -> write cycle

  // ---------------- configuration (latched at start) ----------------
  logic       inv_r, q_r;
  logic [1:0] nc_r;
  logic [3:0] slot_r;
  assign rslot = slot_r;
  assign wslot = slot_r;

  // ---------------- schedule ----------------
  logic       run;
  logic [2:0] li;          // layer 0..6
  logic [5:0] p;           // pair index in layer
  logic [5:0] pmax;
  logic [3:0] logl;        // log2(len) of the current layer
  always_comb begin
    pmax = (nc_r == 2'd0) ? 6'd15 : (nc_r == 2'd1) ? 6'd31 : 6'd63;
    // forward: logL = logn-1-li ; inverse: logL = logn-7+li ; logn = 8+nc
    logl = inv_r ? (4'd1 + {2'b0, nc_r} + {1'b0, li}) : (4'd7 + {2'b0, nc_r} - {1'b0, li});
  end

  logic last_pair, last_layer, issue;
  logic [1:0] bubble;
  assign issue      = run & (bubble == 2'd0);
  assign last_pair  = (p == pmax);
  assign last_layer = (li == 3'd6);

  // pair -> words and zeta indices
  logic [6:0] w0, w1;
  logic [6:0] zi0, zi1, zi2, zi3;
  always_comb begin
    logic [2:0] s;
    logic [6:0] pe, zb, lo_mask;
    pe = {1'b0, p};
    // forward: k = 2^li + g ; inverse: k = 128 - 2^(7-li) + g
    zb = inv_r ? (7'(8'd128 - (8'd128 >> li))) : (7'd1 << li);
    s  = logl[2:0] - 3'd3;
    lo_mask = (7'd1 << s) - 7'd1;
    zi0 = '0; zi1 = '0; zi2 = '0; zi3 = '0;
    if (logl >= 4'd3) begin
      w0  = ((pe & ~lo_mask) << 1) | (pe & lo_mask);
      w1  = w0 | (7'd1 << s);
      zi0 = zb + (w0 >> (s + 3'd1));
    end else begin
      w0 = {pe[5:0], 1'b0};
      w1 = {pe[5:0], 1'b1};
      if (logl == 4'd2) begin
        zi0 = zb + w0;
        zi1 = zb + w1;
      end else begin
        zi0 = zb + {w0[5:0], 1'b0};
        zi1 = zb + {w0[5:0], 1'b1};
        zi2 = zb + {w1[5:0], 1'b0};
        zi3 = zb + {w1[5:0], 1'b1};
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run <= 1'b0; li <= '0; p <= '0; busy <= 1'b0; bubble <= '0;
      inv_r <= 1'b0; q_r <= 1'b0; nc_r <= '0; slot_r <= '0;
    end else begin
      if (start && !busy) begin
        busy <= 1'b1; run <= 1'b1; li <= '0; p <= '0; bubble <= '0;
        inv_r <= inv; q_r <= p_q769(pcode); nc_r <= p_ncode(pcode); slot_r <= slot;
      end else begin
        if (bubble != 2'd0) bubble <= bubble - 2'd1;
        if (issue) begin
          if (last_pair) begin
            p  <= '0;
            li <= li + 3'd1;
            if (last_layer) run <= 1'b0;
            if (nc_r == 2'd0) bubble <= 2'd2;   // N=256: idle cycles between layers (RAW distance)
          end else begin
            p <= p + 6'd1;
          end
        end
        if (done) busy <= 1'b0;
      end
    end
  end

  // read issue: w0 and w1 are always in different banks
  logic b0_is_w0;
  assign b0_is_w0 = ~(^w0);
  always_comb begin
    ren      = issue ? 2'b11 : 2'b00;
    raddr[0] = b0_is_w0 ? w0[6:1] : w1[6:1];
    raddr[1] = b0_is_w0 ? w1[6:1] : w0[6:1];
  end

  // ---------------- pipeline ----------------
  // metadata shift register: index 0 = registered at the end of the issue cycle (c0)
  // stage at cycle c(n+1) sees meta index n.
  typedef struct packed {
    logic       last;   // last pair of the transform
    logic       sc;     // inverse last layer (apply fn[127])
    logic       b0w0;
    logic [1:0] lm;     // lane map: 0 logL>=3, 1 logL==2, 2 logL==1
    logic [6:0] w0, w1;
  } meta_t;
  logic [LAT-1:0] vld;
  meta_t          meta [0:LAT-1];
  logic [6:0]     zi_r [0:3];
  logic signed [15:0] z_r [0:3];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) vld <= '0;
    else        vld <= {vld[LAT-2:0], issue};
  end
  always_ff @(posedge clk) begin
    meta[0].last <= last_pair & last_layer;
    meta[0].sc   <= inv_r & last_layer;
    meta[0].b0w0 <= b0_is_w0;
    meta[0].lm   <= (logl >= 4'd3) ? 2'd0 : (logl == 4'd2) ? 2'd1 : 2'd2;
    meta[0].w0   <= w0;
    meta[0].w1   <= w1;
    for (int i = 1; i < LAT; i++) meta[i] <= meta[i-1];
    zi_r[0] <= zi0; zi_r[1] <= zi1; zi_r[2] <= zi2; zi_r[3] <= zi3;
    // c1: zeta ROM lookup
    for (int i = 0; i < 4; i++)
      z_r[i] <= q_r ? (inv_r ? zeta769_fn(zi_r[i]) : zeta769_f(zi_r[i]))
                    : (inv_r ? zeta257_fn(zi_r[i]) : zeta257_f(zi_r[i]));
  end

  // ---- S1 (c2): read data valid, lane mapping, inverse pre-add/sub ----
  logic [15:0] W0 [0:7], W1 [0:7];
  logic [15:0] X [0:7], Y [0:7];
  logic signed [15:0] Z [0:7];
  always_comb begin
    logic [127:0] a0, a1;
    a0 = meta[1].b0w0 ? rdata[0] : rdata[1];
    a1 = meta[1].b0w0 ? rdata[1] : rdata[0];
    for (int k = 0; k < 8; k++) begin
      W0[k] = a0[16*k +: 16];
      W1[k] = a1[16*k +: 16];
    end
    for (int k = 0; k < 8; k++) begin
      case (meta[1].lm)
        2'd0: begin X[k] = W0[k]; Y[k] = W1[k]; Z[k] = z_r[0]; end
        2'd1: begin
          if (k < 4) begin X[k] = W0[k];   Y[k] = W0[k+4]; Z[k] = z_r[0]; end
          else       begin X[k] = W1[k-4]; Y[k] = W1[k];   Z[k] = z_r[1]; end
        end
        default: begin
          // logL==1: lanes 0..3 from W0, 4..7 from W1; lane k = butterfly (e, e+2),
          // e = 4*((k>>1)&1) + (k&1) inside its word, zeta index k>>1
          case (k)
            0: begin X[k] = W0[0]; Y[k] = W0[2]; Z[k] = z_r[0]; end
            1: begin X[k] = W0[1]; Y[k] = W0[3]; Z[k] = z_r[0]; end
            2: begin X[k] = W0[4]; Y[k] = W0[6]; Z[k] = z_r[1]; end
            3: begin X[k] = W0[5]; Y[k] = W0[7]; Z[k] = z_r[1]; end
            4: begin X[k] = W1[0]; Y[k] = W1[2]; Z[k] = z_r[2]; end
            5: begin X[k] = W1[1]; Y[k] = W1[3]; Z[k] = z_r[2]; end
            6: begin X[k] = W1[4]; Y[k] = W1[6]; Z[k] = z_r[3]; end
            default: begin X[k] = W1[5]; Y[k] = W1[7]; Z[k] = z_r[3]; end
          endcase
        end
      endcase
    end
  end

  logic [15:0]        s1_p [0:7];   // fwd: x ; inv: x+y
  logic signed [15:0] s1_m [0:7];   // fwd: y ; inv: x-y
  logic signed [15:0] s1_z [0:7];
  always_ff @(posedge clk) begin
    for (int k = 0; k < 8; k++) begin
      s1_p[k] <= inv_r ? (X[k] + Y[k]) : X[k];
      s1_m[k] <= inv_r ? (X[k] - Y[k]) : Y[k];
      s1_z[k] <= Z[k];
    end
  end

  // ---- S2 (c3): multiply ----
  logic [15:0]        s2_p [0:7];
  logic signed [31:0] s2_m [0:7];
  always_ff @(posedge clk) begin
    for (int k = 0; k < 8; k++) begin
      s2_p[k] <= s1_p[k];
      s2_m[k] <= s1_z[k] * s1_m[k];
    end
  end

  // ---- S3 (c4): Montgomery reduction, first half ----
  logic [15:0] s3_p [0:7];
  mrd_t        s3_d [0:7];
  always_ff @(posedge clk) begin
    for (int k = 0; k < 8; k++) begin
      s3_p[k] <= s2_p[k];
      s3_d[k] <= mred_d(s2_m[k], q_r);
    end
  end

  // ---- S4 (c5): Montgomery reduction, second half + butterfly ----
  // fwd: a = p + t, b = p - t ; inv: a = p, b = t   (t = fqmul result, each one CSA tree + adder)
  logic [15:0] s4_a [0:7], s4_b [0:7];
  always_ff @(posedge clk) begin
    for (int k = 0; k < 8; k++) begin
      logic [15:0] sa, sb;
      sa = mred_f_add(inv_r ? 16'd0 : s3_p[k], s3_d[k], q_r);
      sb = mred_f_sub(s3_p[k], s3_d[k], q_r);
      s4_a[k] <= inv_r ? s3_p[k] : sa;
      s4_b[k] <= inv_r ? sa : sb;
    end
  end

  // ---- S5 (c6): fn[127] scaling product (inverse last layer), else pass-through ----
  // fn[127] = 255 (q=257 sets) / 655 (512-Star): plc_ntt_pwalu_pkg::mul_fn127 (CSA + one adder)
  logic signed [31:0] s5_ma [0:7], s5_mb [0:7];
  always_ff @(posedge clk) begin
    for (int k = 0; k < 8; k++) begin
      s5_ma[k] <= meta[5].sc ? mul_fn127(s4_a[k], q_r) : {16'd0, s4_a[k]};
      s5_mb[k] <= meta[5].sc ? mul_fn127(s4_b[k], q_r) : {16'd0, s4_b[k]};
    end
  end

  // ---- S6 (c7): Montgomery first half of the scaling; pass-through values otherwise ----
  mrd_t s6_da [0:7], s6_db [0:7];
  logic [15:0] s6_p [0:7], s6_q [0:7];
  always_ff @(posedge clk) begin
    for (int k = 0; k < 8; k++) begin
      s6_da[k] <= mred_d(s5_ma[k], q_r);
      s6_db[k] <= mred_d(s5_mb[k], q_r);
      s6_p[k]  <= meta[6].sc ? 16'd0 : s5_ma[k][15:0];   // pass-through values (sc = 0)
      s6_q[k]  <= meta[6].sc ? 16'd0 : s5_mb[k][15:0];
    end
  end

  // ---- S7 (c8): finish, assemble output words ----
  logic [127:0] o0, o1;
  always_ff @(posedge clk) begin
    logic [15:0] A [0:7], B [0:7];
    logic [15:0] P0 [0:7], P1 [0:7];
    for (int k = 0; k < 8; k++) begin
      A[k] = meta[7].sc ? mred_f(s6_da[k], q_r) : s6_p[k];
      B[k] = meta[7].sc ? mred_f(s6_db[k], q_r) : s6_q[k];
    end
    case (meta[7].lm)
      2'd0: begin
        for (int k = 0; k < 8; k++) begin P0[k] = A[k]; P1[k] = B[k]; end
      end
      2'd1: begin
        for (int k = 0; k < 4; k++) begin
          P0[k] = A[k];   P0[k+4] = B[k];
          P1[k] = A[k+4]; P1[k+4] = B[k+4];
        end
      end
      default: begin
        P0[0] = A[0]; P0[2] = B[0]; P0[1] = A[1]; P0[3] = B[1];
        P0[4] = A[2]; P0[6] = B[2]; P0[5] = A[3]; P0[7] = B[3];
        P1[0] = A[4]; P1[2] = B[4]; P1[1] = A[5]; P1[3] = B[5];
        P1[4] = A[6]; P1[6] = B[6]; P1[5] = A[7]; P1[7] = B[7];
      end
    endcase
    for (int k = 0; k < 8; k++) begin
      o0[16*k +: 16] <= P0[k];
      o1[16*k +: 16] <= P1[k];
    end
  end

  // ---- write (c9) ----
  meta_t mw;
  assign mw = meta[LAT-1];
  always_comb begin
    wen      = vld[LAT-1] ? 2'b11 : 2'b00;
    waddr[0] = mw.b0w0 ? mw.w0[6:1] : mw.w1[6:1];
    waddr[1] = mw.b0w0 ? mw.w1[6:1] : mw.w0[6:1];
    wdata[0] = mw.b0w0 ? o0 : o1;
    wdata[1] = mw.b0w0 ? o1 : o0;
  end
  assign done = vld[LAT-1] & mw.last;

endmodule
