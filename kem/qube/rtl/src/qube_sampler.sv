// Fixed-weight sampler (reference vect_sample_fixed_weight): reads ceil(log2 n)-bit candidates
// LSB-first from the sampling XOF stream, rejects candidates >= n and duplicates of the vector being
// sampled (parallel compare against the accepted positions), and writes accepted positions in order
// to the lane support memories. A program is a list of up to four vectors sampled back to back from
// one stream; each vector has a weight and a route: 1 = SUP0, 2 = SUP1, 3 = both,
// 0 = split (first ceil(w/2) positions to SUP0, the rest to SUP1).
//   KeyGen: y(both) x1(SUP0) x2(SUP1); Enc: f1(both) f2(both) e(SUP1) g(SUP0); Dec: y(SUP1).
module qube_sampler
  import qube_pkg::*;
(
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,
  input  logic [2:0]   ps,
  input  logic [1:0]   prog,        // 0: KeyGen, 1: Enc / re-encryption, 2: Dec y
  output logic         busy,
  output logic         done,        // one-cycle pulse when the last vector is complete
  // sampling stream (XOF farm context 0)
  input  logic         svalid,
  output logic         sready,
  input  logic [255:0] sdata,
  // support writes
  output logic [1:0]   sup_we,
  output logic [17:0]  sup_wdata
);
  logic [17:0] n_r;
  logic [4:0]  nbits_r;
  logic [2:0]  nvec_r;
  logic [8:0]  w_r   [0:3];
  logic [1:0]  rt_r  [0:3];

  logic        run;
  logic [2:0]  vidx;
  logic [8:0]  cnt;
  logic [WMAX-1:0] vmask;
  logic [17:0] cam [0:WMAX-1];

  // bit window
  logic [255:0] b0, b1;
  logic         b0v, b1v;
  logic [8:0]   ptr;          // 0..255 (+ up to 17 before wrap)

  // stage 1 candidate
  logic [17:0]  cand;
  logic         cv;

  // -------- extraction
  logic [511:0] win;
  logic [17:0]  raw;
  logic         can_ext;
  logic [9:0]   nptr;
  always_comb begin
    win  = {b1, b0} >> ptr;
    raw  = win[17:0] & ((18'd1 << nbits_r) - 18'd1);
    nptr = {1'b0, ptr} + {5'd0, nbits_r};
    can_ext = run && b0v && ((nptr <= 10'd256) || b1v);
  end
  logic         shf, v0n, v1n;
  logic [255:0] d0n;
  always_comb begin
    shf = can_ext && (nptr >= 10'd256);
    v0n = shf ? b1v : b0v;
    v1n = shf ? 1'b0 : b1v;
    d0n = shf ? b1 : b0;
  end
  assign sready = run && !(v0n && v1n);

  // -------- check
  logic        dup, acc, last_in_vec, last_vec;
  logic [8:0]  half;
  always_comb begin
    dup = 1'b0;
    for (int i = 0; i < WMAX; i++) dup = dup | (vmask[i] && cam[i] == cand);
    acc = run && cv && (cand < n_r) && !dup;
    last_in_vec = (cnt == w_r[vidx] - 9'd1);
    last_vec    = (vidx == nvec_r - 3'd1);
    half = (w_r[vidx] + 9'd1) >> 1;
    sup_wdata = cand;
    case (rt_r[vidx])
      2'd1: sup_we = {1'b0, acc};
      2'd2: sup_we = {acc, 1'b0};
      2'd3: sup_we = {acc, acc};
      default: sup_we = (cnt < half) ? {1'b0, acc} : {acc, 1'b0};
    endcase
  end

  assign busy = run;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run <= 1'b0; done <= 1'b0; vidx <= '0; cnt <= '0; vmask <= '0;
      b0 <= '0; b1 <= '0; b0v <= 1'b0; b1v <= 1'b0; ptr <= '0; cand <= '0; cv <= 1'b0;
      n_r <= '0; nbits_r <= '0; nvec_r <= '0;
      for (int i = 0; i < 4; i++) begin w_r[i] <= '0; rt_r[i] <= '0; end
      for (int i = 0; i < WMAX; i++) cam[i] <= '0;
    end else begin
      done <= 1'b0;
      if (start) begin
        run <= 1'b1; vidx <= '0; cnt <= '0; vmask <= '0;
        b0v <= 1'b0; b1v <= 1'b0; ptr <= '0; cv <= 1'b0;
        n_r <= p_n(ps); nbits_r <= p_nbits(ps);
        case (prog)
          2'd0: begin nvec_r <= 3'd3; w_r[0] <= p_wy(ps); rt_r[0] <= 2'd3; w_r[1] <= p_wx(ps); rt_r[1] <= 2'd1;
                      w_r[2] <= p_wx(ps); rt_r[2] <= 2'd2; w_r[3] <= 9'd1; rt_r[3] <= 2'd3; end
          2'd1: begin nvec_r <= 3'd4; w_r[0] <= p_wr(ps); rt_r[0] <= 2'd3; w_r[1] <= p_wr(ps); rt_r[1] <= 2'd3;
                      w_r[2] <= p_we(ps); rt_r[2] <= 2'd2; w_r[3] <= p_wg(ps); rt_r[3] <= 2'd1; end
          default: begin nvec_r <= 3'd1; w_r[0] <= p_wy(ps); rt_r[0] <= 2'd2; w_r[1] <= 9'd1; rt_r[1] <= 2'd3;
                      w_r[2] <= 9'd1; rt_r[2] <= 2'd3; w_r[3] <= 9'd1; rt_r[3] <= 2'd3; end
        endcase
      end else if (run) begin
        // window advance and refill
        cv <= can_ext;
        if (can_ext) cand <= raw;
        if (can_ext) ptr <= shf ? 9'(nptr - 10'd256) : nptr[8:0];
        if (svalid && sready) begin
          if (!v0n) begin b0 <= sdata; b0v <= 1'b1; b1v <= 1'b0; end
          else begin b0 <= d0n; b0v <= 1'b1; b1 <= sdata; b1v <= 1'b1; end
        end else begin
          b0 <= d0n; b0v <= v0n; b1v <= v1n;
        end
        // accept
        if (acc) begin
          cam[cnt] <= cand;
          if (last_in_vec) begin
            cnt <= '0; vmask <= '0;
            if (last_vec) begin run <= 1'b0; done <= 1'b1; cv <= 1'b0; end
            else vidx <= vidx + 3'd1;
          end else begin
            cnt <= cnt + 9'd1; vmask <= {vmask[WMAX-2:0], 1'b1};
          end
        end
      end
    end
  end
endmodule
