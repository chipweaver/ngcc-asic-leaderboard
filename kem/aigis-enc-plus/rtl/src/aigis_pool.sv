// SM3 core pool (16 cores): gen_a uses cores 0..11 (quarter q: cores 3q..3q+2), noise uses its
// four dedicated cores 12..15 and, in wide mode (no gen_a job active when the noise job starts),
// also cores 0..11.  A noise worker starts a block on a core only when the core is idle, so
// speculative gen_a blocks still running after gen_a completed finish unobserved.
module aigis_pool (
  input  logic          clk,
  input  logic          rst_n,
  input  logic [1:0]    param,
  // gen_a job
  input  logic          ga_start,
  input  logic [511:0]  ga_seed,
  output logic          ga_busy,
  output logic          ga_done,
  output logic [3:0]    ga_qdone,      // sticky per quarter (cleared by ga_start)
  output logic [3:0][9:0] ga_qprog,    // coefficients written per quarter
  output logic [31:0]          ga_we,
  output logic [31:0][5:0]     ga_wa,
  output logic [31:0][11:0]    ga_wd,
  // noise job
  input  logic          nx_start,
  input  logic [2:0]    nx_eta,
  input  logic [7:0]    nx_nonce,
  input  logic [511:0]  nx_seed,
  output logic          nx_busy,
  output logic          nx_done,
  output logic [31:0]          nx_re,
  output logic [31:0][5:0]     nx_ra,
  input  logic [31:0][11:0]    nx_rd,
  output logic [31:0]          nx_we,
  output logic [31:0][5:0]     nx_wa,
  output logic [31:0][11:0]    nx_wd
);
  localparam logic [255:0] IV = 256'h7380166f4914b2b9172442d7da8a0600a96f30bc163138aae38dee4db0fb0e4e;
  logic [15:0] c_s, c_c, c_b, c_d, c_l;
  logic [15:0][255:0] c_iv, c_v;
  logic [15:0][511:0] c_blk;
  for (genvar k = 0; k < 16; k++) begin : g_core
    aigis_sm3_core u_core (.clk(clk), .rst_n(rst_n), .start(c_s[k]), .cont(c_c[k]), .iv(c_iv[k]), .blk(c_blk[k]),
                           .busy(c_b[k]), .done(c_d[k]), .lastc(c_l[k]), .vout(c_v[k]));
  end

  // ---------------------------------------------------------------- gen_a quarters
  logic [3:0] q_busy, q_done;
  logic [3:0][2:0] q_cs, q_cc;
  logic [3:0][2:0][511:0] q_blk;
  logic [3:0][7:0] q_we;
  logic [3:0][7:0][5:0] q_wa;
  logic [3:0][7:0][11:0] q_wd;
  logic gstart;
  assign gstart = ga_start && !ga_busy;
  for (genvar q = 0; q < 4; q++) begin : g_q
    aigis_drng_q #(.QI(2'(q))) u_q (
      .clk, .rst_n, .start(gstart), .param, .seed(ga_seed), .busy(q_busy[q]), .done(q_done[q]), .wprog(ga_qprog[q]),
      .cs(q_cs[q]), .cc(q_cc[q]), .cblk(q_blk[q]),
      .cbusy(c_b[3*q +: 3]), .cdone(c_d[3*q +: 3]), .clast(c_l[3*q +: 3]), .cv(c_v[3*q +: 3]),
      .we(q_we[q]), .wa(q_wa[q]), .wd(q_wd[q]));
  end
  logic ga_run;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin ga_run <= 1'b0; ga_done <= 1'b0; ga_qdone <= '0; end
    else begin
      ga_done <= 1'b0;
      if (gstart) begin ga_run <= 1'b1; ga_qdone <= '0; end
      else if (ga_run) begin
        ga_qdone <= ga_qdone | q_done;
        if ((ga_qdone | q_done) == 4'hf) begin ga_run <= 1'b0; ga_done <= 1'b1; end
      end
    end
  end
  assign ga_busy = ga_run;
  always_comb begin
    for (int q = 0; q < 4; q++)
      for (int i = 0; i < 8; i++) begin
        ga_we[8*q + i] = q_we[q][i]; ga_wa[8*q + i] = q_wa[q][i]; ga_wd[8*q + i] = q_wd[q][i];
      end
  end

  // ---------------------------------------------------------------- noise
  logic [15:0] n_cs;
  logic [15:0][255:0] n_iv;
  logic [15:0][511:0] n_blk;
  // noise may use core c: dedicated cores 12..15 always, cores of gen_a while gen_a is idle or once
  // their quarter has completed (a noise block starts only on an idle core)
  logic [15:0] cav;
  always_comb
    for (int c = 0; c < 16; c++)
      cav[c] = (c >= 12) || (!ga_run && !ga_start) || (ga_run && ga_qdone[c / 3]);
  aigis_noise u_nx (
    .clk, .rst_n, .start(nx_start && !nx_busy), .param, .eta(nx_eta), .nonce(nx_nonce), .seed(nx_seed),
    .cav(cav), .busy(nx_busy), .done(nx_done),
    .cs(n_cs), .civ(n_iv), .cblk(n_blk), .cbusy(c_b), .cdone(c_d), .cv(c_v),
    .re(nx_re), .ra(nx_ra), .rd(nx_rd), .we(nx_we), .wa(nx_wa), .wd(nx_wd));

  always_comb begin
    for (int q = 0; q < 4; q++)
      for (int j = 0; j < 3; j++) begin
        logic nown;
        nown = cav[3*q + j];
        c_s[3*q + j]   = nown ? n_cs[3*q + j] : q_cs[q][j];
        c_c[3*q + j]   = nown ? 1'b0 : q_cc[q][j];
        c_iv[3*q + j]  = nown ? n_iv[3*q + j] : IV;
        c_blk[3*q + j] = nown ? n_blk[3*q + j] : q_blk[q][j];
      end
    for (int k = 12; k < 16; k++) begin
      c_s[k] = n_cs[k]; c_c[k] = 1'b0; c_iv[k] = n_iv[k]; c_blk[k] = n_blk[k];
    end
  end
`ifndef SYNTHESIS
  // invariant: gen_a never starts while a noise job runs (noise jobs follow gen_a in Enc/Dec and
  // start with it in KeyGen)
  always @(posedge clk) if (gstart && nx_busy)
    $error("pool: gen_a started during a noise job");
`endif
endmodule
