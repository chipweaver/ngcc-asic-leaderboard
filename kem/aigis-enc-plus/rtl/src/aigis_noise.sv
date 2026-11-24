// Noise polynomial: CBD_eta(KDF(seed || nonce, eta*n/4)) into one slot (poly.c getnoise, cbd.c).
// KDF = pseudoXOF: output block ctr = SM3(seed || nonce || ctr_be32).  For b = 64 the first message
// block is the seed itself; its midstate MS is computed once per job on core 12.
// The noise bytes split into region 1 and region 2 (cbd3 = cbd1 + cbd2, cbd5 = cbd4 + cbd1,
// cbd6 = cbd4 + cbd2, cbd2 alone).  A block of region R covers S = 32/64/128 consecutive
// coefficients (cbd4/cbd2/cbd1).  Region-1 blocks are written, region-2 blocks are added
// (read-modify-write) once every region-1 block is written.
// Blocks are numbered o = 0 .. NB1+NB2-1 (region 1 first).  For S <= W block o of a region covers
// lane (o mod 16), words (o div 16)*S ..; for S > W (set I) it covers lanes o*S/W .. o*S/W+S/W-1.
//
// Sixteen sub-workers, sub-worker c on pool core c (cores >= 12 always, others while cav[c]).
// A sub-worker takes the next block on an idle core (one block start per cycle), copies the digest
// into its shift register when the core finishes, locks the block's lanes and writes two
// coefficients per lane and cycle; the core may compute the next block meanwhile (its digest waits
// on the core output until the register is free).
module aigis_noise (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          start,
  input  logic [1:0]    param,
  input  logic [2:0]    eta,
  input  logic [7:0]    nonce,
  input  logic [511:0]  seed,          // seed byte j at bits [8j+7:8j]
  input  logic [15:0]   cav,           // pool core c may be used by noise
  output logic          busy,
  output logic          done,
  // pool cores 0..15
  output logic [15:0]          cs,
  output logic [15:0][255:0]   civ,
  output logic [15:0][511:0]   cblk,
  input  logic [15:0]          cbusy, cdone,
  input  logic [15:0][255:0]   cv,
  // slot port (read role for RMW, write role)
  output logic [31:0]         re,
  output logic [31:0][5:0]    ra,
  input  logic [31:0][11:0]   rd,
  output logic [31:0]         we,
  output logic [31:0][5:0]    wa,
  output logic [31:0][11:0]   wd
);
  import aigis_pkg::*;
  localparam logic [255:0] IV = 256'h7380166f4914b2b9172442d7da8a0600a96f30bc163138aae38dee4db0fb0e4e;

  logic [1:0] p;
  logic [2:0] et;
  logic [7:0] nc;
  logic [6:0] b;
  logic [2:0] lw;
  logic [7:0] W;
  assign b  = bbytes(p);
  assign lw = 3'd5 + {1'b0, p - 2'd1};
  assign W  = wcount(p);
  typedef enum logic [1:0] {N_IDLE, N_MS, N_RUN} nst_t;
  nst_t st;
  logic [7:0] sbt [0:63];
  always_comb for (int j = 0; j < 64; j++) sbt[j] = seed[8*j +: 8];

  // ------------------------------------------------------------- region geometry
  // kind: 1 cbd1, 2 cbd2, 4 cbd4; S coefficients per block; NB blocks
  logic [2:0] k1, k2;
  logic [6:0] nb1, nb2, nbt;
  // geometry of the job, registered at start
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin k1 <= 3'd2; k2 <= 3'd2; nb1 <= '0; nb2 <= '0; nbt <= '0; end
    else if (start && st == N_IDLE) begin
      logic [6:0] n32, n64, n128;
      n32  = 7'd16 << (param - 2'd1);
      n64  = 7'd8  << (param - 2'd1);
      n128 = 7'd4  << (param - 2'd1);
      case (eta)
        3'd2: begin k1 <= 3'd2; nb1 <= n64;  k2 <= 3'd2; nb2 <= 7'd0; nbt <= n64; end
        3'd3: begin k1 <= 3'd1; nb1 <= n128; k2 <= 3'd2; nb2 <= n64;  nbt <= n128 + n64; end
        3'd5: begin k1 <= 3'd4; nb1 <= n32;  k2 <= 3'd1; nb2 <= n128; nbt <= n32 + n128; end
        default: begin k1 <= 3'd4; nb1 <= n32; k2 <= 3'd2; nb2 <= n64; nbt <= n32 + n64; end
      endcase
    end
  end
  function automatic logic [7:0] s_of(input logic [2:0] k);   // coefficients per block
    s_of = (k == 3'd1) ? 8'd128 : (k == 3'd2) ? 8'd64 : 8'd32;
  endfunction
  function automatic logic [2:0] ls_of(input logic [2:0] k);   // log2 S
    ls_of = (k == 3'd1) ? 3'd7 : (k == 3'd2) ? 3'd6 : 3'd5;
  endfunction
  function automatic logic [3:0] bpc_of(input logic [2:0] k); // bits per coefficient
    bpc_of = (k == 3'd1) ? 4'd2 : (k == 3'd2) ? 4'd4 : 4'd8;
  endfunction

  // message block for ctr
  function automatic logic [511:0] msg(input logic [15:0] ctr);
    logic [511:0] r; int o;
    r = '0;
    if (b != 7'd64) begin
      for (int j = 0; j < 32; j++) if (j < int'(b)) r[511 - 8*j -: 8] = sbt[j];
      o = int'(b);
    end else o = 0;
    r[511 - 8*o -: 8] = nc;
    r[511 - 8*(o+1) -: 32] = {16'd0, ctr};
    r[511 - 8*(o+5) -: 8] = 8'h80;
    r[63:0] = 64'(8 * (int'(b) + 5));
    msg = r;
  endfunction
  function automatic logic [511:0] seedblk();
    logic [511:0] r;
    for (int j = 0; j < 64; j++) r[511 - 8*j -: 8] = sbt[j];
    seedblk = r;
  endfunction
  // ctr of block o (KDF output block index, first block ctr = 1)
  function automatic logic [15:0] ctr_of(input logic [6:0] o);
    logic [6:0]  ol;
    logic [2:0]  kk;
    logic [15:0] base;
    logic [7:0]  S;
    if (o < nb1) begin ol = o; kk = k1; base = 16'd0; end
    else begin ol = o - nb1; kk = k2; base = 16'(nb1); end
    S = s_of(kk);
    if (S <= W) ctr_of = (16'(ol & 7'd15) << (lw - ls_of(kk))) + 16'(ol >> 4) + 16'd1 + base;
    else        ctr_of = 16'(ol) + 16'd1 + base;
  endfunction

  // ------------------------------------------------------------- control
  logic [255:0] ms;
  logic [6:0]   nxt;        // next block to start
  logic [6:0]   nw1;        // region-1 blocks written
  logic [6:0]   nwt;        // blocks written
  logic         ms_pend, ms_run;

  // ------------------------------------------------------------- sub-workers
  typedef enum logic [1:0] {W_FREE, W_HOLD, W_WR} wst_t;  // register: free / digest waiting for lanes / writing
  wst_t [15:0]       ws;
  logic [15:0]       crun;        // core c runs a noise block
  logic [15:0][6:0]  co;          // block on core c
  logic [15:0][255:0] dr;         // digest register (byte j at [8j+7:8j])
  logic [15:0][6:0]  ro;          // block in the register
  logic [15:0][6:0]  wi;          // write step
  logic [15:0]       r2;          // register block is region 2
  logic [15:0][15:0] lmask;       // lanes of the register block
  logic [15:0]       lock;        // lanes locked by writers

  // geometry of a block
  function automatic logic [15:0] lanes_of(input logic [6:0] o);
    logic [6:0] ol; logic [2:0] kk; logic [7:0] S; logic [15:0] m; logic [3:0] l0;
    if (o < nb1) begin ol = o; kk = k1; end else begin ol = o - nb1; kk = k2; end
    S = s_of(kk);
    if (S <= W) lanes_of = 16'd1 << (ol & 7'd15);
    else begin
      l0 = 4'(ol << (ls_of(kk) - lw));
      m = (ls_of(kk) - lw == 3'd1) ? 16'd3 : 16'd15;
      lanes_of = m << l0;
    end
  endfunction

  // block starts, decided one cycle ahead: every sub-worker whose core is available, idle and not
  // already starting takes a block (consecutive numbers in core order) while blocks remain; the
  // block number is registered and the core start (cs) follows in the next cycle
  logic [15:0] can_go, go, pend;
  logic [15:0][6:0] gob;          // block number for a starting core
  logic [6:0]  ngo;
  logic [15:0][4:0] rank;
  always_comb begin
    for (int c = 0; c < 16; c++)
      can_go[c] = (st == N_RUN) && cav[c] && !cbusy[c] && !crun[c] && !(ms_run && c == 12);
    for (int c = 0; c < 16; c++) begin
      rank[c] = '0;
      for (int k = 0; k < c; k++) rank[c] = rank[c] + {4'd0, can_go[k]};
      gob[c] = nxt + 7'(rank[c]);
      go[c] = can_go[c] && (gob[c] < nbt);
    end
    ngo = '0;
    for (int c = 0; c < 16; c++) ngo = ngo + {6'd0, go[c]};
  end

  // core done -> register (when free)
  logic [15:0] take;
  always_comb for (int c = 0; c < 16; c++) take[c] = crun[c] && !pend[c] && !cbusy[c] && (ws[c] == W_FREE);

  // writer start: lanes free (lowest index first among conflicting), region-2 waits for region 1
  logic [15:0] wgo, wreq;
  always_comb begin
    for (int c = 0; c < 16; c++) wreq[c] = (ws[c] == W_HOLD) && ((lmask[c] & lock) == '0) && (!r2[c] || nw1 == nb1);
    for (int c = 0; c < 16; c++) begin
      wgo[c] = wreq[c];
      for (int d = 0; d < c; d++) if (wreq[d] && ((lmask[d] & lmask[c]) != '0)) wgo[c] = 1'b0;
    end
  end

  // per-writer step count (S / (2 * lanes)), stored with the digest
  logic [15:0][6:0] wsteps;
  logic [15:0]       wlast;
  always_comb for (int c = 0; c < 16; c++) wlast[c] = (wi[c] + 7'd1 == wsteps[c]);
  function automatic logic [6:0] steps_of(input logic [6:0] o);
    logic [2:0] kk;
    kk = (o < nb1) ? k1 : k2;
    // S <= W: S/2; S > W (set I): W/2
    steps_of = (ls_of(kk) <= lw) ? (7'd1 << (ls_of(kk) - 3'd1)) : (7'd1 << (lw - 3'd1));
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= N_IDLE; busy <= 1'b0; done <= 1'b0; p <= 2'd1; et <= 3'd2; nc <= '0; ms <= '0;
      nxt <= '0; nw1 <= '0; nwt <= '0; ms_pend <= 1'b0; ms_run <= 1'b0;
      for (int c = 0; c < 16; c++) ws[c] <= W_FREE; crun <= '0; pend <= '0; co <= '0; dr <= '0; wsteps <= '0; ro <= '0; wi <= '0; r2 <= '0; lmask <= '0; lock <= '0;
    end else begin
      done <= 1'b0;
      case (st)
        N_IDLE: if (start) begin
          busy <= 1'b1; p <= param; et <= eta; nc <= nonce; nxt <= '0; nw1 <= '0; nwt <= '0;
          for (int c = 0; c < 16; c++) ws[c] <= W_FREE; crun <= '0; lock <= '0;
          if (param == 2'd3) begin st <= N_MS; ms_pend <= 1'b1; end
          else st <= N_RUN;
        end
        N_MS: begin
          if (ms_pend && !cbusy[12]) begin ms_pend <= 1'b0; ms_run <= 1'b1; end
          if (ms_run && cdone[12]) begin ms <= cv[12]; ms_run <= 1'b0; st <= N_RUN; end
        end
        N_RUN: begin
          if (nwt == nbt) begin st <= N_IDLE; busy <= 1'b0; done <= 1'b1; end
        end
        default: st <= N_IDLE;
      endcase
      // block start
      pend <= go;
      for (int c = 0; c < 16; c++) if (go[c]) begin crun[c] <= 1'b1; co[c] <= gob[c]; end
      if (st == N_RUN) nxt <= nxt + ngo;
      for (int c = 0; c < 16; c++) begin
        // digest to register
        if (take[c]) begin
          crun[c] <= 1'b0; ws[c] <= W_HOLD; ro[c] <= co[c]; r2[c] <= (co[c] >= nb1);
          lmask[c] <= lanes_of(co[c]); wi[c] <= '0; wsteps[c] <= steps_of(co[c]);
          for (int j = 0; j < 32; j++) dr[c][8*j +: 8] <= cv[c][255 - 8*j -: 8];
        end
        if (wgo[c]) ws[c] <= W_WR;
        if (ws[c] == W_WR) begin
          logic [2:0] kk; logic [3:0] bpc; logic [7:0] S;
          kk = r2[c] ? k2 : k1; bpc = bpc_of(kk); S = s_of(kk);
          dr[c] <= dr[c] >> {bpc, 1'b0};
          wi[c] <= wi[c] + 7'd1;
          if (wlast[c]) ws[c] <= W_FREE;
        end
      end
      // lanes and counters
      begin
        logic [15:0] lk; logic [6:0] d1, dt;
        lk = lock; d1 = '0; dt = '0;
        for (int c = 0; c < 16; c++) begin
          if (wgo[c]) lk = lk | lmask[c];
          if (ws[c] == W_WR && wlast[c]) begin
            lk = lk & ~lmask[c]; dt = dt + 7'd1; if (!r2[c]) d1 = d1 + 7'd1;
          end
        end
        lock <= lk;
        if (st == N_IDLE && start) lock <= '0;
        else begin nw1 <= nw1 + d1; nwt <= nwt + dt; end
      end
    end
  end

  // ------------------------------------------------------------- core interface
  always_comb begin
    cs = '0;
    for (int c = 0; c < 16; c++) begin
      civ[c]  = (b == 7'd64) ? ms : IV;
      cblk[c] = msg(ctr_of(co[c]));
      cs[c]   = pend[c];
    end
    if (st == N_MS) begin cs[12] = ms_pend && !cbusy[12]; civ[12] = IV; cblk[12] = seedblk(); end
  end

  // ------------------------------------------------------------- slot writes
  // writer c in W_WR, step i: lanes of block ro; per lane two coefficients 2i, 2i+1 of its part.
  // region 1: direct write; region 2: read at step i, add and write one cycle later.
  logic [15:0]       rv;          // RMW read issued last cycle
  logic [15:0][6:0]  rwi;
  logic [15:0][15:0][1:0][3:0] rval_q;  // values read-modified next cycle (lane, coeff) as signed CBD
  function automatic logic [3:0] cbdv(input logic [7:0] bits, input logic [2:0] kk);
    logic [3:0] r;
    case (kk)
      3'd1: r = 4'($signed({3'd0, bits[0]}) - $signed({3'd0, bits[1]}));
      3'd2: r = 4'($signed({3'd0, bits[0]}) + $signed({3'd0, bits[1]}) - $signed({3'd0, bits[2]}) - $signed({3'd0, bits[3]}));
      default: r = 4'($signed({1'b0, 3'({2'd0, bits[0]} + {2'd0, bits[1]} + {2'd0, bits[2]} + {2'd0, bits[3]})})
                    - $signed({1'b0, 3'({2'd0, bits[4]} + {2'd0, bits[5]} + {2'd0, bits[6]} + {2'd0, bits[7]})}));
    endcase
    cbdv = r;
  endfunction
  function automatic logic [11:0] modq4(input logic [3:0] v);
    modq4 = v[3] ? 12'(13'd3329 + {9'h1ff, v}) : {8'd0, v};
  endfunction

  // per writer: the CBD values of its (up to four) lane parts; part j of a block that spans
  // nlw = S/W lanes (set I only) starts at digest bit j*W*bpc and lands in lane l with l mod nlw = j
  logic [15:0][3:0][1:0][3:0] pv;        // writer, part, coefficient
  logic [15:0][6:0]           pw;        // first word of the pair (same in every lane of the block)
  logic [15:0]                wmul;      // writer's block spans several lanes
  logic [15:0][1:0]           wnl;       // lane-part mask: 0 (one lane), 1 (two lanes), 3 (four lanes)
  always_comb begin
    for (int c = 0; c < 16; c++) begin
      logic [2:0] kk; logic [7:0] S; logic [6:0] ol;
      kk = r2[c] ? k2 : k1; S = s_of(kk);
      ol = r2[c] ? ro[c] - nb1 : ro[c];
      wmul[c] = (S > W);
      wnl[c]  = !wmul[c] ? 2'd0 : (kk == 3'd1) ? 2'd3 : 2'd1;
      pw[c]   = wmul[c] ? {wi[c][5:0], 1'b0} : 7'((7'(ol >> 4) << ls_of(kk)) + {wi[c], 1'b0});
      for (int j = 0; j < 4; j++) begin
        logic [7:0] b0, b1;
        // fixed offsets: cbd1 in set I 64 bits per lane part, cbd2 in set I 128 bits per part
        if (!wmul[c] || j == 0) begin b0 = dr[c][7:0]; b1 = dr[c][15:0] >> bpc_of(kk); end
        else if (kk == 3'd1) begin b0 = dr[c][64*j +: 8]; b1 = dr[c][64*j + 2 +: 8]; end
        else begin b0 = dr[c][(128*j) % 256 +: 8]; b1 = dr[c][(128*j) % 256 + 4 +: 8]; end
        pv[c][j][0] = cbdv(b0, kk);
        pv[c][j][1] = cbdv(b1, kk);
      end
    end
  end
  logic [15:0][15:0]           wl_on;     // writer c writes lane l this cycle
  always_comb for (int c = 0; c < 16; c++) for (int l = 0; l < 16; l++) wl_on[c][l] = (ws[c] == W_WR) && lmask[c][l];
  // merge (lanes are exclusive between writers)
  logic [15:0]        l_on, l_r2;
  logic [15:0][6:0]   l_w;
  logic [15:0][1:0][3:0] l_v;
  always_comb begin
    l_on = '0; l_r2 = '0; l_w = '0; l_v = '0;
    for (int l = 0; l < 16; l++)
      for (int c = 0; c < 16; c++)
        if (wl_on[c][l]) begin
          l_on[l] = 1'b1; l_r2[l] = r2[c]; l_w[l] = pw[c];
          l_v[l] = pv[c][l & int'(wnl[c])];
        end
  end
  logic [15:0]        q_on;
  logic [15:0][6:0]   q_w;
  logic [15:0][1:0][3:0] q_v;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) q_on <= '0;
    else q_on <= l_on & l_r2;
  end
  always_ff @(posedge clk) begin q_w <= l_w; q_v <= l_v; end
  always_comb begin
    re = '0; ra = '0; we = '0; wa = '0; wd = '0;
    for (int l = 0; l < 16; l++) begin
      logic b0, qb0;
      b0 = wbank(l_w[l]); qb0 = wbank(q_w[l]);
      // region 1: direct
      if (l_on[l] && !l_r2[l]) begin
        we[2*l]     = 1'b1; wa[2*l]     = l_w[l][6:1]; wd[2*l]     = modq4(b0 ? l_v[l][1] : l_v[l][0]);
        we[2*l + 1] = 1'b1; wa[2*l + 1] = l_w[l][6:1]; wd[2*l + 1] = modq4(b0 ? l_v[l][0] : l_v[l][1]);
      end
      // region 2: read now, add next cycle
      if (l_on[l] && l_r2[l]) begin
        re[2*l] = 1'b1; re[2*l + 1] = 1'b1; ra[2*l] = l_w[l][6:1]; ra[2*l + 1] = l_w[l][6:1];
      end
      if (q_on[l]) begin
        we[2*l]     = 1'b1; wa[2*l]     = q_w[l][6:1]; wd[2*l]     = addq(rd[2*l],     modq4(qb0 ? q_v[l][1] : q_v[l][0]));
        we[2*l + 1] = 1'b1; wa[2*l + 1] = q_w[l][6:1]; wd[2*l + 1] = addq(rd[2*l + 1], modq4(qb0 ? q_v[l][0] : q_v[l][1]));
      end
    end
  end
endmodule
