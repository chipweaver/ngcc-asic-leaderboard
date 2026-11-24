// pseudoXOF(seed || 0x05) counter-mode block farm: four SM3 cores compute consecutive blocks of one
// stream context in lockstep and deliver them in counter order. Two contexts: 0 = sampling stream
// (priority), 1 = dense stream. A context is started with its seed, parameter set, first block index
// b0 (block b = SM3(seed || 05 || BE32(b + 1))) and block count (0 = until stopped). For k = 64 the
// constant 64-byte seed block is compressed once per start (midstate). Output blocks are little-endian
// byte arrays (stream byte 32*b + i on bits [8i+7:8i]). Stopping a context discards its FIFO and any
// batch in flight for it.
module qube_xof_farm
  import qube_pkg::*;
(
  input  logic          clk,
  input  logic          rst_n,
  input  logic [1:0]    ctx_start,
  input  logic [1:0]    ctx_stop,
  input  logic [511:0]  seed0,         // context 0 seed, little-endian, k bytes valid (sampled with ctx_start)
  input  logic [511:0]  seed1,         // context 1 seed
  input  logic [2:0]    ps,
  input  logic [15:0]   b0,            // context 1 first block (context 0 starts at block 0)
  input  logic [15:0]   nblk,          // context 1 block count (context 0 runs until stopped)
  output logic [1:0]    ovalid,
  input  logic [1:0]    oready,
  output logic [255:0]  odata [0:1]
);
  localparam int NC = 4;
  localparam int DEPTH = 8;

  // ---------------- context state
  logic [1:0]    act, mid_ok;
  logic [511:0]  cseed [0:1];
  logic [255:0]  cmid  [0:1];
  logic [6:0]    ck    [0:1];
  logic [15:0]   cnext [0:1];     // next block index to issue
  logic [15:0]   cleft [0:1];     // blocks left to issue (when climit)
  logic [1:0]    climit;
  logic [3:0]    cgen  [0:1];

  // ---------------- FIFOs
  logic [255:0]  fmem [0:1][0:DEPTH-1];
  logic [3:0]    fcnt [0:1];
  logic [2:0]    frd  [0:1];
  logic [2:0]    fwr  [0:1];

  // ---------------- batch in flight
  logic          bact, bmid;
  logic          bctx;
  logic [3:0]    bgen;
  logic [2:0]    bn;              // blocks in batch
  logic [NC-1:0] cstart;
  logic [NC-1:0] cdone;
  logic [255:0]  civ  [0:NC-1];
  logic [511:0]  cblk [0:NC-1];
  logic [255:0]  cout [0:NC-1];

  genvar gi;
  generate for (gi = 0; gi < NC; gi++) begin : g_core
    qube_sm3_core u_core (.clk(clk), .rst_n(rst_n), .start(cstart[gi]), .abort(1'b0),
      .iv(civ[gi]), .blk(cblk[gi]), .busy(), .done(cdone[gi]), .dout(cout[gi]));
  end endgenerate

  // little-endian byte array -> SM3 block bytes (byte 0 at MSB)
  function automatic logic [511:0] le2be512(input logic [511:0] x);
    for (int i = 0; i < 64; i++) le2be512[511 - 8 * i -: 8] = x[8 * i +: 8];
  endfunction
  function automatic logic [255:0] be2le256(input logic [255:0] x);
    for (int i = 0; i < 32; i++) be2le256[8 * i +: 8] = x[255 - 8 * i -: 8];
  endfunction

  // counter block for seed length k (k < 64) or the midstate continuation (k = 64)
  function automatic logic [511:0] ctr_block(input logic [511:0] sd, input logic [6:0] k, input logic [31:0] ctr);
    logic [511:0] le;
    logic [63:0]  lenbits;
    le = '0;
    if (k == 7'd64) begin
      le[7:0]   = 8'h05;
      le[15:8]  = ctr[31:24]; le[23:16] = ctr[23:16]; le[31:24] = ctr[15:8]; le[39:32] = ctr[7:0];
      le[47:40] = 8'h80;
      lenbits   = 64'd552;
    end else begin
      for (int i = 0; i < 48; i++) if (i < k) le[8 * i +: 8] = sd[8 * i +: 8];
      le[8 * k +: 8]       = 8'h05;
      le[8 * (k + 1) +: 8] = ctr[31:24];
      le[8 * (k + 2) +: 8] = ctr[23:16];
      le[8 * (k + 3) +: 8] = ctr[15:8];
      le[8 * (k + 4) +: 8] = ctr[7:0];
      le[8 * (k + 5) +: 8] = 8'h80;
      lenbits = {54'd0, k, 3'd0} + 64'd40;
    end
    for (int i = 0; i < 8; i++) le[8 * (56 + i) +: 8] = lenbits[63 - 8 * i -: 8];
    ctr_block = le2be512(le);
  endfunction

  // ---------------- scheduling
  logic        can0, can1, pick;     // pick: context chosen this cycle
  logic        issue;
  logic [2:0]  nb_c [0:1];
  logic [4:0]  room [0:1];
  always_comb begin
    for (int c = 0; c < 2; c++) begin
      nb_c[c] = (climit[c] && cleft[c] < 16'd4) ? cleft[c][2:0] : 3'd4;
      room[c] = 5'(DEPTH) - {1'b0, fcnt[c]};
    end
    can0 = act[0] && !(climit[0] && cleft[0] == 16'd0) && (!mid_ok[0] || room[0] >= {2'b0, nb_c[0]});
    can1 = act[1] && !(climit[1] && cleft[1] == 16'd0) && (!mid_ok[1] || room[1] >= {2'b0, nb_c[1]});
    pick  = can0 ? 1'b0 : 1'b1;
    issue = !bact && (can0 || can1);
  end
  // midstate needed only for k = 64; mid_ok is set at start for k < 64

  always_comb begin
    for (int i = 0; i < NC; i++) begin
      cstart[i] = 1'b0; civ[i] = SM3_IV0; cblk[i] = '0;
    end
    if (issue) begin
      if (!mid_ok[pick]) begin
        cstart[0] = 1'b1; civ[0] = SM3_IV0; cblk[0] = le2be512(cseed[pick]);
      end else begin
        for (int i = 0; i < NC; i++) begin
          cstart[i] = (3'(i) < nb_c[pick]);
          civ[i]    = (ck[pick] == 7'd64) ? cmid[pick] : SM3_IV0;
          cblk[i]   = ctr_block(cseed[pick], ck[pick], {16'd0, cnext[pick]} + 32'(i) + 32'd1);
        end
      end
    end
  end

  // FIFO outputs
  logic [3:0] fadd [0:1];
  logic [3:0] fsub [0:1];
  always_comb for (int c = 0; c < 2; c++) begin
    ovalid[c] = (fcnt[c] != 4'd0);
    odata[c]  = fmem[c][frd[c]];
    fadd[c]   = (bact && cdone[0] && !bmid && bctx == 1'(c) && bgen == cgen[c] && act[c]) ? {1'b0, bn} : 4'd0;
    fsub[c]   = (ovalid[c] && oready[c]) ? 4'd1 : 4'd0;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      act <= '0; mid_ok <= '0; climit <= '0; bact <= 1'b0; bmid <= 1'b0; bctx <= 1'b0; bgen <= '0; bn <= '0;
      for (int c = 0; c < 2; c++) begin
        cseed[c] <= '0; cmid[c] <= '0; ck[c] <= '0; cnext[c] <= '0; cleft[c] <= '0; cgen[c] <= '0;
        fcnt[c] <= '0; frd[c] <= '0; fwr[c] <= '0;
        for (int d = 0; d < DEPTH; d++) fmem[c][d] <= '0;
      end
    end else begin
      // issue
      if (issue) begin
        bact <= 1'b1; bctx <= pick; bgen <= cgen[pick];
        if (!mid_ok[pick]) begin
          bmid <= 1'b1; bn <= 3'd1;
        end else begin
          bmid <= 1'b0; bn <= nb_c[pick];
          cnext[pick] <= cnext[pick] + {13'd0, nb_c[pick]};
          if (climit[pick]) cleft[pick] <= cleft[pick] - {13'd0, nb_c[pick]};
        end
      end
      // completion (all started cores finish in the same cycle; core 0 always participates)
      if (bact && cdone[0]) begin
        bact <= 1'b0;
        if (bgen == cgen[bctx] && act[bctx]) begin
          if (bmid) begin
            cmid[bctx] <= cout[0]; mid_ok[bctx] <= 1'b1;
          end else begin
            for (int i = 0; i < NC; i++)
              if (3'(i) < bn) fmem[bctx][fwr[bctx] + 3'(i)] <= be2le256(cout[i]);
            fwr[bctx] <= fwr[bctx] + bn;
          end
        end
      end
      // pops and pushes counts
      for (int c = 0; c < 2; c++) begin
        fcnt[c] <= fcnt[c] + fadd[c] - fsub[c];
        if (ovalid[c] && oready[c]) frd[c] <= frd[c] + 3'd1;
      end
      // context control (start/stop override)
      for (int c = 0; c < 2; c++) begin
        if (ctx_stop[c] || ctx_start[c]) begin
          cgen[c] <= cgen[c] + 4'd1;
          fcnt[c] <= '0; frd[c] <= '0; fwr[c] <= '0;
          act[c] <= ctx_start[c];
          if (ctx_start[c]) begin
            cseed[c] <= (c == 0) ? seed0 : seed1; ck[c] <= p_k(ps);
            mid_ok[c] <= (p_k(ps) != 7'd64);
            cnext[c] <= (c == 0) ? 16'd0 : b0; cleft[c] <= (c == 0) ? 16'd0 : nblk;
            climit[c] <= (c == 0) ? 1'b0 : (nblk != 16'd0);
          end
        end
      end
    end
  end
endmodule
