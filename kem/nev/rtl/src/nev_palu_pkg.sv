// nev_palu_pkg: poly ALU command format (dual-issue ALU).
package nev_palu_pkg;
  localparam logic [2:0] PA_NTT  = 3'd0;  // CT levels, ping-pong ba <-> bb (dual: second poly bc <-> bd, levels interleaved);
                                          // result in ba if #levels even else bb (bc / bd for the second poly)
                                          // split: rows [0,k/2) ping-pong ba <-> bb, rows [k/2,k) read ba at level 0,
                                          // then bc/bd (level l writes bc if l even else bd), the last level writes
                                          // the same result buffer as the lower half; levels of the two halves interleave
  localparam logic [2:0] PA_INTT = 3'd1;  // GS levels, ping-pong ba <-> bb (no final scaling)
  localparam logic [2:0] PA_SCALE= 3'd2;  // bd[rd+r] = c*ba[ra+r]; if copy: be[re+r] = ba[ra+r]; r < nrows; mask: slot 0 input forced to 0
                                          // cmode CM_KY: one word per cycle, be[re+r] = cval*ba[ra+r], bd[rd+r] = (cval*Y)*ba[ra+r]
                                          // (ROM k*y_s, ks selects kd / kci); strm: read word w only when w < slim
                                          // (source written by a codec) and not in a cycle the codec writes
  localparam logic [2:0] PA_PROD = 3'd3;  // slot products mod (Z^m - y), see pmode
  localparam logic [2:0] PA_EXP  = 3'd5;  // bd[rd] = cval * ba[ra]^(q-2) ; zflag = any input slot zero
  localparam logic [2:0] PA_ZCHK = 3'd6;  // zflag = any slot of ba[ra] zero

  localparam logic [1:0] CM_Y = 2'd0, CM_CONST = 2'd1, CM_KY = 2'd2;

  // PROD modes (B = 1 << mlog). U operand rows from ba (term index not wrapped) or bb (wrapped,
  // Y-scaled copy at the same rows); V operand rows from bc; outputs to bd.
  //   NORM : o < B, i < B      : U row ra+idx, idx=(o-i) mod B ; V row rc+i ; (+E from be row re+o if extra)
  //   DOWN : o < B, i < 2B     : set=i>=B, i'=i-set*B, idx=(o-i'-set) mod B ; U row ra+2idx+set ; V row rc+2i'+set ;
  //                              set-1 terms negated            (b = a0^2 - Z a1^2)
  //   UP   : o < 2B, i < B     : j=o>>1, p=o&1, idx=(j-i) mod B ; U row ra+2idx+p ; V row rc+i ; odd outputs negated
  // Output row rd+o; neg negates every output (NORM).
  localparam logic [1:0] PM_NORM = 2'd0, PM_DOWN = 2'd1, PM_UP = 2'd2;

  typedef struct packed {
    logic [2:0]  op;
    logic [2:0]  ba, bb, bc, bd, be;     // buffers
    logic [4:0]  ra, rc, rd, re;         // row bases
    logic        dual;                   // NTT: second polynomial bc <-> bd
    logic        split;                  // NTT: one polynomial, rows [k/2,k) on their own ping-pong bc/bd
    logic [1:0]  pmode;                  // PROD mode
    logic [2:0]  mlog;                   // PROD base size
    logic [5:0]  nrows;                  // SCALE rows
    logic        neg;                    // PROD: negate result
    logic        extra;                  // PROD: add E term (be, re)
    logic        copy;                   // SCALE: also copy to be
    logic        mask;                   // SCALE: zero slot 0 (test injection)
    logic [1:0]  cmode;                  // SCALE constant mode
    logic [11:0] cval;                   // SCALE const / EXP multiplier
    logic        strm;                   // SCALE: source streamed behind a codec (slim / sblk)
    logic        ks;                     // SCALE CM_KY: ROM constant (0 kd*y, 1 kci*y)
    logic        cmpc;                   // PROD NORM: subtract C rows (buffer bd) and flag any nonzero output
                                         // (nzflag) instead of writing the result
    logic        tmsg;                   // INTT: decode the message from the last level's outputs (msg, m_we at done)
  } palu_cmd_t;
endpackage
