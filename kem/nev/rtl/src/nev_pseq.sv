// nev_pseq: slot-ring inversion / invertibility judge, sequenced as poly ALU primitives.
// Element d_0 = a (k rows in A). Level l element d_l has size s_l = k >> l and lives at rows
// R(l) = [k - 2 s_l, k - s_l) (l >= 1); d_0 at rows [0, k).
//   down l = 0 .. L-1 (L = log2 k):
//     SCALE  src(l) -> Ysrc(l) = Y*d_l, Csrc(l) = copy of d_l          (mask: slot 0 zeroed, l = 0)
//     PROD   DOWN    d_{l+1} = a0^2 - Z a1^2 (U = src/Ysrc, V = Csrc)  -> T(l+1) rows R(l+1)
//   scalar: inv  : EXP  T(L)[k-2] -> I(L)[k-2] = cval * s^(q-2); zflag -> non-invertible
//           judge: ZCHK T(L)[k-2]
//   up l = L-1 .. 0 (inverse only):
//     PROD   UP      inv(d_l) = (c0 - X c1) * inv(d_{l+1}) (U = src(l)/Ysrc(l), V = I(l+1))
//                    -> I(l) rows R(l)  (l = 0: T2 rows [0, k), the final result)
// src(0) = A, Ysrc(0) = YA0, Csrc(0) = CA0; src(l>=1) = T(l), Ysrc = YCH, Csrc = CCH (rows R(l)).
// Inverse: T(l) = T1 for odd l, T2 for even l >= 2 (the down chain is kept for the up sweep);
//          I(l) = CCH if (L - l) even else CA0 (both free during the up sweep).
// Judge:   T(l) = T1 for odd l, CA0 for even l >= 2 (chain not kept; T2 unused).
module nev_pseq import nev_pkg::*; import nev_palu_pkg::*; (
  input  logic        clk,
  input  logic        rst_n,
  input  logic [2:0]  klog,          // log2 k (2..5)
  input  logic        start,         // pulse
  input  logic        judge,         // 1: judge only
  input  logic        inj,           // test injection: zero slot 0 of the candidate
  input  logic [2:0]  bA, bYA0, bCA0, bT1, bT2, bYCH, bCCH,
  input  logic [11:0] cval,          // scale applied to the inverse
  output logic        busy,
  output logic        done,          // pulse
  output logic        noninv,        // valid with done
  // poly ALU command port
  output logic        a_valid,
  output palu_cmd_t   a_cmd,
  input  logic        a_done,
  input  logic        a_zflag
);
  typedef enum logic [2:0] {P_IDLE, P_DOWN, P_SCAL, P_UP, P_WAIT, P_FIN} ph_t;
  ph_t        ph, ph_ret;
  logic [2:0] lv;            // down: level l; up: level l (counts down)
  logic       sub;           // down: 0 SCALE, 1 PROD
  logic       jdg, injq;
  logic [2:0] A_, YA0, CA0, T1, T2, YCH, CCH;
  logic [11:0] cv;
  logic       nz;

  logic [5:0] k;
  logic [5:0] s_l, s_l1;     // sizes of d_l and d_{l+1}
  logic [4:0] r_l, r_l1;     // base rows of d_l and d_{l+1}
  always_comb begin
    k    = 6'd1 << klog;
    s_l  = k >> lv;
    s_l1 = k >> (lv + 3'd1);
    r_l  = (lv == 3'd0) ? 5'd0 : 5'(k - (s_l << 1));
    r_l1 = 5'(k - (s_l1 << 1));
  end

  // buffer roles
  function automatic logic [2:0] tbuf(input logic [2:0] l, input logic j);
    tbuf = l[0] ? T1 : (j ? CA0 : T2);
  endfunction
  logic [2:0] src_l, ysrc_l, csrc_l, i_l, i_l1;
  always_comb begin
    src_l  = (lv == 3'd0) ? A_  : tbuf(lv, jdg);
    ysrc_l = (lv == 3'd0) ? YA0 : YCH;
    csrc_l = (lv == 3'd0) ? CA0 : CCH;
    i_l    = ((klog - lv) & 3'd1) == 3'd0 ? CCH : CA0;
    i_l1   = ((klog - lv - 3'd1) & 3'd1) == 3'd0 ? CCH : CA0;
  end

  palu_cmd_t cmd_n;
  always_comb begin
    cmd_n = '0;
    case (ph)
      P_DOWN: begin
        if (!sub) begin  // SCALE d_l -> Y*d_l, copy
          cmd_n.op = PA_SCALE; cmd_n.ba = src_l; cmd_n.ra = r_l; cmd_n.nrows = s_l;
          cmd_n.bd = ysrc_l; cmd_n.rd = r_l; cmd_n.copy = 1'b1; cmd_n.be = csrc_l; cmd_n.re = r_l;
          cmd_n.cmode = CM_Y; cmd_n.mask = injq && (lv == 3'd0);
        end else begin   // PROD DOWN -> d_{l+1}
          cmd_n.op = PA_PROD; cmd_n.pmode = PM_DOWN; cmd_n.mlog = klog - lv - 3'd1;
          cmd_n.ba = src_l; cmd_n.bb = ysrc_l; cmd_n.ra = r_l;
          cmd_n.bc = csrc_l; cmd_n.rc = r_l;
          cmd_n.bd = tbuf(lv + 3'd1, jdg); cmd_n.rd = r_l1;
        end
      end
      P_SCAL: begin
        if (jdg) begin cmd_n.op = PA_ZCHK; cmd_n.ba = tbuf(klog, jdg); cmd_n.ra = 5'(k - 6'd2); end
        else begin
          cmd_n.op = PA_EXP; cmd_n.ba = tbuf(klog, jdg); cmd_n.ra = 5'(k - 6'd2);
          cmd_n.bd = CCH; cmd_n.rd = 5'(k - 6'd2); cmd_n.cval = cv;
        end
      end
      P_UP: begin        // PROD UP: inv(d_l) from inv(d_{l+1})
        cmd_n.op = PA_PROD; cmd_n.pmode = PM_UP; cmd_n.mlog = klog - lv - 3'd1;
        cmd_n.ba = src_l; cmd_n.bb = ysrc_l; cmd_n.ra = r_l;
        cmd_n.bc = i_l1; cmd_n.rc = r_l1;
        if (lv == 3'd0) begin cmd_n.bd = T2;  cmd_n.rd = 5'd0; end
        else            begin cmd_n.bd = i_l; cmd_n.rd = r_l;  end
      end
      default: ;
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ph <= P_IDLE; ph_ret <= P_IDLE; lv <= '0; sub <= 1'b0; jdg <= 1'b0; injq <= 1'b0; nz <= 1'b0;
      A_ <= '0; YA0 <= '0; CA0 <= '0; T1 <= '0; T2 <= '0; YCH <= '0; CCH <= '0; cv <= '0;
      a_valid <= 1'b0; a_cmd <= '0; done <= 1'b0; noninv <= 1'b0;
    end else begin
      a_valid <= 1'b0; done <= 1'b0;
      case (ph)
        P_IDLE: if (start) begin
          A_ <= bA; YA0 <= bYA0; CA0 <= bCA0; T1 <= bT1; T2 <= bT2; YCH <= bYCH; CCH <= bCCH;
          cv <= cval; jdg <= judge; injq <= inj; lv <= '0; sub <= 1'b0; nz <= 1'b0; ph <= P_DOWN;
        end
        P_DOWN, P_SCAL, P_UP: begin
          a_valid <= 1'b1; a_cmd <= cmd_n; ph_ret <= ph; ph <= P_WAIT;
        end
        P_WAIT: if (a_done) begin
          case (ph_ret)
            P_DOWN: begin
              if (sub) begin
                sub <= 1'b0;
                if (lv == klog - 3'd1) ph <= P_SCAL; else begin lv <= lv + 3'd1; ph <= P_DOWN; end
              end else begin sub <= 1'b1; ph <= P_DOWN; end
            end
            P_SCAL: begin
              nz <= a_zflag;
              if (jdg || a_zflag) ph <= P_FIN;
              else begin lv <= klog - 3'd1; ph <= P_UP; end
            end
            default: begin  // P_UP
              if (lv == 3'd0) ph <= P_FIN; else begin lv <= lv - 3'd1; ph <= P_UP; end
            end
          endcase
        end
        P_FIN: begin done <= 1'b1; noninv <= nz; ph <= P_IDLE; end
        default: ph <= P_IDLE;
      endcase
    end
  end
  assign busy = (ph != P_IDLE);
endmodule
