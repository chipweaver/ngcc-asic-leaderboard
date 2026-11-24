// Reed-Solomon decoder, bit-exact with model qube.rs_decode (reference Berlekamp compute_elp, z(x)
// and compute_error_values; Chien search over positions 0..N1-1 in place of the additive FFT).
// The received codeword is written byte-wise on sym_we/sym_idx/sym_data before start. Phases:
//   SYN   N1 cycles      Horner syndromes S_i = r(alpha^(i+1)), 2*delta constant multipliers
//   BM    3 cycles/iter  A: dd = dd_ * d_p^-1, B: sigma ^= dd * X, X/T shift, C: next dd_
//                        (2*delta iterations, the last one ends after B)
//   ZLD/ZACC/ZFIN        z(x) via the convolution sigma * S, delta cycles in the lane multipliers
//   CHN   N1 cycles      Chien search sigma(alpha^-i) == 0 with z(alpha^-i) evaluated alongside;
//                        error entries (alpha^i, alpha^-i, z(alpha^-i), i) pushed into lists
//   T2F/T2A 2 cycles per error  t2_e = prod_{f != e} (1 ^ beta_f * beta_e^-1) for all e in parallel
//   FIN   errors+1 cycles      e = t1 * t2^-1 applied to the message byte (positions >= 2*delta)
// The message register is loaded from the codeword at start; only message positions are corrected.
// One general GF multiply per lane per cycle (DMAX lanes shared by BM, z, t2) plus one extra
// multiplier and one inverse table shared by BM-A and FIN.
module qube_rsdec
  import qube_pkg::*;
(
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,
  input  logic [2:0]   ps,
  input  logic         sym_we,
  input  logic [7:0]   sym_idx,
  input  logic [7:0]   sym_data,
  output logic         busy,
  output logic         done,
  output logic [511:0] msg
);
  function automatic logic [7:0] inv_rom(input logic [7:0] a);
    case (a)
      8'd0: inv_rom = 8'd0;
      8'd1: inv_rom = 8'd1;
      8'd2: inv_rom = 8'd142;
      8'd3: inv_rom = 8'd244;
      8'd4: inv_rom = 8'd71;
      8'd5: inv_rom = 8'd167;
      8'd6: inv_rom = 8'd122;
      8'd7: inv_rom = 8'd186;
      8'd8: inv_rom = 8'd173;
      8'd9: inv_rom = 8'd157;
      8'd10: inv_rom = 8'd221;
      8'd11: inv_rom = 8'd152;
      8'd12: inv_rom = 8'd61;
      8'd13: inv_rom = 8'd170;
      8'd14: inv_rom = 8'd93;
      8'd15: inv_rom = 8'd150;
      8'd16: inv_rom = 8'd216;
      8'd17: inv_rom = 8'd114;
      8'd18: inv_rom = 8'd192;
      8'd19: inv_rom = 8'd88;
      8'd20: inv_rom = 8'd224;
      8'd21: inv_rom = 8'd62;
      8'd22: inv_rom = 8'd76;
      8'd23: inv_rom = 8'd102;
      8'd24: inv_rom = 8'd144;
      8'd25: inv_rom = 8'd222;
      8'd26: inv_rom = 8'd85;
      8'd27: inv_rom = 8'd128;
      8'd28: inv_rom = 8'd160;
      8'd29: inv_rom = 8'd131;
      8'd30: inv_rom = 8'd75;
      8'd31: inv_rom = 8'd42;
      8'd32: inv_rom = 8'd108;
      8'd33: inv_rom = 8'd237;
      8'd34: inv_rom = 8'd57;
      8'd35: inv_rom = 8'd81;
      8'd36: inv_rom = 8'd96;
      8'd37: inv_rom = 8'd86;
      8'd38: inv_rom = 8'd44;
      8'd39: inv_rom = 8'd138;
      8'd40: inv_rom = 8'd112;
      8'd41: inv_rom = 8'd208;
      8'd42: inv_rom = 8'd31;
      8'd43: inv_rom = 8'd74;
      8'd44: inv_rom = 8'd38;
      8'd45: inv_rom = 8'd139;
      8'd46: inv_rom = 8'd51;
      8'd47: inv_rom = 8'd110;
      8'd48: inv_rom = 8'd72;
      8'd49: inv_rom = 8'd137;
      8'd50: inv_rom = 8'd111;
      8'd51: inv_rom = 8'd46;
      8'd52: inv_rom = 8'd164;
      8'd53: inv_rom = 8'd195;
      8'd54: inv_rom = 8'd64;
      8'd55: inv_rom = 8'd94;
      8'd56: inv_rom = 8'd80;
      8'd57: inv_rom = 8'd34;
      8'd58: inv_rom = 8'd207;
      8'd59: inv_rom = 8'd169;
      8'd60: inv_rom = 8'd171;
      8'd61: inv_rom = 8'd12;
      8'd62: inv_rom = 8'd21;
      8'd63: inv_rom = 8'd225;
      8'd64: inv_rom = 8'd54;
      8'd65: inv_rom = 8'd95;
      8'd66: inv_rom = 8'd248;
      8'd67: inv_rom = 8'd213;
      8'd68: inv_rom = 8'd146;
      8'd69: inv_rom = 8'd78;
      8'd70: inv_rom = 8'd166;
      8'd71: inv_rom = 8'd4;
      8'd72: inv_rom = 8'd48;
      8'd73: inv_rom = 8'd136;
      8'd74: inv_rom = 8'd43;
      8'd75: inv_rom = 8'd30;
      8'd76: inv_rom = 8'd22;
      8'd77: inv_rom = 8'd103;
      8'd78: inv_rom = 8'd69;
      8'd79: inv_rom = 8'd147;
      8'd80: inv_rom = 8'd56;
      8'd81: inv_rom = 8'd35;
      8'd82: inv_rom = 8'd104;
      8'd83: inv_rom = 8'd140;
      8'd84: inv_rom = 8'd129;
      8'd85: inv_rom = 8'd26;
      8'd86: inv_rom = 8'd37;
      8'd87: inv_rom = 8'd97;
      8'd88: inv_rom = 8'd19;
      8'd89: inv_rom = 8'd193;
      8'd90: inv_rom = 8'd203;
      8'd91: inv_rom = 8'd99;
      8'd92: inv_rom = 8'd151;
      8'd93: inv_rom = 8'd14;
      8'd94: inv_rom = 8'd55;
      8'd95: inv_rom = 8'd65;
      8'd96: inv_rom = 8'd36;
      8'd97: inv_rom = 8'd87;
      8'd98: inv_rom = 8'd202;
      8'd99: inv_rom = 8'd91;
      8'd100: inv_rom = 8'd185;
      8'd101: inv_rom = 8'd196;
      8'd102: inv_rom = 8'd23;
      8'd103: inv_rom = 8'd77;
      8'd104: inv_rom = 8'd82;
      8'd105: inv_rom = 8'd141;
      8'd106: inv_rom = 8'd239;
      8'd107: inv_rom = 8'd179;
      8'd108: inv_rom = 8'd32;
      8'd109: inv_rom = 8'd236;
      8'd110: inv_rom = 8'd47;
      8'd111: inv_rom = 8'd50;
      8'd112: inv_rom = 8'd40;
      8'd113: inv_rom = 8'd209;
      8'd114: inv_rom = 8'd17;
      8'd115: inv_rom = 8'd217;
      8'd116: inv_rom = 8'd233;
      8'd117: inv_rom = 8'd251;
      8'd118: inv_rom = 8'd218;
      8'd119: inv_rom = 8'd121;
      8'd120: inv_rom = 8'd219;
      8'd121: inv_rom = 8'd119;
      8'd122: inv_rom = 8'd6;
      8'd123: inv_rom = 8'd187;
      8'd124: inv_rom = 8'd132;
      8'd125: inv_rom = 8'd205;
      8'd126: inv_rom = 8'd254;
      8'd127: inv_rom = 8'd252;
      8'd128: inv_rom = 8'd27;
      8'd129: inv_rom = 8'd84;
      8'd130: inv_rom = 8'd161;
      8'd131: inv_rom = 8'd29;
      8'd132: inv_rom = 8'd124;
      8'd133: inv_rom = 8'd204;
      8'd134: inv_rom = 8'd228;
      8'd135: inv_rom = 8'd176;
      8'd136: inv_rom = 8'd73;
      8'd137: inv_rom = 8'd49;
      8'd138: inv_rom = 8'd39;
      8'd139: inv_rom = 8'd45;
      8'd140: inv_rom = 8'd83;
      8'd141: inv_rom = 8'd105;
      8'd142: inv_rom = 8'd2;
      8'd143: inv_rom = 8'd245;
      8'd144: inv_rom = 8'd24;
      8'd145: inv_rom = 8'd223;
      8'd146: inv_rom = 8'd68;
      8'd147: inv_rom = 8'd79;
      8'd148: inv_rom = 8'd155;
      8'd149: inv_rom = 8'd188;
      8'd150: inv_rom = 8'd15;
      8'd151: inv_rom = 8'd92;
      8'd152: inv_rom = 8'd11;
      8'd153: inv_rom = 8'd220;
      8'd154: inv_rom = 8'd189;
      8'd155: inv_rom = 8'd148;
      8'd156: inv_rom = 8'd172;
      8'd157: inv_rom = 8'd9;
      8'd158: inv_rom = 8'd199;
      8'd159: inv_rom = 8'd162;
      8'd160: inv_rom = 8'd28;
      8'd161: inv_rom = 8'd130;
      8'd162: inv_rom = 8'd159;
      8'd163: inv_rom = 8'd198;
      8'd164: inv_rom = 8'd52;
      8'd165: inv_rom = 8'd194;
      8'd166: inv_rom = 8'd70;
      8'd167: inv_rom = 8'd5;
      8'd168: inv_rom = 8'd206;
      8'd169: inv_rom = 8'd59;
      8'd170: inv_rom = 8'd13;
      8'd171: inv_rom = 8'd60;
      8'd172: inv_rom = 8'd156;
      8'd173: inv_rom = 8'd8;
      8'd174: inv_rom = 8'd190;
      8'd175: inv_rom = 8'd183;
      8'd176: inv_rom = 8'd135;
      8'd177: inv_rom = 8'd229;
      8'd178: inv_rom = 8'd238;
      8'd179: inv_rom = 8'd107;
      8'd180: inv_rom = 8'd235;
      8'd181: inv_rom = 8'd242;
      8'd182: inv_rom = 8'd191;
      8'd183: inv_rom = 8'd175;
      8'd184: inv_rom = 8'd197;
      8'd185: inv_rom = 8'd100;
      8'd186: inv_rom = 8'd7;
      8'd187: inv_rom = 8'd123;
      8'd188: inv_rom = 8'd149;
      8'd189: inv_rom = 8'd154;
      8'd190: inv_rom = 8'd174;
      8'd191: inv_rom = 8'd182;
      8'd192: inv_rom = 8'd18;
      8'd193: inv_rom = 8'd89;
      8'd194: inv_rom = 8'd165;
      8'd195: inv_rom = 8'd53;
      8'd196: inv_rom = 8'd101;
      8'd197: inv_rom = 8'd184;
      8'd198: inv_rom = 8'd163;
      8'd199: inv_rom = 8'd158;
      8'd200: inv_rom = 8'd210;
      8'd201: inv_rom = 8'd247;
      8'd202: inv_rom = 8'd98;
      8'd203: inv_rom = 8'd90;
      8'd204: inv_rom = 8'd133;
      8'd205: inv_rom = 8'd125;
      8'd206: inv_rom = 8'd168;
      8'd207: inv_rom = 8'd58;
      8'd208: inv_rom = 8'd41;
      8'd209: inv_rom = 8'd113;
      8'd210: inv_rom = 8'd200;
      8'd211: inv_rom = 8'd246;
      8'd212: inv_rom = 8'd249;
      8'd213: inv_rom = 8'd67;
      8'd214: inv_rom = 8'd215;
      8'd215: inv_rom = 8'd214;
      8'd216: inv_rom = 8'd16;
      8'd217: inv_rom = 8'd115;
      8'd218: inv_rom = 8'd118;
      8'd219: inv_rom = 8'd120;
      8'd220: inv_rom = 8'd153;
      8'd221: inv_rom = 8'd10;
      8'd222: inv_rom = 8'd25;
      8'd223: inv_rom = 8'd145;
      8'd224: inv_rom = 8'd20;
      8'd225: inv_rom = 8'd63;
      8'd226: inv_rom = 8'd230;
      8'd227: inv_rom = 8'd240;
      8'd228: inv_rom = 8'd134;
      8'd229: inv_rom = 8'd177;
      8'd230: inv_rom = 8'd226;
      8'd231: inv_rom = 8'd241;
      8'd232: inv_rom = 8'd250;
      8'd233: inv_rom = 8'd116;
      8'd234: inv_rom = 8'd243;
      8'd235: inv_rom = 8'd180;
      8'd236: inv_rom = 8'd109;
      8'd237: inv_rom = 8'd33;
      8'd238: inv_rom = 8'd178;
      8'd239: inv_rom = 8'd106;
      8'd240: inv_rom = 8'd227;
      8'd241: inv_rom = 8'd231;
      8'd242: inv_rom = 8'd181;
      8'd243: inv_rom = 8'd234;
      8'd244: inv_rom = 8'd3;
      8'd245: inv_rom = 8'd143;
      8'd246: inv_rom = 8'd211;
      8'd247: inv_rom = 8'd201;
      8'd248: inv_rom = 8'd66;
      8'd249: inv_rom = 8'd212;
      8'd250: inv_rom = 8'd232;
      8'd251: inv_rom = 8'd117;
      8'd252: inv_rom = 8'd127;
      8'd253: inv_rom = 8'd255;
      8'd254: inv_rom = 8'd126;
      8'd255: inv_rom = 8'd253;
      default: inv_rom = 8'd0;
    endcase
  endfunction

  localparam logic [3:0] IDLE = 4'd0, SYN = 4'd1, BMA = 4'd2, BMB = 4'd3, BMC = 4'd4, ZLD = 4'd5,
                         ZACC = 4'd6, ZFIN = 4'd7, CHN = 4'd8, T2F = 4'd9, T2A = 4'd10, FIN = 4'd11,
                         OUT = 4'd12;

  logic [3:0]  ph;
  logic [5:0]  d_r;
  logic [6:0]  nk_r;                 // 2*delta
  logic [7:0]  n1_r;
  logic [7:0]  ctr;
  logic [7:0]  r    [0:N1MAX-1];     // received codeword
  logic [7:0]  syn  [0:GMAX-1];
  logic [7:0]  sg   [1:DMAX];        // sigma (BM), sigma*alpha^-ij (Chien), t2 factor
  logic [7:0]  xr   [0:DMAX];        // X_sigma_p (BM), sigma copy (z), beta list (Chien, t2)
  logic [7:0]  tr   [0:DMAX];        // syndrome window (BM, z), t1 list (Chien, FIN)
  logic [7:0]  zr   [1:DMAX];        // z accumulators, z*alpha^-ij (Chien), t2 products
  logic [7:0]  invl [0:DMAX-1];      // alpha^-pos list
  logic [7:0]  posl [0:DMAX-1];      // position list
  logic [7:0]  dd_r, dpinv, dd2, invtmp;
  logic        updr;
  logic [7:0]  dxs, degs, degp;
  logic [6:0]  mu, ppp;              // ppp = pp + 1
  logic [7:0]  ainv, apos;
  logic [5:0]  ecnt;
  logic [7:0]  qinv, t1h, mpos;
  logic        fv, mok;
  logic [7:0]  mb_r [0:KMAX-1];

  assign busy = (ph != IDLE);
  always_comb for (int b = 0; b < KMAX; b++) msg[8*b +: 8] = mb_r[b];

  // ---------------- message bytes of the received codeword (loaded at start)
  logic [7:0] mx [0:KMAX-1];
  always_comb begin
    for (int b = 0; b < KMAX; b++) mx[b] = 8'd0;
    case (ps)
      3'd1:    for (int b = 0; b < 24; b++) mx[b] = r[42 + b];
      3'd2:    for (int b = 0; b < 32; b++) mx[b] = r[76 + b];
      3'd3:    for (int b = 0; b < 48; b++) mx[b] = r[80 + b];
      3'd4:    for (int b = 0; b < 64; b++) mx[b] = r[86 + b];
      default: for (int b = 0; b < 16; b++) mx[b] = r[18 + b];
    endcase
  end

  // ---------------- syndromes (Horner, positions N1-1 down to 0)
  logic [7:0] rsel;
  logic [7:0] synn [0:GMAX-1];
  always_comb begin
    rsel = (ctr < 8'(N1MAX)) ? r[ctr] : 8'd0;
    for (int i = 0; i < GMAX; i++) synn[i] = gf_mul(syn[i], gf_alpha_pow(i + 1)) ^ rsel;
  end

  // ---------------- lane multipliers
  logic [7:0] ma [0:DMAX-1];
  logic [7:0] mbv [0:DMAX-1];
  logic [7:0] mp [0:DMAX-1];
  always_comb begin
    for (int l = 0; l < DMAX; l++) begin
      case (ph)
        BMB:     begin ma[l] = dd2;       mbv[l] = xr[l+1];  end
        BMC:     begin ma[l] = sg[l+1];   mbv[l] = tr[l+1];  end
        ZACC:    begin ma[l] = xr[0];     mbv[l] = tr[l+1];  end
        T2F:     begin ma[l] = xr[0];     mbv[l] = invl[l];  end
        T2A:     begin ma[l] = zr[l+1];   mbv[l] = sg[l+1];  end
        default: begin ma[l] = 8'd0;      mbv[l] = 8'd0;     end
      endcase
      mp[l] = gf_mul(ma[l], mbv[l]);
    end
  end

  // ---------------- shared multiplier and inverse table
  logic [7:0] p0, iout;
  always_comb begin
    // only BMA (4'b0010) and FIN (4'b1011) use them: select on ph[3]
    p0   = ph[3] ? gf_mul(t1h, qinv) : gf_mul(dd_r, dpinv);
    iout = inv_rom(ph[3] ? zr[1] : dd_r);
  end

  // ---------------- BM discrepancy, degree test
  logic [7:0] ddn, dxs_c;
  always_comb begin
    ddn = tr[0];
    for (int l = 0; l < DMAX; l++) ddn = ddn ^ mp[l];
    dxs_c = {1'b0, mu} + 8'd1 - {1'b0, ppp} + degp;
  end

  // ---------------- Chien sums
  logic [7:0] ss, sz;
  logic       push;
  always_comb begin
    ss = 8'd1; sz = 8'd1;
    for (int i = 1; i <= DMAX; i++) begin ss = ss ^ sg[i]; sz = sz ^ zr[i]; end
    push = (ss == 8'd0) && (ecnt < d_r);
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ph <= IDLE; done <= 1'b0; d_r <= '0; nk_r <= '0; n1_r <= '0; ctr <= '0;
      dd_r <= '0; dpinv <= '0; dd2 <= '0; invtmp <= '0; updr <= 1'b0; dxs <= '0; degs <= '0; degp <= '0;
      mu <= '0; ppp <= '0; ainv <= '0; apos <= '0; ecnt <= '0; qinv <= '0; t1h <= '0; mpos <= '0;
      fv <= 1'b0; mok <= 1'b0;
      for (int j = 0; j < N1MAX; j++) r[j] <= '0;
      for (int i = 0; i < GMAX; i++) syn[i] <= '0;
      for (int i = 1; i <= DMAX; i++) begin sg[i] <= '0; zr[i] <= '0; end
      for (int i = 0; i <= DMAX; i++) begin xr[i] <= '0; tr[i] <= '0; end
      for (int i = 0; i < DMAX; i++) begin invl[i] <= '0; posl[i] <= '0; end
      for (int b = 0; b < KMAX; b++) mb_r[b] <= '0;
    end else begin
      done <= 1'b0;
      if (sym_we)
        for (int j = 0; j < N1MAX; j++) if (sym_idx == 8'(j)) r[j] <= sym_data;
      if (start) begin
        ph <= SYN; d_r <= p_delta(ps); nk_r <= {p_delta(ps), 1'b0}; n1_r <= p_n1(ps);
        ctr <= p_n1(ps) - 8'd1;
        for (int i = 0; i < GMAX; i++) syn[i] <= '0;
        for (int i = 1; i <= DMAX; i++) sg[i] <= '0;
        for (int i = 0; i <= DMAX; i++) begin xr[i] <= (i == 1) ? 8'd1 : 8'd0; tr[i] <= '0; end
        dpinv <= 8'd1; degs <= '0; degp <= '0; ppp <= '0; mu <= '0; fv <= 1'b0;
        for (int b = 0; b < KMAX; b++) mb_r[b] <= mx[b];
      end else begin
        case (ph)
          SYN: begin
            for (int i = 0; i < GMAX; i++) syn[i] <= synn[i];
            if (ctr == 8'd0) begin
              ph <= BMA; dd_r <= synn[0]; tr[0] <= synn[0];
            end else ctr <= ctr - 8'd1;
          end
          BMA: begin
            dd2 <= p0; invtmp <= iout; dxs <= dxs_c;
            updr <= (dd_r != 8'd0) && (dxs_c > degs);
            ph <= BMB;
          end
          BMB: begin
            for (int i = 1; i <= DMAX; i++) sg[i] <= sg[i] ^ mp[i-1];
            if (updr) degs <= dxs;
            if (mu == nk_r - 7'd1) ph <= ZLD;
            else begin
              if (updr) begin ppp <= mu + 7'd1; dpinv <= invtmp; degp <= degs; end
              xr[1] <= updr ? 8'd1 : xr[0];
              for (int i = 2; i <= DMAX; i++)
                xr[i] <= (6'(i) <= d_r) ? (updr ? sg[i-1] : xr[i-1]) : 8'd0;
              tr[0] <= syn[mu + 7'd1];
              for (int i = 1; i <= DMAX; i++) tr[i] <= tr[i-1];
              ph <= BMC;
            end
          end
          BMC: begin
            dd_r <= ddn; mu <= mu + 7'd1; ph <= BMA;
          end
          ZLD: begin
            xr[0] <= 8'd1;
            for (int i = 1; i <= DMAX; i++) begin xr[i] <= sg[i]; tr[i] <= syn[i-1]; zr[i] <= '0; end
            tr[0] <= '0; ctr <= '0; ph <= ZACC;
          end
          ZACC: begin
            for (int i = 1; i <= DMAX; i++) zr[i] <= zr[i] ^ mp[i-1];
            for (int k = 0; k < DMAX; k++) xr[k] <= xr[k+1];
            xr[DMAX] <= '0;
            tr[0] <= '0;
            for (int i = 1; i <= DMAX; i++) tr[i] <= tr[i-1];
            ctr <= ctr + 8'd1;
            if (ctr == {2'd0, d_r} - 8'd1) ph <= ZFIN;
          end
          ZFIN: begin
            for (int i = 1; i <= DMAX; i++)
              zr[i] <= ((6'(i) <= d_r) && (8'(i) <= degs)) ? (sg[i] ^ zr[i]) : ((i == 1) ? syn[0] : 8'd0);
            for (int i = 0; i <= DMAX; i++) begin xr[i] <= '0; tr[i] <= '0; end
            for (int i = 0; i < DMAX; i++) begin invl[i] <= '0; posl[i] <= '0; end
            ecnt <= '0; ainv <= 8'd1; apos <= 8'd1; ctr <= '0; ph <= CHN;
          end
          CHN: begin
            for (int i = 1; i <= DMAX; i++) begin
              sg[i] <= gf_mul(sg[i], gf_alpha_pow(255 - i));
              zr[i] <= gf_mul(zr[i], gf_alpha_pow(255 - i));
            end
            ainv <= gf_mul(ainv, gf_alpha_pow(254));
            apos <= gf_mul(apos, gf_alpha_pow(1));
            if (push) begin
              xr[0] <= apos; tr[0] <= sz; invl[0] <= ainv; posl[0] <= ctr;
              for (int k = 1; k < DMAX; k++) begin
                xr[k] <= xr[k-1]; tr[k] <= tr[k-1]; invl[k] <= invl[k-1]; posl[k] <= posl[k-1];
              end
              ecnt <= ecnt + 6'd1;
            end
            ctr <= ctr + 8'd1;
            if (ctr == n1_r - 8'd1) begin
              ctr <= '0;
              ph <= (ecnt != 6'd0 || push) ? T2F : OUT;
            end
          end
          T2F: begin
            for (int l = 0; l < DMAX; l++) sg[l+1] <= (ctr == 8'(l)) ? 8'd1 : (8'd1 ^ mp[l]);
            for (int k = 0; k < DMAX; k++) xr[k] <= xr[k+1];
            ph <= T2A;
          end
          T2A: begin
            for (int l = 0; l < DMAX; l++) zr[l+1] <= (ctr == 8'd0) ? sg[l+1] : mp[l];
            if (ctr == {2'd0, ecnt} - 8'd1) begin ctr <= '0; ph <= FIN; end
            else begin ctr <= ctr + 8'd1; ph <= T2F; end
          end
          FIN: begin
            fv <= (ctr != {2'd0, ecnt});
            if (ctr != {2'd0, ecnt}) begin
              qinv <= iout; t1h <= tr[0];
              mpos <= posl[0] - {1'b0, nk_r};
              mok  <= (posl[0] >= {1'b0, nk_r});
              for (int k = 0; k < DMAX; k++) tr[k] <= tr[k+1];
              for (int k = 1; k < DMAX; k++) zr[k] <= zr[k+1];
              for (int k = 0; k < DMAX - 1; k++) posl[k] <= posl[k+1];
              ctr <= ctr + 8'd1;
            end else ph <= OUT;
            if (fv && mok)
              for (int b = 0; b < KMAX; b++) if (mpos == 8'(b)) mb_r[b] <= mb_r[b] ^ p0;
          end
          OUT: begin
            done <= 1'b1; fv <= 1'b0; ph <= IDLE;
          end
          default: ph <= IDLE;
        endcase
      end
    end
  end
endmodule
