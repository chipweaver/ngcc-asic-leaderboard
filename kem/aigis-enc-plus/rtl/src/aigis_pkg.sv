// Aigis-Enc+ performance top: shared constants, parameter-set decode and ROM tables.
// Canonical (non-Montgomery) twiddles: TW[k] = zetas[k]*R^-1 mod q (forward CT NTT),
// TWI[k] = zetas_inv[k]*R^-1 mod q (inverse GS NTT; TWI[126] includes the 1/128 factor of the
// upper output), SLOTZ[i] = NTT_Y[i]*R^-1 mod q (x^m = SLOTZ[i] in NTT slot i).
package aigis_pkg;
  localparam int Q = 3329;
  localparam logic [11:0] QV = 12'd3329;
  localparam logic [11:0] INV128 = 12'd3303;
  localparam int NSLOT = 7;
  localparam int LANES = 16;

  // parameter-set decode (p = 1, 2, 3)
  function automatic logic [1:0] lw_m5(input logic [1:0] p);   // log2(W) - 5 : W = 32 << lw_m5
    lw_m5 = p - 2'd1;
  endfunction
  function automatic logic [7:0] wcount(input logic [1:0] p);  // W = n/16 words per lane
    wcount = 8'd32 << (p - 2'd1);
  endfunction
  function automatic logic [6:0] bbytes(input logic [1:0] p);   // SEED_BYTES
    bbytes = 7'd16 << (p - 2'd1);
  endfunction
  function automatic logic [2:0] eta_s(input logic [1:0] p);
    eta_s = (p == 2'd1) ? 3'd5 : 3'd2;
  endfunction
  function automatic logic [2:0] eta_e(input logic [1:0] p);
    eta_e = (p == 2'd3) ? 3'd3 : 3'd6;
  endfunction
  function automatic logic [2:0] c2bits(input logic [1:0] p);
    c2bits = (p == 2'd2) ? 3'd3 : 3'd4;
  endfunction
  // object byte sizes
  function automatic logic [12:0] pk_bytes(input logic [1:0] p);   // b + 10n/8
    pk_bytes = (p == 2'd1) ? 13'd656 : (p == 2'd2) ? 13'd1312 : 13'd2624;
  endfunction
  function automatic logic [12:0] polyc10_bytes(input logic [1:0] p); // 10n/8
    polyc10_bytes = (p == 2'd1) ? 13'd640 : (p == 2'd2) ? 13'd1280 : 13'd2560;
  endfunction
  function automatic logic [12:0] poly12_bytes(input logic [1:0] p);  // 12n/8
    poly12_bytes = (p == 2'd1) ? 13'd768 : (p == 2'd2) ? 13'd1536 : 13'd3072;
  endfunction
  function automatic logic [12:0] sk_bytes(input logic [1:0] p);      // 12n/8 + pk + 2b
    sk_bytes = (p == 2'd1) ? 13'd1456 : (p == 2'd2) ? 13'd2912 : 13'd5824;
  endfunction
  function automatic logic [12:0] ct_bytes(input logic [1:0] p);      // 10n/8 + c2 n/8
    ct_bytes = (p == 2'd1) ? 13'd896 : (p == 2'd2) ? 13'd1664 : 13'd3584;
  endfunction

  // bank of word w (parity of the word-address bits) and address inside the bank
  function automatic logic wbank(input logic [6:0] w);
    wbank = ^w;
  endfunction

  // modular helpers on canonical values
  function automatic logic [11:0] addq(input logic [11:0] a, input logic [11:0] b);
    logic [12:0] s;
    s = {1'b0, a} + {1'b0, b};
    addq = (s >= 13'd3329) ? 12'(s - 13'd3329) : s[11:0];
  endfunction
  function automatic logic [11:0] subq(input logic [11:0] a, input logic [11:0] b);
    logic [12:0] s;
    s = {1'b0, a} - {1'b0, b};
    subq = s[12] ? 12'(s + 13'd3329) : s[11:0];
  endfunction
  // compress_d(x) = round(x * 2^d / q) mod 2^d for canonical x (exhaustively checked constants)
  function automatic logic [9:0] compress10(input logic [11:0] x);
    logic [47:0] t;
    t = ({26'd0, x, 10'd0} + 48'd1664) * 48'd161271;
    compress10 = t[38:29];
  endfunction
  function automatic logic [3:0] compress_c2(input logic [11:0] x, input logic [1:0] p);
    logic [31:0] t;
    if (p == 2'd2) t = ({17'd0, x, 3'd0} + 32'd1664) * 32'd315;
    else           t = ({16'd0, x, 4'd0} + 32'd1664) * 32'd315;
    compress_c2 = (p == 2'd2) ? {1'b0, t[22:20]} : t[23:20];
  endfunction
  function automatic logic [11:0] decompress10(input logic [9:0] y);
    logic [21:0] t;
    t = {12'd0, y} * 22'd3329 + 22'd512;
    decompress10 = t[21:10];
  endfunction
  function automatic logic [11:0] decompress_c2(input logic [3:0] y, input logic [1:0] p);
    logic [15:0] t;
    if (p == 2'd2) begin t = {13'd0, y[2:0]} * 16'd3329 + 16'd4; decompress_c2 = t[14:3]; end
    else           begin t = {12'd0, y} * 16'd3329 + 16'd8;      decompress_c2 = t[15:4]; end
  endfunction

  function automatic logic [11:0] tw_fwd(input logic [6:0] k);
    case (k)
      7'd0: tw_fwd = 12'd1;
      7'd1: tw_fwd = 12'd1729;
      7'd2: tw_fwd = 12'd2580;
      7'd3: tw_fwd = 12'd3289;
      7'd4: tw_fwd = 12'd2642;
      7'd5: tw_fwd = 12'd630;
      7'd6: tw_fwd = 12'd1897;
      7'd7: tw_fwd = 12'd848;
      7'd8: tw_fwd = 12'd1062;
      7'd9: tw_fwd = 12'd1919;
      7'd10: tw_fwd = 12'd193;
      7'd11: tw_fwd = 12'd797;
      7'd12: tw_fwd = 12'd2786;
      7'd13: tw_fwd = 12'd3260;
      7'd14: tw_fwd = 12'd569;
      7'd15: tw_fwd = 12'd1746;
      7'd16: tw_fwd = 12'd296;
      7'd17: tw_fwd = 12'd2447;
      7'd18: tw_fwd = 12'd1339;
      7'd19: tw_fwd = 12'd1476;
      7'd20: tw_fwd = 12'd3046;
      7'd21: tw_fwd = 12'd56;
      7'd22: tw_fwd = 12'd2240;
      7'd23: tw_fwd = 12'd1333;
      7'd24: tw_fwd = 12'd1426;
      7'd25: tw_fwd = 12'd2094;
      7'd26: tw_fwd = 12'd535;
      7'd27: tw_fwd = 12'd2882;
      7'd28: tw_fwd = 12'd2393;
      7'd29: tw_fwd = 12'd2879;
      7'd30: tw_fwd = 12'd1974;
      7'd31: tw_fwd = 12'd821;
      7'd32: tw_fwd = 12'd289;
      7'd33: tw_fwd = 12'd331;
      7'd34: tw_fwd = 12'd3253;
      7'd35: tw_fwd = 12'd1756;
      7'd36: tw_fwd = 12'd1197;
      7'd37: tw_fwd = 12'd2304;
      7'd38: tw_fwd = 12'd2277;
      7'd39: tw_fwd = 12'd2055;
      7'd40: tw_fwd = 12'd650;
      7'd41: tw_fwd = 12'd1977;
      7'd42: tw_fwd = 12'd2513;
      7'd43: tw_fwd = 12'd632;
      7'd44: tw_fwd = 12'd2865;
      7'd45: tw_fwd = 12'd33;
      7'd46: tw_fwd = 12'd1320;
      7'd47: tw_fwd = 12'd1915;
      7'd48: tw_fwd = 12'd2319;
      7'd49: tw_fwd = 12'd1435;
      7'd50: tw_fwd = 12'd807;
      7'd51: tw_fwd = 12'd452;
      7'd52: tw_fwd = 12'd1438;
      7'd53: tw_fwd = 12'd2868;
      7'd54: tw_fwd = 12'd1534;
      7'd55: tw_fwd = 12'd2402;
      7'd56: tw_fwd = 12'd2647;
      7'd57: tw_fwd = 12'd2617;
      7'd58: tw_fwd = 12'd1481;
      7'd59: tw_fwd = 12'd648;
      7'd60: tw_fwd = 12'd2474;
      7'd61: tw_fwd = 12'd3110;
      7'd62: tw_fwd = 12'd1227;
      7'd63: tw_fwd = 12'd910;
      7'd64: tw_fwd = 12'd17;
      7'd65: tw_fwd = 12'd2761;
      7'd66: tw_fwd = 12'd583;
      7'd67: tw_fwd = 12'd2649;
      7'd68: tw_fwd = 12'd1637;
      7'd69: tw_fwd = 12'd723;
      7'd70: tw_fwd = 12'd2288;
      7'd71: tw_fwd = 12'd1100;
      7'd72: tw_fwd = 12'd1409;
      7'd73: tw_fwd = 12'd2662;
      7'd74: tw_fwd = 12'd3281;
      7'd75: tw_fwd = 12'd233;
      7'd76: tw_fwd = 12'd756;
      7'd77: tw_fwd = 12'd2156;
      7'd78: tw_fwd = 12'd3015;
      7'd79: tw_fwd = 12'd3050;
      7'd80: tw_fwd = 12'd1703;
      7'd81: tw_fwd = 12'd1651;
      7'd82: tw_fwd = 12'd2789;
      7'd83: tw_fwd = 12'd1789;
      7'd84: tw_fwd = 12'd1847;
      7'd85: tw_fwd = 12'd952;
      7'd86: tw_fwd = 12'd1461;
      7'd87: tw_fwd = 12'd2687;
      7'd88: tw_fwd = 12'd939;
      7'd89: tw_fwd = 12'd2308;
      7'd90: tw_fwd = 12'd2437;
      7'd91: tw_fwd = 12'd2388;
      7'd92: tw_fwd = 12'd733;
      7'd93: tw_fwd = 12'd2337;
      7'd94: tw_fwd = 12'd268;
      7'd95: tw_fwd = 12'd641;
      7'd96: tw_fwd = 12'd1584;
      7'd97: tw_fwd = 12'd2298;
      7'd98: tw_fwd = 12'd2037;
      7'd99: tw_fwd = 12'd3220;
      7'd100: tw_fwd = 12'd375;
      7'd101: tw_fwd = 12'd2549;
      7'd102: tw_fwd = 12'd2090;
      7'd103: tw_fwd = 12'd1645;
      7'd104: tw_fwd = 12'd1063;
      7'd105: tw_fwd = 12'd319;
      7'd106: tw_fwd = 12'd2773;
      7'd107: tw_fwd = 12'd757;
      7'd108: tw_fwd = 12'd2099;
      7'd109: tw_fwd = 12'd561;
      7'd110: tw_fwd = 12'd2466;
      7'd111: tw_fwd = 12'd2594;
      7'd112: tw_fwd = 12'd2804;
      7'd113: tw_fwd = 12'd1092;
      7'd114: tw_fwd = 12'd403;
      7'd115: tw_fwd = 12'd1026;
      7'd116: tw_fwd = 12'd1143;
      7'd117: tw_fwd = 12'd2150;
      7'd118: tw_fwd = 12'd2775;
      7'd119: tw_fwd = 12'd886;
      7'd120: tw_fwd = 12'd1722;
      7'd121: tw_fwd = 12'd1212;
      7'd122: tw_fwd = 12'd1874;
      7'd123: tw_fwd = 12'd1029;
      7'd124: tw_fwd = 12'd2110;
      7'd125: tw_fwd = 12'd2935;
      7'd126: tw_fwd = 12'd885;
      7'd127: tw_fwd = 12'd2154;
      default: tw_fwd = 12'd0;
    endcase
  endfunction
  function automatic logic [11:0] tw_inv(input logic [6:0] k);
    case (k)
      7'd0: tw_inv = 12'd1175;
      7'd1: tw_inv = 12'd2444;
      7'd2: tw_inv = 12'd394;
      7'd3: tw_inv = 12'd1219;
      7'd4: tw_inv = 12'd2300;
      7'd5: tw_inv = 12'd1455;
      7'd6: tw_inv = 12'd2117;
      7'd7: tw_inv = 12'd1607;
      7'd8: tw_inv = 12'd2443;
      7'd9: tw_inv = 12'd554;
      7'd10: tw_inv = 12'd1179;
      7'd11: tw_inv = 12'd2186;
      7'd12: tw_inv = 12'd2303;
      7'd13: tw_inv = 12'd2926;
      7'd14: tw_inv = 12'd2237;
      7'd15: tw_inv = 12'd525;
      7'd16: tw_inv = 12'd735;
      7'd17: tw_inv = 12'd863;
      7'd18: tw_inv = 12'd2768;
      7'd19: tw_inv = 12'd1230;
      7'd20: tw_inv = 12'd2572;
      7'd21: tw_inv = 12'd556;
      7'd22: tw_inv = 12'd3010;
      7'd23: tw_inv = 12'd2266;
      7'd24: tw_inv = 12'd1684;
      7'd25: tw_inv = 12'd1239;
      7'd26: tw_inv = 12'd780;
      7'd27: tw_inv = 12'd2954;
      7'd28: tw_inv = 12'd109;
      7'd29: tw_inv = 12'd1292;
      7'd30: tw_inv = 12'd1031;
      7'd31: tw_inv = 12'd1745;
      7'd32: tw_inv = 12'd2688;
      7'd33: tw_inv = 12'd3061;
      7'd34: tw_inv = 12'd992;
      7'd35: tw_inv = 12'd2596;
      7'd36: tw_inv = 12'd941;
      7'd37: tw_inv = 12'd892;
      7'd38: tw_inv = 12'd1021;
      7'd39: tw_inv = 12'd2390;
      7'd40: tw_inv = 12'd642;
      7'd41: tw_inv = 12'd1868;
      7'd42: tw_inv = 12'd2377;
      7'd43: tw_inv = 12'd1482;
      7'd44: tw_inv = 12'd1540;
      7'd45: tw_inv = 12'd540;
      7'd46: tw_inv = 12'd1678;
      7'd47: tw_inv = 12'd1626;
      7'd48: tw_inv = 12'd279;
      7'd49: tw_inv = 12'd314;
      7'd50: tw_inv = 12'd1173;
      7'd51: tw_inv = 12'd2573;
      7'd52: tw_inv = 12'd3096;
      7'd53: tw_inv = 12'd48;
      7'd54: tw_inv = 12'd667;
      7'd55: tw_inv = 12'd1920;
      7'd56: tw_inv = 12'd2229;
      7'd57: tw_inv = 12'd1041;
      7'd58: tw_inv = 12'd2606;
      7'd59: tw_inv = 12'd1692;
      7'd60: tw_inv = 12'd680;
      7'd61: tw_inv = 12'd2746;
      7'd62: tw_inv = 12'd568;
      7'd63: tw_inv = 12'd3312;
      7'd64: tw_inv = 12'd2419;
      7'd65: tw_inv = 12'd2102;
      7'd66: tw_inv = 12'd219;
      7'd67: tw_inv = 12'd855;
      7'd68: tw_inv = 12'd2681;
      7'd69: tw_inv = 12'd1848;
      7'd70: tw_inv = 12'd712;
      7'd71: tw_inv = 12'd682;
      7'd72: tw_inv = 12'd927;
      7'd73: tw_inv = 12'd1795;
      7'd74: tw_inv = 12'd461;
      7'd75: tw_inv = 12'd1891;
      7'd76: tw_inv = 12'd2877;
      7'd77: tw_inv = 12'd2522;
      7'd78: tw_inv = 12'd1894;
      7'd79: tw_inv = 12'd1010;
      7'd80: tw_inv = 12'd1414;
      7'd81: tw_inv = 12'd2009;
      7'd82: tw_inv = 12'd3296;
      7'd83: tw_inv = 12'd464;
      7'd84: tw_inv = 12'd2697;
      7'd85: tw_inv = 12'd816;
      7'd86: tw_inv = 12'd1352;
      7'd87: tw_inv = 12'd2679;
      7'd88: tw_inv = 12'd1274;
      7'd89: tw_inv = 12'd1052;
      7'd90: tw_inv = 12'd1025;
      7'd91: tw_inv = 12'd2132;
      7'd92: tw_inv = 12'd1573;
      7'd93: tw_inv = 12'd76;
      7'd94: tw_inv = 12'd2998;
      7'd95: tw_inv = 12'd3040;
      7'd96: tw_inv = 12'd2508;
      7'd97: tw_inv = 12'd1355;
      7'd98: tw_inv = 12'd450;
      7'd99: tw_inv = 12'd936;
      7'd100: tw_inv = 12'd447;
      7'd101: tw_inv = 12'd2794;
      7'd102: tw_inv = 12'd1235;
      7'd103: tw_inv = 12'd1903;
      7'd104: tw_inv = 12'd1996;
      7'd105: tw_inv = 12'd1089;
      7'd106: tw_inv = 12'd3273;
      7'd107: tw_inv = 12'd283;
      7'd108: tw_inv = 12'd1853;
      7'd109: tw_inv = 12'd1990;
      7'd110: tw_inv = 12'd882;
      7'd111: tw_inv = 12'd3033;
      7'd112: tw_inv = 12'd1583;
      7'd113: tw_inv = 12'd2760;
      7'd114: tw_inv = 12'd69;
      7'd115: tw_inv = 12'd543;
      7'd116: tw_inv = 12'd2532;
      7'd117: tw_inv = 12'd3136;
      7'd118: tw_inv = 12'd1410;
      7'd119: tw_inv = 12'd2267;
      7'd120: tw_inv = 12'd2481;
      7'd121: tw_inv = 12'd1432;
      7'd122: tw_inv = 12'd2699;
      7'd123: tw_inv = 12'd687;
      7'd124: tw_inv = 12'd40;
      7'd125: tw_inv = 12'd749;
      7'd126: tw_inv = 12'd1677;
      default: tw_inv = 12'd0;
    endcase
  endfunction
  function automatic logic [11:0] slot_zeta(input logic [6:0] k);
    case (k)
      7'd0: slot_zeta = 12'd17;
      7'd1: slot_zeta = 12'd3312;
      7'd2: slot_zeta = 12'd2761;
      7'd3: slot_zeta = 12'd568;
      7'd4: slot_zeta = 12'd583;
      7'd5: slot_zeta = 12'd2746;
      7'd6: slot_zeta = 12'd2649;
      7'd7: slot_zeta = 12'd680;
      7'd8: slot_zeta = 12'd1637;
      7'd9: slot_zeta = 12'd1692;
      7'd10: slot_zeta = 12'd723;
      7'd11: slot_zeta = 12'd2606;
      7'd12: slot_zeta = 12'd2288;
      7'd13: slot_zeta = 12'd1041;
      7'd14: slot_zeta = 12'd1100;
      7'd15: slot_zeta = 12'd2229;
      7'd16: slot_zeta = 12'd1409;
      7'd17: slot_zeta = 12'd1920;
      7'd18: slot_zeta = 12'd2662;
      7'd19: slot_zeta = 12'd667;
      7'd20: slot_zeta = 12'd3281;
      7'd21: slot_zeta = 12'd48;
      7'd22: slot_zeta = 12'd233;
      7'd23: slot_zeta = 12'd3096;
      7'd24: slot_zeta = 12'd756;
      7'd25: slot_zeta = 12'd2573;
      7'd26: slot_zeta = 12'd2156;
      7'd27: slot_zeta = 12'd1173;
      7'd28: slot_zeta = 12'd3015;
      7'd29: slot_zeta = 12'd314;
      7'd30: slot_zeta = 12'd3050;
      7'd31: slot_zeta = 12'd279;
      7'd32: slot_zeta = 12'd1703;
      7'd33: slot_zeta = 12'd1626;
      7'd34: slot_zeta = 12'd1651;
      7'd35: slot_zeta = 12'd1678;
      7'd36: slot_zeta = 12'd2789;
      7'd37: slot_zeta = 12'd540;
      7'd38: slot_zeta = 12'd1789;
      7'd39: slot_zeta = 12'd1540;
      7'd40: slot_zeta = 12'd1847;
      7'd41: slot_zeta = 12'd1482;
      7'd42: slot_zeta = 12'd952;
      7'd43: slot_zeta = 12'd2377;
      7'd44: slot_zeta = 12'd1461;
      7'd45: slot_zeta = 12'd1868;
      7'd46: slot_zeta = 12'd2687;
      7'd47: slot_zeta = 12'd642;
      7'd48: slot_zeta = 12'd939;
      7'd49: slot_zeta = 12'd2390;
      7'd50: slot_zeta = 12'd2308;
      7'd51: slot_zeta = 12'd1021;
      7'd52: slot_zeta = 12'd2437;
      7'd53: slot_zeta = 12'd892;
      7'd54: slot_zeta = 12'd2388;
      7'd55: slot_zeta = 12'd941;
      7'd56: slot_zeta = 12'd733;
      7'd57: slot_zeta = 12'd2596;
      7'd58: slot_zeta = 12'd2337;
      7'd59: slot_zeta = 12'd992;
      7'd60: slot_zeta = 12'd268;
      7'd61: slot_zeta = 12'd3061;
      7'd62: slot_zeta = 12'd641;
      7'd63: slot_zeta = 12'd2688;
      7'd64: slot_zeta = 12'd1584;
      7'd65: slot_zeta = 12'd1745;
      7'd66: slot_zeta = 12'd2298;
      7'd67: slot_zeta = 12'd1031;
      7'd68: slot_zeta = 12'd2037;
      7'd69: slot_zeta = 12'd1292;
      7'd70: slot_zeta = 12'd3220;
      7'd71: slot_zeta = 12'd109;
      7'd72: slot_zeta = 12'd375;
      7'd73: slot_zeta = 12'd2954;
      7'd74: slot_zeta = 12'd2549;
      7'd75: slot_zeta = 12'd780;
      7'd76: slot_zeta = 12'd2090;
      7'd77: slot_zeta = 12'd1239;
      7'd78: slot_zeta = 12'd1645;
      7'd79: slot_zeta = 12'd1684;
      7'd80: slot_zeta = 12'd1063;
      7'd81: slot_zeta = 12'd2266;
      7'd82: slot_zeta = 12'd319;
      7'd83: slot_zeta = 12'd3010;
      7'd84: slot_zeta = 12'd2773;
      7'd85: slot_zeta = 12'd556;
      7'd86: slot_zeta = 12'd757;
      7'd87: slot_zeta = 12'd2572;
      7'd88: slot_zeta = 12'd2099;
      7'd89: slot_zeta = 12'd1230;
      7'd90: slot_zeta = 12'd561;
      7'd91: slot_zeta = 12'd2768;
      7'd92: slot_zeta = 12'd2466;
      7'd93: slot_zeta = 12'd863;
      7'd94: slot_zeta = 12'd2594;
      7'd95: slot_zeta = 12'd735;
      7'd96: slot_zeta = 12'd2804;
      7'd97: slot_zeta = 12'd525;
      7'd98: slot_zeta = 12'd1092;
      7'd99: slot_zeta = 12'd2237;
      7'd100: slot_zeta = 12'd403;
      7'd101: slot_zeta = 12'd2926;
      7'd102: slot_zeta = 12'd1026;
      7'd103: slot_zeta = 12'd2303;
      7'd104: slot_zeta = 12'd1143;
      7'd105: slot_zeta = 12'd2186;
      7'd106: slot_zeta = 12'd2150;
      7'd107: slot_zeta = 12'd1179;
      7'd108: slot_zeta = 12'd2775;
      7'd109: slot_zeta = 12'd554;
      7'd110: slot_zeta = 12'd886;
      7'd111: slot_zeta = 12'd2443;
      7'd112: slot_zeta = 12'd1722;
      7'd113: slot_zeta = 12'd1607;
      7'd114: slot_zeta = 12'd1212;
      7'd115: slot_zeta = 12'd2117;
      7'd116: slot_zeta = 12'd1874;
      7'd117: slot_zeta = 12'd1455;
      7'd118: slot_zeta = 12'd1029;
      7'd119: slot_zeta = 12'd2300;
      7'd120: slot_zeta = 12'd2110;
      7'd121: slot_zeta = 12'd1219;
      7'd122: slot_zeta = 12'd2935;
      7'd123: slot_zeta = 12'd394;
      7'd124: slot_zeta = 12'd885;
      7'd125: slot_zeta = 12'd2444;
      7'd126: slot_zeta = 12'd2154;
      7'd127: slot_zeta = 12'd1175;
      default: slot_zeta = 12'd0;
    endcase
  endfunction
endpackage
