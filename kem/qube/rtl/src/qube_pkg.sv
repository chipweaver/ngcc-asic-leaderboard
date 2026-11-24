// Generated parameter tables of the five QUBE sets.
package qube_pkg;
  localparam int W = 512;              // lane / buffer word width (bits)
  localparam int WB = 64;              // bytes per word
  localparam int NWMAX = 266;           // max dense words (QUBE-512)
  localparam int SLOT_AW = 9;          // slot / ACC address width
  localparam int OBJ_AW = 10;          // object buffer word address width (0..543)
  localparam int SUP_AW = 10;          // support memory address width
  localparam int WMAX = 324;            // max fixed weight (CAM depth)
  localparam int N1MAX = 150;
  localparam int DMAX = 43;            // max RS correction capacity delta
  localparam int GMAX = 86;            // max generator degree 2*delta
  localparam int KMAX = 64;
  localparam logic [255:0] SM3_IV0 = 256'h7380166f4914b2b9172442d7da8a0600a96f30bc163138aae38dee4db0fb0e4e;
  localparam logic [255:0] SM3_IV1 = 256'h313eb6a448bef2d536e50983d44b6bab9f2008548371a799073ed128d0a8dd79;  // compress(IV0, key^ipad)
  localparam logic [255:0] SM3_IV2 = 256'h747cc75de966127aa24ac00f62ab5c3f1dd5568d84c3306505fff7c2b97f411a;  // compress(IV0, key^opad)
  // ring length n
  function automatic logic [17:0] p_n(input logic [2:0] ps);
    case (ps)
      3'd0: p_n = 18'd13109;
      3'd1: p_n = 18'd25357;
      3'd2: p_n = 18'd41651;
      3'd3: p_n = 18'd82757;
      3'd4: p_n = 18'd135851;
      default: p_n = 18'd13109;
    endcase
  endfunction
  // security bytes k (seed/salt/ss length)
  function automatic logic [6:0] p_k(input logic [2:0] ps);
    case (ps)
      3'd0: p_k = 7'd16;
      3'd1: p_k = 7'd24;
      3'd2: p_k = 7'd32;
      3'd3: p_k = 7'd48;
      3'd4: p_k = 7'd64;
      default: p_k = 7'd16;
    endcase
  endfunction
  // RS length N1
  function automatic logic [7:0] p_n1(input logic [2:0] ps);
    case (ps)
      3'd0: p_n1 = 8'd34;
      3'd1: p_n1 = 8'd66;
      3'd2: p_n1 = 8'd108;
      3'd3: p_n1 = 8'd128;
      3'd4: p_n1 = 8'd150;
      default: p_n1 = 8'd34;
    endcase
  endfunction
  // RM repetitions N2/128
  function automatic logic [2:0] p_mult(input logic [2:0] ps);
    case (ps)
      3'd0: p_mult = 3'd3;
      3'd1: p_mult = 3'd3;
      3'd2: p_mult = 3'd3;
      3'd3: p_mult = 3'd5;
      3'd4: p_mult = 3'd7;
      default: p_mult = 3'd3;
    endcase
  endfunction
  // RS correction capacity
  function automatic logic [5:0] p_delta(input logic [2:0] ps);
    case (ps)
      3'd0: p_delta = 6'd9;
      3'd1: p_delta = 6'd21;
      3'd2: p_delta = 6'd38;
      3'd3: p_delta = 6'd40;
      3'd4: p_delta = 6'd43;
      default: p_delta = 6'd9;
    endcase
  endfunction
  // candidate bits ceil(log2 n)
  function automatic logic [4:0] p_nbits(input logic [2:0] ps);
    case (ps)
      3'd0: p_nbits = 5'd14;
      3'd1: p_nbits = 5'd15;
      3'd2: p_nbits = 5'd16;
      3'd3: p_nbits = 5'd17;
      3'd4: p_nbits = 5'd18;
      default: p_nbits = 5'd14;
    endcase
  endfunction
  // dense words ceil(n/512)
  function automatic logic [8:0] p_nw(input logic [2:0] ps);
    case (ps)
      3'd0: p_nw = 9'd26;
      3'd1: p_nw = 9'd50;
      3'd2: p_nw = 9'd82;
      3'd3: p_nw = 9'd162;
      3'd4: p_nw = 9'd266;
      default: p_nw = 9'd26;
    endcase
  endfunction
  // 1024-bit lane words per dense vector (2 * p_nw(ps) == 2 * this: each lane word is two 512-bit half-words)
  function automatic logic [7:0] p_nwk(input logic [2:0] ps);
    case (ps)
      3'd0: p_nwk = 8'd13;
      3'd1: p_nwk = 8'd25;
      3'd2: p_nwk = 8'd41;
      3'd3: p_nwk = 8'd81;
      3'd4: p_nwk = 8'd133;
      default: p_nwk = 8'd13;
    endcase
  endfunction
  // valid bits in the last 1024-bit lane word (= 512 + p_lastbits)
  function automatic logic [9:0] p_lastk(input logic [2:0] ps);
    case (ps)
      3'd0: p_lastk = 10'd821;
      3'd1: p_lastk = 10'd781;
      3'd2: p_lastk = 10'd691;
      3'd3: p_lastk = 10'd837;
      3'd4: p_lastk = 10'd683;
      default: p_lastk = 10'd821;
    endcase
  endfunction
  // valid bits in the last dense word
  function automatic logic [8:0] p_lastbits(input logic [2:0] ps);
    case (ps)
      3'd0: p_lastbits = 9'd309;
      3'd1: p_lastbits = 9'd269;
      3'd2: p_lastbits = 9'd179;
      3'd3: p_lastbits = 9'd325;
      3'd4: p_lastbits = 9'd171;
      default: p_lastbits = 9'd309;
    endcase
  endfunction
  // bytes of an n-bit vector
  function automatic logic [14:0] p_nb(input logic [2:0] ps);
    case (ps)
      3'd0: p_nb = 15'd1639;
      3'd1: p_nb = 15'd3170;
      3'd2: p_nb = 15'd5207;
      3'd3: p_nb = 15'd10345;
      3'd4: p_nb = 15'd16982;
      default: p_nb = 15'd1639;
    endcase
  endfunction
  // bytes of v (N1*N2/8)
  function automatic logic [14:0] p_nv(input logic [2:0] ps);
    case (ps)
      3'd0: p_nv = 15'd1632;
      3'd1: p_nv = 15'd3168;
      3'd2: p_nv = 15'd5184;
      3'd3: p_nv = 15'd10240;
      3'd4: p_nv = 15'd16800;
      default: p_nv = 15'd1632;
    endcase
  endfunction
  // words of v
  function automatic logic [8:0] p_nwv(input logic [2:0] ps);
    case (ps)
      3'd0: p_nwv = 9'd26;
      3'd1: p_nwv = 9'd50;
      3'd2: p_nwv = 9'd81;
      3'd3: p_nwv = 9'd160;
      3'd4: p_nwv = 9'd263;
      default: p_nwv = 9'd26;
    endcase
  endfunction
  // pk bytes
  function automatic logic [15:0] p_pkl(input logic [2:0] ps);
    case (ps)
      3'd0: p_pkl = 16'd3294;
      3'd1: p_pkl = 16'd6364;
      3'd2: p_pkl = 16'd10446;
      3'd3: p_pkl = 16'd20738;
      3'd4: p_pkl = 16'd34028;
      default: p_pkl = 16'd3294;
    endcase
  endfunction
  // sk bytes
  function automatic logic [15:0] p_skl(input logic [2:0] ps);
    case (ps)
      3'd0: p_skl = 16'd3326;
      3'd1: p_skl = 16'd6412;
      3'd2: p_skl = 16'd10510;
      3'd3: p_skl = 16'd20834;
      3'd4: p_skl = 16'd34156;
      default: p_skl = 16'd3326;
    endcase
  endfunction
  // ct bytes
  function automatic logic [15:0] p_ctl(input logic [2:0] ps);
    case (ps)
      3'd0: p_ctl = 16'd3287;
      3'd1: p_ctl = 16'd6362;
      3'd2: p_ctl = 16'd10423;
      3'd3: p_ctl = 16'd20633;
      3'd4: p_ctl = 16'd33846;
      default: p_ctl = 16'd3287;
    endcase
  endfunction
  // weight of y
  function automatic logic [8:0] p_wy(input logic [2:0] ps);
    case (ps)
      3'd0: p_wy = 9'd76;
      3'd1: p_wy = 9'd113;
      3'd2: p_wy = 9'd151;
      3'd3: p_wy = 9'd226;
      3'd4: p_wy = 9'd301;
      default: p_wy = 9'd76;
    endcase
  endfunction
  // weight of x1, x2
  function automatic logic [8:0] p_wx(input logic [2:0] ps);
    case (ps)
      3'd0: p_wx = 9'd74;
      3'd1: p_wx = 9'd110;
      3'd2: p_wx = 9'd146;
      3'd3: p_wx = 9'd219;
      3'd4: p_wx = 9'd291;
      default: p_wx = 9'd74;
    endcase
  endfunction
  // weight of f1, f2 (omega_r21 = omega_r22)
  function automatic logic [8:0] p_wr(input logic [2:0] ps);
    case (ps)
      3'd0: p_wr = 9'd29;
      3'd1: p_wr = 9'd43;
      3'd2: p_wr = 9'd57;
      3'd3: p_wr = 9'd85;
      3'd4: p_wr = 9'd113;
      default: p_wr = 9'd29;
    endcase
  endfunction
  // weight of e
  function automatic logic [8:0] p_we(input logic [2:0] ps);
    case (ps)
      3'd0: p_we = 9'd83;
      3'd1: p_we = 9'd121;
      3'd2: p_we = 9'd162;
      3'd3: p_we = 9'd244;
      3'd4: p_we = 9'd324;
      default: p_we = 9'd83;
    endcase
  endfunction
  // weight of g (omega_r11)
  function automatic logic [8:0] p_wg(input logic [2:0] ps);
    case (ps)
      3'd0: p_wg = 9'd27;
      3'd1: p_wg = 9'd39;
      3'd2: p_wg = 9'd52;
      3'd3: p_wg = 9'd78;
      3'd4: p_wg = 9'd103;
      default: p_wg = 9'd27;
    endcase
  endfunction
  // hash variant for 2k-byte outputs (PRG, G)
  function automatic logic [2:0] p_var2k(input logic [2:0] ps);
    case (ps)
      3'd0: p_var2k = 3'd1;
      3'd1: p_var2k = 3'd2;
      3'd2: p_var2k = 3'd2;
      3'd3: p_var2k = 3'd3;
      3'd4: p_var2k = 3'd4;
      default: p_var2k = 3'd1;
    endcase
  endfunction
  // hash variant for k-byte outputs (H, J)
  function automatic logic [2:0] p_vark(input logic [2:0] ps);
    case (ps)
      3'd0: p_vark = 3'd1;
      3'd1: p_vark = 3'd1;
      3'd2: p_vark = 3'd1;
      3'd3: p_vark = 3'd2;
      3'd4: p_vark = 3'd2;
      default: p_vark = 3'd1;
    endcase
  endfunction
  // RS generator polynomial coefficient g[j] (j < 2*delta), g[2*delta] = 1
  function automatic logic [7:0] p_rsgen(input logic [2:0] ps, input logic [6:0] j);
    p_rsgen = 8'd0;
    case (ps)
      3'd0: case (j)
        7'd0: p_rsgen = 8'd179;
        7'd1: p_rsgen = 8'd31;
        7'd2: p_rsgen = 8'd62;
        7'd3: p_rsgen = 8'd180;
        7'd4: p_rsgen = 8'd25;
        7'd5: p_rsgen = 8'd116;
        7'd6: p_rsgen = 8'd230;
        7'd7: p_rsgen = 8'd99;
        7'd8: p_rsgen = 8'd141;
        7'd9: p_rsgen = 8'd254;
        7'd10: p_rsgen = 8'd171;
        7'd11: p_rsgen = 8'd51;
        7'd12: p_rsgen = 8'd136;
        7'd13: p_rsgen = 8'd87;
        7'd14: p_rsgen = 8'd67;
        7'd15: p_rsgen = 8'd209;
        7'd16: p_rsgen = 8'd203;
        7'd17: p_rsgen = 8'd195;
        default: p_rsgen = 8'd0;
      endcase
      3'd1: case (j)
        7'd0: p_rsgen = 8'd33;
        7'd1: p_rsgen = 8'd163;
        7'd2: p_rsgen = 8'd213;
        7'd3: p_rsgen = 8'd243;
        7'd4: p_rsgen = 8'd28;
        7'd5: p_rsgen = 8'd81;
        7'd6: p_rsgen = 8'd115;
        7'd7: p_rsgen = 8'd203;
        7'd8: p_rsgen = 8'd39;
        7'd9: p_rsgen = 8'd38;
        7'd10: p_rsgen = 8'd245;
        7'd11: p_rsgen = 8'd250;
        7'd12: p_rsgen = 8'd92;
        7'd13: p_rsgen = 8'd153;
        7'd14: p_rsgen = 8'd216;
        7'd15: p_rsgen = 8'd133;
        7'd16: p_rsgen = 8'd185;
        7'd17: p_rsgen = 8'd198;
        7'd18: p_rsgen = 8'd114;
        7'd19: p_rsgen = 8'd148;
        7'd20: p_rsgen = 8'd229;
        7'd21: p_rsgen = 8'd237;
        7'd22: p_rsgen = 8'd102;
        7'd23: p_rsgen = 8'd166;
        7'd24: p_rsgen = 8'd6;
        7'd25: p_rsgen = 8'd131;
        7'd26: p_rsgen = 8'd17;
        7'd27: p_rsgen = 8'd51;
        7'd28: p_rsgen = 8'd167;
        7'd29: p_rsgen = 8'd251;
        7'd30: p_rsgen = 8'd142;
        7'd31: p_rsgen = 8'd47;
        7'd32: p_rsgen = 8'd135;
        7'd33: p_rsgen = 8'd58;
        7'd34: p_rsgen = 8'd22;
        7'd35: p_rsgen = 8'd168;
        7'd36: p_rsgen = 8'd143;
        7'd37: p_rsgen = 8'd96;
        7'd38: p_rsgen = 8'd251;
        7'd39: p_rsgen = 8'd18;
        7'd40: p_rsgen = 8'd26;
        7'd41: p_rsgen = 8'd216;
        default: p_rsgen = 8'd0;
      endcase
      3'd2: case (j)
        7'd0: p_rsgen = 8'd118;
        7'd1: p_rsgen = 8'd109;
        7'd2: p_rsgen = 8'd166;
        7'd3: p_rsgen = 8'd135;
        7'd4: p_rsgen = 8'd30;
        7'd5: p_rsgen = 8'd111;
        7'd6: p_rsgen = 8'd99;
        7'd7: p_rsgen = 8'd193;
        7'd8: p_rsgen = 8'd209;
        7'd9: p_rsgen = 8'd233;
        7'd10: p_rsgen = 8'd199;
        7'd11: p_rsgen = 8'd91;
        7'd12: p_rsgen = 8'd222;
        7'd13: p_rsgen = 8'd18;
        7'd14: p_rsgen = 8'd131;
        7'd15: p_rsgen = 8'd176;
        7'd16: p_rsgen = 8'd138;
        7'd17: p_rsgen = 8'd72;
        7'd18: p_rsgen = 8'd58;
        7'd19: p_rsgen = 8'd69;
        7'd20: p_rsgen = 8'd167;
        7'd21: p_rsgen = 8'd38;
        7'd22: p_rsgen = 8'd119;
        7'd23: p_rsgen = 8'd2;
        7'd24: p_rsgen = 8'd217;
        7'd25: p_rsgen = 8'd149;
        7'd26: p_rsgen = 8'd246;
        7'd27: p_rsgen = 8'd117;
        7'd28: p_rsgen = 8'd245;
        7'd29: p_rsgen = 8'd1;
        7'd30: p_rsgen = 8'd125;
        7'd31: p_rsgen = 8'd78;
        7'd32: p_rsgen = 8'd44;
        7'd33: p_rsgen = 8'd21;
        7'd34: p_rsgen = 8'd187;
        7'd35: p_rsgen = 8'd48;
        7'd36: p_rsgen = 8'd255;
        7'd37: p_rsgen = 8'd106;
        7'd38: p_rsgen = 8'd171;
        7'd39: p_rsgen = 8'd43;
        7'd40: p_rsgen = 8'd117;
        7'd41: p_rsgen = 8'd40;
        7'd42: p_rsgen = 8'd96;
        7'd43: p_rsgen = 8'd232;
        7'd44: p_rsgen = 8'd39;
        7'd45: p_rsgen = 8'd32;
        7'd46: p_rsgen = 8'd158;
        7'd47: p_rsgen = 8'd101;
        7'd48: p_rsgen = 8'd72;
        7'd49: p_rsgen = 8'd50;
        7'd50: p_rsgen = 8'd19;
        7'd51: p_rsgen = 8'd224;
        7'd52: p_rsgen = 8'd148;
        7'd53: p_rsgen = 8'd118;
        7'd54: p_rsgen = 8'd177;
        7'd55: p_rsgen = 8'd203;
        7'd56: p_rsgen = 8'd113;
        7'd57: p_rsgen = 8'd39;
        7'd58: p_rsgen = 8'd142;
        7'd59: p_rsgen = 8'd66;
        7'd60: p_rsgen = 8'd105;
        7'd61: p_rsgen = 8'd2;
        7'd62: p_rsgen = 8'd149;
        7'd63: p_rsgen = 8'd107;
        7'd64: p_rsgen = 8'd17;
        7'd65: p_rsgen = 8'd40;
        7'd66: p_rsgen = 8'd4;
        7'd67: p_rsgen = 8'd20;
        7'd68: p_rsgen = 8'd154;
        7'd69: p_rsgen = 8'd81;
        7'd70: p_rsgen = 8'd54;
        7'd71: p_rsgen = 8'd94;
        7'd72: p_rsgen = 8'd29;
        7'd73: p_rsgen = 8'd197;
        7'd74: p_rsgen = 8'd44;
        7'd75: p_rsgen = 8'd225;
        default: p_rsgen = 8'd0;
      endcase
      3'd3: case (j)
        7'd0: p_rsgen = 8'd150;
        7'd1: p_rsgen = 8'd125;
        7'd2: p_rsgen = 8'd39;
        7'd3: p_rsgen = 8'd14;
        7'd4: p_rsgen = 8'd108;
        7'd5: p_rsgen = 8'd164;
        7'd6: p_rsgen = 8'd211;
        7'd7: p_rsgen = 8'd142;
        7'd8: p_rsgen = 8'd155;
        7'd9: p_rsgen = 8'd172;
        7'd10: p_rsgen = 8'd139;
        7'd11: p_rsgen = 8'd88;
        7'd12: p_rsgen = 8'd255;
        7'd13: p_rsgen = 8'd3;
        7'd14: p_rsgen = 8'd167;
        7'd15: p_rsgen = 8'd79;
        7'd16: p_rsgen = 8'd254;
        7'd17: p_rsgen = 8'd6;
        7'd18: p_rsgen = 8'd39;
        7'd19: p_rsgen = 8'd18;
        7'd20: p_rsgen = 8'd20;
        7'd21: p_rsgen = 8'd62;
        7'd22: p_rsgen = 8'd213;
        7'd23: p_rsgen = 8'd116;
        7'd24: p_rsgen = 8'd74;
        7'd25: p_rsgen = 8'd206;
        7'd26: p_rsgen = 8'd227;
        7'd27: p_rsgen = 8'd72;
        7'd28: p_rsgen = 8'd204;
        7'd29: p_rsgen = 8'd156;
        7'd30: p_rsgen = 8'd228;
        7'd31: p_rsgen = 8'd161;
        7'd32: p_rsgen = 8'd227;
        7'd33: p_rsgen = 8'd206;
        7'd34: p_rsgen = 8'd48;
        7'd35: p_rsgen = 8'd254;
        7'd36: p_rsgen = 8'd13;
        7'd37: p_rsgen = 8'd217;
        7'd38: p_rsgen = 8'd233;
        7'd39: p_rsgen = 8'd237;
        7'd40: p_rsgen = 8'd164;
        7'd41: p_rsgen = 8'd37;
        7'd42: p_rsgen = 8'd187;
        7'd43: p_rsgen = 8'd208;
        7'd44: p_rsgen = 8'd156;
        7'd45: p_rsgen = 8'd25;
        7'd46: p_rsgen = 8'd40;
        7'd47: p_rsgen = 8'd80;
        7'd48: p_rsgen = 8'd148;
        7'd49: p_rsgen = 8'd134;
        7'd50: p_rsgen = 8'd206;
        7'd51: p_rsgen = 8'd198;
        7'd52: p_rsgen = 8'd255;
        7'd53: p_rsgen = 8'd25;
        7'd54: p_rsgen = 8'd222;
        7'd55: p_rsgen = 8'd179;
        7'd56: p_rsgen = 8'd76;
        7'd57: p_rsgen = 8'd99;
        7'd58: p_rsgen = 8'd122;
        7'd59: p_rsgen = 8'd26;
        7'd60: p_rsgen = 8'd155;
        7'd61: p_rsgen = 8'd40;
        7'd62: p_rsgen = 8'd37;
        7'd63: p_rsgen = 8'd224;
        7'd64: p_rsgen = 8'd149;
        7'd65: p_rsgen = 8'd170;
        7'd66: p_rsgen = 8'd66;
        7'd67: p_rsgen = 8'd109;
        7'd68: p_rsgen = 8'd112;
        7'd69: p_rsgen = 8'd220;
        7'd70: p_rsgen = 8'd68;
        7'd71: p_rsgen = 8'd16;
        7'd72: p_rsgen = 8'd255;
        7'd73: p_rsgen = 8'd92;
        7'd74: p_rsgen = 8'd109;
        7'd75: p_rsgen = 8'd147;
        7'd76: p_rsgen = 8'd66;
        7'd77: p_rsgen = 8'd128;
        7'd78: p_rsgen = 8'd38;
        7'd79: p_rsgen = 8'd168;
        default: p_rsgen = 8'd0;
      endcase
      3'd4: case (j)
        7'd0: p_rsgen = 8'd179;
        7'd1: p_rsgen = 8'd35;
        7'd2: p_rsgen = 8'd211;
        7'd3: p_rsgen = 8'd94;
        7'd4: p_rsgen = 8'd14;
        7'd5: p_rsgen = 8'd148;
        7'd6: p_rsgen = 8'd49;
        7'd7: p_rsgen = 8'd213;
        7'd8: p_rsgen = 8'd20;
        7'd9: p_rsgen = 8'd97;
        7'd10: p_rsgen = 8'd200;
        7'd11: p_rsgen = 8'd125;
        7'd12: p_rsgen = 8'd44;
        7'd13: p_rsgen = 8'd215;
        7'd14: p_rsgen = 8'd189;
        7'd15: p_rsgen = 8'd14;
        7'd16: p_rsgen = 8'd91;
        7'd17: p_rsgen = 8'd45;
        7'd18: p_rsgen = 8'd113;
        7'd19: p_rsgen = 8'd83;
        7'd20: p_rsgen = 8'd105;
        7'd21: p_rsgen = 8'd205;
        7'd22: p_rsgen = 8'd23;
        7'd23: p_rsgen = 8'd50;
        7'd24: p_rsgen = 8'd220;
        7'd25: p_rsgen = 8'd249;
        7'd26: p_rsgen = 8'd21;
        7'd27: p_rsgen = 8'd189;
        7'd28: p_rsgen = 8'd50;
        7'd29: p_rsgen = 8'd222;
        7'd30: p_rsgen = 8'd234;
        7'd31: p_rsgen = 8'd20;
        7'd32: p_rsgen = 8'd3;
        7'd33: p_rsgen = 8'd198;
        7'd34: p_rsgen = 8'd236;
        7'd35: p_rsgen = 8'd75;
        7'd36: p_rsgen = 8'd21;
        7'd37: p_rsgen = 8'd229;
        7'd38: p_rsgen = 8'd109;
        7'd39: p_rsgen = 8'd247;
        7'd40: p_rsgen = 8'd112;
        7'd41: p_rsgen = 8'd79;
        7'd42: p_rsgen = 8'd68;
        7'd43: p_rsgen = 8'd73;
        7'd44: p_rsgen = 8'd38;
        7'd45: p_rsgen = 8'd155;
        7'd46: p_rsgen = 8'd200;
        7'd47: p_rsgen = 8'd66;
        7'd48: p_rsgen = 8'd81;
        7'd49: p_rsgen = 8'd213;
        7'd50: p_rsgen = 8'd181;
        7'd51: p_rsgen = 8'd27;
        7'd52: p_rsgen = 8'd13;
        7'd53: p_rsgen = 8'd210;
        7'd54: p_rsgen = 8'd254;
        7'd55: p_rsgen = 8'd24;
        7'd56: p_rsgen = 8'd63;
        7'd57: p_rsgen = 8'd147;
        7'd58: p_rsgen = 8'd198;
        7'd59: p_rsgen = 8'd131;
        7'd60: p_rsgen = 8'd130;
        7'd61: p_rsgen = 8'd171;
        7'd62: p_rsgen = 8'd95;
        7'd63: p_rsgen = 8'd22;
        7'd64: p_rsgen = 8'd127;
        7'd65: p_rsgen = 8'd33;
        7'd66: p_rsgen = 8'd175;
        7'd67: p_rsgen = 8'd183;
        7'd68: p_rsgen = 8'd249;
        7'd69: p_rsgen = 8'd10;
        7'd70: p_rsgen = 8'd148;
        7'd71: p_rsgen = 8'd105;
        7'd72: p_rsgen = 8'd79;
        7'd73: p_rsgen = 8'd103;
        7'd74: p_rsgen = 8'd182;
        7'd75: p_rsgen = 8'd58;
        7'd76: p_rsgen = 8'd46;
        7'd77: p_rsgen = 8'd252;
        7'd78: p_rsgen = 8'd194;
        7'd79: p_rsgen = 8'd214;
        7'd80: p_rsgen = 8'd234;
        7'd81: p_rsgen = 8'd35;
        7'd82: p_rsgen = 8'd118;
        7'd83: p_rsgen = 8'd230;
        7'd84: p_rsgen = 8'd214;
        7'd85: p_rsgen = 8'd43;
        default: p_rsgen = 8'd0;
      endcase
      default: p_rsgen = 8'd0;
    endcase
  endfunction

  // GF(2^8) multiply modulo x^8+x^4+x^3+x^2+1 (0x11D)
  function automatic logic [7:0] gf_mul(input logic [7:0] x, input logic [7:0] y);
    logic [14:0] p;
    p = 15'd0;
    for (int i = 0; i < 8; i++) if (y[i]) p = p ^ ({7'd0, x} << i);
    for (int i = 14; i >= 8; i--) if (p[i]) p = p ^ (15'h11D << (i - 8));
    gf_mul = p[7:0];
  endfunction

  function automatic logic [7:0] gf_sq(input logic [7:0] x);
    gf_sq = gf_mul(x, x);
  endfunction

  // a^254 (0 -> 0), the reference gf_inverse addition chain
  function automatic logic [7:0] gf_inv(input logic [7:0] a);
    logic [7:0] inv, t1, t2;
    inv = gf_sq(a);           // a^2
    t1  = gf_mul(inv, a);     // a^3
    inv = gf_sq(inv);         // a^4
    t2  = gf_mul(inv, t1);    // a^7
    t1  = gf_mul(inv, t2);    // a^11
    inv = gf_mul(t1, inv);    // a^15
    inv = gf_sq(inv);         // a^30
    inv = gf_sq(inv);         // a^60
    inv = gf_sq(inv);         // a^120
    inv = gf_mul(inv, t2);    // a^127
    gf_inv = gf_sq(inv);      // a^254
  endfunction

  // multiply by alpha^e for a compile-time constant e (used with constant arguments only)
  function automatic logic [7:0] gf_alpha_pow(input int e);
    logic [7:0] v;
    v = 8'd1;
    for (int i = 0; i < e; i++) v = {v[6:0], 1'b0} ^ (v[7] ? 8'h1D : 8'h00);
    gf_alpha_pow = v;
  endfunction

endpackage
