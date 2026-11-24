// zen_ntt_bf : one 16-butterfly NTT stage, 2 register levels.
//
// Input (combinational, cycle c): a word pair (wa = lower word address, wb = higher), the lower word
// address and the layer configuration. Output registered: valid from cycle c+2.
//   op CT : t = mont(b*z);                 a' = a + t,  b' = a - t
//   op GS : t = mont((a-b)*z);             a' = a + b,  b' = t
//   op GF : (fused last GS layer)    a' = mont((a+b)*C1), b' = mont((a-b)*C2)   (only if HAS_SC)
//   op SC : a' = canon(mont(a*171)), b' = canon(mont(b*171))                         (only if HAS_SC)
//   op BYP: a' = a, b' = b
// Every addition/subtraction is truncated to 16 bits (int16 semantics of the reference).
// Layer types: LT_X  cross-word pair, lane i = (A[i], B[i]), one twiddle f/fn[base + (addr_a >> (ls+1))]
//              LT_W8 within-word len 8, lanes 0..7 word A, 8..15 word B, twiddle per word
//              LT_W4 within-word len 4, twiddle per half word
// Level 1: lane formation, GS pre-add/subtract, multiplications (registered products).
// Level 2: Montgomery reduction, CT add/subtract, canonicalisation, lane re-formation (registered).
// zf (SC only): per 4-coefficient group of the output words, all-zero flag.
module zen_ntt_bf #(
  parameter bit HAS_SC = 1'b0,       // stage 4: SC / GF (second constant multiplier) and BYP
  parameter int NTW = 4,             // twiddle lookups: 4 (W4 layers possible), 2 (W8), 1 (cross-word only)
  parameter int TW = 16
) (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              en,
  input  logic              in_v,
  input  logic [255:0]      in_wa,
  input  logic [255:0]      in_wb,
  input  logic [6:0]        in_addr,        // word address of wa (lower)
  input  logic [6:0]        in_addr_b,      // word address of wb
  input  logic [2:0]        in_op,
  input  logic [1:0]        in_lt,
  input  logic [2:0]        in_ls,
  input  logic [6:0]        in_base,
  input  logic              in_rom,
  input  logic [TW-1:0]     in_tag,
  output logic              out_v,
  output logic [255:0]      out_wa,
  output logic [255:0]      out_wb,
  output logic [2:0]        out_op,
  output logic [TW-1:0]     out_tag,
  output logic [7:0]        out_zf
);
  localparam logic [2:0] OP_CT = 3'd0, OP_GS = 3'd1, OP_SC = 3'd2, OP_BYP = 3'd3, OP_GF = 3'd4;
  localparam logic [1:0] LT_X = 2'd0, LT_W8 = 2'd1, LT_W4 = 2'd2;
  localparam logic signed [10:0] C1 = -11'sd114;   // 655*171*2^-16 mod 769
  localparam logic [9:0]         C2 = 10'd147;     // fn[126]*655*171*2^-32 mod 769

  function automatic logic [15:0] mont(input logic [31:0] a);
    logic [15:0] u;
    logic [31:0] uq;
    begin
      u    = a[15:0] + {a[7:0], 8'd0} - {a[5:0], 10'd0};
      uq   = {{16{u[15]}}, u} + {{7{u[15]}}, u, 9'd0} + {{8{u[15]}}, u, 8'd0};
      mont = a[31:16] - uq[31:16];
    end
  endfunction
  function automatic logic [15:0] canon(input logic [15:0] x);
    canon = x + (x[15] ? 16'd769 : 16'd0);
  endfunction

  // ---------------------------------------------------------------- level 1
  logic [15:0][15:0] la, lb, x_n, y_n;
  logic [3:0][6:0]   idx;
  logic [3:0][9:0]   rom_v, z_n;
  logic [6:0]        blk;

  always_comb begin
    blk = in_addr >> (in_ls + 3'd1);
    case (in_lt)
      LT_W8: begin
        idx[0] = in_base + in_addr;          idx[1] = in_base + in_addr;
        idx[2] = in_base + in_addr_b;        idx[3] = in_base + in_addr_b;
      end
      LT_W4: begin
        idx[0] = in_base + {in_addr[5:0], 1'b0};         idx[1] = in_base + {in_addr[5:0], 1'b1};
        idx[2] = in_base + {in_addr_b[5:0], 1'b0};       idx[3] = in_base + {in_addr_b[5:0], 1'b1};
      end
      default: begin
        idx[0] = in_base + blk; idx[1] = in_base + blk; idx[2] = in_base + blk; idx[3] = in_base + blk;
      end
    endcase
    for (int i = 0; i < 16; i++) begin
      case (in_lt)
        LT_W8: begin
          if (i < 8) begin la[i] = in_wa[16*i +: 16]; lb[i] = in_wa[16*(i+8) +: 16]; end
          else begin       la[i] = in_wb[16*(i-8) +: 16]; lb[i] = in_wb[16*i +: 16]; end
        end
        LT_W4: begin
          if (i < 4)       begin la[i] = in_wa[16*i +: 16];     lb[i] = in_wa[16*(i+4) +: 16]; end
          else if (i < 8)  begin la[i] = in_wa[16*(i+4) +: 16]; lb[i] = in_wa[16*(i+8) +: 16]; end
          else if (i < 12) begin la[i] = in_wb[16*(i-8) +: 16]; lb[i] = in_wb[16*(i-4) +: 16]; end
          else             begin la[i] = in_wb[16*(i-4) +: 16]; lb[i] = in_wb[16*i +: 16]; end
        end
        default: begin la[i] = in_wa[16*i +: 16]; lb[i] = in_wb[16*i +: 16]; end
      endcase
      case (in_op)
        OP_GS, OP_GF: begin x_n[i] = la[i] - lb[i]; y_n[i] = la[i] + lb[i]; end
        default:      begin x_n[i] = lb[i];         y_n[i] = la[i];         end
      endcase
    end
    for (int g = 0; g < 4; g++)
      z_n[g] = (in_op == OP_SC) ? 10'd171 : (in_op == OP_GF) ? C2 : rom_v[(NTW == 4) ? g : (NTW == 2) ? g / 2 : 0];
  end

  genvar gg;
  generate
    for (gg = 0; gg < 4; gg++) begin : g_rom
      if (gg < NTW) begin : g_r
        zen_ntt_twrom u_rom (.sel(in_rom), .idx(idx[(NTW == 2) ? 2 * gg : gg]), .val(rom_v[gg]));
      end else begin : g_n
        assign rom_v[gg] = 10'd0;
      end
    end
  endgenerate

  logic [15:0][26:0] p_n;
  logic [15:0][26:0] py_n;
  generate
    for (gg = 0; gg < 16; gg++) begin : g_mul
      logic signed [26:0] prod;
      assign prod    = $signed(x_n[gg]) * $signed({1'b0, z_n[gg/4]});
      assign p_n[gg] = prod;
      if (HAS_SC) begin : g_sc
        logic signed [26:0] ys, p171, pc1;
        logic signed [26:0] proda;
        assign ys    = {{11{y_n[gg][15]}}, y_n[gg]};
        assign p171  = (ys <<< 7) + (ys <<< 5) + (ys <<< 3) + (ys <<< 1) + ys;          // y * 171
        assign pc1   = 27'sd0 - ((ys <<< 6) + (ys <<< 5) + (ys <<< 4) + (ys <<< 1));  // y * (-114) = y * C1
        assign proda = (in_op == OP_GF) ? pc1 : p171;
        assign py_n[gg] = (in_op == OP_SC || in_op == OP_GF) ? proda : {{11{y_n[gg][15]}}, y_n[gg]};
      end else begin : g_nsc
        assign py_n[gg] = {{11{y_n[gg][15]}}, y_n[gg]};
      end
    end
  endgenerate

  logic              v1;
  logic [15:0][26:0] p2, py2;
  logic [15:0][15:0] xb2;
  logic [2:0]        op2;
  logic [1:0]        lt2;
  logic [TW-1:0]     tag2;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)  v1 <= 1'b0;
    else if (en) v1 <= in_v;
  end
  always_ff @(posedge clk) begin
    if (en) begin
      p2 <= p_n; py2 <= py_n;
      op2 <= in_op; lt2 <= in_lt; tag2 <= in_tag;
    end
  end
  // BYP operand register: only the SC stage (stage 4) bypasses; elsewhere tied off (no flops)
  generate
    if (HAS_SC) begin : g_xb2
      always_ff @(posedge clk) if (en) xb2 <= x_n;
    end else begin : g_noxb2
      assign xb2 = '0;
    end
  endgenerate

  // ---------------------------------------------------------------- level 2
  logic [15:0][15:0] t_n, ya_n, oa, ob;
  logic [255:0]      wao, wbo;
  logic [7:0]        zf_n;
  always_comb begin
    for (int i = 0; i < 16; i++) begin
      t_n[i]  = mont({{5{p2[i][26]}}, p2[i]});
      ya_n[i] = (HAS_SC && (op2 == OP_SC || op2 == OP_GF)) ? mont({{5{py2[i][26]}}, py2[i]}) : py2[i][15:0];
      case (op2)
        OP_GS:   begin oa[i] = ya_n[i];         ob[i] = t_n[i];          end
        OP_GF:   begin oa[i] = ya_n[i];         ob[i] = t_n[i];          end
        OP_SC:   begin oa[i] = canon(ya_n[i]);  ob[i] = canon(t_n[i]);   end
        OP_BYP:  begin oa[i] = ya_n[i];         ob[i] = HAS_SC ? xb2[i] : t_n[i]; end
        default: begin oa[i] = ya_n[i] + t_n[i]; ob[i] = ya_n[i] - t_n[i]; end
      endcase
    end
    wao = '0; wbo = '0;
    for (int i = 0; i < 16; i++) begin
      case (lt2)
        LT_W8: begin
          if (i < 8) begin wao[16*i +: 16] = oa[i]; wao[16*(i+8) +: 16] = ob[i]; end
          else begin       wbo[16*(i-8) +: 16] = oa[i]; wbo[16*i +: 16] = ob[i]; end
        end
        LT_W4: begin
          if (i < 4)       begin wao[16*i +: 16]     = oa[i]; wao[16*(i+4) +: 16] = ob[i]; end
          else if (i < 8)  begin wao[16*(i+4) +: 16] = oa[i]; wao[16*(i+8) +: 16] = ob[i]; end
          else if (i < 12) begin wbo[16*(i-8) +: 16] = oa[i]; wbo[16*(i-4) +: 16] = ob[i]; end
          else             begin wbo[16*(i-4) +: 16] = oa[i]; wbo[16*i +: 16]     = ob[i]; end
        end
        default: begin wao[16*i +: 16] = oa[i]; wbo[16*i +: 16] = ob[i]; end
      endcase
    end
    for (int g = 0; g < 4; g++) begin
      zf_n[g]   = ~|wao[64*g +: 64];
      zf_n[g+4] = ~|wbo[64*g +: 64];
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)  out_v <= 1'b0;
    else if (en) out_v <= v1;
  end
  always_ff @(posedge clk) begin
    if (en) begin
      out_wa  <= wao;
      out_wb  <= wbo;
      out_op  <= op2;
      out_tag <= tag2;
      out_zf  <= zf_n;
    end
  end
endmodule
