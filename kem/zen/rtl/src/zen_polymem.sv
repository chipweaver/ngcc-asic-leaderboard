// Polynomial memory: 3 slots x 2 banks x (64 words x 256 bits), each bank built from two
// 64x128 1r1w macros. Read/write masters are mapped to slots by the controller (rsel/wsel);
// the controller guarantees that at most one active master drives each slot-bank port, except for the
// read master NR-1 (PKU), which may share a slot with another reader (SKF_PACK during baseinv,
// PK_PACK during basemul_mq): it is granted only when no other read master reads that slot in the
// cycle (rd_gnt_last), otherwise its request is suppressed and repeated by the PKU.
module zen_polymem #(
  parameter int NR = 6,   // read masters
  parameter int NW = 5    // write masters
) (
  input  logic                        clk,
  input  logic [NR-1:0][1:0]          rsel,
  input  logic [NW-1:0][1:0]          wsel,
  input  logic [NR-1:0][1:0]          rd_en,
  input  logic [NR-1:0][1:0][5:0]     rd_addr,
  output logic [NR-1:0][1:0][255:0]   rd_data,
  input  logic [NW-1:0][1:0]          wr_en,
  input  logic [NW-1:0][1:0][5:0]     wr_addr,
  input  logic [NW-1:0][1:0][255:0]   wr_data,
  output logic                        rd_gnt_last
);
  logic [NR-1:0][1:0] rd_en_g;
  always_comb begin
    rd_gnt_last = 1'b1;
    for (int m = 0; m < NR-1; m++)
      if ((rd_en[m] != 2'b00) && (rsel[m] == rsel[NR-1])) rd_gnt_last = 1'b0;
    rd_en_g = rd_en;
    if (!rd_gnt_last) rd_en_g[NR-1] = 2'b00;
  end
  logic [2:0][1:0]        s_re, s_we;
  logic [2:0][1:0][5:0]   s_ra, s_wa;
  logic [2:0][1:0][255:0] s_d, s_q;

  always_comb begin
    s_re = '0; s_ra = '0; s_we = '0; s_wa = '0; s_d = '0;
    for (int s = 0; s < 3; s++) begin
      for (int b = 0; b < 2; b++) begin
        for (int m = 0; m < NR; m++) begin
          if (rd_en_g[m][b] && (rsel[m] == s[1:0])) begin
            s_re[s][b] = 1'b1;
            s_ra[s][b] = s_ra[s][b] | rd_addr[m][b];
          end
        end
        for (int m = 0; m < NW; m++) begin
          if (wr_en[m][b] && (wsel[m] == s[1:0])) begin
            s_we[s][b] = 1'b1;
            s_wa[s][b] = s_wa[s][b] | wr_addr[m][b];
            s_d[s][b]  = s_d[s][b]  | wr_data[m][b];
          end
        end
      end
    end
  end

  genvar gs, gb;
  generate
    for (gs = 0; gs < 3; gs++) begin : g_slot
      for (gb = 0; gb < 2; gb++) begin : g_bank
        RF1R1W_64X128 u_lo (.CLK(clk), .RE(s_re[gs][gb]), .RA(s_ra[gs][gb]), .Q(s_q[gs][gb][127:0]),
                            .WE(s_we[gs][gb]), .WA(s_wa[gs][gb]), .D(s_d[gs][gb][127:0]));
        RF1R1W_64X128 u_hi (.CLK(clk), .RE(s_re[gs][gb]), .RA(s_ra[gs][gb]), .Q(s_q[gs][gb][255:128]),
                            .WE(s_we[gs][gb]), .WA(s_wa[gs][gb]), .D(s_d[gs][gb][255:128]));
      end
    end
  endgenerate

  always_comb begin
    for (int m = 0; m < NR; m++) begin
      case (rsel[m])
        2'd0:    rd_data[m] = s_q[0];
        2'd1:    rd_data[m] = s_q[1];
        default: rd_data[m] = s_q[2];
      endcase
    end
  end
endmodule
