// Object memories PK / SK / CT (official byte layouts) with host access while idle.
//  MEM_PK : 512 x 64 2rw   (port A: host | PKU,  port B: hash bulk reader)
//  MEM_SK : 1024 x 64 2rw  (port A: host | PKU,  port B: hash sk copier)
//  MEM_CT : 2 x 128 x 64 2rw, even/odd 8-byte words (port A: host | PASS 16 B | NTT fused combine 16 B,
//           port B: hash bulk reader 8 B | NTT fused combine 16 B)
// Host read data is returned (combinational from the macro outputs) the cycle after the request;
// zen_kem_top registers it once more (2-cycle host read latency).
module zen_objmem (
  input  logic         clk,
  input  logic         dp_sel,          // 1: datapath owns port A (busy), 0: host
  // host
  input  logic         h_en, h_we,
  input  logic [1:0]   h_obj,
  input  logic [9:0]   h_addr,
  input  logic [63:0]  h_wdata,
  output logic [63:0]  h_rdata,         // valid the cycle after a host read (PK/SK/CT)
  // PKU -> MEM_PK
  input  logic         pk_en, pk_we,
  input  logic [9:0]   pk_addr,
  input  logic [63:0]  pk_wdata,
  input  logic [7:0]   pk_wmask,
  output logic [63:0]  pk_rdata,
  // PKU -> MEM_SK
  input  logic         sk_en, sk_we,
  input  logic [9:0]   sk_addr,
  input  logic [63:0]  sk_wdata,
  input  logic [7:0]   sk_wmask,
  output logic [63:0]  sk_rdata,
  // PASS -> MEM_CT (128-bit)
  input  logic         ct_en, ct_we,
  input  logic [6:0]   ct_addr,
  input  logic [127:0] ct_wdata,
  input  logic [15:0]  ct_wmask,
  output logic [127:0] ct_rdata,
  // hash port B
  input  logic         pkb_en,
  input  logic [9:0]   pkb_addr,
  output logic [63:0]  pkb_rdata,
  input  logic         skb_en, skb_we,
  input  logic [9:0]   skb_addr,
  input  logic [63:0]  skb_wdata,
  input  logic [7:0]   skb_wmask,
  output logic [63:0]  skb_rdata,
  input  logic         ctb_en,
  input  logic [9:0]   ctb_addr,         // 64-bit word index
  output logic [63:0]  ctb_rdata,
  // NTT fused combine/compare: 16-byte CT words through port A and port B
  input  logic         na_en, na_we,
  input  logic [6:0]   na_addr,
  input  logic [127:0] na_wdata,
  output logic [127:0] na_rdata,
  input  logic         nb_en, nb_we,
  input  logic [6:0]   nb_addr,
  input  logic [127:0] nb_wdata,
  output logic [127:0] nb_rdata
);
  // ---------------- port A muxes
  logic        pa_en, pa_we, sa_en, sa_we;
  logic [8:0]  pa_addr;
  logic [9:0]  sa_addr;
  logic [63:0] pa_d, sa_d;
  logic [7:0]  pa_m, sa_m;
  logic [1:0]  ca_en, ca_we;
  logic [1:0][6:0]  ca_addr;
  logic [1:0][63:0] ca_d;
  logic [1:0][7:0]  ca_m;

  always_comb begin
    if (dp_sel) begin
      pa_en = pk_en; pa_we = pk_we; pa_addr = pk_addr[8:0]; pa_d = pk_wdata; pa_m = pk_wmask;
      sa_en = sk_en; sa_we = sk_we; sa_addr = sk_addr;      sa_d = sk_wdata; sa_m = sk_wmask;
      if (na_en) begin
        ca_en = 2'b11; ca_we = {na_we, na_we};
        ca_addr = {na_addr, na_addr};
        ca_d = {na_wdata[127:64], na_wdata[63:0]};
        ca_m = 16'hFFFF;
      end else begin
        ca_en = {ct_en, ct_en}; ca_we = {ct_we, ct_we};
        ca_addr = {ct_addr, ct_addr};
        ca_d = {ct_wdata[127:64], ct_wdata[63:0]};
        ca_m = {ct_wmask[15:8], ct_wmask[7:0]};
      end
    end else begin
      // PK: host writes to words >= 512 lie outside the object and are dropped
      pa_en = h_en && (h_obj == 2'd0) && !(h_we && h_addr[9]); pa_we = h_we; pa_addr = h_addr[8:0]; pa_d = h_wdata; pa_m = 8'hFF;
      sa_en = h_en && (h_obj == 2'd1); sa_we = h_we; sa_addr = h_addr;      sa_d = h_wdata; sa_m = 8'hFF;
      ca_en[0] = h_en && (h_obj == 2'd2) && !h_addr[0] && !(h_we && (h_addr[9:8] != 2'b00));   // CT: words >= 256 outside
      ca_en[1] = h_en && (h_obj == 2'd2) &&  h_addr[0] && !(h_we && (h_addr[9:8] != 2'b00));
      ca_we = {h_we, h_we};
      ca_addr = {h_addr[7:1], h_addr[7:1]};
      ca_d = {h_wdata, h_wdata};
      ca_m = {8'hFF, 8'hFF};
    end
  end

  logic [63:0] pa_q, sa_q;
  logic [1:0][63:0] ca_q, cb_q;
  logic [1:0] cb_en;

  SR2RW_512X64 u_pk (.CLK(clk),
    .ENA(pa_en), .WEA(pa_we), .AA(pa_addr), .DA(pa_d), .MA(pa_m), .QA(pa_q),
    .ENB(pkb_en), .WEB(1'b0), .AB(pkb_addr[8:0]), .DB(64'd0), .MB(8'd0), .QB(pkb_rdata));

  SR2RW_1024X64 u_sk (.CLK(clk),
    .ENA(sa_en), .WEA(sa_we), .AA(sa_addr), .DA(sa_d), .MA(sa_m), .QA(sa_q),
    .ENB(skb_en), .WEB(skb_we), .AB(skb_addr), .DB(skb_wdata), .MB(skb_wmask), .QB(skb_rdata));

  // port B: NTT fused combine/compare (both halves, 16-byte word) has priority over the hash reader;
  // the controller guarantees the Decaps rejection hash has finished before the FO compare starts.
  logic [1:0] cb_we;
  logic [6:0] cb_addr;
  logic [1:0][63:0] cb_d;
  assign cb_en[0] = nb_en || (ctb_en && !ctb_addr[0]);
  assign cb_en[1] = nb_en || (ctb_en &&  ctb_addr[0]);
  assign cb_we    = {nb_en && nb_we, nb_en && nb_we};
  assign cb_addr  = nb_en ? nb_addr : ctb_addr[7:1];
  assign cb_d     = {nb_wdata[127:64], nb_wdata[63:0]};
  genvar g;
  generate
    for (g = 0; g < 2; g++) begin : g_ct
      SR2RW_128X64 u_ct (.CLK(clk),
        .ENA(ca_en[g]), .WEA(ca_we[g]), .AA(ca_addr[g]), .DA(ca_d[g]), .MA(ca_m[g]), .QA(ca_q[g]),
        .ENB(cb_en[g]), .WEB(cb_we[g]), .AB(cb_addr), .DB(cb_d[g]), .MB(8'hFF), .QB(cb_q[g]));
    end
  endgenerate

  logic ctb_lsb_q;
  logic [1:0] h_obj_q;
  logic       h_lsb_q;
  always_ff @(posedge clk) begin
    ctb_lsb_q <= ctb_addr[0];
    h_obj_q   <= h_obj;
    h_lsb_q   <= h_addr[0];
  end
  assign ctb_rdata = ctb_lsb_q ? cb_q[1] : cb_q[0];
  assign pk_rdata  = pa_q;
  assign sk_rdata  = sa_q;
  assign ct_rdata  = {ca_q[1], ca_q[0]};
  assign na_rdata  = {ca_q[1], ca_q[0]};
  assign nb_rdata  = {cb_q[1], cb_q[0]};
  always_comb begin
    case (h_obj_q)
      2'd0:    h_rdata = pa_q;
      2'd1:    h_rdata = sa_q;
      2'd2:    h_rdata = h_lsb_q ? ca_q[1] : ca_q[0];
      default: h_rdata = 64'd0;
    endcase
  end
endmodule
