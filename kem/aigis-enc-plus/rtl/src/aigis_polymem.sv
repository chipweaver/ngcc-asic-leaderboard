// Polynomial slot memory: NSLOT slots x 16 lanes x 4 banks of SRAM1R1W_32X12.
// Slot word w of lane g holds coefficient g*W + w; physical bank pb = {po, pe} with
// pe = w0^w2^w4^w6, po = w1^w3^w5, bank address = w >> 2; physical port index 4g + pb.
//
// Logical roles (two-bank view, as seen by every unit except the NTT): port 2g + lb with
// lb = parity(w) = po ^ pe and logical address la = w >> 1.  The wrapper reconstructs
// w = {la, lb ^ parity(la)} and drives the physical macro; a logical role therefore never
// addresses two macros through one logical port, and two logical ports of one lane always land
// in different macros.  Read data of a logical port is the selected slot's macro Q one cycle
// later (slot select and bank choice delayed with it).
// The NTT role addresses physical ports directly (64 ports, 5-bit addresses).
// The controller guarantees that at most one active role addresses a given physical macro per
// direction in a cycle; requests are OR-combined.  A simulation monitor flags
// conflicts per macro.
module aigis_polymem #(
  parameter int NR = 17,
  parameter int NW = 9
) (
  input  logic                    clk,
  input  logic [NR-1:0]           ract,
  input  logic [NR-1:0][2:0]      rsel,
  input  logic [NR-1:0][31:0]     re,
  input  logic [NR-1:0][31:0][5:0]  ra,
  output logic [NR-1:0][31:0][11:0] rd,
  input  logic [NW-1:0]           wact,
  input  logic [NW-1:0][2:0]      wsel,
  input  logic [NW-1:0][31:0]     we,
  input  logic [NW-1:0][31:0][5:0]  wa,
  input  logic [NW-1:0][31:0][11:0] wd,
  // NTT role (physical)
  input  logic [2:0]              n_rsel,
  input  logic [63:0]             n_re,
  input  logic [63:0][4:0]        n_ra,
  output logic [63:0][11:0]       n_rd,
  input  logic [2:0]              n_wsel,
  input  logic [63:0]             n_we,
  input  logic [63:0][4:0]        n_wa,
  input  logic [63:0][11:0]       n_wd
);
  localparam int NS = aigis_pkg::NSLOT;
  logic [NS-1:0][63:0]       s_re, s_we;
  logic [NS-1:0][63:0][4:0]  s_ra, s_wa;
  logic [NS-1:0][63:0][11:0] s_wd, s_q;

  // logical (lb, la) -> po (pe = po ^ lb), physical address
  function automatic logic po_of(input logic [5:0] la);
    po_of = la[0] ^ la[2] ^ la[4];
  endfunction

  // Per slot: merge the logical roles on the 32 logical ports (the controller gives a slot at most
  // one logical reader and one logical writer at a time), then translate every logical port to
  // the physical macro of its parity class: logical (g, lb=0) -> physical 4g (po=0) or 4g+3 (po=1),
  // logical (g, lb=1) -> physical 4g+1 (po=0) or 4g+2 (po=1).  The NTT role is physical.
  logic [NS-1:0][31:0]       l_re, l_we;
  logic [NS-1:0][31:0][5:0]  l_ra, l_wa;
  logic [NS-1:0][31:0][11:0] l_wd;
  always_comb begin
    l_re = '0; l_ra = '0; l_we = '0; l_wa = '0; l_wd = '0;
    for (int s = 0; s < NS; s++) begin
      for (int r = 0; r < NR; r++)
        if (ract[r] && rsel[r] == 3'(s))
          for (int i = 0; i < 32; i++) begin
            l_re[s][i] = l_re[s][i] | re[r][i];
            l_ra[s][i] = l_ra[s][i] | (re[r][i] ? ra[r][i] : 6'd0);
          end
      for (int w = 0; w < NW; w++)
        if (wact[w] && wsel[w] == 3'(s))
          for (int i = 0; i < 32; i++) begin
            l_we[s][i] = l_we[s][i] | we[w][i];
            l_wa[s][i] = l_wa[s][i] | (we[w][i] ? wa[w][i] : 6'd0);
            l_wd[s][i] = l_wd[s][i] | (we[w][i] ? wd[w][i] : 12'd0);
          end
    end
  end

  always_comb begin
    s_re = '0; s_ra = '0; s_we = '0; s_wa = '0; s_wd = '0;
    for (int s = 0; s < NS; s++) begin
      logic nrs, nws;
      nrs = (n_rsel == 3'(s)); nws = (n_wsel == 3'(s));
      for (int g = 0; g < 16; g++)
        for (int lb = 0; lb < 2; lb++) begin
          logic rpo, wpo;
          int p0, p1;                       // physical index for po = 0 / po = 1
          p0 = 4*g + lb; p1 = 4*g + 3 - lb;
          rpo = po_of(l_ra[s][2*g + lb]);
          wpo = po_of(l_wa[s][2*g + lb]);
          s_re[s][p0] = l_re[s][2*g + lb] && !rpo;
          s_re[s][p1] = l_re[s][2*g + lb] && rpo;
          s_ra[s][p0] = l_ra[s][2*g + lb][5:1];
          s_ra[s][p1] = l_ra[s][2*g + lb][5:1];
          s_we[s][p0] = l_we[s][2*g + lb] && !wpo;
          s_we[s][p1] = l_we[s][2*g + lb] && wpo;
          s_wa[s][p0] = l_wa[s][2*g + lb][5:1];
          s_wa[s][p1] = l_wa[s][2*g + lb][5:1];
          s_wd[s][p0] = l_wd[s][2*g + lb];
          s_wd[s][p1] = l_wd[s][2*g + lb];
        end
      for (int i = 0; i < 64; i++) begin
        if (nrs && n_re[i]) begin s_re[s][i] = 1'b1; s_ra[s][i] = n_ra[i]; end
        if (nws && n_we[i]) begin s_we[s][i] = 1'b1; s_wa[s][i] = n_wa[i]; s_wd[s][i] = n_wd[i]; end
      end
    end
  end

  for (genvar s = 0; s < NS; s++) begin : g_slot
    for (genvar i = 0; i < 64; i++) begin : g_lb
      SRAM1R1W_32X12 u_ram (
        .CLK(clk), .RE(s_re[s][i]), .RA(s_ra[s][i]), .Q(s_q[s][i]),
        .WE(s_we[s][i]), .WA(s_wa[s][i]), .D(s_wd[s][i]));
    end
  end

  // read return: per slot the logical view of its Q (bank choice registered with the read),
  // then the per-role slot select as in the two-bank design
  logic [NR-1:0][2:0]  rsel_q;
  logic [2:0]          n_rsel_q;
  logic [NS-1:0][31:0] lpo_q;
  logic [NS-1:0][31:0][11:0] l_q;
  always_ff @(posedge clk) begin
    rsel_q <= rsel; n_rsel_q <= n_rsel;
    for (int s = 0; s < NS; s++)
      for (int i = 0; i < 32; i++) lpo_q[s][i] <= po_of(l_ra[s][i]);
  end
  always_comb begin
    for (int s = 0; s < NS; s++)
      for (int g = 0; g < 16; g++)
        for (int lb = 0; lb < 2; lb++)
          l_q[s][2*g + lb] = lpo_q[s][2*g + lb] ? s_q[s][4*g + 3 - lb] : s_q[s][4*g + lb];
    n_rd = s_q[n_rsel_q];
    for (int r = 0; r < NR; r++) rd[r] = l_q[rsel_q[r]];
  end

`ifndef SYNTHESIS
  // conflict monitor: two drivers on one physical macro port in one cycle
  always @(posedge clk) begin
    for (int s = 0; s < NS; s++) begin
      int nr [0:63]; int nw [0:63];
      for (int i = 0; i < 64; i++) begin
        nr[i] = (n_rsel == 3'(s) && n_re[i]) ? 1 : 0;
        nw[i] = (n_wsel == 3'(s) && n_we[i]) ? 1 : 0;
      end
      for (int r = 0; r < NR; r++) if (ract[r] && rsel[r] == 3'(s))
        for (int g = 0; g < 16; g++) for (int lb = 0; lb < 2; lb++) if (re[r][2*g + lb]) begin
          logic po; po = po_of(ra[r][2*g + lb]);
          nr[4*g + 2*int'(po) + (int'(po) ^ lb)]++;
        end
      for (int w = 0; w < NW; w++) if (wact[w] && wsel[w] == 3'(s))
        for (int g = 0; g < 16; g++) for (int lb = 0; lb < 2; lb++) if (we[w][2*g + lb]) begin
          logic po; po = po_of(wa[w][2*g + lb]);
          nw[4*g + 2*int'(po) + (int'(po) ^ lb)]++;
        end
      begin
        int lr, lw;
        lr = 0; lw = 0;
        for (int r = 0; r < NR; r++) if (ract[r] && rsel[r] == 3'(s) && |re[r]) lr++;
        for (int w = 0; w < NW; w++) if (wact[w] && wsel[w] == 3'(s) && |we[w]) lw++;
        if (lr > 1) $error("polymem: %0d logical read roles on slot %0d", lr, s);
        if (lw > 1) $error("polymem: %0d logical write roles on slot %0d", lw, s);
      end
      for (int i = 0; i < 64; i++) begin
        if (nr[i] > 1) $error("polymem: %0d readers on slot %0d macro %0d", nr[i], s, i);
        if (nw[i] > 1) $error("polymem: %0d writers on slot %0d macro %0d", nw[i], s, i);
      end
    end
  end
`endif
endmodule
