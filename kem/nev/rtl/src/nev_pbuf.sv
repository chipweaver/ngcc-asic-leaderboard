// nev_pbuf: 8 polynomial buffers, each 128 words x 192 bits as four 32-word 1rw banks.
// Word w of a buffer lives in bank {^w[6:1], ^w[6:0]} at bank address w[6:2]. Consequences used by
// the poly ALU (dual issue): two words differing in exactly one address bit are in different banks,
// and the four words {w, w+s, w', w'+s} touched by two butterflies of one NTT level (or the words
// {row,g}, {row,g+1} of two adjacent slot groups) always fall into four different banks.
// Masters (registered requests):
//   read ports  : NR (index r)  -> data returned exactly 2 cycles after the request cycle
//   write ports : NW (index w)  -> written at the next clock edge
// Read ports [0, NRH) (poly ALU) and all write ports are never arbitrated: the sequencer guarantees
// that they do not share a bank (a conflict is a sequencing bug and is reported in simulation).
// Read ports >= NRH (codecs) are granted (rgnt) only when no other port uses their bank in that
// cycle; an ungranted request is held by its master and retried, which lets a codec stream out of a
// buffer while the poly ALU is still writing it.
module nev_pbuf import nev_pkg::*; #(
  parameter int NR = 6,
  parameter int NW = 7,
  parameter int NRH = 4             // read ports [0, NRH) have priority; ports >= NRH are granted only
                                    // when no higher-priority port and no write uses their bank
) (
  input  logic                 clk,
  input  logic                 rst_n,
  input  logic [NR-1:0]        rv,
  input  logic [NR-1:0][2:0]   rbuf,
  input  logic [NR-1:0][6:0]   raddr,
  output logic [NR-1:0][WW-1:0] rdata,
  output logic [NR-1:0]        rgnt,      // read accepted this cycle (always 1 for ports < NRH)
  input  logic [NW-1:0]        wv,
  input  logic [NW-1:0][2:0]   wbuf,
  input  logic [NW-1:0][6:0]   waddr,
  input  logic [NW-1:0][WW-1:0] wdata
);
  localparam int NB = 4 * NBUF;

  function automatic logic [1:0] bsel(input logic [6:0] a);
    bsel = {^a[6:1], ^a[6:0]};
  endfunction

  logic [NR-1:0][4:0]     rbk;
  logic [NW-1:0][4:0]     wbk;
  always_comb begin
    for (int r = 0; r < NR; r++) rbk[r] = {rbuf[r], bsel(raddr[r])};
    for (int w = 0; w < NW; w++) wbk[w] = {wbuf[w], bsel(waddr[w])};
  end

  logic [NB-1:0]          b_cs, b_we;
  logic [NB-1:0][4:0]     b_addr;
  logic [NB-1:0][WW-1:0]  b_wdata, b_q;
  logic [NB-1:0][WW-1:0]  b_qr;          // registered SRAM output

  // grants of the low-priority read ports
  logic [NR-1:0] rok;
  always_comb begin
    for (int r = 0; r < NR; r++) begin
      rgnt[r] = 1'b1;
      if (r >= NRH) begin
        for (int w = 0; w < NW; w++) if (wv[w] && (wbk[w] == rbk[r])) rgnt[r] = 1'b0;
        for (int r2 = 0; r2 < r; r2++) if (rv[r2] && (rbk[r2] == rbk[r])) rgnt[r] = 1'b0;
      end
      rok[r] = rv[r] && rgnt[r];
    end
  end

  // per-bank request selection (priority: high reads > writes > granted low reads)
  always_comb begin
    for (int b = 0; b < NB; b++) begin
      b_cs[b]    = 1'b0;
      b_we[b]    = 1'b0;
      b_addr[b]  = '0;
      b_wdata[b] = '0;
      for (int r = NR - 1; r >= NRH; r--) begin
        if (rok[r] && (rbk[r] == b[4:0])) begin
          b_cs[b] = 1'b1; b_we[b] = 1'b0; b_addr[b] = raddr[r][6:2];
        end
      end
      for (int w = NW - 1; w >= 0; w--) begin
        if (wv[w] && (wbk[w] == b[4:0])) begin
          b_cs[b] = 1'b1; b_we[b] = 1'b1; b_addr[b] = waddr[w][6:2]; b_wdata[b] = wdata[w];
        end
      end
      for (int r = NRH - 1; r >= 0; r--) begin
        if (rv[r] && (rbk[r] == b[4:0])) begin
          b_cs[b] = 1'b1; b_we[b] = 1'b0; b_addr[b] = raddr[r][6:2];
        end
      end
    end
  end

  for (genvar b = 0; b < NB; b++) begin : g_bank
    nev_sram_1rw #(.DEPTH(32), .WIDTH(WW), .AW(5)) u_ram (
      .clk(clk), .cs(b_cs[b]), .we(b_we[b]), .addr(b_addr[b]), .wdata(b_wdata[b]), .rdata(b_q[b]));
    always_ff @(posedge clk) b_qr[b] <= b_q[b];
  end

  // read return: remember the bank of each read request for two cycles
  logic [NR-1:0][4:0] sel1, sel2;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sel1 <= '0; sel2 <= '0;
    end else begin
      for (int r = 0; r < NR; r++) begin
        sel1[r] <= rbk[r];
        sel2[r] <= sel1[r];
      end
    end
  end
  always_comb begin
    for (int r = 0; r < NR; r++) rdata[r] = b_qr[sel2[r]];
  end

`ifndef SYNTHESIS
  // conflict detection (simulation only); n_conflict is checked by the benches
  int unsigned n_conflict;
  initial n_conflict = 0;
  always @(posedge clk) begin
    if (rst_n) begin
      for (int b = 0; b < NB; b++) begin
        int n;
        n = 0;
        for (int w = 0; w < NW; w++) if (wv[w] && (wbk[w] == b[4:0])) n++;
        for (int r = 0; r < NR; r++) if (rok[r] && (rbk[r] == b[4:0])) n++;
        if (n > 1) begin
          n_conflict++;
          $error("nev_pbuf: %0d simultaneous accesses to buffer %0d bank %0d at %t", n, b >> 2, b & 3, $time);
        end
      end
    end
  end
`endif
endmodule
