// Single-port (1rw) synchronous SRAM wrapper.
// Simulation: behavioural array, registered read (q valid the cycle after ce & !we, held otherwise).
// Synthesis (`SYNTHESIS defined): black-box macro instance whose Liberty model is generated from
// the CACTI 90 nm estimate of the same shape (access time -> CLK->Q, cycle time -> min period).
// BW > 0 adds a per-lane write mask (lane = DW/BW bits).
module sram_sp #(
  parameter int AW = 9,
  parameter int DW = 256,
  parameter int BW = 0
) (
  input  logic                  clk,
  input  logic                  ce,
  input  logic                  we,
  input  logic [AW-1:0]         a,
  input  logic [DW-1:0]         d,
  input  logic [(BW>0?BW:1)-1:0] wm,
  output logic [DW-1:0]         q
);
`ifndef SYNTHESIS
  logic [DW-1:0] mem [0:(1<<AW)-1];
  logic [DW-1:0] bitmask;
  integer k;
  always_comb begin
    bitmask = '1;
    if (BW > 0)
      for (k = 0; k < DW; k = k + 1) bitmask[k] = wm[k / (DW / (BW > 0 ? BW : 1))];
  end
  // plain always (not always_ff) so testbenches may preload/inspect mem hierarchically
  always @(posedge clk) begin
    if (ce) begin
      if (we) mem[a] <= (mem[a] & ~bitmask) | (d & bitmask);
      else    q <= mem[a];
    end
  end
`else
  generate
    if (AW == 7 && DW == 256 && BW == 0) begin : g_d
      SRAM1RW_128X256 u_m (.CLK(clk), .CE(ce), .WE(we), .A(a), .D(d), .Q(q));
    end else if (AW == 10 && DW == 18 && BW == 0) begin : g_l
      SRAM1RW_1024X18 u_m (.CLK(clk), .CE(ce), .WE(we), .A(a), .D(d), .Q(q));
    end else if (AW == 12 && DW == 64 && BW == 8) begin : g_b4k
      SRAM1RWM_4096X64 u_m (.CLK(clk), .CE(ce), .WE(we), .A(a), .D(d), .WM(wm), .Q(q));
    end else if (AW == 11 && DW == 64 && BW == 8) begin : g_b2k
      SRAM1RWM_2048X64 u_m (.CLK(clk), .CE(ce), .WE(we), .A(a), .D(d), .WM(wm), .Q(q));
    end else if (AW == 10 && DW == 64 && BW == 8) begin : g_b1k
      SRAM1RWM_1024X64 u_m (.CLK(clk), .CE(ce), .WE(we), .A(a), .D(d), .WM(wm), .Q(q));
    end else begin : g_unsupported
      // no macro of this shape: fail elaboration instead of leaving q undriven
      SRAM_SHAPE_NOT_SUPPORTED u_m ();
    end
  endgenerate
`endif
endmodule
