// Behavioural models of the SRAM macros (functional and netlist simulation).  Synthesis treats the
// macros as black boxes linked from the CACTI-derived timing/area library; the whole body below
// is excluded under SYNTHESIS.  All macros: one clock, synchronous read (Q registered, valid after
// the edge that samples RE), write committed at the edge.  Read and write of the same address in one
// cycle returns X in this model and never happens by construction in the RTL.
`ifndef SYNTHESIS
// polynomial slot lane bank: 32 x 12, one read port + one write port
module SRAM1R1W_32X12 (
  input  logic        CLK,
  input  logic        RE,
  input  logic [4:0]  RA,
  output logic [11:0] Q,
  input  logic        WE,
  input  logic [4:0]  WA,
  input  logic [11:0] D
);
  logic [11:0] mem [0:31];
  always @(posedge CLK) begin
    if (RE) Q <= (WE && (WA == RA)) ? 12'bx : mem[RA];
    if (WE) mem[WA] <= D;
  end
endmodule

// object buffer: 128 x 64, one read port + one write port (CT words 320..447)
module SRAM1R1W_128X64 (
  input  logic        CLK,
  input  logic        RE,
  input  logic [6:0]  RA,
  output logic [63:0] Q,
  input  logic        WE,
  input  logic [6:0]  WA,
  input  logic [63:0] D
);
  logic [63:0] mem [0:127];
  always @(posedge CLK) begin
    if (RE) Q <= (WE && (WA == RA)) ? 64'bx : mem[RA];
    if (WE) mem[WA] <= D;
  end
endmodule

// object buffer: 512 x 64, one read port + one write port (PK and CT objects)
module SRAM1R1W_512X64 (
  input  logic        CLK,
  input  logic        RE,
  input  logic [8:0]  RA,
  output logic [63:0] Q,
  input  logic        WE,
  input  logic [8:0]  WA,
  input  logic [63:0] D
);
  logic [63:0] mem [0:511];
  always @(posedge CLK) begin
    if (RE) Q <= (WE && (WA == RA)) ? 64'bx : mem[RA];
    if (WE) mem[WA] <= D;
  end
endmodule

// object buffer: 1024 x 64, one read port + one write port (SK object)
module SRAM1R1W_1024X64 (
  input  logic        CLK,
  input  logic        RE,
  input  logic [9:0]  RA,
  output logic [63:0] Q,
  input  logic        WE,
  input  logic [9:0]  WA,
  input  logic [63:0] D
);
  logic [63:0] mem [0:1023];
  always @(posedge CLK) begin
    if (RE) Q <= (WE && (WA == RA)) ? 64'bx : mem[RA];
    if (WE) mem[WA] <= D;
  end
endmodule
`endif
