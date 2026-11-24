// Behavioural models of the SRAM macros (functional simulation). Synthesis treats every macro as a
// black box linked from the CACTI-derived Liberty model; the bodies are excluded under SYNTHESIS.
// Synchronous macros, one clock: inputs are sampled on the rising edge, a read returns Q after that
// edge and holds it until the next read. 1RW: CE=1 & WE=0 reads A, CE=1 & WE=1 writes D (WM selects
// 64-bit lanes on the masked variants). 1R1W: RE reads RA, WE writes WA; a read and a write of the
// same address in one cycle returns X (never issued by the RTL).
`ifndef SYNTHESIS
// dense slot half: 256 x 512, single port (a 1024-bit lane word is two macros at one address)
module SRAM1RW_256X512 (
  input  logic         CLK,
  input  logic         CE,
  input  logic         WE,
  input  logic [7:0]   A,
  input  logic [511:0] D,
  output logic [511:0] Q
);
  logic [511:0] mem [0:255];
  always @(posedge CLK) if (CE) begin
    if (WE) mem[A] <= D; else Q <= mem[A];
  end
endmodule

// object buffer bank: 512 x 512, single port, 64-bit lane write mask
module SRAM1RWM_512X512 (
  input  logic         CLK,
  input  logic         CE,
  input  logic         WE,
  input  logic [7:0]   WM,
  input  logic [8:0]   A,
  input  logic [511:0] D,
  output logic [511:0] Q
);
  logic [511:0] mem [0:511];
  always @(posedge CLK) if (CE) begin
    if (WE) begin
      for (int l = 0; l < 8; l++) if (WM[l]) mem[A][64 * l +: 64] <= D[64 * l +: 64];
    end else Q <= mem[A];
  end
endmodule

// object buffer tail bank: 32 x 512, single port, 64-bit lane write mask
module SRAM1RWM_32X512 (
  input  logic         CLK,
  input  logic         CE,
  input  logic         WE,
  input  logic [7:0]   WM,
  input  logic [4:0]   A,
  input  logic [511:0] D,
  output logic [511:0] Q
);
  logic [511:0] mem [0:31];
  always @(posedge CLK) if (CE) begin
    if (WE) begin
      for (int l = 0; l < 8; l++) if (WM[l]) mem[A][64 * l +: 64] <= D[64 * l +: 64];
    end else Q <= mem[A];
  end
endmodule

// accumulator bank half: 128 x 512, single port (ACC word j is in bank j[0] at address j[7:1],
// a 1024-bit word is the two halves of its bank at one address)
module SRAM1RW_128X512 (
  input  logic         CLK,
  input  logic         CE,
  input  logic         WE,
  input  logic [6:0]   A,
  input  logic [511:0] D,
  output logic [511:0] Q
);
  logic [511:0] mem [0:127];
  always @(posedge CLK) if (CE) begin
    if (WE) mem[A] <= D; else Q <= mem[A];
  end
endmodule

// support list: 1024 x 18, one read port + one write port
module SRAM1R1W_1024X18 (
  input  logic         CLK,
  input  logic         RE,
  input  logic [9:0]   RA,
  output logic [17:0]  Q,
  input  logic         WE,
  input  logic [9:0]   WA,
  input  logic [17:0]  D
);
  logic [17:0] mem [0:1023];
  always @(posedge CLK) begin
    if (RE) Q <= (WE && (WA == RA)) ? 'x : mem[RA];
    if (WE) mem[WA] <= D;
  end
endmodule
`endif
