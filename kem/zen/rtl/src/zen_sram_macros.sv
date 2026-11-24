// Behavioral models of the SRAM macros (functional simulation and post-synthesis netlist
// simulation). Synthesis treats these cells as black boxes: the whole file body is excluded
// when SYNTHESIS is defined, and the cells are linked from the CACTI-derived timing library.
// All macros: synchronous, single clock, read data registered (valid after the clock edge
// that samples the read), write commits at the clock edge. Same-address read/write in one cycle
// returns undefined data (X) in this model and is avoided by construction in the RTL.
`ifndef SYNTHESIS

// 64 x 128, one read port + one write port (1r1w)
module RF1R1W_64X128 (
  input  logic         CLK,
  input  logic         RE,
  input  logic [5:0]   RA,
  output logic [127:0] Q,
  input  logic         WE,
  input  logic [5:0]   WA,
  input  logic [127:0] D
);
  logic [127:0] mem [0:63];
  always @(posedge CLK) begin
    if (RE) Q <= (WE && (WA == RA)) ? {128{1'bx}} : mem[RA];
    if (WE) mem[WA] <= D;
  end
endmodule

// generic true dual-port (2rw) with byte write mask; instantiated per geometry below
module zen_sram2rw_beh #(parameter int DEPTH = 512, parameter int AW = 9) (
  input  logic          CLK,
  input  logic          ENA, WEA,
  input  logic [AW-1:0] AA,
  input  logic [63:0]   DA,
  input  logic [7:0]    MA,
  output logic [63:0]   QA,
  input  logic          ENB, WEB,
  input  logic [AW-1:0] AB,
  input  logic [63:0]   DB,
  input  logic [7:0]    MB,
  output logic [63:0]   QB
);
  logic [63:0] mem [0:DEPTH-1];
  integer i;
  always @(posedge CLK) begin
    if (ENA && !WEA) QA <= (ENB && WEB && (AB == AA)) ? {64{1'bx}} : mem[AA];
    if (ENB && !WEB) QB <= (ENA && WEA && (AA == AB)) ? {64{1'bx}} : mem[AB];
    if (ENA && WEA) for (i = 0; i < 8; i = i + 1) if (MA[i]) mem[AA][8*i +: 8] <= DA[8*i +: 8];
    if (ENB && WEB) for (i = 0; i < 8; i = i + 1) if (MB[i]) mem[AB][8*i +: 8] <= DB[8*i +: 8];
  end
endmodule

module SR2RW_512X64 (
  input  logic CLK, ENA, WEA, input logic [8:0] AA, input logic [63:0] DA, input logic [7:0] MA, output logic [63:0] QA,
  input  logic ENB, WEB, input logic [8:0] AB, input logic [63:0] DB, input logic [7:0] MB, output logic [63:0] QB);
  zen_sram2rw_beh #(.DEPTH(512), .AW(9)) u (.*);
endmodule

module SR2RW_1024X64 (
  input  logic CLK, ENA, WEA, input logic [9:0] AA, input logic [63:0] DA, input logic [7:0] MA, output logic [63:0] QA,
  input  logic ENB, WEB, input logic [9:0] AB, input logic [63:0] DB, input logic [7:0] MB, output logic [63:0] QB);
  zen_sram2rw_beh #(.DEPTH(1024), .AW(10)) u (.*);
endmodule

module SR2RW_128X64 (
  input  logic CLK, ENA, WEA, input logic [6:0] AA, input logic [63:0] DA, input logic [7:0] MA, output logic [63:0] QA,
  input  logic ENB, WEB, input logic [6:0] AB, input logic [63:0] DB, input logic [7:0] MB, output logic [63:0] QB);
  zen_sram2rw_beh #(.DEPTH(128), .AW(7)) u (.*);
endmodule

`endif
