// nev_pkg: shared constants and parameter-set decoding for nev_kem_top.
package nev_pkg;

  localparam int LANES = 16;              // coefficients per poly-buffer word
  localparam int CW    = 12;              // coefficient width (q < 4096)
  localparam int WW    = LANES * CW;      // poly word width (192)
  localparam int PWA   = 7;               // poly word address bits (N=2048 -> 128 words)
  localparam int NBUF  = 8;               // number of poly buffers

  // object memory: 8-byte words, field word bases (host map) and per-SRAM index
  localparam int OMA   = 9;               // word address within one object SRAM (384 words)
  localparam logic [10:0] BASE_SKF = 11'h000;
  localparam logic [10:0] BASE_PK  = 11'h180;
  localparam logic [10:0] BASE_HPK = 11'h300;
  localparam logic [10:0] BASE_SS  = 11'h308;
  localparam logic [10:0] BASE_CT  = 11'h400;

  // q selector encoding
  localparam logic [1:0] Q769  = 2'd0;
  localparam logic [1:0] Q1409 = 2'd1;
  localparam logic [1:0] Q3329 = 2'd2;

  // operations
  localparam logic [1:0] OP_KG  = 2'd0;
  localparam logic [1:0] OP_ENC = 2'd1;
  localparam logic [1:0] OP_DEC = 2'd2;

  // eta / distribution codes (as in the reference params.h)
  //  1: cbd1  2: cbd2  3: cbd3  4: cbd4  7: cbd7  8: bias8 ternary  9: bias3 ternary
  typedef struct packed {
    logic [1:0]  qsel;      // Q769 / Q1409 / Q3329
    logic [1:0]  nlog;      // 0:N=512 1:N=1024 2:N=2048
    logic        d64;       // 1: NTT_DIM=64, 0: NTT_DIM=128
    logic [1:0]  slog;      // 0:S=16 1:S=32 2:S=64
    logic [3:0]  eta_f;
    logic [3:0]  eta_g;
    logic [3:0]  eta_r;     // 0 for compressed sets (no r)
    logic [3:0]  eta_e;
    logic        comp;      // compressed ciphertext (C*-c)
    logic [11:0] poly_bytes;
  } pset_t;

  function automatic pset_t pset(input logic [3:0] idx);
    pset_t p;
    p = '0;
    case (idx)
      4'd0:  begin p.qsel=Q769;  p.nlog=2'd0; p.d64=1'b0; p.slog=2'd0; p.eta_f=4'd1; p.eta_g=4'd2; p.eta_r=4'd9; p.eta_e=4'd2; p.comp=1'b0; p.poly_bytes=12'd615;  end // C1
      4'd1:  begin p.qsel=Q769;  p.nlog=2'd1; p.d64=1'b0; p.slog=2'd1; p.eta_f=4'd1; p.eta_g=4'd8; p.eta_r=4'd9; p.eta_e=4'd2; p.comp=1'b0; p.poly_bytes=12'd1229; end // C2
      4'd2:  begin p.qsel=Q769;  p.nlog=2'd2; p.d64=1'b0; p.slog=2'd2; p.eta_f=4'd8; p.eta_g=4'd8; p.eta_r=4'd9; p.eta_e=4'd1; p.comp=1'b0; p.poly_bytes=12'd2458; end // C3
      4'd3:  begin p.qsel=Q769;  p.nlog=2'd0; p.d64=1'b0; p.slog=2'd0; p.eta_f=4'd1; p.eta_g=4'd2; p.eta_r=4'd0; p.eta_e=4'd2; p.comp=1'b1; p.poly_bytes=12'd615;  end // C1-c
      4'd4:  begin p.qsel=Q769;  p.nlog=2'd1; p.d64=1'b0; p.slog=2'd1; p.eta_f=4'd1; p.eta_g=4'd8; p.eta_r=4'd0; p.eta_e=4'd2; p.comp=1'b1; p.poly_bytes=12'd1229; end // C2-c
      4'd5:  begin p.qsel=Q769;  p.nlog=2'd2; p.d64=1'b0; p.slog=2'd2; p.eta_f=4'd8; p.eta_g=4'd8; p.eta_r=4'd0; p.eta_e=4'd1; p.comp=1'b1; p.poly_bytes=12'd2458; end // C3-c
      4'd6:  begin p.qsel=Q3329; p.nlog=2'd0; p.d64=1'b0; p.slog=2'd0; p.eta_f=4'd7; p.eta_g=4'd7; p.eta_r=4'd7; p.eta_e=4'd7; p.comp=1'b0; p.poly_bytes=12'd768;  end // D1
      4'd7:  begin p.qsel=Q3329; p.nlog=2'd1; p.d64=1'b0; p.slog=2'd1; p.eta_f=4'd4; p.eta_g=4'd4; p.eta_r=4'd4; p.eta_e=4'd4; p.comp=1'b0; p.poly_bytes=12'd1536; end // D2
      4'd8:  begin p.qsel=Q3329; p.nlog=2'd2; p.d64=1'b0; p.slog=2'd2; p.eta_f=4'd2; p.eta_g=4'd3; p.eta_r=4'd2; p.eta_e=4'd3; p.comp=1'b0; p.poly_bytes=12'd3072; end // D3
      4'd9:  begin p.qsel=Q1409; p.nlog=2'd0; p.d64=1'b1; p.slog=2'd0; p.eta_f=4'd3; p.eta_g=4'd3; p.eta_r=4'd3; p.eta_e=4'd3; p.comp=1'b0; p.poly_bytes=12'd672;  end // R1
      4'd10: begin p.qsel=Q1409; p.nlog=2'd1; p.d64=1'b1; p.slog=2'd1; p.eta_f=4'd2; p.eta_g=4'd2; p.eta_r=4'd2; p.eta_e=4'd2; p.comp=1'b0; p.poly_bytes=12'd1344; end // R2
      4'd11: begin p.qsel=Q1409; p.nlog=2'd2; p.d64=1'b1; p.slog=2'd2; p.eta_f=4'd9; p.eta_g=4'd9; p.eta_r=4'd9; p.eta_e=4'd2; p.comp=1'b0; p.poly_bytes=12'd2688; end // R3
      default: p = '0;
    endcase
    return p;
  endfunction

  function automatic logic [11:0] qval(input logic [1:0] qsel);
    case (qsel)
      Q769:    qval = 12'd769;
      Q1409:   qval = 12'd1409;
      default: qval = 12'd3329;
    endcase
  endfunction

  // N = 512 << nlog ; number of poly words = N/16 = 32 << nlog
  function automatic logic [7:0] nwords(input logic [1:0] nlog);
    nwords = 8'd32 << nlog;
  endfunction

  // S in bytes = 16 << slog
  function automatic logic [6:0] sbytes(input logic [1:0] slog);
    sbytes = 7'd16 << slog;
  endfunction

endpackage
