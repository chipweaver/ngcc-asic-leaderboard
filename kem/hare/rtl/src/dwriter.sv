// Dense-operand writer: stores an n-bit vector arriving as 256-bit words (word i = bits
// 256i..256i+255) into an eight-bank dense memory (word i -> bank i[2:0], address i>>3) and
// appends the cyclic extension (bits n..n+1023 = bits 0..1023) so that any 1024-bit window
// starting below n lies in five adjacent words. Bits >= n of the last data word are replaced by
// the extension. Accepts one word per cycle (in_valid); done pulses after the last word's writes.
module dwriter import hare_pkg::*; (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,      // resets the word counter
  input  logic [1:0]   lvl,
  input  logic         in_valid,
  input  logic [255:0] in_data,
  output dm_req_t      mreq,
  output logic         done
);
  logic [9:0]    idx;
  // words 0..3 of the vector, then (from word 4 on) the extension pre-shifted by nr:
  // er = {words 3..0} << nr, so er[255:0] fills word L above bit nr and er[256i +: 256] is word L+i
  logic [1279:0] er;
  logic [9:0]    nw;
  logic [7:0]    nr;
  assign nw = p_nw(lvl);
  assign nr = p_nr(lvl);

  logic         last;
  logic [255:0] mask, wl;
  logic [255:0] wx [1:4];          // extension words L+1 .. L+4
  logic [9:0]   ix [1:4];
  assign last = (idx == nw - 10'd1);
  always_comb begin
    mask = ~(256'd0) >> (9'd256 - {1'b0, nr});
    wl   = (in_data & mask) | er[255:0];
    for (int i = 1; i <= 4; i = i + 1) begin
      wx[i] = er[256*i +: 256];
      ix[i] = idx + 10'(i);
    end
  end

  always_comb begin
    mreq = '0;
    if (in_valid) begin
      // words idx .. idx+4 always sit in five different banks
      mreq.ce[idx[2:0]] = 1'b1; mreq.we[idx[2:0]] = 1'b1; mreq.a[idx[2:0]] = idx[9:3];
      mreq.d[idx[2:0]]  = last ? wl : in_data;
      if (last) begin
        for (int i = 1; i <= 4; i = i + 1) begin
          mreq.ce[ix[i][2:0]] = 1'b1; mreq.we[ix[i][2:0]] = 1'b1;
          mreq.a[ix[i][2:0]]  = ix[i][9:3]; mreq.d[ix[i][2:0]] = wx[i];
        end
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      idx <= '0; er <= '0; done <= 1'b0;
    end else begin
      done <= 1'b0;
      if (start) idx <= '0;
      else if (in_valid) begin
        if (idx < 10'd4) er[256*idx[1:0] +: 256] <= in_data;
        if (idx == 10'd4) er <= er << nr;       // nw >= 82: done long before the last word
        idx <= idx + 10'd1;
        if (last) done <= 1'b1;
      end
    end
  end
endmodule
