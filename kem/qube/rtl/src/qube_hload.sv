// Dense XOF loader: pairs 256-bit stream blocks into 512-bit stream words and writes NW slot words
// out_j = stream bytes [skip + 64j, skip + 64j + 64) to the selected lane slot(s); bits >= n of the
// last word are cleared; slot is a mask (KeyGen writes a dense vector to both slots of a lane).
// h1 uses skip 0 from block 0, h2 skip nb mod 32 from block floor(nb/32).
module qube_hload
  import qube_pkg::*;
(
  input  logic          clk,
  input  logic          rst_n,
  input  logic          start,
  input  logic [2:0]    ps,
  input  logic [5:0]    skip,
  input  logic [1:0]    lanes,
  input  logic [1:0]    slot,         // slot mask (both slots receive the same words)
  output logic          busy,
  output logic          done,
  input  logic          ivalid,
  output logic          iready,
  input  logic [255:0]  idata,
  output logic [1:0]    sw_en,
  output logic [1:0]    sw_slot,
  output logic [8:0]    sw_addr,
  output logic [511:0]  sw_data
);
  logic         run;
  logic [5:0]   sk;
  logic [1:0]   ln;
  logic [1:0]   sl;
  logic [8:0]   nw, j;
  logic [8:0]   lastbits;
  logic         half;        // low block of the current pair received
  logic [255:0] lo;
  logic [511:0] prev;
  logic         pv;          // prev holds a stream word
  logic [511:0] lmask;

  always_comb for (int i = 0; i < 512; i++) lmask[i] = (9'(i) < lastbits);

  logic [511:0] word;
  logic [1023:0] fun;
  logic [511:0] outw;
  always_comb begin
    word  = {idata, lo};
    fun   = {word, prev} >> {sk, 3'd0};
    outw  = (sk == 6'd0) ? word : fun[511:0];
    if (j == nw - 9'd1) outw = outw & lmask;
  end
  assign iready = run;
  assign busy = run;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run <= 1'b0; done <= 1'b0; sk <= '0; ln <= '0; sl <= '0; nw <= '0; j <= '0; lastbits <= '0;
      half <= 1'b0; lo <= '0; prev <= '0; pv <= 1'b0;
      sw_en <= '0; sw_slot <= '0; sw_addr <= '0; sw_data <= '0;
    end else begin
      done <= 1'b0; sw_en <= '0;
      if (start) begin
        run <= 1'b1; sk <= skip; ln <= lanes; sl <= slot; nw <= p_nw(ps); lastbits <= p_lastbits(ps);
        j <= '0; half <= 1'b0; pv <= 1'b0;
      end else if (run && ivalid) begin
        if (!half) begin lo <= idata; half <= 1'b1; end
        else begin
          half <= 1'b0; prev <= word; pv <= 1'b1;
          // with a skip the first stream word only primes prev
          if (sk == 6'd0 || pv) begin
            sw_en <= ln; sw_slot <= sl; sw_addr <= j; sw_data <= outw;
            j <= j + 9'd1;
            if (j == nw - 9'd1) begin run <= 1'b0; done <= 1'b1; end
          end
        end
      end
    end
  end
endmodule
