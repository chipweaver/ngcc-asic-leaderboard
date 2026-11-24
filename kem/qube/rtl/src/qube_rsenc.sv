// Reed-Solomon encoder (reference reed_solomon_encode, model qube.rs_encode): systematic LFSR over
// GF(2^8) with the generator g(x) of degree 2*delta. start samples ps and msg (message byte i on
// msg[8i+7:8i], k = p_k(ps) bytes used). One LFSR step per cycle with 2*delta parallel multipliers,
// message bytes fed from the highest (k-1) down to 0; done pulses after the k-th step.
// cdw[0..2*delta-1] = parity, cdw[2*delta..N1-1] = message, cdw[>= N1] = 0; valid until next start.
module qube_rsenc
  import qube_pkg::*;
(
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,
  input  logic [2:0]   ps,
  input  logic [511:0] msg,
  output logic         busy,
  output logic         done,
  output logic [7:0]   cdw [0:N1MAX-1]
);
  logic         run;
  logic [4:0]   psh;            // one-hot parameter set
  logic [6:0]   left;           // LFSR steps remaining
  logic [511:0] sr;             // message shift register, next byte on sr[511:504]
  logic [7:0]   c_r [0:N1MAX-1];

  // ---------------- start-time placement (constant shifts per parameter set)
  logic [511:0]         mm, srinit;
  logic [8*N1MAX-1:0]   wide, place;
  always_comb begin
    case (ps)
      3'd1:    mm = {320'd0, msg[191:0]};
      3'd2:    mm = {256'd0, msg[255:0]};
      3'd3:    mm = {128'd0, msg[383:0]};
      3'd4:    mm = msg;
      default: mm = {384'd0, msg[127:0]};
    endcase
    wide = {{(8*N1MAX-512){1'b0}}, mm};
    case (ps)
      3'd1:    begin place = wide << (8*42); srinit = mm << (8*(64-24)); end
      3'd2:    begin place = wide << (8*76); srinit = mm << (8*(64-32)); end
      3'd3:    begin place = wide << (8*80); srinit = mm << (8*(64-48)); end
      3'd4:    begin place = wide << (8*86); srinit = mm; end
      default: begin place = wide << (8*18); srinit = mm << (8*(64-16)); end
    endcase
  end

  // ---------------- LFSR step
  logic [6:0] nk;
  logic [7:0] tap, gate;
  logic [7:0] gc   [0:GMAX-1];
  logic [7:0] prod [0:GMAX-1];
  always_comb begin
    nk = ({7{psh[0]}} & 7'd18) | ({7{psh[1]}} & 7'd42) | ({7{psh[2]}} & 7'd76) |
         ({7{psh[3]}} & 7'd80) | ({7{psh[4]}} & 7'd86);
    tap = ({8{psh[0]}} & c_r[17]) | ({8{psh[1]}} & c_r[41]) | ({8{psh[2]}} & c_r[75]) |
          ({8{psh[3]}} & c_r[79]) | ({8{psh[4]}} & c_r[85]);
    gate = sr[511:504] ^ tap;
    for (int j = 0; j < GMAX; j++) begin
      gc[j] = 8'd0;
      for (int q = 0; q < 5; q++) gc[j] = gc[j] | ({8{psh[q]}} & p_rsgen(3'(q), 7'(j)));
      prod[j] = gf_mul(gate, gc[j]);
    end
  end

  assign busy = run;
  assign cdw  = c_r;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run <= 1'b0; done <= 1'b0; psh <= '0; left <= '0; sr <= '0;
      for (int j = 0; j < N1MAX; j++) c_r[j] <= '0;
    end else begin
      done <= 1'b0;
      if (start) begin
        run  <= 1'b1;
        case (ps)
          3'd1:    psh <= 5'b00010;
          3'd2:    psh <= 5'b00100;
          3'd3:    psh <= 5'b01000;
          3'd4:    psh <= 5'b10000;
          default: psh <= 5'b00001;
        endcase
        left <= p_k(ps);
        sr   <= srinit;
        for (int j = 0; j < N1MAX; j++) c_r[j] <= place[8*j +: 8];
      end else if (run) begin
        c_r[0] <= prod[0];
        for (int j = 1; j < GMAX; j++) if (7'(j) < nk) c_r[j] <= c_r[j-1] ^ prod[j];
        sr   <= {sr[503:0], 8'd0};
        left <= left - 7'd1;
        if (left == 7'd1) begin run <= 1'b0; done <= 1'b1; end
      end
    end
  end
endmodule
