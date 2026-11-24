// One object buffer (PK, SK or CT): 544 words of 512 bits as a 512-word bank and a 32-word tail bank,
// single port each, 64-bit write lanes. While the engine is idle the host port owns it (one 64-bit
// word per access, read data returned through rd_q two edges after the request edge). While busy
// the engine requesters share it: one writer (full words) and five readers (0 mover, 1/2 hash unit
// engines E0/E1, 3/4 J-hash unit engines E0/E1), fixed priority writer > 1 > 2 > 3 > 4 > 0 (a hash
// engine needs one word per compression; the mover's read stream tolerates the gaps). A granted read returns its word with rv[i] in the
// third cycle after the grant cycle. Macro inputs are driven from registers.
module qube_objbuf (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          eng,          // 1: engine owns the port
  // host
  input  logic          h_en,
  input  logic          h_we,
  input  logic [12:0]   h_addr,       // 8-byte word index
  input  logic [63:0]   h_wdata,
  output logic [511:0]  rd_q,         // registered macro output (host lane select outside)
  output logic [2:0]    h_lane,       // lane of the host read whose data is in rd_q
  // engine write
  input  logic          w_rq,
  input  logic [9:0]    w_addr,
  input  logic [511:0]  w_data,
  output logic          w_gnt,
  // engine reads
  input  logic [4:0]    r_rq,
  input  logic [9:0]    r_addr [0:4],
  output logic [4:0]    r_gnt,
  output logic [4:0]    rv
);
  // ---------------- arbitration
  always_comb begin
    w_gnt = eng && w_rq;
    r_gnt = 5'b00000;
    if (eng && !w_rq) begin
      if (r_rq[1]) r_gnt = 5'b00010;
      else if (r_rq[2]) r_gnt = 5'b00100;
      else if (r_rq[3]) r_gnt = 5'b01000;
      else if (r_rq[4]) r_gnt = 5'b10000;
      else if (r_rq[0]) r_gnt = 5'b00001;
    end
  end

  // coverage probe (CH-PAR (a)): a hash-engine read granted while the mover's read waits
  logic cov_eng_over_mover, cov_eng_wait;
  assign cov_eng_over_mover = (|r_gnt[4:1]) && r_rq[0];
  // an engine read request waiting behind another engine's read (five-reader arbitration)
  assign cov_eng_wait = (|(r_rq[4:1] & ~r_gnt[4:1])) && (|r_gnt[4:1]);

  // ---------------- registered macro inputs
  logic         p_ce, p_we;
  logic [9:0]   p_a;
  logic [7:0]   p_wm;
  logic [511:0] p_d;
  logic [4:0]   p_tag;      // one-hot reader of the read in flight
  logic         p_host;
  logic [2:0]   p_lane;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      p_ce <= 1'b0; p_we <= 1'b0; p_a <= '0; p_wm <= '0; p_d <= '0; p_tag <= '0; p_host <= 1'b0; p_lane <= '0;
    end else begin
      p_tag <= '0; p_host <= 1'b0;
      if (!eng) begin
        // host words 0..4351 (544 buffer words); accesses beyond are ignored
        p_ce <= h_en && (h_addr < 13'd4352); p_we <= h_we; p_a <= h_addr[12:3]; p_wm <= 8'd1 << h_addr[2:0];
        p_d <= {8{h_wdata}}; p_host <= h_en && !h_we; p_lane <= h_addr[2:0];
      end else if (w_gnt) begin
        p_ce <= 1'b1; p_we <= 1'b1; p_a <= w_addr; p_wm <= 8'hFF; p_d <= w_data;
      end else if (|r_gnt) begin
        p_ce <= 1'b1; p_we <= 1'b0;
        p_a <= r_gnt[0] ? r_addr[0] : r_gnt[1] ? r_addr[1] : r_gnt[2] ? r_addr[2] : r_gnt[3] ? r_addr[3] : r_addr[4];
        p_tag <= r_gnt;
      end else begin
        p_ce <= 1'b0; p_we <= 1'b0;
      end
    end
  end

  logic [511:0] q_lo, q_hi;
  logic         sel_hi_q;
  SRAM1RWM_512X512 u_lo (.CLK(clk), .CE(p_ce && !p_a[9]), .WE(p_we), .WM(p_wm), .A(p_a[8:0]), .D(p_d), .Q(q_lo));
  SRAM1RWM_32X512  u_hi (.CLK(clk), .CE(p_ce && p_a[9]), .WE(p_we), .WM(p_wm), .A(p_a[4:0]), .D(p_d), .Q(q_hi));

  logic [4:0] q_tag;
  logic       q_host;
  logic [2:0] q_lane;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sel_hi_q <= 1'b0; q_tag <= '0; q_host <= 1'b0; q_lane <= '0; rd_q <= '0; rv <= '0; h_lane <= '0;
    end else begin
      if (p_ce && !p_we) sel_hi_q <= p_a[9];
      q_tag <= p_tag; q_host <= p_host; q_lane <= p_lane;
      rv <= q_tag;
      if (q_host || (|q_tag)) rd_q <= sel_hi_q ? q_hi : q_lo;
      h_lane <= q_lane;
    end
  end
endmodule
