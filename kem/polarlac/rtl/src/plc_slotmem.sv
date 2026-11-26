// Polynomial slot memory: 16 slots x 2 banks of plc_sram_1r1w(64x128) and the engine crossbar.
// Read ports: request (slot, ren, raddr) combinationally steered to the slot's bank; SRAM data
// registered once more here -> engine sees rdata exactly 2 cycles after ren.
// Write ports: steered combinationally, write lands at the clock edge.
// Slot ownership is exclusive by construction (controller/TSAMP); simultaneous requests to the same
// bank by two ports are a design error reported in simulation.
module plc_slotmem #(
  parameter int NR = 7,       // engine read ports
  parameter int NW = 5        // engine write ports
) (
  input  logic                       clk,
  input  logic                       rst_n,
  input  logic [NR-1:0][3:0]         r_slot,
  input  logic [NR-1:0][1:0]         r_en,
  input  logic [NR-1:0][1:0][5:0]    r_addr,
  output logic [NR-1:0][1:0][127:0]  r_data,
  input  logic [NW-1:0][3:0]         w_slot,
  input  logic [NW-1:0][1:0]         w_en,
  input  logic [NW-1:0][1:0][5:0]    w_addr,
  input  logic [NW-1:0][1:0][127:0]  w_data
);
  localparam int NS = 16;

  logic [NS-1:0][1:0]        s_ren, s_wen;
  logic [NS-1:0][1:0][5:0]   s_raddr, s_waddr;
  logic [NS-1:0][1:0][127:0] s_rdata, s_wdata;

  // ---------------- request steering (AND-OR, one-hot by construction) ----------------
  always_comb begin
    s_ren = '0; s_wen = '0; s_raddr = '0; s_waddr = '0; s_wdata = '0;
    for (int s = 0; s < NS; s++)
      for (int b = 0; b < 2; b++) begin
        for (int p = 0; p < NR; p++)
          if (r_en[p][b] && r_slot[p] == 4'(s)) begin
            s_ren[s][b]   = 1'b1;
            s_raddr[s][b] = s_raddr[s][b] | r_addr[p][b];
          end
        for (int p = 0; p < NW; p++)
          if (w_en[p][b] && w_slot[p] == 4'(s)) begin
            s_wen[s][b]   = 1'b1;
            s_waddr[s][b] = s_waddr[s][b] | w_addr[p][b];
            s_wdata[s][b] = s_wdata[s][b] | w_data[p][b];
          end
      end
  end

  for (genvar s = 0; s < NS; s++) begin : g_s
    for (genvar b = 0; b < 2; b++) begin : g_b
      plc_sram_1r1w #(.DEPTH(64), .WIDTH(128)) u_ram (
        .clk  (clk),
        .ren  (s_ren[s][b]),
        .raddr(s_raddr[s][b]),
        .rdata(s_rdata[s][b]),
        .wen  (s_wen[s][b]),
        .waddr(s_waddr[s][b]),
        .wdata(s_wdata[s][b])
      );
    end
  end

  // ---------------- read return: pipelined slot id, registered data ----------------
  logic [NR-1:0][3:0] r_slot_q;
  logic [NR-1:0][1:0] r_en_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      r_slot_q <= '0;
      r_en_q   <= '0;
    end else begin
      r_slot_q <= r_slot;
      r_en_q   <= r_en;
    end
  end
  always_ff @(posedge clk) begin
    for (int p = 0; p < NR; p++)
      for (int b = 0; b < 2; b++)
        if (r_en_q[p][b]) r_data[p][b] <= s_rdata[r_slot_q[p]][b];
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin
    if (rst_n) begin
      for (int s = 0; s < NS; s++)
        for (int b = 0; b < 2; b++) begin
          int nr, nw;
          nr = 0; nw = 0;
          for (int p = 0; p < NR; p++) if (r_en[p][b] && r_slot[p] == 4'(s)) nr++;
          for (int p = 0; p < NW; p++) if (w_en[p][b] && w_slot[p] == 4'(s)) nw++;
          if (nr > 1) $error("plc_slotmem: %0d read ports on slot %0d bank %0d", nr, s, b);
          if (nw > 1) $error("plc_slotmem: %0d write ports on slot %0d bank %0d", nw, s, b);
        end
    end
  end
`endif
endmodule
