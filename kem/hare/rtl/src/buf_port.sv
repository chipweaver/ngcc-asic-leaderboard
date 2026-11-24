// Object-buffer port: host access while idle, otherwise a fixed-priority arbiter over N
// internal requesters (index 0 highest). gnt is combinational; read data (q) is valid in the
// cycle after the grant and is broadcast to all requesters.
module buf_port import hare_pkg::*; #(
  parameter int N = 4,
  parameter int AW0 = 11,
  parameter int AW1 = 10
) (
  input  logic        clk,
  input  logic        busy,
  input  logic        h_en,
  input  logic        h_we,
  input  logic [12:0] h_a,
  input  logic [63:0] h_d,
  input  logic [7:0]  h_be,
  input  ob_req_t [N-1:0] r,
  output logic [N-1:0] gnt,
  output logic [63:0] q
);
  logic        ce, we;
  logic [12:0] a;
  logic [63:0] d;
  logic [7:0]  be;
  integer i;
  always_comb begin
    gnt = '0; ce = 1'b0; we = 1'b0; a = '0; d = '0; be = '0;
    if (!busy) begin
      ce = h_en; we = h_we; a = h_a; d = h_d; be = h_be;
    end else begin
      for (i = N - 1; i >= 0; i = i - 1) begin
        if (r[i].req) begin
          gnt = '0; gnt[i] = 1'b1;
          ce = 1'b1; we = r[i].we; a = r[i].a; d = r[i].d; be = r[i].be;
        end
      end
    end
  end
  obj_buf #(.AW0(AW0), .AW1(AW1)) u_mem (.clk, .ce, .we, .a, .d, .be, .q);
endmodule
