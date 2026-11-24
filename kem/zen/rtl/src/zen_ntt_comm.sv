// zen_ntt_comm: multipath delay commutator between two NTT butterfly stages.
// Input pairs (U = element with local bit 0, L = element with local bit 1 of the PREVIOUS stage);
// output pairs at local distance d (d = 4/2/1 elements in the group's local index space) for the
// next stage; d = 0 bypasses. Each element carries its word, word address, group-local index L,
// valid and pass tag. Shift registers advance every cycle.
//   phase = U valid and bit log2(d) of U's local index
//   Y1 = L delayed by d;   phase 0: P_in = U, Q = Y1;   phase 1: P_in = Y1, Q = U
//   output pair = (P delayed by d, Q)
// d is taken from the pass of the incoming valid element and held while the commutator drains
// (the issue logic separates passes by >= 8 cycles so elements of two passes never mix).
module zen_ntt_comm #(
  parameter int DMAX = 4,
  parameter int EW   = 269            // element width: {pass, v, L[3:0], addr[6:0], word[255:0]}
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic [2:0]    d_in,         // requested delay for the incoming element's pass (0,1,2,4)
  input  logic [EW-1:0] u,
  input  logic [EW-1:0] l,
  output logic [EW-1:0] ou,
  output logic [EW-1:0] ol
);
  localparam int VB = EW - 2;         // valid bit position
  logic [2:0] d;
  logic [EW-1:0] ysh [0:DMAX-1];
  logic [EW-1:0] psh [0:DMAX-1];
  logic [EW-1:0] y1, pin, q, pout;
  logic          phase;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) d <= 3'd0;
    else if (u[VB]) d <= d_in;
  end

  always_comb begin
    logic [2:0] dd;
    dd = u[VB] ? d_in : d;
    case (dd)
      3'd1:    phase = u[VB] && u[256+7+0];
      3'd2:    phase = u[VB] && u[256+7+1];
      default: phase = u[VB] && u[256+7+2];
    endcase
    y1   = (dd == 3'd0) ? '0 : ysh[(dd > DMAX) ? DMAX-1 : dd-1];
    pout = (dd == 3'd0) ? '0 : psh[(dd > DMAX) ? DMAX-1 : dd-1];
    if (!phase) begin pin = u;  q = y1; end
    else        begin pin = y1; q = u;  end
    if (dd == 3'd0) begin ou = u; ol = l; end
    else begin
      ou = pout; ol = q;
      if (!(pout[VB] && q[VB])) begin ou[VB] = 1'b0; ol[VB] = 1'b0; end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < DMAX; i++) begin ysh[i] <= '0; psh[i] <= '0; end
    end else begin
      ysh[0] <= l; psh[0] <= pin;
      for (int i = 1; i < DMAX; i++) begin ysh[i] <= ysh[i-1]; psh[i] <= psh[i-1]; end
    end
  end
endmodule
