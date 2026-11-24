// Reset synchroniser: asynchronous assertion, two-flop synchronised deassertion.
module aigis_rst_sync (
  input  logic clk,
  input  logic rst_n,
  output logic rst_n_out
);
  logic s1;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin s1 <= 1'b0; rst_n_out <= 1'b0; end
    else begin s1 <= 1'b1; rst_n_out <= s1; end
  end
endmodule
