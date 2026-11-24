// Reset synchroniser: asynchronous assertion, release after two clk edges.
module qube_rst_sync (
  input  logic clk,
  input  logic rst_n,
  output logic rst_sync_n
);
  logic r1, r2;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin r1 <= 1'b0; r2 <= 1'b0; end
    else begin r1 <= 1'b1; r2 <= r1; end
  end
  assign rst_sync_n = r2;
endmodule
