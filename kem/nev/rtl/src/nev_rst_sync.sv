// nev_rst_sync: asynchronous-assert, synchronous-release reset synchroniser (two flops).
module nev_rst_sync (
  input  logic clk,
  input  logic rst_n,
  output logic rst_sync_n
);
  logic r1;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin r1 <= 1'b0; rst_sync_n <= 1'b0; end
    else        begin r1 <= 1'b1; rst_sync_n <= r1;  end
  end
endmodule
