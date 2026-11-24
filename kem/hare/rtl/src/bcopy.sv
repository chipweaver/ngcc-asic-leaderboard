// Buffer-to-dense copy engine: reads nw*4 consecutive 64-bit words starting at word base
// from an object buffer (req/gnt, data the cycle after the grant) and emits 256-bit words.
module bcopy import hare_pkg::*; (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         start,
  input  logic [12:0]  base,
  input  logic [9:0]   nw,
  output ob_req_t      breq,
  input  logic         bgnt,
  input  logic [63:0]  bq,
  output logic         out_valid,
  output logic [255:0] out_data,
  output logic         busy
);
  logic [11:0] rd_cnt;     // reads issued
  logic [1:0]  lane;       // lane of the returning word
  logic        pend;
  logic [191:0] asm_q;
  logic [11:0] total;
  assign total = {nw, 2'b00};

  always_comb begin
    breq = '0;
    breq.req = busy && (rd_cnt != total);
    breq.a   = base + rd_cnt[12-1:0];
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rd_cnt <= '0; lane <= '0; pend <= 1'b0; asm_q <= '0; busy <= 1'b0;
      out_valid <= 1'b0; out_data <= '0;
    end else begin
      out_valid <= 1'b0;
      if (start) begin
        busy <= 1'b1; rd_cnt <= '0; lane <= '0; pend <= 1'b0;
      end else if (busy) begin
        pend <= breq.req && bgnt;
        if (breq.req && bgnt) rd_cnt <= rd_cnt + 12'd1;
        if (pend) begin
          lane <= lane + 2'd1;
          case (lane)
            2'd0: asm_q[63:0]    <= bq;
            2'd1: asm_q[127:64]  <= bq;
            2'd2: asm_q[191:128] <= bq;
            default: begin
              out_valid <= 1'b1;
              out_data  <= {bq, asm_q};
            end
          endcase
          if (lane == 2'd3 && rd_cnt == total && !(breq.req && bgnt)) busy <= 1'b0;
        end
      end
    end
  end
endmodule
