// Copy engine: n 64-bit words from (src kind, addr) to (dst kind, addr).
// kinds: 0 regfile, 1 PK, 2 SK, 3 CT (buffer kinds map to buffer select kind-1), 4 SS (dst only).
module aigis_cp (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          start,
  input  logic [2:0]    sk,
  input  logic [9:0]    sa,
  input  logic [2:0]    dk,
  input  logic [9:0]    da,
  input  logic [9:0]    n,
  output logic          busy,
  output logic          done,
  input  logic [87:0][63:0] rf,
  output logic          rf_we,
  output logic [6:0]    rf_waddr,
  output logic [63:0]   rf_wdata,
  output logic          br_req,
  output logic [1:0]    br_sel,
  output logic [9:0]    br_addr,
  input  logic          br_gnt,
  input  logic          br_rv,
  input  logic [63:0]   br_data,
  output logic          bw_req,
  output logic [1:0]    bw_sel,
  output logic [9:0]    bw_addr,
  output logic [63:0]   bw_data,
  input  logic          bw_gnt,
  output logic          ss_we,
  output logic [2:0]    ss_idx,
  output logic [63:0]   ss_data
);
  logic [2:0] sk_q, dk_q;
  logic [9:0] sa_q, da_q, n_q, ri, wi;
  logic [2:0] outst;                 // buffer reads in flight
  logic [3:0][63:0] fq;
  logic [2:0] fc;
  logic [1:0] fh, ft;
  logic rd_go, push, pop, popok;
  assign rd_go = busy && (ri < n_q) && ({1'b0, fc} + {1'b0, outst} < 4'd4);
  assign br_req  = rd_go && (sk_q != 3'd0);
  assign br_sel  = 2'(sk_q - 3'd1);
  assign br_addr = sa_q + ri;
  assign push = (rd_go && sk_q == 3'd0) || br_rv;
  // write side
  assign bw_req  = busy && (fc != 3'd0) && (dk_q >= 3'd1) && (dk_q <= 3'd3);
  assign bw_sel  = 2'(dk_q - 3'd1);
  assign bw_addr = da_q + wi;
  assign bw_data = fq[fh];
  assign popok = (dk_q == 3'd0 || dk_q == 3'd4) ? 1'b1 : bw_gnt;
  assign pop = busy && (fc != 3'd0) && popok;
  always_comb begin
    rf_we = pop && dk_q == 3'd0; rf_waddr = 7'(da_q + wi); rf_wdata = fq[fh];
    ss_we = pop && dk_q == 3'd4; ss_idx = 3'(da_q + wi); ss_data = fq[fh];
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin busy <= 1'b0; done <= 1'b0; ri <= '0; wi <= '0; outst <= '0; fc <= '0; fh <= '0; ft <= '0;
      sk_q <= '0; dk_q <= '0; sa_q <= '0; da_q <= '0; n_q <= '0; fq <= '0; end
    else begin
      done <= 1'b0;
      if (start && !busy) begin
        busy <= 1'b1; sk_q <= sk; dk_q <= dk; sa_q <= sa; da_q <= da; n_q <= n;
        ri <= '0; wi <= '0; outst <= '0; fc <= '0; fh <= '0; ft <= '0;
      end else if (busy) begin
        if ((sk_q == 3'd0 && rd_go) || (br_req && br_gnt)) ri <= ri + 10'd1;
        outst <= outst + ((br_req && br_gnt) ? 3'd1 : 3'd0) - (br_rv ? 3'd1 : 3'd0);
        if (push) begin fq[ft] <= (sk_q == 3'd0) ? rf[7'(sa_q + ri)] : br_data; ft <= ft + 2'd1; end
        if (pop) begin fh <= fh + 2'd1; wi <= wi + 10'd1; end
        fc <= fc + (push ? 3'd1 : 3'd0) - (pop ? 3'd1 : 3'd0);
        if (pop && wi + 10'd1 == n_q) begin busy <= 1'b0; done <= 1'b1; end
      end
    end
  end
endmodule
