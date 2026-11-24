// Command controller: latches (op, param) on accept and runs the operation as a table of jobs.
// Each job names a unit, its arguments and a dependency mask.  Every cycle each
// idle unit starts its lowest-index job whose dependencies are done; the job's arguments are
// registered (cur[u]) together with the unit's start pulse, so unit configuration and poly-slot
// routing come from registers.  The command completes when every job is done.
module aigis_ctrl (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        cmd_valid,
  output logic        cmd_ready,
  input  logic [1:0]  cmd_op,
  input  logic [1:0]  cmd_param,
  output logic        busy,
  output logic        done,
  output logic        err,
  output logic [1:0]  param,
  output logic        accept,
  input  logic        fail,          // re-encryption mismatch (Dec)
  input  logic [11:0] u_done,        // unit done pulses
  output logic [11:0] u_start,       // unit start pulses
  output aigis_ctrl_pkg::job_t [11:0] cur,  // arguments of the job running on each unit
  output logic [11:0] u_sdone        // every start-dependency (followed job) of the running job has finished
);
  import aigis_pkg::*;
  import aigis_ctrl_pkg::*;

  logic [1:0] op;
  logic       run;
  logic       rdy;
  job_t [NJ-1:0] tab;
  always_comb tab = job_table(op, param, fail);

  logic [NJ-1:0] started, fin;
  logic [11:0]   ubusy;
  logic [11:0][4:0] uj;

  // ready-to-start selection
  logic [11:0]      sel_v;
  logic [11:0][4:0] sel_j;
  always_comb begin
    sel_v = '0; sel_j = '0;
    for (int u = 0; u < 12; u++) begin
      for (int j = NJ - 1; j >= 0; j--) begin
        if (run && tab[j].v && tab[j].unit == 4'(u) && !started[j] && ((tab[j].deps & ~fin) == '0) && ((tab[j].sdeps & ~started) == '0)) begin
          sel_v[u] = 1'b1; sel_j[u] = 5'(j);
        end
      end
      if (ubusy[u]) sel_v[u] = 1'b0;
    end
  end
  always_comb for (int u = 0; u < 12; u++) u_sdone[u] = ((cur[u].sdeps & ~fin) == '0);
  logic all_done;
  always_comb begin
    all_done = 1'b1;
    for (int j = 0; j < NJ; j++) if (tab[j].v && !fin[j]) all_done = 1'b0;
  end

  assign cmd_ready = rdy && !run;
  assign accept    = cmd_valid && cmd_ready;
  assign busy      = run;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rdy <= 1'b0; run <= 1'b0; done <= 1'b0; err <= 1'b0; op <= '0; param <= 2'd1;
      started <= '0; fin <= '0; ubusy <= '0; u_start <= '0; cur <= '0; uj <= '0;
    end else begin
      rdy <= 1'b1;
      u_start <= '0;
      if (accept) begin
        done <= 1'b0; err <= 1'b0;
        if (cmd_op == 2'd3 || cmd_param == 2'd0) begin
          done <= 1'b1; err <= 1'b1;           // invalid command: nothing to run
        end else begin
          run <= 1'b1; op <= cmd_op; param <= cmd_param; started <= '0; fin <= '0;
        end
      end else if (run) begin
        for (int u = 0; u < 12; u++) begin
          if (sel_v[u]) begin
            started[sel_j[u]] <= 1'b1; uj[u] <= sel_j[u]; cur[u] <= tab[sel_j[u]];
            u_start[u] <= 1'b1; ubusy[u] <= 1'b1;
          end
          if (u_done[u] && ubusy[u]) begin fin[uj[u]] <= 1'b1; ubusy[u] <= 1'b0; end
        end
        if (all_done) begin run <= 1'b0; done <= 1'b1; end
      end
    end
  end
endmodule
