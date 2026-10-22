`timescale 1ns / 1ps
import lynxTypes::*;

/**
 * tb_stamp_rec: stamp_rec against a model (random events on 3 points,
 * DEPTH 8 so the rings wrap): counts, the last DEPTH stamps of each point,
 * events in the same cycle, and a re-arm.
 */
module tb_stamp_rec;

localparam integer N = 3, D = 8;

logic aclk = 0, aresetn = 0;
always #2 aclk = ~aclk;

logic [N-1:0] ev = '0;
logic arm = 0;
logic [3:0] sel_pt = 0;
logic [$clog2(D)-1:0] sel_idx = 0;
logic [31:0] sel_data, now;
logic [N-1:0][31:0] cnt;

stamp_rec #(.N_PTS(N), .DEPTH(D)) dut (.*);

int errors = 0;
`define CHECK(c, msg) if (!(c)) begin errors++; $display("FAIL [%0t] %s", $time, msg); end

// model: the cycle since the arm at which each event was stamped
longint t = 0, t_arm = 0;
longint m [N][$];
logic [N-1:0] ev_d = '0;
bit armed = 0;
always @(posedge aclk) if (aresetn) begin
    t++;
    if (armed) for (int p = 0; p < N; p++) if (ev_d[p] && !arm) m[p].push_back(t - t_arm - 1);
    ev_d <= ev;
    if (arm) begin t_arm = t; armed = 1; for (int p = 0; p < N; p++) m[p] = {}; end
end

task automatic check(input string what);
    logic [31:0] v;
    repeat (3) @(posedge aclk);
    for (int p = 0; p < N; p++) begin
        `CHECK(cnt[p] == m[p].size(), $sformatf("%s: point %0d count %0d, model %0d", what, p, cnt[p], m[p].size()))
        for (int e = (m[p].size() > D ? m[p].size() - D : 0); e < m[p].size(); e++) begin
            @(negedge aclk); sel_pt = p; sel_idx = e % D;
            repeat (3) @(posedge aclk);
            `CHECK(sel_data == 32'(m[p][e]), $sformatf("%s: point %0d event %0d: %0d, model %0d", what, p, e, sel_data, m[p][e]))
        end
    end
endtask

initial begin
    repeat (5) @(posedge aclk);
    aresetn = 1;
    // events before the first arm are not recorded
    @(negedge aclk); ev = '1; @(negedge aclk); ev = '0;
    repeat (3) @(posedge aclk);
    `CHECK(cnt == '0, "events before the arm counted")
    @(negedge aclk); arm = 1; @(negedge aclk); arm = 0;
    for (int i = 0; i < 60; i++) begin
        @(negedge aclk); ev = N'($urandom);
    end
    @(negedge aclk); ev = '0;
    check("run 1");
    @(negedge aclk); arm = 1; @(negedge aclk); arm = 0;
    for (int i = 0; i < 5; i++) begin
        @(negedge aclk); ev = 3'b101;
        @(negedge aclk); ev = '0;
    end
    check("after re-arm");
    if (errors == 0) $display("TB PASS (tb_stamp_rec)");
    else             $display("TB FAIL (tb_stamp_rec): %0d errors", errors);
    $finish;
end

endmodule
