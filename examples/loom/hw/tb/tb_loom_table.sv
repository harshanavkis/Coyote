`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_loom_table — block-level test for the window table.
 */
module tb_loom_table;

logic aclk = 0;
logic aresetn = 0;
always #2 aclk = ~aclk;

logic                  commit;
logic [3:0]            prog_idx;
logic                  prog_valid, prog_route;
logic [PID_BITS-1:0]   prog_pid;
logic [VADDR_BITS-1:0] prog_base;
logic [LEN_BITS-1:0]   prog_len;

logic [3:0]            lu_idx;
logic                  lu_valid, lu_route;
logic [PID_BITS-1:0]   lu_pid;
logic [VADDR_BITS-1:0] lu_base;
logic [LEN_BITS-1:0]   lu_len;

int errors = 0;

loom_table dut (
    .aclk(aclk), .aresetn(aresetn),
    .commit(commit), .prog_idx(prog_idx), .prog_valid(prog_valid),
    .prog_route(prog_route), .prog_pid(prog_pid), .prog_base(prog_base),
    .prog_len(prog_len),
    .lu_idx(lu_idx), .lu_valid(lu_valid), .lu_route(lu_route),
    .lu_pid(lu_pid), .lu_base(lu_base), .lu_len(lu_len)
);

task check(input bit cond, input string msg);
    if (!cond) begin
        errors++;
        $display("FAIL: %s", msg);
    end
endtask

task program_entry(input [3:0] idx, input v, input r,
                   input [PID_BITS-1:0] pid,
                   input [VADDR_BITS-1:0] base,
                   input [LEN_BITS-1:0] len);
    @(negedge aclk);
    prog_idx = idx; prog_valid = v; prog_route = r;
    prog_pid = pid; prog_base = base; prog_len = len;
    commit = 1;
    @(negedge aclk);
    commit = 0;
endtask

initial begin
    commit = 0; prog_idx = 0; prog_valid = 0; prog_route = 0;
    prog_pid = 0; prog_base = 0; prog_len = 0; lu_idx = 0;

    repeat (5) @(negedge aclk);
    aresetn = 1;
    repeat (2) @(negedge aclk);

    // 1. All entries invalid after reset
    for (int i = 0; i < 16; i++) begin
        lu_idx = i[3:0];
        #1 check(!lu_valid, $sformatf("entry %0d valid after reset", i));
    end

    // 2. Program a local entry, look it up
    program_entry(4'd1, 1'b1, 1'b0, 6'd1, 48'h7f1b_d420_0000, 28'h040_0000);
    lu_idx = 4'd1;
    #1 check(lu_valid && !lu_route && lu_pid == 6'd1 &&
             lu_base == 48'h7f1b_d420_0000 && lu_len == 28'h040_0000,
             "local entry 1 mismatch");

    // 3. Program an rdma entry, entry 1 must be untouched
    program_entry(4'd2, 1'b1, 1'b1, 6'd0, 48'h7f9e_8860_0000, 28'h040_0000);
    lu_idx = 4'd2;
    #1 check(lu_valid && lu_route && lu_base == 48'h7f9e_8860_0000,
             "rdma entry 2 mismatch");
    lu_idx = 4'd1;
    #1 check(lu_valid && !lu_route && lu_base == 48'h7f1b_d420_0000,
             "entry 1 clobbered by programming entry 2");

    // 4. Invalidate entry 1
    program_entry(4'd1, 1'b0, 1'b0, '0, '0, '0);
    lu_idx = 4'd1;
    #1 check(!lu_valid, "entry 1 still valid after invalidate");

    if (errors == 0) $display("TB PASS (tb_loom_table)");
    else             $display("TB FAIL (tb_loom_table): %0d errors", errors);
    $finish;
end

endmodule
