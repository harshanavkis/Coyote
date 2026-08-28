`timescale 1ns / 1ps
import lynxTypes::*;

/**
 * The minus-nw top driven end to end, THROUGH ITS ARBITER.
 *
 * tb_loom_loopback instantiates loom_rx with .grant(1'b1), so no testbench
 * has ever run the receive path against the shared-path arbiter. On one card
 * that matters: the engine holds the arbiter while it streams (eng_busy), and
 * loom_rx needs a grant to accept even the header, so the two can deadlock -
 * which is exactly what the first hardware run did, with every RX counter at
 * zero and the transmit path 100% stalled. examples/loom cannot reach that
 * state because the two ends are on different cards.
 *
 * So this instantiates the real top, programs a window over AXI-Lite, issues
 * a descriptor, and checks the payload lands. It models the shell around it:
 * sq_rd answered with a source pattern on axis_host_recv, and sq_wr plus
 * axis_host_send captured into a memory model.
 */
module tb_loom_mnw;

localparam [47:0] DST_VA  = 48'h7f6a_1000_0000;
localparam [47:0] SRC_VA  = 48'h7f6a_2000_0000;
localparam int    COPY_B  = 4096;             // one PMTU packet of payload
localparam [3:0]  WIN     = 4'd1;
localparam [5:0]  MY_PID  = 6'd0;

logic aclk = 0;
logic aresetn = 0;
always #2 aclk = ~aclk;

AXI4L axi_ctrl (.aclk(aclk), .aresetn(aresetn));
metaIntf #(.STYPE(irq_not_t)) notify (.*);
metaIntf #(.STYPE(req_t)) sq_rd (.*);
metaIntf #(.STYPE(req_t)) sq_wr (.*);
metaIntf #(.STYPE(ack_t)) cq_rd (.*);
metaIntf #(.STYPE(ack_t)) cq_wr (.*);
AXI4SR axis_host_recv [N_STRM_AXI] (.*);
AXI4SR axis_host_send [N_STRM_AXI] (.*);

int errors = 0;
task check(input bit cond, input string msg);
    if (cond) $display("PASS: %s", msg);
    else begin $display("FAIL: %s", msg); errors++; end
endtask

// Shell side defaults
assign notify.ready = 1'b1;
assign cq_rd.valid  = 1'b0;
assign cq_rd.data   = '0;
assign cq_wr.valid  = 1'b0;
assign cq_wr.data   = '0;
assign sq_rd.ready  = 1'b1;
assign sq_wr.ready  = 1'b1;
assign axis_host_send[0].tready = 1'b1;

`include "vfpga_top.svh"

// ---- source pattern, and the pull feeder that answers the engine's sq_rd --
function automatic logic [63:0] src_word(input int idx);
    src_word = {32'h5A5A_0000, 32'(idx)};
endfunction

int pull_jobs[$];
always @(posedge aclk)
    if (sq_rd.valid && sq_rd.ready) pull_jobs.push_back(int'(sq_rd.data.len));

initial begin
    axis_host_recv[0].tvalid = 0;
    axis_host_recv[0].tdata  = '0;
    axis_host_recv[0].tkeep  = '0;
    axis_host_recv[0].tlast  = 0;
    axis_host_recv[0].tid    = '0;
    forever begin
        int beats, base_w;
        @(posedge aclk);
        if (pull_jobs.size() == 0) continue;
        beats  = (pull_jobs.pop_front() + 63) / 64;
        base_w = 0;
        for (int i = 0; i < beats; i++) begin
            @(negedge aclk);
            for (int l = 0; l < 8; l++)
                axis_host_recv[0].tdata[64*l +: 64] = src_word(base_w + i*8 + l);
            axis_host_recv[0].tkeep  = {64{1'b1}};
            axis_host_recv[0].tvalid = 1;
            axis_host_recv[0].tlast  = (i == beats - 1);
            do @(posedge aclk); while (!axis_host_recv[0].tready);
            @(negedge aclk);
            axis_host_recv[0].tvalid = 0;
            axis_host_recv[0].tlast  = 0;
        end
    end
end

// ---- host memory: what sq_wr + axis_host_send actually land ---------------
logic [63:0] mem [logic [47:0]];
logic [47:0] wr_cursor;
logic [27:0] wr_left;
int host_writes = 0;

always @(posedge aclk) begin
    if (sq_wr.valid && sq_wr.ready) begin
        wr_cursor = sq_wr.data.vaddr;
        wr_left   = sq_wr.data.len;
        host_writes++;
    end
    if (axis_host_send[0].tvalid && axis_host_send[0].tready) begin
        for (int l = 0; l < 8 && wr_left > 0; l++) begin
            mem[(wr_cursor + l*8) >> 3] = axis_host_send[0].tdata[64*l +: 64];
            wr_left = (wr_left >= 8) ? wr_left - 8 : 0;
        end
        wr_cursor = wr_cursor + 64;
    end
end

// ---- CSR access ----------------------------------------------------------
task axil_write(input [15:0] addr, input [63:0] data);
    @(negedge aclk);
    axi_ctrl.awaddr = {48'b0, addr}; axi_ctrl.awvalid = 1;
    axi_ctrl.wdata = data; axi_ctrl.wstrb = 8'hFF; axi_ctrl.wvalid = 1;
    axi_ctrl.bready = 1;
    do @(posedge aclk); while (!(axi_ctrl.awready && axi_ctrl.wready));
    @(negedge aclk);
    axi_ctrl.awvalid = 0; axi_ctrl.wvalid = 0;
    while (!axi_ctrl.bvalid) @(posedge aclk);
    @(negedge aclk);
endtask

task program_window(input [3:0] idx, input bit route, input [5:0] pid,
                    input [63:0] base, input [63:0] len);
    axil_write(16'd0,  {60'b0, idx});
    axil_write(16'd8,  {62'b0, route, 1'b1});
    axil_write(16'd16, {58'b0, pid});
    axil_write(16'd24, base);
    axil_write(16'd32, len);
    axil_write(16'd40, 64'd1);
endtask

task issue_copy(input [3:0] win, input [27:0] off, input [27:0] len);
    axil_write(16'd64,  {win, 32'b0, off});
    axil_write(16'd72,  {16'b0, SRC_VA});
    axil_write(16'd80,  {36'b0, len});
    axil_write(16'd88,  {58'b0, MY_PID});
    axil_write(16'd104, 64'd0);          // no fence: the landing is the check
    axil_write(16'd96,  64'd1);
endtask

// ---- test ----------------------------------------------------------------
initial begin
    axi_ctrl.awvalid = 0; axi_ctrl.wvalid = 0; axi_ctrl.arvalid = 0;
    axi_ctrl.bready  = 1; axi_ctrl.rready = 1; axi_ctrl.wstrb = 8'hFF;
    repeat (20) @(posedge aclk);
    aresetn = 1;
    repeat (20) @(posedge aclk);

    // rdma route, so the engine emits a header then payload and the loopback
    // carries it to loom_rx, which writes under RX_PID
    program_window(WIN, 1'b1, MY_PID, {16'b0, DST_VA}, 64'h100_0000);
    axil_write(16'd128, {16'b0, DST_VA});     // RDMA_STAGING_VA (word 16)
    axil_write(16'd168, {58'b0, MY_PID});     // RX_PID (word 21)

    issue_copy(WIN, 28'h0, COPY_B);

    // The deadlock this TB exists for shows up as nothing ever arriving.
    fork begin
        int guard = 0;
        while (host_writes == 0 && guard < 200000) begin
            @(posedge aclk); guard++;
        end
        if (host_writes == 0)
            $display("       no host write in %0d cycles - engine and rx deadlocked on the arbiter", guard);
    end join

    repeat (40000) @(posedge aclk);

    check(host_writes > 0, "the loopback produced a host write");
    if (host_writes > 0) begin
        bit ok = 1;
        for (int i = 0; i < COPY_B/8; i++)
            if (!mem.exists((DST_VA + 48'(i*8)) >> 3) ||
                mem[(DST_VA + 48'(i*8)) >> 3] != src_word(i)) begin
                if (ok) $display("       first bad word %0d", i);
                ok = 0;
            end
        check(ok, "payload landed at the header's target, intact");
    end

    // A spanning message: 16 PMTU packets under one header, so the
    // arbiter has to hand back and forth while loom_rx ignores 15
    // intermediate tlasts and ends where the header said.
    begin
        int w0 = host_writes;
        bit ok = 1;
        issue_copy(WIN, 28'h10000, 28'd65536);
        repeat (200000) @(posedge aclk);
        check(host_writes > w0, "spanning message produced a host write");
        for (int i = 0; i < 65536/8; i++)
            if (!mem.exists((DST_VA + 48'h10000 + 48'(i*8)) >> 3) ||
                mem[(DST_VA + 48'h10000 + 48'(i*8)) >> 3] != src_word(i)) begin
                if (ok) $display("       first bad word %0d", i);
                ok = 0;
            end
        check(ok, "64 KB spanning 16 packets lands intact");
        check(host_writes - w0 == 1,
              $sformatf("one host write per MESSAGE, not per packet (%0d)",
                        host_writes - w0));
    end

    // Back to back, to prove the arbiter releases and re-acquires repeatedly
    begin
        int w0 = host_writes;
        issue_copy(WIN, 28'h20000, 28'd4096);
        issue_copy(WIN, 28'h21000, 28'd4096);
        repeat (100000) @(posedge aclk);
        check(host_writes - w0 == 2,
              $sformatf("two back-to-back descriptors both landed (%0d)",
                        host_writes - w0));
    end

    if (errors == 0) $display("TB PASS (tb_loom_mnw)");
    else             $display("TB FAIL (tb_loom_mnw): %0d errors", errors);
    $finish;
end

initial begin
    #50ms;
    $display("TB FAIL (tb_loom_mnw): timeout");
    $finish;
end

endmodule
