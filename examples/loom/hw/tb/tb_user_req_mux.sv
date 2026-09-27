`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_user_req_mux - the shell's request demux, both variants.
 *
 * user_req_mux takes the vFPGA's single sq_wr and splits it by strm into the
 * shell's host-DMA path and its RoCE path. With QDEPTH = 0 (the structure it
 * has always had) there is one register in front of a combinational demux, so
 * the request at the head owns the port until its own path takes it: a remote
 * write waiting for the wire stalls a local write behind it. That is the
 * head-of-line blocking that halves bidirectional bandwidth.
 *
 * T1 DEMONSTRATES the blocking on QDEPTH = 0 (it is a property of the code,
 * not a bug to fix in place - stock Coyote behaves this way).
 * T2 asserts it is GONE with QDEPTH = 16: with the remote branch refusing
 *    everything, local writes still flow.
 * T3 order within each path, and nothing lost or duplicated, under random
 *    backpressure on both branches.
 * T4 the remote branch's payload lands in req_2, as the shell expects.
 */
module tb_user_req_mux;

logic aclk = 0;
logic aresetn = 0;
always #2 aclk = ~aclk;

int errors = 0;
task check(input bit cond, input string msg);
    if (!cond) begin errors++; $display("FAIL: %s", msg); end
endtask

// ---- one instance of each variant, driven in parallel ----
metaIntf #(.STYPE(req_t)) sq_rd_0 (.*), sq_wr_0 (.*), l_rd_0 (.*), l_wr_0 (.*);
metaIntf #(.STYPE(dreq_t)) r_rd_0 (.*), r_wr_0 (.*);
metaIntf #(.STYPE(req_t)) sq_rd_q (.*), sq_wr_q (.*), l_rd_q (.*), l_wr_q (.*);
metaIntf #(.STYPE(dreq_t)) r_rd_q (.*), r_wr_q (.*);

user_req_mux #(.ID_REG(0), .QDEPTH(0)) dut_0 (
    .user_sq_rd(sq_rd_0), .user_sq_wr(sq_wr_0),
    .user_local_rd(l_rd_0), .user_local_wr(l_wr_0),
    .user_remote_rd(r_rd_0), .user_remote_wr(r_wr_0),
    .aclk(aclk), .aresetn(aresetn)
);
user_req_mux #(.ID_REG(0), .QDEPTH(16)) dut_q (
    .user_sq_rd(sq_rd_q), .user_sq_wr(sq_wr_q),
    .user_local_rd(l_rd_q), .user_local_wr(l_wr_q),
    .user_remote_rd(r_rd_q), .user_remote_wr(r_wr_q),
    .aclk(aclk), .aresetn(aresetn)
);

// ---- sinks ----
int n_local_0, n_local_q, n_remote_0, n_remote_q;
logic [47:0] local_va_q [$];
logic [47:0] remote_va_q [$];
always @(posedge aclk) if (aresetn) begin
    if (l_wr_0.valid && l_wr_0.ready) n_local_0++;
    if (r_wr_0.valid && r_wr_0.ready) n_remote_0++;
    if (l_wr_q.valid && l_wr_q.ready) begin n_local_q++;  local_va_q.push_back(l_wr_q.data.vaddr[47:0]); end
    if (r_wr_q.valid && r_wr_q.ready) begin n_remote_q++; remote_va_q.push_back(r_wr_q.data.req_2.vaddr[47:0]); end
end

// ---- drivers: one per DUT, each presenting a request and HOLDING it until
// that DUT takes it. That is what a producer must do (a presented request
// cannot be withdrawn), and it is what makes head-of-line blocking visible:
// a request the shell will not take keeps the ones behind it waiting.
req_t que_0 [$], que_q [$];

initial forever begin
    @(negedge aclk);
    if (aresetn && que_0.size() > 0) begin
        sq_wr_0.data = que_0[0]; sq_wr_0.valid = 1;
        do @(posedge aclk); while (!sq_wr_0.ready);
        @(negedge aclk); sq_wr_0.valid = 0;
        void'(que_0.pop_front());
    end
end

initial forever begin
    @(negedge aclk);
    if (aresetn && que_q.size() > 0) begin
        sq_wr_q.data = que_q[0]; sq_wr_q.valid = 1;
        do @(posedge aclk); while (!sq_wr_q.ready);
        @(negedge aclk); sq_wr_q.valid = 0;
        void'(que_q.pop_front());
    end
end

function automatic req_t mk(input bit remote, input [47:0] va);
    req_t r;
    r = '0;
    r.opcode = 1;
    r.strm   = remote ? STRM_RDMA : STRM_HOST;
    r.vaddr  = {16'b0, va};
    r.len    = 4096;
    return r;
endfunction

// queue the same request at both DUTs
task automatic post(input bit remote, input [47:0] va);
    que_0.push_back(mk(remote, va));
    que_q.push_back(mk(remote, va));
    @(negedge aclk);
endtask

task automatic settle(input int cycles);
    repeat (cycles) @(posedge aclk);
endtask

initial begin
    sq_rd_0.valid = 0; sq_wr_0.valid = 0; sq_rd_q.valid = 0; sq_wr_q.valid = 0;
    sq_rd_0.data = '0; sq_wr_0.data = '0; sq_rd_q.data = '0; sq_wr_q.data = '0;
    l_rd_0.ready = 1; l_wr_0.ready = 1; r_rd_0.ready = 1; r_wr_0.ready = 1;
    l_rd_q.ready = 1; l_wr_q.ready = 1; r_rd_q.ready = 1; r_wr_q.ready = 1;
    n_local_0 = 0; n_local_q = 0; n_remote_0 = 0; n_remote_q = 0;

    repeat (5) @(negedge aclk);
    aresetn = 1;
    repeat (4) @(negedge aclk);

    // --- T0: with the remote path refusing, ONE remote write does not block
    //     anything even on the stock module: it parks in that path's output
    //     register. The tolerance is one request deep, which is why the
    //     blocking only shows up in steady state - and why this test posts
    //     two remote writes in T1.
    @(negedge aclk); r_wr_0.ready = 0; r_wr_q.ready = 0;
    post(1'b1, 48'h0_1000);
    post(1'b0, 48'h0_2000);
    settle(200);
    check(n_local_0 == 1,
          $sformatf("T0 QDEPTH=0: one blocked remote write parks in the output reg, the local one still flows (%0d of 1)", n_local_0));
    check(n_local_q == 1,
          $sformatf("T0 QDEPTH=16: same (%0d of 1)", n_local_q));

    // --- T1/T2: now the steady state. The remote path still refuses, its
    //     output register is already occupied, so a SECOND remote write has
    //     nowhere to go and sits at the head of sq_wr. On the stock module
    //     the four local writes behind it are stuck; with per-path queues
    //     they are not.
    n_local_0 = 0; n_local_q = 0;
    post(1'b1, 48'h1_0000);
    for (int i = 0; i < 4; i++) post(1'b0, 48'h2_0000 + 48'(i * 64));
    settle(400);
    check(n_local_0 == 0,
          $sformatf("T1 QDEPTH=0: %0d local writes got through behind a blocked remote one, expected 0 (head-of-line blocking)", n_local_0));
    check(n_local_q == 4,
          $sformatf("T2 QDEPTH=16: %0d of 4 local writes flowed while the remote path refused", n_local_q));
    check(n_remote_0 == 0 && n_remote_q == 0, "T1/T2: nothing left the refusing remote branch");

    // open the remote path: the queued request must drain, not be lost
    @(negedge aclk); r_wr_0.ready = 1; r_wr_q.ready = 1;
    settle(400);
    check(n_remote_q == 2, $sformatf("T2: both queued remote writes drained once their path opened (%0d of 2)", n_remote_q));
    check(remote_va_q.size() == 2 && remote_va_q[0] == 48'h0_1000 && remote_va_q[1] == 48'h1_0000,
          "T4: the remote branch carries each request in req_2, in order, vaddr preserved");
    check(n_local_0 == 4 && n_remote_0 == 2,
          $sformatf("T1: QDEPTH=0 drains too once unblocked (local %0d of 4, remote %0d of 2)", n_local_0, n_remote_0));

    // --- T3: order within each path, and conservation, under random
    //     backpressure on both branches (QDEPTH = 16).
    n_local_q = 0; n_remote_q = 0; local_va_q.delete(); remote_va_q.delete();
    fork
        begin : bp
            for (int i = 0; i < 6000; i++) begin
                @(negedge aclk);
                l_wr_q.ready = ($urandom_range(0, 3) != 0);
                r_wr_q.ready = ($urandom_range(0, 5) != 0);
            end
        end
        begin : traffic
            for (int i = 0; i < 24; i++) begin
                post(1'b0, 48'h4_0000 + 48'(i * 64));
                post(1'b1, 48'h8_0000 + 48'(i * 64));
            end
            while (que_q.size() > 0) settle(10);
        end
    join_any
    disable fork;
    @(negedge aclk); l_wr_q.ready = 1; r_wr_q.ready = 1;
    settle(400);
    check(n_local_q == 24 && n_remote_q == 24,
          $sformatf("T3: every request came out exactly once (local %0d, remote %0d of 24 each)", n_local_q, n_remote_q));
    begin
        bit ordered = 1;
        foreach (local_va_q[i])  if (local_va_q[i]  != 48'h4_0000 + 48'(i * 64)) ordered = 0;
        foreach (remote_va_q[i]) if (remote_va_q[i] != 48'h8_0000 + 48'(i * 64)) ordered = 0;
        check(ordered, "T3: order preserved within each path");
    end

    if (errors == 0) $display("TB PASS (tb_user_req_mux)");
    else             $display("TB FAIL (tb_user_req_mux): %0d errors", errors);
    $finish;
end

initial begin
    #1ms;
    $display("TB FAIL (tb_user_req_mux): timeout");
    $finish;
end

endmodule
