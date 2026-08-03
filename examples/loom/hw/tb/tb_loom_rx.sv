`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_loom_rx — block test for the receive-side message parser.
 * Mocks the incoming request (rq_wr fields), the message payload stream,
 * and the shared write path (wr_ready / output stream + grant).
 *
 * Wire-message format under test (see loom_rx.sv header):
 *   header beat: lane0 = {len[27:0], op[7:0]}, lane1 = target VA,
 *                lane2 = inline data (op 2)
 *   op 1 WRITE: payload beats follow; op 2 WRITE_INLINE: single beat.
 */
module tb_loom_rx;

logic aclk = 0;
logic aresetn = 0;
always #2 aclk = ~aclk;

req_t rq_req;
logic rq_valid = 0, rq_ready;

req_t wr_req;
logic wr_valid;
logic wr_ready = 1;

logic [AXI_DATA_BITS-1:0]   s_tdata = 0;
logic [AXI_DATA_BITS/8-1:0] s_tkeep = 0;
logic s_tvalid = 0, s_tready, s_tlast = 0;

logic [AXI_DATA_BITS-1:0]   m_tdata;
logic [AXI_DATA_BITS/8-1:0] m_tkeep;
logic m_tvalid, m_tlast;
logic m_tready = 1;

logic req, grant = 0, busy, cnt_rx_fwd;

int errors = 0;
int fwd_pulses = 0;

localparam [7:0] OP_WR = 8'd1, OP_INL = 8'd2;
localparam [47:0] STAGING = 48'h7f00_0000_0000;
localparam [47:0] TARGET  = 48'h7f9e_8860_0000;

loom_rx dut (
    .aclk(aclk), .aresetn(aresetn),
    .rq_req(rq_req), .rq_valid(rq_valid), .rq_ready(rq_ready),
    .rdma_staging_va(STAGING),
    .wr_req(wr_req), .wr_valid(wr_valid), .wr_ready(wr_ready),
    .s_tdata(s_tdata), .s_tkeep(s_tkeep), .s_tvalid(s_tvalid),
    .s_tready(s_tready), .s_tlast(s_tlast),
    .m_tdata(m_tdata), .m_tkeep(m_tkeep), .m_tvalid(m_tvalid),
    .m_tready(m_tready), .m_tlast(m_tlast),
    .req(req), .grant(grant), .busy(busy),
    .cnt_rx_fwd(cnt_rx_fwd)
);

req_t wrq[$];
typedef struct { logic [63:0] data; logic [7:0] keep8; logic last; } beat_t;
beat_t outq[$];

always @(posedge aclk) begin
    if (wr_valid && wr_ready) wrq.push_back(wr_req);
    if (m_tvalid && m_tready)
        outq.push_back('{data: m_tdata[63:0], keep8: m_tkeep[7:0], last: m_tlast});
    if (cnt_rx_fwd) fwd_pulses++;
end

task check(input bit cond, input string msg);
    if (!cond) begin
        errors++;
        $display("FAIL: %s", msg);
    end
endtask

task incoming(input [PID_BITS-1:0] pid, input [27:0] wire_len,
              input [47:0] vaddr = STAGING);
    @(negedge aclk);
    rq_req = '0;
    rq_req.pid = pid; rq_req.vaddr = {16'b0, vaddr}; rq_req.len = wire_len;
    rq_valid = 1;
    do @(posedge aclk); while (!rq_ready);
    @(negedge aclk);
    rq_valid = 0;
endtask

// One message beat with explicit lanes
task send_msg_beat(input [63:0] q0, input [63:0] q1, input [63:0] q2,
                   input last);
    @(negedge aclk);
    s_tdata = '0;
    s_tdata[63:0]    = q0;
    s_tdata[127:64]  = q1;
    s_tdata[191:128] = q2;
    s_tkeep  = {64{1'b1}};
    s_tvalid = 1;
    s_tlast  = last;
    do @(posedge aclk); while (!s_tready);
    @(negedge aclk);
    s_tvalid = 0; s_tlast = 0;
endtask

task wait_idle();
    do @(posedge aclk); while (busy);
    repeat (2) @(posedge aclk);
endtask

req_t r;

initial begin
    rq_req = '0;

    repeat (5) @(negedge aclk);
    aresetn = 1;
    repeat (2) @(negedge aclk);

    // --- 1. No grant, no acceptance ---
    @(negedge aclk); rq_valid = 1; rq_req.pid = 6'd0;
    rq_req.vaddr = STAGING; rq_req.len = 64;
    repeat (10) @(posedge aclk);
    check(req && !rq_ready && !busy, "must request but not accept without grant");
    @(negedge aclk); rq_valid = 0;

    // --- 2. Inline store message: exact 8 B write, no clobber ---
    @(negedge aclk); grant = 1;
    incoming(6'd2, 28'd64);
    send_msg_beat({28'b0, 28'd8, OP_INL}, {16'b0, TARGET + 48'h40},
                  64'hEE00_0000_0000_0001, 1'b1);
    wait_idle();
    check(wrq.size() == 1, "inline: one wr_req");
    if (wrq.size() > 0) begin
        r = wrq.pop_front();
        check(r.opcode == LOCAL_WRITE && r.strm == STRM_HOST && r.pid == 6'd2 &&
              r.vaddr == TARGET + 48'h40 && r.len == 8 && r.last,
              "inline: exact 8 B write at header target");
    end
    check(outq.size() == 1 && outq[0].data == 64'hEE00_0000_0000_0001 &&
          outq[0].keep8 == 8'hFF && outq[0].last,
          "inline: data moved to lane 0, 8 B keep");
    outq.delete();
    check(fwd_pulses == 1, "inline: one fwd pulse");

    // --- 3. Bulk message: header + 2 payload beats ---
    incoming(6'd0, 28'd192);
    send_msg_beat({28'b0, 28'd128, OP_WR}, {16'b0, TARGET + 48'h100}, 64'b0, 1'b0);
    send_msg_beat(64'hB0B0_0000, 64'b0, 64'b0, 1'b0);
    send_msg_beat(64'hB0B0_0001, 64'b0, 64'b0, 1'b1);
    wait_idle();
    check(wrq.size() == 1, "bulk: one wr_req");
    if (wrq.size() > 0) begin
        r = wrq.pop_front();
        check(r.pid == 6'd0 && r.vaddr == TARGET + 48'h100 && r.len == 128,
              "bulk: header-described write");
    end
    check(outq.size() == 2 && outq[0].data == 64'hB0B0_0000 && !outq[0].last &&
          outq[1].data == 64'hB0B0_0001 && outq[1].last,
          "bulk: payload beats forwarded, header stripped");
    outq.delete();
    check(fwd_pulses == 2, "bulk: fwd pulse");

    // --- 4. Backpressure: inline data held until output ready ---
    @(negedge aclk); m_tready = 0;
    incoming(6'd1, 28'd64);
    send_msg_beat({28'b0, 28'd8, OP_INL}, {16'b0, TARGET + 48'h200},
                  64'hDD01, 1'b1);
    repeat (15) @(posedge aclk);
    check(outq.size() == 0 && busy, "backpressure: inline beat held");
    @(negedge aclk); m_tready = 1;
    wait_idle();
    check(outq.size() == 1 && outq[0].data == 64'hDD01,
          "backpressure: inline beat after release");
    outq.delete(); wrq.delete();

    // --- 5. Unknown op: drained, nothing written ---
    incoming(6'd0, 28'd128);
    send_msg_beat({28'b0, 28'd8, 8'd9}, {16'b0, TARGET}, 64'hBAD0, 1'b0);
    send_msg_beat(64'hBAD1, 64'b0, 64'b0, 1'b1);
    wait_idle();
    check(wrq.size() == 0 && outq.size() == 0, "unknown op: no write, drained");

    // --- 5b. DIRECT write (vaddr != staging): verbatim forward ---
    incoming(6'd3, 28'd128, TARGET + 48'h300);
    send_msg_beat(64'hD1D1_0000, 64'hAAAA, 64'hBBBB, 1'b0);
    send_msg_beat(64'hD1D1_0001, 64'b0, 64'b0, 1'b1);
    wait_idle();
    check(wrq.size() == 1, "direct: one wr_req");
    if (wrq.size() > 0) begin
        r = wrq.pop_front();
        check(r.pid == 6'd3 && r.vaddr == TARGET + 48'h300 && r.len == 128,
              "direct: request forwarded verbatim");
    end
    check(outq.size() == 2 && outq[0].data == 64'hD1D1_0000 && !outq[0].last &&
          outq[1].data == 64'hD1D1_0001 && outq[1].last,
          "direct: ALL beats forwarded incl. the first");
    outq.delete();

    // --- 6. Back-to-back inline messages ---
    incoming(6'd0, 28'd64);
    send_msg_beat({28'b0, 28'd8, OP_INL}, {16'b0, TARGET}, 64'h1111, 1'b1);
    wait_idle();
    incoming(6'd1, 28'd64);
    send_msg_beat({28'b0, 28'd8, OP_INL}, {16'b0, TARGET + 48'h1000}, 64'h2222, 1'b1);
    wait_idle();
    check(wrq.size() == 2 && wrq[0].pid == 6'd0 && wrq[1].pid == 6'd1 &&
          wrq[1].vaddr == TARGET + 48'h1000, "back-to-back wr_reqs");
    check(outq.size() == 2 && outq[0].data == 64'h1111 && outq[1].data == 64'h2222,
          "back-to-back beats");

    if (errors == 0) $display("TB PASS (tb_loom_rx)");
    else             $display("TB FAIL (tb_loom_rx): %0d errors", errors);
    $finish;
end

initial begin
    #500us;
    $display("TB FAIL (tb_loom_rx): timeout");
    $finish;
end

endmodule
