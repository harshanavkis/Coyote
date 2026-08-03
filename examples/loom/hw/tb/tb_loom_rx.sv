`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_loom_rx — block test for the receive-side forwarder.
 * Mocks the incoming request (rq_wr fields), the payload stream, and the
 * shared write path (wr_ready / output stream + grant).
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

loom_rx dut (
    .aclk(aclk), .aresetn(aresetn),
    .rq_req(rq_req), .rq_valid(rq_valid), .rq_ready(rq_ready),
    .wr_req(wr_req), .wr_valid(wr_valid), .wr_ready(wr_ready),
    .s_tdata(s_tdata), .s_tkeep(s_tkeep), .s_tvalid(s_tvalid),
    .s_tready(s_tready), .s_tlast(s_tlast),
    .m_tdata(m_tdata), .m_tkeep(m_tkeep), .m_tvalid(m_tvalid),
    .m_tready(m_tready), .m_tlast(m_tlast),
    .req(req), .grant(grant), .busy(busy),
    .cnt_rx_fwd(cnt_rx_fwd)
);

req_t wrq[$];
typedef struct { logic [63:0] data; logic last; } beat_t;
beat_t outq[$];

always @(posedge aclk) begin
    if (wr_valid && wr_ready) wrq.push_back(wr_req);
    if (m_tvalid && m_tready) outq.push_back('{data: m_tdata[63:0], last: m_tlast});
    if (cnt_rx_fwd) fwd_pulses++;
end

task check(input bit cond, input string msg);
    if (!cond) begin
        errors++;
        $display("FAIL: %s", msg);
    end
endtask

task incoming(input [PID_BITS-1:0] pid, input [47:0] vaddr, input [27:0] len);
    @(negedge aclk);
    rq_req = '0;
    rq_req.pid = pid; rq_req.vaddr = vaddr; rq_req.len = len;
    rq_valid = 1;
    do @(posedge aclk); while (!rq_ready);
    @(negedge aclk);
    rq_valid = 0;
endtask

task send_beats(input int n, input [63:0] base_val);
    for (int i = 0; i < n; i++) begin
        @(negedge aclk);
        s_tdata  = {448'b0, base_val + 64'(i)};
        s_tkeep  = {64{1'b1}};
        s_tvalid = 1;
        s_tlast  = (i == n-1);
        do @(posedge aclk); while (!s_tready);
        @(negedge aclk);
        s_tvalid = 0; s_tlast = 0;
    end
endtask

task wait_idle();
    do @(posedge aclk); while (busy);
    repeat (2) @(posedge aclk);
endtask

localparam [47:0] BUF_C = 48'h7f9e_8860_0000;
req_t r;

initial begin
    rq_req = '0;

    repeat (5) @(negedge aclk);
    aresetn = 1;
    repeat (2) @(negedge aclk);

    // --- 1. No grant, no acceptance ---
    @(negedge aclk); rq_valid = 1; rq_req.pid = 6'd0; rq_req.vaddr = BUF_C; rq_req.len = 128;
    repeat (10) @(posedge aclk);
    check(req && !rq_ready && !busy, "must request but not accept without grant");
    @(negedge aclk); rq_valid = 0;

    // --- 2. Granted single write, 2 beats ---
    @(negedge aclk); grant = 1;
    incoming(6'd0, BUF_C + 48'h40, 28'd128);
    fork send_beats(2, 64'hEE00); join_none
    wait_idle();
    check(wrq.size() == 1, "one wr_req");
    if (wrq.size() > 0) begin
        r = wrq.pop_front();
        check(r.opcode == LOCAL_WRITE && r.strm == STRM_HOST && r.pid == 6'd0 &&
              r.vaddr == BUF_C + 48'h40 && r.len == 128 && r.last,
              "wr_req fields");
    end
    check(outq.size() == 2 && outq[0].data == 64'hEE00 && !outq[0].last &&
          outq[1].data == 64'hEE01 && outq[1].last, "forwarded beats");
    outq.delete();
    check(fwd_pulses == 1, "one rx_fwd pulse");

    // --- 3. Backpressure on the output stalls the input ---
    @(negedge aclk); m_tready = 0;
    incoming(6'd1, BUF_C + 48'h100, 28'd64);
    @(negedge aclk);
    s_tdata = {448'b0, 64'hDD00}; s_tkeep = {64{1'b1}}; s_tvalid = 1; s_tlast = 1;
    repeat (10) @(posedge aclk);
    check(!s_tready && outq.size() == 0, "stalled under backpressure");
    @(negedge aclk); m_tready = 1;
    do @(posedge aclk); while (!s_tready);
    @(negedge aclk); s_tvalid = 0; s_tlast = 0;
    wait_idle();
    check(outq.size() == 1 && outq[0].data == 64'hDD00 && outq[0].last,
          "beat delivered after backpressure released");
    outq.delete(); wrq.delete();

    // --- 4. Back-to-back requests ---
    incoming(6'd0, BUF_C, 28'd64);
    fork send_beats(1, 64'h1111); join_none
    wait_idle();
    incoming(6'd1, BUF_C + 48'h1000, 28'd64);
    fork send_beats(1, 64'h2222); join_none
    wait_idle();
    check(wrq.size() == 2 && wrq[0].pid == 6'd0 && wrq[1].pid == 6'd1 &&
          wrq[1].vaddr == BUF_C + 48'h1000, "back-to-back wr_reqs");
    check(outq.size() == 2 && outq[0].data == 64'h1111 && outq[1].data == 64'h2222,
          "back-to-back beats");
    check(fwd_pulses == 4, "four rx_fwd pulses total");

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
