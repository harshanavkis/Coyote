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

logic req, grant = 0, busy, cnt_rx_fwd, cnt_rx_drop;

int errors = 0;
int fwd_pulses = 0;
int gen = 0;        // presenter generation; bumped on every dut_reset
int drop_pulses = 0;

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
    .cnt_rx_fwd(cnt_rx_fwd), .cnt_rx_drop(cnt_rx_drop)
);

req_t wrq[$];
typedef struct { logic [63:0] data; logic [7:0] keep8; logic last; } beat_t;
beat_t outq[$];

always @(posedge aclk) begin
    if (wr_valid && wr_ready) begin
        wrq.push_back(wr_req);
        // The header target reaches the shell TLB unchecked, so a write
        // into page 0 is exactly the fault that oopsed the driver on
        // hardware (tlb_put_user_pages_ctid, "virtual address 0")
        if (wr_req.vaddr < 48'h1000) begin
            errors++;
            $display("FAIL: wr_req into page 0 (vaddr 0x%0h, len %0d)",
                     wr_req.vaddr, wr_req.len);
        end
    end
    if (m_tvalid && m_tready)
        outq.push_back('{data: m_tdata[63:0], keep8: m_tkeep[7:0], last: m_tlast});
    if (cnt_rx_fwd) fwd_pulses++;
    if (cnt_rx_drop) drop_pulses++;
end

// Print what the DUT actually issued: a framing bug shows up as the wrong
// number of writes or the wrong target, and the count alone is ambiguous
task dump_wrq(input string tag);
    $display("  [%s] %0d wr_req(s), %0d beat(s) still queued",
             tag, wrq.size(), sq.size());
    for (int i = 0; i < wrq.size(); i++)
        $display("    wr_req[%0d] pid %0d vaddr 0x%0h len %0d last %0d",
                 i, wrq[i].pid, wrq[i].vaddr, wrq[i].len, wrq[i].last);
endtask

task check(input bit cond, input string msg);
    if (!cond) begin
        errors++;
        $display("FAIL: %s", msg);
    end
endtask

task incoming(input [PID_BITS-1:0] pid, input [27:0] wire_len,
              input [47:0] vaddr = STAGING, input bit last_f = 1'b1);
    @(negedge aclk);
    rq_req = '0;
    rq_req.pid = pid; rq_req.vaddr = {16'b0, vaddr}; rq_req.len = wire_len;
    rq_req.last = last_f;
    rq_valid = 1;
    do @(posedge aclk); while (!rq_ready);
    @(negedge aclk);
    rq_valid = 0;
endtask

// Payload source. Beats QUEUE, and the head is held on the wire until the
// DUT takes it - the shell's rrsp stream behaves that way, and it is the
// whole point: a beat the DUT never consumes stays at the head and is what
// the next transaction parses. Sole driver of s_t*.
typedef struct { logic [63:0] q0, q1, q2; bit last; } sbeat_t;
sbeat_t sq[$];

initial forever begin
    if (sq.size() == 0) begin
        @(negedge aclk);
        s_tvalid = 0; s_tlast = 0;
    end else begin
        @(negedge aclk);
        s_tdata = '0;
        s_tdata[63:0]    = sq[0].q0;
        s_tdata[127:64]  = sq[0].q1;
        s_tdata[191:128] = sq[0].q2;
        s_tkeep  = {64{1'b1}};
        s_tvalid = 1;
        s_tlast  = sq[0].last;
        @(posedge aclk);
        if (s_tready) void'(sq.pop_front());
    end
end

// One message beat with explicit lanes
task send_msg_beat(input [63:0] q0, input [63:0] q1, input [63:0] q2,
                   input last);
    sq.push_back('{q0, q1, q2, last});
    @(negedge aclk);
endtask

// Wait for the queued beats to be taken; leftovers are a framing bug, so
// report rather than hang
task drain_beats(input int cycles, input string what);
    int n;
    n = 0;
    while (sq.size() > 0 && n < cycles) begin @(posedge aclk); n++; end
    if (sq.size() > 0)
        check(1'b0, {"beats left unconsumed on the stream: ", what});
endtask

task wait_idle();
    do @(posedge aclk); while (busy);
    repeat (2) @(posedge aclk);
endtask

// Bounded wait_idle: a framing bug parks the DUT in a stream state waiting
// for beats that already went by, and a plain wait_idle would surface that
// only as the global timeout, with no indication of which case wedged
task wait_idle_to(input int cycles, input string what);
    int n;
    n = 0;
    while (busy && n < cycles) begin @(posedge aclk); n++; end
    if (busy) check(1'b0, {"stuck, never returned to idle: ", what});
    repeat (2) @(posedge aclk);
endtask

// Quiesced = idle, no beat left on the stream, no request pending, and it
// stays that way. Plain "not busy" is useless here: the DUT drops out of
// busy for a single cycle between two pipelined transactions, and sampling
// there reads the sequence as finished when it has barely started
task wait_quiet(input int cycles, input string what);
    int n, settled;
    n = 0; settled = 0;
    while (n < cycles && settled < 8) begin
        @(posedge aclk);
        n++;
        if (!busy && sq.size() == 0 && !rq_valid) settled++;
        else settled = 0;
    end
    if (settled < 8)
        check(1'b0, {"never quiesced (stuck mid-transaction): ", what});
endtask

// Present the next request while the DUT is still mid-transaction - what
// the shell does, since rq_wr.valid rises for the next packet while the
// current payload is still moving. Serialized behind the previous
// presenter so only one process ever drives rq_valid.
task present_pending(input [PID_BITS-1:0] pid, input [27:0] wire_len,
                     input [47:0] vaddr, input bit last_f = 1'b1);
    int g;
    g = gen;
    fork
        begin
            wait (busy);
            repeat (2) @(negedge aclk);   // previous presenter has retired
            // a presenter left over from a wedged case must never drive
            if (g == gen) incoming(pid, wire_len, vaddr, last_f);
        end
    join_none
    wait (rq_valid || g != gen);          // on the wire before we go on
endtask

// Framing cases must not inherit a wedged DUT from the case before them.
// Bumping gen retires any presenter still parked in the previous case
// (disable fork would take this process down with it under xsim)
task dut_reset();
    gen++;
    @(negedge aclk);
    aresetn = 0; rq_valid = 0; m_tready = 1;
    sq.delete();
    repeat (4) @(negedge aclk);
    aresetn = 1;
    repeat (2) @(negedge aclk);
    wrq.delete(); outq.delete();
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

    // ---------------------------------------------------------------------
    // Framing cases. Cases 1-6 above hand the DUT one request at a time,
    // always with a tlast-terminated payload, always fully drained before
    // the next request exists. The shell does neither:
    //   - rq_wr.last is LOW whenever the stream ends WITHOUT a tlast (the
    //     field's own comment in lynx_pkg.sv); ib_transport_protocol emits
    //     that for every RDMA_WRITE_FIRST/MIDDLE fragment;
    //   - the next rq_wr is valid while the previous payload still moves.
    // Request/payload framing is the only thing standing between the wire
    // and an unvalidated LOCAL_WRITE, so it decides whether a header target
    // can ever be garbage. The monitor additionally fails any write into
    // page 0 - the fault that took the driver down on hardware.
    // ---------------------------------------------------------------------

    // --- 7. Next request pending while the previous payload streams ---
    dut_reset();
    incoming(6'd4, 28'd192, TARGET + 48'h400);
    send_msg_beat(64'hC0C0_0000, 64'b0, 64'b0, 1'b0);
    present_pending(6'd5, 28'd64, STAGING);
    send_msg_beat(64'hC0C0_0001, 64'b0, 64'b0, 1'b0);
    send_msg_beat(64'hC0C0_0002, 64'b0, 64'b0, 1'b1);
    send_msg_beat({28'b0, 28'd8, OP_INL}, {16'b0, TARGET + 48'h500},
                  64'h7777, 1'b1);
    wait_quiet(400, "7: request pending across a direct write");
    dump_wrq("case 7");
    check(wrq.size() == 2, "pipelined: two wr_reqs");
    if (wrq.size() == 2)
        check(wrq[0].vaddr == TARGET + 48'h400 && wrq[0].len == 192 &&
              wrq[1].vaddr == TARGET + 48'h500 && wrq[1].len == 8,
              "pipelined: message target survives a preceding direct write");

    // --- 8. The 6.2a ordering sequence: bulk, bulk, then the flag store ---
    // Same shape as loom_host's ordering test, which is where hardware
    // stopped landing writes
    dut_reset();
    incoming(6'd1, 28'd128, TARGET + 48'h10000);
    send_msg_beat(64'hBEEF_0000, 64'b0, 64'b0, 1'b0);
    present_pending(6'd1, 28'd128, TARGET + 48'h20000);
    send_msg_beat(64'hBEEF_0001, 64'b0, 64'b0, 1'b1);
    send_msg_beat(64'hBEEF_0002, 64'b0, 64'b0, 1'b0);
    present_pending(6'd1, 28'd64, STAGING);
    send_msg_beat(64'hBEEF_0003, 64'b0, 64'b0, 1'b1);
    send_msg_beat({28'b0, 28'd8, OP_INL}, {16'b0, TARGET + 48'h800},
                  64'hF1A6, 1'b1);
    wait_quiet(400, "8: bulk, bulk, flag store");
    dump_wrq("case 8");
    check(wrq.size() == 3, "6.2a sequence: three wr_reqs");
    if (wrq.size() == 3)
        check(wrq[2].vaddr == TARGET + 48'h800 && wrq[2].len == 8,
              "6.2a sequence: flag store lands at base+0x800");

    // --- 9. Request whose stream carries no tlast (rq_wr.last = 0) ---
    // The transaction must end after the request's own len; ST_STREAM
    // instead waits for a tlast that belongs to the NEXT message
    dut_reset();
    incoming(6'd2, 28'd128, TARGET + 48'h600, 1'b0);
    send_msg_beat(64'hA5A5_0000, 64'b0, 64'b0, 1'b0);
    send_msg_beat(64'hA5A5_0001, 64'b0, 64'b0, 1'b0);   // len reached, no tlast
    present_pending(6'd2, 28'd64, STAGING);
    send_msg_beat({28'b0, 28'd8, OP_INL}, {16'b0, TARGET + 48'h700},
                  64'h5555, 1'b1);
    wait_quiet(400, "9: request with last=0");
    dump_wrq("case 9");
    check(wrq.size() == 2, "no-tlast: transaction ends after its own len");
    if (wrq.size() == 2)
        check(wrq[1].vaddr == TARGET + 48'h700 && wrq[1].len == 8,
              "no-tlast: next message parsed from its own header");

    // --- 10. Inline message that is not a single beat ---
    // l_hdr_last latches whether the header beat was the last one and is
    // never read, so a trailing beat stays in the stream and becomes the
    // next message's header. Lane 0 of the trailer here looks like an
    // inline op and lane 1 like a small VA: that is how a host write ends
    // up pointed into page 0
    dut_reset();
    incoming(6'd3, 28'd128, STAGING);
    send_msg_beat({28'b0, 28'd8, OP_INL}, {16'b0, TARGET + 48'h900},
                  64'h9999, 1'b0);
    send_msg_beat({28'b0, 28'd8, OP_INL}, 64'h800, 64'hDEAD, 1'b1);
    wait_quiet(200, "10: multi-beat inline message");
    drain_beats(50, "10: trailing beat of a multi-beat inline message");
    incoming(6'd3, 28'd64, STAGING);
    send_msg_beat({28'b0, 28'd8, OP_INL}, {16'b0, TARGET + 48'hA00},
                  64'hAAAA, 1'b1);
    wait_quiet(300, "10: message after a multi-beat one");
    dump_wrq("case 10");
    check(wrq.size() == 2, "multi-beat inline: one write per message");
    if (wrq.size() == 2)
        check(wrq[0].vaddr == TARGET + 48'h900 &&
              wrq[1].vaddr == TARGET + 48'hA00,
              "multi-beat inline: trailer drained, next header parsed clean");

    // --- 11. Headers that do not match the contract are never translated ---
    // A beat that is not a header must not become a write: whatever sits in
    // lane 1 would go to the shell TLB under the exporter's pid. Each of
    // these is one field away from a legal inline header
    dut_reset();
    drop_pulses = 0;
    incoming(6'd0, 28'd64);
    send_msg_beat({28'b0, 28'd16, OP_INL}, {16'b0, TARGET}, 64'h1, 1'b1);
    wait_quiet(200, "11: inline with a length other than 8");
    incoming(6'd0, 28'd64);
    send_msg_beat({28'b0, 28'd8, OP_INL}, {16'b0, TARGET + 48'h4}, 64'h2, 1'b1);
    wait_quiet(200, "11: inline with an unaligned target");
    incoming(6'd0, 28'd64);
    send_msg_beat({28'hF, 28'd8, OP_INL}, {16'b0, TARGET}, 64'h3, 1'b1);
    wait_quiet(200, "11: reserved field set");
    incoming(6'd0, 28'd64);
    send_msg_beat({28'b0, 28'd100, OP_WR}, {16'b0, TARGET}, 64'h4, 1'b1);
    wait_quiet(200, "11: bulk length not a 64 B multiple");
    incoming(6'd0, 28'd64);        // a payload beat where a header belongs
    send_msg_beat(64'h5A5A_0000_0000_0018, 64'h5A5A_0000_0000_0019,
                  64'h5A5A_0000_0000_001A, 1'b1);
    wait_quiet(200, "11: payload beat parsed as a header");
    dump_wrq("case 11");
    check(wrq.size() == 0, "malformed headers: nothing written");
    check(drop_pulses == 5, "malformed headers: every one counted");

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
