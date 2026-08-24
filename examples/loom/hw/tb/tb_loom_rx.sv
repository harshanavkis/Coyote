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
logic cnt_rx_move, cnt_rx_starve, cnt_rx_stall;
logic cnt_rx_stall_head, cnt_rx_stall_body, cnt_rx_req, cnt_rx_span;
int req_pulses = 0, span_pulses = 0;
int head_pulses = 0, body_pulses = 0;
int move_pulses = 0, starve_pulses = 0, stall_pulses = 0;

int errors = 0;
int fwd_pulses = 0;
int gen = 0;        // presenter generation; bumped on every dut_reset
int drop_pulses = 0;

localparam [7:0] OP_WR = 8'd1, OP_INL = 8'd2;
localparam [47:0] STAGING = 48'h7f00_0000_0000;
// Incoming writes land under the QP owner's pid, which is a CSR now rather
// than a field of each request - jigsaw's controller uses a configured pid
// the same way, and it is fixed for the life of the connection.
localparam [PID_BITS-1:0] RX_PID = 6'd2;
localparam [47:0] TARGET  = 48'h7f9e_8860_0000;

loom_rx dut (
    .aclk(aclk), .aresetn(aresetn),
    .rq_req(rq_req), .rq_valid(rq_valid), .rq_ready(rq_ready),
    .rdma_staging_va(STAGING), .rx_pid(RX_PID),
    .wr_req(wr_req), .wr_valid(wr_valid), .wr_ready(wr_ready),
    .s_tdata(s_tdata), .s_tkeep(s_tkeep), .s_tvalid(s_tvalid),
    .s_tready(s_tready), .s_tlast(s_tlast),
    .m_tdata(m_tdata), .m_tkeep(m_tkeep), .m_tvalid(m_tvalid),
    .m_tready(m_tready), .m_tlast(m_tlast),
    .req(req), .grant(grant), .busy(busy),
    .cnt_rx_fwd(cnt_rx_fwd), .cnt_rx_drop(cnt_rx_drop),
    .cnt_rx_move(cnt_rx_move), .cnt_rx_starve(cnt_rx_starve),
    .cnt_rx_stall(cnt_rx_stall),
    .cnt_rx_stall_head(cnt_rx_stall_head),
    .cnt_rx_stall_body(cnt_rx_stall_body),
    .cnt_rx_req(cnt_rx_req), .cnt_rx_span(cnt_rx_span)
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
    if (cnt_rx_move) move_pulses++;
    if (cnt_rx_starve) starve_pulses++;
    if (cnt_rx_stall) stall_pulses++;
    if (cnt_rx_stall_head) head_pulses++;
    if (cnt_rx_stall_body) body_pulses++;
    if (cnt_rx_req) req_pulses++;
    if (cnt_rx_span) span_pulses++;
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
    int n;
    n = 0;
    // A transaction is started by a BEAT now, not by a request, so busy can
    // still be low when this is called - waiting on it alone returns before
    // the DUT has even looked at the stream. Wait for the beats to be taken
    // and the transaction to finish.
    while (n < 2000 && (sq.size() > 0 || busy)) begin @(posedge aclk); n++; end
    repeat (4) @(posedge aclk);
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

    // --- 1. Requests are drained, not obeyed; a header beat is what starts
    //     a transaction, and it still needs the grant first ---
    @(negedge aclk); rq_valid = 1; rq_req.pid = 6'd0;
    rq_req.vaddr = STAGING; rq_req.len = 64;
    repeat (10) @(posedge aclk);
    check(rq_ready && !busy && !req,
          "a request alone is drained and starts nothing");
    sq.push_back('{{28'b0, 28'd8, OP_INL}, {16'b0, TARGET}, 64'hAA, 1'b1});
    repeat (4) @(posedge aclk);
    check(req && !busy,
          "a waiting header beat asks for the path but cannot start without grant");
    sq.delete();                    // withdraw it; case 2 supplies its own
    repeat (2) @(posedge aclk);
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
        check(r.opcode == LOCAL_WRITE && r.strm == STRM_HOST && r.pid == RX_PID &&
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
        check(r.pid == RX_PID && r.vaddr == TARGET + 48'h100 && r.len == 128,
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

    // --- 5b. BULK as a WRITE message: header names the target, payload
    //     follows. There is no direct path any more - rq_wr.vaddr never
    //     decides where a DMA lands, so a stray packet cannot name its own
    //     destination. The request is 64 B longer than its payload for the
    //     header it carries.
    incoming(6'd3, 28'd192, TARGET + 48'h300);   // vaddr deliberately ignored
    send_msg_beat({28'b0, 28'd128, OP_WR}, {16'b0, TARGET + 48'h300}, 64'b0, 1'b0);
    send_msg_beat(64'hD1D1_0000, 64'hAAAA, 64'hBBBB, 1'b0);
    send_msg_beat(64'hD1D1_0001, 64'b0, 64'b0, 1'b1);
    wait_idle();
    check(wrq.size() == 1, "bulk message: one wr_req");
    if (wrq.size() > 0) begin
        r = wrq.pop_front();
        check(r.pid == RX_PID && r.vaddr == TARGET + 48'h300 && r.len == 128,
              "bulk message: target and length come from the header");
    end
    check(outq.size() == 2 && outq[0].data == 64'hD1D1_0000 && !outq[0].last &&
          outq[1].data == 64'hD1D1_0001 && outq[1].last,
          "bulk message: payload beats forwarded, header stripped");
    outq.delete();

    // The target really is the header's, not the request's: send them
    // deliberately different and the header must win.
    incoming(6'd3, 28'd128, TARGET + 48'h9999);
    send_msg_beat({28'b0, 28'd64, OP_WR}, {16'b0, TARGET + 48'h700}, 64'b0, 1'b0);
    send_msg_beat(64'hC0C0_0001, 64'b0, 64'b0, 1'b1);
    wait_idle();
    check(wrq.size() == 1, "header wins: one wr_req");
    if (wrq.size() > 0) begin
        r = wrq.pop_front();
        check(r.vaddr == TARGET + 48'h700,
              $sformatf("header wins over rq_wr.vaddr (got %0h)", r.vaddr));
    end
    outq.delete();

    // --- 6. Back-to-back inline messages ---
    incoming(6'd0, 28'd64);
    send_msg_beat({28'b0, 28'd8, OP_INL}, {16'b0, TARGET}, 64'h1111, 1'b1);
    wait_idle();
    incoming(6'd1, 28'd64);
    send_msg_beat({28'b0, 28'd8, OP_INL}, {16'b0, TARGET + 48'h1000}, 64'h2222, 1'b1);
    wait_idle();
    check(wrq.size() == 2 && wrq[0].pid == RX_PID && wrq[1].pid == RX_PID &&
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
    incoming(6'd4, 28'd256, TARGET + 48'h400);
    send_msg_beat({28'b0, 28'd192, OP_WR}, {16'b0, TARGET + 48'h400}, 64'b0, 1'b0);
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
              "pipelined: message target survives a preceding bulk");

    // --- 8. The 6.2a ordering sequence: bulk, bulk, then the flag store ---
    // Same shape as loom_host's ordering test, which is where hardware
    // stopped landing writes
    dut_reset();
    incoming(6'd1, 28'd128, TARGET + 48'h10000);
    send_msg_beat({28'b0, 28'd64, OP_WR}, {16'b0, TARGET + 48'h10000}, 64'b0, 1'b0);
    send_msg_beat(64'hBEEF_0000, 64'b0, 64'b0, 1'b1);
    present_pending(6'd1, 28'd128, TARGET + 48'h20000);
    send_msg_beat({28'b0, 28'd64, OP_WR}, {16'b0, TARGET + 48'h20000}, 64'b0, 1'b0);
    send_msg_beat(64'hBEEF_0002, 64'b0, 64'b0, 1'b1);
    present_pending(6'd1, 28'd64, STAGING);
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
    send_msg_beat({28'b0, 28'd64, OP_WR}, {16'b0, TARGET + 48'h600}, 64'b0, 1'b0);
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
    // A trailer is a malformed sender: an inline message is ONE beat by
    // contract. There is no request framing to absorb it any more - the
    // shape jigsaw uses reads every idle beat as a header - so it must at
    // least be REJECTED rather than honoured. Give it the reserved field
    // set, which is what a payload beat looks like.
    send_msg_beat(64'h5A5A_0000_0000_0018, 64'h800, 64'hDEAD, 1'b1);
    wait_quiet(200, "10: multi-beat inline message");
    drain_beats(50, "10: trailing beat of a multi-beat inline message");
    incoming(6'd3, 28'd64, STAGING);
    send_msg_beat({28'b0, 28'd8, OP_INL}, {16'b0, TARGET + 48'hA00},
                  64'hAAAA, 1'b1);
    wait_quiet(300, "10: message after a multi-beat one");
    dump_wrq("case 10");
    check(wrq.size() == 2, "multi-beat inline: one write per message, trailer rejected");
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

    // --- 12. A header whose length is not a whole number of beats ---
    // loom_engine cannot emit one (rdma bulk with len[5:0] != 0 is dropped
    // at the source) but the RX takes its length from the wire, where
    // nothing enforces it. A sender that disagrees with us must cost one
    // rejected message and NOT the framing of everything after: the beats
    // it already owns get drained, and the next header parses clean.
    dut_reset();
    drop_pulses = 0;
    incoming(6'd6, 28'd192, TARGET + 48'hB00);
    send_msg_beat({28'b0, 28'd100, OP_WR}, {16'b0, TARGET + 48'hB00}, 64'b0, 1'b0);
    // These follow a header that was rejected, so they are read as headers
    // themselves. 0xDEAD_0001 would parse as a VALID op-1 write of
    // 0xDEAD00 bytes to address 0 - the hazard this architecture carries in
    // exchange for having nothing to desynchronise. Use beats that cannot
    // be mistaken for headers, and require every one to be counted.
    send_msg_beat(64'h5A5A_0000_0000_0020, 64'b0, 64'b0, 1'b0);
    send_msg_beat(64'h5A5A_0000_0000_0021, 64'b0, 64'b0, 1'b1);
    wait_quiet(200, "12: header claiming 100 B");
    incoming(6'd6, 28'd64, STAGING);
    send_msg_beat({28'b0, 28'd8, OP_INL}, {16'b0, TARGET + 48'hC00},
                  64'hCC, 1'b1);
    wait_quiet(200, "12: message after a rejected length");
    dump_wrq("case 12");
    check(wrq.size() == 1, "odd length: the bad header wrote nothing");
    if (wrq.size() >= 1)
        check(wrq[0].vaddr == TARGET + 48'hC00,
              "odd length: next header parsed clean after the rejection");
    check(drop_pulses == 3,
          $sformatf("odd length: the header and both stray beats counted (%0d)",
                    drop_pulses));

    // --- 13. last=1 request that delivers MORE beats than its length ---
    // This is the hardware failure. When the shell says it will terminate
    // the stream, tlast IS the boundary and the length is advisory: on the
    // two-host run a 4096 B write did not deliver exactly ceil(4096/64)
    // beats, a length-derived boundary ended the transaction off by a beat,
    // the write we had asked for was never satisfied, sq_wr backed up and
    // this module parked in ST_WR_REQ with rx_fwd frozen
    dut_reset();
    // The header's length is authoritative, so a stream carrying MORE than
    // it claims leaves a stale beat behind. That beat must not become a
    // write: the next parse sees it, fails the contract and counts it, and
    // the message after that lands normally. One message is lost, nothing
    // is written to an address that came from payload.
    drop_pulses = 0;
    incoming(6'd7, 28'd192, TARGET + 48'hD00);
    send_msg_beat({28'b0, 28'd64, OP_WR}, {16'b0, TARGET + 48'hD00}, 64'b0, 1'b0);
    send_msg_beat(64'hFEED_0001, 64'b0, 64'b0, 1'b0);
    send_msg_beat(64'hFEED_0002, 64'b0, 64'b0, 1'b1);   // one beat too many
    wait_quiet(300, "13: last=1 stream longer than its length");
    incoming(6'd7, 28'd64, STAGING);
    send_msg_beat({28'b0, 28'd8, OP_INL}, {16'b0, TARGET + 48'hE00},
                  64'hEE, 1'b1);
    wait_quiet(300, "13: message after an over-long stream");
    dump_wrq("case 13");
    check(wrq.size() == 2, "over-long stream: both writes issued");
    if (wrq.size() == 2)
        check(wrq[0].vaddr == TARGET + 48'hD00 && wrq[0].len == 64 &&
              wrq[1].vaddr == TARGET + 48'hE00,
              "over-long stream: header's write correct, next message clean");
    // Rejected, not drained. There is no request framing to say the beat
    // belonged to the message that did not want it, so it is read as a
    // header, fails the contract and is counted. One rejection, and the
    // message after it still parses clean - which is the property that
    // matters: a malformed sender costs one message, not the framing of
    // everything after.
    check(drop_pulses == 1,
          $sformatf("over-long stream: the extra beat is rejected and counted (%0d)",
                    drop_pulses));

    // --- 13. Receive-path cycle accounting. These three partition every
    //     cycle spent in ST_STREAM and are what attributes the receiver's
    //     ceiling: the FSM costs ~6 cycles per packet against 64 beats of
    //     data, so a measured cost far above the floor is stall, and which
    //     counter moves says which side of the stream to fix. They ride a
    //     bitstream build, and a counter that miscounts is worse than none -
    //     it sends the next round of work at the wrong half of the path.
    move_pulses = 0; starve_pulses = 0; stall_pulses = 0; req_pulses = 0;
    outq.delete(); wrq.delete();
    incoming(6'd1, 28'd320, TARGET, 1'b1);        // header + 4 payload beats
    send_msg_beat({28'b0, 28'd256, OP_WR}, {16'b0, TARGET}, 64'b0, 1'b0);
    // starved: give it one beat, then leave the ingress dry for a while
    send_msg_beat(64'hA0, 64'hA1, 64'hA2, 1'b0);
    repeat (6) @(posedge aclk);
    send_msg_beat(64'hB0, 64'hB1, 64'hB2, 1'b0);
    // stalled: beats available, host write path refusing them
    @(negedge aclk); m_tready = 0;
    send_msg_beat(64'hC0, 64'hC1, 64'hC2, 1'b0);
    send_msg_beat(64'hD0, 64'hD1, 64'hD2, 1'b1);
    repeat (6) @(posedge aclk);
    @(negedge aclk); m_tready = 1;
    wait_idle_to(400, "13: receive-path cycle accounting");
    check(move_pulses == 4,
          $sformatf("accounting: one move per beat forwarded (%0d of 4)",
                    move_pulses));
    check(starve_pulses > 0,
          "accounting: starve fires when the ingress is dry");
    check(stall_pulses > 0,
          "accounting: stall fires when the host write path is not ready");
    check(outq.size() == 4, "accounting: all four beats still forwarded");
    check(head_pulses + body_pulses == stall_pulses,
          "accounting: head and body partition the stalls exactly");
    // The stall above was applied AFTER beats had already moved, so it is
    // body. A head stall is the shell withholding m_tready before the
    // packet's first beat, which is the single-outstanding cost this is
    // meant to expose.
    check(body_pulses > 0 && head_pulses == 0,
          $sformatf("accounting: a mid-packet stall counts as body (%0d head, %0d body)",
                    head_pulses, body_pulses));
    head_pulses = 0; body_pulses = 0; stall_pulses = 0;
    outq.delete(); wrq.delete();
    @(negedge aclk); m_tready = 0;                 // refuse before ANY beat
    incoming(6'd1, 28'd192, TARGET, 1'b1);
    send_msg_beat({28'b0, 28'd128, OP_WR}, {16'b0, TARGET}, 64'b0, 1'b0);
    send_msg_beat(64'hE0, 64'hE1, 64'hE2, 1'b0);
    send_msg_beat(64'hF0, 64'hF1, 64'hF2, 1'b1);
    repeat (8) @(posedge aclk);
    @(negedge aclk); m_tready = 1;
    wait_idle_to(400, "13b: head stall");
    check(head_pulses > 0,
          $sformatf("accounting: a stall before the first beat counts as head (%0d)",
                    head_pulses));
    // Requests accepted vs completed. On hardware these came apart and
    // nothing could say whether the missing ones never arrived or arrived
    // and never finished; both transactions above accepted exactly one
    // request and finished it.
    check(req_pulses == 2,
          $sformatf("accounting: one pulse per request accepted (%0d of 2)",
                    req_pulses));

    // --- 14. A message spanning three requests absorbs exactly two of
    //     them. Nothing else observes that absorption, so if it ever
    //     mis-counts the payload lands wrong with no counter moving.
    dut_reset();
    req_pulses = 0; span_pulses = 0;
    outq.delete(); wrq.delete();
    incoming(6'd2, 28'd128, STAGING);                 // 2 beats: hdr + 1
    send_msg_beat({28'b0, 28'd192, OP_WR}, {16'b0, TARGET + 48'h2000}, 64'b0, 1'b0);
    send_msg_beat(64'h5100, 64'b0, 64'b0, 1'b0);
    present_pending(6'd2, 28'd64, STAGING + 48'h1000);  // continuation
    send_msg_beat(64'h5101, 64'b0, 64'b0, 1'b0);
    present_pending(6'd2, 28'd64, STAGING + 48'h2000);  // continuation
    send_msg_beat(64'h5102, 64'b0, 64'b0, 1'b1);
    wait_quiet(400, "14: message spanning three requests");
    dump_wrq("case 14");
    check(wrq.size() == 1, "spanning: ONE write for the whole message");
    if (wrq.size() >= 1)
        check(wrq[0].vaddr == TARGET + 48'h2000 && wrq[0].len == 192,
              "spanning: target and length are the header's");
    check(outq.size() == 3, "spanning: all three payload beats forwarded");
    check(req_pulses == 3,
          $sformatf("spanning: three requests accepted (%0d)", req_pulses));
    check(span_pulses == 2,
          $sformatf("spanning: two of them absorbed as continuations (%0d)",
                    span_pulses));

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
