import lynxTypes::*;

/**
 * loom_rx
 *
 * Receive side. EVERY incoming transaction is a Loom message and its
 * destination comes from the message header - never from rq_wr.vaddr. The
 * request is used for its pid (whose address space to land in) and to keep
 * the interface flowing; its address is ignored.
 *
 * That is deliberate, and it is how jigsaw's controller works. A write
 * addressed by the incoming RETH means the shell's per-packet cursor decides
 * where a DMA lands: one host write per PMTU packet, 256 of them for a 1 MB
 * transfer, and a stray packet - a retransmission arriving out of a message,
 * say - becomes a write to whatever address it carried. Taking the target
 * from our own header instead gives one write per MESSAGE however many
 * packets it spans, and leaves no path by which an unrecognized beat can
 * name its own destination: it fails hdr_ok and is counted, not written.
 *
 * Message header (first beat, staging only):
 *
 *   lane0 = {reserved, len[27:0], op[7:0]}     (keep in sync w/ loom_engine)
 *   lane1 = target VA (the exporter's own VA + offset)
 *   lane2 = inline data (op WRITE_INLINE only)
 *
 *   op 1 WRITE:        payload beats follow the header; forward them as
 *                      sq_wr {LOCAL_WRITE, pid, target VA, hdr len}
 *   op 2 WRITE_INLINE: single-beat message; issue the EXACT hdr-len
 *                      (8 B) write with lane2's data - this is how a
 *                      sub-64 B store crosses the wire without ever
 *                      putting a sub-beat RDMA payload on it (gate G5)
 *                      and without clobbering destination neighbors.
 *
 * The local write runs under the QP owner's pid: when the QP belongs to
 * the exporting process's cThread, the shell TLB translates the target
 * VA in exactly the right address space.
 *
 * Transaction-serialized like the engine; starts only when granted the
 * shared {sq_wr, axis_host_send} path (top-level arbiter). A request owns
 * exactly ceil(len/64) beats of the payload stream - that, not tlast, is
 * what ends a transaction - and a header that does not match the contract
 * is drained and counted (cnt_rx_drop) instead of being translated.
 */
module loom_rx (
    input  logic                        aclk,
    input  logic                        aresetn,

    // Incoming write requests (rq_wr)
    input  req_t                        rq_req,
    input  logic                        rq_valid,
    output logic                        rq_ready,

    // Staging vaddr (from loom_ctrl). No longer selects anything - every
    // transaction is parsed as a message - but kept wired so the exporter's
    // staging address is available here if a sanity check is ever wanted.
    /* verilator lint_off UNUSED */
    input  logic [VADDR_BITS-1:0]       rdma_staging_va,

    // Whose address space incoming writes land in: the QP owner's cThread,
    // fixed for the connection and written by loomd at QP setup
    input  logic [PID_BITS-1:0]         rx_pid,
    /* verilator lint_on UNUSED */

    // Write requests out (to sq_wr via arbiter)
    output req_t                        wr_req,
    output logic                        wr_valid,
    input  logic                        wr_ready,

    // Payload in (axis_rrsp_recv)
    input  logic [AXI_DATA_BITS-1:0]    s_tdata,
    input  logic [AXI_DATA_BITS/8-1:0]  s_tkeep,
    input  logic                        s_tvalid,
    output logic                        s_tready,
    input  logic                        s_tlast,

    // Payload out (to axis_host_send via arbiter)
    output logic [AXI_DATA_BITS-1:0]    m_tdata,
    output logic [AXI_DATA_BITS/8-1:0]  m_tkeep,
    output logic                        m_tvalid,
    input  logic                        m_tready,
    output logic                        m_tlast,

    // Arbitration
    output logic                        req,       // wants the shared path
    input  logic                        grant,     // exclusive ownership while high
    output logic                        busy,

    // Debug counter pulses (to loom_ctrl)
    output logic                        cnt_rx_fwd,
    output logic                        cnt_rx_drop,
    // A beat the shell never announced. rq_wr says, packet by packet, how
    // many bytes are about to be delivered; anything beyond that is a beat
    // no request accounts for, and it is swallowed rather than forwarded.
    output logic                        cnt_rx_orphan,

    // Where the cycles go while forwarding. This module holds NO buffer: a
    // beat moves only when the RoCE ingress and the host write path are
    // ready in the SAME cycle, so every bubble on either side costs a cycle
    // and a lost cycle on ingress is eventually a dropped packet. The FSM
    // itself is cheap - one cycle to accept the request, one for the sq_wr
    // handshake, ~2 to hand the arbiter back, against 64 beats of data - so
    // when the measured cost per packet runs far above the 64-beat floor,
    // the difference is stall, not overhead, and these say which side.
    output logic                        cnt_rx_move,    // both ready
    output logic                        cnt_rx_starve,  // ingress had nothing
    output logic                        cnt_rx_stall,
    output logic                        cnt_rx_bp,   // host write not ready
    // Stall split by WHERE in the packet it lands, which is what tells the
    // two candidate fixes apart. This module is single-outstanding on the
    // write side: it posts one sq_wr, streams that packet, and only then
    // takes the next request - so if the shell withholds m_tready until it
    // has accepted and translated the request, every packet pays that
    // latency serially and the stalls bunch up BEFORE its first beat.
    // Head-heavy means overlap the next request with the current stream;
    // body-heavy means the host write path is bursty and wants a buffer;
    // neither means its sustained bandwidth is simply the ceiling.
    output logic                        cnt_rx_stall_head,
    output logic                        cnt_rx_stall_body,

    // Requests ACCEPTED off rq_wr. cnt_rx_fwd counts the ones this module
    // finished, and the difference between the two is the question that
    // could not be answered on hardware for a whole round of debugging:
    // when completions came up short of the packets the shell must have
    // sent, nothing said whether the requests never arrived or arrived and
    // were never finished. Those have opposite causes and opposite fixes.
    output logic                        cnt_rx_req,

    // Continuation requests absorbed by a spanning message. A bulk transfer
    // is one logical write across many packets, so most of its rq_wr's are
    // swallowed here rather than becoming transactions - and nothing else
    // observes that. If the absorption ever mis-counts, the payload lands
    // wrong with no counter moving, which is the situation this whole
    // investigation started in. Expect (packets per message - 1) per bulk.
    output logic                        cnt_rx_span
);

// Wire-message header ops (keep in sync with loom_engine.sv)
localparam [7:0] MSG_OP_WRITE        = 8'd1;
localparam [7:0] MSG_OP_WRITE_INLINE = 8'd2;

typedef enum logic [2:0] {
    ST_IDLE, ST_STREAM, ST_PKT_REQ, ST_INLINE_DATA
} state_t;
state_t state;

logic [7:0]            l_op;
logic [27:0]           l_len;
logic [VADDR_BITS-1:0] l_va;
logic [63:0]           l_inline;
logic [22:0]           l_beats;      // beats of THIS PACKET still on the stream
// One host write PER PACKET, not per message. Loom used to issue a single
// write covering the whole message and swallow the shell's per-packet
// requests as credit - 1027 of 1033 on a 4 MB transfer - which handed the
// DMA one 4 MB descriptor where perf_rdma hands it 1024 x 4 KB. perf_rdma
// forwards each request (sq_wr.valid = rq_wr.valid) and never loses a
// packet; Loom's single long descriptor cannot pipeline, the host write
// path goes bursty (measured: every stall mid-packet, none at a packet's
// first beat), the ingress FIFO overflows and packets vanish upstream of
// the PSN check. Across four runs loss correlated perfectly with
// corruption: 0 lost -> intact, 12-19 lost -> corrupt.
//
// The addresses are OURS, not the shell's: the RETH names the STAGING
// buffer (loom_engine.sv), so an incoming rq_wr carries no destination.
// Only the header does. So the split is computed here - the shell
// fragments deterministically at PMTU, and the first packet gives up 64 B
// of its payload to the header.
logic [VADDR_BITS-1:0] w_va;         // where THIS packet's write lands
logic [27:0]           w_left;       // message bytes after this packet
logic                  l_moved;      // this transaction has had at least one beat
// A WRITE message (op 1) is ONE logical write that may span several PMTU
// packets, so its beat budget comes from the HEADER's length, not from the
// request's, and the intermediate rq_wr's and tlasts belong to packets
// rather than to the transaction. This is what lets the receive path issue
// one host write per message instead of one per packet.

// Where a transaction ends is decided by the MESSAGE HEADER's length, and by
// nothing else. Not by tlast, not by rq_wr - this is what jigsaw's
// controller does, and it is the whole reason its receive side has nothing
// to desynchronise: it latches a length out of its own header, counts the
// payload beats down, and never looks at the request stream or tlast at all.
//
// Loom carries the same information in the same place, so it can be read the
// same way. Everything that used to pair beats against requests is gone: the
// per-request beat budget, the rule for absorbing a spanning message's
// continuation requests, the interlock keeping those two from colliding.
// That machinery is what produced a one-beat displacement at 1 MB - a count
// that went wrong once and stayed wrong - and none of it was ever needed.
function automatic logic [22:0] beats_of(input logic [27:0] len);
    logic [28:0] padded;
    padded   = {1'b0, len} + 29'd63;
    beats_of = padded[28:6];
endfunction

// Last beat this transaction may take off the stream.
//
// rq_wr.last is the shell telling us whether it will terminate this stream:
// high means a tlast is coming and IS the boundary, low means the stream
// just stops (req_t in lynx_pkg; ib_transport_protocol emits low for every
// RDMA_WRITE_FIRST/MIDDLE fragment). Deriving the boundary from the length
// instead is wrong whenever the delivered beat count differs from
// ceil(len/64) by even one: the transaction ends off by a beat, the write
// we asked the shell for is never satisfied, sq_wr backs up and this module
// parks with the ingress held off - which is exactly how the two-host run
// wedged, with rx_fwd frozen at 17 of 26 and nothing rejected. The count is used ONLY
// where there is no tlast to wait for, and to bound the drain.
// A spanning message ends where its header said it would and nowhere else:
// the tlast at every intermediate packet boundary is not its boundary.

// Header contract. The exporter hands the parsed target straight to the
// shell TLB under the QP owner's pid, so a header this side does not
// recognize must never become a write: whatever sits in lane 1 would be
// written to. loom_engine only ever emits the two forms below, so anything
// else is a beat that is not a header (or a sender that disagrees with us),
// and is dropped and counted rather than translated.
//   inline: lane0 == {28'b0, 28'd8, op2}, target 8 B aligned
//   write:  lane0 == {28'b0, len, op1}, len a nonzero multiple of 64 B
wire [27:0] hdr_len = s_tdata[35:8];
wire [7:0]  hdr_op  = s_tdata[7:0];
wire hdr_ok = (s_tdata[63:36] == 28'b0) &&
              ((hdr_op == MSG_OP_WRITE_INLINE &&
                hdr_len == 28'd8 && s_tdata[64 +: 3] == 3'b0) ||
               (hdr_op == MSG_OP_WRITE &&
                hdr_len != 28'd0 && hdr_len[5:0] == 6'b0));

// Requests are still DRAINED - rq_ready never falls, because holding it low
// backs up the shell's request path and that is exactly how a two-host run
// wedged once, with rx_fwd frozen at 17 of 26. What changed is that they are
// no longer IGNORED: each one announces how many bytes the shell is about to
// deliver, and that count becomes credit.
//
// A payload beat is forwarded only against credit. Loom used to believe the
// payload stream alone, so a beat the shell never announced - the head of a
// packet streamed to the user before the stack decided to drop it - was
// absorbed as payload and displaced every byte after it, permanently and
// silently. The header still says WHERE the bytes go, so a stray packet
// still cannot name its own destination; the shell now says HOW MANY there
// are. The two agree exactly over a message: the shell announces len + 64,
// and the header's beats_of(len) plus the header beat is the same count.
//
// The pid comes from a CSR, written once at QP setup - the QP owner is fixed
// for the life of the connection, so there is nothing per-request about it.
assign rq_ready = 1'b1;

logic [31:0] p_credit;
wire         covered   = (p_credit != 32'd0);
wire         take_beat = s_tvalid && s_tready;
wire [22:0]  cred_add  = (rq_valid && rq_ready) ? beats_of(rq_req.len) : 23'd0;

// The split is the shell's own, not a guess. rdma_req_parser.sv fragments an
// APP_WRITE at exactly PMTU: ST_PARSE_WRITE_INIT emits RC_RDMA_WRITE_FIRST
// with plen = PMTU_BYTES and ST_PARSE_WRITE emits _MIDDLE the same way, the
// remainder falling out as _LAST. So packet 0 is the first PMTU bytes of the
// message - for us 64 B of header plus PMTU-64 of payload - and every packet
// after it is a full PMTU until the tail. These two lengths therefore land
// one host write on each wire packet exactly, which is the whole point.
//
// Correctness does NOT depend on that alignment holding: the writes are
// contiguous and sum to hdr_len either way, so a different fragmentation
// would only make a write span a packet boundary - which is what the old
// single-write-per-message code did for the entire message. Only the
// one-descriptor-per-packet property depends on it.
wire [27:0] first_len = (hdr_len > (PMTU_BYTES - 28'd64)) ? (PMTU_BYTES - 28'd64)
                                                          : hdr_len;
wire [27:0] next_len  = (w_left  >  PMTU_BYTES[27:0])     ? PMTU_BYTES[27:0]
                                                          : w_left;

// A header beat waiting on the payload stream is what wants the shared path
assign req      = s_tvalid || (state != ST_IDLE);
assign busy     = (state != ST_IDLE);

// Last payload beat of the message
wire stream_end = (l_beats <= 23'd1);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        state <= ST_IDLE;
        l_op <= 0; l_len <= 0; l_va <= 0; l_inline <= 0;
        l_beats <= 0; l_moved <= 1'b0; w_va <= 0; w_left <= 0;
        p_credit <= 0;
    end else begin
        p_credit <= p_credit + {9'b0, cred_add} - (take_beat ? 32'd1 : 32'd0);
        case (state)
            // The header beat arrives on the payload stream like any other,
            // and is recognised by its contents rather than announced by a
            // request. A beat that is not a header is skipped and counted;
            // with a sender that agrees with us every beat here IS one.
            // The write request is issued COMBINATIONALLY from this same
            // beat and the header is consumed only once the shell has taken
            // it. There is deliberately no ST_WR_REQ: that state held
            // s_tready LOW while it waited for wr_ready, and s_tready low is
            // not merely a delay here - the shell's receive FSM emits the
            // host-write command and the ACK in the SAME step, so stalling
            // the ingress stops ACKS BEING GENERATED. The sender's retransmit
            // timer is 1 ms since the last ack, and a replay cannot re-read a
            // vFPGA-sourced payload (req_1 = 0 in user_req_mux.sv), so lost
            // beats displace everything after them. perf_rdma cannot hit this
            // because its receive path is a pass-through
            // (rq_wr.ready = sq_wr.ready); this makes ours behave the same, so
            // the stream stalls only when the shell itself will not take a
            // request. Re-latching an unconsumed beat is idempotent.
            ST_IDLE: if (s_tvalid && grant && covered) begin
                l_op     <= hdr_op;
                l_len    <= hdr_len;
                l_va     <= s_tdata[64 +: VADDR_BITS];
                l_inline <= s_tdata[128 +: 64];
                // Only THIS packet, and where the next one lands. The write
                // being accepted here covers first_len bytes, not hdr_len.
                l_beats  <= beats_of(first_len);
                w_va     <= s_tdata[64 +: VADDR_BITS]
                            + {{(VADDR_BITS-28){1'b0}}, first_len};
                w_left   <= hdr_len - first_len;
                l_moved  <= 1'b0;
                if (hdr_ok && wr_ready)
                    state <= (hdr_op == MSG_OP_WRITE_INLINE) ? ST_INLINE_DATA
                                                             : ST_STREAM;
            end

            ST_INLINE_DATA: if (m_tready) state <= ST_IDLE;

            // Forward exactly the payload the header promised, however many
            // packets it spans. Intermediate tlasts belong to packets, not
            // to this message, and are ignored.
            // An uncovered beat is swallowed here without advancing the
            // message: it is not payload, whatever it looks like.
            // stream_end now ends a PACKET's write, not the message.
            ST_STREAM: if (s_tvalid && m_tready && covered) begin
                l_moved <= 1'b1;
                l_beats <= l_beats - 23'd1;
                if (stream_end)
                    state <= (w_left != 28'd0) ? ST_PKT_REQ : ST_IDLE;
            end

            // The next packet's write. Waits on the SHELL's wr_ready and
            // nothing of our own, the same shape as the header's handshake.
            ST_PKT_REQ: if (wr_ready) begin
                l_beats <= beats_of(next_len);
                w_va    <= w_va + {{(VADDR_BITS-28){1'b0}}, next_len};
                w_left  <= w_left - next_len;
                l_moved <= 1'b0;
                state   <= ST_STREAM;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

// The local write names the header's target under the QP owner's pid
always_comb begin
    wr_req = '0;
    wr_req.opcode = LOCAL_WRITE;
    wr_req.strm   = STRM_HOST;
    wr_req.pid    = rx_pid;
    // ONE WRITE PER PACKET. The first is driven from the header beat on the
    // wire (issued in the same cycle that beat is accepted); every later one
    // from the running target this module keeps, because the shell's request
    // stream names STAGING and cannot say where the bytes belong.
    wr_req.vaddr  = (state == ST_PKT_REQ) ? w_va
                                          : s_tdata[64 +: VADDR_BITS];
    wr_req.len    = (state == ST_PKT_REQ) ? next_len : first_len;
    wr_req.dest   = 0;
    wr_req.last   = 1'b1;
    wr_valid = ((state == ST_IDLE) && s_tvalid && grant && covered && hdr_ok)
               || (state == ST_PKT_REQ);
end

always_comb begin
    // Consume the incoming stream while parsing, forwarding, or draining
    // Hold the payload off while a spanning message waits for its next
    // request: those beats belong to a request this module has not taken yet
    // A header beat is taken in ST_IDLE (only while granted); payload beats
    // move when the host write path will take them
    // A header is consumed only when the shell accepts its request; an
    // uncovered or malformed beat is still drained so it cannot back up.
    s_tready = ((state == ST_IDLE) && grant &&
                ((covered && hdr_ok) ? wr_ready : 1'b1)) ||
               ((state == ST_STREAM) && (covered ? m_tready : 1'b1));

    if (state == ST_INLINE_DATA) begin
        // Constructed beat: exact-length write, data LSB-aligned
        m_tdata  = {{(AXI_DATA_BITS-64){1'b0}}, l_inline};
        m_tkeep  = {{(AXI_DATA_BITS/8-8){1'b0}}, 8'hFF};
        m_tlast  = 1'b1;
        m_tvalid = 1'b1;
    end else begin
        m_tdata  = s_tdata;
        m_tkeep  = s_tkeep;
        // The write we issued must be terminated even when the incoming
        // stream carries no tlast of its own (rq_wr.last low): the beat
        // budget ends the transaction, so it ends the stream too
        m_tlast  = stream_end;
        m_tvalid = (state == ST_STREAM) && s_tvalid && covered;
    end
end

assign cnt_rx_fwd  = ((state == ST_INLINE_DATA) && m_tready) ||
                     ((state == ST_STREAM) && s_tvalid && m_tready && covered && stream_end);
assign cnt_rx_drop = (state == ST_IDLE) && s_tvalid && grant && covered && !hdr_ok;
assign cnt_rx_orphan = take_beat && !covered;

assign cnt_rx_move   = (state == ST_STREAM) &&  s_tvalid &&  m_tready && covered;
assign cnt_rx_starve = (state == ST_STREAM) && !s_tvalid;
assign cnt_rx_stall  = (state == ST_STREAM) &&  s_tvalid && !m_tready;

// EVERY cycle this module refuses a beat the shell is offering, in ANY state.
// cnt_rx_stall above is ST_STREAM-only and therefore blind to the grant wait
// and to the request handshake - the two places Loom differs from perf_rdma -
// so a run could show "0.0% stalled" while loom_rx was in fact holding the
// ingress off. That matters more than throughput: the shell emits the host
// write and the packet's ACK from the same FSM step, so backpressure here
// stops ACKs, and a QP that goes 1 ms without one retransmits. This counter
// is the direct measure of the thing being fixed - it must read ~0.
assign cnt_rx_bp     = s_tvalid && !s_tready;

assign cnt_rx_stall_head = cnt_rx_stall && !l_moved;
assign cnt_rx_stall_body = cnt_rx_stall &&  l_moved;

assign cnt_rx_req = rq_valid && rq_ready;   // drained, not acted on

assign cnt_rx_span = rq_valid && rq_ready && (state == ST_STREAM);

endmodule
