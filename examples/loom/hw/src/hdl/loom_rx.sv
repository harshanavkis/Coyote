import lynxTypes::*;

/**
 * loom_rx
 *
 * Receive side, hybrid dispatch on the incoming RETH vaddr:
 *
 *   vaddr == STAGING: a Loom inline MESSAGE (sub-64 B store envelope).
 *     Parse the header beat and issue the EXACT write it describes.
 *   vaddr != STAGING: a DIRECT RDMA WRITE (bulk) - forward verbatim as
 *     sq_wr {LOCAL_WRITE, pid, vaddr, len} + all beats untouched. (If
 *     the stock shell already lands these without user logic - gate G3
 *     - this path simply never fires; both are correct.)
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

    // Staging vaddr (from loom_ctrl): selects message-parse vs direct
    input  logic [VADDR_BITS-1:0]       rdma_staging_va,

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
    output logic                        cnt_rx_drop
);

// Wire-message header ops (keep in sync with loom_engine.sv)
localparam [7:0] MSG_OP_WRITE        = 8'd1;
localparam [7:0] MSG_OP_WRITE_INLINE = 8'd2;

typedef enum logic [2:0] {
    ST_IDLE, ST_HDR, ST_WR_REQ, ST_STREAM, ST_INLINE_DATA, ST_DRAIN
} state_t;
state_t state;

logic [PID_BITS-1:0]   l_pid;
logic [7:0]            l_op;
logic [27:0]           l_len;
logic [VADDR_BITS-1:0] l_va;
logic [63:0]           l_inline;
logic [22:0]           l_beats;      // beats of this request still on the stream
logic                  l_req_last;   // shell will terminate this stream with tlast

// Where a transaction ends is decided by the REQUEST's length, not by
// tlast. rq_wr.last is low whenever the shell ends the stream WITHOUT a
// tlast (see req_t in lynx_pkg), and ib_transport_protocol emits exactly
// that for every RDMA_WRITE_FIRST/MIDDLE fragment; waiting for a tlast then
// runs into the next message's beats, and from there every parse reads
// payload as a header - which is how a store's target becomes whatever
// happened to sit in lane 1. tlast still ends a transaction early when it
// arrives first; whatever the request still owns after the write is drained
// rather than left behind for the next parse to trip over.
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
// parks in ST_WR_REQ - which is exactly how the two-host run wedged, with
// rx_fwd frozen at 17 of 26 and nothing rejected. The count is used ONLY
// where there is no tlast to wait for, and to bound the drain.
wire stream_end = l_req_last ? s_tlast : (l_beats <= 23'd1);

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

// Accept a request only when granted, so a transaction never starts while
// the engine owns the shared write path
assign rq_ready = (state == ST_IDLE) && grant;
assign req      = rq_valid || (state != ST_IDLE);
assign busy     = (state != ST_IDLE);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        state <= ST_IDLE;
        l_pid <= 0; l_op <= 0; l_len <= 0; l_va <= 0; l_inline <= 0;
        l_beats <= 0; l_req_last <= 1'b1;
    end else begin
        case (state)
            ST_IDLE: if (rq_valid && grant) begin
                l_pid      <= rq_req.pid;
                l_beats     <= beats_of(rq_req.len[27:0]);
                l_req_last  <= rq_req.last;
                if (rq_req.vaddr[VADDR_BITS-1:0] == rdma_staging_va) begin
                    state <= ST_HDR;             // Loom message: parse
                end else begin
                    // Direct bulk write: take target and length from the
                    // request itself, forward every beat untouched
                    l_va  <= rq_req.vaddr[VADDR_BITS-1:0];
                    l_len <= rq_req.len[27:0];
                    l_op  <= MSG_OP_WRITE;
                    state <= ST_WR_REQ;
                end
            end

            // Parse the header beat
            ST_HDR: if (s_tvalid) begin
                l_op     <= s_tdata[7:0];
                l_len    <= s_tdata[35:8];
                l_va     <= s_tdata[64 +: VADDR_BITS];
                l_inline <= s_tdata[128 +: 64];
                l_beats  <= l_beats - 23'd1;
                if (hdr_ok)
                    state <= ST_WR_REQ;
                else
                    // Not a header we recognize: drain what the request
                    // still owns and write nothing (cnt_rx_drop pulses below)
                    state <= stream_end ? ST_IDLE : ST_DRAIN;
            end

            ST_WR_REQ: if (wr_ready)
                state <= (l_op == MSG_OP_WRITE_INLINE) ? ST_INLINE_DATA : ST_STREAM;

            // One constructed beat: the inline data moved to lane 0. A
            // message that was not a single beat still owns the rest of
            // its request, so those beats are drained here instead of
            // being left for the next parse to read as a header
            ST_INLINE_DATA: if (m_tready)
                state <= (l_beats != 0) ? ST_DRAIN : ST_IDLE;

            // Forward the remaining payload beats
            ST_STREAM: if (s_tvalid && m_tready) begin
                l_beats <= l_beats - 23'd1;
                if (stream_end) state <= ST_IDLE;
            end

            ST_DRAIN: if (s_tvalid) begin
                l_beats <= l_beats - 23'd1;
                if (s_tlast || l_beats <= 23'd1) state <= ST_IDLE;
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
    wr_req.pid    = l_pid;
    wr_req.vaddr  = l_va;
    wr_req.len    = l_len;
    wr_req.dest   = 0;
    wr_req.last   = 1'b1;
    wr_valid = (state == ST_WR_REQ);
end

always_comb begin
    // Consume the incoming stream while parsing, forwarding, or draining
    s_tready = (state == ST_HDR) || (state == ST_DRAIN) ||
               ((state == ST_STREAM) && m_tready);

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
        m_tvalid = (state == ST_STREAM) && s_tvalid;
    end
end

assign cnt_rx_fwd  = ((state == ST_INLINE_DATA) && m_tready) ||
                     ((state == ST_STREAM) && s_tvalid && m_tready && stream_end);
assign cnt_rx_drop = (state == ST_HDR) && s_tvalid && !hdr_ok;

endmodule
