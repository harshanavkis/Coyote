import lynxTypes::*;

/**
 * loom_rx
 *
 * Receive side: parses incoming Loom wire MESSAGES and issues the exact
 * local write they describe. Every message arrives as an RDMA WRITE to
 * this host's staging vaddr (data-meaningless; jigsaw's remote_vaddr
 * pattern) - the request surfaces on rq_wr {pid = QP owner, staging
 * vaddr, wire len} and the payload on axis_rrsp_recv. The first beat is
 * the header:
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
 * shared {sq_wr, axis_host_send} path (top-level arbiter). Unknown ops
 * drain the message without writing anything (counted as forwarded for
 * now; a dedicated counter can come with the two-host phase).
 */
module loom_rx (
    input  logic                        aclk,
    input  logic                        aresetn,

    // Incoming write requests (rq_wr)
    input  req_t                        rq_req,
    input  logic                        rq_valid,
    output logic                        rq_ready,

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

    // Debug counter pulse (to loom_ctrl)
    output logic                        cnt_rx_fwd
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
logic                  l_hdr_last;   // single-beat message?

// Accept a request only when granted, so a transaction never starts while
// the engine owns the shared write path
assign rq_ready = (state == ST_IDLE) && grant;
assign req      = rq_valid || (state != ST_IDLE);
assign busy     = (state != ST_IDLE);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        state <= ST_IDLE;
        l_pid <= 0; l_op <= 0; l_len <= 0; l_va <= 0; l_inline <= 0;
        l_hdr_last <= 0;
    end else begin
        case (state)
            ST_IDLE: if (rq_valid && grant) begin
                l_pid <= rq_req.pid;
                state <= ST_HDR;
            end

            // Parse the header beat
            ST_HDR: if (s_tvalid) begin
                l_op     <= s_tdata[7:0];
                l_len    <= s_tdata[35:8];
                l_va     <= s_tdata[64 +: VADDR_BITS];
                l_inline <= s_tdata[128 +: 64];
                l_hdr_last <= s_tlast;
                if (s_tdata[7:0] == MSG_OP_WRITE ||
                    s_tdata[7:0] == MSG_OP_WRITE_INLINE)
                    state <= ST_WR_REQ;
                else
                    // Unknown op: drain the rest of the message, write nothing
                    state <= s_tlast ? ST_IDLE : ST_DRAIN;
            end

            ST_WR_REQ: if (wr_ready)
                state <= (l_op == MSG_OP_WRITE_INLINE) ? ST_INLINE_DATA : ST_STREAM;

            // One constructed beat: the inline data moved to lane 0
            ST_INLINE_DATA: if (m_tready) state <= ST_IDLE;

            // Forward the remaining payload beats
            ST_STREAM: if (s_tvalid && s_tlast && m_tready) state <= ST_IDLE;

            ST_DRAIN: if (s_tvalid && s_tlast) state <= ST_IDLE;

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
        m_tlast  = s_tlast;
        m_tvalid = (state == ST_STREAM) && s_tvalid;
    end
end

assign cnt_rx_fwd = ((state == ST_INLINE_DATA) && m_tready) ||
                    ((state == ST_STREAM) && s_tvalid && s_tlast && m_tready);

endmodule
