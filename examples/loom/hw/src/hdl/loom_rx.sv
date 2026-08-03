import lynxTypes::*;

/**
 * loom_rx
 *
 * Receive side: forwards an incoming RDMA write to local memory.
 * Accepts one request from rq_wr (req_t: pid = QP owner at this host,
 * vaddr = RETH vaddr = the exporter's own VA, len), then issues
 * wr_req {LOCAL_WRITE, STRM_HOST, pid, vaddr, len} and forwards the
 * payload stream (axis_rrsp_recv) to the host output until tlast.
 *
 * Transaction-serialized like loom_engine; sq_wr and the host output
 * stream are shared with the engine through the top-level arbiter (the
 * `grant` input). Exact interposition semantics of the stock shell
 * (whether incoming writes surface here at all, and on which stream)
 * are hardware gate G3; in simulation the TB defines them.
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
    output logic                        req,       // wants the shared sq_wr/host stream
    input  logic                        grant,     // exclusive ownership while high
    output logic                        busy,

    // Debug counter pulse (to loom_ctrl)
    output logic                        cnt_rx_fwd
);

typedef enum logic [1:0] { ST_IDLE, ST_WR_REQ, ST_STREAM } state_t;
state_t state;

logic [PID_BITS-1:0]   l_pid;
logic [VADDR_BITS-1:0] l_vaddr;
logic [LEN_BITS-1:0]   l_len;

// Accept a request only when granted, so a transaction never starts while
// the engine owns the shared write path.
assign rq_ready = (state == ST_IDLE) && grant;
assign req      = rq_valid || (state != ST_IDLE);
assign busy     = (state != ST_IDLE);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        state <= ST_IDLE;
        l_pid <= 0; l_vaddr <= 0; l_len <= 0;
    end else begin
        case (state)
            ST_IDLE: if (rq_valid && grant) begin
                l_pid   <= rq_req.pid;
                l_vaddr <= rq_req.vaddr;
                l_len   <= rq_req.len;
                state   <= ST_WR_REQ;
            end
            ST_WR_REQ: if (wr_ready) state <= ST_STREAM;
            ST_STREAM: if (s_tvalid && s_tlast && m_tready) state <= ST_IDLE;
            default: state <= ST_IDLE;
        endcase
    end
end

always_comb begin
    wr_req = '0;
    wr_req.opcode = LOCAL_WRITE;
    wr_req.strm   = STRM_HOST;
    wr_req.pid    = l_pid;
    wr_req.vaddr  = l_vaddr;
    wr_req.len    = l_len;
    wr_req.dest   = 0;
    wr_req.last   = 1'b1;
    wr_valid = (state == ST_WR_REQ);
end

always_comb begin
    s_tready = (state == ST_STREAM) && m_tready;
    m_tdata  = s_tdata;
    m_tkeep  = s_tkeep;
    m_tlast  = s_tlast;
    m_tvalid = (state == ST_STREAM) && s_tvalid;
end

assign cnt_rx_fwd = (state == ST_STREAM) && s_tvalid && s_tlast && m_tready;

endmodule
