import lynxTypes::*;

/**
 * loom_engine
 *
 * Consumer of the order FIFO. Transaction-serialized: exactly one FIFO
 * entry is in flight at a time, which is what makes the FIFO an order
 * point (a flag store behind a DMA descriptor cannot pass it).
 *
 * STORE entry (small write, <= 8 B):
 *   local: wr_req {LOCAL_WRITE, STRM_HOST, pid, base+off, 8} + 1 beat on
 *          the host output stream (data LSB-aligned, tkeep low 8 bytes)
 *   rdma:  wr_req {APP_WRITE, STRM_RDMA, remote, pid = QP owner,
 *          base+off, 8} + 1 beat on the net output stream
 *
 * DESC entry (bulk):
 *   1. rd_req {LOCAL_READ, STRM_HOST, src_pid, src_va, len} (the pull)
 *   2. wr_req as above with the descriptor length
 *   3. forward the pull stream to the selected output until tlast
 *   4. if COMPL_VA != 0: wr_req {LOCAL_WRITE, compl_pid, compl_va, 8} +
 *      one beat carrying an incrementing completion count
 *
 * Invalid window / bounds violation: entry dropped, cnt_drop pulsed.
 * Sub-8B stores (wstrb != 0xFF) are issued as full 8 B writes for now
 * (hardware gate G2 covers sub-line write semantics).
 */
module loom_engine (
    input  logic                        aclk,
    input  logic                        aresetn,

    // Order FIFO (from loom_ctrl)
    input  logic                        fifo_empty,
    input  logic                        fifo_is_desc,
    input  logic [3:0]                  fifo_win,
    input  logic [27:0]                 fifo_off,
    input  logic [27:0]                 fifo_len,
    input  logic [PID_BITS-1:0]         fifo_src_pid,
    input  logic [63:0]                 fifo_payload,
    output logic                        fifo_pop,

    // Window table lookup
    output logic [3:0]                  lu_idx,
    input  logic                        lu_valid,
    input  logic                        lu_route,
    input  logic [PID_BITS-1:0]         lu_pid,
    input  logic [VADDR_BITS-1:0]       lu_base,
    input  logic [LEN_BITS-1:0]         lu_len,

    // Completion config (from loom_ctrl)
    input  logic [PID_BITS-1:0]         compl_pid,
    input  logic [VADDR_BITS-1:0]       compl_va,

    // sq_rd (pull requests)
    output req_t                        rd_req,
    output logic                        rd_valid,
    input  logic                        rd_ready,

    // sq_wr (write requests; shared with loom_rx via top-level arbiter)
    output req_t                        wr_req,
    output logic                        wr_valid,
    input  logic                        wr_ready,

    // Pull payload in (axis_host_recv)
    input  logic [AXI_DATA_BITS-1:0]    s_tdata,
    input  logic [AXI_DATA_BITS/8-1:0]  s_tkeep,
    input  logic                        s_tvalid,
    output logic                        s_tready,
    input  logic                        s_tlast,

    // Local/completion data out (axis_host_send)
    output logic [AXI_DATA_BITS-1:0]    m_host_tdata,
    output logic [AXI_DATA_BITS/8-1:0]  m_host_tkeep,
    output logic                        m_host_tvalid,
    input  logic                        m_host_tready,
    output logic                        m_host_tlast,

    // RDMA data out (axis_rreq_send)
    output logic [AXI_DATA_BITS-1:0]    m_net_tdata,
    output logic [AXI_DATA_BITS/8-1:0]  m_net_tkeep,
    output logic                        m_net_tvalid,
    input  logic                        m_net_tready,
    output logic                        m_net_tlast,

    // Debug counter pulses (to loom_ctrl)
    output logic                        cnt_local_wr,
    output logic                        cnt_rdma_wr,
    output logic                        cnt_drop,
    output logic                        cnt_compl,

    output logic                        busy
);

typedef enum logic [3:0] {
    ST_IDLE, ST_CHECK, ST_WR_REQ, ST_WR_DATA,
    ST_RD_REQ, ST_DMA_WR_REQ, ST_STREAM,
    ST_CP_REQ, ST_CP_DATA
} state_t;

state_t state;

// Latched entry + table hit
logic                  l_is_desc;
logic [27:0]           l_off, l_len;
logic [PID_BITS-1:0]   l_src_pid;
logic [63:0]           l_payload;
logic                  l_valid, l_route;
logic [PID_BITS-1:0]   l_pid;
logic [VADDR_BITS-1:0] l_base;
logic [LEN_BITS-1:0]   l_lim;

logic [63:0] compl_cnt;

assign lu_idx = fifo_win;
assign busy   = (state != ST_IDLE);

// Bounds: end offset of the access within the segment
wire [28:0] end_off = l_is_desc ? ({1'b0, l_off} + {1'b0, l_len})
                                : ({1'b0, l_off} + 29'd8);
wire ok = l_valid && (l_is_desc ? (l_len != 0) : 1'b1)
                  && (end_off <= {1'b0, l_lim});

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        state <= ST_IDLE;
        compl_cnt <= 0;
        l_is_desc <= 0; l_off <= 0; l_len <= 0; l_src_pid <= 0; l_payload <= 0;
        l_valid <= 0; l_route <= 0; l_pid <= 0; l_base <= 0; l_lim <= 0;
    end else begin
        case (state)
            ST_IDLE: if (!fifo_empty) begin
                l_is_desc <= fifo_is_desc;
                l_off     <= fifo_off;
                l_len     <= fifo_len;
                l_src_pid <= fifo_src_pid;
                l_payload <= fifo_payload;
                l_valid   <= lu_valid;
                l_route   <= lu_route;
                l_pid     <= lu_pid;
                l_base    <= lu_base;
                l_lim     <= lu_len;
                state     <= ST_CHECK;
            end

            ST_CHECK:
                if (!ok)            state <= ST_IDLE;        // cnt_drop pulses below
                else if (l_is_desc) state <= ST_RD_REQ;
                else                state <= ST_WR_REQ;

            ST_WR_REQ:    if (wr_ready) state <= ST_WR_DATA;
            ST_WR_DATA:
                if (( l_route && m_net_tready) ||
                    (!l_route && m_host_tready)) state <= ST_IDLE;

            ST_RD_REQ:    if (rd_ready) state <= ST_DMA_WR_REQ;
            ST_DMA_WR_REQ: if (wr_ready) state <= ST_STREAM;
            ST_STREAM:
                if (s_tvalid && s_tlast &&
                    (( l_route && m_net_tready) || (!l_route && m_host_tready)))
                    state <= (compl_va != 0) ? ST_CP_REQ : ST_IDLE;

            ST_CP_REQ:    if (wr_ready) state <= ST_CP_DATA;
            ST_CP_DATA:
                if (m_host_tready) begin
                    compl_cnt <= compl_cnt + 1;
                    state <= ST_IDLE;
                end

            default: state <= ST_IDLE;
        endcase
    end
end

// Pop in the same cycle the entry is latched
assign fifo_pop = (state == ST_IDLE) && !fifo_empty;

// -------------------------------------------------------------------------
// Requests
// -------------------------------------------------------------------------
wire [VADDR_BITS-1:0] dst_vaddr = l_base + {{(VADDR_BITS-28){1'b0}}, l_off};

always_comb begin
    rd_req = '0;
    rd_req.opcode = LOCAL_READ;
    rd_req.strm   = STRM_HOST;
    rd_req.pid    = l_src_pid;
    rd_req.vaddr  = l_payload[VADDR_BITS-1:0];
    rd_req.len    = l_len[LEN_BITS-1:0];
    rd_req.dest   = 0;
    rd_req.last   = 1'b1;
    rd_valid = (state == ST_RD_REQ);
end

always_comb begin
    wr_req = '0;
    wr_req.last = 1'b1;
    wr_req.dest = 0;

    if (state == ST_CP_REQ || state == ST_CP_DATA) begin
        wr_req.opcode = LOCAL_WRITE;
        wr_req.strm   = STRM_HOST;
        wr_req.pid    = compl_pid;
        wr_req.vaddr  = compl_va;
        wr_req.len    = 8;
    end else if (l_route) begin
        // rdma: RETH vaddr = exporter's VA (base + offset)
        wr_req.opcode = APP_WRITE;
        wr_req.strm   = STRM_RDMA;
        wr_req.mode   = 1'b0;        // RDMA_MODE_PARSE: shell fragments to PMTU
        wr_req.rdma   = 1'b1;
        wr_req.remote = 1'b1;
        wr_req.actv   = 1'b1;
        wr_req.pid    = l_pid;       // QP owner
        wr_req.vaddr  = dst_vaddr;
        wr_req.len    = l_is_desc ? l_len[LEN_BITS-1:0] : 'd8;
    end else begin
        wr_req.opcode = LOCAL_WRITE;
        wr_req.strm   = STRM_HOST;
        wr_req.pid    = l_pid;
        wr_req.vaddr  = dst_vaddr;
        wr_req.len    = l_is_desc ? l_len[LEN_BITS-1:0] : 'd8;
    end

    wr_valid = (state == ST_WR_REQ) || (state == ST_DMA_WR_REQ) || (state == ST_CP_REQ);
end

// -------------------------------------------------------------------------
// Data streams
// -------------------------------------------------------------------------
// Store/completion beat: data LSB-aligned, low 8 bytes valid (gate G2)
wire [AXI_DATA_BITS-1:0]   beat_data = {{(AXI_DATA_BITS-64){1'b0}},
                                        (state == ST_CP_DATA) ? (compl_cnt + 1) : l_payload};
wire [AXI_DATA_BITS/8-1:0] beat_keep = {{(AXI_DATA_BITS/8-8){1'b0}}, 8'hFF};

wire stream_local = (state == ST_STREAM) && !l_route;
wire stream_net   = (state == ST_STREAM) &&  l_route;

always_comb begin
    // Pull stream ready only while forwarding, from the selected output
    s_tready = (stream_local && m_host_tready) || (stream_net && m_net_tready);

    // Host output: store beat (local), DMA forward (local), completion beat
    m_host_tdata  = stream_local ? s_tdata : beat_data;
    m_host_tkeep  = stream_local ? s_tkeep : beat_keep;
    m_host_tlast  = stream_local ? s_tlast : 1'b1;
    m_host_tvalid = ((state == ST_WR_DATA) && !l_route) ||
                    (stream_local && s_tvalid) ||
                    (state == ST_CP_DATA);

    // Net output: store beat (rdma), DMA forward (rdma)
    m_net_tdata  = stream_net ? s_tdata : beat_data;
    m_net_tkeep  = stream_net ? s_tkeep : beat_keep;
    m_net_tlast  = stream_net ? s_tlast : 1'b1;
    m_net_tvalid = ((state == ST_WR_DATA) && l_route) ||
                   (stream_net && s_tvalid);
end

// -------------------------------------------------------------------------
// Counter pulses
// -------------------------------------------------------------------------
assign cnt_drop     = (state == ST_CHECK) && !ok;
assign cnt_local_wr = ((state == ST_WR_DATA) && !l_route && m_host_tready) ||
                      (stream_local && s_tvalid && s_tlast && m_host_tready);
assign cnt_rdma_wr  = ((state == ST_WR_DATA) && l_route && m_net_tready) ||
                      (stream_net && s_tvalid && s_tlast && m_net_tready);
assign cnt_compl    = (state == ST_CP_DATA) && m_host_tready;

endmodule
