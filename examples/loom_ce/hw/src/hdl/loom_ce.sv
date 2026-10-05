import lynxTypes::*;

/**
 * loom_ce
 *
 * The emulated accelerator's copy engine: one copy per START, then an
 * optional completion fence. A put (get = 0) copies from card memory (HBM)
 * to a destination VA, a get (get = 1) from a source VA into card memory.
 *
 *   1. sq_rd {LOCAL_READ, pid, src_va, len}: a put's source arrives on the
 *      card stream (STRM_CARD); a get's on the host stream (STRM_HOST):
 *      src_va is mapped by the V80's MMU onto the U280's uwin (an imported
 *      dma-buf), so these are peer-to-peer PCIe reads of the window, which
 *      loom_read answers
 *   2. sq_wr {LOCAL_WRITE, pid, dst_va, len}: a put's on the host stream,
 *      dst_va on the U280's uwin, so these are peer-to-peer PCIe writes into
 *      loom_ingress; a get's on the card stream, into HBM
 *   3. the read stream is forwarded to the write stream, len/64 beats; the
 *      count, not the stream's tlast, ends the copy (a read may come back
 *      as several tlast-terminated segments)
 *   4. a get then waits until every write it issued has completed (cq_wr),
 *      so BUSY, COPIES and the fence mean the data is in HBM
 *   5. if fence_va != 0: sq_wr {LOCAL_WRITE, STRM_HOST, 8 B, fence_va} + one
 *      beat with the incremented copy count (the copy-engine semaphore
 *      release). A put's follows the data on the same write stream, so it
 *      reaches the U280 behind it; loom_ingress keeps that order.
 *
 * len is a nonzero multiple of 64 (loom_ingress takes full lines and whole 8 B
 * words); src_va and dst_va are 64 B-aligned. Software guarantees both.
 */
module loom_ce (
    input  logic                        aclk,
    input  logic                        aresetn,

    // From loom_ce_ctrl
    input  logic                        start,
    input  logic [VADDR_BITS-1:0]       src_va,
    input  logic [VADDR_BITS-1:0]       dst_va,
    input  logic [LEN_BITS-1:0]         len,
    input  logic [PID_BITS-1:0]         pid,
    input  logic [VADDR_BITS-1:0]       fence_va,
    input  logic                        get,          // 1: source on the host stream, destination in card memory
    output logic                        busy,
    output logic [31:0]                 copies,
    output logic [63:0]                 cycles,

    // Requests
    output req_t                        rd_req,
    output logic                        rd_valid,
    input  logic                        rd_ready,
    output req_t                        wr_req,
    output logic                        wr_valid,
    input  logic                        wr_ready,
    input  logic                        wr_done,      // cq_wr.valid

    // Card stream in (axis_card_recv): a put's source
    input  logic [AXI_DATA_BITS-1:0]    s_tdata,
    input  logic [AXI_DATA_BITS/8-1:0]  s_tkeep,
    input  logic                        s_tvalid,
    output logic                        s_tready,

    // Host stream out (axis_host_send): a put's destination, the fence
    output logic [AXI_DATA_BITS-1:0]    m_tdata,
    output logic [AXI_DATA_BITS/8-1:0]  m_tkeep,
    output logic                        m_tvalid,
    input  logic                        m_tready,
    output logic                        m_tlast,

    // Host stream in (axis_host_recv): a get's source
    input  logic [AXI_DATA_BITS-1:0]    s_host_tdata,
    input  logic [AXI_DATA_BITS/8-1:0]  s_host_tkeep,
    input  logic                        s_host_tvalid,
    output logic                        s_host_tready,

    // Card stream out (axis_card_send): a get's destination
    output logic [AXI_DATA_BITS-1:0]    m_card_tdata,
    output logic [AXI_DATA_BITS/8-1:0]  m_card_tkeep,
    output logic                        m_card_tvalid,
    input  logic                        m_card_tready,
    output logic                        m_card_tlast,

    output logic                        cnt_in_wait   // copying, and no source data presented
);

typedef enum logic [2:0] { ST_IDLE, ST_RD_REQ, ST_WR_REQ, ST_STREAM, ST_WR_WAIT, ST_FN_REQ, ST_FN_DATA } state_t;
state_t state;

logic [VADDR_BITS-1:0] l_src, l_dst, l_fence;
logic [LEN_BITS-1:0]   l_len;
logic [PID_BITS-1:0]   l_pid;
logic                  l_get;
logic [LEN_BITS-7:0]   left;          // beats still to forward
logic                  timing;        // START seen, data write not yet completed
logic                  data_posted;   // the data write request has been taken
logic [7:0]            wr_out;        // write requests taken, not yet completed

// The copy's read and write streams
wire src_valid = l_get ? s_host_tvalid : s_tvalid;
wire dst_ready = l_get ? m_card_tready : m_tready;

wire last_beat = (left == 1);
wire beat      = (state == ST_STREAM) && src_valid && dst_ready;

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        state <= ST_IDLE;
        copies <= 0;
        cycles <= 0;
        timing <= 1'b0;
        data_posted <= 1'b0;
        wr_out <= 0;
        l_get <= 1'b0;
    end else begin
        wr_out <= wr_out + 8'(wr_valid && wr_ready) - 8'(wr_done);
        case (state)
            ST_IDLE: if (start) begin
                l_src   <= src_va;
                l_dst   <= dst_va;
                l_len   <= len;
                l_pid   <= pid;
                l_fence <= fence_va;
                l_get   <= get;
                left    <= len[LEN_BITS-1:6];
                state   <= ST_RD_REQ;
            end
            ST_RD_REQ: if (rd_ready) state <= ST_WR_REQ;
            ST_WR_REQ: if (wr_ready) state <= ST_STREAM;
            ST_STREAM: if (beat) begin
                left <= left - 1'b1;
                if (last_beat) begin
                    if (l_get) state <= ST_WR_WAIT;
                    else if (l_fence != 0) state <= ST_FN_REQ;
                    else begin
                        copies <= copies + 1;
                        state  <= ST_IDLE;
                    end
                end
            end
            // A get: every write taken has completed (this cycle's
            // completion counts), so the data is in card memory
            ST_WR_WAIT: if (wr_out == 0 || (wr_out == 1 && wr_done)) begin
                if (l_fence != 0) state <= ST_FN_REQ;
                else begin
                    copies <= copies + 1;
                    state  <= ST_IDLE;
                end
            end
            ST_FN_REQ:  if (wr_ready) state <= ST_FN_DATA;
            ST_FN_DATA: if (m_tready) begin
                copies <= copies + 1;
                state  <= ST_IDLE;
            end
            default: state <= ST_IDLE;
        endcase

        // Copy time: START to the data write's completion, which is the
        // first write completion after the data request (the fence is later)
        if (state == ST_IDLE && start) begin
            timing <= 1'b1;
            cycles <= 0;
            data_posted <= 1'b0;
        end else if (timing) begin
            cycles <= cycles + 1;
            if (state == ST_WR_REQ && wr_ready) data_posted <= 1'b1;
            if (data_posted && wr_done) timing <= 1'b0;
        end
    end
end

assign busy        = (state != ST_IDLE);
assign cnt_in_wait = (state == ST_STREAM) && !src_valid;

always_comb begin
    rd_req        = '0;
    rd_req.opcode = LOCAL_READ;
    rd_req.strm   = l_get ? STRM_HOST : STRM_CARD;
    rd_req.dest   = 0;
    rd_req.pid    = l_pid;
    rd_req.vaddr  = l_src;
    rd_req.len    = l_len;
    rd_req.last   = 1'b1;
    rd_valid      = (state == ST_RD_REQ);

    wr_req        = '0;
    wr_req.opcode = LOCAL_WRITE;
    wr_req.dest   = 0;
    wr_req.pid    = l_pid;
    wr_req.last   = 1'b1;
    if (state == ST_FN_REQ) begin
        wr_req.strm  = STRM_HOST;
        wr_req.vaddr = l_fence;
        wr_req.len   = 8;
    end else begin
        wr_req.strm  = l_get ? STRM_CARD : STRM_HOST;
        wr_req.vaddr = l_dst;
        wr_req.len   = l_len;
    end
    wr_valid      = (state == ST_WR_REQ) || (state == ST_FN_REQ);

    s_tready      = (state == ST_STREAM) && !l_get && m_tready;
    s_host_tready = (state == ST_STREAM) &&  l_get && m_card_tready;

    m_card_tdata  = s_host_tdata;
    m_card_tkeep  = s_host_tkeep;
    m_card_tlast  = last_beat;
    m_card_tvalid = (state == ST_STREAM) && l_get && s_host_tvalid;

    if (state == ST_FN_DATA) begin
        m_tdata  = {{(AXI_DATA_BITS-64){1'b0}}, 32'b0, copies + 32'd1};
        m_tkeep  = {{(AXI_DATA_BITS/8-8){1'b0}}, 8'hFF};
        m_tlast  = 1'b1;
        m_tvalid = 1'b1;
    end else begin
        m_tdata  = s_tdata;
        m_tkeep  = s_tkeep;
        m_tlast  = last_beat;
        m_tvalid = (state == ST_STREAM) && !l_get && s_tvalid;
    end
end

endmodule
