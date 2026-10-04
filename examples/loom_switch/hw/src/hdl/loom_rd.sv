import lynxTypes::*;

/**
 * loom_rd (loom_switch)
 *
 * Get responder: answers the get requests loom_rx hands over (one job per
 * request, in arrival order). A get is two writes on the wire, so nothing
 * here needs the RoCE stack's READ: the requester stored a request word into
 * a get window, loom_rx validated it against this host's exports, and this
 * module
 *
 *   1. reads len bytes at va under pid from local memory (sq_rd, host
 *      stream RD_DEST), up to RD_AHEAD jobs ahead of the one being sent;
 *   2. writes them back as self-describing RDMA writes, PMTU each, RETH =
 *      the requester's return reference + offset, exactly as the ingress
 *      sends a copy (the requester's loom_rx lands them in its export);
 *   3. then one inline store of the request word to return reference + len:
 *      the requester's completion. An err job (a request loom_rx rejected)
 *      gets only that store, with all ones instead of the word (a valid
 *      request word is never all ones: its return export would be 0xFF).
 *
 * A packet is posted only when all its beats are buffered here (BUF_BEATS),
 * so a response never holds the shared payload stream waiting on a host
 * read. Packets and the completion are posted under the ack window (win_ok)
 * on the QP owned by qp_pid, through the same sq_wr and payload stream as
 * the ingress; the top level keeps the payload in request order.
 *
 * len is a multiple of 64 (loom_rx checks): the read data arrives as len/64
 * full beats, which is what is counted.
 */
module loom_rd #(
    parameter integer NET_DEST  = 0,
    parameter integer RD_DEST   = 1,
    parameter integer JOB_DEPTH = 512,
    parameter integer RD_AHEAD  = 4,
    parameter integer BUF_BEATS = 128
) (
    input  logic                        aclk,
    input  logic                        aresetn,

    // Jobs (from loom_rx)
    input  logic                        s_job_valid,
    output logic                        s_job_ready,
    input  logic [PID_BITS-1:0]         s_job_pid,
    input  logic [VADDR_BITS-1:0]       s_job_va,
    input  logic [22:0]                 s_job_len,
    input  logic [47:0]                 s_job_ret,
    input  logic [63:0]                 s_job_cval,
    input  logic                        s_job_err,

    // The QP owner the responses go out on (CSR RD_CTL)
    input  logic [PID_BITS-1:0]         qp_pid,

    // Host reads (sq_rd) and their data (axis_host_recv[RD_DEST])
    output req_t                        rd_req,
    output logic                        rd_valid,
    input  logic                        rd_ready,
    input  logic [AXI_DATA_BITS-1:0]    s_tdata,
    input  logic                        s_tvalid,
    output logic                        s_tready,

    // sq_wr (through the top-level arbiter) and the ack window
    output req_t                        wr_req,
    output logic                        wr_valid,
    input  logic                        wr_ready,
    input  logic                        win_ok,
    output logic                        rdma_post,

    // Payload out (axis_rreq_send[NET_DEST], through the top-level order mux)
    output logic [AXI_DATA_BITS-1:0]    m_tdata,
    output logic [AXI_DATA_BITS/8-1:0]  m_tkeep,
    output logic                        m_tvalid,
    input  logic                        m_tready,
    output logic                        m_tlast,

    output logic                        busy,

    // Counter pulses
    output logic                        cnt_job,     // a job taken (a read issued, or an err job)
    output logic                        cnt_err,     // an error completion sent
    output logic                        cnt_pkt,     // a response packet posted
    output logic                        cnt_cmp,     // a completion sent (err ones included)
    output logic                        cnt_wait,    // a request waited on the window or sq_wr
    output logic                        cnt_starve   // a packet waited for its read data
);

localparam [7:0] MSG_OP_WRITE_INLINE = 8'd2;   // keep in sync with loom_ingress.sv / loom_rx.sv
localparam [7:0] INLINE_EXP          = 8'hFF;
localparam integer PKT_BEATS = PMTU_BYTES / 64;
localparam integer JW        = PID_BITS + VADDR_BITS + 23 + 48 + 64 + 1;
localparam integer CW        = $clog2(BUF_BEATS + 1);

typedef struct packed {
    logic [PID_BITS-1:0]   pid;
    logic [VADDR_BITS-1:0] va;
    logic [22:0]           len;
    logic [47:0]           ret;
    logic [63:0]           cval;
    logic                  err;
} job_t;

// ---------------------------------------------------------------------------
// Job queue
// ---------------------------------------------------------------------------
logic jq_full, jq_empty, jq_wbusy, jq_rbusy, jq_pop;
job_t jq_out, jq_in;

assign jq_in = '{pid: s_job_pid, va: s_job_va, len: s_job_len, ret: s_job_ret,
                 cval: s_job_cval, err: s_job_err};
assign s_job_ready = !jq_full && !jq_wbusy;

xpm_fifo_sync #(
    .FIFO_MEMORY_TYPE("block"),
    .FIFO_WRITE_DEPTH(JOB_DEPTH),
    .WRITE_DATA_WIDTH(JW),
    .READ_DATA_WIDTH(JW),
    .READ_MODE("fwft"),
    .FIFO_READ_LATENCY(0),
    .USE_ADV_FEATURES("0000"),
    .ECC_MODE("no_ecc"),
    .DOUT_RESET_VALUE("0"),
    .FULL_RESET_VALUE(0),
    .PROG_FULL_THRESH(10),
    .PROG_EMPTY_THRESH(10),
    .RD_DATA_COUNT_WIDTH(1),
    .WR_DATA_COUNT_WIDTH(1),
    .WAKEUP_TIME(0),
    .SIM_ASSERT_CHK(0)
) inst_job_fifo (
    .sleep(1'b0), .rst(!aresetn), .wr_clk(aclk),
    .wr_en(s_job_valid && s_job_ready), .din(jq_in),
    .full(jq_full), .overflow(), .wr_rst_busy(jq_wbusy),
    .rd_en(jq_pop), .dout(jq_out), .empty(jq_empty), .underflow(), .rd_rst_busy(jq_rbusy),
    .prog_full(), .wr_data_count(), .prog_empty(), .rd_data_count(),
    .almost_full(), .almost_empty(), .data_valid(), .wr_ack(),
    .injectsbiterr(1'b0), .injectdbiterr(1'b0), .sbiterr(), .dbiterr()
);

// ---------------------------------------------------------------------------
// Stage A: take a job, issue its read (an err job has none), queue it for
// the sender - up to RD_AHEAD jobs ahead
// ---------------------------------------------------------------------------
job_t aq_mem [RD_AHEAD];
logic [$clog2(RD_AHEAD):0] aq_wp, aq_rp;
wire  aq_full  = (aq_wp - aq_rp) == ($clog2(RD_AHEAD)+1)'(RD_AHEAD);
wire  aq_empty = (aq_wp == aq_rp);
job_t aq_head;
assign aq_head = aq_mem[aq_rp[$clog2(RD_AHEAD)-1:0]];

wire  j_avail = !jq_empty && !jq_rbusy && !aq_full;
assign rd_valid = j_avail && !jq_out.err;
assign jq_pop   = j_avail && (jq_out.err || rd_ready);

always_comb begin
    rd_req        = '0;
    rd_req.opcode = LOCAL_READ;
    rd_req.strm   = STRM_HOST;
    rd_req.dest   = RD_DEST;
    rd_req.pid    = jq_out.pid;
    rd_req.vaddr  = jq_out.va;
    rd_req.len    = LEN_BITS'(jq_out.len);
    rd_req.last   = 1'b1;
end

logic aq_pop;
always_ff @(posedge aclk) begin
    if (!aresetn) begin
        aq_wp <= '0;
        aq_rp <= '0;
    end else begin
        if (jq_pop) begin
            aq_mem[aq_wp[$clog2(RD_AHEAD)-1:0]] <= jq_out;
            aq_wp <= aq_wp + 1'b1;
        end
        if (aq_pop) aq_rp <= aq_rp + 1'b1;
    end
end

// ---------------------------------------------------------------------------
// Read data buffer: the reads' beats, in order, across jobs
// ---------------------------------------------------------------------------
logic [AXI_DATA_BITS-1:0] bf_tdata;
logic bf_tvalid, bf_tready;
logic [CW-1:0] bf_cnt;

xpm_fifo_axis #(
    .CLOCKING_MODE("common_clock"),
    .FIFO_MEMORY_TYPE("block"),
    .PACKET_FIFO("false"),
    .FIFO_DEPTH(BUF_BEATS),
    .TDATA_WIDTH(AXI_DATA_BITS),
    .USE_ADV_FEATURES("0000")
) inst_buf (
    .s_aresetn(aresetn), .s_aclk(aclk), .m_aclk(aclk),
    .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready),
    .s_axis_tdata(s_tdata), .s_axis_tstrb('0), .s_axis_tkeep('1),
    .s_axis_tlast(1'b0), .s_axis_tid('0), .s_axis_tdest('0), .s_axis_tuser('0),
    .m_axis_tvalid(bf_tvalid), .m_axis_tready(bf_tready),
    .m_axis_tdata(bf_tdata), .m_axis_tstrb(), .m_axis_tkeep(),
    .m_axis_tlast(), .m_axis_tid(), .m_axis_tdest(), .m_axis_tuser(),
    .prog_full_axis(), .wr_data_count_axis(), .almost_full_axis(),
    .prog_empty_axis(), .rd_data_count_axis(), .almost_empty_axis(),
    .injectsbiterr_axis(1'b0), .injectdbiterr_axis(1'b0),
    .sbiterr_axis(), .dbiterr_axis()
);

wire bf_push = s_tvalid && s_tready;
wire bf_pop  = bf_tvalid && bf_tready;
always_ff @(posedge aclk) begin
    if (!aresetn) bf_cnt <= '0;
    else          bf_cnt <= bf_cnt + CW'(bf_push) - CW'(bf_pop);
end

// ---------------------------------------------------------------------------
// Stage B: the head job's packets, then its completion
// ---------------------------------------------------------------------------
typedef enum logic [2:0] { S_IDLE, S_WAIT, S_REQ, S_DATA, S_CREQ, S_CDATA } sstate_t;
sstate_t st;
job_t    cur;
logic [22:0] off;              // bytes sent of cur
logic [6:0]  d_left;           // beats left in the packet being sent

wire [22:0] left   = cur.len - off;
wire [12:0] plen   = (left > 23'(PMTU_BYTES)) ? 13'(PMTU_BYTES) : 13'(left);
wire [6:0]  pbeats = 7'(plen >> 6);

assign aq_pop = (st == S_IDLE) && !aq_empty;

always_ff @(posedge aclk) begin
    if (!aresetn) st <= S_IDLE;
    else case (st)
        S_IDLE: if (!aq_empty) begin
            cur <= aq_head;
            off <= '0;
            st  <= aq_head.err ? S_CREQ : S_WAIT;
        end
        S_WAIT: if (bf_cnt >= CW'(pbeats)) st <= S_REQ;
        S_REQ: if (wr_valid && wr_ready) begin
            d_left <= pbeats;
            st     <= S_DATA;
        end
        S_DATA: if (bf_pop) begin
            d_left <= d_left - 1'b1;
            if (d_left == 7'd1) begin
                off <= off + 23'(plen);
                st  <= (left == 23'(plen)) ? S_CREQ : S_WAIT;
            end
        end
        S_CREQ: if (wr_valid && wr_ready) st <= S_CDATA;
        S_CDATA: if (m_tready) st <= S_IDLE;
        default: st <= S_IDLE;
    endcase
end

always_comb begin
    wr_req        = '0;
    wr_req.opcode = RC_RDMA_WRITE_ONLY;
    wr_req.strm   = STRM_RDMA;
    wr_req.mode   = 1'b1;              // RDMA_MODE_RAW: one request, one packet
    wr_req.rdma   = 1'b1;
    wr_req.remote = 1'b1;
    wr_req.actv   = 1'b1;
    wr_req.dest   = NET_DEST;
    wr_req.pid    = qp_pid;
    wr_req.last   = 1'b1;
    if (st == S_CREQ) begin
        wr_req.vaddr = {INLINE_EXP, 40'd0};
        wr_req.len   = LEN_BITS'(64);
    end else begin
        wr_req.vaddr = VADDR_BITS'(cur.ret) + VADDR_BITS'(off);
        wr_req.len   = LEN_BITS'(plen);
    end
end
assign wr_valid  = ((st == S_REQ) || (st == S_CREQ)) && win_ok;
assign rdma_post = wr_valid && wr_ready;

// Completion: an inline store {lane0 op WRITE_INLINE, len 8; lane1 return
// reference + len; lane2 the request word, or all ones for an err job}
wire [63:0] c_q0 = {28'd0, 28'd8, MSG_OP_WRITE_INLINE};
wire [63:0] c_q1 = {16'd0, cur.ret + 48'(cur.len)};
wire [63:0] c_q2 = cur.err ? 64'hFFFF_FFFF_FFFF_FFFF : cur.cval;

assign bf_tready = (st == S_DATA) && m_tready;

always_comb begin
    m_tdata  = (st == S_CDATA) ? {{(AXI_DATA_BITS-192){1'b0}}, c_q2, c_q1, c_q0} : bf_tdata;
    m_tkeep  = '1;
    m_tlast  = (st == S_CDATA) || (d_left == 7'd1);
    m_tvalid = (st == S_CDATA) || ((st == S_DATA) && bf_tvalid);
end

assign busy = !jq_empty || !aq_empty || (st != S_IDLE) || (bf_cnt != '0);

assign cnt_job    = jq_pop;
assign cnt_err    = (st == S_CDATA) && m_tready && cur.err;
assign cnt_pkt    = (st == S_REQ) && rdma_post;
assign cnt_cmp    = (st == S_CDATA) && m_tready;
assign cnt_wait   = ((st == S_REQ) || (st == S_CREQ)) && !(wr_valid && wr_ready);
assign cnt_starve = (st == S_WAIT) && (bf_cnt < CW'(pbeats));

endmodule
