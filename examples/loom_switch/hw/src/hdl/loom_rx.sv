import lynxTypes::*;

/**
 * loom_rx (loom_switch)
 *
 * Receive side. Every incoming packet is self-describing: an RDMA
 * WRITE_ONLY whose RETH is a reference into THIS host's export table,
 * {export index [47:40], offset [39:0]} (loom_exports). There is no message
 * and no header to parse, so nothing waits for the end of a transfer:
 *
 *   bulk packet:  land its payload at export.base + offset under export.pid,
 *                 if offset + len <= export.len - else drop it, counted
 *   inline store: index INLINE_EXP, one 64 B beat {lane0 op WRITE_INLINE,
 *                 len 8; lane1 the word's reference; lane2 data} - land the
 *                 exact 8 B
 *   get request:  the same beat with op GET_REQ {lane1 the reference to read
 *                 here; lane2 the request word {len/64 [63:48], the
 *                 requester's return reference [47:0]}} - a job for loom_rd
 *                 (m_job), which reads and writes back. A request with no
 *                 export, out of bounds, len 0 or not 64 B-aligned (source or
 *                 return) is still a job, marked err: loom_rd answers it with
 *                 the error completion only. The beat waits while the job
 *                 queue is full.
 *
 * The stack announces every packet on rq_wr (its RETH address and payload
 * length) before the packet's beats arrive (axis_mux_user_rq passes a
 * request on as its data starts). rq_ready stays high - holding it backs up
 * the shell's request path, which once wedged a two-host run - so the
 * announcements queue here (RQ_DEPTH; they run ahead of the beats by what
 * the ingress FIFO in front of this module holds).
 *
 * A generator takes the announcements in order, looks each up, and POSTS ITS
 * WRITE AHEAD of its data, keeping up to RX_WR_OUTSTANDING packets posted
 * ahead of the one streaming: the shell always has the next descriptor,
 * which is what lets the host write path run at the wire's rate. The data
 * side follows the same order: it streams each posted packet's beats,
 * drains a dropped one's, and lands an inline store (whose target is in its
 * beat, so the generator waits for it - writes leave in stream order).
 *
 * Bulk writes go with last = 0: no completion writeback (a writeback per
 * packet costs the host write path ~28 cycles a packet) and no tlast; the
 * shell ends them by byte count, as it ends every FIRST/MIDDLE fragment
 * (axis_mux_host_sink counts beats; mmu_arbiter sets last on the DMA
 * request itself). The 8 B store keeps last = 1 and tlast, the form proven
 * on hardware.
 *
 * Beats nothing announced are swallowed and counted (cnt_rx_orphan): the
 * head of a packet the stack streamed and then dropped must not land.
 */
module loom_rx #(
    parameter integer RX_WR_OUTSTANDING = 8,
    parameter integer RQ_DEPTH          = 8192
) (
    input  logic                        aclk,
    input  logic                        aresetn,

    // Incoming write requests (rq_wr): one per packet
    input  req_t                        rq_req,
    input  logic                        rq_valid,
    output logic                        rq_ready,

    // Export table (loom_exports): a = the generator's lookup, b = an inline store's
    output logic [7:0]                  xa_idx,
    input  logic                        xa_hit,
    input  logic [PID_BITS-1:0]         xa_pid,
    input  logic [VADDR_BITS-1:0]       xa_base,
    input  logic [39:0]                 xa_len,
    output logic [7:0]                  xb_idx,
    input  logic                        xb_hit,
    input  logic [PID_BITS-1:0]         xb_pid,
    input  logic [VADDR_BITS-1:0]       xb_base,
    input  logic [39:0]                 xb_len,

    // Write requests out (to sq_wr via the top-level arbiter)
    output req_t                        wr_req,
    output logic                        wr_valid,
    input  logic                        wr_ready,

    // Payload in (the ingress FIFO in front of axis_rrsp_recv)
    input  logic [AXI_DATA_BITS-1:0]    s_tdata,
    input  logic [AXI_DATA_BITS/8-1:0]  s_tkeep,
    input  logic                        s_tvalid,
    output logic                        s_tready,
    /* verilator lint_off UNUSED */
    input  logic                        s_tlast,
    /* verilator lint_on UNUSED */

    // Get jobs (to loom_rd): read len bytes at va under pid, write them back
    // to ret, then cval to ret + len; err = answer with the error completion
    output logic                        m_job_valid,
    input  logic                        m_job_ready,
    output logic [PID_BITS-1:0]         m_job_pid,
    output logic [VADDR_BITS-1:0]       m_job_va,
    output logic [22:0]                 m_job_len,
    output logic [47:0]                 m_job_ret,
    output logic [63:0]                 m_job_cval,
    output logic                        m_job_err,

    // Payload out (axis_host_send[1])
    output logic [AXI_DATA_BITS-1:0]    m_tdata,
    output logic [AXI_DATA_BITS/8-1:0]  m_tkeep,
    output logic                        m_tvalid,
    input  logic                        m_tready,
    output logic                        m_tlast,

    output logic                        busy,

    // Counter pulses (to loom_ctrl)
    output logic                        cnt_rx_fwd,      // a packet's (or store's) write finished
    output logic                        cnt_rx_drop,     // a packet dropped: no export, out of bounds, bad inline
    output logic                        cnt_rx_orphan,   // a beat nothing announced, swallowed
    output logic                        cnt_rx_move,     // streaming: both ready
    output logic                        cnt_rx_starve,   // streaming: no beat
    output logic                        cnt_rx_stall,    // streaming: host write not ready
    output logic                        cnt_rx_bp,       // a beat offered and refused, any state
    output logic                        cnt_rx_req,      // an rq_wr taken
    output logic                        cnt_rx_pkt,      // a bulk packet's write posted
    output logic                        cnt_rx_store,    // an inline store's write posted
    output logic                        cnt_rx_post_wait,// a write presented, sq_wr not ready
    output logic                        cnt_rx_at_limit, // an announced packet held by RX_WR_OUTSTANDING
    output logic                        cnt_rx_pkt_wait, // a beat waited for its packet's write to be posted
    output logic                        cnt_rx_rq_ovfl   // an rq_wr arrived with the queue full (must be 0)
);

localparam [7:0] MSG_OP_WRITE_INLINE = 8'd2;   // keep in sync with loom_ingress.sv
localparam [7:0] MSG_OP_GET_REQ      = 8'd3;
localparam [7:0] INLINE_EXP          = 8'hFF;

localparam integer PLEN_W = 15;             // a packet's payload length (<= PMTU)
localparam integer BTS_W  = 8;              // a packet's beats
localparam integer OQ_W   = $clog2(RX_WR_OUTSTANDING + 1);

typedef enum logic [1:0] { K_WRITE, K_DROP, K_INLINE } kind_t;

function automatic logic [BTS_W-1:0] beats_of(input logic [PLEN_W-1:0] len);
    logic [PLEN_W:0] padded;
    padded   = {1'b0, len} + (PLEN_W+1)'(63);
    beats_of = BTS_W'(padded[PLEN_W:6]);
endfunction

// ---------------------------------------------------------------------------
// Announcements: {payload length, RETH address} per packet, in order
// ---------------------------------------------------------------------------
logic              e_full, e_empty, e_wbusy, e_rbusy, e_pop;
logic [63:0]       e_dout;
wire               e_valid = !e_empty && !e_rbusy;
wire [VADDR_BITS-1:0] e_addr = e_dout[VADDR_BITS-1:0];
wire [PLEN_W-1:0]  e_len   = e_dout[VADDR_BITS +: PLEN_W];
wire               e_push  = rq_valid && !e_full && !e_wbusy;

assign rq_ready = 1'b1;

xpm_fifo_sync #(
    .FIFO_MEMORY_TYPE("block"),
    .FIFO_WRITE_DEPTH(RQ_DEPTH),
    .WRITE_DATA_WIDTH(64),
    .READ_DATA_WIDTH(64),
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
) inst_rq_fifo (
    .sleep(1'b0), .rst(!aresetn), .wr_clk(aclk),
    .wr_en(e_push),
    .din({{(64-VADDR_BITS-PLEN_W){1'b0}}, PLEN_W'(rq_req.len), rq_req.vaddr}),
    .full(e_full), .overflow(), .wr_rst_busy(e_wbusy),
    .rd_en(e_pop), .dout(e_dout), .empty(e_empty), .underflow(), .rd_rst_busy(e_rbusy),
    .prog_full(), .wr_data_count(), .prog_empty(), .rd_data_count(),
    .almost_full(), .almost_empty(), .data_valid(), .wr_ack(),
    .injectsbiterr(1'b0), .injectdbiterr(1'b0), .sbiterr(), .dbiterr()
);

// Announced and not yet looked up. The FIFO's output lags a push by a few
// cycles; this count does not, so a beat that follows its rq_wr closely
// waits for it instead of being taken for an orphan.
logic [$clog2(RQ_DEPTH):0] e_cnt;
always_ff @(posedge aclk) begin
    if (!aresetn) e_cnt <= '0;
    else          e_cnt <= e_cnt + ($clog2(RQ_DEPTH)+1)'(e_push) - ($clog2(RQ_DEPTH)+1)'(e_pop);
end

// ---------------------------------------------------------------------------
// Generator: one registered lookup stage (L), then the post
// ---------------------------------------------------------------------------
logic                  l_valid;
kind_t                 l_kind;
logic [PID_BITS-1:0]   l_pid;
logic [VADDR_BITS-1:0] l_va;
logic [PLEN_W-1:0]     l_len;
logic                  g_block;      // an inline store is between L and its landing

// Posted-ahead packets, for the data side: {kind, beats}
logic [BTS_W+1:0]      oq_mem [RX_WR_OUTSTANDING];
logic [OQ_W-1:0]       oq_cnt;
logic [$clog2(RX_WR_OUTSTANDING)-1:0] oq_wp, oq_rp;
wire                   oq_empty = (oq_cnt == '0);
wire                   oq_full  = (oq_cnt == OQ_W'(RX_WR_OUTSTANDING));

// The next announcement, looked up
wire [39:0]  e_off   = e_addr[39:0];
assign xa_idx = e_addr[47:40];
wire [40:0]  e_end   = {1'b0, e_off} + 41'(e_len);
wire e_inline = (xa_idx == INLINE_EXP) && (e_len == PLEN_W'(64));
wire e_ok     = xa_hit && (e_len != '0) && (e_len[5:0] == 6'b0) && (e_end <= {1'b0, xa_len});

// L posts (a write) or passes (a drop, an inline) when the oq has room
wire l_post  = l_valid && (l_kind == K_WRITE) && !oq_full;
wire l_pass  = l_valid && (l_kind != K_WRITE) && !oq_full;
wire l_done  = (l_post && wr_ready) || l_pass;
wire l_load  = e_valid && !g_block && (!l_valid || l_done) &&
               !(l_pass && (l_kind == K_INLINE));          // nothing after an inline until it lands
assign e_pop = l_load;

// ---------------------------------------------------------------------------
// Data side
// ---------------------------------------------------------------------------
typedef enum logic [2:0] { D_IDLE, D_WRITE, D_DROP, D_INL_HDR, D_INL_DATA } dstate_t;
dstate_t dstate;
logic [BTS_W-1:0]      d_left;
logic [63:0]           d_inline;

wire oq_take = (dstate == D_IDLE) && !oq_empty;
wire [BTS_W-1:0] oq_beats = oq_mem[oq_rp][BTS_W-1:0];
wire [1:0]       oq_kind  = oq_mem[oq_rp][BTS_W +: 2];

// An inline store's header, on the stream
wire [7:0]       i_op  = s_tdata[7:0];
wire [27:0]      i_len = s_tdata[35:8];
wire [VADDR_BITS-1:0] i_ref = s_tdata[64 +: VADDR_BITS];
assign xb_idx = i_ref[47:40];
wire [40:0]      i_end = {1'b0, i_ref[39:0]} + 41'd8;
wire i_ok = (s_tdata[63:36] == '0) && (i_op == MSG_OP_WRITE_INLINE) && (i_len == 28'd8) &&
            (i_ref[2:0] == 3'b0) && xb_hit && (i_end <= {1'b0, xb_len});
wire i_here = (dstate == D_INL_HDR) && s_tvalid;
wire i_post = i_here && i_ok;

// A get request: lane1 = i_ref, the source here; lane2 the request word
wire        g_is   = (s_tdata[63:36] == '0) && (i_op == MSG_OP_GET_REQ) && (i_len == 28'd8);
wire [63:0] g_word = s_tdata[128 +: 64];
wire [22:0] g_len  = {g_word[63:48], 6'b0};
wire [40:0] g_end  = {1'b0, i_ref[39:0]} + 41'(g_len);
wire g_ok   = xb_hit && (g_word[63:48] != 16'd0) && (i_ref[5:0] == 6'b0) &&
              (g_word[5:0] == 6'b0) && (g_end <= {1'b0, xb_len});

assign m_job_valid = i_here && g_is;
assign m_job_pid   = xb_pid;
assign m_job_va    = xb_base + VADDR_BITS'(i_ref[39:0]);
assign m_job_len   = g_len;
assign m_job_ret   = g_word[47:0];
assign m_job_cval  = g_word;
assign m_job_err   = !g_ok;

// The write: the generator's, or (while it waits) an inline store's
assign wr_valid = l_post || i_post;
always_comb begin
    wr_req        = '0;
    wr_req.opcode = LOCAL_WRITE;
    wr_req.strm   = STRM_HOST;
    wr_req.dest   = 1;            // this module's own host stream (axis_host_send[1])
    if (i_post) begin
        wr_req.pid   = xb_pid;
        wr_req.vaddr = xb_base + VADDR_BITS'(i_ref[39:0]);
        wr_req.len   = LEN_BITS'(8);
        wr_req.last  = 1'b1;
    end else begin
        wr_req.pid   = l_pid;
        wr_req.vaddr = l_va;
        wr_req.len   = LEN_BITS'(l_len);
        wr_req.last  = 1'b0;
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        l_valid <= 1'b0;
        g_block <= 1'b0;
        oq_cnt  <= '0; oq_wp <= '0; oq_rp <= '0;
        dstate  <= D_IDLE;
        d_left  <= '0;
    end else begin
        // L
        if (l_load) begin
            l_valid <= 1'b1;
            l_kind  <= e_inline ? K_INLINE : (e_ok ? K_WRITE : K_DROP);
            l_pid   <= xa_pid;
            l_va    <= xa_base + VADDR_BITS'(e_off);
            l_len   <= e_len;
        end else if (l_done) begin
            l_valid <= 1'b0;
        end
        if (l_pass && (l_kind == K_INLINE)) g_block <= 1'b1;

        // posted-ahead queue
        if (l_done) begin
            oq_mem[oq_wp] <= {l_kind, (l_kind == K_INLINE) ? BTS_W'(1) : beats_of(l_len)};
            oq_wp <= (oq_wp == RX_WR_OUTSTANDING - 1) ? '0 : oq_wp + 1'b1;
        end
        if (oq_take) oq_rp <= (oq_rp == RX_WR_OUTSTANDING - 1) ? '0 : oq_rp + 1'b1;
        oq_cnt <= oq_cnt + OQ_W'(l_done) - OQ_W'(oq_take);

        // data side
        case (dstate)
            D_IDLE: if (oq_take) begin
                d_left <= oq_beats;
                case (kind_t'(oq_kind))
                    K_WRITE:  dstate <= D_WRITE;
                    K_DROP:   dstate <= D_DROP;
                    default:  dstate <= D_INL_HDR;
                endcase
            end
            D_WRITE: if (s_tvalid && m_tready) begin
                d_left <= d_left - 1'b1;
                if (d_left == BTS_W'(1)) dstate <= D_IDLE;
            end
            D_DROP: if (s_tvalid) begin
                d_left <= d_left - 1'b1;
                if (d_left == BTS_W'(1)) dstate <= D_IDLE;
            end
            // an inline store: post its 8 B write (or drop it), then the
            // beat; a get request: hand it to loom_rd
            D_INL_HDR: if (i_here) begin
                d_inline <= s_tdata[128 +: 64];
                if (g_is)          begin if (m_job_ready) begin dstate <= D_IDLE; g_block <= 1'b0; end end
                else if (!i_ok)    begin dstate <= D_IDLE; g_block <= 1'b0; end
                else if (wr_ready) dstate <= D_INL_DATA;
            end
            D_INL_DATA: if (m_tready) begin
                dstate  <= D_IDLE;
                g_block <= 1'b0;
            end
            default: dstate <= D_IDLE;
        endcase
    end
end

// Nothing announced, posted or streaming: a beat now is an orphan
wire nothing = (dstate == D_IDLE) && oq_empty && !l_valid && (e_cnt == '0);

always_comb begin
    s_tready = ((dstate == D_WRITE) && m_tready) ||
               (dstate == D_DROP) ||
               ((dstate == D_INL_HDR) && (g_is ? m_job_ready : (!i_ok || wr_ready))) ||
               nothing;

    if (dstate == D_INL_DATA) begin
        // constructed beat: the exact 8 B, LSB-aligned
        m_tdata  = {{(AXI_DATA_BITS-64){1'b0}}, d_inline};
        m_tkeep  = {{(AXI_DATA_BITS/8-8){1'b0}}, 8'hFF};
        m_tlast  = 1'b1;
        m_tvalid = 1'b1;
    end else begin
        m_tdata  = s_tdata;
        m_tkeep  = s_tkeep;
        m_tlast  = 1'b0;          // bulk writes end by byte count (last = 0)
        m_tvalid = (dstate == D_WRITE) && s_tvalid;
    end
end

assign busy = !nothing;

wire w_end = (dstate == D_WRITE) && s_tvalid && m_tready && (d_left == BTS_W'(1));
assign cnt_rx_fwd       = w_end || ((dstate == D_INL_DATA) && m_tready);
assign cnt_rx_drop      = (l_pass && (l_kind == K_DROP)) || (i_here && !i_ok && !g_is);
assign cnt_rx_orphan    = nothing && s_tvalid;
assign cnt_rx_move      = (dstate == D_WRITE) &&  s_tvalid &&  m_tready;
assign cnt_rx_starve    = (dstate == D_WRITE) && !s_tvalid;
assign cnt_rx_stall     = (dstate == D_WRITE) &&  s_tvalid && !m_tready;
assign cnt_rx_bp        = s_tvalid && !s_tready;
assign cnt_rx_req       = e_push;
assign cnt_rx_pkt       = l_post && wr_ready;
assign cnt_rx_store     = i_post && wr_ready;
assign cnt_rx_post_wait = wr_valid && !wr_ready;
assign cnt_rx_at_limit  = l_valid && oq_full;
assign cnt_rx_pkt_wait  = (dstate == D_IDLE) && oq_empty && s_tvalid && !nothing;
assign cnt_rx_rq_ovfl   = rq_valid && (e_full || e_wbusy);

endmodule
