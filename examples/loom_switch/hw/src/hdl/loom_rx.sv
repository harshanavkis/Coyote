import lynxTypes::*;

/**
 * loom_rx (loom_switch)
 *
 * Receive side. EVERY incoming transaction is a Loom message and its
 * destination comes from the message header - never from rq_wr.vaddr, which
 * names the sender's staging buffer. A header this module does not recognize
 * never becomes a write: it is dropped and counted, with the rest of its
 * message.
 *
 * Message header (first beat of a message's first packet):
 *
 *   lane0 = {reserved, dst_pid, len[27:0], op[7:0]}   (keep in sync with
 *   lane1 = target VA                                   loom_ingress)
 *   lane2 = inline data (op WRITE_INLINE only)
 *
 *   op 2 WRITE_INLINE: a single-beat message (WRITE_ONLY, 64 B); land the
 *                      exact 8 B (len) write of lane2 at lane1.
 *   op 3 STREAM:       len 0 - the sender does not know it. The payload
 *                      follows the header in this packet and in every packet
 *                      of the message after it; the message ends at the
 *                      packet whose rq_wr has last set (the stack raises it
 *                      on RC_RDMA_WRITE_LAST/ONLY, ib_transport_protocol.cpp).
 *
 * The stack announces every packet on rq_wr - its wire length (header
 * included) and last - before the packet's beats arrive (axis_mux_user_rq
 * passes a request on as its data starts). rq_ready stays high: holding it
 * backs up the shell's request path, which once wedged a two-host run. So
 * the announcements queue here (RQ_DEPTH; they can run ahead by what the
 * ingress FIFO in front of this module holds) and are consumed in order:
 *
 *   - a message's first packet: its write is posted from the header beat
 *     {LOCAL_WRITE, dst_pid, target VA, len - 64};
 *   - every later packet: the generator posts its write AHEAD of its data,
 *     at the running target, as soon as it is announced, keeping up to
 *     RX_WR_OUTSTANDING posted ahead of the packet being streamed. The shell
 *     queues descriptors in order, so the payload lands against them as is.
 *
 * One write per packet, posted ahead: the shell always has the next
 * descriptor, which is what lets the host write path run at the wire's rate
 * (one write in flight at a time capped it at ~10 GB/s, and a message per
 * packet - the old ingress framing - forced exactly that).
 *
 * last (the completion writeback, cnfg_slave meta_done_wr -> wback[1]) is set
 * only on a message's final write, as perf_rdma's receiver does: a writeback
 * per packet costs the host write path ~28 cycles a packet. A last=0 write is
 * ended by its byte count and its stream carries no tlast.
 *
 * Beats with no announcement (none queued) are swallowed and counted
 * (cnt_rx_orphan): the head of a packet the stack streamed and then dropped
 * must not land.
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

    // Payload out (axis_host_send[1])
    output logic [AXI_DATA_BITS-1:0]    m_tdata,
    output logic [AXI_DATA_BITS/8-1:0]  m_tkeep,
    output logic                        m_tvalid,
    input  logic                        m_tready,
    output logic                        m_tlast,

    // Arbitration
    output logic                        req,       // wants the shared path
    input  logic                        grant,     // exclusive ownership while high
    output logic                        busy,

    // Counter pulses (to loom_ctrl)
    output logic                        cnt_rx_fwd,      // a packet's write finished
    output logic                        cnt_rx_drop,     // a message dropped (bad header)
    output logic                        cnt_rx_orphan,   // a beat nothing announced, swallowed
    output logic                        cnt_rx_move,     // streaming: both ready
    output logic                        cnt_rx_starve,   // streaming: no beat
    output logic                        cnt_rx_stall,    // streaming: host write not ready
    output logic                        cnt_rx_bp,       // a beat offered and refused, any state
    output logic                        cnt_rx_req,      // an rq_wr taken
    output logic                        cnt_rx_msg,      // a STREAM message started
    output logic                        cnt_rx_msg_end,  // a message's final beat landed
    output logic                        cnt_rx_post,     // a write posted
    output logic                        cnt_rx_post_wait,// a write presented, sq_wr not ready
    output logic                        cnt_rx_at_limit, // an announced packet held by RX_WR_OUTSTANDING
    output logic                        cnt_rx_pkt_wait, // its beats waiting for the next write's post
    output logic                        cnt_rx_rq_ovfl   // an rq_wr arrived with the queue full (must be 0)
);

// Wire-message header ops (keep in sync with loom_ingress.sv)
localparam [7:0] MSG_OP_WRITE_INLINE = 8'd2;
localparam [7:0] MSG_OP_STREAM       = 8'd3;

localparam integer PLEN_W = 15;             // a packet's wire length (<= PMTU)
localparam integer BTS_W  = 8;              // a packet's beats
localparam integer OQ_W   = $clog2(RX_WR_OUTSTANDING + 1);

// ---------------------------------------------------------------------------
// Announcements: {last, wire length} per packet, in order
// ---------------------------------------------------------------------------
logic              e_full, e_empty, e_wbusy, e_rbusy, e_pop;
logic [15:0]       e_dout;
wire               e_valid = !e_empty && !e_rbusy;
wire [PLEN_W-1:0]  e_len   = e_dout[PLEN_W-1:0];
wire               e_last  = e_dout[15];
wire               e_push  = rq_valid && !e_full && !e_wbusy;

assign rq_ready = 1'b1;

// Announced and not yet consumed. The FIFO's output lags a push by a few
// cycles; this count does not, so a beat that follows its rq_wr closely
// waits for it instead of being taken for an orphan.
logic [$clog2(RQ_DEPTH):0] e_cnt;
wire announced = (e_cnt != '0);
always_ff @(posedge aclk) begin
    if (!aresetn) e_cnt <= '0;
    else          e_cnt <= e_cnt + ($clog2(RQ_DEPTH)+1)'(e_push) - ($clog2(RQ_DEPTH)+1)'(e_pop);
end

xpm_fifo_sync #(
    .FIFO_MEMORY_TYPE("block"),
    .FIFO_WRITE_DEPTH(RQ_DEPTH),
    .WRITE_DATA_WIDTH(16),
    .READ_DATA_WIDTH(16),
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
    .wr_en(e_push), .din({rq_req.last, PLEN_W'(rq_req.len)}),
    .full(e_full), .overflow(), .wr_rst_busy(e_wbusy),
    .rd_en(e_pop), .dout(e_dout), .empty(e_empty), .underflow(), .rd_rst_busy(e_rbusy),
    .prog_full(), .wr_data_count(), .prog_empty(), .rd_data_count(),
    .almost_full(), .almost_empty(), .data_valid(), .wr_ack(),
    .injectsbiterr(1'b0), .injectdbiterr(1'b0), .sbiterr(), .dbiterr()
);

function automatic logic [BTS_W-1:0] beats_of(input logic [PLEN_W-1:0] len);
    logic [PLEN_W:0] padded;
    padded   = {1'b0, len} + (PLEN_W+1)'(63);
    beats_of = BTS_W'(padded[PLEN_W:6]);
endfunction

// ---------------------------------------------------------------------------
// Header contract
//   inline: lane0 == {0, dst_pid, 28'd8, op2}, target 8 B aligned, a 64 B packet
//   stream: lane0 == {0, dst_pid, 28'd0, op3}, target 64 B aligned, the packet
//           carries payload after the header
// dst_pid is the address space the bytes land in (the exporter's ctid on THIS
// host); trusted from the wire, like the target VA.
// ---------------------------------------------------------------------------
wire [27:0]         hdr_len = s_tdata[35:8];
wire [7:0]          hdr_op  = s_tdata[7:0];
wire [PID_BITS-1:0] hdr_pid = s_tdata[36 +: PID_BITS];
wire [VADDR_BITS-1:0] hdr_va = s_tdata[64 +: VADDR_BITS];
wire hdr_rsvd_ok = (s_tdata[63:36+PID_BITS] == '0);
wire hdr_inline  = hdr_rsvd_ok && (hdr_op == MSG_OP_WRITE_INLINE) && (hdr_len == 28'd8) &&
                   (hdr_va[2:0] == 3'b0) && (e_len == PLEN_W'(64));
wire hdr_stream  = hdr_rsvd_ok && (hdr_op == MSG_OP_STREAM) && (hdr_len == 28'd0) &&
                   (hdr_va[5:0] == 6'b0) && (e_len > PLEN_W'(64)) && (e_len[5:0] == 6'b0);

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
typedef enum logic [2:0] {
    ST_IDLE, ST_INLINE_DATA, ST_STREAM, ST_PKT_WAIT, ST_DROP
} state_t;
state_t state;

logic [PID_BITS-1:0]   l_pid;        // the message's landing address space
logic [63:0]           l_inline;
logic [BTS_W-1:0]      d_beats;      // beats of the packet being streamed still to come
logic                  d_last;       // that packet is the message's last
logic                  g_open;       // the message has packets not yet posted
logic [VADDR_BITS-1:0] g_va;         // where the next posted write lands
logic                  x_last;       // ST_DROP: the packet being drained ends its message

// Posted-ahead writes: {beats, last} of each, in order, for the data side
logic [BTS_W:0]        oq_mem [RX_WR_OUTSTANDING];
logic [OQ_W-1:0]       oq_cnt;
logic [$clog2(RX_WR_OUTSTANDING)-1:0] oq_wp, oq_rp;
wire                   oq_empty = (oq_cnt == '0);
wire                   oq_full  = (oq_cnt == OQ_W'(RX_WR_OUTSTANDING));
wire [BTS_W-1:0]       oq_beats = oq_mem[oq_rp][BTS_W-1:0];
wire                   oq_last  = oq_mem[oq_rp][BTS_W];

// The header beat of a message's first packet, announced: post its write
wire hdr_here  = (state == ST_IDLE) && s_tvalid && grant && e_valid;
wire hdr_post  = hdr_here && (hdr_inline || hdr_stream);
// Later packets: posted by the generator while the message is open
wire gen_want  = g_open && e_valid && ((state == ST_STREAM) || (state == ST_PKT_WAIT));
wire gen_post  = gen_want && !oq_full;

assign wr_valid = hdr_post || gen_post;
wire   wr_hs    = wr_valid && wr_ready;

always_comb begin
    wr_req        = '0;
    wr_req.opcode = LOCAL_WRITE;
    wr_req.strm   = STRM_HOST;
    // dest 1: this module's own host stream (axis_host_send[1])
    wr_req.dest   = 1;
    if (state == ST_IDLE) begin
        wr_req.pid   = hdr_pid;
        wr_req.vaddr = hdr_va;
        wr_req.len   = hdr_inline ? LEN_BITS'(8) : LEN_BITS'(e_len - PLEN_W'(64));
        wr_req.last  = hdr_inline || e_last;
    end else begin
        wr_req.pid   = l_pid;
        wr_req.vaddr = g_va;
        wr_req.len   = LEN_BITS'(e_len);
        wr_req.last  = e_last;
    end
end

// The data side's handshakes
wire take_beat = s_tvalid && s_tready;
wire pkt_end   = (state == ST_STREAM) && s_tvalid && m_tready && (d_beats == BTS_W'(1));
// Its next packet's write is already posted (or is being posted this cycle
// and is the only one): it streams on without a gap
wire next_now  = !oq_empty;

assign e_pop = (hdr_here && (hdr_post ? wr_ready : 1'b1)) ||       // header: posted or dropped
               (gen_post && wr_ready) ||
               ((state == ST_DROP) && (d_beats == '0) && !x_last && e_valid);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        state  <= ST_IDLE;
        g_open <= 1'b0;
        oq_cnt <= '0; oq_wp <= '0; oq_rp <= '0;
        d_beats <= '0; d_last <= 1'b0; x_last <= 1'b0;
    end else begin
        // posted-ahead queue
        if (gen_post && wr_ready) begin
            oq_mem[oq_wp] <= {e_last, beats_of(e_len)};
            oq_wp <= (oq_wp == RX_WR_OUTSTANDING - 1) ? '0 : oq_wp + 1'b1;
            g_va  <= g_va + VADDR_BITS'(e_len);
            if (e_last) g_open <= 1'b0;
        end
        oq_cnt <= oq_cnt + OQ_W'(gen_post && wr_ready)
                         - OQ_W'((pkt_end && !d_last && next_now) ||
                                 ((state == ST_PKT_WAIT) && !oq_empty));

        case (state)
            // A message's first beat: its header. Posted only when the shell
            // takes the write; a bad header drops the message.
            ST_IDLE: if (hdr_here) begin
                if (hdr_post) begin
                    if (wr_ready) begin
                        l_pid    <= hdr_pid;
                        l_inline <= s_tdata[128 +: 64];
                        if (hdr_inline) state <= ST_INLINE_DATA;
                        else begin
                            state   <= ST_STREAM;
                            d_beats <= beats_of(e_len) - BTS_W'(1);
                            d_last  <= e_last;
                            g_open  <= !e_last;
                            g_va    <= hdr_va + VADDR_BITS'(e_len - PLEN_W'(64));
                        end
                    end
                end else begin
                    // the header beat is taken now; the packet's other beats
                    // and the message's later packets are drained
                    d_beats <= beats_of(e_len) - BTS_W'(1);
                    x_last  <= e_last;
                    if (!(e_last && beats_of(e_len) == BTS_W'(1))) state <= ST_DROP;
                end
            end

            ST_INLINE_DATA: if (m_tready) state <= ST_IDLE;

            ST_STREAM: if (pkt_end) begin
                if (d_last) state <= ST_IDLE;
                else if (next_now) begin
                    d_beats <= oq_beats;
                    d_last  <= oq_last;
                    oq_rp   <= (oq_rp == RX_WR_OUTSTANDING - 1) ? '0 : oq_rp + 1'b1;
                end else state <= ST_PKT_WAIT;
            end else if (s_tvalid && m_tready) begin
                d_beats <= d_beats - BTS_W'(1);
            end

            // The next packet's write is not posted yet (sq_wr busy, or not
            // announced yet)
            ST_PKT_WAIT: if (!oq_empty) begin
                d_beats <= oq_beats;
                d_last  <= oq_last;
                oq_rp   <= (oq_rp == RX_WR_OUTSTANDING - 1) ? '0 : oq_rp + 1'b1;
                state   <= ST_STREAM;
            end

            // Drain the dropped message: this packet's beats, then each later
            // packet's as it is announced, up to the one with last
            ST_DROP: if (d_beats != '0) begin
                if (s_tvalid) d_beats <= d_beats - BTS_W'(1);
            end else if (x_last) begin
                state <= ST_IDLE;
            end else if (e_valid) begin
                d_beats <= beats_of(e_len);
                x_last  <= e_last;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

assign req  = s_tvalid || (state != ST_IDLE);
assign busy = (state != ST_IDLE);

always_comb begin
    // ST_IDLE: a header is taken when its write is (or it is dropped); a beat
    // nothing announced is swallowed. ST_STREAM: payload moves when the host
    // write path takes it. ST_PKT_WAIT: hold for the next write, unless
    // nothing is announced or posted - then the beat is an orphan.
    s_tready = ((state == ST_IDLE) && s_tvalid &&
                (!announced || (e_valid && grant && (hdr_post ? wr_ready : 1'b1)))) ||
               ((state == ST_STREAM) && m_tready) ||
               ((state == ST_PKT_WAIT) && oq_empty && !announced) ||
               ((state == ST_DROP) && (d_beats != '0));

    if (state == ST_INLINE_DATA) begin
        // Constructed beat: exact-length write, data LSB-aligned
        m_tdata  = {{(AXI_DATA_BITS-64){1'b0}}, l_inline};
        m_tkeep  = {{(AXI_DATA_BITS/8-8){1'b0}}, 8'hFF};
        m_tlast  = 1'b1;
        m_tvalid = 1'b1;
    end else begin
        m_tdata  = s_tdata;
        m_tkeep  = s_tkeep;
        // tlast only on the message's final beat: the one write posted with
        // last = 1
        m_tlast  = d_last && (d_beats == BTS_W'(1));
        m_tvalid = (state == ST_STREAM) && s_tvalid;
    end
end

assign cnt_rx_fwd       = ((state == ST_INLINE_DATA) && m_tready) || pkt_end;
assign cnt_rx_drop      = hdr_here && !hdr_post;
assign cnt_rx_orphan    = take_beat && (((state == ST_IDLE) && !announced) || (state == ST_PKT_WAIT));
assign cnt_rx_move      = (state == ST_STREAM) &&  s_tvalid &&  m_tready;
assign cnt_rx_starve    = (state == ST_STREAM) && !s_tvalid;
assign cnt_rx_stall     = (state == ST_STREAM) &&  s_tvalid && !m_tready;
// EVERY cycle this module refuses a beat the shell offers, in any state
assign cnt_rx_bp        = s_tvalid && !s_tready;
assign cnt_rx_req       = e_push;
assign cnt_rx_msg       = hdr_post && hdr_stream && wr_ready;
assign cnt_rx_msg_end   = pkt_end && d_last;
assign cnt_rx_post      = wr_hs;
assign cnt_rx_post_wait = wr_valid && !wr_ready;
assign cnt_rx_at_limit  = gen_want && oq_full;
assign cnt_rx_pkt_wait  = (state == ST_PKT_WAIT) && s_tvalid;
assign cnt_rx_rq_ovfl   = rq_valid && (e_full || e_wbusy);

endmodule
