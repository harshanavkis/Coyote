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
 *   rdma:  ONE full 64 B wire message (never a sub-64 B RDMA payload -
 *          gate G5: sub-beat payloads are outside the shell's exercised
 *          envelope): wr_req {APP_WRITE, STRM_RDMA, vaddr = STAGING, 64}
 *          + a message beat {lane0 = header (op WRITE_INLINE, len),
 *          lane1 = target VA (base+off), lane2 = data}. The far side's
 *          loom_rx parses the header and issues the EXACT 8 B local
 *          write - padding never clobbers destination bytes. The wire
 *          thus carries op-len-vaddr, the design's message format; the
 *          RETH staging vaddr is data-meaningless (jigsaw's
 *          remote_vaddr pattern).
 *
 * DESC entry (bulk, >= 64 B): DIRECT on both routes - no Loom framing.
 *   1. rd_req {LOCAL_READ, STRM_HOST, src_pid, src_va, len} (the pull)
 *   2. wr_req: local {LOCAL_WRITE, pid, base+off, len}; rdma
 *      {APP_WRITE, STRM_RDMA, vaddr = base+off, len} - a plain RDMA
 *      WRITE whose RETH carries the true target: op-len-vaddr on the
 *      wire is RDMA's own header, nothing re-encoded (the inline
 *      message exists ONLY because a sub-64 B store cannot say
 *      "envelope 64, true write 8" in a RETH)
 *   3. forward the pull stream to the selected output until tlast
 *   4. if the descriptor's fence VA != 0: wr_req {LOCAL_WRITE, src_pid,
 *      fence_va, 8} + one beat carrying an incrementing completion count
 *      (the copy-engine semaphore-release pattern)
 *
 * READ entry (aperture load, local windows only):
 *   pull one full 64 B ALIGNED line containing the target
 *   (sq_rd {LOCAL_READ, dest pid, line address, 64}), lane-select the
 *   requested 8 B from the returned beat, and hand it back to loom_ctrl
 *   (rd_resp), which completes the held-open AXI-Lite read. The aligned
 *   full-line pull deliberately avoids sub-line DMA (min-payload/
 *   alignment hazards - cf. the 64 B minimum RDMA payload jigsaw hit).
 *   An INVALID read (dead window, bounds, or an rdma-route window -
 *   remote loads arrive with the two-host phase) still ALWAYS responds:
 *   poison (all-ones) + cnt_drop, so the issuing CPU is never wedged.
 *
 * TRANSMIT PACING (rdma DESC), 2026-09-13. The source is a copy engine
 * that streams: the pull is ONE LOCAL_READ for the whole descriptor and
 * its beats arrive at whatever rate the host side delivers. Loom does not
 * meter the source. It holds the data in ITS OWN FIFO (TX_FIFO_BEATS,
 * URAM) and moves it into the RoCE stack only as the stack can take it:
 *   - one sq_wr per PMTU packet (RDMA_MODE_RAW, FIRST/MIDDLE/LAST/ONLY
 *     chosen here, so the wire and loom_rx see exactly what the shell's
 *     own parser produced before), each with last=1 so rdma_flow returns
 *     one cq_wr per PACKET (req_t.last is what gates the ack; it is not
 *     on the wire);
 *   - a WINDOW: inflight = posted - acked, a packet is posted only while
 *     inflight < tx_window (CSR 66, reset 16, 0 = none). The far stack's
 *     acceptance, not a constant, bounds what is in flight. Measured:
 *     rate = W x PMTU / ack RTT (~5.2 us here) up to the pipe's ceiling,
 *     and W must stay under the receiver's upstream buffer (~40 packets:
 *     rx_crossing 2048 + incoming FIFO 512 beats) - 32 holds, 64 drops;
 *   TX_PACE (the fractional pacer) stays as a manual cap, off by default.
 * If the FIFO fills, the pull's tready drops - the only influence Loom has
 * on a streaming source, counted (cnt_tx_fifo_full), never relied on.
 *
 * Invalid window / bounds violation: entry dropped (writes) or answered
 * with poison (reads), cnt_drop pulsed either way.
 * Sub-8B stores (wstrb != 0xFF) are issued as full 8 B writes for now
 * (hardware gate G2 covers sub-line write semantics).
 */
module loom_engine (
    input  logic                        aclk,
    input  logic                        aresetn,

    // Order FIFO (from loom_ctrl)
    input  logic                        fifo_empty,
    input  logic                        fifo_is_desc,
    input  logic                        fifo_is_read,
    input  logic [3:0]                  fifo_win,
    input  logic [27:0]                 fifo_off,
    input  logic [27:0]                 fifo_len,
    input  logic [PID_BITS-1:0]         fifo_src_pid,
    input  logic [VADDR_BITS-1:0]       fifo_compl_va,
    input  logic [63:0]                 fifo_payload,
    output logic                        fifo_pop,

    // Window table lookup
    output logic [3:0]                  lu_idx,
    input  logic                        lu_valid,
    input  logic                        lu_route,
    input  logic [PID_BITS-1:0]         lu_pid,
    input  logic [VADDR_BITS-1:0]       lu_base,
    input  logic [LEN_BITS-1:0]         lu_len,

    // RDMA staging VA (RETH vaddr for all outgoing messages)
    input  logic [VADDR_BITS-1:0]       rdma_staging_va,

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

    // Read response (to loom_ctrl): completes the held-open AXI read
    output logic [63:0]                 rd_resp_data,
    output logic                        rd_resp_valid,

    // Debug counter pulses (to loom_ctrl)
    output logic                        cnt_local_wr,
    output logic                        cnt_rdma_wr,
    output logic                        cnt_drop,
    output logic                        cnt_compl,

    // Stage cycle counters (to loom_ctrl, RO CSR words 50-63): per-stage
    // cycle accumulators and completed-op counts for the T3 latency
    // measurements. Index: 0 lookup, 1 store-local, 2 store-rdma,
    // 3 dma-local, 4 dma-rdma, 5 read, 6 fence.
    output logic [63:0]                 stage_acc [7],
    output logic [63:0]                 stage_cnt [7],

    // Where the transmit stream's cycles go, mirroring loom_rx's. The engine
    // is a pass-through in ST_STREAM: a beat moves only when the host pull
    // has one AND the network takes it, so a cycle is spent one of three
    // ways. cyc/op alone cannot tell them apart, and they have opposite
    // meanings - starved is a gap WE put into the outgoing packet stream
    // because the pull ran dry; stalled is the shell pushing back, which is
    // the fabric working as intended.
    // Pull framing disagreement. ST_STREAM forwards exactly beats_of(len)
    // beats off the shared pull stream and never checks that the response
    // it is draining is the one it asked for - a surplus beat from any
    // source is forwarded as payload and shifts the message permanently,
    // which is the corruption signature hardware shows (region covered, no
    // header rejected, payload displaced by whole beats).
    //
    // The response's own framing is the cross-check: on the LAST beat of
    // the budget, s_tlast should also be high. Low means the response has
    // more beats than were taken and the surplus will be consumed by
    // whatever streams next. Intermediate tlasts are ignored on purpose -
    // the shell may return a large read as several chunks.
    output logic                        cnt_pull_desync,
    // Transmit window (see the header). tx_window 0 = no window.
    input  logic [7:0]                  tx_window,
    // One pulse per remote-write acknowledgement (cq_wr with remote set)
    input  logic                        ack_valid,
    output logic [15:0]                 tx_inflight,
    output logic                        cnt_tx_ack,
    output logic                        cnt_tx_winfull,  // a packet waited on the window
    output logic                        cnt_tx_reqwait,  // a packet waited on sq_wr.ready
    output logic                        cnt_tx_fifo_full,// the pull was held: our FIFO full
    // Egress pacing as a fraction of the burst rate: payload beats on the
    // rdma route may move at most pace_num/pace_den of cycles (a rate
    // accumulator with one beat of credit, so no bursts). Off when either
    // is 0 or num >= den. See the PACING block below for why.
    input  logic [7:0]                  pace_num,
    input  logic [7:0]                  pace_den,
    output logic                        cnt_tx_move,
    output logic                        cnt_tx_starve,
    output logic                        cnt_tx_stall,
    // Cycles the pacer held the net output (validates the knob took)
    output logic                        cnt_tx_paced,

    output logic                        busy
);

// FSM. One FIFO entry is processed start-to-finish before the next pop:
//
//   ST_IDLE       wait for a FIFO entry; latch it + its table hit, pop
//   ST_CHECK      validate (window valid, bounds, len != 0 for DESC)
//   -- STORE path --
//   ST_WR_REQ     hold wr_req until the shell accepts it (sq_wr handshake)
//   ST_WR_DATA    drive the single data beat until the stream takes it
//   -- DESC path --
//   ST_RD_REQ     issue the pull request (sq_rd); the shell starts
//                 translating and streaming the source buffer
//   ST_DMA_WR_REQ local: the one write request for the descriptor.
//                 rdma: the request for the NEXT PACKET, posted only while
//                 the window allows (inflight < tx_window)
//   ST_HDR_BEAT   rdma, packet 0 only: the 64 B Loom header beat
//   ST_STREAM     forward buffered pull beats to the selected output; on
//                 the packet's last beat go back to ST_DMA_WR_REQ if the
//                 message has more packets, else finish
//   ST_CP_REQ/    optional fence: one more write request + one beat
//   ST_CP_DATA    carrying the incremented completion count
//
// The serialization is deliberate: it is what turns the shared FIFO into
// an order point. Overlap/pipelining across entries would need
// per-window queues and completion tracking (future work, alongside the
// coalescer and per-destination scheduling).
typedef enum logic [3:0] {
    ST_IDLE, ST_CHECK, ST_WR_REQ, ST_WR_DATA,
    ST_RD_REQ, ST_DMA_WR_REQ, ST_HDR_BEAT, ST_STREAM,
    ST_CP_REQ, ST_CP_DATA,
    ST_RDP_REQ, ST_RDP_WAIT, ST_RD_RESP
} state_t;

// Wire-message header ops (keep in sync with loom_rx.sv)
localparam [7:0] MSG_OP_WRITE        = 8'd1;   // header beat + payload beats
localparam [7:0] MSG_OP_WRITE_INLINE = 8'd2;   // single beat, data in lane 2

state_t state;

// Latched copy of the FIFO head and its window-table hit. Latching at
// pop time matters for two reasons: (1) the FIFO head and the table
// lookup (lu_idx = fifo_win, combinational read) are only guaranteed
// stable while the entry is at the head - once popped, the next entry
// replaces them; (2) the table could be reprogrammed mid-transaction by
// the control plane, and a latched route makes each transaction see one
// consistent snapshot (the same reason the design compiles bindings
// ahead of time instead of consulting live state per beat).
logic                  l_is_desc, l_is_read;
logic [27:0]           l_off, l_len;
// Egress pacer state (defined in the PACING block further down); declared
// here because the FSM uses the gated ready before that block.
logic [15:0]           pace_acc;     // credit, in units of 1/pace_den beat
logic                  pace_hold;
wire                   m_net_rdy = m_net_tready && !pace_hold;
logic [PID_BITS-1:0]   l_src_pid;
logic [VADDR_BITS-1:0] l_compl_va;
logic [63:0]           l_payload;
logic                  l_valid, l_route;
logic [PID_BITS-1:0]   l_pid;
logic [VADDR_BITS-1:0] l_base;
logic [LEN_BITS-1:0]   l_lim;

// Beats of the descriptor's payload still to forward. The pull's tlast is
// NOT the transaction boundary: the shell may return a large local read as
// several tlast-terminated segments, and exiting on the first one leaves
// the sq_wr we already posted short of the length it claimed. The shell
// then takes the NEXT transaction's beat to make up the difference, which
// is how an inline store message was seen landing verbatim inside a bulk
// destination - header, target VA and payload intact - and why the damage
// accumulates: every shortfall shifts the request/payload pairing for
// everything after it. The length we asked for is the authority; tlast
// only ends a transaction early if it arrives when nothing is owed.
//
// SCOPE, honestly: this is correct on its own terms and reproduced in
// simulation (tb_loom_loopback segments the pull, and the pre-fix engine
// fails there). It has NOT been shown to be the cause of the hardware
// corruption seen at 256 KB. Buffers are HPF, so 2 MB huge pages: a 4 MB
// transfer spans two of them and plausibly does come back in segments -
// and 4 MB is exactly the size that never fences - but 256 KB sits inside
// one huge page and has no obvious reason to be split. If the 256 KB
// message-into-bulk corruption survives a rebuild, this fix is not it.
logic [22:0] l_sbeats;

// -------------------------------------------------------------------------
// Packetising on the RDMA route.
//
// A descriptor is ONE Loom message: a 64 B header beat followed by the
// payload. It goes to the stack as one request PER PMTU PACKET, framed
// here the way the shell's parser framed it before (FIRST/MIDDLE/LAST, or
// ONLY when it fits one packet), so nothing on the wire or in loom_rx
// changes. Packet 0 carries the header beat and PMTU-64 bytes of payload;
// every later packet carries PMTU until the tail. Each request is posted
// under the window and carries last=1 for its own ack.
//
// With one request per packet every packet owns one retransmit slot.
// Wire packet in 64 B beats - the shell fragments at PMTU.
localparam integer PKT_BEATS = PMTU_BYTES / 64;
// Loom's own transmit buffer, beats. 4096 x 64 B = 256 KB, in URAM.
localparam integer TX_FIFO_BEATS = 4096;

wire [VADDR_BITS-1:0] dst_vaddr = l_base + {{(VADDR_BITS-28){1'b0}}, l_off};
logic [28:0] p_left;     // message bytes (header included) not yet posted
logic        p_first;    // the next packet is the message's first
wire  [28:0] p_len  = (p_left > PMTU_BYTES) ? 29'(PMTU_BYTES) : p_left;
wire         p_more = (p_left > PMTU_BYTES);
// Offset of the next packet inside the message, for the RETH/staging vaddr
wire  [28:0] p_off  = ({1'b0, l_len} + 29'd64) - p_left;

function automatic logic [22:0] beats_of(input logic [27:0] len);
    logic [28:0] padded;
    padded   = {1'b0, len} + 29'd63;
    beats_of = padded[28:6];
endfunction

// Last payload beat of the current packet (rdma) / descriptor (local)
wire stream_last = (l_sbeats <= 23'd1);

// -------------------------------------------------------------------------
// Transmit buffer. The pull stream lands here unconditionally while there
// is room; everything below reads f_* instead of the pull directly. The
// local route and the aperture line read pass through it untouched.
// -------------------------------------------------------------------------
logic [AXI_DATA_BITS-1:0]   f_tdata;
logic [AXI_DATA_BITS/8-1:0] f_tkeep;
logic                       f_tvalid, f_tready, f_tlast;

xpm_fifo_axis #(
    .CLOCKING_MODE("common_clock"),
    .FIFO_MEMORY_TYPE("ultra"),
    .PACKET_FIFO("false"),
    .FIFO_DEPTH(TX_FIFO_BEATS),
    .TDATA_WIDTH(AXI_DATA_BITS),
    .USE_ADV_FEATURES("0000")
) inst_tx_fifo (
    .s_aresetn(aresetn), .s_aclk(aclk), .m_aclk(aclk),
    .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready),
    .s_axis_tdata(s_tdata), .s_axis_tstrb('0), .s_axis_tkeep(s_tkeep),
    .s_axis_tlast(s_tlast), .s_axis_tid('0), .s_axis_tdest('0), .s_axis_tuser('0),
    .m_axis_tvalid(f_tvalid), .m_axis_tready(f_tready),
    .m_axis_tdata(f_tdata), .m_axis_tstrb(), .m_axis_tkeep(f_tkeep),
    .m_axis_tlast(f_tlast), .m_axis_tid(), .m_axis_tdest(), .m_axis_tuser(),
    .prog_full_axis(), .wr_data_count_axis(), .almost_full_axis(),
    .prog_empty_axis(), .rd_data_count_axis(), .almost_empty_axis(),
    .injectsbiterr_axis(1'b0), .injectdbiterr_axis(1'b0),
    .sbiterr_axis(), .dbiterr_axis()
);
assign cnt_tx_fifo_full = s_tvalid && !s_tready;

// -------------------------------------------------------------------------
// Window: packets posted to the stack and not yet acknowledged.
// -------------------------------------------------------------------------
logic [15:0] inflight;
wire        win_ok   = (tx_window == 8'd0) || (inflight < {8'd0, tx_window});
wire        rdma_req = l_route && ((state == ST_WR_REQ) || (state == ST_DMA_WR_REQ));
logic       wr_valid_i;
wire        wr_hs    = wr_valid_i && wr_ready;
always_ff @(posedge aclk) begin
    if (!aresetn) inflight <= 16'd0;
    else if (wr_hs && rdma_req && !ack_valid) inflight <= inflight + 16'd1;
    else if (ack_valid && !(wr_hs && rdma_req) && (inflight != 16'd0)) inflight <= inflight - 16'd1;
end
assign tx_inflight    = inflight;
assign cnt_tx_ack     = ack_valid;
assign cnt_tx_winfull = rdma_req && !win_ok;
assign cnt_tx_reqwait = rdma_req &&  win_ok && !wr_ready;

logic [63:0] compl_cnt;
logic [63:0] rd_data;      // lane-selected read result (or poison)
logic [2:0]  rd_lane;      // which 8 B lane of the pulled line

assign lu_idx = fifo_win;
assign busy   = (state != ST_IDLE);

// Bounds check: the access must end inside the window's segment. Stores
// are fixed 8 B; descriptors use their full length (and len==0 is invalid)
wire [28:0] end_off = l_is_desc ? ({1'b0, l_off} + {1'b0, l_len})
                                : ({1'b0, l_off} + 29'd8);
// Reads are additionally local-only for now: an rdma-route window load
// is answered with poison until the two-host phase implements RDMA READ.
// Rdma bulk additionally requires 64 B-multiple lengths: the HLS TX
// merge path for a partial last word is unexercised upstream (the
// append_payload alignment TODO), so we exclude it by contract instead
// of trusting it - local DMA stays byte-granular (XDMA C2H descriptors)
wire ok = l_valid && (l_is_desc ? (l_len != 0) : 1'b1)
                  && (l_is_read ? !l_route : 1'b1)
                  && ((l_is_desc && l_route) ? (l_len[5:0] == 6'b0) : 1'b1)
                  && (end_off <= {1'b0, l_lim});

// Read target: the 64 B line containing (base + off), and the lane in it
wire [VADDR_BITS-1:0] rd_addr    = l_base + {{(VADDR_BITS-28){1'b0}}, l_off};
wire [VADDR_BITS-1:0] rd_line_va = {rd_addr[VADDR_BITS-1:6], 6'b0};

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        state <= ST_IDLE;
        compl_cnt <= 0;
        l_is_desc <= 0; l_is_read <= 0; l_off <= 0; l_len <= 0;
        l_src_pid <= 0; l_compl_va <= 0;
        rd_data <= 0; rd_lane <= 0;
        l_payload <= 0; l_sbeats <= 0;
        p_left <= 0; p_first <= 0;
        l_valid <= 0; l_route <= 0; l_pid <= 0; l_base <= 0; l_lim <= 0;
    end else begin
        case (state)
            // Latch the FIFO head and its table hit in one shot; fifo_pop
            // (combinational below) retires the entry in this same cycle
            ST_IDLE: if (!fifo_empty) begin
                l_is_desc <= fifo_is_desc;
                l_is_read <= fifo_is_read;
                l_off     <= fifo_off;
                l_len     <= fifo_len;
                l_src_pid <= fifo_src_pid;
                l_compl_va <= fifo_compl_va;
                l_payload <= fifo_payload;
                l_sbeats  <= beats_of(fifo_len);
                l_valid   <= lu_valid;
                l_route   <= lu_route;
                l_pid     <= lu_pid;
                l_base    <= lu_base;
                l_lim     <= lu_len;
                state     <= ST_CHECK;
            end

            // Validation gate: invalid window or out-of-bounds access is
            // dropped here and never reaches the shell
            ST_CHECK:
                if (l_is_read) begin
                    // A read ALWAYS answers: poison on any invalidity
                    rd_lane <= rd_addr[5:3];
                    if (!ok) begin
                        rd_data <= 64'hFFFF_FFFF_FFFF_FFFF;   // poison
                        state   <= ST_RD_RESP;
                    end else
                        state   <= ST_RDP_REQ;
                end
                else if (!ok)       state <= ST_IDLE;        // cnt_drop pulses below
                else if (l_is_desc) state <= ST_RD_REQ;
                else                state <= ST_WR_REQ;

            // ---- STORE: one write request, then one data beat ----

            ST_WR_REQ:    if (wr_hs) state <= ST_WR_DATA;
            ST_WR_DATA:
                if (( l_route && m_net_rdy) ||
                    (!l_route && m_host_tready)) state <= ST_IDLE;

            // ---- DESC: pull request, then per-packet requests + stream ----
            ST_RD_REQ:    if (rd_ready) begin
                p_left  <= {1'b0, l_len} + 29'd64;   // header + payload
                p_first <= 1'b1;
                state   <= ST_DMA_WR_REQ;
            end
            // One request per PMTU packet on the rdma route (posted under
            // the window), one for the whole descriptor on the local route.
            // The header beat goes out with packet 0 only.
            ST_DMA_WR_REQ: if (wr_hs) begin
                if (l_route) begin
                    l_sbeats <= p_first ? 23'((p_len - 29'd64) >> 6) : 23'(p_len >> 6);
                    state    <= p_first ? ST_HDR_BEAT : ST_STREAM;
                end else begin
                    l_sbeats <= beats_of(l_len);
                    state    <= ST_STREAM;
                end
            end
            ST_HDR_BEAT:   if (m_net_rdy) state <= ST_STREAM;
            ST_STREAM:
                if (f_tvalid &&
                    (( l_route && m_net_rdy) || (!l_route && m_host_tready))) begin
                    l_sbeats <= l_sbeats - 23'd1;
                    if (stream_last) begin
                        p_left  <= p_left - p_len;
                        p_first <= 1'b0;
                        if (l_route && p_more)
                            state <= ST_DMA_WR_REQ;
                        else
                            state <= (l_compl_va != 0) ? ST_CP_REQ : ST_IDLE;
                    end
                end

            // ---- READ: aligned line pull, lane select, respond ----
            ST_RDP_REQ:   if (rd_ready) state <= ST_RDP_WAIT;
            ST_RDP_WAIT:
                if (f_tvalid) begin
                    rd_data <= f_tdata[64*rd_lane +: 64];
                    if (f_tlast) state <= ST_RD_RESP;
                end
            ST_RD_RESP:   state <= ST_IDLE;    // rd_resp_valid pulses below

            // Fence release: skipped entirely when the descriptor's
            // completion VA is 0
            ST_CP_REQ:    if (wr_hs) state <= ST_CP_DATA;
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

// Pull request (DESC only). Field meanings on Coyote's sq_rd:
//   opcode LOCAL_READ + strm STRM_HOST: read host memory, deliver the
//     data to user logic on axis_host_recv[dest]
//   pid: WHICH cThread's address space the vaddr lives in - the shell
//     TLB translates (pid, vaddr) to physical pages. Using the
//     descriptor's src_pid is what lets any attached process name its
//     own buffer as the DMA source.
//   vaddr: the issuer's source VA, passed through verbatim from the
//     descriptor (payload field) - no address rewriting anywhere.
//   last: request a tlast at the end of the stream so ST_STREAM knows
//     when the transfer is done without counting bytes itself.
always_comb begin
    rd_req = '0;
    rd_req.opcode = LOCAL_READ;
    rd_req.strm   = STRM_HOST;
    rd_req.dest   = 0;
    rd_req.last   = 1'b1;
    if (state == ST_RDP_REQ) begin
        // Aperture read: pull the full aligned line from the DESTINATION
        // process's buffer (pid = the window's pid, like a local write)
        rd_req.pid   = l_pid;
        rd_req.vaddr = rd_line_va;
        rd_req.len   = 64;
    end else begin
        // DMA pull: the issuer's source buffer
        rd_req.pid   = l_src_pid;
        rd_req.vaddr = l_payload[VADDR_BITS-1:0];
        rd_req.len   = l_len[LEN_BITS-1:0];
    end
    rd_valid = (state == ST_RD_REQ) || (state == ST_RDP_REQ);
end

always_comb begin
    wr_req = '0;
    wr_req.last = 1'b1;
    wr_req.dest = 0;

    if (state == ST_CP_REQ || state == ST_CP_DATA) begin
        // Completion (fence) write: the descriptor carries its own fence
        // address, released under the issuer's pid - the CE semaphore model
        wr_req.opcode = LOCAL_WRITE;
        wr_req.strm   = STRM_HOST;
        wr_req.pid    = l_src_pid;
        wr_req.vaddr  = l_compl_va;
        wr_req.len    = 8;
    end else if (l_route && l_is_desc) begin
        // One packet of the message, framed here (see Packetising above).
        // RAW mode hands the opcode straight to the stack; the fields are
        // the ones the shell's parser would have derived from a PARSE
        // request, plus last=1 on every packet for its own ack.
        wr_req.opcode = p_first ? (p_more ? RC_RDMA_WRITE_FIRST  : RC_RDMA_WRITE_ONLY)
                                : (p_more ? RC_RDMA_WRITE_MIDDLE : RC_RDMA_WRITE_LAST);
        wr_req.strm   = STRM_RDMA;
        wr_req.mode   = 1'b1;        // RDMA_MODE_RAW
        wr_req.rdma   = 1'b1;
        wr_req.remote = 1'b1;
        wr_req.actv   = 1'b1;
        wr_req.pid    = l_pid;       // QP owner
        // Staging + offset in the message, as the parser advances it; only
        // packet 0's goes into a RETH and the far side ignores it anyway.
        wr_req.vaddr  = rdma_staging_va + {{(VADDR_BITS-29){1'b0}}, p_off};
        wr_req.len    = p_len[LEN_BITS-1:0];
    end else if (l_route) begin
        // All rdma writes go out as wire MESSAGES at the staging vaddr:
        // stores are one 64 B beat, bulk is a 64 B header + the payload
        // Rdma route: an RDMA WRITE on the window's QP. Field meanings:
        //   opcode APP_WRITE + mode 0 (RDMA_MODE_PARSE): the shell's
        //     request parser fragments the payload into PMTU-sized
        //     RC_RDMA_WRITE packets - we hand over one request for the
        //     whole transfer regardless of size.
        //   pid: selects WHOSE QP carries the write (the QP established
        //     for this window's binding). It does not name a memory
        //     space here - translation happens at the far host.
        //   vaddr: goes into the RETH on the wire. We set it to the
        //     exporter's own VA (window base + offset), so the receiving
        //     host's TLB - under the far QP owner's pid - finishes the
        //     translation. This is the "wire carries the exporter's VA /
        //     offset in affine encoding" property of the design.
        //   remote/rdma/actv: shell-side routing flags marking this as
        //     an active-side remote RDMA operation (conventions taken
        //     from the working jigsaw example).
        wr_req.opcode = APP_WRITE;
        wr_req.strm   = STRM_RDMA;
        wr_req.mode   = 1'b0;        // RDMA_MODE_PARSE: shell fragments to PMTU
        wr_req.rdma   = 1'b1;
        wr_req.remote = 1'b1;
        wr_req.actv   = 1'b1;
        wr_req.pid    = l_pid;       // QP owner
        // BOTH forms are messages at the STAGING vaddr; the header carries
        // the true target. Bulk adds one 64 B header beat to the length it
        // claims. (An earlier comment here said "bulk goes DIRECT, RETH =
        // true target" - that was stale and contradicted the line below it.
        // It matters: because the RETH names staging, the incoming rq_wr on
        // the far side carries NO destination information, so loom_rx cannot
        // forward requests the way perf_rdma does and MUST parse the header
        // to learn where the bytes go.)
        wr_req.vaddr  = rdma_staging_va;
        wr_req.len    = 'd64;
    end else begin
        // Local route: a host-memory write through the shell TLB. pid
        // names the DESTINATION process's address space (the exporter's
        // cThread), vaddr is the exporter's own buffer VA plus the
        // window offset. This is the cross-pid write at the heart of the
        // prototype: user logic writing one attached process's memory on
        // behalf of another (hardware gate G1 verifies the TLB honors it).
        wr_req.opcode = LOCAL_WRITE;
        wr_req.strm   = STRM_HOST;
        wr_req.pid    = l_pid;
        wr_req.vaddr  = dst_vaddr;
        wr_req.len    = l_is_desc ? l_len[LEN_BITS-1:0] : 'd8;
    end

    // Rdma requests wait for the window; local ones and the fence do not
    wr_valid_i = ((state == ST_WR_REQ)     && (!l_route || win_ok)) ||
                 ((state == ST_DMA_WR_REQ) && (!l_route || win_ok)) ||
                  (state == ST_CP_REQ);
    wr_valid = wr_valid_i;
end

// Wire-message header beat: lane0 = {reserved, len[27:0], op[7:0]},
// lane1 = target VA (the exporter's VA + offset), lane2 = inline data
wire [63:0] hdr_q0_inline = {28'b0, 28'd8, MSG_OP_WRITE_INLINE};
// The header names the MESSAGE's target and length - the whole
// descriptor, however many packets carry it
wire [63:0] hdr_q1        = {{(64-VADDR_BITS){1'b0}}, dst_vaddr};

wire [AXI_DATA_BITS-1:0] msg_inline_beat =
    {{(AXI_DATA_BITS-192){1'b0}}, l_payload, hdr_q1, hdr_q0_inline};

// Bulk header: same layout, op 1, and the length is the payload's - lane 2
// is unused because the data follows as its own beats
wire [63:0] hdr_q0_write = {28'b0, l_len, MSG_OP_WRITE};
wire [AXI_DATA_BITS-1:0] msg_write_beat =
    {{(AXI_DATA_BITS-192){1'b0}}, 64'b0, hdr_q1, hdr_q0_write};

// -------------------------------------------------------------------------
// Data streams
//
// Three kinds of beats leave this module:
//   - the single beat of a STORE (or of a fence write): built here, data
//     in the low 8 bytes, tkeep marking exactly those 8 bytes valid.
//     The shell writes `len` bytes starting at the request's vaddr; how
//     a sub-line beat aligns against a non-64B-aligned vaddr is exactly
//     hardware gate G2 (LSB alignment assumed until measured).
//   - forwarded DMA beats: passed through combinationally from
//     axis_host_recv (data/keep/last untouched), so the engine adds no
//     buffering or latency - backpressure from the selected output
//     propagates back into the transmit FIFO, and from there to the
//     shell's pull engine via s_tready only once the FIFO is full.
// The host and net outputs are driven from the same sources but gated by
// the latched route, so exactly one of them carries traffic per
// transaction.
// -------------------------------------------------------------------------
// Store/completion beat: data LSB-aligned, low 8 bytes valid (gate G2)
wire [AXI_DATA_BITS-1:0]   beat_data = {{(AXI_DATA_BITS-64){1'b0}},
                                        (state == ST_CP_DATA) ? (compl_cnt + 1) : l_payload};
wire [AXI_DATA_BITS/8-1:0] beat_keep = {{(AXI_DATA_BITS/8-8){1'b0}}, 8'hFF};

wire stream_local = (state == ST_STREAM) && !l_route;
wire stream_net   = (state == ST_STREAM) &&  l_route;

// -------------------------------------------------------------------------
// TX_PACE - a manual rate cap, off by default.
//
// Flow control is the window above: what is in flight is bounded by the far
// stack's acks. This is an optional cap on top of it, kept because a fixed
// fraction of the burst rate is occasionally the right experiment (and it
// is what held the link together before the window existed: 41/64 =
// 10.25 GB/s at 250 MHz x 64 B was the receiver's clean ceiling then).
//
// RATE ACCUMULATOR, not 1-in-N: every cycle earns pace_num units of
// credit, a payload beat costs pace_den, credit saturates at two beats so
// starvation cannot bank more than a two-beat burst. A 1-in-N hold was not
// binding on hardware for N >= 4, because the hold let the pull catch up.
//
// AXI-Stream legality: pace_hold gates BOTH tvalid and the engine's view of
// tready, and it can only change on a clock edge; a beat that is offered
// (tvalid high, credit present) stays offered until it moves, because
// credit only ever DROPS on a move. The header beat is not paced.
//
// pace_num/pace_den are CSRs (reset 0 = off).
// -------------------------------------------------------------------------
wire        pace_en   = (pace_num != 8'd0) && (pace_den != 8'd0) && (pace_num < pace_den);
wire        net_moved = stream_net && f_tvalid && m_net_rdy;
wire [15:0] pace_cost = {8'd0, pace_den};
// Credit ceiling: TWO beats, not one. With a one-beat cap the surplus
// earned during a hold cycle is discarded, so every fraction above 1/2
// collapses to alternate cycles (41/64 measured 50% in sim). Two beats is
// enough that nothing is ever discarded in steady state (credit after a
// hold is at most den-1+num < 2*den) and bounds any post-starvation burst
// to two back-to-back beats.
wire [15:0] pace_cap  = {7'd0, pace_den, 1'b0};

// Outside a message (and when off) the pacer parks with exactly one beat
// of credit and no hold, so the first payload beat of a message moves at
// once. Inside a message it keeps running across the per-packet request
// states, so the rate is exact over the message and not one free beat per
// packet; it must NOT run during the header beat, whose tvalid is not
// gated by pace_hold (a hold there would re-present the beat). A move is
// only possible while pace_hold is low, which by construction means
// pace_acc >= pace_den, so the subtraction below never underflows.
wire pace_run = stream_net ||
                ((state == ST_DMA_WR_REQ) && l_route && l_is_desc && !p_first);
always_ff @(posedge aclk) begin
    if (!aresetn || !pace_en || !pace_run) begin
        pace_acc  <= pace_cost;
        pace_hold <= 1'b0;
    end else begin
        // earn, spend, saturate at one beat; hold whenever credit < 1 beat
        logic [16:0] nxt;
        nxt = {1'b0, pace_acc} + {9'd0, pace_num} - (net_moved ? {1'b0, pace_cost} : 17'd0);
        if (nxt > {1'b0, pace_cap}) nxt = {1'b0, pace_cap};
        pace_acc  <= nxt[15:0];
        pace_hold <= (nxt[15:0] < pace_cost);
    end
end
assign cnt_tx_paced = pace_hold && stream_net;

always_comb begin
    // Pull stream ready while forwarding (from the selected output) or
    // while sinking the read line (always ready: nothing downstream)
    f_tready = (stream_local && m_host_tready) || (stream_net && m_net_rdy) ||
               (state == ST_RDP_WAIT);

    // Host output: store beat (local), DMA forward (local), completion beat
    m_host_tdata  = stream_local ? f_tdata : beat_data;
    m_host_tkeep  = stream_local ? f_tkeep : beat_keep;
    m_host_tlast  = stream_local ? stream_last : 1'b1;
    m_host_tvalid = ((state == ST_WR_DATA) && !l_route) ||
                    (stream_local && f_tvalid) ||
                    (state == ST_CP_DATA);

    // Net output: inline message beat (rdma store) or forwarded DMA
    // beats (rdma bulk, raw payload). The message beat is a full 64 B
    // (keep all ones) - nothing sub-beat ever goes on the wire
    m_net_tdata  = (state == ST_WR_DATA)  ? msg_inline_beat :
                   (state == ST_HDR_BEAT) ? msg_write_beat  : f_tdata;
    // Every beat on the wire is a full 64 B, payload included. The header
    // above always was; the payload was NOT - it forwarded the pull
    // stream's keep verbatim, so a single partial beat coming back from the
    // host read would have made the packetiser emit fewer than 64 bytes
    // there and shifted every byte after it. A descriptor's length is a
    // multiple of 64 by contract (loom_engine drops the rest at the source
    // and hdr_ok requires hdr_len[5:0] == 0), so every payload beat is full
    // and forcing this is identical when the contract holds and corrective
    // when it does not.
    m_net_tkeep  = {(AXI_DATA_BITS/8){1'b1}};
    // One tlast per PACKET (stream_last counts the packet's beats), which
    // is what a last=1 request asks for on its stream
    m_net_tlast  = stream_net ? stream_last : (state == ST_WR_DATA);
    m_net_tvalid = ((state == ST_WR_DATA) && l_route) ||
                   (state == ST_HDR_BEAT) ||
                   (stream_net && f_tvalid && !pace_hold);
end

// -------------------------------------------------------------------------
// Counter pulses (single-cycle events consumed by loom_ctrl's counters)
//
// A write is counted at the moment its LAST beat is accepted: for stores
// that is the one ST_WR_DATA handshake; for DMA it is the tlast beat
// handshake in ST_STREAM. Drops are counted in the one cycle ST_CHECK
// rejects an entry; fences when the completion beat is taken. Each
// condition includes the corresponding ready, so a stalled beat is not
// double-counted while it waits.
// -------------------------------------------------------------------------
assign rd_resp_data  = rd_data;
assign rd_resp_valid = (state == ST_RD_RESP);

// Only the rdma route: the local route's beats never reach a wire, and
// mixing them in makes the figure unreadable the same way the store phase
// did on the receive side.
// The header beat counts too - it is a beat on the same wire, and waiting
// to place it is the same stall as waiting to place payload. It can never
// be starved: it is generated here, not pulled.
wire tx_stream = (state == ST_STREAM)   && l_route;
wire tx_hdr    = (state == ST_HDR_BEAT);
assign cnt_tx_move   = (tx_hdr    &&  m_net_rdy) ||
                       (tx_stream &&  f_tvalid && m_net_rdy);
assign cnt_tx_starve =  tx_stream && !f_tvalid;

assign cnt_tx_stall  = (tx_hdr    && !m_net_tready) ||
                       (tx_stream &&  f_tvalid && !m_net_tready);

assign cnt_drop     = (state == ST_CHECK) && !ok;
// Residue on the pull stream at the moment a new read is issued. Nothing
// should be waiting there: the previous transfer consumed exactly its own
// budget, so a beat present now belongs to no one and will be forwarded as
// the payload of the transfer about to start.
//
// Deliberately NOT keyed on s_tlast. The shell may return a large read as
// several tlast-terminated chunks, so "the last beat carries tlast" is
// satisfied by any chunk boundary and detects nothing.
assign cnt_pull_desync = (state == ST_RD_REQ) && rd_ready && f_tvalid;

// A descriptor's last beat on the rdma route is the last beat of its
// LAST packet; the per-message counters key on that, not on every packet
wire desc_net_done = stream_net && f_tvalid && stream_last && !p_more && m_net_rdy;
assign cnt_local_wr = ((state == ST_WR_DATA) && !l_route && m_host_tready) ||
                      (stream_local && f_tvalid && stream_last && m_host_tready);
assign cnt_rdma_wr  = ((state == ST_WR_DATA) && l_route && m_net_rdy) ||
                      desc_net_done;
assign cnt_compl    = (state == ST_CP_DATA) && m_host_tready;

// -------------------------------------------------------------------------
// Stage cycle counters (T3)
//
// Every cycle the FSM spends outside ST_IDLE/ST_CHECK is attributed to
// the class of the transaction in flight (fence cycles to the fence
// class, not the underlying descriptor's). The lookup accumulator gets
// exactly 2 cycles per popped entry - the pop/latch cycle and the
// ST_CHECK cycle - covering the table lookup and the bounds check (the
// user-logic part of translation); acc[0] == 2*cnt[0] is a testable
// invariant. Counts increment on the same completion conditions as the
// debug counter pulses, so dropped entries are popped (cnt[0]) but never
// counted as completed ops. With no backpressure a store spends exactly
// one cycle in ST_WR_REQ and one in ST_WR_DATA, so acc[1] (or [2])
// advances by 2 per store; stall cycles accumulate into the stage where
// the transaction waits, which is precisely what T3 wants attributed.
// -------------------------------------------------------------------------
wire in_fence = (state == ST_CP_REQ) || (state == ST_CP_DATA);
wire [2:0] work_class = in_fence  ? 3'd6 :
                        l_is_read ? 3'd5 :
                        l_is_desc ? (l_route ? 3'd4 : 3'd3) :
                                    (l_route ? 3'd2 : 3'd1);
wire in_work = (state != ST_IDLE) && (state != ST_CHECK);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        for (int i = 0; i < 7; i++) begin
            stage_acc[i] <= 0;
            stage_cnt[i] <= 0;
        end
    end else begin
        if (fifo_pop) begin
            stage_acc[0] <= stage_acc[0] + 1;
            stage_cnt[0] <= stage_cnt[0] + 1;
        end
        if (state == ST_CHECK)
            stage_acc[0] <= stage_acc[0] + 1;
        if (in_work)
            stage_acc[work_class] <= stage_acc[work_class] + 1;

        if ((state == ST_WR_DATA) && !l_route && m_host_tready)
            stage_cnt[1] <= stage_cnt[1] + 1;
        if ((state == ST_WR_DATA) && l_route && m_net_rdy)
            stage_cnt[2] <= stage_cnt[2] + 1;
        if (stream_local && f_tvalid && stream_last && m_host_tready)
            stage_cnt[3] <= stage_cnt[3] + 1;
        if (desc_net_done)
            stage_cnt[4] <= stage_cnt[4] + 1;
        if (state == ST_RD_RESP)
            stage_cnt[5] <= stage_cnt[5] + 1;
        if (cnt_compl)
            stage_cnt[6] <= stage_cnt[6] + 1;
    end
end

endmodule
