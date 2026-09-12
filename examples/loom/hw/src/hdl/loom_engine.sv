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
    // A payload beat that arrived from the host read with a partial keep.
    // Nonzero means the pull really does hand back sub-beat data and the
    // wire used to carry it.
    output logic                        cnt_tx_partial,
    // Chunk size in bytes from CSR 28. 0 disables chunking entirely, which
    // is the pre-chunking behaviour and lets both be compared on one
    // bitstream. Reset value is the derived safe size.
    input  logic [27:0]                 chunk_bytes,
    // Egress pacing as a fraction of the burst rate: payload beats on the
    // rdma route may move at most pace_num/pace_den of cycles (a rate
    // accumulator with one beat of credit, so no bursts). Off when either
    // is 0 or num >= den. See the PACING block below for why.
    input  logic [7:0]                  pace_num,
    input  logic [7:0]                  pace_den,
    output logic                        cnt_tx_move,
    output logic                        cnt_tx_starve,
    // Starvation that lands INSIDE a wire packet - see the block below.
    output logic                        cnt_tx_starve_mid,
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
//   ST_DMA_WR_REQ issue the matching write request before any data moves,
//                 so the shell knows where the forwarded stream goes
//   ST_STREAM     forward pull beats to the selected output; leave on the
//                 handshake of the tlast beat
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
// Chunking on the RDMA route.
//
// The shell buffers every outgoing packet into HBM for replay in
// RDMA_N_WR_OUTSTANDING slots of PMTU per pid. A message longer than that
// WRAPS the buffer, so when RC replays an early packet the slot already
// holds a later packet's bytes; the receiver takes it, because the PSN is
// right, and the payload is displaced by whole beats. Nothing guards it:
// rdma_flow.sv counts outstanding REQUESTS, not packets, so one 4 MB
// message is 1025 packets and one request and sails straight through.
//
// perf_rdma is immune because its payload comes from the shell's data mover
// via dreq req_1, which can RE-READ host memory on a replay. A vFPGA
// request has req_1 tied to zero, so user-streamed payload cannot be
// re-read and must be buffered - into exactly this space.
//
// So a message must fit the buffer, header included. Derived from the
// shell's own parameters rather than hardcoded: raise
// RDMA_N_WR_OUTSTANDING and the chunk follows with no edit here.
// Measured at the current config (16 x 4096 - 64 = 65472), cold, no ramp:
//   65472 (16 packets)  INTACT, 0 retrans, 9.840 GB/s
//  131072 (32 packets)  CORRUPT, 78 retrans
//  262144 (64 packets)  CORRUPT, 375 retrans
// 4194304 (1025 pkts)   CORRUPT, ~1500 retrans
// The slots are per PID, SHARED BY EVERY OUTSTANDING MESSAGE - not per
// message. rdma_flow.sv admits RDMA_N_WR_OUTSTANDING requests, and the
// engine issues chunks back to back with nothing limiting concurrency, so
// the real invariant is
//
//     (chunks in flight) x (packets per chunk)  <=  RDMA_N_WR_OUTSTANDING
//
// Sizing the chunk to the whole buffer satisfies that only if exactly one
// chunk is ever in flight, and THE ENGINE CANNOT KNOW THAT. Pacing on
// cq_wr was tried and REMOVED - do not put it back without new evidence.
// Measured on build_sep6, which wired the engine's credit to cq_wr with no
// opcode filter (jigsaw's exact mechanism), cold 4 MB:
//   engine chunks, 65472 B, 1 in flight   WEDGED, warm-up never fenced
//   same size, software paced             0 retrans, PASS, 9.842 GB/s
// The counters built for that run say why. Per 4 MB region:
//   software-issued descriptors   71 acks / 65 chunks + 7 setup, one each
//   engine-generated chunks        0 acks beyond setup, engine wedged
// A descriptor submitted through the normal request path produces a
// completion; a chunk the engine splits internally produces NONE. So there
// is no retire signal to pace against in hardware, filtered or not, and
// the opcode-filter argument was a red herring on both sides.
//
// What remains here is the chunk size alone, as a runtime knob. Keeping a
// message inside the retransmit buffer is worth doing on its own: it is
// what lets a replay be served correctly from that buffer instead of
// needing the re-read the shell denies a vFPGA request.
// The safe size, kept here for the record and used as loom_ctrl's reset
// value for CSR 28. The engine reads the CSR, so software can change it or
// switch chunking off without a rebuild.
localparam integer CHUNK_BYTES = RDMA_N_WR_OUTSTANDING * PMTU_BYTES - 64;
// Wire packet in 64 B beats - the shell fragments at PMTU.
localparam integer PKT_BEATS = PMTU_BYTES / 64;

// Declared here because ST_RD_REQ latches the first chunk's target from it.
wire [VADDR_BITS-1:0] dst_vaddr = l_base + {{(VADDR_BITS-28){1'b0}}, l_off};
logic [27:0]           c_left;    // payload bytes of this descriptor still to send
logic [27:0]           c_len;     // bytes in the chunk being sent
logic [VADDR_BITS-1:0] c_va;      // where this chunk lands
// chunk_bytes == 0 means do not chunk: one message for the whole descriptor.
wire         chunk_en = (chunk_bytes != 28'd0);
wire  [27:0] chunk_lim = chunk_en ? chunk_bytes : 28'h FFFFFFF;
wire  [27:0] c_next = (c_left > chunk_lim) ? chunk_lim : c_left;
wire         c_last = (c_left <= chunk_lim);

function automatic logic [22:0] beats_of(input logic [27:0] len);
    logic [28:0] padded;
    padded   = {1'b0, len} + 29'd63;
    beats_of = padded[28:6];
endfunction

// Last payload beat this descriptor may forward
wire stream_last = (l_sbeats <= 23'd1);

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
        c_left <= 0; c_len <= 0; c_va <= 0;
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
                c_left    <= fifo_len;
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

            ST_WR_REQ:    if (wr_ready) state <= ST_WR_DATA;
            ST_WR_DATA:
                if (( l_route && m_net_rdy) ||
                    (!l_route && m_host_tready)) state <= ST_IDLE;

            // ---- DESC: pull request, write request, stream, fence ----
            ST_RD_REQ:    if (rd_ready) begin
                c_va  <= dst_vaddr;      // first chunk lands at the target
                state <= ST_DMA_WR_REQ;
            end
            // Rdma bulk now travels as a WRITE message: a header beat
            // carrying {op 1, len} + the target VA, then the payload. The
            // far side takes the destination and the LENGTH from that
            // header, so it can issue ONE host write for the whole message
            // instead of one per PMTU packet - which is what the receive
            // path was doing, 256 of them for a 1 MB transfer. Local route
            // is untouched: it writes host memory directly, no wire, no
            // header.
            ST_DMA_WR_REQ: if (wr_ready) begin
                c_len    <= l_route ? c_next : l_len;
                l_sbeats <= beats_of(l_route ? c_next : l_len);
                state    <= l_route ? ST_HDR_BEAT : ST_STREAM;
            end
            ST_HDR_BEAT:   if (m_net_rdy) state <= ST_STREAM;
            ST_STREAM:
                if (s_tvalid &&
                    (( l_route && m_net_rdy) || (!l_route && m_host_tready))) begin
                    l_sbeats <= l_sbeats - 23'd1;
                    if (stream_last) begin
                        // The pull is ONE read of the whole descriptor and
                        // keeps streaming across chunk boundaries; only the
                        // framing on the wire changes. The fence is carried
                        // by the LAST chunk alone (wr_req below passes
                        // compl_va 0 otherwise), so a caller still sees one
                        // descriptor and one completion and never learns
                        // the chunk size.
                        c_left <= c_left - c_len;
                        c_va   <= c_va + {{(VADDR_BITS-28){1'b0}}, c_len};
                        if (l_route && !c_last)
                            state <= ST_DMA_WR_REQ;
                        else
                            state <= (l_compl_va != 0) ? ST_CP_REQ : ST_IDLE;
                    end
                end

            // Fence release: skipped entirely when the descriptor's
            // completion VA is 0
            // ---- READ: aligned line pull, lane select, respond ----
            ST_RDP_REQ:   if (rd_ready) state <= ST_RDP_WAIT;
            ST_RDP_WAIT:
                if (s_tvalid) begin
                    rd_data <= s_tdata[64*rd_lane +: 64];
                    if (s_tlast) state <= ST_RD_RESP;
                end
            ST_RD_RESP:   state <= ST_IDLE;    // rd_resp_valid pulses below

            // Fence release: skipped entirely when the descriptor's
            // completion VA is 0
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
        wr_req.len    = l_is_desc ? (c_next[LEN_BITS-1:0] + 'd64) : 'd64;
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

    wr_valid = (state == ST_WR_REQ) || (state == ST_DMA_WR_REQ) || (state == ST_CP_REQ);
end

// Wire-message header beat: lane0 = {reserved, len[27:0], op[7:0]},
// lane1 = target VA (the exporter's VA + offset), lane2 = inline data
wire [63:0] hdr_q0_inline = {28'b0, 28'd8, MSG_OP_WRITE_INLINE};
// The header names THIS CHUNK's target and length; on the local route,
// which is not chunked, c_va/c_len are the descriptor's own.
// A DESC names THIS CHUNK's target, which advances across the message. A
// STORE never goes through ST_RD_REQ, so c_va is not loaded for it and it
// must name the descriptor's own target.
wire [63:0] hdr_q1        = {{(64-VADDR_BITS){1'b0}},
                             l_is_desc ? c_va : dst_vaddr};

wire [AXI_DATA_BITS-1:0] msg_inline_beat =
    {{(AXI_DATA_BITS-192){1'b0}}, l_payload, hdr_q1, hdr_q0_inline};

// Bulk header: same layout, op 1, and the length is the payload's - lane 2
// is unused because the data follows as its own beats
wire [63:0] hdr_q0_write = {28'b0, c_len, MSG_OP_WRITE};
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
//     propagates straight back into the shell's pull engine via s_tready.
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
// PACING - the flow control this link does not have.
//
// The receiver's host-write path saturates at ~10 GB/s (amy: 63% moving /
// 34% stalled, FLAT as the offered rate rises - measured across the chunk
// sweep, and again with this pacer: rx stall 0 and ingress-FIFO-full 0 at
// 8 GB/s, 32k and 29k at 10.6). This engine bursts at 64 B/cycle; the wire
// has no PFC, no DCQCN and no prog_full consumer, so nothing tells the
// sender to slow down. Above the receiver's ceiling its ~3000 beats of
// buffering fill in ~1.7 MB, the CMAC (no tready) drops beats before any
// counter, the PSN gap NAKs, Go-Back-N replays, and the replay lands
// displaced. Paced under the ceiling, a lone 4 MB message lands byte-exact
// with 0 retransmissions - the first time in 119 runs (2026-09-12).
//
// RATE ACCUMULATOR, not 1-in-N. The first version held one cycle after
// every N beats; on hardware that was NOT BINDING for N >= 4, because the
// hold let the host pull catch up and simply replaced the pull's own ~21%
// starvation instead of adding to it, and it had no setting between 50%
// and 67%. This one caps the fraction of cycles a payload beat may move at
// pace_num/pace_den exactly: every cycle in ST_STREAM earns pace_num
// units of credit, a beat costs pace_den, credit saturates at two beats so
// starvation cannot bank more than a two-beat burst. 41/64 is ~10.25 GB/s
// at 250 MHz x 64 B.
//
// AXI-Stream legality: pace_hold gates BOTH tvalid and the engine's view of
// tready, and it can only change on a clock edge; a beat that is offered
// (tvalid high, credit present) stays offered until it moves, because
// credit only ever DROPS on a move. The header beat is not paced.
//
// pace_num/pace_den are CSRs (reset 0 = off) so one bitstream sweeps the
// rate; the highest clean setting IS the receiver's drain ceiling.
// -------------------------------------------------------------------------
wire        pace_en   = (pace_num != 8'd0) && (pace_den != 8'd0) && (pace_num < pace_den);
wire        net_moved = stream_net && s_tvalid && m_net_rdy;
wire [15:0] pace_cost = {8'd0, pace_den};
// Credit ceiling: TWO beats, not one. With a one-beat cap the surplus
// earned during a hold cycle is discarded, so every fraction above 1/2
// collapses to alternate cycles (41/64 measured 50% in sim). Two beats is
// enough that nothing is ever discarded in steady state (credit after a
// hold is at most den-1+num < 2*den) and bounds any post-starvation burst
// to two back-to-back beats.
wire [15:0] pace_cap  = {7'd0, pace_den, 1'b0};

// Outside ST_STREAM (and when off) the pacer parks with exactly one beat
// of credit and no hold, so the first payload beat of a message moves at
// once. A move is only possible while pace_hold is low, which by
// construction means pace_acc >= pace_den, so the subtraction below never
// underflows.
always_ff @(posedge aclk) begin
    if (!aresetn || !pace_en || !stream_net) begin
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
    s_tready = (stream_local && m_host_tready) || (stream_net && m_net_rdy) ||
               (state == ST_RDP_WAIT);

    // Host output: store beat (local), DMA forward (local), completion beat
    m_host_tdata  = stream_local ? s_tdata : beat_data;
    m_host_tkeep  = stream_local ? s_tkeep : beat_keep;
    m_host_tlast  = stream_local ? stream_last : 1'b1;
    m_host_tvalid = ((state == ST_WR_DATA) && !l_route) ||
                    (stream_local && s_tvalid) ||
                    (state == ST_CP_DATA);

    // Net output: inline message beat (rdma store) or forwarded DMA
    // beats (rdma bulk, raw payload). The message beat is a full 64 B
    // (keep all ones) - nothing sub-beat ever goes on the wire
    m_net_tdata  = (state == ST_WR_DATA)  ? msg_inline_beat :
                   (state == ST_HDR_BEAT) ? msg_write_beat  : s_tdata;
    // Every beat on the wire is a full 64 B, payload included. The header
    // above always was; the payload was NOT - it forwarded the pull
    // stream's keep verbatim, so a single partial beat coming back from the
    // host read would have made the packetiser emit fewer than 64 bytes
    // there and shifted every byte after it. A descriptor's length is a
    // multiple of 64 by contract (loom_engine drops the rest at the source
    // and hdr_ok requires hdr_len[5:0] == 0), so every payload beat is full
    // and forcing this is identical when the contract holds and corrective
    // when it does not. cnt_tx_partial says which.
    m_net_tkeep  = {(AXI_DATA_BITS/8){1'b1}};
    m_net_tlast  = stream_net ? stream_last : (state == ST_WR_DATA);
    m_net_tvalid = ((state == ST_WR_DATA) && l_route) ||
                   (state == ST_HDR_BEAT) ||
                   (stream_net && s_tvalid && !pace_hold);
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
                       (tx_stream &&  s_tvalid && m_net_rdy);
assign cnt_tx_starve =  tx_stream && !s_tvalid;

// WHERE the starvation lands, which is the thing that matters and which
// cnt_tx_starve cannot tell you.
//
// The shell fragments this message at PMTU (rdma_req_parser.sv emits
// plen = PMTU_BYTES per fragment), so on the wire a packet is
// PMTU_BYTES/64 = 64 beats, counted from the message's header beat. A gap
// BETWEEN packets costs throughput and nothing else. A gap INSIDE one is a
// different animal: the packetiser has begun a frame it cannot finish.
//
// Measured (2026-09-08): the CLEAN control gaps mid-frame MORE than the
// corrupt runs do and loses nothing, so a mid-frame gap does not cause the
// loss. Kept as a shape diagnostic only. The loss is on the RECEIVER - its
// host write saturates at ~10 GB/s and the link has no flow control - and
// the pacer above is what addresses it.
logic [6:0] pkt_beat;
always_ff @(posedge aclk) begin
    if (!aresetn)                     pkt_beat <= 7'd0;
    else if (state == ST_HDR_BEAT)    pkt_beat <= 7'd0;   // message restarts a packet
    else if (cnt_tx_move)             pkt_beat <= (pkt_beat == PKT_BEATS-1) ? 7'd0
                                                                           : pkt_beat + 7'd1;
end
assign cnt_tx_starve_mid = cnt_tx_starve && (pkt_beat != 7'd0);
assign cnt_tx_stall  = (tx_hdr    && !m_net_tready) ||
                       (tx_stream &&  s_tvalid && !m_net_tready);

assign cnt_drop     = (state == ST_CHECK) && !ok;
// Residue on the pull stream at the moment a new read is issued. Nothing
// should be waiting there: the previous transfer consumed exactly its own
// budget, so a beat present now belongs to no one and will be forwarded as
// the payload of the transfer about to start.
//
// Deliberately NOT keyed on s_tlast. The shell may return a large read as
// several tlast-terminated chunks, so "the last beat carries tlast" is
// satisfied by any chunk boundary and detects nothing.
assign cnt_pull_desync = (state == ST_RD_REQ) && rd_ready && s_tvalid;
assign cnt_tx_partial  = stream_net && s_tvalid && m_net_rdy &&
                         (s_tkeep != {(AXI_DATA_BITS/8){1'b1}});

assign cnt_local_wr = ((state == ST_WR_DATA) && !l_route && m_host_tready) ||
                      (stream_local && s_tvalid && stream_last && m_host_tready);
assign cnt_rdma_wr  = ((state == ST_WR_DATA) && l_route && m_net_rdy) ||
                      (stream_net && s_tvalid && stream_last && m_net_rdy);
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
        if (stream_local && s_tvalid && s_tlast && m_host_tready)
            stage_cnt[3] <= stage_cnt[3] + 1;
        if (stream_net && s_tvalid && s_tlast && m_net_rdy)
            stage_cnt[4] <= stage_cnt[4] + 1;
        if (state == ST_RD_RESP)
            stage_cnt[5] <= stage_cnt[5] + 1;
        if (cnt_compl)
            stage_cnt[6] <= stage_cnt[6] + 1;
    end
end

endmodule
