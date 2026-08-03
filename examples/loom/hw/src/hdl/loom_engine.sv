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
    ST_RD_REQ, ST_DMA_WR_REQ, ST_STREAM,
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
logic [PID_BITS-1:0]   l_src_pid;
logic [VADDR_BITS-1:0] l_compl_va;
logic [63:0]           l_payload;
logic                  l_valid, l_route;
logic [PID_BITS-1:0]   l_pid;
logic [VADDR_BITS-1:0] l_base;
logic [LEN_BITS-1:0]   l_lim;

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
        l_payload <= 0;
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
                if (( l_route && m_net_tready) ||
                    (!l_route && m_host_tready)) state <= ST_IDLE;

            // ---- DESC: pull request, write request, stream, fence ----
            ST_RD_REQ:    if (rd_ready) state <= ST_DMA_WR_REQ;
            ST_DMA_WR_REQ: if (wr_ready) state <= ST_STREAM;
            ST_STREAM:
                if (s_tvalid && s_tlast &&
                    (( l_route && m_net_tready) || (!l_route && m_host_tready)))
                    state <= (l_compl_va != 0) ? ST_CP_REQ : ST_IDLE;

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
wire [VADDR_BITS-1:0] dst_vaddr = l_base + {{(VADDR_BITS-28){1'b0}}, l_off};

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
        // Bulk goes DIRECT (RETH = true target); only sub-64 B stores
        // use the staging-addressed inline message envelope
        wr_req.vaddr  = l_is_desc ? dst_vaddr : rdma_staging_va;
        wr_req.len    = l_is_desc ? l_len[LEN_BITS-1:0] : 'd64;
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
wire [63:0] hdr_q1        = {{(64-VADDR_BITS){1'b0}}, dst_vaddr};

wire [AXI_DATA_BITS-1:0] msg_inline_beat =
    {{(AXI_DATA_BITS-192){1'b0}}, l_payload, hdr_q1, hdr_q0_inline};

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

always_comb begin
    // Pull stream ready while forwarding (from the selected output) or
    // while sinking the read line (always ready: nothing downstream)
    s_tready = (stream_local && m_host_tready) || (stream_net && m_net_tready) ||
               (state == ST_RDP_WAIT);

    // Host output: store beat (local), DMA forward (local), completion beat
    m_host_tdata  = stream_local ? s_tdata : beat_data;
    m_host_tkeep  = stream_local ? s_tkeep : beat_keep;
    m_host_tlast  = stream_local ? s_tlast : 1'b1;
    m_host_tvalid = ((state == ST_WR_DATA) && !l_route) ||
                    (stream_local && s_tvalid) ||
                    (state == ST_CP_DATA);

    // Net output: inline message beat (rdma store) or forwarded DMA
    // beats (rdma bulk, raw payload). The message beat is a full 64 B
    // (keep all ones) - nothing sub-beat ever goes on the wire
    m_net_tdata  = (state == ST_WR_DATA) ? msg_inline_beat : s_tdata;
    m_net_tkeep  = stream_net ? s_tkeep : {(AXI_DATA_BITS/8){1'b1}};
    m_net_tlast  = stream_net ? s_tlast : (state == ST_WR_DATA);
    m_net_tvalid = ((state == ST_WR_DATA) && l_route) ||
                   (stream_net && s_tvalid);
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

assign cnt_drop     = (state == ST_CHECK) && !ok;
assign cnt_local_wr = ((state == ST_WR_DATA) && !l_route && m_host_tready) ||
                      (stream_local && s_tvalid && s_tlast && m_host_tready);
assign cnt_rdma_wr  = ((state == ST_WR_DATA) && l_route && m_net_tready) ||
                      (stream_net && s_tvalid && s_tlast && m_net_tready);
assign cnt_compl    = (state == ST_CP_DATA) && m_host_tready;

endmodule
