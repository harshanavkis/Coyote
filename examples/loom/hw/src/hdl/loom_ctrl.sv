import lynxTypes::*;

/**
 * loom_ctrl
 *
 * AXI4-Lite slave for the Loom vFPGA. The 64 KB user ctrl region is split:
 *   - byte 0x0000-0x0FFF: CSR page (table programming, DMA descriptor
 *     staging incl. per-descriptor fence VA, RO debug counters)
 *   - byte 0x1000-0xFFFF: aperture, 15 windows of 4 KB. Window index =
 *     byte_addr[15:12] (1..15), offset within window = byte_addr[11:0].
 *     Every write beat landing here is captured as a small-write
 *     transaction (posted; bvalid returned immediately).
 *
 * Captured stores and triggered DMA descriptors are pushed into ONE
 * arrival-ordered FIFO (the order point), so a store issued after a
 * descriptor cannot overtake it downstream (data-then-flag across paths).
 *
 * CSR map (64-bit word indices; byte offset = idx * 8):
 *    0 TBL_IDX      (RW) window index to program (1..15)
 *    1 TBL_CFG      (RW) bit0 = valid, bit1 = route (0 local, 1 rdma)
 *    2 TBL_PID      (RW) local: destination cThread pid; rdma: QP-owner pid
 *    3 TBL_BASE     (RW) destination VA base (exporter's own VA)
 *    4 TBL_LEN      (RW) segment length in bytes (bounds)
 *    5 TBL_COMMIT   (W)  write 1 -> commit staged entry to table[TBL_IDX]
 *    8 DMA_DST      (RW) [63:60] window, [27:0] segment offset
 *    9 DMA_SRC_VA   (RW) source VA (issuer's buffer, verbatim)
 *   10 DMA_LEN      (RW) transfer length in bytes
 *   11 DMA_SRC_PID  (RW) issuer's cThread pid (used for the pull)
 *   12 DMA_TRIGGER  (W)  write 1 -> enqueue descriptor into the order FIFO
 *   13 DMA_COMPL_VA (RW) per-descriptor completion (fence) VA; 0 = none.
 *   16 RDMA_STAGING_VA (RW) RETH vaddr used for ALL outgoing rdma
 *                        messages (per-host staging address, exchanged at
 *                        QP setup; the true target rides the message
 *                        header - the wire carries op-len-vaddr).
 *                        When the descriptor retires, the engine writes an
 *                        incrementing count to (DMA_SRC_PID, DMA_COMPL_VA),
 *                        like a copy engine's semaphore release.
 *   32-40 (RO) debug counters:
 *   32 stores captured        33 descriptors queued
 *   34 local writes issued    35 rdma writes issued
 *   36 rx writes forwarded    37 bounds/invalid drops
 *   38 order-FIFO overflows   39 completions written
 *   40 reads captured          41 rx headers rejected
 *   48-63 (RO) stage cycle counters (T3 per-stage latencies):
 *   48 free-running cycle counter
 *   49 queue-wait accumulator: sum over popped entries of their order-FIFO
 *      residency in cycles (push timestamp carried in the entry) - t-queue
 *   50-56 engine stage accumulators (cycles): 50 lookup (pop + check,
 *      2/entry - t-lookup incl. the bounds check, i.e. the user-logic part
 *      of t-translate), 51 store-local (t-forward), 52 store-rdma
 *      (t-encap), 53 dma-local, 54 dma-rdma, 55 read (aligned-line pull
 *      service incl. the shell round trip), 56 fence
 *   57-63 matching completed-op counts (57 = entries popped, the divisor
 *      for 49/50; drops are popped but never complete, so 58-63 count
 *      only successful ops). Averages = acc/cnt, computed by software;
 *      never cleared (deltas).
 *
 * Aperture READS (loads through a window) are captured too: the read is
 * pushed into the same order FIFO (kind READ) and the AXI-Lite read
 * channel is HELD OPEN (rvalid deferred) until the engine returns the
 * 8 B result - so a load observes program order against earlier stores
 * and descriptors. Two safety rules: a read is never dropped (arready is
 * withheld while the FIFO is full - backpressuring a non-posted read is
 * legal, dropping one wedges the issuing CPU until PCIe completion
 * timeout), and reads are single-outstanding by construction (this
 * slave completes one read before accepting the next), which is also
 * why the design's read-credit tracker degenerates to depth-1 here.
 */
module loom_ctrl (
    input  logic                        aclk,
    input  logic                        aresetn,

    AXI4L.s                             axi_ctrl,

    // Table programming (to loom_table)
    output logic                        tbl_commit,
    output logic [3:0]                  tbl_idx,
    output logic                        tbl_valid,
    output logic                        tbl_route,
    output logic [PID_BITS-1:0]         tbl_pid,
    output logic [VADDR_BITS-1:0]       tbl_base,
    output logic [LEN_BITS-1:0]         tbl_len,

    // Order FIFO, pop side (to loom_engine)
    output logic                        fifo_empty,
    output logic                        fifo_is_desc,
    output logic                        fifo_is_read,
    output logic [3:0]                  fifo_win,
    output logic [27:0]                 fifo_off,
    output logic [27:0]                 fifo_len,      // DESC: length; STORE: wstrb in [7:0]
    output logic [PID_BITS-1:0]         fifo_src_pid,  // DESC only
    output logic [VADDR_BITS-1:0]       fifo_compl_va, // DESC only: fence VA (0 = none)
    output logic [63:0]                 fifo_payload,  // STORE: data; DESC: source VA
    input  logic                        fifo_pop,

    // RDMA staging VA (to loom_engine): RETH vaddr for outgoing messages
    output logic [VADDR_BITS-1:0]       rdma_staging_va,

    // Read response (from loom_engine): completes the held-open AXI read
    input  logic [63:0]                 rd_resp_data,
    input  logic                        rd_resp_valid,

    // Debug counter pulses (from engine / rx)
    input  logic                        cnt_local_wr,
    input  logic                        cnt_rdma_wr,
    input  logic                        cnt_rx_fwd,
    input  logic                        cnt_rx_drop,
    input  logic                        cnt_drop,
    input  logic                        cnt_compl,
    input  logic                        cnt_rx_move,
    input  logic                        cnt_rx_starve,
    input  logic                        cnt_rx_stall,
    input  logic                        cnt_rx_stall_head,
    input  logic                        cnt_rx_stall_body,
    input  logic                        cnt_rx_req,
    input  logic                        cnt_rx_span,

    // Stage cycle counters (accumulated in loom_engine, read out here)
    input  logic [63:0]                 stage_acc [7],
    input  logic [63:0]                 stage_cnt [7]
);

// -------------------------------------------------------------------------
// Constants
// -------------------------------------------------------------------------
localparam integer ADDR_LSB = $clog2(AXIL_DATA_BITS/8);   // 3
localparam integer CSR_BITS = 9;                          // 512 words in the CSR page

localparam integer R_TBL_IDX     = 0;
localparam integer R_TBL_CFG     = 1;
localparam integer R_TBL_PID     = 2;
localparam integer R_TBL_BASE    = 3;
localparam integer R_TBL_LEN     = 4;
localparam integer R_TBL_COMMIT  = 5;
localparam integer R_DMA_DST     = 8;
localparam integer R_DMA_SRC_VA  = 9;
localparam integer R_DMA_LEN     = 10;
localparam integer R_DMA_SRC_PID = 11;
localparam integer R_DMA_TRIGGER = 12;
localparam integer R_DMA_COMPL_VA = 13;
// Word 16 = the first word of an otherwise empty 64 B line, deliberately
// NOT word 14. A host ctrl write covers its whole line forward from the
// target, so at word 14 this register sat inside the burst of every
// descriptor staging write at word 8: dma() zeroed it, the next store left
// with RETH = 0, and the far side wrote eight bytes at VA 0. Bursts stop at
// a line boundary, so from here only a write to this register can reach it,
// and the rest of its line (17-23) is unused.
localparam integer R_RDMA_STAGING = 16;
localparam integer R_DBG_BASE    = 32;
localparam integer N_DBG         = 10;
// Receive-path cycle accounting (RO 42-44). Words 42-47 were unused; these
// three partition the cycles loom_rx spends forwarding, so the receiver's
// ceiling can be attributed without a probe on the host side (a CSR poll is
// a PCIe transaction into the card that is landing the payload, and spinning
// on it slows the receiver enough to change the answer).
localparam integer R_RX_MOVE     = 42;
localparam integer R_RX_STARVE   = 43;
localparam integer R_RX_STALL    = 44;
localparam integer R_RX_ST_HEAD  = 45;   // stalled before the packet's first beat
localparam integer R_RX_ST_BODY  = 46;   // stalled after it
localparam integer R_RX_REQ      = 47;   // rq_wr requests accepted
localparam integer R_RX_SPAN     = 17;   // continuation requests absorbed
localparam integer R_CYC         = 48;
localparam integer R_QUEUE_ACC   = 49;
localparam integer R_STG_ACC     = 50;   // 7 words: 50-56
localparam integer R_STG_CNT     = 57;   // 7 words: 57-63
localparam integer N_STG         = 7;

// Order FIFO entry: {ts(32), kind(2), win(4), off(28), len(28), pid(PID_BITS), compl_va(VADDR_BITS), payload(64)}
// kind: 00 = STORE, 01 = DESC, 10 = READ. ts = push-time cycle count
// (low 32 bits of the free-running counter; wrap-safe 32-bit subtraction
// at pop time gives the entry's FIFO residency for the t-queue stat).
localparam integer ENTRY_W    = 32 + 2 + 4 + 28 + 28 + PID_BITS + VADDR_BITS + 64;
localparam [1:0] KIND_STORE = 2'b00, KIND_DESC = 2'b01, KIND_READ = 2'b10;
localparam integer FIFO_AW    = 6;
localparam integer FIFO_DEPTH = 1 << FIFO_AW;

// -------------------------------------------------------------------------
// AXI4-Lite handshake registers (standard Coyote ctrl-slave pattern)
//
// This slave accepts one write at a time: it waits until BOTH awvalid and
// wvalid are up (the XDMA bridge presents address and data together),
// latches awaddr, asserts awready/wready for exactly one cycle, and then
// raises bvalid until the master takes the response. aw_en blocks a new
// address phase until the previous response was accepted, so there is
// never more than one write in flight inside this module. Reads follow
// the same single-outstanding pattern on ar/r.
//
// Note the consequence for the aperture: the write response (bvalid) is
// returned as soon as the beat is captured into the order FIFO, before
// anything downstream happens. Toward the host this makes aperture
// stores posted writes, which is exactly the peer-store semantics we are
// emulating (a GPU peer store completes at the fabric, not at the peer).
// -------------------------------------------------------------------------
logic [15:0] axi_awaddr;
logic        axi_awready;
logic [15:0] axi_araddr;
logic        axi_arready;
logic [1:0]  axi_bresp;
logic        axi_bvalid;
logic        axi_wready;
logic [AXIL_DATA_BITS-1:0] axi_rdata;
logic [1:0]  axi_rresp;
logic        axi_rvalid;
logic        aw_en;

// Aperture-read bookkeeping: rd_pending is set from the AR handshake of a
// window load until its deferred rvalid completes
logic rd_pending;
wire  ar_is_aperture = |axi_ctrl.araddr[15:12];
wire  ar_hs          = axi_arready && axi_ctrl.arvalid;

logic ctrl_reg_wren, ctrl_reg_rden;
assign ctrl_reg_wren = axi_wready && axi_ctrl.wvalid && axi_awready && axi_ctrl.awvalid;
assign ctrl_reg_rden = axi_arready && axi_ctrl.arvalid && ~axi_rvalid && !ar_is_aperture;

// Address decode: any write with a nonzero window index (addr[15:12]) is
// an aperture store; window 0 is the CSR page
wire        wr_is_aperture = |axi_awaddr[15:12];
wire [3:0]  wr_win         = axi_awaddr[15:12];
wire [11:0] wr_win_off     = axi_awaddr[11:0];
wire [CSR_BITS-1:0] wr_idx = axi_awaddr[ADDR_LSB +: CSR_BITS];
wire [CSR_BITS-1:0] rd_idx = axi_araddr[ADDR_LSB +: CSR_BITS];

// -------------------------------------------------------------------------
// CSRs
//
// A table entry (5 words) and a DMA descriptor (5 words) are both wider
// than one AXI-Lite beat, so they cannot be written atomically. The
// scheme used here is stage-then-pulse: software writes the individual
// r_tbl_* / r_dma_* staging registers in any order, then writes 1 to
// COMMIT (table) or TRIGGER (descriptor). Only that final write has a
// side effect - it snapshots the staged values into the table / the
// order FIFO in a single cycle, so a half-written entry can never be
// observed by the data path. Staged values persist, so back-to-back
// descriptors that share fields (e.g. same source pid) only need to
// rewrite what changed.
// -------------------------------------------------------------------------
logic [63:0] r_tbl_idx, r_tbl_cfg, r_tbl_pid, r_tbl_base, r_tbl_len;
logic [63:0] r_dma_dst, r_dma_src_va, r_dma_len, r_dma_src_pid, r_dma_compl_va;
logic [63:0] r_rdma_staging;
logic [63:0] dbg [N_DBG];
logic [63:0] rx_move, rx_starve, rx_stall, rx_st_head, rx_st_body, rx_req_cnt;
logic [63:0] rx_span_cnt;

// COMMIT/TRIGGER are edge-style: they fire on the write pulse itself
// (ctrl_reg_wren), not on a stored value, so writing 1 twice fires twice
// and there is nothing to clear afterwards. Requiring wstrb[0] and
// wdata[0] means a write must actually assert the low byte with bit 0
// set - writes with empty strobes (e.g. the randomized padding writes
// the Coyote simulation TB generates around real accesses) cannot fire
// a spurious commit or trigger.
wire csr_wr        = ctrl_reg_wren && !wr_is_aperture;
wire commit_pulse  = csr_wr && (wr_idx == R_TBL_COMMIT)  && axi_ctrl.wstrb[0] && axi_ctrl.wdata[0];
wire trigger_pulse = csr_wr && (wr_idx == R_DMA_TRIGGER) && axi_ctrl.wstrb[0] && axi_ctrl.wdata[0];

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        r_tbl_idx <= 0; r_tbl_cfg <= 0; r_tbl_pid <= 0; r_tbl_base <= 0; r_tbl_len <= 0;
        r_dma_dst <= 0; r_dma_src_va <= 0; r_dma_len <= 0; r_dma_src_pid <= 0;
        r_dma_compl_va <= 0; r_rdma_staging <= 0;
    end else if (csr_wr) begin
        case (wr_idx)
            R_TBL_IDX:     r_tbl_idx     <= axi_ctrl.wdata;
            R_TBL_CFG:     r_tbl_cfg     <= axi_ctrl.wdata;
            R_TBL_PID:     r_tbl_pid     <= axi_ctrl.wdata;
            R_TBL_BASE:    r_tbl_base    <= axi_ctrl.wdata;
            R_TBL_LEN:     r_tbl_len     <= axi_ctrl.wdata;
            R_DMA_DST:     r_dma_dst     <= axi_ctrl.wdata;
            R_DMA_SRC_VA:  r_dma_src_va  <= axi_ctrl.wdata;
            R_DMA_LEN:     r_dma_len     <= axi_ctrl.wdata;
            R_DMA_SRC_PID: r_dma_src_pid <= axi_ctrl.wdata;
            R_DMA_COMPL_VA: r_dma_compl_va <= axi_ctrl.wdata;
            R_RDMA_STAGING: r_rdma_staging <= axi_ctrl.wdata;
            default: ;
        endcase
    end
end

assign tbl_commit = commit_pulse;
assign tbl_idx    = r_tbl_idx[3:0];
assign tbl_valid  = r_tbl_cfg[0];
assign tbl_route  = r_tbl_cfg[1];
assign tbl_pid    = r_tbl_pid[PID_BITS-1:0];
assign tbl_base   = r_tbl_base[VADDR_BITS-1:0];
assign tbl_len    = r_tbl_len[LEN_BITS-1:0];
assign rdma_staging_va = r_rdma_staging[VADDR_BITS-1:0];

// -------------------------------------------------------------------------
// Order FIFO - the ordering heart of the design
//
// Both producer paths (aperture store beats and descriptor triggers)
// funnel into this single FIFO in bus-arrival order, and the engine
// consumes it strictly in order, one transaction at a time. That is the
// entire mechanism behind the data-then-flag guarantee: if software
// triggers a DMA and then stores a flag, the flag entry physically sits
// behind the descriptor entry and cannot be issued until the DMA's whole
// stream (and fence) has drained.
//
// Implementation: classic power-of-two circular buffer with (AW+1)-bit
// read/write pointers. Empty when the pointers are equal; full when the
// low AW bits match but the wrap (MSB) bits differ - the extra bit
// distinguishes "same slot, zero laps apart" from "same slot, one lap
// apart" without a separate count register.
//
// A full FIFO DROPS the incoming entry (and counts it in dbg[6]) instead
// of stalling: stalling would deassert wready/bvalid back through the
// AXI-Lite bridge into the PCIe fabric, turning our posted aperture
// stores into blocking ones. Software sizes its outstanding work against
// the depth (64) or checks the overflow counter.
// -------------------------------------------------------------------------
logic [ENTRY_W-1:0] fifo_mem [FIFO_DEPTH];
logic [FIFO_AW:0]   wptr, rptr;

wire fifo_full_i  = (wptr[FIFO_AW] != rptr[FIFO_AW]) && (wptr[FIFO_AW-1:0] == rptr[FIFO_AW-1:0]);
wire fifo_empty_i = (wptr == rptr);

// Push rules: aperture write beats and descriptor triggers enter the same
// FIFO in arrival order; a full FIFO drops (counted) rather than stalls
// the AXI-Lite bridge (posted-write semantics toward the host)
// A beat with no write strobes modifies nothing by AXI semantics, so it is
// not a store. This matters because a host ctrl write arrives as a burst
// covering its whole 64 B line - the sim models it too ("Write burst which
// happens in real hardware", ctrl_simulation.svh) - and without this gate
// one 8 B aperture store became eight FIFO entries and eight wire messages,
// the extra seven landing 8 B writes at +8..+56 of the target. Hardware
// showed them: dst2[0x800..0x838] came back as the stored value followed by
// five zeros and two words of bus garbage. The CSR page survived the same
// burst only because COMMIT/TRIGGER demand wstrb[0] && wdata[0] below.
wire push_store = ctrl_reg_wren && wr_is_aperture && (|axi_ctrl.wstrb) &&
                  !fifo_full_i;
wire push_desc  = trigger_pulse && !fifo_full_i;
// Reads are pushed at the AR handshake; arready is only granted when the
// FIFO has room (see the arready block), so a read can never be dropped
wire push_read  = ar_hs && ar_is_aperture;
wire push_drop  = ((ctrl_reg_wren && wr_is_aperture && (|axi_ctrl.wstrb)) ||
                   trigger_pulse) && fifo_full_i;

// Free-running cycle counter (RO word 48; also the source of the push
// timestamps and, on hardware, of software-side interval measurements)
logic [63:0] cycle_cnt;
wire  [31:0] ts_now = cycle_cnt[31:0];

// STORE: len-field slot carries wstrb in its low 8 bits (engine currently
// assumes full 8 B stores; wstrb kept for a later sub-word extension)
wire [ENTRY_W-1:0] store_entry = {ts_now, KIND_STORE, wr_win, {16'b0, wr_win_off},
                                  {20'b0, axi_ctrl.wstrb},
                                  {PID_BITS{1'b0}}, {VADDR_BITS{1'b0}},
                                  axi_ctrl.wdata};
wire [ENTRY_W-1:0] desc_entry  = {ts_now, KIND_DESC, r_dma_dst[63:60], r_dma_dst[27:0],
                                  r_dma_len[27:0], r_dma_src_pid[PID_BITS-1:0],
                                  r_dma_compl_va[VADDR_BITS-1:0],
                                  {{(64-VADDR_BITS){1'b0}}, r_dma_src_va[VADDR_BITS-1:0]}};
// READ entry: window + offset only (uses the AR address at handshake time)
wire [ENTRY_W-1:0] read_entry  = {ts_now, KIND_READ, axi_ctrl.araddr[15:12],
                                  {16'b0, axi_ctrl.araddr[11:0]},
                                  28'b0, {PID_BITS{1'b0}}, {VADDR_BITS{1'b0}}, 64'b0};

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        wptr <= 0; rptr <= 0;
    end else begin
        if (push_store || push_desc || push_read) begin
            fifo_mem[wptr[FIFO_AW-1:0]] <= push_read ? read_entry :
                                           push_desc ? desc_entry : store_entry;
            wptr <= wptr + 1'b1;
        end
        if (fifo_pop && !fifo_empty_i)
            rptr <= rptr + 1'b1;
    end
end

// Head decode. The entry is a manual bit-pack, MSB to LSB:
//   { tag(1) | win(4) | off(28) | len(28) | pid(PID) | compl_va(VADDR) | payload(64) }
// tag: 0 = STORE, 1 = DESC. Field reuse between the two entry kinds:
//   STORE: off = 12-bit window offset (upper bits 0), the len slot's low
//          8 bits carry the AXI write strobes, payload = store data,
//          pid/compl_va unused (zero).
//   DESC:  off = 28-bit segment offset, len = transfer bytes, pid = the
//          issuer's cThread id (used for the pull and the fence),
//          compl_va = fence address (0 = no fence), payload holds the
//          source VA in its low VADDR_BITS.
// The read is combinational (head is just a slice of the memory word at
// rptr), so the engine sees a valid head in the same cycle fifo_empty
// deasserts.
wire [ENTRY_W-1:0] head = fifo_mem[rptr[FIFO_AW-1:0]];
wire [31:0] head_ts = head[ENTRY_W-1 -: 32];

assign fifo_payload  = head[63:0];
assign fifo_compl_va = head[64 +: VADDR_BITS];
assign fifo_src_pid  = head[64+VADDR_BITS +: PID_BITS];
assign fifo_len      = head[64+VADDR_BITS+PID_BITS +: 28];
assign fifo_off      = head[64+VADDR_BITS+PID_BITS+28 +: 28];
assign fifo_win      = head[64+VADDR_BITS+PID_BITS+56 +: 4];
assign fifo_is_desc  = head[64+VADDR_BITS+PID_BITS+60 +: 2] == KIND_DESC;
assign fifo_is_read  = head[64+VADDR_BITS+PID_BITS+60 +: 2] == KIND_READ;
assign fifo_empty   = fifo_empty_i;

// -------------------------------------------------------------------------
// Debug counters
//
// Free-running 64-bit event counters, read-only through the CSR page,
// never cleared (software computes deltas). The first two (stores
// captured, descriptors queued) are counted here at push time; the rest
// arrive as single-cycle pulses from the engine (writes issued per
// route, drops, fences) and from rx (writes forwarded). Their intended
// use is exact accounting in tests - e.g. local_wr == stores + descs
// when everything is valid and local - and first-line triage on
// hardware (a store that "vanished" shows up as either a drop or an
// overflow here before any ILA is needed).
// -------------------------------------------------------------------------
// Queue-wait accumulator: at pop time, the wrap-safe 32-bit difference
// between now and the entry's push timestamp is its FIFO residency. The
// sum over all pops (divided by the engine's pop count, stage_cnt[0])
// is the t-queue average.
logic [63:0] queue_acc;

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        cycle_cnt <= 0;
        queue_acc <= 0;
    end else begin
        cycle_cnt <= cycle_cnt + 1;
        if (fifo_pop && !fifo_empty_i)
            queue_acc <= queue_acc + {32'b0, ts_now - head_ts};
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        for (int i = 0; i < N_DBG; i++) dbg[i] <= 0;
        rx_move <= 0; rx_starve <= 0; rx_stall <= 0;
        rx_st_head <= 0; rx_st_body <= 0; rx_req_cnt <= 0; rx_span_cnt <= 0;
    end else begin
        if (push_store)   dbg[0] <= dbg[0] + 1;
        if (push_desc)    dbg[1] <= dbg[1] + 1;
        if (cnt_local_wr) dbg[2] <= dbg[2] + 1;
        if (cnt_rdma_wr)  dbg[3] <= dbg[3] + 1;
        if (cnt_rx_fwd)   dbg[4] <= dbg[4] + 1;
        if (cnt_drop)     dbg[5] <= dbg[5] + 1;
        if (push_drop)    dbg[6] <= dbg[6] + 1;
        if (cnt_compl)    dbg[7] <= dbg[7] + 1;
        if (push_read)    dbg[8] <= dbg[8] + 1;
        if (cnt_rx_drop)  dbg[9] <= dbg[9] + 1;
        if (cnt_rx_move)   rx_move   <= rx_move + 1;
        if (cnt_rx_starve) rx_starve <= rx_starve + 1;
        if (cnt_rx_stall)  rx_stall  <= rx_stall + 1;
        if (cnt_rx_stall_head) rx_st_head <= rx_st_head + 1;
        if (cnt_rx_stall_body) rx_st_body <= rx_st_body + 1;
        if (cnt_rx_req)        rx_req_cnt <= rx_req_cnt + 1;
        if (cnt_rx_span)       rx_span_cnt <= rx_span_cnt + 1;
    end
end

// -------------------------------------------------------------------------
// AXI4-Lite read data
// -------------------------------------------------------------------------
always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_rdata <= 0;
    end else if (rd_pending && rd_resp_valid) begin
        // Deferred aperture-read completion: the engine's 8 B result
        axi_rdata <= rd_resp_data;
    end else if (ctrl_reg_rden) begin
        axi_rdata <= 0;
        case (rd_idx)
            R_TBL_IDX:     axi_rdata <= r_tbl_idx;
            R_TBL_CFG:     axi_rdata <= r_tbl_cfg;
            R_TBL_PID:     axi_rdata <= r_tbl_pid;
            R_TBL_BASE:    axi_rdata <= r_tbl_base;
            R_TBL_LEN:     axi_rdata <= r_tbl_len;
            R_DMA_DST:     axi_rdata <= r_dma_dst;
            R_DMA_SRC_VA:  axi_rdata <= r_dma_src_va;
            R_DMA_LEN:     axi_rdata <= r_dma_len;
            R_DMA_SRC_PID: axi_rdata <= r_dma_src_pid;
            R_DMA_COMPL_VA: axi_rdata <= r_dma_compl_va;
            R_RDMA_STAGING: axi_rdata <= r_rdma_staging;
            default:
                if (rd_idx >= R_DBG_BASE && rd_idx < R_DBG_BASE + N_DBG)
                    axi_rdata <= dbg[rd_idx - R_DBG_BASE];
                else if (rd_idx == R_RX_MOVE)
                    axi_rdata <= rx_move;
                else if (rd_idx == R_RX_STARVE)
                    axi_rdata <= rx_starve;
                else if (rd_idx == R_RX_STALL)
                    axi_rdata <= rx_stall;
                else if (rd_idx == R_RX_ST_HEAD)
                    axi_rdata <= rx_st_head;
                else if (rd_idx == R_RX_ST_BODY)
                    axi_rdata <= rx_st_body;
                else if (rd_idx == R_RX_SPAN)
                    axi_rdata <= rx_span_cnt;
                else if (rd_idx == R_RX_REQ)
                    axi_rdata <= rx_req_cnt;
                else if (rd_idx == R_CYC)
                    axi_rdata <= cycle_cnt;
                else if (rd_idx == R_QUEUE_ACC)
                    axi_rdata <= queue_acc;
                else if (rd_idx >= R_STG_ACC && rd_idx < R_STG_ACC + N_STG)
                    axi_rdata <= stage_acc[rd_idx - R_STG_ACC];
                else if (rd_idx >= R_STG_CNT && rd_idx < R_STG_CNT + N_STG)
                    axi_rdata <= stage_cnt[rd_idx - R_STG_CNT];
        endcase
    end
end

// -------------------------------------------------------------------------
// Standard AXI4-Lite control (Coyote boilerplate)
// -------------------------------------------------------------------------
assign axi_ctrl.awready = axi_awready;
assign axi_ctrl.arready = axi_arready;
assign axi_ctrl.bresp   = axi_bresp;
assign axi_ctrl.bvalid  = axi_bvalid;
assign axi_ctrl.wready  = axi_wready;
assign axi_ctrl.rdata   = axi_rdata;
assign axi_ctrl.rresp   = axi_rresp;
assign axi_ctrl.rvalid  = axi_rvalid;

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_awready <= 1'b0; axi_awaddr <= 0; aw_en <= 1'b1;
    end else begin
        if (~axi_awready && axi_ctrl.awvalid && axi_ctrl.wvalid && aw_en) begin
            axi_awready <= 1'b1; aw_en <= 1'b0;
            axi_awaddr  <= axi_ctrl.awaddr[15:0];
        end else if (axi_ctrl.bready && axi_bvalid) begin
            aw_en <= 1'b1; axi_awready <= 1'b0;
        end else begin
            axi_awready <= 1'b0;
        end
    end
end

// AR channel. CSR reads accept unconditionally (single-outstanding as
// before). Aperture reads additionally require: no read already pending,
// room in the order FIFO (this is the never-drop rule - the master just
// sees arready withheld), and no write handshake completing in the same
// cycle (the FIFO has one write port; writes keep priority).
always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_arready <= 1'b0; axi_araddr <= 0;
        rd_pending  <= 1'b0;
    end else begin
        if (~axi_arready && axi_ctrl.arvalid && !rd_pending &&
            (!ar_is_aperture ||
             (!fifo_full_i && !(axi_ctrl.awvalid && axi_ctrl.wvalid)))) begin
            axi_arready <= 1'b1; axi_araddr <= axi_ctrl.araddr[15:0];
        end else begin
            axi_arready <= 1'b0;
        end
        if (ar_hs && ar_is_aperture)
            rd_pending <= 1'b1;
        else if (axi_rvalid && axi_ctrl.rready)
            rd_pending <= 1'b0;
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_bvalid <= 0; axi_bresp <= 2'b0;
    end else begin
        if (axi_awready && axi_ctrl.awvalid && ~axi_bvalid && axi_wready && axi_ctrl.wvalid) begin
            axi_bvalid <= 1'b1; axi_bresp <= 2'b0;
        end else if (axi_ctrl.bready && axi_bvalid) begin
            axi_bvalid <= 1'b0;
        end
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_wready <= 1'b0;
    end else begin
        if (~axi_wready && axi_ctrl.wvalid && axi_ctrl.awvalid && aw_en)
            axi_wready <= 1'b1;
        else
            axi_wready <= 1'b0;
    end
end

// R channel: CSR reads complete on the cycle after the AR handshake;
// aperture reads complete only when the engine's response arrives
always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_rvalid <= 0; axi_rresp <= 0;
    end else begin
        if (axi_arready && axi_ctrl.arvalid && ~axi_rvalid && !ar_is_aperture) begin
            axi_rvalid <= 1'b1; axi_rresp <= 2'b0;
        end else if (rd_pending && rd_resp_valid) begin
            axi_rvalid <= 1'b1; axi_rresp <= 2'b0;
        end else if (axi_rvalid && axi_ctrl.rready) begin
            axi_rvalid <= 1'b0;
        end
    end
end

endmodule
