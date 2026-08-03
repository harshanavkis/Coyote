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
 *                        When the descriptor retires, the engine writes an
 *                        incrementing count to (DMA_SRC_PID, DMA_COMPL_VA),
 *                        like a copy engine's semaphore release.
 *   32-39 (RO) debug counters:
 *   32 stores captured        33 descriptors queued
 *   34 local writes issued    35 rdma writes issued
 *   36 rx writes forwarded    37 bounds/invalid drops
 *   38 order-FIFO overflows   39 completions written
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
    output logic [3:0]                  fifo_win,
    output logic [27:0]                 fifo_off,
    output logic [27:0]                 fifo_len,      // DESC: length; STORE: wstrb in [7:0]
    output logic [PID_BITS-1:0]         fifo_src_pid,  // DESC only
    output logic [VADDR_BITS-1:0]       fifo_compl_va, // DESC only: fence VA (0 = none)
    output logic [63:0]                 fifo_payload,  // STORE: data; DESC: source VA
    input  logic                        fifo_pop,

    // Debug counter pulses (from engine / rx)
    input  logic                        cnt_local_wr,
    input  logic                        cnt_rdma_wr,
    input  logic                        cnt_rx_fwd,
    input  logic                        cnt_drop,
    input  logic                        cnt_compl
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
localparam integer R_DBG_BASE    = 32;
localparam integer N_DBG         = 8;

// Order FIFO entry: {tag(1), win(4), off(28), len(28), pid(PID_BITS), compl_va(VADDR_BITS), payload(64)}
localparam integer ENTRY_W    = 1 + 4 + 28 + 28 + PID_BITS + VADDR_BITS + 64;
localparam integer FIFO_AW    = 6;
localparam integer FIFO_DEPTH = 1 << FIFO_AW;

// -------------------------------------------------------------------------
// AXI4-Lite handshake registers (standard Coyote ctrl-slave pattern)
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

logic ctrl_reg_wren, ctrl_reg_rden;
assign ctrl_reg_wren = axi_wready && axi_ctrl.wvalid && axi_awready && axi_ctrl.awvalid;
assign ctrl_reg_rden = axi_arready && axi_ctrl.arvalid && ~axi_rvalid;

// Address decode: any write with a nonzero window index (addr[15:12]) is
// an aperture store; window 0 is the CSR page
wire        wr_is_aperture = |axi_awaddr[15:12];
wire [3:0]  wr_win         = axi_awaddr[15:12];
wire [11:0] wr_win_off     = axi_awaddr[11:0];
wire [CSR_BITS-1:0] wr_idx = axi_awaddr[ADDR_LSB +: CSR_BITS];
wire [CSR_BITS-1:0] rd_idx = axi_araddr[ADDR_LSB +: CSR_BITS];

// -------------------------------------------------------------------------
// CSRs
// -------------------------------------------------------------------------
logic [63:0] r_tbl_idx, r_tbl_cfg, r_tbl_pid, r_tbl_base, r_tbl_len;
logic [63:0] r_dma_dst, r_dma_src_va, r_dma_len, r_dma_src_pid, r_dma_compl_va;
logic [63:0] dbg [N_DBG];

// COMMIT/TRIGGER act on write pulses (not stored values) and require the
// low byte strobe - this also makes them immune to strobe-less padding
// writes some environments generate
wire csr_wr        = ctrl_reg_wren && !wr_is_aperture;
wire commit_pulse  = csr_wr && (wr_idx == R_TBL_COMMIT)  && axi_ctrl.wstrb[0] && axi_ctrl.wdata[0];
wire trigger_pulse = csr_wr && (wr_idx == R_DMA_TRIGGER) && axi_ctrl.wstrb[0] && axi_ctrl.wdata[0];

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        r_tbl_idx <= 0; r_tbl_cfg <= 0; r_tbl_pid <= 0; r_tbl_base <= 0; r_tbl_len <= 0;
        r_dma_dst <= 0; r_dma_src_va <= 0; r_dma_len <= 0; r_dma_src_pid <= 0;
        r_dma_compl_va <= 0;
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

// -------------------------------------------------------------------------
// Order FIFO
// -------------------------------------------------------------------------
logic [ENTRY_W-1:0] fifo_mem [FIFO_DEPTH];
logic [FIFO_AW:0]   wptr, rptr;

wire fifo_full_i  = (wptr[FIFO_AW] != rptr[FIFO_AW]) && (wptr[FIFO_AW-1:0] == rptr[FIFO_AW-1:0]);
wire fifo_empty_i = (wptr == rptr);

// Push rules: aperture write beats and descriptor triggers enter the same
// FIFO in arrival order; a full FIFO drops (counted) rather than stalls
// the AXI-Lite bridge (posted-write semantics toward the host)
wire push_store = ctrl_reg_wren && wr_is_aperture && !fifo_full_i;
wire push_desc  = trigger_pulse && !fifo_full_i;
wire push_drop  = ((ctrl_reg_wren && wr_is_aperture) || trigger_pulse) && fifo_full_i;

// STORE: len-field slot carries wstrb in its low 8 bits (engine currently
// assumes full 8 B stores; wstrb kept for a later sub-word extension)
wire [ENTRY_W-1:0] store_entry = {1'b0, wr_win, {16'b0, wr_win_off},
                                  {20'b0, axi_ctrl.wstrb},
                                  {PID_BITS{1'b0}}, {VADDR_BITS{1'b0}},
                                  axi_ctrl.wdata};
wire [ENTRY_W-1:0] desc_entry  = {1'b1, r_dma_dst[63:60], r_dma_dst[27:0],
                                  r_dma_len[27:0], r_dma_src_pid[PID_BITS-1:0],
                                  r_dma_compl_va[VADDR_BITS-1:0],
                                  {{(64-VADDR_BITS){1'b0}}, r_dma_src_va[VADDR_BITS-1:0]}};

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        wptr <= 0; rptr <= 0;
    end else begin
        if (push_store || push_desc) begin
            fifo_mem[wptr[FIFO_AW-1:0]] <= push_desc ? desc_entry : store_entry;
            wptr <= wptr + 1'b1;
        end
        if (fifo_pop && !fifo_empty_i)
            rptr <= rptr + 1'b1;
    end
end

wire [ENTRY_W-1:0] head = fifo_mem[rptr[FIFO_AW-1:0]];

assign fifo_payload  = head[63:0];
assign fifo_compl_va = head[64 +: VADDR_BITS];
assign fifo_src_pid  = head[64+VADDR_BITS +: PID_BITS];
assign fifo_len      = head[64+VADDR_BITS+PID_BITS +: 28];
assign fifo_off      = head[64+VADDR_BITS+PID_BITS+28 +: 28];
assign fifo_win      = head[64+VADDR_BITS+PID_BITS+56 +: 4];
assign fifo_is_desc  = head[64+VADDR_BITS+PID_BITS+60];
assign fifo_empty   = fifo_empty_i;

// -------------------------------------------------------------------------
// Debug counters
// -------------------------------------------------------------------------
always_ff @(posedge aclk) begin
    if (!aresetn) begin
        for (int i = 0; i < N_DBG; i++) dbg[i] <= 0;
    end else begin
        if (push_store)   dbg[0] <= dbg[0] + 1;
        if (push_desc)    dbg[1] <= dbg[1] + 1;
        if (cnt_local_wr) dbg[2] <= dbg[2] + 1;
        if (cnt_rdma_wr)  dbg[3] <= dbg[3] + 1;
        if (cnt_rx_fwd)   dbg[4] <= dbg[4] + 1;
        if (cnt_drop)     dbg[5] <= dbg[5] + 1;
        if (push_drop)    dbg[6] <= dbg[6] + 1;
        if (cnt_compl)    dbg[7] <= dbg[7] + 1;
    end
end

// -------------------------------------------------------------------------
// AXI4-Lite read data
// -------------------------------------------------------------------------
always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_rdata <= 0;
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
            default:
                if (rd_idx >= R_DBG_BASE && rd_idx < R_DBG_BASE + N_DBG)
                    axi_rdata <= dbg[rd_idx - R_DBG_BASE];
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

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_arready <= 1'b0; axi_araddr <= 0;
    end else begin
        if (~axi_arready && axi_ctrl.arvalid) begin
            axi_arready <= 1'b1; axi_araddr <= axi_ctrl.araddr[15:0];
        end else begin
            axi_arready <= 1'b0;
        end
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

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_rvalid <= 0; axi_rresp <= 0;
    end else begin
        if (axi_arready && axi_ctrl.arvalid && ~axi_rvalid) begin
            axi_rvalid <= 1'b1; axi_rresp <= 2'b0;
        end else if (axi_rvalid && axi_ctrl.rready) begin
            axi_rvalid <= 1'b0;
        end
    end
end

endmodule
