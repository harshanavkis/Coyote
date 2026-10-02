import lynxTypes::*;

/**
 * loom_ctrl (loom_switch)
 *
 * AXI4-Lite slave for the switch's CSR page (the vFPGA's user ctrl region).
 * Control only: every byte of data enters through the uwin (loom_ingress),
 * so there is no aperture, no order FIFO and no descriptor here. Word
 * numbers follow examples/loom's loom_ctrl where the register survives, so
 * the same software writes them.
 *
 * CSR map (64-bit word indices; byte offset = idx * 8):
 *    0 TBL_IDX      (RW) window index to program (1..15)
 *    1 TBL_CFG      (RW) bit0 = valid, bit1 = route (0 local, 1 rdma)
 *    2 TBL_PID      (RW) [5:0] local: destination pid; rdma: QP-owner pid.
 *                        [13:8] unused
 *    3 TBL_BASE     (RW) local: destination VA base; rdma: the REMOTE
 *                        REFERENCE {far export index [47:40], offset [39:0]}
 *                        that the window's offset 0 lands at
 *    4 TBL_LEN      (RW) window length in bytes (bounds)
 *    5 TBL_COMMIT   (W)  write 1 -> commit the staged entry to table[TBL_IDX]
 *   80 TBL_USTART   (RW) the window's start in the uwin (bytes); its own
 *                        64 B line, for the reason given at TX_CTL below
 *   16 RDMA_STAGING_VA (RW) unused (every packet's RETH is its own reference)
 *   14/15 (RO) rx ingress FIFO full while the shell had a beat; longest run
 *   24 (RO) rx longest stall run          26 (RO) rx orphan beats
 *   30/31 (RO) rx backpressure; longest run
 *   36 (RO) rx writes forwarded           41 (RO) rx headers rejected
 *   42-44 (RO) rx move / starve / stall   47 (RO) rx requests accepted
 *   48 (RO) free-running cycle counter
 *   66 TX_CTL (RW) [7:0] ack window: rdma packets posted and not yet acked,
 *                  0 = none; reset 16
 *   67 (RO) packets unacked now   68 (RO) acks
 *   69 (RO) cycles a packet waited on the window
 *   70 (RO) cycles a packet waited on sq_wr.ready
 *   72/73 (RO) cycles a local / rdma request waited on sq_wr.ready
 *   74 (RO) cycles the ingress had a request while loom_rx's was presented
 *   75 (RO) cycles loom_rx had a request and was not presented (must be 0)
 *   76 RX_CHUNK (RW) [3:0] unused since loom_rx posts one write per packet; reset 1
 *   81-84 (RO) the shell's host DMA boundary: 81 beats moved, 82 cycles a
 *      beat waited on the DMA engine, 83 the longest such run, 84 cycles a
 *      write request waited on the engine
 *   88-94 (RO) ingress: 88 bursts, 89 bursts dropped, 90 local packets,
 *          91 rdma packets, 92 stores, 93 beats with a partial 8 B word,
 *          94 packets closed by the idle timer
 *   95-108 (RO) ingress debug (loom_ingress cnt_dbg, bit = word - 95):
 *          95/96 cycles the host / net output was valid and not ready,
 *          97 cycles sending a packet with the data FIFO empty,
 *          98-102 cycles a W beat waited: 98 no burst looked up yet, 99 B slot
 *          busy, 100 data FIFO full, 101 queue full, 102 a partial beat
 *          still sending its stores; 103/104 bursts dropped for no window /
 *          past the window's end; 105-107 bursts of 1, 2-4, more than 4
 *          beats; 108 bursts at a non-64 B-aligned address
 *   176-181 the export table (loom_exports), staged then committed like
 *          the window table: 176 EXP_IDX, 177 EXP_CFG (bit0 valid), 178
 *          EXP_PID (landing pid), 179 EXP_BASE (landing VA), 180 EXP_LEN
 *          (bytes), 181 EXP_COMMIT (write 1)
 *   112-130 (RO) cnt_x[i] at word 112+i, its longest run of consecutive
 *          cycles at 144+i (since the bitstream was loaded):
 *     loom_rx:  112 bulk packets landed (writes posted), 113 stores landed,
 *               114 packets dropped (no export, out of bounds, bad
 *               inline), 115 cycles a write waited on sq_wr, 116 cycles an
 *               announced packet waited on RX_WR_OUTSTANDING, 117 cycles a
 *               beat waited for its packet's write to be posted, 118 rq_wr
 *               lost to a full announcement queue (must be 0)
 *     ingress:  119 full (PMTU) rdma packets, 120 partial rdma packets
 *               closed by the idle timer, 121 partial rdma packets closed
 *               by a non-continuing write or a store
 *     shell (dynamic_top / user_wrapper dbg_host_out): 122 host DMA
 *               write requests issued, 123 cycles a host write request
 *               waited to enter the MMU, 124 requests the MMU took, 125
 *               cycles host write data waited inside the shell for its
 *               request to come out of the MMU (data ahead of the request
 *               order), 126 page-fault interrupts, 127 completion writebacks,
 *               128 cycles a writeback waited, 129 cycles loom_rx's (dest 1)
 *               write request waited in the credit stage for its data,
 *               130 cycles it waited there to go downstream (to the MMU)
 *     shell MMU, region 0's write FSM (tlb_fsm dbg): 131 cycles a host
 *               request waited for a DMA completion (N_TLB_ACTV issued,
 *               none done), 132 cycles it waited on the DMA request port,
 *               133 cycles waiting for the TLB mutex, 134 cycles in a miss,
 *               invalidation or locked state, 135 host DMA completions
 * Counters are free-running and never cleared (software takes deltas). The
 * ingress pulses are registered once before they count.
 */
module loom_ctrl #(
    parameter integer N_DBG = 14,
    parameter integer N_X   = 24
) (
    input  logic                        aclk,
    input  logic                        aresetn,

    AXI4L.s                             axi_ctrl,

    // Table programming (to loom_table)
    output logic                        tbl_commit,
    output logic [3:0]                  tbl_idx,
    output logic                        tbl_valid,
    output logic                        tbl_route,
    output logic [PID_BITS-1:0]         tbl_pid,
    output logic [PID_BITS-1:0]         tbl_dst_pid,
    output logic [VADDR_BITS-1:0]       tbl_base,
    output logic [LEN_BITS-1:0]         tbl_len,
    output logic [26:0]                 tbl_ustart,

    output logic [VADDR_BITS-1:0]       rdma_staging_va,

    // Export programming (to loom_exports)
    output logic                        exp_commit,
    output logic [7:0]                  exp_idx,
    output logic                        exp_valid,
    output logic [PID_BITS-1:0]         exp_pid,
    output logic [VADDR_BITS-1:0]       exp_base,
    output logic [39:0]                 exp_len,
    output logic [7:0]                  tx_window,
    output logic [3:0]                  rx_chunk,

    // Ack window state and waits
    input  logic [15:0]                 tx_inflight,
    input  logic                        cnt_tx_ack,
    input  logic                        cnt_tx_winfull,
    input  logic                        cnt_tx_reqwait,

    // sq_wr accounting
    input  logic                        cnt_wr_wait_local,
    input  logic                        cnt_wr_wait_rdma,
    input  logic                        cnt_wr_blk_ing,
    input  logic                        cnt_wr_blk_rx,

    // loom_rx
    input  logic                        cnt_rx_fwd,
    input  logic                        cnt_rx_drop,
    input  logic                        cnt_rx_orphan,
    input  logic                        cnt_rx_move,
    input  logic                        cnt_rx_starve,
    input  logic                        cnt_rx_stall,
    input  logic                        cnt_rx_bp,
    input  logic                        cnt_rx_req,
    input  logic                        cnt_rx_fifo_full,

    // the shell's host DMA boundary (dynamic_top dbg_host_out): a beat moved,
    // a beat waited on the DMA engine, a write request waited on it
    input  logic                        cnt_hout_move,
    input  logic                        cnt_hout_bp,
    input  logic                        cnt_hreq_bp,

    // loom_ingress
    input  logic                        cnt_ing_burst,
    input  logic                        cnt_ing_drop,
    input  logic                        cnt_ing_pkt_local,
    input  logic                        cnt_ing_pkt_rdma,
    input  logic                        cnt_ing_store,
    input  logic                        cnt_ing_store_drop,
    input  logic                        cnt_ing_flush,
    input  logic [N_DBG-1:0]            cnt_ing_dbg,

    // Further pulses, word 112+i (list in the map above)
    input  logic [N_X-1:0]              cnt_x
);

localparam integer ADDR_LSB = $clog2(AXIL_DATA_BITS/8);   // 3
localparam integer CSR_BITS = 9;                          // 512 words in the CSR page

localparam integer R_TBL_IDX      = 0;
localparam integer R_TBL_CFG      = 1;
localparam integer R_TBL_PID      = 2;
localparam integer R_TBL_BASE     = 3;
localparam integer R_TBL_LEN      = 4;
localparam integer R_TBL_COMMIT   = 5;
localparam integer R_TBL_USTART   = 80;
localparam integer R_RDMA_STAGING = 16;
localparam integer R_RX_FIFO_FULL     = 14;
localparam integer R_RX_FIFO_FULL_MAX = 15;
localparam integer R_RX_STALL_MAX = 24;
localparam integer R_RX_ORPHAN    = 26;
localparam integer R_RX_BP        = 30;
localparam integer R_RX_BP_MAX    = 31;
localparam integer R_RX_FWD       = 36;
localparam integer R_RX_DROP      = 41;
localparam integer R_RX_MOVE      = 42;
localparam integer R_RX_STARVE    = 43;
localparam integer R_RX_STALL     = 44;
localparam integer R_RX_REQ       = 47;
localparam integer R_CYC          = 48;
// Hardware writes to words 0-5 have been seen to clobber words 4 and 6 of
// the same 64 B line (examples/loom csr_probe.cpp), so a register that must
// survive table programming sits on a line of its own
localparam integer R_TX_CTL        = 66;
localparam integer R_TX_STATE      = 67;
localparam integer R_TX_ACKS       = 68;
localparam integer R_TX_WINFULL    = 69;
localparam integer R_TX_REQWAIT    = 70;
localparam integer R_WR_WAIT_LOCAL = 72;
localparam integer R_WR_WAIT_RDMA  = 73;
localparam integer R_WR_BLK_ING    = 74;
localparam integer R_WR_BLK_RX     = 75;
localparam integer R_RX_CHUNK      = 76;
localparam integer R_HOUT_MOVE     = 81;
localparam integer R_HOUT_BP       = 82;
localparam integer R_HOUT_BP_MAX   = 83;
localparam integer R_HREQ_BP       = 84;
localparam integer R_ING_BASE      = 88;
localparam integer N_ING           = 7;
localparam integer R_DBG_BASE      = 95;
localparam integer R_X_BASE        = 112;
localparam integer R_EXP_IDX       = 176;
localparam integer R_EXP_CFG       = 177;
localparam integer R_EXP_PID       = 178;
localparam integer R_EXP_BASE      = 179;
localparam integer R_EXP_LEN       = 180;
localparam integer R_EXP_COMMIT    = 181;
localparam integer R_XMAX_BASE     = 144;

// -------------------------------------------------------------------------
// AXI4-Lite handshake (single outstanding write and read, as examples/loom)
// -------------------------------------------------------------------------
logic [15:0] axi_awaddr, axi_araddr;
logic        axi_awready, axi_arready, axi_wready, axi_bvalid, axi_rvalid, aw_en;
logic [1:0]  axi_bresp, axi_rresp;
logic [AXIL_DATA_BITS-1:0] axi_rdata;

wire ctrl_reg_wren = axi_wready && axi_ctrl.wvalid && axi_awready && axi_ctrl.awvalid;
wire ctrl_reg_rden = axi_arready && axi_ctrl.arvalid && ~axi_rvalid;
// Only the CSR page (the region's first 4 KB) has registers
wire [CSR_BITS-1:0] wr_idx = axi_awaddr[ADDR_LSB +: CSR_BITS];
wire [CSR_BITS-1:0] rd_idx = axi_araddr[ADDR_LSB +: CSR_BITS];
wire csr_wr  = ctrl_reg_wren && (axi_awaddr[15:12] == 4'd0);
wire csr_rd  = (axi_araddr[15:12] == 4'd0);

// -------------------------------------------------------------------------
// CSRs: stage-then-commit for the table entry. COMMIT fires on the write
// pulse and needs wstrb[0] && wdata[0], so a write with empty strobes (the
// padding around a host ctrl write's line) cannot fire it; other registers
// take full-strobe writes only.
// -------------------------------------------------------------------------
logic [63:0] r_tbl_idx, r_tbl_cfg, r_tbl_pid, r_tbl_base, r_tbl_len, r_tbl_ustart;
logic [63:0] r_rdma_staging, r_tx_ctl, r_rx_chunk;
logic [63:0] r_exp_idx, r_exp_cfg, r_exp_pid, r_exp_base, r_exp_len;

wire commit_pulse = csr_wr && (wr_idx == R_TBL_COMMIT) && axi_ctrl.wstrb[0] && axi_ctrl.wdata[0];
wire exp_pulse    = csr_wr && (wr_idx == R_EXP_COMMIT) && axi_ctrl.wstrb[0] && axi_ctrl.wdata[0];

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        r_tbl_idx <= 0; r_tbl_cfg <= 0; r_tbl_pid <= 0; r_tbl_base <= 0; r_tbl_len <= 0;
        r_tbl_ustart <= 0; r_rdma_staging <= 0;
        r_exp_idx <= 0; r_exp_cfg <= 0; r_exp_pid <= 0; r_exp_base <= 0; r_exp_len <= 0;
        r_tx_ctl   <= 64'd16;
        r_rx_chunk <= 64'd1;
    end else if (csr_wr && (&axi_ctrl.wstrb)) begin
        case (wr_idx)
            R_TBL_IDX:      r_tbl_idx      <= axi_ctrl.wdata;
            R_TBL_CFG:      r_tbl_cfg      <= axi_ctrl.wdata;
            R_TBL_PID:      r_tbl_pid      <= axi_ctrl.wdata;
            R_TBL_BASE:     r_tbl_base     <= axi_ctrl.wdata;
            R_TBL_LEN:      r_tbl_len      <= axi_ctrl.wdata;
            R_TBL_USTART:   r_tbl_ustart   <= axi_ctrl.wdata;
            R_RDMA_STAGING: r_rdma_staging <= axi_ctrl.wdata;
            R_TX_CTL:       r_tx_ctl       <= axi_ctrl.wdata;
            R_RX_CHUNK:     r_rx_chunk     <= axi_ctrl.wdata;
            R_EXP_IDX:      r_exp_idx      <= axi_ctrl.wdata;
            R_EXP_CFG:      r_exp_cfg      <= axi_ctrl.wdata;
            R_EXP_PID:      r_exp_pid      <= axi_ctrl.wdata;
            R_EXP_BASE:     r_exp_base     <= axi_ctrl.wdata;
            R_EXP_LEN:      r_exp_len      <= axi_ctrl.wdata;
            default: ;
        endcase
    end
end

assign tbl_commit      = commit_pulse;
assign tbl_idx         = r_tbl_idx[3:0];
assign tbl_valid       = r_tbl_cfg[0];
assign tbl_route       = r_tbl_cfg[1];
assign tbl_pid         = r_tbl_pid[PID_BITS-1:0];
assign tbl_dst_pid     = r_tbl_pid[8 +: PID_BITS];
assign tbl_base        = r_tbl_base[VADDR_BITS-1:0];
assign tbl_len         = r_tbl_len[LEN_BITS-1:0];
assign tbl_ustart      = r_tbl_ustart[26:0];
assign rdma_staging_va = r_rdma_staging[VADDR_BITS-1:0];
assign tx_window       = r_tx_ctl[7:0];
assign rx_chunk        = (r_rx_chunk[3:0] == 4'd0) ? 4'd1 : r_rx_chunk[3:0];   // 0 is not a size
assign exp_commit      = exp_pulse;
assign exp_idx         = r_exp_idx[7:0];
assign exp_valid       = r_exp_cfg[0];
assign exp_pid         = r_exp_pid[PID_BITS-1:0];
assign exp_base        = r_exp_base[VADDR_BITS-1:0];
assign exp_len         = r_exp_len[39:0];

// -------------------------------------------------------------------------
// Counters
// -------------------------------------------------------------------------
logic [63:0] cycle_cnt;
logic [63:0] rx_fwd, rx_drop, rx_orphan, rx_move, rx_starve, rx_stall, rx_req;
logic [63:0] rx_stall_run, rx_stall_max, rx_bp, rx_bp_run, rx_bp_max, rx_ff, rx_ff_run, rx_ff_max;
logic [63:0] tx_acks, tx_winfull, tx_reqwait;
logic [63:0] wr_wait_local, wr_wait_rdma, wr_blk_ing, wr_blk_rx;
logic [63:0] hout_move, hout_bp, hout_bp_run, hout_bp_max, hreq_bp;
logic [63:0] ing [N_ING];
logic [63:0] dbg [N_DBG];
logic [N_ING-1:0] ing_pulse;
logic [N_DBG-1:0] dbg_pulse;
logic [N_X-1:0]   x_pulse;
logic [63:0]      xc [N_X];
logic [31:0]      xrun [N_X], xmax [N_X];
always_ff @(posedge aclk) begin
    if (!aresetn) begin
        ing_pulse <= '0;
        dbg_pulse <= '0;
        x_pulse   <= '0;
    end else begin
        x_pulse   <= cnt_x;
        ing_pulse <= {cnt_ing_flush, cnt_ing_store_drop, cnt_ing_store, cnt_ing_pkt_rdma,
                      cnt_ing_pkt_local, cnt_ing_drop, cnt_ing_burst};
        dbg_pulse <= cnt_ing_dbg;
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        cycle_cnt <= 0;
        rx_fwd <= 0; rx_drop <= 0; rx_orphan <= 0; rx_move <= 0; rx_starve <= 0; rx_stall <= 0; rx_req <= 0;
        rx_stall_run <= 0; rx_stall_max <= 0; rx_bp <= 0; rx_bp_run <= 0; rx_bp_max <= 0;
        rx_ff <= 0; rx_ff_run <= 0; rx_ff_max <= 0;
        tx_acks <= 0; tx_winfull <= 0; tx_reqwait <= 0;
        wr_wait_local <= 0; wr_wait_rdma <= 0; wr_blk_ing <= 0; wr_blk_rx <= 0;
        hout_move <= 0; hout_bp <= 0; hout_bp_run <= 0; hout_bp_max <= 0; hreq_bp <= 0;
        for (int i = 0; i < N_ING; i++) ing[i] <= 0;
        for (int i = 0; i < N_DBG; i++) dbg[i] <= 0;
    end else begin
        cycle_cnt <= cycle_cnt + 1;
        if (cnt_rx_fwd)    rx_fwd    <= rx_fwd + 1;
        if (cnt_rx_drop)   rx_drop   <= rx_drop + 1;
        if (cnt_rx_orphan) rx_orphan <= rx_orphan + 1;
        if (cnt_rx_move)   rx_move   <= rx_move + 1;
        if (cnt_rx_starve) rx_starve <= rx_starve + 1;
        if (cnt_rx_stall)  rx_stall  <= rx_stall + 1;
        if (cnt_rx_req)    rx_req    <= rx_req + 1;
        if (cnt_rx_stall) begin
            rx_stall_run <= rx_stall_run + 1;
            if (rx_stall_run + 1 > rx_stall_max) rx_stall_max <= rx_stall_run + 1;
        end else rx_stall_run <= 0;
        if (cnt_rx_bp) begin
            rx_bp     <= rx_bp + 1;
            rx_bp_run <= rx_bp_run + 1;
            if (rx_bp_run + 1 > rx_bp_max) rx_bp_max <= rx_bp_run + 1;
        end else rx_bp_run <= 0;
        if (cnt_rx_fifo_full) begin
            rx_ff     <= rx_ff + 1;
            rx_ff_run <= rx_ff_run + 1;
            if (rx_ff_run + 1 > rx_ff_max) rx_ff_max <= rx_ff_run + 1;
        end else rx_ff_run <= 0;
        if (cnt_tx_ack)        tx_acks       <= tx_acks + 1;
        if (cnt_tx_winfull)    tx_winfull    <= tx_winfull + 1;
        if (cnt_tx_reqwait)    tx_reqwait    <= tx_reqwait + 1;
        if (cnt_wr_wait_local) wr_wait_local <= wr_wait_local + 1;
        if (cnt_wr_wait_rdma)  wr_wait_rdma  <= wr_wait_rdma + 1;
        if (cnt_wr_blk_ing)    wr_blk_ing    <= wr_blk_ing + 1;
        if (cnt_wr_blk_rx)     wr_blk_rx     <= wr_blk_rx + 1;
        if (cnt_hout_move)     hout_move     <= hout_move + 1;
        if (cnt_hreq_bp)       hreq_bp       <= hreq_bp + 1;
        if (cnt_hout_bp) begin
            hout_bp     <= hout_bp + 1;
            hout_bp_run <= hout_bp_run + 1;
            if (hout_bp_run + 1 > hout_bp_max) hout_bp_max <= hout_bp_run + 1;
        end else hout_bp_run <= 0;
        for (int i = 0; i < N_ING; i++) if (ing_pulse[i]) ing[i] <= ing[i] + 1;
        for (int i = 0; i < N_DBG; i++) if (dbg_pulse[i]) dbg[i] <= dbg[i] + 1;
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        for (int i = 0; i < N_X; i++) begin xc[i] <= 0; xrun[i] <= 0; xmax[i] <= 0; end
    end else begin
        for (int i = 0; i < N_X; i++) begin
            if (x_pulse[i]) begin
                xc[i]   <= xc[i] + 1;
                xrun[i] <= xrun[i] + 1;
                if (xrun[i] + 1 > xmax[i]) xmax[i] <= xrun[i] + 1;
            end else xrun[i] <= 0;
        end
    end
end

// -------------------------------------------------------------------------
// Read data
// -------------------------------------------------------------------------
always_ff @(posedge aclk) begin
    if (!aresetn) axi_rdata <= 0;
    else if (ctrl_reg_rden) begin
        axi_rdata <= 0;
        if (csr_rd) case (rd_idx)
            R_TBL_IDX:          axi_rdata <= r_tbl_idx;
            R_TBL_CFG:          axi_rdata <= r_tbl_cfg;
            R_TBL_PID:          axi_rdata <= r_tbl_pid;
            R_TBL_BASE:         axi_rdata <= r_tbl_base;
            R_TBL_LEN:          axi_rdata <= r_tbl_len;
            R_TBL_USTART:       axi_rdata <= r_tbl_ustart;
            R_RDMA_STAGING:     axi_rdata <= r_rdma_staging;
            R_RX_FIFO_FULL:     axi_rdata <= rx_ff;
            R_RX_FIFO_FULL_MAX: axi_rdata <= rx_ff_max;
            R_RX_STALL_MAX:     axi_rdata <= rx_stall_max;
            R_RX_ORPHAN:        axi_rdata <= rx_orphan;
            R_RX_BP:            axi_rdata <= rx_bp;
            R_RX_BP_MAX:        axi_rdata <= rx_bp_max;
            R_HOUT_MOVE:        axi_rdata <= hout_move;
            R_HOUT_BP:          axi_rdata <= hout_bp;
            R_HOUT_BP_MAX:      axi_rdata <= hout_bp_max;
            R_HREQ_BP:          axi_rdata <= hreq_bp;
            R_RX_FWD:           axi_rdata <= rx_fwd;
            R_RX_DROP:          axi_rdata <= rx_drop;
            R_RX_MOVE:          axi_rdata <= rx_move;
            R_RX_STARVE:        axi_rdata <= rx_starve;
            R_RX_STALL:         axi_rdata <= rx_stall;
            R_RX_REQ:           axi_rdata <= rx_req;
            R_CYC:              axi_rdata <= cycle_cnt;
            R_TX_CTL:           axi_rdata <= r_tx_ctl;
            R_TX_STATE:         axi_rdata <= {48'b0, tx_inflight};
            R_TX_ACKS:          axi_rdata <= tx_acks;
            R_TX_WINFULL:       axi_rdata <= tx_winfull;
            R_TX_REQWAIT:       axi_rdata <= tx_reqwait;
            R_WR_WAIT_LOCAL:    axi_rdata <= wr_wait_local;
            R_WR_WAIT_RDMA:     axi_rdata <= wr_wait_rdma;
            R_WR_BLK_ING:       axi_rdata <= wr_blk_ing;
            R_WR_BLK_RX:        axi_rdata <= wr_blk_rx;
            R_RX_CHUNK:         axi_rdata <= {60'b0, rx_chunk};
            R_EXP_IDX:          axi_rdata <= r_exp_idx;
            R_EXP_CFG:          axi_rdata <= r_exp_cfg;
            R_EXP_PID:          axi_rdata <= r_exp_pid;
            R_EXP_BASE:         axi_rdata <= r_exp_base;
            R_EXP_LEN:          axi_rdata <= r_exp_len;
            default:
                if (rd_idx >= R_ING_BASE && rd_idx < R_ING_BASE + N_ING)
                    axi_rdata <= ing[rd_idx - R_ING_BASE];
                else if (rd_idx >= R_DBG_BASE && rd_idx < R_DBG_BASE + N_DBG)
                    axi_rdata <= dbg[rd_idx - R_DBG_BASE];
                else if (rd_idx >= R_X_BASE && rd_idx < R_X_BASE + N_X)
                    axi_rdata <= xc[rd_idx - R_X_BASE];
                else if (rd_idx >= R_XMAX_BASE && rd_idx < R_XMAX_BASE + N_X)
                    axi_rdata <= {32'b0, xmax[rd_idx - R_XMAX_BASE]};
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
