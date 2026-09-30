import lynxTypes::*;

/**
 * loom_ce_ctrl
 *
 * AXI4-Lite CSR page of the emulated accelerator's copy engine (loom_ce).
 * Software stages a copy and starts it; the engine reports progress in
 * read-only counters.
 *
 * CSR map (64-bit word indices; byte offset = idx * 8):
 *   0 START    (W)  write 1 -> start the staged copy (ignored while busy)
 *   1 SRC_VA   (RW) source VA in card memory (HBM), 64 B-aligned
 *   2 DST_VA   (RW) destination VA: on the V80 MMU, the U280's uwin (dma-buf)
 *   3 LEN      (RW) bytes to copy, a multiple of 64
 *   4 PID      (RW) the cThread (ctid) whose address space both VAs are in
 *   5 FENCE_VA (RW) where the completion count is written after the data,
 *                   in the same address space; 0 = no fence
 *   8 BUSY     (RO) a copy is in flight
 *   9 COPIES   (RO) copies finished (the fence count)
 *  10 CYCLES   (RO) cycles from START to the data write's completion, last copy
 *  48 CE_OUT_BP  (RO) cycles the copy's write stream was valid and not ready
 *                     (the peer-to-peer writes into the U280 held back)
 *  49 CE_IN_WAIT (RO) cycles the copy waited for card data (HBM read)
 *  50 WR_WAIT    (RO) cycles a write request waited on sq_wr
 * The counters are free-running (software takes deltas); the pulses are
 * registered once before they count.
 */
module loom_ce_ctrl (
    input  logic                        aclk,
    input  logic                        aresetn,

    AXI4L.s                             axi_ctrl,

    output logic                        start,
    output logic [VADDR_BITS-1:0]       src_va,
    output logic [VADDR_BITS-1:0]       dst_va,
    output logic [LEN_BITS-1:0]         len,
    output logic [PID_BITS-1:0]         pid,
    output logic [VADDR_BITS-1:0]       fence_va,

    input  logic                        busy,
    input  logic [31:0]                 copies,
    input  logic [63:0]                 cycles,

    input  logic                        cnt_ce_out_bp,
    input  logic                        cnt_ce_in_wait,
    input  logic                        cnt_wr_wait
);

localparam integer ADDR_LSB = $clog2(AXIL_DATA_BITS/8);
localparam integer CSR_BITS = 6;

localparam integer R_START    = 0;
localparam integer R_SRC_VA   = 1;
localparam integer R_DST_VA   = 2;
localparam integer R_LEN      = 3;
localparam integer R_PID      = 4;
localparam integer R_FENCE_VA = 5;
localparam integer R_BUSY     = 8;
localparam integer R_COPIES   = 9;
localparam integer R_CYCLES   = 10;
localparam integer R_PERF     = 48;   // 48-50, in the order of perf_pulse below
localparam integer N_PERF     = 3;

logic [15:0] axi_awaddr, axi_araddr;
logic        axi_awready, axi_arready, axi_wready, axi_bvalid, axi_rvalid, aw_en;
logic [1:0]  axi_bresp, axi_rresp;
logic [AXIL_DATA_BITS-1:0] axi_rdata;

wire ctrl_reg_wren = axi_wready && axi_ctrl.wvalid && axi_awready && axi_ctrl.awvalid;
wire ctrl_reg_rden = axi_arready && axi_ctrl.arvalid && ~axi_rvalid;
wire [CSR_BITS-1:0] wr_idx = axi_awaddr[ADDR_LSB +: CSR_BITS];
wire [CSR_BITS-1:0] rd_idx = axi_araddr[ADDR_LSB +: CSR_BITS];

// START fires on the write pulse and needs wstrb[0] && wdata[0], so the
// empty-strobe writes around a host ctrl write's line cannot fire it; the
// other registers take full-strobe writes only (as examples/loom)
logic [63:0] r_src, r_dst, r_len, r_pid, r_fence;
assign start = ctrl_reg_wren && (wr_idx == R_START) && axi_ctrl.wstrb[0] && axi_ctrl.wdata[0];

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        r_src <= 0; r_dst <= 0; r_len <= 0; r_pid <= 0; r_fence <= 0;
    end else if (ctrl_reg_wren && (&axi_ctrl.wstrb)) begin
        case (wr_idx)
            R_SRC_VA:   r_src   <= axi_ctrl.wdata;
            R_DST_VA:   r_dst   <= axi_ctrl.wdata;
            R_LEN:      r_len   <= axi_ctrl.wdata;
            R_PID:      r_pid   <= axi_ctrl.wdata;
            R_FENCE_VA: r_fence <= axi_ctrl.wdata;
            default: ;
        endcase
    end
end

assign src_va   = r_src[VADDR_BITS-1:0];
assign dst_va   = r_dst[VADDR_BITS-1:0];
assign len      = r_len[LEN_BITS-1:0];
assign pid      = r_pid[PID_BITS-1:0];
assign fence_va = r_fence[VADDR_BITS-1:0];

logic [63:0]       perf_cnt [N_PERF];
logic [N_PERF-1:0] perf_pulse;
always_ff @(posedge aclk) begin
    if (!aresetn) begin
        perf_pulse <= '0;
        for (int i = 0; i < N_PERF; i++) perf_cnt[i] <= 0;
    end else begin
        perf_pulse <= {cnt_wr_wait, cnt_ce_in_wait, cnt_ce_out_bp};
        for (int i = 0; i < N_PERF; i++) if (perf_pulse[i]) perf_cnt[i] <= perf_cnt[i] + 1;
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) axi_rdata <= 0;
    else if (ctrl_reg_rden) begin
        case (rd_idx)
            R_SRC_VA:   axi_rdata <= r_src;
            R_DST_VA:   axi_rdata <= r_dst;
            R_LEN:      axi_rdata <= r_len;
            R_PID:      axi_rdata <= r_pid;
            R_FENCE_VA: axi_rdata <= r_fence;
            R_BUSY:     axi_rdata <= {63'b0, busy};
            R_COPIES:   axi_rdata <= {32'b0, copies};
            R_CYCLES:   axi_rdata <= cycles;
            default:
                if (rd_idx >= R_PERF && rd_idx < R_PERF + N_PERF)
                    axi_rdata <= perf_cnt[rd_idx - R_PERF];
                else
                    axi_rdata <= 0;
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
    end else if (~axi_arready && axi_ctrl.arvalid) begin
        axi_arready <= 1'b1; axi_araddr <= axi_ctrl.araddr[15:0];
    end else begin
        axi_arready <= 1'b0;
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_bvalid <= 0; axi_bresp <= 2'b0;
    end else if (axi_awready && axi_ctrl.awvalid && ~axi_bvalid && axi_wready && axi_ctrl.wvalid) begin
        axi_bvalid <= 1'b1; axi_bresp <= 2'b0;
    end else if (axi_ctrl.bready && axi_bvalid) begin
        axi_bvalid <= 1'b0;
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) axi_wready <= 1'b0;
    else if (~axi_wready && axi_ctrl.wvalid && axi_ctrl.awvalid && aw_en) axi_wready <= 1'b1;
    else axi_wready <= 1'b0;
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        axi_rvalid <= 0; axi_rresp <= 0;
    end else if (axi_arready && axi_ctrl.arvalid && ~axi_rvalid) begin
        axi_rvalid <= 1'b1; axi_rresp <= 2'b0;
    end else if (axi_rvalid && axi_ctrl.rready) begin
        axi_rvalid <= 1'b0;
    end
end

endmodule
