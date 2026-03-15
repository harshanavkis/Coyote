/**
 * Jigsaw vFPGA Top
 *
 * Uses RDMA WRITE as transport. The jigsaw protocol handles all addressing
 * internally — the RoCE stack is just a pipe. RDMA WRITE submissions use
 * APP_WRITE with RDMA_MODE_PARSE for automatic PMTU fragmentation.
 * PID is set once at init via MMIO. Remote vaddr is set after QP exchange.
 */

// ============================================================================
// Pipeline registers for host AXI streams
// ============================================================================
AXI4SR axis_in_int (.*);
axisr_reg inst_reg_in  (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_host_recv[0]), .m_axis(axis_in_int));

AXI4SR axis_out_int (.*);
axisr_reg inst_reg_out (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_out_int), .m_axis(axis_host_send[0]));

// ============================================================================
// MMIO / control signals
// ============================================================================
(* mark_debug = "true" *) logic [VADDR_BITS-1:0] mmio_vaddr;
(* mark_debug = "true" *) logic mmio_ctrl;
(* mark_debug = "true" *) logic mmio_clear;
(* mark_debug = "true" *) logic mmio_write_done;
(* mark_debug = "true" *) logic mmio_read_done;
(* mark_debug = "true" *) logic [PID_BITS-1:0] coyote_pid;
(* mark_debug = "true" *) logic [VADDR_BITS-1:0] remote_vaddr;

// Local DMA SQ signals
(* mark_debug = "true" *) logic sq_valid_write;
(* mark_debug = "true" *) logic sq_dir_write;
(* mark_debug = "true" *) logic [VADDR_BITS-1:0] sq_addr_write;
(* mark_debug = "true" *) logic [LEN_BITS-1:0] sq_len_write;
(* mark_debug = "true" *) logic sq_valid_read;
(* mark_debug = "true" *) logic sq_dir_read;
(* mark_debug = "true" *) logic [VADDR_BITS-1:0] sq_addr_read;
(* mark_debug = "true" *) logic [LEN_BITS-1:0] sq_len_read;

// RDMA submission signals
(* mark_debug = "true" *) logic rdma_wr_valid;
(* mark_debug = "true" *) logic [LEN_BITS-1:0] rdma_wr_len;

// ============================================================================
// AXI control register parser
// ============================================================================
jigsaw_hc_axi_ctrl_parser inst_axi_ctrl_parser (
    .aclk(aclk),
    .aresetn(aresetn),
    .axi_ctrl(axi_ctrl),
    .mmio_vaddr(mmio_vaddr),
    .mmio_ctrl(mmio_ctrl),
    .mmio_clear(mmio_clear),
    .mmio_write_done(mmio_write_done),
    .mmio_read_done(mmio_read_done),
    .coyote_pid(coyote_pid),
    .remote_vaddr(remote_vaddr)
);

// ============================================================================
// Jigsaw host controller
// ============================================================================
jigsaw_host_controller #(
    .ADDR_WIDTH(64),
    .LEN_WIDTH(64)
) inst_host_controller (
    .aclk(aclk),
    .aresetn(aresetn),

    // Network side — incoming RDMA WRITEs from remote
    .network_in_tdata(axis_rrsp_recv[0].tdata),
    .network_in_tkeep(axis_rrsp_recv[0].tkeep),
    .network_in_tvalid(axis_rrsp_recv[0].tvalid),
    .network_in_tready(axis_rrsp_recv[0].tready),
    .network_in_tlast(axis_rrsp_recv[0].tlast),
    .network_in_tuser(1'b0),

    // Network side — outgoing RDMA WRITEs
    .network_out_tdata(axis_rreq_send[0].tdata),
    .network_out_tkeep(axis_rreq_send[0].tkeep),
    .network_out_tvalid(axis_rreq_send[0].tvalid),
    .network_out_tready(axis_rreq_send[0].tready),
    .network_out_tlast(axis_rreq_send[0].tlast),
    .network_out_tuser(),

    // Host side — DMA from/to host memory
    .host_in_tdata(axis_in_int.tdata),
    .host_in_tkeep(axis_in_int.tkeep),
    .host_in_tvalid(axis_in_int.tvalid),
    .host_in_tready(axis_in_int.tready),
    .host_in_tlast(axis_in_int.tlast),
    .host_in_tuser(1'b0),

    .host_out_tdata(axis_out_int.tdata),
    .host_out_tkeep(axis_out_int.tkeep),
    .host_out_tvalid(axis_out_int.tvalid),
    .host_out_tready(axis_out_int.tready),
    .host_out_tlast(axis_out_int.tlast),
    .host_out_tuser(),

    // Local DMA submission
    .sq_valid_write(sq_valid_write),
    .sq_dir_write(sq_dir_write),
    .sq_addr_write(sq_addr_write),
    .sq_len_write(sq_len_write),

    .sq_valid_read(sq_valid_read),
    .sq_dir_read(sq_dir_read),
    .sq_addr_read(sq_addr_read),
    .sq_len_read(sq_len_read),

    // MMIO control
    .mmio_vaddr(mmio_vaddr),
    .mmio_ctrl(mmio_ctrl),
    .mmio_clear(mmio_clear),
    .mmio_write_done(mmio_write_done),
    .mmio_read_done(mmio_read_done),

    // RDMA submission
    .rdma_wr_valid(rdma_wr_valid),
    .rdma_wr_len(rdma_wr_len),
    .rdma_wr_ready(sq_wr.ready)
);

// ============================================================================
// SQ submission logic
// ============================================================================
always_comb begin
    // ----- Local DMA READ -----
    sq_rd.data          = 0;
    sq_rd.data.last     = 1'b1;
    sq_rd.data.pid      = coyote_pid;
    sq_rd.data.len      = sq_len_read;
    sq_rd.data.vaddr    = sq_addr_read;
    sq_rd.data.strm     = STRM_HOST;
    sq_rd.data.opcode   = LOCAL_READ;
    sq_rd.valid         = sq_valid_read;
    cq_rd.ready         = 1'b1;

    // ----- RDMA WRITE or Local DMA WRITE -----
    sq_wr.data          = 0;
    if (rdma_wr_valid) begin
        // RDMA WRITE — auto-fragmented by rdma_req_parser
        sq_wr.data.last     = 1'b1;
        sq_wr.data.pid      = coyote_pid;
        sq_wr.data.len      = rdma_wr_len;
        sq_wr.data.vaddr    = remote_vaddr;     // WRITE target address on remote
        sq_wr.data.strm     = STRM_RDMA;
        sq_wr.data.remote   = 1'b1;
        sq_wr.data.opcode   = APP_WRITE;        // parser fragments into RC_RDMA_WRITE_*
        sq_wr.data.mode     = 1'b0;            // RDMA_MODE_PARSE
        sq_wr.data.dest     = 0;               // axi channel 0
        sq_wr.valid         = 1'b1;
    end else begin
        // Local DMA WRITE
        sq_wr.data.last     = 1'b1;
        sq_wr.data.pid      = coyote_pid;
        sq_wr.data.len      = sq_len_write;
        sq_wr.data.vaddr    = sq_addr_write;
        sq_wr.data.strm     = STRM_HOST;
        sq_wr.data.opcode   = LOCAL_WRITE;
        sq_wr.valid         = sq_valid_write;
    end
    cq_wr.ready         = 1'b1;
end

// ============================================================================
// Tie off unused interfaces
// ============================================================================
always_comb axis_rreq_recv[0].tie_off_s();   // not receiving RDMA READ responses
always_comb axis_rrsp_send[0].tie_off_m();   // not sending RDMA READ responses
always_comb rq_rd.ready = 1'b1;              // drain incoming RDMA read requests
always_comb rq_wr.ready = 1'b1;              // drain incoming RDMA write requests
always_comb notify.tie_off_m();              // not using notifications