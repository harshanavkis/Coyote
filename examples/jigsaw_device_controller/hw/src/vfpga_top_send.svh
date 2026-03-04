/**
 * Jigsaw Device Controller vFPGA Top
 *
 * txn_generator connected directly to RDMA stack.
 * Incoming RDMA SENDs (from host) arrive on axis_rrsp_recv[0] → txn_generator_in
 * Outgoing RDMA SENDs (to host) leave via txn_generator_out → axis_rreq_send[0]
 * RDMA SEND submissions use APP_SEND with RDMA_MODE_PARSE for auto-fragmentation.
 */

// ============================================================================
// MMIO / control signals
// ============================================================================
(* mark_debug = "true" *) logic [PID_BITS-1:0] coyote_pid;

// RDMA submission signals (from txn_generator)
(* mark_debug = "true" *) logic rdma_wr_valid;
(* mark_debug = "true" *) logic [63:0] rdma_wr_len;

// ============================================================================
// AXI control register parser
// ============================================================================
jigsaw_dc_axi_ctrl_parser inst_axi_ctrl_parser (
    .aclk(aclk),
    .aresetn(aresetn),
    .axi_ctrl(axi_ctrl),
    .coyote_pid(coyote_pid)
);

// ============================================================================
// Transaction generator
// ============================================================================
txn_generator inst_txn_generator (
    .aclk(aclk),
    .aresetn(aresetn),

    // Incoming data ← RDMA responder recv channel (incoming RDMA SENDs from remote)
    .txn_generator_in_tdata(axis_rrsp_recv[0].tdata),
    .txn_generator_in_tkeep(axis_rrsp_recv[0].tkeep),
    .txn_generator_in_tvalid(axis_rrsp_recv[0].tvalid),
    .txn_generator_in_tready(axis_rrsp_recv[0].tready),
    .txn_generator_in_tlast(axis_rrsp_recv[0].tlast),
    .txn_generator_in_tuser(1'b0),

    // Outgoing data → RDMA request send channel (outgoing RDMA SENDs)
    .txn_generator_out_tdata(axis_rreq_send[0].tdata),
    .txn_generator_out_tkeep(axis_rreq_send[0].tkeep),
    .txn_generator_out_tvalid(axis_rreq_send[0].tvalid),
    .txn_generator_out_tready(axis_rreq_send[0].tready),
    .txn_generator_out_tlast(axis_rreq_send[0].tlast),
    .txn_generator_out_tuser(),

    // RDMA submission signals
    .rdma_wr_valid(rdma_wr_valid),
    .rdma_wr_len(rdma_wr_len)
);

// ============================================================================
// SQ submission logic
// ============================================================================
always_comb begin
    // ----- No local DMA on device side -----
    sq_rd.data  = 0;
    sq_rd.valid = 1'b0;
    cq_rd.ready = 1'b1;

    sq_wr.data          = 0;
    if (rdma_wr_valid) begin
        // RDMA SEND — auto-fragmented by rdma_req_parser
        sq_wr.data.last     = 1'b1;
        sq_wr.data.pid      = coyote_pid;
        sq_wr.data.len      = rdma_wr_len;
        sq_wr.data.vaddr    = 0;               // SEND has no remote vaddr
        sq_wr.data.strm     = STRM_RDMA;
        sq_wr.data.remote   = 1'b1;
        sq_wr.data.opcode   = APP_SEND;        // parser fragments into RC_SEND_*
        sq_wr.data.mode     = 1'b0;            // RDMA_MODE_PARSE
        sq_wr.data.dest     = 0;               // axi channel 0
        sq_wr.valid         = 1'b1;
    end else begin
        sq_wr.valid         = 1'b0;
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
always_comb axis_host_recv[0].tie_off_s();   // not using host streams on device side
always_comb axis_host_send[0].tie_off_m();   // not using host streams on device side
always_comb notify.tie_off_m();              // not using notifications