/**
 * Loom vFPGA top
 *
 * loom_ctrl captures aperture stores and DMA descriptors into one
 * arrival-ordered FIFO; loom_engine drains it (local writes via
 * sq_wr/axis_host_send, rdma via sq_wr/axis_rreq_send); loom_rx forwards
 * incoming RDMA writes (rq_wr + axis_rrsp_recv) as local writes.
 *
 * The engine and rx share sq_wr and axis_host_send[0]; a registered
 * arbiter grants the path to one of them for whole transactions
 * (engine priority). The engine is gated by masking its FIFO-empty input
 * rather than by an engine-side grant port.
 */

// ---------------------------------------------------------------------------
// ctrl <-> table <-> engine wiring
// ---------------------------------------------------------------------------
logic                   tbl_commit;
logic [3:0]             tbl_idx;
logic                   tbl_valid;
logic                   tbl_route;
logic [PID_BITS-1:0]    tbl_pid;
logic [VADDR_BITS-1:0]  tbl_base;
logic [LEN_BITS-1:0]    tbl_len;

logic [PID_BITS-1:0]    compl_pid;
logic [VADDR_BITS-1:0]  compl_va;

logic                   fifo_empty, fifo_is_desc, fifo_pop;
logic [3:0]             fifo_win;
logic [27:0]            fifo_off, fifo_len;
logic [PID_BITS-1:0]    fifo_src_pid;
logic [63:0]            fifo_payload;

logic [3:0]             lu_idx;
logic                   lu_valid, lu_route;
logic [PID_BITS-1:0]    lu_pid;
logic [VADDR_BITS-1:0]  lu_base;
logic [LEN_BITS-1:0]    lu_len;

logic cnt_local_wr, cnt_rdma_wr, cnt_rx_fwd, cnt_drop, cnt_compl;

// Engine shell-side signals
req_t eng_rd_req, eng_wr_req;
logic eng_rd_valid, eng_wr_valid;
logic eng_busy;
logic [AXI_DATA_BITS-1:0]   eng_host_tdata, eng_net_tdata;
logic [AXI_DATA_BITS/8-1:0] eng_host_tkeep, eng_net_tkeep;
logic eng_host_tvalid, eng_host_tlast, eng_net_tvalid, eng_net_tlast;

// RX shell-side signals
req_t rx_wr_req;
logic rx_wr_valid;
logic rx_req_arb, rx_busy;
logic [AXI_DATA_BITS-1:0]   rx_tdata;
logic [AXI_DATA_BITS/8-1:0] rx_tkeep;
logic rx_tvalid, rx_tlast;

// ---------------------------------------------------------------------------
// Arbiter: exclusive, whole-transaction ownership of {sq_wr, axis_host_send}
// ---------------------------------------------------------------------------
typedef enum logic [1:0] { ARB_IDLE, ARB_ENG, ARB_RX } arb_t;
arb_t arb;
logic rx_started;

wire eng_grant = (arb == ARB_ENG);
wire rx_grant  = (arb == ARB_RX);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        arb <= ARB_IDLE;
        rx_started <= 1'b0;
    end else begin
        case (arb)
            ARB_IDLE: begin
                rx_started <= 1'b0;
                if (!fifo_empty)      arb <= ARB_ENG;   // engine priority
                else if (rx_req_arb)  arb <= ARB_RX;
            end
            ARB_ENG:
                if (fifo_empty && !eng_busy) arb <= ARB_IDLE;
            ARB_RX: begin
                if (rx_busy) rx_started <= 1'b1;
                if (rx_started && !rx_busy) arb <= ARB_IDLE;
            end
            default: arb <= ARB_IDLE;
        endcase
    end
end

// ---------------------------------------------------------------------------
// Modules
// ---------------------------------------------------------------------------
loom_ctrl inst_loom_ctrl (
    .aclk(aclk), .aresetn(aresetn), .axi_ctrl(axi_ctrl),
    .tbl_commit(tbl_commit), .tbl_idx(tbl_idx), .tbl_valid(tbl_valid),
    .tbl_route(tbl_route), .tbl_pid(tbl_pid), .tbl_base(tbl_base),
    .tbl_len(tbl_len),
    .compl_pid(compl_pid), .compl_va(compl_va),
    .fifo_empty(fifo_empty), .fifo_is_desc(fifo_is_desc),
    .fifo_win(fifo_win), .fifo_off(fifo_off), .fifo_len(fifo_len),
    .fifo_src_pid(fifo_src_pid), .fifo_payload(fifo_payload),
    .fifo_pop(fifo_pop),
    .cnt_local_wr(cnt_local_wr), .cnt_rdma_wr(cnt_rdma_wr),
    .cnt_rx_fwd(cnt_rx_fwd), .cnt_drop(cnt_drop), .cnt_compl(cnt_compl)
);

loom_table inst_loom_table (
    .aclk(aclk), .aresetn(aresetn),
    .commit(tbl_commit), .prog_idx(tbl_idx), .prog_valid(tbl_valid),
    .prog_route(tbl_route), .prog_pid(tbl_pid), .prog_base(tbl_base),
    .prog_len(tbl_len),
    .lu_idx(lu_idx), .lu_valid(lu_valid), .lu_route(lu_route),
    .lu_pid(lu_pid), .lu_base(lu_base), .lu_len(lu_len)
);

loom_engine inst_loom_engine (
    .aclk(aclk), .aresetn(aresetn),
    // FIFO-empty masked by the arbiter: the engine runs only while granted
    .fifo_empty(fifo_empty || !eng_grant),
    .fifo_is_desc(fifo_is_desc), .fifo_win(fifo_win), .fifo_off(fifo_off),
    .fifo_len(fifo_len), .fifo_src_pid(fifo_src_pid),
    .fifo_payload(fifo_payload), .fifo_pop(fifo_pop),
    .lu_idx(lu_idx), .lu_valid(lu_valid), .lu_route(lu_route),
    .lu_pid(lu_pid), .lu_base(lu_base), .lu_len(lu_len),
    .compl_pid(compl_pid), .compl_va(compl_va),
    .rd_req(eng_rd_req), .rd_valid(eng_rd_valid), .rd_ready(sq_rd.ready),
    .wr_req(eng_wr_req), .wr_valid(eng_wr_valid),
    .wr_ready(sq_wr.ready && eng_grant),
    .s_tdata(axis_host_recv[0].tdata), .s_tkeep(axis_host_recv[0].tkeep),
    .s_tvalid(axis_host_recv[0].tvalid), .s_tready(axis_host_recv[0].tready),
    .s_tlast(axis_host_recv[0].tlast),
    .m_host_tdata(eng_host_tdata), .m_host_tkeep(eng_host_tkeep),
    .m_host_tvalid(eng_host_tvalid),
    .m_host_tready(axis_host_send[0].tready && eng_grant),
    .m_host_tlast(eng_host_tlast),
    .m_net_tdata(eng_net_tdata), .m_net_tkeep(eng_net_tkeep),
    .m_net_tvalid(eng_net_tvalid),
    .m_net_tready(axis_rreq_send[0].tready),
    .m_net_tlast(eng_net_tlast),
    .cnt_local_wr(cnt_local_wr), .cnt_rdma_wr(cnt_rdma_wr),
    .cnt_drop(cnt_drop), .cnt_compl(cnt_compl),
    .busy(eng_busy)
);

loom_rx inst_loom_rx (
    .aclk(aclk), .aresetn(aresetn),
    .rq_req(rq_wr.data), .rq_valid(rq_wr.valid), .rq_ready(rq_wr.ready),
    .wr_req(rx_wr_req), .wr_valid(rx_wr_valid),
    .wr_ready(sq_wr.ready && rx_grant),
    .s_tdata(axis_rrsp_recv[0].tdata), .s_tkeep(axis_rrsp_recv[0].tkeep),
    .s_tvalid(axis_rrsp_recv[0].tvalid), .s_tready(axis_rrsp_recv[0].tready),
    .s_tlast(axis_rrsp_recv[0].tlast),
    .m_tdata(rx_tdata), .m_tkeep(rx_tkeep), .m_tvalid(rx_tvalid),
    .m_tready(axis_host_send[0].tready && rx_grant), .m_tlast(rx_tlast),
    .req(rx_req_arb), .grant(rx_grant), .busy(rx_busy),
    .cnt_rx_fwd(cnt_rx_fwd)
);

// ---------------------------------------------------------------------------
// Shared-path muxes
// ---------------------------------------------------------------------------
always_comb begin
    sq_wr.data  = rx_grant ? rx_wr_req  : eng_wr_req;
    sq_wr.valid = rx_grant ? rx_wr_valid : (eng_wr_valid && eng_grant);
end

always_comb begin
    sq_rd.data  = eng_rd_req;
    sq_rd.valid = eng_rd_valid;
end

always_comb begin
    axis_host_send[0].tdata  = rx_grant ? rx_tdata  : eng_host_tdata;
    axis_host_send[0].tkeep  = rx_grant ? rx_tkeep  : eng_host_tkeep;
    axis_host_send[0].tlast  = rx_grant ? rx_tlast  : eng_host_tlast;
    axis_host_send[0].tvalid = rx_grant ? rx_tvalid : (eng_host_tvalid && eng_grant);
    axis_host_send[0].tid    = '0;
end

always_comb begin
    axis_rreq_send[0].tdata  = eng_net_tdata;
    axis_rreq_send[0].tkeep  = eng_net_tkeep;
    axis_rreq_send[0].tlast  = eng_net_tlast;
    axis_rreq_send[0].tvalid = eng_net_tvalid;
    axis_rreq_send[0].tid    = '0;
end

// ---------------------------------------------------------------------------
// Tie-offs
// ---------------------------------------------------------------------------
always_comb notify.tie_off_m();
always_comb cq_rd.ready = 1'b1;
always_comb cq_wr.ready = 1'b1;
always_comb rq_rd.ready = 1'b1;

always_comb axis_rreq_recv[0].tie_off_s();
always_comb axis_rrsp_send[0].tie_off_m();
