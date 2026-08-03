/**
 * Loom vFPGA top (Phase 1 stub)
 *
 * Instantiates the ctrl slave (CSR page + aperture capture + order FIFO)
 * and the window table. The engine and RX forwarder land in later phases;
 * until then the FIFO is left undrained and all shell-facing data
 * interfaces are tied off.
 */

// ---------------------------------------------------------------------------
// ctrl <-> table <-> (future engine) wiring
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

logic                   fifo_empty;
logic                   fifo_is_desc;
logic [3:0]             fifo_win;
logic [27:0]            fifo_off;
logic [27:0]            fifo_len;
logic [PID_BITS-1:0]    fifo_src_pid;
logic [63:0]            fifo_payload;

logic [3:0]             lu_idx;
logic                   lu_valid;
logic                   lu_route;
logic [PID_BITS-1:0]    lu_pid;
logic [VADDR_BITS-1:0]  lu_base;
logic [LEN_BITS-1:0]    lu_len;

loom_ctrl inst_loom_ctrl (
    .aclk(aclk),
    .aresetn(aresetn),
    .axi_ctrl(axi_ctrl),

    .tbl_commit(tbl_commit),
    .tbl_idx(tbl_idx),
    .tbl_valid(tbl_valid),
    .tbl_route(tbl_route),
    .tbl_pid(tbl_pid),
    .tbl_base(tbl_base),
    .tbl_len(tbl_len),

    .compl_pid(compl_pid),
    .compl_va(compl_va),

    .fifo_empty(fifo_empty),
    .fifo_is_desc(fifo_is_desc),
    .fifo_win(fifo_win),
    .fifo_off(fifo_off),
    .fifo_len(fifo_len),
    .fifo_src_pid(fifo_src_pid),
    .fifo_payload(fifo_payload),
    .fifo_pop(1'b0),            // engine arrives in Phase 2

    .cnt_local_wr(1'b0),
    .cnt_rdma_wr(1'b0),
    .cnt_rx_fwd(1'b0),
    .cnt_drop(1'b0),
    .cnt_compl(1'b0)
);

loom_table inst_loom_table (
    .aclk(aclk),
    .aresetn(aresetn),

    .commit(tbl_commit),
    .prog_idx(tbl_idx),
    .prog_valid(tbl_valid),
    .prog_route(tbl_route),
    .prog_pid(tbl_pid),
    .prog_base(tbl_base),
    .prog_len(tbl_len),

    .lu_idx(fifo_win),
    .lu_valid(lu_valid),
    .lu_route(lu_route),
    .lu_pid(lu_pid),
    .lu_base(lu_base),
    .lu_len(lu_len)
);

// ---------------------------------------------------------------------------
// Tie-offs (data path arrives in Phases 2-4)
// ---------------------------------------------------------------------------
always_comb notify.tie_off_m();
always_comb sq_rd.tie_off_m();
always_comb sq_wr.tie_off_m();
always_comb cq_rd.tie_off_s();
always_comb cq_wr.tie_off_s();

always_comb axis_host_recv[0].tie_off_s();
always_comb axis_host_send[0].tie_off_m();

always_comb axis_rreq_recv[0].tie_off_s();
always_comb axis_rreq_send[0].tie_off_m();
always_comb axis_rrsp_recv[0].tie_off_s();
always_comb axis_rrsp_send[0].tie_off_m();
always_comb rq_rd.ready = 1'b1;
always_comb rq_wr.ready = 1'b1;
