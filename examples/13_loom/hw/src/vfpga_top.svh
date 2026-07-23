import lynxTypes::*;
import loom_pkg::*;

// ---- buses between orchestrator and router ----
loom_txn_t                 txn;
logic                      txn_valid, txn_ready;
logic                      rsp_valid, rsp_err;
logic [AXIL_DATA_BITS-1:0] rsp_data;
logic [AXIL_DATA_BITS-1:0] range_tbl [N_RANGE*RANGE_W];
logic [PID_BITS-1:0]       coyote_pid;
logic                      cnt_clear;
logic                      r_busy, r_done;
logic [AXIL_DATA_BITS-1:0] dbg_cnt [N_CNT*2];
logic [63:0]               dbg_tick;
logic [31:0]               drop_count;

// ---- orchestrator: AXI-Lite path ----
loom_orchestrator inst_orch (
    .aclk       (aclk),
    .aresetn    (aresetn),
    .axi_ctrl   (axi_ctrl),

    .txn        (txn),
    .txn_valid  (txn_valid),
    .txn_ready  (txn_ready),
    .rsp_valid  (rsp_valid),
    .rsp_data   (rsp_data),
    .rsp_err    (rsp_err),

    .busy       (r_busy),
    .done_valid (r_done),
    .dbg_cnt    (dbg_cnt),
    .dbg_tick   (dbg_tick),
    .drop_count (drop_count),

    .range_tbl  (range_tbl),
    .coyote_pid (coyote_pid),
    .cnt_clear  (cnt_clear)
);

// ---- router: descriptors + data streams ----
loom_router inst_router (
    .aclk       (aclk),
    .aresetn    (aresetn),

    .txn        (txn),
    .txn_valid  (txn_valid),
    .txn_ready  (txn_ready),
    .rsp_valid  (rsp_valid),
    .rsp_data   (rsp_data),
    .rsp_err    (rsp_err),

    .range_tbl  (range_tbl),
    .coyote_pid (coyote_pid),
    .cnt_clear  (cnt_clear),
    .dbg_cnt    (dbg_cnt),
    .dbg_tick   (dbg_tick),
    .drop_count (drop_count),
    .busy       (r_busy),
    .done_valid (r_done),

    // shell descriptor queues
    .sq_rd      (sq_rd),
    .sq_wr      (sq_wr),
    .cq_rd      (cq_rd),
    .cq_wr      (cq_wr),
    .rq_rd      (rq_rd),
    .rq_wr      (rq_wr),

    // shell data streams (constant-indexed array elements)
    .host_recv0 (axis_host_recv[0]),
    .host_recv1 (axis_host_recv[1]),
    .host_send0 (axis_host_send[0]),
    .host_send1 (axis_host_send[1]),
    .rreq_send0 (axis_rreq_send[0]),
    .rreq_recv0 (axis_rreq_recv[0]),
    .rrsp_send0 (axis_rrsp_send[0]),
    .rrsp_recv0 (axis_rrsp_recv[0])
);

// ---- tie off unused shell interfaces ----
always_comb notify.tie_off_m();
