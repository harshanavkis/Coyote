/**
 * Loom switch vFPGA top
 *
 * One data path: everything that writes into a window - the V80 copy
 * engine peer-to-peer, the host CPU through its write-combining mapping -
 * arrives on axi_udata, and loom_ingress turns it into local writes or
 * Loom messages by binding (loom_table). loom_rx lands incoming messages
 * as local writes. loom_ctrl is the CSR page (table programming, staging
 * VA, ack window, counters).
 *
 * Streams: the ingress sends on axis_host_send[0] and axis_rreq_send[0];
 * loom_rx lands on axis_host_send[1] (wr_req.dest 1) - a separate queue,
 * credits and FIFO in the shell for each. The two share only sq_wr,
 * arbitrated per request with loom_rx first, as in examples/loom (see its
 * vfpga_top.svh for why, and for the arbiter's two known defects, which
 * carry over unchanged).
 */

// ---------------------------------------------------------------------------
// ctrl -> table
// ---------------------------------------------------------------------------
logic                   tbl_commit;
logic [3:0]             tbl_idx;
logic                   tbl_valid, tbl_route;
logic [PID_BITS-1:0]    tbl_pid, tbl_dst_pid;
logic [VADDR_BITS-1:0]  tbl_base;
logic [LEN_BITS-1:0]    tbl_len;
logic [26:0]            tbl_ustart;

logic [VADDR_BITS-1:0]  rdma_staging_va;
logic [7:0]             tx_window;
logic [3:0]             rx_chunk;

// table -> ingress
logic [26:0]            ua_addr;
logic                   ua_hit, ua_route;
logic [3:0]             ua_idx;
logic [PID_BITS-1:0]    ua_pid, ua_dst_pid;
logic [VADDR_BITS-1:0]  ua_base;
logic [26:0]            ua_ustart;
logic [LEN_BITS:0]      ua_end;
logic                   ua_ce1, ua_ce2;

// ---------------------------------------------------------------------------
// Ack window: rdma packets and stores posted by the ingress and not yet
// acknowledged. cq_wr with remote set is one ack per packet (req_t.last is
// what gates it; every ingress request has last = 1). loom_rx's landings
// complete on cq_wr too, with remote clear.
// ---------------------------------------------------------------------------
wire         ack_valid = cq_wr.valid && cq_wr.data.remote;
logic        rdma_post;
logic [15:0] tx_inflight;
wire         win_ok = (tx_window == 8'd0) || (tx_inflight < {8'd0, tx_window});

always_ff @(posedge aclk) begin
    if (!aresetn) tx_inflight <= 16'd0;
    else if (rdma_post && !ack_valid) tx_inflight <= tx_inflight + 16'd1;
    else if (ack_valid && !rdma_post && (tx_inflight != 16'd0)) tx_inflight <= tx_inflight - 16'd1;
end

// ---------------------------------------------------------------------------
// Producers' shell-side signals
// ---------------------------------------------------------------------------
req_t ing_wr_req, rx_wr_req;
logic ing_wr_valid, rx_wr_valid;

logic [AXI_DATA_BITS-1:0]   ing_host_tdata, ing_net_tdata, rx_tdata;
logic [AXI_DATA_BITS/8-1:0] ing_host_tkeep, ing_net_tkeep, rx_tkeep;
logic ing_host_tvalid, ing_host_tlast, ing_net_tvalid, ing_net_tlast, rx_tvalid, rx_tlast;

logic cnt_ing_burst, cnt_ing_drop, cnt_ing_pkt_local, cnt_ing_pkt_rdma;
logic cnt_ing_store, cnt_ing_store_drop, cnt_ing_flush, cnt_ing_win_wait, cnt_ing_req_wait;
logic [13:0] cnt_ing_dbg;
logic cnt_rx_fwd, cnt_rx_drop, cnt_rx_orphan, rx_cnt_move, rx_cnt_starve, rx_cnt_stall;
logic rx_cnt_bp, rx_cnt_req, rx_cnt_fifo_full;
logic cnt_wr_wait_local, cnt_wr_wait_rdma, cnt_wr_blk_ing, cnt_wr_blk_rx;
/* verilator lint_off UNUSED */
logic rx_req_arb, rx_busy;
/* verilator lint_on UNUSED */

// Registered host streams, as examples/loom
AXI4SR axis_wr (.*);
axisr_reg inst_reg_wr    (.aclk(aclk), .aresetn(aresetn),
                          .s_axis(axis_wr), .m_axis(axis_host_send[0]));
AXI4SR axis_wr_rx (.*);
axisr_reg inst_reg_wr_rx (.aclk(aclk), .aresetn(aresetn),
                          .s_axis(axis_wr_rx), .m_axis(axis_host_send[1]));

// sq_wr: loom_rx's request when it has one, else the ingress's
wire rx_takes_wr = rx_wr_valid;

// ---------------------------------------------------------------------------
// Modules
// ---------------------------------------------------------------------------
loom_ctrl inst_loom_ctrl (
    .aclk(aclk), .aresetn(aresetn), .axi_ctrl(axi_ctrl),
    .tbl_commit(tbl_commit), .tbl_idx(tbl_idx), .tbl_valid(tbl_valid),
    .tbl_route(tbl_route), .tbl_pid(tbl_pid), .tbl_dst_pid(tbl_dst_pid),
    .tbl_base(tbl_base), .tbl_len(tbl_len), .tbl_ustart(tbl_ustart),
    .rdma_staging_va(rdma_staging_va), .tx_window(tx_window), .rx_chunk(rx_chunk),
    .tx_inflight(tx_inflight), .cnt_tx_ack(ack_valid),
    .cnt_tx_winfull(cnt_ing_win_wait), .cnt_tx_reqwait(cnt_ing_req_wait),
    .cnt_wr_wait_local(cnt_wr_wait_local), .cnt_wr_wait_rdma(cnt_wr_wait_rdma),
    .cnt_wr_blk_ing(cnt_wr_blk_ing), .cnt_wr_blk_rx(cnt_wr_blk_rx),
    .cnt_rx_fwd(cnt_rx_fwd), .cnt_rx_drop(cnt_rx_drop), .cnt_rx_orphan(cnt_rx_orphan),
    .cnt_rx_move(rx_cnt_move), .cnt_rx_starve(rx_cnt_starve), .cnt_rx_stall(rx_cnt_stall),
    .cnt_rx_bp(rx_cnt_bp), .cnt_rx_req(rx_cnt_req), .cnt_rx_fifo_full(rx_cnt_fifo_full),
    .cnt_hout_move(dbg_host_out[0]), .cnt_hout_bp(dbg_host_out[1]), .cnt_hreq_bp(dbg_host_out[2]),
    .cnt_ing_burst(cnt_ing_burst), .cnt_ing_drop(cnt_ing_drop),
    .cnt_ing_pkt_local(cnt_ing_pkt_local), .cnt_ing_pkt_rdma(cnt_ing_pkt_rdma),
    .cnt_ing_store(cnt_ing_store), .cnt_ing_store_drop(cnt_ing_store_drop),
    .cnt_ing_flush(cnt_ing_flush), .cnt_ing_dbg(cnt_ing_dbg)
);

loom_table inst_loom_table (
    .aclk(aclk), .aresetn(aresetn),
    .commit(tbl_commit), .prog_idx(tbl_idx), .prog_valid(tbl_valid),
    .prog_route(tbl_route), .prog_pid(tbl_pid), .prog_dst_pid(tbl_dst_pid),
    .prog_base(tbl_base), .prog_len(tbl_len), .prog_ustart(tbl_ustart),
    // no aperture: the index lookup is unused
    .lu_idx(4'd0), .lu_valid(), .lu_route(), .lu_pid(), .lu_dst_pid(), .lu_base(), .lu_len(),
    .ua_ce1(ua_ce1), .ua_ce2(ua_ce2),
    .ua_addr(ua_addr), .ua_hit(ua_hit), .ua_idx(ua_idx), .ua_route(ua_route),
    .ua_pid(ua_pid), .ua_dst_pid(ua_dst_pid), .ua_base(ua_base),
    .ua_ustart(ua_ustart), .ua_end(ua_end)
);

loom_ingress #(.HOST_DEST(0), .NET_DEST(0)) inst_loom_ingress (
    .aclk(aclk), .aresetn(aresetn), .axi_udata(axi_udata),
    .ua_ce1(ua_ce1), .ua_ce2(ua_ce2),
    .ua_addr(ua_addr), .ua_hit(ua_hit), .ua_route(ua_route), .ua_pid(ua_pid),
    .ua_dst_pid(ua_dst_pid), .ua_base(ua_base), .ua_ustart(ua_ustart),
    .ua_end(ua_end), .ua_idx(ua_idx),
    .rdma_staging_va(rdma_staging_va),
    .wr_req(ing_wr_req), .wr_valid(ing_wr_valid),
    .wr_ready(sq_wr.ready && !rx_takes_wr),
    .win_ok(win_ok), .rdma_post(rdma_post),
    .m_host_tdata(ing_host_tdata), .m_host_tkeep(ing_host_tkeep),
    .m_host_tvalid(ing_host_tvalid), .m_host_tready(axis_wr.tready),
    .m_host_tlast(ing_host_tlast),
    .m_net_tdata(ing_net_tdata), .m_net_tkeep(ing_net_tkeep),
    .m_net_tvalid(ing_net_tvalid), .m_net_tready(axis_rreq_send[0].tready),
    .m_net_tlast(ing_net_tlast),
    .cnt_burst(cnt_ing_burst), .cnt_drop(cnt_ing_drop),
    .cnt_pkt_local(cnt_ing_pkt_local), .cnt_pkt_rdma(cnt_ing_pkt_rdma),
    .cnt_store(cnt_ing_store), .cnt_store_drop(cnt_ing_store_drop),
    .cnt_flush(cnt_ing_flush),
    .cnt_win_wait(cnt_ing_win_wait), .cnt_req_wait(cnt_ing_req_wait),
    .cnt_dbg(cnt_ing_dbg)
);

// ---------------------------------------------------------------------------
// Ingress FIFO in front of loom_rx: decouples the host write path from the
// RoCE receive path. 4096 beats (64 packets, URAM; init_ip.tcl) so the
// sender's ack window fits while the host write path is slower than the
// wire: with 512 beats (8 packets) under a 16-packet window, a slow landing
// (the V80's window) made the stack lose packets -> go-back-N retransmits.
// ---------------------------------------------------------------------------
logic [511:0] rxf_tdata;
logic [63:0]  rxf_tkeep;
logic         rxf_tvalid, rxf_tready, rxf_tlast;

axis_data_fifo_rx4096 inst_rx_ingress_fifo (
    .s_axis_aclk(aclk), .s_axis_aresetn(aresetn),
    .s_axis_tdata(axis_rrsp_recv[0].tdata),
    .s_axis_tkeep(axis_rrsp_recv[0].tkeep),
    .s_axis_tvalid(axis_rrsp_recv[0].tvalid),
    .s_axis_tready(axis_rrsp_recv[0].tready),
    .s_axis_tlast(axis_rrsp_recv[0].tlast),
    .m_axis_tdata(rxf_tdata),
    .m_axis_tkeep(rxf_tkeep),
    .m_axis_tvalid(rxf_tvalid),
    .m_axis_tready(rxf_tready),
    .m_axis_tlast(rxf_tlast)
);
assign rx_cnt_fifo_full = axis_rrsp_recv[0].tvalid && !axis_rrsp_recv[0].tready;

loom_rx inst_loom_rx (
    .aclk(aclk), .aresetn(aresetn),
    .rq_req(rq_wr.data), .rq_valid(rq_wr.valid), .rq_ready(rq_wr.ready),
    .rdma_staging_va(rdma_staging_va),
    .rx_chunk(rx_chunk),
    .wr_req(rx_wr_req), .wr_valid(rx_wr_valid),
    .wr_ready(sq_wr.ready),
    .s_tdata(rxf_tdata), .s_tkeep(rxf_tkeep),
    .s_tvalid(rxf_tvalid), .s_tready(rxf_tready),
    .s_tlast(rxf_tlast),
    .m_tdata(rx_tdata), .m_tkeep(rx_tkeep), .m_tvalid(rx_tvalid),
    .m_tready(axis_wr_rx.tready), .m_tlast(rx_tlast),
    .req(rx_req_arb), .grant(1'b1), .busy(rx_busy),
    .cnt_rx_move(rx_cnt_move), .cnt_rx_starve(rx_cnt_starve),
    .cnt_rx_stall(rx_cnt_stall), .cnt_rx_bp(rx_cnt_bp), .cnt_rx_req(rx_cnt_req),
    .cnt_rx_fwd(cnt_rx_fwd), .cnt_rx_drop(cnt_rx_drop),
    .cnt_rx_orphan(cnt_rx_orphan)
);

// ---------------------------------------------------------------------------
// Shared sq_wr, and the streams (one producer each)
// ---------------------------------------------------------------------------
always_comb begin
    sq_wr.data  = rx_takes_wr ? rx_wr_req : ing_wr_req;
    sq_wr.valid = rx_wr_valid || ing_wr_valid;
end

wire wr_stalled = sq_wr.valid && !sq_wr.ready;
assign cnt_wr_wait_local = wr_stalled && is_strm_local(sq_wr.data.strm);
assign cnt_wr_wait_rdma  = wr_stalled && !is_strm_local(sq_wr.data.strm);
assign cnt_wr_blk_ing    = ing_wr_valid && rx_takes_wr && !sq_wr.ready;
assign cnt_wr_blk_rx     = rx_wr_valid && !rx_takes_wr;

always_comb begin
    axis_wr.tdata  = ing_host_tdata;
    axis_wr.tkeep  = ing_host_tkeep;
    axis_wr.tlast  = ing_host_tlast;
    axis_wr.tvalid = ing_host_tvalid;
    axis_wr.tid    = '0;
end

always_comb begin
    axis_wr_rx.tdata  = rx_tdata;
    axis_wr_rx.tkeep  = rx_tkeep;
    axis_wr_rx.tlast  = rx_tlast;
    axis_wr_rx.tvalid = rx_tvalid;
    axis_wr_rx.tid    = '0;
end

always_comb begin
    axis_rreq_send[0].tdata  = ing_net_tdata;
    axis_rreq_send[0].tkeep  = ing_net_tkeep;
    axis_rreq_send[0].tlast  = ing_net_tlast;
    axis_rreq_send[0].tvalid = ing_net_tvalid;
    axis_rreq_send[0].tid    = '0;
end

// ---------------------------------------------------------------------------
// Tie-offs: nothing pulls from host memory, nothing answers remote reads
// ---------------------------------------------------------------------------
always_comb notify.tie_off_m();
always_comb sq_rd.tie_off_m();
always_comb cq_rd.ready = 1'b1;
always_comb cq_wr.ready = 1'b1;
always_comb rq_rd.ready = 1'b1;

always_comb axis_host_recv[0].tie_off_s();
always_comb axis_host_recv[1].tie_off_s();
always_comb axis_rreq_recv[0].tie_off_s();
always_comb axis_rrsp_send[0].tie_off_m();
