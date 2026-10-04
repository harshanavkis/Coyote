/**
 * Loom switch vFPGA top
 *
 * One data path: everything that writes into a window - the V80 copy
 * engine peer-to-peer, the host CPU through its write-combining mapping -
 * arrives on axi_udata, and loom_ingress turns it into local writes or
 * self-describing RDMA packets by binding (loom_table): each rdma packet's
 * RETH names {far export index, offset}. loom_rx lands every incoming
 * packet on its own, inside this host's exports (loom_exports). loom_ctrl
 * is the CSR page (window and export programming, ack window, counters).
 *
 * Streams: the ingress sends on axis_host_send[0] and axis_rreq_send[0];
 * loom_rx lands on axis_host_send[1] (wr_req.dest 1) - a separate queue,
 * credits and FIFO in the shell for each. They share sq_wr, arbitrated per
 * request with loom_rx first, as in examples/loom (see its vfpga_top.svh for
 * why, and for the arbiter's two known defects, which carry over unchanged).
 *
 * Gets: a store into a get window goes out as a get request (loom_ingress);
 * an incoming one becomes a job (loom_rx) for loom_rd, which reads on
 * sq_rd / axis_host_recv[1] and writes the data back as rdma packets plus a
 * completion store. loom_rd shares sq_wr (after loom_rx, alternating with
 * the ingress), the ack window and axis_rreq_send[0] with the ingress; the
 * payload stream follows the order the two's requests were taken.
 */

// ---------------------------------------------------------------------------
// ctrl -> table
// ---------------------------------------------------------------------------
logic                   tbl_commit;
logic [3:0]             tbl_idx;
logic                   tbl_valid, tbl_route, tbl_get;
logic [PID_BITS-1:0]    tbl_pid, tbl_dst_pid;
logic [VADDR_BITS-1:0]  tbl_base;
logic [LEN_BITS-1:0]    tbl_len;
logic [26:0]            tbl_ustart;

/* verilator lint_off UNUSED */
logic [VADDR_BITS-1:0]  rdma_staging_va;     // CSR 16, unused: each packet's RETH is its own reference
logic [3:0]             rx_chunk;            // CSR 76, unused
/* verilator lint_on UNUSED */
logic [7:0]             tx_window;
logic [PID_BITS-1:0]    rd_qp_pid;

// table -> ingress
logic [26:0]            ua_addr;
logic                   ua_hit, ua_route, ua_get;
logic [3:0]             ua_idx;
logic [PID_BITS-1:0]    ua_pid, ua_dst_pid;
logic [VADDR_BITS-1:0]  ua_base;
logic [26:0]            ua_ustart;
logic [LEN_BITS:0]      ua_end;
logic                   ua_ce1, ua_ce2;

// ---------------------------------------------------------------------------
// Ack window: rdma packets and stores posted by the ingress and loom_rd and
// not yet acknowledged. cq_wr with remote set is one ack per packet
// (req_t.last is what gates it; every rdma request has last = 1). loom_rx's
// landings complete on cq_wr too, with remote clear.
// ---------------------------------------------------------------------------
wire         ack_valid = cq_wr.valid && cq_wr.data.remote;
logic        ing_post, rd_post;
wire         rdma_post = ing_post || rd_post;      // one sq_wr: never both
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
logic ing_net_tready, rd_tready;   // axis_rreq_send[0].tready, to whichever's turn it is

logic cnt_ing_burst, cnt_ing_drop, cnt_ing_pkt_local, cnt_ing_pkt_rdma;
logic cnt_ing_store, cnt_ing_store_drop, cnt_ing_flush, cnt_ing_win_wait, cnt_ing_req_wait;
logic [13:0] cnt_ing_dbg;
logic cnt_rx_fwd, cnt_rx_drop, cnt_rx_orphan, rx_cnt_move, rx_cnt_starve, rx_cnt_stall;
logic rx_cnt_bp, rx_cnt_req, rx_cnt_fifo_full;
logic rx_cnt_pkt, rx_cnt_store, rx_cnt_post_wait, rx_cnt_at_limit, rx_cnt_pkt_wait, rx_cnt_rq_ovfl;
logic cnt_ing_rdma_full, cnt_ing_rdma_flush, cnt_ing_rdma_cut, cnt_ing_get, cnt_ing_get_drop;

// loom_rd: its requests, its payload, and loom_rx's jobs for it
req_t rd_wr_req;
logic rd_wr_valid;
logic [AXI_DATA_BITS-1:0]   rd_tdata;
logic [AXI_DATA_BITS/8-1:0] rd_tkeep;
logic rd_tvalid, rd_tlast;
logic                  job_valid, job_ready, job_err;
logic [PID_BITS-1:0]   job_pid;
logic [VADDR_BITS-1:0] job_va;
logic [22:0]           job_len;
logic [47:0]           job_ret;
logic [63:0]           job_cval;
logic cnt_rd_job, cnt_rd_err, cnt_rd_pkt, cnt_rd_cmp, cnt_rd_wait, cnt_rd_starve;
/* verilator lint_off UNUSED */
logic rd_busy;
/* verilator lint_on UNUSED */

// ctrl -> exports, and loom_rx's two lookups
logic                   exp_commit, exp_valid;
logic [7:0]             exp_idx;
logic [PID_BITS-1:0]    exp_pid;
logic [VADDR_BITS-1:0]  exp_base;
logic [39:0]            exp_len;
logic [7:0]             xa_idx, xb_idx;
logic                   xa_hit, xb_hit;
logic [PID_BITS-1:0]    xa_pid, xb_pid;
logic [VADDR_BITS-1:0]  xa_base, xb_base;
logic [39:0]            xa_len, xb_len;
logic cnt_wr_wait_local, cnt_wr_wait_rdma, cnt_wr_blk_ing, cnt_wr_blk_rx;
/* verilator lint_off UNUSED */
logic rx_busy;
/* verilator lint_on UNUSED */

// Registered host streams, as examples/loom
AXI4SR axis_wr (.*);
axisr_reg inst_reg_wr    (.aclk(aclk), .aresetn(aresetn),
                          .s_axis(axis_wr), .m_axis(axis_host_send[0]));
AXI4SR axis_wr_rx (.*);
axisr_reg inst_reg_wr_rx (.aclk(aclk), .aresetn(aresetn),
                          .s_axis(axis_wr_rx), .m_axis(axis_host_send[1]));

// sq_wr: loom_rx's request when it has one; else the ingress's or loom_rd's,
// alternating when both wait. One of the two presented and not taken is
// presented again until it is (unless loom_rx's comes first).
logic rx_takes_wr, hold_v, hold_rd, rd_turn, hold_ok, sel_rd;
assign rx_takes_wr = rx_wr_valid;
assign hold_ok     = hold_v && (hold_rd ? rd_wr_valid : ing_wr_valid);
assign sel_rd      = hold_ok ? hold_rd : (rd_wr_valid && (!ing_wr_valid || rd_turn));

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        hold_v  <= 1'b0;
        hold_rd <= 1'b0;
        rd_turn <= 1'b0;
    end else if (!rx_takes_wr && (ing_wr_valid || rd_wr_valid)) begin
        if (sq_wr.ready) begin
            hold_v  <= 1'b0;
            rd_turn <= !sel_rd;
        end else begin
            hold_v  <= 1'b1;
            hold_rd <= sel_rd;
        end
    end
end

// ---------------------------------------------------------------------------
// Modules
// ---------------------------------------------------------------------------
loom_ctrl inst_loom_ctrl (
    .aclk(aclk), .aresetn(aresetn), .axi_ctrl(axi_ctrl),
    .tbl_commit(tbl_commit), .tbl_idx(tbl_idx), .tbl_valid(tbl_valid),
    .tbl_route(tbl_route), .tbl_get(tbl_get), .tbl_pid(tbl_pid), .tbl_dst_pid(tbl_dst_pid),
    .tbl_base(tbl_base), .tbl_len(tbl_len), .tbl_ustart(tbl_ustart),
    .rdma_staging_va(rdma_staging_va), .tx_window(tx_window), .rx_chunk(rx_chunk),
    .rd_qp_pid(rd_qp_pid),
    .exp_commit(exp_commit), .exp_idx(exp_idx), .exp_valid(exp_valid), .exp_pid(exp_pid),
    .exp_base(exp_base), .exp_len(exp_len),
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
    .cnt_ing_flush(cnt_ing_flush), .cnt_ing_dbg(cnt_ing_dbg),
    // words 112+ (and their longest runs at 144+), in loom_ctrl's list
    .cnt_x({cnt_rd_starve, cnt_rd_wait, cnt_rd_cmp, cnt_rd_pkt, cnt_rd_err, cnt_rd_job,
            cnt_ing_get_drop, cnt_ing_get,
            dbg_host_out[14:10], dbg_host_out[16:15], dbg_host_out[9:3],
            cnt_ing_rdma_cut, cnt_ing_rdma_flush, cnt_ing_rdma_full,
            rx_cnt_rq_ovfl, rx_cnt_pkt_wait, rx_cnt_at_limit, rx_cnt_post_wait,
            cnt_rx_drop, rx_cnt_store, rx_cnt_pkt})
);

loom_exports inst_loom_exports (
    .aclk(aclk), .aresetn(aresetn),
    .commit(exp_commit), .prog_idx(exp_idx), .prog_valid(exp_valid), .prog_pid(exp_pid),
    .prog_base(exp_base), .prog_len(exp_len),
    .a_idx(xa_idx), .a_hit(xa_hit), .a_pid(xa_pid), .a_base(xa_base), .a_len(xa_len),
    .b_idx(xb_idx), .b_hit(xb_hit), .b_pid(xb_pid), .b_base(xb_base), .b_len(xb_len)
);

loom_table inst_loom_table (
    .aclk(aclk), .aresetn(aresetn),
    .commit(tbl_commit), .prog_idx(tbl_idx), .prog_valid(tbl_valid),
    .prog_route(tbl_route), .prog_get(tbl_get), .prog_pid(tbl_pid), .prog_dst_pid(tbl_dst_pid),
    .prog_base(tbl_base), .prog_len(tbl_len), .prog_ustart(tbl_ustart),
    // no aperture: the index lookup is unused
    .lu_idx(4'd0), .lu_valid(), .lu_route(), .lu_pid(), .lu_dst_pid(), .lu_base(), .lu_len(),
    .ua_ce1(ua_ce1), .ua_ce2(ua_ce2),
    .ua_addr(ua_addr), .ua_hit(ua_hit), .ua_idx(ua_idx), .ua_route(ua_route), .ua_get(ua_get),
    .ua_pid(ua_pid), .ua_dst_pid(ua_dst_pid), .ua_base(ua_base),
    .ua_ustart(ua_ustart), .ua_end(ua_end)
);

loom_ingress #(.HOST_DEST(0), .NET_DEST(0)) inst_loom_ingress (
    .aclk(aclk), .aresetn(aresetn), .axi_udata(axi_udata),
    .ua_ce1(ua_ce1), .ua_ce2(ua_ce2),
    .ua_addr(ua_addr), .ua_hit(ua_hit), .ua_route(ua_route), .ua_get(ua_get), .ua_pid(ua_pid),
    .ua_dst_pid(ua_dst_pid), .ua_base(ua_base), .ua_ustart(ua_ustart),
    .ua_end(ua_end), .ua_idx(ua_idx),
    .wr_req(ing_wr_req), .wr_valid(ing_wr_valid),
    .wr_ready(sq_wr.ready && !rx_takes_wr && !sel_rd),
    .win_ok(win_ok), .rdma_post(ing_post),
    .m_host_tdata(ing_host_tdata), .m_host_tkeep(ing_host_tkeep),
    .m_host_tvalid(ing_host_tvalid), .m_host_tready(axis_wr.tready),
    .m_host_tlast(ing_host_tlast),
    .m_net_tdata(ing_net_tdata), .m_net_tkeep(ing_net_tkeep),
    .m_net_tvalid(ing_net_tvalid), .m_net_tready(ing_net_tready),
    .m_net_tlast(ing_net_tlast),
    .cnt_burst(cnt_ing_burst), .cnt_drop(cnt_ing_drop),
    .cnt_pkt_local(cnt_ing_pkt_local), .cnt_pkt_rdma(cnt_ing_pkt_rdma),
    .cnt_store(cnt_ing_store), .cnt_store_drop(cnt_ing_store_drop),
    .cnt_flush(cnt_ing_flush),
    .cnt_win_wait(cnt_ing_win_wait), .cnt_req_wait(cnt_ing_req_wait),
    .cnt_rdma_full(cnt_ing_rdma_full), .cnt_rdma_flush(cnt_ing_rdma_flush),
    .cnt_rdma_cut(cnt_ing_rdma_cut),
    .cnt_get(cnt_ing_get), .cnt_get_drop(cnt_ing_get_drop),
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
    .xa_idx(xa_idx), .xa_hit(xa_hit), .xa_pid(xa_pid), .xa_base(xa_base), .xa_len(xa_len),
    .xb_idx(xb_idx), .xb_hit(xb_hit), .xb_pid(xb_pid), .xb_base(xb_base), .xb_len(xb_len),
    .wr_req(rx_wr_req), .wr_valid(rx_wr_valid),
    .wr_ready(sq_wr.ready),
    .m_job_valid(job_valid), .m_job_ready(job_ready), .m_job_pid(job_pid), .m_job_va(job_va),
    .m_job_len(job_len), .m_job_ret(job_ret), .m_job_cval(job_cval), .m_job_err(job_err),
    .s_tdata(rxf_tdata), .s_tkeep(rxf_tkeep),
    .s_tvalid(rxf_tvalid), .s_tready(rxf_tready),
    .s_tlast(rxf_tlast),
    .m_tdata(rx_tdata), .m_tkeep(rx_tkeep), .m_tvalid(rx_tvalid),
    .m_tready(axis_wr_rx.tready), .m_tlast(rx_tlast),
    .busy(rx_busy),
    .cnt_rx_move(rx_cnt_move), .cnt_rx_starve(rx_cnt_starve),
    .cnt_rx_stall(rx_cnt_stall), .cnt_rx_bp(rx_cnt_bp), .cnt_rx_req(rx_cnt_req),
    .cnt_rx_fwd(cnt_rx_fwd), .cnt_rx_drop(cnt_rx_drop),
    .cnt_rx_orphan(cnt_rx_orphan),
    .cnt_rx_pkt(rx_cnt_pkt), .cnt_rx_store(rx_cnt_store), .cnt_rx_post_wait(rx_cnt_post_wait),
    .cnt_rx_at_limit(rx_cnt_at_limit), .cnt_rx_pkt_wait(rx_cnt_pkt_wait),
    .cnt_rx_rq_ovfl(rx_cnt_rq_ovfl)
);

loom_rd #(.NET_DEST(0), .RD_DEST(1)) inst_loom_rd (
    .aclk(aclk), .aresetn(aresetn),
    .s_job_valid(job_valid), .s_job_ready(job_ready), .s_job_pid(job_pid), .s_job_va(job_va),
    .s_job_len(job_len), .s_job_ret(job_ret), .s_job_cval(job_cval), .s_job_err(job_err),
    .qp_pid(rd_qp_pid),
    .rd_req(sq_rd.data), .rd_valid(sq_rd.valid), .rd_ready(sq_rd.ready),
    .s_tdata(axis_host_recv[1].tdata), .s_tvalid(axis_host_recv[1].tvalid),
    .s_tready(axis_host_recv[1].tready),
    .wr_req(rd_wr_req), .wr_valid(rd_wr_valid),
    .wr_ready(sq_wr.ready && !rx_takes_wr && sel_rd),
    .win_ok(win_ok), .rdma_post(rd_post),
    .m_tdata(rd_tdata), .m_tkeep(rd_tkeep), .m_tvalid(rd_tvalid), .m_tready(rd_tready),
    .m_tlast(rd_tlast),
    .busy(rd_busy),
    .cnt_job(cnt_rd_job), .cnt_err(cnt_rd_err), .cnt_pkt(cnt_rd_pkt), .cnt_cmp(cnt_rd_cmp),
    .cnt_wait(cnt_rd_wait), .cnt_starve(cnt_rd_starve)
);

// ---------------------------------------------------------------------------
// Shared sq_wr, and the streams
// ---------------------------------------------------------------------------
always_comb begin
    sq_wr.data  = rx_takes_wr ? rx_wr_req : (sel_rd ? rd_wr_req : ing_wr_req);
    sq_wr.valid = rx_wr_valid || ing_wr_valid || rd_wr_valid;
end

// RDMA payload order: the stack pairs payload with requests in the order it
// took them, so axis_rreq_send[0] carries the ingress's and loom_rd's
// packets in the order their requests were taken (each sends one packet's
// beats, tlast on the last, after its request and before its next one)
logic       no_mem [4];
logic [2:0] no_wp, no_rp;
wire        no_empty = (no_wp == no_rp);
wire        no_rd    = no_mem[no_rp[1:0]];
wire        net_take = sq_wr.valid && sq_wr.ready && !rx_takes_wr && (sel_rd || (sq_wr.data.strm == STRM_RDMA));
wire        net_done = axis_rreq_send[0].tvalid && axis_rreq_send[0].tready && axis_rreq_send[0].tlast;

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        no_wp <= '0;
        no_rp <= '0;
    end else begin
        if (net_take) begin
            no_mem[no_wp[1:0]] <= sel_rd;
            no_wp <= no_wp + 1'b1;
        end
        if (net_done) no_rp <= no_rp + 1'b1;
    end
end

assign ing_net_tready = axis_rreq_send[0].tready && !no_empty && !no_rd;
assign rd_tready      = axis_rreq_send[0].tready && !no_empty &&  no_rd;

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
    axis_rreq_send[0].tdata  = no_rd ? rd_tdata : ing_net_tdata;
    axis_rreq_send[0].tkeep  = no_rd ? rd_tkeep : ing_net_tkeep;
    axis_rreq_send[0].tlast  = no_rd ? rd_tlast : ing_net_tlast;
    axis_rreq_send[0].tvalid = !no_empty && (no_rd ? rd_tvalid : ing_net_tvalid);
    axis_rreq_send[0].tid    = '0;
end

// ---------------------------------------------------------------------------
// Tie-offs: no RoCE READs (gets are writes both ways); the reads' completions
// are not needed (loom_rd counts the data)
// ---------------------------------------------------------------------------
always_comb notify.tie_off_m();
always_comb cq_rd.ready = 1'b1;
always_comb cq_wr.ready = 1'b1;
always_comb rq_rd.ready = 1'b1;

always_comb axis_host_recv[0].tie_off_s();
always_comb axis_rreq_recv[0].tie_off_s();
always_comb axis_rrsp_send[0].tie_off_m();
