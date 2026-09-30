/**
 * Loom copy engine vFPGA top (V80)
 *
 * The emulated accelerator's copy engine: loom_ce_ctrl is the CSR page,
 * loom_ce copies from card memory (HBM) to a destination VA - the U280's
 * user data window, imported as a dma-buf - and releases a fence.
 *
 * The receive side: the far U280's loom_rx writes what arrives for this
 * accelerator into this vFPGA's uwin (EN_UWIN; exported as a dma-buf and
 * imported by the U280), and loom_ingress - the U280's, copied unchanged -
 * lands it in card memory through one window, [0, LAND_LEN) of the uwin at
 * card VA LAND_BASE: full lines as packets, 8 B stores as 8 B writes, both
 * as LOCAL_WRITE on the card stream.
 *
 * sq_wr is shared per request by the copy engine and the landing (each has
 * its own data stream: host 0 and card 0). A presented request stays
 * presented until it is taken; otherwise the two alternate. cq_wr
 * completions are told apart by stream.
 */

logic                  ce_start, ce_busy;
logic [VADDR_BITS-1:0] ce_src_va, ce_dst_va, ce_fence_va;
logic [LEN_BITS-1:0]   ce_len;
logic [PID_BITS-1:0]   ce_pid;
logic [31:0]           ce_copies;
logic [63:0]           ce_cycles;

logic [VADDR_BITS-1:0] land_base;
logic [LEN_BITS-1:0]   land_len;
logic [PID_BITS-1:0]   land_pid;

// Registered streams, as example 07
AXI4SR axis_card_in (.*);
axisr_reg inst_reg_in   (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_card_recv[0]), .m_axis(axis_card_in));
AXI4SR axis_host_out (.*);
axisr_reg inst_reg_out  (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_host_out), .m_axis(axis_host_send[0]));
AXI4SR axis_card_out (.*);
axisr_reg inst_reg_land (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_card_out), .m_axis(axis_card_send[0]));

// ---------------------------------------------------------------------------
// sq_wr: the copy engine's and the landing's requests
// ---------------------------------------------------------------------------
req_t ce_wr_req, land_wr_req;
logic ce_wr_valid, land_wr_valid;

logic wr_lock, wr_lock_land, wr_last_land;
wire  wr_sel_land = wr_lock ? wr_lock_land : (land_wr_valid && (!ce_wr_valid || !wr_last_land));

always_comb begin
    sq_wr.data  = wr_sel_land ? land_wr_req : ce_wr_req;
    sq_wr.valid = wr_sel_land ? land_wr_valid : ce_wr_valid;
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        wr_lock      <= 1'b0;
        wr_last_land <= 1'b0;
    end else begin
        wr_lock      <= sq_wr.valid && !sq_wr.ready;
        wr_lock_land <= wr_sel_land;
        if (sq_wr.valid && sq_wr.ready) wr_last_land <= wr_sel_land;
    end
end

wire ce_wr_done   = cq_wr.valid && (cq_wr.data.strm == STRM_HOST);
wire land_wr_done = cq_wr.valid && (cq_wr.data.strm == STRM_CARD);

// ---------------------------------------------------------------------------
// Copy engine
// ---------------------------------------------------------------------------
logic cnt_land_burst, cnt_land_drop, cnt_land_store, cnt_land_partial;

loom_ce_ctrl inst_loom_ce_ctrl (
    .aclk(aclk), .aresetn(aresetn), .axi_ctrl(axi_ctrl),
    .start(ce_start), .src_va(ce_src_va), .dst_va(ce_dst_va), .len(ce_len),
    .pid(ce_pid), .fence_va(ce_fence_va),
    .busy(ce_busy), .copies(ce_copies), .cycles(ce_cycles),
    .land_base(land_base), .land_len(land_len), .land_pid(land_pid),
    .cnt_land_req(land_wr_valid && sq_wr.ready && wr_sel_land),
    .cnt_land_done(land_wr_done),
    .cnt_land_burst(cnt_land_burst), .cnt_land_drop(cnt_land_drop),
    .cnt_land_store(cnt_land_store), .cnt_land_partial(cnt_land_partial)
);

loom_ce inst_loom_ce (
    .aclk(aclk), .aresetn(aresetn),
    .start(ce_start), .src_va(ce_src_va), .dst_va(ce_dst_va), .len(ce_len),
    .pid(ce_pid), .fence_va(ce_fence_va),
    .busy(ce_busy), .copies(ce_copies), .cycles(ce_cycles),
    .rd_req(sq_rd.data), .rd_valid(sq_rd.valid), .rd_ready(sq_rd.ready),
    .wr_req(ce_wr_req), .wr_valid(ce_wr_valid), .wr_ready(sq_wr.ready && !wr_sel_land),
    .wr_done(ce_wr_done),
    .s_tdata(axis_card_in.tdata), .s_tkeep(axis_card_in.tkeep),
    .s_tvalid(axis_card_in.tvalid), .s_tready(axis_card_in.tready),
    .m_tdata(axis_host_out.tdata), .m_tkeep(axis_host_out.tkeep),
    .m_tvalid(axis_host_out.tvalid), .m_tready(axis_host_out.tready),
    .m_tlast(axis_host_out.tlast)
);
assign axis_host_out.tid = '0;

// ---------------------------------------------------------------------------
// Landing: one window, looked up in loom_ingress's two table stages
// ---------------------------------------------------------------------------
logic                  ua_ce1, ua_ce2;
logic [26:0]           ua_addr;
logic                  land_hit1, ua_hit;

always_ff @(posedge aclk) if (ua_ce1) land_hit1 <= (land_len != 0) && ({1'b0, ua_addr} < land_len);
always_ff @(posedge aclk) if (ua_ce2) ua_hit    <= land_hit1;

loom_ingress #(.HOST_DEST(0), .LOCAL_STRM(STRM_CARD)) inst_loom_land (
    .aclk(aclk), .aresetn(aresetn), .axi_udata(axi_udata),
    .ua_ce1(ua_ce1), .ua_ce2(ua_ce2), .ua_addr(ua_addr),
    .ua_hit(ua_hit), .ua_route(1'b0), .ua_pid(land_pid), .ua_dst_pid('0),
    .ua_base(land_base), .ua_ustart('0), .ua_end({1'b0, land_len}), .ua_idx(4'd1),
    .rdma_staging_va('0),
    .wr_req(land_wr_req), .wr_valid(land_wr_valid),
    .wr_ready(sq_wr.ready && wr_sel_land),
    .win_ok(1'b1), .rdma_post(),
    .m_host_tdata(axis_card_out.tdata), .m_host_tkeep(axis_card_out.tkeep),
    .m_host_tvalid(axis_card_out.tvalid), .m_host_tready(axis_card_out.tready),
    .m_host_tlast(axis_card_out.tlast),
    .m_net_tdata(), .m_net_tkeep(), .m_net_tvalid(), .m_net_tready(1'b1), .m_net_tlast(),
    .cnt_burst(cnt_land_burst), .cnt_drop(cnt_land_drop),
    .cnt_pkt_local(), .cnt_pkt_rdma(),
    .cnt_store(cnt_land_store), .cnt_store_drop(cnt_land_partial),
    .cnt_flush(), .cnt_win_wait(), .cnt_req_wait(), .cnt_dbg()
);
assign axis_card_out.tid = '0;

// Tie-offs: completions are always taken; nothing reads the host
always_comb cq_rd.ready = 1'b1;
always_comb cq_wr.ready = 1'b1;
always_comb notify.tie_off_m();
always_comb axis_host_recv[0].tie_off_s();
