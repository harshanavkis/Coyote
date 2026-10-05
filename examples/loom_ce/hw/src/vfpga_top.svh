/**
 * Loom copy engine vFPGA top (V80)
 *
 * The emulated accelerator's copy engine: loom_ce_ctrl is the CSR page,
 * loom_ce copies between card memory (HBM) and the U280's user data window,
 * imported as a dma-buf (a put writes the window, a get reads it), and
 * releases a fence.
 */

logic                  ce_start, ce_busy;
logic [VADDR_BITS-1:0] ce_src_va, ce_dst_va, ce_fence_va;
logic [LEN_BITS-1:0]   ce_len;
logic [PID_BITS-1:0]   ce_pid;
logic                  ce_get;
logic [31:0]           ce_copies;
logic [63:0]           ce_cycles;
logic                  cnt_ce_in_wait;

// Registered streams, as example 07
AXI4SR axis_card_in (.*);
axisr_reg inst_reg_in  (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_card_recv[0]), .m_axis(axis_card_in));
AXI4SR axis_host_out (.*);
axisr_reg inst_reg_out (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_host_out), .m_axis(axis_host_send[0]));
AXI4SR axis_host_in (.*);
axisr_reg inst_reg_hin (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_host_recv[0]), .m_axis(axis_host_in));
AXI4SR axis_card_out (.*);
axisr_reg inst_reg_cout (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_card_out), .m_axis(axis_card_send[0]));

loom_ce_ctrl inst_loom_ce_ctrl (
    .aclk(aclk), .aresetn(aresetn), .axi_ctrl(axi_ctrl),
    .start(ce_start), .src_va(ce_src_va), .dst_va(ce_dst_va), .len(ce_len),
    .pid(ce_pid), .fence_va(ce_fence_va), .get(ce_get),
    .busy(ce_busy), .copies(ce_copies), .cycles(ce_cycles),
    .cnt_ce_out_bp((axis_host_out.tvalid && !axis_host_out.tready) ||
                   (axis_card_out.tvalid && !axis_card_out.tready)),
    .cnt_ce_in_wait(cnt_ce_in_wait),
    .cnt_wr_wait(sq_wr.valid && !sq_wr.ready)
);

loom_ce inst_loom_ce (
    .aclk(aclk), .aresetn(aresetn),
    .start(ce_start), .src_va(ce_src_va), .dst_va(ce_dst_va), .len(ce_len),
    .pid(ce_pid), .fence_va(ce_fence_va), .get(ce_get),
    .busy(ce_busy), .copies(ce_copies), .cycles(ce_cycles),
    .rd_req(sq_rd.data), .rd_valid(sq_rd.valid), .rd_ready(sq_rd.ready),
    .wr_req(sq_wr.data), .wr_valid(sq_wr.valid), .wr_ready(sq_wr.ready),
    .wr_done(cq_wr.valid),
    .s_tdata(axis_card_in.tdata), .s_tkeep(axis_card_in.tkeep),
    .s_tvalid(axis_card_in.tvalid), .s_tready(axis_card_in.tready),
    .m_tdata(axis_host_out.tdata), .m_tkeep(axis_host_out.tkeep),
    .m_tvalid(axis_host_out.tvalid), .m_tready(axis_host_out.tready),
    .m_tlast(axis_host_out.tlast),
    .s_host_tdata(axis_host_in.tdata), .s_host_tkeep(axis_host_in.tkeep),
    .s_host_tvalid(axis_host_in.tvalid), .s_host_tready(axis_host_in.tready),
    .m_card_tdata(axis_card_out.tdata), .m_card_tkeep(axis_card_out.tkeep),
    .m_card_tvalid(axis_card_out.tvalid), .m_card_tready(axis_card_out.tready),
    .m_card_tlast(axis_card_out.tlast),
    .cnt_in_wait(cnt_ce_in_wait)
);
assign axis_host_out.tid = '0;
assign axis_card_out.tid = '0;

// Tie-offs: completions are always taken
always_comb cq_rd.ready = 1'b1;
always_comb cq_wr.ready = 1'b1;
always_comb notify.tie_off_m();
