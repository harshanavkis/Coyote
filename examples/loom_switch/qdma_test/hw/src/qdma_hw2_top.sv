// qdma_hw rev2 top (U280, device 10ee:9f3f): soft QDMA 5.0 with the static's configuration + qdma_hw2_core.
// Tie-offs as the static's BD/static_top_qdma: sys_rst_n and soft_reset_n held high (no PERST pin on the U280),
// no completions, no credits, no user interrupts, bypass-out and descriptor status always ready.
module qdma_hw2_top (
  output [15:0] pci_exp_txp,
  output [15:0] pci_exp_txn,
  input  [15:0] pci_exp_rxp,
  input  [15:0] pci_exp_rxn,
  input         sys_clk_p,
  input         sys_clk_n
);
  wire sys_clk, sys_clk_gt;
  IBUFDS_GTE4 #(.REFCLK_HROW_CK_SEL(2'b00)) refclk_ibuf (.O(sys_clk_gt), .ODIV2(sys_clk), .I(sys_clk_p), .CEB(1'b0), .IB(sys_clk_n));

  wire axi_aclk, axi_aresetn, user_lnk_up, phy_ready;

  // the static's sresetn = proc_sys_reset on axi_aresetn: a few synchronous cycles after it
  (* ASYNC_REG = "TRUE" *) reg [3:0] rst_sh = 4'd0;
  always @(posedge axi_aclk) rst_sh <= {rst_sh[2:0], axi_aresetn};
  wire rstn = rst_sh[3];

  wire [3:0]   m_axib_awid, m_axib_arid, m_axib_bid, m_axib_rid;
  wire [63:0]  m_axib_awaddr, m_axib_araddr;
  wire [7:0]   m_axib_awlen, m_axib_arlen;
  wire         m_axib_awvalid, m_axib_awready, m_axib_wlast, m_axib_wvalid, m_axib_wready, m_axib_bvalid, m_axib_bready;
  wire         m_axib_arvalid, m_axib_arready, m_axib_rlast, m_axib_rvalid, m_axib_rready;
  wire [511:0] m_axib_wdata, m_axib_rdata;
  wire [63:0]  m_axib_wstrb;
  wire [1:0]   m_axib_bresp, m_axib_rresp;

  wire [31:0]  m_axil_awaddr, m_axil_wdata, m_axil_araddr, m_axil_rdata;
  wire [3:0]   m_axil_wstrb;
  wire [1:0]   m_axil_bresp, m_axil_rresp;
  wire         m_axil_awvalid, m_axil_awready, m_axil_wvalid, m_axil_wready, m_axil_bvalid, m_axil_bready;
  wire         m_axil_arvalid, m_axil_arready, m_axil_rvalid, m_axil_rready;

  wire [63:0]  c2h_byp_in_st_csh_addr;
  wire [2:0]   c2h_byp_in_st_csh_port_id;
  wire [10:0]  c2h_byp_in_st_csh_qid;
  wire         c2h_byp_in_st_csh_error, c2h_byp_in_st_csh_vld, c2h_byp_in_st_csh_rdy;
  wire [7:0]   c2h_byp_in_st_csh_func;
  wire [6:0]   c2h_byp_in_st_csh_pfch_tag;
  wire [511:0] s_axis_c2h_tdata;
  wire [31:0]  s_axis_c2h_tcrc;
  wire         s_axis_c2h_ctrl_marker, s_axis_c2h_ctrl_has_cmpt, s_axis_c2h_tvalid, s_axis_c2h_tlast, s_axis_c2h_tready;
  wire [2:0]   s_axis_c2h_ctrl_port_id;
  wire [6:0]   s_axis_c2h_ctrl_ecc;
  wire [15:0]  s_axis_c2h_ctrl_len;
  wire [10:0]  s_axis_c2h_ctrl_qid;
  wire [5:0]   s_axis_c2h_mty;
  wire         axis_c2h_status_drop, axis_c2h_status_valid, axis_c2h_status_cmp, axis_c2h_status_error;
  wire         axis_c2h_status_last, axis_c2h_dmawr_cmp;
  wire [10:0]  axis_c2h_status_qid;

  wire [63:0]  h2c_byp_in_st_addr;
  wire [15:0]  h2c_byp_in_st_len, h2c_byp_in_st_cidx;
  wire         h2c_byp_in_st_eop, h2c_byp_in_st_sop, h2c_byp_in_st_mrkr_req, h2c_byp_in_st_sdi, h2c_byp_in_st_error;
  wire         h2c_byp_in_st_no_dma, h2c_byp_in_st_vld, h2c_byp_in_st_rdy;
  wire [2:0]   h2c_byp_in_st_port_id;
  wire [10:0]  h2c_byp_in_st_qid;
  wire [7:0]   h2c_byp_in_st_func;
  wire [511:0] m_axis_h2c_tdata;
  wire [31:0]  m_axis_h2c_tcrc, m_axis_h2c_tuser_mdata;
  wire [10:0]  m_axis_h2c_tuser_qid;
  wire [2:0]   m_axis_h2c_tuser_port_id;
  wire         m_axis_h2c_tuser_err, m_axis_h2c_tuser_zero_byte, m_axis_h2c_tvalid, m_axis_h2c_tlast, m_axis_h2c_tready;
  wire [5:0]   m_axis_h2c_tuser_mty;
  wire [7:0]   qsts_out_op;
  wire [63:0]  qsts_out_data;
  wire [2:0]   qsts_out_port_id;
  wire [12:0]  qsts_out_qid;
  wire         qsts_out_vld, qsts_out_rdy;

  wire         tm_dsc_sts_vld, tm_dsc_sts_error, tm_dsc_sts_dir, tm_dsc_sts_qinv, tm_dsc_sts_qen, tm_dsc_sts_rdy;
  wire [10:0]  tm_dsc_sts_qid;
  wire [15:0]  tm_dsc_sts_avl;

  qdma_0 qdma_0_i (
    .sys_clk(sys_clk), .sys_clk_gt(sys_clk_gt), .sys_rst_n(1'b1), .soft_reset_n(1'b1), .phy_ready(phy_ready),
    .user_lnk_up(user_lnk_up),
    .pci_exp_txp(pci_exp_txp), .pci_exp_txn(pci_exp_txn), .pci_exp_rxp(pci_exp_rxp), .pci_exp_rxn(pci_exp_rxn),
    .axi_aclk(axi_aclk), .axi_aresetn(axi_aresetn),

    .usr_irq_in_vld(1'b0), .usr_irq_in_vec(11'd0), .usr_irq_in_fnc(8'd0), .usr_irq_out_ack(), .usr_irq_out_fail(),

    .h2c_byp_out_dsc(), .h2c_byp_out_st_mm(), .h2c_byp_out_dsc_sz(), .h2c_byp_out_qid(), .h2c_byp_out_error(),
    .h2c_byp_out_func(), .h2c_byp_out_cidx(), .h2c_byp_out_port_id(), .h2c_byp_out_vld(), .h2c_byp_out_rdy(1'b1),
    .h2c_byp_out_fmt(),
    .c2h_byp_out_dsc(), .c2h_byp_out_st_mm(), .c2h_byp_out_qid(), .c2h_byp_out_dsc_sz(), .c2h_byp_out_error(),
    .c2h_byp_out_func(), .c2h_byp_out_cidx(), .c2h_byp_out_port_id(), .c2h_byp_out_vld(), .c2h_byp_out_rdy(1'b1),
    .c2h_byp_out_fmt(), .c2h_byp_out_pfch_tag(),

    .c2h_byp_in_st_csh_addr(c2h_byp_in_st_csh_addr), .c2h_byp_in_st_csh_port_id(c2h_byp_in_st_csh_port_id),
    .c2h_byp_in_st_csh_qid(c2h_byp_in_st_csh_qid), .c2h_byp_in_st_csh_error(c2h_byp_in_st_csh_error),
    .c2h_byp_in_st_csh_func(c2h_byp_in_st_csh_func), .c2h_byp_in_st_csh_vld(c2h_byp_in_st_csh_vld),
    .c2h_byp_in_st_csh_rdy(c2h_byp_in_st_csh_rdy), .c2h_byp_in_st_csh_pfch_tag(c2h_byp_in_st_csh_pfch_tag),

    .h2c_byp_in_st_addr(h2c_byp_in_st_addr), .h2c_byp_in_st_len(h2c_byp_in_st_len), .h2c_byp_in_st_eop(h2c_byp_in_st_eop),
    .h2c_byp_in_st_sop(h2c_byp_in_st_sop), .h2c_byp_in_st_mrkr_req(h2c_byp_in_st_mrkr_req),
    .h2c_byp_in_st_port_id(h2c_byp_in_st_port_id), .h2c_byp_in_st_sdi(h2c_byp_in_st_sdi),
    .h2c_byp_in_st_qid(h2c_byp_in_st_qid), .h2c_byp_in_st_error(h2c_byp_in_st_error),
    .h2c_byp_in_st_func(h2c_byp_in_st_func), .h2c_byp_in_st_cidx(h2c_byp_in_st_cidx),
    .h2c_byp_in_st_no_dma(h2c_byp_in_st_no_dma), .h2c_byp_in_st_vld(h2c_byp_in_st_vld), .h2c_byp_in_st_rdy(h2c_byp_in_st_rdy),

    .tm_dsc_sts_vld(tm_dsc_sts_vld), .tm_dsc_sts_port_id(), .tm_dsc_sts_qen(tm_dsc_sts_qen), .tm_dsc_sts_byp(),
    .tm_dsc_sts_dir(tm_dsc_sts_dir), .tm_dsc_sts_mm(), .tm_dsc_sts_error(tm_dsc_sts_error), .tm_dsc_sts_qid(tm_dsc_sts_qid),
    .tm_dsc_sts_avl(tm_dsc_sts_avl), .tm_dsc_sts_qinv(tm_dsc_sts_qinv), .tm_dsc_sts_irq_arm(), .tm_dsc_sts_rdy(tm_dsc_sts_rdy),
    .tm_dsc_sts_pidx(),

    .dsc_crdt_in_crdt(16'd0), .dsc_crdt_in_qid(11'd0), .dsc_crdt_in_dir(1'b0), .dsc_crdt_in_fence(1'b0),
    .dsc_crdt_in_vld(1'b0), .dsc_crdt_in_rdy(),

    .m_axil_awaddr(m_axil_awaddr), .m_axil_awuser(), .m_axil_awprot(), .m_axil_awvalid(m_axil_awvalid),
    .m_axil_awready(m_axil_awready), .m_axil_wdata(m_axil_wdata), .m_axil_wstrb(m_axil_wstrb), .m_axil_wvalid(m_axil_wvalid),
    .m_axil_wready(m_axil_wready), .m_axil_bvalid(m_axil_bvalid), .m_axil_bresp(m_axil_bresp), .m_axil_bready(m_axil_bready),
    .m_axil_araddr(m_axil_araddr), .m_axil_aruser(), .m_axil_arprot(), .m_axil_arvalid(m_axil_arvalid),
    .m_axil_arready(m_axil_arready), .m_axil_rdata(m_axil_rdata), .m_axil_rresp(m_axil_rresp), .m_axil_rvalid(m_axil_rvalid),
    .m_axil_rready(m_axil_rready),

    .m_axib_awid(m_axib_awid), .m_axib_awaddr(m_axib_awaddr), .m_axib_awlen(m_axib_awlen), .m_axib_awuser(),
    .m_axib_awsize(), .m_axib_awburst(), .m_axib_awprot(), .m_axib_awvalid(m_axib_awvalid), .m_axib_awready(m_axib_awready),
    .m_axib_awlock(), .m_axib_awcache(), .m_axib_wdata(m_axib_wdata), .m_axib_wstrb(m_axib_wstrb), .m_axib_wlast(m_axib_wlast),
    .m_axib_wvalid(m_axib_wvalid), .m_axib_wready(m_axib_wready), .m_axib_bid(m_axib_bid), .m_axib_bresp(m_axib_bresp),
    .m_axib_bvalid(m_axib_bvalid), .m_axib_bready(m_axib_bready), .m_axib_arid(m_axib_arid), .m_axib_araddr(m_axib_araddr),
    .m_axib_arlen(m_axib_arlen), .m_axib_aruser(), .m_axib_arsize(), .m_axib_arburst(), .m_axib_arprot(),
    .m_axib_arvalid(m_axib_arvalid), .m_axib_arready(m_axib_arready), .m_axib_arlock(), .m_axib_arcache(),
    .m_axib_rid(m_axib_rid), .m_axib_rdata(m_axib_rdata), .m_axib_rresp(m_axib_rresp), .m_axib_rlast(m_axib_rlast),
    .m_axib_rvalid(m_axib_rvalid), .m_axib_rready(m_axib_rready),

    .m_axis_h2c_tdata(m_axis_h2c_tdata), .m_axis_h2c_tcrc(m_axis_h2c_tcrc), .m_axis_h2c_tuser_qid(m_axis_h2c_tuser_qid),
    .m_axis_h2c_tuser_port_id(m_axis_h2c_tuser_port_id), .m_axis_h2c_tuser_err(m_axis_h2c_tuser_err),
    .m_axis_h2c_tuser_mdata(m_axis_h2c_tuser_mdata), .m_axis_h2c_tuser_mty(m_axis_h2c_tuser_mty),
    .m_axis_h2c_tuser_zero_byte(m_axis_h2c_tuser_zero_byte), .m_axis_h2c_tvalid(m_axis_h2c_tvalid),
    .m_axis_h2c_tlast(m_axis_h2c_tlast), .m_axis_h2c_tready(m_axis_h2c_tready),

    .s_axis_c2h_tdata(s_axis_c2h_tdata), .s_axis_c2h_tcrc(s_axis_c2h_tcrc), .s_axis_c2h_ctrl_marker(s_axis_c2h_ctrl_marker),
    .s_axis_c2h_ctrl_port_id(s_axis_c2h_ctrl_port_id), .s_axis_c2h_ctrl_ecc(s_axis_c2h_ctrl_ecc),
    .s_axis_c2h_ctrl_len(s_axis_c2h_ctrl_len), .s_axis_c2h_ctrl_qid(s_axis_c2h_ctrl_qid),
    .s_axis_c2h_ctrl_has_cmpt(s_axis_c2h_ctrl_has_cmpt), .s_axis_c2h_mty(s_axis_c2h_mty), .s_axis_c2h_tvalid(s_axis_c2h_tvalid),
    .s_axis_c2h_tlast(s_axis_c2h_tlast), .s_axis_c2h_tready(s_axis_c2h_tready),

    .s_axis_c2h_cmpt_tdata(512'd0), .s_axis_c2h_cmpt_size(2'd0), .s_axis_c2h_cmpt_dpar(16'd0), .s_axis_c2h_cmpt_tvalid(1'b0),
    .s_axis_c2h_cmpt_ctrl_qid(11'd0), .s_axis_c2h_cmpt_ctrl_cmpt_type(2'd0), .s_axis_c2h_cmpt_ctrl_wait_pld_pkt_id(16'd0),
    .s_axis_c2h_cmpt_ctrl_port_id(3'd0), .s_axis_c2h_cmpt_ctrl_marker(1'b0), .s_axis_c2h_cmpt_ctrl_user_trig(1'b0),
    .s_axis_c2h_cmpt_ctrl_col_idx(3'd0), .s_axis_c2h_cmpt_ctrl_err_idx(3'd0), .s_axis_c2h_cmpt_tready(),
    .s_axis_c2h_cmpt_ctrl_no_wrb_marker(1'b0),

    .axis_c2h_status_drop(axis_c2h_status_drop), .axis_c2h_status_valid(axis_c2h_status_valid),
    .axis_c2h_status_cmp(axis_c2h_status_cmp), .axis_c2h_status_error(axis_c2h_status_error),
    .axis_c2h_status_last(axis_c2h_status_last), .axis_c2h_status_qid(axis_c2h_status_qid),
    .axis_c2h_dmawr_cmp(axis_c2h_dmawr_cmp),

    .qsts_out_op(qsts_out_op), .qsts_out_data(qsts_out_data), .qsts_out_port_id(qsts_out_port_id), .qsts_out_qid(qsts_out_qid),
    .qsts_out_vld(qsts_out_vld), .qsts_out_rdy(qsts_out_rdy)
  );

  qdma_hw2_core core (
    .clk(axi_aclk), .rstn(rstn),
    .m_axib_awid(m_axib_awid), .m_axib_awaddr(m_axib_awaddr), .m_axib_awlen(m_axib_awlen), .m_axib_awvalid(m_axib_awvalid),
    .m_axib_awready(m_axib_awready), .m_axib_wdata(m_axib_wdata), .m_axib_wstrb(m_axib_wstrb), .m_axib_wlast(m_axib_wlast),
    .m_axib_wvalid(m_axib_wvalid), .m_axib_wready(m_axib_wready), .m_axib_bid(m_axib_bid), .m_axib_bresp(m_axib_bresp),
    .m_axib_bvalid(m_axib_bvalid), .m_axib_bready(m_axib_bready), .m_axib_arid(m_axib_arid), .m_axib_araddr(m_axib_araddr),
    .m_axib_arlen(m_axib_arlen), .m_axib_arvalid(m_axib_arvalid), .m_axib_arready(m_axib_arready), .m_axib_rid(m_axib_rid),
    .m_axib_rdata(m_axib_rdata), .m_axib_rresp(m_axib_rresp), .m_axib_rlast(m_axib_rlast), .m_axib_rvalid(m_axib_rvalid),
    .m_axib_rready(m_axib_rready),
    .m_axil_awaddr(m_axil_awaddr), .m_axil_awvalid(m_axil_awvalid), .m_axil_awready(m_axil_awready),
    .m_axil_wdata(m_axil_wdata), .m_axil_wvalid(m_axil_wvalid), .m_axil_wready(m_axil_wready), .m_axil_bvalid(m_axil_bvalid),
    .m_axil_bresp(m_axil_bresp), .m_axil_bready(m_axil_bready), .m_axil_araddr(m_axil_araddr), .m_axil_arvalid(m_axil_arvalid),
    .m_axil_arready(m_axil_arready), .m_axil_rdata(m_axil_rdata), .m_axil_rresp(m_axil_rresp), .m_axil_rvalid(m_axil_rvalid),
    .m_axil_rready(m_axil_rready),
    .c2h_byp_in_st_csh_addr(c2h_byp_in_st_csh_addr), .c2h_byp_in_st_csh_port_id(c2h_byp_in_st_csh_port_id),
    .c2h_byp_in_st_csh_qid(c2h_byp_in_st_csh_qid), .c2h_byp_in_st_csh_error(c2h_byp_in_st_csh_error),
    .c2h_byp_in_st_csh_func(c2h_byp_in_st_csh_func), .c2h_byp_in_st_csh_pfch_tag(c2h_byp_in_st_csh_pfch_tag),
    .c2h_byp_in_st_csh_vld(c2h_byp_in_st_csh_vld), .c2h_byp_in_st_csh_rdy(c2h_byp_in_st_csh_rdy),
    .s_axis_c2h_tdata(s_axis_c2h_tdata), .s_axis_c2h_tcrc(s_axis_c2h_tcrc), .s_axis_c2h_ctrl_marker(s_axis_c2h_ctrl_marker),
    .s_axis_c2h_ctrl_port_id(s_axis_c2h_ctrl_port_id), .s_axis_c2h_ctrl_ecc(s_axis_c2h_ctrl_ecc),
    .s_axis_c2h_ctrl_len(s_axis_c2h_ctrl_len), .s_axis_c2h_ctrl_qid(s_axis_c2h_ctrl_qid),
    .s_axis_c2h_ctrl_has_cmpt(s_axis_c2h_ctrl_has_cmpt), .s_axis_c2h_mty(s_axis_c2h_mty), .s_axis_c2h_tvalid(s_axis_c2h_tvalid),
    .s_axis_c2h_tlast(s_axis_c2h_tlast), .s_axis_c2h_tready(s_axis_c2h_tready),
    .axis_c2h_status_drop(axis_c2h_status_drop), .axis_c2h_status_valid(axis_c2h_status_valid),
    .axis_c2h_status_cmp(axis_c2h_status_cmp), .axis_c2h_status_error(axis_c2h_status_error),
    .axis_c2h_status_last(axis_c2h_status_last), .axis_c2h_status_qid(axis_c2h_status_qid),
    .axis_c2h_dmawr_cmp(axis_c2h_dmawr_cmp),
    .h2c_byp_in_st_addr(h2c_byp_in_st_addr), .h2c_byp_in_st_len(h2c_byp_in_st_len), .h2c_byp_in_st_eop(h2c_byp_in_st_eop),
    .h2c_byp_in_st_sop(h2c_byp_in_st_sop), .h2c_byp_in_st_mrkr_req(h2c_byp_in_st_mrkr_req),
    .h2c_byp_in_st_port_id(h2c_byp_in_st_port_id), .h2c_byp_in_st_sdi(h2c_byp_in_st_sdi), .h2c_byp_in_st_qid(h2c_byp_in_st_qid),
    .h2c_byp_in_st_error(h2c_byp_in_st_error), .h2c_byp_in_st_func(h2c_byp_in_st_func), .h2c_byp_in_st_cidx(h2c_byp_in_st_cidx),
    .h2c_byp_in_st_no_dma(h2c_byp_in_st_no_dma), .h2c_byp_in_st_vld(h2c_byp_in_st_vld), .h2c_byp_in_st_rdy(h2c_byp_in_st_rdy),
    .m_axis_h2c_tdata(m_axis_h2c_tdata), .m_axis_h2c_tcrc(m_axis_h2c_tcrc), .m_axis_h2c_tuser_qid(m_axis_h2c_tuser_qid),
    .m_axis_h2c_tuser_port_id(m_axis_h2c_tuser_port_id), .m_axis_h2c_tuser_err(m_axis_h2c_tuser_err),
    .m_axis_h2c_tuser_mdata(m_axis_h2c_tuser_mdata), .m_axis_h2c_tuser_mty(m_axis_h2c_tuser_mty),
    .m_axis_h2c_tuser_zero_byte(m_axis_h2c_tuser_zero_byte), .m_axis_h2c_tvalid(m_axis_h2c_tvalid),
    .m_axis_h2c_tlast(m_axis_h2c_tlast), .m_axis_h2c_tready(m_axis_h2c_tready),
    .qsts_out_op(qsts_out_op), .qsts_out_data(qsts_out_data), .qsts_out_port_id(qsts_out_port_id), .qsts_out_qid(qsts_out_qid),
    .qsts_out_vld(qsts_out_vld), .qsts_out_rdy(qsts_out_rdy),
    .tm_dsc_sts_vld(tm_dsc_sts_vld), .tm_dsc_sts_error(tm_dsc_sts_error), .tm_dsc_sts_qid(tm_dsc_sts_qid),
    .tm_dsc_sts_dir(tm_dsc_sts_dir), .tm_dsc_sts_qinv(tm_dsc_sts_qinv), .tm_dsc_sts_qen(tm_dsc_sts_qen),
    .tm_dsc_sts_avl(tm_dsc_sts_avl), .tm_dsc_sts_rdy(tm_dsc_sts_rdy)
  );
endmodule
