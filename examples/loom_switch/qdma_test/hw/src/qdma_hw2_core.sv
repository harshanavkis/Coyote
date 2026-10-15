// qdma_hw rev2 core: everything behind the soft QDMA, at its axi_aclk (250 MHz = the static's xclk).
// The C2H/H2C path is the U280 static's (Coyote 7cb6023d, static_top_qdma_tmplt.txt), unchanged:
//   qdma_gen (shell stand-in) -> qdma_wr_wrapper / qdma_rd_wrapper (N_HCHAN 3, N_OUTSTANDING 8) -> qdma_reg_slice -> QDMA.
// BAR4 (AXI bridge master) -> bar4_ctl: rev1 sink + counters, control registers and status words in its top 1 MB.
// BAR0 (AXI-Lite master, the static's cnfg BAR) -> always-OK responder (reads return 0x9F3F0000 | addr[15:0]).
import lynxTypes::*;

module qdma_hw2_core (
  input  logic          clk,
  input  logic          rstn,

  // BAR4 AXI bridge master
  input  logic [3:0]    m_axib_awid,
  input  logic [63:0]   m_axib_awaddr,
  input  logic [7:0]    m_axib_awlen,
  input  logic          m_axib_awvalid,
  output logic          m_axib_awready,
  input  logic [511:0]  m_axib_wdata,
  input  logic [63:0]   m_axib_wstrb,
  input  logic          m_axib_wlast,
  input  logic          m_axib_wvalid,
  output logic          m_axib_wready,
  output logic [3:0]    m_axib_bid,
  output logic [1:0]    m_axib_bresp,
  output logic          m_axib_bvalid,
  input  logic          m_axib_bready,
  input  logic [3:0]    m_axib_arid,
  input  logic [63:0]   m_axib_araddr,
  input  logic [7:0]    m_axib_arlen,
  input  logic          m_axib_arvalid,
  output logic          m_axib_arready,
  output logic [3:0]    m_axib_rid,
  output logic [511:0]  m_axib_rdata,
  output logic [1:0]    m_axib_rresp,
  output logic          m_axib_rlast,
  output logic          m_axib_rvalid,
  input  logic          m_axib_rready,

  // BAR0 AXI-Lite master
  input  logic [31:0]   m_axil_awaddr,
  input  logic          m_axil_awvalid,
  output logic          m_axil_awready,
  input  logic [31:0]   m_axil_wdata,
  input  logic          m_axil_wvalid,
  output logic          m_axil_wready,
  output logic          m_axil_bvalid,
  output logic [1:0]    m_axil_bresp,
  input  logic          m_axil_bready,
  input  logic [31:0]   m_axil_araddr,
  input  logic          m_axil_arvalid,
  output logic          m_axil_arready,
  output logic [31:0]   m_axil_rdata,
  output logic [1:0]    m_axil_rresp,
  output logic          m_axil_rvalid,
  input  logic          m_axil_rready,

  // C2H descriptor bypass in (simple bypass, prefetch tag)
  output logic [63:0]   c2h_byp_in_st_csh_addr,
  output logic [2:0]    c2h_byp_in_st_csh_port_id,
  output logic [10:0]   c2h_byp_in_st_csh_qid,
  output logic          c2h_byp_in_st_csh_error,
  output logic [7:0]    c2h_byp_in_st_csh_func,
  output logic [6:0]    c2h_byp_in_st_csh_pfch_tag,
  output logic          c2h_byp_in_st_csh_vld,
  input  logic          c2h_byp_in_st_csh_rdy,
  // C2H data
  output logic [511:0]  s_axis_c2h_tdata,
  output logic [31:0]   s_axis_c2h_tcrc,
  output logic          s_axis_c2h_ctrl_marker,
  output logic [2:0]    s_axis_c2h_ctrl_port_id,
  output logic [6:0]    s_axis_c2h_ctrl_ecc,
  output logic [15:0]   s_axis_c2h_ctrl_len,
  output logic [10:0]   s_axis_c2h_ctrl_qid,
  output logic          s_axis_c2h_ctrl_has_cmpt,
  output logic [5:0]    s_axis_c2h_mty,
  output logic          s_axis_c2h_tvalid,
  output logic          s_axis_c2h_tlast,
  input  logic          s_axis_c2h_tready,
  // C2H status
  input  logic          axis_c2h_status_drop,
  input  logic          axis_c2h_status_valid,
  input  logic          axis_c2h_status_cmp,
  input  logic          axis_c2h_status_error,
  input  logic          axis_c2h_status_last,
  input  logic [10:0]   axis_c2h_status_qid,
  input  logic          axis_c2h_dmawr_cmp,

  // H2C descriptor bypass in
  output logic [63:0]   h2c_byp_in_st_addr,
  output logic [15:0]   h2c_byp_in_st_len,
  output logic          h2c_byp_in_st_eop,
  output logic          h2c_byp_in_st_sop,
  output logic          h2c_byp_in_st_mrkr_req,
  output logic [2:0]    h2c_byp_in_st_port_id,
  output logic          h2c_byp_in_st_sdi,
  output logic [10:0]   h2c_byp_in_st_qid,
  output logic          h2c_byp_in_st_error,
  output logic [7:0]    h2c_byp_in_st_func,
  output logic [15:0]   h2c_byp_in_st_cidx,
  output logic          h2c_byp_in_st_no_dma,
  output logic          h2c_byp_in_st_vld,
  input  logic          h2c_byp_in_st_rdy,
  // H2C data
  input  logic [511:0]  m_axis_h2c_tdata,
  input  logic [31:0]   m_axis_h2c_tcrc,
  input  logic [10:0]   m_axis_h2c_tuser_qid,
  input  logic [2:0]    m_axis_h2c_tuser_port_id,
  input  logic          m_axis_h2c_tuser_err,
  input  logic [31:0]   m_axis_h2c_tuser_mdata,
  input  logic [5:0]    m_axis_h2c_tuser_mty,
  input  logic          m_axis_h2c_tuser_zero_byte,
  input  logic          m_axis_h2c_tvalid,
  input  logic          m_axis_h2c_tlast,
  output logic          m_axis_h2c_tready,
  // H2C status (queue status out)
  input  logic [7:0]    qsts_out_op,
  input  logic [63:0]   qsts_out_data,
  input  logic [2:0]    qsts_out_port_id,
  input  logic [12:0]   qsts_out_qid,
  input  logic          qsts_out_vld,
  output logic          qsts_out_rdy,

  // descriptor status (observed only; the static ties tm_dsc_sts_rdy high)
  input  logic          tm_dsc_sts_vld,
  input  logic          tm_dsc_sts_error,
  input  logic [10:0]   tm_dsc_sts_qid,
  input  logic          tm_dsc_sts_dir,
  input  logic          tm_dsc_sts_qinv,
  input  logic          tm_dsc_sts_qen,
  input  logic [15:0]   tm_dsc_sts_avl,
  output logic          tm_dsc_sts_rdy
);
  localparam integer NCH   = N_HCHAN;      // 3, as the static
  localparam integer NCTL  = 64;
  localparam integer NSTAT = 128;

  // ================================================================== static's DMA section (static_top_qdma_tmplt.txt)
  qdmaH2CS    h2c_data ();
  qdmaH2CS    h2c_data_int ();
  qdmaH2CIntf h2c_cmd ();
  qdmaH2CIntf h2c_cmd_int ();
  qdmaH2CSts  h2c_sts ();
  qdmaC2HS    c2h_data ();
  qdmaC2HS    c2h_data_int ();
  qdmaC2HIntf c2h_cmd ();
  qdmaC2HIntf c2h_cmd_int ();
  qdmaC2HSts  c2h_sts ();
  logic        qdma_pfch_tag_valid;
  logic [11:0] qdma_pfch_tag_qid;
  logic [6:0]  qdma_pfch_tag;

  AXI4S   axis_dyn_out_int [NCH] (.aclk(clk), .aresetn(rstn));
  AXI4S   axis_dyn_in_int  [NCH] (.aclk(clk), .aresetn(rstn));
  dmaIntf dma_rd_req_int   [NCH] (.aclk(clk));
  dmaIntf dma_wr_req_int   [NCH] (.aclk(clk));

  // H2C data stream (IP -> h2c_data_int)
  assign h2c_data_int.payload.tdata     = m_axis_h2c_tdata;
  assign h2c_data_int.payload.tcrc      = m_axis_h2c_tcrc;
  assign h2c_data_int.payload.qid[10:0] = m_axis_h2c_tuser_qid;
  assign h2c_data_int.payload.qid[11]   = 1'b0;
  assign h2c_data_int.payload.port_id   = m_axis_h2c_tuser_port_id;
  assign h2c_data_int.payload.err       = m_axis_h2c_tuser_err;
  assign h2c_data_int.payload.mdata     = m_axis_h2c_tuser_mdata;
  assign h2c_data_int.payload.mty       = m_axis_h2c_tuser_mty;
  assign h2c_data_int.payload.zero_byte = m_axis_h2c_tuser_zero_byte;
  assign h2c_data_int.payload.rsrvd     = 1'b0;
  assign h2c_data_int.tlast             = m_axis_h2c_tlast;
  assign h2c_data_int.tvalid            = m_axis_h2c_tvalid;
  assign m_axis_h2c_tready              = h2c_data_int.tready;

  // H2C command stream (h2c_cmd -> IP descriptor bypass in)
  assign h2c_byp_in_st_addr     = h2c_cmd.req.addr;
  assign h2c_byp_in_st_cidx     = h2c_cmd.req.cidx;
  assign h2c_byp_in_st_eop      = h2c_cmd.req.eop;
  assign h2c_byp_in_st_error    = h2c_cmd.req.error;
  assign h2c_byp_in_st_func     = h2c_cmd.req.func[7:0];
  assign h2c_byp_in_st_len      = h2c_cmd.req.len;
  assign h2c_byp_in_st_mrkr_req = h2c_cmd.req.mrkr_req;
  assign h2c_byp_in_st_no_dma   = h2c_cmd.req.no_dma;
  assign h2c_byp_in_st_port_id  = h2c_cmd.req.port_id;
  assign h2c_byp_in_st_qid      = h2c_cmd.req.qid[10:0];
  assign h2c_byp_in_st_sdi      = h2c_cmd.req.sdi;
  assign h2c_byp_in_st_sop      = h2c_cmd.req.sop;
  assign h2c_byp_in_st_vld      = h2c_cmd.valid;
  assign h2c_cmd.ready          = h2c_byp_in_st_rdy;

  // H2C status
  assign h2c_sts.data    = qsts_out_data;
  assign h2c_sts.op      = qsts_out_op;
  assign h2c_sts.port_id = qsts_out_port_id;
  assign h2c_sts.qid     = {1'b0, qsts_out_qid[10:0]};
  assign h2c_sts.valid   = qsts_out_vld;
  assign qsts_out_rdy    = 1'b1;

  // C2H data stream (c2h_data -> IP)
  assign s_axis_c2h_tdata         = c2h_data.payload.tdata;
  assign s_axis_c2h_tcrc          = c2h_data.payload.tcrc;
  assign s_axis_c2h_ctrl_len      = c2h_data.payload.len;
  assign s_axis_c2h_ctrl_qid      = c2h_data.payload.qid[10:0];
  assign s_axis_c2h_ctrl_has_cmpt = c2h_data.payload.has_cmpt;
  assign s_axis_c2h_ctrl_marker   = c2h_data.payload.marker;
  assign s_axis_c2h_ctrl_port_id  = c2h_data.payload.port_id;
  assign s_axis_c2h_ctrl_ecc      = c2h_data.payload.ecc;
  assign s_axis_c2h_mty           = c2h_data.payload.mty;
  assign s_axis_c2h_tlast         = c2h_data.tlast;
  assign s_axis_c2h_tvalid        = c2h_data.tvalid;
  assign c2h_data.tready          = s_axis_c2h_tready;

  // C2H command stream (c2h_cmd -> IP descriptor bypass in)
  assign c2h_byp_in_st_csh_addr     = c2h_cmd.req.addr;
  assign c2h_byp_in_st_csh_error    = c2h_cmd.req.error;
  assign c2h_byp_in_st_csh_func     = c2h_cmd.req.func[7:0];
  assign c2h_byp_in_st_csh_pfch_tag = c2h_cmd.req.pfch_tag;
  assign c2h_byp_in_st_csh_port_id  = c2h_cmd.req.port_id;
  assign c2h_byp_in_st_csh_qid      = c2h_cmd.req.qid[10:0];
  assign c2h_byp_in_st_csh_vld      = c2h_cmd.valid;
  assign c2h_cmd.ready              = c2h_byp_in_st_csh_rdy;

  // C2H status
  assign c2h_sts.drop  = axis_c2h_status_drop;
  assign c2h_sts.error = axis_c2h_status_error;
  assign c2h_sts.last  = axis_c2h_status_last;
  assign c2h_sts.qid   = {1'b0, axis_c2h_status_qid};
  assign c2h_sts.cmp   = axis_c2h_status_cmp;
  assign c2h_sts.valid = axis_c2h_status_valid;

  assign tm_dsc_sts_rdy = 1'b1;

  qdma_rd_wrapper #(
    .N_CHAN(NCH)
  ) inst_qdma_rd_wrapper (
    .aclk(clk),
    .aresetn(rstn),
    .s_dma_rd(dma_rd_req_int),
    .m_qdma_h2c_cmd(h2c_cmd_int),
    .s_qdma_h2c_sts(h2c_sts),
    .qdma_out(h2c_data),
    .dyn_out(axis_dyn_out_int)
  );

  qdma_wr_wrapper #(
    .N_CHAN(NCH),
    .N_QUEUES_PER_CHAN(N_OUTSTANDING)
  ) inst_qdma_wr_wrapper (
    .aclk(clk),
    .aresetn(rstn),
    .s_dma_wr(dma_wr_req_int),
    .m_qdma_c2h_cmd(c2h_cmd_int),
    .s_qdma_c2h_sts(c2h_sts),
    .qdma_in(c2h_data_int),
    .dyn_in(axis_dyn_in_int),
    .pfch_tag_valid(qdma_pfch_tag_valid),
    .pfch_tag_qid(qdma_pfch_tag_qid),
    .pfch_tag(qdma_pfch_tag)
  );

  qdma_reg_slice inst_qdma_reg_slice (
    .aclk(clk),
    .aresetn(rstn),
    .s_h2c_cmd(h2c_cmd_int),
    .m_h2c_cmd(h2c_cmd),
    .s_c2h_cmd(c2h_cmd_int),
    .m_c2h_cmd(c2h_cmd),
    .s_h2c_data(h2c_data_int),
    .m_h2c_data(h2c_data),
    .s_c2h_data(c2h_data_int),
    .m_c2h_data(c2h_data)
  );

  // ================================================================== BAR4: control + status
  logic [63:0]     ctl  [0:NCTL-1];
  logic [NCTL-1:0] ctl_wr;
  logic [63:0]     stat [0:NSTAT-1];

  bar4_ctl #(.ID_W(4), .DATA_W(512), .ADDR_W(64), .NCTL(NCTL), .NSTAT(NSTAT)) inst_bar4 (
    .clk(clk), .rstn(rstn),
    .awid(m_axib_awid), .awaddr(m_axib_awaddr), .awlen(m_axib_awlen), .awvalid(m_axib_awvalid), .awready(m_axib_awready),
    .wdata(m_axib_wdata), .wstrb(m_axib_wstrb), .wlast(m_axib_wlast), .wvalid(m_axib_wvalid), .wready(m_axib_wready),
    .bid(m_axib_bid), .bresp(m_axib_bresp), .bvalid(m_axib_bvalid), .bready(m_axib_bready),
    .arid(m_axib_arid), .araddr(m_axib_araddr), .arlen(m_axib_arlen), .arvalid(m_axib_arvalid), .arready(m_axib_arready),
    .rid(m_axib_rid), .rdata(m_axib_rdata), .rresp(m_axib_rresp), .rlast(m_axib_rlast), .rvalid(m_axib_rvalid),
    .rready(m_axib_rready),
    .ctl(ctl), .ctl_wr(ctl_wr), .stat(stat)
  );

  // Control registers (64-bit, window offset 8 * index):
  //   0 CMD (write pulses): bit0 C2H go, bit1 H2C go, bit2 clear counters, bit3 abort
  //   1 PFCH_TAG: the driver's static_slave register 17: bit31 valid (level), [19:8] qid - 256, [6:0] tag
  //   2 C2H channel mask [2:0]      3 H2C channel mask [2:0]
  //   8 + 8*ch  C2H ch: +0 base, +1 len, +2 count, +3 stride, +4 wrap, +5 seed
  //  32 + 8*ch  H2C ch: same layout
  wire cmd_wr  = ctl_wr[0];
  wire c2h_go  = cmd_wr && ctl[0][0];
  wire h2c_go  = cmd_wr && ctl[0][1];
  wire cnt_clr = cmd_wr && ctl[0][2];
  wire abort   = cmd_wr && ctl[0][3];
  assign qdma_pfch_tag_valid = ctl[1][31];
  assign qdma_pfch_tag_qid   = ctl[1][19:8];
  assign qdma_pfch_tag       = ctl[1][6:0];

  logic [NCH-1:0][63:0] c2h_base, h2c_base;
  logic [NCH-1:0][15:0] c2h_len, h2c_len;
  logic [NCH-1:0][31:0] c2h_cnt, h2c_cnt, c2h_stride, h2c_stride, c2h_wrap, h2c_wrap, c2h_seed, h2c_seed;
  for (genvar ch = 0; ch < NCH; ch++) begin : g_cfg
    assign c2h_base[ch]   = ctl[8 + 8*ch + 0];
    assign c2h_len[ch]    = ctl[8 + 8*ch + 1][15:0];
    assign c2h_cnt[ch]    = ctl[8 + 8*ch + 2][31:0];
    assign c2h_stride[ch] = ctl[8 + 8*ch + 3][31:0];
    assign c2h_wrap[ch]   = ctl[8 + 8*ch + 4][31:0];
    assign c2h_seed[ch]   = ctl[8 + 8*ch + 5][31:0];
    assign h2c_base[ch]   = ctl[32 + 8*ch + 0];
    assign h2c_len[ch]    = ctl[32 + 8*ch + 1][15:0];
    assign h2c_cnt[ch]    = ctl[32 + 8*ch + 2][31:0];
    assign h2c_stride[ch] = ctl[32 + 8*ch + 3][31:0];
    assign h2c_wrap[ch]   = ctl[32 + 8*ch + 4][31:0];
    assign h2c_seed[ch]   = ctl[32 + 8*ch + 5][31:0];
  end

  // ================================================================== shell stand-in
  logic     [NCH-1:0]        g_wr_valid, g_wr_ready, g_rd_valid, g_rd_ready;
  dma_req_t [NCH-1:0]        g_wr_req, g_rd_req;
  logic     [NCH-1:0]        g_c2h_tvalid, g_c2h_tready, g_c2h_tlast, g_h2c_tvalid, g_h2c_tready, g_h2c_tlast;
  logic     [NCH-1:0][511:0] g_c2h_tdata, g_h2c_tdata;
  logic     [NCH-1:0][63:0]  g_c2h_tkeep, g_h2c_tkeep;
  logic     [NCH-1:0]        wr_done, rd_done;
  logic     [NCH-1:0][63:0]  st_c2h_req, st_c2h_pkt, st_c2h_state, st_h2c_req, st_h2c_pkt, st_h2c_beat, st_h2c_bad;
  logic     [NCH-1:0][63:0]  st_h2c_lasterr, st_h2c_tlast, st_h2c_first0, st_h2c_first1, st_h2c_state;

  for (genvar ch = 0; ch < NCH; ch++) begin : g_if
    assign dma_wr_req_int[ch].req      = g_wr_req[ch];
    assign dma_wr_req_int[ch].valid    = g_wr_valid[ch];
    assign g_wr_ready[ch]              = dma_wr_req_int[ch].ready;
    assign wr_done[ch]                 = dma_wr_req_int[ch].rsp.done;
    assign axis_dyn_in_int[ch].tdata   = g_c2h_tdata[ch];
    assign axis_dyn_in_int[ch].tkeep   = g_c2h_tkeep[ch];
    assign axis_dyn_in_int[ch].tlast   = g_c2h_tlast[ch];
    assign axis_dyn_in_int[ch].tvalid  = g_c2h_tvalid[ch];
    assign g_c2h_tready[ch]            = axis_dyn_in_int[ch].tready;

    assign dma_rd_req_int[ch].req      = g_rd_req[ch];
    assign dma_rd_req_int[ch].valid    = g_rd_valid[ch];
    assign g_rd_ready[ch]              = dma_rd_req_int[ch].ready;
    assign rd_done[ch]                 = dma_rd_req_int[ch].rsp.done;
    assign g_h2c_tdata[ch]             = axis_dyn_out_int[ch].tdata;
    assign g_h2c_tkeep[ch]             = axis_dyn_out_int[ch].tkeep;
    assign g_h2c_tlast[ch]             = axis_dyn_out_int[ch].tlast;
    assign g_h2c_tvalid[ch]            = axis_dyn_out_int[ch].tvalid;
    assign axis_dyn_out_int[ch].tready = g_h2c_tready[ch];
  end

  qdma_gen #(.N_CH(NCH)) inst_gen (
    .clk(clk), .rstn(rstn),
    .c2h_go(c2h_go), .h2c_go(h2c_go), .abort(abort), .clr(cnt_clr),
    .c2h_mask(ctl[2][NCH-1:0]), .h2c_mask(ctl[3][NCH-1:0]),
    .c2h_base(c2h_base), .h2c_base(h2c_base), .c2h_len(c2h_len), .h2c_len(h2c_len), .c2h_cnt(c2h_cnt), .h2c_cnt(h2c_cnt),
    .c2h_stride(c2h_stride), .h2c_stride(h2c_stride), .c2h_wrap(c2h_wrap), .h2c_wrap(h2c_wrap),
    .c2h_seed(c2h_seed), .h2c_seed(h2c_seed),
    .wr_valid(g_wr_valid), .wr_ready(g_wr_ready), .wr_req(g_wr_req),
    .c2h_tvalid(g_c2h_tvalid), .c2h_tready(g_c2h_tready), .c2h_tdata(g_c2h_tdata), .c2h_tkeep(g_c2h_tkeep), .c2h_tlast(g_c2h_tlast),
    .rd_valid(g_rd_valid), .rd_ready(g_rd_ready), .rd_req(g_rd_req),
    .h2c_tvalid(g_h2c_tvalid), .h2c_tready(g_h2c_tready), .h2c_tdata(g_h2c_tdata), .h2c_tkeep(g_h2c_tkeep), .h2c_tlast(g_h2c_tlast),
    .st_c2h_req(st_c2h_req), .st_c2h_pkt(st_c2h_pkt), .st_c2h_state(st_c2h_state),
    .st_h2c_req(st_h2c_req), .st_h2c_pkt(st_h2c_pkt), .st_h2c_beat(st_h2c_beat), .st_h2c_bad(st_h2c_bad),
    .st_h2c_lasterr(st_h2c_lasterr), .st_h2c_tlast(st_h2c_tlast), .st_h2c_first0(st_h2c_first0),
    .st_h2c_first1(st_h2c_first1), .st_h2c_state(st_h2c_state)
  );

  // ================================================================== counters at the QDMA's ports
  // Inputs registered once (timing: the status/counter logic is off the IP's paths).
  logic        r_c2h_cmd_v, r_c2h_cmd_r, r_c2h_v, r_c2h_r, r_c2h_last, r_sts_v, r_sts_drop, r_sts_err, r_sts_cmp, r_sts_last;
  logic        r_dmawr, r_h2c_cmd_v, r_h2c_cmd_r, r_h2c_v, r_h2c_r, r_h2c_last, r_h2c_err, r_h2c_zb, r_qsts_v, r_tm_v, r_tm_err;
  logic [10:0] r_sts_qid, r_c2h_cmd_qid, r_tm_qid;
  logic [6:0]  r_c2h_cmd_tag;
  logic [5:0]  r_c2h_mty, r_h2c_mty;
  logic [7:0]  r_qsts_op;
  logic [12:0] r_qsts_qid;
  logic [63:0] r_qsts_data;
  logic [15:0] r_c2h_len;
  logic        r_tm_dir, r_tm_qinv, r_tm_qen;
  logic [15:0] r_tm_avl;
  always_ff @(posedge clk) begin
    r_c2h_cmd_v <= c2h_byp_in_st_csh_vld; r_c2h_cmd_r <= c2h_byp_in_st_csh_rdy;
    r_c2h_cmd_qid <= c2h_byp_in_st_csh_qid; r_c2h_cmd_tag <= c2h_byp_in_st_csh_pfch_tag;
    r_c2h_v <= s_axis_c2h_tvalid; r_c2h_r <= s_axis_c2h_tready; r_c2h_last <= s_axis_c2h_tlast; r_c2h_mty <= s_axis_c2h_mty;
    r_c2h_len <= s_axis_c2h_ctrl_len;
    r_sts_v <= axis_c2h_status_valid; r_sts_drop <= axis_c2h_status_drop; r_sts_err <= axis_c2h_status_error;
    r_sts_cmp <= axis_c2h_status_cmp; r_sts_last <= axis_c2h_status_last; r_sts_qid <= axis_c2h_status_qid;
    r_dmawr <= axis_c2h_dmawr_cmp;
    r_h2c_cmd_v <= h2c_byp_in_st_vld; r_h2c_cmd_r <= h2c_byp_in_st_rdy;
    r_h2c_v <= m_axis_h2c_tvalid; r_h2c_r <= m_axis_h2c_tready; r_h2c_last <= m_axis_h2c_tlast;
    r_h2c_err <= m_axis_h2c_tuser_err; r_h2c_zb <= m_axis_h2c_tuser_zero_byte; r_h2c_mty <= m_axis_h2c_tuser_mty;
    r_qsts_v <= qsts_out_vld; r_qsts_op <= qsts_out_op; r_qsts_qid <= qsts_out_qid; r_qsts_data <= qsts_out_data;
    r_tm_v <= tm_dsc_sts_vld; r_tm_err <= tm_dsc_sts_error; r_tm_qid <= tm_dsc_sts_qid; r_tm_dir <= tm_dsc_sts_dir;
    r_tm_qinv <= tm_dsc_sts_qinv; r_tm_qen <= tm_dsc_sts_qen; r_tm_avl <= tm_dsc_sts_avl;
  end

  wire c2h_cmd_hs = r_c2h_cmd_v && r_c2h_cmd_r;
  wire c2h_hs     = r_c2h_v && r_c2h_r;
  wire h2c_cmd_hs = r_h2c_cmd_v && r_h2c_cmd_r;
  wire h2c_hs     = r_h2c_v && r_h2c_r;

  logic [63:0] cyc, n_clr;
  logic [63:0] k_c2h_cmd, k_c2h_beat, k_c2h_tlast, k_c2h_bytes, k_sts, k_sts_drop, k_sts_err, k_sts_cmp, k_sts_last;
  logic [63:0] k_sts_lastv, k_dmawr, k_c2h_cmd_stall, k_c2h_stall, t_c2h_cmd0, t_c2h_beat_l, t_c2h_sts_l, t_dmawr_l;
  logic [63:0] k_h2c_cmd, k_h2c_beat, k_h2c_tlast, k_h2c_bytes, k_h2c_err, k_h2c_zb, k_qsts, k_qsts_op1, k_qsts_err;
  logic [63:0] k_qsts_lastv, k_h2c_cmd_stall, t_h2c_cmd0, t_h2c_beat_l, t_qsts_l, k_tm, k_tm_err, k_tm_lastv;
  logic [63:0] k_c2h_cmd_lastv, k_c2h_lenerr;
  logic [NCH-1:0][63:0] k_wr_done, k_rd_done;
  logic        c2h_cmd0_seen, h2c_cmd0_seen;
  logic [15:0] c2h_pkt_bytes;

  always_ff @(posedge clk) begin
    if (!rstn) cyc <= 0; else cyc <= cyc + 1'b1;
  end

  always_ff @(posedge clk) begin
    if (!rstn || cnt_clr) begin
      if (!rstn) n_clr <= 0; else n_clr <= n_clr + 1'b1;
      {k_c2h_cmd, k_c2h_beat, k_c2h_tlast, k_c2h_bytes, k_sts, k_sts_drop, k_sts_err, k_sts_cmp, k_sts_last} <= '0;
      {k_sts_lastv, k_dmawr, k_c2h_cmd_stall, k_c2h_stall, t_c2h_cmd0, t_c2h_beat_l, t_c2h_sts_l, t_dmawr_l} <= '0;
      {k_h2c_cmd, k_h2c_beat, k_h2c_tlast, k_h2c_bytes, k_h2c_err, k_h2c_zb, k_qsts, k_qsts_op1, k_qsts_err} <= '0;
      {k_qsts_lastv, k_h2c_cmd_stall, t_h2c_cmd0, t_h2c_beat_l, t_qsts_l, k_tm, k_tm_err, k_tm_lastv} <= '0;
      {k_c2h_cmd_lastv, k_c2h_lenerr} <= '0;
      k_wr_done <= '0; k_rd_done <= '0;
      c2h_cmd0_seen <= 1'b0; h2c_cmd0_seen <= 1'b0; c2h_pkt_bytes <= 0;
    end else begin
      // C2H
      if (c2h_cmd_hs) begin
        k_c2h_cmd <= k_c2h_cmd + 1'b1;
        k_c2h_cmd_lastv <= {41'd0, r_c2h_cmd_tag, 5'd0, r_c2h_cmd_qid};   // [22:16] tag, [10:0] qid
        if (!c2h_cmd0_seen) begin c2h_cmd0_seen <= 1'b1; t_c2h_cmd0 <= cyc; end
      end
      if (r_c2h_cmd_v && !r_c2h_cmd_r) k_c2h_cmd_stall <= k_c2h_cmd_stall + 1'b1;
      if (r_c2h_v && !r_c2h_r) k_c2h_stall <= k_c2h_stall + 1'b1;
      if (c2h_hs) begin
        k_c2h_beat  <= k_c2h_beat + 1'b1;
        k_c2h_bytes <= k_c2h_bytes + (r_c2h_last ? (7'd64 - r_c2h_mty) : 7'd64);
        t_c2h_beat_l <= cyc;
        // ctrl_len must equal the bytes of the packet (the wrapper drives it on every beat)
        if (r_c2h_last) begin
          k_c2h_tlast <= k_c2h_tlast + 1'b1;
          if (c2h_pkt_bytes + (7'd64 - r_c2h_mty) != r_c2h_len) k_c2h_lenerr <= k_c2h_lenerr + 1'b1;
          c2h_pkt_bytes <= 0;
        end else begin
          c2h_pkt_bytes <= c2h_pkt_bytes + 16'd64;
        end
      end
      if (r_sts_v) begin
        k_sts <= k_sts + 1'b1;
        if (r_sts_drop) k_sts_drop <= k_sts_drop + 1'b1;
        if (r_sts_err)  k_sts_err  <= k_sts_err + 1'b1;
        if (r_sts_cmp)  k_sts_cmp  <= k_sts_cmp + 1'b1;
        if (r_sts_last) k_sts_last <= k_sts_last + 1'b1;
        k_sts_lastv <= {44'd0, r_sts_last, r_sts_cmp, r_sts_err, r_sts_drop, 5'd0, r_sts_qid};
        t_c2h_sts_l <= cyc;
      end
      if (r_dmawr) begin k_dmawr <= k_dmawr + 1'b1; t_dmawr_l <= cyc; end
      // H2C
      if (h2c_cmd_hs) begin
        k_h2c_cmd <= k_h2c_cmd + 1'b1;
        if (!h2c_cmd0_seen) begin h2c_cmd0_seen <= 1'b1; t_h2c_cmd0 <= cyc; end
      end
      if (r_h2c_cmd_v && !r_h2c_cmd_r) k_h2c_cmd_stall <= k_h2c_cmd_stall + 1'b1;
      if (h2c_hs) begin
        k_h2c_beat  <= k_h2c_beat + 1'b1;
        k_h2c_bytes <= k_h2c_bytes + (r_h2c_last ? (7'd64 - r_h2c_mty) : 7'd64);
        t_h2c_beat_l <= cyc;
        if (r_h2c_last) k_h2c_tlast <= k_h2c_tlast + 1'b1;
        if (r_h2c_err)  k_h2c_err <= k_h2c_err + 1'b1;
        if (r_h2c_zb)   k_h2c_zb <= k_h2c_zb + 1'b1;
      end
      if (r_qsts_v) begin
        k_qsts <= k_qsts + 1'b1;
        if (r_qsts_op == 8'h1) k_qsts_op1 <= k_qsts_op1 + 1'b1;
        if (r_qsts_op == 8'h1 && r_qsts_data[16]) k_qsts_err <= k_qsts_err + 1'b1;
        k_qsts_lastv <= {r_qsts_op, 3'd0, r_qsts_qid, 8'd0, r_qsts_data[31:0]};
        t_qsts_l <= cyc;
      end
      if (r_tm_v) begin
        k_tm <= k_tm + 1'b1;
        if (r_tm_err) k_tm_err <= k_tm_err + 1'b1;
        k_tm_lastv <= {28'd0, r_tm_err, r_tm_qinv, r_tm_qen, r_tm_dir, r_tm_avl, 5'd0, r_tm_qid};   // [35] err [34] qinv [33] qen [32] dir [31:16] avl [10:0] qid
      end
      for (int ch = 0; ch < NCH; ch++) begin
        if (wr_done[ch]) k_wr_done[ch] <= k_wr_done[ch] + 1'b1;
        if (rd_done[ch]) k_rd_done[ch] <= k_rd_done[ch] + 1'b1;
      end
    end
  end

  // Status words (64-bit, window read offset 8 * index); per channel ch at 64 + 16*ch.
  always_comb begin
    for (int s = 0; s < NSTAT; s++) stat[s] = 64'd0;
    stat[0]  = 64'h51DA_0C0F_FEE0_0002;   // rev2 magic
    stat[1]  = cyc;
    stat[2]  = k_c2h_cmd;        stat[3]  = k_c2h_beat;     stat[4]  = k_c2h_tlast;    stat[5]  = k_c2h_bytes;
    stat[6]  = k_sts;            stat[7]  = k_sts_drop;     stat[8]  = k_sts_err;      stat[9]  = k_sts_cmp;
    stat[10] = k_sts_last;       stat[11] = k_sts_lastv;    stat[12] = k_dmawr;        stat[13] = k_c2h_cmd_stall;
    stat[14] = k_c2h_stall;      stat[15] = t_c2h_cmd0;     stat[16] = t_c2h_beat_l;   stat[17] = t_c2h_sts_l;
    stat[18] = t_dmawr_l;        stat[19] = k_h2c_cmd;      stat[20] = k_h2c_beat;     stat[21] = k_h2c_tlast;
    stat[22] = k_h2c_bytes;      stat[23] = k_h2c_err;      stat[24] = k_h2c_zb;       stat[25] = k_qsts;
    stat[26] = k_qsts_op1;       stat[27] = k_qsts_err;     stat[28] = k_qsts_lastv;   stat[29] = k_h2c_cmd_stall;
    stat[30] = t_h2c_cmd0;       stat[31] = t_h2c_beat_l;   stat[32] = t_qsts_l;       stat[33] = k_tm;
    stat[34] = k_tm_err;         stat[35] = k_tm_lastv;     stat[36] = k_c2h_cmd_lastv; stat[37] = k_c2h_lenerr;
    stat[38] = n_clr;
    for (int ch = 0; ch < NCH; ch++) begin
      stat[64 + 16*ch + 0]  = st_c2h_req[ch];
      stat[64 + 16*ch + 1]  = st_c2h_pkt[ch];
      stat[64 + 16*ch + 2]  = k_wr_done[ch];
      stat[64 + 16*ch + 3]  = st_c2h_state[ch];
      stat[64 + 16*ch + 4]  = st_h2c_req[ch];
      stat[64 + 16*ch + 5]  = st_h2c_pkt[ch];
      stat[64 + 16*ch + 6]  = st_h2c_beat[ch];
      stat[64 + 16*ch + 7]  = st_h2c_bad[ch];
      stat[64 + 16*ch + 8]  = st_h2c_lasterr[ch];
      stat[64 + 16*ch + 9]  = k_rd_done[ch];
      stat[64 + 16*ch + 10] = st_h2c_first0[ch];
      stat[64 + 16*ch + 11] = st_h2c_first1[ch];
      stat[64 + 16*ch + 12] = st_h2c_tlast[ch];
      stat[64 + 16*ch + 13] = st_h2c_state[ch];
    end
  end

  // ================================================================== BAR0 AXI-Lite: always-OK responder
  logic       axil_bv, axil_rv, axil_awp, axil_wp;
  logic [31:0] axil_rd;
  assign m_axil_awready = !axil_awp && !axil_bv;
  assign m_axil_wready  = !axil_wp && !axil_bv;
  assign m_axil_bvalid  = axil_bv;
  assign m_axil_bresp   = 2'b00;
  assign m_axil_arready = !axil_rv;
  assign m_axil_rvalid  = axil_rv;
  assign m_axil_rdata   = axil_rd;
  assign m_axil_rresp   = 2'b00;
  always_ff @(posedge clk) begin
    if (!rstn) begin
      axil_bv <= 1'b0; axil_rv <= 1'b0; axil_awp <= 1'b0; axil_wp <= 1'b0; axil_rd <= 0;
    end else begin
      if (m_axil_awvalid && m_axil_awready) axil_awp <= 1'b1;
      if (m_axil_wvalid && m_axil_wready)   axil_wp  <= 1'b1;
      if ((axil_awp || (m_axil_awvalid && m_axil_awready)) && (axil_wp || (m_axil_wvalid && m_axil_wready)) && !axil_bv) begin
        axil_bv <= 1'b1; axil_awp <= 1'b0; axil_wp <= 1'b0;
      end else if (axil_bv && m_axil_bready) axil_bv <= 1'b0;
      if (m_axil_arvalid && m_axil_arready) begin axil_rv <= 1'b1; axil_rd <= 32'h9F3F_0000 | m_axil_araddr[15:0]; end
      else if (axil_rv && m_axil_rready) axil_rv <= 1'b0;
    end
  end
endmodule
