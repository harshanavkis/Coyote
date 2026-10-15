// qdma_hw rev2 unit TB: qdma_hw2_core (Coyote wrappers + reg slices + qdma_gen + bar4_ctl) against a behavioural QDMA.
// The model is NOT the IP (the QDMA sim hangs in xsim); it checks what the core hands the QDMA:
//  C2H: descriptor before data, packet qid = descriptor qid, prefetch tag = the tag the "driver" programmed for that
//       queue, len/mty/tlast consistent, ECC = AMD's example encoder (qdma_ecc_enc) on every beat; data -> memory.
//  H2C: sop/eop/mrkr_req set, data served from memory, qsts_out op 1 per packet.
// The test drives BAR4 like the kernel module will (64-bit writes/reads in the 0x0FF0_0000 window).
`timescale 1ns / 1ps
module tb_core;
  localparam logic [63:0] WIN = 64'h0FF0_0000;
  logic clk = 1'b0, rstn = 1'b0;
  always #2 clk = ~clk;   // 250 MHz

  int errors = 0;
  task automatic err(input string s); errors++; if (errors < 200) $display("[%0t] ERROR %s", $time, s); endtask

  // ---------------------------------------------------------------- DUT ports
  logic [3:0]   m_axib_awid = 0, m_axib_arid = 0, m_axib_bid, m_axib_rid;
  logic [63:0]  m_axib_awaddr = 0, m_axib_araddr = 0;
  logic [7:0]   m_axib_awlen = 0, m_axib_arlen = 0;
  logic         m_axib_awvalid = 0, m_axib_awready, m_axib_wlast = 0, m_axib_wvalid = 0, m_axib_wready;
  logic         m_axib_bvalid, m_axib_bready = 1, m_axib_arvalid = 0, m_axib_arready, m_axib_rlast, m_axib_rvalid, m_axib_rready = 1;
  logic [511:0] m_axib_wdata = 0, m_axib_rdata;
  logic [63:0]  m_axib_wstrb = 0;
  logic [1:0]   m_axib_bresp, m_axib_rresp;
  logic [31:0]  m_axil_rdata;
  logic [1:0]   m_axil_bresp, m_axil_rresp;
  logic         m_axil_awready, m_axil_wready, m_axil_bvalid, m_axil_arready, m_axil_rvalid;

  logic [63:0]  c2h_byp_in_st_csh_addr;
  logic [2:0]   c2h_byp_in_st_csh_port_id;
  logic [10:0]  c2h_byp_in_st_csh_qid;
  logic         c2h_byp_in_st_csh_error, c2h_byp_in_st_csh_vld, c2h_byp_in_st_csh_rdy = 0;
  logic [7:0]   c2h_byp_in_st_csh_func;
  logic [6:0]   c2h_byp_in_st_csh_pfch_tag;
  logic [511:0] s_axis_c2h_tdata;
  logic [31:0]  s_axis_c2h_tcrc;
  logic         s_axis_c2h_ctrl_marker, s_axis_c2h_ctrl_has_cmpt, s_axis_c2h_tvalid, s_axis_c2h_tlast, s_axis_c2h_tready = 0;
  logic [2:0]   s_axis_c2h_ctrl_port_id;
  logic [6:0]   s_axis_c2h_ctrl_ecc;
  logic [15:0]  s_axis_c2h_ctrl_len;
  logic [10:0]  s_axis_c2h_ctrl_qid;
  logic [5:0]   s_axis_c2h_mty;
  logic         axis_c2h_status_drop = 0, axis_c2h_status_valid = 0, axis_c2h_status_cmp = 0, axis_c2h_status_error = 0;
  logic         axis_c2h_status_last = 0, axis_c2h_dmawr_cmp = 0;
  logic [10:0]  axis_c2h_status_qid = 0;

  logic [63:0]  h2c_byp_in_st_addr;
  logic [15:0]  h2c_byp_in_st_len, h2c_byp_in_st_cidx;
  logic         h2c_byp_in_st_eop, h2c_byp_in_st_sop, h2c_byp_in_st_mrkr_req, h2c_byp_in_st_sdi, h2c_byp_in_st_error;
  logic         h2c_byp_in_st_no_dma, h2c_byp_in_st_vld, h2c_byp_in_st_rdy = 0;
  logic [2:0]   h2c_byp_in_st_port_id;
  logic [10:0]  h2c_byp_in_st_qid;
  logic [7:0]   h2c_byp_in_st_func;
  logic [511:0] m_axis_h2c_tdata = 0;
  logic [10:0]  m_axis_h2c_tuser_qid = 0;
  logic [5:0]   m_axis_h2c_tuser_mty = 0;
  logic         m_axis_h2c_tvalid = 0, m_axis_h2c_tlast = 0, m_axis_h2c_tready;
  logic [7:0]   qsts_out_op = 0;
  logic [63:0]  qsts_out_data = 0;
  logic [12:0]  qsts_out_qid = 0;
  logic         qsts_out_vld = 0, qsts_out_rdy, tm_dsc_sts_rdy;

  qdma_hw2_core dut (
    .clk(clk), .rstn(rstn),
    .m_axib_awid(m_axib_awid), .m_axib_awaddr(m_axib_awaddr), .m_axib_awlen(m_axib_awlen), .m_axib_awvalid(m_axib_awvalid),
    .m_axib_awready(m_axib_awready), .m_axib_wdata(m_axib_wdata), .m_axib_wstrb(m_axib_wstrb), .m_axib_wlast(m_axib_wlast),
    .m_axib_wvalid(m_axib_wvalid), .m_axib_wready(m_axib_wready), .m_axib_bid(m_axib_bid), .m_axib_bresp(m_axib_bresp),
    .m_axib_bvalid(m_axib_bvalid), .m_axib_bready(m_axib_bready), .m_axib_arid(m_axib_arid), .m_axib_araddr(m_axib_araddr),
    .m_axib_arlen(m_axib_arlen), .m_axib_arvalid(m_axib_arvalid), .m_axib_arready(m_axib_arready), .m_axib_rid(m_axib_rid),
    .m_axib_rdata(m_axib_rdata), .m_axib_rresp(m_axib_rresp), .m_axib_rlast(m_axib_rlast), .m_axib_rvalid(m_axib_rvalid),
    .m_axib_rready(m_axib_rready),
    .m_axil_awaddr(32'd0), .m_axil_awvalid(1'b0), .m_axil_awready(m_axil_awready), .m_axil_wdata(32'd0), .m_axil_wvalid(1'b0),
    .m_axil_wready(m_axil_wready), .m_axil_bvalid(m_axil_bvalid), .m_axil_bresp(m_axil_bresp), .m_axil_bready(1'b1),
    .m_axil_araddr(32'd0), .m_axil_arvalid(1'b0), .m_axil_arready(m_axil_arready), .m_axil_rdata(m_axil_rdata),
    .m_axil_rresp(m_axil_rresp), .m_axil_rvalid(m_axil_rvalid), .m_axil_rready(1'b1),
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
    .m_axis_h2c_tdata(m_axis_h2c_tdata), .m_axis_h2c_tcrc(32'd0), .m_axis_h2c_tuser_qid(m_axis_h2c_tuser_qid),
    .m_axis_h2c_tuser_port_id(3'd0), .m_axis_h2c_tuser_err(1'b0), .m_axis_h2c_tuser_mdata(32'd0),
    .m_axis_h2c_tuser_mty(m_axis_h2c_tuser_mty), .m_axis_h2c_tuser_zero_byte(1'b0), .m_axis_h2c_tvalid(m_axis_h2c_tvalid),
    .m_axis_h2c_tlast(m_axis_h2c_tlast), .m_axis_h2c_tready(m_axis_h2c_tready),
    .qsts_out_op(qsts_out_op), .qsts_out_data(qsts_out_data), .qsts_out_port_id(3'd0), .qsts_out_qid(qsts_out_qid),
    .qsts_out_vld(qsts_out_vld), .qsts_out_rdy(qsts_out_rdy),
    .tm_dsc_sts_vld(1'b0), .tm_dsc_sts_error(1'b0), .tm_dsc_sts_qid(11'd0), .tm_dsc_sts_dir(1'b0), .tm_dsc_sts_qinv(1'b0),
    .tm_dsc_sts_qen(1'b0), .tm_dsc_sts_avl(16'd0), .tm_dsc_sts_rdy(tm_dsc_sts_rdy)
  );

  // ---------------------------------------------------------------- reference ECC (AMD QDMA example's encoder + layout)
  logic [56:0] ref_ecc_in;
  logic [6:0]  ref_ecc;
  assign ref_ecc_in = {17'h0, 1'b0, 1'b0, 1'b0, 4'b0, s_axis_c2h_ctrl_has_cmpt, s_axis_c2h_ctrl_marker,
                       s_axis_c2h_ctrl_port_id, 1'b0, s_axis_c2h_ctrl_qid, s_axis_c2h_ctrl_len};
  qdma_ecc_enc #(.C_DATA_WIDTH(57), .C_CHK_BIT_WIDTH(7), .C_REG_INPUT(0), .C_REG_OUTPUT(0), .C_PIPELINE(0), .C_USE_CLKEN(0))
    ref_enc (.ecc_clk(1'b0), .ecc_reset(1'b0), .ecc_enc_clk_en_in(1'b1), .ecc_enc_data_in(ref_ecc_in),
             .ecc_enc_data_out(), .ecc_enc_chk_bits_out(ref_ecc));

  // ---------------------------------------------------------------- memory + pattern
  bit [7:0] mem [bit [63:0]];
  function automatic logic [31:0] pword(input int ch, input int i, input int k, input logic [31:0] seed);
    return seed + (32'(ch) << 24) + (32'(i) << 12) + 32'(k);
  endfunction
  function automatic logic [7:0] pbyte(input int ch, input int i, input int j, input logic [31:0] seed);
    logic [31:0] w = pword(ch, i, j / 4, seed);
    return w[8*(j%4) +: 8];
  endfunction
  function automatic logic [6:0] tag_of(input int qid); return 7'((qid - 256) * 2 + 3); endfunction

  // ---------------------------------------------------------------- C2H model
  typedef struct { logic [63:0] addr; logic [10:0] qid; logic [6:0] tag; } c2h_desc_t;
  c2h_desc_t dq[$];
  typedef struct { logic [10:0] qid; logic [15:0] len; bit [7:0] data[$]; } c2h_pkt_t;
  c2h_pkt_t  pq[$];
  int        pkt_before_desc = 0;
  int        sq[$];
  bit [7:0]  pkt[$];
  logic [10:0] pkt_qid; logic [15:0] pkt_len; bit in_pkt = 0;
  int        c2h_pkts = 0;
  int        rnd_c2h_cmd = 7, rnd_c2h_data = 8, rnd_sts = 3;   // out of 10: ready / status probabilities

  always @(posedge clk) begin
    c2h_byp_in_st_csh_rdy <= ($urandom % 10) < rnd_c2h_cmd;
    s_axis_c2h_tready     <= ($urandom % 10) < rnd_c2h_data;
    if (c2h_byp_in_st_csh_vld && c2h_byp_in_st_csh_rdy) begin
      if (c2h_byp_in_st_csh_error || c2h_byp_in_st_csh_func != 0 || c2h_byp_in_st_csh_port_id != 0) err("c2h desc error/func/port set");
      dq.push_back('{c2h_byp_in_st_csh_addr, c2h_byp_in_st_csh_qid, c2h_byp_in_st_csh_pfch_tag});
    end
    if (s_axis_c2h_tvalid && s_axis_c2h_tready) begin
      int nb;
      if (s_axis_c2h_ctrl_ecc !== ref_ecc) err($sformatf("c2h ecc %h ref %h (qid %0d len %0d)", s_axis_c2h_ctrl_ecc, ref_ecc, s_axis_c2h_ctrl_qid, s_axis_c2h_ctrl_len));
      if (s_axis_c2h_ctrl_has_cmpt || s_axis_c2h_ctrl_marker || s_axis_c2h_ctrl_port_id != 0) err("c2h ctrl has_cmpt/marker/port");
      if (!in_pkt) begin pkt_qid = s_axis_c2h_ctrl_qid; pkt_len = s_axis_c2h_ctrl_len; in_pkt = 1; end
      else if (s_axis_c2h_ctrl_qid != pkt_qid || s_axis_c2h_ctrl_len != pkt_len) err("c2h qid/len changed inside a packet");
      if (!s_axis_c2h_tlast && s_axis_c2h_mty != 0) err("c2h mty on a non-last beat");
      nb = s_axis_c2h_tlast ? 64 - s_axis_c2h_mty : 64;
      for (int b = 0; b < nb; b++) pkt.push_back(s_axis_c2h_tdata[8*b +: 8]);
      if (s_axis_c2h_tlast) begin
        in_pkt = 0;
        if (dq.size() == 0) pkt_before_desc++;     // the QDMA must pair it with the descriptor that follows
        pq.push_back('{pkt_qid, pkt_len, pkt});
        pkt.delete();
      end
    end
    // pair packets with descriptors in order (the wrapper issues both in the same order)
    while (dq.size() != 0 && pq.size() != 0) begin
      c2h_desc_t d;
      c2h_pkt_t  p;
      d = dq.pop_front(); p = pq.pop_front();
      if (d.qid != p.qid) err($sformatf("c2h packet qid %0d != descriptor qid %0d", p.qid, d.qid));
      if (d.qid < 256 || d.qid > 279) err($sformatf("c2h qid %0d outside 256..279", d.qid));
      if (d.tag != tag_of(d.qid)) err($sformatf("c2h qid %0d tag %0d != programmed %0d", d.qid, d.tag, tag_of(d.qid)));
      if (p.data.size() != p.len) err($sformatf("c2h packet %0d bytes, ctrl_len %0d", p.data.size(), p.len));
      for (int b = 0; b < p.data.size(); b++) mem[d.addr + b] = p.data[b];
      sq.push_back(d.qid);
      c2h_pkts++;
    end
    axis_c2h_status_valid <= 1'b0; axis_c2h_dmawr_cmp <= 1'b0;
    if (sq.size() != 0 && ($urandom % 10) < rnd_sts) begin
      axis_c2h_status_valid <= 1'b1; axis_c2h_status_qid <= 11'(sq.pop_front()); axis_c2h_status_last <= 1'b1;
      axis_c2h_status_drop <= 1'b0; axis_c2h_status_error <= 1'b0; axis_c2h_status_cmp <= 1'b0; axis_c2h_dmawr_cmp <= 1'b1;
    end
  end

  // ---------------------------------------------------------------- H2C model
  typedef struct { logic [63:0] addr; logic [15:0] len; logic [10:0] qid; } h2c_cmd_t;
  h2c_cmd_t hq[$];
  int       qsq[$];
  h2c_cmd_t cur;
  bit       hbusy = 0;
  int       hbeat, hnb;
  int       rnd_h2c_cmd = 7, rnd_h2c_data = 8;

  always @(posedge clk) begin
    h2c_byp_in_st_rdy <= ($urandom % 10) < rnd_h2c_cmd;
    if (h2c_byp_in_st_vld && h2c_byp_in_st_rdy) begin
      if (!h2c_byp_in_st_sop || !h2c_byp_in_st_eop || !h2c_byp_in_st_mrkr_req) err("h2c cmd without sop/eop/mrkr_req");
      if (h2c_byp_in_st_qid < 1 || h2c_byp_in_st_qid > 3) err($sformatf("h2c qid %0d outside 1..3", h2c_byp_in_st_qid));
      hq.push_back('{h2c_byp_in_st_addr, h2c_byp_in_st_len, h2c_byp_in_st_qid});
    end
    if (m_axis_h2c_tvalid && m_axis_h2c_tready) begin
      if (m_axis_h2c_tlast) begin hbusy = 0; qsq.push_back(cur.qid); end
      else hbeat++;
      m_axis_h2c_tvalid <= 1'b0;
    end
    if (!m_axis_h2c_tvalid || m_axis_h2c_tready) begin
      if (!hbusy && hq.size() != 0) begin cur = hq.pop_front(); hbusy = 1; hbeat = 0; hnb = (cur.len + 63) / 64; end
      if (hbusy && ($urandom % 10) < rnd_h2c_data) begin
        int rem;
        logic [511:0] d;
        rem = cur.len - hbeat * 64;
        for (int b = 0; b < 64; b++) d[8*b +: 8] = (b < rem && mem.exists(cur.addr + hbeat*64 + b)) ? mem[cur.addr + hbeat*64 + b] : 8'hEE;
        m_axis_h2c_tdata <= d; m_axis_h2c_tuser_qid <= cur.qid; m_axis_h2c_tlast <= (hbeat == hnb - 1);
        m_axis_h2c_tuser_mty <= (hbeat == hnb - 1 && rem < 64) ? 6'(64 - rem) : 6'd0;
        m_axis_h2c_tvalid <= 1'b1;
      end else m_axis_h2c_tvalid <= 1'b0;
    end
    qsts_out_vld <= 1'b0;
    if (qsq.size() != 0 && ($urandom % 3) == 0) begin
      qsts_out_vld <= 1'b1; qsts_out_op <= 8'h1; qsts_out_qid <= 13'(qsq.pop_front()); qsts_out_data <= 64'd0;
    end
  end

  // ---------------------------------------------------------------- BAR4 access (as the kernel module)
  task automatic wr64(input logic [63:0] a, input logic [63:0] d, input bit split = 0);
    @(posedge clk);
    m_axib_awaddr <= a; m_axib_awlen <= 0; m_axib_awid <= 4'd1; m_axib_awvalid <= 1'b1;
    if (!split) begin
      m_axib_wdata <= 512'(d) << (64 * a[5:3]); m_axib_wstrb <= 64'hFF << (8 * a[5:3]); m_axib_wlast <= 1'b1; m_axib_wvalid <= 1'b1;
    end
    @(posedge clk);
    m_axib_awvalid <= 1'b0;
    if (split) begin
      m_axib_wdata <= 512'(d) << (64 * a[5:3]); m_axib_wstrb <= 64'hFF << (8 * a[5:3]); m_axib_wlast <= 1'b1; m_axib_wvalid <= 1'b1;
      @(posedge clk);
    end
    m_axib_wvalid <= 1'b0;
  endtask
  task automatic rd64(input logic [63:0] a, output logic [63:0] d);
    @(posedge clk);
    m_axib_araddr <= a; m_axib_arlen <= 0; m_axib_arid <= 4'd2; m_axib_arvalid <= 1'b1;
    do @(posedge clk); while (!m_axib_arready);
    m_axib_arvalid <= 1'b0;
    do @(posedge clk); while (!m_axib_rvalid);
    d = m_axib_rdata[63:0];
  endtask
  task automatic ctl(input int idx, input logic [63:0] v, input bit split = 0); wr64(WIN + 8*idx, v, split); endtask
  function automatic logic [63:0] st_addr(input int idx); return WIN + 8*idx; endfunction
  task automatic stat(input int idx, output logic [63:0] v); rd64(st_addr(idx), v); endtask
  task automatic expect_stat(input int idx, input logic [63:0] v, input string what);
    logic [63:0] g; stat(idx, g); if (g !== v) err($sformatf("%s: stat[%0d] = %0d, expected %0d", what, idx, g, v));
  endtask
  task automatic wait_stat(input int idx, input logic [63:0] v, input string what);
    logic [63:0] g;
    for (int t = 0; t < 20000; t++) begin stat(idx, g); if (g >= v) return; end
    err($sformatf("timeout %s: stat[%0d] = %0d, waiting for %0d", what, idx, g, v));
  endtask

  localparam logic [63:0] C2H_BASE = 64'h0000_0010_0000_0000;
  localparam logic [63:0] H2C_BASE = 64'h0000_0020_0000_0000;

  task automatic cfg_ch(input bit h2c, input int ch, input int len, input int n, input logic [31:0] seed);
    int b = (h2c ? 32 : 8) + 8*ch;
    ctl(b + 0, (h2c ? H2C_BASE : C2H_BASE) + 64'(ch) * 64'h100_0000);
    ctl(b + 1, len); ctl(b + 2, n); ctl(b + 3, 4096); ctl(b + 4, n); ctl(b + 5, seed);
  endtask

  task automatic check_c2h_mem(input int ch, input int len, input int n, input logic [31:0] seed);
    int bad = 0;
    for (int i = 0; i < n; i++)
      for (int j = 0; j < len; j++) begin
        logic [63:0] a = C2H_BASE + 64'(ch) * 64'h100_0000 + 64'(i) * 4096 + j;
        if (!mem.exists(a) || mem[a] !== pbyte(ch, i, j, seed)) bad++;
      end
    if (bad) err($sformatf("c2h ch %0d len %0d: %0d bytes wrong in host memory", ch, len, bad));
  endtask

  task automatic fill_h2c(input int ch, input int len, input int n, input logic [31:0] seed);
    for (int i = 0; i < n; i++)
      for (int j = 0; j < len; j++) mem[H2C_BASE + 64'(ch) * 64'h100_0000 + 64'(i) * 4096 + j] = pbyte(ch, i, j, seed);
  endtask

  // one C2H run on the channels in mask: all packets in host memory, byte exact, counters consistent
  task automatic run_c2h(input int len, input int n, input logic [2:0] mask, input logic [31:0] seed0);
    int nch = mask[0] + mask[1] + mask[2];
    ctl(0, 64'h4);                       // clear counters
    ctl(2, mask);
    for (int ch = 0; ch < 3; ch++) if (mask[ch]) cfg_ch(0, ch, len, n, seed0 + ch);
    ctl(0, 64'h1);                       // C2H go
    for (int ch = 0; ch < 3; ch++) if (mask[ch]) wait_stat(64 + 16*ch + 2, n, $sformatf("c2h ch%0d done", ch));
    repeat (50) @(posedge clk);
    for (int ch = 0; ch < 3; ch++) if (mask[ch]) begin
      expect_stat(64 + 16*ch + 0, n, "c2h req"); expect_stat(64 + 16*ch + 1, n, "c2h pkt"); expect_stat(64 + 16*ch + 2, n, "c2h done");
      check_c2h_mem(ch, len, n, seed0 + ch);
    end
    expect_stat(2, nch * n, "c2h desc at QDMA"); expect_stat(4, nch * n, "c2h tlast at QDMA");
    expect_stat(5, nch * n * len, "c2h bytes at QDMA"); expect_stat(6, nch * n, "c2h status");
    expect_stat(7, 0, "c2h drop"); expect_stat(8, 0, "c2h error"); expect_stat(37, 0, "c2h len errors");
    $display("[%0t] C2H len %0d x %0d on mask %b done (errors so far %0d)", $time, len, n, mask, errors);
  endtask

  task automatic run_h2c(input int len, input int n, input logic [2:0] mask, input logic [31:0] seed0, input int corrupt = 0);
    int nch = mask[0] + mask[1] + mask[2];
    ctl(0, 64'h4);
    ctl(3, mask);
    for (int ch = 0; ch < 3; ch++) if (mask[ch]) begin fill_h2c(ch, len, n, seed0 + ch); cfg_ch(1, ch, len, n, seed0 + ch); end
    if (corrupt) mem[H2C_BASE + 64'h100_0000 + 64'(n/2) * 4096 + len - 1] ^= 8'h5A;   // ch1, packet n/2, last byte
    ctl(0, 64'h2);                       // H2C go
    for (int ch = 0; ch < 3; ch++) if (mask[ch]) wait_stat(64 + 16*ch + 9, n, $sformatf("h2c ch%0d done", ch));
    repeat (50) @(posedge clk);
    for (int ch = 0; ch < 3; ch++) if (mask[ch]) begin
      expect_stat(64 + 16*ch + 4, n, "h2c req"); expect_stat(64 + 16*ch + 5, n, "h2c pkt");
      expect_stat(64 + 16*ch + 7, (corrupt && ch == 1) ? 1 : 0, "h2c bad beats");
      expect_stat(64 + 16*ch + 8, 0, "h2c keep/last errors"); expect_stat(64 + 16*ch + 12, n, "h2c tlast");
    end
    expect_stat(19, nch * n, "h2c desc at QDMA"); expect_stat(22, nch * n * len, "h2c bytes at QDMA");
    expect_stat(26, nch * n, "qsts op 1");
    if (corrupt) begin
      logic [63:0] f0; stat(64 + 16 + 10, f0);
      if (f0[63] !== 1'b1 || f0[31:0] !== n/2 || f0[49] !== 1'b1) err($sformatf("h2c first mismatch record %h", f0));
    end
    $display("[%0t] H2C len %0d x %0d on mask %b%s done (errors so far %0d)", $time, len, n, mask, corrupt ? " (one byte corrupted)" : "", errors);
  endtask

  initial begin
    logic [63:0] v;
    #1000000;   // 1 ms
    $display("TB TIMEOUT errors=%0d", errors);
    $finish;
  end

  initial begin
    logic [63:0] v;
    repeat (20) @(posedge clk);
    rstn = 1'b1;
    repeat (20) @(posedge clk);
    stat(0, v); if (v !== 64'h51DA_0C0F_FEE0_0002) err($sformatf("magic %h", v));
    rd64(64'h0, v); if (v !== 64'h51DA_0C0F_FEE0_0001) err($sformatf("sink magic %h", v));
    // control register write/readback, aligned and split AW/W, 32-bit half write
    ctl(10, 64'h1122_3344_5566_7788); rd64(WIN + 64'h800 + 8*10, v); if (v !== 64'h1122_3344_5566_7788) err($sformatf("ctl readback %h", v));
    ctl(11, 64'hA5A5_0000_FFFF_1234, 1); rd64(WIN + 64'h800 + 8*11, v); if (v !== 64'hA5A5_0000_FFFF_1234) err($sformatf("ctl split readback %h", v));
    // prefetch tags as the driver: (1<<31) | ((qid-256) << 8) | tag, then 0
    for (int q = 256; q < 288; q++) begin ctl(1, (64'h1 << 31) | (64'(q - 256) << 8) | 64'(tag_of(q))); ctl(1, 0); end

    run_c2h(64,   24, 3'b001, 32'h1000_0000);
    run_c2h(4096, 10, 3'b001, 32'h2000_0000);
    run_c2h(64,   20, 3'b111, 32'h3000_0000);
    run_c2h(4096, 9,  3'b111, 32'h4000_0000);
    run_c2h(100,  7,  3'b111, 32'h5000_0000);
    run_h2c(64,   20, 3'b111, 32'h6000_0000);
    run_h2c(4096, 9,  3'b111, 32'h7000_0000);
    run_h2c(100,  7,  3'b111, 32'h8000_0000);
    run_h2c(4096, 6,  3'b010, 32'h9000_0000, 1);
    // concurrent C2H + H2C on all channels, back-pressure off (throughput shape)
    rnd_c2h_cmd = 10; rnd_c2h_data = 10; rnd_h2c_cmd = 10; rnd_h2c_data = 10; rnd_sts = 10;
    ctl(0, 64'h4); ctl(2, 3'b111); ctl(3, 3'b111);
    for (int ch = 0; ch < 3; ch++) begin
      cfg_ch(0, ch, 4096, 16, 32'hA000_0000 + ch); fill_h2c(ch, 4096, 16, 32'hB000_0000 + ch); cfg_ch(1, ch, 4096, 16, 32'hB000_0000 + ch);
    end
    ctl(0, 64'h3);
    for (int ch = 0; ch < 3; ch++) begin wait_stat(64 + 16*ch + 2, 16, "conc c2h"); wait_stat(64 + 16*ch + 9, 16, "conc h2c"); end
    for (int ch = 0; ch < 3; ch++) begin check_c2h_mem(ch, 4096, 16, 32'hA000_0000 + ch); expect_stat(64 + 16*ch + 7, 0, "conc h2c bad"); end
    begin
      logic [63:0] t0, t1, by;
      stat(15, t0); stat(16, t1); stat(5, by);
      $display("C2H unthrottled model: %0d bytes in %0d cycles = %0.2f B/cycle (wrapper ceiling, model QDMA always ready)", by, t1 - t0, real'(by) / real'(t1 - t0));
    end
    $display("C2H packets complete before their descriptor reached the QDMA: %0d", pkt_before_desc);
    $display("TB %s errors=%0d", errors == 0 ? "PASS" : "FAIL", errors);
    $finish;
  end
endmodule
