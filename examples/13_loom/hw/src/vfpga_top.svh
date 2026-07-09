/**
 * Loom — two mechanisms sharing the sq/stream datapath (size-selected in SW).
 *   DMA (large): loom_engine copy engine, sq_rd->FIFO->sq_wr (2 DMAs, host<->host).
 *   APERTURE (small): reg_intf traps AXI-Lite rd/wr and performs it against the
 *     PEER thread's memory, 8 B per access. write=posted sq_wr; read=blocking sq_rd.
 * Arbiter shares sq_rd/sq_wr + host streams (one at a time, aperture priority).
 */

// ---------------- Register interface + aperture controller ----------------
logic                  d_start, d_clear, d_busy, d_done;
logic [LEN_BITS-1:0]   d_len;
logic [PID_BITS-1:0]   d_src_pid, d_dst_pid;
logic [VADDR_BITS-1:0] d_src_addr, d_dst_addr;
logic [3:0]            d_src_strm, d_dst_strm;

logic                  ap_busy, ap_rd_req, ap_wr_req, ap_rd_ack, ap_wr_ack, ap_rd_cq;
logic [PID_BITS-1:0]   ap_pid;
logic [VADDR_BITS-1:0] ap_addr;
logic [3:0]            ap_stream;
logic [AXI_DATA_BITS-1:0] ap_send_tdata, ap_recv_tdata;
logic                  ap_send_tvalid, ap_send_tlast, ap_send_tready, ap_recv_tvalid, ap_recv_tready;

loom_reg_intf inst_reg (
    .aclk(aclk), .aresetn(aresetn), .axi_ctrl(axi_ctrl),
    .start(d_start), .clear_start(d_clear), .busy(d_busy), .done_valid(d_done),
    .len(d_len),
    .src_pid(d_src_pid), .src_addr(d_src_addr), .src_stream(d_src_strm),
    .dst_pid(d_dst_pid), .dst_addr(d_dst_addr), .dst_stream(d_dst_strm),
    .ap_busy(ap_busy), .ap_rd_req(ap_rd_req), .ap_wr_req(ap_wr_req),
    .ap_pid(ap_pid), .ap_addr(ap_addr), .ap_stream(ap_stream),
    .ap_rd_ack(ap_rd_ack), .ap_wr_ack(ap_wr_ack), .ap_rd_cq(ap_rd_cq),
    .ap_send_tdata(ap_send_tdata), .ap_send_tvalid(ap_send_tvalid), .ap_send_tlast(ap_send_tlast),
    .ap_send_tready(ap_send_tready),
    .ap_recv_tvalid(ap_recv_tvalid), .ap_recv_tdata(ap_recv_tdata), .ap_recv_tready(ap_recv_tready)
);

// ---------------- Shared FIFO (DMA path) ----------------
logic [AXI_DATA_BITS-1:0]   fifo_in_tdata,  fifo_out_tdata;
logic [AXI_DATA_BITS/8-1:0] fifo_in_tkeep,  fifo_out_tkeep;
logic fifo_in_tlast,  fifo_in_tvalid,  fifo_in_tready;
logic fifo_out_tlast, fifo_out_tvalid, fifo_out_tready;

axis_data_fifo_loom inst_fifo (
    .s_axis_aclk(aclk), .s_axis_aresetn(aresetn),
    .s_axis_tdata(fifo_in_tdata),   .s_axis_tkeep(fifo_in_tkeep),
    .s_axis_tlast(fifo_in_tlast),   .s_axis_tvalid(fifo_in_tvalid),
    .s_axis_tready(fifo_in_tready),
    .m_axis_tdata(fifo_out_tdata),  .m_axis_tkeep(fifo_out_tkeep),
    .m_axis_tlast(fifo_out_tlast),  .m_axis_tvalid(fifo_out_tvalid),
    .m_axis_tready(fifo_out_tready)
);
logic dma_rd_last, dma_wr_last;
assign dma_rd_last = fifo_in_tvalid  && fifo_in_tready  && fifo_in_tlast;
assign dma_wr_last = fifo_out_tvalid && fifo_out_tready && fifo_out_tlast;

// ---------------- DMA copy engine ----------------
logic                  ed_rd_req, ed_rd_ack, ed_wr_req, ed_wr_ack, ed_rd_active, ed_wr_active;
logic [PID_BITS-1:0]   ed_rd_pid, ed_wr_pid;
logic [VADDR_BITS-1:0] ed_rd_addr, ed_wr_addr;
logic [LEN_BITS-1:0]   ed_rd_len, ed_wr_len;
logic [3:0]            ed_rd_strm, ed_wr_strm;
logic                  ed_rd_last, ed_wr_last, ed_rd_cq, ed_wr_cq;

loom_engine inst_dma (
    .aclk(aclk), .aresetn(aresetn),
    .start(d_start), .clear_start(d_clear), .busy(d_busy), .done_valid(d_done),
    .len(d_len),
    .src_pid(d_src_pid), .src_addr(d_src_addr), .src_stream(d_src_strm),
    .dst_pid(d_dst_pid), .dst_addr(d_dst_addr), .dst_stream(d_dst_strm),
    .rd_req(ed_rd_req), .rd_pid(ed_rd_pid), .rd_addr(ed_rd_addr), .rd_len(ed_rd_len), .rd_stream(ed_rd_strm),
    .rd_ack(ed_rd_ack), .rd_last(ed_rd_last), .rd_cq(ed_rd_cq),
    .wr_req(ed_wr_req), .wr_pid(ed_wr_pid), .wr_addr(ed_wr_addr), .wr_len(ed_wr_len), .wr_stream(ed_wr_strm),
    .wr_ack(ed_wr_ack), .wr_last(ed_wr_last), .wr_cq(ed_wr_cq),
    .rd_active(ed_rd_active), .wr_active(ed_wr_active)
);

// ---------------- Arbiter (aperture priority) ----------------
localparam logic SEL_DMA = 1'b0, SEL_AP = 1'b1;
logic arb_locked, arb_sel;
always_ff @(posedge aclk) begin
    if (!aresetn) begin arb_locked <= 1'b0; arb_sel <= SEL_DMA; end
    else if (!arb_locked) begin
        if (ap_busy)     begin arb_sel <= SEL_AP;  arb_locked <= 1'b1; end
        else if (d_busy) begin arb_sel <= SEL_DMA; arb_locked <= 1'b1; end
    end else if ((arb_sel==SEL_AP && !ap_busy) || (arb_sel==SEL_DMA && !d_busy))
        arb_locked <= 1'b0;
end
wire sel_dma = arb_locked && (arb_sel == SEL_DMA);
wire sel_ap  = arb_locked && (arb_sel == SEL_AP);

localparam integer AP_BYTES = 8;   // one 64-bit aperture access

// ---------------- Shared sq (winner) ----------------
always_comb begin
    sq_rd.data = 0; sq_rd.data.opcode = LOCAL_READ;  sq_rd.data.strm = STRM_HOST; sq_rd.data.last = 1'b1;
    sq_wr.data = 0; sq_wr.data.opcode = LOCAL_WRITE; sq_wr.data.strm = STRM_HOST; sq_wr.data.last = 1'b1;
    if (sel_ap) begin
        sq_rd.data.dest = ap_stream; sq_rd.data.pid = ap_pid; sq_rd.data.vaddr = ap_addr; sq_rd.data.len = AP_BYTES;
        sq_rd.valid = ap_rd_req;
        sq_wr.data.dest = ap_stream; sq_wr.data.pid = ap_pid; sq_wr.data.vaddr = ap_addr; sq_wr.data.len = AP_BYTES;
        sq_wr.valid = ap_wr_req;
    end else begin
        sq_rd.data.dest = ed_rd_strm; sq_rd.data.pid = ed_rd_pid; sq_rd.data.vaddr = ed_rd_addr; sq_rd.data.len = ed_rd_len;
        sq_rd.valid = sel_dma ? ed_rd_req : 1'b0;
        sq_wr.data.dest = ed_wr_strm; sq_wr.data.pid = ed_wr_pid; sq_wr.data.vaddr = ed_wr_addr; sq_wr.data.len = ed_wr_len;
        sq_wr.valid = sel_dma ? ed_wr_req : 1'b0;
    end
    cq_rd.ready = 1'b1;
    cq_wr.ready = 1'b1;
end

wire w_rd_ack = sq_rd.valid && sq_rd.ready;
wire w_wr_ack = sq_wr.valid && sq_wr.ready;

always_comb begin
    ed_rd_ack = sel_dma ? w_rd_ack : 1'b0;  ed_rd_last = sel_dma ? dma_rd_last : 1'b0;  ed_rd_cq = sel_dma ? cq_rd.valid : 1'b0;
    ed_wr_ack = sel_dma ? w_wr_ack : 1'b0;  ed_wr_last = sel_dma ? dma_wr_last : 1'b0;  ed_wr_cq = sel_dma ? cq_wr.valid : 1'b0;
    ap_rd_ack = sel_ap ? w_rd_ack : 1'b0;   ap_wr_ack = sel_ap ? w_wr_ack : 1'b0;       ap_rd_cq  = sel_ap ? cq_rd.valid : 1'b0;
end

// ---------------- Shared host streams ----------------
logic [N_STRM_AXI-1:0][AXI_DATA_BITS-1:0]   recv_tdata;
logic [N_STRM_AXI-1:0]                       recv_tvalid, recv_tlast, send_tready;
logic [N_STRM_AXI-1:0][AXI_DATA_BITS/8-1:0]  recv_tkeep;

genvar gi;
generate
    for (gi = 0; gi < N_STRM_AXI; gi++) begin : gen_stream
        assign recv_tdata[gi]  = axis_host_recv[gi].tdata;
        assign recv_tvalid[gi] = axis_host_recv[gi].tvalid;
        assign recv_tkeep[gi]  = axis_host_recv[gi].tkeep;
        assign recv_tlast[gi]  = axis_host_recv[gi].tlast;
        assign send_tready[gi] = axis_host_send[gi].tready;

        // recv[gi].tready: DMA read -> FIFO, aperture read -> ap
        assign axis_host_recv[gi].tready =
            (sel_dma && ed_rd_active && (gi==ed_rd_strm)) ? fifo_in_tready :
            (sel_ap  && (gi==ap_stream))                  ? ap_recv_tready : 1'b0;

        // send[gi]: DMA write <- FIFO, aperture write <- ap beat
        assign axis_host_send[gi].tvalid =
            (sel_dma && ed_wr_active && (gi==ed_wr_strm)) ? fifo_out_tvalid :
            (sel_ap  && (gi==ap_stream))                  ? ap_send_tvalid : 1'b0;
        assign axis_host_send[gi].tdata  = sel_ap ? ap_send_tdata : fifo_out_tdata;
        assign axis_host_send[gi].tkeep  = sel_ap ? {{(AXI_DATA_BITS/8-AP_BYTES){1'b0}}, {AP_BYTES{1'b1}}} : fifo_out_tkeep;
        assign axis_host_send[gi].tlast  = sel_ap ? ap_send_tlast : fifo_out_tlast;
        assign axis_host_send[gi].tid    = '0;
    end
endgenerate

// FIFO endpoints (DMA path)
always_comb begin
    fifo_in_tvalid  = (sel_dma && ed_rd_active) ? recv_tvalid[ed_rd_strm] : 1'b0;
    fifo_in_tdata   = recv_tdata[ed_rd_strm];
    fifo_in_tkeep   = recv_tkeep[ed_rd_strm];
    fifo_in_tlast   = recv_tlast[ed_rd_strm];
    fifo_out_tready = (sel_dma && ed_wr_active) ? send_tready[ed_wr_strm] : 1'b0;
end

// Aperture stream endpoints
always_comb begin
    ap_recv_tvalid = (sel_ap) ? recv_tvalid[ap_stream] : 1'b0;
    ap_recv_tdata  = recv_tdata[ap_stream];
    ap_send_tready = (sel_ap) ? send_tready[ap_stream] : 1'b0;
end

always_comb notify.tie_off_m();
