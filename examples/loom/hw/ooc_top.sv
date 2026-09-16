`timescale 1ns / 1ps
import lynxTypes::*;

/**
 * ooc_top - synthesis-only wrapper for ./ooc_synth.sh top.
 *
 * design_user_logic_c0_0 takes SystemVerilog interfaces as ports and
 * cannot be a synthesis top by itself (the shell instantiates it with
 * typed interfaces). This declares the interfaces the way the shell does
 * and exposes every signal as a plain port, so the vFPGA can be
 * synthesized out of context - the check for vfpga_top.svh's glue that
 * ./ooc_synth.sh <module> does for one module. Never part of a build.
 */
module ooc_top (
    input  logic aclk,
    input  logic aresetn,
    // axi_ctrl (AXI4-Lite slave)
    input  logic [AXI_ADDR_BITS-1:0]   axi_ctrl_awaddr,
    input  logic                       axi_ctrl_awvalid,
    output logic                       axi_ctrl_awready,
    input  logic [AXIL_DATA_BITS-1:0]  axi_ctrl_wdata,
    input  logic [AXIL_DATA_BITS/8-1:0] axi_ctrl_wstrb,
    input  logic                       axi_ctrl_wvalid,
    output logic                       axi_ctrl_wready,
    output logic [1:0]                 axi_ctrl_bresp,
    output logic                       axi_ctrl_bvalid,
    input  logic                       axi_ctrl_bready,
    input  logic [AXI_ADDR_BITS-1:0]   axi_ctrl_araddr,
    input  logic                       axi_ctrl_arvalid,
    output logic                       axi_ctrl_arready,
    output logic [AXIL_DATA_BITS-1:0]  axi_ctrl_rdata,
    output logic [1:0]                 axi_ctrl_rresp,
    output logic                       axi_ctrl_rvalid,
    input  logic                       axi_ctrl_rready,
    // request / completion queues
    output req_t  sq_rd_data, input logic sq_rd_ready, output logic sq_rd_valid,
    output req_t  sq_wr_data, input logic sq_wr_ready, output logic sq_wr_valid,
    input  ack_t  cq_rd_data, output logic cq_rd_ready, input logic cq_rd_valid,
    input  ack_t  cq_wr_data, output logic cq_wr_ready, input logic cq_wr_valid,
    input  req_t  rq_rd_data, output logic rq_rd_ready, input logic rq_rd_valid,
    input  req_t  rq_wr_data, output logic rq_wr_ready, input logic rq_wr_valid,
    output irq_not_t notify_data, input logic notify_ready, output logic notify_valid,
    // host streams
    input  logic [N_STRM_AXI-1:0][AXI_DATA_BITS-1:0]   host_recv_tdata,
    input  logic [N_STRM_AXI-1:0][AXI_DATA_BITS/8-1:0] host_recv_tkeep,
    input  logic [N_STRM_AXI-1:0][PID_BITS-1:0]        host_recv_tid,
    input  logic [N_STRM_AXI-1:0]                      host_recv_tvalid,
    input  logic [N_STRM_AXI-1:0]                      host_recv_tlast,
    output logic [N_STRM_AXI-1:0]                      host_recv_tready,
    output logic [N_STRM_AXI-1:0][AXI_DATA_BITS-1:0]   host_send_tdata,
    output logic [N_STRM_AXI-1:0][AXI_DATA_BITS/8-1:0] host_send_tkeep,
    output logic [N_STRM_AXI-1:0][PID_BITS-1:0]        host_send_tid,
    output logic [N_STRM_AXI-1:0]                      host_send_tvalid,
    output logic [N_STRM_AXI-1:0]                      host_send_tlast,
    input  logic [N_STRM_AXI-1:0]                      host_send_tready,
    // rdma streams
    input  logic [N_RDMA_AXI-1:0][AXI_DATA_BITS-1:0]   rreq_recv_tdata,
    input  logic [N_RDMA_AXI-1:0][AXI_DATA_BITS/8-1:0] rreq_recv_tkeep,
    input  logic [N_RDMA_AXI-1:0][PID_BITS-1:0]        rreq_recv_tid,
    input  logic [N_RDMA_AXI-1:0]                      rreq_recv_tvalid,
    input  logic [N_RDMA_AXI-1:0]                      rreq_recv_tlast,
    output logic [N_RDMA_AXI-1:0]                      rreq_recv_tready,
    output logic [N_RDMA_AXI-1:0][AXI_DATA_BITS-1:0]   rreq_send_tdata,
    output logic [N_RDMA_AXI-1:0][AXI_DATA_BITS/8-1:0] rreq_send_tkeep,
    output logic [N_RDMA_AXI-1:0][PID_BITS-1:0]        rreq_send_tid,
    output logic [N_RDMA_AXI-1:0]                      rreq_send_tvalid,
    output logic [N_RDMA_AXI-1:0]                      rreq_send_tlast,
    input  logic [N_RDMA_AXI-1:0]                      rreq_send_tready,
    input  logic [N_RDMA_AXI-1:0][AXI_DATA_BITS-1:0]   rrsp_recv_tdata,
    input  logic [N_RDMA_AXI-1:0][AXI_DATA_BITS/8-1:0] rrsp_recv_tkeep,
    input  logic [N_RDMA_AXI-1:0][PID_BITS-1:0]        rrsp_recv_tid,
    input  logic [N_RDMA_AXI-1:0]                      rrsp_recv_tvalid,
    input  logic [N_RDMA_AXI-1:0]                      rrsp_recv_tlast,
    output logic [N_RDMA_AXI-1:0]                      rrsp_recv_tready,
    output logic [N_RDMA_AXI-1:0][AXI_DATA_BITS-1:0]   rrsp_send_tdata,
    output logic [N_RDMA_AXI-1:0][AXI_DATA_BITS/8-1:0] rrsp_send_tkeep,
    output logic [N_RDMA_AXI-1:0][PID_BITS-1:0]        rrsp_send_tid,
    output logic [N_RDMA_AXI-1:0]                      rrsp_send_tvalid,
    output logic [N_RDMA_AXI-1:0]                      rrsp_send_tlast,
    input  logic [N_RDMA_AXI-1:0]                      rrsp_send_tready
);

AXI4L axi_ctrl (.aclk(aclk), .aresetn(aresetn));
metaIntf #(.STYPE(irq_not_t)) notify (.*);
metaIntf #(.STYPE(req_t)) sq_rd (.*);
metaIntf #(.STYPE(req_t)) sq_wr (.*);
metaIntf #(.STYPE(ack_t)) cq_rd (.*);
metaIntf #(.STYPE(ack_t)) cq_wr (.*);
metaIntf #(.STYPE(req_t)) rq_rd (.*);
metaIntf #(.STYPE(req_t)) rq_wr (.*);
AXI4SR axis_host_recv [N_STRM_AXI] (.*);
AXI4SR axis_host_send [N_STRM_AXI] (.*);
AXI4SR axis_rreq_recv [N_RDMA_AXI] (.*);
AXI4SR axis_rreq_send [N_RDMA_AXI] (.*);
AXI4SR axis_rrsp_recv [N_RDMA_AXI] (.*);
AXI4SR axis_rrsp_send [N_RDMA_AXI] (.*);

design_user_logic_c0_0 inst_dut (
    .axi_ctrl(axi_ctrl), .notify(notify),
    .sq_rd(sq_rd), .sq_wr(sq_wr), .cq_rd(cq_rd), .cq_wr(cq_wr),
    .rq_rd(rq_rd), .rq_wr(rq_wr),
    .axis_host_recv(axis_host_recv), .axis_host_send(axis_host_send),
    .axis_rreq_recv(axis_rreq_recv), .axis_rreq_send(axis_rreq_send),
    .axis_rrsp_recv(axis_rrsp_recv), .axis_rrsp_send(axis_rrsp_send),
    .aclk(aclk), .aresetn(aresetn)
);

always_comb begin
    axi_ctrl.awaddr = axi_ctrl_awaddr; axi_ctrl.awvalid = axi_ctrl_awvalid; axi_ctrl_awready = axi_ctrl.awready;
    axi_ctrl.awprot = '0; axi_ctrl.awqos = '0; axi_ctrl.awregion = '0;
    axi_ctrl.wdata = axi_ctrl_wdata; axi_ctrl.wstrb = axi_ctrl_wstrb; axi_ctrl.wvalid = axi_ctrl_wvalid; axi_ctrl_wready = axi_ctrl.wready;
    axi_ctrl_bresp = axi_ctrl.bresp; axi_ctrl_bvalid = axi_ctrl.bvalid; axi_ctrl.bready = axi_ctrl_bready;
    axi_ctrl.araddr = axi_ctrl_araddr; axi_ctrl.arvalid = axi_ctrl_arvalid; axi_ctrl_arready = axi_ctrl.arready;
    axi_ctrl.arprot = '0; axi_ctrl.arqos = '0; axi_ctrl.arregion = '0;
    axi_ctrl_rdata = axi_ctrl.rdata; axi_ctrl_rresp = axi_ctrl.rresp; axi_ctrl_rvalid = axi_ctrl.rvalid; axi_ctrl.rready = axi_ctrl_rready;

    sq_rd_data = sq_rd.data; sq_rd_valid = sq_rd.valid; sq_rd.ready = sq_rd_ready;
    sq_wr_data = sq_wr.data; sq_wr_valid = sq_wr.valid; sq_wr.ready = sq_wr_ready;
    cq_rd.data = cq_rd_data; cq_rd.valid = cq_rd_valid; cq_rd_ready = cq_rd.ready;
    cq_wr.data = cq_wr_data; cq_wr.valid = cq_wr_valid; cq_wr_ready = cq_wr.ready;
    rq_rd.data = rq_rd_data; rq_rd.valid = rq_rd_valid; rq_rd_ready = rq_rd.ready;
    rq_wr.data = rq_wr_data; rq_wr.valid = rq_wr_valid; rq_wr_ready = rq_wr.ready;
    notify_data = notify.data; notify_valid = notify.valid; notify.ready = notify_ready;
end

for (genvar i = 0; i < N_STRM_AXI; i++) begin : g_host
    always_comb begin
        axis_host_recv[i].tdata = host_recv_tdata[i]; axis_host_recv[i].tkeep = host_recv_tkeep[i];
        axis_host_recv[i].tid = host_recv_tid[i]; axis_host_recv[i].tvalid = host_recv_tvalid[i];
        axis_host_recv[i].tlast = host_recv_tlast[i]; host_recv_tready[i] = axis_host_recv[i].tready;
        host_send_tdata[i] = axis_host_send[i].tdata; host_send_tkeep[i] = axis_host_send[i].tkeep;
        host_send_tid[i] = axis_host_send[i].tid; host_send_tvalid[i] = axis_host_send[i].tvalid;
        host_send_tlast[i] = axis_host_send[i].tlast; axis_host_send[i].tready = host_send_tready[i];
    end
end
for (genvar i = 0; i < N_RDMA_AXI; i++) begin : g_rdma
    always_comb begin
        axis_rreq_recv[i].tdata = rreq_recv_tdata[i]; axis_rreq_recv[i].tkeep = rreq_recv_tkeep[i];
        axis_rreq_recv[i].tid = rreq_recv_tid[i]; axis_rreq_recv[i].tvalid = rreq_recv_tvalid[i];
        axis_rreq_recv[i].tlast = rreq_recv_tlast[i]; rreq_recv_tready[i] = axis_rreq_recv[i].tready;
        rreq_send_tdata[i] = axis_rreq_send[i].tdata; rreq_send_tkeep[i] = axis_rreq_send[i].tkeep;
        rreq_send_tid[i] = axis_rreq_send[i].tid; rreq_send_tvalid[i] = axis_rreq_send[i].tvalid;
        rreq_send_tlast[i] = axis_rreq_send[i].tlast; axis_rreq_send[i].tready = rreq_send_tready[i];
        axis_rrsp_recv[i].tdata = rrsp_recv_tdata[i]; axis_rrsp_recv[i].tkeep = rrsp_recv_tkeep[i];
        axis_rrsp_recv[i].tid = rrsp_recv_tid[i]; axis_rrsp_recv[i].tvalid = rrsp_recv_tvalid[i];
        axis_rrsp_recv[i].tlast = rrsp_recv_tlast[i]; rrsp_recv_tready[i] = axis_rrsp_recv[i].tready;
        rrsp_send_tdata[i] = axis_rrsp_send[i].tdata; rrsp_send_tkeep[i] = axis_rrsp_send[i].tkeep;
        rrsp_send_tid[i] = axis_rrsp_send[i].tid; rrsp_send_tvalid[i] = axis_rrsp_send[i].tvalid;
        rrsp_send_tlast[i] = axis_rrsp_send[i].tlast; axis_rrsp_send[i].tready = rrsp_send_tready[i];
    end
end

endmodule
