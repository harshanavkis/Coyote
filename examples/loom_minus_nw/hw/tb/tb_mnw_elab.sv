`timescale 1ns / 1ps
import lynxTypes::*;

// Minimal elaboration harness for the minus-nw top: declares the interfaces
// the shell would provide and instantiates the design in place, the same way
// design_user_logic_c0_0 does for examples/loom.
module tb_mnw_elab;

logic aclk = 0;
logic aresetn = 0;
always #2 aclk = ~aclk;

AXI4L axi_ctrl (.aclk(aclk), .aresetn(aresetn));
metaIntf #(.STYPE(irq_not_t)) notify (.*);
metaIntf #(.STYPE(req_t)) sq_rd (.*);
metaIntf #(.STYPE(req_t)) sq_wr (.*);
metaIntf #(.STYPE(ack_t)) cq_rd (.*);
metaIntf #(.STYPE(ack_t)) cq_wr (.*);
AXI4SR axis_host_recv [N_STRM_AXI] (.*);
AXI4SR axis_host_send [N_STRM_AXI] (.*);

// Shell side: accept everything, offer nothing. Enough to prove the design
// holds together; real stimulus lives in the loopback TB.
initial begin
    axi_ctrl.awvalid = 0; axi_ctrl.wvalid = 0;
    axi_ctrl.arvalid = 0; axi_ctrl.bready = 1; axi_ctrl.rready = 1;
end
assign notify.ready = 1'b1;
assign sq_rd.ready = 1'b1;
assign sq_wr.ready = 1'b1;
assign axis_host_send[0].tready = 1'b1;
assign axis_host_recv[0].tvalid = 1'b0;
assign axis_host_recv[0].tdata  = '0;
assign axis_host_recv[0].tkeep  = '0;
assign axis_host_recv[0].tlast  = 1'b0;
assign axis_host_recv[0].tid    = '0;
assign cq_rd.valid = 1'b0;
assign cq_rd.data  = '0;
assign cq_wr.valid = 1'b0;
assign cq_wr.data  = '0;

`include "vfpga_top.svh"

initial begin
    #100 aresetn = 1;
    #500 $display("MNW ELABORATION OK");
    $finish;
end

endmodule
