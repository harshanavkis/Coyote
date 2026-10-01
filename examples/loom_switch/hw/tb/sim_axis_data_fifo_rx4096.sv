`timescale 1ns / 1ps

// Simulation stand-in for the 4096-beat ingress FIFO IP (init_ip.tcl): the
// behavioural 512-beat model (examples/loom/hw/tb) at depth 4096.
module axis_data_fifo_rx4096 (
    input  logic         s_axis_aclk,
    input  logic         s_axis_aresetn,
    input  logic [511:0] s_axis_tdata,
    input  logic [63:0]  s_axis_tkeep,
    input  logic         s_axis_tvalid,
    output logic         s_axis_tready,
    input  logic         s_axis_tlast,
    output logic [511:0] m_axis_tdata,
    output logic [63:0]  m_axis_tkeep,
    output logic         m_axis_tvalid,
    input  logic         m_axis_tready,
    output logic         m_axis_tlast
);
axis_data_fifo_512 #(.DEPTH(4096)) inst (.*);
endmodule
