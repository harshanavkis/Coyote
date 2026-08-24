`timescale 1ns / 1ps
import lynxTypes::*;

/**
 * Simulation stand-in for the Xilinx AXI4-Stream register slice that
 * axisr_reg wraps. The IP is not available to xsim, and tb_loom_top
 * elaborates vfpga_top, which instantiates two of them.
 *
 * A one-deep skid buffer: full AXI-Stream throughput, one cycle of latency,
 * and tready registered so it does not run combinationally from output to
 * input - which is the whole reason the slices are there.
 */
module axisr_register_slice_512 (
    input  logic         aclk,
    input  logic         aresetn,

    input  logic         s_axis_tvalid,
    output logic         s_axis_tready,
    input  logic [511:0] s_axis_tdata,
    input  logic [63:0]  s_axis_tkeep,
    input  logic [5:0]   s_axis_tid,
    input  logic         s_axis_tlast,

    output logic         m_axis_tvalid,
    input  logic         m_axis_tready,
    output logic [511:0] m_axis_tdata,
    output logic [63:0]  m_axis_tkeep,
    output logic [5:0]   m_axis_tid,
    output logic         m_axis_tlast
);

typedef struct packed {
    logic [511:0] tdata;
    logic [63:0]  tkeep;
    logic [5:0]   tid;
    logic         tlast;
} beat_t;

beat_t main_q, skid_q;
logic  main_v, skid_v;

wire beat_t s_beat = '{s_axis_tdata, s_axis_tkeep, s_axis_tid, s_axis_tlast};

assign s_axis_tready = !skid_v;

assign m_axis_tvalid = main_v;
assign m_axis_tdata  = main_q.tdata;
assign m_axis_tkeep  = main_q.tkeep;
assign m_axis_tid    = main_q.tid;
assign m_axis_tlast  = main_q.tlast;

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        main_v <= 1'b0;
        skid_v <= 1'b0;
    end else begin
        if (!main_v || m_axis_tready) begin
            // the output stage is free: take the skid entry first, then the input
            if (skid_v) begin
                main_q <= skid_q;  main_v <= 1'b1;  skid_v <= 1'b0;
            end else begin
                main_q <= s_beat;  main_v <= s_axis_tvalid;
            end
        end else if (s_axis_tvalid && s_axis_tready) begin
            // output stalled and a beat arrived: hold it
            skid_q <= s_beat;  skid_v <= 1'b1;
        end
    end
end

endmodule
