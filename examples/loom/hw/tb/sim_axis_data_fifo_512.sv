`timescale 1ns / 1ps
import lynxTypes::*;

/**
 * Simulation stand-in for the Xilinx AXI4-Stream data FIFO. The IP is not
 * available to xsim, and tb_loom_top elaborates vfpga_top, which now puts one
 * in front of loom_rx.
 *
 * Depth matches what the real IP is generated with for this configuration:
 * common_infrastructure.tcl sets FIFO_DEPTH to n_outs * (pmtu/64), and
 * base.tcl gives n_outs 8, pmtu 4096 -> 512 beats, eight packets' worth.
 *
 * Why it exists at all: loom_rx holds no buffer, so a stall on the host write
 * path reached axis_rrsp_recv.tready in the same cycle. Backpressuring an RC
 * receiver that long delays its ACKs past the sender's retransmit timer, and
 * a retransmission perturbs the payload beat stream that loom_rx counts
 * position from. This decouples the two.
 */
module axis_data_fifo_512 #(
    parameter int DEPTH = 512
) (
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

typedef struct packed {
    logic [511:0] tdata;
    logic [63:0]  tkeep;
    logic         tlast;
} beat_t;

beat_t mem [DEPTH-1:0];
int unsigned wr_ptr, rd_ptr, count;

// Registered tready, as the IP has: it reflects occupancy, never the output
// side's readiness in the same cycle. That decoupling is the entire point.
assign s_axis_tready = (count < DEPTH);

assign m_axis_tvalid = (count != 0);
assign m_axis_tdata  = mem[rd_ptr].tdata;
assign m_axis_tkeep  = mem[rd_ptr].tkeep;
assign m_axis_tlast  = mem[rd_ptr].tlast;

wire push = s_axis_tvalid && s_axis_tready;
wire pop  = m_axis_tvalid && m_axis_tready;

always_ff @(posedge s_axis_aclk) begin
    if (!s_axis_aresetn) begin
        wr_ptr <= 0;
        rd_ptr <= 0;
        count  <= 0;
    end else begin
        if (push) begin
            mem[wr_ptr] <= '{s_axis_tdata, s_axis_tkeep, s_axis_tlast};
            wr_ptr <= (wr_ptr == DEPTH-1) ? 0 : wr_ptr + 1;
        end
        if (pop)
            rd_ptr <= (rd_ptr == DEPTH-1) ? 0 : rd_ptr + 1;

        case ({push, pop})
            2'b10: count <= count + 1;
            2'b01: count <= count - 1;
            default: ;
        endcase
    end
end

endmodule
