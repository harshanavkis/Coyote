`timescale 1ns / 1ps

/**
 * Simulation stand-ins for Coyote's meta register-slice IPs.
 *
 * meta_reg picks one of axis_register_slice_meta_<W> by DATA_BITS; the IP is
 * not available to xsim, and tb_user_req_mux elaborates user_req_mux, which
 * uses meta_reg for req_t (128 b here) and dreq_t (256 b). One skid buffer
 * serves both: full AXI-stream semantics, one cycle of latency, no bubbles -
 * the same thing the IP does.
 */
module axis_reg_slice_skid #(
    parameter int W = 128
) (
    input  logic          aclk,
    input  logic          aresetn,
    input  logic          s_axis_tvalid,
    output logic          s_axis_tready,
    input  logic [W-1:0]  s_axis_tdata,
    output logic          m_axis_tvalid,
    input  logic          m_axis_tready,
    output logic [W-1:0]  m_axis_tdata
);
logic         full;
logic [W-1:0] data;

assign s_axis_tready = !full || m_axis_tready;
assign m_axis_tvalid = full;
assign m_axis_tdata  = data;

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        full <= 1'b0;
        data <= '0;
    end else begin
        if (s_axis_tvalid && s_axis_tready) begin
            full <= 1'b1;
            data <= s_axis_tdata;
        end else if (m_axis_tvalid && m_axis_tready) begin
            full <= 1'b0;
        end
    end
end
endmodule

module axis_register_slice_meta_128 (
    input  logic          aclk,
    input  logic          aresetn,
    input  logic          s_axis_tvalid,
    output logic          s_axis_tready,
    input  logic [127:0]  s_axis_tdata,
    output logic          m_axis_tvalid,
    input  logic          m_axis_tready,
    output logic [127:0]  m_axis_tdata
);
axis_reg_slice_skid #(.W(128)) inst (.*);
endmodule

module axis_register_slice_meta_256 (
    input  logic          aclk,
    input  logic          aresetn,
    input  logic          s_axis_tvalid,
    output logic          s_axis_tready,
    input  logic [255:0]  s_axis_tdata,
    output logic          m_axis_tvalid,
    input  logic          m_axis_tready,
    output logic [255:0]  m_axis_tdata
);
axis_reg_slice_skid #(.W(256)) inst (.*);
endmodule
