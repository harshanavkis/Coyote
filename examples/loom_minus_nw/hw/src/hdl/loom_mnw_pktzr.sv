`timescale 1ns / 1ps
import lynxTypes::*;

/**
 * Packetiser for the minus-nw loopback.
 *
 * On hardware the RoCE stack fragments an outgoing message at PMTU and the
 * receive path sees a tlast at every packet boundary; loom_rx ignores those
 * and ends a transaction where its header's length says. With the stack
 * removed the engine's stream would arrive as one unbroken run, so the
 * branch that skips intermediate tlasts would never be exercised. This puts
 * the boundaries back.
 *
 * BEATS_PER_PKT 64 = 4096 B at a 512 bit datapath, matching cfg(pmtu).
 *
 * drop_n/drop_at are the reason this module is worth having beyond framing:
 * they remove beats from the middle of a message on demand. A retransmission
 * perturbs the payload stream on real hardware, and loom_rx tracks position
 * as a running beat count, so the question of whether a perturbed stream is
 * recoverable can be asked here with no network involved.
 */
module loom_mnw_pktzr #(
    parameter int BEATS_PER_PKT = 64
) (
    input  logic                        aclk,
    input  logic                        aresetn,

    // Drop injection (from a CSR, held stable while idle)
    input  logic [31:0]                 drop_at,      // beat index, 0 = off
    input  logic [7:0]                  drop_n,

    input  logic [AXI_DATA_BITS-1:0]    s_tdata,
    input  logic [AXI_DATA_BITS/8-1:0]  s_tkeep,
    input  logic                        s_tvalid,
    output logic                        s_tready,
    input  logic                        s_tlast,

    output logic [AXI_DATA_BITS-1:0]    m_tdata,
    output logic [AXI_DATA_BITS/8-1:0]  m_tkeep,
    output logic                        m_tvalid,
    input  logic                        m_tready,
    output logic                        m_tlast,

    output logic [31:0]                 cnt_dropped
);

// Two counters, because the two things they gate have different periods:
// pkt_idx places a tlast every PMTU and resets per packet, msg_idx is the
// position within the MESSAGE and is what drop_at names - the same running
// count loom_rx derives its write position from.
logic [31:0] pkt_idx;
logic [31:0] msg_idx;
logic [7:0]  drop_left;
logic [31:0] dropped;

assign cnt_dropped = dropped;

// A beat is swallowed rather than forwarded while drop_left is nonzero.
wire dropping = (drop_left != 8'd0);

// Swallowed beats are still consumed from the source, so the engine is not
// backpressured by the injection itself - the stream loses beats, which is
// what a perturbation looks like, rather than stalling.
assign s_tready = dropping ? 1'b1 : m_tready;
assign m_tvalid = s_tvalid && !dropping;
assign m_tdata  = s_tdata;
assign m_tkeep  = s_tkeep;

// tlast at every packet boundary, and at the source's own end of message
assign m_tlast  = s_tlast || (pkt_idx == BEATS_PER_PKT - 1);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        pkt_idx   <= 0;
        msg_idx   <= 0;
        drop_left <= 0;
        dropped   <= 0;
    end else if (s_tvalid && s_tready) begin
        if (dropping) begin
            drop_left <= drop_left - 8'd1;
            dropped   <= dropped + 32'd1;
        end else if (drop_at != 0 && msg_idx == drop_at) begin
            drop_left <= drop_n;
        end

        pkt_idx <= s_tlast ? 32'd0
                           : ((pkt_idx == BEATS_PER_PKT - 1) ? 32'd0
                                                             : pkt_idx + 32'd1);
        msg_idx <= s_tlast ? 32'd0 : msg_idx + 32'd1;
    end
end

endmodule
