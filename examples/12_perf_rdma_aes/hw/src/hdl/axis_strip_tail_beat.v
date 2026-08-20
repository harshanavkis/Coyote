// Splits the final beat off every AXI-Stream frame.
//
//   in:  b0 ... b(k-1) bk    out: b0 ... b(k-1) with tlast moved onto b(k-1)
//                            tail: bk, one beat per frame
//
// This is what keeps the crypto datapath byte-count preserving: Coyote commits
// a fragment's length to the RoCE header before the vFPGA sees it, so the last
// 16 B are reserved as the tag slot -- encrypt drops a pad beat and its tag
// takes the slot, decrypt drops the received tag and the computed one takes it.
//
// Releasing a held beat needs to know whether the next one ends the frame, so
// the held beat waits for its successor. Within a Coyote fragment the source
// streams contiguously, so that is one beat of latency, not lost throughput.
//
// A single-beat frame has no payload and is dropped; the length convention
// (payload >= 64 B) never produces one.

`timescale 1ns / 1ps

module axis_strip_tail_beat #(
    parameter DATA_WIDTH = 128,
    parameter KEEP_WIDTH = DATA_WIDTH/8
) (
    input  wire                     clk,
    input  wire                     rst,

    input  wire [DATA_WIDTH-1:0]    s_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]    s_axis_tkeep,
    input  wire                     s_axis_tvalid,
    output wire                     s_axis_tready,
    input  wire                     s_axis_tlast,

    output wire [DATA_WIDTH-1:0]    m_axis_tdata,
    output wire [KEEP_WIDTH-1:0]    m_axis_tkeep,
    output wire                     m_axis_tvalid,
    input  wire                     m_axis_tready,
    output wire                     m_axis_tlast,

    // The stripped final beat, one per frame
    output wire [DATA_WIDTH-1:0]    tail_tdata,
    output wire                     tail_tvalid,
    input  wire                     tail_tready
);

    reg [DATA_WIDTH-1:0] hold_tdata;
    reg [KEEP_WIDTH-1:0] hold_tkeep;
    reg                  hold_valid;

    reg [DATA_WIDTH-1:0] tail_tdata_reg;
    reg                  tail_tvalid_reg;

    assign tail_tdata  = tail_tdata_reg;
    assign tail_tvalid = tail_tvalid_reg;

    // The tail register can take a new beat if it is empty or is being
    // drained this cycle.
    wire tail_free = !tail_tvalid_reg || tail_tready;

    // The beat now arriving tells us whether the held beat ends the frame.
    wire fwd_last = s_axis_tlast;
    wire fwd_ok   = hold_valid && s_axis_tvalid && (!fwd_last || tail_free);

    assign m_axis_tdata  = hold_tdata;
    assign m_axis_tkeep  = hold_tkeep;
    assign m_axis_tlast  = fwd_last;
    assign m_axis_tvalid = fwd_ok;

    // s_tready never depends on m_tready via m_tvalid: the downstream AES
    // engine drives its tready from FIFO occupancy only, so this is a
    // plain feed-through, not a combinational loop.
    assign s_axis_tready = hold_valid ? (m_axis_tready && (!fwd_last || tail_free))
                                      : 1'b1;

    wire in_fire = s_axis_tvalid && s_axis_tready;

    always @(posedge clk) begin
        if (rst) begin
            hold_valid      <= 1'b0;
            tail_tvalid_reg <= 1'b0;
        end else begin
            if (tail_tvalid_reg && tail_tready) begin
                tail_tvalid_reg <= 1'b0;
            end

            if (in_fire) begin
                if (s_axis_tlast) begin
                    // Tail beat: capture it (only if it actually follows
                    // payload) and close the frame.
                    if (hold_valid) begin
                        tail_tdata_reg  <= s_axis_tdata;
                        tail_tvalid_reg <= 1'b1;
                    end
                    hold_valid <= 1'b0;
                end else begin
                    hold_tdata <= s_axis_tdata;
                    hold_tkeep <= s_axis_tkeep;
                    hold_valid <= 1'b1;
                end
            end
        end
    end

endmodule
