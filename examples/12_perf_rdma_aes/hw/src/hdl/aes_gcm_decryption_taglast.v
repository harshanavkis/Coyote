// AES-256-GCM decrypt wrapper for a tag-LAST wire format.
//
//   in:  ct0 ... ct(k-1) rx_tag        out: pt0 ... pt(k-1) computed_tag
//
// Upstream's aes_gcm_decryption.v wants the tag FIRST, which suits Jigsaw where
// software builds the frame. Here the peer is an FPGA whose encrypt engine can
// only emit its tag after the whole packet, so the wire format is tag-last and
// moving it to the front would mean buffering a frame on the transmit side.
//
// The computed tag also leaves as 16 valid bytes rather than upstream's empty
// tlast beat: the shell already committed the fragment length, so the frame out
// must be exactly as long as the frame in. Software ignores those bytes.
//
// ghash_tag_val pulses with the outgoing tag beat on a match. Quarantine policy
// lives in the bank, not here.

`timescale 1ns / 1ps

module aes_gcm_decryption_taglast #(
    parameter [95:0] IV_INIT = 96'h0,
    parameter [95:0] IV_STRIDE = 96'h1
) (
    input  wire clk,
    input  wire rst,

    input  wire iv_dir,

    input  wire [127:0] aes_in_tdata,
    input  wire [15:0]  aes_in_tkeep,
    input  wire         aes_in_tvalid,
    output wire         aes_in_tready,
    input  wire         aes_in_tlast,
    input  wire         aes_in_tuser,

    output wire [127:0] aes_out_tdata,
    output wire [15:0]  aes_out_tkeep,
    output wire         aes_out_tvalid,
    input  wire         aes_out_tready,
    output wire         aes_out_tlast,
    output wire         aes_out_tuser,

    output wire         ghash_tag_val
);

    // ------------------------------------------------------------------
    // Split the received tag off the end of the frame
    // ------------------------------------------------------------------
    wire [127:0] ct_tdata;
    wire [15:0]  ct_tkeep;
    wire         ct_tvalid;
    wire         ct_tready;
    wire         ct_tlast;

    wire [127:0] rx_tag_tdata;
    wire         rx_tag_tvalid;
    wire         rx_tag_tready;

    axis_strip_tail_beat #(
        .DATA_WIDTH(128),
        .KEEP_WIDTH(16)
    ) tag_split (
        .clk(clk),
        .rst(rst),
        .s_axis_tdata(aes_in_tdata),
        .s_axis_tkeep(aes_in_tkeep),
        .s_axis_tvalid(aes_in_tvalid),
        .s_axis_tready(aes_in_tready),
        .s_axis_tlast(aes_in_tlast),
        .m_axis_tdata(ct_tdata),
        .m_axis_tkeep(ct_tkeep),
        .m_axis_tvalid(ct_tvalid),
        .m_axis_tready(ct_tready),
        .m_axis_tlast(ct_tlast),
        .tail_tdata(rx_tag_tdata),
        .tail_tvalid(rx_tag_tvalid),
        .tail_tready(rx_tag_tready)
    );

    // ------------------------------------------------------------------
    // Received-tag queue. The streaming engine keeps several packets in
    // flight, so the tag of packet p must wait here until packet p's
    // computed tag comes out. 16 entries matches upstream's depth.
    // ------------------------------------------------------------------
    reg [127:0] rx_tag_mem [0:15];
    reg [4:0]   rx_tag_wr;
    reg [4:0]   rx_tag_rd;

    wire [4:0] rx_tag_count = rx_tag_wr - rx_tag_rd;
    wire       rx_tag_full  = (rx_tag_count >= 5'd16);

    assign rx_tag_tready = !rx_tag_full;

    always @(posedge clk) begin
        if (rst) begin
            rx_tag_wr <= 5'd0;
        end else if (rx_tag_tvalid && rx_tag_tready) begin
            rx_tag_mem[rx_tag_wr[3:0]] <= rx_tag_tdata;
            rx_tag_wr <= rx_tag_wr + 1;
        end
    end

    // ------------------------------------------------------------------
    // Engine (decrypt mode)
    // ------------------------------------------------------------------
    wire [127:0] eng_in_tdata_be;
    wire [15:0]  eng_in_tkeep_be;

    endian_swap le_to_be (
        .data_in(ct_tdata),
        .tkeep_in(ct_tkeep),
        .data_out(eng_in_tdata_be),
        .tkeep_out(eng_in_tkeep_be)
    );

    wire [127:0] eng_out_tdata;
    wire [15:0]  eng_out_tbval;
    wire         eng_out_tvalid;
    wire         eng_out_tlast;
    wire         eng_out_is_tag;

    aes_gcm_stream #(
        .KEY(256'h0),
        .IV_INIT(IV_INIT),
        .IV_STRIDE(IV_STRIDE)
    ) engine (
        .clk(clk),
        .rst(rst),
        .enc_dec(1'b1),
        .iv_dir(iv_dir),
        .s_tdata(eng_in_tdata_be),
        .s_tbval(eng_in_tkeep_be),
        .s_tvalid(ct_tvalid),
        .s_tready(ct_tready),
        .s_tlast(ct_tlast),
        .m_tdata(eng_out_tdata),
        .m_tbval(eng_out_tbval),
        .m_tvalid(eng_out_tvalid),
        .m_tready(aes_out_tready),
        .m_tlast(eng_out_tlast),
        .m_is_tag(eng_out_is_tag)
    );

    wire [127:0] eng_out_tdata_le;
    wire [15:0]  eng_out_tkeep_le;

    endian_swap be_to_le (
        .data_in(eng_out_tdata),
        .tkeep_in(eng_out_tbval),
        .data_out(eng_out_tdata_le),
        .tkeep_out(eng_out_tkeep_le)
    );

    // The computed tag occupies the slot the received tag vacated, so it
    // leaves as 16 valid bytes and the frame length is preserved.
    assign aes_out_tdata  = eng_out_tdata_le;
    assign aes_out_tkeep  = eng_out_is_tag ? 16'hFFFF : eng_out_tkeep_le;
    assign aes_out_tvalid = eng_out_tvalid;
    assign aes_out_tlast  = eng_out_tlast;
    assign aes_out_tuser  = 1'b0;

    wire tag_out_fire = eng_out_tvalid && aes_out_tready && eng_out_is_tag;

    always @(posedge clk) begin
        if (rst) begin
            rx_tag_rd <= 5'd0;
        end else if (tag_out_fire) begin
            rx_tag_rd <= rx_tag_rd + 1;
        end
    end

    assign ghash_tag_val = tag_out_fire &&
                           (eng_out_tdata_le == rx_tag_mem[rx_tag_rd[3:0]]);

endmodule
