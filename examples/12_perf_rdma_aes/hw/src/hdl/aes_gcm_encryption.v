module aes_gcm_encryption #(
    parameter [95:0] IV_INIT = 96'h0,
    parameter [95:0] IV_STRIDE = 96'h1
) (
    input  wire clk,
    input  wire rst,

    input wire  enc_dec,
    input wire  iv_dir,

    input  wire [127:0] aes_in_tdata,
    input  wire [15:0] aes_in_tkeep,
    input  wire aes_in_tvalid,
    output wire aes_in_tready,
    input  wire aes_in_tlast,
    input  wire aes_in_tuser,

    output wire [127:0] aes_out_tdata,
    output wire [15:0] aes_out_tkeep,
    output wire aes_out_tvalid,
    input  wire aes_out_tready,
    output wire aes_out_tlast,
    output wire aes_out_tuser
);

    // Streaming engine: no per-packet init, IV increments per packet.
    // Output frame = ciphertext beats followed by the 16-byte tag beat
    // (tlast), same framing as before.

    wire [127:0] aes_in_tdata_be;
    wire [15:0] aes_in_tkeep_be;

    endian_swap le_to_be(
        .data_in(aes_in_tdata),
        .tkeep_in(aes_in_tkeep),
        .data_out(aes_in_tdata_be),
        .tkeep_out(aes_in_tkeep_be)
    );

    wire [127:0] eng_out_tdata;
    wire [15:0] eng_out_tbval;
    wire eng_out_tvalid;
    wire eng_out_tlast;
    wire eng_out_is_tag;

    aes_gcm_stream #(
        .KEY(256'h0),
        .IV_INIT(IV_INIT),
        .IV_STRIDE(IV_STRIDE)
    ) engine (
        .clk(clk),
        .rst(rst),
        .enc_dec(enc_dec),
        .iv_dir(iv_dir),
        .s_tdata(aes_in_tdata_be),
        .s_tbval(aes_in_tkeep_be),
        .s_tvalid(aes_in_tvalid),
        .s_tready(aes_in_tready),
        .s_tlast(aes_in_tlast),
        .m_tdata(eng_out_tdata),
        .m_tbval(eng_out_tbval),
        .m_tvalid(eng_out_tvalid),
        .m_tready(aes_out_tready),
        .m_tlast(eng_out_tlast),
        .m_is_tag(eng_out_is_tag)
    );

    endian_swap be_to_le(
        .data_in(eng_out_tdata),
        .tkeep_in(eng_out_tbval),
        .data_out(aes_out_tdata),
        .tkeep_out(aes_out_tkeep)
    );

    assign aes_out_tvalid = eng_out_tvalid;
    assign aes_out_tlast = eng_out_tlast;
    assign aes_out_tuser = 1'b0;

endmodule
