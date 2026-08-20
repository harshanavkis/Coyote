module endian_swap #(
    parameter DATA_WIDTH = 128,
    parameter KEEP_WIDTH = DATA_WIDTH / 8
)(
    input  wire [DATA_WIDTH-1:0] data_in,
    input  wire [KEEP_WIDTH-1:0] tkeep_in,
    output wire [DATA_WIDTH-1:0] data_out,
    output wire [KEEP_WIDTH-1:0] tkeep_out
);

    genvar i;
    
    // Reverse bytes in data
    generate
        for (i = 0; i < KEEP_WIDTH; i = i + 1) begin : data_swap
            assign data_out[(KEEP_WIDTH - 1 - i)*8 +: 8] = data_in[i*8 +: 8];
        end
    endgenerate

    // Reverse bits in tkeep
    generate
        for (i = 0; i < KEEP_WIDTH; i = i + 1) begin : tkeep_swap
            assign tkeep_out[KEEP_WIDTH - 1 - i] = tkeep_in[i];
        end
    endgenerate

endmodule
