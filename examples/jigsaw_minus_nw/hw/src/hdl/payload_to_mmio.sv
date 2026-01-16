module payload_to_mmio (
    input wire aclk,
    input wire aresetn,
    input wire [7:0] op_code,
    input wire [63:0] address,
    input wire [63:0] payload_data,
    input wire payload_valid,
    output wire payload_ready,
    output reg [71:0] read_data,
    output reg read_data_valid,
    input wire read_data_ready,
    // DMA arbitration: block MMIO output when DMA is in output state
    input wire dma_output_active,
    output reg dma_start,
    output reg dma_direction,
    output reg [63:0] dma_src_addr,
    output reg [63:0] dma_dst_addr,
    output reg [63:0] dma_len,
    input wire dma_status,
    input wire dma_status_valid,
    input wire computation_status,
    input wire computation_status_valid,
    input wire clear_dma_start,
    input wire [63:0] dma_tx_len,
    input wire dma_tx_len_valid
);

// Constants
localparam NUM_REGS = 8;

// Register map:
//  0 (RW) - DMA command register is bitwise OR of the following:
//    bit 0 - Start transfer
//    bit 1 - Direction (0 = H2D, 1 = D2H)
localparam DMA_CMD_REG = 0;
//  1 (RW) - DMA source address
localparam DMA_SRC_ADDR_REG = 1;
//  2 (RW) - DMA destination address
localparam DMA_DST_ADDR_REG = 2;
//  3 (RW) - DMA length
localparam DMA_LEN_REG = 3;
//  4 (RW) - DMA status register: 0th bit for DMA completion, 1st bit for computation completion
localparam DMA_STATUS_REG = 4;
//  5 (RW) - Start computation
localparam START_COMPUTATION_REG = 5;
//  6 (RW) - Cycles per computation
localparam CYCLES_PER_COMPUTATION_REG = 6;
//  7 (RW) - DMA transfer length
localparam DMA_TX_LEN_REG = 7;

// Internal registers
reg [63:0] slv_reg [0:NUM_REGS-1];

// Internal signal for pending read data
reg [71:0] read_data_pending;
reg read_data_pending_valid;

// Always ready to accept new transactions (no backpressure)
assign payload_ready = 1'b1;

assign dma_start = slv_reg[DMA_CMD_REG][0];
assign dma_direction = slv_reg[DMA_CMD_REG][1];
assign dma_src_addr = slv_reg[DMA_SRC_ADDR_REG];
assign dma_dst_addr = slv_reg[DMA_DST_ADDR_REG];
assign dma_len = slv_reg[DMA_LEN_REG];

always @(posedge aclk) begin
    if (!aresetn) begin
        // Reset all registers
        for (int i = 0; i < NUM_REGS; i++) begin
            slv_reg[i] <= 64'b0;
        end
        read_data <= 72'b0;
        read_data_valid <= 1'b0;
        read_data_pending <= 72'b0;
        read_data_pending_valid <= 1'b0;
    end else begin
        // Latch DMA status when valid
        if (dma_status_valid) begin
            slv_reg[DMA_STATUS_REG][0] <= dma_status;
        end

        // Latch DMA transfer length when valid
        if (dma_tx_len_valid) begin
            slv_reg[DMA_TX_LEN_REG] <= dma_tx_len;
        end
        
        // Latch computation status when valid
        if (computation_status_valid) begin
            slv_reg[DMA_STATUS_REG][1] <= computation_status;
        end

        // Clear command register when clear_dma_start is asserted
        if (clear_dma_start) begin
            slv_reg[DMA_CMD_REG] <= 64'b0;
        end

        // Handle read data output with ready/valid handshaking
        if (read_data_valid && read_data_ready) begin
            // Data accepted by downstream, clear valid
            read_data_valid <= 1'b0;
        end
        
        // Check if we can output pending data or new data
        // Gate by dma_output_active to prevent MMIO from outputting while DMA is active
        if ((!read_data_valid || read_data_ready) && !dma_output_active) begin
            // Output path is available and DMA is not outputting
            if (read_data_pending_valid) begin
                // Output pending data
                read_data <= read_data_pending;
                read_data_valid <= 1'b1;
                read_data_pending_valid <= 1'b0;
            end else if (payload_valid && payload_ready && op_code == 8'h0) begin
                // New read operation - output directly
                case (address)
                    64'h0:  read_data <= {slv_reg[DMA_CMD_REG], {8'd2}};
                    64'h8:  read_data <= {slv_reg[DMA_SRC_ADDR_REG], {8'd2}};
                    64'h10: read_data <= {slv_reg[DMA_DST_ADDR_REG], {8'd2}};
                    64'h18: read_data <= {slv_reg[DMA_LEN_REG], {8'd2}};
                    64'h20: read_data <= {slv_reg[DMA_STATUS_REG], {8'd2}};
                    64'h28: read_data <= {slv_reg[START_COMPUTATION_REG], {8'd2}};
                    64'h30: read_data <= {slv_reg[CYCLES_PER_COMPUTATION_REG], {8'd2}};
                    64'h38: read_data <= {slv_reg[DMA_TX_LEN_REG], {8'd2}};
                    default: read_data <= 72'b0;
                endcase
                read_data_valid <= 1'b1;
            end
        end else if (payload_valid && payload_ready && op_code == 8'h0) begin
            // Output path is blocked, store to pending
            case (address)
                64'h0:  read_data_pending <= {slv_reg[DMA_CMD_REG], {8'd2}};
                64'h8:  read_data_pending <= {slv_reg[DMA_SRC_ADDR_REG], {8'd2}};
                64'h10: read_data_pending <= {slv_reg[DMA_DST_ADDR_REG], {8'd2}};
                64'h18: read_data_pending <= {slv_reg[DMA_LEN_REG], {8'd2}};
                64'h20: read_data_pending <= {slv_reg[DMA_STATUS_REG], {8'd2}};
                64'h28: read_data_pending <= {slv_reg[START_COMPUTATION_REG], {8'd2}};
                64'h30: read_data_pending <= {slv_reg[CYCLES_PER_COMPUTATION_REG], {8'd2}};
                64'h38: read_data_pending <= {slv_reg[DMA_TX_LEN_REG], {8'd2}};
                default: read_data_pending <= 72'b0;
            endcase
            read_data_pending_valid <= 1'b1;
        end
        
        // Handle write operations
        if (payload_valid && payload_ready && op_code == 8'h1) begin
            case (address)
                64'h0:  slv_reg[DMA_CMD_REG] <= payload_data;
                64'h8:  slv_reg[DMA_SRC_ADDR_REG] <= payload_data;
                64'h10: slv_reg[DMA_DST_ADDR_REG] <= payload_data;
                64'h18: slv_reg[DMA_LEN_REG] <= payload_data;
                64'h20: slv_reg[DMA_STATUS_REG] <= payload_data;
                64'h28: slv_reg[START_COMPUTATION_REG] <= payload_data;
                64'h30: slv_reg[CYCLES_PER_COMPUTATION_REG] <= payload_data;
                default: ; // Ignore writes to invalid addresses
            endcase
        end
    end
end

endmodule