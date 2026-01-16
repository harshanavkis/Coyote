module payload_to_dma #(
    parameter KEEP_WIDTH = AXI_DATA_BITS/8
)
(
    input wire aclk,
    input wire aresetn,

    input wire dma_start,
    input wire dma_direction,
    input wire [63:0] dma_src_addr,
    input wire [63:0] dma_dst_addr,
    input wire [63:0] dma_len,
    output reg dma_status,
    output reg dma_status_valid,
    output reg clear_dma_start,

    // MMIO arbitration: block DMA output when MMIO response is being sent
    input wire mmio_output_active,
    // Indicates DMA wants to output (in SEND_HEADER or SEND_PAYLOAD state)
    output wire dma_output_active,

    input wire [AXI_DATA_BITS-1:0] payload_to_dma_in_tdata,
    input wire [KEEP_WIDTH-1:0] payload_to_dma_in_tkeep,
    input wire payload_to_dma_in_tvalid,
    output reg payload_to_dma_in_tready,
    input wire payload_to_dma_in_tlast,
    input wire payload_to_dma_in_tuser,

    output reg [AXI_DATA_BITS-1:0] payload_to_dma_out_tdata,
    output reg [KEEP_WIDTH-1:0] payload_to_dma_out_tkeep,
    output reg payload_to_dma_out_tvalid,
    input wire payload_to_dma_out_tready,
    output reg payload_to_dma_out_tlast,
    output reg payload_to_dma_out_tuser,

    output reg [63:0] dma_tx_length,
    output reg dma_tx_length_valid
);

    // IDLE: DMA engine is waiting for DMA commands
    localparam IDLE = 0;
    // CLEAR_DMA_REG: DMA engine is clearing the DMA register
    localparam CLEAR_DMA_REG = 1;
    // SEND_HEADER: DMA engine is sending a header with the request (rd/wr, address, length).
    localparam SEND_HEADER = 2;
    // SEND_PAYLOAD: DMA engine is sending payload data if it is a write request.
    localparam SEND_PAYLOAD = 3;
    // RECEIVE_PAYLOAD: DMA engine is receiving payload data if it is a read request.
    localparam RECEIVE_PAYLOAD = 4;

    reg [2:0] state;
    reg [2:0] next_state;
    reg [7:0] h2d;
    reg [63:0] dma_d2h_count;

    // DMA is active when in output states (SEND_HEADER or SEND_PAYLOAD)
    assign dma_output_active = (state == SEND_HEADER) || (state == SEND_PAYLOAD);

    always @(posedge aclk)
    begin
        if (!aresetn)
            state <= IDLE;
        else
            state <= next_state;
    end

    always @(*)
    begin
        next_state = IDLE;
        case (state)
            IDLE: 
                if (dma_start)
                    next_state = CLEAR_DMA_REG;
                else
                    next_state = IDLE;
            CLEAR_DMA_REG:
                // Only transition to SEND_HEADER when MMIO is not active
                if (!mmio_output_active)
                    next_state = SEND_HEADER;
                else
                    next_state = CLEAR_DMA_REG;
            SEND_HEADER:
                // Only transition when MMIO is not active AND downstream is ready
                if (!mmio_output_active && payload_to_dma_out_tready) begin
                    if (h2d)
                        next_state = SEND_PAYLOAD;
                    else
                        next_state = RECEIVE_PAYLOAD;
                end else
                    next_state = SEND_HEADER;
            SEND_PAYLOAD:
                // Only transition when MMIO is not active AND downstream is ready
                if (!mmio_output_active && payload_to_dma_out_tready && (dma_d2h_count + KEEP_WIDTH >= dma_len))
                    next_state = IDLE;
                else
                    next_state = SEND_PAYLOAD;
            RECEIVE_PAYLOAD:
                if (payload_to_dma_in_tlast && (dma_d2h_count + KEEP_WIDTH >= dma_len))
                    next_state = IDLE;
                else
                    next_state = RECEIVE_PAYLOAD;
            default: 
                next_state = IDLE;
        endcase
    end

    always @(posedge aclk) begin
        if (!aresetn) begin
            h2d <= 8'b0;
            dma_d2h_count <= 64'b0;
        end else if (state == CLEAR_DMA_REG) begin
            h2d <= dma_direction;
            dma_d2h_count <= 64'b0;
        end else if (state == SEND_PAYLOAD && payload_to_dma_out_tready) begin
            dma_d2h_count <= dma_d2h_count + KEEP_WIDTH;
        end else if (state == RECEIVE_PAYLOAD && payload_to_dma_in_tvalid) begin
            dma_d2h_count <= dma_d2h_count + KEEP_WIDTH;
        end
    end

    always_comb
    begin
       // Default values
       dma_status = 1'b0;
       dma_status_valid = 1'b0;
       clear_dma_start = 1'b0;
       payload_to_dma_out_tdata = {AXI_DATA_BITS{1'b0}};
       payload_to_dma_out_tkeep = {KEEP_WIDTH{1'b0}};
       payload_to_dma_out_tvalid = 1'b0;
       payload_to_dma_out_tlast = 1'b0;
       payload_to_dma_out_tuser = 1'b0;

       payload_to_dma_in_tready = 1'b1;

       dma_tx_length_valid = 1'b0;
       dma_tx_length = 64'b0;
       
       case (state)
            IDLE: begin
               // Do nothing 
            end 
                
            CLEAR_DMA_REG: begin
                clear_dma_start = 1'b1;
                dma_status_valid = 1'b1;
                dma_status = 1'b0;
            end

            SEND_HEADER: begin
                if (h2d) begin // TODO: right now when h2d is 1, it is a d2h actually
                    payload_to_dma_out_tdata = {dma_len, dma_dst_addr, h2d};
                    payload_to_dma_out_tlast = 1'b0; // Header and payload are merged
                    payload_to_dma_out_tkeep = {KEEP_WIDTH{1'b1}};
                end else begin
                    payload_to_dma_out_tdata = {dma_len, dma_src_addr, h2d};
                    payload_to_dma_out_tlast = 1'b1; // Read has no payload
                    payload_to_dma_out_tkeep = {{(KEEP_WIDTH - 17){1'b0}}, {17{1'b1}}};
                end
                // Gate output by MMIO not being active - prevent overlapping tvalid
                payload_to_dma_out_tvalid = !mmio_output_active;
            end

            SEND_PAYLOAD: begin
                // TODO: for D2H we should maybe send tlast after SEND_HEADER and SEND_PAYLOAD, instead of sending for each
                // TODO: Check if we need to use payload_to_dma_out_tready while triggering payload_to_dma_out_tvalid
                payload_to_dma_out_tdata = {AXI_DATA_BITS{1'b0}};
                payload_to_dma_out_tkeep = {KEEP_WIDTH{1'b1}};
                // Gate output by MMIO not being active - prevent overlapping tvalid
                payload_to_dma_out_tvalid = !mmio_output_active;
                payload_to_dma_out_tlast = (dma_d2h_count + KEEP_WIDTH >= dma_len);

                // On write completion, set the status of DMA register so that it can be polled by the CPU
                dma_status_valid = (dma_d2h_count + KEEP_WIDTH >= dma_len);
                dma_status = (dma_d2h_count + KEEP_WIDTH >= dma_len);
                dma_tx_length_valid = (dma_d2h_count + KEEP_WIDTH >= dma_len);
                dma_tx_length = dma_d2h_count + KEEP_WIDTH;
            end
            RECEIVE_PAYLOAD: begin
                // On read completion, set the status of DMA register so that it can be polled by the CPU
                dma_status_valid = payload_to_dma_in_tlast == 1'b1;
                dma_status = payload_to_dma_in_tlast == 1'b1;
                dma_tx_length_valid = payload_to_dma_in_tlast && payload_to_dma_in_tvalid;
                dma_tx_length = dma_d2h_count + KEEP_WIDTH;

                // TODO: payload_to_dma_in_tdata also contains the header, this must be stripped off
            end
            default: begin
                // Do nothing
            end 
       endcase 
    end

endmodule