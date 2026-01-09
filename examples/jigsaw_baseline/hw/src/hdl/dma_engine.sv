module dma_engine #(
    parameter KEEP_WIDTH = AXI_DATA_BITS/8
) (
    input  logic                        aclk,
    input  logic                        aresetn,

    input logic [AXI_DATA_BITS-1:0] dma_in_tdata,
    input logic [KEEP_WIDTH-1:0] dma_in_tkeep,
    input logic dma_in_tvalid,
    output logic dma_in_tready,
    input logic dma_in_tlast,
    
    output logic [AXI_DATA_BITS-1:0] dma_out_tdata,
    output logic [KEEP_WIDTH-1:0] dma_out_tkeep,
    output logic dma_out_tvalid,
    input logic dma_out_tready,
    output logic dma_out_tlast,

    // DMA engine to MMIO controller
    input logic dma_start,
    input logic dma_direction,
    input logic [VADDR_BITS-1:0] dma_src_addr,
    input logic [VADDR_BITS-1:0] dma_dst_addr,
    input logic [LEN_BITS-1:0] dma_len,
    output logic dma_status,
    output logic dma_status_valid,
    output logic clear_dma_start,

    // DMA engine to coyote's sq/cq
    output logic coyote_dma_d2h,
    output logic [VADDR_BITS-1:0] coyote_dma_addr,
    output logic [LEN_BITS-1:0] coyote_dma_len,
    output logic coyote_dma_req,
    output logic coyote_dma_tx_len_valid,
    output logic [LEN_BITS-1:0] coyote_dma_tx_len
);

// IDLE: DMA engine is waiting for DMA commands
localparam IDLE = 0;
// CLEAR_DMA_REG: DMA engine is clearing the DMA register
localparam CLEAR_DMA_REG = 1;
// SEND_REQUEST: DMA engine is sending a request to coyote
localparam SEND_REQUEST = 2;
// SEND_PAYLOAD: DMA engine is sending payload data if it is a write request.
localparam SEND_PAYLOAD = 3;
// RECEIVE_PAYLOAD: DMA engine is receiving payload data if it is a read request.
localparam RECEIVE_PAYLOAD = 4;

logic [2:0] state;
logic [2:0] next_state;
logic [7:0] d2h;
logic [63:0] dma_d2h_count;
logic [LEN_BITS-1:0] coyote_dma_tx_len_reg;

always @(posedge aclk)
begin
    if (aresetn == 1'b0)
        state <= IDLE;
    else
        state <= next_state;
end

always @(*)
begin
    next_state = state;
    case (state)
        IDLE: 
            if (dma_start)
                next_state = CLEAR_DMA_REG;
            else
                next_state = IDLE;
        CLEAR_DMA_REG:
            next_state = SEND_REQUEST;
        SEND_REQUEST:
            if (d2h) begin
                next_state = SEND_PAYLOAD;
            end else begin
                next_state = RECEIVE_PAYLOAD;
            end
        SEND_PAYLOAD:
            if (dma_out_tready && (dma_d2h_count + KEEP_WIDTH >= dma_len))
                next_state = IDLE;
            else
                next_state = SEND_PAYLOAD;
        RECEIVE_PAYLOAD:
            if (dma_in_tlast && (dma_d2h_count + KEEP_WIDTH >= dma_len))
                next_state = IDLE;
            else
                next_state = RECEIVE_PAYLOAD;
        default: 
            next_state = IDLE;
    endcase
end

always @(posedge aclk) begin
    if (aresetn == 1'b0) begin
        d2h <= 8'b0;
        dma_d2h_count <= 64'b0;
        coyote_dma_tx_len_reg <= 64'b0;
    end else if (state == CLEAR_DMA_REG) begin
        d2h <= dma_direction;
        dma_d2h_count <= 64'b0;
    end else if (state == SEND_PAYLOAD && dma_out_tready) begin
        dma_d2h_count <= dma_d2h_count + KEEP_WIDTH;
        if (dma_d2h_count + KEEP_WIDTH >= dma_len)
            coyote_dma_tx_len_reg <= dma_d2h_count + KEEP_WIDTH;
    end else if (state == RECEIVE_PAYLOAD && dma_in_tvalid) begin
        dma_d2h_count <= dma_d2h_count + KEEP_WIDTH;
        if (dma_in_tlast)
            coyote_dma_tx_len_reg <= dma_d2h_count + KEEP_WIDTH;
    end
end

always_comb
begin
    // Default values
    dma_status = 1'b0;
    dma_status_valid = 1'b0;
    clear_dma_start = 1'b0;
    dma_out_tdata = {AXI_DATA_BITS{1'b0}};
    dma_out_tkeep = {KEEP_WIDTH{1'b0}};
    dma_out_tvalid = 1'b0;
    dma_out_tlast = 1'b0;

    dma_in_tready = 1'b1;
    coyote_dma_req = 1'b0;
    coyote_dma_addr = {VADDR_BITS{1'b0}};

    coyote_dma_d2h = d2h;
    coyote_dma_len = dma_len;
    coyote_dma_tx_len_valid = 1'b0;
    // coyote_dma_tx_len = coyote_dma_tx_len_reg;
    
    case (state)
        IDLE: begin
            // Do nothing 
        end 
            
        CLEAR_DMA_REG: begin
            clear_dma_start = 1'b1;
            dma_status_valid = 1'b1;
            dma_status = 1'b0;
        end

        SEND_REQUEST: begin
            coyote_dma_req = 1'b1;

            if (d2h)
                coyote_dma_addr = dma_dst_addr;
            else
                coyote_dma_addr = dma_src_addr;
        end

        SEND_PAYLOAD: begin
            dma_out_tdata = {AXI_DATA_BITS{1'b1}};
            dma_out_tkeep = {KEEP_WIDTH{1'b1}};
            dma_out_tvalid = 1'b1;
            dma_out_tlast = ((dma_d2h_count + KEEP_WIDTH) >= dma_len);

            // On write completion, set the status of DMA register so that it can be polled by the CPU
            dma_status_valid = ((dma_d2h_count + KEEP_WIDTH) >= dma_len);
            dma_status = ((dma_d2h_count + KEEP_WIDTH) >= dma_len);
            coyote_dma_tx_len_valid = ((dma_d2h_count + KEEP_WIDTH) >= dma_len);
            coyote_dma_tx_len = dma_d2h_count + KEEP_WIDTH;
        end
        RECEIVE_PAYLOAD: begin
            // On read completion, set the status of DMA register so that it can be polled by the CPU
            dma_status_valid = dma_in_tlast && dma_in_tvalid;
            dma_status = dma_in_tlast && dma_in_tvalid;
            coyote_dma_tx_len_valid = dma_in_tlast && dma_in_tvalid;
            coyote_dma_tx_len = dma_d2h_count + KEEP_WIDTH;
        end
        default: begin
            // Do nothing
        end 
    endcase

end

endmodule