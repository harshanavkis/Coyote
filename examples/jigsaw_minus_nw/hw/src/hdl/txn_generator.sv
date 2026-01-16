module txn_generator #(
    parameter KEEP_WIDTH = AXI_DATA_BITS/8,  // TKEEP width (one bit per byte)
    parameter OP_WIDTH = 8,
    parameter ADDR_WIDTH = 64,
    parameter LEN_WIDTH = 64
)
(
    input  wire aclk,
    input  wire aresetn,

    input  wire [AXI_DATA_BITS - 1:0] txn_generator_in_tdata,
    input  wire [KEEP_WIDTH - 1:0] txn_generator_in_tkeep,
    input  wire txn_generator_in_tvalid,
    output wire txn_generator_in_tready,
    input  wire txn_generator_in_tlast,
    input  wire txn_generator_in_tuser,

    output wire [AXI_DATA_BITS - 1:0] txn_generator_out_tdata,
    output wire [KEEP_WIDTH - 1:0] txn_generator_out_tkeep,
    output wire txn_generator_out_tvalid,
    input  wire txn_generator_out_tready,
    output wire txn_generator_out_tlast,
    output wire txn_generator_out_tuser
);

    /*
    Packets can be of three types: rd, wr, reply

    For such packet types originating from the input (txn_generator_in):
        - rd, wr: indicates MMIO access
        - reply: indicates reply to a DMA read operation from the device
    
    Similarly while sending, the generator must prepend the type:
        - reply: contains the MMIO read request
        - rd, wr: performs DMA to the remote memory region, includes address in plaintext
            - For wr, payload is encrypted
    
    Packet structure received:
        - |8-bit TYPE|Payload|, where TYPE is one of 0 (rd), 1 (wr), 2 (reply)
    
    Payload structure (for MMIO rd/wr):
        - |64-bit address|64-bit length|Payload Data|
    
    Packet structure (for DMA reply in case of rd):
        - |8-bit TYPE|Payload Data|

    Payload data for DMA:
        - Variable size depending on DMA'ed data

    Output Arbitration Logic (MMIO vs DMA mutual exclusion):
    +-------------------------------------------+-------------------------+-------------------------------+
    | Scenario                                  | MMIO                    | DMA                           |
    +-------------------------------------------+-------------------------+-------------------------------+
    | MMIO response pending, DMA idle           | Sends response          | Waits                         |
    | DMA in output state, MMIO pending         | Waits (ready blocked)   | Stays in state, tvalid low    |
    | DMA wants to start, MMIO active           | n/a                     | Stays in CLEAR_DMA_REG        |
    | Neither active                            | First to start wins     | First to start wins           |
    +-------------------------------------------+-------------------------+-------------------------------+
    
    Signals used:
        - mmio_read_valid (mmio_output_active): MMIO has response to send
        - dma_output_active: DMA is in SEND_HEADER or SEND_PAYLOAD state
        - read_data_ready: gated by ~dma_output_active (blocks MMIO when DMA outputting)
        - payload_to_dma_out_tvalid: gated by ~mmio_output_active (blocks DMA when MMIO outputting)
    */

    localparam OP_POS = 0;
    localparam ADDR_POS = OP_POS + OP_WIDTH;
    localparam LEN_POS = ADDR_POS + ADDR_WIDTH;
    localparam DATA_POS = LEN_POS + LEN_WIDTH;

    wire [OP_WIDTH-1:0] dev_op;
    assign dev_op = txn_generator_in_tdata[OP_POS +: OP_WIDTH];

    // MMIO path signals
    wire [71:0] mmio_read_data;
    wire mmio_read_valid;
    wire mmio_ready;
    
    reg [AXI_DATA_BITS - 1:0] mmio_reg_tdata;
    reg [KEEP_WIDTH - 1:0] mmio_reg_tkeep;
    reg mmio_reg_tvalid;
    reg mmio_reg_tlast;
    reg mmio_reg_tuser;

    // DMA path signals
    wire [AXI_DATA_BITS-1:0] dma_fifo_in_tdata;
    wire [KEEP_WIDTH-1:0] dma_fifo_in_tkeep;
    wire dma_fifo_in_tvalid;
    wire dma_fifo_in_tready;
    wire dma_fifo_in_tlast;
    wire dma_fifo_in_tuser;

    wire [AXI_DATA_BITS-1:0] dma_fifo_out_tdata;
    wire [KEEP_WIDTH-1:0] dma_fifo_out_tkeep;
    wire dma_fifo_out_tvalid;
    wire dma_fifo_out_tready;
    wire dma_fifo_out_tlast;
    wire dma_fifo_out_tuser;

    wire [AXI_DATA_BITS-1:0] payload_to_dma_out_tdata;
    wire [KEEP_WIDTH-1:0] payload_to_dma_out_tkeep;
    wire payload_to_dma_out_tvalid;
    wire payload_to_dma_out_tready;
    wire payload_to_dma_out_tlast;
    wire payload_to_dma_out_tuser;

    wire dma_start;
    wire dma_direction;
    wire [63:0] dma_src_addr;
    wire [63:0] dma_dst_addr;
    wire [63:0] dma_len;
    wire dma_status;
    wire dma_status_valid;
    wire clear_dma_start;
    wire dma_output_active;  // Indicates DMA is in output state (SEND_HEADER or SEND_PAYLOAD)

    wire [63:0] dma_tx_len;
    wire dma_tx_len_valid;

    // State machine to track multi-beat transactions
    // MMIO is single-beat, so no state needed for it
    // Only DMA needs state tracking for multi-beat transfers
    localparam IDLE = 1'd0;
    localparam DMA_ACTIVE = 1'd1;

    reg state, state_next;

    always @(posedge aclk) begin
        if (aresetn == 1'b0)
            state <= IDLE;
        else
            state <= state_next;
    end

    // State transition logic
    always @(*) begin
        state_next = state;
        
        case (state)
            IDLE: begin
                // Only transition to DMA_ACTIVE for op=2 (DMA reply)
                // MMIO (op=0, op=1) is single-beat, handled without state change
                if (txn_generator_in_tvalid && txn_generator_in_tready) begin
                    if (dev_op == 8'd2 && !txn_generator_in_tlast)
                        state_next = DMA_ACTIVE;
                    // For MMIO (op=0, op=1), stay in IDLE since it's single-beat
                end
            end
            
            DMA_ACTIVE: begin
                // DMA can be multi-beat, wait for tlast
                if (txn_generator_in_tvalid && txn_generator_in_tready && txn_generator_in_tlast)
                    state_next = IDLE;
            end
            
            default: state_next = IDLE;
        endcase
    end

    // Route to MMIO or DMA based on current state
    // MMIO: only when IDLE AND opcode is 0 or 1
    // DMA: when IDLE with op=2, or in DMA_ACTIVE state (for subsequent beats)
    wire route_to_mmio = (state == IDLE && (dev_op == 8'd0 || dev_op == 8'd1));
    wire route_to_dma = (state == IDLE && dev_op == 8'd2) || 
                        (state == DMA_ACTIVE);

    // MMIO path: Direct register (single beat only)
    wire mmio_can_accept = !mmio_reg_tvalid;
    
    always @(posedge aclk) begin
        if (aresetn == 1'b0) begin
            mmio_reg_tdata <= {AXI_DATA_BITS{1'b0}};
            mmio_reg_tkeep <= {KEEP_WIDTH{1'b0}};
            mmio_reg_tvalid <= 1'b0;
            mmio_reg_tlast <= 1'b0;
            mmio_reg_tuser <= 1'b0;
        end else begin
            if (route_to_mmio && txn_generator_in_tvalid && mmio_can_accept) begin
                mmio_reg_tdata <= txn_generator_in_tdata;
                mmio_reg_tkeep <= txn_generator_in_tkeep;
                mmio_reg_tvalid <= 1'b1;
                mmio_reg_tlast <= txn_generator_in_tlast;
                mmio_reg_tuser <= txn_generator_in_tuser;
            end else if (mmio_ready && mmio_reg_tvalid) begin
                mmio_reg_tvalid <= 1'b0;
            end
        end
    end

    // DMA path: Through FIFO (can handle multi-beat transactions)
    assign dma_fifo_in_tdata = txn_generator_in_tdata;
    assign dma_fifo_in_tkeep = txn_generator_in_tkeep;
    assign dma_fifo_in_tvalid = route_to_dma && txn_generator_in_tvalid;
    assign dma_fifo_in_tlast = txn_generator_in_tlast;
    assign dma_fifo_in_tuser = txn_generator_in_tuser;

    // Input ready based on current routing
    assign txn_generator_in_tready = (route_to_mmio && mmio_can_accept) || 
                                      (route_to_dma && dma_fifo_in_tready);

    // DMA FIFO instantiation
    axis_data_fifo_512 dma_input_fifo (
        .s_axis_aresetn(aresetn),
        .s_axis_aclk(aclk),
        
        // Input from txn_generator_in
        .s_axis_tdata(dma_fifo_in_tdata),
        .s_axis_tkeep(dma_fifo_in_tkeep),
        .s_axis_tvalid(dma_fifo_in_tvalid),
        .s_axis_tready(dma_fifo_in_tready),
        .s_axis_tlast(dma_fifo_in_tlast),
        
        // Output to payload_to_dma
        .m_axis_tdata(dma_fifo_out_tdata),
        .m_axis_tkeep(dma_fifo_out_tkeep),
        .m_axis_tvalid(dma_fifo_out_tvalid),
        .m_axis_tready(dma_fifo_out_tready),
        .m_axis_tlast(dma_fifo_out_tlast)
    );

    // MMIO module
    payload_to_mmio payload_to_mmio (
        .aclk(aclk),
        .aresetn(aresetn),
        .op_code(mmio_reg_tdata[OP_POS +: OP_WIDTH]),
        .address(mmio_reg_tdata[ADDR_POS +: ADDR_WIDTH]),
        .payload_data(mmio_reg_tdata[DATA_POS+: 64]),
        .payload_valid(mmio_reg_tvalid),
        .payload_ready(mmio_ready),
        .read_data(mmio_read_data),
        .read_data_valid(mmio_read_valid),
        .read_data_ready(txn_generator_out_tready & ~dma_output_active),  // Block MMIO output when DMA is in output state
        .dma_output_active(dma_output_active),  // Prevent MMIO from asserting read_data_valid while DMA is outputting
        .dma_start(dma_start),
        .dma_direction(dma_direction),
        .dma_src_addr(dma_src_addr),
        .dma_dst_addr(dma_dst_addr),
        .dma_len(dma_len),
        .dma_status(dma_status),
        .dma_status_valid(dma_status_valid),
        .computation_status(1'b0),
        .computation_status_valid(1'b0),
        .clear_dma_start(clear_dma_start),
        .dma_tx_len(dma_tx_len),
        .dma_tx_len_valid(dma_tx_len_valid)
    );

    // DMA module
    payload_to_dma payload_to_dma (
        .aclk(aclk),
        .aresetn(aresetn),
        .dma_start(dma_start),
        .dma_direction(dma_direction),
        .dma_src_addr(dma_src_addr),
        .dma_dst_addr(dma_dst_addr),
        .dma_len(dma_len),
        .dma_status(dma_status),
        .dma_status_valid(dma_status_valid),
        .clear_dma_start(clear_dma_start),
        .mmio_output_active(mmio_read_valid),  // Gate DMA output when MMIO response is active
        .dma_output_active(dma_output_active), // DMA wants to output (in SEND_HEADER or SEND_PAYLOAD)
        .payload_to_dma_in_tdata(dma_fifo_out_tdata),
        .payload_to_dma_in_tkeep(dma_fifo_out_tkeep),
        .payload_to_dma_in_tvalid(dma_fifo_out_tvalid),
        .payload_to_dma_in_tready(dma_fifo_out_tready),
        .payload_to_dma_in_tlast(dma_fifo_out_tlast),
        .payload_to_dma_in_tuser(dma_fifo_out_tuser),
        .payload_to_dma_out_tdata(payload_to_dma_out_tdata),
        .payload_to_dma_out_tkeep(payload_to_dma_out_tkeep),
        .payload_to_dma_out_tvalid(payload_to_dma_out_tvalid),
        .payload_to_dma_out_tready(payload_to_dma_out_tready),
        .payload_to_dma_out_tlast(payload_to_dma_out_tlast),
        .payload_to_dma_out_tuser(payload_to_dma_out_tuser),
        .dma_tx_length(dma_tx_len),
        .dma_tx_length_valid(dma_tx_len_valid)
    );

    // Output mux: MMIO takes priority
    assign txn_generator_out_tdata = mmio_read_valid ? {{(AXI_DATA_BITS-72){1'b0}}, mmio_read_data} : payload_to_dma_out_tdata;
    assign txn_generator_out_tkeep = mmio_read_valid ? {{(KEEP_WIDTH-9){1'b0}}, 9'h1FF} : payload_to_dma_out_tkeep;
    assign txn_generator_out_tvalid = mmio_read_valid | payload_to_dma_out_tvalid;
    assign txn_generator_out_tlast = mmio_read_valid ? 1'b1 : payload_to_dma_out_tlast;
    assign txn_generator_out_tuser = mmio_read_valid ? 1'b0 : payload_to_dma_out_tuser;

    assign payload_to_dma_out_tready = txn_generator_out_tready & ~mmio_read_valid;

endmodule