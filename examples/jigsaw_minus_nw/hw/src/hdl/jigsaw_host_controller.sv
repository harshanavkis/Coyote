module jigsaw_host_controller #(
    parameter OP_WIDTH = 8,
    parameter ADDR_WIDTH = 64,
    parameter LEN_WIDTH = 64,
    parameter KEEP_WIDTH = AXI_DATA_BITS/8
) (
    input logic aclk,
    input logic aresetn,
    
    // Network side interfaces: in represents data coming from the network
    input  logic [AXI_DATA_BITS - 1:0] network_in_tdata,
    input  logic [KEEP_WIDTH - 1:0] network_in_tkeep,
    input  logic network_in_tvalid,
    output logic network_in_tready,
    input  logic network_in_tlast,
    input  logic network_in_tuser,

    // Network side interfaces: out represents data going to the network
    output logic [AXI_DATA_BITS - 1:0] network_out_tdata,
    output logic [KEEP_WIDTH - 1:0] network_out_tkeep,
    output logic network_out_tvalid,
    input  logic network_out_tready,
    output logic network_out_tlast,
    output logic network_out_tuser,

    // Host side interfaces: in represents data coming from the host
    input  logic [AXI_DATA_BITS - 1:0] host_in_tdata,
    input  logic [KEEP_WIDTH - 1:0] host_in_tkeep,
    input  logic host_in_tvalid,
    output logic host_in_tready,
    input  logic host_in_tlast,
    input  logic host_in_tuser,

    // Host side interfaces: out represents data going to the host
    output logic [AXI_DATA_BITS - 1:0] host_out_tdata,
    output logic [KEEP_WIDTH - 1:0] host_out_tkeep,
    output logic host_out_tvalid,
    input  logic host_out_tready,
    output logic host_out_tlast,
    output logic host_out_tuser,

    // Submission queue interfaces
    output logic sq_valid_write,
    output logic sq_dir_write,
    output logic [63:0] sq_addr_write,
    output logic [63:0] sq_len_write,

    output logic sq_valid_read,
    output logic sq_dir_read,
    output logic [63:0] sq_addr_read,
    output logic [63:0] sq_len_read,

    // Host side MMIO vaddr
    input logic [63:0] mmio_vaddr,

    // MMIO specific control signals
    input logic mmio_ctrl,
    output logic mmio_clear,
    output logic mmio_write_done,
    output logic mmio_read_done
);

localparam OP_POS = 0;
localparam ADDR_POS = OP_POS + OP_WIDTH;
localparam LEN_POS = ADDR_POS + ADDR_WIDTH;
localparam DATA_POS = LEN_POS + LEN_WIDTH;

// MMIO process
// MMIO is a single cycle operation, but is different for reads and writes
// Read: If the CPU triggers a MMIO read, the request can be sent over the network
//  iff a DMA read is not in progress to prevent overlapping transactions.
//  However we need to wait for a response for a read. Since our transaction ordering
//  prevents overlapping transactions, we can just wait for a packet with with a reply (0x2) header
//  provided a DMA write isn't in progress.
// Write: If the CPU triggers a MMIO write, the request can be sent over the network
//  iff a DMA read isn't in progress to prevent overlapping transactions.
//  However we don't need to wait for a response for a write. Since our transaction ordering
//  prevents overlapping transactions, we can just send the request and move on.
// For getting/setting the payload use the host_in* interfaces and the sq/cq,
// but why: to reduce the number of PCIe messages with the host

logic [1:0] mmio_state_cur, mmio_state_next;

localparam MMIO_IDLE = 2'b00;
localparam MMIO_ACTIVE = 2'b01;
localparam MMIO_RSP = 2'b10;

logic [1:0] dma_rd_state_cur, dma_rd_state_next;
logic [1:0] dma_wr_state_cur, dma_wr_state_next;

localparam DMA_IDLE = 2'b00;
localparam DMA_RD = 2'b01;
localparam DMA_WR = 2'b10;

// Register mmio_ctrl to ensure proper synchronization and known reset value
reg mmio_ctrl_reg;

always @(posedge aclk) begin
    if (!aresetn) begin
        mmio_state_cur <= MMIO_IDLE;
        dma_wr_state_cur <= DMA_IDLE;
        dma_rd_state_cur <= DMA_IDLE;
        mmio_ctrl_reg <= 1'b0;
    end else begin
        mmio_state_cur <= mmio_state_next;
        dma_wr_state_cur <= dma_wr_state_next;
        dma_rd_state_cur <= dma_rd_state_next;
        mmio_ctrl_reg <= mmio_ctrl;
    end
end

// Combinatorial logic for MMIO and DMA
always @(*) begin
    // Default values to avoid latches and multiple drivers
    mmio_state_next = mmio_state_cur;
    dma_wr_state_next = dma_wr_state_cur;
    dma_rd_state_next = dma_rd_state_cur;

    mmio_clear = 1'b0;

    sq_valid_write = 1'b0;
    sq_dir_write = 1'b0;
    sq_addr_write = 64'b0;
    sq_len_write = 64'b0;

    sq_valid_read = 1'b0;
    sq_dir_read = 1'b0;
    sq_addr_read = 64'b0;
    sq_len_read = 64'b0;

    host_out_tdata = network_in_tdata;
    host_out_tkeep = network_in_tkeep;
    host_out_tvalid = 1'b0;
    host_out_tlast = 1'b0;
    host_out_tuser = network_in_tuser;

    network_out_tdata = 512'b0;
    network_out_tkeep = 64'b0;
    network_out_tvalid = 1'b0;
    network_out_tlast = 1'b0;
    network_out_tuser = 1'b0;

    mmio_write_done = 1'b0;
    mmio_read_done = 1'b0;

    // MMIO State Machine
    case (mmio_state_cur)
        MMIO_IDLE: begin
            if (mmio_ctrl_reg && dma_rd_state_cur == DMA_IDLE && dma_wr_state_cur == DMA_IDLE) begin
                mmio_state_next = MMIO_ACTIVE;
                mmio_clear = 1'b1;
                sq_valid_read = 1'b1;
                sq_dir_read = 1'b0;
                sq_addr_read = mmio_vaddr + 64'd24;
                sq_len_read = 64'd25;
            end
        end
        MMIO_ACTIVE: begin
            if (host_in_tvalid && host_in_tlast) begin
                // Only when both tvalid and tlast since we can send partial tkeep only in this case
                network_out_tvalid = host_in_tvalid;
                network_out_tlast = host_in_tlast;
                if (host_in_tdata[OP_POS +: OP_WIDTH] == 8'd0) begin
                    network_out_tdata = {{(AXI_DATA_BITS - 136){1'b0}}, host_in_tdata[135:0]};
                    // Since we do not have a payload for read, we send smaller tkeep
                    network_out_tkeep = {{(KEEP_WIDTH - 17){1'b0}}, 17'h1FFFF};
                    mmio_state_next = MMIO_IDLE;
                end else if (host_in_tdata[OP_POS +: OP_WIDTH] == 8'd1) begin
                    network_out_tdata = {{(AXI_DATA_BITS - 200){1'b0}}, host_in_tdata[199:0]};
                    network_out_tkeep = {{(KEEP_WIDTH - 25){1'b0}}, 25'h1FFFFFF};
                    mmio_state_next = MMIO_IDLE;
                    mmio_write_done = network_out_tready;
                end else begin
                    // Wrong MMIO OP
                    mmio_state_next = MMIO_IDLE;
                end
            end
        end
        default: mmio_state_next = MMIO_IDLE;
    endcase

    // DMA Write State Machine
    case (dma_wr_state_cur)
        DMA_IDLE: begin
            // Check MMIO is not active AND not pending (mmio_ctrl_reg) to prevent race
            if (network_in_tvalid && host_out_tready && mmio_state_cur == MMIO_IDLE && !mmio_ctrl_reg) begin
                if (network_in_tdata[OP_POS +: OP_WIDTH] == 8'd1) begin
                    // D2H DMA: First beat is header only (op + addr + len)
                    // payload_to_dma sends header and payload in separate beats
                    // Capture header info but don't send data from this beat
                    dma_wr_state_next = DMA_WR;
                    sq_valid_write = 1'b1;
                    sq_dir_write = 1'b1;
                    sq_addr_write = network_in_tdata[ADDR_POS +: ADDR_WIDTH];
                    sq_len_write = network_in_tdata[LEN_POS +: LEN_WIDTH];
                    
                    // Don't send data from header beat - wait for payload beats
                    // This beat is consumed (network_in_tready will be high)
                    // but we don't output anything to host_out
                end else if (network_in_tdata[OP_POS +: OP_WIDTH] == 8'd2) begin
                    // This is a MMIO Response from network_in
                    sq_valid_write = 1'b1;
                    sq_dir_write = 1'b0;
                    sq_addr_write = mmio_vaddr + 64'd16;
                    sq_len_write = 64'd8;

                    host_out_tdata = {{(AXI_DATA_BITS - ADDR_WIDTH){1'b0}}, network_in_tdata[ADDR_POS +: ADDR_WIDTH]};
                    host_out_tkeep = {{(KEEP_WIDTH - 8){1'b0}}, 8'hFF};
                    host_out_tvalid = network_in_tvalid;
                    host_out_tlast = network_in_tlast;
                    mmio_read_done = host_out_tready;
                end
            end
        end
        DMA_WR: begin
            // Pass through payload beats directly to host_out
            if (network_in_tvalid && host_out_tready) begin
                host_out_tvalid = 1'b1;
                host_out_tdata = network_in_tdata;
                host_out_tkeep = network_in_tkeep;
                if (network_in_tlast) begin
                    dma_wr_state_next = DMA_IDLE;
                    host_out_tlast = 1'b1;
                end
            end
        end
        default: dma_wr_state_next = DMA_IDLE;
    endcase

    // DMA Read State Machine
    case (dma_rd_state_cur)
        DMA_IDLE: begin
            // Only check for DMA Read when:
            // - DMA Write is NOT active (to prevent race with payload beats)
            // - MMIO is not active AND not pending (mmio_ctrl_reg) to prevent overlap
            if (network_in_tvalid && mmio_state_cur == MMIO_IDLE && !mmio_ctrl_reg && dma_wr_state_cur == DMA_IDLE) begin
                if (network_in_tdata[OP_POS +: OP_WIDTH] == 8'd0) begin
                    if (!sq_valid_read) begin // Arbitration: Prioritize MMIO over DMA Read for SQ access
                        // Fix for combinatorial loop: Valid/Data generation should NOT depend on Ready
                        // TODO: This wastes 63 bytes per transaction, we should probably use
                        // and axis_arb_mux: https://github.com/alexforencich/verilog-axis/blob/master/rtl/axis_arb_mux.v.
                        // This is because partial tkeep can only be used when tlast is high.
                        network_out_tvalid = 1'b1;
                        network_out_tdata = {{(AXI_DATA_BITS - 8){1'b0}}, {8'd2}};
                        network_out_tkeep = {{KEEP_WIDTH}{1'b1}};

                        // Only transition state and issue SQ request if ready
                        if (network_out_tready) begin
                            dma_rd_state_next = DMA_RD;
                            sq_valid_read = 1'b1;
                            sq_dir_read = 1'b0;
                            sq_addr_read = network_in_tdata[ADDR_POS +: ADDR_WIDTH];
                            sq_len_read = network_in_tdata[LEN_POS +: LEN_WIDTH];
                        end
                    end
                end
            end
        end
        DMA_RD: begin
            // DMA Read uses host_in to send data to network_out
            // If MMIO is active, it also wants to use network_out.
            // MMIO_ACTIVE has priority for network_out in this implementation
            if (mmio_state_cur != MMIO_ACTIVE) begin
                network_out_tvalid = host_in_tvalid; 
                network_out_tlast = host_in_tlast;
                network_out_tdata = host_in_tdata;
                network_out_tkeep = host_in_tkeep;
                network_out_tuser = host_in_tuser;
                
                if (host_in_tvalid && network_out_tready) begin
                    if (host_in_tlast) begin
                        dma_rd_state_next = DMA_IDLE;
                    end
                end
            end
        end
        default: dma_rd_state_next = DMA_IDLE;
    endcase
end

// assign host_in_tready = network_out_tready;
// assign network_in_tready = host_out_tready;

always_comb begin
    // host_in_tready: Only ready if we are in a state that uses host_in 
    // AND the network_out is ready to take it.
    host_in_tready = (mmio_state_cur == MMIO_ACTIVE || dma_rd_state_cur == DMA_RD) && network_out_tready;

    // network_in_tready: This is the most critical one.
    // It must check which output is needed based on the opcode.
    if (dma_wr_state_cur == DMA_WR) begin
        // We are currently streaming a DMA write to the host
        network_in_tready = host_out_tready;
    end else if (dma_wr_state_cur == DMA_IDLE && dma_rd_state_cur == DMA_IDLE) begin
        // We are in IDLE, looking for a new command
        // Block if MMIO is active OR pending (mmio_ctrl_reg) to prevent overlap
        if (mmio_state_cur != MMIO_IDLE || mmio_ctrl_reg) begin
            network_in_tready = 1'b0;
        end else begin
            case (network_in_tdata[OP_POS +: OP_WIDTH])
                8'd0:    network_in_tready = network_out_tready; // DMA Read needs network_out
                8'd1, 8'd2: network_in_tready = host_out_tready; // DMA Write / MMIO Resp need host_out
                default: network_in_tready = 1'b0;
            endcase
        end
    end else begin
        network_in_tready = 1'b0;
    end
end

endmodule