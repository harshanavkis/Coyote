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
    input  logic sq_ready_read,
    input  logic sq_ready_write,

    // Host side MMIO vaddr
    input logic [63:0] mmio_vaddr,

    // MMIO specific control signals
    input logic mmio_ctrl,
    output logic mmio_clear,
    output logic mmio_write_done,
    output logic mmio_read_done,

    // MMIO Direct Payload
    input logic [7:0] mmio_op,
    input logic [63:0] mmio_addr,
    input logic [63:0] mmio_data,
    output logic [63:0] mmio_read_data_in,
    output logic mmio_read_data_in_valid,

    // RDMA submission outputs (separate from local DMA SQ)
    output logic rdma_wr_valid,
    output logic [LEN_BITS-1:0] rdma_wr_len,

    // Debug counters exported to top-level mark_debug signals and local host CSRs.
    output logic [23:0][63:0] debug_counters
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
localparam DMA_REQ = 2'b11;

// Register mmio_ctrl to ensure proper synchronization and known reset value
reg mmio_ctrl_reg;

// Beat counter for DMA Write: tracks remaining bytes to receive from network
reg [63:0] dma_wr_remaining;
reg [63:0] dma_wr_addr_reg;
reg [63:0] dma_wr_len_reg;
reg [63:0] dma_rd_addr_reg;
reg [63:0] dma_rd_len_reg;
reg dma_rd_sq_sent;
reg rdma_meta_sent;
logic [23:0][63:0] debug_counter_reg;

wire incoming_dma_cmd =
    network_in_tvalid &&
    (network_in_tdata[OP_POS +: OP_WIDTH] == 8'd0 ||
     network_in_tdata[OP_POS +: OP_WIDTH] == 8'd1);
wire rdma_meta_fire = rdma_wr_valid && sq_ready_write;
wire rdma_first_can_send = rdma_meta_sent || rdma_meta_fire;
wire rdma_data_fire = network_out_tvalid && network_out_tready;
wire network_in_fire = network_in_tvalid && network_in_tready;
wire network_out_fire = network_out_tvalid && network_out_tready;
wire host_out_fire = host_out_tvalid && host_out_tready;

assign debug_counters = debug_counter_reg;

always @(posedge aclk) begin
    if (!aresetn) begin
        mmio_state_cur <= MMIO_IDLE;
        dma_wr_state_cur <= DMA_IDLE;
        dma_rd_state_cur <= DMA_IDLE;
        mmio_ctrl_reg <= 1'b0;
        dma_wr_remaining <= 64'b0;
        dma_wr_addr_reg <= 64'b0;
        dma_wr_len_reg <= 64'b0;
        dma_rd_addr_reg <= 64'b0;
        dma_rd_len_reg <= 64'b0;
        dma_rd_sq_sent <= 1'b0;
        rdma_meta_sent <= 1'b0;
        debug_counter_reg <= '0;
    end else begin
        mmio_state_cur <= mmio_state_next;
        dma_wr_state_cur <= dma_wr_state_next;
        dma_rd_state_cur <= dma_rd_state_next;
        mmio_ctrl_reg <= mmio_ctrl;

        debug_counter_reg[0] <= {
            38'b0,
            network_in_tdata[OP_POS +: OP_WIDTH],
            sq_ready_read,
            sq_ready_write,
            host_in_tready,
            host_in_tvalid,
            host_out_tready,
            host_out_tvalid,
            network_out_tready,
            network_out_tvalid,
            network_in_tready,
            network_in_tvalid,
            mmio_ctrl_reg,
            rdma_meta_sent,
            dma_wr_state_cur,
            dma_rd_state_cur,
            mmio_state_cur
        };

        if (rdma_meta_fire)
            rdma_meta_sent <= 1'b1;

        if (rdma_data_fire && network_out_tlast)
            rdma_meta_sent <= 1'b0;

        // Latch command headers when accepted. SQ requests are then held valid
        // in DMA_REQ until Coyote accepts them.
        if (dma_wr_state_cur == DMA_IDLE && network_in_tvalid && network_in_tready &&
            network_in_tdata[OP_POS +: OP_WIDTH] == 8'd1) begin
            dma_wr_remaining <= network_in_tdata[LEN_POS +: LEN_WIDTH];
            dma_wr_addr_reg <= network_in_tdata[ADDR_POS +: ADDR_WIDTH];
            dma_wr_len_reg <= network_in_tdata[LEN_POS +: LEN_WIDTH];
        end
        // Decrement on each payload beat handshake in DMA_WR state
        else if (dma_wr_state_cur == DMA_WR && network_in_tvalid && network_in_tready) begin
            dma_wr_remaining <= dma_wr_remaining - KEEP_WIDTH;
        end

        if (dma_rd_state_cur == DMA_IDLE && network_in_tvalid && network_in_tready &&
            network_in_tdata[OP_POS +: OP_WIDTH] == 8'd0) begin
            dma_rd_addr_reg <= network_in_tdata[ADDR_POS +: ADDR_WIDTH];
            dma_rd_len_reg <= network_in_tdata[LEN_POS +: LEN_WIDTH];
            dma_rd_sq_sent <= 1'b0;
        end else if (dma_rd_state_cur == DMA_REQ && !dma_rd_sq_sent && sq_valid_read && sq_ready_read) begin
            dma_rd_sq_sent <= 1'b1;
        end else if (dma_rd_state_cur == DMA_IDLE) begin
            dma_rd_sq_sent <= 1'b0;
        end

        if (mmio_state_cur == MMIO_IDLE && mmio_state_next == MMIO_ACTIVE)
            debug_counter_reg[1] <= debug_counter_reg[1] + 1'b1;
        if (mmio_state_cur == MMIO_ACTIVE && mmio_op == 8'd0 && network_out_fire)
            debug_counter_reg[2] <= debug_counter_reg[2] + 1'b1;
        if (mmio_state_cur == MMIO_ACTIVE && mmio_op == 8'd1 && network_out_fire)
            debug_counter_reg[3] <= debug_counter_reg[3] + 1'b1;
        if (network_in_fire && dma_wr_state_cur == DMA_IDLE &&
            network_in_tdata[OP_POS +: OP_WIDTH] == 8'd2) begin
            debug_counter_reg[4] <= debug_counter_reg[4] + 1'b1;
            debug_counter_reg[21] <= network_in_tdata[ADDR_POS +: ADDR_WIDTH];
        end
        if (mmio_write_done)
            debug_counter_reg[5] <= debug_counter_reg[5] + 1'b1;
        if (network_in_fire && dma_wr_state_cur == DMA_IDLE &&
            network_in_tdata[OP_POS +: OP_WIDTH] == 8'd1) begin
            debug_counter_reg[6] <= debug_counter_reg[6] + 1'b1;
            debug_counter_reg[22] <= network_in_tdata[LEN_POS +: LEN_WIDTH];
        end
        if (host_out_fire && dma_wr_state_cur == DMA_WR)
            debug_counter_reg[7] <= debug_counter_reg[7] + 1'b1;
        if (host_out_fire && dma_wr_state_cur == DMA_WR && host_out_tlast)
            debug_counter_reg[8] <= debug_counter_reg[8] + 1'b1;
        if (sq_valid_write && sq_ready_write)
            debug_counter_reg[9] <= debug_counter_reg[9] + 1'b1;
        if (network_in_fire && dma_rd_state_cur == DMA_IDLE && dma_wr_state_cur == DMA_IDLE &&
            network_in_tdata[OP_POS +: OP_WIDTH] == 8'd0) begin
            debug_counter_reg[10] <= debug_counter_reg[10] + 1'b1;
            debug_counter_reg[23] <= network_in_tdata[LEN_POS +: LEN_WIDTH];
        end
        if (sq_valid_read && sq_ready_read)
            debug_counter_reg[11] <= debug_counter_reg[11] + 1'b1;
        if (dma_rd_state_cur == DMA_REQ && network_out_fire)
            debug_counter_reg[12] <= debug_counter_reg[12] + 1'b1;
        if (dma_rd_state_cur == DMA_RD && network_out_fire)
            debug_counter_reg[13] <= debug_counter_reg[13] + 1'b1;
        if (rdma_meta_fire)
            debug_counter_reg[14] <= debug_counter_reg[14] + 1'b1;
        if (network_out_fire && network_out_tlast)
            debug_counter_reg[15] <= debug_counter_reg[15] + 1'b1;
        if (rdma_wr_valid && !sq_ready_write)
            debug_counter_reg[16] <= debug_counter_reg[16] + 1'b1;
        if (network_out_tvalid && !network_out_tready)
            debug_counter_reg[17] <= debug_counter_reg[17] + 1'b1;
        if (network_in_tvalid && !network_in_tready)
            debug_counter_reg[18] <= debug_counter_reg[18] + 1'b1;
        if (host_out_tvalid && !host_out_tready)
            debug_counter_reg[19] <= debug_counter_reg[19] + 1'b1;
        if (host_in_tvalid && !host_in_tready)
            debug_counter_reg[20] <= debug_counter_reg[20] + 1'b1;
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
    mmio_read_data_in = 64'b0;
    mmio_read_data_in_valid = 1'b0;

    rdma_wr_valid = 1'b0;
    rdma_wr_len = 0;

    // MMIO State Machine
    case (mmio_state_cur)
        MMIO_IDLE: begin
            if (mmio_ctrl_reg && dma_rd_state_cur == DMA_IDLE && dma_wr_state_cur == DMA_IDLE &&
                !incoming_dma_cmd) begin
                mmio_state_next = MMIO_ACTIVE;
                mmio_clear = 1'b1;
            end
        end
        MMIO_ACTIVE: begin
            // construct MMIO request directly from registers
            network_out_tlast = 1'b1;
            if (mmio_op == 8'd0) begin
                network_out_tdata = {{(AXI_DATA_BITS - 136){1'b0}}, 64'd0, mmio_addr, mmio_op};
                network_out_tkeep = {KEEP_WIDTH{1'b1}};
                network_out_tvalid = rdma_first_can_send;
                rdma_wr_valid = !rdma_meta_sent;
                rdma_wr_len = 64; // 64 bytes
                
                if (network_out_tready && rdma_first_can_send) begin
                    mmio_state_next = MMIO_IDLE;
                end
            end else if (mmio_op == 8'd1) begin
                network_out_tdata = {{(AXI_DATA_BITS - 200){1'b0}}, mmio_data, 64'd0, mmio_addr, mmio_op};
                network_out_tkeep = {KEEP_WIDTH{1'b1}};
                network_out_tvalid = rdma_first_can_send;
                rdma_wr_valid = !rdma_meta_sent;
                rdma_wr_len = 64; // 64 bytes
                
                if (network_out_tready && rdma_first_can_send) begin
                    mmio_write_done = 1'b1;
                    mmio_state_next = MMIO_IDLE;
                end
            end else begin
                // Wrong MMIO OP
                mmio_state_next = MMIO_IDLE;
            end
        end
        default: mmio_state_next = MMIO_IDLE;
    endcase

    // DMA Write State Machine
    case (dma_wr_state_cur)
        DMA_IDLE: begin
            if (network_in_tvalid && network_in_tready && dma_wr_state_cur == DMA_IDLE) begin
                if (network_in_tdata[OP_POS +: OP_WIDTH] == 8'd2) begin
                    // MMIO Response: Route to registers immediately
                    mmio_read_data_in = network_in_tdata[ADDR_POS +: ADDR_WIDTH];
                    mmio_read_data_in_valid = 1'b1;
                    mmio_read_done = 1'b1;
                end else if (network_in_tdata[OP_POS +: OP_WIDTH] == 8'd1) begin
                    // D2H DMA: consume and latch the header only. The local
                    // write SQ request is held in DMA_REQ until accepted.
                    dma_wr_state_next = DMA_REQ;
                end
            end
        end
        DMA_REQ: begin
            sq_valid_write = 1'b1;
            sq_dir_write = 1'b1;
            sq_addr_write = dma_wr_addr_reg;
            sq_len_write = dma_wr_len_reg;

            if (sq_ready_write) begin
                dma_wr_state_next = DMA_WR;
            end
        end
        DMA_WR: begin
            // Pass through payload beats directly to host_out. 
            if (network_in_tvalid) begin
                host_out_tvalid = 1'b1;
                host_out_tdata = network_in_tdata;
                host_out_tkeep = network_in_tkeep;
                // Use beat count instead of tlast (tlast fires per PMTU fragment)
                if (dma_wr_remaining <= KEEP_WIDTH) begin
                    host_out_tlast = 1'b1;
                    if (host_out_tready) begin
                        dma_wr_state_next = DMA_IDLE;
                    end
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
            if (network_in_tvalid && network_in_tready && dma_wr_state_cur == DMA_IDLE) begin
                if (network_in_tdata[OP_POS +: OP_WIDTH] == 8'd0) begin
                    // H2D read request header is latched first; the local read
                    // SQ request and RDMA reply header are issued from DMA_REQ.
                    dma_rd_state_next = DMA_REQ;
                end
            end
        end
        DMA_REQ: begin
            sq_valid_read = !dma_rd_sq_sent;
            sq_dir_read = 1'b0;
            sq_addr_read = dma_rd_addr_reg;
            sq_len_read = dma_rd_len_reg;

            network_out_tdata = {{(AXI_DATA_BITS - 8){1'b0}}, {8'd2}};
            network_out_tkeep = {{KEEP_WIDTH}{1'b1}};
            network_out_tvalid = rdma_first_can_send && (dma_rd_sq_sent || sq_ready_read);
            rdma_wr_valid = !rdma_meta_sent;
            rdma_wr_len = 64 + dma_rd_len_reg;

            if (network_out_tready && rdma_first_can_send &&
                (dma_rd_sq_sent || sq_ready_read)) begin
                dma_rd_state_next = DMA_RD;
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
    host_in_tready = (dma_rd_state_cur == DMA_RD) && network_out_tready;

    // network_in_tready: This is the most critical one.
    // It must check which output is needed based on the opcode.
    if (dma_wr_state_cur == DMA_WR) begin
        // We are currently streaming a DMA write to the host
        network_in_tready = host_out_tready;
    end else if (dma_wr_state_cur == DMA_IDLE && dma_rd_state_cur == DMA_IDLE) begin
        // We are at a Jigsaw header boundary (Op 0, 1, or 2)
        if (network_in_tdata[OP_POS +: OP_WIDTH] == 8'd2) begin
            // Block incoming command parsing while MMIO request is active/pending.
            network_in_tready = 1'b1;
        end else begin
            case (network_in_tdata[OP_POS +: OP_WIDTH])
                8'd0:    network_in_tready = (mmio_state_cur == MMIO_IDLE); // Latch H2D read header, then issue SQ/RDMA in DMA_REQ
                8'd1:    network_in_tready = (mmio_state_cur == MMIO_IDLE); // Latch D2H write header, then issue local write SQ in DMA_REQ
                default: network_in_tready = 1'b0;
            endcase
        end
    end else begin
        network_in_tready = 1'b0;
    end
end

endmodule
