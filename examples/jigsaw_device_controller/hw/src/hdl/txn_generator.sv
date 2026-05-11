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
    output wire txn_generator_out_tuser,

    // RDMA submission outputs: asserted on the first beat of each outgoing packet
    output wire rdma_wr_valid,
    output wire [63:0] rdma_wr_len,
    input  wire sq_wr_ready,

    // Debug counters exported to top-level mark_debug signals and local device CSRs.
    output wire [23:0][63:0] debug_counters
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
        - mmio_reg_tvalid/mmio_read_valid: MMIO request/response is pending
        - dma_output_active: DMA is in SEND_HEADER or SEND_PAYLOAD state
        - read_data_ready: fires only when the MMIO response beat is accepted
        - payload_to_dma launch: held off while MMIO is pending, but an
          already-launched DMA packet drains without MMIO interruption
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

    // SW-driven DMA signals (from payload_to_mmio register bank)
    wire sw_dma_start;
    wire sw_dma_direction;
    wire [63:0] dma_src_addr;
    wire [63:0] dma_dst_addr;
    wire [63:0] dma_h2d_len;
    wire [63:0] dma_d2h_len;
    wire [63:0] dma_len;
    wire dma_status;
    wire dma_status_valid;
    wire clear_dma_start;
    wire dma_output_active;  // Indicates DMA is in output state (SEND_HEADER or SEND_PAYLOAD)

    wire [63:0] dma_tx_len;
    wire dma_tx_len_valid;
    wire mmio_read_fire;
    logic [23:0][63:0] debug_counter_reg;
    logic dma_start_q;
    logic start_computation_q;

    // Computation engine signals
    wire start_computation;
    wire [63:0] cycles_per_computation;
    wire comp_dma_start;
    wire comp_dma_direction;
    wire computation_active;
    wire computation_status;
    wire computation_status_valid;
    wire clear_computation_start;

    // Muxed DMA control (computation engine overrides SW when active)
    wire dma_start;
    wire dma_direction;
    assign dma_start     = computation_active ? comp_dma_start     : sw_dma_start;
    assign dma_direction = computation_active ? comp_dma_direction : sw_dma_direction;
    assign dma_len       = dma_direction      ? dma_d2h_len        : dma_h2d_len;
    assign debug_counters = debug_counter_reg;

    wire dma_start_pulse = dma_start && !dma_start_q;
    wire start_computation_pulse = start_computation && !start_computation_q;

    // State machine to track multi-beat transactions
    // MMIO is single-beat, so no state needed for it
    // Only DMA needs state tracking for multi-beat transfers
    localparam IDLE = 1'd0;
    localparam DMA_ACTIVE = 1'd1;

    reg state, state_next;

    // Beat counter for DMA reply payload: the op=2 header beat is consumed here
    // and not forwarded into payload_to_dma.
    reg [63:0] dma_reply_remaining;

    always @(posedge aclk) begin
        if (aresetn == 1'b0) begin
            state <= IDLE;
            dma_reply_remaining <= 64'b0;
        end else begin
            state <= state_next;

            // Load payload counter when the DMA reply header (op=2) is accepted.
            if (state == IDLE && txn_generator_in_tvalid && txn_generator_in_tready && dev_op == 8'd2) begin
                dma_reply_remaining <= dma_len;
            end
            // Decrement on each payload handshake during DMA_ACTIVE.
            else if (state == DMA_ACTIVE && txn_generator_in_tvalid && txn_generator_in_tready) begin
                dma_reply_remaining <= dma_reply_remaining - KEEP_WIDTH;
            end
        end
    end

    // State transition logic
    always @(*) begin
        state_next = state;
        
        case (state)
            IDLE: begin
                // Consume the op=2 DMA reply header, then receive dma_len bytes
                // of payload in DMA_ACTIVE.
                // MMIO (op=0, op=1) is single-beat, handled without state change
                if (txn_generator_in_tvalid && txn_generator_in_tready) begin
                    if (dev_op == 8'd2 && dma_len != 64'd0)
                        state_next = DMA_ACTIVE;
                    // For MMIO (op=0, op=1), stay in IDLE since it's single-beat
                end
            end
            
            DMA_ACTIVE: begin
                // DMA can be multi-beat, use beat counter instead of tlast
                // (tlast fires per PMTU fragment)
                if (txn_generator_in_tvalid && txn_generator_in_tready && dma_reply_remaining <= KEEP_WIDTH)
                    state_next = IDLE;
            end
            
            default: state_next = IDLE;
        endcase
    end

    // Route to MMIO or DMA based on current state
    // MMIO: only when IDLE AND opcode is 0 or 1
    // DMA: the op=2 header is consumed locally; only subsequent payload beats
    // are forwarded to payload_to_dma.
    wire route_to_mmio = (state == IDLE && (dev_op == 8'd0 || dev_op == 8'd1));
    wire route_to_dma_reply_header = (state == IDLE && dev_op == 8'd2);
    wire route_to_dma = (state == DMA_ACTIVE);
    wire mmio_input_fire = route_to_mmio && txn_generator_in_tvalid && txn_generator_in_tready;

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
            if (mmio_input_fire) begin
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
    assign txn_generator_in_tready = (route_to_mmio && mmio_can_accept && !clear_dma_start) ||
                                      route_to_dma_reply_header ||
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
        .read_data_ready(mmio_read_fire),       // Read response was accepted by the output stream
        .dma_output_active(dma_output_active),  // Prevent MMIO from asserting read_data_valid while DMA is outputting
        .dma_start(sw_dma_start),
        .dma_direction(sw_dma_direction),
        .dma_src_addr(dma_src_addr),
        .dma_dst_addr(dma_dst_addr),
        .dma_h2d_len(dma_h2d_len),
        .dma_d2h_len(dma_d2h_len),
        .dma_status(dma_status),
        .dma_status_valid(dma_status_valid),
        .computation_status(computation_status),
        .computation_status_valid(computation_status_valid),
        .clear_dma_start(clear_dma_start),
        .dma_tx_len(dma_tx_len),
        .dma_tx_len_valid(dma_tx_len_valid),
        .start_computation(start_computation),
        .cycles_per_computation(cycles_per_computation),
        .clear_computation_start(clear_computation_start)
    );

    // Computation engine: orchestrates H2D → compute → D2H pipeline
    computation_engine inst_computation_engine (
        .aclk(aclk),
        .aresetn(aresetn),
        .start_computation(start_computation),
        .cycles_per_computation(cycles_per_computation),
        .dma_status(dma_status),
        .dma_status_valid(dma_status_valid),
        .comp_dma_start(comp_dma_start),
        .comp_dma_direction(comp_dma_direction),
        .computation_active(computation_active),
        .computation_status(computation_status),
        .computation_status_valid(computation_status_valid),
        .clear_computation_start(clear_computation_start)
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
        .mmio_output_active(mmio_input_fire || mmio_reg_tvalid || mmio_read_valid),
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

    // RDMA submission logic: submit metadata once, before the first data beat
    // of each outgoing packet is allowed to leave.
    reg packet_active;
    reg packet_meta_sent;
    reg packet_source_valid;
    reg packet_source_mmio;

    // Fix C: credit-based metadata pipelining (mirror of HC change).
    // The original `!packet_meta_sent` gate prevented a new metadata
    // submission until the current packet's tlast — strict depth-1
    // serialization. Adding an N_OUTSTANDING-deep credit counter lets DC
    // submit metadata for up to N_OUTSTANDING packets concurrently, matching
    // the shell's true queue capacity. Credit consumed on meta_fire (not
    // paired with same-cycle pkt_done), returned on pkt_done.
    localparam integer META_CRED_BITS = $clog2(N_OUTSTANDING + 1);
    reg [META_CRED_BITS-1:0] meta_credits;

    wire candidate_select_mmio = mmio_read_valid && !dma_output_active;
    wire candidate_out_valid = candidate_select_mmio ? mmio_read_valid : payload_to_dma_out_tvalid;
    wire select_mmio = packet_source_valid ? packet_source_mmio : candidate_select_mmio;
    wire selected_out_valid = select_mmio ? mmio_read_valid : payload_to_dma_out_tvalid;
    wire first_beat_pending = selected_out_valid && !packet_active;
    wire meta_credits_avail = (meta_credits != 0);
    wire meta_fire = first_beat_pending && !packet_meta_sent && sq_wr_ready && meta_credits_avail;
    wire first_beat_can_send = !first_beat_pending || packet_meta_sent || meta_fire;
    wire out_fire = txn_generator_out_tvalid && txn_generator_out_tready;
    wire pkt_done = out_fire && txn_generator_out_tlast;

    // Output mux: active DMA output takes priority; MMIO polls wait behind it.
    assign txn_generator_out_tdata = select_mmio ? {{(AXI_DATA_BITS-72){1'b0}}, mmio_read_data} : payload_to_dma_out_tdata;
    assign txn_generator_out_tkeep = select_mmio ? {KEEP_WIDTH{1'b1}} : payload_to_dma_out_tkeep;
    assign txn_generator_out_tvalid = selected_out_valid && first_beat_can_send;
    assign txn_generator_out_tlast = select_mmio ? 1'b1 : payload_to_dma_out_tlast;
    assign txn_generator_out_tuser = select_mmio ? 1'b0 : payload_to_dma_out_tuser;

    assign payload_to_dma_out_tready = txn_generator_out_tready & ~select_mmio & first_beat_can_send;
    assign mmio_read_fire = select_mmio && txn_generator_out_tvalid && txn_generator_out_tready;

    always @(posedge aclk) begin
        if (!aresetn) begin
            packet_active <= 1'b0;
            packet_meta_sent <= 1'b0;
            packet_source_valid <= 1'b0;
            packet_source_mmio <= 1'b0;
            meta_credits <= N_OUTSTANDING[META_CRED_BITS-1:0];
        end else begin
            if (!packet_source_valid && candidate_out_valid) begin
                packet_source_valid <= 1'b1;
                packet_source_mmio <= candidate_select_mmio;
            end

            if (meta_fire)
                packet_meta_sent <= 1'b1;

            if (out_fire) begin
                if (txn_generator_out_tlast) begin
                    packet_active <= 1'b0;
                    packet_meta_sent <= 1'b0;
                    packet_source_valid <= 1'b0;
                end else begin
                    packet_active <= 1'b1;
                end
            end

            // Fix C: credit accounting. -1 on meta_fire (no concurrent done),
            // +1 on pkt_done (no concurrent fire). Same-cycle fire+done
            // (single-beat packets) is a net zero.
            case ({meta_fire, pkt_done})
                2'b10: meta_credits <= meta_credits - 1'b1;
                2'b01: meta_credits <= meta_credits + 1'b1;
                default: ;
            endcase
        end
    end

    // rdma_wr_valid: assert when there is a pending first beat, metadata for
    // the current packet hasn't fired yet, AND a shell-pipeline credit is
    // available. Allows up to N_OUTSTANDING packets in flight.
    assign rdma_wr_valid = first_beat_pending && !packet_meta_sent && meta_credits_avail;

    // rdma_wr_len: total packet length in bytes
    // Note: dma_direction is cleared by clear_dma_start before SEND_HEADER,
    // so we read the opcode from the actual output data (which uses the latched h2d).
    wire [7:0] out_opcode = txn_generator_out_tdata[OP_POS +: OP_WIDTH];
    wire [63:0] rdma_pkt_len = select_mmio ? 64'd64 :
                               (out_opcode == 8'd1 ? (64'd64 + dma_len) : 64'd64);

    assign rdma_wr_len = rdma_pkt_len;

    always @(posedge aclk) begin
        if (!aresetn) begin
            debug_counter_reg <= '0;
            dma_start_q <= 1'b0;
            start_computation_q <= 1'b0;
        end else begin
            dma_start_q <= dma_start;
            start_computation_q <= start_computation;

            debug_counter_reg[0] <= {
                42'b0,
                dev_op,
                txn_generator_out_tready,
                txn_generator_out_tvalid,
                txn_generator_in_tready,
                txn_generator_in_tvalid,
                sq_wr_ready,
                packet_meta_sent,
                packet_active,
                computation_status,
                computation_status_valid,
                computation_active,
                dma_status,
                dma_status_valid,
                dma_output_active,
                state
            };

            if (route_to_mmio && txn_generator_in_tvalid && txn_generator_in_tready && dev_op == 8'd0)
                debug_counter_reg[1] <= debug_counter_reg[1] + 1'b1;
            if (route_to_mmio && txn_generator_in_tvalid && txn_generator_in_tready && dev_op == 8'd1)
                debug_counter_reg[2] <= debug_counter_reg[2] + 1'b1;
            if (mmio_read_valid && !dma_output_active)
                debug_counter_reg[3] <= debug_counter_reg[3] + 1'b1;
            if (mmio_read_fire)
                debug_counter_reg[4] <= debug_counter_reg[4] + 1'b1;
            if (route_to_dma_reply_header && txn_generator_in_tvalid && txn_generator_in_tready) begin
                debug_counter_reg[5] <= debug_counter_reg[5] + 1'b1;
                debug_counter_reg[21] <= dma_len;
            end
            if (route_to_dma && txn_generator_in_tvalid && txn_generator_in_tready)
                debug_counter_reg[6] <= debug_counter_reg[6] + 1'b1;
            if (route_to_dma && txn_generator_in_tvalid && txn_generator_in_tready &&
                dma_reply_remaining <= KEEP_WIDTH)
                debug_counter_reg[7] <= debug_counter_reg[7] + 1'b1;
            if (dma_start_pulse)
                debug_counter_reg[8] <= debug_counter_reg[8] + 1'b1;
            if (dma_start_pulse && !dma_direction)
                debug_counter_reg[9] <= debug_counter_reg[9] + 1'b1;
            if (dma_start_pulse && dma_direction)
                debug_counter_reg[10] <= debug_counter_reg[10] + 1'b1;
            if (dma_status_valid && !dma_status)
                debug_counter_reg[11] <= debug_counter_reg[11] + 1'b1;
            if (dma_status_valid && dma_status)
                debug_counter_reg[12] <= debug_counter_reg[12] + 1'b1;
            if (start_computation_pulse)
                debug_counter_reg[13] <= debug_counter_reg[13] + 1'b1;
            if (computation_status_valid && computation_status)
                debug_counter_reg[14] <= debug_counter_reg[14] + 1'b1;
            if (meta_fire)
                debug_counter_reg[15] <= debug_counter_reg[15] + 1'b1;
            if (txn_generator_out_tvalid && txn_generator_out_tready && !packet_active)
                debug_counter_reg[16] <= debug_counter_reg[16] + 1'b1;
            if (txn_generator_out_tvalid && txn_generator_out_tready && txn_generator_out_tlast)
                debug_counter_reg[17] <= debug_counter_reg[17] + 1'b1;
            if (rdma_wr_valid && !sq_wr_ready)
                debug_counter_reg[18] <= debug_counter_reg[18] + 1'b1;
            if (txn_generator_out_tvalid && !txn_generator_out_tready)
                debug_counter_reg[19] <= debug_counter_reg[19] + 1'b1;
            if (txn_generator_in_tvalid && !txn_generator_in_tready)
                debug_counter_reg[20] <= debug_counter_reg[20] + 1'b1;
            if (dma_tx_len_valid)
                debug_counter_reg[22] <= dma_tx_len;
            debug_counter_reg[23] <= rdma_pkt_len;
        end
    end

endmodule
