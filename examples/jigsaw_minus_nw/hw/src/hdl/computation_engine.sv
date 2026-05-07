/**
 * computation_engine
 * @brief Orchestrates the full computation pipeline: H2D DMA → compute (spin) → D2H DMA.
 *
 * When start_computation is pulsed, the FSM:
 *   1. Issues a H2D DMA read (dma_start=1, dma_direction=0)
 *   2. Waits for H2D DMA completion
 *   3. Spins for cycles_per_computation clock cycles
 *   4. Issues a D2H DMA write (dma_start=1, dma_direction=1) — skipped when dma_d2h_len == 0
 *   5. Waits for D2H DMA completion
 *   6. Signals computation done (computation_status=1, computation_status_valid=1)
 *
 * Skipping a phase advances the FSM without pulsing the DMA engine, so
 * Coyote never sees a len=0 sq_rd/sq_wr request.
 */
module computation_engine (
    input  logic        aclk,
    input  logic        aresetn,

    // Control inputs (from AXI ctrl parser registers)
    input  logic        start_computation,
    input  logic [63:0] cycles_per_computation,

    // Zero-length D2H is treated as "skip the D2H phase".
    input  logic [63:0] dma_d2h_len,

    // DMA status feedback (from DMA engine)
    input  logic        dma_status,
    input  logic        dma_status_valid,

    // DMA control outputs (muxed into DMA engine in top-level)
    output logic        comp_dma_start,
    output logic        comp_dma_direction,

    // Computation status outputs (back to AXI ctrl parser)
    output logic        computation_active,
    output logic        computation_status,
    output logic        computation_status_valid,

    // Clear start_computation register in ctrl parser
    output logic        clear_computation_start
);

// FSM states
localparam IDLE       = 3'd0;
localparam H2D_START  = 3'd1;
localparam H2D_WAIT   = 3'd2;
localparam COMPUTE    = 3'd3;
localparam D2H_START  = 3'd4;
localparam D2H_WAIT   = 3'd5;
localparam DONE       = 3'd6;

logic [2:0] state, next_state;
logic [63:0] cycle_counter;

// State register
always_ff @(posedge aclk) begin
    if (aresetn == 1'b0)
        state <= IDLE;
    else
        state <= next_state;
end

// Next-state logic
always_comb begin
    next_state = state;
    case (state)
        IDLE:
            if (start_computation)
                next_state = H2D_START;
        H2D_START:
            // Single-cycle pulse to trigger DMA, then wait
            next_state = H2D_WAIT;
        H2D_WAIT:
            if (dma_status_valid && dma_status) begin
                if (cycles_per_computation == 0)
                    next_state = D2H_START;
                else
                    next_state = COMPUTE;
            end
        COMPUTE:
            if (cycle_counter == 0)
                next_state = D2H_START;
        D2H_START:
            // Skip the D2H phase entirely when there's nothing to write back.
            next_state = (dma_d2h_len == 0) ? DONE : D2H_WAIT;
        D2H_WAIT:
            if (dma_status_valid && dma_status)
                next_state = DONE;
        DONE:
            next_state = IDLE;
        default:
            next_state = IDLE;
    endcase
end

// Cycle counter for compute phase
always_ff @(posedge aclk) begin
    if (aresetn == 1'b0) begin
        cycle_counter <= 64'd0;
    end else begin
        if (state == H2D_WAIT && next_state == COMPUTE) begin
            // Load counter on transition into COMPUTE
            // Subtract 1 because the COMPUTE state itself counts as a cycle
            cycle_counter <= (cycles_per_computation > 0) ? cycles_per_computation - 1 : 64'd0;
        end else if (state == COMPUTE && cycle_counter > 0) begin
            cycle_counter <= cycle_counter - 1;
        end
    end
end

// Output logic
always_comb begin
    comp_dma_start          = 1'b0;
    comp_dma_direction      = 1'b0;
    computation_active      = (state != IDLE);
    computation_status      = 1'b0;
    computation_status_valid = 1'b0;
    clear_computation_start = 1'b0;

    case (state)
        IDLE: begin
            // Nothing
        end
        H2D_START: begin
            comp_dma_start     = 1'b1;
            comp_dma_direction = 1'b0;  // H2D (read from host)
            clear_computation_start = 1'b1;  // Auto-clear the start register
        end
        H2D_WAIT: begin
            // Hold direction so DMA engine latches it correctly in CLEAR_DMA_REG
            comp_dma_direction = 1'b0;  // H2D
        end
        COMPUTE: begin
            // Spinning for the configured number of cycles
        end
        D2H_START: begin
            comp_dma_start     = (dma_d2h_len != 0);  // Skip pulse when nothing to write
            comp_dma_direction = 1'b1;  // D2H (write to host)
        end
        D2H_WAIT: begin
            // Hold direction so DMA engine latches it correctly in CLEAR_DMA_REG
            comp_dma_direction = 1'b1;  // D2H
        end
        DONE: begin
            computation_status       = 1'b1;
            computation_status_valid = 1'b1;
        end
        default: begin
            // Nothing
        end
    endcase
end

endmodule
