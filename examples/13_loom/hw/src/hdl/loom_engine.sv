/**
 * loom_engine — read src -> FIFO -> write dst, one transfer at a time.
 * Abstract descriptor in; abstract DMA (sq) requests out + phase flags. The
 * shared FIFO/stream datapath lives in vfpga_top so an arbiter can share it
 * between multiple engines. Same behavior as the original inline FSM.
 */

import lynxTypes::*;

module loom_engine (
  input  logic                        aclk,
  input  logic                        aresetn,

  // Descriptor in
  input  logic                        start,
  output logic                        clear_start,   // edge-accepted -> clear CMD[0]
  output logic                        busy,
  output logic                        done_valid,
  input  logic [LEN_BITS-1:0]         len,
  input  logic [PID_BITS-1:0]         src_pid,
  input  logic [VADDR_BITS-1:0]       src_addr,
  input  logic [3:0]                  src_stream,
  input  logic [PID_BITS-1:0]         dst_pid,
  input  logic [VADDR_BITS-1:0]       dst_addr,
  input  logic [3:0]                  dst_stream,

  // Abstract DMA request out (mapped to sq_rd/sq_wr in vfpga_top)
  output logic                        rd_req,
  output logic [PID_BITS-1:0]         rd_pid,
  output logic [VADDR_BITS-1:0]       rd_addr,
  output logic [LEN_BITS-1:0]         rd_len,
  output logic [3:0]                  rd_stream,
  input  logic                        rd_ack,        // sq_rd.valid && sq_rd.ready
  input  logic                        rd_last,       // last read beat into FIFO
  input  logic                        rd_cq,         // cq_rd.valid

  output logic                        wr_req,
  output logic [PID_BITS-1:0]         wr_pid,
  output logic [VADDR_BITS-1:0]       wr_addr,
  output logic [LEN_BITS-1:0]         wr_len,
  output logic [3:0]                  wr_stream,
  input  logic                        wr_ack,        // sq_wr.valid && sq_wr.ready
  input  logic                        wr_last,       // last write beat out of FIFO
  input  logic                        wr_cq,         // cq_wr.valid

  // Phase flags (for the shared FIFO/stream steering)
  output logic                        rd_active,     // in READ phase
  output logic                        wr_active      // in WRITE phase
);

typedef enum logic [1:0] {ST_IDLE, ST_READ, ST_WRITE} state_t;
state_t state_C;
logic   rd_issued, wr_issued, armed, wr_data_done;

assign clear_start = start && armed && (state_C == ST_IDLE);
assign busy        = (state_C != ST_IDLE);
assign rd_active   = (state_C == ST_READ);
assign wr_active   = (state_C == ST_WRITE);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        state_C <= ST_IDLE;
        rd_issued <= 1'b0; wr_issued <= 1'b0; armed <= 1'b1;
        wr_data_done <= 1'b0; done_valid <= 1'b0;
    end else begin
        done_valid <= 1'b0;
        if (clear_start)     armed <= 1'b0;
        else if (!start)     armed <= 1'b1;

        case (state_C)
            ST_IDLE: begin
                if (clear_start) begin
                    rd_issued <= 1'b0; wr_issued <= 1'b0; wr_data_done <= 1'b0;
                    state_C   <= ST_READ;
                end
            end
            ST_READ: begin
                if (rd_ack)  rd_issued <= 1'b1;
                if (rd_last) state_C   <= ST_WRITE;
            end
            ST_WRITE: begin
                if (wr_ack)  wr_issued <= 1'b1;
                if (wr_last) wr_data_done <= 1'b1;
                if ((wr_data_done || wr_last) && wr_cq) begin
                    state_C    <= ST_IDLE;
                    done_valid <= 1'b1;
                end
            end
            default: state_C <= ST_IDLE;
        endcase
    end
end

always_comb begin
    rd_req    = (state_C == ST_READ) && ~rd_issued;
    rd_pid    = src_pid;
    rd_addr   = src_addr;
    rd_len    = len;
    rd_stream = src_stream;

    wr_req    = (state_C == ST_WRITE) && ~wr_issued;
    wr_pid    = dst_pid;
    wr_addr   = dst_addr;
    wr_len    = len;
    wr_stream = dst_stream;
end

endmodule
