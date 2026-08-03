import lynxTypes::*;

/**
 * loom_table
 *
 * Window table: one entry per 4 KB aperture window (1..15; index 0 unused).
 *   route = 0 (local): write via sq_wr {LOCAL_WRITE, STRM_HOST, pid, base+off}
 *   route = 1 (rdma):  write via sq_wr {APP_WRITE, STRM_RDMA, pid = QP owner, base+off}
 * base is always the exporter's own VA; len is the segment bounds.
 * Programmed only through the CSR page (loom_ctrl).
 */
module loom_table (
    input  logic                    aclk,
    input  logic                    aresetn,

    // Program port (from loom_ctrl)
    input  logic                    commit,
    input  logic [3:0]              prog_idx,
    input  logic                    prog_valid,
    input  logic                    prog_route,
    input  logic [PID_BITS-1:0]     prog_pid,
    input  logic [VADDR_BITS-1:0]   prog_base,
    input  logic [LEN_BITS-1:0]     prog_len,

    // Lookup port (combinational)
    input  logic [3:0]              lu_idx,
    output logic                    lu_valid,
    output logic                    lu_route,
    output logic [PID_BITS-1:0]     lu_pid,
    output logic [VADDR_BITS-1:0]   lu_base,
    output logic [LEN_BITS-1:0]     lu_len
);

localparam integer N_WIN = 16;

logic                  e_valid [N_WIN];
logic                  e_route [N_WIN];
logic [PID_BITS-1:0]   e_pid   [N_WIN];
logic [VADDR_BITS-1:0] e_base  [N_WIN];
logic [LEN_BITS-1:0]   e_len   [N_WIN];

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        for (int i = 0; i < N_WIN; i++) e_valid[i] <= 1'b0;
    end else if (commit) begin
        e_valid[prog_idx] <= prog_valid;
        e_route[prog_idx] <= prog_route;
        e_pid[prog_idx]   <= prog_pid;
        e_base[prog_idx]  <= prog_base;
        e_len[prog_idx]   <= prog_len;
    end
end

assign lu_valid = e_valid[lu_idx];
assign lu_route = e_route[lu_idx];
assign lu_pid   = e_pid[lu_idx];
assign lu_base  = e_base[lu_idx];
assign lu_len   = e_len[lu_idx];

endmodule
