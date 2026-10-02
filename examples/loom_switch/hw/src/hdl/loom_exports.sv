import lynxTypes::*;

/**
 * loom_exports - the regions this host lets peers write into
 *
 * An incoming packet's RETH names {export index [47:40], offset [39:0]}.
 * Entry i says where export i lives on this host and whose address space it
 * is: {valid, pid, base VA, length}. loom_rx lands a packet only inside an
 * export (offset + length <= the export's length), so a peer can write
 * nowhere this host did not export - the receive-side check an RDMA NIC
 * makes with an rkey.
 *
 * Programmed like loom_table: loom_ctrl stages {idx, valid, pid, base, len}
 * and pulses commit. Two combinational read ports (the packet path and the
 * inline path); indices past N_EXP miss.
 */
module loom_exports #(
    parameter integer N_EXP = 16
) (
    input  logic                        aclk,
    input  logic                        aresetn,

    input  logic                        commit,
    input  logic [7:0]                  prog_idx,
    input  logic                        prog_valid,
    input  logic [PID_BITS-1:0]         prog_pid,
    input  logic [VADDR_BITS-1:0]       prog_base,
    input  logic [39:0]                 prog_len,

    input  logic [7:0]                  a_idx,
    output logic                        a_hit,
    output logic [PID_BITS-1:0]         a_pid,
    output logic [VADDR_BITS-1:0]       a_base,
    output logic [39:0]                 a_len,

    input  logic [7:0]                  b_idx,
    output logic                        b_hit,
    output logic [PID_BITS-1:0]         b_pid,
    output logic [VADDR_BITS-1:0]       b_base,
    output logic [39:0]                 b_len
);

localparam integer IW = $clog2(N_EXP);

logic                  e_valid [N_EXP];
logic [PID_BITS-1:0]   e_pid   [N_EXP];
logic [VADDR_BITS-1:0] e_base  [N_EXP];
logic [39:0]           e_len   [N_EXP];

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        for (int i = 0; i < N_EXP; i++) e_valid[i] <= 1'b0;
    end else if (commit && (prog_idx < 8'(N_EXP))) begin
        e_valid[prog_idx[IW-1:0]] <= prog_valid;
        e_pid  [prog_idx[IW-1:0]] <= prog_pid;
        e_base [prog_idx[IW-1:0]] <= prog_base;
        e_len  [prog_idx[IW-1:0]] <= prog_len;
    end
end

always_comb begin
    a_hit  = (a_idx < 8'(N_EXP)) && e_valid[a_idx[IW-1:0]];
    a_pid  = e_pid [a_idx[IW-1:0]];
    a_base = e_base[a_idx[IW-1:0]];
    a_len  = e_len [a_idx[IW-1:0]];
    b_hit  = (b_idx < 8'(N_EXP)) && e_valid[b_idx[IW-1:0]];
    b_pid  = e_pid [b_idx[IW-1:0]];
    b_base = e_base[b_idx[IW-1:0]];
    b_len  = e_len [b_idx[IW-1:0]];
end

endmodule
