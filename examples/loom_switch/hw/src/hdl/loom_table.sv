import lynxTypes::*;

/**
 * loom_table (loom_switch)
 *
 * Window table: one entry per window (1..15; index 0 unused), shared by the
 * two ways a producer names a window:
 *   - the AXI-Lite aperture: window = addr[15:12], 4 KB each (lu_* port)
 *   - the ingress window (uwin): a range [ustart, ustart + len) of the uwin,
 *     placed by the control daemon, so a binding can be as large as the
 *     uwin itself (ua_* port)
 *   route = 0 (local): write via sq_wr {LOCAL_WRITE, STRM_HOST, pid, base+off}
 *   route = 1 (rdma):  a Loom message on the QP owned by pid; dst_pid = the
 *                      exporter's ctid on the far host, carried in the
 *                      message header
 * base is always the exporter's own VA; len is the segment bounds.
 * Programmed only through the CSR page (loom_ctrl). Overlapping uwin ranges
 * are the daemon's to avoid; the lowest index wins.
 */
module loom_table #(
    parameter integer UWIN_BITS = 27
) (
    input  logic                    aclk,
    input  logic                    aresetn,

    // Program port (from loom_ctrl)
    input  logic                    commit,
    input  logic [3:0]              prog_idx,
    input  logic                    prog_valid,
    input  logic                    prog_route,
    input  logic [PID_BITS-1:0]     prog_pid,
    input  logic [PID_BITS-1:0]     prog_dst_pid,
    input  logic [VADDR_BITS-1:0]   prog_base,
    input  logic [LEN_BITS-1:0]     prog_len,
    input  logic [UWIN_BITS-1:0]    prog_ustart,

    // Aperture lookup by index (combinational)
    input  logic [3:0]              lu_idx,
    output logic                    lu_valid,
    output logic                    lu_route,
    output logic [PID_BITS-1:0]     lu_pid,
    output logic [PID_BITS-1:0]     lu_dst_pid,
    output logic [VADDR_BITS-1:0]   lu_base,
    output logic [LEN_BITS-1:0]     lu_len,

    // Ingress lookup by uwin address (combinational): the window whose range
    // holds ua_addr, and ua_addr's offset inside it
    input  logic [UWIN_BITS-1:0]    ua_addr,
    output logic                    ua_hit,
    output logic [3:0]              ua_idx,
    output logic                    ua_route,
    output logic [PID_BITS-1:0]     ua_pid,
    output logic [PID_BITS-1:0]     ua_dst_pid,
    output logic [VADDR_BITS-1:0]   ua_base,
    output logic [LEN_BITS-1:0]     ua_len,
    output logic [LEN_BITS-1:0]     ua_off
);

localparam integer N_WIN = 16;

logic                  e_valid [N_WIN];
logic                  e_route [N_WIN];
logic [PID_BITS-1:0]   e_pid   [N_WIN];
logic [PID_BITS-1:0]   e_dpid  [N_WIN];
logic [VADDR_BITS-1:0] e_base  [N_WIN];
logic [LEN_BITS-1:0]   e_len   [N_WIN];
logic [UWIN_BITS-1:0]  e_ustart[N_WIN];

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        for (int i = 0; i < N_WIN; i++) e_valid[i] <= 1'b0;
    end else if (commit) begin
        e_valid[prog_idx]  <= prog_valid;
        e_route[prog_idx]  <= prog_route;
        e_pid[prog_idx]    <= prog_pid;
        e_dpid[prog_idx]   <= prog_dst_pid;
        e_base[prog_idx]   <= prog_base;
        e_len[prog_idx]    <= prog_len;
        e_ustart[prog_idx] <= prog_ustart;
    end
end

assign lu_valid   = e_valid[lu_idx];
assign lu_route   = e_route[lu_idx];
assign lu_pid     = e_pid[lu_idx];
assign lu_dst_pid = e_dpid[lu_idx];
assign lu_base    = e_base[lu_idx];
assign lu_len     = e_len[lu_idx];

// Range match: 15 comparators, lowest index wins
logic [LEN_BITS-1:0] rel [N_WIN];
always_comb begin
    ua_hit = 1'b0;
    ua_idx = 4'd0;
    for (int i = N_WIN-1; i >= 1; i--) begin
        rel[i] = LEN_BITS'(ua_addr - e_ustart[i]);
        if (e_valid[i] && (ua_addr >= e_ustart[i]) && (rel[i] < e_len[i])) begin
            ua_hit = 1'b1;
            ua_idx = 4'(i);
        end
    end
    rel[0] = '0;
end

assign ua_route   = e_route[ua_idx];
assign ua_pid     = e_pid[ua_idx];
assign ua_dst_pid = e_dpid[ua_idx];
assign ua_base    = e_base[ua_idx];
assign ua_len     = e_len[ua_idx];
assign ua_off     = rel[ua_idx];

endmodule
