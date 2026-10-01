import lynxTypes::*;

/**
 * uwin_mon (EN_UWIN_HBM)
 *
 * Counts the shell's axi_main port (static -> shell, xclk) where the user data
 * window's writes enter the shell, before the shell's clock-crossing
 * SmartConnect: whether beats already arrive slowly from the static layer (CPM
 * bridge, NoC) or are held back by the shell. Free-running, in xclk:
 *   0 cycles   1 AW handshakes   2 W beats   3 W stalled (wvalid, !wready)
 *   4 W starved (beats announced by AW, !wvalid)   5 AW stalled   6 B handshakes
 * A snapshot is taken every 256 cycles and held, with a toggle that flips
 * after it: the reader (uwin_hbm, aclk) captures the snapshot when the
 * synchronized toggle changes, by which time it has long been stable.
 */
module uwin_mon #(
    parameter integer N = 7
) (
    input  logic                xclk,
    input  logic                xresetn,

    input  logic                awvalid,
    input  logic                awready,
    input  logic [7:0]          awlen,
    input  logic                wvalid,
    input  logic                wready,
    input  logic                bvalid,
    input  logic                bready,

    output logic [64*N-1:0]     snap,
    output logic                snap_tgl
);

logic [63:0] ctr [N];
logic signed [15:0] w_owed;
logic [7:0] tick;

always_ff @(posedge xclk) begin
    if (!xresetn) begin
        for (int i = 0; i < N; i++) ctr[i] <= '0;
        w_owed   <= '0;
        tick     <= '0;
        snap     <= '0;
        snap_tgl <= 1'b0;
    end else begin
        w_owed <= w_owed + ((awvalid && awready) ? 16'(awlen) + 16'sd1 : 16'sd0) - ((wvalid && wready) ? 16'sd1 : 16'sd0);
        ctr[0] <= ctr[0] + 1;
        ctr[1] <= ctr[1] + (awvalid && awready);
        ctr[2] <= ctr[2] + (wvalid && wready);
        ctr[3] <= ctr[3] + (wvalid && !wready);
        ctr[4] <= ctr[4] + ((w_owed > 0) && !wvalid);
        ctr[5] <= ctr[5] + (awvalid && !awready);
        ctr[6] <= ctr[6] + (bvalid && bready);
        tick <= tick + 1;
        if (tick == 8'd0)
            for (int i = 0; i < N; i++) snap[64*i +: 64] <= ctr[i];
        if (tick == 8'd1)
            snap_tgl <= ~snap_tgl;
    end
end

endmodule
