import lynxTypes::*;

/**
 * uwin_mon (EN_UWIN_HBM)
 *
 * Counts the shell's axi_main port (static -> shell, xclk) where the user data
 * window's writes and reads enter the shell, before the shell's clock-crossing
 * SmartConnect: whether beats already arrive slowly from the static layer (CPM
 * bridge, NoC) or are held back by the shell. Free-running, in xclk:
 *   0 cycles   1 AW handshakes   2 W beats   3 W stalled (wvalid, !wready)
 *   4 W starved (beats announced by AW, !wvalid)   5 AW stalled   6 B handshakes
 *   7 AR handshakes   8 R beats   9 reads completed (last R beat)
 *   10 cycles with a read outstanding (AR taken, last R not yet)
 *   11 sum of reads outstanding over cycles (/ 9 = mean AR-to-last-R cycles)
 *   12 most reads outstanding at once   13 AR stalled (arvalid, !arready)
 *   14 R stalled (rvalid, !rready: static side slow)
 * It sits on the static's side of uwin_ra (the axi_main read-ahead): what the
 * peer sees, ring hits included. uwin_ra's own counters (xclk) ride along:
 * the snapshot is these N words, then its N_RA.
 * A snapshot is taken every 256 cycles and held, with a toggle that flips
 * after it: the reader (uwin_hbm, aclk) captures the snapshot when the
 * synchronized toggle changes, by which time it has long been stable.
 */
module uwin_mon #(
    parameter integer N    = 15,
    parameter integer N_RA = 24
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

    input  logic                arvalid,
    input  logic                arready,
    input  logic [7:0]          arlen,      // not counted: R beats / AR is the mean
    input  logic                rvalid,
    input  logic                rready,
    input  logic                rlast,

    input  logic [64*N_RA-1:0]  ra_ctr,     // uwin_ra's counters

    output logic [64*(N+N_RA)-1:0] snap,
    output logic                snap_tgl
);

logic [63:0] ctr [N];
logic signed [15:0] w_owed;
logic [15:0] rd_out;                    // AR taken, last R not yet
logic [7:0] tick;
wire ar_hs   = arvalid && arready;
wire r_hs    = rvalid && rready;
wire rd_done = r_hs && rlast;

always_ff @(posedge xclk) begin
    if (!xresetn) begin
        for (int i = 0; i < N; i++) ctr[i] <= '0;
        w_owed   <= '0;
        rd_out   <= '0;
        tick     <= '0;
        snap     <= '0;
        snap_tgl <= 1'b0;
    end else begin
        w_owed <= w_owed + ((awvalid && awready) ? 16'(awlen) + 16'sd1 : 16'sd0) - ((wvalid && wready) ? 16'sd1 : 16'sd0);
        rd_out <= rd_out + (ar_hs ? 16'd1 : 16'd0) - (rd_done ? 16'd1 : 16'd0);
        ctr[0] <= ctr[0] + 1;
        ctr[1] <= ctr[1] + (awvalid && awready);
        ctr[2] <= ctr[2] + (wvalid && wready);
        ctr[3] <= ctr[3] + (wvalid && !wready);
        ctr[4] <= ctr[4] + ((w_owed > 0) && !wvalid);
        ctr[5] <= ctr[5] + (awvalid && !awready);
        ctr[6] <= ctr[6] + (bvalid && bready);
        ctr[7] <= ctr[7] + ar_hs;
        ctr[8] <= ctr[8] + r_hs;
        ctr[9] <= ctr[9] + rd_done;
        ctr[10] <= ctr[10] + (rd_out != 0);
        ctr[11] <= ctr[11] + rd_out;
        if (64'(rd_out) > ctr[12]) ctr[12] <= 64'(rd_out);
        ctr[13] <= ctr[13] + (arvalid && !arready);
        ctr[14] <= ctr[14] + (rvalid && !rready);
        tick <= tick + 1;
        if (tick == 8'd0) begin
            for (int i = 0; i < N; i++) snap[64*i +: 64] <= ctr[i];
            snap[64*N +: 64*N_RA] <= ra_ctr;
        end
        if (tick == 8'd1)
            snap_tgl <= ~snap_tgl;
    end
end

endmodule
