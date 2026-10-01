import lynxTypes::*;

/**
 * uwin_hbm (EN_UWIN_HBM)
 *
 * The user data window straight into card memory: a write (or read) at uwin
 * offset x - the low UWIN_BITS of the address the shell's interconnect hands
 * over - goes to HBM (card) physical address BASE + x. Everything else passes
 * through unchanged, so AXI ordering is the master's (the V80's SmartConnect
 * port carries ID 0: all window traffic is one ID). BASE is the last HBM
 * block, which the driver keeps out of its allocator.
 *
 * READS: the stripe behind this (axi_stripe_rd) raises rlast on every beat of
 * a request's last fragment; the vFPGA card channels do not care (Coyote
 * rewrites last), but this window answers the shell's interconnect, which
 * needs AXI's rlast. So rlast is made here: each accepted read's length is
 * queued (in order: one ID), and rlast is raised on its last beat.
 *
 * COUNTERS: where a peer's writes into the window spend their cycles. A read
 * of the window's last 4 KiB page returns them instead of HBM (the read still
 * goes to HBM, for its beats; only the data is replaced): 64-bit word i at
 * page offset 8*i, free-running, the reader takes deltas.
 *   0 cycles            6 B responses
 *   1 AW bursts         7 B stalled (bvalid, !bready)
 *   2 W beats           8 cycles with a write outstanding (AW taken, no B yet)
 *   3 W stalled         9 sum of writes outstanding over cycles
 *     (wvalid, !wready: memory side slow)       (/ bursts = mean AW-to-B cycles)
 *   4 W starved         10 most writes outstanding at once
 *     (beats announced by AW, !wvalid: PCIe side slow)
 *   5 AW stalled        11 W beats with a partial strobe
 * Words 16-22: uwin_mon's counts of the shell's axi_main port (xclk), where
 * the window's writes enter the shell - see uwin_mon.sv.
 */
module uwin_hbm #(
    parameter integer          UWIN_BITS = 27,
    parameter logic [63:0]     BASE      = 64'h0,
    parameter integer          N_RD      = 16      // reads in flight
) (
    input  logic                aclk,
    input  logic                aresetn,

    AXI4.s                      s_axi,
    AXI4.m                      m_axi,

    // uwin_mon's snapshot of axi_main (xclk), and its toggle
    input  logic [64*7-1:0]     main_snap,
    input  logic                main_tgl
);

// axi_main counters: captured when the synchronized toggle changes (the
// snapshot has been stable since well before it flipped)
localparam integer N_MAIN = 7;
(* ASYNC_REG = "TRUE" *) logic [2:0] main_tgl_s;
logic [64*N_MAIN-1:0] main_ctr;
always_ff @(posedge aclk) begin
    if (!aresetn) begin
        main_tgl_s <= '0;
        main_ctr   <= '0;
    end else begin
        main_tgl_s <= {main_tgl_s[1:0], main_tgl};
        if (main_tgl_s[2] != main_tgl_s[1]) main_ctr <= main_snap;
    end
end

// AR
assign m_axi.araddr   = BASE + {{(64-UWIN_BITS){1'b0}}, s_axi.araddr[UWIN_BITS-1:0]};
assign m_axi.arburst  = s_axi.arburst;
assign m_axi.arcache  = s_axi.arcache;
assign m_axi.arid     = s_axi.arid;
assign m_axi.arlen    = s_axi.arlen;
assign m_axi.arlock   = s_axi.arlock;
assign m_axi.arprot   = s_axi.arprot;
assign m_axi.arqos    = s_axi.arqos;
assign m_axi.arregion = s_axi.arregion;
assign m_axi.arsize   = s_axi.arsize;
// a read is accepted only with room to remember its length (and whether it
// reads the counter page, from which line)
localparam integer N_CTR = 12;
logic [7:0] rq_len [N_RD];
logic       rq_ctr [N_RD];
logic [5:0] rq_line [N_RD];
logic [$clog2(N_RD):0] rq_wp, rq_rp;
wire rq_full  = (rq_wp - rq_rp) == ($clog2(N_RD)+1)'(N_RD);
logic [7:0] r_beat;
assign m_axi.arvalid  = s_axi.arvalid && !rq_full;
assign s_axi.arready  = m_axi.arready && !rq_full;

// AW
assign m_axi.awaddr   = BASE + {{(64-UWIN_BITS){1'b0}}, s_axi.awaddr[UWIN_BITS-1:0]};
assign m_axi.awburst  = s_axi.awburst;
assign m_axi.awcache  = s_axi.awcache;
assign m_axi.awid     = s_axi.awid;
assign m_axi.awlen    = s_axi.awlen;
assign m_axi.awlock   = s_axi.awlock;
assign m_axi.awprot   = s_axi.awprot;
assign m_axi.awqos    = s_axi.awqos;
assign m_axi.awregion = s_axi.awregion;
assign m_axi.awsize   = s_axi.awsize;
assign m_axi.awvalid  = s_axi.awvalid;
assign s_axi.awready  = m_axi.awready;

// W
assign m_axi.wdata    = s_axi.wdata;
assign m_axi.wlast    = s_axi.wlast;
assign m_axi.wstrb    = s_axi.wstrb;
assign m_axi.wvalid   = s_axi.wvalid;
assign s_axi.wready   = m_axi.wready;

// B, R
assign s_axi.bid      = m_axi.bid;
assign s_axi.bresp    = m_axi.bresp;
assign s_axi.bvalid   = m_axi.bvalid;
assign m_axi.bready   = s_axi.bready;

// Counters
logic [63:0] ctr [N_CTR];
logic signed [15:0] w_owed;          // beats announced by AW, not yet on W
logic [15:0]        wr_out;          // AW taken, B not yet
wire aw_hs = s_axi.awvalid && s_axi.awready;
wire w_hs  = s_axi.wvalid && s_axi.wready;
wire b_hs  = s_axi.bvalid && s_axi.bready;

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        for (int i = 0; i < N_CTR; i++) ctr[i] <= '0;
        w_owed <= '0;
        wr_out <= '0;
    end else begin
        w_owed <= w_owed + (aw_hs ? 16'(s_axi.awlen) + 16'sd1 : 16'sd0) - (w_hs ? 16'sd1 : 16'sd0);
        wr_out <= wr_out + (aw_hs ? 16'd1 : 16'd0) - (b_hs ? 16'd1 : 16'd0);
        ctr[0]  <= ctr[0] + 1;
        ctr[1]  <= ctr[1] + aw_hs;
        ctr[2]  <= ctr[2] + w_hs;
        ctr[3]  <= ctr[3] + (s_axi.wvalid && !s_axi.wready);
        ctr[4]  <= ctr[4] + ((w_owed > 0) && !s_axi.wvalid);
        ctr[5]  <= ctr[5] + (s_axi.awvalid && !s_axi.awready);
        ctr[6]  <= ctr[6] + b_hs;
        ctr[7]  <= ctr[7] + (s_axi.bvalid && !s_axi.bready);
        ctr[8]  <= ctr[8] + (wr_out != 0);
        ctr[9]  <= ctr[9] + wr_out;
        if (64'(wr_out) > ctr[10]) ctr[10] <= 64'(wr_out);
        ctr[11] <= ctr[11] + (w_hs && (s_axi.wstrb != '1));
    end
end

// A counter-page read's beat: line rq_line + r_beat, eight counters per line
wire [$clog2(N_RD)-1:0] rp = rq_rp[$clog2(N_RD)-1:0];
wire [5:0] r_line = rq_line[rp] + r_beat[5:0];
logic [AXI_DATA_BITS-1:0] ctr_data;
always_comb begin
    ctr_data = '0;
    for (int j = 0; j < AXI_DATA_BITS/64; j++)
        if (8*r_line + j < N_CTR) ctr_data[64*j +: 64] = ctr[8*r_line + j];
        else if (8*r_line + j >= 16 && 8*r_line + j < 16 + N_MAIN)
            ctr_data[64*j +: 64] = main_ctr[64*(8*r_line + j - 16) +: 64];
end
wire ar_ctr = (s_axi.araddr[UWIN_BITS-1:12] == '1);

assign s_axi.rdata    = rq_ctr[rp] ? ctr_data : m_axi.rdata;
assign s_axi.rid      = m_axi.rid;
wire  r_last = (r_beat == rq_len[rp]);
assign s_axi.rlast    = r_last;

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        rq_wp  <= '0;
        rq_rp  <= '0;
        r_beat <= '0;
    end else begin
        if (s_axi.arvalid && s_axi.arready) begin
            rq_len[rq_wp[$clog2(N_RD)-1:0]]  <= s_axi.arlen;
            rq_ctr[rq_wp[$clog2(N_RD)-1:0]]  <= ar_ctr;
            rq_line[rq_wp[$clog2(N_RD)-1:0]] <= s_axi.araddr[11:6];
            rq_wp <= rq_wp + 1'b1;
        end
        if (m_axi.rvalid && s_axi.rready) begin
            if (r_last) begin
                r_beat <= '0;
                rq_rp  <= rq_rp + 1'b1;
            end else begin
                r_beat <= r_beat + 1'b1;
            end
        end
    end
end
assign s_axi.rresp    = m_axi.rresp;
assign s_axi.rvalid   = m_axi.rvalid;
assign m_axi.rready   = s_axi.rready;

endmodule
