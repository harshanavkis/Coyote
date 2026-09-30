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
 */
module uwin_hbm #(
    parameter integer          UWIN_BITS = 27,
    parameter logic [63:0]     BASE      = 64'h0,
    parameter integer          N_RD      = 16      // reads in flight
) (
    input  logic                aclk,
    input  logic                aresetn,

    AXI4.s                      s_axi,
    AXI4.m                      m_axi
);

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
// a read is accepted only with room to remember its length
logic [7:0] rq_len [N_RD];
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

assign s_axi.rdata    = m_axi.rdata;
assign s_axi.rid      = m_axi.rid;
wire  r_last = (r_beat == rq_len[rq_rp[$clog2(N_RD)-1:0]]);
assign s_axi.rlast    = r_last;

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        rq_wp  <= '0;
        rq_rp  <= '0;
        r_beat <= '0;
    end else begin
        if (s_axi.arvalid && s_axi.arready) begin
            rq_len[rq_wp[$clog2(N_RD)-1:0]] <= s_axi.arlen;
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
