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
 * COUNTERS: where a peer's writes into the window, and its reads of it, spend
 * their cycles. A read of the window's last 4 KiB page returns them instead of
 * HBM (the read still goes to HBM, for its beats; only the data is replaced):
 * 64-bit word i at page offset 8*i, free-running, the reader takes deltas.
 *   0 cycles            6 B responses
 *   1 AW bursts         7 B stalled (bvalid, !bready)
 *   2 W beats           8 cycles with a write outstanding (AW taken, no B yet)
 *   3 W stalled         9 sum of writes outstanding over cycles
 *     (wvalid, !wready: memory side slow)       (/ bursts = mean AW-to-B cycles)
 *   4 W starved         10 most writes outstanding at once
 *     (beats announced by AW, !wvalid: PCIe side slow)
 *   5 AW stalled        11 W beats with a partial strobe
 *                       12 bursts discarded (DISCARD below)
 *                       13 bursts with a nonzero AWID
 *                       14 the read-ahead mode: 1 while on (READ-AHEAD below)
 *                       15 the mode: 1 while discarding
 * Words 16-30: uwin_mon's counts of the shell's axi_main port (xclk), where
 * the window's writes and reads enter the shell - see uwin_mon.sv.
 * Words 64-76: uwin_ra's counts (xclk), the read-ahead on axi_main (on/off:
 * lines 59/58) - see uwin_ra.sv, words 0-12 there. Like 16-30, a snapshot
 * taken every 256 xclk cycles.
 * Words 32-39: the window's reads (counter-page reads among them)
 *   32 AR bursts        36 sum of reads outstanding over cycles
 *   33 R beats             (/ 34 = mean AR-to-last-R cycles)
 *   34 reads completed  37 most reads outstanding at once
 *   35 cycles with a read outstanding (AR taken, last R not yet)
 *   38 AR stalled (arvalid, !arready: memory side slow, or N_RD in flight)
 *   39 R stalled (rvalid, !rready: PCIe side slow)
 * Words 40-49: read-ahead (all 0 while it was never on)
 *   40 stream reads with every line already requested (hits)
 *   41 stream reads that waited for a line not yet requested (late: a
 *      stream's first read, or the peer ahead of the read-ahead)
 *   42 reads of HBM passed through while on (not part of a stream)
 *   43 read-ahead ARs           46 streams started
 *   44 lines read ahead         47 streams ended by an AR that does not
 *      (- 45 = fetched, unused)    continue them (a jump, the counter page)
 *   45 lines answered from      48 ... by a write (AW)
 *      the ring                 49 ... by RA_IDLE cycles without an AR
 * Every other word of the page reads 0.
 *
 * DISCARD (diagnostic): writes are accepted at once, counted, answered here
 * (OKAY) and never reach HBM - the window's own rate with the memory taken
 * out. A read of the counter page's line 63 (window offset UWIN_SIZE - 64)
 * asks for it, line 62 (UWIN_SIZE - 128) for HBM again; reads always go to
 * HBM. The mode changes only with no write in flight (none outstanding to
 * HBM, none waiting here for W or B, no W beat owed), so B responses never
 * mix the two sources, and word 15 says when it has.
 *
 * READ-AHEAD: a peer reading a buffer in order (an RDMA NIC fetching a
 * message: 256 B reads, ~32 in flight across PCIe) waits for HBM on every
 * read. uwin_ra does the same before the shell's clock crossing, where it
 * saves more (lines 59/58); keep this one off while that one is on. Off at
 * reset (the window then works exactly as without it); a read of
 * the counter page's line 61 (UWIN_SIZE - 4096 + 61*64) turns it on, line 60
 * off, at once; word 14 says which.
 *   Stream: an AR continues the previous one if it starts at the line where
 *   that one ended (same ID, 64 B beats, INCR, neither in the counter page).
 *   With read-ahead on and no write outstanding (every AW taken has had its
 *   B), the first such AR starts a stream. Every read of the stream is
 *   answered from a ring of RA_RING lines on chip, which uwin_hbm fills with
 *   its own ARs (the stream's ID; up to RA_BURST lines, never across an
 *   RA_BURST-line boundary) up to RA_LINES lines beyond the stream's last
 *   request, at most N_RD ARs outstanding toward HBM in all.
 *   End: an AR that does not continue the stream (a jump, a counter-page
 *   read - the mode switch among them), any write taken (AW), or RA_IDLE
 *   cycles without an AR ends it (DRAIN): reads ahead stop beyond the
 *   stream's accepted reads, those are answered from the ring, later reads
 *   pass through to HBM; once they are answered the ring is dropped (beats
 *   of fills still in flight are discarded: n_old) and a stream may start
 *   again. The cost of an end: a read passed through right after it waits
 *   behind the fills still in flight (one ID), up to RA_LINES lines.
 *   Order (one ID: R in AR order): R leaves in the peer's AR order (the
 *   length queue, which also says ring or HBM per read). m_axi's R comes in
 *   m_axi's AR order, kept in mq (a fill, or a peer's read): a fill's beats
 *   go into the ring at once (room was reserved when it was issued), a peer
 *   read's beats wait until it is the head of the length queue. No deadlock:
 *   a peer read goes to HBM only once fills cover every line of the stream's
 *   accepted reads (in DRAIN), so the fills a stream read waits for are never
 *   behind a later read's data. A stream's lines are filled in order and
 *   come back in order, so the ring is a FIFO indexed by line mod RA_RING
 *   (one write port, one registered read port: BRAM), no address lookup.
 *   Staleness: the ring answers only reads of the stream it was filled for,
 *   all accepted before the stream ended. A write's AW ends the stream, and a
 *   new one starts only after the write's B, so a read issued after a window
 *   write completed never sees data fetched before it. What uwin_hbm cannot
 *   see is the card path (the copy engine writes HBM directly): a stream read
 *   may return a line fetched up to RA_LINES lines (16 KiB) ahead of the
 *   stream's requests, i.e. before a concurrent card-path write to it landed.
 *   A peer that learns of such a write other than through the window must
 *   break the stream first (a non-sequential read: re-reading a flag does;
 *   the first read of a flag right behind the data does not), wait RA_IDLE
 *   cycles, or turn read-ahead off.
 */
module uwin_hbm #(
    parameter integer          UWIN_BITS = 27,
    parameter logic [63:0]     BASE      = 64'h0,
    parameter integer          N_RD      = 64,      // reads in flight (the shell's axim_udata: 64); ARs to HBM too
    parameter integer          RA_LINES  = 256,     // read-ahead: lines (64 B) beyond the stream's requests (16 KiB)
    parameter integer          RA_BURST  = 16,      // lines per read-ahead AR (1 KiB: a stripe fragment); 2..64, power of 2
    parameter integer          RA_RING   = 512,     // the ring, lines (32 KiB); power of 2, >= 256 + RA_BURST (a peer
                                                    // burst), >= RA_LINES + the peer's reads in flight to keep ahead
    parameter integer          RA_IDLE   = 1024     // cycles without an AR that end a stream
) (
    input  logic                aclk,
    input  logic                aresetn,

    AXI4.s                      s_axi,
    AXI4.m                      m_axi,

    // uwin_mon's snapshot of axi_main (xclk): its 15 counters, then uwin_ra's
    // 13; and its toggle
    input  logic [64*28-1:0]    main_snap,
    input  logic                main_tgl
);

// axi_main counters: captured when the synchronized toggle changes (the
// snapshot has been stable since well before it flipped)
localparam integer N_MAIN = 15, N_XRA = 13;
(* ASYNC_REG = "TRUE" *) logic [2:0] main_tgl_s;
logic [64*(N_MAIN+N_XRA)-1:0] main_ctr;
always_ff @(posedge aclk) begin
    if (!aresetn) begin
        main_tgl_s <= '0;
        main_ctr   <= '0;
    end else begin
        main_tgl_s <= {main_tgl_s[1:0], main_tgl};
        if (main_tgl_s[2] != main_tgl_s[1]) main_ctr <= main_snap;
    end
end

// READ-AHEAD state (lines: window offset / 64, one bit of headroom)
localparam integer LB = UWIN_BITS - 6;
localparam integer RB = $clog2(RA_RING);
localparam logic [LB:0] CTR_LN = (LB+1)'(2**LB - 64);  // the counter page's first line
typedef enum logic [1:0] {ST_IDLE, ST_ACT, ST_DRAIN} ra_st_t;
ra_st_t                 st;
logic                   ra_on;
logic [LB:0]            p_next;             // the previous AR's end
logic                   p_ok;               // ... it could be continued (64 B beats, INCR, not the counter page)
logic [AXI_ID_BITS-1:0] p_id;
logic [LB:0]            s_req;              // the end of the stream's accepted reads
logic [LB:0]            s_hi;               // the end of its fills issued
logic [LB:0]            s_got;              // ... returned (written into the ring)
logic [LB:0]            g_rd;               // the next line read out of the ring (into ring_q)
logic [LB:0]            n_old;              // fill beats still to come for a dropped ring: discarded
logic                   out_v;              // ring_q holds line g_rd - 1, not yet answered
logic [AXI_ID_BITS-1:0] s_id;               // the stream's ID and attributes (its first read's)
logic [3:0]             s_cache, s_qos, s_region;
logic [2:0]             s_prot;
logic [0:0]             s_lock;
logic                   fl_v;               // a fill, issued (s_hi past it), not yet taken by m_axi
logic [LB:0]            fl_ln;
logic [7:0]             fl_len;
logic                   ar_lock;            // a peer read is on m_axi, not taken: it stays one
logic [$clog2(RA_IDLE):0] idle;

// AR: a fill, else the peer's read
assign m_axi.araddr   = BASE + {{(64-UWIN_BITS){1'b0}}, fl_v ? {fl_ln[LB-1:0], 6'b0} : s_axi.araddr[UWIN_BITS-1:0]};
assign m_axi.arburst  = fl_v ? 2'b01    : s_axi.arburst;
assign m_axi.arcache  = fl_v ? s_cache  : s_axi.arcache;
assign m_axi.arid     = fl_v ? s_id     : s_axi.arid;
assign m_axi.arlen    = fl_v ? fl_len   : s_axi.arlen;
assign m_axi.arlock   = fl_v ? s_lock   : s_axi.arlock;
assign m_axi.arprot   = fl_v ? s_prot   : s_axi.arprot;
assign m_axi.arqos    = fl_v ? s_qos    : s_axi.arqos;
assign m_axi.arregion = fl_v ? s_region : s_axi.arregion;
assign m_axi.arsize   = fl_v ? 3'd6     : s_axi.arsize;
// a read is accepted only with room to remember its length (whether it reads
// the counter page, from which line; whether the ring answers it)
localparam integer N_CTR = 16;
logic [7:0] rq_len [N_RD];
logic       rq_ctr [N_RD];
logic       rq_ring [N_RD];
logic [5:0] rq_line [N_RD];
logic [$clog2(N_RD):0] rq_wp, rq_rp;
wire rq_full  = (rq_wp - rq_rp) == ($clog2(N_RD)+1)'(N_RD);
wire [$clog2(N_RD)-1:0] rp = rq_rp[$clog2(N_RD)-1:0];
logic [7:0] r_beat;
wire ar_ctr = (s_axi.araddr[UWIN_BITS-1:12] == '1);   // the counter page
// m_axi's ARs in flight, in order: a fill (its arlen), or a peer's read
logic       mq_fill [N_RD];
logic [7:0] mq_len [N_RD];
logic [$clog2(N_RD):0] mq_wp, mq_rp;
wire mq_full  = (mq_wp - mq_rp) == ($clog2(N_RD)+1)'(N_RD);
wire [$clog2(N_RD)-1:0] mrp = mq_rp[$clog2(N_RD)-1:0];
logic [7:0] f_beat;

// DISCARD: one ring of AWIDs, in order (pushed on AW; lp passes an entry when
// its burst's last W beat is taken; rp when its B is answered)
localparam integer N_DQ = 32;
logic                   discard;
logic                   mode_pend, mode_val;
logic [AXI_ID_BITS-1:0] dq_id [N_DQ];
logic [$clog2(N_DQ):0]  dq_wp, dq_lp, dq_rp;
wire dq_full  = (dq_wp - dq_rp) == ($clog2(N_DQ)+1)'(N_DQ);
wire dq_w     = (dq_wp != dq_lp);          // a burst waiting for its W beats
wire dq_b     = (dq_lp != dq_rp);          // a burst waiting for its B

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
assign m_axi.awvalid  = s_axi.awvalid && !discard;
assign s_axi.awready  = discard ? !dq_full : m_axi.awready;

// W
assign m_axi.wdata    = s_axi.wdata;
assign m_axi.wlast    = s_axi.wlast;
assign m_axi.wstrb    = s_axi.wstrb;
assign m_axi.wvalid   = s_axi.wvalid && !discard;
assign s_axi.wready   = discard ? dq_w : m_axi.wready;

// B, R
assign s_axi.bid      = discard ? dq_id[dq_rp[$clog2(N_DQ)-1:0]] : m_axi.bid;
assign s_axi.bresp    = discard ? 2'b00 : m_axi.bresp;
assign s_axi.bvalid   = discard ? dq_b : m_axi.bvalid;
assign m_axi.bready   = discard ? 1'b1 : s_axi.bready;

// Counters
logic [63:0] ctr [N_CTR];
wire  ar_hs = s_axi.arvalid && s_axi.arready;
logic signed [15:0] w_owed;          // beats announced by AW, not yet on W
logic [15:0]        wr_out;          // AW taken, B not yet
wire aw_hs = s_axi.awvalid && s_axi.awready;
wire w_hs  = s_axi.wvalid && s_axi.wready;
wire b_hs  = s_axi.bvalid && s_axi.bready;
// reads: words 32-39
localparam integer N_RCTR = 8;
logic [63:0] rctr [N_RCTR];
logic [15:0] rd_out;                 // AR taken, last R not yet
wire r_hs    = s_axi.rvalid && s_axi.rready;
wire rd_done = r_hs && s_axi.rlast;

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        for (int i = 0; i < N_CTR; i++) ctr[i] <= '0;
        for (int i = 0; i < N_RCTR; i++) rctr[i] <= '0;
        w_owed <= '0;
        wr_out <= '0;
        rd_out <= '0;
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
        ctr[12] <= ctr[12] + (aw_hs && discard);
        ctr[13] <= ctr[13] + (aw_hs && (s_axi.awid != '0));
        ctr[14] <= 64'(ra_on);
        ctr[15] <= 64'(discard);
        rd_out  <= rd_out + (ar_hs ? 16'd1 : 16'd0) - (rd_done ? 16'd1 : 16'd0);
        rctr[0] <= rctr[0] + ar_hs;
        rctr[1] <= rctr[1] + r_hs;
        rctr[2] <= rctr[2] + rd_done;
        rctr[3] <= rctr[3] + (rd_out != 0);
        rctr[4] <= rctr[4] + rd_out;
        if (64'(rd_out) > rctr[5]) rctr[5] <= 64'(rd_out);
        rctr[6] <= rctr[6] + (s_axi.arvalid && !s_axi.arready);
        rctr[7] <= rctr[7] + (s_axi.rvalid && !s_axi.rready);
    end
end

// DISCARD: the mode request (a counter-page read of line 63 or 62), applied
// once no write is in flight; the ring of discarded bursts
wire wr_idle = (wr_out == 0) && (w_owed == 0) && (dq_wp == dq_rp);
always_ff @(posedge aclk) begin
    if (!aresetn) begin
        discard   <= 1'b0;
        mode_pend <= 1'b0;
        mode_val  <= 1'b0;
        dq_wp     <= '0;
        dq_lp     <= '0;
        dq_rp     <= '0;
    end else begin
        if (ar_hs && ar_ctr && (s_axi.araddr[11:7] == 5'h1F)) begin
            mode_pend <= 1'b1;
            mode_val  <= s_axi.araddr[6];
        end else if (mode_pend && wr_idle && !(s_axi.awvalid && s_axi.awready)) begin
            discard   <= mode_val;
            mode_pend <= 1'b0;
        end
        if (discard && aw_hs) begin
            dq_id[dq_wp[$clog2(N_DQ)-1:0]] <= s_axi.awid;
            dq_wp <= dq_wp + 1'b1;
        end
        if (discard && w_hs && s_axi.wlast) dq_lp <= dq_lp + 1'b1;
        if (discard && s_axi.bvalid && s_axi.bready) dq_rp <= dq_rp + 1'b1;
    end
end

// READ-AHEAD: does the peer's AR continue (or start) the stream; a read the
// ring does not answer goes to HBM only with no fill waiting and, while the
// stream drains, once fills cover its accepted reads (one in flight on m_axi
// stays a pass-through read: ar_lock)
wire [LB:0] a_ln   = (LB+1)'(s_axi.araddr[UWIN_BITS-1:6]);
wire [LB:0] a_end  = a_ln + (LB+1)'(s_axi.arlen) + 1'b1;
wire        a_cont = (s_axi.arsize == 3'd6) && (s_axi.arburst == 2'b01) && !ar_ctr;
wire        a_seq  = p_ok && a_cont && (a_ln == p_next) && (s_axi.arid == p_id);
wire        idle_to = (idle == ($clog2(RA_IDLE)+1)'(RA_IDLE));
wire        ar_ring = a_seq && (((st == ST_ACT) && !idle_to)
                             || ((st == ST_IDLE) && ra_on && !ar_lock && (wr_out == 0)));
wire        pass_ok = (st == ST_IDLE) || ((st == ST_DRAIN) && (s_hi >= s_req));
wire        m_peer  = s_axi.arvalid && !ar_ring && pass_ok && !fl_v && !rq_full && !mq_full;
assign m_axi.arvalid = fl_v || m_peer;
assign s_axi.arready = ar_ring ? !rq_full : (pass_ok && !fl_v && !rq_full && !mq_full && m_axi.arready);
wire ring_hs = ar_hs && ar_ring;
wire mar_hs  = m_axi.arvalid && m_axi.arready;
wire a_hit   = (st == ST_ACT) && (a_end <= s_hi);       // every line already requested
// the stream ends: a write, RA_IDLE cycles without an AR, an AR that does not continue it
wire end_w   = ((st == ST_ACT) || ((st == ST_IDLE) && ring_hs)) && aw_hs;
wire end_i   = (st == ST_ACT) && !aw_hs && idle_to;
wire end_j   = (st == ST_ACT) && !aw_hs && !idle_to && s_axi.arvalid && !ar_ring;

// the next fill: up to the next RA_BURST boundary, RA_LINES beyond the
// stream's requests (draining: up to them), with room in the ring and in mq
// (never while a peer read may go to m_axi: pass_ok excludes it, so the mq
// slot checked here is still free when the fill is taken)
wire [LB:0] f_n  = (LB+1)'(RA_BURST) - (LB+1)'(s_hi[$clog2(RA_BURST)-1:0]);
wire        f_go = !fl_v && !mq_full && (s_hi < CTR_LN) && (s_hi + f_n - g_rd <= (LB+1)'(RA_RING))
                && (((st == ST_ACT) && (s_hi < s_req + (LB+1)'(RA_LINES))) || ((st == ST_DRAIN) && (s_hi < s_req)));

// R: m_axi's head beat is a fill's (into the ring, never held), or the head
// peer read's; the peer's head read is the ring's (ring_q), or m_axi's
wire hd_fill = (mq_wp != mq_rp) && mq_fill[mrp];
wire hd_ring = (rq_wp != rq_rp) && rq_ring[rp];
wire f_hs    = m_axi.rvalid && hd_fill;
wire f_wr    = f_hs && (n_old == 0);                    // ... into the ring at s_got
wire g_take  = r_hs && hd_ring;
wire g_load  = (st != ST_IDLE) && (g_rd != s_got) && (!out_v || g_take);
wire [LB:0] s_lo = g_rd - out_v;                        // the next line the stream's reads take

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        st      <= ST_IDLE;
        ra_on   <= 1'b0;
        p_next  <= '0;
        p_ok    <= 1'b0;
        p_id    <= '0;
        s_req   <= '0;
        s_hi    <= '0;
        s_got   <= '0;
        g_rd    <= '0;
        n_old   <= '0;
        out_v   <= 1'b0;
        fl_v    <= 1'b0;
        ar_lock <= 1'b0;
        idle    <= '0;
        mq_wp   <= '0;
        mq_rp   <= '0;
        f_beat  <= '0;
    end else begin
        if (ar_hs) begin
            p_next <= a_end;
            p_ok   <= a_cont;
            p_id   <= s_axi.arid;
            if (ar_ctr && (s_axi.araddr[11:7] == 5'h1E)) ra_on <= s_axi.araddr[6];
        end
        ar_lock <= m_peer && !m_axi.arready;
        idle    <= ((st != ST_ACT) || ar_hs) ? '0 : idle + 1'b1;
        // fills: issued (s_hi), taken by m_axi (mq), their beats into the ring (s_got)
        if (f_go) begin
            fl_v   <= 1'b1;
            fl_ln  <= s_hi;
            fl_len <= 8'(f_n - 1'b1);
            s_hi   <= s_hi + f_n;
        end else if (fl_v && m_axi.arready) begin
            fl_v   <= 1'b0;
        end
        if (mar_hs) begin
            mq_fill[mq_wp[$clog2(N_RD)-1:0]] <= fl_v;
            mq_len[mq_wp[$clog2(N_RD)-1:0]]  <= fl_len;
            mq_wp <= mq_wp + 1'b1;
        end
        if (f_hs) begin
            if (f_wr) s_got <= s_got + 1'b1;
            else      n_old <= n_old - 1'b1;
            if (f_beat == mq_len[mrp]) begin
                f_beat <= '0;
                mq_rp  <= mq_rp + 1'b1;
            end else begin
                f_beat <= f_beat + 1'b1;
            end
        end else if (r_hs && !hd_ring && s_axi.rlast) begin
            mq_rp <= mq_rp + 1'b1;
        end
        // the ring's output register
        if (g_load) g_rd <= g_rd + 1'b1;
        if (st == ST_IDLE)  out_v <= 1'b0;
        else if (g_load)    out_v <= 1'b1;
        else if (g_take)    out_v <= 1'b0;
        // the stream
        case (st)
            ST_IDLE: if (ring_hs) begin
                st       <= aw_hs ? ST_DRAIN : ST_ACT;
                s_req    <= a_end;
                s_hi     <= a_ln;
                s_got    <= a_ln;
                g_rd     <= a_ln;
                s_id     <= s_axi.arid;
                s_cache  <= s_axi.arcache;
                s_lock   <= s_axi.arlock;
                s_prot   <= s_axi.arprot;
                s_qos    <= s_axi.arqos;
                s_region <= s_axi.arregion;
            end
            ST_ACT: begin
                if (ring_hs) s_req <= a_end;
                if (end_w || end_j || end_i) st <= ST_DRAIN;
            end
            default: if (s_lo == s_req) begin
                st    <= ST_IDLE;
                n_old <= n_old + (s_hi - s_got) - f_hs;
                s_got <= s_hi;
            end
        endcase
    end
end

// the ring: written in line order by fill beats, read in line order into ring_q
(* ram_style = "block" *) logic [AXI_DATA_BITS+1:0] ring [RA_RING];
logic [AXI_DATA_BITS+1:0] ring_q;
always_ff @(posedge aclk) begin
    if (f_wr)   ring[s_got[RB-1:0]] <= {m_axi.rresp, m_axi.rdata};
    if (g_load) ring_q <= ring[g_rd[RB-1:0]];
end

// read-ahead counters: words 40-49
localparam integer N_ACTR = 10;
logic [63:0] actr [N_ACTR];
always_ff @(posedge aclk) begin
    if (!aresetn) begin
        for (int i = 0; i < N_ACTR; i++) actr[i] <= '0;
    end else begin
        actr[0] <= actr[0] + (ring_hs && a_hit);
        actr[1] <= actr[1] + (ring_hs && !a_hit);
        actr[2] <= actr[2] + (ar_hs && !ar_ring && ra_on && !ar_ctr);
        actr[3] <= actr[3] + (mar_hs && fl_v);
        actr[4] <= actr[4] + f_hs;
        actr[5] <= actr[5] + g_take;
        actr[6] <= actr[6] + (ring_hs && (st == ST_IDLE));
        actr[7] <= actr[7] + end_j;
        actr[8] <= actr[8] + end_w;
        actr[9] <= actr[9] + end_i;
    end
end

// A counter-page read's beat: line rq_line + r_beat, eight counters per line
wire [5:0] r_line = rq_line[rp] + r_beat[5:0];
logic [AXI_DATA_BITS-1:0] ctr_data;
always_comb begin
    ctr_data = '0;
    for (int j = 0; j < AXI_DATA_BITS/64; j++)
        if (8*r_line + j < N_CTR) ctr_data[64*j +: 64] = ctr[8*r_line + j];
        else if (8*r_line + j >= 16 && 8*r_line + j < 16 + N_MAIN)
            ctr_data[64*j +: 64] = main_ctr[64*(8*r_line + j - 16) +: 64];
        else if (8*r_line + j >= 32 && 8*r_line + j < 32 + N_RCTR)
            ctr_data[64*j +: 64] = rctr[8*r_line + j - 32];
        else if (8*r_line + j >= 40 && 8*r_line + j < 40 + N_ACTR)
            ctr_data[64*j +: 64] = actr[8*r_line + j - 40];
        else if (8*r_line + j >= 64 && 8*r_line + j < 64 + N_XRA)
            ctr_data[64*j +: 64] = main_ctr[64*(N_MAIN + 8*r_line + j - 64) +: 64];
end

assign s_axi.rdata    = hd_ring ? ring_q[AXI_DATA_BITS-1:0] : rq_ctr[rp] ? ctr_data : m_axi.rdata;
assign s_axi.rid      = hd_ring ? s_id : m_axi.rid;
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
            rq_ring[rq_wp[$clog2(N_RD)-1:0]] <= ar_ring;
            rq_line[rq_wp[$clog2(N_RD)-1:0]] <= s_axi.araddr[11:6];
            rq_wp <= rq_wp + 1'b1;
        end
        if (r_hs) begin
            if (r_last) begin
                r_beat <= '0;
                rq_rp  <= rq_rp + 1'b1;
            end else begin
                r_beat <= r_beat + 1'b1;
            end
        end
    end
end
assign s_axi.rresp    = hd_ring ? ring_q[AXI_DATA_BITS +: 2] : m_axi.rresp;
assign s_axi.rvalid   = hd_ring ? out_v : (m_axi.rvalid && !hd_fill);
assign m_axi.rready   = hd_fill || (s_axi.rready && !hd_ring);

endmodule
