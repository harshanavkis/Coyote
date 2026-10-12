import lynxTypes::*;

/**
 * uwin_ra (EN_UWIN_HBM)
 *
 * Read-ahead for peers' sequential reads of the user data window, on the
 * shell's axi_main port (static -> shell, xclk), before the shell's
 * clock-crossing interconnect. A peer reading V80 memory through the window
 * (an RDMA NIC fetching a message: 256 B reads) has at most 8 reads in flight
 * at axi_main - the limit is upstream, in the static - so its rate is
 * 8 x 256 B / (a read's time here + ~110 ns upstream). Answered behind the
 * interconnect (uwin_hbm's read-ahead, aclk) a read still spends ~180 ns here,
 * ~163 ns of it crossing to aclk and back; answered here, ~20 ns.
 *
 * axi_main carries the window (WIN_BASE, BAR4 offset 0x0800_0000: 2^UWIN_BITS
 * bytes, its last 4 KiB the counter page, answered by uwin_hbm) and the shell's
 * registers below it. Only reads of the window outside the counter page are
 * ever read ahead or answered here; everything else passes through unchanged.
 * AW and B are always wires; W is held W_LAG cycles behind its AW (below).
 *
 * W BEHIND AW (W_LAG, writes into the V80): the shell's SmartConnect takes
 * 4-beat bursts (a peer's 256 B writes) at one per 6-7 cycles when a burst's
 * AW reaches it with, or one cycle before, its first W beat - as the static
 * presents them - and at one per 4 cycles (every cycle a beat) once the AW is
 * 3 or more cycles ahead (simulated, the generated IP: 6.75 / 6.00 / 4.13 /
 * 4.00 cycles a burst at 0 / 1 / 2 / 3 ahead). W beats wait in a FIFO here
 * until their burst's AW was taken by m_axi W_LAG cycles ago, then follow it,
 * one path, in order (+W_LAG cycles a write). The static never waits for W
 * before AW (AXI), and the SmartConnect takes AW without W, so holding W
 * cannot deadlock. W_LAG 0: W is wires too.
 *
 * ONE ID, ONE ORDER: the Versal static leaves axi_main's IDs unconnected
 * (static_top: arid/awid not driven, rid/bid not used), so every read is one
 * ID and the static takes R in AR order - of all reads, the registers' too.
 * R leaves here in AR order, every read alike. This module's own ARs (fills)
 * carry the stream's ID, so the interconnect (one ID: in order across its
 * targets) answers m_axi in m_axi's AR order. Word 12 counts ARs with a
 * nonzero ARID (0 expected).
 *
 * MODES: off at reset; a read of the counter page's line 59 (window offset
 * UWIN_SIZE - 4096 + 59*64) turns read-ahead on, line 58 off (word 0 says
 * which). Lines 61/60 are uwin_hbm's own read-ahead, behind the interconnect:
 * keep that one off while this one is on (it would read ahead of the fills).
 *   PASS (off): every AR and R signal is a wire, exactly as without this
 *   module; it only watches (control reads, the previous AR, writes
 *   outstanding, reads outstanding: nb).
 *   TRACK (on): ARs and R go through two-entry skid buffers, into and out of
 *   the module both ways, so every path here is register to register at
 *   333 MHz: AR in (the stream compare against the previous AR is made on the
 *   way in), the decision, AR out; R in, the ring or the pass-through, R out.
 *   A read that passes through pays 4 cycles for it.
 *   PASS -> TRACK once on and no AR or R is on the wires untaken (the switch
 *   never drops a valid); reads still outstanding from PASS are answered
 *   first (nb). TRACK -> PASS once off and nothing is outstanding (new ARs
 *   are held back meanwhile).
 *
 * READ-AHEAD (TRACK), uwin_hbm's (see uwin_hbm.sv), at this port:
 *   Stream: an AR continues the previous one if it starts at the line where
 *   that one ended (same ID, 64 B beats, INCR, both window reads outside the
 *   counter page). With read-ahead on and no write outstanding (every AW taken
 *   has had its B, registers' included), the first such AR starts a stream.
 *   Every read of the stream is answered from a ring of RA_RING lines on chip
 *   (BRAM), which this module fills with its own ARs (through the interconnect,
 *   uwin_hbm and HBM; the stream's ID; up to RA_BURST lines, never across an
 *   RA_BURST-line boundary) up to RA_LINES lines beyond the stream's last
 *   request.
 *   End: an AR that does not continue the stream (a jump, a register read, a
 *   counter-page read - the switch among them), a write to the window (its
 *   AW), or RA_IDLE cycles without an AR ends it (DRAIN): reads ahead stop
 *   beyond the stream's accepted reads, those are answered from the ring,
 *   later reads pass through; once they are answered the ring is dropped
 *   (beats of fills still in flight are discarded: n_old) and a stream may
 *   start again. A read passed through right after an end waits behind the
 *   fills still in flight (one ID), up to RA_LINES lines.
 *   Order: R leaves in the peer's AR order (rq: ring or pass-through per read;
 *   the ring makes its reads' rlast). m_axi's R comes in m_axi's AR order (mq:
 *   a fill, or a peer's read): a fill's beats go into the ring at once (room
 *   was reserved when it was issued), a peer read's beats wait until it is
 *   the head of rq. No deadlock: a peer read goes to m_axi only once fills
 *   cover every line of the stream's accepted reads (in DRAIN), so the fills
 *   a stream read waits for are never behind a later read's data. A stream's
 *   lines are filled in order and come back in order, so the ring is a FIFO
 *   indexed by line mod RA_RING (one write port, one read port with its output
 *   register: BRAM), no address lookup.
 *   Staleness: the ring answers only reads of the stream it was filled for,
 *   all accepted before the stream ended. A window write's AW ends the stream
 *   (one cycle later: the read taken in that cycle was issued before the
 *   write's B, so either data is right for it), and a new one starts only
 *   after the write's B, so a read issued after a window write completed
 *   never sees data fetched before it. The card path (the copy engine writes
 *   HBM directly) is invisible here, as for uwin_hbm: a stream read may return
 *   a line fetched up to RA_LINES lines (16 KiB) ahead of the stream's
 *   requests; a peer that learns of such a write other than through the
 *   window must break the stream first (a non-sequential read), wait RA_IDLE
 *   cycles, or turn read-ahead off.
 *
 * COUNTERS (xclk, free-running; uwin_mon snapshots them with its own and
 * uwin_hbm shows them on the counter page as words 64-76, 0-12 here):
 *   0 the mode: 1 while on           7 streams started
 *   1 stream reads with every line   8 streams ended by an AR that does not
 *     already requested (hits)         continue them (a jump, a register or
 *   2 stream reads that waited for a   counter-page read)
 *     line not yet requested (late)  9 ... by a write to the window (AW)
 *   3 window reads passed through    10 ... by RA_IDLE cycles without an AR
 *     while on (not part of a stream) 11 sum of read-ahead ARs outstanding over
 *   4 read-ahead ARs (taken by m_axi)    cycles (m_axi took the AR, its last
 *   5 lines read ahead (- 6 =            beat not yet in; / 4 = mean cycles)
 *     fetched, unused)               12 ARs with a nonzero ARID (any address)
 *   6 lines answered from the ring
 */
module uwin_ra #(
    parameter logic [63:0] WIN_BASE  = 64'h0800_0000,   // the window on axi_main (BAR4 offset, cr_ctrl.tcl)
    parameter integer      UWIN_BITS = 27,              // its size, 2^UWIN_BITS bytes (uwin_hbm's)
    parameter integer      N_RD      = 64,              // reads tracked: the peer's here, and ARs toward the interconnect; power of 2
    parameter integer      RA_LINES  = 256,             // read-ahead: lines (64 B) beyond the stream's requests (16 KiB)
    parameter integer      RA_BURST  = 16,              // lines per read-ahead AR (1 KiB); 2..16 (axi_main's MAX_BURST_LENGTH), power of 2
    parameter integer      RA_RING   = 512,             // the ring, lines (32 KiB); power of 2, >= 256 + RA_BURST (a peer
                                                        // burst), >= RA_LINES + the peer's reads in flight to keep ahead
    parameter integer      RA_IDLE   = 1024,            // xclk cycles without an AR that end a stream (3.1 us at 333 MHz)
    parameter integer      W_LAG     = 3,               // xclk cycles a burst's first W beat follows its AW at m_axi (0: wires)
    parameter integer      WQ        = 8                // W beats held here (power of 2)
) (
    input  logic                xclk,
    input  logic                xresetn,

    AXI4.s                      s_axi,                  // the static's axi_main: the peer side
    AXI4.m                      m_axi,                  // into the shell's interconnect

    output logic [64*13-1:0]    ctr                     // the counters above, for uwin_mon's snapshot
);

localparam integer LB = UWIN_BITS - 6;                  // lines: window offset / 64, one bit of headroom
localparam integer RB = $clog2(RA_RING);
localparam integer QB = $clog2(N_RD);
localparam logic [LB:0] CTR_LN = (LB+1)'(2**LB - 64);   // the counter page's first line
localparam logic [63-UWIN_BITS:0] WIN_TAG = WIN_BASE[63:UWIN_BITS];
typedef enum logic [1:0] {ST_IDLE, ST_ACT, ST_DRAIN} ra_st_t;

typedef struct packed {
    logic [63:0]            addr;
    logic [AXI_ID_BITS-1:0] id;
    logic [7:0]             len;
    logic [2:0]             size;
    logic [1:0]             burst;
    logic [3:0]             cache;
    logic [0:0]             lock;
    logic [2:0]             prot;
    logic [3:0]             qos;
    logic [3:0]             region;
} ar_t;
typedef struct packed {                                 // an AR as the way in saw it
    ar_t         ar;
    logic [LB:0] ln;                                    // its first line
    logic [LB:0] fin;                                   // the line after its last
    logic        seq;                                   // it continues the previous AR
    logic        wd;                                    // a window read outside the counter page
} arq_t;
typedef struct packed {                                 // an AR to m_axi
    ar_t  ar;
    logic fill;
} aro_t;
typedef struct packed {
    logic [AXI_DATA_BITS-1:0] data;
    logic [1:0]               resp;
    logic [AXI_ID_BITS-1:0]   id;
    logic                     last;
} r_t;

logic                   trk;                // TRACK (else PASS: AR and R are wires)
logic                   ra_on;

// THE WAY IN: what an AR is, and whether it continues the previous one (the
// previous AR taken at s_axi, in either mode)
logic [LB:0]            p_next;             // the previous AR's end
logic                   p_ok;               // ... it could be continued
logic [AXI_ID_BITS-1:0] p_id;
wire        a_win  = (s_axi.araddr[63:UWIN_BITS] == WIN_TAG);
wire        a_ctr  = a_win && (s_axi.araddr[UWIN_BITS-1:12] == '1);
wire [LB:0] a_ln   = (LB+1)'(s_axi.araddr[UWIN_BITS-1:6]);
wire [LB:0] a_fin  = a_ln + (LB+1)'(s_axi.arlen) + 1'b1;
wire        a_cont = a_win && !a_ctr && (s_axi.arsize == 3'd6) && (s_axi.arburst == 2'b01);
wire        a_seq  = p_ok && a_cont && (a_ln == p_next) && (s_axi.arid == p_id);
wire        s_ar_hs = s_axi.arvalid && s_axi.arready;

arq_t in_d, q;
logic in_room, q_v, pop_q;
always_comb begin
    in_d.ar.addr   = s_axi.araddr;
    in_d.ar.id     = s_axi.arid;
    in_d.ar.len    = s_axi.arlen;
    in_d.ar.size   = s_axi.arsize;
    in_d.ar.burst  = s_axi.arburst;
    in_d.ar.cache  = s_axi.arcache;
    in_d.ar.lock   = s_axi.arlock;
    in_d.ar.prot   = s_axi.arprot;
    in_d.ar.qos    = s_axi.arqos;
    in_d.ar.region = s_axi.arregion;
    in_d.ln        = a_ln;
    in_d.fin       = a_fin;
    in_d.seq       = a_seq;
    in_d.wd        = a_win && !a_ctr;
end
uwin_ra_skid #(.W($bits(arq_t))) inst_ar_in (.clk(xclk), .rstn(xresetn),
    .s_valid(s_axi.arvalid && trk && ra_on), .s_ready(in_room), .s_data(in_d),
    .m_valid(q_v), .m_ready(pop_q), .m_data(q));
assign s_axi.arready = trk ? (ra_on && in_room) : m_axi.arready;

// AR to m_axi: a fill, else the head peer read
aro_t o_d, o_q;
logic o_room, o_v, o_push;
uwin_ra_skid #(.W($bits(aro_t))) inst_ar_out (.clk(xclk), .rstn(xresetn),
    .s_valid(o_push), .s_ready(o_room), .s_data(o_d),
    .m_valid(o_v), .m_ready(m_axi.arready), .m_data(o_q));
assign m_axi.araddr   = trk ? o_q.ar.addr   : s_axi.araddr;
assign m_axi.arid     = trk ? o_q.ar.id     : s_axi.arid;
assign m_axi.arlen    = trk ? o_q.ar.len    : s_axi.arlen;
assign m_axi.arsize   = trk ? o_q.ar.size   : s_axi.arsize;
assign m_axi.arburst  = trk ? o_q.ar.burst  : s_axi.arburst;
assign m_axi.arcache  = trk ? o_q.ar.cache  : s_axi.arcache;
assign m_axi.arlock   = trk ? o_q.ar.lock   : s_axi.arlock;
assign m_axi.arprot   = trk ? o_q.ar.prot   : s_axi.arprot;
assign m_axi.arqos    = trk ? o_q.ar.qos    : s_axi.arqos;
assign m_axi.arregion = trk ? o_q.ar.region : s_axi.arregion;
assign m_axi.arvalid  = trk ? o_v           : s_axi.arvalid;

// R from m_axi, R to s_axi
r_t   ri_d, ri, ro_d, ro;
logic ri_room, ri_v, ri_pop, ro_room, ro_v, ro_push;
assign ri_d = '{data: m_axi.rdata, resp: m_axi.rresp, id: m_axi.rid, last: m_axi.rlast};
uwin_ra_skid #(.W($bits(r_t))) inst_r_in (.clk(xclk), .rstn(xresetn),
    .s_valid(m_axi.rvalid && trk), .s_ready(ri_room), .s_data(ri_d),
    .m_valid(ri_v), .m_ready(ri_pop), .m_data(ri));
uwin_ra_skid #(.W($bits(r_t))) inst_r_out (.clk(xclk), .rstn(xresetn),
    .s_valid(ro_push), .s_ready(ro_room), .s_data(ro_d),
    .m_valid(ro_v), .m_ready(s_axi.rready), .m_data(ro));
assign s_axi.rdata    = trk ? ro.data : m_axi.rdata;
assign s_axi.rresp    = trk ? ro.resp : m_axi.rresp;
assign s_axi.rid      = trk ? ro.id   : m_axi.rid;
assign s_axi.rlast    = trk ? ro.last : m_axi.rlast;
assign s_axi.rvalid   = trk ? ro_v    : m_axi.rvalid;
assign m_axi.rready   = trk ? ri_room : s_axi.rready;

// AW, B: wires; W below
assign m_axi.awaddr   = s_axi.awaddr;
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
generate if (W_LAG == 0) begin : g_w_wires
assign m_axi.wdata    = s_axi.wdata;
assign m_axi.wlast    = s_axi.wlast;
assign m_axi.wstrb    = s_axi.wstrb;
assign m_axi.wvalid   = s_axi.wvalid;
assign s_axi.wready   = m_axi.wready;
end else begin : g_w_lag
// W beats in arrival order: a FIFO, then a skid buffer (registered outputs)
typedef struct packed {
    logic [AXI_DATA_BITS-1:0]   data;
    logic [AXI_DATA_BITS/8-1:0] strb;
    logic                       last;
} w_t;
localparam integer WB = $clog2(WQ);
w_t             wq_mem [WQ];
logic [WB-1:0]  wq_wp, wq_rp;
logic [WB:0]    wq_n;
logic           wo_room, wo_v, w_go;
w_t             wo;
wire wq_push = s_axi.wvalid && s_axi.wready;
wire wq_pop  = (wq_n != '0) && wo_room;
assign s_axi.wready = (wq_n != (WB+1)'(WQ));
always_ff @(posedge xclk) begin
    if (!xresetn) begin
        wq_wp <= '0;
        wq_rp <= '0;
        wq_n  <= '0;
    end else begin
        if (wq_push) wq_wp <= wq_wp + 1'b1;
        if (wq_pop)  wq_rp <= wq_rp + 1'b1;
        wq_n <= wq_n + (wq_push ? 1'b1 : 1'b0) - (wq_pop ? 1'b1 : 1'b0);
    end
end
always_ff @(posedge xclk) if (wq_push) wq_mem[wq_wp] <= '{data: s_axi.wdata, strb: s_axi.wstrb, last: s_axi.wlast};
uwin_ra_skid #(.W($bits(w_t))) inst_w_out (.clk(xclk), .rstn(xresetn),
    .s_valid(wq_n != '0), .s_ready(wo_room), .s_data(wq_mem[wq_rp]),
    .m_valid(wo_v), .m_ready(m_axi.wready && w_go), .m_data(wo));

// A burst's first beat goes once its AW was taken W_LAG cycles ago: aw_d
// delays m_axi's AW handshakes, w_cr counts those not yet followed by a
// first beat, w_mid is set inside a burst (its first beat gone)
logic [W_LAG-1:0] aw_d;
logic [7:0]       w_cr;
logic             w_mid;
wire w_hs    = m_axi.wvalid && m_axi.wready;
wire w_first = w_hs && !w_mid;
assign w_go  = w_mid || (w_cr != '0) || aw_d[W_LAG-1];
always_ff @(posedge xclk) begin
    if (!xresetn) begin
        aw_d  <= '0;
        w_cr  <= '0;
        w_mid <= 1'b0;
    end else begin
        aw_d  <= W_LAG'({aw_d, m_axi.awvalid && m_axi.awready});
        w_cr  <= w_cr + (aw_d[W_LAG-1] ? 8'd1 : 8'd0) - (w_first ? 8'd1 : 8'd0);
        if (w_hs) w_mid <= !wo.last;
    end
end
assign m_axi.wdata  = wo.data;
assign m_axi.wstrb  = wo.strb;
assign m_axi.wlast  = wo.last;
assign m_axi.wvalid = wo_v && w_go;
end endgenerate
assign s_axi.bid      = m_axi.bid;
assign s_axi.bresp    = m_axi.bresp;
assign s_axi.bvalid   = m_axi.bvalid;
assign m_axi.bready   = s_axi.bready;

// writes outstanding (any address), and a window write taken (last cycle)
logic [15:0] wr_out;
logic        wr_zero;                       // wr_out == 0
logic        aw_ev;
wire aw_hs = s_axi.awvalid && s_axi.awready;
wire b_hs  = s_axi.bvalid && s_axi.bready;
wire [15:0] wr_nx = wr_out + (aw_hs ? 16'd1 : 16'd0) - (b_hs ? 16'd1 : 16'd0);

// rq: the peer's reads in AR order (TRACK; the ring's or passed through, a
// ring read's length); mq: m_axi's ARs in order (a fill, or a peer's read).
// Their heads are registers
logic          rq_ring [N_RD];
logic [7:0]    rq_len [N_RD];
logic [QB-1:0] rq_wp, rq_rp;
logic [QB:0]   rq_n;
logic          rq_hv, rq_hr;
logic [7:0]    rq_hl;
wire           rq_full = (rq_n == (QB+1)'(N_RD));
logic          mq_fill [N_RD];
logic [QB-1:0] mq_wp, mq_rp;
logic [QB:0]   mq_n;
logic          mq_hv, mq_hf;
wire           mq_full = (mq_n == (QB+1)'(N_RD));
logic [15:0]   nb;                          // reads taken in PASS, last R not yet (older than rq's)
logic          nb_nz;                       // nb != 0

// READ-AHEAD state
ra_st_t                 st;
logic [LB:0]            s_req;              // the end of the stream's accepted reads
logic [LB:0]            s_hi;               // the end of its fills issued
logic [LB:0]            s_got;              // ... returned (written into the ring)
logic [LB:0]            g_rd;               // the next line read out of the ring
logic [LB:0]            n_old;              // fill beats still to come for a dropped ring: discarded
logic                   v1, v2;             // the ring's read: the BRAM's latch, its output register (ring_q); lines below g_rd
logic [AXI_ID_BITS-1:0] s_id;               // the stream's ID and attributes (its first read's)
logic [3:0]             s_cache, s_qos, s_region;
logic [2:0]             s_prot;
logic [0:0]             s_lock;
logic                   fl_v;               // a fill, issued (s_hi past it), not yet in the AR buffer
logic [LB:0]            fl_ln;
logic [7:0]             fl_len;
logic [$clog2(RA_IDLE):0] idle;
logic                   idle_to;            // RA_IDLE cycles in ACT without an AR
logic                   dcov;               // last cycle: draining, fills covered the accepted reads
logic [7:0]             r_beat;             // the head ring read's beats answered
logic [15:0]            fo;                 // read-ahead ARs taken by m_axi, last beat not yet in

// the head AR (q): the ring's, or passed through. A read the ring does not
// answer goes to m_axi only with no fill waiting and, while the stream drains,
// once fills cover its accepted reads (dcov: a cycle late, never early - in
// DRAIN s_req is fixed and s_hi only grows)
wire ar_ring = q_v && q.seq && (((st == ST_ACT) && !idle_to) || ((st == ST_IDLE) && ra_on && wr_zero));
wire pass_ok = (st == ST_IDLE) || ((st == ST_DRAIN) && dcov);
wire ring_hs = ar_ring && !rq_full;
wire m_peer  = q_v && !ar_ring && pass_ok && !fl_v && o_room && !rq_full && !mq_full;
assign pop_q = ring_hs || m_peer;
wire fl_push = fl_v && o_room;
assign o_push = fl_push || m_peer;
always_comb begin
    o_d.ar = q.ar;
    o_d.fill = fl_v;
    if (fl_v) begin
        o_d.ar.addr   = {WIN_TAG, fl_ln[LB-1:0], 6'b0};
        o_d.ar.id     = s_id;
        o_d.ar.len    = fl_len;
        o_d.ar.size   = 3'd6;
        o_d.ar.burst  = 2'b01;
        o_d.ar.cache  = s_cache;
        o_d.ar.lock   = s_lock;
        o_d.ar.prot   = s_prot;
        o_d.ar.qos    = s_qos;
        o_d.ar.region = s_region;
    end
end
wire a_hit = (st == ST_ACT) && (q.fin <= s_hi);         // every line already requested
// the stream ends: a window write, RA_IDLE cycles without an AR, an AR that does not continue it
wire end_w = (st == ST_ACT) && aw_ev;
wire end_i = (st == ST_ACT) && !aw_ev && idle_to;
wire end_j = (st == ST_ACT) && !aw_ev && !idle_to && q_v && !ar_ring;

// the next fill: up to the next RA_BURST boundary, RA_LINES beyond the
// stream's requests (draining: up to them), with room in the ring and in mq
// (never while a peer read may go to m_axi: pass_ok excludes it, so the mq
// slot checked here is still free when the fill goes)
wire [LB:0] f_n  = (LB+1)'(RA_BURST) - (LB+1)'(s_hi[$clog2(RA_BURST)-1:0]);
wire        f_go = !fl_v && !mq_full && (s_hi < CTR_LN) && (s_hi + f_n - g_rd <= (LB+1)'(RA_RING))
                && (((st == ST_ACT) && (s_hi < s_req + (LB+1)'(RA_LINES))) || ((st == ST_DRAIN) && (s_hi < s_req)));

// R: the head of R in is a fill's beat (into the ring, never held), else the
// head peer read's (nb's first); R out takes the ring's next line (ring_q)
// when rq's head is the ring's, else R in's beat
logic [AXI_DATA_BITS+1:0] ring_q;
wire hd_fill = !nb_nz && mq_hv && mq_hf;
wire hd_ring = !nb_nz && rq_hv && rq_hr;
wire f_hs    = ri_v && hd_fill;
wire f_wr    = f_hs && (n_old == 0);                    // ... into the ring at s_got
wire g_take  = hd_ring && v2 && ro_room;
wire p_take  = !hd_ring && !hd_fill && ri_v && ro_room;
wire r_lst   = (r_beat == rq_hl);
assign ri_pop  = f_hs || p_take;
assign ro_push = g_take || p_take;
assign ro_d    = hd_ring ? r_t'{data: ring_q[AXI_DATA_BITS-1:0], resp: ring_q[AXI_DATA_BITS +: 2], id: s_id, last: r_lst} : ri;
wire rq_push = ring_hs || m_peer;
wire rq_pop  = (g_take && r_lst) || (p_take && !nb_nz && ri.last);
wire mq_pop  = (f_hs && ri.last) || (p_take && !nb_nz && ri.last);
wire nb_push = !trk && s_ar_hs;
wire nb_pop  = trk ? (p_take && nb_nz && ri.last) : (m_axi.rvalid && m_axi.rready && m_axi.rlast);
wire mo_fill = trk && o_v && o_q.fill && m_axi.arready;  // m_axi takes a fill
// the ring's read: BRAM, then its output register (ring_q)
wire s2_load = v1 && (!v2 || g_take);
wire s1_load = (st != ST_IDLE) && (g_rd != s_got) && (!v1 || s2_load);
wire [LB:0] s_lo = g_rd - v1 - v2;                      // the next line the stream's reads take
// nothing in flight, nothing buffered: TRACK may become PASS
wire quiet = (st == ST_IDLE) && !fl_v && (rq_n == 0) && (mq_n == 0) && (nb == 0) && !q_v && !o_v && !ri_v && !ro_v;

always_ff @(posedge xclk) begin
    if (rq_push) begin
        rq_ring[rq_wp] <= ring_hs;
        rq_len[rq_wp]  <= q.ar.len;
    end
    if (o_push) mq_fill[mq_wp] <= fl_v;
end

always_ff @(posedge xclk) begin
    if (!xresetn) begin
        trk     <= 1'b0;
        ra_on   <= 1'b0;
        p_next  <= '0;
        p_ok    <= 1'b0;
        p_id    <= '0;
        wr_out  <= '0;
        wr_zero <= 1'b1;
        aw_ev   <= 1'b0;
        rq_wp   <= '0;
        rq_rp   <= '0;
        rq_n    <= '0;
        rq_hv   <= 1'b0;
        mq_wp   <= '0;
        mq_rp   <= '0;
        mq_n    <= '0;
        mq_hv   <= 1'b0;
        nb      <= '0;
        nb_nz   <= 1'b0;
        st      <= ST_IDLE;
        s_req   <= '0;
        s_hi    <= '0;
        s_got   <= '0;
        g_rd    <= '0;
        n_old   <= '0;
        v1      <= 1'b0;
        v2      <= 1'b0;
        fl_v    <= 1'b0;
        idle    <= '0;
        idle_to <= 1'b0;
        dcov    <= 1'b0;
        r_beat  <= '0;
        fo      <= '0;
    end else begin
        // the way in; the switch (counter page line 59 on, 58 off)
        if (s_ar_hs) begin
            p_next <= a_fin;
            p_ok   <= a_cont;
            p_id   <= s_axi.arid;
            if (a_ctr && (s_axi.araddr[11:7] == 5'h1D)) ra_on <= s_axi.araddr[6];
        end
        if (!trk) begin
            if (ra_on && !(s_axi.arvalid && !m_axi.arready) && !(m_axi.rvalid && !s_axi.rready)) trk <= 1'b1;
        end else if (!ra_on && quiet) begin
            trk <= 1'b0;
        end
        wr_out  <= wr_nx;
        wr_zero <= (wr_nx == 0);
        aw_ev   <= aw_hs && (s_axi.awaddr[63:UWIN_BITS] == WIN_TAG);
        // rq, mq, nb
        rq_wp <= rq_wp + rq_push;
        rq_rp <= rq_rp + rq_pop;
        rq_n  <= rq_n + rq_push - rq_pop;
        if (rq_pop) begin
            if (rq_n > 1) begin
                rq_hr <= rq_ring[rq_rp + 1'b1];
                rq_hl <= rq_len[rq_rp + 1'b1];
            end else if (rq_push) begin
                rq_hr <= ring_hs;
                rq_hl <= q.ar.len;
            end else begin
                rq_hv <= 1'b0;
            end
        end else if (!rq_hv && rq_push) begin
            rq_hv <= 1'b1;
            rq_hr <= ring_hs;
            rq_hl <= q.ar.len;
        end
        mq_wp <= mq_wp + o_push;
        mq_rp <= mq_rp + mq_pop;
        mq_n  <= mq_n + o_push - mq_pop;
        if (mq_pop) begin
            if (mq_n > 1)    mq_hf <= mq_fill[mq_rp + 1'b1];
            else if (o_push) mq_hf <= fl_v;
            else             mq_hv <= 1'b0;
        end else if (!mq_hv && o_push) begin
            mq_hv <= 1'b1;
            mq_hf <= fl_v;
        end
        nb    <= nb + nb_push - nb_pop;
        nb_nz <= nb_push || (nb > 16'd1) || (nb_nz && !nb_pop);
        if (g_take) r_beat <= r_lst ? 8'd0 : r_beat + 1'b1;
        fo    <= fo + mo_fill - (f_hs && ri.last);
        idle    <= ((st != ST_ACT) || pop_q) ? '0 : idle + !idle_to;
        idle_to <= (st == ST_ACT) && !pop_q && (idle_to || (idle == ($clog2(RA_IDLE)+1)'(RA_IDLE - 1)));
        dcov    <= (st == ST_DRAIN) && (s_hi >= s_req);
        // fills: issued (s_hi), into the AR buffer, their beats into the ring (s_got)
        if (f_go) begin
            fl_v   <= 1'b1;
            fl_ln  <= s_hi;
            fl_len <= 8'(f_n - 1'b1);
            s_hi   <= s_hi + f_n;
        end else if (fl_push) begin
            fl_v   <= 1'b0;
        end
        if (f_hs) begin
            if (f_wr) s_got <= s_got + 1'b1;
            else      n_old <= n_old - 1'b1;
        end
        // the ring's read
        if (s1_load) g_rd <= g_rd + 1'b1;
        if (st == ST_IDLE) begin
            v1 <= 1'b0;
            v2 <= 1'b0;
        end else begin
            if (s1_load)      v1 <= 1'b1;
            else if (s2_load) v1 <= 1'b0;
            if (s2_load)      v2 <= 1'b1;
            else if (g_take)  v2 <= 1'b0;
        end
        // the stream
        case (st)
            ST_IDLE: if (ring_hs) begin
                st       <= ST_ACT;
                s_req    <= q.fin;
                s_hi     <= q.ln;
                s_got    <= q.ln;
                g_rd     <= q.ln;
                s_id     <= q.ar.id;
                s_cache  <= q.ar.cache;
                s_lock   <= q.ar.lock;
                s_prot   <= q.ar.prot;
                s_qos    <= q.ar.qos;
                s_region <= q.ar.region;
            end
            ST_ACT: begin
                if (ring_hs) s_req <= q.fin;
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

// the ring: written in line order by fill beats, read in line order through
// the BRAM's output register into ring_q
(* ram_style = "block" *) logic [AXI_DATA_BITS+1:0] ring [RA_RING];
logic [AXI_DATA_BITS+1:0] ring_a;
always_ff @(posedge xclk) begin
    if (f_wr)    ring[s_got[RB-1:0]] <= {ri.resp, ri.data};
    if (s1_load) ring_a <= ring[g_rd[RB-1:0]];
    if (s2_load) ring_q <= ring_a;
end

// counters (the decision's events a cycle late: off its paths)
localparam integer N_CTR = 13;
logic [63:0] c [N_CTR];
logic [6:0]  ev;
always_ff @(posedge xclk) begin
    if (!xresetn) begin
        for (int i = 0; i < N_CTR; i++) c[i] <= '0;
        ev <= '0;
    end else begin
        ev    <= {ring_hs && a_hit, ring_hs && !a_hit, m_peer && ra_on && q.wd, ring_hs && (st == ST_IDLE), end_j, end_w, end_i};
        c[0]  <= 64'(ra_on);
        c[1]  <= c[1] + ev[6];
        c[2]  <= c[2] + ev[5];
        c[3]  <= c[3] + ev[4];
        c[4]  <= c[4] + mo_fill;
        c[5]  <= c[5] + f_hs;
        c[6]  <= c[6] + g_take;
        c[7]  <= c[7] + ev[3];
        c[8]  <= c[8] + ev[2];
        c[9]  <= c[9] + ev[1];
        c[10] <= c[10] + ev[0];
        c[11] <= c[11] + fo;
        c[12] <= c[12] + (s_ar_hs && (s_axi.arid != '0));
    end
end
always_comb for (int i = 0; i < N_CTR; i++) ctr[64*i +: 64] = c[i];

endmodule

/**
 * Two-entry skid buffer: both ways registered (s_ready is a flip-flop), one
 * transfer per cycle.
 */
module uwin_ra_skid #(
    parameter integer W = 8
) (
    input  logic            clk,
    input  logic            rstn,
    input  logic            s_valid,
    output logic            s_ready,
    input  logic [W-1:0]    s_data,
    output logic            m_valid,
    input  logic            m_ready,
    output logic [W-1:0]    m_data
);

logic [W-1:0] b;
logic         bv;
assign s_ready = !bv;

always_ff @(posedge clk) begin
    if (!rstn) begin
        m_valid <= 1'b0;
        bv      <= 1'b0;
    end else if (!m_valid || m_ready) begin
        m_valid <= bv || s_valid;
        bv      <= 1'b0;
    end else if (s_valid && !bv) begin
        bv      <= 1'b1;
    end
end
always_ff @(posedge clk) begin
    if (!m_valid || m_ready) m_data <= bv ? b : s_data;
    if (m_valid && !m_ready && !bv) b <= s_data;
end

endmodule
