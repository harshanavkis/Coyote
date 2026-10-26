import lynxTypes::*;

/**
 * loom_read (loom_switch)
 *
 * Reads on the ingress window (uwin): the AR and R channels of axi_udata.
 * Whatever reads the window - a CPU load through the write-combining
 * mapping, a copy engine's DMA read peer-to-peer - reads the bytes the
 * window is bound to, and this switch does the rest. A read of an rdma
 * window is a get from the far export reference base + offset:
 *
 *   1. AR: take a read slot (N_SLOTS, a ring; R answers in AR order), look
 *      the address up (loom_table ub_*), and work out the 64 B lines the
 *      read's beats fall in: from the line of araddr to the line of its last
 *      beat (INCR or FIXED; a burst stays inside its 4 KiB page, so a read
 *      is at most 64 lines).
 *   2. Reads that continue each other share one get: a read of the same
 *      window and page that starts at the byte where the previous one ended
 *      joins the open get; the get is sent when a read that does not
 *      continue it arrives, when it reaches the end of its page (so at most
 *      4 KiB), or after CO_WAIT cycles with no read joining. Nothing is
 *      fetched before a read asks for it. A get is a request through
 *      loom_ingress (s_get), in its queue order: ref = the window's far
 *      reference + the offset of the first line, word = {lines [63:48],
 *      return reference [47:0]}, the return reference being its first
 *      read's slot: {RRET_EXP, slot * 8 KiB}.
 *   3. The far loom_rx / loom_rd answer as for any get: the lines as one
 *      rdma packet to return reference + offset (a get is at most 4 KiB, one
 *      PMTU), or, if the far side rejected the request, only the word, all
 *      ones, to return reference + len. This switch's loom_rx sees RRET_EXP
 *      and hands the lines to the region's buffer (s_land), and the end of
 *      the packet (or the rejection) to this module as the get's completion
 *      (s_cmp): 8 KiB per region in the reference space, so return
 *      reference + len (len <= 4 KiB) still names the region.
 *   4. R: when the head read's data is there (its get's, or its read-ahead
 *      block's), its beats go out, each the line its address falls in (the
 *      data on the lanes the address selects, as the AXI data bus carries
 *      it). A get's buffer is kept until its last read is answered: a slot
 *      is taken only when it is not the buffer of a read still waiting.
 *
 * READ-AHEAD (DESIGN.md 17). Reads that continue each other - each INCR read
 * starting at the byte where the previous one ended, in the same rdma window
 * - form a stream once they have covered a whole page and enter the next:
 * from that read on, the window is fetched ahead in 4 KiB blocks (one page
 * each) and the reads are answered from the blocks, so they no longer wait a
 * round trip each. A read of a page or less (all of it in one copy engine
 * burst) never starts one: it is fetched exactly, by the coalescer. The prefetcher starts
 * RA_D0 pages ahead of the page the reads are in and doubles that each time
 * the reads enter a new page, up to N_BLK pages (a short transfer fetches
 * little past its end; both bounds can be lowered at run time). A block fetched early can be older than a
 * read it serves. That is safe because readers synchronize through Loom
 * (DESIGN.md 6): the stream is dropped on anything through which a reader
 * could learn that the far data changed -
 *   - a read that does not continue it (another address, the same bytes
 *     again, another window, a read that fails),
 *   - a write landing on this host from any peer (s_drop_pkt / s_drop_store:
 *     loom_rx's writes to exports, not the answers to this switch's reads),
 *   - a write into the uwin from this host (s_drop_write).
 * A fetch issued at or after a drop is fresh: its request leaves after what
 * dropped the stream arrived. Reads attached to a block before a drop are
 * still answered from it (they were issued before the reader could have
 * seen what dropped it), and the blocks such reads need are still fetched.
 * A new stream starts only when the dropped one's blocks have drained and,
 * after a landing or a uwin write, RA_QUIET cycles have passed (so constant
 * drops, e.g. the peer's bulk puts landing, do not keep fetching blocks that
 * are thrown away); a transfer's first read, which drops the last
 * transfer's stream, does not keep its own from starting. Streams need a page-aligned window start in the uwin.
 *
 * (RD_CTL bit 9 starts a stream on the second consecutive read instead, as
 * built first.)
 *
 * Regions: slots 0..N_SLOTS-1 (a get's buffer, its lines from its first
 * line), then blocks N_SLOTS..N_SLOTS+N_BLK-1 (a page, line = the page
 * offset).
 *
 * A read that cannot be served - no window, a local window, past the
 * window's end, a burst wider than the bus or crossing its 4 KiB page - or
 * that the far side rejected, is answered OKAY with all ones (the value a
 * failed PCIe read returns), and counted (cnt_fail; cnt_far_err for the far
 * side's, once per get). Nothing times out: a read whose answer never comes
 * holds its slot, and the reads behind it, until a reset.
 */
module loom_read #(
    parameter integer UWIN_BITS = 27,
    parameter integer N_SLOTS   = 64,
    parameter integer CO_WAIT   = 32,       // cycles an open get waits for a continuing read
    parameter integer N_BLK     = 32,       // read-ahead blocks (pages), a power of two
    parameter integer RA_D0     = 4,        // pages fetched ahead when a stream starts
    parameter integer RA_QUIET  = 512,      // cycles with no drop before a stream may start
    parameter         BUF_MEM   = "ultra"
) (
    input  logic                        aclk,
    input  logic                        aresetn,

    // The uwin's read channels (axi_udata AR / R)
    input  logic [UWIN_BITS-1:0]        s_araddr,
    input  logic [7:0]                  s_arlen,
    input  logic [2:0]                  s_arsize,
    input  logic [1:0]                  s_arburst,
    input  logic [AXI_ID_BITS-1:0]      s_arid,
    input  logic                        s_arvalid,
    output logic                        s_arready,
    output logic [AXI_DATA_BITS-1:0]    s_rdata,
    output logic [1:0]                  s_rresp,
    output logic                        s_rlast,
    output logic [AXI_ID_BITS-1:0]      s_rid,
    output logic                        s_rvalid,
    input  logic                        s_rready,

    // Table lookup by uwin address (loom_table ub_* port, two stages)
    output logic                        ub_ce1,
    output logic                        ub_ce2,
    output logic [UWIN_BITS-1:0]        ub_addr,
    input  logic                        ub_hit,
    input  logic                        ub_route,
    input  logic [PID_BITS-1:0]         ub_pid,
    input  logic [VADDR_BITS-1:0]       ub_base,
    input  logic [UWIN_BITS-1:0]        ub_ustart,
    input  logic [LEN_BITS:0]           ub_end,

    // Get requests (to loom_ingress s_get)
    output logic                        m_get_valid,
    input  logic                        m_get_ready,
    output logic [PID_BITS-1:0]         m_get_pid,
    output logic [VADDR_BITS-1:0]       m_get_ref,
    output logic [63:0]                 m_get_word,

    // Answers (from loom_rx): lines into a region's buffer, completions
    input  logic                        s_land_valid,
    input  logic [$clog2(N_SLOTS+N_BLK)+5:0] s_land_line,
    input  logic [AXI_DATA_BITS-1:0]    s_land_data,
    input  logic                        s_cmp_valid,
    input  logic [$clog2(N_SLOTS+N_BLK)-1:0] s_cmp_slot,    // the region
    input  logic                        s_cmp_err,

    // Read-ahead off (CSR RD_CTL[8]): no stream starts, a running one is
    // dropped; the most pages fetched ahead and the pages ahead at a stream's
    // start (RD_CTL[21:16], [29:24]; 0 = N_BLK, RA_D0)
    input  logic                        ra_off,
    input  logic                        ra_early,      // RD_CTL[9]: start on the second consecutive read
    input  logic [5:0]                  ra_dmax,
    input  logic [5:0]                  ra_d0,
    // What drops a read-ahead stream, besides reads
    input  logic                        s_drop_store,  // an inline store landed on this host
    input  logic                        s_drop_pkt,    // a packet landed on this host
    input  logic                        s_drop_write,  // this host wrote into the uwin

    output logic                        busy,

    // Counter pulses
    output logic                        cnt_read,      // a read taken (AR)
    output logic                        cnt_done,      // a read answered (its last R beat)
    output logic                        cnt_fail,      // a read answered with all ones
    output logic                        cnt_far_err,   // a get the far side rejected
    output logic                        cnt_slot_wait, // cycles an AR waited for a free slot
    output logic                        cnt_stray,     // a completion for a region with no get out (must be 0)
    output logic                        cnt_ra_read,   // a read answered from read-ahead
    output logic                        cnt_ra_block,  // a read-ahead block requested
    output logic                        cnt_ra_start,  // a stream started
    output logic                        cnt_ra_drop_read,   // a stream dropped by a read
    output logic                        cnt_ra_drop_store,  // ... by an inline store landing
    output logic                        cnt_ra_drop_pkt,    // ... by a packet landing
    output logic                        cnt_ra_drop_write   // ... by a write into the uwin
);

localparam [7:0] RRET_EXP = 8'hFE;    // keep in sync with loom_rx.sv
localparam integer SLOT_W = $clog2(N_SLOTS);
localparam integer BLK_W  = $clog2(N_BLK);
localparam integer N_REG  = N_SLOTS + N_BLK;
localparam integer REG_W  = $clog2(N_REG);
localparam integer LINE_W = REG_W + 6;
localparam integer SK     = 4;         // R skid buffer
localparam integer UA     = UWIN_BITS + 1;              // uwin byte addresses, with a carry
localparam [UA-1:0] PAGE    = UA'(4096);
localparam [UA-1:0] RA_SPAN = UA'(N_BLK) << 12;         // the pages a stream may hold
localparam integer QW     = $clog2(RA_QUIET + 1);
localparam integer DW     = BLK_W + 1;

// ---------------------------------------------------------------------------
// Slots: [rp, wp) allocated, in AR order. done / err / sent are per get,
// at its first read's slot.
// ---------------------------------------------------------------------------
logic [SLOT_W:0]        wp, rp;
wire  [SLOT_W-1:0]      h = rp[SLOT_W-1:0];
logic [N_SLOTS-1:0]     done, err, sent;

// Per slot, what R needs
logic [AXI_ID_BITS-1:0] m_id   [N_SLOTS];
logic [7:0]             m_len  [N_SLOTS];
logic [2:0]             m_size [N_SLOTS];
logic                   m_fix  [N_SLOTS];
logic [11:0]            m_addr [N_SLOTS];
logic [SLOT_W-1:0]      m_get  [N_SLOTS];  // its get's first slot: the buffer its lines are in
logic [5:0]             m_l0   [N_SLOTS];  // that get's first line in the page
logic                   m_ra   [N_SLOTS];  // answered from a read-ahead block
logic [BLK_W-1:0]       m_blk  [N_SLOTS];

// The head read's get buffer and the slots after it stay taken. The reads
// out are counted in SLOT_W+1 bits on their own: wp - rp evaluated in the
// sum's width is wrong once wp has wrapped and rp has not.
wire  [SLOT_W-1:0]      hg     = m_get[h];
wire  [SLOT_W:0]        s_out  = wp - rp;
wire  [SLOT_W+1:0]      s_used = (SLOT_W+2)'(s_out) + ((rp != wp) ? (SLOT_W+2)'(SLOT_W'(h - hg)) : '0);
wire                    s_full = s_used >= (SLOT_W+2)'(N_SLOTS);

// Read-ahead blocks (ring entry = page number mod N_BLK)
logic [N_BLK-1:0]       b_sent, b_done, b_err;
logic [6:0]             b_cnt [N_BLK];      // reads attached and not yet answered

// The stream: pages [s_tail, s_fetch) requested and held; the next read
// must start at byte s_next; pages below s_need are wanted by attached
// reads; s_d pages are fetched ahead of s_cur, the page the reads are in
logic                   s_act;
logic [PID_BITS-1:0]    s_pid;
logic [VADDR_BITS-1:0]  s_base;
logic [UWIN_BITS-1:0]   s_ustart;
logic [UA-1:0]          s_end, s_next, s_tail, s_fetch, s_need, s_cur;
logic [BLK_W:0]         s_d;

// The ramp's bounds (CSR; 0 or too large = the defaults)
wire  [DW-1:0]          d_max = ((ra_dmax == 6'd0) || (int'(ra_dmax) > N_BLK)) ? DW'(N_BLK) : DW'(ra_dmax);
wire  [DW-1:0]          d_0r  = (ra_d0 == 6'd0) ? DW'(RA_D0) : ((int'(ra_d0) > N_BLK) ? DW'(N_BLK) : DW'(ra_d0));
wire  [DW-1:0]          d_0   = (d_0r > d_max) ? d_max : d_0r;
wire  [DW:0]            d_2x  = {s_d, 1'b0};

// The last read (for starting a stream)
logic                   l_ok;
logic [UA-1:0]          l_end;
logic [16:0]            l_run;              // bytes the consecutive reads up to the last one cover (saturating)
logic [PID_BITS-1:0]    l_pid;
logic [VADDR_BITS-1:0]  l_base;
logic [UWIN_BITS-1:0]   l_ustart;

logic [QW-1:0]          quiet;              // cycles since the last drop, saturating

// A block's get, waiting for the get port (the AR path's go first)
logic                   pf_v;
logic [PID_BITS-1:0]    pf_pid;
logic [VADDR_BITS-1:0]  pf_ref;
logic [6:0]             pf_nl;
logic [BLK_W-1:0]       pf_idx;

// ---------------------------------------------------------------------------
// AR: take, look up (two cycles), then answer from read-ahead, or join the
// open get or open one; send it
// ---------------------------------------------------------------------------
typedef enum logic [1:0] { A_IDLE, A_L1, A_L2, A_REQ } astate_t;
astate_t ast;

// The open get
logic                         o_v;
logic [SLOT_W-1:0]            o_slot;    // its first read's slot
logic [UWIN_BITS-1:0]         o_win;     // the window (its start in the uwin)
logic [UWIN_BITS-13:0]        o_page;
logic [12:0]                  o_next;    // where in the page a continuing read starts; 4096: none can
logic [5:0]                   o_l0;      // its first line in the page
logic [6:0]                   o_nl;      // lines
logic [PID_BITS-1:0]          o_pid;
logic [VADDR_BITS-1:0]        o_ref;
logic [$clog2(CO_WAIT+1)-1:0] o_age;     // cycles since a read last joined

wire o_close = o_v && (o_next[12] || (o_age == CO_WAIT));

assign s_arready = (ast == A_IDLE) && !s_full && !o_close;
wire   ar_hs     = s_arvalid && s_arready;

assign ub_addr = {s_araddr[UWIN_BITS-1:6], 6'b0};
assign ub_ce1  = (ast == A_IDLE);
assign ub_ce2  = (ast == A_L1);

// The lines the read's beats fall in: up to the line of its last beat
wire [14:0] ar_lo   = {3'b0, s_araddr[11:0]};
wire [14:0] ar_al   = ar_lo & ~((15'd1 << s_arsize) - 15'd1);
wire [14:0] ar_last = (s_arburst == 2'b00) ? ar_lo : ar_al + ({7'd0, s_arlen} << s_arsize);
wire        ar_bad  = (s_arsize > 3'd6) || (ar_last[14:12] != 3'd0);
// Where a read continuing this one starts (a fixed burst: none can)
wire [12:0] ar_next = (s_arburst == 2'b00) ? 13'h1000 : 13'(ar_last + (15'd1 << s_arsize));

logic [UWIN_BITS-1:0] a_addr;
logic [5:0]           a_lastl;      // the line of its last beat
logic [12:0]          a_next;
logic                 a_bad, a_incr;
logic [SLOT_W-1:0]    a_slot;

wire [UWIN_BITS-1:0] a_line   = {a_addr[UWIN_BITS-1:6], 6'b0};    // the first line
wire [6:0]           a_nlines = 7'(a_lastl) - 7'(a_addr[11:6]) + 7'd1;
wire [UWIN_BITS:0]   a_end    = {1'b0, a_line} + {{(UWIN_BITS-12){1'b0}}, a_nlines, 6'b0};
wire                 a_ok     = ub_hit && ub_route && !a_bad && ((LEN_BITS+1)'(a_end) <= ub_end);
wire [LEN_BITS-1:0]  a_off    = LEN_BITS'(a_line - ub_ustart);
wire                 a_join   = o_v && a_ok && (ub_ustart == o_win) && (a_addr[UWIN_BITS-1:12] == o_page) &&
                                ({1'b0, a_addr[11:0]} == o_next);

// The read's bytes [a_b0, a_b1) in the uwin, and its page
wire [UA-1:0]        a_b0  = {1'b0, a_addr};
wire [UA-1:0]        a_b1  = {1'b0, a_addr[UWIN_BITS-1:12], 12'b0} + UA'(a_next);
wire [UA-1:0]        a_pg  = {1'b0, a_line[UWIN_BITS-1:12], 12'b0};
wire [BLK_W-1:0]     a_idx = a_pg[12 +: BLK_W];

wire in_l2      = (ast == A_L2);
wire ev_land    = s_drop_store || s_drop_pkt || s_drop_write;
wire ring_idle  = (s_tail == s_fetch) && (s_need <= s_fetch) && !pf_v;
wire a_win_s    = (ub_pid == s_pid) && (ub_base == s_base) && (ub_ustart == s_ustart);
wire a_win_l    = (ub_pid == l_pid) && (ub_base == l_base) && (ub_ustart == l_ustart);
wire a_seq      = s_act && a_ok && a_incr && a_win_s && (a_b0 == s_next);
wire a_room     = (a_pg - s_tail) < RA_SPAN;
wire a_cont     = a_seq && a_room;                 // answered from the stream
wire a_hold     = a_seq && !a_room;                // waits for the oldest page to be used up
wire a_cont_l   = a_ok && a_incr && l_ok && a_win_l && (a_b0 == l_end);   // continues the last read
// a stream starts where the reads, a page or more already, enter a new page
wire a_enter    = ra_early || ((a_b0[11:0] == 12'd0) && (l_run >= 17'd4096));
wire a_start    = !s_act && !ra_off && a_cont_l && a_enter && ring_idle &&
                  (quiet == QW'(RA_QUIET)) && (ub_ustart[11:0] == 12'd0) && !ev_land;
wire a_attach   = in_l2 && (a_cont || a_start);
wire ev_read    = in_l2 && s_act && !a_cont && !a_hold;
wire ra_drop    = s_act && (ev_read || ev_land || ra_off);

// This read's get (the AR path), and a block's (the prefetcher)
logic [PID_BITS-1:0]    g_pid;
logic [VADDR_BITS-1:0]  g_ref;
logic [63:0]            g_word;

wire a_req   = (ast == A_REQ);
wire pf_take = pf_v && !a_req && m_get_ready;

assign m_get_valid = a_req || pf_v;
assign m_get_pid   = a_req ? g_pid  : pf_pid;
assign m_get_ref   = a_req ? g_ref  : pf_ref;
assign m_get_word  = a_req ? g_word : {9'd0, pf_nl, RRET_EXP, 40'({REG_W'(N_SLOTS + int'(pf_idx)), 13'd0})};
wire [SLOT_W-1:0] g_slot = g_word[13 +: SLOT_W];

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        ast <= A_IDLE;
        wp  <= '0;
        o_v <= 1'b0;
    end else begin
        if (o_v && (o_age != CO_WAIT)) o_age <= o_age + 1'b1;
        case (ast)
        A_IDLE: if (o_close) begin
            g_pid  <= o_pid;
            g_ref  <= o_ref;
            g_word <= {9'd0, o_nl, RRET_EXP, 40'({REG_W'(o_slot), 13'd0})};
            o_v    <= 1'b0;
            ast    <= A_REQ;
        end else if (ar_hs) begin
            a_addr  <= s_araddr;
            a_lastl <= ar_last[11:6];
            a_next  <= ar_next;
            a_bad   <= ar_bad;
            a_incr  <= (s_arburst == 2'b01);
            a_slot  <= wp[SLOT_W-1:0];
            m_id  [wp[SLOT_W-1:0]] <= s_arid;
            m_len [wp[SLOT_W-1:0]] <= s_arlen;
            m_size[wp[SLOT_W-1:0]] <= s_arsize;
            m_fix [wp[SLOT_W-1:0]] <= (s_arburst == 2'b00);
            m_addr[wp[SLOT_W-1:0]] <= s_araddr[11:0];
            m_get [wp[SLOT_W-1:0]] <= wp[SLOT_W-1:0];
            m_l0  [wp[SLOT_W-1:0]] <= s_araddr[11:6];
            m_ra  [wp[SLOT_W-1:0]] <= 1'b0;
            wp  <= wp + 1'b1;
            ast <= A_L1;
        end
        A_L1: ast <= A_L2;
        // A read the stream answers attaches to its page's block (and sends
        // an open get: it does not join one); one too far ahead of the
        // oldest page held waits here. No get is open while a stream runs:
        // stream reads only attach, so the wait cannot hold back an older
        // read's get.
        A_L2: if (a_hold) begin
            ast <= A_L2;
        end else if (a_attach) begin
            m_ra [a_slot] <= 1'b1;
            m_blk[a_slot] <= a_idx;
            if (o_v) begin
                g_pid  <= o_pid;
                g_ref  <= o_ref;
                g_word <= {9'd0, o_nl, RRET_EXP, 40'({REG_W'(o_slot), 13'd0})};
            end
            ast <= o_v ? A_REQ : A_IDLE;
            o_v <= 1'b0;
        end else if (a_join) begin
            m_get[a_slot] <= o_slot;
            m_l0 [a_slot] <= o_l0;
            o_next <= a_next;
            o_nl   <= 7'(a_lastl) - 7'(o_l0) + 7'd1;
            o_age  <= '0;
            ast    <= A_IDLE;
        end else begin
            // Any other read sends the open get; a good one opens the next
            if (o_v) begin
                g_pid  <= o_pid;
                g_ref  <= o_ref;
                g_word <= {9'd0, o_nl, RRET_EXP, 40'({REG_W'(o_slot), 13'd0})};
            end
            ast <= o_v ? A_REQ : A_IDLE;
            o_v <= a_ok;
            if (a_ok) begin
                o_slot <= a_slot;
                o_win  <= ub_ustart;
                o_page <= a_addr[UWIN_BITS-1:12];
                o_next <= a_next;
                o_l0   <= a_addr[11:6];
                o_nl   <= a_nlines;
                o_pid  <= ub_pid;
                o_ref  <= ub_base + VADDR_BITS'(a_off);
                o_age  <= '0;
            end
        end
        A_REQ: if (m_get_ready) ast <= A_IDLE;
        default: ast <= A_IDLE;
        endcase
    end
end

// ---------------------------------------------------------------------------
// The stream, the prefetcher and the block ring
// ---------------------------------------------------------------------------
logic             r_free;          // the head read's last beat issued this cycle
logic             r_ra;
logic [BLK_W-1:0] r_blk;

wire [UA-1:0]    f_left  = s_end - s_fetch;
wire             pf_go   = !pf_v && ((s_fetch - s_tail) < RA_SPAN) && (s_fetch < s_end) &&
                           ((s_act && (s_fetch < s_cur + (UA'(s_d) << 12))) || (s_fetch < s_need));
wire [BLK_W-1:0] f_idx   = s_fetch[12 +: BLK_W];
wire [BLK_W-1:0] t_idx   = s_tail[12 +: BLK_W];
// The oldest page is given back once its answer is in, no attached read is
// still unanswered, and the stream has moved past it (or is gone)
wire             t_free  = (s_tail != s_fetch) && b_done[t_idx] && !b_sent[t_idx] && (b_cnt[t_idx] == 7'd0) &&
                           (!s_act || ((s_next - s_tail) >= PAGE));

// Completions: a get's (its first read's slot), or a block's
wire               c_is_slot = (int'(s_cmp_slot) < N_SLOTS);
wire               c_is_blk  = !c_is_slot && (int'(s_cmp_slot) < N_REG);
wire [SLOT_W-1:0]  c_slot    = s_cmp_slot[SLOT_W-1:0];
wire [BLK_W-1:0]   c_blk     = BLK_W'(int'(s_cmp_slot) - N_SLOTS);
wire               cmp_take  = s_cmp_valid && c_is_slot && sent[c_slot];
wire               bcmp_take = s_cmp_valid && c_is_blk && b_sent[c_blk];

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        s_act   <= 1'b0;
        s_tail  <= '0;
        s_fetch <= '0;
        s_need  <= '0;
        s_next  <= '0;
        s_cur   <= '0;
        s_d     <= '0;
        l_ok    <= 1'b0;
        l_run   <= '0;
        quiet   <= QW'(RA_QUIET);
        pf_v    <= 1'b0;
        b_sent  <= '0;
        b_done  <= '0;
        b_err   <= '0;
        for (int i = 0; i < N_BLK; i++) b_cnt[i] <= '0;
    end else begin
        // the quiet time after a landing or a uwin write (a read that drops
        // a stream may start the next one at once: the start rule and the
        // drained ring keep interleaved readers from restarting streams)
        if (ev_land) quiet <= '0;
        else if (quiet != QW'(RA_QUIET)) quiet <= quiet + 1'b1;

        if (ra_drop) s_act <= 1'b0;
        else if (in_l2 && a_start) begin
            s_act    <= 1'b1;
            s_pid    <= ub_pid;
            s_base   <= ub_base;
            s_ustart <= ub_ustart;
            s_end    <= UA'(ub_end);
        end

        // the last read, and the stream's next byte
        if (in_l2 && !a_hold) begin
            l_ok     <= a_ok && a_incr;
            l_end    <= a_b1;
            if (!(a_ok && a_incr))  l_run <= '0;
            else if (a_cont_l)      l_run <= l_run[16] ? l_run : l_run + 17'(a_b1 - a_b0);
            else                    l_run <= 17'(a_b1 - a_b0);
            l_pid    <= ub_pid;
            l_base   <= ub_base;
            l_ustart <= ub_ustart;
        end
        if (a_attach) begin
            s_next <= a_b1;
            if (a_start || (a_pg + PAGE > s_need)) s_need <= a_pg + PAGE;
        end
        // ramp: d_0 pages ahead at the start, doubled each time the reads
        // enter a new page, up to d_max
        if (in_l2 && a_start) begin
            s_cur <= a_pg;
            s_d   <= d_0;
        end else if (a_attach && (a_pg != s_cur)) begin
            s_cur <= a_pg;
            s_d   <= (d_2x > {1'b0, d_max}) ? d_max : DW'(d_2x);
        end

        // pages: a stream starts on its first page; the prefetcher asks for
        // the next; the oldest is given back
        if (in_l2 && a_start) begin
            s_tail  <= a_pg;
            s_fetch <= a_pg;
        end else begin
            if (pf_go) s_fetch <= s_fetch + PAGE;
            if (t_free) s_tail <= s_tail + PAGE;
        end

        if (pf_go) begin
            pf_v   <= 1'b1;
            pf_idx <= f_idx;
            pf_pid <= s_pid;
            pf_ref <= s_base + VADDR_BITS'(s_fetch - {1'b0, s_ustart});
            pf_nl  <= (f_left >= PAGE) ? 7'd64 : 7'(f_left[12:6]);
            b_sent[f_idx] <= 1'b1;
            b_done[f_idx] <= 1'b0;
            b_err [f_idx] <= 1'b0;
        end else if (pf_take) pf_v <= 1'b0;

        if (bcmp_take) begin
            b_sent[c_blk] <= 1'b0;
            b_done[c_blk] <= 1'b1;
            b_err [c_blk] <= s_cmp_err;
        end
        if (t_free) b_done[t_idx] <= 1'b0;

        for (int i = 0; i < N_BLK; i++)
            b_cnt[i] <= b_cnt[i] + 7'(a_attach && (a_idx == BLK_W'(i))) - 7'(r_free && r_ra && (r_blk == BLK_W'(i)));
    end
end

// ---------------------------------------------------------------------------
// Completions and the region buffer
// ---------------------------------------------------------------------------
wire a_fail = in_l2 && !a_hold && !a_ok;

logic r_act;

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        done <= '0;
        err  <= '0;
        sent <= '0;
    end else begin
        if (ar_hs) done[wp[SLOT_W-1:0]] <= 1'b0;
        if (a_req && m_get_ready) sent[g_slot] <= 1'b1;
        if (a_fail) begin
            done[a_slot] <= 1'b1;
            err[a_slot]  <= 1'b1;
        end
        if (cmp_take) begin
            sent[c_slot] <= 1'b0;
            done[c_slot] <= 1'b1;
            err[c_slot]  <= s_cmp_err;
        end
    end
end

logic                   rd_en;
logic [LINE_W-1:0]      rd_line;
logic [AXI_DATA_BITS-1:0] buf_dout;

xpm_memory_sdpram #(
    .ADDR_WIDTH_A(LINE_W),
    .ADDR_WIDTH_B(LINE_W),
    .AUTO_SLEEP_TIME(0),
    .BYTE_WRITE_WIDTH_A(AXI_DATA_BITS),
    .CASCADE_HEIGHT(0),
    .CLOCKING_MODE("common_clock"),
    .ECC_MODE("no_ecc"),
    .MEMORY_INIT_FILE("none"),
    .MEMORY_INIT_PARAM("0"),
    .MEMORY_OPTIMIZATION("true"),
    .MEMORY_PRIMITIVE(BUF_MEM),
    .MEMORY_SIZE(N_REG * 64 * AXI_DATA_BITS),
    .MESSAGE_CONTROL(0),
    .READ_DATA_WIDTH_B(AXI_DATA_BITS),
    .READ_LATENCY_B(2),
    .READ_RESET_VALUE_B("0"),
    .RST_MODE_A("SYNC"),
    .RST_MODE_B("SYNC"),
    .SIM_ASSERT_CHK(0),
    .USE_EMBEDDED_CONSTRAINT(0),
    .USE_MEM_INIT(0),
    .WAKEUP_TIME("disable_sleep"),
    .WRITE_DATA_WIDTH_A(AXI_DATA_BITS),
    .WRITE_MODE_B("read_first")
) inst_buf (
    .clka(aclk), .clkb(aclk),
    .ena(s_land_valid), .wea(s_land_valid), .addra(s_land_line), .dina(s_land_data),
    .enb(rd_en), .regceb(1'b1), .addrb(rd_line), .doutb(buf_dout),
    .rstb(1'b0), .sleep(1'b0),
    .injectsbiterra(1'b0), .injectdbiterra(1'b0), .sbiterrb(), .dbiterrb()
);

// ---------------------------------------------------------------------------
// R: the head read's beats, in order, once its data is there (its get's, or
// its block's). Each beat reads the line its address falls in from that
// buffer (2 cycles), then waits in the skid buffer.
// ---------------------------------------------------------------------------
logic [7:0]             r_left;
logic [12:0]            r_cur;       // the beat's address in the page
logic [2:0]             r_size;
logic                   r_fix, r_err;
logic [SLOT_W-1:0]      r_get;       // its get's buffer
logic [5:0]             r_l0;        // and that get's first line
logic [AXI_ID_BITS-1:0] r_id;

// Two read stages, then the skid buffer
logic                   p1_v, p2_v, p1_last, p2_last, p1_err, p2_err;
logic [AXI_ID_BITS-1:0] p1_id, p2_id;
typedef struct packed {
    logic [AXI_DATA_BITS-1:0] data;
    logic                     last;
    logic [AXI_ID_BITS-1:0]   id;
} rbeat_t;
rbeat_t sk_mem [SK];
logic [$clog2(SK):0] sk_wp, sk_rp;
wire  [$clog2(SK):0] sk_cnt = sk_wp - sk_rp;
wire  sk_empty = (sk_wp == sk_rp);

wire             h_ra    = m_ra[h];
wire [BLK_W-1:0] h_blk   = m_blk[h];
wire             h_ready = h_ra ? b_done[h_blk] : done[hg];
wire             h_err   = h_ra ? b_err[h_blk]  : err[hg];

wire r_start = !r_act && (rp != wp) && h_ready;
wire r_room  = ({1'b0, sk_cnt} + 2'(p1_v) + 2'(p2_v)) < ($clog2(SK)+2)'(SK);
wire r_issue = r_act && r_room;
assign r_free  = r_issue && (r_left == 8'd0);
assign rd_en   = r_issue;
assign rd_line = r_ra ? {REG_W'(N_SLOTS + int'(r_blk)), r_cur[11:6]}
                      : {REG_W'(r_get), 6'(r_cur[11:6] - r_l0)};

wire [12:0] r_step = 13'd1 << r_size;
wire [12:0] r_next = r_fix ? r_cur : ((r_cur & ~(r_step - 13'd1)) + r_step);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        r_act <= 1'b0;
        rp    <= '0;
        p1_v  <= 1'b0;
        p2_v  <= 1'b0;
    end else begin
        if (r_start) begin
            r_act  <= 1'b1;
            r_left <= m_len[h];
            r_cur  <= {1'b0, m_addr[h]};
            r_size <= m_size[h];
            r_fix  <= m_fix[h];
            r_get  <= hg;
            r_l0   <= m_l0[h];
            r_id   <= m_id[h];
            r_err  <= h_err;
            r_ra   <= h_ra;
            r_blk  <= h_blk;
        end else if (r_issue) begin
            r_left <= r_left - 1'b1;
            r_cur  <= r_next;
            if (r_left == 8'd0) begin
                r_act <= 1'b0;
                rp    <= rp + 1'b1;
            end
        end
        p1_v <= r_issue;
        p2_v <= p1_v;
    end
end

always_ff @(posedge aclk) begin
    p1_last <= (r_left == 8'd0);
    p1_err  <= r_err;
    p1_id   <= r_id;
    p2_last <= p1_last;
    p2_err  <= p1_err;
    p2_id   <= p1_id;
end

wire r_pop = s_rvalid && s_rready;
always_ff @(posedge aclk) begin
    if (!aresetn) begin
        sk_wp <= '0;
        sk_rp <= '0;
    end else begin
        if (p2_v) begin
            sk_mem[sk_wp[$clog2(SK)-1:0]] <= '{data: p2_err ? {AXI_DATA_BITS{1'b1}} : buf_dout,
                                               last: p2_last, id: p2_id};
            sk_wp <= sk_wp + 1'b1;
        end
        if (r_pop) sk_rp <= sk_rp + 1'b1;
    end
end

assign s_rvalid = !sk_empty;
assign s_rdata  = sk_mem[sk_rp[$clog2(SK)-1:0]].data;
assign s_rlast  = sk_mem[sk_rp[$clog2(SK)-1:0]].last;
assign s_rid    = sk_mem[sk_rp[$clog2(SK)-1:0]].id;
assign s_rresp  = 2'b00;

// Not busy: reads waiting for answers from the far side (a get out, a block
// out, or a read waiting for the oldest block to be used up)
assign busy = ((ast != A_IDLE) && !(in_l2 && a_hold)) || o_v || r_act || p1_v || p2_v || !sk_empty ||
              ((rp != wp) && h_ready) || pf_v;

assign cnt_read      = ar_hs;
assign cnt_done      = r_pop && s_rlast;
assign cnt_fail      = r_start && h_err;
assign cnt_far_err   = (cmp_take || bcmp_take) && s_cmp_err;
assign cnt_slot_wait = s_arvalid && (ast == A_IDLE) && s_full;
assign cnt_stray     = s_cmp_valid && !cmp_take && !bcmp_take;

assign cnt_ra_read       = a_attach;
assign cnt_ra_block      = pf_take;
assign cnt_ra_start      = in_l2 && a_start;
assign cnt_ra_drop_read  = ev_read;
assign cnt_ra_drop_store = s_act && s_drop_store;
assign cnt_ra_drop_pkt   = s_act && s_drop_pkt;
assign cnt_ra_drop_write = s_act && s_drop_write;

endmodule
