import lynxTypes::*;

/**
 * loom_ingress
 *
 * AXI4 write slave on the ingress window (uwin), Loom's one data path. A
 * producer - the V80 copy engine peer-to-peer over PCIe, or the host CPU
 * through a write-combining mapping - writes to an address, and the address
 * names the destination: the table's uwin ranges map it to a binding and an
 * offset in it. Loom turns the write stream into packets and stores:
 *
 *   packet, local: sq_wr {LOCAL_WRITE, LOCAL_STRM, dest HOST_DEST, pid,
 *          base+off, len} + the payload beats on the m_host stream
 *          (LOCAL_STRM is STRM_HOST on the U280; the V80 lands in its card
 *          memory with STRM_CARD)
 *   packet, rdma: one self-describing RDMA write: sq_wr {RC_RDMA_WRITE_ONLY,
 *          RAW, STRM_RDMA, dest NET_DEST, pid = QP owner, vaddr = base+off,
 *          len} + the payload beats, nothing else. For an rdma binding,
 *          base is a REMOTE REFERENCE, not a VA: {export index [47:40],
 *          offset [39:0]} into the far host's export table, so the RETH of
 *          every packet says where that packet lands. The far loom_rx needs
 *          no header, no message and no end-of-transfer: it lands each
 *          packet on its own, the moment the stack announces it.
 *   store, local: sq_wr {LOCAL_WRITE, 8 B} + one beat, data in lane 0
 *   store, rdma: a 64 B inline message, RETH = {INLINE_EXP, 0}:
 *          {lane0 = op WRITE_INLINE, len 8; lane1 = base+off (the remote
 *          reference of the word); lane2 = data}, which loom_rx lands as
 *          the exact 8 B write
 *
 * PACKETS. Full beats (all 64 strobes) that continue the same binding at the
 * next offset are gathered into one packet, up to PMTU (64 beats) on either
 * route. A packet closes when it is full, when the next beat does not
 * continue it (another window or offset, or a store), or when no beat has
 * been presented for FLUSH_CYCLES and the packet could leave at once (the
 * output idle, nothing queued, room in the ack window) - write combining, as
 * a CPU's WC buffer does; it decides only how a tail is packed, never where
 * anything lands.
 * The request states the length, so a packet is stored before it is
 * forwarded; the data FIFO holds several, so the next one gathers while the
 * last one leaves.
 *
 * STORES. A beat that is not full is a set of 8 B stores: every aligned 8 B
 * word whose strobes are all set is one store, in lane order, one per cycle.
 * A word with only some of its strobes set is dropped and counted
 * (cnt_store_drop, once per beat); words with none are nothing. A CPU
 * write-combining buffer that fills a whole line hands over a full beat,
 * which joins packets like bulk.
 *
 * ORDER. One queue for packets and stores: they leave in the order their
 * beats arrived, across bindings, so a flag stored after data stays behind
 * it.
 *
 * CONTRACT. Bursts are INCR bursts of full-width beats (awsize is not
 * looked at). The address is rounded down to 64 B and the strobes say which
 * bytes are written, so a write that starts inside a line - a CPU's 32 B
 * half-line flush at line+32, a lone 8 B store at its own address - is a
 * partial first beat, i.e. stores. Window starts and lengths are 64 B
 * multiples. A burst with no window, or whose lines end past the window's
 * length, is accepted, discarded and counted (cnt_drop). B is answered once
 * the burst's last beat is taken. Reads are answered with zeros.
 *
 * LOOKUP. AW goes through three register stages (the table's two, then the
 * offset and bounds), each advancing whenever the next has room, so up to
 * three bursts are looked up ahead of the one being written and bursts
 * follow each other with no gap, at one a cycle for 1-beat bursts.
 *
 * WINDOW. rdma packets and stores are posted only while win_ok (the ack
 * window); rdma_post pulses on each so the window can count them.
 */
module loom_ingress #(
    parameter integer UWIN_BITS    = 27,
    // cnt_dbg bits: 0/1 host/net output valid and not ready; 2 sending a
    // packet with the data FIFO empty; 3-7 a W beat presented and not taken,
    // by reason: 3 no burst looked up yet, 4 the B slot busy, 5 data FIFO
    // full, 6 queue full, 7 a partial beat still sending its stores; 8/9 a
    // burst dropped for no window / past the window's end; 10-12 bursts of
    // 1, 2-4, more than 4 beats; 13 a burst whose address is not 64 B-aligned
    parameter integer N_DBG        = 14,
    parameter integer HOST_DEST    = 0,
    parameter logic [STRM_BITS-1:0] LOCAL_STRM = STRM_HOST,
    parameter integer NET_DEST     = 0,
    parameter integer FLUSH_CYCLES = 16,
    parameter integer FIFO_BEATS   = 256
) (
    input  logic                        aclk,
    input  logic                        aresetn,

    AXI4.s                              axi_udata,

    // Table lookup by uwin address (loom_table ua_* port, two stages)
    output logic                        ua_ce1,
    output logic                        ua_ce2,
    output logic [UWIN_BITS-1:0]        ua_addr,
    input  logic                        ua_hit,
    input  logic                        ua_route,
    input  logic [PID_BITS-1:0]         ua_pid,
    input  logic [PID_BITS-1:0]         ua_dst_pid,
    input  logic [VADDR_BITS-1:0]       ua_base,
    input  logic [UWIN_BITS-1:0]        ua_ustart,
    input  logic [LEN_BITS:0]           ua_end,
    input  logic [3:0]                  ua_idx,

    // sq_wr (shared with loom_rx in vfpga_top)
    output req_t                        wr_req,
    output logic                        wr_valid,
    input  logic                        wr_ready,

    // Ack window
    input  logic                        win_ok,
    output logic                        rdma_post,

    // Local payload out (axis_host_send[HOST_DEST])
    output logic [AXI_DATA_BITS-1:0]    m_host_tdata,
    output logic [AXI_DATA_BITS/8-1:0]  m_host_tkeep,
    output logic                        m_host_tvalid,
    input  logic                        m_host_tready,
    output logic                        m_host_tlast,

    // RDMA message out (axis_rreq_send[NET_DEST])
    output logic [AXI_DATA_BITS-1:0]    m_net_tdata,
    output logic [AXI_DATA_BITS/8-1:0]  m_net_tkeep,
    output logic                        m_net_tvalid,
    input  logic                        m_net_tready,
    output logic                        m_net_tlast,

    // Counter pulses
    output logic                        cnt_burst,      // a burst accepted
    output logic                        cnt_drop,       // a burst discarded
    output logic                        cnt_pkt_local,  // a local packet sent (last beat)
    output logic                        cnt_pkt_rdma,   // an rdma packet sent (last beat)
    output logic                        cnt_store,      // a store sent
    output logic                        cnt_store_drop, // a beat with a partial 8 B word
    output logic                        cnt_flush,      // a packet closed by the idle timer
    output logic                        cnt_win_wait,   // an rdma request waited on the window
    output logic                        cnt_req_wait,   // an rdma request waited on wr_ready
    output logic                        cnt_rdma_full,  // a full (PMTU) rdma packet queued
    output logic                        cnt_rdma_flush, // a partial rdma packet closed by the idle timer
    output logic                        cnt_rdma_cut,   // a partial rdma packet closed by a non-continuing write
    output logic [N_DBG-1:0]            cnt_dbg         // debug pulses, listed at N_DBG
);

localparam [7:0] MSG_OP_WRITE_INLINE = 8'd2;   // keep in sync with loom_rx.sv
localparam [7:0] INLINE_EXP          = 8'hFF;  // RETH export index of an inline message
localparam integer PKT_BEATS  = PMTU_BYTES / 64;
localparam integer BEAT_W     = $clog2(PKT_BEATS + 1);

// ---------------------------------------------------------------------------
// AW lookup: s1 takes the address (the table compares it), s2 has the table's
// pick, s3 the offset and the bounds. Each stage advances when the next has
// room; the burst stage takes s3 (aw_take).
// ---------------------------------------------------------------------------
logic                   aw_take;          // the burst stage takes s3

logic                   s1_v, s2_v, s3_v;
logic [UWIN_BITS-1:0]   s1_addr, s2_addr;
logic [UWIN_BITS:0]     s1_aend, s2_aend; // end of the burst's lines
logic [7:0]             s1_len, s2_len, s3_len;
logic [AXI_ID_BITS-1:0] s1_id, s2_id, s3_id;
logic                   s1_mis, s2_mis, s3_mis;
logic                   s3_hit, s3_ok, s3_route;
logic [3:0]             s3_idx;
logic [PID_BITS-1:0]    s3_pid, s3_dst_pid;
logic [VADDR_BITS-1:0]  s3_base;
logic [LEN_BITS-1:0]    s3_off;

wire s3_en = !s3_v || aw_take;
wire s2_en = !s2_v || s3_en;
wire s1_en = !s1_v || s2_en;

assign axi_udata.awready = s1_en;
wire aw_hs = axi_udata.awvalid && s1_en;
wire [UWIN_BITS-1:0] aw_line = {axi_udata.awaddr[UWIN_BITS-1:6], 6'b0};

assign ua_ce1  = s1_en;
assign ua_ce2  = s2_en;
assign ua_addr = aw_line;

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        s1_v <= 1'b0;
        s2_v <= 1'b0;
        s3_v <= 1'b0;
    end else begin
        if (s1_en) s1_v <= aw_hs;
        if (s2_en) s2_v <= s1_v;
        if (s3_en) s3_v <= s2_v;
    end
end

always_ff @(posedge aclk) begin
    if (s1_en) begin
        s1_addr <= aw_line;
        s1_aend <= {1'b0, aw_line} + {{(UWIN_BITS-14){1'b0}}, axi_udata.awlen, 6'b0} + (UWIN_BITS+1)'(64);
        s1_len  <= axi_udata.awlen;
        s1_id   <= axi_udata.awid;
        s1_mis  <= axi_udata.awaddr[5:0] != 6'b0;
    end
    if (s2_en) begin
        s2_addr <= s1_addr;
        s2_aend <= s1_aend;
        s2_len  <= s1_len;
        s2_id   <= s1_id;
        s2_mis  <= s1_mis;
    end
    if (s3_en) begin
        s3_hit     <= ua_hit;
        s3_ok      <= ua_hit && ((LEN_BITS+1)'(s2_aend) <= ua_end);
        s3_idx     <= ua_idx;
        s3_route   <= ua_route;
        s3_pid     <= ua_pid;
        s3_dst_pid <= ua_dst_pid;
        s3_base    <= ua_base;
        s3_off     <= LEN_BITS'(s2_addr - ua_ustart);
        s3_len     <= s2_len;
        s3_id      <= s2_id;
        s3_mis     <= s2_mis;
    end
end

// ---------------------------------------------------------------------------
// Burst stage: the burst whose beats are being accepted
// ---------------------------------------------------------------------------
logic                  b_act;             // a burst is loaded
logic                  b_ok;              // its beats are taken, not discarded
logic [3:0]            b_idx;
logic                  b_route;
logic [PID_BITS-1:0]   b_pid, b_dst_pid;
logic [VADDR_BITS-1:0] b_base;
logic [LEN_BITS-1:0]   b_off;             // binding offset of the next beat
logic [AXI_ID_BITS-1:0] b_id;

// B: one response register
logic   bq_valid;
logic [AXI_ID_BITS-1:0] bq_id;

// Queue and data FIFO room
logic   pq_full;
logic   df_ready;
logic   pk_open;

wire w_in   = axi_udata.wvalid && b_act;
wire b_room = !axi_udata.wlast || !bq_valid || axi_udata.bready;

// The beat presented: full, or a set of stores
wire w_full = &axi_udata.wstrb;
logic [7:0] wd_all, wd_bad;               // per 8 B word: all strobes / some
always_comb
    for (int l = 0; l < 8; l++) begin
        wd_all[l] = &axi_udata.wstrb[8*l +: 8];
        wd_bad[l] = |axi_udata.wstrb[8*l +: 8] && !wd_all[l];
    end

// Stores of a partial beat go out one word per cycle; st_done holds the words
// of the presented beat already queued
logic [7:0] st_done;
wire  [7:0] st_rem  = wd_all & ~st_done;
wire  [7:0] st_one  = st_rem & (~st_rem + 8'd1);    // lowest remaining word
logic [2:0] st_lane;
always_comb begin
    st_lane = 3'd0;
    for (int l = 7; l >= 0; l--) if (st_rem[l]) st_lane = 3'(l);
end
wire part       = w_in && b_ok && !w_full;
// A partial beat first closes an open packet (so it cannot pass it), then
// queues its words, and is taken with its last one (or at once if it has none)
wire part_close = part && pk_open;
wire part_store = part && !pk_open && (st_rem != 8'd0);
wire part_last  = (st_rem & ~st_one) == 8'd0;
wire part_take  = part && !pk_open && (!part_store || !pq_full) && part_last;

wire full_ok    = df_ready && !pq_full;

assign axi_udata.wready = b_act && b_room &&
                          (!b_ok || (w_full ? full_ok : part_take));
wire w_hs    = axi_udata.wvalid && axi_udata.wready;
wire w_end   = w_hs && axi_udata.wlast;

assign aw_take = s3_v && (!b_act || w_end);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        b_act <= 1'b0;
        b_ok  <= 1'b0;
    end else begin
        if (aw_take) begin
            b_act     <= 1'b1;
            b_ok      <= s3_ok;
            b_idx     <= s3_idx;
            b_route   <= s3_route;
            b_pid     <= s3_pid;
            b_dst_pid <= s3_dst_pid;
            b_base    <= s3_base;
            b_off     <= s3_off;
            b_id      <= s3_id;
        end else if (w_end) begin
            b_act <= 1'b0;
        end
        if (w_hs && !w_end) b_off <= b_off + LEN_BITS'(64);
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn)                       st_done <= '0;
    else if (w_hs)                      st_done <= '0;
    else if (part_store && !pq_full)    st_done <= st_done | st_one;
end

assign cnt_burst      = aw_take && s3_ok;
assign cnt_drop       = aw_take && !s3_ok;
assign cnt_store_drop = w_hs && b_ok && !w_full && (wd_bad != 8'd0);

always_ff @(posedge aclk) begin
    if (!aresetn) bq_valid <= 1'b0;
    else if (w_end) begin
        bq_valid <= 1'b1;
        bq_id    <= b_id;
    end else if (axi_udata.bready) bq_valid <= 1'b0;
end
assign axi_udata.bvalid = bq_valid;
assign axi_udata.bid    = bq_id;
assign axi_udata.bresp  = 2'b00;

// ---------------------------------------------------------------------------
// Packet gathering
// ---------------------------------------------------------------------------
logic [3:0]            pk_idx;
logic                  pk_route;
logic [PID_BITS-1:0]   pk_pid, pk_dst_pid;
logic [VADDR_BITS-1:0] pk_va;             // base + offset of the first beat
logic [LEN_BITS-1:0]   pk_next;           // binding offset the next beat must have
logic [BEAT_W-1:0]     pk_beats;
logic [$clog2(FLUSH_CYCLES+1)-1:0] idle;

wire good_beat = w_hs && b_ok && w_full;
wire cont      = pk_open && (pk_idx == b_idx) && (pk_next == b_off);
wire [BEAT_W-1:0] cap_cur = BEAT_W'(PKT_BEATS);
// The idle timer closes a partial packet only when it can leave at once:
// nothing queued or being sent ahead of it and, on the rdma route, room in
// the ack window. Closing it earlier gets nothing on the wire sooner, it
// only splits the run. Under backpressure that split fed itself: the
// producer, held by the window, resumed with pauses past the timer; each
// pause closed a short packet; short packets filled the packet-counted
// window with less data, which held the producer again. 16 MiB copies went
// out as ~11k packets instead of 4096, at ~4 GB/s instead of ~11.
logic send_idle;
wire flush     = pk_open && !w_in && send_idle && (idle >= FLUSH_CYCLES - 1);

// Queue entries: a packet (its beats are in the data FIFO) or a store
typedef struct packed {
    logic                  store;
    logic                  route;
    logic [PID_BITS-1:0]   pid;
    logic [PID_BITS-1:0]   dst_pid;
    logic [VADDR_BITS-1:0] va;
    logic [BEAT_W-1:0]     beats;
    logic [63:0]           data;
} pkt_t;

// At most one entry is queued per cycle: the open packet when a full beat
// does not continue it, when a beat fills it, when a partial beat arrives, or
// on the idle timer (only in cycles with no beat); or one store
logic pq_push;
pkt_t pq_in;

always_comb begin
    pq_push = 1'b0;
    pq_in   = '{store: 1'b0, route: pk_route, pid: pk_pid, dst_pid: pk_dst_pid,
                va: pk_va, beats: pk_beats, data: 64'd0};
    if (good_beat) begin
        if (pk_open && !cont) begin
            pq_push = 1'b1;                           // close the open one
        end else if (cont && (pk_beats + 1'b1 == cap_cur)) begin
            pq_push     = 1'b1;                       // this beat fills it
            pq_in.beats = pk_beats + 1'b1;
        end
    end else if (part_close) begin
        pq_push = !pq_full;
    end else if (part_store) begin
        pq_push = !pq_full;
        pq_in   = '{store: 1'b1, route: b_route, pid: b_pid, dst_pid: b_dst_pid,
                    va: b_base + VADDR_BITS'(b_off) + VADDR_BITS'({st_lane, 3'b0}),
                    beats: '0, data: axi_udata.wdata[64*st_lane +: 64]};
    end else if (flush) begin
        pq_push = 1'b1;
    end
end

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        pk_open <= 1'b0;
        idle    <= '0;
    end else begin
        if (w_in || !pk_open)              idle <= '0;
        else if (idle != FLUSH_CYCLES - 1) idle <= idle + 1'b1;
        if (good_beat) begin
            if (cont) begin
                pk_beats <= pk_beats + 1'b1;
                pk_next  <= pk_next + LEN_BITS'(64);
                if (pk_beats + 1'b1 == cap_cur) pk_open <= 1'b0;
            end else begin
                pk_open    <= 1'b1;
                pk_idx     <= b_idx;
                pk_route   <= b_route;
                pk_pid     <= b_pid;
                pk_dst_pid <= b_dst_pid;
                pk_va      <= b_base + VADDR_BITS'(b_off);
                pk_next    <= b_off + LEN_BITS'(64);
                pk_beats   <= BEAT_W'(1);
            end
        end else if ((part_close && !pq_full) || flush) begin
            pk_open <= 1'b0;
        end
    end
end

assign cnt_flush      = flush;
assign cnt_rdma_full  = pq_push && !pq_in.store && pq_in.route && (pq_in.beats == BEAT_W'(PKT_BEATS));
assign cnt_rdma_flush = flush && pk_route;
assign cnt_rdma_cut   = pq_push && !pq_in.store && pq_in.route && (pq_in.beats != BEAT_W'(PKT_BEATS)) && !flush;

// ---------------------------------------------------------------------------
// Queue: closed packets and stores waiting to be sent
// ---------------------------------------------------------------------------
localparam integer PQ_DEPTH = 8;

pkt_t pq_mem [PQ_DEPTH];
logic [$clog2(PQ_DEPTH):0] pq_wp, pq_rp;
wire  pq_empty = (pq_wp == pq_rp);
assign pq_full = (pq_wp - pq_rp) == ($clog2(PQ_DEPTH)+1)'(PQ_DEPTH);
logic pq_pop;
pkt_t pq_head;
assign pq_head = pq_mem[pq_rp[$clog2(PQ_DEPTH)-1:0]];

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        pq_wp <= '0;
        pq_rp <= '0;
    end else begin
        if (pq_push) begin
            pq_mem[pq_wp[$clog2(PQ_DEPTH)-1:0]] <= pq_in;
            pq_wp <= pq_wp + 1'b1;
        end
        if (pq_pop) pq_rp <= pq_rp + 1'b1;
    end
end

// ---------------------------------------------------------------------------
// Data FIFO: payload beats of open and closed packets, in arrival order
// ---------------------------------------------------------------------------
logic [AXI_DATA_BITS-1:0] df_tdata;
logic df_tvalid, df_tready;

xpm_fifo_axis #(
    .CLOCKING_MODE("common_clock"),
    .FIFO_MEMORY_TYPE("block"),
    .PACKET_FIFO("false"),
    .FIFO_DEPTH(FIFO_BEATS),
    .TDATA_WIDTH(AXI_DATA_BITS),
    .USE_ADV_FEATURES("0000")
) inst_data_fifo (
    .s_aresetn(aresetn), .s_aclk(aclk), .m_aclk(aclk),
    .s_axis_tvalid(good_beat), .s_axis_tready(df_ready),
    .s_axis_tdata(axi_udata.wdata), .s_axis_tstrb('0), .s_axis_tkeep('1),
    .s_axis_tlast(1'b0), .s_axis_tid('0), .s_axis_tdest('0), .s_axis_tuser('0),
    .m_axis_tvalid(df_tvalid), .m_axis_tready(df_tready),
    .m_axis_tdata(df_tdata), .m_axis_tstrb(), .m_axis_tkeep(),
    .m_axis_tlast(), .m_axis_tid(), .m_axis_tdest(), .m_axis_tuser(),
    .prog_full_axis(), .wr_data_count_axis(), .almost_full_axis(),
    .prog_empty_axis(), .rd_data_count_axis(), .almost_empty_axis(),
    .injectsbiterr_axis(1'b0), .injectdbiterr_axis(1'b0),
    .sbiterr_axis(), .dbiterr_axis()
);

// ---------------------------------------------------------------------------
// Send: request, then the rdma header (a store's whole message), then the
// payload (a local store's one beat)
// ---------------------------------------------------------------------------
typedef enum logic [2:0] { O_IDLE, O_REQ, O_HDR, O_DATA, O_STORE } ostate_t;
ostate_t ostate;
pkt_t    o;
logic [BEAT_W-1:0] o_left;

assign pq_pop = (ostate == O_IDLE) && !pq_empty;
assign send_idle = (ostate == O_IDLE) && pq_empty && (!pk_route || win_ok);
wire   o_last = (o_left == BEAT_W'(1));
wire   o_rdy  = o.route ? m_net_tready : m_host_tready;
wire   o_beat = (ostate == O_DATA) && df_tvalid && o_rdy;

always_ff @(posedge aclk) begin
    if (!aresetn) ostate <= O_IDLE;
    else case (ostate)
        O_IDLE: if (!pq_empty) begin
            o      <= pq_head;
            ostate <= O_REQ;
        end
        O_REQ: if (wr_valid && wr_ready) begin
            o_left <= o.beats;
            ostate <= o.store ? (o.route ? O_HDR : O_STORE) : O_DATA;
        end
        O_HDR:   if (m_net_tready) ostate <= O_IDLE;
        O_STORE: if (m_host_tready) ostate <= O_IDLE;
        O_DATA: if (o_beat) begin
            o_left <= o_left - 1'b1;
            if (o_last) ostate <= O_IDLE;
        end
        default: ostate <= O_IDLE;
    endcase
end

wire [LEN_BITS-1:0] o_bytes = o.store ? LEN_BITS'(8) : (LEN_BITS'(o.beats) << 6);

always_comb begin
    wr_req      = '0;
    wr_req.last = 1'b1;
    wr_req.pid  = o.pid;
    if (o.route) begin
        wr_req.opcode = RC_RDMA_WRITE_ONLY;
        wr_req.strm   = STRM_RDMA;
        wr_req.mode   = 1'b1;          // RDMA_MODE_RAW: one request, one packet
        wr_req.rdma   = 1'b1;
        wr_req.remote = 1'b1;
        wr_req.actv   = 1'b1;
        wr_req.dest   = NET_DEST;
        // the RETH: where this packet lands (a remote reference), or the
        // inline marker for a store's 64 B message
        wr_req.vaddr  = o.store ? {INLINE_EXP, 40'd0} : o.va;
        wr_req.len    = o.store ? LEN_BITS'(64) : o_bytes;
    end else begin
        wr_req.opcode = LOCAL_WRITE;
        wr_req.strm   = LOCAL_STRM;
        wr_req.dest   = HOST_DEST;
        wr_req.vaddr  = o.va;
        wr_req.len    = o_bytes;
    end
    wr_valid = (ostate == O_REQ) && (!o.route || win_ok);
end
assign rdma_post    = wr_valid && wr_ready && o.route;
assign cnt_win_wait = (ostate == O_REQ) && o.route && !win_ok;
assign cnt_req_wait = wr_valid && !wr_ready && o.route;

// Inline message (a store on the rdma route): lane 0 {zero, len 8, op};
// lane 1 the word's remote reference; lane 2 its data
wire [63:0] hdr_q0 = {28'd0, 28'd8, MSG_OP_WRITE_INLINE};
wire [63:0] hdr_q1 = {{(64-VADDR_BITS){1'b0}}, o.va};
wire [63:0] hdr_q2 = o.data;

assign df_tready = (ostate == O_DATA) && o_rdy;

always_comb begin
    m_host_tdata  = (ostate == O_STORE) ? {{(AXI_DATA_BITS-64){1'b0}}, o.data} : df_tdata;
    m_host_tkeep  = (ostate == O_STORE) ? {{(AXI_DATA_BITS/8-8){1'b0}}, 8'hFF} : '1;
    m_host_tlast  = (ostate == O_STORE) || o_last;
    m_host_tvalid = (ostate == O_STORE) || ((ostate == O_DATA) && !o.route && df_tvalid);

    m_net_tdata   = (ostate == O_HDR) ? {{(AXI_DATA_BITS-192){1'b0}}, hdr_q2, hdr_q1, hdr_q0} : df_tdata;
    m_net_tkeep   = '1;
    m_net_tlast   = (ostate == O_HDR) || ((ostate == O_DATA) && o_last);
    m_net_tvalid  = (ostate == O_HDR) || ((ostate == O_DATA) && o.route && df_tvalid);
end

assign cnt_pkt_local = o_beat && o_last && !o.route;
assign cnt_pkt_rdma  = o_beat && o_last &&  o.route;
assign cnt_store     = ((ostate == O_HDR) && o.store && m_net_tready) ||
                       ((ostate == O_STORE) && m_host_tready);

// ---------------------------------------------------------------------------
// Debug pulses (bit meanings at N_DBG)
// ---------------------------------------------------------------------------
wire w_wait = axi_udata.wvalid && !axi_udata.wready;
wire w_taking = w_wait && b_act && b_room && b_ok;

assign cnt_dbg[0]  = m_host_tvalid && !m_host_tready;
assign cnt_dbg[1]  = m_net_tvalid && !m_net_tready;
assign cnt_dbg[2]  = (ostate == O_DATA) && !df_tvalid;
assign cnt_dbg[3]  = w_wait && !b_act;
assign cnt_dbg[4]  = w_wait && b_act && !b_room;
assign cnt_dbg[5]  = w_taking && w_full && !df_ready;
assign cnt_dbg[6]  = w_taking && pq_full && (df_ready || !w_full);
assign cnt_dbg[7]  = w_taking && !w_full && !pq_full;
assign cnt_dbg[8]  = aw_take && !s3_hit;
assign cnt_dbg[9]  = aw_take && s3_hit && !s3_ok;
assign cnt_dbg[10] = aw_take && (s3_len == 8'd0);
assign cnt_dbg[11] = aw_take && (s3_len != 8'd0) && (s3_len < 8'd4);
assign cnt_dbg[12] = aw_take && (s3_len >= 8'd4);
assign cnt_dbg[13] = aw_take && s3_mis;

// ---------------------------------------------------------------------------
// Reads: zeros, one beat per requested beat
// ---------------------------------------------------------------------------
logic       r_act;
logic [7:0] r_left;
logic [AXI_ID_BITS-1:0] r_id;

assign axi_udata.arready = !r_act;
always_ff @(posedge aclk) begin
    if (!aresetn) r_act <= 1'b0;
    else if (!r_act && axi_udata.arvalid) begin
        r_act  <= 1'b1;
        r_left <= axi_udata.arlen;
        r_id   <= axi_udata.arid;
    end else if (r_act && axi_udata.rready) begin
        if (r_left == 8'd0) r_act <= 1'b0;
        r_left <= r_left - 1'b1;
    end
end
assign axi_udata.rvalid = r_act;
assign axi_udata.rdata  = '0;
assign axi_udata.rresp  = 2'b00;
assign axi_udata.rid    = r_id;
assign axi_udata.rlast  = r_act && (r_left == 8'd0);

endmodule
