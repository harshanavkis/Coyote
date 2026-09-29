import lynxTypes::*;

/**
 * loom_ingress
 *
 * AXI4 write slave on the ingress window (uwin). A producer - the V80 copy
 * engine, peer-to-peer over PCIe - writes to an address, and the address
 * names the destination: the table's uwin ranges map it to a binding and an
 * offset in it. Loom turns the write stream into packets:
 *
 *   local: sq_wr {LOCAL_WRITE, STRM_HOST, dest HOST_DEST, pid, base+off, len}
 *          + the payload beats on its own host stream
 *   rdma:  one single-packet Loom message: sq_wr {RC_RDMA_WRITE_ONLY, RAW,
 *          STRM_RDMA, dest NET_DEST, pid = QP owner, vaddr = staging,
 *          64 + len} + the header beat {op WRITE, len, dst_pid, base+off}
 *          + the payload beats, which is exactly what loom_engine's bulk
 *          path sends for a message that fits one packet, so the far
 *          loom_rx lands it unchanged
 *
 * PACKETS. Writes arrive as bursts of PCIe-payload size (a few beats each).
 * Consecutive beats that continue the same binding at the next offset are
 * gathered into one packet, up to PMTU on the wire (header included, so 63
 * payload beats on rdma, 64 on local). A packet closes when it is full, when
 * the next beat does not continue it (another window or offset), or when no
 * beat has been presented for FLUSH_CYCLES. Both the request and the rdma
 * header state the length, so a packet is stored before it is forwarded;
 * the data FIFO holds several, so the next one gathers while the last one
 * leaves.
 *
 * ORDER. One data FIFO and one packet queue: packets leave in the order
 * their beats arrived, across bindings.
 *
 * CONTRACT. Bursts are 64 B-aligned, full-width, full-strobe INCR bursts
 * (loom_ce only emits those; wstrb is not looked at). A burst with no
 * window, a misaligned address or an end past the window's length is
 * accepted, discarded and counted (cnt_drop). B is answered once the
 * burst's last beat is in the FIFO. Reads are answered with zeros.
 *
 * WINDOW. rdma packets are posted only while win_ok (the shared ack window
 * in vfpga_top); rdma_post pulses on each so the window can count them.
 */
module loom_ingress #(
    parameter integer UWIN_BITS    = 27,
    parameter integer HOST_DEST    = 2,
    parameter integer NET_DEST     = 1,
    parameter integer FLUSH_CYCLES = 16,
    parameter integer FIFO_BEATS   = 256
) (
    input  logic                        aclk,
    input  logic                        aresetn,

    AXI4.s                              axi_udata,

    // Table lookup by uwin address (loom_table ua_* port)
    output logic [UWIN_BITS-1:0]        ua_addr,
    input  logic                        ua_hit,
    input  logic                        ua_route,
    input  logic [PID_BITS-1:0]         ua_pid,
    input  logic [PID_BITS-1:0]         ua_dst_pid,
    input  logic [VADDR_BITS-1:0]       ua_base,
    input  logic [LEN_BITS-1:0]         ua_len,
    input  logic [LEN_BITS-1:0]         ua_off,
    input  logic [3:0]                  ua_idx,

    // RDMA staging VA (RETH vaddr of every outgoing message)
    input  logic [VADDR_BITS-1:0]       rdma_staging_va,

    // sq_wr (shared in vfpga_top)
    output req_t                        wr_req,
    output logic                        wr_valid,
    input  logic                        wr_ready,

    // Shared ack window
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
    output logic                        cnt_burst,     // a burst accepted into a packet
    output logic                        cnt_drop,      // a burst discarded
    output logic                        cnt_pkt_local, // a local packet sent (last beat)
    output logic                        cnt_pkt_rdma,  // an rdma packet sent (last beat)
    output logic                        cnt_flush      // a packet closed by the idle timer
);

localparam [7:0] MSG_OP_WRITE = 8'd1;   // keep in sync with loom_rx.sv
localparam integer PKT_BEATS  = PMTU_BYTES / 64;
localparam integer BEAT_W     = $clog2(PKT_BEATS + 1);

// ---------------------------------------------------------------------------
// AW: one slot ahead of the burst being written, so the next burst's lookup
// is ready when the current one ends and bursts follow with no gap.
// ---------------------------------------------------------------------------
logic                  aw_full;
logic [UWIN_BITS-1:0]  aw_addr;
logic [7:0]            aw_len;
logic [AXI_ID_BITS-1:0] aw_id;
logic                  aw_take;           // the burst stage takes the slot

assign axi_udata.awready = !aw_full || aw_take;
always_ff @(posedge aclk) begin
    if (!aresetn) aw_full <= 1'b0;
    else if (axi_udata.awvalid && axi_udata.awready) begin
        aw_full <= 1'b1;
        aw_addr <= axi_udata.awaddr[UWIN_BITS-1:0];
        aw_len  <= axi_udata.awlen;
        aw_id   <= axi_udata.awid;
    end else if (aw_take) aw_full <= 1'b0;
end

assign ua_addr = aw_addr;

// ---------------------------------------------------------------------------
// Burst stage: the burst whose beats are being accepted, with its table hit
// latched when it took the slot.
// ---------------------------------------------------------------------------
logic                  b_act;             // a burst is loaded
logic                  b_ok;              // its beats go into packets
logic [3:0]            b_idx;
logic                  b_route;
logic [PID_BITS-1:0]   b_pid, b_dst_pid;
logic [VADDR_BITS-1:0] b_base;
logic [LEN_BITS-1:0]   b_off;             // binding offset of the next beat
logic [AXI_ID_BITS-1:0] b_id;

// End of the slot's burst inside its window
wire [LEN_BITS:0] aw_end = {1'b0, ua_off} + {{(LEN_BITS-13){1'b0}}, aw_len, 6'b0} + (LEN_BITS+1)'(64);
wire aw_ok = ua_hit && (aw_addr[5:0] == 6'b0) && (aw_end <= {1'b0, ua_len});

// B: one response register
logic   bq_valid;
logic [AXI_ID_BITS-1:0] bq_id;

// Packet queue and data FIFO room
logic   pq_full;
logic   df_ready;

wire w_in    = axi_udata.wvalid && b_act;
// A good beat may close a packet, so it needs room in both; the last beat of
// a burst also needs the B register
wire w_room  = (!b_ok || (df_ready && !pq_full)) &&
               (!axi_udata.wlast || !bq_valid || axi_udata.bready);
assign axi_udata.wready = b_act && w_room;
wire w_hs    = axi_udata.wvalid && axi_udata.wready;
wire w_end   = w_hs && axi_udata.wlast;

assign aw_take = aw_full && (!b_act || w_end);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        b_act <= 1'b0;
        b_ok  <= 1'b0;
    end else begin
        if (aw_take) begin
            b_act     <= 1'b1;
            b_ok      <= aw_ok;
            b_idx     <= ua_idx;
            b_route   <= ua_route;
            b_pid     <= ua_pid;
            b_dst_pid <= ua_dst_pid;
            b_base    <= ua_base;
            b_off     <= ua_off;
            b_id      <= aw_id;
        end else if (w_end) begin
            b_act <= 1'b0;
        end
        if (w_hs && !w_end) b_off <= b_off + LEN_BITS'(64);
    end
end

assign cnt_burst = aw_take && aw_ok;
assign cnt_drop  = aw_take && !aw_ok;

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
logic                  pk_open;
logic [3:0]            pk_idx;
logic                  pk_route;
logic [PID_BITS-1:0]   pk_pid, pk_dst_pid;
logic [VADDR_BITS-1:0] pk_va;             // base + offset of the first beat
logic [LEN_BITS-1:0]   pk_next;           // binding offset the next beat must have
logic [BEAT_W-1:0]     pk_beats;
logic [$clog2(FLUSH_CYCLES+1)-1:0] idle;

wire good_beat = w_hs && b_ok;
wire cont      = pk_open && (pk_idx == b_idx) && (pk_next == b_off);
wire [BEAT_W-1:0] cap_cur = pk_route ? BEAT_W'(PKT_BEATS-1) : BEAT_W'(PKT_BEATS);
wire flush     = pk_open && !w_in && !pq_full && (idle >= FLUSH_CYCLES - 1);

// At most one packet closes per cycle: the open one when a beat does not
// continue it, or when the beat fills it, or on the idle timer (only in
// cycles with no beat)
logic                  pq_push;
logic                  pq_route_i;
logic [PID_BITS-1:0]   pq_pid_i, pq_dst_pid_i;
logic [VADDR_BITS-1:0] pq_va_i;
logic [BEAT_W-1:0]     pq_beats_i;

always_comb begin
    pq_push      = 1'b0;
    pq_route_i   = pk_route;
    pq_pid_i     = pk_pid;
    pq_dst_pid_i = pk_dst_pid;
    pq_va_i      = pk_va;
    pq_beats_i   = pk_beats;
    if (good_beat) begin
        if (pk_open && !cont) begin
            pq_push = 1'b1;                           // close the open one
        end else if (cont && (pk_beats + 1'b1 == cap_cur)) begin
            pq_push    = 1'b1;                        // this beat fills it
            pq_beats_i = pk_beats + 1'b1;
        end
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
        end else if (flush) begin
            pk_open <= 1'b0;
        end
    end
end

assign cnt_flush = flush;

// ---------------------------------------------------------------------------
// Packet queue: closed packets waiting to be sent
// ---------------------------------------------------------------------------
localparam integer PQ_DEPTH = 8;
typedef struct packed {
    logic                  route;
    logic [PID_BITS-1:0]   pid;
    logic [PID_BITS-1:0]   dst_pid;
    logic [VADDR_BITS-1:0] va;
    logic [BEAT_W-1:0]     beats;
} pkt_t;

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
            pq_mem[pq_wp[$clog2(PQ_DEPTH)-1:0]] <= '{route: pq_route_i, pid: pq_pid_i,
                dst_pid: pq_dst_pid_i, va: pq_va_i, beats: pq_beats_i};
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
// Send: request, rdma header, payload
// ---------------------------------------------------------------------------
typedef enum logic [1:0] { O_IDLE, O_REQ, O_HDR, O_DATA } ostate_t;
ostate_t ostate;
pkt_t    o;
logic [BEAT_W-1:0] o_left;

assign pq_pop = (ostate == O_IDLE) && !pq_empty;
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
            ostate <= o.route ? O_HDR : O_DATA;
        end
        O_HDR: if (m_net_tready) ostate <= O_DATA;
        O_DATA: if (o_beat) begin
            o_left <= o_left - 1'b1;
            if (o_last) ostate <= O_IDLE;
        end
        default: ostate <= O_IDLE;
    endcase
end

wire [LEN_BITS-1:0] o_bytes = LEN_BITS'(o.beats) << 6;

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
        wr_req.vaddr  = rdma_staging_va;
        wr_req.len    = o_bytes + LEN_BITS'(64);
    end else begin
        wr_req.opcode = LOCAL_WRITE;
        wr_req.strm   = STRM_HOST;
        wr_req.dest   = HOST_DEST;
        wr_req.vaddr  = o.va;
        wr_req.len    = o_bytes;
    end
    wr_valid = (ostate == O_REQ) && (!o.route || win_ok);
end
assign rdma_post = wr_valid && wr_ready && o.route;

// Header lane 0: {zero, dst_pid, len[27:0], op}; lane 1: target VA
wire [63:0] hdr_q0 = {{(28-PID_BITS){1'b0}}, o.dst_pid, o_bytes, MSG_OP_WRITE};
wire [63:0] hdr_q1 = {{(64-VADDR_BITS){1'b0}}, o.va};

assign df_tready = (ostate == O_DATA) && o_rdy;

always_comb begin
    m_host_tdata  = df_tdata;
    m_host_tkeep  = '1;
    m_host_tlast  = o_last;
    m_host_tvalid = (ostate == O_DATA) && !o.route && df_tvalid;

    m_net_tdata   = (ostate == O_HDR) ? {{(AXI_DATA_BITS-128){1'b0}}, hdr_q1, hdr_q0} : df_tdata;
    m_net_tkeep   = '1;
    m_net_tlast   = (ostate == O_DATA) && o_last;
    m_net_tvalid  = (ostate == O_HDR) || ((ostate == O_DATA) && o.route && df_tvalid);
end

assign cnt_pkt_local = o_beat && o_last && !o.route;
assign cnt_pkt_rdma  = o_beat && o_last &&  o.route;

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
