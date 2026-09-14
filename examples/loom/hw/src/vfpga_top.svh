/**
 * Loom vFPGA top
 *
 * loom_ctrl captures aperture stores and DMA descriptors into one
 * arrival-ordered FIFO; loom_engine drains it (local writes via
 * sq_wr/axis_host_send, rdma via sq_wr/axis_rreq_send); loom_rx forwards
 * incoming RDMA writes (rq_wr + axis_rrsp_recv) as local writes.
 *
 * The engine and rx share sq_wr; a registered arbiter grants it to one of
 * them for whole transactions (engine priority). The engine is gated by
 * masking its FIFO-empty input rather than by an engine-side grant port.
 * Data goes out on separate host streams: the engine on axis_host_send[0],
 * rx on axis_host_send[1] (wr_req.dest 1), as perf_rdma's receiver does -
 * separate queue, credits and FIFO in the shell for each.
 */

// ---------------------------------------------------------------------------
// ctrl <-> table <-> engine wiring
// ---------------------------------------------------------------------------
logic                   tbl_commit;
logic [3:0]             tbl_idx;
logic                   tbl_valid;
logic                   tbl_route;
logic [PID_BITS-1:0]    tbl_pid;
logic [VADDR_BITS-1:0]  tbl_base;
logic [LEN_BITS-1:0]    tbl_len;

logic                   fifo_empty, fifo_is_desc, fifo_is_read, fifo_pop;
logic [3:0]             fifo_win;
logic [27:0]            fifo_off, fifo_len;
logic [PID_BITS-1:0]    fifo_src_pid;
logic [VADDR_BITS-1:0]  fifo_compl_va;
logic [63:0]            fifo_payload;

logic [3:0]             lu_idx;
logic                   lu_valid, lu_route;
logic [PID_BITS-1:0]    lu_pid;
logic [VADDR_BITS-1:0]  lu_base;
logic [LEN_BITS-1:0]    lu_len;

logic cnt_local_wr, cnt_rdma_wr, cnt_rx_fwd, cnt_rx_drop, cnt_drop, cnt_compl;
logic cnt_pull_desync;
logic cnt_rx_orphan;
logic cnt_tx_partial;
// Starvation landing INSIDE a wire packet - see loom_engine.sv
logic tx_cnt_starve_mid;
// cq_wr IS the engine's ack signal now - one per PACKET. build_sep6 tried
// to pace on it and wedged because the engine's internally split chunks
// carried last=0 and so produced no completion at all (71 acks for 65
// software chunks, 0 for engine chunks). req_t.last is what gates the ack
// (rdma_flow.sv: ack_que_in.valid = s_ack.data.last); the engine now posts
// one request per PMTU packet with last=1, so every packet acks. Only
// remote acks count: local writes (stores, fences, loom_rx's landings on
// the far side) complete on the same interface with remote clear.
wire ack_valid = cq_wr.valid && cq_wr.data.remote;
logic [7:0]  tx_window;
logic [15:0] tx_inflight;
logic       cnt_tx_ack, cnt_tx_winfull, cnt_tx_reqwait, cnt_tx_fifo_full;

// Stage cycle counters: engine -> ctrl (RO CSR words 50-63)
logic [63:0] stage_acc [7];
logic [63:0] stage_cnt [7];

// Aperture-read response: engine -> ctrl (completes the held-open AXI read)
logic [63:0] rd_resp_data;
logic        rd_resp_valid;

// RDMA staging VA: ctrl -> engine (RETH vaddr for all outgoing messages)
logic [VADDR_BITS-1:0] rdma_staging_va;
logic [PID_BITS-1:0]   rx_pid;

// Engine shell-side signals
req_t eng_rd_req, eng_wr_req;
logic eng_rd_valid, eng_wr_valid;
logic eng_busy;
logic [AXI_DATA_BITS-1:0]   eng_host_tdata, eng_net_tdata;
logic [AXI_DATA_BITS/8-1:0] eng_host_tkeep, eng_net_tkeep;
logic eng_host_tvalid, eng_host_tlast, eng_net_tvalid, eng_net_tlast;

// RX shell-side signals
req_t rx_wr_req;
logic rx_wr_valid;
logic rx_req_arb, rx_busy;
logic rx_cnt_move, rx_cnt_starve, rx_cnt_stall;
// Ingress backpressure in ANY state - see loom_rx.sv
logic rx_cnt_bp;
// Partial tkeep arriving from the network - see loom_rx.sv
logic rx_cnt_partial;
logic tx_cnt_move, tx_cnt_starve, tx_cnt_stall;
logic tx_cnt_paced, rx_cnt_fifo_full;
logic [7:0] pace_num, pace_den;
logic rx_cnt_st_head, rx_cnt_st_body, rx_cnt_req, rx_cnt_span;
logic [AXI_DATA_BITS-1:0]   rx_tdata;
logic [AXI_DATA_BITS/8-1:0] rx_tkeep;
logic rx_tvalid, rx_tlast;

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Pipeline registers on the host streams, as jigsaw's controller has. Without
// them a stream is combinational end to end: the pull's tready is derived
// from the network's, so every bubble on one side lands instantly on the
// other and there is no elasticity anywhere. The transmit path measures 84%
// starved waiting on the pull, which is what this is aimed at.
// ---------------------------------------------------------------------------
AXI4SR axis_pull (.*);
axisr_reg inst_reg_pull (.aclk(aclk), .aresetn(aresetn),
                         .s_axis(axis_host_recv[0]), .m_axis(axis_pull));

AXI4SR axis_wr (.*);
axisr_reg inst_reg_wr   (.aclk(aclk), .aresetn(aresetn),
                         .s_axis(axis_wr), .m_axis(axis_host_send[0]));

// rx's own landing stream (dest 1). Registered like the engine's.
AXI4SR axis_wr_rx (.*);
axisr_reg inst_reg_wr_rx (.aclk(aclk), .aresetn(aresetn),
                          .s_axis(axis_wr_rx), .m_axis(axis_host_send[1]));

// Arbiter: exclusive, whole-transaction ownership of sq_wr
//
// Why it exists: the engine (stores, DMA writes, fences) and rx
// (forwarded incoming writes) both need the single sq_wr request channel.
// The data streams are separate since dest 1 (axis_host_send[0] engine,
// [1] rx), so only the request channel is shared now; ownership is still
// granted for WHOLE transactions, which keeps the request/data pairing of
// each producer trivially in order and costs nothing on the receiver,
// where the engine is idle during a transfer.
//
// Why it is registered: a combinational arbiter here closes a loop -
// the grant would depend on the engine's pop decision, which depends on
// the (masked) fifo_empty, which would depend on the grant. Registering
// the decision breaks the loop at the cost of one idle cycle per
// ownership change, which is negligible against transaction lengths.
//
// How the engine is gated without a grant port: its fifo_empty input is
// OR-masked with !eng_grant, so outside its grant window the engine
// simply believes the FIFO is empty and stays in IDLE. rx has an
// explicit req/grant pair instead, because its trigger (rq_wr.valid)
// lives outside our modules. Engine priority is a policy choice: local
// work drains ahead of network ingress; rx work waits (bounded by the
// FIFO running dry) and cannot be starved indefinitely by design since
// software's aperture writes are finite.
// ---------------------------------------------------------------------------
typedef enum logic [1:0] { ARB_IDLE, ARB_ENG, ARB_RX } arb_t;
arb_t arb;
logic rx_started;

wire eng_grant = (arb == ARB_ENG);
wire rx_grant  = (arb == ARB_RX);

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        arb <= ARB_IDLE;
        rx_started <= 1'b0;
    end else begin
        case (arb)
            ARB_IDLE: begin
                rx_started <= 1'b0;
                if (!fifo_empty)      arb <= ARB_ENG;   // engine priority
                else if (rx_req_arb)  arb <= ARB_RX;
            end
            // Engine keeps the path while it has queued work; releasing
            // requires both an empty FIFO and a finished transaction
            ARB_ENG:
                if (fifo_empty && !eng_busy) arb <= ARB_IDLE;
            // rx_started distinguishes "granted but not yet begun" from
            // "finished": leave only after busy has risen and fallen
            ARB_RX: begin
                if (rx_busy) rx_started <= 1'b1;
                if (rx_started && !rx_busy) arb <= ARB_IDLE;
            end
            default: arb <= ARB_IDLE;
        endcase
    end
end

// ---------------------------------------------------------------------------
// Modules
// ---------------------------------------------------------------------------
loom_ctrl inst_loom_ctrl (
    .aclk(aclk), .aresetn(aresetn), .axi_ctrl(axi_ctrl),
    .tbl_commit(tbl_commit), .tbl_idx(tbl_idx), .tbl_valid(tbl_valid),
    .tbl_route(tbl_route), .tbl_pid(tbl_pid), .tbl_base(tbl_base),
    .tbl_len(tbl_len),
    .fifo_empty(fifo_empty), .fifo_is_desc(fifo_is_desc),
    .fifo_is_read(fifo_is_read),
    .fifo_win(fifo_win), .fifo_off(fifo_off), .fifo_len(fifo_len),
    .fifo_src_pid(fifo_src_pid), .fifo_compl_va(fifo_compl_va),
    .fifo_payload(fifo_payload), .fifo_pop(fifo_pop),
    .rdma_staging_va(rdma_staging_va), .rx_pid(rx_pid),
    .pace_num(pace_num), .pace_den(pace_den),
    .tx_window(tx_window), .tx_inflight(tx_inflight),
    .cnt_tx_ack(cnt_tx_ack), .cnt_tx_winfull(cnt_tx_winfull),
    .cnt_tx_reqwait(cnt_tx_reqwait), .cnt_tx_fifo_full(cnt_tx_fifo_full),
    .rd_resp_data(rd_resp_data), .rd_resp_valid(rd_resp_valid),
    .cnt_local_wr(cnt_local_wr), .cnt_rdma_wr(cnt_rdma_wr),
    .cnt_rx_fwd(cnt_rx_fwd), .cnt_rx_drop(cnt_rx_drop),
    .cnt_rx_orphan(cnt_rx_orphan),
    .cnt_drop(cnt_drop), .cnt_compl(cnt_compl),
    .cnt_rx_move(rx_cnt_move), .cnt_rx_starve(rx_cnt_starve),
    .cnt_rx_stall(rx_cnt_stall), .cnt_rx_bp(rx_cnt_bp), .cnt_rx_partial(rx_cnt_partial),
    .cnt_rx_stall_head(rx_cnt_st_head),
    .cnt_rx_stall_body(rx_cnt_st_body), .cnt_rx_req(rx_cnt_req),
    .cnt_pull_desync(cnt_pull_desync), .cnt_tx_partial(cnt_tx_partial),
    .cnt_tx_move(tx_cnt_move), .cnt_tx_starve(tx_cnt_starve),
    .cnt_tx_starve_mid(tx_cnt_starve_mid),
    .cnt_tx_stall(tx_cnt_stall),
    .cnt_tx_paced(tx_cnt_paced), .cnt_rx_fifo_full(rx_cnt_fifo_full),
    .cnt_rx_span(rx_cnt_span),
    .stage_acc(stage_acc), .stage_cnt(stage_cnt)
);

loom_table inst_loom_table (
    .aclk(aclk), .aresetn(aresetn),
    .commit(tbl_commit), .prog_idx(tbl_idx), .prog_valid(tbl_valid),
    .prog_route(tbl_route), .prog_pid(tbl_pid), .prog_base(tbl_base),
    .prog_len(tbl_len),
    .lu_idx(lu_idx), .lu_valid(lu_valid), .lu_route(lu_route),
    .lu_pid(lu_pid), .lu_base(lu_base), .lu_len(lu_len)
);

loom_engine inst_loom_engine (
    .aclk(aclk), .aresetn(aresetn),
    // FIFO-empty masked by the arbiter: the engine runs only while granted
    .fifo_empty(fifo_empty || !eng_grant),
    .fifo_is_desc(fifo_is_desc), .fifo_is_read(fifo_is_read),
    .fifo_win(fifo_win), .fifo_off(fifo_off),
    .fifo_len(fifo_len), .fifo_src_pid(fifo_src_pid),
    .fifo_compl_va(fifo_compl_va),
    .fifo_payload(fifo_payload), .fifo_pop(fifo_pop),
    .lu_idx(lu_idx), .lu_valid(lu_valid), .lu_route(lu_route),
    .lu_pid(lu_pid), .lu_base(lu_base), .lu_len(lu_len),
    .rdma_staging_va(rdma_staging_va),
    .rd_req(eng_rd_req), .rd_valid(eng_rd_valid), .rd_ready(sq_rd.ready),
    .wr_req(eng_wr_req), .wr_valid(eng_wr_valid),
    .wr_ready(sq_wr.ready && eng_grant),
    .s_tdata(axis_pull.tdata), .s_tkeep(axis_pull.tkeep),
    .s_tvalid(axis_pull.tvalid), .s_tready(axis_pull.tready),
    .s_tlast(axis_pull.tlast),
    .m_host_tdata(eng_host_tdata), .m_host_tkeep(eng_host_tkeep),
    .m_host_tvalid(eng_host_tvalid),
    .m_host_tready(axis_wr.tready && eng_grant),
    .m_host_tlast(eng_host_tlast),
    .m_net_tdata(eng_net_tdata), .m_net_tkeep(eng_net_tkeep),
    .m_net_tvalid(eng_net_tvalid),
    .m_net_tready(axis_rreq_send[0].tready),
    .m_net_tlast(eng_net_tlast),
    .rd_resp_data(rd_resp_data), .rd_resp_valid(rd_resp_valid),
    .cnt_local_wr(cnt_local_wr), .cnt_rdma_wr(cnt_rdma_wr),
    .cnt_drop(cnt_drop), .cnt_compl(cnt_compl),
    .stage_acc(stage_acc), .stage_cnt(stage_cnt),
    .cnt_pull_desync(cnt_pull_desync), .cnt_tx_partial(cnt_tx_partial),
    .cnt_tx_move(tx_cnt_move), .cnt_tx_starve(tx_cnt_starve),
    .cnt_tx_starve_mid(tx_cnt_starve_mid),
    .cnt_tx_stall(tx_cnt_stall),
    .cnt_tx_paced(tx_cnt_paced), .pace_num(pace_num), .pace_den(pace_den),
    .tx_window(tx_window), .ack_valid(ack_valid), .tx_inflight(tx_inflight),
    .cnt_tx_ack(cnt_tx_ack), .cnt_tx_winfull(cnt_tx_winfull),
    .cnt_tx_reqwait(cnt_tx_reqwait), .cnt_tx_fifo_full(cnt_tx_fifo_full),
    .busy(eng_busy)
);

// ---------------------------------------------------------------------------
// Ingress FIFO in front of loom_rx.
//
// loom_rx holds no buffer of its own, so its s_tready is the host write
// path's readiness; this decouples the two by 512 beats (~2 us at line
// rate). Backpressure inside the card cannot reach the wire - RoCE over
// plain Ethernet has no link-level flow control here - so once the
// receiving stack's own buffers (rx_crossing 2048 + incoming 512 beats,
// ~40 packets) fill, the CMAC drops. What keeps that from happening is
// the SENDER's window (loom_engine): at most tx_window packets are
// unacked, so a stall here, however long, backs up at most that many
// packets into the shell. cnt_rx_fifo_full counts the stall cycles.
// ---------------------------------------------------------------------------
logic [511:0] rxf_tdata;
logic [63:0]  rxf_tkeep;
logic         rxf_tvalid, rxf_tready, rxf_tlast;

axis_data_fifo_512 inst_rx_ingress_fifo (
    .s_axis_aclk(aclk), .s_axis_aresetn(aresetn),
    .s_axis_tdata(axis_rrsp_recv[0].tdata),
    .s_axis_tkeep(axis_rrsp_recv[0].tkeep),
    .s_axis_tvalid(axis_rrsp_recv[0].tvalid),
    .s_axis_tready(axis_rrsp_recv[0].tready),
    .s_axis_tlast(axis_rrsp_recv[0].tlast),
    .m_axis_tdata(rxf_tdata),
    .m_axis_tkeep(rxf_tkeep),
    .m_axis_tvalid(rxf_tvalid),
    .m_axis_tready(rxf_tready),
    .m_axis_tlast(rxf_tlast)
);

// The shell has a beat for us and the ingress FIFO will not take it: the
// FIFO is full. This is the one link in the loss chain nothing else counts.
// From here the backpressure walks into the shell (stack input stalls,
// rx_crossing fills) and the CMAC, which has no tready, drops. Nonzero on a
// corrupt run and zero on the clean control is the confirmation.
assign rx_cnt_fifo_full = axis_rrsp_recv[0].tvalid && !axis_rrsp_recv[0].tready;

loom_rx inst_loom_rx (
    .aclk(aclk), .aresetn(aresetn),
    .rq_req(rq_wr.data), .rq_valid(rq_wr.valid), .rq_ready(rq_wr.ready),
    .rdma_staging_va(rdma_staging_va), .rx_pid(rx_pid),
    .wr_req(rx_wr_req), .wr_valid(rx_wr_valid),
    .wr_ready(sq_wr.ready && rx_grant),
    .s_tdata(rxf_tdata), .s_tkeep(rxf_tkeep),
    .s_tvalid(rxf_tvalid), .s_tready(rxf_tready),
    .s_tlast(rxf_tlast),
    .m_tdata(rx_tdata), .m_tkeep(rx_tkeep), .m_tvalid(rx_tvalid),
    .m_tready(axis_wr_rx.tready && rx_grant), .m_tlast(rx_tlast),
    .req(rx_req_arb), .grant(rx_grant), .busy(rx_busy),
    .cnt_rx_move(rx_cnt_move), .cnt_rx_starve(rx_cnt_starve),
    .cnt_rx_stall(rx_cnt_stall), .cnt_rx_bp(rx_cnt_bp), .cnt_rx_partial(rx_cnt_partial),
    .cnt_rx_stall_head(rx_cnt_st_head),
    .cnt_rx_stall_body(rx_cnt_st_body), .cnt_rx_req(rx_cnt_req),
    .cnt_rx_span(rx_cnt_span),
    .cnt_rx_fwd(cnt_rx_fwd), .cnt_rx_drop(cnt_rx_drop),
    .cnt_rx_orphan(cnt_rx_orphan)
);

// ---------------------------------------------------------------------------
// Shared-path muxes
//
// Data/valid toward the shared resources select on the current grant;
// the corresponding readys are masked on the way INTO each producer
// (see the eng/rx instantiations above: `sq_wr.ready && eng_grant` etc.),
// so an ungranted producer can neither drive nor mistakenly complete a
// handshake. Resources with a single user need no mux: sq_rd, the pull
// stream and axis_host_send[0] belong to the engine, the rdma TX stream
// to the engine, the rdma RX stream and axis_host_send[1] to loom_rx.
// ---------------------------------------------------------------------------
always_comb begin
    sq_wr.data  = rx_grant ? rx_wr_req  : eng_wr_req;
    sq_wr.valid = rx_grant ? rx_wr_valid : (eng_wr_valid && eng_grant);
end

always_comb begin
    sq_rd.data  = eng_rd_req;
    sq_rd.valid = eng_rd_valid;
end

always_comb begin
    axis_wr.tdata  = eng_host_tdata;
    axis_wr.tkeep  = eng_host_tkeep;
    axis_wr.tlast  = eng_host_tlast;
    axis_wr.tvalid = eng_host_tvalid && eng_grant;
    axis_wr.tid    = '0;
end

always_comb begin
    axis_wr_rx.tdata  = rx_tdata;
    axis_wr_rx.tkeep  = rx_tkeep;
    axis_wr_rx.tlast  = rx_tlast;
    axis_wr_rx.tvalid = rx_tvalid && rx_grant;
    axis_wr_rx.tid    = '0;
end

always_comb begin
    axis_rreq_send[0].tdata  = eng_net_tdata;
    axis_rreq_send[0].tkeep  = eng_net_tkeep;
    axis_rreq_send[0].tlast  = eng_net_tlast;
    axis_rreq_send[0].tvalid = eng_net_tvalid;
    axis_rreq_send[0].tid    = '0;
end

// ---------------------------------------------------------------------------
// Tie-offs
// ---------------------------------------------------------------------------
always_comb notify.tie_off_m();
always_comb cq_rd.ready = 1'b1;
// cq_wr is always taken; the engine counts the remote ones (ack_valid above)
always_comb cq_wr.ready = 1'b1;
always_comb rq_rd.ready = 1'b1;

always_comb axis_host_recv[1].tie_off_s();   // only the engine pulls, on [0]
always_comb axis_rreq_recv[0].tie_off_s();
always_comb axis_rrsp_send[0].tie_off_m();
