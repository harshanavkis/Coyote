/**
 * Loom minus-nw vFPGA top: both ends of the link on one vFPGA.
 *
 * Same loom_ctrl/table/engine/rx as examples/loom (the HDL is symlinked, not
 * forked), with the RoCE stack removed and the engine's outgoing stream fed
 * straight back into loom_rx through a packetiser. EN_RDMA is 0, so nothing
 * between the two ends can retransmit, reorder or drop, and the receive path
 * is fed at the engine's full rate with no shell buffering in the way.
 *
 * What it is for: the two-host runs cannot separate "the host write path is
 * slower than the line" from "the network perturbed the stream". Here there
 * is no network, so loom_rx's move/starve/stall split measures the host
 * write path and nothing else.
 *
 * What it deliberately does NOT reproduce: retransmission. There is no
 * transport, so the beat displacement seen on hardware cannot arise on its
 * own - the packetiser's drop injection is how that is asked for instead.
 *
 * Original description follows.
 *
 * loom_ctrl captures aperture stores and DMA descriptors into one
 * arrival-ordered FIFO; loom_engine drains it (local writes via
 * sq_wr/axis_host_send, rdma via sq_wr/axis_rreq_send); loom_rx forwards
 * incoming RDMA writes (rq_wr + axis_rrsp_recv) as local writes.
 *
 * The engine and rx share sq_wr and axis_host_send[0]; a registered
 * arbiter grants the path to one of them for whole transactions
 * (engine priority). The engine is gated by masking its FIFO-empty input
 * rather than by an engine-side grant port.
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
logic eng_net_tready;

// The engine's rdma-route write request has no queue with EN_RDMA 0; it is
// acked and dropped, and only its payload matters (it goes to the loopback).
wire eng_wr_is_rdma = (eng_wr_req.strm == STRM_RDMA);

// Drop injection into the loopback. Tied off for now: wiring these to CSRs
// means adding words to loom_ctrl, which is shared with examples/loom, and
// the first thing this build is for is measuring the host write path with
// nothing perturbing the stream. The ports exist so that is a one-line
// change later.
wire [31:0] mnw_drop_at = 32'd0;
wire [7:0]  mnw_drop_n  = 8'd0;

// RX shell-side signals
req_t rx_wr_req;
logic rx_wr_valid;
logic rx_req_arb, rx_busy;
logic rx_cnt_move, rx_cnt_starve, rx_cnt_stall;
logic tx_cnt_move, tx_cnt_starve, tx_cnt_stall;
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

// ---------------------------------------------------------------------------
// Loopback: the engine's outgoing stream IS the receive path's ingress.
//
// No FIFO here on purpose. examples/loom puts one in front of loom_rx to
// keep host-write stalls off the RoCE ingress; with no RoCE stack there is
// nothing to protect, and feeding loom_rx with zero slack is what makes its
// stall counter a measurement of the host write path rather than of the
// buffer in front of it.
// ---------------------------------------------------------------------------
logic [AXI_DATA_BITS-1:0]   rxf_tdata;
logic [AXI_DATA_BITS/8-1:0] rxf_tkeep;
logic rxf_tvalid, rxf_tready, rxf_tlast;
logic [31:0] mnw_dropped;

loom_mnw_pktzr #(.BEATS_PER_PKT(64)) inst_pktzr (
    .aclk(aclk), .aresetn(aresetn),
    .drop_at(mnw_drop_at), .drop_n(mnw_drop_n),
    .s_tdata(eng_net_tdata), .s_tkeep(eng_net_tkeep),
    .s_tvalid(eng_net_tvalid), .s_tready(eng_net_tready),
    .s_tlast(eng_net_tlast),
    .m_tdata(rxf_tdata), .m_tkeep(rxf_tkeep),
    .m_tvalid(rxf_tvalid), .m_tready(rxf_tready),
    .m_tlast(rxf_tlast),
    .cnt_dropped(mnw_dropped)
);

// Arbiter: exclusive, whole-transaction ownership of {sq_wr, axis_host_send}
//
// Why it exists: the engine (stores, DMA writes, fences) and rx
// (forwarded incoming writes) both need the single sq_wr request channel
// and the single host output stream. Interleaving beats of two
// transactions on one stream would corrupt both, so ownership is granted
// for WHOLE transactions.
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
    .rd_resp_data(rd_resp_data), .rd_resp_valid(rd_resp_valid),
    .cnt_local_wr(cnt_local_wr), .cnt_rdma_wr(cnt_rdma_wr),
    .cnt_rx_fwd(cnt_rx_fwd), .cnt_rx_drop(cnt_rx_drop),
    .cnt_drop(cnt_drop), .cnt_compl(cnt_compl),
    .cnt_rx_move(rx_cnt_move), .cnt_rx_starve(rx_cnt_starve),
    .cnt_rx_stall(rx_cnt_stall), .cnt_rx_stall_head(rx_cnt_st_head),
    .cnt_rx_stall_body(rx_cnt_st_body), .cnt_rx_req(rx_cnt_req),
    .cnt_tx_move(tx_cnt_move), .cnt_tx_starve(tx_cnt_starve),
    .cnt_tx_stall(tx_cnt_stall),
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
    .wr_ready(eng_grant && (eng_wr_is_rdma ? 1'b1 : sq_wr.ready)),
    .s_tdata(axis_pull.tdata), .s_tkeep(axis_pull.tkeep),
    .s_tvalid(axis_pull.tvalid), .s_tready(axis_pull.tready),
    .s_tlast(axis_pull.tlast),
    .m_host_tdata(eng_host_tdata), .m_host_tkeep(eng_host_tkeep),
    .m_host_tvalid(eng_host_tvalid),
    .m_host_tready(axis_wr.tready && eng_grant),
    .m_host_tlast(eng_host_tlast),
    .m_net_tdata(eng_net_tdata), .m_net_tkeep(eng_net_tkeep),
    .m_net_tvalid(eng_net_tvalid),
    .m_net_tready(eng_net_tready),
    .m_net_tlast(eng_net_tlast),
    .rd_resp_data(rd_resp_data), .rd_resp_valid(rd_resp_valid),
    .cnt_local_wr(cnt_local_wr), .cnt_rdma_wr(cnt_rdma_wr),
    .cnt_drop(cnt_drop), .cnt_compl(cnt_compl),
    .stage_acc(stage_acc), .stage_cnt(stage_cnt),
    .cnt_tx_move(tx_cnt_move), .cnt_tx_starve(tx_cnt_starve),
    .cnt_tx_stall(tx_cnt_stall),
    .busy(eng_busy)
);

loom_rx inst_loom_rx (
    .aclk(aclk), .aresetn(aresetn),
    .rq_req('0), .rq_valid(1'b0), .rq_ready(),
    .rdma_staging_va(rdma_staging_va), .rx_pid(rx_pid),
    .wr_req(rx_wr_req), .wr_valid(rx_wr_valid),
    .wr_ready(sq_wr.ready && rx_grant),
    .s_tdata(rxf_tdata), .s_tkeep(rxf_tkeep),
    .s_tvalid(rxf_tvalid), .s_tready(rxf_tready),
    .s_tlast(rxf_tlast),
    .m_tdata(rx_tdata), .m_tkeep(rx_tkeep), .m_tvalid(rx_tvalid),
    .m_tready(axis_wr.tready && rx_grant), .m_tlast(rx_tlast),
    .req(rx_req_arb), .grant(rx_grant), .busy(rx_busy),
    .cnt_rx_move(rx_cnt_move), .cnt_rx_starve(rx_cnt_starve),
    .cnt_rx_stall(rx_cnt_stall), .cnt_rx_stall_head(rx_cnt_st_head),
    .cnt_rx_stall_body(rx_cnt_st_body), .cnt_rx_req(rx_cnt_req),
    .cnt_rx_span(rx_cnt_span),
    .cnt_rx_fwd(cnt_rx_fwd), .cnt_rx_drop(cnt_rx_drop)
);

// ---------------------------------------------------------------------------
// Shared-path muxes
//
// Data/valid toward the shared resources select on the current grant;
// the corresponding readys are masked on the way INTO each producer
// (see the eng/rx instantiations above: `sq_wr.ready && eng_grant` etc.),
// so an ungranted producer can neither drive nor mistakenly complete a
// handshake. Resources with a single user need no mux: sq_rd and the
// pull stream belong to the engine, the rdma TX stream to the engine,
// the rdma RX stream to loom_rx.
// ---------------------------------------------------------------------------
always_comb begin
    sq_wr.data  = rx_grant ? rx_wr_req  : eng_wr_req;
    sq_wr.valid = rx_grant ? rx_wr_valid
                           : (eng_wr_valid && eng_grant && !eng_wr_is_rdma);
end

always_comb begin
    sq_rd.data  = eng_rd_req;
    sq_rd.valid = eng_rd_valid;
end

always_comb begin
    axis_wr.tdata  = rx_grant ? rx_tdata  : eng_host_tdata;
    axis_wr.tkeep  = rx_grant ? rx_tkeep  : eng_host_tkeep;
    axis_wr.tlast  = rx_grant ? rx_tlast  : eng_host_tlast;
    axis_wr.tvalid = rx_grant ? rx_tvalid : (eng_host_tvalid && eng_grant);
    axis_wr.tid    = '0;
end



// ---------------------------------------------------------------------------
// Tie-offs
// ---------------------------------------------------------------------------
// rq_rd and the four rdma streams do not exist with EN_RDMA 0, so only the
// interfaces the shell still presents are tied off here.
always_comb notify.tie_off_m();
always_comb cq_rd.ready = 1'b1;
always_comb cq_wr.ready = 1'b1;
