`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_loom_loopback — the two-host data plane in one simulation.
 *
 * Client half (real loom_ctrl + loom_table + loom_engine, wired as in
 * vfpga_top) drives its rdma-route windows; every outgoing rdma request
 * and its beats are handed to a model of the shell's RDMA path, which
 * fragments them at PMTU exactly the way ib_transport_protocol does, and
 * the far side (real loom_rx) lands them in a modelled host memory.
 *
 * Why this exists on top of the per-module TBs: loom_engine's header
 * ENCODE and loom_rx's header DECODE are two hand-written bit layouts in
 * two files held together by a "keep in sync" comment, and each block TB
 * checks its own side against literals it writes itself - so both can
 * agree on the same misunderstanding and still pass. Here the bits only
 * ever exist as the engine emits them, and the check is what shows up in
 * far-side memory. The sequence is loom_host's, operation for operation.
 *
 * Shell model (from ib_transport_protocol.cpp + req_t in lynx_pkg):
 *   - a request of len L becomes ceil(L/PMTU) rq_wr's, vaddr advancing by
 *     PMTU (RETH on the first, MSN cursor after that);
 *   - rq_wr.last is high only on the final fragment, and the payload
 *     stream carries exactly one tlast, at the end of the whole message.
 */
module tb_loom_loopback;

localparam int PMTU = 4096;

// A request merged across packets whose payload is still framed per packet.
// loom_rx takes stream_end from s_tlast whenever rq_wr.last is high, so such
// a request ends at the FIRST packet's tlast while the sq_wr it already
// issued claimed the whole merged length - the far shell then completes that
// write from the next beats. It shifts the payload AND completes fewer
// transactions than were issued, which is the pair of symptoms hardware
// shows. Held OFF because the shell in this tree cannot emit that shape (see
// the case itself); turn it on the day one does, or to see the hazard.
localparam bit MERGED_REQ_HAZARD = 0;

// Beats per local-read segment, 0 = one unbroken response. The engine must
// forward what its request claimed regardless of how the pull is chopped up.
localparam int PULL_SEG = 4;

logic aclk = 0;
logic aresetn = 0;
always #2 aclk = ~aclk;

AXI4L axi_ctrl (.aclk(aclk), .aresetn(aresetn));

// ---- client half: ctrl <-> table <-> engine ----
logic                  tbl_commit;
logic [3:0]            tbl_idx;
logic                  tbl_valid, tbl_route;
logic [PID_BITS-1:0]   tbl_pid;
logic [VADDR_BITS-1:0] tbl_base;
logic [LEN_BITS-1:0]   tbl_len;

logic                  fifo_empty, fifo_is_desc, fifo_is_read, fifo_pop;
logic [3:0]            fifo_win;
logic [27:0]           fifo_off, fifo_len;
logic [PID_BITS-1:0]   fifo_src_pid;
logic [VADDR_BITS-1:0] fifo_compl_va;
logic [63:0]           fifo_payload;

logic [3:0]            lu_idx;
logic                  lu_valid, lu_route;
logic [PID_BITS-1:0]   lu_pid;
logic [VADDR_BITS-1:0] lu_base;
logic [LEN_BITS-1:0]   lu_len;

logic cnt_local_wr, cnt_rdma_wr, cnt_drop, cnt_compl;
logic [63:0] rd_resp_data;
logic        rd_resp_valid;
logic [VADDR_BITS-1:0] rdma_staging_va;
logic [PID_BITS-1:0]   ctrl_rx_pid;
logic [63:0] stage_acc [7];
logic [63:0] stage_cnt [7];

req_t eng_rd_req, eng_wr_req;
logic eng_rd_valid, eng_wr_valid;
logic eng_rd_ready = 1, eng_wr_ready = 1;

logic [AXI_DATA_BITS-1:0]   pull_tdata = 0;
logic [AXI_DATA_BITS/8-1:0] pull_tkeep = 0;
logic pull_tvalid = 0, pull_tready, pull_tlast = 0;

logic [AXI_DATA_BITS-1:0]   m_host_tdata, m_net_tdata;
logic [AXI_DATA_BITS/8-1:0] m_host_tkeep, m_net_tkeep;
logic m_host_tvalid, m_net_tvalid, m_host_tlast, m_net_tlast;
logic [AXI_DATA_BITS-1:0] net_beats[$];
logic m_host_tready = 1;

// The network side does NOT take beats unconditionally. The shell buffers
// what it has not yet put on the wire, and when that fills it stops
// accepting - RDMA_N_WR_OUTSTANDING packets' worth, which at PMTU 4096 is
// 1024 beats. The model used to hold m_net_tready high forever, so no TB
// has ever driven the engine against a full window, and on hardware a 4 MB
// message (65537 beats, 1025 packets) parked it in ST_STREAM for 1.25
// billion cycles with the network refusing every beat.
localparam int NET_WINDOW_BEATS = 16 * (PMTU / 64);
wire m_net_tready = (net_beats.size() < NET_WINDOW_BEATS);
logic eng_busy;

int errors = 0;

loom_ctrl inst_ctrl (
    .aclk(aclk), .aresetn(aresetn), .axi_ctrl(axi_ctrl),
    .tbl_commit(tbl_commit), .tbl_idx(tbl_idx), .tbl_valid(tbl_valid),
    .tbl_route(tbl_route), .tbl_pid(tbl_pid), .tbl_base(tbl_base),
    .tbl_len(tbl_len),
    .fifo_empty(fifo_empty), .fifo_is_desc(fifo_is_desc),
    .fifo_is_read(fifo_is_read),
    .fifo_win(fifo_win), .fifo_off(fifo_off), .fifo_len(fifo_len),
    .fifo_src_pid(fifo_src_pid), .fifo_compl_va(fifo_compl_va),
    .fifo_payload(fifo_payload), .fifo_pop(fifo_pop),
    .rdma_staging_va(rdma_staging_va), .rx_pid(ctrl_rx_pid),
    .rd_resp_data(rd_resp_data), .rd_resp_valid(rd_resp_valid),
    .cnt_local_wr(cnt_local_wr), .cnt_rdma_wr(cnt_rdma_wr),
    .cnt_rx_fwd(1'b0), .cnt_rx_drop(1'b0), .cnt_drop(cnt_drop),
    .cnt_compl(cnt_compl),
    .stage_acc(stage_acc), .stage_cnt(stage_cnt)
);

loom_table inst_table (
    .aclk(aclk), .aresetn(aresetn),
    .commit(tbl_commit), .prog_idx(tbl_idx), .prog_valid(tbl_valid),
    .prog_route(tbl_route), .prog_pid(tbl_pid), .prog_base(tbl_base),
    .prog_len(tbl_len),
    .lu_idx(lu_idx), .lu_valid(lu_valid), .lu_route(lu_route),
    .lu_pid(lu_pid), .lu_base(lu_base), .lu_len(lu_len)
);

loom_engine inst_engine (
    .aclk(aclk), .aresetn(aresetn),
    .fifo_empty(fifo_empty), .fifo_is_desc(fifo_is_desc),
    .fifo_is_read(fifo_is_read),
    .fifo_win(fifo_win), .fifo_off(fifo_off), .fifo_len(fifo_len),
    .fifo_src_pid(fifo_src_pid), .fifo_compl_va(fifo_compl_va),
    .fifo_payload(fifo_payload), .fifo_pop(fifo_pop),
    .lu_idx(lu_idx), .lu_valid(lu_valid), .lu_route(lu_route),
    .lu_pid(lu_pid), .lu_base(lu_base), .lu_len(lu_len),
    .rdma_staging_va(rdma_staging_va),
    .rd_req(eng_rd_req), .rd_valid(eng_rd_valid), .rd_ready(eng_rd_ready),
    .wr_req(eng_wr_req), .wr_valid(eng_wr_valid), .wr_ready(eng_wr_ready),
    .s_tdata(pull_tdata), .s_tkeep(pull_tkeep), .s_tvalid(pull_tvalid),
    .s_tready(pull_tready), .s_tlast(pull_tlast),
    .m_host_tdata(m_host_tdata), .m_host_tkeep(m_host_tkeep),
    .m_host_tvalid(m_host_tvalid), .m_host_tready(m_host_tready),
    .m_host_tlast(m_host_tlast),
    .m_net_tdata(m_net_tdata), .m_net_tkeep(m_net_tkeep),
    .m_net_tvalid(m_net_tvalid), .m_net_tready(m_net_tready),
    .m_net_tlast(m_net_tlast),
    .rd_resp_data(rd_resp_data), .rd_resp_valid(rd_resp_valid),
    .cnt_local_wr(cnt_local_wr), .cnt_rdma_wr(cnt_rdma_wr),
    .cnt_drop(cnt_drop), .cnt_compl(cnt_compl),
    .stage_acc(stage_acc), .stage_cnt(stage_cnt),
    .busy(eng_busy)
);

// ---- far side: real loom_rx ----
req_t rx_rq_req;
logic rx_rq_valid = 0, rx_rq_ready;
req_t rx_wr_req;
logic rx_wr_valid;
logic rx_wr_ready = 1;

logic [AXI_DATA_BITS-1:0]   rx_s_tdata = 0;
logic [AXI_DATA_BITS/8-1:0] rx_s_tkeep = 0;
logic rx_s_tvalid = 0, rx_s_tready, rx_s_tlast = 0;

logic [AXI_DATA_BITS-1:0]   rx_m_tdata;
logic [AXI_DATA_BITS/8-1:0] rx_m_tkeep;
logic rx_m_tvalid, rx_m_tlast;
logic rx_m_tready = 1;
logic rx_req_arb, rx_busy, rx_fwd, rx_drop;

localparam [47:0] STAGING  = 48'h7d24_8ca0_0000;   // exporter's staging VA
localparam [47:0] BASE1    = 48'h7f1b_d420_0000;   // exported dst1
localparam [47:0] BASE2    = 48'h7f9e_8860_0000;   // exported dst2
localparam [47:0] SRC_VA   = 48'h7f6a_2000_0000;   // importer's source buffer
localparam [47:0] CPL_VA   = 48'h7f6a_3000_0000;   // importer's fence word
localparam [PID_BITS-1:0] QP_OWNER = 6'd1;         // exporter's data ctid

loom_rx inst_rx (
    .aclk(aclk), .aresetn(aresetn),
    .rq_req(rx_rq_req), .rq_valid(rx_rq_valid), .rq_ready(rx_rq_ready),
    .rdma_staging_va(STAGING), .rx_pid(QP_OWNER),
    .wr_req(rx_wr_req), .wr_valid(rx_wr_valid), .wr_ready(rx_wr_ready),
    .s_tdata(rx_s_tdata), .s_tkeep(rx_s_tkeep), .s_tvalid(rx_s_tvalid),
    .s_tready(rx_s_tready), .s_tlast(rx_s_tlast),
    .m_tdata(rx_m_tdata), .m_tkeep(rx_m_tkeep), .m_tvalid(rx_m_tvalid),
    .m_tready(rx_m_tready), .m_tlast(rx_m_tlast),
    .req(rx_req_arb), .grant(1'b1), .busy(rx_busy),
    .cnt_rx_fwd(rx_fwd), .cnt_rx_drop(rx_drop)
);

// -------------------------------------------------------------------------
// Source buffer contents: the pattern loom_host writes into src
// -------------------------------------------------------------------------
function automatic logic [63:0] src_word(input int idx);
    src_word = {32'h5A5A_0000, 32'(idx)};
endfunction

// -------------------------------------------------------------------------
// Pull responder: answers every sq_rd the way the shell's host-read path
// would, with the source buffer's contents
// -------------------------------------------------------------------------
req_t pull_req;
int   pull_beats;

// One EXTRA beat on the pull response, once. The engine ends its outgoing
// stream on a free-running count (loom_engine stream_last = l_sbeats <= 1)
// and never checks that a beat belongs to the request it asked for, so a
// single surplus beat on the shared axis_host_recv should shift the payload
// permanently - with no packet loss, no reordering and no retransmission
// anywhere in this model.
bit pull_extra_beat = 0;
bit pull_extra_done = 0;
int pull_extra_count = 0;
initial forever begin
    @(posedge aclk);
    if (eng_rd_valid && eng_rd_ready) begin
        pull_req   = eng_rd_req;
        pull_beats = (pull_req.len + 63) / 64;
        for (int b = 0; b < pull_beats; b++) begin
            @(negedge aclk);
            for (int l = 0; l < 8; l++)
                pull_tdata[64*l +: 64] =
                    src_word(int'((pull_req.vaddr - SRC_VA) >> 3) + b*8 + l);
            pull_tkeep  = {64{1'b1}};
            pull_tvalid = 1;
            // Segment the response: the shell may return a large local read
            // as several tlast-terminated chunks, and an engine that treats
            // the first tlast as the end of the transfer leaves the sq_wr it
            // already posted short of the length it claimed. The far side
            // then completes that request with the NEXT transaction's beat.
            pull_tlast  = (b == pull_beats - 1) ||
                          (PULL_SEG != 0 && ((b + 1) % PULL_SEG == 0));
            do @(posedge aclk); while (!pull_tready);
        end
        // The surplus beat: valid data, correct framing, simply one more
        // than was asked for.
        if (pull_extra_beat && !pull_extra_done) begin
            pull_extra_done = 1;
            @(negedge aclk);
            for (int l = 0; l < 8; l++)
                pull_tdata[64*l +: 64] = 64'hDEAD_0000 + 64'(l);
            pull_tkeep  = {64{1'b1}};
            pull_tvalid = 1;
            pull_tlast  = 1;
            do @(posedge aclk); while (!pull_tready);
            pull_extra_count++;
        end
        @(negedge aclk);
        pull_tvalid = 0; pull_tlast = 0;
    end
end

// -------------------------------------------------------------------------
// Shell model: the engine's rdma requests and their beats become RoCE
// packets on the far side. Requests and payload are captured first so the
// fragmenter can present them with the shell's framing.
// -------------------------------------------------------------------------
typedef struct { logic [47:0] vaddr; logic [27:0] len; } wreq_t;
wreq_t net_reqs[$];
int fences = 0;

// Beat accounting. A request claims `len` bytes; the engine must put
// exactly ceil(len/64) beats on the net stream for it and no more. On
// hardware a mismatch does not fail loudly - the shell simply pairs the
// next transaction's beat with this request, so an inline store message
// was seen landing verbatim inside a bulk destination, header and all, and
// every pairing after it is shifted.
int net_owed = 0;                      // beats the posted requests still owe
int net_extra = 0;                     // beats nobody asked for

always @(posedge aclk) begin
    if (eng_wr_valid && eng_wr_ready) begin
        if (eng_wr_req.strm == STRM_RDMA) begin
            net_reqs.push_back('{eng_wr_req.vaddr, eng_wr_req.len});
            net_owed += int((eng_wr_req.len + 63) / 64);
        end else
            fences++;                       // local completion write
    end
    if (m_net_tvalid && m_net_tready) begin
        net_beats.push_back(m_net_tdata);
        if (net_owed > 0) net_owed--;
        else              net_extra++;      // a beat outside any request
    end
end

// The link has two independent streams, and modelling them as one is what
// used to hide this class of bug. rq_wr and the payload are separate AXI
// channels: the shell does NOT wait for the far side to finish a
// transaction before pushing the next request, and it does not stop
// streaming payload at a receiver that is still owed beats. So a
// transaction that ends short does not stall the link - the beats of the
// NEXT transaction flow into the receiver that is still waiting, and land
// wherever the unfinished request said they should. Driving both from one
// task, gated on the far side being idle, turns that into a deadlock the
// model reports as "engine never produced the payload", which is a
// statement about the model and not about the hardware.
//
// How many PMTU packets the shell merges into one rq_wr. Hardware shows
// BOTH shapes on the same workload: rx_fwd counted 17x64 requests for
// 17x256 KB (one request per packet, req_pkts 1) on one run and ~26 per
// transfer (~10 KB each, req_pkts ~3) on the next, for a 4.7x throughput
// difference. loom_rx derives its beat budget from rq_wr.len, so the two
// shapes exercise different arithmetic in it, and only the first has ever
// been simulated. The test sequence sets this between cases.
int req_pkts = 1;

// Where TLAST falls in the payload stream. The model has always put ONE
// tlast at the end of a whole message, but hardware assembles the receive
// payload per PACKET (merge_rx_pkgs), and one memCmd is emitted per packet
// with PKG_NF on FIRST/MIDDLE and PKG_F on LAST/ONLY - which is what
// becomes rq_wr.last (roce_stack.sv:232). loom_rx's stream_end reads
// `l_req_last ? s_tlast : (l_beats <= 1)`, so the two placements take
// different branches on every fragment but the last, and only one of them
// has ever been simulated. 1 = a tlast per packet, as the RX path builds it.
int tlast_per_pkt = 0;

// ---------------------------------------------------------------------------
// Beat loss: what a retransmission does to the OUTGOING payload stream.
//
// The shell does not replay a buffered packet on retransmit. It re-reads
// localAddr/length and feeds that into the same transmit path the engine is
// streaming into (ib_transport_protocol.cpp writes memCmdInternal with
// rev.localAddr). The two share the outgoing payload path, so the engine's
// in-flight message loses beats to the replay - which is why hardware showed
// data arriving EARLY (word 103218 held the value belonging 16 words later),
// i.e. beats missing ahead of it, rather than duplicated.
//
// The duplicate packets themselves never reach loom_rx: the receiver drops
// them by PSN (93 of them on the run above) BEFORE the payload path. So the
// damage is entirely on the sender, and modelling it means removing beats
// from the middle of a message, not injecting any.
//
// drop_at_beat counts within the message's payload; 0 disables.
int drop_at_beat  = 0;
int drop_n_beats  = 0;
int beats_dropped = 0;
// Position within the MESSAGE, not within a fragment: bi in the payload
// driver restarts at every PMTU frame, so it never reaches a megabyte.
int msg_beat      = 0;
bit drop_done     = 0;      // steal from ONE message, as one retransmit would

// Frame boundaries the payload driver needs from the request driver: how
// many beats each fragment carries, and which one ends the message (the
// single tlast).
typedef struct { int beats; bit last_frag; } frame_t;
frame_t net_frames[$];

// Request driver: ceil(len/PMTU) fragments per message, vaddr advancing,
// rq_wr.last only on the final one
task deliver_reqs();
    wreq_t w;
    int nfrag, frag_len, guard, span;
    w = net_reqs.pop_front();
    span  = PMTU * req_pkts;
    nfrag = (w.len + span - 1) / span;
    for (int f = 0; f < nfrag; f++) begin
        frag_len = (f == nfrag - 1) ? (w.len - f*span) : span;
        @(negedge aclk);
        rx_rq_req = '0;
        rx_rq_req.pid   = QP_OWNER;
        rx_rq_req.vaddr = w.vaddr + f*span;
        rx_rq_req.len   = frag_len;
        rx_rq_req.last  = (f == nfrag - 1);
        rx_rq_valid = 1;
        net_frames.push_back('{(frag_len + 63) / 64, (f == nfrag - 1)});
        guard = 0;
        do begin @(posedge aclk); guard++; end
        while (!rx_rq_ready && guard < 20000);
        @(negedge aclk);
        rx_rq_valid = 0;
        if (guard >= 20000) begin
            check(1'b0, $sformatf(
                "far side never accepted the request @%0h (stuck)", w.vaddr));
            return;
        end
    end
endtask

initial forever begin
    @(posedge aclk);
    if (net_reqs.size() > 0) deliver_reqs();
end

// Payload driver: independent of the requests, as the stream is on
// hardware. It never asks which transaction produced a beat - it puts
// them on the wire in the order they were produced, and the far side
// pairs them with whatever request it is currently serving.
initial forever begin
    frame_t fr;
    int guard;
    @(posedge aclk);
    if (net_frames.size() > 0) begin
        fr = net_frames.pop_front();
        for (int bi = 0; bi < fr.beats; bi++) begin  // msg_beat spans frames
            // Long enough that a beat from a LATER transaction still
            // arrives; only a client with nothing left to send reaches it
            guard = 0;
            while (net_beats.size() == 0 && guard < 4000) begin
                @(posedge aclk); guard++;
            end
            if (net_beats.size() == 0) begin
                // Expected when beats have been stolen: the message really
                // IS short, which is the whole point of the case.
                if (beats_dropped == 0)
                    check(1'b0, $sformatf(
                        "shell model: payload starved (%0d of %0d beats)",
                        bi, fr.beats));
                break;
            end
            // Steal beats the way the shell's retransmit re-read does
            if (drop_at_beat != 0 && !drop_done && msg_beat == drop_at_beat) begin
                drop_done = 1;
                for (int d = 0; d < drop_n_beats; d++)
                    if (net_beats.size() > 0) begin
                        void'(net_beats.pop_front());
                        beats_dropped++;
                    end
            end
            if (net_beats.size() == 0) break;
            @(negedge aclk);
            rx_s_tdata  = net_beats.pop_front();
            rx_s_tkeep  = {64{1'b1}};
            rx_s_tvalid = 1;
            rx_s_tlast  = tlast_per_pkt
                          ? (((bi % (PMTU/64)) == (PMTU/64 - 1)) ||
                             (bi == fr.beats - 1))
                          : ((bi == fr.beats - 1) && fr.last_frag);
            guard = 0;
            do begin @(posedge aclk); guard++; end
            while (!rx_s_tready && guard < 20000);
            @(negedge aclk);
            rx_s_tvalid = 0; rx_s_tlast = 0;
            msg_beat++;
            if (guard >= 20000) begin
                check(1'b0, "far side stopped taking beats (stuck mid-transaction)");
                break;
            end
        end
        // A message ends at its last fragment; position restarts there, the
        // same way loom_rx restarts its own count at the next header.
        if (fr.last_frag) msg_beat = 0;
    end
end

// -------------------------------------------------------------------------
// Far-side host memory: what loom_rx actually lands, and in what order
// -------------------------------------------------------------------------
logic [63:0] mem [logic [47:0]];      // keyed by vaddr >> 3
logic [47:0] writes[$];               // vaddr of every word written, in order

logic [47:0] wr_cursor;
logic [27:0] wr_left;
int rx_txns = 0;                      // loom_rx transactions completed
initial forever begin
    @(posedge aclk);
    if (rx_wr_valid && rx_wr_ready) begin
        wr_cursor = rx_wr_req.vaddr;
        wr_left   = rx_wr_req.len;
        rx_txns++;
    end
    if (rx_m_tvalid && rx_m_tready) begin
        for (int l = 0; l < 8 && wr_left > 0; l++) begin
            mem[(wr_cursor + l*8) >> 3] = rx_m_tdata[64*l +: 64];
            writes.push_back(wr_cursor + l*8);
            wr_left = (wr_left >= 8) ? wr_left - 8 : 0;
        end
        wr_cursor = wr_cursor + 64;
    end
end

// -------------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------------
task check(input bit cond, input string msg);
    if (!cond) begin
        errors++;
        $display("FAIL: %s", msg);
    end
endtask

task axil_write(input [15:0] addr, input [63:0] data);
    @(negedge aclk);
    axi_ctrl.awaddr = {48'b0, addr}; axi_ctrl.awvalid = 1;
    axi_ctrl.wdata = data; axi_ctrl.wstrb = 8'hFF; axi_ctrl.wvalid = 1;
    axi_ctrl.bready = 1;
    do @(posedge aclk); while (!(axi_ctrl.awready && axi_ctrl.wready));
    @(negedge aclk);
    axi_ctrl.awvalid = 0; axi_ctrl.wvalid = 0;
    while (!axi_ctrl.bvalid) @(posedge aclk);
    @(negedge aclk);
endtask

// importBuf: an rdma window at the exporter's VA, pid = local QP owner
task import_win(input [3:0] idx, input [63:0] base, input [63:0] len);
    axil_write(16'd0,  {60'b0, idx});
    axil_write(16'd8,  64'b11);                  // valid + rdma route
    axil_write(16'd16, {58'b0, QP_OWNER});
    axil_write(16'd24, base);
    axil_write(16'd32, len);
    axil_write(16'd40, 64'd1);
endtask

// releaseWindow: exactly what BundledOrchestrator writes
task release_win(input [3:0] idx);
    axil_write(16'd0,  {60'b0, idx});
    axil_write(16'd8,  64'd0);
    axil_write(16'd40, 64'd1);
endtask

task store(input [3:0] win, input [11:0] off, input [63:0] val);
    axil_write({win, off}, val);
endtask

task copy(input [3:0] win, input [27:0] off, input [63:0] src_va,
          input [27:0] len, input [63:0] compl);
    axil_write(16'd64,  {win, 32'b0, off});
    axil_write(16'd72,  src_va);
    axil_write(16'd80,  {36'b0, len});
    axil_write(16'd88,  {58'b0, QP_OWNER});
    axil_write(16'd104, compl);
    axil_write(16'd96,  64'd1);
endtask

// Wait for the client and the link to go quiet. Delivery itself is the
// free-running process above: on hardware the shell drains the net stream
// while the client is still issuing, and pumping it from this thread
// instead would hide every ordering effect between the two - including a
// short transaction whose missing beats are supplied by the transaction
// that follows it, which cannot happen if the client is blocked here
// while the link waits.
task settle();
    int n;
    n = 0;
    while (n < 4000000 && (!fifo_empty || eng_busy || net_reqs.size() > 0 ||
                         rx_busy || net_beats.size() > 0 ||
                         net_frames.size() > 0)) begin
        @(posedge aclk); n++;
    end
    repeat (10) @(posedge aclk);
    if (net_reqs.size() > 0 || net_beats.size() > 0 || net_frames.size() > 0)
        check(1'b0, "shell model: message left undelivered");
endtask

task check_payload(input [47:0] at, input int words, input string msg);
    bit ok;
    int first;
    ok = 1;
    first = -1;
    for (int i = 0; i < words; i++)
        if (!mem.exists((at + i*8) >> 3) || mem[(at + i*8) >> 3] != src_word(i)) begin
            ok = 0;
            if (first < 0) first = i;
        end
    check(ok, msg);
    // What sat in the first bad word is the evidence: on hardware the
    // corruption was an intact inline wire message (op 2 in lane 0, target
    // VA in lane 1) written verbatim into the bulk destination, so print
    // the word and its neighbours rather than only the fact of a mismatch
    if (!ok) begin
        $display("       first bad word %0d @0x%0h: got %0h want %0h",
                 first, at + first*8,
                 mem.exists((at + first*8) >> 3) ? mem[(at + first*8) >> 3] : 64'hx,
                 src_word(first));
        for (int l = 0; l < 3; l++)
            if (mem.exists((at + (first+l)*8) >> 3))
                $display("       lane %0d: %0h", l, mem[(at + (first+l)*8) >> 3]);
    end
endtask

int n_writes_before;
int landed, ovf, fwd_before;
logic [63:0] ovf_before;

initial begin
    axi_ctrl.awvalid = 0; axi_ctrl.wvalid = 0; axi_ctrl.arvalid = 0;
    axi_ctrl.bready = 0; axi_ctrl.rready = 0;
    axi_ctrl.awaddr = 0; axi_ctrl.wdata = 0; axi_ctrl.wstrb = 0;
    axi_ctrl.araddr = 0;
    rx_rq_req = '0;

    repeat (5) @(negedge aclk);
    aresetn = 1;
    repeat (2) @(negedge aclk);

    // attachPeer: staging CSR from the exporter's hello, then two imports
    axil_write(16'd128, {16'b0, STAGING});
    import_win(4'd1, {16'b0, BASE1}, 64'h100_0000);   // 16 MB: MB-scale cases below
    import_win(4'd2, {16'b0, BASE2}, 64'h100_0000);

    // --- loom_host: small stores through both windows ---
    store(4'd1, 12'h40, 64'hB00B_0000_0000_0001);
    store(4'd2, 12'h40, 64'hB00B_0000_0000_0002);
    settle();
    check(mem.exists((BASE1 + 48'h40) >> 3) &&
          mem[(BASE1 + 48'h40) >> 3] == 64'hB00B_0000_0000_0001,
          "store via w1 lands at the exporter's VA");
    check(mem.exists((BASE2 + 48'h40) >> 3) &&
          mem[(BASE2 + 48'h40) >> 3] == 64'hB00B_0000_0000_0002,
          "store via w2 lands at the exporter's VA");
    check(writes.size() == 2, "stores wrote one word each, nothing else");

    // --- loom_host: bulk with fence ---
    copy(4'd1, 28'h10000, {16'b0, SRC_VA}, 28'd4096, {16'b0, CPL_VA});
    settle();
    check_payload(BASE1 + 48'h10000, 512, "bulk payload @0x10000 matches src");
    check(fences == 1, "fence 1 written locally");

    // --- loom_host: ordering copy, then the flag through the other window ---
    copy(4'd1, 28'h20000, {16'b0, SRC_VA}, 28'd4096, {16'b0, CPL_VA});
    store(4'd2, 12'h800, 64'hF1A6);
    settle();
    check_payload(BASE1 + 48'h20000, 512, "bulk payload @0x20000 matches src");
    check(mem.exists((BASE2 + 48'h800) >> 3) &&
          mem[(BASE2 + 48'h800) >> 3] == 64'hF1A6, "ordering flag lands");
    check(writes[writes.size()-1] == BASE2 + 48'h800,
          "flag is the LAST word written: payload complete before it");
    check(fences == 2, "fence 2 written locally");

    // --- loom_host: release, then a store to the released window ---
    n_writes_before = writes.size();
    release_win(4'd2);
    store(4'd2, 12'h40, 64'hDEAD);
    settle();
    check(writes.size() == n_writes_before,
          "store to a released window never reaches the far side");

    // --- beyond loom_host: a copy larger than PMTU, so the shell
    //     fragments it into FIRST/MIDDLE/LAST (rq_wr.last low on all but
    //     the last, one tlast for the whole message). T2 and 6.2b get
    //     here the moment a transfer exceeds 4 KB
    copy(4'd1, 28'h30000, {16'b0, SRC_VA}, 28'd12288, {16'b0, CPL_VA});
    settle();
    check_payload(BASE1 + 48'h30000, 1536, "fragmented copy lands intact");

    // A message after a fragmented one must still parse from its own header
    store(4'd1, 12'h80, 64'hAF7E_0001);
    settle();
    check(mem.exists((BASE1 + 48'h80) >> 3) &&
          mem[(BASE1 + 48'h80) >> 3] == 64'hAF7E_0001,
          "store after a fragmented copy parses its own header");

    // --- Boundary sizes. 4096 and 12288 are both whole multiples of PMTU,
    //     so nothing above exercises the minimum transfer or an uneven final
    //     fragment - and the short last fragment is where the beat budget has
    //     to land exactly, on a request whose rq_wr.last is high but which
    //     carries a single beat
    copy(4'd1, 28'h40000, {16'b0, SRC_VA}, 28'd64, {16'b0, CPL_VA});
    settle();
    check_payload(BASE1 + 48'h40000, 8, "minimum bulk: 64 B, one beat, lands");

    copy(4'd1, 28'h50000, {16'b0, SRC_VA}, 28'd4160, {16'b0, CPL_VA});
    settle();
    check_payload(BASE1 + 48'h50000, 520,
                  "PMTU+64 copy lands (short final fragment)");
    check(fences == 5, "every copy released its fence");

    store(4'd1, 12'hC0, 64'hB0DA_0001);
    settle();
    check(mem.exists((BASE1 + 48'hC0) >> 3) &&
          mem[(BASE1 + 48'hC0) >> 3] == 64'hB0DA_0001,
          "store after an unevenly fragmented copy parses its own header");

    // --- the bulk -> store transition. This is the hardware bench's shape:
    //     a multi-fragment copy, then a run of inline stores, with nothing
    //     settling in between, so the shell model is holding the bulk's
    //     beats AND the stores' message beats in one queue when it pairs
    //     them to requests. That is the situation in which a request short
    //     of its payload takes the NEXT transaction's beat - on hardware
    //     256 KB came back with an inline store message written verbatim
    //     into the bulk destination, at word 0 of the region, meaning a
    //     request holding the bulk's vaddr took that beat having received
    //     none of its own. Every case above either settles between the two
    //     phases or follows the copy with a single store.
    n_writes_before = writes.size();
    copy(4'd1, 28'h60000, {16'b0, SRC_VA}, 28'd16384, {16'b0, CPL_VA});
    for (int st = 0; st < 8; st++)
        store(4'd1, 12'h100 + 12'(st*8), 64'hC0DE_0000 + 64'(st));
    settle();
    check_payload(BASE1 + 48'h60000, 2048,
                  "bulk survives the store phase that follows it");
    for (int st = 0; st < 8; st++)
        check(mem.exists((BASE1 + 48'h100 + 48'(st*8)) >> 3) &&
              mem[(BASE1 + 48'h100 + 48'(st*8)) >> 3] == (64'hC0DE_0000 + 64'(st)),
              $sformatf("store %0d after the bulk lands at its own target", st));

    // --- the same transition with the shell MERGING packets into one
    //     rq_wr, which is the shape the faster hardware run showed. The
    //     receiver's beat budget now spans several packets per request,
    //     so a boundary it gets wrong lands somewhere different
    req_pkts = 3;
    copy(4'd1, 28'h70000, {16'b0, SRC_VA}, 28'd16384, {16'b0, CPL_VA});
    for (int st = 0; st < 8; st++)
        store(4'd1, 12'h200 + 12'(st*8), 64'hBA7C_0000 + 64'(st));
    settle();
    check_payload(BASE1 + 48'h70000, 2048,
                  "batched requests: bulk survives the store phase");
    for (int st = 0; st < 8; st++)
        check(mem.exists((BASE1 + 48'h200 + 48'(st*8)) >> 3) &&
              mem[(BASE1 + 48'h200 + 48'(st*8)) >> 3] == (64'hBA7C_0000 + 64'(st)),
              $sformatf("batched requests: store %0d lands at its own target", st));
    req_pkts = 1;

    // --- the failing hardware workload's actual scale: a 256 KB copy (64
    //     PMTU packets) followed by 256 inline stores, no settle between.
    //     The order FIFO is 64 deep and DROPS a push when full rather than
    //     stalling the AXI-Lite bridge (posted-write semantics toward the
    //     host, loom_ctrl "Order FIFO"), so a store phase this long, issued
    //     while the engine is still streaming the bulk, is guaranteed to
    //     overflow it. That is by design - what must hold is that nothing
    //     vanishes silently: every store either lands or is counted in
    //     dbg[6]. Software that wants all 256 has to size against the depth
    //     or read the counter, which the bench does not do.
    ovf_before = inst_ctrl.dbg[6];
    copy(4'd1, 28'h100000, {16'b0, SRC_VA}, 28'd262144, {16'b0, CPL_VA});
    for (int st = 0; st < 256; st++)
        store(4'd1, 12'h400 + 12'(st*8), 64'h5709_0000 + 64'(st));
    settle();
    check_payload(BASE1 + 48'h100000, 32768,
                  "256 KB bulk survives the 256-store phase that follows it");
    landed = 0;
    for (int st = 0; st < 256; st++)
        if (mem.exists((BASE1 + 48'h400 + 48'(st*8)) >> 3) &&
            mem[(BASE1 + 48'h400 + 48'(st*8)) >> 3] == (64'h5709_0000 + 64'(st)))
            landed++;
    ovf = int'(inst_ctrl.dbg[6] - ovf_before);
    $display("       256-store phase: %0d landed, %0d dropped on a full FIFO",
             landed, ovf);
    check(landed + ovf == 256,
          $sformatf("every store lands or is counted (%0d + %0d)", landed, ovf));
    // A store that made it through must be intact and at its own address:
    // the drops must not shift the ones that survive
    for (int st = 0; st < landed; st++)
        check(mem.exists((BASE1 + 48'h400 + 48'(st*8)) >> 3) &&
              mem[(BASE1 + 48'h400 + 48'(st*8)) >> 3] == (64'h5709_0000 + 64'(st)),
              $sformatf("surviving store %0d is intact and in place", st));

    // Respecting the depth, the same phase loses nothing: the FIFO is the
    // constraint, not the transition itself
    ovf_before = inst_ctrl.dbg[6];
    copy(4'd1, 28'h140000, {16'b0, SRC_VA}, 28'd262144, {16'b0, CPL_VA});
    settle();
    for (int batch = 0; batch < 4; batch++) begin
        for (int st = 0; st < 60; st++)
            store(4'd1, 12'h800 + 12'((batch*60 + st)*8), 64'h9E11_0000 + 64'(batch*60 + st));
        settle();
    end
    check_payload(BASE1 + 48'h140000, 32768,
                  "256 KB bulk intact when the store phase respects the depth");
    landed = 0;
    for (int st = 0; st < 240; st++)
        if (mem.exists((BASE1 + 48'h800 + 48'(st*8)) >> 3) &&
            mem[(BASE1 + 48'h800 + 48'(st*8)) >> 3] == (64'h9E11_0000 + 64'(st)))
            landed++;
    check(landed == 240,
          $sformatf("all 240 stores land when issued within the depth (%0d)", landed));
    check(inst_ctrl.dbg[6] == ovf_before, "no overflow when the depth is respected");

    // --- the receive path frames payload per packet, so drive it that way:
    //     rq_wr.last still only on the final fragment, but a tlast at the
    //     end of every one. On hardware the corrupt runs completed roughly
    //     one loom_rx transaction for every three the shell issued, and the
    //     intact ones matched exactly, so what ends a transaction is where
    //     the fault has to be
    tlast_per_pkt = 1;
    copy(4'd1, 28'h180000, {16'b0, SRC_VA}, 28'd262144, {16'b0, CPL_VA});
    for (int st = 0; st < 60; st++)
        store(4'd1, 12'hA00 + 12'(st*8), 64'h7A57_0000 + 64'(st));
    settle();
    check_payload(BASE1 + 48'h180000, 32768,
                  "per-packet tlast: 256 KB bulk lands intact");
    begin
        bit all_ok;
        all_ok = 1;
        for (int st = 0; st < 60; st++)
            if (!mem.exists((BASE1 + 48'hA00 + 48'(st*8)) >> 3) ||
                mem[(BASE1 + 48'hA00 + 48'(st*8)) >> 3] != (64'h7A57_0000 + 64'(st)))
                all_ok = 0;
        check(all_ok, "per-packet tlast: every store lands at its own target");
    end
    tlast_per_pkt = 0;

    // --- the two together, which is the shape neither knob tested alone:
    //     the shell merges packets into one rq_wr AND the payload is still
    //     framed per packet. loom_rx takes stream_end from s_tlast whenever
    //     rq_wr.last is high, so a merged final request ends at the FIRST
    //     packet's tlast while the sq_wr it already issued claimed the whole
    //     merged length - and the far shell completes that write from
    //     whatever beats come next. That both shifts the payload and
    //     completes FEWER transactions than the shell issued, which is
    //     exactly the pair of things hardware showed.
    // It is OFF because the shell this runs against cannot produce the
    // shape: m_rdma_wr_req is a straight pass-through of the HLS command
    // stream (roce_stack.sv:262), one memCmd per packet
    // (ib_transport_protocol.cpp:628/632/657/661), and PMTU_BYTES is 4096.
    // Turning it on fails, and the failure is real - it is a hazard waiting
    // on a shell that merges, not a bug in anything running today.
    if (MERGED_REQ_HAZARD) begin
    req_pkts = 2;
    tlast_per_pkt = 1;
    fwd_before = rx_txns;
    copy(4'd1, 28'h1C0000, {16'b0, SRC_VA}, 28'd262144, {16'b0, CPL_VA});
    for (int st = 0; st < 60; st++)
        store(4'd1, 12'hC00 + 12'(st*8), 64'hD15C_0000 + 64'(st));
    settle();
    check_payload(BASE1 + 48'h1C0000, 32768,
                  "merged requests + per-packet tlast: bulk lands intact");
    begin
        bit all_ok;
        all_ok = 1;
        for (int st = 0; st < 60; st++)
            if (!mem.exists((BASE1 + 48'hC00 + 48'(st*8)) >> 3) ||
                mem[(BASE1 + 48'hC00 + 48'(st*8)) >> 3] != (64'hD15C_0000 + 64'(st)))
                all_ok = 0;
        check(all_ok, "merged requests + per-packet tlast: stores land");
    end
    // One loom_rx transaction per request the shell issued is the
    // invariant hardware violates 3:1 on every corrupt run
    $display("       rx transactions: %0d completed, %0d requests issued",
             rx_txns - fwd_before, (262144 + 4096*2 - 1)/(4096*2) + 60);
    check(rx_txns - fwd_before == (262144 + 4096*2 - 1)/(4096*2) + 60,
          "one rx transaction per request the shell issued");
    req_pkts = 1;
    tlast_per_pkt = 0;
    end

    // --- one host write per MESSAGE, not per packet. This is the entire
    //     point of carrying the target VA in a Loom header rather than the
    //     RETH: 64 KB is 17 PMTU packets on the wire, and the receive path
    //     used to issue a host write for every one of them. The far side
    //     now takes the destination and the full length from the header and
    //     issues ONE, spanning the packets underneath it.
    fwd_before = rx_txns;
    copy(4'd1, 28'h90000, {16'b0, SRC_VA}, 28'd65536, {16'b0, CPL_VA});
    settle();
    check_payload(BASE1 + 48'h90000, 8192,
                  "64 KB message spanning 17 packets lands intact");
    check(rx_txns - fwd_before == 1,
          $sformatf("64 KB landed as ONE host write, not per packet (%0d)",
                    rx_txns - fwd_before));

    // --- The size that fails on hardware. A 1 MB message is 257 PMTU
    //     packets and 16384 payload beats, and the failure there was a -1
    //     beat displacement, consistent across the whole region, starting
    //     at packet 99 of 256 - a beat budget going wrong once and staying
    //     wrong. Everything above is 17 packets or fewer, so if the desync
    //     needs scale nothing here would have caught it.
    //     512 KB first: 129 packets, straddling where the shift appeared.
    fwd_before = rx_txns;
    copy(4'd1, 28'h400000, {16'b0, SRC_VA}, 28'd524288, {16'b0, CPL_VA});
    settle();
    check_payload(BASE1 + 48'h400000, 65536,
                  "512 KB message (129 packets) lands intact");
    check(rx_txns - fwd_before == 1,
          $sformatf("512 KB landed as ONE host write (%0d)",
                    rx_txns - fwd_before));

    fwd_before = rx_txns;
    copy(4'd1, 28'h800000, {16'b0, SRC_VA}, 28'd1048576, {16'b0, CPL_VA});
    settle();
    check_payload(BASE1 + 48'h800000, 131072,
                  "1 MB message (257 packets) lands intact");
    check(rx_txns - fwd_before == 1,
          $sformatf("1 MB landed as ONE host write (%0d)", rx_txns - fwd_before));

    // Same size, framed the way the RX path actually builds it: a tlast at
    // every packet boundary rather than one at the end of the message. The
    // model has defaulted to per-message since it was written, so no
    // MB-scale case has ever run against per-packet framing - and a
    // spanning message is exactly the thing that has to ignore 256
    // intermediate tlasts and stop only where its header said.
    tlast_per_pkt = 1;
    fwd_before = rx_txns;
    copy(4'd1, 28'hC00000, {16'b0, SRC_VA}, 28'd1048576, {16'b0, CPL_VA});
    settle();
    check_payload(BASE1 + 48'hC00000, 131072,
                  "1 MB, per-packet tlast: lands intact");
    check(rx_txns - fwd_before == 1,
          $sformatf("1 MB, per-packet tlast: ONE host write (%0d)",
                    rx_txns - fwd_before));
    tlast_per_pkt = 0;

    // ---------------------------------------------------------------------
    // The two-host failure, reproduced: 1 MB x1 is clean, 1 MB x2 back to
    // back is not. Model the sender-side damage a retransmission does -
    // beats stolen from the message in flight - and see whether loom_rx
    // lands the hardware signature: region fully covered, nothing "never
    // written", no header rejected, and payload displaced by whole beats
    // because position is a running count that cannot recover.
    // ---------------------------------------------------------------------
    beats_dropped = 0;
    fwd_before    = rx_txns;
    drop_done     = 0;
    drop_at_beat  = 4096;       // partway into the message
    drop_n_beats  = 2;          // hardware's smallest observed displacement
    copy(4'd1, 28'hD00000, {16'b0, SRC_VA}, 28'd1048576, {16'b0, CPL_VA});
    copy(4'd1, 28'hE00000, {16'b0, SRC_VA}, 28'd1048576, {16'b0, CPL_VA});
    settle();
    drop_at_beat = 0;
    drop_n_beats = 0;

    check(beats_dropped == 2,
          $sformatf("the model stole %0d beats mid-message", beats_dropped));
    check(rx_drop === 1'b0,
          "no header rejected even though the stream lost beats");

    begin
        int wrong = 0, never = 0, first_bad = -1;
        for (int i = 0; i < 131072; i++) begin
            logic [47:0] va = BASE1 + 48'hD00000 + 48'(i*8);
            if (!mem.exists(va >> 3)) never++;
            else if (mem[va >> 3] != src_word(i)) begin
                if (first_bad < 0) first_bad = i;
                wrong++;
            end
        end
        $display("       stolen-beat message: %0d of 131072 words wrong, %0d never written, first bad %0d",
                 wrong, never, first_bad);
        // The hardware signature, precisely: everything arrived and the
        // whole region was covered, but it landed in the wrong place.
        check(never == 0,
              "whole region still covered - nothing was lost, only displaced");
        check(wrong > 0, "losing beats DOES displace the payload");
        if (first_bad >= 0 && mem.exists((BASE1 + 48'hD00000 + 48'(first_bad*8)) >> 3))
            $display("       at word %0d: got %016x, want %016x (delta %0d words)",
                     first_bad,
                     mem[(BASE1 + 48'hD00000 + 48'(first_bad*8)) >> 3],
                     src_word(first_bad),
                     int'(mem[(BASE1 + 48'hD00000 + 48'(first_bad*8)) >> 3] -
                          src_word(first_bad)));
    end

    // ---------------------------------------------------------------------
    // The sender-side counterpart, and the one that matters: a PERFECT
    // link. No beats lost, no duplicates, no retransmission - just one
    // surplus beat on the pull. loom_engine forwards the next l_sbeats
    // beats off axis_host_recv without checking they belong to its own
    // request, so the payload should shift and stay shifted.
    // ---------------------------------------------------------------------
    pull_extra_beat = 1;
    pull_extra_done = 0;
    fwd_before      = rx_txns;
    copy(4'd1, 28'h700000, {16'b0, SRC_VA}, 28'd262144, {16'b0, CPL_VA});
    copy(4'd1, 28'h740000, {16'b0, SRC_VA}, 28'd262144, {16'b0, CPL_VA});
    // A third message so the shifted one's tail is filled from the stream
    // behind it, exactly as it is on hardware where 32 run back to back.
    // Without it the model simply runs dry and the tail reads "never
    // written", which is an artefact of the harness, not of the bug.
    copy(4'd1, 28'h780000, {16'b0, SRC_VA}, 28'd262144, {16'b0, CPL_VA});
    settle();
    pull_extra_beat = 0;

    check(pull_extra_count == 1,
          $sformatf("exactly one surplus pull beat injected (%0d)",
                    pull_extra_count));
    begin
        int wrong = 0, never = 0, first_bad = -1;
        for (int i = 0; i < 32768; i++) begin
            logic [47:0] va = BASE1 + 48'h740000 + 48'(i*8);
            if (!mem.exists(va >> 3)) never++;
            else if (mem[va >> 3] != src_word(i)) begin
                if (first_bad < 0) first_bad = i;
                wrong++;
            end
        end
        $display("       surplus-beat run: %0d of 32768 words wrong, %0d never written, first bad %0d",
                 wrong, never, first_bad);
        if (first_bad >= 0 && mem.exists((BASE1 + 48'h740000 + 48'(first_bad*8)) >> 3))
            $display("       at word %0d: got %016x, want %016x",
                     first_bad,
                     mem[(BASE1 + 48'h740000 + 48'(first_bad*8)) >> 3],
                     src_word(first_bad));
        check(wrong > 0,
              "a single surplus pull beat corrupts, with a PERFECT link");
        check(rx_drop === 1'b0,
              "and no header is rejected, so it looks like clean traffic");
    end

    check(rx_drop === 1'b0, "no header was ever rejected on the far side");

    // The engine must have delivered exactly what its requests claimed
    check(net_extra == 0,
          $sformatf("no beat outside a request (%0d extra)", net_extra));
    if (pull_extra_count == 0)
        check(net_owed == 0,
              $sformatf("no request left short of payload (%0d owed)",
                        net_owed));

    if (errors == 0) $display("TB PASS (tb_loom_loopback)");
    else             $display("TB FAIL (tb_loom_loopback): %0d errors", errors);
    $finish;
end

initial begin
    #400ms;
    $display("TB FAIL (tb_loom_loopback): timeout");
    $finish;
end

endmodule
