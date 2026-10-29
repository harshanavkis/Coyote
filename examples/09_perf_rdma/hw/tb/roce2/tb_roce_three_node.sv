`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * Three RoCE nodes behind a switch (2026-10-28, the 3-rank RDMA freeze at window 32: rdma_colln a2a 1 MiB per rank,
 * --acked-step --flag-in-data, froze one card's input at random on perf_rdma-w32/w32c, never in 2-rank pairs nor at
 * window 16). roce_node3 = tb_roce_two_node.sv's roce_node (the shell's roce_stack, same host model, probes, retransmission
 * memory) with one QP per peer (QP q -> node (ID + 1 + q) % 3, remote QPN = the peer's QP index for this node) and the
 * a2a step pattern: per step one WR_BYTES WRITE to each peer (349,568 B = 1 MiB / 3 rounded to 128 B, as rdma_colln), the
 * flag in its last 8 B (no flag WRs); the next step's posts wait until both WRs are acknowledged and both peers' blocks of
 * the step have landed, then POSTGAP (+ JITTER) cycles. Every landed byte is checked (its sender from the address). sw3 =
 * the switch: per-port MAC buffer, line rate, cables, store-and-forward egress queues of SWQ beats (tail drop) and the
 * nodes' input FIFOs (whole-packet drop). TB PASS / TB DONE with drops / TB FAIL: receive stall (a node's input refused
 * STALL_CYC cycles in a row, the hardware's STRM down) / data / timeout; on a stop dump_state3.tcl lists every HLS FIFO
 * that is full or holds data per node.
 */
module roce_node3 #(
    parameter int          ID = 0,
    parameter logic [31:0] IP0 = 32'h0A0B0B0A,   // the three nodes' IPs (byte palindromes: no byte-order question)
    parameter logic [31:0] IP1 = 32'h0C0D0D0C,
    parameter logic [31:0] IP2 = 32'h0E0F0F0E,
    parameter int          NQ = 2,                 // one QP per peer: QP q goes to node (ID + 1 + q) % 3
    parameter int          NWR = 2,                // a2a steps (iterations)
    parameter int          POSTGAP = 2000,         // cycles from a step's end (both blocks landed, both WRs acked) to the next posts
    parameter int          JITTER = 0,             // + up to this many random cycles before each post (the CPU)
    parameter int          WR_BYTES = 1 << 20,
    parameter bit          SEND = 1,
    parameter int          MEM_BITS = 64,
    parameter int          FLAG_BYTES = 8,    // the flag WRITE after each round (8 B = a 64 B packet at IP level: icrc.sv's bug)
    parameter int          CORRUPT_BEAT = -1, // negative control of the check: flip a byte of the n-th landed beat (-1 = none)
    parameter bit          VERBOSE = 1,       // per-packet / per-ACK / trace prints (0: rate runs, much faster)
    parameter bit          PEER_SEND = 1,     // the other node sends (for the receive-side probe window)
    parameter int          LANDSTALL_PCT = 0, // 2026-10-28: the host refuses a landing beat / request at random in this % of cycles
    parameter int          LSEED = 1
) (
    input  logic   nclk,
    input  logic   nresetn,
    AXI4S.m        m_tx,
    AXI4S.s        s_rx,
    output logic   setup_done,
    output int     n_posted,
    output int     n_acks,
    output int     n_landed,
    output int     n_wr_bytes,
    output int     n_bad,
    output int     n_memrd
);
    localparam int PKTS = (WR_BYTES + PMTU_BYTES - 1) / PMTU_BYTES;
    localparam logic [31:0] LIP = ID == 0 ? IP0 : ID == 1 ? IP1 : IP2;

    metaIntf #(.STYPE(rdma_qp_ctx_t))  qp   (.aclk(nclk), .aresetn(nresetn));
    metaIntf #(.STYPE(rdma_qp_conn_t)) conn (.aclk(nclk), .aresetn(nresetn));
    metaIntf #(.STYPE(dreq_t))         sq   (.aclk(nclk), .aresetn(nresetn));
    metaIntf #(.STYPE(ack_t))          ack  (.aclk(nclk), .aresetn(nresetn));
    metaIntf #(.STYPE(req_t))          rd_req (.aclk(nclk), .aresetn(nresetn));
    metaIntf #(.STYPE(req_t))          wr_req (.aclk(nclk), .aresetn(nresetn));
    AXI4S #(.AXI4S_DATA_BITS(AXI_NET_BITS)) axis_rd_req (.aclk(nclk), .aresetn(nresetn));
    AXI4S #(.AXI4S_DATA_BITS(AXI_NET_BITS)) axis_rd_rsp (.aclk(nclk), .aresetn(nresetn));
    AXI4S #(.AXI4S_DATA_BITS(AXI_NET_BITS)) axis_wr     (.aclk(nclk), .aresetn(nresetn));
    metaIntf #(.STYPE(logic[MEM_CMD_BITS-1:0])) mem_rd_cmd (.aclk(nclk), .aresetn(nresetn));
    metaIntf #(.STYPE(logic[MEM_CMD_BITS-1:0])) mem_wr_cmd (.aclk(nclk), .aresetn(nresetn));
    metaIntf #(.STYPE(logic[MEM_STS_BITS-1:0])) mem_rd_sts (.aclk(nclk), .aresetn(nresetn));
    metaIntf #(.STYPE(logic[MEM_STS_BITS-1:0])) mem_wr_sts (.aclk(nclk), .aresetn(nresetn));
    AXI4S #(.AXI4S_DATA_BITS(AXI_NET_BITS)) axis_mem_rd (.aclk(nclk), .aresetn(nresetn));
    AXI4S #(.AXI4S_DATA_BITS(AXI_NET_BITS)) axis_mem_wr (.aclk(nclk), .aresetn(nresetn));

    roce_stack inst_roce (
        .nclk(nclk), .nresetn(nresetn),
        .s_axis_rx(s_rx), .m_axis_tx(m_tx),
        .s_rdma_qp_interface(qp), .s_rdma_conn_interface(conn),
        .s_rdma_sq(sq), .m_rdma_ack(ack),
        .m_rdma_rd_req(rd_req), .m_rdma_wr_req(wr_req),
        .s_axis_rdma_rd_req(axis_rd_req), .s_axis_rdma_rd_rsp(axis_rd_rsp), .m_axis_rdma_wr(axis_wr),
        .local_ip_address(LIP),
        .m_rdma_mem_rd_cmd(mem_rd_cmd), .m_rdma_mem_wr_cmd(mem_wr_cmd),
        .s_rdma_mem_rd_sts(mem_rd_sts), .s_rdma_mem_wr_sts(mem_wr_sts),
        .s_axis_rdma_mem_rd(axis_mem_rd), .m_axis_rdma_mem_wr(axis_mem_wr),
        .ibv_rx_pkg_count_valid(), .ibv_rx_pkg_count_data(), .ibv_tx_pkg_count_valid(), .ibv_tx_pkg_count_data(),
        .crc_drop_pkg_count_valid(), .crc_drop_pkg_count_data(), .psn_drop_pkg_count_valid(), .psn_drop_pkg_count_data(),
        .retrans_count_valid(), .retrans_count_data()
    );

    // rdma_flow's pointer table (ram_sp_nc, no reset) powers up as zeros in the BRAM, as X in simulation
    initial begin
        #1;
        foreach (inst_roce.inst_rdma_flow.inst_pntr_table.ram[i]) inst_roce.inst_rdma_flow.inst_pntr_table.ram[i] = '0;
    end

    // icrc.sv latches tkeep even without tvalid; the HLS core's idle tkeep is X in simulation (uninitialised FIFO memory, a
    // stale value on the FPGA), which made icrc's tready X and desynchronised the core's output register slice. While idle,
    // hold the last valid beat's tkeep (the stale value; 0 before the first beat).
    logic [63:0] last_keep = '0;
    always @(negedge nclk) if (inst_roce.trimmer_to_icrc.tvalid && !$isunknown(inst_roce.trimmer_to_icrc.tkeep)) last_keep <= inst_roce.trimmer_to_icrc.tkeep;
    always @(inst_roce.trimmer_to_icrc.tvalid or last_keep)
        if (!inst_roce.trimmer_to_icrc.tvalid) force inst_roce.trimmer_to_icrc.tkeep = last_keep; else release inst_roce.trimmer_to_icrc.tkeep;

    function automatic logic [23:0] psn(int node, int q); return 24'h100000 * (node + 1) + 24'h1111 * (q + 1); endfunction
    function automatic int peer(int q); return (ID + 1 + q) % 3; endfunction
    function automatic int rqp(int q); return (ID - peer(q) + 3) % 3 - 1; endfunction        // the peer's QP index for this node
    function automatic logic [31:0] pip(int q); automatic int p = peer(q); return p == 0 ? IP0 : p == 1 ? IP1 : IP2; endfunction
    // receive layout: the block from sender s lands at (s + 1) << 36, two halves (iteration parity) of WR_BYTES
    function automatic int sender_of(logic [63:0] a); return int'(a >> 36) - 1; endfunction

    // the payload: every 8 B word = {sender ^ 0x5A, its remote address}, so a landed byte names where it belongs
    function automatic logic [511:0] beat_data(int node, logic [63:0] raddr);
        logic [511:0] d;
        for (int w = 0; w < 8; w++) d[64*w +: 64] = {8'(node) ^ 8'h5A, 56'(raddr + 8 * w)};
        return d;
    endfunction

    // ---- QP setup (as cnfg_slave packs cThread::writeQpContext) ----
    initial begin
        qp.valid = 0; conn.valid = 0; qp.data = '0; conn.data = '0; setup_done = 0;
        @(posedge nresetn); repeat (100) @(posedge nclk);
        for (int q = 0; q < NQ; q++) begin
            qp.data = '0;
            qp.data.qp_num = q; qp.data.local_psn = psn(ID, q); qp.data.remote_psn = psn(peer(q), rqp(q));
            qp.valid = 1; do @(posedge nclk); while (!qp.ready); qp.valid = 0;
            repeat (20) @(posedge nclk);
            conn.data = '0;
            conn.data.local_qpn = q; conn.data.remote_qpn = rqp(q); conn.data.remote_ip_address = {4{pip(q)}};
            conn.data.remote_udp_port = 16'd18488;
            conn.valid = 1; do @(posedge nclk); while (!conn.ready); conn.valid = 0;
            repeat (20) @(posedge nclk);
        end
        repeat (200) @(posedge nclk);
        setup_done = 1;
        $display("[%0t] node %0d: %0d QPs set up, %0d commands queued", $time, ID, NQ, cmds.size());
    end

    // ---- commands: one per packet, as dreq_rdma_parser_wr emits them; per step (rdma_colln a2a, --acked-step, --flag-in-data):
    // one WR of WR_BYTES to each peer (QP 0 first: peers in rotated order), then wait until both WRs are acknowledged and
    // both peers' blocks of this step have landed (the flag = the block's last 8 B), then POSTGAP (+ jitter) cycles ----
    dreq_t cmds[$];
    int    clen[$];
    int    step = 0;                 // steps whose posts are queued
    int    from_n[3] = '{0, 0, 0};   // landed packets per sender
    logic [31:0] jrng = 32'h2545F491 ^ (ID * 7 + LSEED);
    function automatic int jit(); jrng = jrng ^ (jrng << 13); jrng = jrng ^ (jrng >> 17); jrng = jrng ^ (jrng << 5); return JITTER ? int'(jrng % JITTER) : 0; endfunction
    initial begin
        wait (setup_done);
        for (int r = 0; r < NWR && SEND; r++) begin
            if (r) begin
                wait (n_acks >= NQ * r && from_n[peer(0)] >= PKTS * r && from_n[peer(1)] >= PKTS * r);
                repeat (POSTGAP) @(posedge nclk);
            end
            for (int q = 0; q < NQ; q++) begin
                repeat (jit()) @(posedge nclk);
                for (int p = 0; p < PKTS; p++) begin
                    automatic dreq_t d = '0;
                    automatic int len = (p == PKTS - 1) ? WR_BYTES - p * PMTU_BYTES : PMTU_BYTES;
                    automatic logic [63:0] half = (r & 1) ? WR_BYTES : 0;
                    d.req_1.opcode = PKTS == 1 ? RC_RDMA_WRITE_ONLY : p == 0 ? RC_RDMA_WRITE_FIRST : p == PKTS - 1 ? RC_RDMA_WRITE_LAST : RC_RDMA_WRITE_MIDDLE;
                    d.req_1.strm = STRM_HOST; d.req_1.mode = RDMA_MODE_RAW; d.req_1.rdma = 1; d.req_1.remote = 1;
                    d.req_1.pid = q; d.req_1.last = (p == PKTS - 1); d.req_1.actv = 1; d.req_1.host = 1;
                    d.req_1.vaddr = (64'(ID + 1) << 36) + half + p * PMTU_BYTES;          // remote: the peer's block from this node
                    d.req_1.len = len;
                    d.req_2.vaddr = (64'(q + 1) << 32) + half + p * PMTU_BYTES;           // local: this QP's source
                    d.req_2.strm = STRM_HOST;
                    cmds.push_back(d); clen.push_back(len);
                end
            end
            step = r + 1;
        end
    end
    // (queue-driven outputs are registered stages: xsim does not re-evaluate continuous assigns on queue methods)
    int pay[$];   // payload lengths of the accepted commands, in order
    bit payl[$];  // the command is its WR's last packet (the host read's tlast: one read per WR, axis_mux_user_wr passes it)
    logic [63:0] payr[$];   // its remote address (the payload's content)
    logic sq_v = 0; dreq_t sq_d = '0; int sq_len = 0;
    assign sq.valid = sq_v;
    assign sq.data  = sq_d;
    initial n_posted = 0;
    bit sq_hs = 0, sq_rdy = 0;
    always @(negedge nclk) begin sq_hs = sq_v && sq.ready; sq_rdy = sq.ready; end
    always @(posedge nclk) begin
        #1;
        if (sq_hs) begin pay.push_back(sq_len); payl.push_back(sq_d.req_1.last); payr.push_back(sq_d.req_1.vaddr); n_posted <= n_posted + 1; end
        if (!sq_v || sq_rdy) begin
            if (setup_done && cmds.size() != 0) begin sq_d <= cmds.pop_front(); sq_len <= clen.pop_front(); sq_v <= 1; end
            else sq_v <= 0;
        end
    end

    // ---- payload in command order (the shell's host read) ----
    int beat = 0, pkt_no = 0, cur_len = 0;
    logic [63:0] cur_raddr = 0;
    logic pv = 0, cur_last = 0;         // a packet's payload is being delivered; it ends its WR
    wire [31:0] cur_beats = (cur_len + 63) / 64;
    assign axis_rd_req.tvalid = pv;
    wire  pkt_end = beat == cur_beats - 1;
    assign axis_rd_req.tlast  = pkt_end && cur_last;
    assign axis_rd_req.tkeep  = (beat == cur_beats - 1 && cur_len % 64) ? ((64'd1 << (cur_len % 64)) - 1) : '1;
    assign axis_rd_req.tdata  = beat_data(ID, cur_raddr + 64 * beat);
    bit pay_hs = 0, pay_end = 0;
    always @(negedge nclk) begin pay_hs = pv && axis_rd_req.tready; pay_end = pkt_end; end
    always @(posedge nclk) begin
        #1;
        if (pay_hs) begin
            if (pay_end) begin beat <= 0; pkt_no <= pkt_no + 1; pv <= 0; end
            else beat <= beat + 1;
        end
        if ((!pv || (pay_hs && pay_end)) && pay.size() != 0) begin
            cur_len <= pay.pop_front(); cur_last <= payl.pop_front(); cur_raddr <= payr.pop_front(); pv <= 1; beat <= 0;
        end
    end
    assign axis_rd_rsp.tvalid = 0; assign axis_rd_rsp.tdata = 0; assign axis_rd_rsp.tkeep = 0; assign axis_rd_rsp.tlast = 0;

    // ---- host sinks: always ready; every landed byte is checked against the peer's payload for its address ----
    assign rd_req.ready = 1; assign ack.ready = 1;
    logic [31:0] lrng = 32'h85ebca6b ^ LSEED; logic land_rdy = 1;
    always @(posedge nclk) begin
        #1;
        lrng = lrng ^ (lrng << 13); lrng = lrng ^ (lrng >> 17); lrng = lrng ^ (lrng << 5);
        land_rdy <= !(LANDSTALL_PCT > 0 && (lrng % 100) < LANDSTALL_PCT);
    end
    assign wr_req.ready = land_rdy; assign axis_wr.tready = land_rdy;
    initial begin n_acks = 0; n_landed = 0; n_wr_bytes = 0; n_bad = 0; end
    logic [63:0] lq_a[$]; int lq_n[$];                       // landing writes: address, bytes
    logic [511:0] dq_d[$]; logic [63:0] dq_k[$];             // landed beats
    int lbeat = 0;                                           // beat within the current landing write
    int nbeat = 0;                                           // landed beats checked
    always @(negedge nclk) begin
        if (ack.valid) n_acks <= n_acks + 1;
        if (wr_req.valid && wr_req.ready) begin n_landed <= n_landed + 1; lq_a.push_back(wr_req.data.vaddr); lq_n.push_back(wr_req.data.len); end
        if (axis_wr.tvalid && axis_wr.tready) begin n_wr_bytes <= n_wr_bytes + $countones(axis_wr.tkeep); dq_d.push_back(axis_wr.tdata); dq_k.push_back(axis_wr.tkeep); end
        if (rd_req.valid) $display("[%0t] node %0d: unexpected read request", $time, ID);
        while (lq_a.size() != 0 && dq_d.size() != 0) begin
            automatic logic [511:0] exp = beat_data(sender_of(lq_a[0]), lq_a[0] + 64 * lbeat);
            automatic logic [511:0] got = dq_d.pop_front();
            automatic logic [63:0]  k = dq_k.pop_front();
            if (nbeat == CORRUPT_BEAT) got[7:0] = ~got[7:0];
            nbeat = nbeat + 1;
            for (int b = 0; b < 64; b++)
                if (k[b] && got[8*b +: 8] !== exp[8*b +: 8]) begin
                    if (n_bad < 8) $display("[%0t] node %0d DATA WRONG: landing 0x%h beat %0d byte %0d: word got %h expected %h",
                                            $time, ID, lq_a[0], lbeat, b, got[64*(b/8) +: 64], exp[64*(b/8) +: 64]);
                    n_bad = n_bad + 1;
                    break;
                end
            if (lbeat == (lq_n[0] + 63) / 64 - 1) begin
                if (sender_of(lq_a[0]) >= 0 && sender_of(lq_a[0]) < 3) from_n[sender_of(lq_a[0])]++;
                lbeat = 0; void'(lq_a.pop_front()); void'(lq_n.pop_front());
            end
            else lbeat = lbeat + 1;
        end
    end

    // ---- probes inside roce_stack: handshake counts along the transmit path ----
    int c_sq = 0, c_rdcmd = 0, c_pay = 0, c_net = 0, c_tx = 0, c_memwr = 0, c_memrd = 0, c_rx = 0, c_gap = 0;
    always @(negedge nclk) begin
        if (inst_roce.rdma_sq.valid && inst_roce.rdma_sq.ready) c_sq <= c_sq + 1;
        if (inst_roce.rdma_rd_req.valid && inst_roce.rdma_rd_req.ready) c_rdcmd <= c_rdcmd + 1;
        if (axis_rd_req.tvalid && axis_rd_req.tready) c_pay <= c_pay + 1;
        if (inst_roce.axis_rdma_rd.tvalid && inst_roce.axis_rdma_rd.tready) c_net <= c_net + 1;
        if (inst_roce.roce_to_icrc.tvalid && inst_roce.roce_to_icrc.tready && inst_roce.roce_to_icrc.tlast) c_tx <= c_tx + 1;
        if (mem_wr_cmd.valid) c_memwr <= c_memwr + 1;
        if (mem_rd_cmd.valid) c_memrd <= c_memrd + 1;
        if (s_rx.tvalid && s_rx.tready && s_rx.tlast) c_rx <= c_rx + 1;
        if (inst_roce.gap_enforcer_to_hls.tvalid && inst_roce.gap_enforcer_to_hls.tready && inst_roce.gap_enforcer_to_hls.tlast) c_gap <= c_gap + 1;
    end
    // framing log: beats per packet and the last beat's byte count, first FRAME_LOG packets at four points
    int fb[4] = '{0, 0, 0, 0}, fn[4] = '{0, 0, 0, 0};
    logic [63:0] hdr[4];
    localparam int FRAME_LOG = 4;
    task automatic frame(int i, string w, logic v, logic r, logic l, logic [63:0] k, logic [63:0] d);
        if (v && r) begin
            if (fb[i] == 0) hdr[i] = d;
            fb[i]++;
            if (l) begin
                if (fn[i] < FRAME_LOG) $display("[%0t] node %0d frame %s: packet %0d = %0d beats, last beat %0d B, first 8 B %h", $time, ID, w, fn[i], fb[i], $countones(k), hdr[i]);
                fn[i]++; fb[i] = 0;
            end
        end
    endtask
    always @(negedge nclk) begin
        frame(0, "hls tx", inst_roce.roce_to_icrc.tvalid, inst_roce.roce_to_icrc.tready, inst_roce.roce_to_icrc.tlast, inst_roce.roce_to_icrc.tkeep, inst_roce.roce_to_icrc.tdata[63:0]);
        frame(1, "tx out", m_tx.tvalid, m_tx.tready, m_tx.tlast, m_tx.tkeep, m_tx.tdata[63:0]);
        frame(2, "rx in ", s_rx.tvalid, s_rx.tready, s_rx.tlast, s_rx.tkeep, s_rx.tdata[63:0]);
        frame(3, "->hls ", inst_roce.gap_enforcer_to_hls.tvalid, inst_roce.gap_enforcer_to_hls.tready, inst_roce.gap_enforcer_to_hls.tlast, inst_roce.gap_enforcer_to_hls.tkeep, inst_roce.gap_enforcer_to_hls.tdata[63:0]);
    end
    // transmit-path trace (node 0, first TRACE_N writes per FIFO): write index, last bit, kept bytes, first 8 B
    localparam int TRACE_N = 140;
    int tn[13] = '{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int cyc_n = 0; always @(negedge nclk) cyc_n <= cyc_n + 1;
    task automatic tr(int i, string w, logic we, logic [1023:0] d);
        if (VERBOSE && ID == 0 && we && tn[i] < TRACE_N) begin
            $display("TRACE %s #%0d last %0d keep %0d data %h", w, tn[i], d[576], $countones(d[575:512]), d[63:0]);
            tn[i]++;
        end
    endtask
    always @(negedge nclk) begin
        tr(0, "rethMerge", inst_roce.rocev2_inst.inst.tx_pkg_arbiter_512_0_U0_tx_rethMerge2rethShift_write, inst_roce.rocev2_inst.inst.tx_pkg_arbiter_512_0_U0_tx_rethMerge2rethShift_din);
        tr(1, "rawPay   ", inst_roce.rocev2_inst.inst.tx_pkg_arbiter_512_0_U0_tx_rawPayFifo_write, inst_roce.rocev2_inst.inst.tx_pkg_arbiter_512_0_U0_tx_rawPayFifo_din);
        tr(2, "rethShift", inst_roce.rocev2_inst.inst.lshiftWordByOctet_512_13_0_U0_tx_rethShift2payFifo_write, inst_roce.rocev2_inst.inst.lshiftWordByOctet_512_13_0_U0_tx_rethShift2payFifo_din);
        tr(3, "exh2pay  ", inst_roce.rocev2_inst.inst.generate_exh_512_0_U0_tx_exh2payFifo_write, inst_roce.rocev2_inst.inst.generate_exh_512_0_U0_tx_exh2payFifo_din);
        tr(4, "appended ", inst_roce.rocev2_inst.inst.append_payload_512_0_U0_tx_exh2shiftFifo_write, inst_roce.rocev2_inst.inst.append_payload_512_0_U0_tx_exh2shiftFifo_din);
        tr(5, "shift2ibh", inst_roce.rocev2_inst.inst.lshiftWordByOctet_512_11_0_U0_tx_shift2ibhFifo_write, inst_roce.rocev2_inst.inst.lshiftWordByOctet_512_11_0_U0_tx_shift2ibhFifo_din);
        tr(6, "ib2udp   ", inst_roce.rocev2_inst.inst.prepend_ibh_header_512_0_U0_tx_ib2udpFifo_write, inst_roce.rocev2_inst.inst.prepend_ibh_header_512_0_U0_tx_ib2udpFifo_din);
        tr(7, "shift2udp", inst_roce.rocev2_inst.inst.udp_lshiftWordByOctet_512_1_U0_tx_shift2udpFifo_write, inst_roce.rocev2_inst.inst.udp_lshiftWordByOctet_512_1_U0_tx_shift2udpFifo_din);
        tr(8, "udp2ip   ", inst_roce.rocev2_inst.inst.generate_udp_512_U0_tx_udp2ipFifo_write, inst_roce.rocev2_inst.inst.generate_udp_512_U0_tx_udp2ipFifo_din);
        tr(9, "shift2ip ", inst_roce.rocev2_inst.inst.ipv4_lshiftWordByOctet_512_2_U0_tx_shift2ipv4Fifo_write, inst_roce.rocev2_inst.inst.ipv4_lshiftWordByOctet_512_2_U0_tx_shift2ipv4Fifo_din);
        tr(10, "ip2crc   ", inst_roce.rocev2_inst.inst.ipv4_generate_ipv4_512_U0_tx_ip2crcFifo_write, inst_roce.rocev2_inst.inst.ipv4_generate_ipv4_512_U0_tx_ip2crcFifo_din);
        tr(11, "icrc out ", inst_roce.rocev2_inst.inst.insert_icrc_512_0_U0_m_axis_tx_data_internal_write, inst_roce.rocev2_inst.inst.insert_icrc_512_0_U0_m_axis_tx_data_internal_din);
        tr(12, "port out ", inst_roce.roce_to_icrc.tvalid && inst_roce.roce_to_icrc.tready, {447'b0, inst_roce.roce_to_icrc.tlast, inst_roce.roce_to_icrc.tkeep, inst_roce.roce_to_icrc.tdata});
        if (VERBOSE && ID == 0 && cyc_n >= 440 && cyc_n < 530) $display("TRACE port cyc %0d valid %0d ready %0d last %0d data %h", cyc_n, inst_roce.roce_to_icrc.tvalid, inst_roce.roce_to_icrc.tready, inst_roce.roce_to_icrc.tlast, inst_roce.roce_to_icrc.tdata[63:0]);
        if (VERBOSE && ID == 0 && inst_roce.rocev2_inst.inst.split_tx_meta_U0_tx_udpMetaFifo_write) $display("TRACE udpMeta %h", inst_roce.rocev2_inst.inst.split_tx_meta_U0_tx_udpMetaFifo_din);
        if (VERBOSE && ID == 0 && inst_roce.rocev2_inst.inst.split_tx_meta_U0_tx_udp2ipMetaFifo_write) $display("TRACE ipMeta  %h", inst_roce.rocev2_inst.inst.split_tx_meta_U0_tx_udp2ipMetaFifo_din);
    end
    always @(nclk) if (VERBOSE && ID == 0 && cyc_n >= 474 && cyc_n <= 479)
        $display("EDGE t=%0t clk=%0d port valid %0d ready %0d data %h | rs state %0d sel_rd %0d ack_out %0d | icrc halffull %0d full %0d inflight %0d",
            $time, nclk, inst_roce.roce_to_icrc.tvalid, inst_roce.roce_to_icrc.tready, inst_roce.roce_to_icrc.tdata[15:0],
            inst_roce.rocev2_inst.inst.convert_net_axis_to_axis_512_U0.regslice_both_m_axis_tx_data_V_data_V_U.B_V_data_1_state,
            inst_roce.rocev2_inst.inst.convert_net_axis_to_axis_512_U0.regslice_both_m_axis_tx_data_V_data_V_U.B_V_data_1_sel_rd,
            inst_roce.rocev2_inst.inst.convert_net_axis_to_axis_512_U0.regslice_both_m_axis_tx_data_V_data_V_U.B_V_data_1_ack_out,
            inst_roce.inst_icrc.halffull_signal, inst_roce.inst_icrc.full_signal, inst_roce.inst_icrc.crc32_in_flight_comb);
    always @(nclk) if (VERBOSE && ID == 0 && cyc_n >= 472 && cyc_n <= 478)
        $display("RS   t=%0t clk=%0d A %h B %h sel_wr %0d sel_rd %0d state %0d vld_in %0d ack_in %0d ack_out %0d",
            $time, nclk,
            inst_roce.rocev2_inst.inst.convert_net_axis_to_axis_512_U0.regslice_both_m_axis_tx_data_V_data_V_U.B_V_data_1_payload_A[15:0],
            inst_roce.rocev2_inst.inst.convert_net_axis_to_axis_512_U0.regslice_both_m_axis_tx_data_V_data_V_U.B_V_data_1_payload_B[15:0],
            inst_roce.rocev2_inst.inst.convert_net_axis_to_axis_512_U0.regslice_both_m_axis_tx_data_V_data_V_U.B_V_data_1_sel_wr,
            inst_roce.rocev2_inst.inst.convert_net_axis_to_axis_512_U0.regslice_both_m_axis_tx_data_V_data_V_U.B_V_data_1_sel_rd,
            inst_roce.rocev2_inst.inst.convert_net_axis_to_axis_512_U0.regslice_both_m_axis_tx_data_V_data_V_U.B_V_data_1_state,
            inst_roce.rocev2_inst.inst.convert_net_axis_to_axis_512_U0.regslice_both_m_axis_tx_data_V_data_V_U.vld_in,
            inst_roce.rocev2_inst.inst.convert_net_axis_to_axis_512_U0.regslice_both_m_axis_tx_data_V_data_V_U.ack_in,
            inst_roce.rocev2_inst.inst.convert_net_axis_to_axis_512_U0.regslice_both_m_axis_tx_data_V_data_V_U.ack_out);
    int xcount = 0;
    always @(negedge nclk) if (ID == 0 && nresetn && $isunknown({inst_roce.roce_to_icrc.tready, inst_roce.inst_icrc.crc32_in_flight, inst_roce.inst_icrc.stage_masking_keep})) begin
        if (xcount < 6) $display("XCHK t=%0t cyc %0d tready %b in_flight %b masking_keep_x %0d", $time, cyc_n, inst_roce.roce_to_icrc.tready,
                                 inst_roce.inst_icrc.crc32_in_flight, $isunknown(inst_roce.inst_icrc.stage_masking_keep));
        xcount++;
    end
    final if (ID == 0) $display("XCHK total cycles with X after reset: %0d", xcount);
    // retransmitter <-> rx_exh_fsm bookkeeping (defect D1: rx_exh_fsm takes one retrans2rx_init record per ACK)
    int r_upd_w = 0, r_upd_r = 0, r_init_w = 0, r_init_r = 0, r_ins = 0, r_rxreq = 0, r_timer = 0;
    always @(negedge nclk) begin
        if (inst_roce.rocev2_inst.inst.rx_ibh_fsm_0_U0_rx2retrans_upd_write) begin
            r_upd_w <= r_upd_w + 1;
            if (VERBOSE) $display("ACKUPD node %0d #%0d t=%0t din %h", ID, r_upd_w, $time, inst_roce.rocev2_inst.inst.rx_ibh_fsm_0_U0_rx2retrans_upd_din);
        end
        if (inst_roce.rocev2_inst.inst.process_retransmissions_0_U0_rx2retrans_upd_read) r_upd_r <= r_upd_r + 1;
        if (inst_roce.rocev2_inst.inst.rx_ibh_fsm_0_U0_rx_ibhEventFifo_write)
            $display("IBHEVENT node %0d t=%0t din %h (rx_ibh_fsm: duplicate-ACK or NAK event)", ID, $time, inst_roce.rocev2_inst.inst.rx_ibh_fsm_0_U0_rx_ibhEventFifo_din);
        if (inst_roce.rocev2_inst.inst.process_retransmissions_0_U0_retrans2rx_init_write) r_init_w <= r_init_w + 1;
        if (inst_roce.rocev2_inst.inst.rx_exh_fsm_512_0_U0_retrans2rx_init_read) r_init_r <= r_init_r + 1;
        if (inst_roce.rocev2_inst.inst.merge_retrans_request_U0_tx2retrans_insertRequest_write) r_ins <= r_ins + 1;
        if (inst_roce.rocev2_inst.inst.rx_exh_fsm_512_0_U0_rx2retrans_req_write) r_rxreq <= r_rxreq + 1;
        if (inst_roce.rocev2_inst.inst.transport_timer_0_U0_timer2retrans_req_write) r_timer <= r_timer + 1;
    end
    // per-packet BTH at this node's receive input (IP 20 + UDP 8: BTH opcode byte 28, dest QP bytes 33-35, PSN bytes 37-39)
    bit rx_first = 1; int rxn = 0;
    function automatic logic [7:0] byt(logic [511:0] d, int i); return d[8*i +: 8]; endfunction
    always @(negedge nclk) if (s_rx.tvalid && s_rx.tready) begin
        if (rx_first && VERBOSE) begin
            $display("RXPKT node %0d #%0d t=%0t op %02h dqp %0d psn %06h", ID, rxn, $time, byt(s_rx.tdata, 28),
                     {byt(s_rx.tdata, 33), byt(s_rx.tdata, 34), byt(s_rx.tdata, 35)}, {byt(s_rx.tdata, 37), byt(s_rx.tdata, 38), byt(s_rx.tdata, 39)});
            rxn++;
        end
        rx_first = s_rx.tlast;
    end
    // per-packet BTH and length at the HLS core's transmit output and at the stack's output (after ICRC)
    bit cf = 1, of = 1; int cb = 0, ob = 0; logic [511:0] ch, oh;
    always @(negedge nclk) begin
        if (inst_roce.roce_to_icrc.tvalid && inst_roce.roce_to_icrc.tready) begin
            if (cf) begin ch = inst_roce.roce_to_icrc.tdata; cb = 0; end
            cb++;
            if (VERBOSE && inst_roce.roce_to_icrc.tlast) $display("TXCORE node %0d t=%0t op %02h dqp %0d psn %06h beats %0d lastkeep %0d", ID, $time, byt(ch, 28),
                {byt(ch, 33), byt(ch, 34), byt(ch, 35)}, {byt(ch, 37), byt(ch, 38), byt(ch, 39)}, cb, $countones(inst_roce.roce_to_icrc.tkeep));
            cf = inst_roce.roce_to_icrc.tlast;
        end
        if (m_tx.tvalid && m_tx.tready) begin
            if (of) begin oh = m_tx.tdata; ob = 0; end
            ob++;
            if (VERBOSE && m_tx.tlast) $display("TXOUT  node %0d t=%0t op %02h dqp %0d psn %06h beats %0d lastkeep %0d", ID, $time, byt(oh, 28),
                {byt(oh, 33), byt(oh, 34), byt(oh, 35)}, {byt(oh, 37), byt(oh, 38), byt(oh, 39)}, ob, $countones(m_tx.tkeep));
            of = m_tx.tlast;
        end
    end
    // the stack's PSN check verdicts (rx_ibh_fsm, DBG_IBV): {ctl[103:96], epsn[95:64], psn[63:32], opcode[31:0]}
    always @(negedge nclk) if (inst_roce.m_axis_dbg_1.valid && inst_roce.m_axis_dbg_1.data[103:96] != 0 && inst_roce.m_axis_dbg_1.data[103:96] != 1)
        $display("PSNCHK node %0d t=%0t op %02h psn %06h epsn %06h verdict %0d", ID, $time, inst_roce.m_axis_dbg_1.data[7:0],
                 inst_roce.m_axis_dbg_1.data[55:32], inst_roce.m_axis_dbg_1.data[87:64], inst_roce.m_axis_dbg_1.data[103:96]);

    task automatic print_probes();
        $display("    retrans %s: ack updates written %0d read %0d, init records written %0d consumed %0d, inserts %0d, rx retrans req %0d, timer %0d, rt_state %0d, exh state %0d consumeReadInit %0d",
                 ID ? "B" : "A", r_upd_w, r_upd_r, r_init_w, r_init_r, r_ins, r_rxreq, r_timer,
                 inst_roce.rocev2_inst.inst.process_retransmissions_0_U0.rt_state,
                 inst_roce.rocev2_inst.inst.rx_exh_fsm_512_0_U0.pe_fsmState, inst_roce.rocev2_inst.inst.rx_exh_fsm_512_0_U0.consumeReadInit);
        $display("    counters %s: psn drops %0d, crc drops %0d, retransmissions %0d", ID ? "B" : "A",
                 inst_roce.psn_drop_pkg_count_data, inst_roce.crc_drop_pkg_count_data, inst_roce.retrans_count_data);
        $display("    probes %s: sq->hls %0d, rd_cmd %0d, payload beats %0d, mux->hls beats %0d, hls tx pkts %0d, mem wr/rd cmds %0d/%0d, rx pkts in %0d, ->hls %0d",
                 ID ? "B" : "A", c_sq, c_rdcmd, c_pay, c_net, c_tx, c_memwr, c_memrd, c_rx, c_gap);
    endtask

    // ---- 2026-10-27 rate / window / latency probes (owner: find the stage that limits traffic in both directions) ----
    // Cycles of cyc_n. A data packet = BTH opcode WRITE FIRST/MIDDLE/LAST/ONLY, an ACK = 0x11; key = {QP, PSN} (an ACK
    // carries the acknowledged PSN; QP numbers are equal on both sides). Window slot: rdma_flow's req_out takes one, an
    // rdma_ack frees one (one per packet). Round trip of a packet = req_out .. its slot freed, split into
    //   tx   req_out .. its first beat at this node's transmit output (m_tx)
    //   net  m_tx .. the ACK's first beat at this node's receive input (s_rx): link queue + delay, the peer, back
    //   ack  s_rx (ACK) .. rdma_ack (the gap enforcer, the stack's receive path)
    // and, measured at the peer for this node's data: peer = its s_rx (data) .. its m_tx (the ACK for it).
    function automatic bit is_data(logic [7:0] op); return op == 8'h06 || op == 8'h07 || op == 8'h08 || op == 8'h0A; endfunction
    function automatic int pkey(logic [511:0] d); return {byt(d, 35), byt(d, 37), byt(d, 38), byt(d, 39)}; endfunction
    longint reqt[NQ][$], txq[NQ][$];            // per QP: req_out times (popped at the slot's release / at m_tx)
    longint t_txd[int], t_rxd[int], t_rxa[$];   // data out, data in (from the peer), ACKs in (for the gap-enforcer wait)
    int     ostq[NQ];
    longint s_rtt = 0, n_rtt = 0, x_rtt = 0, s_ltx = 0, n_ltx = 0, s_lnet = 0, n_lnet = 0, s_lpeer = 0, n_lpeer = 0, s_lgap = 0, n_lgap = 0;
    longint k_tx = 0, k_rx = 0, c_ost = 0, c_blk = 0, c_txbp = 0, c_mtxbp = 0, c_paybp = 0, c_rxbp = 0, c_gaph = 0, c_rxfull = 0;
    longint n_dout = 0, n_aout = 0, n_din = 0, n_ain = 0, n_req = 0, n_rel = 0;
    longint fdt[$];                             // full data packets from the peer: first-beat time at s_rx (after the link)
    longint b_out = 0, b_in = 0;                // beats out / in while active
    bit pt_f = 1, pr_f = 1, ph_f = 1; logic [511:0] pt_h, pr_h; longint pt_t, pr_t; int pt_b = 0, pr_b = 0;
    localparam longint NCMD = longint'(NWR) * NQ * PKTS;
    always @(negedge nclk) if (nresetn) begin
        automatic bit tx_act = SEND && n_req > 0 && n_rel < NCMD;
        automatic bit rx_act = PEER_SEND && n_din > 0 && n_din < NCMD;
        automatic int oc = 0;
        // window slots
        if (inst_roce.inst_rdma_flow.req_out.valid && inst_roce.inst_rdma_flow.req_out.ready) begin
            automatic int q = inst_roce.inst_rdma_flow.req_out.data.req_1.pid;
            reqt[q].push_back(cyc_n); txq[q].push_back(cyc_n); ostq[q]++; n_req++;
        end
        if (inst_roce.rdma_ack.valid && inst_roce.rdma_ack.ready) begin
            automatic int q = inst_roce.rdma_ack.data.ack.pid;
            if (reqt[q].size()) begin
                automatic longint r = cyc_n - reqt[q].pop_front();
                s_rtt += r; n_rtt++; if (r > x_rtt) x_rtt = r;
            end
            ostq[q]--; n_rel++;
        end
        for (int q = 0; q < NQ; q++) oc += ostq[q];
        if (tx_act) begin
            k_tx++; c_ost += oc;
            if (sq.valid && !sq.ready && ostq[sq.data.req_1.pid] >= RDMA_N_WR_OUTSTANDING) c_blk++;
            if (inst_roce.roce_to_icrc.tvalid && !inst_roce.roce_to_icrc.tready) c_txbp++;
            if (m_tx.tvalid && !m_tx.tready) c_mtxbp++;
            if (inst_roce.axis_rdma_rd.tvalid && !inst_roce.axis_rdma_rd.tready) c_paybp++;
        end
        if (tx_act || rx_act) begin
            k_rx++;
            if (inst_roce.gap_enforcer_to_hls.tvalid && !inst_roce.gap_enforcer_to_hls.tready) c_rxbp++;
            if (inst_roce.fifo_to_gap_enforcer.tvalid && inst_roce.inst_ack_gap_enforcer.gap_counter != 0) c_gaph++;
            if (s_rx.tvalid && !s_rx.tready) c_rxfull++;
            if (m_tx.tvalid && m_tx.tready) b_out++;
            if (s_rx.tvalid && s_rx.tready) b_in++;
        end
        // transmit output: data packets (tx latency, key time, full-packet times), ACKs (the peer's latency)
        if (m_tx.tvalid && m_tx.tready) begin
            if (pt_f) begin pt_h = m_tx.tdata; pt_t = cyc_n; pt_b = 0; end
            pt_b++;
            if (pt_f && is_data(byt(m_tx.tdata, 28))) begin
                automatic int q = byt(m_tx.tdata, 35);
                n_dout++;
                t_txd[pkey(m_tx.tdata)] = cyc_n;
                if (q < NQ && txq[q].size()) begin s_ltx += cyc_n - txq[q].pop_front(); n_ltx++; end
            end
            if (pt_f && byt(m_tx.tdata, 28) == 8'h11) begin
                automatic int k = pkey(m_tx.tdata);
                n_aout++;
                if (t_rxd.exists(k)) begin s_lpeer += cyc_n - t_rxd[k]; n_lpeer++; t_rxd.delete(k); end
            end
            pt_f = m_tx.tlast;
        end
        // receive input: data from the peer (key time), ACKs (round trip on the wire side)
        if (s_rx.tvalid && s_rx.tready) begin
            if (pr_f) begin pr_h = s_rx.tdata; pr_t = cyc_n; pr_b = 0; end
            pr_b++;
            if (s_rx.tlast && is_data(byt(pr_h, 28)) && pr_b >= 60) fdt.push_back(pr_t);
            if (pr_f && is_data(byt(s_rx.tdata, 28))) begin n_din++; t_rxd[pkey(s_rx.tdata)] = cyc_n; end
            if (pr_f && byt(s_rx.tdata, 28) == 8'h11) begin
                automatic int k = pkey(s_rx.tdata);
                n_ain++; t_rxa.push_back(cyc_n);
                if (t_txd.exists(k)) begin s_lnet += cyc_n - t_txd[k]; n_lnet++; t_txd.delete(k); end
            end
            pr_f = s_rx.tlast;
        end
        // the ACK into the stack's receive path, after the input FIFO and the gap enforcer
        if (inst_roce.gap_enforcer_to_hls.tvalid && inst_roce.gap_enforcer_to_hls.tready) begin
            if (ph_f && byt(inst_roce.gap_enforcer_to_hls.tdata, 28) == 8'h11 && t_rxa.size()) begin s_lgap += cyc_n - t_rxa.pop_front(); n_lgap++; end
            ph_f = inst_roce.gap_enforcer_to_hls.tlast;
        end
    end
    function automatic real avg(longint s, longint n); return n ? real'(s) / n : 0.0; endfunction
    task automatic print_rate();
        automatic int n = fdt.size();
        automatic real st = 0, al = 0;
        if (n >= 8) begin
            automatic int i1 = n / 4, i2 = 3 * n / 4;
            st = real'(i2 - i1) * PMTU_BYTES / (real'(fdt[i2] - fdt[i1]) * 4.0);   // B/ns = GB/s at 250 MHz
            al = real'(n - 1) * PMTU_BYTES / (real'(fdt[n - 1] - fdt[0]) * 4.0);
        end
        $display("    RATE %s: full data packets in (the peer's direction) %0d, steady (middle half) %.2f GB/s, first..last %.2f GB/s | link beats out %.3f / in %.3f per cycle (25/32 = 0.781 line)",
                 ID ? "B" : "A", n, st, al, avg(b_out, k_rx), avg(b_in, k_rx));
        $display("    WIN  %s: sending %0d cycles, outstanding avg %.1f (window %0d per QP x %0d QPs), head blocked by a full window %.1f%% | data out %0d ACK out %0d | data in %0d ACK in %0d",
                 ID ? "B" : "A", k_tx, avg(c_ost, k_tx), RDMA_N_WR_OUTSTANDING, NQ, 100.0 * avg(c_blk, k_tx), n_dout, n_aout, n_din, n_ain);
        $display("    LAT  %s: round trip avg %.0f max %0d cycles = tx %.0f + net %.0f + ack-rx %.0f (rest) | ack: gap-enforcer wait %.0f | peer-side for the peer's data: rx->ACK out %.0f (n %0d/%0d/%0d)",
                 ID ? "B" : "A", avg(s_rtt, n_rtt), x_rtt, avg(s_ltx, n_ltx), avg(s_lnet, n_lnet), avg(s_rtt, n_rtt) - avg(s_ltx, n_ltx) - avg(s_lnet, n_lnet),
                 avg(s_lgap, n_lgap), avg(s_lpeer, n_lpeer), n_rtt, n_lnet, n_lpeer);
        $display("    STALL %s (%% of active cycles): stack tx out not ready %.1f, m_tx not ready (link buffer) %.1f, payload waits for the stack %.1f | rx: stack not ready %.1f, gap enforcer holds %.1f, input FIFO full %.1f",
                 ID ? "B" : "A", 100.0 * avg(c_txbp, k_tx), 100.0 * avg(c_mtxbp, k_tx), 100.0 * avg(c_paybp, k_tx), 100.0 * avg(c_rxbp, k_rx), 100.0 * avg(c_gaph, k_rx), 100.0 * avg(c_rxfull, k_rx));
    endtask

    // ---- retransmission buffer: a memory that decodes MEM_BITS address bits ----
    localparam logic [63:0] AMASK = MEM_BITS >= 64 ? '1 : (64'd1 << MEM_BITS) - 1;
    logic [511:0] mem [longint];
    logic [MEM_CMD_BITS-1:0] wq[$], rq[$];
    longint wbeat = 0, rbeat = 0;
    assign mem_wr_cmd.ready = 1; assign mem_rd_cmd.ready = 1; assign axis_mem_wr.tready = 1;
    assign mem_rd_sts.valid = 0; assign mem_wr_sts.valid = 0; assign mem_rd_sts.data = 0; assign mem_wr_sts.data = 0;
    logic rv = 0; logic [31:0] rlen = 0; logic [63:0] raddr = 0; logic [511:0] rdat = '0;
    assign axis_mem_rd.tvalid = rv;
    assign axis_mem_rd.tlast  = rbeat == (rlen + 63) / 64 - 1;
    assign axis_mem_rd.tkeep  = '1;
    assign axis_mem_rd.tdata  = rdat;
    bit mwc_v = 0, mrc_v = 0, mw_v = 0, mw_l = 0, mr_hs = 0, mr_l = 0;
    logic [MEM_CMD_BITS-1:0] mwc_d, mrc_d; logic [511:0] mw_d;
    always @(negedge nclk) begin
        mwc_v = mem_wr_cmd.valid; mwc_d = mem_wr_cmd.data; mrc_v = mem_rd_cmd.valid; mrc_d = mem_rd_cmd.data;
        mw_v = axis_mem_wr.tvalid; mw_d = axis_mem_wr.tdata; mw_l = axis_mem_wr.tlast;
        mr_hs = rv && axis_mem_rd.tready; mr_l = axis_mem_rd.tlast;
    end
    initial n_memrd = 0;
    always @(posedge nclk) begin
        #1;
        if (mwc_v) wq.push_back(mwc_d);
        if (mrc_v) begin rq.push_back(mrc_d); n_memrd <= n_memrd + 1; end
        if (mw_v && wq.size() != 0) begin
            mem[((wq[0][0+:64] & AMASK) >> 6) + wbeat] = mw_d;
            if (mw_l) begin wbeat = 0; void'(wq.pop_front()); end else wbeat = wbeat + 1;
        end
        if (mr_hs) begin
            if (mr_l) begin rbeat <= 0; rv <= 0; end
            else begin rbeat <= rbeat + 1; rdat <= mem.exists((raddr >> 6) + rbeat + 1) ? mem[(raddr >> 6) + rbeat + 1] : '0; end
        end
        if ((!rv || (mr_hs && mr_l)) && rq.size() != 0) begin
            automatic logic [MEM_CMD_BITS-1:0] c = rq.pop_front();
            automatic logic [63:0] a = c[0+:64] & AMASK;
            raddr <= a; rlen <= c[64+:32]; rbeat <= 0; rv <= 1;
            rdat <= mem.exists(a >> 6) ? mem[a >> 6] : '0;
        end
    end
endmodule
// Three ports of a store-and-forward switch (the N8550 between the U280s), per port:
//   node m_tx -> MAC transmit buffer (TXBUF beats; back-pressures the stack when full) -> line rate (25 beats / 32 cycles)
//   -> cable (DL cycles) -> switch ingress: a packet is routed by its IPv4 destination once complete; it enters the
//   egress queue of the destination port if SWQ beats have room, else the switch drops it (sw_drops, per egress port)
//   -> egress at line rate -> cable (DL) -> the node's input FIFO (RXQ beats; whole-packet drop when it cannot take the
//   packet, the CMAC without flow control: nic_drops) -> s_rx.
// Race-free as roce_link: handshakes latched at the falling edge, state updated 1 ps after the rising edge.
module sw3 #(
    parameter int DL = 250,
    parameter int TXBUF = 128,
    parameter int SWQ = 4096,
    parameter int RXQ = 512,
    parameter int STALL_CYC = 20000,
    parameter logic [31:0] IP0 = 32'h0A0B0B0A, IP1 = 32'h0C0D0D0C, IP2 = 32'h0E0F0F0E
) (
    input  logic nclk,
    input  logic nresetn,
    AXI4S.s t0, AXI4S.s t1, AXI4S.s t2,
    AXI4S.m r0, AXI4S.m r1, AXI4S.m r2,
    output int   sw_drops[3],
    output int   nic_drops[3],
    output int   sw_max[3],         // most beats an egress queue held
    output logic stalled[3],        // a node's input refused for STALL_CYC cycles in a row
    output int   tx_pkts[3],
    output int   rx_pkts[3]
);
    // beats packed as {last, keep, data} (xsim lost the top 32 data bits of a struct beat in an array of queues)
    typedef logic [576:0] bw_t;
    bw_t txq[3][$], wq[3][$], pk[3][$], eq[3][$], ow[3][$], rq[3][$];
    longint wqt[3][$], owt[3][$];   // arrival times of the beats in the two cables
    longint now = 0;
    int cr_in[3] = '{0, 0, 0}, cr_out[3] = '{0, 0, 0}, stall_run[3] = '{0, 0, 0};
    bit dropping[3] = '{0, 0, 0}, in_pkt[3] = '{0, 0, 0};
    logic t_v[3], t_r[3] = '{1, 1, 1}, t_l[3]; logic [511:0] t_d[3]; logic [63:0] t_k[3];
    logic o_v[3] = '{0, 0, 0}, o_r[3]; bw_t ob[3];
    assign t_v[0] = t0.tvalid; assign t_d[0] = t0.tdata; assign t_k[0] = t0.tkeep; assign t_l[0] = t0.tlast; assign t0.tready = t_r[0];
    assign t_v[1] = t1.tvalid; assign t_d[1] = t1.tdata; assign t_k[1] = t1.tkeep; assign t_l[1] = t1.tlast; assign t1.tready = t_r[1];
    assign t_v[2] = t2.tvalid; assign t_d[2] = t2.tdata; assign t_k[2] = t2.tkeep; assign t_l[2] = t2.tlast; assign t2.tready = t_r[2];
    assign r0.tvalid = o_v[0]; assign r0.tdata = o_v[0] ? ob[0][511:0] : '0; assign r0.tkeep = o_v[0] ? ob[0][575:512] : '0; assign r0.tlast = o_v[0] ? ob[0][576] : 1'b0; assign o_r[0] = r0.tready;
    assign r1.tvalid = o_v[1]; assign r1.tdata = o_v[1] ? ob[1][511:0] : '0; assign r1.tkeep = o_v[1] ? ob[1][575:512] : '0; assign r1.tlast = o_v[1] ? ob[1][576] : 1'b0; assign o_r[1] = r1.tready;
    assign r2.tvalid = o_v[2]; assign r2.tdata = o_v[2] ? ob[2][511:0] : '0; assign r2.tkeep = o_v[2] ? ob[2][575:512] : '0; assign r2.tlast = o_v[2] ? ob[2][576] : 1'b0; assign o_r[2] = r2.tready;
    function automatic int route(logic [511:0] d);   // IPv4 destination = bytes 16..19 of the first beat
        automatic logic [31:0] ip = {d[8*16 +: 8], d[8*17 +: 8], d[8*18 +: 8], d[8*19 +: 8]};
        return ip == IP0 ? 0 : ip == IP1 ? 1 : ip == IP2 ? 2 : -1;
    endfunction
    initial for (int i = 0; i < 3; i++) begin sw_drops[i] = 0; nic_drops[i] = 0; sw_max[i] = 0; stalled[i] = 0; tx_pkts[i] = 0; rx_pkts[i] = 0; end
    bit tx_hs[3], rx_hs[3], rx_wait[3]; bw_t tb[3];
    int bad_route = 0;
    always @(negedge nclk) for (int i = 0; i < 3; i++) begin
        tx_hs[i] = t_v[i] && t_r[i]; tb[i] = {t_l[i], t_k[i], t_d[i]};
        rx_hs[i] = o_v[i] && o_r[i]; rx_wait[i] = o_v[i] && !o_r[i];
    end
    always @(posedge nclk) begin
        bw_t b;
        #1;
        now <= now + 1;
        for (int i = 0; i < 3; i++) begin
            // node i transmit: MAC buffer, line rate, cable
            if (tx_hs[i]) begin txq[i].push_back(tb[i]); if (tb[i][576]) tx_pkts[i]++; end
            if (cr_in[i] >= 32 && txq[i].size() != 0) begin wq[i].push_back(txq[i].pop_front()); wqt[i].push_back(now + DL); cr_in[i] -= 7; end
            else cr_in[i] = cr_in[i] + 25 > 64 ? 64 : cr_in[i] + 25;
            t_r[i] <= txq[i].size() < TXBUF;
            // switch ingress i: assemble, route when complete, tail-drop at a full egress queue
            if (wq[i].size() != 0 && wqt[i][0] <= now) begin
                b = wq[i].pop_front(); void'(wqt[i].pop_front()); pk[i].push_back(b);
                if (b[576]) begin
                    automatic int e = route(pk[i][0][511:0]);
                    if (e < 0) begin bad_route++; pk[i].delete(); end
                    else if (eq[e].size() + pk[i].size() > SWQ) begin sw_drops[e]++; pk[i].delete(); end
                    else begin while (pk[i].size()) eq[e].push_back(pk[i].pop_front()); if (eq[e].size() > sw_max[e]) sw_max[e] = eq[e].size(); end
                end
            end
        end
        for (int e = 0; e < 3; e++) begin
            // egress e: line rate, cable, the node's input FIFO with whole-packet drop
            if (cr_out[e] >= 32 && eq[e].size() != 0) begin ow[e].push_back(eq[e].pop_front()); owt[e].push_back(now + DL); cr_out[e] -= 7; end
            else cr_out[e] = cr_out[e] + 25 > 64 ? 64 : cr_out[e] + 25;
            if (ow[e].size() != 0 && owt[e][0] <= now) begin
                b = ow[e].pop_front(); void'(owt[e].pop_front());
                if (!in_pkt[e]) dropping[e] = rq[e].size() + 66 > RXQ;
                if (dropping[e]) begin if (b[576]) nic_drops[e]++; end
                else rq[e].push_back(b);
                in_pkt[e] = !b[576];
            end
            if (rx_hs[e]) begin if (ob[e][576]) rx_pkts[e]++; o_v[e] <= 0; end
            if ((!o_v[e] || rx_hs[e]) && rq[e].size() != 0) begin ob[e] <= rq[e].pop_front(); o_v[e] <= 1; end
            stall_run[e] = rx_wait[e] ? stall_run[e] + 1 : 0;
            if (stall_run[e] == STALL_CYC) stalled[e] <= 1;
        end
    end
endmodule

module tb_roce_three_node;
    parameter int NWR = 20, WR_BYTES = 349568, DL = 250, STALL_CYC = 20000, TXBUF = 128, SWQ = 4096, RXQ = 512, MEM_BITS = 64;
    parameter int POSTGAP = 2000, JITTER = 0, SEED = 1, LANDSTALL_PCT = 0;
    parameter bit VERBOSE = 0;
    parameter longint TIMEOUT = 64'd3000000;
    localparam int NQ = 2;
    localparam logic [31:0] IP0 = 32'h0A0B0B0A, IP1 = 32'h0C0D0D0C, IP2 = 32'h0E0F0F0E;
    logic nclk = 0, nresetn = 0;
    always #2 nclk = ~nclk;   // 250 MHz
    initial begin repeat (50) @(posedge nclk); nresetn = 1; end

    AXI4S #(.AXI4S_DATA_BITS(AXI_NET_BITS)) x0 (.aclk(nclk), .aresetn(nresetn)), y0 (.aclk(nclk), .aresetn(nresetn));
    AXI4S #(.AXI4S_DATA_BITS(AXI_NET_BITS)) x1 (.aclk(nclk), .aresetn(nresetn)), y1 (.aclk(nclk), .aresetn(nresetn));
    AXI4S #(.AXI4S_DATA_BITS(AXI_NET_BITS)) x2 (.aclk(nclk), .aresetn(nresetn)), y2 (.aclk(nclk), .aresetn(nresetn));
    logic s[3]; int p[3], k[3], l[3], y[3], x[3], m[3];
    int sw_drops[3], nic_drops[3], sw_max[3], tx_pkts[3], rx_pkts[3]; logic stalled[3];

    roce_node3 #(.ID(0), .IP0(IP0), .IP1(IP1), .IP2(IP2), .NQ(NQ), .NWR(NWR), .WR_BYTES(WR_BYTES), .POSTGAP(POSTGAP), .JITTER(JITTER), .MEM_BITS(MEM_BITS), .FLAG_BYTES(0), .VERBOSE(VERBOSE), .LANDSTALL_PCT(LANDSTALL_PCT), .LSEED(SEED * 31 + 5))
        n0 (.nclk(nclk), .nresetn(nresetn), .m_tx(x0), .s_rx(y0), .setup_done(s[0]), .n_posted(p[0]), .n_acks(k[0]), .n_landed(l[0]), .n_wr_bytes(y[0]), .n_bad(x[0]), .n_memrd(m[0]));
    roce_node3 #(.ID(1), .IP0(IP0), .IP1(IP1), .IP2(IP2), .NQ(NQ), .NWR(NWR), .WR_BYTES(WR_BYTES), .POSTGAP(POSTGAP), .JITTER(JITTER), .MEM_BITS(MEM_BITS), .FLAG_BYTES(0), .VERBOSE(VERBOSE), .LANDSTALL_PCT(LANDSTALL_PCT), .LSEED(SEED * 131 + 9))
        n1 (.nclk(nclk), .nresetn(nresetn), .m_tx(x1), .s_rx(y1), .setup_done(s[1]), .n_posted(p[1]), .n_acks(k[1]), .n_landed(l[1]), .n_wr_bytes(y[1]), .n_bad(x[1]), .n_memrd(m[1]));
    roce_node3 #(.ID(2), .IP0(IP0), .IP1(IP1), .IP2(IP2), .NQ(NQ), .NWR(NWR), .WR_BYTES(WR_BYTES), .POSTGAP(POSTGAP), .JITTER(JITTER), .MEM_BITS(MEM_BITS), .FLAG_BYTES(0), .VERBOSE(VERBOSE), .LANDSTALL_PCT(LANDSTALL_PCT), .LSEED(SEED * 257 + 3))
        n2 (.nclk(nclk), .nresetn(nresetn), .m_tx(x2), .s_rx(y2), .setup_done(s[2]), .n_posted(p[2]), .n_acks(k[2]), .n_landed(l[2]), .n_wr_bytes(y[2]), .n_bad(x[2]), .n_memrd(m[2]));
    sw3 #(.DL(DL), .TXBUF(TXBUF), .SWQ(SWQ), .RXQ(RXQ), .STALL_CYC(STALL_CYC), .IP0(IP0), .IP1(IP1), .IP2(IP2))
        sw (.nclk(nclk), .nresetn(nresetn), .t0(x0), .t1(x1), .t2(x2), .r0(y0), .r1(y1), .r2(y2),
            .sw_drops(sw_drops), .nic_drops(nic_drops), .sw_max(sw_max), .stalled(stalled), .tx_pkts(tx_pkts), .rx_pkts(rx_pkts));

    localparam int PKTS = (WR_BYTES + PMTU_BYTES - 1) / PMTU_BYTES;
    longint cyc = 0;
    task automatic report(string why);
        $display("[%0t] %s | cycle %0d | steps posted %0d/%0d/%0d of %0d | sw drops %0d/%0d/%0d (egress max %0d/%0d/%0d beats of %0d) | nic drops %0d/%0d/%0d | input stalled %0d/%0d/%0d",
                 $time, why, cyc, n0.step, n1.step, n2.step, NWR, sw_drops[0], sw_drops[1], sw_drops[2], sw_max[0], sw_max[1], sw_max[2], SWQ,
                 nic_drops[0], nic_drops[1], nic_drops[2], stalled[0], stalled[1], stalled[2]);
        $display("    node 0: posted %0d acks %0d landed %0d from n1 %0d n2 %0d wrong %0d | tx pkts %0d rx pkts %0d", p[0], k[0], l[0], n0.from_n[1], n0.from_n[2], x[0], tx_pkts[0], rx_pkts[0]);
        $display("    node 1: posted %0d acks %0d landed %0d from n0 %0d n2 %0d wrong %0d | tx pkts %0d rx pkts %0d", p[1], k[1], l[1], n1.from_n[0], n1.from_n[2], x[1], tx_pkts[1], rx_pkts[1]);
        $display("    node 2: posted %0d acks %0d landed %0d from n0 %0d n1 %0d wrong %0d | tx pkts %0d rx pkts %0d", p[2], k[2], l[2], n2.from_n[0], n2.from_n[1], x[2], tx_pkts[2], rx_pkts[2]);
        $display("  node 0:"); n0.print_probes(); n0.print_rate();
        $display("  node 1:"); n1.print_probes(); n1.print_rate();
        $display("  node 2:"); n2.print_probes(); n2.print_rate();
    endtask
    always @(negedge nclk) begin
        cyc <= cyc + 1;
        if (cyc % 100000 == 0 && cyc) report("progress");
        if (stalled[0] || stalled[1] || stalled[2]) begin report("TB FAIL: receive stall"); $stop; end
        if (k[0] == NQ * NWR && k[1] == NQ * NWR && k[2] == NQ * NWR && l[0] == NQ * NWR * PKTS && l[1] == NQ * NWR * PKTS && l[2] == NQ * NWR * PKTS && cyc > 1000) begin
            if (x[0] || x[1] || x[2]) report("TB FAIL: data");
            else if (sw_drops[0] || sw_drops[1] || sw_drops[2] || nic_drops[0] || nic_drops[1] || nic_drops[2]) report("TB DONE with drops");
            else report("TB PASS");
            $finish;
        end
        if (cyc == TIMEOUT) begin report("TB FAIL: timeout"); $stop; end
    end
endmodule
