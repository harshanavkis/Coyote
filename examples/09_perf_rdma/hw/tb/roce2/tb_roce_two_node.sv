`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * Two RoCE nodes back to back (2026-10-25): the shell's roce_stack (HLS rocev2 + rdma_flow + rdma_mux_retrans +
 * ack_gap_enforcer, the RTL of a perf_rdma build) on each side, joined by a link that runs at 100 Gb/s of the 128 Gb/s
 * internal bus, delays every beat by LINK_DELAY cycles and, like the CMAC without flow control, drops a whole packet when
 * the receiver's input FIFO (RXQ beats) cannot take it. The host side of each node posts RDMA WRITEs the way the shell
 * does after dreq_rdma_parser_wr (one command per 4 KiB packet, FIRST/MIDDLE/LAST, req_1.vaddr remote, req_2.vaddr local),
 * delivers each command's payload in command order (the shell's parallel host read), and takes every landing write and
 * completion at once (ready always high), so a stall here is inside the stack. The retransmission buffer is a memory model.
 *
 * Traffic per node: NWR rounds of {one WR_BYTES WRITE per QP, then one 8 B WRITE per QP} on NQ QPs (rdma_mqp's pattern);
 * SEND_A / SEND_B pick the directions. Ends with TB PASS when every WR is acknowledged and landed, or TB FAIL on a receive
 * stall (input valid, not ready, for STALL_CYC cycles), a timeout, or a link drop.
 *
 * Race-free timing: every handshake is latched at the falling edge (all signals are stable then and stay so until the
 * rising edge the DUT samples), and the bench updates its state and outputs 1 ps after the rising edge.
 *
 * 2026-10-26 (owner: raise rdma_flow's window, "test it in simulation properly first"): every payload word encodes its
 * sender and remote address and the receiver checks every landed byte against it (TB FAIL: data); the link drops data
 * packets on purpose (DROP_FIRST: the first one, 1-based, 0 = none; DROP_EVERY: then every n-th; MAX_DROPS; counted on
 * the wire, retransmissions included), so the receiver NAKs and the sender resends from its retransmission buffer; the
 * buffer model decodes MEM_BITS address bits (64: every address distinct; 33: as the U280's HBM port, which takes
 * path_0_M_AXI_ARADDR[32:0]); the report gives the most packets node A had outstanding in rdma_flow.
 */
module roce_node #(
    parameter int          ID = 0,
    parameter logic [31:0] LIP = 32'h0A0B0B0A,
    parameter logic [31:0] RIP = 32'h0C0D0D0C,
    parameter int          NQ = 2,
    parameter int          NWR = 2,
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
            qp.data.qp_num = q; qp.data.local_psn = psn(ID, q); qp.data.remote_psn = psn(1 - ID, q);
            qp.valid = 1; do @(posedge nclk); while (!qp.ready); qp.valid = 0;
            repeat (20) @(posedge nclk);
            conn.data = '0;
            conn.data.local_qpn = q; conn.data.remote_qpn = q; conn.data.remote_ip_address = {4{RIP}};
            conn.data.remote_udp_port = 16'd18488;
            conn.valid = 1; do @(posedge nclk); while (!conn.ready); conn.valid = 0;
            repeat (20) @(posedge nclk);
        end
        repeat (200) @(posedge nclk);
        setup_done = 1;
        $display("[%0t] node %0d: %0d QPs set up, %0d commands queued", $time, ID, NQ, cmds.size());
    end

    // ---- commands: one per packet, as dreq_rdma_parser_wr emits them ----
    dreq_t cmds[$];
    int    clen[$];
    initial begin
        for (int r = 0; r < NWR && SEND; r++) begin
            for (int q = 0; q < NQ; q++)
                for (int p = 0; p < PKTS; p++) begin
                    dreq_t d = '0;
                    automatic int len = (p == PKTS - 1) ? WR_BYTES - p * PMTU_BYTES : PMTU_BYTES;
                    d.req_1.opcode = PKTS == 1 ? RC_RDMA_WRITE_ONLY : p == 0 ? RC_RDMA_WRITE_FIRST : p == PKTS - 1 ? RC_RDMA_WRITE_LAST : RC_RDMA_WRITE_MIDDLE;
                    d.req_1.strm = STRM_HOST; d.req_1.mode = RDMA_MODE_RAW; d.req_1.rdma = 1; d.req_1.remote = 1;
                    d.req_1.pid = q; d.req_1.last = (p == PKTS - 1); d.req_1.actv = 1; d.req_1.host = 1;
                    d.req_1.vaddr = (64'(q + 1) << 32) + WR_BYTES + p * PMTU_BYTES;    // remote: the peer's receive half
                    d.req_1.len = len;
                    d.req_2.vaddr = (64'(q + 1) << 32) + p * PMTU_BYTES;               // local: this side's send half
                    d.req_2.strm = STRM_HOST;
                    cmds.push_back(d); clen.push_back(len);
                end
            for (int q = 0; q < NQ && FLAG_BYTES > 0; q++) begin   // FLAG_BYTES 0: no flag WRs (2026-10-28, Loom-like streams)
                dreq_t d = '0;
                d.req_1.opcode = RC_RDMA_WRITE_ONLY; d.req_1.strm = STRM_HOST; d.req_1.mode = RDMA_MODE_RAW; d.req_1.rdma = 1;
                d.req_1.remote = 1; d.req_1.pid = q; d.req_1.last = 1; d.req_1.actv = 1; d.req_1.host = 1;
                d.req_1.vaddr = (64'(q + 1) << 32) + 2 * WR_BYTES; d.req_1.len = FLAG_BYTES;
                d.req_2.vaddr = (64'(q + 1) << 32) + WR_BYTES - FLAG_BYTES; d.req_2.strm = STRM_HOST;
                cmds.push_back(d); clen.push_back(FLAG_BYTES);
            end
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
            automatic logic [511:0] exp = beat_data(1 - ID, lq_a[0] + 64 * lbeat);
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
            if (lbeat == (lq_n[0] + 63) / 64 - 1) begin lbeat = 0; void'(lq_a.pop_front()); void'(lq_n.pop_front()); end
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
    localparam longint NCMD = longint'(NWR) * NQ * (PKTS + (FLAG_BYTES > 0));
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

// The link from one node's transmit to the other's receive: line rate, delay, whole-packet drop at a full input FIFO, and
// forced drops of data packets (more than one beat; ACKs and NAKs are one beat) to make the sender retransmit.
module roce_link #(
    parameter int DELAY = 500,
    parameter int RXQ = 512,
    parameter int STALL_CYC = 20000,
    parameter int TXBUF = 0,          // transmit buffer in beats before the line-rate limit; 0 = unbounded (never back-pressures)
    parameter int TXSTALL_PCT = 0,    // 2026-10-28: the MAC also refuses a beat at random in this % of cycles (0 = never)
    parameter int SEED = 1,           // seed of the stall pattern (per link)
    parameter int DROP_FIRST = 0,     // first data packet to drop (1-based), 0 = none
    parameter int DROP_EVERY = 0,     // then every n-th data packet (0 = only the first)
    parameter int MAX_DROPS = 0
) (
    input  logic nclk,
    input  logic nresetn,
    AXI4S.s      s_tx,
    AXI4S.m      m_rx,
    output int   drops,
    output int   forced,
    output logic stalled,
    output int   tx_pkts,
    output int   rx_pkts
);
    typedef struct { logic [511:0] d; logic [63:0] k; logic l; longint t; } beat_t;
    beat_t wire_q[$], rx_q[$], tx_q[$];
    longint now = 0;
    int credit = 0;          // 100 Gb/s of 128: 25 beats in 32 cycles
    logic [31:0] rng = 32'h9e3779b9 ^ SEED;
    bit dropping = 0, in_pkt = 0;
    int stall_run = 0;
    logic tx_rdy = 1;
    assign s_tx.tready = tx_rdy;
    logic ov = 0; beat_t ob;
    assign m_rx.tvalid = ov;
    assign m_rx.tdata = ov ? ob.d : '0;
    assign m_rx.tkeep = ov ? ob.k : '0;
    assign m_rx.tlast = ov ? ob.l : 1'b0;
    initial begin drops = 0; forced = 0; stalled = 0; tx_pkts = 0; rx_pkts = 0; end
    int n_data = 0;
    bit forced_pkt = 0;
    bit tx_hs = 0, rx_hs = 0, rx_wait = 0; beat_t tx_b;
    always @(negedge nclk) begin
        tx_hs = s_tx.tvalid && s_tx.tready; tx_b.d = s_tx.tdata; tx_b.k = s_tx.tkeep; tx_b.l = s_tx.tlast;
        rx_hs = ov && m_rx.tready; rx_wait = m_rx.tvalid && !m_rx.tready;
    end
    always @(posedge nclk) begin
        beat_t b;
        #1;
        now <= now + 1;
        if (tx_hs) begin tx_q.push_back(tx_b); if (tx_b.l) tx_pkts <= tx_pkts + 1; end
        if (credit >= 32 && tx_q.size() != 0) begin   // line rate: 25 beats in 32 cycles
            b = tx_q.pop_front(); b.t = now + DELAY; wire_q.push_back(b);
            credit <= credit - 32 + 25;
        end else credit <= credit + 25 > 64 ? 64 : credit + 25;
        rng = rng ^ (rng << 13); rng = rng ^ (rng >> 17); rng = rng ^ (rng << 5);   // xorshift32 (xsim: no seeded $urandom)
        tx_rdy <= (TXBUF == 0 || tx_q.size() < TXBUF) && !(TXSTALL_PCT > 0 && (rng % 100) < TXSTALL_PCT);
        if (wire_q.size() != 0 && wire_q[0].t <= now) begin
            b = wire_q.pop_front();
            if (!in_pkt) begin   // a packet's first beat: a forced drop? room for a whole packet? (rx_q = the input FIFO)
                automatic bit force_drop = 0;
                if (!b.l) begin  // a data packet
                    n_data = n_data + 1;
                    force_drop = DROP_FIRST > 0 && forced < MAX_DROPS && n_data >= DROP_FIRST &&
                                 (DROP_EVERY == 0 ? n_data == DROP_FIRST : (n_data - DROP_FIRST) % DROP_EVERY == 0);
                end
                if (force_drop) forced <= forced + 1;
                forced_pkt = force_drop;
                dropping = force_drop || rx_q.size() + 66 > RXQ;
            end
            if (dropping) begin if (b.l && !forced_pkt) drops <= drops + 1; end
            else rx_q.push_back(b);
            in_pkt = !b.l;
        end
        if (rx_hs) begin if (ob.l) rx_pkts <= rx_pkts + 1; ov <= 0; end
        if ((!ov || rx_hs) && rx_q.size() != 0) begin ob <= rx_q.pop_front(); ov <= 1; end
        stall_run <= rx_wait ? stall_run + 1 : 0;
        if (stall_run == STALL_CYC) stalled <= 1;
    end
endmodule

module tb_roce_two_node;
    parameter int NQ = 2, NWR = 2, WR_BYTES = 1 << 20, DELAY = 500, STALL_CYC = 20000, TXBUF = 128, MEM_BITS = 64, FLAG_BYTES = 8, CORRUPT_BEAT = -1;
    parameter bit VERBOSE = 1;
    parameter int DROP_FIRST = 0, DROP_EVERY = 0, MAX_DROPS = 0;
    parameter int TXSTALL_PCT = 0, SEED = 1;   // 2026-10-28: random transmit back-pressure (roce_link)
    parameter int LANDSTALL_PCT = 0;           // 2026-10-28: random landing back-pressure (roce_node host)
    parameter bit SEND_A = 1, SEND_B = 1;
    parameter longint TIMEOUT = 64'd300000;
    logic nclk = 0, nresetn = 0;
    always #2 nclk = ~nclk;   // 250 MHz
    initial begin repeat (50) @(posedge nclk); nresetn = 1; end

    AXI4S #(.AXI4S_DATA_BITS(AXI_NET_BITS)) a_tx (.aclk(nclk), .aresetn(nresetn)), a_rx (.aclk(nclk), .aresetn(nresetn));
    AXI4S #(.AXI4S_DATA_BITS(AXI_NET_BITS)) b_tx (.aclk(nclk), .aresetn(nresetn)), b_rx (.aclk(nclk), .aresetn(nresetn));
    logic sa, sb, st_ab, st_ba;
    int pa, pb, ka, kb, la, lb, ya, yb, xa, xb, ma, mb, d_ab, d_ba, f_ab, f_ba, t_ab, r_ab, t_ba, r_ba;

    roce_node #(.ID(0), .LIP(32'h0A0B0B0A), .RIP(32'h0C0D0D0C), .NQ(NQ), .NWR(NWR), .WR_BYTES(WR_BYTES), .SEND(SEND_A), .MEM_BITS(MEM_BITS), .FLAG_BYTES(FLAG_BYTES), .CORRUPT_BEAT(-1), .VERBOSE(VERBOSE), .PEER_SEND(SEND_B), .LANDSTALL_PCT(LANDSTALL_PCT), .LSEED(SEED * 31 + 5))
        na (.nclk(nclk), .nresetn(nresetn), .m_tx(a_tx), .s_rx(a_rx), .setup_done(sa), .n_posted(pa), .n_acks(ka), .n_landed(la), .n_wr_bytes(ya), .n_bad(xa), .n_memrd(ma));
    roce_node #(.ID(1), .LIP(32'h0C0D0D0C), .RIP(32'h0A0B0B0A), .NQ(NQ), .NWR(NWR), .WR_BYTES(WR_BYTES), .SEND(SEND_B), .MEM_BITS(MEM_BITS), .FLAG_BYTES(FLAG_BYTES), .CORRUPT_BEAT(CORRUPT_BEAT), .VERBOSE(VERBOSE), .PEER_SEND(SEND_A), .LANDSTALL_PCT(LANDSTALL_PCT), .LSEED(SEED * 131 + 9))
        nb (.nclk(nclk), .nresetn(nresetn), .m_tx(b_tx), .s_rx(b_rx), .setup_done(sb), .n_posted(pb), .n_acks(kb), .n_landed(lb), .n_wr_bytes(yb), .n_bad(xb), .n_memrd(mb));
    roce_link #(.DELAY(DELAY), .STALL_CYC(STALL_CYC), .TXBUF(TXBUF), .DROP_FIRST(DROP_FIRST), .DROP_EVERY(DROP_EVERY), .MAX_DROPS(MAX_DROPS), .TXSTALL_PCT(TXSTALL_PCT), .SEED(SEED))
        l_ab (.nclk(nclk), .nresetn(nresetn), .s_tx(a_tx), .m_rx(b_rx), .drops(d_ab), .forced(f_ab), .stalled(st_ab), .tx_pkts(t_ab), .rx_pkts(r_ab));
    roce_link #(.DELAY(DELAY), .STALL_CYC(STALL_CYC), .TXBUF(TXBUF), .DROP_FIRST(DROP_FIRST), .DROP_EVERY(DROP_EVERY), .MAX_DROPS(MAX_DROPS), .TXSTALL_PCT(TXSTALL_PCT), .SEED(SEED * 7919 + 1))
        l_ba (.nclk(nclk), .nresetn(nresetn), .s_tx(b_tx), .m_rx(a_rx), .drops(d_ba), .forced(f_ba), .stalled(st_ba), .tx_pkts(t_ba), .rx_pkts(r_ba));

    // rdma_flow's window at node A: requests it passed to the stack minus the acknowledgements that freed a slot
    int ost = 0, ost_max = 0;
    always @(negedge nclk) begin
        ost = ost + (na.inst_roce.inst_rdma_flow.m_req.valid && na.inst_roce.inst_rdma_flow.m_req.ready)
                  - (na.inst_roce.inst_rdma_flow.s_ack.valid && na.inst_roce.inst_rdma_flow.s_ack.ready);
        if (ost > ost_max) ost_max = ost;
    end

    localparam int PKTS = (WR_BYTES + PMTU_BYTES - 1) / PMTU_BYTES;
    localparam int WRS = NWR * NQ * (1 + (FLAG_BYTES > 0));        // data + flag WRs per sending node
    localparam int LANDS = NWR * NQ * (PKTS + (FLAG_BYTES > 0));   // landing writes per receiving node
    longint cyc = 0;
    task automatic report(string why);
        $display("[%0t] %s | cycle %0d | A: posted %0d acks %0d landed %0d (%0d B) | B: posted %0d acks %0d landed %0d (%0d B) | pkts A->B sent %0d taken %0d, B->A sent %0d taken %0d | drops A->B %0d B->A %0d | stall A->B %0d B->A %0d",
                 $time, why, cyc, pa, ka, la, ya, pb, kb, lb, yb, t_ab, r_ab, t_ba, r_ba, d_ab, d_ba, st_ab, st_ba);
        $display("    check: wrong landings A %0d B %0d | forced drops A->B %0d B->A %0d | retransmission-buffer reads A %0d B %0d | A outstanding max %0d (window %0d) | MEM_BITS %0d",
                 xa, xb, f_ab, f_ba, ma, mb, ost_max, RDMA_N_WR_OUTSTANDING, MEM_BITS);
        na.print_probes();
        nb.print_probes();
        na.print_rate();
        nb.print_rate();
    endtask
    always @(negedge nclk) begin
        cyc <= cyc + 1;
        if (cyc % 50000 == 0 && cyc) report("progress");
        if (st_ab || st_ba) begin report("TB FAIL: receive stall"); $stop; end
        if ((d_ab || d_ba) && !$isunknown(d_ab)) begin end
        if ((!SEND_A || ka == WRS) && (!SEND_B || kb == WRS) && (!SEND_A || lb == LANDS) && (!SEND_B || la == LANDS) && sa && sb && cyc > 1000) begin
            if (xa || xb) report("TB FAIL: data"); else if (d_ab || d_ba) report("TB DONE with drops"); else report("TB PASS");
            $finish;
        end
        if (cyc == TIMEOUT) begin report("TB FAIL: timeout"); $stop; end
    end
endmodule
