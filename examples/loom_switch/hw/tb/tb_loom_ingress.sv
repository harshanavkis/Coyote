`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_loom_ingress - loom_table + loom_ingress; the TB drives AXI4 bursts on
 * the uwin and mocks the shell (sq_wr, the host and rdma send streams, the
 * ack window).
 *
 * Every test lists its bursts; a reference packetiser turns them into the
 * packets expected (route, pid, target, beats, payload), and the checker
 * compares the requests, inline messages and payload beats exactly, in
 * order. An rdma packet is self-describing: its request's vaddr (the RETH)
 * is the binding's far export reference plus the offset, and its payload
 * follows with no header.
 *
 * Covers: aligned packets on both routes, a run that starts mid-packet,
 * back-to-back bursts over several packets, 1-beat bursts at one a cycle,
 * alternating windows, an offset gap in one window, drops (past the
 * window's end, unmapped, invalid entry), a 64 MiB window, 8 B stores (lone,
 * several in one beat, inside a run, a flag behind bulk, partial words
 * dropped), writes that start inside a line (32 B half-lines, 8 B stores at
 * their own address, a burst from line+32), backpressure everywhere, the
 * debug counters (drop reasons, burst sizes, misaligned bursts, and every
 * cycle a W beat waits counted under exactly one reason). Reads and get
 * requests are loom_read's (tb_loom_switch_top).
 */
module tb_loom_ingress;

localparam integer UWIN_BITS = 27;
localparam integer FLUSH     = 16;
localparam integer PKT_BEATS = PMTU_BYTES / 64;

logic aclk = 0;
logic aresetn = 0;
always #2 aclk = ~aclk;

AXI4 #(.AXI4_ADDR_BITS(64)) axi (.aclk(aclk));

// table programming
logic                  tbl_commit = 0;
logic [3:0]            tbl_idx = 0;
logic                  tbl_valid = 0, tbl_route = 0;
logic [PID_BITS-1:0]   tbl_pid = 0, tbl_dst_pid = 0;
logic [VADDR_BITS-1:0] tbl_base = 0;
logic [LEN_BITS-1:0]   tbl_len = 0;
logic [UWIN_BITS-1:0]  tbl_ustart = 0;

logic [UWIN_BITS-1:0]  ua_addr;
logic                  ua_hit, ua_route;
logic [3:0]            ua_idx;
logic [PID_BITS-1:0]   ua_pid, ua_dst_pid;
logic [VADDR_BITS-1:0] ua_base;
logic [UWIN_BITS-1:0]  ua_ustart;
logic [LEN_BITS:0]     ua_end;
logic                  ua_ce1, ua_ce2;

localparam logic [VADDR_BITS-1:0] STAGING = 48'h7f00_0000_0000;

req_t wr_req;
logic wr_valid, wr_ready;
logic win_ok, rdma_post;
logic [AXI_DATA_BITS-1:0]   m_host_tdata, m_net_tdata;
logic [AXI_DATA_BITS/8-1:0] m_host_tkeep, m_net_tkeep;
logic m_host_tvalid, m_host_tready, m_host_tlast;
logic m_net_tvalid, m_net_tready, m_net_tlast;
logic cnt_burst, cnt_drop, cnt_pkt_local, cnt_pkt_rdma, cnt_store, cnt_store_drop, cnt_flush;
logic cnt_rdma_full, cnt_rdma_flush, cnt_rdma_cut;
localparam integer N_DBG = 14;
logic [N_DBG-1:0] cnt_dbg;

loom_table #(.UWIN_BITS(UWIN_BITS)) inst_table (
    .aclk(aclk), .aresetn(aresetn),
    .commit(tbl_commit), .prog_idx(tbl_idx), .prog_valid(tbl_valid),
    .prog_route(tbl_route), .prog_pid(tbl_pid), .prog_dst_pid(tbl_dst_pid),
    .prog_base(tbl_base), .prog_len(tbl_len), .prog_ustart(tbl_ustart),
    .lu_idx(4'd0), .lu_valid(), .lu_route(), .lu_pid(), .lu_dst_pid(),
    .lu_base(), .lu_len(),
    .ua_ce1(ua_ce1), .ua_ce2(ua_ce2),
    .ua_addr(ua_addr), .ua_hit(ua_hit), .ua_idx(ua_idx), .ua_route(ua_route),
    .ua_pid(ua_pid), .ua_dst_pid(ua_dst_pid), .ua_base(ua_base),
    .ua_ustart(ua_ustart), .ua_end(ua_end),
    .ub_ce1(1'b0), .ub_ce2(1'b0), .ub_addr('0), .ub_hit(), .ub_route(), .ub_pid(), .ub_base(),
    .ub_ustart(), .ub_end()
);

loom_ingress #(.UWIN_BITS(UWIN_BITS), .FLUSH_CYCLES(FLUSH)) dut (
    .aclk(aclk), .aresetn(aresetn), .axi_udata(axi),
    .ua_ce1(ua_ce1), .ua_ce2(ua_ce2),
    .ua_addr(ua_addr), .ua_hit(ua_hit), .ua_route(ua_route), .ua_pid(ua_pid),
    .ua_dst_pid(ua_dst_pid), .ua_base(ua_base), .ua_ustart(ua_ustart),
    .ua_end(ua_end), .ua_idx(ua_idx),
    .s_get_valid(1'b0), .s_get_ready(), .s_get_pid('0), .s_get_ref('0), .s_get_word('0),
    .wr_req(wr_req), .wr_valid(wr_valid), .wr_ready(wr_ready),
    .win_ok(win_ok), .rdma_post(rdma_post),
    .m_host_tdata(m_host_tdata), .m_host_tkeep(m_host_tkeep),
    .m_host_tvalid(m_host_tvalid), .m_host_tready(m_host_tready),
    .m_host_tlast(m_host_tlast),
    .m_net_tdata(m_net_tdata), .m_net_tkeep(m_net_tkeep),
    .m_net_tvalid(m_net_tvalid), .m_net_tready(m_net_tready),
    .m_net_tlast(m_net_tlast),
    .cnt_burst(cnt_burst), .cnt_drop(cnt_drop), .cnt_pkt_local(cnt_pkt_local),
    .cnt_pkt_rdma(cnt_pkt_rdma), .cnt_store(cnt_store), .cnt_store_drop(cnt_store_drop),
    .cnt_flush(cnt_flush), .cnt_win_wait(), .cnt_req_wait(),
    .cnt_rdma_full(cnt_rdma_full), .cnt_rdma_flush(cnt_rdma_flush), .cnt_rdma_cut(cnt_rdma_cut),
    .cnt_dbg(cnt_dbg)
);

int errors = 0;
`define CHECK(c, msg) if (!(c)) begin errors++; $display("FAIL [%0t] %s", $time, msg); end

// ---------------------------------------------------------------------------
// The windows every test uses
// ---------------------------------------------------------------------------
typedef struct {
    bit                    valid;
    bit                    route;
    logic [PID_BITS-1:0]   pid, dst_pid;
    logic [VADDR_BITS-1:0] base;
    longint                len;
    longint                ustart;
} win_t;
win_t W [16];

task automatic program_win(input int i, input win_t w);
    W[i] = w;
    @(posedge aclk);
    tbl_idx <= 4'(i); tbl_valid <= w.valid; tbl_route <= w.route;
    tbl_pid <= w.pid; tbl_dst_pid <= w.dst_pid; tbl_base <= w.base;
    tbl_len <= LEN_BITS'(w.len); tbl_ustart <= UWIN_BITS'(w.ustart);
    tbl_commit <= 1;
    @(posedge aclk);
    tbl_commit <= 0;
endtask

// Payload of the beat at a uwin address: unique per address and per run
int unsigned salt = 0;
function automatic logic [AXI_DATA_BITS-1:0] pat(input longint uaddr);
    logic [AXI_DATA_BITS-1:0] d;
    for (int l = 0; l < 8; l++) d[64*l +: 64] = {32'(salt + l), 32'(uaddr)};
    return d;
endfunction

// ---------------------------------------------------------------------------
// Bursts and the reference packetiser
// ---------------------------------------------------------------------------
typedef struct {
    longint      uaddr;      // uwin address (the first beat's line is uaddr rounded down to 64 B)
    int          beats;
    bit          gap_after;  // idle past the flush timer after this burst
    bit          gap_held;   // the same idle while the output is held: no close
    logic [63:0] strb [64];  // per beat
} burst_t;

typedef struct {
    bit                    store;   // an 8 B store; else a packet
    logic [63:0]           data;    // the store's word
    bit                    route;
    logic [PID_BITS-1:0]   pid, dst_pid;
    logic [VADDR_BITS-1:0] va;
    longint                uaddr;   // uwin address of the first beat
    int                    beats;
} pkt_t;

burst_t bursts [$];
pkt_t   exp_pkts [$];
int     exp_drops, exp_store_drops;
int     exp_nowin, exp_oob, exp_b1, exp_b4, exp_bmore, exp_mis;

function automatic longint line_of(input longint uaddr);
    return uaddr & ~64'd63;
endfunction

// Which window holds a burst's first line, and whether it is taken (the
// RTL's rule); oob says a window holds it but its lines end past the window
function automatic int win_of(input burst_t b, output longint off, output bit oob);
    longint a = line_of(b.uaddr);
    oob = 0;
    for (int i = 1; i < 16; i++)
        if (W[i].valid && a >= W[i].ustart && a < W[i].ustart + W[i].len) begin
            off = a - W[i].ustart;
            if (off + 64*b.beats > W[i].len) begin oob = 1; return -1; end
            return i;
        end
    return -1;
endfunction

task automatic model();
    bit     open = 0;
    int     p_win;
    longint p_next;
    pkt_t   p;
    exp_pkts.delete();
    exp_drops = 0;
    exp_store_drops = 0;
    exp_nowin = 0; exp_oob = 0; exp_b1 = 0; exp_b4 = 0; exp_bmore = 0; exp_mis = 0;
    p.store = 0; p.data = 0;
    foreach (bursts[k]) begin
        longint off;
        bit oob;
        int wi = win_of(bursts[k], off, oob);
        if (bursts[k].beats == 1)     exp_b1++;
        else if (bursts[k].beats <= 4) exp_b4++;
        else                           exp_bmore++;
        exp_mis += (bursts[k].uaddr % 64 != 0);
        if (wi < 0) begin
            exp_drops++;
            if (oob) exp_oob++; else exp_nowin++;
            continue;
        end
        for (int j = 0; j < bursts[k].beats; j++) begin
            longint o = off + 64*j;
            int cap = PKT_BEATS;
            logic [63:0] sb = bursts[k].strb[j];
            if (sb != '1) begin
                // stores: close the open packet, then one per whole word
                bit bad = 0;
                if (open) begin exp_pkts.push_back(p); open = 0; end
                for (int l = 0; l < 8; l++) begin
                    if (sb[8*l +: 8] == 8'hFF) begin
                        pkt_t st;
                        logic [AXI_DATA_BITS-1:0] line = pat(W[wi].ustart + o);
                        st.store = 1; st.route = W[wi].route; st.pid = W[wi].pid;
                        st.dst_pid = W[wi].dst_pid; st.va = W[wi].base + o + 8*l;
                        st.uaddr = W[wi].ustart + o; st.beats = 0;
                        st.data = line[64*l +: 64];
                        exp_pkts.push_back(st);
                    end else if (sb[8*l +: 8] != 8'h00) bad = 1;
                end
                exp_store_drops += bad;
                continue;
            end
            if (open && p_win == wi && p_next == o) begin
                p.beats++;
            end else begin
                if (open) exp_pkts.push_back(p);
                p.route = W[wi].route; p.pid = W[wi].pid; p.dst_pid = W[wi].dst_pid;
                p.va = W[wi].base + o; p.uaddr = W[wi].ustart + o; p.beats = 1;
                p_win = wi; open = 1;
            end
            p_next = o + 64;
            if (p.beats == cap) begin exp_pkts.push_back(p); open = 0; end
        end
        if (bursts[k].gap_after && open) begin exp_pkts.push_back(p); open = 0; end
    end
    if (open) exp_pkts.push_back(p);
endtask

// ---------------------------------------------------------------------------
// AXI master: AW and W run independently, so bursts go back to back
// ---------------------------------------------------------------------------
bit bp = 0;          // random backpressure and W bubbles
bit win_shut = 0;    // hold the ack window shut until every burst is answered
// the send state as a plain vector: xsim reads int'(dut.ostate) in a wait
// as 0 while it is not
wire [2:0] dut_ost = dut.ostate;
int aw_q [$];        // indices into bursts
int w_q  [$];
int b_seen = 0, b_expect_id = 0;

initial begin
    axi.awvalid = 0; axi.wvalid = 0; axi.arvalid = 0; axi.bready = 1; axi.rready = 1;
    axi.awburst = 2'b01; axi.awsize = 3'd6; axi.awcache = 0; axi.awlock = 0;
    axi.awprot = 0; axi.awqos = 0; axi.awregion = 0;
    axi.arburst = 2'b01; axi.arsize = 3'd6; axi.arcache = 0; axi.arlock = 0;
    axi.arprot = 0; axi.arqos = 0; axi.arregion = 0;
    axi.wstrb = '1; axi.wlast = 0; axi.wdata = 0;
end

always begin
    if (aw_q.size() == 0) @(posedge aclk);
    else begin
        int k = aw_q.pop_front();
        axi.awaddr  <= 64'h0800_0000 + bursts[k].uaddr;   // the BAR offset, as the shell passes it
        axi.awlen   <= 8'(bursts[k].beats - 1);
        axi.awid    <= AXI_ID_BITS'(k);
        axi.awvalid <= 1;
        do @(posedge aclk); while (!axi.awready);
        axi.awvalid <= 0;
    end
end

always begin
    if (w_q.size() == 0) @(posedge aclk);
    else begin
        int k = w_q.pop_front();
        for (int j = 0; j < bursts[k].beats; j++) begin
            if (bp) repeat ($urandom_range(0, 1) ? 0 : $urandom_range(1, 3)) @(posedge aclk);
            axi.wdata  <= pat(line_of(bursts[k].uaddr) + 64*j);
            axi.wstrb  <= bursts[k].strb[j];
            axi.wlast  <= (j == bursts[k].beats - 1);
            axi.wvalid <= 1;
            do @(posedge aclk); while (!axi.wready);
            axi.wvalid <= 0;
        end
        if (bursts[k].gap_after || bursts[k].gap_held) repeat (4*FLUSH) @(posedge aclk);
    end
end

always @(posedge aclk) begin
    if (axi.bvalid && axi.bready) begin
        `CHECK(axi.bid == AXI_ID_BITS'(b_expect_id), $sformatf("bid %0d, expected %0d", axi.bid, b_expect_id))
        `CHECK(axi.bresp == 2'b00, "bresp not OKAY")
        b_expect_id++;
        b_seen++;
    end
    if (bp) axi.bready <= $urandom_range(0, 3) != 0;
    else    axi.bready <= 1;
end

// ---------------------------------------------------------------------------
// Shell mocks and capture
// ---------------------------------------------------------------------------
req_t reqs [$];
logic [AXI_DATA_BITS-1:0] host_beats [$], net_beats [$];
logic [AXI_DATA_BITS/8-1:0] host_keep [$];
bit   host_last [$], net_last [$];
int   n_burst = 0, n_drop = 0, n_pkt_local = 0, n_pkt_rdma = 0, n_post = 0;
int   n_store = 0, n_store_drop = 0;
int   n_full = 0, n_part = 0;
int   n_dbg [N_DBG];
bit   check_rate = 0;    // the next run's bursts must go in at one beat a cycle
// Input rate: W beats taken between the first and the last
longint cyc = 0, w_first = -1, w_last = 0, w_beats = 0;
always @(posedge aclk) cyc++;

always @(posedge aclk) begin
    wr_ready      <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
    win_ok        <= win_shut ? 1'b0 : bp ? ($urandom_range(0, 3) != 0) : 1'b1;
    m_host_tready <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
    m_net_tready  <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
end

always @(posedge aclk) if (aresetn) begin
    if (wr_valid && wr_ready) reqs.push_back(wr_req);
    if (m_host_tvalid && m_host_tready) begin
        host_beats.push_back(m_host_tdata); host_last.push_back(m_host_tlast);
        host_keep.push_back(m_host_tkeep);
    end
    if (m_net_tvalid && m_net_tready) begin
        net_beats.push_back(m_net_tdata); net_last.push_back(m_net_tlast);
        `CHECK(m_net_tkeep == '1, "net tkeep not full")
    end
    n_burst     += cnt_burst;
    n_drop      += cnt_drop;
    n_pkt_local += cnt_pkt_local;
    n_pkt_rdma  += cnt_pkt_rdma;
    n_post      += rdma_post;
    n_store     += cnt_store;
    n_store_drop += cnt_store_drop;
    n_full      += cnt_rdma_full;
    n_part      += cnt_rdma_flush + cnt_rdma_cut;
    for (int i = 0; i < N_DBG; i++) n_dbg[i] += cnt_dbg[i];
    // a W beat that waits is counted under exactly one reason
    `CHECK((axi.wvalid && !axi.wready) == (cnt_dbg[7:3] != 0) && $onehot0(cnt_dbg[7:3]),
           $sformatf("W wait reasons %b with wvalid %0d wready %0d", cnt_dbg[7:3], axi.wvalid, axi.wready))
    `CHECK(cnt_dbg[0] == (m_host_tvalid && !m_host_tready) && cnt_dbg[1] == (m_net_tvalid && !m_net_tready),
           "output backpressure pulses")
    if (axi.wvalid && axi.wready) begin
        if (w_first < 0) w_first = cyc;
        w_last = cyc;
        w_beats++;
    end
    if (wr_valid && wr_req.strm == STRM_RDMA) `CHECK(win_ok, "rdma request presented while the window is shut")
end

// ---------------------------------------------------------------------------
// Run one test: send, wait for everything to drain, compare
// ---------------------------------------------------------------------------
task automatic run(input string name);
    int n_local = 0, n_rdma = 0, n_st = 0, n_st_rdma = 0, n_bursts;
    int e_full = 0, e_part = 0;
    int e0 = errors;
    reqs.delete(); host_beats.delete(); net_beats.delete();
    host_last.delete(); net_last.delete(); host_keep.delete();
    n_burst = 0; n_drop = 0; n_pkt_local = 0; n_pkt_rdma = 0; n_post = 0; b_seen = 0;
    n_store = 0; n_store_drop = 0; n_full = 0; n_part = 0;
    for (int i = 0; i < N_DBG; i++) n_dbg[i] = 0;
    w_first = -1; w_beats = 0;
    salt = $urandom();
    model();

    b_expect_id = 0;
    foreach (bursts[k]) begin aw_q.push_back(k); w_q.push_back(k); end
    n_bursts = bursts.size();
    // done when every burst answered and the output has gone quiet
    wait (b_seen == n_bursts);
    repeat (8*FLUSH + 200) @(posedge aclk);
    win_shut = 0;
    // a packet held open by the shut window flushes once it opens
    do @(posedge aclk); while (dut.pk_open);
    repeat (4) @(posedge aclk);
    wait (dut_ost == 3'd0 && dut.pq_empty);
    repeat (20) @(posedge aclk);

    `CHECK(reqs.size() == exp_pkts.size(),
           $sformatf("%s: %0d requests, expected %0d", name, reqs.size(), exp_pkts.size()))
    foreach (exp_pkts[i]) begin
        pkt_t e = exp_pkts[i];
        req_t r;
        if (i >= reqs.size()) break;
        r = reqs[i];
        `CHECK(r.pid == e.pid && r.last == 1'b1, $sformatf("%s pkt %0d: pid %0d last %0d", name, i, r.pid, r.last))
        if (e.store && e.route) begin
            logic [AXI_DATA_BITS-1:0] h;
            `CHECK(r.opcode == RC_RDMA_WRITE_ONLY && r.strm == STRM_RDMA && r.mode == 1'b1 &&
                   r.dest == 0 && r.vaddr == {8'hFF, 40'd0} && r.len == 64,
                   $sformatf("%s pkt %0d: rdma store request op %0h va %h len %0d", name, i, r.opcode, r.vaddr, r.len))
            h = net_beats.pop_front();
            `CHECK(net_last.pop_front(), $sformatf("%s pkt %0d: no tlast on the inline message", name, i))
            `CHECK(h[63:0] == {28'd0, 28'd8, 8'd2} && h[127:64] == 64'(e.va) &&
                   h[191:128] == e.data && h[AXI_DATA_BITS-1:192] == '0,
                   $sformatf("%s pkt %0d: inline message %h %h %h, expected va %h data %h",
                             name, i, h[191:128], h[127:64], h[63:0], e.va, e.data))
            n_st++; n_st_rdma++;
        end else if (e.store) begin
            logic [AXI_DATA_BITS-1:0] d;
            `CHECK(r.opcode == LOCAL_WRITE && r.strm == STRM_HOST && r.dest == 0 &&
                   r.vaddr == e.va && r.len == 8,
                   $sformatf("%s pkt %0d: local store request va %h len %0d, expected va %h", name, i, r.vaddr, r.len, e.va))
            d = host_beats.pop_front();
            `CHECK(d[63:0] == e.data && host_keep.pop_front() == 64'hFF && host_last.pop_front(),
                   $sformatf("%s pkt %0d: local store beat %h, expected %h", name, i, d[63:0], e.data))
            n_st++;
        end else if (e.route) begin
            logic [AXI_DATA_BITS-1:0] h;
            // self-describing: the RETH is the binding's remote reference
            // plus the offset, the payload follows with no header
            `CHECK(r.opcode == RC_RDMA_WRITE_ONLY && r.strm == STRM_RDMA && r.mode == 1'b1 &&
                   r.rdma && r.remote && r.dest == 0 && r.vaddr == e.va &&
                   r.len == LEN_BITS'(64*e.beats),
                   $sformatf("%s pkt %0d: rdma request op %0h strm %0d dest %0d va %h len %0d, expected va %h len %0d",
                             name, i, r.opcode, r.strm, r.dest, r.vaddr, r.len, e.va, 64*e.beats))
            if (e.beats == PKT_BEATS) e_full++; else e_part++;
            for (int j = 0; j < e.beats; j++) begin
                `CHECK(net_beats.pop_front() == pat(e.uaddr + 64*j), $sformatf("%s pkt %0d: payload beat %0d", name, i, j))
                `CHECK(net_last.pop_front() == (j == e.beats - 1), $sformatf("%s pkt %0d: tlast at beat %0d", name, i, j))
            end
            n_rdma++;
        end else begin
            `CHECK(r.opcode == LOCAL_WRITE && r.strm == STRM_HOST && r.dest == 0 &&
                   r.vaddr == e.va && r.len == LEN_BITS'(64*e.beats),
                   $sformatf("%s pkt %0d: local request va %h len %0d, expected va %h len %0d",
                             name, i, r.vaddr, r.len, e.va, 64*e.beats))
            for (int j = 0; j < e.beats; j++) begin
                `CHECK(host_beats.pop_front() == pat(e.uaddr + 64*j), $sformatf("%s pkt %0d: payload beat %0d", name, i, j))
                `CHECK(host_last.pop_front() == (j == e.beats - 1), $sformatf("%s pkt %0d: tlast at beat %0d", name, i, j))
                `CHECK(host_keep.pop_front() == '1, $sformatf("%s pkt %0d: tkeep at beat %0d", name, i, j))
            end
            n_local++;
        end
    end
    `CHECK(host_beats.size() == 0 && net_beats.size() == 0,
           $sformatf("%s: %0d host / %0d net beats left over", name, host_beats.size(), net_beats.size()))
    `CHECK(n_drop == exp_drops && n_burst == n_bursts - exp_drops,
           $sformatf("%s: %0d bursts %0d drops, expected %0d / %0d", name, n_burst, n_drop, n_bursts - exp_drops, exp_drops))
    `CHECK(n_pkt_local == n_local && n_pkt_rdma == n_rdma && n_post == n_rdma + n_st_rdma,
           $sformatf("%s: counters local %0d rdma %0d post %0d", name, n_pkt_local, n_pkt_rdma, n_post))
    `CHECK(n_full == e_full && n_part == e_part,
           $sformatf("%s: rdma packets full %0d partial %0d, expected %0d / %0d", name, n_full, n_part, e_full, e_part))
    `CHECK(n_store == n_st && n_store_drop == exp_store_drops,
           $sformatf("%s: counters store %0d store_drop %0d, expected %0d / %0d",
                     name, n_store, n_store_drop, n_st, exp_store_drops))
    `CHECK(n_dbg[8] == exp_nowin && n_dbg[9] == exp_oob,
           $sformatf("%s: drops no window %0d past end %0d, expected %0d / %0d", name, n_dbg[8], n_dbg[9], exp_nowin, exp_oob))
    `CHECK(n_dbg[10] == exp_b1 && n_dbg[11] == exp_b4 && n_dbg[12] == exp_bmore && n_dbg[13] == exp_mis,
           $sformatf("%s: bursts 1 / 2-4 / >4 / misaligned %0d %0d %0d %0d, expected %0d %0d %0d %0d", name,
                     n_dbg[10], n_dbg[11], n_dbg[12], n_dbg[13], exp_b1, exp_b4, exp_bmore, exp_mis))
    if (!bp) `CHECK(n_dbg[0] == 0 && n_dbg[1] == 0, $sformatf("%s: output backpressure with none applied", name))
    // the first beat waits for its burst's lookup; after that, one a cycle
    if (check_rate && !bp)
        `CHECK(w_last - w_first + 1 == w_beats && n_dbg[3] <= 4,
               $sformatf("%s: %0d beats in %0d cycles, %0d cycles waiting on a lookup", name,
                         w_beats, w_last - w_first + 1, n_dbg[3]))
    check_rate = 0;
    $display("%s %s: %0d bursts, %0d local + %0d rdma packets, %0d stores, %0d drops, input %0d beats in %0d cycles",
             (errors == e0) ? "ok  " : "FAIL", name, n_bursts, n_local, n_rdma, n_st, exp_drops,
             w_beats, w_last - w_first + 1);
    bursts.delete();
endtask

function automatic void add(input longint uaddr, input int beats, input bit gap = 0);
    burst_t b;
    b.uaddr = uaddr; b.beats = beats; b.gap_after = gap; b.gap_held = 0;
    for (int j = 0; j < 64; j++) b.strb[j] = '1;
    bursts.push_back(b);
endfunction

// One beat with these strobes (a set of 8 B stores)
function automatic void add_st(input longint uaddr, input logic [63:0] strb, input bit gap = 0);
    add(uaddr, 1, gap);
    bursts[bursts.size()-1].strb[0] = strb;
endfunction

// 8 B word w's strobes
function automatic logic [63:0] word(input int w);
    return 64'hFF << (8*w);
endfunction

localparam logic [63:0] LO32 = 64'h0000_0000_FFFF_FFFF;   // a line's first 32 B
localparam logic [63:0] HI32 = 64'hFFFF_FFFF_0000_0000;   // its last 32 B

// Bursts of `per` beats covering `total` beats from uaddr
function automatic void add_run(input longint uaddr, input int total, input int per);
    for (int k = 0; k < total; k += per) add(uaddr + 64*k, (total - k < per) ? total - k : per);
endfunction

localparam longint W1 = 64'h0000_0000;   // local, 1 MB
localparam longint W2 = 64'h0010_0000;   // rdma, 1 MB
localparam longint W3 = 64'h0020_0000;   // local, 512 B
localparam longint W4 = 64'h0030_0000;   // programmed invalid
localparam longint W5 = 64'h0400_0000;   // rdma, 64 MiB, up to the uwin's end

task automatic suite(input string tag);
    // aligned: one full packet on each route
    add(W1, 64);
    add(W2, 63);
    run({tag, "aligned"});

    // a run starting mid-packet: 64 beats from offset 7*64 in 4-beat bursts
    add_run(W2 + 7*64, 64, 4);
    run({tag, "unaligned start"});

    // back to back over several packets on both routes
    add_run(W1 + 4096, 256, 4);
    add_run(W2 + 65536, 256, 4);
    add_run(W1 + 8192, 256, 8);
    run({tag, "back-to-back"});

    // 1-beat bursts, back to back: the lookup keeps up at one a cycle
    add_run(W1 + 64'hC000, 128, 1);
    add_run(W2 + 64'hC000, 126, 1);
    check_rate = 1;
    run({tag, "1-beat bursts"});

    // alternating windows: every switch closes the open packet
    for (int k = 0; k < 8; k++) begin
        add(W1 + 32768 + 256*k, 4);
        add(W2 + 32768 + 256*k, 4);
    end
    run({tag, "mixed windows"});

    // an offset gap inside one window, and an idle gap inside a run
    add(W1, 4); add(W1 + 1024, 4); add(W1 + 1280, 4, 1); add(W1 + 1536, 4);
    run({tag, "gaps"});

    // a run that pauses past the timer while the window is shut stays one
    // packet: the timer closes a packet only when it could leave at once.
    // (Closing on every pause under backpressure split 16 MiB copies into
    // ~11k packets.) The window opens after the last burst, which then
    // flushes the open packet.
    for (int k = 0; k < 8; k++) begin
        add(W2 + 64'hE000 + 256*k, 4);
        bursts[bursts.size()-1].gap_held = 1;
    end
    win_shut = 1;
    run({tag, "pauses behind a shut window"});

    // drops: past the end, unmapped, invalid entry; good ones around them
    add(W3, 8);                // exactly fills the 512 B window
    add(W3 + 64, 8);           // one beat past its end
    add(64'h0100_0000, 4);     // no window
    add(W3 + 448 + 32, 2);     // from line+32, its second line past the end
    bursts[bursts.size()-1].strb[0] = HI32;
    add_st(W3 + 512 + 8, word(1));   // the line after the window's end
    add(W4, 4);                // invalid entry
    add(W1 + 65536, 4);
    run({tag, "drops"});

    // a 64 MiB window: its last bytes are taken, one beat past is not
    add(W5, 4);
    add(W5 + 64'h400_0000 - 256, 4);
    add(W5 + 64'h400_0000 - 128, 4);
    run({tag, "64 MiB window"});

    // stores: lone words on both routes, several in one beat, an empty beat,
    // a partial word next to a whole one
    add_st(W1 + 64'h2000, word(3));
    add_st(W2 + 64'h2000, word(7));
    add_st(W1 + 64'h2040, word(0) | word(2) | word(5));
    add_st(W2 + 64'h2040, word(1) | word(6));
    add_st(W1 + 64'h2080, 64'h0);
    add_st(W1 + 64'h20C0, 64'h0F << 8 | word(4));
    add_st(W2 + 64'h20C0, 64'h01);
    run({tag, "stores"});

    // a flag behind bulk, on each route; a store inside a run closes the
    // packet before it; a whole line after a run joins its packet
    add_run(W2 + 64'h4000, 64, 4);
    add_st(W2 + 64'h8000, word(0));
    add_run(W1 + 64'h4000, 16, 4);
    add_st(W1 + 64'h8000, word(0));
    add(W1 + 64'hA000, 3);
    bursts[bursts.size()-1].strb[1] = word(7);
    add(W1 + 64'hB000, 4);
    add(W1 + 64'hB100, 1);
    run({tag, "flag behind bulk"});

    // writes that start inside a line, as a Zen 3 host sends them: each
    // write-combined line as two 32 B halves (line+0, then line+32), and 8 B
    // stores at their own address; a burst from line+32, whose first beat
    // is stores and the rest a packet; a half line inside a run closes its
    // packet; a half line ending at a window's end is taken
    for (int k = 0; k < 4; k++) begin
        add_st(W1 + 64'hD000 + 64*k,      LO32);
        add_st(W1 + 64'hD000 + 64*k + 32, HI32);
        add_st(W2 + 64'hD000 + 64*k,      LO32);
        add_st(W2 + 64'hD000 + 64*k + 32, HI32);
    end
    for (int w = 0; w < 8; w++) add_st(W1 + 64'hD400 + 8*w, word(w));
    add_st(W2 + 64'hD400 + 40, word(5));
    add(W1 + 64'hD800 + 32, 3);
    bursts[bursts.size()-1].strb[0] = HI32;
    add(W2 + 64'hD900 + 32, 5);
    bursts[bursts.size()-1].strb[0] = HI32;
    add_run(W1 + 64'hDC00, 8, 4);
    add_st(W1 + 64'hDC00 + 8*64 + 32, HI32);
    add_run(W1 + 64'hDC00 + 9*64, 4, 4);
    add_st(W3 + 512 - 32, HI32);
    run({tag, "inside a line"});
endtask

initial begin
    win_t w;
    repeat (10) @(posedge aclk);
    aresetn = 1;
    repeat (10) @(posedge aclk);

    w = '{valid: 1, route: 0, pid: 3, dst_pid: 0, base: 48'h1000_0000, len: 64'h10_0000, ustart: W1};
    program_win(1, w);
    // rdma bindings: base is the far export reference {index, offset}
    w = '{valid: 1, route: 1, pid: 5, dst_pid: 0, base: {8'd3, 40'h10_0000}, len: 64'h10_0000, ustart: W2};
    program_win(2, w);
    w = '{valid: 1, route: 0, pid: 4, dst_pid: 0, base: 48'h3000_0000, len: 512, ustart: W3};
    program_win(3, w);
    w = '{valid: 0, route: 0, pid: 4, dst_pid: 0, base: 48'h4000_0000, len: 64'h10_0000, ustart: W4};
    program_win(4, w);
    w = '{valid: 1, route: 1, pid: 6, dst_pid: 0, base: {8'd7, 40'h0}, len: 64'h400_0000, ustart: W5};
    program_win(5, w);

    bp = 0;
    suite("");
    bp = 1;
    suite("bp: ");
    bp = 0;

    if (errors == 0) $display("TB PASS (tb_loom_ingress)");
    else             $display("TB FAIL (tb_loom_ingress): %0d errors", errors);
    $finish;
end

initial begin
    #20ms;
    $display("TB FAIL (tb_loom_ingress): timeout");
    $finish;
end

endmodule
