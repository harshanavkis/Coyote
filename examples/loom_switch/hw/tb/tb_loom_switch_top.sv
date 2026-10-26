`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_loom_switch_top - the generated wrapper (design_user_logic_c0_0, which
 * includes vfpga_top.svh) with the TB as the shell: AXI-Lite on the CSR page,
 * AXI4 on the uwin, sq_wr / cq_wr, the host and rdma send streams, and
 * rq_wr + axis_rrsp_recv for incoming messages.
 *
 * Covers: window and export programming and CSR readback; bulk and stores
 * through the uwin on both routes, exact (an rdma packet is self-describing:
 * RETH = the binding's far export reference + offset, no header); the ack
 * window holding rdma packets and releasing them on acks; loom_rx landing
 * every packet on its own inside an export (inline stores, bulk packets,
 * writes posted ahead of their data, drops for no export and out of bounds,
 * orphan beats), under backpressure; the ingress and loom_rx racing for
 * sq_wr, with the arbitration invariants checked every cycle; a long copy
 * through the uwin replayed into loom_rx as the far stack delivers it,
 * landing byte-exact; the counters, including the shell's write path ones.
 * Gets: loom_rd answers an incoming get (host read, response packets,
 * completion), rejects bad ones with the error completion, stays under the
 * ack window, and shares the payload stream with the ingress in request
 * order. Reads on the uwin (loom_read): a read of an rdma window becomes a
 * get request; window 3 is bound to export 7 HERE, so the request is
 * replayed into loom_rx as the far side, loom_rd answers, and the answer is
 * replayed into loom_rx as the reader, which answers R: a CPU-sized load,
 * bursts in flight answered out of order (whole, unaligned, narrow, fixed),
 * reads that fail (all ones), a read after an open write packet, and all
 * slots in use, then wrapping, under backpressure. Reads that continue each
 * other share one get: a page of 256 B reads, the reads that must not join
 * (a gap, going back, the next page, the next window), the timer, a page
 * boundary, CPU loads inside lines, a shared get the far side rejects, and
 * the shared answer kept until its last read is answered. Read-ahead (T26-T33,
 * T35): streams answered from blocks, every drop, rejected blocks, the ring
 * wrapping, pages wanted after a drop, the ramp's bounds, no stream while old
 * blocks are out, read-ahead switched off mid-stream, the default start rule
 * (T37), no quiet time after a drop by a read (T38); T34 reads in flight
 * across the slot pointers' wrap.
 */
module tb_loom_switch_top;

logic aclk = 0;
logic aresetn = 0;
always #2 aclk = ~aclk;

AXI4L axi_ctrl (.aclk(aclk), .aresetn(aresetn));
AXI4  axi_udata (.aclk(aclk));
metaIntf #(.STYPE(irq_not_t)) notify (.*);
metaIntf #(.STYPE(req_t)) sq_rd (.*);
metaIntf #(.STYPE(req_t)) sq_wr (.*);
metaIntf #(.STYPE(ack_t)) cq_rd (.*);
metaIntf #(.STYPE(ack_t)) cq_wr (.*);
metaIntf #(.STYPE(req_t)) rq_rd (.*);
metaIntf #(.STYPE(req_t)) rq_wr (.*);
AXI4SR axis_host_recv [N_STRM_AXI] (.*);
AXI4SR axis_host_send [N_STRM_AXI] (.*);
AXI4SR axis_rreq_recv [N_RDMA_AXI] (.*);
AXI4SR axis_rreq_send [N_RDMA_AXI] (.*);
AXI4SR axis_rrsp_recv [N_RDMA_AXI] (.*);
AXI4SR axis_rrsp_send [N_RDMA_AXI] (.*);

// The shell's host DMA boundary pulses (dynamic_top dbg_host_out), driven by T9
logic [16:0] dbg_host_out = '0;

design_user_logic_c0_0 inst_dut (
    .axi_ctrl(axi_ctrl), .dbg_host_out(dbg_host_out), .axi_udata(axi_udata), .notify(notify),
    .sq_rd(sq_rd), .sq_wr(sq_wr), .cq_rd(cq_rd), .cq_wr(cq_wr),
    .rq_rd(rq_rd), .rq_wr(rq_wr),
    .axis_host_recv(axis_host_recv), .axis_host_send(axis_host_send),
    .axis_rreq_recv(axis_rreq_recv), .axis_rreq_send(axis_rreq_send),
    .axis_rrsp_recv(axis_rrsp_recv), .axis_rrsp_send(axis_rrsp_send),
    .aclk(aclk), .aresetn(aresetn)
);

int errors = 0;
`define CHECK(c, msg) if (!(c)) begin errors++; $display("FAIL [%0t] %s", $time, msg); end

localparam logic [47:0] STAGING = 48'h7d24_8ca0_0000;
localparam logic [47:0] B1 = 48'h7f10_0000_0000;   // window 1: local, pid 3
localparam logic [47:0] B2 = {8'd2, 40'h0};        // window 2: rdma, QP pid 5, far export 2
// This host's exports: 2 (pid 9, 1 MiB) and 7 (pid 7, 4 MiB)
localparam logic [47:0] X2 = 48'h7e20_0000_0000;
localparam logic [47:0] X7 = 48'h7e70_0000_0000;
localparam longint      U1 = 64'h000_0000;
localparam longint      U2 = 64'h010_0000;
localparam longint      U3 = 64'h020_0000;            // window 3: rdma, QP pid 5, export 7 HERE at 0x3000
localparam logic [47:0] R3 = {8'd7, 40'h3000};
localparam longint      U4 = 64'h030_0000;            // window 4: rdma, export 5 here (not programmed)
localparam logic [47:0] R4 = {8'd5, 40'h0};
localparam longint      U5 = 64'h040_0000;            // windows 5 and 6: rdma, export 7 here, 2 KiB each,
localparam logic [47:0] R5 = {8'd7, 40'h10_0000};     // meeting in the middle of one page
localparam longint      U6 = 64'h040_0800;
localparam logic [47:0] R6 = {8'd7, 40'h20_0000};
localparam logic [63:0] RA_EARLY = 64'h200;         // RD_CTL[9]: a stream starts on the second consecutive read
localparam logic [63:0] RA_FIRST = RA_EARLY | (64'd2 << 24);   // ... and 2 pages ahead at the start: the first build
localparam longint      U7 = 64'h050_0000;            // window 7: rdma, export 7 here at 0, 1 MiB (read-ahead)
localparam logic [47:0] R7 = {8'd7, 40'h0};

function automatic logic [AXI_DATA_BITS-1:0] pat(input longint uaddr);
    logic [AXI_DATA_BITS-1:0] d;
    for (int l = 0; l < 8; l++) d[64*l +: 64] = {32'hC0DE_0000 + 32'(l), 32'(uaddr)};
    return d;
endfunction
function automatic logic [63:0] word(input int w);
    return 64'hFF << (8*w);
endfunction
// What the host-read mock returns for a read at addr
function automatic logic [AXI_DATA_BITS-1:0] hpat(input logic [47:0] addr);
    logic [AXI_DATA_BITS-1:0] d;
    for (int l = 0; l < 8; l++) d[64*l +: 64] = {32'hBEEF_0000 + 32'(l), 32'(addr)};
    return d;
endfunction
// A get request word: len bytes (a multiple of 64) back to reference ret
function automatic logic [63:0] gword(input int len, input logic [47:0] ret);
    return {16'(len / 64), ret};
endfunction

// ---------------------------------------------------------------------------
// Shell mocks
// ---------------------------------------------------------------------------
bit bp = 0;            // random backpressure on sq_wr and the send streams
bit acks_on = 1;       // the far side acknowledges rdma packets
int acks_owed = 0;

initial begin
    sq_rd.ready = 1;
    cq_rd.valid = 0; cq_rd.data = '0;
    rq_rd.valid = 0; rq_rd.data = '0;
    rq_wr.valid = 0; rq_wr.data = '0;
    axis_host_recv[0].tvalid = 0; axis_host_recv[0].tdata = '0; axis_host_recv[0].tkeep = '0;
    axis_host_recv[0].tlast = 0; axis_host_recv[0].tid = '0;
    axis_host_recv[1].tvalid = 0; axis_host_recv[1].tdata = '0; axis_host_recv[1].tkeep = '0;
    axis_host_recv[1].tlast = 0; axis_host_recv[1].tid = '0;
    axis_rreq_recv[0].tvalid = 0; axis_rreq_recv[0].tdata = '0; axis_rreq_recv[0].tkeep = '0;
    axis_rreq_recv[0].tlast = 0; axis_rreq_recv[0].tid = '0;
    axis_rrsp_recv[0].tvalid = 0; axis_rrsp_recv[0].tdata = '0; axis_rrsp_recv[0].tkeep = '0;
    axis_rrsp_recv[0].tlast = 0; axis_rrsp_recv[0].tid = '0;
    axis_rrsp_send[0].tready = 1;
    notify.ready = 1;
end

always @(posedge aclk) begin
    axi_udata.rready        <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
    sq_wr.ready             <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
    sq_rd.ready             <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
    axis_host_send[0].tready <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
    axis_host_send[1].tready <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
    axis_rreq_send[0].tready <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
end

// Acks: one per rdma request, some cycles later, while acks_on. While
// lcmp_on (T25) also one local completion per landing write (dest 1)
bit lcmp_on = 0;
int lcmp_owed = 0;
always @(posedge aclk) begin
    cq_wr.valid <= 1'b0;
    cq_wr.data  <= '0;
    if (sq_wr.valid && sq_wr.ready && sq_wr.data.strm == STRM_RDMA) acks_owed++;
    if (lcmp_on && sq_wr.valid && sq_wr.ready && sq_wr.data.strm == STRM_HOST && sq_wr.data.dest == 1) lcmp_owed++;
    if (acks_on && acks_owed > 0 && $urandom_range(0, 3) == 0) begin
        cq_wr.valid       <= 1'b1;
        cq_wr.data.remote <= 1'b1;
        acks_owed--;
    end else if (lcmp_owed > 0 && $urandom_range(0, 3) == 0) begin
        cq_wr.valid     <= 1'b1;
        cq_wr.data.dest <= 1;
        lcmp_owed--;
    end
end

// ---------------------------------------------------------------------------
// Capture
// ---------------------------------------------------------------------------
req_t wr_ing [$], wr_rx [$], wr_rd [$];   // sq_wr handshakes, by producer
typedef struct { bit rd; int beats; logic [47:0] va; } netreq_t;
netreq_t net_req [$];                      // rdma requests in the order taken
req_t rd_log [$];                          // sq_rd handshakes
logic [AXI_DATA_BITS-1:0]   h0_d [$], h1_d [$], n_d [$], r_d [$];
logic [AXI_DATA_BITS/8-1:0] h0_k [$];
bit                         h0_l [$], h1_l [$], n_l [$], r_l [$];
int                         r_id [$];
logic [1:0]                 r_rs [$];

always @(posedge aclk) if (aresetn) begin
    if (sq_wr.valid && sq_wr.ready) begin
        if (inst_dut.rx_takes_wr)  wr_rx.push_back(sq_wr.data);
        else if (inst_dut.sel_rd)  wr_rd.push_back(sq_wr.data);
        else                       wr_ing.push_back(sq_wr.data);
        if (!inst_dut.rx_takes_wr && sq_wr.data.strm == STRM_RDMA) begin
            netreq_t nr;
            nr.rd    = inst_dut.sel_rd;
            nr.beats = int'(sq_wr.data.len + 63) / 64;
            nr.va    = sq_wr.data.vaddr;
            net_req.push_back(nr);
        end
    end
    if (sq_rd.valid && sq_rd.ready) rd_log.push_back(sq_rd.data);
    if (axis_host_send[0].tvalid && axis_host_send[0].tready) begin
        h0_d.push_back(axis_host_send[0].tdata); h0_k.push_back(axis_host_send[0].tkeep);
        h0_l.push_back(axis_host_send[0].tlast);
    end
    if (axis_host_send[1].tvalid && axis_host_send[1].tready) begin
        h1_d.push_back(axis_host_send[1].tdata); h1_l.push_back(axis_host_send[1].tlast);
    end
    if (axis_rreq_send[0].tvalid && axis_rreq_send[0].tready) begin
        n_d.push_back(axis_rreq_send[0].tdata); n_l.push_back(axis_rreq_send[0].tlast);
    end
    if (axi_udata.rvalid && axi_udata.rready) begin
        r_d.push_back(axi_udata.rdata); r_l.push_back(axi_udata.rlast);
        r_id.push_back(int'(axi_udata.rid)); r_rs.push_back(axi_udata.rresp);
    end
end

// loom_read: the cycle of the last AR taken and of the last get request
// handed to the ingress, the get requests so far, and the head slot pointer
// when the last AR was taken
longint cyc = 0, t_ar = 0, t_get = 0;
int     n_get = 0;
logic [6:0] rp_at_ar;
always @(posedge aclk) begin
    cyc++;
    if (axi_udata.arvalid && axi_udata.arready) begin t_ar = cyc; rp_at_ar = inst_dut.inst_loom_read.rp; end
    if (inst_dut.get_valid && inst_dut.get_ready) begin t_get = cyc; n_get++; end
end
int CO;     // loom_read's timer
initial CO = inst_dut.inst_loom_read.CO_WAIT;

// Arbitration invariants: sq_wr carries loom_rx's request whenever it has
// one, else the ingress's or loom_rd's (whichever is selected, and it has
// one), and never a request nobody presented
always @(posedge aclk) if (aresetn) begin
    if (sq_wr.valid) begin
        `CHECK(inst_dut.rx_wr_valid || inst_dut.ing_wr_valid || inst_dut.rd_wr_valid, "sq_wr valid with no producer")
        if (inst_dut.rx_wr_valid) begin
            `CHECK(sq_wr.data === inst_dut.rx_wr_req, "rx had a request, sq_wr carried another")
        end else if (inst_dut.sel_rd) begin
            `CHECK(inst_dut.rd_wr_valid && sq_wr.data === inst_dut.rd_wr_req, "loom_rd selected, sq_wr carried another")
        end else begin
            `CHECK(inst_dut.ing_wr_valid && sq_wr.data === inst_dut.ing_wr_req, "sq_wr carried something other than the ingress's request")
        end
    end
    `CHECK(!(inst_dut.cnt_wr_blk_rx), "loom_rx waited its turn")
end

// Host-read mock: every sq_rd answered in order on axis_host_recv[1], len/64
// beats of hpat(vaddr + 64 j), with gaps under bp
req_t rdq [$];
always @(posedge aclk) if (aresetn && sq_rd.valid && sq_rd.ready) rdq.push_back(sq_rd.data);
initial begin
    forever begin
        @(negedge aclk);
        if (rdq.size() > 0) begin
            req_t r;
            int n;
            r = rdq[0];
            n = int'(r.len) / 64;
            for (int j = 0; j < n; j++) begin
                while (bp && $urandom_range(0, 2) == 0) @(negedge aclk);
                axis_host_recv[1].tdata = hpat(r.vaddr + 48'(64*j)); axis_host_recv[1].tkeep = '1;
                axis_host_recv[1].tlast = (j == n - 1); axis_host_recv[1].tvalid = 1;
                do @(posedge aclk); while (!axis_host_recv[1].tready);
                @(negedge aclk);
                axis_host_recv[1].tvalid = 0;
            end
            void'(rdq.pop_front());
        end
    end
end


// T24 (latency at zero load): the cycle of every handshake on the switch's
// edges while lat_on
bit lat_on = 0;
longint ev_aw [$], ev_w1 [$], ev_wl [$], ev_ing [$], ev_nb1 [$], ev_nbl [$], ev_rq [$], ev_rr1 [$], ev_rrl [$],
        ev_rx [$], ev_h1 [$], ev_ar [$], ev_sqrd [$], ev_hr [$], ev_rd [$], ev_r [$];
bit w_mid = 0, nb_mid = 0, rr_mid = 0;
always @(posedge aclk) if (aresetn && lat_on) begin
    if (axi_udata.awvalid && axi_udata.awready) ev_aw.push_back(cyc);
    if (axi_udata.wvalid && axi_udata.wready) begin
        if (!w_mid) ev_w1.push_back(cyc);
        if (axi_udata.wlast) ev_wl.push_back(cyc);
        w_mid = !axi_udata.wlast;
    end
    if (sq_wr.valid && sq_wr.ready) begin
        if (inst_dut.rx_takes_wr) ev_rx.push_back(cyc);
        else if (inst_dut.sel_rd) ev_rd.push_back(cyc);
        else                      ev_ing.push_back(cyc);
    end
    if (axis_rreq_send[0].tvalid && axis_rreq_send[0].tready) begin
        if (!nb_mid) ev_nb1.push_back(cyc);
        if (axis_rreq_send[0].tlast) ev_nbl.push_back(cyc);
        nb_mid = !axis_rreq_send[0].tlast;
    end
    if (rq_wr.valid && rq_wr.ready) ev_rq.push_back(cyc);
    if (axis_rrsp_recv[0].tvalid && axis_rrsp_recv[0].tready) begin
        if (!rr_mid) ev_rr1.push_back(cyc);
        if (axis_rrsp_recv[0].tlast) ev_rrl.push_back(cyc);
        rr_mid = !axis_rrsp_recv[0].tlast;
    end
    if (axis_host_send[1].tvalid && axis_host_send[1].tready) ev_h1.push_back(cyc);
    if (axi_udata.arvalid && axi_udata.arready) ev_ar.push_back(cyc);
    if (sq_rd.valid && sq_rd.ready) ev_sqrd.push_back(cyc);
    if (axis_host_recv[1].tvalid && axis_host_recv[1].tready) ev_hr.push_back(cyc);
    if (axi_udata.rvalid && axi_udata.rready) ev_r.push_back(cyc);
end
task automatic lat_clear();
    ev_aw = {}; ev_w1 = {}; ev_wl = {}; ev_ing = {}; ev_nb1 = {}; ev_nbl = {}; ev_rq = {}; ev_rr1 = {}; ev_rrl = {};
    ev_rx = {}; ev_h1 = {}; ev_ar = {}; ev_sqrd = {}; ev_hr = {}; ev_rd = {}; ev_r = {};
    w_mid = 0; nb_mid = 0; rr_mid = 0;
endtask
// one latency line: cycles (and ns at 250 MHz) from a to b; a missing event is an error
task automatic lat(input string what, input longint a [$], input int ia, input longint b [$], input int ib);
    longint d;
    `CHECK(a.size() > ia && b.size() > ib, $sformatf("T24 %s: event missing (%0d, %0d)", what, a.size(), b.size()))
    if (a.size() <= ia || b.size() <= ib) return;
    d = b[ib] - a[ia];
    `CHECK(d >= 0 && d < 2000, $sformatf("T24 %s: %0d cycles", what, d))
    $display("lat  %-58s %5d cycles %7.1f ns", what, d, 4.0 * d);
endtask

// ---------------------------------------------------------------------------
// AXI-Lite (CSR page) and AXI4 (uwin) masters
// ---------------------------------------------------------------------------
task automatic csr_wr(input int word, input logic [63:0] data);
    @(negedge aclk);
    axi_ctrl.awaddr = 64'(word * 8); axi_ctrl.awvalid = 1;
    axi_ctrl.wdata = data; axi_ctrl.wstrb = 8'hFF; axi_ctrl.wvalid = 1; axi_ctrl.bready = 1;
    do @(posedge aclk); while (!(axi_ctrl.awready && axi_ctrl.wready));
    @(negedge aclk);
    axi_ctrl.awvalid = 0; axi_ctrl.wvalid = 0;
    while (!axi_ctrl.bvalid) @(posedge aclk);
    @(negedge aclk);
endtask

task automatic csr_rd(input int word, output logic [63:0] data);
    @(negedge aclk);
    axi_ctrl.araddr = 64'(word * 8); axi_ctrl.arvalid = 1; axi_ctrl.rready = 1;
    do @(posedge aclk); while (!axi_ctrl.arready);
    @(negedge aclk);
    axi_ctrl.arvalid = 0;
    while (!axi_ctrl.rvalid) @(posedge aclk);
    data = axi_ctrl.rdata;
    @(negedge aclk);
endtask

task automatic program_win(input int idx, input bit route, input int pid, input int dst_pid,
                           input logic [47:0] base, input longint len, input longint ustart);
    csr_wr(0, 64'(idx));
    csr_wr(1, {62'b0, route, 1'b1});
    csr_wr(2, 64'(pid) | (64'(dst_pid) << 8));
    csr_wr(3, 64'(base));
    csr_wr(4, 64'(len));
    csr_wr(80, 64'(ustart));
    csr_wr(5, 64'd1);
endtask

semaphore uwin_lock = new(1);
int uwin_id = 0;
task automatic uwr(input longint uaddr, input int beats, input logic [63:0] strb = '1);
    uwin_lock.get();
    @(negedge aclk);
    axi_udata.awaddr = 64'h0800_0000 + uaddr; axi_udata.awlen = 8'(beats - 1);
    axi_udata.awid = 6'(uwin_id++); axi_udata.awvalid = 1;
    do @(posedge aclk); while (!axi_udata.awready);
    @(negedge aclk);
    axi_udata.awvalid = 0;
    for (int j = 0; j < beats; j++) begin
        axi_udata.wdata = pat(uaddr + 64*j); axi_udata.wstrb = strb;
        axi_udata.wlast = (j == beats - 1); axi_udata.wvalid = 1;
        do @(posedge aclk); while (!axi_udata.wready);
        @(negedge aclk);
    end
    axi_udata.wvalid = 0;
    while (!axi_udata.bvalid) @(posedge aclk);
    @(negedge aclk);
    uwin_lock.put();
endtask

initial begin
    axi_udata.awburst = 2'b01; axi_udata.awsize = 3'd6; axi_udata.awcache = 0; axi_udata.awlock = 0;
    axi_udata.awprot = 0; axi_udata.awqos = 0; axi_udata.awregion = 0; axi_udata.awvalid = 0;
    axi_udata.arvalid = 0; axi_udata.wvalid = 0; axi_udata.bready = 1;
    axi_udata.araddr = 0; axi_udata.arlen = 0; axi_udata.arid = 0; axi_udata.arburst = 1;
    axi_udata.arsize = 6; axi_udata.arcache = 0; axi_udata.arlock = 0; axi_udata.arprot = 0;
    axi_udata.arqos = 0; axi_udata.arregion = 0;
end

// ---------------------------------------------------------------------------
// Incoming messages: rq_wr per packet, then its beats on axis_rrsp_recv
// ---------------------------------------------------------------------------
semaphore rx_lock = new(1);
task automatic program_export(input int idx, input int pid, input logic [47:0] base, input longint len);
    csr_wr(176, 64'(idx));
    csr_wr(177, 64'd1);
    csr_wr(178, 64'(pid));
    csr_wr(179, 64'(base));
    csr_wr(180, 64'(len));
    csr_wr(181, 64'd1);
endtask

// One packet as the far stack delivers it: rq_wr {RETH address, payload
// length, last} first, then the beats
task automatic rx_packet(input int len, input logic [AXI_DATA_BITS-1:0] beats [$], input logic [47:0] reth);
    @(negedge aclk);
    rq_wr.data = '0; rq_wr.data.pid = 6'd2; rq_wr.data.vaddr = reth; rq_wr.data.len = len[LEN_BITS-1:0];
    rq_wr.data.last = 1;
    rq_wr.valid = 1;
    do @(posedge aclk); while (!rq_wr.ready);
    @(negedge aclk);
    rq_wr.valid = 0;
    foreach (beats[i]) begin
        axis_rrsp_recv[0].tdata = beats[i]; axis_rrsp_recv[0].tkeep = '1;
        axis_rrsp_recv[0].tlast = (i == beats.size() - 1); axis_rrsp_recv[0].tvalid = 1;
        do @(posedge aclk); while (!axis_rrsp_recv[0].tready);
        @(negedge aclk);
    end
    axis_rrsp_recv[0].tvalid = 0;
endtask

// An incoming get request: RETH {0xFF, 0}, one beat {op 3, len 8; the
// reference to read here {idx, off}; the request word}
task automatic rx_get(input int idx, input longint off, input logic [63:0] gw);
    logic [AXI_DATA_BITS-1:0] b [$];
    logic [AXI_DATA_BITS-1:0] h = '0;
    h[63:0] = {28'd0, 28'd8, 8'd3}; h[127:64] = {16'b0, 8'(idx), 40'(off)}; h[191:128] = gw;
    b.push_back(h);
    rx_lock.get();
    rx_packet(64, b, {8'hFF, 40'd0});
    rx_lock.put();
endtask

// The response to a get of len bytes from export base + soff, back to ret:
// the read, then wr_rd's packets (each its own beats in n_d), then the
// completion carrying cval
task automatic exp_get(input string name, input logic [47:0] base, input longint soff, input int len,
                       input int pid, input logic [47:0] ret, input logic [63:0] cval);
    int at = 0;
    req_t r;
    logic [AXI_DATA_BITS-1:0] h;
    `CHECK(rd_log.size() == 1, $sformatf("%s: %0d host reads", name, rd_log.size()))
    if (rd_log.size() != 0) begin
        r = rd_log.pop_front();
        `CHECK(r.opcode == LOCAL_READ && r.strm == STRM_HOST && r.dest == 1 && r.pid == pid &&
               r.vaddr == base + 48'(soff) && r.len == LEN_BITS'(len),
               $sformatf("%s: read op %0d dest %0d pid %0d va %h len %0d", name, r.opcode, r.dest, r.pid, r.vaddr, r.len))
    end
    while (at < len) begin
        int plen = (len - at > PMTU_BYTES) ? PMTU_BYTES : len - at;
        `CHECK(wr_rd.size() != 0, $sformatf("%s: response packet at %0d missing", name, at))
        if (wr_rd.size() == 0) return;
        r = wr_rd.pop_front();
        `CHECK(r.opcode == RC_RDMA_WRITE_ONLY && r.strm == STRM_RDMA && r.mode && r.dest == 0 && r.pid == 5 &&
               r.last && r.vaddr == ret + 48'(at) && r.len == LEN_BITS'(plen),
               $sformatf("%s: response at %0d: op %0d va %h len %0d pid %0d", name, at, r.opcode, r.vaddr, r.len, r.pid))
        for (int j = 0; j < plen / 64; j++)
            `CHECK(n_d.size() != 0 && n_d.pop_front() == hpat(base + 48'(soff + at + 64*j)) &&
                   n_l.pop_front() == (j == plen/64 - 1), $sformatf("%s: response beat %0d", name, at/64 + j))
        at += plen;
    end
    `CHECK(wr_rd.size() == 1, $sformatf("%s: %0d requests after the data, want the completion", name, wr_rd.size()))
    if (wr_rd.size() == 0) return;
    r = wr_rd.pop_front();
    `CHECK(r.opcode == RC_RDMA_WRITE_ONLY && r.vaddr == {8'hFF, 40'd0} && r.len == 64 && r.pid == 5,
           $sformatf("%s: completion request va %h len %0d", name, r.vaddr, r.len))
    h = n_d.pop_front(); void'(n_l.pop_front());
    `CHECK(h[63:0] == {28'd0, 28'd8, 8'd2} && h[127:64] == 64'(ret + 48'(len)) && h[191:128] == cval,
           $sformatf("%s: completion %h at %h, want %h at %h", name, h[191:128], h[127:64], cval, ret + 48'(len)))
endtask

// An inline store: RETH {0xFF, 0}, one beat {op 2, len 8; the word's
// reference {idx, off}; data}
task automatic rx_inline(input int idx, input longint off, input logic [63:0] data);
    logic [AXI_DATA_BITS-1:0] b [$];
    logic [AXI_DATA_BITS-1:0] h = '0;
    h[63:0] = {28'd0, 28'd8, 8'd2}; h[127:64] = {16'b0, 8'(idx), 40'(off)}; h[191:128] = data;
    b.push_back(h);
    rx_lock.get();
    rx_packet(64, b, {8'hFF, 40'd0});
    rx_lock.put();
endtask

// len bytes into export idx at off, in PMTU packets, each self-describing
// (RETH = {idx, off + its offset}); payload pat(0), pat(1), ...
task automatic rx_bulk(input int idx, input longint off, input int len);
    logic [AXI_DATA_BITS-1:0] b [$];
    int left = len, k = 0, at = 0;
    rx_lock.get();
    while (left > 0) begin
        int n = (left > PMTU_BYTES) ? PMTU_BYTES / 64 : left / 64;
        b.delete();
        for (int j = 0; j < n; j++) begin b.push_back(pat(64'(k))); k++; end
        rx_packet(64*n, b, {8'(idx), 40'(off + at)});
        at += 64*n; left -= 64*n;
    end
    rx_lock.put();
endtask

// The landing of rx_bulk(idx, off, len) into an export at base, pid: one
// write per packet, contiguous, last = 0; the payload in order, no tlast
task automatic exp_bulk(input string name, input logic [47:0] base, input longint off, input int len, input int pid);
    int at = 0;
    while (at < len) begin
        int plen = (len - at > PMTU_BYTES) ? PMTU_BYTES : len - at;
        req_t r;
        `CHECK(wr_rx.size() != 0, $sformatf("%s: write at %0d missing", name, at))
        if (wr_rx.size() == 0) return;
        r = wr_rx.pop_front();
        `CHECK(r.opcode == LOCAL_WRITE && r.strm == STRM_HOST && r.dest == 1 && r.pid == pid &&
               r.vaddr == base + 48'(off + at) && r.len == LEN_BITS'(plen) && !r.last,
               $sformatf("%s: write at %0d: va %h len %0d last %0d pid %0d", name, at, r.vaddr, r.len, r.last, r.pid))
        at += plen;
    end
    `CHECK(wr_rx.size() == 0, $sformatf("%s: %0d extra writes", name, wr_rx.size()))
    `CHECK(h1_d.size() == len / 64, $sformatf("%s: %0d beats landed, want %0d", name, h1_d.size(), len / 64))
    foreach (h1_d[j]) begin
        `CHECK(h1_d[j] == pat(64'(j)), $sformatf("%s: beat %0d", name, j))
        `CHECK(!h1_l[j], $sformatf("%s: tlast at beat %0d", name, j))
    end
endtask

task automatic quiesce();
    int idle = 0;
    while (idle < 64) begin
        @(posedge aclk);
        if (!sq_wr.valid && !inst_dut.inst_loom_rx.busy && inst_dut.inst_loom_ingress.pq_empty &&
            !inst_dut.inst_loom_rd.busy && rdq.size() == 0 && !sq_rd.valid &&
            int'(inst_dut.inst_loom_ingress.ostate) == 0 && !inst_dut.inst_loom_ingress.pk_open &&
            !axis_host_send[0].tvalid && !axis_host_send[1].tvalid && !axis_rreq_send[0].tvalid &&
            !inst_dut.inst_loom_read.busy && !axi_udata.rvalid)
            idle++;
        else idle = 0;
    end
endtask

// Everything captured but the R beats (the loopback keeps those)
task automatic clear_net();
    wr_ing.delete(); wr_rx.delete(); wr_rd.delete(); net_req.delete(); rd_log.delete();
    h0_d.delete(); h0_k.delete(); h0_l.delete(); h1_d.delete(); h1_l.delete(); n_d.delete(); n_l.delete();
endtask

task automatic clear();
    clear_net();
    r_d.delete(); r_l.delete(); r_id.delete(); r_rs.delete();
endtask

// Checks: the next ingress request and its beats
task automatic exp_local(input string name, input logic [47:0] va, input longint uaddr, input int beats);
    req_t r;
    `CHECK(wr_ing.size() != 0, {name, ": no ingress request"})
    if (wr_ing.size() == 0) return;
    r = wr_ing.pop_front();
    `CHECK(r.opcode == LOCAL_WRITE && r.strm == STRM_HOST && r.dest == 0 && r.pid == 3 &&
           r.vaddr == va && r.len == LEN_BITS'(64*beats),
           $sformatf("%s: request op %0d dest %0d pid %0d va %h len %0d", name, r.opcode, r.dest, r.pid, r.vaddr, r.len))
    for (int j = 0; j < beats; j++) begin
        `CHECK(h0_d.size() != 0 && h0_d.pop_front() == pat(uaddr + 64*j) && h0_l.pop_front() == (j == beats-1) &&
               h0_k.pop_front() == '1, $sformatf("%s: beat %0d", name, j))
    end
endtask

task automatic exp_rdma(input string name, input logic [47:0] va, input longint uaddr, input int beats);
    req_t r;
    logic [AXI_DATA_BITS-1:0] h;
    `CHECK(wr_ing.size() != 0, {name, ": no ingress request"})
    if (wr_ing.size() == 0) return;
    r = wr_ing.pop_front();
    `CHECK(r.opcode == RC_RDMA_WRITE_ONLY && r.strm == STRM_RDMA && r.mode && r.dest == 0 && r.pid == 5 &&
           r.vaddr == va && r.len == LEN_BITS'(64*beats),
           $sformatf("%s: rdma request op %0d va %h len %0d", name, r.opcode, r.vaddr, r.len))
    for (int j = 0; j < beats; j++)
        `CHECK(n_d.size() != 0 && n_d.pop_front() == pat(uaddr + 64*j) && n_l.pop_front() == (j == beats-1),
               $sformatf("%s: beat %0d", name, j))
endtask

// ---------------------------------------------------------------------------
// Reads on the uwin
// ---------------------------------------------------------------------------
semaphore ar_lock = new(1);
task automatic uread(input longint uaddr, input int arlen, input int size = 6, input int burst = 1,
                     input int id = 0);
    ar_lock.get();
    @(negedge aclk);
    axi_udata.araddr = 64'h0800_0000 + uaddr; axi_udata.arlen = 8'(arlen);
    axi_udata.arsize = 3'(size); axi_udata.arburst = 2'(burst); axi_udata.arid = 6'(id);
    axi_udata.arvalid = 1;
    do @(posedge aclk); while (!axi_udata.arready);
    @(negedge aclk);
    axi_udata.arvalid = 0;
    ar_lock.put();
endtask

// The address of a read's beat i (AXI INCR or FIXED)
function automatic longint beat_addr(input longint a, input int i, input int size, input int burst);
    longint al = a & ~((64'd1 << size) - 1);
    if (burst == 0 || i == 0) return a;
    return al + (longint'(i) << size);
endfunction

// The R beats of a read at window offset woff: each the line its address
// falls in, as the far host reads it (hpat at the window's far address
// fbase + line; window 3: export 7 + 0x3000), or all ones for a read that
// fails
task automatic exp_read(input string name, input longint woff, input int arlen, input int size = 6,
                        input int burst = 1, input int id = 0, input bit fail = 0,
                        input logic [47:0] fbase = X7 + 48'h3000);
    for (int i = 0; i <= arlen; i++) begin
        longint line = beat_addr(woff, i, size, burst) & ~64'h3F;
        logic [AXI_DATA_BITS-1:0] want;
        want = fail ? '1 : hpat(fbase + 48'(line));
        `CHECK(r_d.size() != 0, $sformatf("%s: R beat %0d missing", name, i))
        if (r_d.size() == 0) return;
        `CHECK(r_d.pop_front() == want && r_id.pop_front() == id && r_l.pop_front() == (i == arlen) &&
               r_rs.pop_front() == 2'b00, $sformatf("%s: R beat %0d (line %h)", name, i, line))
    end
endtask

// Read-ahead: end any stream (a read where no window is: answered all ones
// here, nothing on the network), then wait out the quiet time after a drop
task automatic ra_settle();
    uread(64'h070_0000, 0, 6, 1, 63);
    repeat (800) @(posedge aclk);
    clear();
endtask

// The far side and the way back, by loopback: the get requests sent so far
// (other rdma packets' beats skipped) are replayed into loom_rx as the far
// side; loom_rd's answers, one packet per get (its lines, or a rejection's
// completion), are replayed into loom_rx as the reader, in order or last get
// first. The get request messages are kept in lg_reqs.
logic [AXI_DATA_BITS-1:0] lg_reqs [$];
int last_gets;      // loop_gets: the get requests it answered (want_gets < 0: any number)
task automatic loop_gets(input string name, input int want_gets, input bit reverse = 0);
    logic [AXI_DATA_BITS-1:0] reqs [$], beats [$], b [$];
    logic [47:0] pva [$];
    int plen [$], pfirst [$], gfirst [$], gcnt [$];
    quiesce();
    foreach (net_req[i]) begin
        for (int j = 0; j < net_req[i].beats; j++) begin
            logic [AXI_DATA_BITS-1:0] d;
            d = n_d.pop_front(); void'(n_l.pop_front());
            if (net_req[i].va == {8'hFF, 40'd0} && d[7:0] == 8'd3) reqs.push_back(d);
        end
    end
    if (want_gets >= 0)
        `CHECK(reqs.size() == want_gets, $sformatf("%s: %0d get requests out, want %0d", name, reqs.size(), want_gets))
    last_gets = reqs.size();
    lg_reqs = reqs;
    clear_net();
    rx_lock.get();
    foreach (reqs[i]) begin b.delete(); b.push_back(reqs[i]); rx_packet(64, b, {8'hFF, 40'd0}); end
    rx_lock.put();
    quiesce();
    foreach (net_req[i]) begin
        // one packet per get: its lines, or a rejection's completion
        `CHECK(net_req[i].rd, $sformatf("%s: a request other than an answer", name))
        gfirst.push_back(pva.size()); gcnt.push_back(1);
        pva.push_back(net_req[i].va); plen.push_back(net_req[i].beats * 64); pfirst.push_back(beats.size());
        for (int j = 0; j < net_req[i].beats; j++) begin beats.push_back(n_d.pop_front()); void'(n_l.pop_front()); end
    end
    `CHECK(gcnt.size() == reqs.size() && gfirst.size() == gcnt.size(),
           $sformatf("%s: %0d answers for %0d requests", name, gcnt.size(), reqs.size()))
    clear_net();
    rx_lock.get();
    for (int k = 0; k < gcnt.size(); k++) begin
        int g = reverse ? gcnt.size() - 1 - k : k;
        for (int p = gfirst[g]; p < gfirst[g] + gcnt[g]; p++) begin
            b.delete();
            for (int j = 0; j < plen[p] / 64; j++) b.push_back(beats[pfirst[p] + j]);
            rx_packet(plen[p], b, pva[p]);
        end
    end
    rx_lock.put();
    quiesce();
endtask

// Get request k of the last loop_gets: from far reference fref, lines lines,
// back to read slot slot (any slot if negative)
task automatic chk_get(input string name, input int k, input logic [47:0] fref, input int lines, input int slot = -1);
    `CHECK(lg_reqs.size() > k, $sformatf("%s: no get request %0d", name, k))
    if (lg_reqs.size() <= k) return;
    `CHECK(lg_reqs[k][127:64] == 64'(fref) && lg_reqs[k][191:176] == 16'(lines) && lg_reqs[k][175:168] == 8'hFE &&
           (slot < 0 || lg_reqs[k][167:128] == 40'(slot * 8192)),
           $sformatf("%s: get from %h, %0d lines, back to %h; want %h, %0d lines, slot %0d", name,
                     lg_reqs[k][127:64], lg_reqs[k][191:176], lg_reqs[k][167:128], fref, lines, slot))
endtask

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
logic [63:0] v, c0 [7], c1 [7], rxf0;
int n_local_t7;

initial begin
    axi_ctrl.awvalid = 0; axi_ctrl.wvalid = 0; axi_ctrl.arvalid = 0;
    axi_ctrl.bready = 0; axi_ctrl.rready = 0;
    axi_ctrl.awaddr = 0; axi_ctrl.wdata = 0; axi_ctrl.wstrb = 0; axi_ctrl.araddr = 0;
    repeat (5) @(negedge aclk);
    aresetn = 1;
    repeat (5) @(negedge aclk);

    // --- T0: programming and readback ---
    program_win(1, 0, 3, 0, B1, 64'h10_0000, U1);
    program_win(2, 1, 5, 9, B2, 64'h10_0000, U2);
    csr_wr(16, 64'(STAGING));
    csr_rd(80, v); `CHECK(v == U2, $sformatf("T0: TBL_USTART reads %h", v))
    csr_rd(66, v); `CHECK(v == 16, $sformatf("T0: TX_CTL reset %0d", v))
    csr_rd(76, v); `CHECK(v == 1, $sformatf("T0: RX_CHUNK reset %0d", v))
    csr_rd(16, v); `CHECK(v == 64'(STAGING), "T0: staging VA readback")
    program_export(2, 9, X2, 64'h10_0000);
    program_export(7, 7, X7, 64'h40_0000);
    csr_rd(179, v); `CHECK(v == 64'(X7), $sformatf("T0: EXP_BASE reads %h", v))
    $display("ok   T0 programming");
    for (int i = 0; i < 7; i++) csr_rd(88 + i, c0[i]);

    // --- T1/T2: bulk through the uwin, both routes ---
    uwr(U1 + 64'h100, 16);
    uwr(U2 + 64'h40, 8);
    quiesce();
    exp_local("T1 local bulk", B1 + 48'h100, U1 + 64'h100, 16);
    exp_rdma("T2 rdma bulk", B2 + 48'h40, U2 + 64'h40, 8);
    `CHECK(wr_ing.size() == 0 && h0_d.size() == 0 && n_d.size() == 0, "T1/T2: extra traffic")
    $display("ok   T1/T2 bulk");
    clear();

    // --- T3: stores through the uwin ---
    uwr(U1 + 64'h200, 1, word(2));
    uwr(U2 + 64'h200, 1, word(0));
    quiesce();
    begin
        req_t r; logic [AXI_DATA_BITS-1:0] h, p1, p2;
        p1 = pat(U1 + 64'h200); p2 = pat(U2 + 64'h200);
        r = wr_ing.pop_front();
        `CHECK(r.opcode == LOCAL_WRITE && r.dest == 0 && r.vaddr == B1 + 48'h210 && r.len == 8,
               $sformatf("T3 local store: va %h len %0d", r.vaddr, r.len))
        h = h0_d.pop_front();
        `CHECK(h[63:0] == p1[191:128] && h0_k.pop_front() == 64'hFF, "T3 local store: data")
        r = wr_ing.pop_front();
        `CHECK(r.opcode == RC_RDMA_WRITE_ONLY && r.len == 64 && r.vaddr == {8'hFF, 40'd0}, "T3 rdma store: request")
        h = n_d.pop_front();
        `CHECK(h[63:0] == {28'd0, 28'd8, 8'd2} && h[127:64] == 64'(B2 + 48'h200) &&
               h[191:128] == p2[63:0], "T3 rdma store: inline message")
    end
    $display("ok   T3 stores");
    clear();

    // --- T4: the ack window holds rdma packets until acks return ---
    csr_wr(66, 64'd2);
    acks_on = 0;
    for (int k = 0; k < 4; k++) uwr(U2 + 64'h1000 * (k + 1), 1);
    repeat (300) @(posedge aclk);
    `CHECK(wr_ing.size() == 2, $sformatf("T4: %0d rdma packets posted with the window at 2 and no acks", wr_ing.size()))
    csr_rd(67, v); `CHECK(v == 2, $sformatf("T4: TX_STATE %0d while shut", v))
    csr_rd(69, v); `CHECK(v > 0, "T4: TX_WINFULL did not count")
    acks_on = 1;
    quiesce();
    repeat (100) @(posedge aclk);
    `CHECK(wr_ing.size() == 4, $sformatf("T4: %0d rdma packets after the acks", wr_ing.size()))
    csr_rd(67, v); `CHECK(v == 0, $sformatf("T4: TX_STATE %0d after the acks", v))
    csr_wr(66, 64'd16);
    $display("ok   T4 ack window");
    clear();

    // --- T5: loom_rx lands an inline message on host stream 1 ---
    rx_inline(7, 64'h1008, 64'hFEED_F00D);
    quiesce();
    `CHECK(wr_rx.size() == 1 && wr_rx[0].opcode == LOCAL_WRITE && wr_rx[0].pid == 7 && wr_rx[0].last &&
           wr_rx[0].vaddr == X7 + 48'h1008 && wr_rx[0].len == 8, "T5: rx landing request")
    `CHECK(h1_d.size() == 1 && h1_d[0][63:0] == 64'hFEED_F00D, "T5: rx landing data")
    $display("ok   T5 rx inline");
    clear();

    // --- T6: bulk packets land on their own: 9 KiB into export 7 at 0x2000
    //     (4096 + 4096 + 1024), each packet's write at its own RETH ---
    rx_bulk(7, 64'h2000, 4096 + 4096 + 1024);
    quiesce();
    exp_bulk("T6 bulk", X7, 64'h2000, 4096 + 4096 + 1024, 7);
    clear();
    bp = 1;
    $assertoff(0, tb_loom_switch_top.sq_wr);
    rx_bulk(2, 64'h40, 4096 + 4096 + 1024);
    quiesce();
    bp = 0;
    $asserton(0, tb_loom_switch_top.sq_wr);
    exp_bulk("T6 backpressure", X2, 64'h40, 4096 + 4096 + 1024, 9);
    clear();
    $display("ok   T6 rx bulk packets");

    // --- T6b: writes are posted AHEAD of the data: with the host stream
    //     stalled, every packet of a 3-packet transfer has its write posted ---
    force axis_host_send[1].tready = 1'b0;
    fork rx_bulk(7, 64'h8000, 4096 + 4096 + 1024); join_none
    repeat (3000) @(posedge aclk);
    `CHECK(wr_rx.size() == 3 && h1_d.size() == 0,
           $sformatf("T6b: %0d writes posted with the host stream stalled, %0d beats landed", wr_rx.size(), h1_d.size()))
    release axis_host_send[1].tready;
    wait fork;
    quiesce();
    exp_bulk("T6b", X7, 64'h8000, 4096 + 4096 + 1024, 7);
    clear();
    $display("ok   T6b rx writes posted ahead");

    // --- T6c: drops and orphans: a packet for an export that is not there,
    //     one past an export's end, an inline store out of bounds, two beats
    //     nothing announced; then good ones still land ---
    begin
        logic [63:0] d0, d1, o0, o1;
        csr_rd(41, d0); csr_rd(26, o0);
        rx_bulk(5, 64'h0, 1024);                         // export 5: not programmed
        rx_bulk(2, 64'h10_0000 - 512, 1024);             // past export 2's 1 MiB
        rx_inline(2, 64'h10_0000, 64'h1);                // the word after export 2
        @(negedge aclk);
        for (int j = 0; j < 2; j++) begin
            axis_rrsp_recv[0].tdata = pat(64'(1000 + j)); axis_rrsp_recv[0].tkeep = '1;
            axis_rrsp_recv[0].tlast = (j == 1); axis_rrsp_recv[0].tvalid = 1;
            do @(posedge aclk); while (!axis_rrsp_recv[0].tready);
            @(negedge aclk);
        end
        axis_rrsp_recv[0].tvalid = 0;
        repeat (100) @(posedge aclk);      // the orphans drain before the next announcement
        rx_inline(7, 64'h3000, 64'hABCD);
        rx_bulk(7, 64'h4000, 512);
        quiesce();
        csr_rd(41, d1); csr_rd(26, o1);
        `CHECK(d1 - d0 == 3, $sformatf("T6c: %0d packets dropped, want 3", d1 - d0))
        `CHECK(o1 - o0 == 2, $sformatf("T6c: %0d orphan beats", o1 - o0))
        `CHECK(wr_rx.size() == 2 && wr_rx[0].vaddr == X7 + 48'h3000 && wr_rx[0].len == 8 &&
               wr_rx[1].vaddr == X7 + 48'h4000 && wr_rx[1].len == 512,
               $sformatf("T6c: %0d writes after the drops", wr_rx.size()))
        `CHECK(h1_d.size() == 9 && h1_d[0][63:0] == 64'hABCD, $sformatf("T6c: %0d beats landed", h1_d.size()))
        clear();
    end
    $display("ok   T6c rx drops and orphans");

    // --- T7: ingress and loom_rx race for sq_wr under backpressure ---
    // With sq_wr.ready low, loom_rx's request replaces the ingress's one
    // already presented: the arbiter's known defect 2 (examples/loom
    // vfpga_top.svh), kept until the shell's per-branch request queues. The
    // interface's stability assertion flags exactly that, so it is off here;
    // what must hold instead is that neither producer loses a request.
    $assertoff(0, tb_loom_switch_top.sq_wr);
    csr_rd(36, rxf0);
    bp = 1;
    fork
        for (int k = 0; k < 32; k++) uwr(U1 + 64'h8000 + 256*k, 4);
        for (int k = 0; k < 16; k++) uwr(U2 + 64'h8000 + 64*k, 1, word(k % 8));
        for (int k = 0; k < 24; k++) rx_inline(7, 64'h10_0000 + 64'(8*k), 64'(k));
    join
    quiesce();
    bp = 0;
    repeat (50) @(posedge aclk);
    $asserton(0, tb_loom_switch_top.sq_wr);
    begin
        int n_local = 0, n_st = 0, beats = 0;
        foreach (wr_ing[i]) if (wr_ing[i].strm == STRM_HOST) begin n_local++; beats += wr_ing[i].len / 64; end
                            else n_st++;
        // 128 contiguous beats in at least two local packets (the store
        // bursts interleave and each one closes the packet being gathered);
        // 16 rdma stores
        n_local_t7 = n_local;
        `CHECK(n_local >= 2 && beats == 128 && n_st == 16,
               $sformatf("T7: ingress %0d local packets (%0d beats), %0d rdma stores", n_local, beats, n_st))
        `CHECK(h0_d.size() == 128 && n_d.size() == 16, $sformatf("T7: %0d host / %0d net beats", h0_d.size(), n_d.size()))
        for (int j = 0; j < 128; j++) `CHECK(h0_d[j] == pat(U1 + 64'h8000 + 64*j), $sformatf("T7: host beat %0d", j))
        `CHECK(wr_rx.size() == 24 && h1_d.size() == 24, $sformatf("T7: %0d rx landings", wr_rx.size()))
        for (int k = 0; k < 24 && k < wr_rx.size(); k++)
            `CHECK(wr_rx[k].vaddr == X7 + 48'h10_0000 + 48'(8*k) && h1_d[k][63:0] == 64'(k),
                   $sformatf("T7: rx landing %0d", k))
    end
    $display("ok   T7 ingress and rx racing for sq_wr");

    // --- T8: counters ---
    for (int i = 0; i < 7; i++) csr_rd(88 + i, c1[i]);
    // bursts: 2 + 2 + 4 + 32 + 16; local packets: 1 + T7's; rdma packets: 1 + 4;
    // stores: 2 + 16; no drops
    `CHECK(c1[0] - c0[0] == 56 && c1[1] == c0[1], $sformatf("T8: bursts %0d, dropped %0d", c1[0] - c0[0], c1[1] - c0[1]))
    `CHECK(c1[2] - c0[2] == 1 + n_local_t7 && c1[3] - c0[3] == 5, $sformatf("T8: packets local %0d rdma %0d", c1[2] - c0[2], c1[3] - c0[3]))
    `CHECK(c1[4] - c0[4] == 18 && c1[5] == c0[5], $sformatf("T8: stores %0d, partial %0d", c1[4] - c0[4], c1[5] - c0[5]))
    csr_rd(36, v); `CHECK(v - rxf0 == 24, $sformatf("T8: rx forwarded %0d in T7", v - rxf0))
    csr_rd(75, v); `CHECK(v == 0, "T8: WR_BLK_RX nonzero")
    $display("ok   T8 counters");

    // --- T9: the host DMA boundary counters (81-84): 10 cycles moving, then
    //     a 7-cycle and a 25-cycle engine stall, 4 cycles of request stall ---
    begin
        logic [63:0] h0 [4], h1 [4];
        for (int i = 0; i < 4; i++) csr_rd(81 + i, h0[i]);
        @(negedge aclk); dbg_host_out = 3'b001; repeat (10) @(negedge aclk);
        dbg_host_out = 3'b010; repeat (7) @(negedge aclk);
        dbg_host_out = 3'b001; repeat (2) @(negedge aclk);
        dbg_host_out = 3'b010; repeat (25) @(negedge aclk);
        dbg_host_out = 3'b100; repeat (4) @(negedge aclk);
        dbg_host_out = 3'b000; repeat (4) @(negedge aclk);
        for (int i = 0; i < 4; i++) csr_rd(81 + i, h1[i]);
        `CHECK(h1[0] - h0[0] == 12, $sformatf("T9: moved %0d, want 12", h1[0] - h0[0]))
        `CHECK(h1[1] - h0[1] == 32, $sformatf("T9: engine stall %0d, want 32", h1[1] - h0[1]))
        `CHECK(h1[2] == 25,          $sformatf("T9: longest engine stall %0d, want 25", h1[2]))
        `CHECK(h1[3] - h0[3] == 4,  $sformatf("T9: request stall %0d, want 4", h1[3] - h0[3]))
    end
    $display("ok   T9 host DMA boundary counters");

    // --- T9b: the shell's further pulses (dbg_host_out[16:3]) reach words
    //     122-135, each with its longest run at 154-167; one bit at a time
    //     checks the mapping: [9:3] -> 122-128, [15] -> 129, [16] -> 130,
    //     [14:10] (the MMU write FSM) -> 131-135 ---
    begin
        int map [14] = '{3, 4, 5, 6, 7, 8, 9, 15, 16, 10, 11, 12, 13, 14};
        logic [63:0] a0 [14], a1 [14], m [14];
        for (int i = 0; i < 14; i++) csr_rd(122 + i, a0[i]);
        for (int i = 0; i < 14; i++) begin
            @(negedge aclk); dbg_host_out = 17'(1) << map[i]; repeat (i + 1) @(negedge aclk);
            dbg_host_out = '0; repeat (2) @(negedge aclk);
        end
        repeat (4) @(negedge aclk);
        for (int i = 0; i < 14; i++) begin csr_rd(122 + i, a1[i]); csr_rd(154 + i, m[i]); end
        for (int i = 0; i < 14; i++)
            `CHECK(a1[i] - a0[i] == i + 1 && m[i] >= i + 1,
                   $sformatf("T9b: word %0d (bit %0d) moved %0d, longest %0d, want %0d", 122 + i, map[i], a1[i] - a0[i], m[i], i + 1))
    end
    $display("ok   T9b shell write-path counters");

    // --- T10: a long copy through the uwin on the rdma route (PCIe-sized
    //     4-beat bursts, 300 beats from window offset 0x20000), then its
    //     packets replayed into loom_rx exactly as the far stack delivers
    //     them (rq_wr = the packet's RETH and length, then its beats). Window
    //     2's reference is export 2 here, so the copy must land byte-exact at
    //     X2 + 0x20000, one write per packet, and the counters must agree ---
    begin
        req_t pk [$];
        logic [AXI_DATA_BITS-1:0] nd [$];
        int nbeats = 300;
        logic [63:0] p0, p1, f0, f1, c0x, c1x;
        csr_rd(112, p0); csr_rd(119, f0); csr_rd(121, c0x);
        clear();
        for (int k = 0; k < nbeats; k += 4) uwr(U2 + 64'h20000 + 64*k, 4);
        quiesce();
        `CHECK(wr_ing.size() == 5, $sformatf("T10: %0d rdma packets for 300 beats (want 4 x 64 + 44)", wr_ing.size()))
        foreach (wr_ing[i])
            `CHECK(wr_ing[i].opcode == RC_RDMA_WRITE_ONLY && wr_ing[i].last &&
                   wr_ing[i].vaddr == B2 + 48'h20000 + 48'(4096 * i) &&
                   wr_ing[i].len == ((i == 4) ? 44*64 : 4096),
                   $sformatf("T10: packet %0d op %0h len %0d va %h", i, wr_ing[i].opcode, wr_ing[i].len, wr_ing[i].vaddr))
        `CHECK(n_d.size() == nbeats, $sformatf("T10: %0d wire beats, want %0d (no headers)", n_d.size(), nbeats))
        foreach (wr_ing[i]) pk.push_back(wr_ing[i]);
        foreach (n_d[i]) nd.push_back(n_d[i]);
        clear();
        rx_lock.get();
        foreach (pk[i]) begin
            logic [AXI_DATA_BITS-1:0] b [$];
            b.delete();     // static in this block: one per packet
            for (int j = 0; j < pk[i].len / 64; j++) b.push_back(nd.pop_front());
            rx_packet(int'(pk[i].len), b, pk[i].vaddr);
        end
        rx_lock.put();
        quiesce();
        begin
            int off = 0;
            `CHECK(wr_rx.size() == 5, $sformatf("T10: %0d landing writes", wr_rx.size()))
            foreach (wr_rx[i]) begin
                int plen = (i == 4) ? 44*64 : 4096;
                `CHECK(wr_rx[i].vaddr == X2 + 48'h20000 + 48'(off) && wr_rx[i].len == plen &&
                       wr_rx[i].pid == 9 && !wr_rx[i].last,
                       $sformatf("T10: landing %0d va %h len %0d pid %0d last %0d", i, wr_rx[i].vaddr, wr_rx[i].len, wr_rx[i].pid, wr_rx[i].last))
                off += plen;
            end
            `CHECK(h1_d.size() == nbeats, $sformatf("T10: %0d beats landed", h1_d.size()))
            foreach (h1_d[j])
                `CHECK(h1_d[j] == pat(U2 + 64'h20000 + 64*j),
                       $sformatf("T10: landed beat %0d: addr %h want %h", j, h1_d[j][31:0], 32'(U2 + 64'h20000 + 64*j)))
        end
        csr_rd(112, p1); csr_rd(119, f1); csr_rd(121, c1x);
        `CHECK(p1 - p0 == 5, $sformatf("T10: rx landed %0d packets", p1 - p0))
        `CHECK(f1 - f0 == 4, $sformatf("T10: ingress counted %0d full packets", f1 - f0))
        csr_rd(118, v); `CHECK(v == 0, $sformatf("T10: %0d rq_wr lost to a full queue", v))
        clear();
    end
    $display("ok   T10 long copy, uwin to landing");

    // --- T11: a read of an rdma window is a get request: an 8 B load at
    //     window 3 offset 0x48 asks export 7 (window 3's far reference) for
    //     the line at 0x3040, back to read slot 0 (op 3; lane1 = the
    //     reference, lane2 = {1 line, {0xFE, slot 0}}); answered by loopback,
    //     R is that line ---
    begin
        logic [63:0] g0, g1, y0 [8], y1 [8];
        program_win(3, 1, 5, 0, R3, 64'h1_0000, U3);
        // T11-T25 test reads without read-ahead (RD_CTL[8]); T26-T33 with it
        csr_wr(184, 64'h105);
        clear();
        csr_rd(136, g0);
        for (int i = 0; i < 8; i++) csr_rd(192 + i, y0[i]);
        uread(U3 + 64'h48, 0, 3, 1, 9);
        quiesce();
        `CHECK(wr_ing.size() == 1 && wr_ing[0].opcode == RC_RDMA_WRITE_ONLY && wr_ing[0].vaddr == {8'hFF, 40'd0} &&
               wr_ing[0].len == 64 && wr_ing[0].pid == 5, $sformatf("T11: %0d ingress requests", wr_ing.size()))
        `CHECK(n_d.size() == 1 && n_d[0][63:0] == {28'd0, 28'd8, 8'd3} && n_d[0][127:64] == 64'(R3 + 48'h40) &&
               n_d[0][191:128] == {16'd1, 8'hFE, 40'd0}, "T11: get request message")
        `CHECK(r_d.size() == 0 && rd_log.size() == 0 && wr_rd.size() == 0, "T11: answered before the far side")
        csr_rd(136, g1); `CHECK(g1 - g0 == 1, $sformatf("T11: %0d get requests counted", g1 - g0))
        loop_gets("T11", 1);
        exp_read("T11", 64'h48, 0, 3, 1, 9);
        `CHECK(r_d.size() == 0, $sformatf("T11: %0d extra R beats", r_d.size()))
        for (int i = 0; i < 8; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[0] - y0[0] == 1 && y1[1] - y0[1] == 1 && y1[2] == y0[2] && y1[5] == y0[5] && y1[6] - y0[6] == 1,
               $sformatf("T11: reads %0d answered %0d failed %0d stray %0d lines %0d",
                         y1[0] - y0[0], y1[1] - y0[1], y1[2] - y0[2], y1[5] - y0[5], y1[6] - y0[6]))
        clear();
    end
    $display("ok   T11 a load is a get");

    // --- T12: loom_rd answers a get of 9 KiB from export 7 at 0x3000: one
    //     host read, packets of 4096 + 4096 + 1024 to the return reference,
    //     then the completion (the request word at ret + len) ---
    begin
        logic [63:0] c [8], cc [8], gw;
        logic [47:0] ret;
        csr_wr(184, 64'h105); csr_rd(184, v); `CHECK(v == 64'h105, $sformatf("T12: RD_CTL reads %0h", v))
        for (int i = 0; i < 8; i++) csr_rd(136 + i, c[i]);
        ret = {8'd2, 40'h8000};
        gw  = gword(9216, ret);
        rx_get(7, 64'h3000, gw);
        quiesce();
        exp_get("T12 get", X7, 64'h3000, 9216, 7, ret, gw);
        `CHECK(wr_rd.size() == 0 && wr_ing.size() == 0 && wr_rx.size() == 0 && n_d.size() == 0, "T12: extra traffic")
        for (int i = 0; i < 8; i++) csr_rd(136 + i, cc[i]);
        `CHECK(cc[2] - c[2] == 1 && cc[3] == c[3] && cc[4] - c[4] == 3 && cc[5] - c[5] == 1,
               $sformatf("T12: jobs %0d err %0d packets %0d completions %0d", cc[2] - c[2], cc[3] - c[3], cc[4] - c[4], cc[5] - c[5]))
        clear();
    end
    $display("ok   T12 get answered");

    // --- T12b: bad requests get the error completion only (all ones at
    //     ret + len) and no read: no export, past the export's end, len 0,
    //     misaligned source, misaligned return ---
    begin
        logic [63:0] e0, e1;
        logic [47:0] ret [5];
        int lens [5] = '{1024, 128, 0, 64, 64};
        ret[0] = {8'd2, 40'h9000}; ret[1] = {8'd2, 40'h9400}; ret[2] = {8'd2, 40'h9800};
        ret[3] = {8'd2, 40'h9C00}; ret[4] = {8'd2, 40'hA008};
        csr_rd(139, e0);
        rx_get(5, 64'h0,               gword(lens[0], ret[0]));
        rx_get(2, 64'h10_0000 - 64,    gword(lens[1], ret[1]));
        rx_get(7, 64'h3000,            gword(lens[2], ret[2]));
        rx_get(7, 64'h3008,            gword(lens[3], ret[3]));
        rx_get(7, 64'h3000,            gword(lens[4], ret[4]));
        quiesce();
        `CHECK(rd_log.size() == 0, $sformatf("T12b: %0d host reads for bad requests", rd_log.size()))
        `CHECK(wr_rd.size() == 5 && n_d.size() == 5, $sformatf("T12b: %0d completions", wr_rd.size()))
        for (int i = 0; i < 5 && i < n_d.size(); i++)
            `CHECK(wr_rd[i].vaddr == {8'hFF, 40'd0} && n_d[i][191:128] == '1 && n_d[i][127:64] == 64'(ret[i] + 48'(lens[i])),
                   $sformatf("T12b: completion %0d: %h at %h", i, n_d[i][191:128], n_d[i][127:64]))
        csr_rd(139, e1);
        `CHECK(e1 - e0 == 5, $sformatf("T12b: %0d error completions counted", e1 - e0))
        clear();
    end
    $display("ok   T12b bad gets");

    // --- T12c: responses are held by the ack window like any rdma packet ---
    begin
        logic [63:0] w0, w1, gw;
        gw = gword(4 * 4096, {8'd2, 40'h0});
        csr_wr(66, 64'd2);
        acks_on = 0;
        csr_rd(142, w0);
        rx_get(7, 64'h0, gw);
        repeat (3000) @(posedge aclk);
        `CHECK(wr_rd.size() == 2, $sformatf("T12c: %0d responses posted with the window at 2 and no acks", wr_rd.size()))
        csr_rd(142, w1); `CHECK(w1 > w0, "T12c: the wait did not count")
        acks_on = 1;
        quiesce();
        repeat (100) @(posedge aclk);
        exp_get("T12c", X7, 64'h0, 4 * 4096, 7, {8'd2, 40'h0}, gw);
        csr_wr(66, 64'd16);
        clear();
    end
    $display("ok   T12c responses under the ack window");

    // --- T12d: ingress rdma packets and get responses share sq_wr and the
    //     payload stream under backpressure: every request's beats follow
    //     it, in the order the requests were taken ---
    $assertoff(0, tb_loom_switch_top.sq_wr);
    bp = 1;
    fork
        for (int k = 0; k < 16; k++) uwr(U2 + 64'h4_0000 + 256*k, 4);
        for (int k = 0; k < 4; k++) rx_get(7, 64'h1_0000 * k, gword(4096 + 1024, {8'd2, 40'(64'h2_0000 * k)}));
    join
    quiesce();
    bp = 0;
    repeat (50) @(posedge aclk);
    $asserton(0, tb_loom_switch_top.sq_wr);
    begin
        int ing_beats = 0, rsp_beats = 0, cmps = 0, bad = 0;
        foreach (net_req[i]) begin
            for (int j = 0; j < net_req[i].beats; j++) begin
                logic [AXI_DATA_BITS-1:0] d;
                bit l;
                if (n_d.size() == 0) begin bad++; break; end
                d = n_d.pop_front(); l = n_l.pop_front();
                if (l != (j == net_req[i].beats - 1)) bad++;
                if (!net_req[i].rd) begin
                    if (d != pat(U2 + longint'(net_req[i].va - B2) + 64*j)) bad++;
                    ing_beats++;
                end else if (net_req[i].va == {8'hFF, 40'd0}) begin
                    cmps++;
                end else begin
                    longint k = longint'(net_req[i].va[39:0]) / 64'h2_0000;
                    longint at = longint'(net_req[i].va[39:0]) % 64'h2_0000;
                    if (d != hpat(X7 + 48'(64'h1_0000 * k + at + 64*j))) bad++;
                    rsp_beats++;
                end
            end
        end
        `CHECK(bad == 0 && ing_beats == 64 && rsp_beats == 4 * 80 && cmps == 4 && n_d.size() == 0,
               $sformatf("T12d: %0d bad beats; ingress %0d, responses %0d, completions %0d, %0d left over",
                         bad, ing_beats, rsp_beats, cmps, n_d.size()))
        clear();
    end
    $display("ok   T12d ingress and responses share the payload stream");

    // --- T13: reads in flight, as a copy engine issues them, answered out
    //     of order (last get first): R still answers in AR order, each beat
    //     the line its address falls in. A whole 4 KiB burst, a 64 B one, a
    //     burst starting mid-line (4 lines), a narrow one (8 B beats over 3
    //     lines), a fixed one (4 beats of one line) ---
    begin
        longint off [5] = '{64'h1000, 64'h40, 64'h2010, 64'h22F8, 64'h500};
        int     len [5] = '{63, 0, 3, 15, 3};
        int     sz  [5] = '{6, 6, 6, 3, 6};
        int     bu  [5] = '{1, 1, 1, 1, 0};
        clear();
        for (int k = 0; k < 5; k++) uread(U3 + off[k], len[k], sz[k], bu[k], 20 + k);
        loop_gets("T13", 5, 1);
        for (int k = 0; k < 5; k++) exp_read($sformatf("T13 read %0d", k), off[k], len[k], sz[k], bu[k], 20 + k);
        `CHECK(r_d.size() == 0, $sformatf("T13: %0d extra R beats", r_d.size()))
        clear();
    end
    $display("ok   T13 reads in flight, answered out of order");

    // --- T14: reads that fail are answered with all ones, and the reads
    //     behind them still complete: no window, a local window, lines past
    //     a window's end (window 4 is 0x1040 long), a burst crossing its 4 KiB
    //     page; and one the far side rejects (window 4: export 5 is not
    //     there) ---
    begin
        logic [63:0] y0 [8], y1 [8];
        program_win(4, 1, 5, 0, R4, 64'h1040, U4);
        clear();
        for (int i = 0; i < 8; i++) csr_rd(192 + i, y0[i]);
        uread(64'h070_0000, 0, 6, 1, 1);                 // no window
        uread(U1 + 64'h40, 1, 6, 1, 2);                  // local window 1
        uread(U4 + 64'h1000, 1, 6, 1, 3);                // its second line past the end
        uread(U3 + 64'h0FC0, 1, 6, 1, 4);                // crosses 0x1000
        uread(U4 + 64'h80, 0, 6, 1, 5);                  // the far side rejects it
        uread(U3 + 64'h80, 0, 6, 1, 6);                  // a good one behind them
        loop_gets("T14", 2);
        exp_read("T14 no window", 0, 0, 6, 1, 1, 1);
        exp_read("T14 local",     0, 1, 6, 1, 2, 1);
        exp_read("T14 past end",  0, 1, 6, 1, 3, 1);
        exp_read("T14 crossing",  0, 1, 6, 1, 4, 1);
        exp_read("T14 rejected",  0, 0, 6, 1, 5, 1);
        exp_read("T14 good",      64'h80, 0, 6, 1, 6);
        `CHECK(r_d.size() == 0, $sformatf("T14: %0d extra R beats", r_d.size()))
        for (int i = 0; i < 8; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[0] - y0[0] == 6 && y1[1] - y0[1] == 6 && y1[2] - y0[2] == 5 && y1[3] - y0[3] == 1 && y1[5] == y0[5],
               $sformatf("T14: reads %0d answered %0d failed %0d far %0d stray %0d",
                         y1[0] - y0[0], y1[1] - y0[1], y1[2] - y0[2], y1[3] - y0[3], y1[5] - y0[5]))
        clear();
    end
    $display("ok   T14 failed reads");

    // --- T15: a read goes out after the writes taken before it: 8 lines
    //     written to window 3, then read at once, while their packet is still
    //     open (the idle timer has not closed it): the read closes it, the
    //     packet leaves whole, then the get request. The read runs to the
    //     page's end, so its get leaves without waiting for a continuing read ---
    begin
        logic [63:0] c0x, c1x;
        clear();
        csr_rd(121, c0x);
        uwr(U3 + 64'h100, 8);
        uread(U3 + 64'h100, 59, 6, 1, 7);
        quiesce();
        `CHECK(net_req.size() == 2 && !net_req[0].rd && net_req[0].va == R3 + 48'h100 && net_req[0].beats == 8 &&
               net_req[1].va == {8'hFF, 40'd0} && net_req[1].beats == 1,
               $sformatf("T15: %0d requests on the wire, first %h x %0d", net_req.size(),
                         net_req.size() ? net_req[0].va : 48'd0, net_req.size() ? net_req[0].beats : 0))
        csr_rd(121, c1x); `CHECK(c1x - c0x == 1, $sformatf("T15: %0d packets cut", c1x - c0x))
        loop_gets("T15", 1);
        exp_read("T15", 64'h100, 59, 6, 1, 7);
        clear();
    end
    $display("ok   T15 a read after an open write packet");

    // --- T16: every slot in use, under backpressure: 64 one-line reads
    //     (every other line, so each is a get of its own) take all 64 slots,
    //     a 65th waits for one (counted), and gets it once the first 64 are
    //     answered; the ring wraps ---
    $assertoff(0, tb_loom_switch_top.sq_wr);
    bp = 1;
    begin
        logic [63:0] w0, w1;
        bit took = 0;
        clear();
        csr_rd(196, w0);
        for (int k = 0; k < 64; k++) uread(U3 + 64'h4000 + 128*k, 0, 6, 1, k);
        fork begin uread(U3 + 64'h8000, 0, 6, 1, 40); took = 1; end join_none
        repeat (300) @(posedge aclk);
        `CHECK(!took, "T16: a 65th read was taken with every slot in use")
        csr_rd(196, w1); `CHECK(w1 - w0 > 100, $sformatf("T16: %0d cycles waiting for a slot", w1 - w0))
        loop_gets("T16", 64);
        wait (took);
        for (int k = 0; k < 64; k++) exp_read($sformatf("T16 read %0d", k), 64'h4000 + 128*k, 0, 6, 1, k);
        loop_gets("T16 65th", 1);
        exp_read("T16 65th", 64'h8000, 0, 6, 1, 40);
        `CHECK(r_d.size() == 0, $sformatf("T16: %0d extra R beats", r_d.size()))
        clear();
    end
    bp = 0;
    repeat (50) @(posedge aclk);
    $asserton(0, tb_loom_switch_top.sq_wr);
    $display("ok   T16 every slot in use, then wrapping");

    // --- T17: reads that continue each other share one get: 16 x 256 B (a
    //     copy engine's reads) over one page of window 3 are one 4 KiB get
    //     back to the first read's slot, sent as soon as the page is done
    //     (before the timer); every read is answered from it, exact ---
    begin
        logic [63:0] g0, g1, y0, y1;
        int s0;
        clear();
        s0 = int'(inst_dut.inst_loom_read.wp[5:0]);
        csr_rd(136, g0); csr_rd(192, y0);
        for (int k = 0; k < 16; k++) uread(U3 + 64'h5000 + 256*k, 3, 6, 1, k);
        quiesce();
        `CHECK(t_get - t_ar < CO, $sformatf("T17: the get left %0d cycles after the last read", t_get - t_ar))
        csr_rd(136, g1); csr_rd(192, y1);
        `CHECK(g1 - g0 == 1 && y1 - y0 == 16, $sformatf("T17: %0d get requests for %0d reads", g1 - g0, y1 - y0))
        loop_gets("T17", 1);
        chk_get("T17", 0, R3 + 48'h5000, 64, s0);
        for (int k = 0; k < 16; k++) exp_read($sformatf("T17 read %0d", k), 64'h5000 + 256*k, 3, 6, 1, k);
        `CHECK(r_d.size() == 0, $sformatf("T17: %0d extra R beats", r_d.size()))
        clear();
    end
    $display("ok   T17 a page of reads, one get");

    // --- T18: a read that does not continue the open get starts its own: a
    //     gap of one line, going back, the same place in the next page, the
    //     next window inside the same page (windows 5 and 6 meet in the middle
    //     of a page); each get carries only its read's lines ---
    begin
        longint      off [6] = '{64'h6000, 64'h6140, 64'h6040, 64'h7140, 64'h700, 64'h0};
        longint      ua [6];
        logic [47:0] fr [6], fb [6];
        program_win(5, 1, 5, 0, R5, 64'h800, U5);
        program_win(6, 1, 5, 0, R6, 64'h800, U6);
        clear();
        for (int k = 0; k < 6; k++) begin
            ua[k] = (k < 4) ? U3 : (k == 4) ? U5 : U6;
            fr[k] = (k < 4) ? R3 : (k == 4) ? R5 : R6;
            fb[k] = (k < 4) ? X7 + 48'h3000 : (k == 4) ? X7 + 48'h10_0000 : X7 + 48'h20_0000;
            uread(ua[k] + off[k], 3, 6, 1, k);
        end
        loop_gets("T18", 6);
        for (int k = 0; k < 6; k++) chk_get($sformatf("T18 get %0d", k), k, fr[k] + 48'(off[k]), 4);
        for (int k = 0; k < 6; k++) exp_read($sformatf("T18 read %0d", k), off[k], 3, 6, 1, k, 0, fb[k]);
        `CHECK(r_d.size() == 0, $sformatf("T18: %0d extra R beats", r_d.size()))
        clear();
    end
    $display("ok   T18 reads that do not continue, a get each");

    // --- T19: an open get waits for a continuing read up to the timer: 3 x
    //     256 B, CO/2 cycles apart, are one get of 12 lines, sent only once
    //     the timer runs out after the third; a 4th read continuing them
    //     after that is a get of its own ---
    begin
        int n0;
        clear();
        n0 = n_get;
        for (int k = 0; k < 3; k++) begin
            uread(U3 + 64'h8000 + 256*k, 3, 6, 1, k);
            repeat (CO / 2) @(posedge aclk);
        end
        `CHECK(n_get == n0, "T19: the get left before the timer ran out")
        `CHECK(inst_dut.inst_loom_read.busy, "T19: not busy with a get open")
        quiesce();
        `CHECK(n_get == n0 + 1 && t_get - t_ar >= CO,
               $sformatf("T19: %0d gets, the last %0d cycles after the last read", n_get - n0, t_get - t_ar))
        uread(U3 + 64'h8300, 3, 6, 1, 3);
        loop_gets("T19", 2);
        chk_get("T19 shared", 0, R3 + 48'h8000, 12);
        chk_get("T19 after the timer", 1, R3 + 48'h8300, 4);
        for (int k = 0; k < 4; k++) exp_read($sformatf("T19 read %0d", k), 64'h8000 + 256*k, 3, 6, 1, k);
        `CHECK(r_d.size() == 0, $sformatf("T19: %0d extra R beats", r_d.size()))
        clear();
    end
    $display("ok   T19 the timer closes a get");

    // --- T20: a get stays inside its page: 24 x 256 B from 0x9800 run into
    //     the next page: a get of the first page's 8 reads (32 lines) and one
    //     of the next page's 16 (64 lines) ---
    begin
        clear();
        for (int k = 0; k < 24; k++) uread(U3 + 64'h9800 + 256*k, 3, 6, 1, k);
        loop_gets("T20", 2);
        chk_get("T20 first page", 0, R3 + 48'h9800, 32);
        chk_get("T20 next page", 1, R3 + 48'hA000, 64);
        for (int k = 0; k < 24; k++) exp_read($sformatf("T20 read %0d", k), 64'h9800 + 256*k, 3, 6, 1, k);
        `CHECK(r_d.size() == 0, $sformatf("T20: %0d extra R beats", r_d.size()))
        clear();
    end
    $display("ok   T20 a page boundary splits");

    // --- T21: CPU loads chain inside lines: 12 x 32 B loads (arsize 5) from
    //     0xB020, each where the last ended, then two 64 B loads (the first
    //     unaligned, 0xB1A0): one get of the 8 lines 0xB000-0xB1FF ---
    begin
        clear();
        for (int k = 0; k < 12; k++) uread(U3 + 64'hB020 + 32*k, 0, 5, 1, k);
        uread(U3 + 64'hB1A0, 0, 6, 1, 12);
        uread(U3 + 64'hB1C0, 0, 6, 1, 13);
        loop_gets("T21", 1);
        chk_get("T21", 0, R3 + 48'hB000, 8);
        for (int k = 0; k < 12; k++) exp_read($sformatf("T21 load %0d", k), 64'hB020 + 32*k, 0, 5, 1, k);
        exp_read("T21 load 12", 64'hB1A0, 0, 6, 1, 12);
        exp_read("T21 load 13", 64'hB1C0, 0, 6, 1, 13);
        `CHECK(r_d.size() == 0, $sformatf("T21: %0d extra R beats", r_d.size()))
        clear();
    end
    $display("ok   T21 CPU loads inside lines");

    // --- T22: the far side rejects a shared get (window 4: export 5 is not
    //     there): every read of it is answered all ones, each counted, the
    //     rejection once. A read that fails here (it continues the open get
    //     but runs past window 5's end) does not join it; good reads around
    //     them still complete ---
    begin
        logic [63:0] y0 [8], y1 [8];
        clear();
        for (int i = 0; i < 8; i++) csr_rd(192 + i, y0[i]);
        for (int k = 0; k < 4; k++) uread(U4 + 64'h400 + 256*k, 3, 6, 1, k);
        uread(U5 + 64'h700, 1, 6, 1, 4);
        uread(U5 + 64'h780, 2, 6, 1, 5);                 // 0x780-0x83F: past 0x800
        uread(U3 + 64'hB400, 0, 6, 1, 6);
        loop_gets("T22", 3);
        chk_get("T22 rejected", 0, R4 + 48'h400, 16);
        chk_get("T22 window 5", 1, R5 + 48'h700, 2);
        for (int k = 0; k < 4; k++) exp_read($sformatf("T22 read %0d", k), 0, 3, 6, 1, k, 1);
        exp_read("T22 window 5", 64'h700, 1, 6, 1, 4, 0, X7 + 48'h10_0000);
        exp_read("T22 past the end", 0, 2, 6, 1, 5, 1);
        exp_read("T22 good", 64'hB400, 0, 6, 1, 6);
        `CHECK(r_d.size() == 0, $sformatf("T22: %0d extra R beats", r_d.size()))
        for (int i = 0; i < 8; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[0] - y0[0] == 7 && y1[1] - y0[1] == 7 && y1[2] - y0[2] == 5 && y1[3] - y0[3] == 1 && y1[5] == y0[5],
               $sformatf("T22: reads %0d answered %0d failed %0d far %0d stray %0d",
                         y1[0] - y0[0], y1[1] - y0[1], y1[2] - y0[2], y1[3] - y0[3], y1[5] - y0[5]))
        clear();
    end
    $display("ok   T22 a shared get rejected");

    // --- T23: a shared get's lines stay in its first slot's buffer until its
    //     last read is answered: a 16-read get, then 48 one-line reads (every
    //     other line, a get each) take every slot; under backpressure the 65th
    //     read, which takes the shared get's first slot, is taken only once
    //     all 16 of its reads are answered ---
    $assertoff(0, tb_loom_switch_top.sq_wr);
    bp = 1;
    begin
        logic [6:0] wp0;
        bit took = 0;
        clear();
        wp0 = inst_dut.inst_loom_read.wp;
        for (int k = 0; k < 16; k++) uread(U3 + 64'hC000 + 256*k, 3, 6, 1, k);
        for (int k = 0; k < 48; k++) uread(U3 + 64'hD000 + 128*k, 0, 6, 1, 16 + k);
        fork begin uread(U3 + 64'hF000, 0, 6, 1, 0); took = 1; end join_none
        repeat (300) @(posedge aclk);
        `CHECK(!took, "T23: a 65th read was taken with every slot in use")
        loop_gets("T23", 49);
        wait (took);
        `CHECK(7'(rp_at_ar - wp0) >= 16,
               $sformatf("T23: the 65th read was taken with %0d of the shared get's 16 reads answered", 7'(rp_at_ar - wp0)))
        for (int k = 0; k < 16; k++) exp_read($sformatf("T23 shared %0d", k), 64'hC000 + 256*k, 3, 6, 1, k);
        for (int k = 0; k < 48; k++) exp_read($sformatf("T23 own %0d", k), 64'hD000 + 128*k, 0, 6, 1, 16 + k);
        loop_gets("T23 65th", 1);
        exp_read("T23 65th", 64'hF000, 0, 6, 1, 0);
        `CHECK(r_d.size() == 0, $sformatf("T23: %0d extra R beats", r_d.size()))
        clear();
    end
    bp = 0;
    repeat (50) @(posedge aclk);
    $asserton(0, tb_loom_switch_top.sq_wr);
    $display("ok   T23 a shared answer kept until its last read");

    // --- T24: latency at zero load, cycles through the switch's own logic
    //     (the edges: uwin AXI in, sq_wr + the rdma send stream out; rq_wr +
    //     the receive stream in, the landing write out; the host read and
    //     R for gets). Printed as "lat" lines for the simulator; the shell,
    //     the RoCE stack, PCIe and the wire are outside ---
    bp = 0; acks_on = 1;
    clear(); quiesce();
    // sender: a lone 8 B store, one 64 B line, 4 KiB as P2P hands it over (16 x 256 B)
    lat_clear(); lat_on = 1;
    uwr(U2 + 64'h3_0000, 1, word(1));
    quiesce(); lat_on = 0;
    lat("put out: 8 B store, W -> rdma request", ev_w1, 0, ev_ing, 0);
    lat("put out: 8 B store, W -> first beat on the send stream", ev_w1, 0, ev_nb1, 0);
    lat_clear(); lat_on = 1;
    uwr(U2 + 64'h3_1000, 1);
    quiesce(); lat_on = 0;
    lat("put out: 64 B line, W -> rdma request", ev_w1, 0, ev_ing, 0);
    lat("put out: 64 B line, W -> first beat on the send stream", ev_w1, 0, ev_nb1, 0);
    lat_clear(); lat_on = 1;
    for (int k = 0; k < 16; k++) uwr(U2 + 64'h3_2000 + 256*k, 4);
    quiesce(); lat_on = 0;
    `CHECK(ev_ing.size() == 1, $sformatf("T24: 4 KiB went out as %0d packets", ev_ing.size()))
    lat("put out: 4 KiB (16 x 256 B), first W -> rdma request", ev_w1, 0, ev_ing, 0);
    lat("put out: 4 KiB, first W -> first beat out", ev_w1, 0, ev_nb1, 0);
    lat("put out: 4 KiB, last W -> last beat out", ev_wl, 15, ev_nbl, 0);
    clear();
    // receiver: an inline store message, a 4 KiB packet
    lat_clear(); lat_on = 1;
    rx_inline(7, 64'h5008, 64'h1234);
    quiesce(); lat_on = 0;
    lat("put in: inline store, request in -> landing write", ev_rq, 0, ev_rx, 0);
    lat("put in: inline store, payload beat in -> landing beat", ev_rr1, 0, ev_h1, 0);
    lat_clear(); lat_on = 1;
    rx_bulk(7, 64'h6000, 4096);
    quiesce(); lat_on = 0;
    lat("put in: 4 KiB packet, request in -> landing write", ev_rq, 0, ev_rx, 0);
    lat("put in: 4 KiB, first beat in -> first landing beat", ev_rr1, 0, ev_h1, 0);
    lat("put in: 4 KiB, last beat in -> last landing beat", ev_rrl, 0, ev_h1, 63);
    clear();
    // gets: an 8 B load on window 3 (export 7 here, by loopback)
    lat_clear(); lat_on = 1;
    uread(U3 + 64'hA048, 0, 3, 1, 4);
    loop_gets("T24", 1);
    lat_on = 0;
    lat("get, reader: AR -> get request out (incl. the coalescer timer)", ev_ar, 0, ev_ing, 0);
    lat("get, reader: AR -> first beat of the request out", ev_ar, 0, ev_nb1, 0);
    lat("get, far side: request in -> host read", ev_rq, 0, ev_sqrd, 0);
    lat("get, far side: host data in -> answer request out", ev_hr, 0, ev_rd, 0);
    lat("get, far side: host data in -> first answer beat out", ev_hr, 0, ev_nb1, 1);
    lat("get, reader: answer in -> R", ev_rq, 1, ev_r, 0);
    exp_read("T24", 64'hA048, 0, 3, 1, 4);
    clear();
    $display("ok   T24 latency at zero load (lat lines)");

    // --- T25: stamps (stamp_rec, CSR 224-240): one stamp per point for an
    //     8 B store out and a 4 KiB packet in, the counts, and the stamps'
    //     differences equal to the cycles the testbench saw; a second arm
    //     clears the counts ---
    begin
        logic [63:0] v, n [9], s [9];
        bp = 0; acks_on = 1; lcmp_on = 1;
        clear(); quiesce();
        csr_wr(224, 64'd1);
        lat_clear(); lat_on = 1;
        uwr(U2 + 64'h3_0000, 1, word(1));
        quiesce();
        rx_bulk(7, 64'h6000, 4096);
        quiesce(); repeat (100) @(posedge aclk);
        lat_on = 0;
        csr_rd(224, v);
        `CHECK(v == {32'd4096, 32'd9}, $sformatf("T25: ST_CTL info %h", v))
        for (int p = 0; p < 9; p++) begin
            csr_rd(232 + p, n[p]);
            csr_wr(225, 64'(p) << 16);
            csr_rd(226, s[p]);
            `CHECK(n[p] == 1, $sformatf("T25: point %0d counted %0d events, want 1", p, n[p]))
            `CHECK(!$isunknown(s[p]), $sformatf("T25: point %0d stamp unknown", p))
        end
        `CHECK(s[1] - s[0] == 64'(ev_ing[0] - ev_w1[0]), $sformatf("T25: W -> post %0d, tb %0d", s[1] - s[0], ev_ing[0] - ev_w1[0]))
        `CHECK(s[2] - s[0] == 64'(ev_nb1[0] - ev_w1[0]), $sformatf("T25: W -> beat out %0d, tb %0d", s[2] - s[0], ev_nb1[0] - ev_w1[0]))
        // the mock acks a random 1-4 cycles after the request is posted (it
        // may be the cycle of the first beat): the ack comes after the post
        `CHECK(s[3] > s[1], $sformatf("T25: ack before the packet: post %0d, first beat %0d, ack %0d", s[1], s[2], s[3]))
        `CHECK(s[5] - s[4] == 64'(ev_rr1[0] - ev_rq[0]), $sformatf("T25: rq -> beat in %0d, tb %0d", s[5] - s[4], ev_rr1[0] - ev_rq[0]))
        `CHECK(s[6] - s[4] == 64'(ev_rx[0] - ev_rq[0]), $sformatf("T25: rq -> landing write %0d, tb %0d", s[6] - s[4], ev_rx[0] - ev_rq[0]))
        `CHECK(s[7] - s[4] == 64'(ev_h1[0] - ev_rq[0]), $sformatf("T25: rq -> landing beat %0d, tb %0d", s[7] - s[4], ev_h1[0] - ev_rq[0]))
        `CHECK(s[8] > s[6], "T25: landing completion before the landing write")
        `CHECK(s[4] - s[0] == 64'(ev_rq[0] - ev_w1[0]), $sformatf("T25: one clock for both %0d, tb %0d", s[4] - s[0], ev_rq[0] - ev_w1[0]))
        for (int p = 0; p < 9; p++) $display("st   point %0d: cycle %0d", p, s[p]);
        // a 16 KiB message (4 packets) and an inline store: one stamp per landing at points 6 and 7
        csr_wr(224, 64'd1);
        lat_clear(); lat_on = 1;
        rx_bulk(7, 64'h8000, 16384);
        rx_inline(7, 64'h5010, 64'h55);
        quiesce(); lat_on = 0;
        for (int p = 6; p <= 7; p++) begin
            csr_rd(232 + p, v);
            `CHECK(v == 5, $sformatf("T25: point %0d counted %0d landings, want 5", p, v))
        end
        for (int k = 0; k < 5; k++) begin
            logic [63:0] a, b;
            csr_wr(225, (64'd6 << 16) | 64'(k)); csr_rd(226, a);
            csr_wr(225, (64'd7 << 16) | 64'(k)); csr_rd(226, b);
            `CHECK(b - a == 64'(ev_h1[k == 4 ? 256 : 64*k] - ev_rx[k]),
                   $sformatf("T25: landing %0d write -> first beat %0d, tb %0d", k, b - a, ev_h1[k == 4 ? 256 : 64*k] - ev_rx[k]))
        end
        csr_wr(224, 64'd1);
        csr_rd(232, v);
        `CHECK(v == 0, $sformatf("T25: count after the second arm %0d", v))
        csr_rd(227, v);
        `CHECK(v > 0 && v < 100, $sformatf("T25: ST_NOW %0d after the second arm", v))
        lcmp_on = 0;
        clear();
    end
    $display("ok   T25 stamps");

    // --- T26-T33: read-ahead (DESIGN.md 17), on from here (RD_CTL[8] = 0);
    //     T26-T36 as the first build: streams start on the second
    //     consecutive read (RD_CTL[9]), 2 pages ahead; T37/T38 the defaults
    //     (a page of reads, then entering the next; 4 pages ahead).
    //     Counters 192+i: 199 reads answered from read-ahead, 200 blocks,
    //     201 starts, drops by 202 a read, 203 a store, 204 a packet, 205 a
    //     write. The prefetcher starts 2 pages ahead and doubles that on
    //     every new page the reads enter ---
    csr_wr(184, RA_FIRST | 64'd5);
    program_win(7, 1, 5, 0, R7, 64'h10_0000, U7);

    // --- T26: 64 consecutive 256 B reads (a copy engine's) of window 3
    //     from 0x8000: the first is its own get, the second starts a stream;
    //     the window is fetched ahead in pages up to its end (8 blocks,
    //     0x8000 .. 0xF000) and the other 63 reads are answered from them ---
    begin
        logic [63:0] y0 [16], y1 [16];
        ra_settle();
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
        for (int k = 0; k < 64; k++) uread(U3 + 64'h8000 + 256*k, 3, 6, 1, k);
        loop_gets("T26", 9);
        chk_get("T26 own get", 0, R3 + 48'h8000, 4, -1);
        for (int k = 0; k < 8; k++) chk_get($sformatf("T26 block %0d", k), 1 + k, R3 + 48'h8000 + 48'(4096*k), 64, 64 + ((8 + k) % 32));
        for (int k = 0; k < 64; k++) exp_read($sformatf("T26 read %0d", k), 64'h8000 + 256*k, 3, 6, 1, k);
        `CHECK(r_d.size() == 0, $sformatf("T26: %0d extra R beats", r_d.size()))
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[0] - y0[0] == 64 && y1[1] - y0[1] == 64 && y1[2] == y0[2] && y1[5] == y0[5] &&
               y1[7] - y0[7] == 63 && y1[8] - y0[8] == 8 && y1[9] - y0[9] == 1 &&
               y1[10] == y0[10] && y1[11] == y0[11] && y1[12] == y0[12] && y1[13] == y0[13],
               $sformatf("T26: reads %0d answered %0d failed %0d stray %0d; read-ahead reads %0d blocks %0d starts %0d drops %0d/%0d/%0d/%0d",
                         y1[0] - y0[0], y1[1] - y0[1], y1[2] - y0[2], y1[5] - y0[5], y1[7] - y0[7], y1[8] - y0[8],
                         y1[9] - y0[9], y1[10] - y0[10], y1[11] - y0[11], y1[12] - y0[12], y1[13] - y0[13]))
        clear();
    end
    $display("ok   T26 read-ahead: a stream answered from blocks, up to the window's end");

    // --- T27: a store landing here drops the stream: 8 consecutive reads
    //     from 0 (a stream from the second; pages 0 and 1 requested), then an
    //     inline store lands in export 2, then 8 more consecutive reads: one
    //     shared get. After the quiet time, with the old blocks back, reads
    //     continuing read 15 start a new stream at 0x1000 (pages 1, 2) ---
    begin
        logic [63:0] y0 [16], y1 [16];
        ra_settle();
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
        for (int k = 0; k < 8; k++) uread(U3 + 256*k, 3, 6, 1, k);
        repeat (100) @(posedge aclk);
        rx_inline(2, 64'h40, 64'h1234);
        repeat (20) @(posedge aclk);
        for (int k = 8; k < 16; k++) uread(U3 + 256*k, 3, 6, 1, k);
        loop_gets("T27", 1 + 2 + 1);
        chk_get("T27 after the drop", 3, R3 + 48'h800, 32, -1);
        for (int k = 0; k < 16; k++) exp_read($sformatf("T27 read %0d", k), 256*k, 3, 6, 1, k);
        `CHECK(r_d.size() == 0, $sformatf("T27: %0d extra R beats", r_d.size()))
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[7] - y0[7] == 7 && y1[8] - y0[8] == 2 && y1[9] - y0[9] == 1 && y1[11] - y0[11] == 1 &&
               y1[10] == y0[10] && y1[12] == y0[12] && y1[13] == y0[13],
               $sformatf("T27: read-ahead reads %0d blocks %0d starts %0d drops by read %0d store %0d packet %0d write %0d",
                         y1[7] - y0[7], y1[8] - y0[8], y1[9] - y0[9], y1[10] - y0[10], y1[11] - y0[11],
                         y1[12] - y0[12], y1[13] - y0[13]))
        clear();
        repeat (700) @(posedge aclk);
        for (int k = 16; k < 20; k++) uread(U3 + 256*k, 3, 6, 1, k);
        loop_gets("T27 new stream", 2);
        chk_get("T27 new stream block 0", 0, R3 + 48'h1000, 64, 64 + 1);
        for (int k = 16; k < 20; k++) exp_read($sformatf("T27 read %0d", k), 256*k, 3, 6, 1, k);
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
        `CHECK(y0[9] - y1[9] == 1 && y0[7] - y1[7] == 4,
               $sformatf("T27: new stream: starts %0d, read-ahead reads %0d", y0[9] - y1[9], y0[7] - y1[7]))
        clear();
    end
    $display("ok   T27 a store landing drops the stream; a new one after the quiet time");

    // --- T28: the same bytes again are never answered from read-ahead (a
    //     reader polling): reads at 0x8000, 0x8100, 0x8200, then 0x8200
    //     again (drops the stream: a fresh get), then 0x8300 (joins that
    //     get: no new stream while the old blocks are out) ---
    begin
        logic [63:0] y0 [16], y1 [16];
        longint off [5] = '{64'h8000, 64'h8100, 64'h8200, 64'h8200, 64'h8300};
        ra_settle();
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
        for (int k = 0; k < 3; k++) uread(U3 + off[k], 3, 6, 1, k);
        repeat (100) @(posedge aclk);
        for (int k = 3; k < 5; k++) uread(U3 + off[k], 3, 6, 1, k);
        loop_gets("T28", 1 + 2 + 1);
        chk_get("T28 fresh", 3, R3 + 48'h8200, 8, -1);
        for (int k = 0; k < 5; k++) exp_read($sformatf("T28 read %0d", k), off[k], 3, 6, 1, k);
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[7] - y0[7] == 2 && y1[10] - y0[10] == 1 && y1[9] - y0[9] == 1,
               $sformatf("T28: read-ahead reads %0d, drops by read %0d, starts %0d",
                         y1[7] - y0[7], y1[10] - y0[10], y1[9] - y0[9]))
        clear();
    end
    $display("ok   T28 the same bytes again: a fresh get, the stream dropped");

    // --- T29 / T30: a write into the uwin (a put to window 3) and a packet
    //     landing here (4 KiB into export 2) drop a stream likewise ---
    for (int t = 0; t < 2; t++) begin
        logic [63:0] y0 [16], y1 [16];
        string nm;
        nm = t ? "T30" : "T29";
        ra_settle();
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
        uread(U3 + 64'hC000, 3, 6, 1, 1);
        uread(U3 + 64'hC100, 3, 6, 1, 2);
        repeat (100) @(posedge aclk);
        if (t) rx_bulk(2, 64'h1000, 4096);
        else   uwr(U3 + 64'h100, 1);
        repeat (20) @(posedge aclk);
        uread(U3 + 64'hC200, 3, 6, 1, 3);
        loop_gets(nm, 1 + 2 + 1);
        exp_read({nm, " read 1"}, 64'hC000, 3, 6, 1, 1);
        exp_read({nm, " read 2"}, 64'hC100, 3, 6, 1, 2);
        exp_read({nm, " read 3"}, 64'hC200, 3, 6, 1, 3);
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[7] - y0[7] == 1 && y1[9] - y0[9] == 1 && y1[12] - y0[12] == t && y1[13] - y0[13] == 1 - t &&
               y1[10] == y0[10] && y1[11] == y0[11],
               $sformatf("%s: read-ahead reads %0d starts %0d drops by read %0d store %0d packet %0d write %0d", nm,
                         y1[7] - y0[7], y1[9] - y0[9], y1[10] - y0[10], y1[11] - y0[11], y1[12] - y0[12], y1[13] - y0[13]))
        clear();
    end
    $display("ok   T29 a write into the uwin drops the stream");
    $display("ok   T30 a packet landing drops the stream");

    // --- T31: blocks the far side rejects, and a partial last page:
    //     window 4 (0x1040 long, export 5 not on the far side): 16
    //     consecutive 256 B reads and a 64 B one at 0x1000; a stream from the
    //     second read, 2 blocks (0, and 0x1000 with one line); every read
    //     all ones ---
    begin
        logic [63:0] y0 [16], y1 [16];
        ra_settle();
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
        for (int k = 0; k < 16; k++) uread(U4 + 256*k, 3, 6, 1, k);
        uread(U4 + 64'h1000, 0, 6, 1, 16);
        loop_gets("T31", 1 + 2);
        chk_get("T31 partial page", 2, R4 + 48'h1000, 1, 64 + 1);
        for (int k = 0; k < 16; k++) exp_read($sformatf("T31 read %0d", k), 0, 3, 6, 1, k, 1);
        exp_read("T31 read 16", 0, 0, 6, 1, 16, 1);
        `CHECK(r_d.size() == 0, $sformatf("T31: %0d extra R beats", r_d.size()))
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[2] - y0[2] == 17 && y1[3] - y0[3] == 3 && y1[5] == y0[5] && y1[7] - y0[7] == 16 && y1[8] - y0[8] == 2,
               $sformatf("T31: failed %0d far-rejected %0d stray %0d read-ahead reads %0d blocks %0d",
                         y1[2] - y0[2], y1[3] - y0[3], y1[5] - y0[5], y1[7] - y0[7], y1[8] - y0[8]))
        clear();
    end
    $display("ok   T31 rejected blocks answer all ones; a partial last page");

    // --- T32: the block ring wraps, and a reader that runs ahead of it
    //     waits. Window 7 (1 MiB of export 7): 1024 consecutive 256 B reads
    //     (256 KiB, 64 pages through 32 blocks), then 64 consecutive 4 KiB
    //     reads (64 slots of whole pages: more than the ring holds, so reads
    //     wait for the oldest page to be given back). Gets are answered as
    //     they go out; every beat is checked ---
    begin
        logic [63:0] y0 [16], y1 [16];
        int sz [2] = '{256, 4096};
        int nr [2] = '{1024, 64};
        for (int t = 0; t < 2; t++) begin
            int got = 0, gets = 0, rounds = 0;
            bit issued = 0;
            longint base = t ? 64'h4_0000 : 64'h0;
            ra_settle();
            for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
            fork
                begin
                    for (int k = 0; k < nr[t]; k++) uread(U7 + base + longint'(sz[t]) * k, sz[t] / 64 - 1, 6, 1, k % 64);
                    issued = 1;
                end
            join_none
            while (got < nr[t] && rounds < 400) begin
                loop_gets($sformatf("T32.%0d", t), -1);
                gets += last_gets;
                rounds++;
                while (r_d.size() >= sz[t] / 64 && got < nr[t]) begin
                    for (int i = 0; i < sz[t] / 64; i++) begin
                        longint line = base + longint'(sz[t]) * got + 64 * i;
                        `CHECK(r_d.pop_front() == hpat(X7 + 48'(line)) && r_id.pop_front() == got % 64 &&
                               r_l.pop_front() == (i == sz[t] / 64 - 1) && r_rs.pop_front() == 2'b00,
                               $sformatf("T32.%0d: read %0d beat %0d (line %h)", t, got, i, line))
                    end
                    got++;
                end
            end
            wait (issued);
            // the stream keeps fetching ahead after the last read: answer
            // those gets too (the far side always does), so its blocks drain
            do begin loop_gets($sformatf("T32.%0d tail", t), -1); gets += last_gets; end while (last_gets != 0);
            `CHECK(got == nr[t] && r_d.size() == 0, $sformatf("T32.%0d: %0d of %0d reads answered, %0d beats left", t, got, nr[t], r_d.size()))
            for (int i = 0; i < 16; i++) csr_rd(192 + i, y1[i]);
            `CHECK(y1[9] - y0[9] == 1 && y1[7] - y0[7] == nr[t] - 1 && y1[2] == y0[2] && y1[5] == y0[5] &&
                   y1[10] == y0[10] && y1[11] == y0[11] && y1[12] == y0[12] && y1[13] == y0[13],
                   $sformatf("T32.%0d: starts %0d read-ahead reads %0d failed %0d stray %0d drops %0d/%0d/%0d/%0d", t,
                             y1[9] - y0[9], y1[7] - y0[7], y1[2] - y0[2], y1[5] - y0[5],
                             y1[10] - y0[10], y1[11] - y0[11], y1[12] - y0[12], y1[13] - y0[13]))
            $display("     T32.%0d: %0d reads of %0d B, %0d gets (%0d blocks) in %0d rounds", t, nr[t], sz[t], gets,
                     y1[8] - y0[8], rounds);
            clear();
        end
    end
    $display("ok   T32 the block ring wraps; a reader ahead of it waits");

    // --- T33: reads attached to pages not yet requested when the stream is
    //     dropped still get their pages. Window 7 at 0x8_0000: one 4 KiB read
    //     (its own get), then the get port is held (the ingress's request
    //     queue forced full: it takes no get), four more consecutive 4 KiB
    //     reads attach to pages the prefetcher has not requested yet, a store
    //     lands (the drop), the port is let go: the four pages are fetched all
    //     the same and every read is answered ---
    begin
        logic [63:0] y0 [16], y1 [16];
        ra_settle();
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
        uread(U7 + 64'h8_0000, 63, 6, 1, 0);
        wait (int'(inst_dut.inst_loom_read.ast) == 0 && !inst_dut.inst_loom_read.m_get_valid);
        repeat (5) @(posedge aclk);
        force inst_dut.inst_loom_ingress.pq_full = 1'b1;
        for (int k = 1; k < 5; k++) uread(U7 + 64'h8_0000 + 4096*k, 63, 6, 1, k);
        repeat (20) @(posedge aclk);
        `CHECK(inst_dut.inst_loom_read.s_need - inst_dut.inst_loom_read.s_fetch >= 3 * 4096,
               $sformatf("T33: pages wanted %h, requested up to %h", inst_dut.inst_loom_read.s_need,
                         inst_dut.inst_loom_read.s_fetch))
        rx_inline(2, 64'h80, 64'h5678);
        repeat (20) @(posedge aclk);
        force inst_dut.inst_loom_ingress.pq_full = 1'b0;
        @(posedge aclk);
        release inst_dut.inst_loom_ingress.pq_full;
        loop_gets("T33", 1 + 4);
        for (int k = 0; k < 5; k++)
            for (int i = 0; i < 64; i++) begin
                longint line = 64'h8_0000 + 4096*k + 64*i;
                `CHECK(r_d.size() != 0, $sformatf("T33: read %0d beat %0d missing", k, i))
                if (r_d.size() == 0) break;
                `CHECK(r_d.pop_front() == hpat(X7 + 48'(line)) && r_id.pop_front() == k && r_l.pop_front() == (i == 63) &&
                       r_rs.pop_front() == 2'b00, $sformatf("T33: read %0d beat %0d", k, i))
            end
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[9] - y0[9] == 1 && y1[7] - y0[7] == 4 && y1[8] - y0[8] == 4 && y1[11] - y0[11] == 1,
               $sformatf("T33: starts %0d read-ahead reads %0d blocks %0d drops by store %0d",
                         y1[9] - y0[9], y1[7] - y0[7], y1[8] - y0[8], y1[11] - y0[11]))
        clear();
    end
    $display("ok   T33 pages wanted by attached reads are fetched after a drop");

    // --- T34: reads in flight across the slot pointers' wrap: with
    //     read-ahead off, single-line reads 128 B apart (a get each) bring
    //     the write pointer to 120 of its 128 values; then 16 reads are
    //     issued and none answered: all 16 are taken at once (the 8 after
    //     the wrap too), no cycle waiting for a slot; then answered, exact ---
    csr_wr(184, RA_FIRST | 64'h105);
    begin
        logic [63:0] y0, y1, w0, w1;
        int n;
        ra_settle();
        n = (120 - int'(inst_dut.inst_loom_read.wp)) & 127;
        while (n > 0) begin
            int b = (n > 48) ? 48 : n;
            for (int k = 0; k < b; k++) uread(U3 + 64'h2000 + 128*k, 0, 6, 1, k);
            loop_gets("T34 fill", b);
            for (int k = 0; k < b; k++) exp_read($sformatf("T34 fill %0d", k), 64'h2000 + 128*k, 0, 6, 1, k);
            clear();
            n -= b;
        end
        `CHECK(int'(inst_dut.inst_loom_read.wp) == 120, $sformatf("T34: write pointer %0d, want 120", inst_dut.inst_loom_read.wp))
        csr_rd(192, y0); csr_rd(196, w0);
        fork
            for (int k = 0; k < 16; k++) uread(U3 + 64'h6000 + 128*k, 0, 6, 1, k);
        join_none
        repeat (300) @(posedge aclk);
        csr_rd(192, y1); csr_rd(196, w1);
        `CHECK(y1 - y0 == 16 && w1 == w0, $sformatf("T34: %0d of 16 reads taken across the wrap, %0d cycles waiting for a slot",
                                                   y1 - y0, w1 - w0))
        wait fork;
        loop_gets("T34", 16);
        for (int k = 0; k < 16; k++) exp_read($sformatf("T34 read %0d", k), 64'h6000 + 128*k, 0, 6, 1, k);
        `CHECK(r_d.size() == 0, $sformatf("T34: %0d extra R beats", r_d.size()))
        clear();
    end
    csr_wr(184, RA_FIRST | 64'd5);
    $display("ok   T34 reads in flight across the slot pointers' wrap");

    // --- T35: the ramp's bounds from RD_CTL: at most 2 pages ahead
    //     ([21:16] = 2), 64 consecutive 256 B reads of window 7 from
    //     0x1_0000 (4 pages) fetch pages 0x10 .. 0x14 (5 blocks); with 8
    //     pages at the start ([29:24] = 8) two reads fetch 8 pages ---
    begin
        logic [63:0] y0 [16], y1 [16];
        csr_wr(184, RA_FIRST | 64'd5 | (64'd2 << 16));
        ra_settle();
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
        for (int k = 0; k < 64; k++) uread(U7 + 64'h1_0000 + 256*k, 3, 6, 1, k);
        loop_gets("T35 at most 2 ahead", 1 + 5);
        for (int k = 0; k < 64; k++) exp_read($sformatf("T35 read %0d", k), 64'h1_0000 + 256*k, 3, 6, 1, k, 0, X7);
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[7] - y0[7] == 63 && y1[8] - y0[8] == 5 && y1[9] - y0[9] == 1,
               $sformatf("T35: read-ahead reads %0d blocks %0d starts %0d", y1[7] - y0[7], y1[8] - y0[8], y1[9] - y0[9]))
        clear();
        csr_wr(184, RA_EARLY | 64'd5 | (64'd8 << 24));
        ra_settle();
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
        uread(U7 + 64'h2_0000, 3, 6, 1, 0);
        uread(U7 + 64'h2_0100, 3, 6, 1, 1);
        loop_gets("T35 8 at the start", 1 + 8);
        for (int k = 0; k < 2; k++) exp_read($sformatf("T35 start read %0d", k), 64'h2_0000 + 256*k, 3, 6, 1, k, 0, X7);
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[8] - y0[8] == 8, $sformatf("T35: %0d blocks at the start, want 8", y1[8] - y0[8]))
        clear();
        csr_wr(184, RA_FIRST | 64'd5);
        ra_settle();
    end
    $display("ok   T35 the ramp's bounds from RD_CTL");

    // --- T36: no new stream while a dropped one's blocks are out, and
    //     RD_CTL bit 8 drops a running stream. (a) A stream on window 3 at
    //     0x8000 (blocks for pages 8, 9 out, unanswered), a read elsewhere
    //     drops it; after the quiet time two consecutive reads of window 7
    //     at 0x2_8000 (page 0x28: the same ring entries as pages 8, 9) start
    //     no stream: one shared get, and every read gets its own page's data
    //     though the old blocks are answered first. (b) A stream on window 3
    //     at 0xA000, then read-ahead off: the next consecutive reads are not
    //     answered from it (one shared get) ---
    begin
        logic [63:0] y0 [16], y1 [16];
        ra_settle();
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
        uread(U3 + 64'h8000, 3, 6, 1, 0);
        uread(U3 + 64'h8100, 3, 6, 1, 1);
        repeat (50) @(posedge aclk);
        uread(U3 + 64'hE000, 0, 6, 1, 2);
        repeat (700) @(posedge aclk);
        uread(U7 + 64'h2_8000, 3, 6, 1, 3);
        uread(U7 + 64'h2_8100, 3, 6, 1, 4);
        loop_gets("T36 (a)", 1 + 2 + 1 + 1);
        exp_read("T36 read 0", 64'h8000, 3, 6, 1, 0);
        exp_read("T36 read 1", 64'h8100, 3, 6, 1, 1);
        exp_read("T36 read 2", 64'hE000, 0, 6, 1, 2);
        exp_read("T36 read 3", 64'h2_8000, 3, 6, 1, 3, 0, X7);
        exp_read("T36 read 4", 64'h2_8100, 3, 6, 1, 4, 0, X7);
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[9] - y0[9] == 1 && y1[7] - y0[7] == 1 && y1[10] - y0[10] == 1,
               $sformatf("T36 (a): starts %0d read-ahead reads %0d drops by read %0d", y1[9] - y0[9], y1[7] - y0[7], y1[10] - y0[10]))
        clear();
        ra_settle();
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
        uread(U3 + 64'hA000, 3, 6, 1, 0);
        uread(U3 + 64'hA100, 3, 6, 1, 1);
        repeat (50) @(posedge aclk);
        csr_wr(184, RA_FIRST | 64'h105);
        uread(U3 + 64'hA200, 3, 6, 1, 2);
        uread(U3 + 64'hA300, 3, 6, 1, 3);
        loop_gets("T36 (b)", 1 + 2 + 1);
        for (int k = 0; k < 4; k++) exp_read($sformatf("T36 (b) read %0d", k), 64'hA000 + 256*k, 3, 6, 1, k);
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[9] - y0[9] == 1 && y1[7] - y0[7] == 1,
               $sformatf("T36 (b): starts %0d read-ahead reads %0d", y1[9] - y0[9], y1[7] - y0[7]))
        clear();
        csr_wr(184, RA_FIRST | 64'd5);
        ra_settle();
    end
    $display("ok   T36 no stream while old blocks are out; read-ahead off drops a stream");

    // --- T37: the default start rule: a stream starts where consecutive
    //     reads, a whole page already, enter the next page. (a) 16 x 256 B
    //     in one page (window 3 at 0x8000): no stream, one get. (b) 16 x
    //     256 B from 0x9800, entering 0xA000 after 2 KiB: no stream, two
    //     gets. (c) 64 x 256 B from 0xC000: page 0xC is one get, the stream
    //     starts at 0xD000 (pages 0xD, 0xE, then 0xF as the reads enter 0xE:
    //     the window's end); 48 reads from read-ahead ---
    csr_wr(184, 64'd5);
    begin
        logic [63:0] y0 [16], y1 [16];
        ra_settle();
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
        for (int k = 0; k < 16; k++) uread(U3 + 64'h8000 + 256*k, 3, 6, 1, k);
        loop_gets("T37 (a) one page", 1);
        for (int k = 0; k < 16; k++) exp_read($sformatf("T37 (a) read %0d", k), 64'h8000 + 256*k, 3, 6, 1, k);
        repeat (40) @(posedge aclk);
        for (int k = 0; k < 16; k++) uread(U3 + 64'h9800 + 256*k, 3, 6, 1, k);
        loop_gets("T37 (b) across a page after 2 KiB", 2);
        for (int k = 0; k < 16; k++) exp_read($sformatf("T37 (b) read %0d", k), 64'h9800 + 256*k, 3, 6, 1, k);
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[9] == y0[9] && y1[7] == y0[7], $sformatf("T37 (a, b): starts %0d read-ahead reads %0d, want 0",
                                                         y1[9] - y0[9], y1[7] - y0[7]))
        clear();
        ra_settle();
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
        for (int k = 0; k < 64; k++) uread(U3 + 64'hC000 + 256*k, 3, 6, 1, k);
        loop_gets("T37 (c)", 1 + 3);
        chk_get("T37 (c) page 0xC", 0, R3 + 48'hC000, 64, -1);
        chk_get("T37 (c) block 0xD", 1, R3 + 48'hD000, 64, 64 + (13 % 32));
        for (int k = 0; k < 64; k++) exp_read($sformatf("T37 (c) read %0d", k), 64'hC000 + 256*k, 3, 6, 1, k);
        `CHECK(r_d.size() == 0, $sformatf("T37: %0d extra R beats", r_d.size()))
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[9] - y0[9] == 1 && y1[7] - y0[7] == 48 && y1[8] - y0[8] == 3,
               $sformatf("T37 (c): starts %0d read-ahead reads %0d blocks %0d", y1[9] - y0[9], y1[7] - y0[7], y1[8] - y0[8]))
        clear();
        // (d) 32 x 256 B from 0x9800: a page of reads by 0xA800, but the
        // stream starts only where they enter 0xB000 (pages 0xB-0xE, 4
        // ahead by default); page 0xA is one get
        ra_settle();
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
        for (int k = 0; k < 32; k++) uread(U3 + 64'h9800 + 256*k, 3, 6, 1, k);
        loop_gets("T37 (d)", 2 + 4);
        chk_get("T37 (d) page 0xA", 1, R3 + 48'hA000, 64, -1);
        chk_get("T37 (d) block 0xB", 2, R3 + 48'hB000, 64, 64 + 11);
        for (int k = 0; k < 32; k++) exp_read($sformatf("T37 (d) read %0d", k), 64'h9800 + 256*k, 3, 6, 1, k);
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[9] - y0[9] == 1 && y1[7] - y0[7] == 8,
               $sformatf("T37 (d): starts %0d read-ahead reads %0d", y1[9] - y0[9], y1[7] - y0[7]))
        clear();
        ra_settle();
    end
    $display("ok   T37 a stream starts entering a page after a whole page of reads");

    // --- T38: a read that drops a stream does not hold back the next one
    //     (no quiet time after a drop by a read): a stream on window 3 at
    //     0x4000 (first rule), answered; then reads at 0x6000, 0x6100 at
    //     once: the first drops the stream, the second starts the next ---
    csr_wr(184, RA_FIRST | 64'd5);
    begin
        logic [63:0] y0 [16], y1 [16];
        ra_settle();
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y0[i]);
        uread(U3 + 64'h4000, 3, 6, 1, 0);
        uread(U3 + 64'h4100, 3, 6, 1, 1);
        loop_gets("T38 first stream", 1 + 2);
        exp_read("T38 read 0", 64'h4000, 3, 6, 1, 0);
        exp_read("T38 read 1", 64'h4100, 3, 6, 1, 1);
        uread(U3 + 64'h6000, 3, 6, 1, 2);
        uread(U3 + 64'h6100, 3, 6, 1, 3);
        loop_gets("T38 next stream", 1 + 2);
        exp_read("T38 read 2", 64'h6000, 3, 6, 1, 2);
        exp_read("T38 read 3", 64'h6100, 3, 6, 1, 3);
        for (int i = 0; i < 16; i++) csr_rd(192 + i, y1[i]);
        `CHECK(y1[9] - y0[9] == 2 && y1[10] - y0[10] == 1,
               $sformatf("T38: starts %0d, drops by a read %0d", y1[9] - y0[9], y1[10] - y0[10]))
        clear();
        csr_wr(184, 64'd5);
        ra_settle();
    end
    $display("ok   T38 a read that drops a stream does not hold back the next");

    if (errors == 0) $display("TB PASS (tb_loom_switch_top)");
    else             $display("TB FAIL (tb_loom_switch_top): %0d errors", errors);
    $finish;
end

initial begin
    #10ms;
    $display("TB FAIL (tb_loom_switch_top): timeout");
    $finish;
end

endmodule
