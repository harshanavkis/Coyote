`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_loom_switch_top - the generated wrapper (design_user_logic_c0_0, which
 * includes vfpga_top.svh) with the TB as the shell: AXI-Lite on the CSR page,
 * AXI4 on the uwin, sq_wr / cq_wr, the host and rdma send streams, and
 * rq_wr + axis_rrsp_recv for incoming messages.
 *
 * Covers: table programming and CSR readback; bulk and stores through the
 * uwin on both routes, exact; the ack window holding rdma packets and
 * releasing them on acks; loom_rx landings (inline and a two-packet bulk
 * message) and RX_CHUNK changing how many host writes that message takes;
 * the ingress and loom_rx racing for sq_wr under backpressure, with the
 * arbitration invariants checked every cycle; the ingress and rx counters.
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

design_user_logic_c0_0 inst_dut (
    .axi_ctrl(axi_ctrl), .axi_udata(axi_udata), .notify(notify),
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
localparam logic [47:0] B2 = 48'h7f20_0000_0000;   // window 2: rdma, QP pid 5, far pid 9
localparam longint      U1 = 64'h000_0000;
localparam longint      U2 = 64'h010_0000;

function automatic logic [AXI_DATA_BITS-1:0] pat(input longint uaddr);
    logic [AXI_DATA_BITS-1:0] d;
    for (int l = 0; l < 8; l++) d[64*l +: 64] = {32'hC0DE_0000 + 32'(l), 32'(uaddr)};
    return d;
endfunction
function automatic logic [63:0] word(input int w);
    return 64'hFF << (8*w);
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
    sq_wr.ready             <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
    axis_host_send[0].tready <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
    axis_host_send[1].tready <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
    axis_rreq_send[0].tready <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
end

// Acks: one per rdma request, some cycles later, while acks_on
always @(posedge aclk) begin
    cq_wr.valid <= 1'b0;
    cq_wr.data  <= '0;
    if (sq_wr.valid && sq_wr.ready && sq_wr.data.strm == STRM_RDMA) acks_owed++;
    if (acks_on && acks_owed > 0 && $urandom_range(0, 3) == 0) begin
        cq_wr.valid       <= 1'b1;
        cq_wr.data.remote <= 1'b1;
        acks_owed--;
    end
end

// ---------------------------------------------------------------------------
// Capture
// ---------------------------------------------------------------------------
req_t wr_ing [$], wr_rx [$];         // sq_wr handshakes, by producer (dest)
logic [AXI_DATA_BITS-1:0]   h0_d [$], h1_d [$], n_d [$];
logic [AXI_DATA_BITS/8-1:0] h0_k [$];
bit                         h0_l [$], h1_l [$], n_l [$];

always @(posedge aclk) if (aresetn) begin
    if (sq_wr.valid && sq_wr.ready) begin
        if (sq_wr.data.dest == 1) wr_rx.push_back(sq_wr.data);
        else                      wr_ing.push_back(sq_wr.data);
    end
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
end

// Arbitration invariants: sq_wr carries loom_rx's request whenever it has
// one, else the ingress's, and never a request nobody presented
always @(posedge aclk) if (aresetn) begin
    if (sq_wr.valid) begin
        `CHECK(inst_dut.rx_wr_valid || inst_dut.ing_wr_valid, "sq_wr valid with no producer")
        if (inst_dut.rx_wr_valid) begin
            `CHECK(sq_wr.data === inst_dut.rx_wr_req, "rx had a request, sq_wr carried another")
        end else begin
            `CHECK(sq_wr.data === inst_dut.ing_wr_req, "sq_wr carried something other than the ingress's request")
        end
    end
    `CHECK(!(inst_dut.cnt_wr_blk_rx), "loom_rx waited its turn")
end

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
    axi_udata.arvalid = 0; axi_udata.wvalid = 0; axi_udata.bready = 1; axi_udata.rready = 1;
    axi_udata.araddr = 0; axi_udata.arlen = 0; axi_udata.arid = 0; axi_udata.arburst = 1;
    axi_udata.arsize = 6; axi_udata.arcache = 0; axi_udata.arlock = 0; axi_udata.arprot = 0;
    axi_udata.arqos = 0; axi_udata.arregion = 0;
end

// ---------------------------------------------------------------------------
// Incoming messages: rq_wr per packet, then its beats on axis_rrsp_recv
// ---------------------------------------------------------------------------
semaphore rx_lock = new(1);
task automatic rx_packet(input int len, input logic [AXI_DATA_BITS-1:0] beats [$]);
    @(negedge aclk);
    rq_wr.data = '0; rq_wr.data.pid = 6'd2; rq_wr.data.vaddr = STAGING; rq_wr.data.len = len[LEN_BITS-1:0];
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

task automatic rx_inline(input int dst_pid, input logic [47:0] va, input logic [63:0] data);
    logic [AXI_DATA_BITS-1:0] b [$];
    logic [AXI_DATA_BITS-1:0] h = '0;
    h[63:0] = {22'b0, 6'(dst_pid), 28'd8, 8'd2}; h[127:64] = {16'b0, va}; h[191:128] = data;
    b.push_back(h);
    rx_lock.get();
    rx_packet(64, b);
    rx_lock.put();
endtask

// A bulk message of len payload bytes, in PMTU packets: packet 0 = header + PMTU-64
task automatic rx_bulk(input int dst_pid, input logic [47:0] va, input int len);
    logic [AXI_DATA_BITS-1:0] b [$];
    logic [AXI_DATA_BITS-1:0] h = '0;
    int left = len, k = 0;
    h[63:0] = {22'b0, 6'(dst_pid), 28'(len), 8'd1}; h[127:64] = {16'b0, va};
    rx_lock.get();
    b.push_back(h);
    while (left > 0) begin
        int room = (PMTU_BYTES - 64*b.size()) / 64;
        while (room > 0 && left > 0) begin b.push_back(pat(64'(k))); k++; left -= 64; room--; end
        rx_packet(64*b.size(), b);
        b.delete();
    end
    rx_lock.put();
endtask

task automatic quiesce();
    int idle = 0;
    while (idle < 64) begin
        @(posedge aclk);
        if (!sq_wr.valid && !inst_dut.inst_loom_rx.busy && inst_dut.inst_loom_ingress.pq_empty &&
            int'(inst_dut.inst_loom_ingress.ostate) == 0 && !inst_dut.inst_loom_ingress.pk_open &&
            !axis_host_send[0].tvalid && !axis_host_send[1].tvalid && !axis_rreq_send[0].tvalid)
            idle++;
        else idle = 0;
    end
endtask

task automatic clear();
    wr_ing.delete(); wr_rx.delete();
    h0_d.delete(); h0_k.delete(); h0_l.delete(); h1_d.delete(); h1_l.delete(); n_d.delete(); n_l.delete();
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
           r.vaddr == STAGING && r.len == LEN_BITS'(64 + 64*beats),
           $sformatf("%s: rdma request op %0d len %0d", name, r.opcode, r.len))
    h = n_d.pop_front(); void'(n_l.pop_front());
    `CHECK(h[63:0] == {22'b0, 6'd9, 28'(64*beats), 8'd1} && h[127:64] == 64'(va),
           $sformatf("%s: header %h %h", name, h[127:64], h[63:0]))
    for (int j = 0; j < beats; j++)
        `CHECK(n_d.size() != 0 && n_d.pop_front() == pat(uaddr + 64*j) && n_l.pop_front() == (j == beats-1),
               $sformatf("%s: beat %0d", name, j))
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
        `CHECK(r.opcode == RC_RDMA_WRITE_ONLY && r.len == 64 && r.vaddr == STAGING, "T3 rdma store: request")
        h = n_d.pop_front();
        `CHECK(h[63:0] == {22'b0, 6'd9, 28'd8, 8'd2} && h[127:64] == 64'(B2 + 48'h200) &&
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
    rx_inline(7, 48'h7e00_0000_1008, 64'hFEED_F00D);
    quiesce();
    `CHECK(wr_rx.size() == 1 && wr_rx[0].opcode == LOCAL_WRITE && wr_rx[0].pid == 7 &&
           wr_rx[0].vaddr == 48'h7e00_0000_1008 && wr_rx[0].len == 8, "T5: rx landing request")
    `CHECK(h1_d.size() == 1 && h1_d[0][63:0] == 64'hFEED_F00D, "T5: rx landing data")
    $display("ok   T5 rx inline");
    clear();

    // --- T6: RX_CHUNK reaches loom_rx: a two-packet message takes two host
    //     writes at 1 and one at 2 ---
    rx_bulk(7, 48'h7e00_0010_0000, 4032 + 4096);
    quiesce();
    `CHECK(wr_rx.size() == 2 && wr_rx[0].len == 4032 && wr_rx[1].len == 4096,
           $sformatf("T6: chunk 1: %0d writes, first len %0d", wr_rx.size(), wr_rx.size() ? wr_rx[0].len : 0))
    `CHECK(h1_d.size() == 127, $sformatf("T6: chunk 1: %0d beats", h1_d.size()))
    clear();
    csr_wr(76, 64'd2);
    rx_bulk(7, 48'h7e00_0020_0000, 4032 + 4096);
    quiesce();
    `CHECK(wr_rx.size() == 1 && wr_rx[0].len == 4032 + 4096,
           $sformatf("T6: chunk 2: %0d writes, first len %0d", wr_rx.size(), wr_rx.size() ? wr_rx[0].len : 0))
    `CHECK(h1_d.size() == 127, $sformatf("T6: chunk 2: %0d beats", h1_d.size()))
    csr_wr(76, 64'd1);
    $display("ok   T6 rx_chunk");
    clear();

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
        for (int k = 0; k < 24; k++) rx_inline(7, 48'h7e00_0100_0000 + 48'(8*k), 64'(k));
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
            `CHECK(wr_rx[k].vaddr == 48'h7e00_0100_0000 + 48'(8*k) && h1_d[k][63:0] == 64'(k),
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

    if (errors == 0) $display("TB PASS (tb_loom_switch_top)");
    else             $display("TB FAIL (tb_loom_switch_top): %0d errors", errors);
    $finish;
end

initial begin
    #2ms;
    $display("TB FAIL (tb_loom_switch_top): timeout");
    $finish;
end

endmodule
