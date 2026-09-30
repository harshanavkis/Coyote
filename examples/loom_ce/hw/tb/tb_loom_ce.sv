`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_loom_ce - the generated wrapper (design_user_logic_c0_0, including
 * vfpga_top.svh) with the TB as the V80 shell: AXI-Lite on the CSR page,
 * sq_rd answered with card data, sq_wr and the host stream captured, cq_wr
 * completing each write.
 *
 * Card reads come back in 1 KB tlast-terminated segments, so only the beat
 * count can end a copy. Covers: a copy with a fence and one without, the
 * exact requests, data and fence value; START while busy ignored; a 64 KiB
 * copy under random backpressure on every channel; CSR readback; the cycle
 * counter stopping at the data write's completion; the stall counters (48-50)
 * moving under backpressure.
 */
module tb_loom_ce;

logic aclk = 0;
logic aresetn = 0;
always #2 aclk = ~aclk;

AXI4L axi_ctrl (.aclk(aclk), .aresetn(aresetn));
metaIntf #(.STYPE(irq_not_t)) notify (.*);
metaIntf #(.STYPE(req_t)) sq_rd (.*);
metaIntf #(.STYPE(req_t)) sq_wr (.*);
metaIntf #(.STYPE(ack_t)) cq_rd (.*);
metaIntf #(.STYPE(ack_t)) cq_wr (.*);
AXI4SR axis_host_recv [N_STRM_AXI] (.*);
AXI4SR axis_host_send [N_STRM_AXI] (.*);
AXI4SR axis_card_recv [N_CARD_AXI] (.*);
AXI4SR axis_card_send [N_CARD_AXI] (.*);

design_user_logic_c0_0 inst_dut (
    .axi_ctrl(axi_ctrl), .notify(notify),
    .sq_rd(sq_rd), .sq_wr(sq_wr), .cq_rd(cq_rd), .cq_wr(cq_wr),
    .axis_host_recv(axis_host_recv), .axis_host_send(axis_host_send),
    .axis_card_recv(axis_card_recv), .axis_card_send(axis_card_send),
    .aclk(aclk), .aresetn(aresetn)
);

int errors = 0;
`define CHECK(c, msg) if (!(c)) begin errors++; $display("FAIL [%0t] %s", $time, msg); end

function automatic logic [AXI_DATA_BITS-1:0] pat(input longint va);
    logic [AXI_DATA_BITS-1:0] d;
    for (int l = 0; l < 8; l++) d[64*l +: 64] = {32'hCE00_0000 + 32'(l), 32'(va)};
    return d;
endfunction

// ---------------------------------------------------------------------------
// Shell mocks
// ---------------------------------------------------------------------------
bit bp = 0;

initial begin
    cq_rd.valid = 0; cq_rd.data = '0;
    axis_host_recv[0].tvalid = 0; axis_host_recv[0].tdata = '0; axis_host_recv[0].tkeep = '0;
    axis_host_recv[0].tlast = 0; axis_host_recv[0].tid = '0;
    axis_card_send[0].tready = 1;
    axis_card_recv[0].tvalid = 0; axis_card_recv[0].tdata = '0; axis_card_recv[0].tkeep = '0;
    axis_card_recv[0].tlast = 0; axis_card_recv[0].tid = '0;
    notify.ready = 1;
end

always @(posedge aclk) begin
    sq_rd.ready              <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
    sq_wr.ready              <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
    axis_host_send[0].tready <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
end

// Card reads: each accepted sq_rd is answered with its beats, in 1 KB
// segments each ending in tlast
req_t rd_q [$], rd_seen [$];
always @(posedge aclk) if (aresetn && sq_rd.valid && sq_rd.ready) begin
    rd_q.push_back(sq_rd.data);
    rd_seen.push_back(sq_rd.data);
end

initial forever begin
    req_t r;
    wait (rd_q.size() > 0);
    r = rd_q.pop_front();
    for (int k = 0; k < r.len / 64; k++) begin
        @(negedge aclk);
        if (bp) while ($urandom_range(0, 2) == 0) @(negedge aclk);
        axis_card_recv[0].tdata  = pat(r.vaddr + 64*k);
        axis_card_recv[0].tkeep  = '1;
        axis_card_recv[0].tlast  = (k % 16 == 15) || (k == r.len / 64 - 1);
        axis_card_recv[0].tvalid = 1;
        do @(posedge aclk); while (!axis_card_recv[0].tready);
        @(negedge aclk);
        axis_card_recv[0].tvalid = 0;
    end
end

// Writes: requests and beats captured; a write completes (cq_wr) a few
// cycles after its last beat
req_t wr_seen [$], wr_open [$];
logic [AXI_DATA_BITS-1:0]   h_d [$];
logic [AXI_DATA_BITS/8-1:0] h_k [$];
bit                         h_l [$];
int                         cq_owed = 0, cq_delay = 0;

always @(posedge aclk) if (aresetn) begin
    if (sq_wr.valid && sq_wr.ready) begin wr_seen.push_back(sq_wr.data); wr_open.push_back(sq_wr.data); end
    if (axis_host_send[0].tvalid && axis_host_send[0].tready) begin
        h_d.push_back(axis_host_send[0].tdata); h_k.push_back(axis_host_send[0].tkeep);
        h_l.push_back(axis_host_send[0].tlast);
        if (axis_host_send[0].tlast) begin void'(wr_open.pop_front()); cq_owed++; end
    end
end
always @(posedge aclk) begin
    cq_wr.valid <= 1'b0;
    cq_wr.data  <= '0;
    if (cq_owed > 0) begin
        if (cq_delay == 20) begin
            cq_wr.valid <= 1'b1;
            cq_owed--;
            cq_delay = 0;
        end else cq_delay++;
    end
end

// ---------------------------------------------------------------------------
// CSR access
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

task automatic copy(input logic [47:0] src, input logic [47:0] dst, input int len,
                    input logic [47:0] fence, input bit go = 1);
    csr_wr(1, 64'(src)); csr_wr(2, 64'(dst)); csr_wr(3, 64'(len));
    csr_wr(4, 64'd3);    csr_wr(5, 64'(fence));
    if (go) csr_wr(0, 64'd1);
endtask

task automatic wait_idle();
    logic [63:0] b;
    do begin
        repeat (50) @(posedge aclk);
        csr_rd(8, b);
    end while (b[0] || cq_owed > 0);
    repeat (40) @(posedge aclk);
endtask

// The next copy in the capture queues: its read, its write, its beats, then
// the fence (if any) with the expected count
task automatic expect_copy(input string name, input logic [47:0] src, input logic [47:0] dst,
                           input int len, input logic [47:0] fence, input int count);
    req_t r;
    `CHECK(rd_seen.size() != 0, {name, ": no read request"})
    if (rd_seen.size() != 0) begin
        r = rd_seen.pop_front();
        `CHECK(r.opcode == LOCAL_READ && r.strm == STRM_CARD && r.pid == 3 && r.vaddr == src &&
               r.len == LEN_BITS'(len) && r.last, $sformatf("%s: read op %0d strm %0d va %h len %0d", name, r.opcode, r.strm, r.vaddr, r.len))
    end
    `CHECK(wr_seen.size() != 0, {name, ": no write request"})
    if (wr_seen.size() != 0) begin
        r = wr_seen.pop_front();
        `CHECK(r.opcode == LOCAL_WRITE && r.strm == STRM_HOST && r.dest == 0 && r.pid == 3 &&
               r.vaddr == dst && r.len == LEN_BITS'(len) && r.last,
               $sformatf("%s: write op %0d strm %0d va %h len %0d", name, r.opcode, r.strm, r.vaddr, r.len))
    end
    for (int k = 0; k < len / 64; k++) begin
        `CHECK(h_d.size() != 0, $sformatf("%s: beat %0d missing", name, k))
        if (h_d.size() == 0) break;
        `CHECK(h_d.pop_front() == pat(src + 64*k) && h_k.pop_front() == '1 &&
               h_l.pop_front() == (k == len / 64 - 1), $sformatf("%s: beat %0d", name, k))
    end
    if (fence != 0) begin
        logic [AXI_DATA_BITS-1:0] d;
        `CHECK(wr_seen.size() != 0, {name, ": no fence request"})
        if (wr_seen.size() != 0) begin
            r = wr_seen.pop_front();
            `CHECK(r.opcode == LOCAL_WRITE && r.strm == STRM_HOST && r.pid == 3 && r.vaddr == fence && r.len == 8,
                   $sformatf("%s: fence request va %h len %0d", name, r.vaddr, r.len))
        end
        d = h_d.pop_front();
        `CHECK(d[63:0] == 64'(count) && h_k.pop_front() == 64'hFF && h_l.pop_front(),
               $sformatf("%s: fence beat %0d, expected %0d", name, d[63:0], count))
    end
endtask

logic [63:0] v, c0, c1;

initial begin
    axi_ctrl.awvalid = 0; axi_ctrl.wvalid = 0; axi_ctrl.arvalid = 0;
    axi_ctrl.bready = 0; axi_ctrl.rready = 0;
    axi_ctrl.awaddr = 0; axi_ctrl.wdata = 0; axi_ctrl.wstrb = 0; axi_ctrl.araddr = 0;
    repeat (5) @(negedge aclk);
    aresetn = 1;
    repeat (5) @(negedge aclk);

    // --- T1: a 4 KB copy with a fence ---
    copy(48'h7f00_1000_0000, 48'h7e00_0000_0000, 4096, 48'h7f00_2000_0000);
    wait_idle();
    expect_copy("T1", 48'h7f00_1000_0000, 48'h7e00_0000_0000, 4096, 48'h7f00_2000_0000, 1);
    csr_rd(9, v);  `CHECK(v == 1, $sformatf("T1: COPIES %0d", v))
    csr_rd(10, c0); `CHECK(c0 > 64, $sformatf("T1: CYCLES %0d", c0))
    repeat (100) @(posedge aclk);
    csr_rd(10, c1); `CHECK(c1 == c0, "T1: CYCLES kept counting after the copy")
    `CHECK(rd_seen.size() == 0 && wr_seen.size() == 0 && h_d.size() == 0, "T1: extra traffic")
    $display("ok   T1 copy with fence (%0d cycles)", c0);

    // --- T2: without a fence ---
    copy(48'h7f00_1000_4000, 48'h7e00_0001_0000, 1024, 48'h0);
    wait_idle();
    expect_copy("T2", 48'h7f00_1000_4000, 48'h7e00_0001_0000, 1024, 48'h0, 0);
    csr_rd(9, v); `CHECK(v == 2, $sformatf("T2: COPIES %0d", v))
    `CHECK(wr_seen.size() == 0 && h_d.size() == 0, "T2: a fence was written")
    $display("ok   T2 copy without fence");

    // --- T3: START while busy is ignored ---
    bp = 1;
    copy(48'h7f00_1100_0000, 48'h7e00_0010_0000, 16384, 48'h7f00_2000_0000);
    csr_wr(0, 64'd1);                 // again, while the first is running
    csr_rd(8, v); `CHECK(v[0], "T3: not busy during a 16 KiB copy")
    wait_idle();
    expect_copy("T3", 48'h7f00_1100_0000, 48'h7e00_0010_0000, 16384, 48'h7f00_2000_0000, 3);
    csr_rd(9, v); `CHECK(v == 3, $sformatf("T3: COPIES %0d (a START while busy was taken?)", v))
    `CHECK(rd_seen.size() == 0 && wr_seen.size() == 0 && h_d.size() == 0, "T3: a second copy ran")
    $display("ok   T3 START while busy ignored");

    // --- T4: 64 KiB under backpressure on every channel ---
    copy(48'h7f00_1200_0000, 48'h7e00_0020_0000, 65536, 48'h7f00_2000_0040);
    wait_idle();
    expect_copy("T4", 48'h7f00_1200_0000, 48'h7e00_0020_0000, 65536, 48'h7f00_2000_0040, 4);
    bp = 0;
    begin
        logic [63:0] p48, p49, p50;
        csr_rd(48, p48); csr_rd(49, p49); csr_rd(50, p50);
        `CHECK(p48 > 0 && p49 > 0 && p50 > 0,
               $sformatf("T4: stall counters CE out bp %0d, CE in wait %0d, sq_wr wait %0d", p48, p49, p50))
    end
    $display("ok   T4 64 KiB under backpressure");

    // --- T5: readback ---
    csr_rd(1, v); `CHECK(v == 64'h7f00_1200_0000, "T5: SRC_VA")
    csr_rd(2, v); `CHECK(v == 64'h7e00_0020_0000, "T5: DST_VA")
    csr_rd(3, v); `CHECK(v == 65536, "T5: LEN")
    csr_rd(4, v); `CHECK(v == 3, "T5: PID")
    csr_rd(5, v); `CHECK(v == 64'h7f00_2000_0040, "T5: FENCE_VA")
    csr_rd(8, v); `CHECK(v == 0, "T5: BUSY")
    $display("ok   T5 readback");

    if (errors == 0) $display("TB PASS (tb_loom_ce)");
    else             $display("TB FAIL (tb_loom_ce): %0d errors", errors);
    $finish;
end

initial begin
    #2ms;
    $display("TB FAIL (tb_loom_ce): timeout");
    $finish;
end

endmodule
