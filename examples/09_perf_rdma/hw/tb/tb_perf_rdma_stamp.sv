`timescale 1ns / 1ps
import lynxTypes::*;

/**
 * tb_perf_rdma_stamp: the stamps of the perf_rdma vFPGA (stamp_csr +
 * stamp_rec, CSR words 224-240). The shell is a mock: a host-read packet
 * goes out to the stack and is acked; incoming WRITEs (one 4 KiB packet,
 * then a 16 KiB message as 4 packets and a 64 B one) land on
 * axis_host_send[1] and complete on cq_wr. Every point counts its events
 * and every stamp difference equals the cycles the testbench saw.
 */
module tb_perf_rdma_stamp;

logic aclk = 0;
logic [0:0] aresetn = 0;
always #2 aclk = ~aclk;

AXI4L axi_ctrl (.*);
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
    .axi_ctrl(axi_ctrl), .notify(notify),
    .sq_rd(sq_rd), .sq_wr(sq_wr), .cq_rd(cq_rd), .cq_wr(cq_wr), .rq_rd(rq_rd), .rq_wr(rq_wr),
    .axis_host_recv(axis_host_recv), .axis_host_send(axis_host_send),
    .axis_rreq_recv(axis_rreq_recv), .axis_rreq_send(axis_rreq_send),
    .axis_rrsp_recv(axis_rrsp_recv), .axis_rrsp_send(axis_rrsp_send),
    .aclk(aclk), .aresetn(aresetn)
);

int errors = 0;
`define CHECK(c, msg) if (!(c)) begin errors++; $display("FAIL [%0t] %s", $time, msg); end

// shell mocks: everything downstream always ready
initial begin
    axi_ctrl.awvalid = 0; axi_ctrl.wvalid = 0; axi_ctrl.arvalid = 0; axi_ctrl.bready = 1; axi_ctrl.rready = 1;
    axi_ctrl.awaddr = '0; axi_ctrl.araddr = '0; axi_ctrl.wdata = '0; axi_ctrl.wstrb = '0;
    notify.ready = 1; sq_rd.ready = 1; sq_wr.ready = 1;
    cq_rd.valid = 0; cq_rd.data = '0; cq_wr.valid = 0; cq_wr.data = '0;
    rq_rd.valid = 0; rq_rd.data = '0; rq_wr.valid = 0; rq_wr.data = '0;
    axis_host_recv[0].tvalid = 0; axis_host_recv[0].tdata = '0; axis_host_recv[0].tkeep = '0;
    axis_host_recv[0].tlast = 0; axis_host_recv[0].tid = '0; axis_host_send[0].tready = 1;
    axis_host_recv[1].tvalid = 0; axis_host_recv[1].tdata = '0; axis_host_recv[1].tkeep = '0;
    axis_host_recv[1].tlast = 0; axis_host_recv[1].tid = '0; axis_host_send[1].tready = 1;
    axis_rreq_recv[0].tvalid = 0; axis_rreq_recv[0].tdata = '0; axis_rreq_recv[0].tkeep = '0;
    axis_rreq_recv[0].tlast = 0; axis_rreq_recv[0].tid = '0; axis_rreq_send[0].tready = 1;
    axis_rrsp_recv[0].tvalid = 0; axis_rrsp_recv[0].tdata = '0; axis_rrsp_recv[0].tkeep = '0;
    axis_rrsp_recv[0].tlast = 0; axis_rrsp_recv[0].tid = '0; axis_rrsp_send[0].tready = 1;
end

// the cycles the testbench sees
longint cyc = 0;
longint ev_hr [$], ev_rq [$], ev_rr [$], ev_land [$], ev_h [$], ev_ack [$], ev_cmp [$];
bit hr_mid = 0, rr_mid = 0;
always @(posedge aclk) begin
    cyc++;
    if (axis_host_recv[0].tvalid && axis_host_recv[0].tready) begin
        if (!hr_mid) ev_hr.push_back(cyc);
        hr_mid = !axis_host_recv[0].tlast;
    end
    if (rq_wr.valid && rq_wr.ready) ev_rq.push_back(cyc);
    if (axis_rrsp_recv[0].tvalid && axis_rrsp_recv[0].tready) begin
        if (!rr_mid) ev_rr.push_back(cyc);
        rr_mid = !axis_rrsp_recv[0].tlast;
    end
    if (sq_wr.valid && sq_wr.ready && sq_wr.data.dest == 1) ev_land.push_back(cyc);
    if (axis_host_send[1].tvalid && axis_host_send[1].tready) ev_h.push_back(cyc);
    if (cq_wr.valid && cq_wr.data.remote) ev_ack.push_back(cyc);
    if (cq_wr.valid && !cq_wr.data.remote) ev_cmp.push_back(cyc);
end

task automatic csr_wr(input int word, input logic [63:0] data);
    @(negedge aclk);
    axi_ctrl.awaddr = 64'(word * 8); axi_ctrl.awvalid = 1;
    axi_ctrl.wdata = data; axi_ctrl.wstrb = 8'hFF; axi_ctrl.wvalid = 1;
    do @(posedge aclk); while (!(axi_ctrl.awready && axi_ctrl.wready));
    @(negedge aclk);
    axi_ctrl.awvalid = 0; axi_ctrl.wvalid = 0;
    while (!axi_ctrl.bvalid) @(posedge aclk);
    @(negedge aclk);
endtask

task automatic csr_rd(input int word, output logic [63:0] data);
    @(negedge aclk);
    axi_ctrl.araddr = 64'(word * 8); axi_ctrl.arvalid = 1;
    do @(posedge aclk); while (!axi_ctrl.arready);
    @(negedge aclk);
    axi_ctrl.arvalid = 0;
    while (!axi_ctrl.rvalid) @(posedge aclk);
    data = axi_ctrl.rdata;
    @(negedge aclk);
endtask

task automatic stamp(input int p, input int k, output logic [63:0] v);
    csr_wr(225, (64'(p) << 16) | 64'(k));
    csr_rd(226, v);
endtask

task automatic beats(input int n, input bit host);   // n beats of one packet, tlast on the last
    for (int i = 0; i < n; i++) begin
        @(negedge aclk);
        if (host) begin axis_host_recv[0].tvalid = 1; axis_host_recv[0].tlast = (i == n - 1); axis_host_recv[0].tkeep = '1; end
        else      begin axis_rrsp_recv[0].tvalid = 1; axis_rrsp_recv[0].tlast = (i == n - 1); axis_rrsp_recv[0].tkeep = '1; end
    end
    @(negedge aclk);
    axis_host_recv[0].tvalid = 0; axis_rrsp_recv[0].tvalid = 0;
endtask

task automatic rx_write(input int len);   // one incoming WRITE packet: rq_wr, a gap, its beats
    @(negedge aclk);
    rq_wr.data = '0; rq_wr.data.opcode = RC_RDMA_WRITE_ONLY; rq_wr.data.len = LEN_BITS'(len); rq_wr.valid = 1;
    @(negedge aclk); rq_wr.valid = 0;
    repeat (3) @(negedge aclk);
    beats((len + 63) / 64, 0);
endtask

task automatic cq(input bit remote);
    @(negedge aclk);
    cq_wr.data = '0; cq_wr.data.remote = remote; cq_wr.data.dest = remote ? 0 : 1; cq_wr.valid = 1;
    @(negedge aclk); cq_wr.valid = 0;
endtask

initial begin
    logic [63:0] v, n [9], s [9], a, b;
    repeat (10) @(posedge aclk);
    aresetn = 1;
    repeat (10) @(posedge aclk);
    csr_rd(224, v);
    `CHECK(v == {32'd4096, 32'd9}, $sformatf("ST_CTL info %h", v))
    csr_wr(224, 64'd1);
    // sender: one 64 B packet back from the host read, acked 20 cycles later
    beats(1, 1);
    repeat (20) @(negedge aclk);
    cq(1);
    // receiver: one 4 KiB WRITE packet, its completion
    rx_write(4096);
    repeat (5) @(negedge aclk);
    cq(0);
    repeat (10) @(negedge aclk);
    for (int p = 0; p < 9; p++) begin
        csr_rd(232 + p, n[p]);
        stamp(p, 0, s[p]);
        `CHECK(n[p] == (p == 1 ? 0 : 1), $sformatf("point %0d counted %0d events", p, n[p]))
        if (p != 1) `CHECK(!$isunknown(s[p]), $sformatf("point %0d stamp unknown", p))
        $display("st   point %0d: cycle %0d", p, s[p]);
    end
    `CHECK(s[2] == s[0], "host-read beat and beat to the stack differ")
    `CHECK(s[3] - s[0] == 64'(ev_ack[0] - ev_hr[0]), $sformatf("beat -> ack %0d, tb %0d", s[3] - s[0], ev_ack[0] - ev_hr[0]))
    `CHECK(s[5] - s[4] == 64'(ev_rr[0] - ev_rq[0]), $sformatf("rq -> beat in %0d, tb %0d", s[5] - s[4], ev_rr[0] - ev_rq[0]))
    `CHECK(s[6] - s[4] == 64'(ev_land[0] - ev_rq[0]), $sformatf("rq -> landing write %0d, tb %0d", s[6] - s[4], ev_land[0] - ev_rq[0]))
    `CHECK(s[7] - s[4] == 64'(ev_h[0] - ev_rq[0]), $sformatf("rq -> landing beat %0d, tb %0d", s[7] - s[4], ev_h[0] - ev_rq[0]))
    `CHECK(s[8] - s[4] == 64'(ev_cmp[0] - ev_rq[0]), $sformatf("rq -> completion %0d, tb %0d", s[8] - s[4], ev_cmp[0] - ev_rq[0]))
    `CHECK(s[4] - s[0] == 64'(ev_rq[0] - ev_hr[0]), $sformatf("one clock for both %0d, tb %0d", s[4] - s[0], ev_rq[0] - ev_hr[0]))

    // a 16 KiB message as 4 packets, then a 64 B one: one stamp per landing at points 6 and 7
    csr_wr(224, 64'd1);
    ev_land = {}; ev_h = {};
    for (int k = 0; k < 4; k++) rx_write(4096);
    rx_write(64);
    repeat (10) @(negedge aclk);
    for (int p = 6; p <= 7; p++) begin
        csr_rd(232 + p, v);
        `CHECK(v == 5, $sformatf("point %0d counted %0d landings, want 5", p, v))
    end
    for (int k = 0; k < 5; k++) begin
        stamp(6, k, a); stamp(7, k, b);
        `CHECK(b - a == 64'(ev_h[64*k] - ev_land[k]), $sformatf("landing %0d write -> first beat %0d, tb %0d", k, b - a, ev_h[64*k] - ev_land[k]))
    end
    csr_wr(224, 64'd1);
    csr_rd(232 + 4, v);
    `CHECK(v == 0, $sformatf("count after the re-arm %0d", v))

    if (errors == 0) $display("TB PASS (tb_perf_rdma_stamp)");
    else             $display("TB FAIL (tb_perf_rdma_stamp): %0d errors", errors);
    $finish;
end

initial begin
    #1ms;
    $display("TB FAIL (tb_perf_rdma_stamp): timeout");
    $finish;
end

endmodule
