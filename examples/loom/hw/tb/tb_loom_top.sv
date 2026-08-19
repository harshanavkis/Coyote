`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_loom_top — top-level test of the generated wrapper
 * (design_user_logic_c0_0, which includes vfpga_top.svh verbatim).
 *
 * Focus: engine/rx arbitration on the shared {sq_wr, axis_host_send[0]}
 * path — mutual exclusion (continuous assertion), transaction atomicity
 * under races in both directions, starvation recovery, and a mixed soak
 * with exact counter accounting (completion disabled for clean sums).
 */
module tb_loom_top;

logic aclk = 0;
logic aresetn = 0;
always #2 aclk = ~aclk;

// ---- interfaces ----
AXI4L axi_ctrl (.aclk(aclk), .aresetn(aresetn));
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
    .sq_rd(sq_rd), .sq_wr(sq_wr), .cq_rd(cq_rd), .cq_wr(cq_wr),
    .rq_rd(rq_rd), .rq_wr(rq_wr),
    .axis_host_recv(axis_host_recv), .axis_host_send(axis_host_send),
    .axis_rreq_recv(axis_rreq_recv), .axis_rreq_send(axis_rreq_send),
    .axis_rrsp_recv(axis_rrsp_recv), .axis_rrsp_send(axis_rrsp_send),
    .aclk(aclk), .aresetn(aresetn)
);

int errors = 0;

task check(input bit cond, input string msg);
    if (!cond) begin
        errors++;
        $display("FAIL: %s", msg);
    end
endtask

// ---- shell-side mocks ----
initial begin
    sq_rd.ready = 1; sq_wr.ready = 1;
    cq_rd.valid = 0; cq_wr.valid = 0;
    cq_rd.data = '0; cq_wr.data = '0;
    rq_rd.valid = 0; rq_rd.data = '0;
    rq_wr.valid = 0; rq_wr.data = '0;
    axis_host_recv[0].tvalid = 0; axis_host_recv[0].tdata = '0;
    axis_host_recv[0].tkeep = '0; axis_host_recv[0].tlast = 0;
    axis_host_recv[0].tid = '0;
    axis_host_send[0].tready = 1;
    axis_rreq_send[0].tready = 1;
    axis_rreq_recv[0].tvalid = 0; axis_rreq_recv[0].tdata = '0;
    axis_rreq_recv[0].tkeep = '0; axis_rreq_recv[0].tlast = 0;
    axis_rreq_recv[0].tid = '0;
    axis_rrsp_recv[0].tvalid = 0; axis_rrsp_recv[0].tdata = '0;
    axis_rrsp_recv[0].tkeep = '0; axis_rrsp_recv[0].tlast = 0;
    axis_rrsp_recv[0].tid = '0;
    axis_rrsp_send[0].tready = 1;
    notify.ready = 1;
end

// ---- capture ----
req_t wrq[$], rdq[$];
int host_beats = 0, net_beats = 0;

always @(posedge aclk) begin
    if (sq_wr.valid && sq_wr.ready) wrq.push_back(sq_wr.data);
    if (sq_rd.valid && sq_rd.ready) rdq.push_back(sq_rd.data);
    if (axis_host_send[0].tvalid && axis_host_send[0].tready) host_beats++;
    if (axis_rreq_send[0].tvalid && axis_rreq_send[0].tready) net_beats++;
end

// ---- continuous arbitration invariants ----
always @(posedge aclk) if (aresetn) begin
    if (inst_dut.eng_grant && inst_dut.rx_grant) begin
        errors++;
        $display("FAIL: both grants high");
    end
    if (inst_dut.inst_loom_engine.busy && inst_dut.inst_loom_rx.busy) begin
        errors++;
        $display("FAIL: engine and rx busy simultaneously");
    end
end

// ---- pull feeder: answer engine sq_rd with beats on axis_host_recv ----
int pull_jobs[$];      // lengths
always @(posedge aclk)
    if (sq_rd.valid && sq_rd.ready) pull_jobs.push_back(int'(sq_rd.data.len));

initial forever begin
    int len, beats;
    wait (pull_jobs.size() > 0);
    len = pull_jobs.pop_front();
    beats = (len + 63) / 64;
    for (int i = 0; i < beats; i++) begin
        @(negedge aclk);
        axis_host_recv[0].tdata  = {8{64'hD0D0_0000 + 64'(i)}};
        axis_host_recv[0].tkeep  = {64{1'b1}};
        axis_host_recv[0].tvalid = 1;
        axis_host_recv[0].tlast  = (i == beats-1);
        do @(posedge aclk); while (!axis_host_recv[0].tready);
        @(negedge aclk);
        axis_host_recv[0].tvalid = 0;
        axis_host_recv[0].tlast  = 0;
    end
end

// ---- rx driver: issue a request then feed one 64 B payload beat ----
int rx_pending = 0;    // requests waiting to be issued
int rx_done = 0;       // fully forwarded

initial forever begin
    @(negedge aclk);
    if (rx_pending > 0) begin
        rq_wr.data = '0;
        rq_wr.data.pid = 6'd2;
        rq_wr.data.vaddr = 48'h7f00_0000_0000;   // staging (data-meaningless)
        rq_wr.data.len = 64;
        rq_wr.valid = 1;
        do @(posedge aclk); while (!rq_wr.ready);
        @(negedge aclk);
        rq_wr.valid = 0;
        // Inline wire message: {op WRITE_INLINE, len 8} | target VA | data
        axis_rrsp_recv[0].tdata  = '0;
        axis_rrsp_recv[0].tdata[63:0]    = {28'b0, 28'd8, 8'd2};
        axis_rrsp_recv[0].tdata[127:64]  = {16'b0, 48'h7f9e_8860_0000};
        axis_rrsp_recv[0].tdata[191:128] = 64'hEE00;
        axis_rrsp_recv[0].tkeep  = {64{1'b1}};
        axis_rrsp_recv[0].tvalid = 1;
        axis_rrsp_recv[0].tlast  = 1;
        do @(posedge aclk); while (!axis_rrsp_recv[0].tready);
        @(negedge aclk);
        axis_rrsp_recv[0].tvalid = 0;
        axis_rrsp_recv[0].tlast  = 0;
        rx_pending--;
        rx_done++;
    end
end

// ---- AXI-Lite tasks ----
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

task axil_read(input [15:0] addr, output [63:0] data);
    @(negedge aclk);
    axi_ctrl.araddr = {48'b0, addr}; axi_ctrl.arvalid = 1; axi_ctrl.rready = 1;
    do @(posedge aclk); while (!axi_ctrl.arready);
    @(negedge aclk);
    axi_ctrl.arvalid = 0;
    while (!axi_ctrl.rvalid) @(posedge aclk);
    data = axi_ctrl.rdata;
    @(negedge aclk);
endtask

localparam [47:0] STAGING_VA = 48'h7d24_8ca0_0000;   // T7: rdma route
localparam [47:0] SRC_VA_T   = 48'h7f6a_2000_0000;

task program_win(input [3:0] idx, input route,
                 input [PID_BITS-1:0] pid, input [63:0] base, input [63:0] len);
    axil_write(16'd0,  {60'b0, idx});
    axil_write(16'd8,  {62'b0, route, 1'b1});
    axil_write(16'd16, {58'b0, pid});
    axil_write(16'd24, base);
    axil_write(16'd32, len);
    axil_write(16'd40, 64'd1);
endtask

task descriptor(input [3:0] win, input [27:0] off, input [27:0] len);
    axil_write(16'd64,  {win, 32'b0, off});
    axil_write(16'd72,  64'h0000_7f6a_2000_0000);
    axil_write(16'd80,  {36'b0, len});
    axil_write(16'd88,  64'd0);
    axil_write(16'd104, 64'd0);          // fence VA = 0: no completion
    axil_write(16'd96,  64'd1);
endtask

// descriptor() with a per-descriptor fence VA
task descriptor_compl(input [3:0] win, input [27:0] off, input [27:0] len,
                      input [63:0] compl);
    axil_write(16'd64,  {win, 32'b0, off});
    axil_write(16'd72,  {16'b0, SRC_VA_T});
    axil_write(16'd80,  {36'b0, len});
    axil_write(16'd88,  64'd1);
    axil_write(16'd104, compl);
    axil_write(16'd96,  64'd1);
endtask

task wait_quiesce();
    int idle;
    idle = 0;
    while (idle < 30) begin
        @(posedge aclk);
        if (!inst_dut.inst_loom_engine.busy && !inst_dut.inst_loom_rx.busy &&
            inst_dut.fifo_empty && rx_pending == 0)
            idle++;
        else
            idle = 0;
    end
endtask

localparam [47:0] BASE_B = 48'h7f1b_d420_0000;

logic [63:0] rdata;
int n_stores, n_descs, n_rx, desc_beats;
req_t r;

initial begin
    axi_ctrl.awvalid = 0; axi_ctrl.wvalid = 0; axi_ctrl.arvalid = 0;
    axi_ctrl.bready = 0; axi_ctrl.rready = 0;
    axi_ctrl.awaddr = 0; axi_ctrl.wdata = 0; axi_ctrl.wstrb = 0; axi_ctrl.araddr = 0;

    repeat (5) @(negedge aclk);
    aresetn = 1;
    repeat (2) @(negedge aclk);

    // Windows 1, 2 local; completion disabled (COMPL_VA = 0)
    program_win(4'd1, 1'b0, 6'd1, {16'b0, BASE_B}, 64'h40_0000);
    program_win(4'd2, 1'b0, 6'd2, {16'b0, BASE_B + 48'h100_0000}, 64'h40_0000);
    axil_write(16'd128, 64'h7f00_0000_0000);   // staging vaddr (rx dispatch)

    // --- T1: smoke through the full top ---
    axil_write(16'h1040, 64'h1111);
    wait_quiesce();
    check(wrq.size() == 1 && wrq[0].vaddr == BASE_B + 48'h40 && wrq[0].pid == 6'd1,
          "T1: store through wrapper");
    check(host_beats == 1, "T1: one host beat");
    wrq.delete(); host_beats = 0;

    // --- T2: rx arrives while engine mid-DMA -> rx waits ---
    descriptor(4'd1, 28'h100, 28'd192);          // 3 beats; feeder will supply
    rx_pending++;                     // race the rx request in
    wait_quiesce();
    // order in wrq: DMA write first, rx write after
    check(wrq.size() == 2, "T2: two wr_reqs");
    if (wrq.size() == 2) begin
        check(wrq[0].vaddr == BASE_B + 48'h100 && wrq[0].len == 192,
              "T2: DMA wr first");
        check(wrq[1].vaddr == 48'h7f9e_8860_0000 && wrq[1].pid == 6'd2,
              "T2: rx wr second");
    end
    check(rx_done == 1, "T2: rx completed");
    wrq.delete(); rdq.delete(); host_beats = 0;

    // --- T3: engine store while rx in flight -> engine waits; then serves ---
    rx_pending++;
    axil_write(16'h2040, 64'h3333);               // store win 2 racing rx
    wait_quiesce();
    check(wrq.size() == 2, "T3: two wr_reqs");
    check(rx_done == 2, "T3: rx completed");
    // whichever went first, both transactions must be complete and atomic
    wrq.delete(); host_beats = 0;

    // --- T4: starvation recovery under a store burst ---
    rx_pending++;
    for (int i = 0; i < 10; i++)
        axil_write(16'h1000 + 16'(8*i), 64'(i));
    wait_quiesce();
    check(rx_done == 3, "T4: rx served after store burst");
    check(wrq.size() == 11, "T4: 10 stores + 1 rx write");
    wrq.delete(); host_beats = 0;

    // --- T5: mixed soak with exact accounting ---
    n_stores = 0; n_descs = 0; n_rx = 0; desc_beats = 0;
    for (int i = 0; i < 40; i++) begin
        case (i % 4)
            0, 1: begin
                axil_write(16'((((i % 2) + 1) << 12) | (8 * (i % 512))), 64'(i));
                n_stores++;
            end
            2: begin
                descriptor(4'd1, 28'(64 * i), 28'(64 * (1 + i % 4)));
                n_descs++; desc_beats += (1 + i % 4);
            end
            3: begin
                rx_pending++;
                n_rx++;
            end
        endcase
    end
    wait_quiesce();
    check(rx_done == 3 + n_rx, "T5: all rx served");
    check(wrq.size() == n_stores + n_descs + n_rx,
          $sformatf("T5: wr_req count %0d, expected %0d",
                    wrq.size(), n_stores + n_descs + n_rx));
    check(host_beats == n_stores + desc_beats + n_rx,
          $sformatf("T5: host beats %0d, expected %0d",
                    host_beats, n_stores + desc_beats + n_rx));
    check(net_beats == 0, "T5: nothing on net (all local)");

    // Counters (completion disabled -> local_wr = stores + descs)
    axil_read(16'((32 + 0) * 8), rdata);
    check(rdata == 64'(1 + 1 + 10 + n_stores), $sformatf("T5: stores counter %0d", rdata));
    axil_read(16'((32 + 2) * 8), rdata);
    check(rdata == 64'(1 + 1 + 1 + 10 + n_stores + n_descs),
          $sformatf("T5: local_wr counter %0d", rdata));
    axil_read(16'((32 + 4) * 8), rdata);
    check(rdata == 64'(3 + n_rx), $sformatf("T5: rx_fwd counter %0d", rdata));
    axil_read(16'((32 + 5) * 8), rdata);
    check(rdata == 64'd0, "T5: no drops");
    axil_read(16'((32 + 6) * 8), rdata);
    check(rdata == 64'd0, "T5: no overflow");

    // --- T6: aperture read through the wrapper (feeder answers the pull) ---
    begin
        logic [63:0] rd_out;
        axil_read(16'h1040, rd_out);   // feeder beat data: all lanes D0D0_0000
        check(rd_out == 64'hD0D0_0000, $sformatf("T6: read got %h", rd_out));
        axil_read(16'((32 + 8) * 8), rdata);
        check(rdata == 64'd1, "T6: reads-captured counter");
    end

    // --- T7: rdma route through the wrapper ---
    // Everything above is local route, so the top-level rdma path - the
    // engine's net stream out through vfpga_top's muxes while the arbiter
    // holds the pair - had never carried a beat. On the importer side of
    // the two-host setup every window is rdma routed, so this is that
    // host's entire data plane
    begin
        int net_before, host_before;
        net_before  = net_beats;
        host_before = host_beats;
        wrq.delete();

        axil_write(16'd128, {16'b0, STAGING_VA});          // staging CSR
        program_win(4'd3, 1'b1, 6'd4, {16'b0, BASE_B}, 64'h40_0000);

        axil_write(16'h3040, 64'hFEED_0000_0000_0001);     // store -> message
        wait_quiesce();
        check(net_beats == net_before + 1,
              "T7: rdma store put one message beat on the net stream");
        check(host_beats == host_before, "T7: rdma store touched no host beat");
        check(wrq.size() == 1 && wrq[0].strm == STRM_RDMA &&
              wrq[0].vaddr == STAGING_VA && wrq[0].len == 64,
              "T7: rdma store request is a 64 B write at the staging vaddr");

        // Bulk on the rdma route: pulled from the host, pushed to the net,
        // with the fence landing back on the host stream
        wrq.delete();
        net_before  = net_beats;
        host_before = host_beats;
        descriptor_compl(4'd3, 28'h1000, 28'd256, {16'b0, BASE_B + 48'h3000});
        wait_quiesce();
        check(net_beats == net_before + 4,
              "T7: rdma bulk streamed its 4 beats to the net");
        check(host_beats == host_before + 1, "T7: fence beat went to the host");
        check(wrq.size() == 2 && wrq[0].strm == STRM_RDMA &&
              wrq[0].vaddr == BASE_B + 48'h1000 && wrq[0].len == 256 &&
              wrq[1].strm == STRM_HOST && wrq[1].len == 8,
              "T7: bulk request goes out rdma, fence stays local");
    end

    if (errors == 0) $display("TB PASS (tb_loom_top)");
    else             $display("TB FAIL (tb_loom_top): %0d errors", errors);
    $finish;
end

initial begin
    #2ms;
    $display("TB FAIL (tb_loom_top): timeout");
    $finish;
end

endmodule
