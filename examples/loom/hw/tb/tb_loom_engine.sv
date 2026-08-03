`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_loom_engine — composite block test: real loom_ctrl + loom_table +
 * loom_engine wired as in vfpga_top; the TB mocks the shell side (sq_rd /
 * sq_wr acceptance, pull-stream data, output-stream capture).
 *
 * Covers: local store, rdma store, DMA local, DMA rdma, completion
 * writes, invalid-window and bounds drops, and the ordering property
 * (a flag store behind a DMA descriptor is not issued until the DMA
 * stream and its completion are done).
 */
module tb_loom_engine;

logic aclk = 0;
logic aresetn = 0;
always #2 aclk = ~aclk;

AXI4L axi_ctrl (.aclk(aclk), .aresetn(aresetn));

// ctrl <-> table <-> engine wiring
logic                  tbl_commit;
logic [3:0]            tbl_idx;
logic                  tbl_valid, tbl_route;
logic [PID_BITS-1:0]   tbl_pid;
logic [VADDR_BITS-1:0] tbl_base;
logic [LEN_BITS-1:0]   tbl_len;
logic [PID_BITS-1:0]   compl_pid;
logic [VADDR_BITS-1:0] compl_va;

logic                  fifo_empty, fifo_is_desc, fifo_pop;
logic [3:0]            fifo_win;
logic [27:0]           fifo_off, fifo_len;
logic [PID_BITS-1:0]   fifo_src_pid;
logic [63:0]           fifo_payload;

logic [3:0]            lu_idx;
logic                  lu_valid, lu_route;
logic [PID_BITS-1:0]   lu_pid;
logic [VADDR_BITS-1:0] lu_base;
logic [LEN_BITS-1:0]   lu_len;

logic cnt_local_wr, cnt_rdma_wr, cnt_drop, cnt_compl;

// Shell-side mocks
req_t rd_req, wr_req;
logic rd_valid, wr_valid;
logic rd_ready = 1, wr_ready = 1;

logic [AXI_DATA_BITS-1:0]   s_tdata = 0;
logic [AXI_DATA_BITS/8-1:0] s_tkeep = 0;
logic s_tvalid = 0, s_tready, s_tlast = 0;

logic [AXI_DATA_BITS-1:0]   m_host_tdata, m_net_tdata;
logic [AXI_DATA_BITS/8-1:0] m_host_tkeep, m_net_tkeep;
logic m_host_tvalid, m_net_tvalid, m_host_tlast, m_net_tlast;
logic m_host_tready = 1, m_net_tready = 1;
logic busy;

int errors = 0;

loom_ctrl inst_ctrl (
    .aclk(aclk), .aresetn(aresetn), .axi_ctrl(axi_ctrl),
    .tbl_commit(tbl_commit), .tbl_idx(tbl_idx), .tbl_valid(tbl_valid),
    .tbl_route(tbl_route), .tbl_pid(tbl_pid), .tbl_base(tbl_base),
    .tbl_len(tbl_len),
    .compl_pid(compl_pid), .compl_va(compl_va),
    .fifo_empty(fifo_empty), .fifo_is_desc(fifo_is_desc),
    .fifo_win(fifo_win), .fifo_off(fifo_off), .fifo_len(fifo_len),
    .fifo_src_pid(fifo_src_pid), .fifo_payload(fifo_payload),
    .fifo_pop(fifo_pop),
    .cnt_local_wr(cnt_local_wr), .cnt_rdma_wr(cnt_rdma_wr),
    .cnt_rx_fwd(1'b0), .cnt_drop(cnt_drop), .cnt_compl(cnt_compl)
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
    .fifo_win(fifo_win), .fifo_off(fifo_off), .fifo_len(fifo_len),
    .fifo_src_pid(fifo_src_pid), .fifo_payload(fifo_payload),
    .fifo_pop(fifo_pop),
    .lu_idx(lu_idx), .lu_valid(lu_valid), .lu_route(lu_route),
    .lu_pid(lu_pid), .lu_base(lu_base), .lu_len(lu_len),
    .compl_pid(compl_pid), .compl_va(compl_va),
    .rd_req(rd_req), .rd_valid(rd_valid), .rd_ready(rd_ready),
    .wr_req(wr_req), .wr_valid(wr_valid), .wr_ready(wr_ready),
    .s_tdata(s_tdata), .s_tkeep(s_tkeep), .s_tvalid(s_tvalid),
    .s_tready(s_tready), .s_tlast(s_tlast),
    .m_host_tdata(m_host_tdata), .m_host_tkeep(m_host_tkeep),
    .m_host_tvalid(m_host_tvalid), .m_host_tready(m_host_tready),
    .m_host_tlast(m_host_tlast),
    .m_net_tdata(m_net_tdata), .m_net_tkeep(m_net_tkeep),
    .m_net_tvalid(m_net_tvalid), .m_net_tready(m_net_tready),
    .m_net_tlast(m_net_tlast),
    .cnt_local_wr(cnt_local_wr), .cnt_rdma_wr(cnt_rdma_wr),
    .cnt_drop(cnt_drop), .cnt_compl(cnt_compl),
    .busy(busy)
);

// ---- capture queues ----
req_t rdq[$], wrq[$];
typedef struct { logic [63:0] data; logic last; } beat_t;
beat_t hostq[$], netq[$];

always @(posedge aclk) begin
    if (rd_valid && rd_ready) rdq.push_back(rd_req);
    if (wr_valid && wr_ready) wrq.push_back(wr_req);
    if (m_host_tvalid && m_host_tready)
        hostq.push_back('{data: m_host_tdata[63:0], last: m_host_tlast});
    if (m_net_tvalid && m_net_tready)
        netq.push_back('{data: m_net_tdata[63:0], last: m_net_tlast});
end

// ---- helpers ----
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

task program_win(input [3:0] idx, input route,
                 input [PID_BITS-1:0] pid, input [63:0] base, input [63:0] len);
    axil_write(16'd0,  {60'b0, idx});
    axil_write(16'd8,  {62'b0, route, 1'b1});
    axil_write(16'd16, {58'b0, pid});
    axil_write(16'd24, base);
    axil_write(16'd32, len);
    axil_write(16'd40, 64'd1);
endtask

task descriptor(input [3:0] win, input [27:0] off, input [63:0] src_va,
                input [27:0] len, input [PID_BITS-1:0] src_pid);
    axil_write(16'd64, {win, 32'b0, off});
    axil_write(16'd72, src_va);
    axil_write(16'd80, {36'b0, len});
    axil_write(16'd88, {58'b0, src_pid});
    axil_write(16'd96, 64'd1);
endtask

// Feed the pull stream: n beats, data = base_val + beat index
task send_beats(input int n, input [63:0] base_val);
    for (int i = 0; i < n; i++) begin
        @(negedge aclk);
        s_tdata  = {448'b0, base_val + 64'(i)};
        s_tkeep  = {64{1'b1}};
        s_tvalid = 1;
        s_tlast  = (i == n-1);
        do @(posedge aclk); while (!s_tready);
        @(negedge aclk);
        s_tvalid = 0; s_tlast = 0;
    end
endtask

task wait_idle();
    do @(posedge aclk); while (busy || !fifo_empty);
    repeat (2) @(posedge aclk);
endtask

// Localparams for the example VAs
localparam [47:0] BASE_B = 48'h7f1b_d420_0000;   // win 1, local, pid 1
localparam [47:0] BASE_C = 48'h7f9e_8860_0000;   // win 2, rdma, pid 3
localparam [47:0] SRC_VA = 48'h7f6a_2000_0000;
localparam [47:0] CPL_VA = 48'h7f6a_3000_0000;

logic [63:0] rdata;
req_t r;
int base_host;

initial begin
    axi_ctrl.awvalid = 0; axi_ctrl.wvalid = 0; axi_ctrl.arvalid = 0;
    axi_ctrl.bready = 0; axi_ctrl.rready = 0;
    axi_ctrl.awaddr = 0; axi_ctrl.wdata = 0; axi_ctrl.wstrb = 0; axi_ctrl.araddr = 0;

    repeat (5) @(negedge aclk);
    aresetn = 1;
    repeat (2) @(negedge aclk);

    // Setup: two windows + completion config
    program_win(4'd1, 1'b0, 6'd1, {16'b0, BASE_B}, 64'h40_0000);
    program_win(4'd2, 1'b1, 6'd3, {16'b0, BASE_C}, 64'h40_0000);
    axil_write(16'd128, 64'd0);                    // COMPL_PID = 0
    axil_write(16'd136, {16'b0, CPL_VA});          // COMPL_VA

    // --- 1. Local store ---
    axil_write(16'h1040, 64'hDEAD_BEEF_0000_0001);
    wait_idle();
    check(wrq.size() == 1, "local store: one wr_req");
    if (wrq.size() > 0) begin
        r = wrq.pop_front();
        check(r.opcode == LOCAL_WRITE && r.strm == STRM_HOST && r.pid == 6'd1 &&
              r.vaddr == BASE_B + 48'h40 && r.len == 8 && r.last && !r.remote,
              "local store: wr_req fields");
    end
    check(hostq.size() == 1 && hostq[0].data == 64'hDEAD_BEEF_0000_0001 && hostq[0].last,
          "local store: host beat");
    hostq.delete();
    check(netq.size() == 0, "local store: nothing on net");

    // --- 2. RDMA store ---
    axil_write(16'h2040, 64'hDEAD_BEEF_0000_0002);
    wait_idle();
    check(wrq.size() == 1, "rdma store: one wr_req");
    if (wrq.size() > 0) begin
        r = wrq.pop_front();
        check(r.opcode == APP_WRITE && r.strm == STRM_RDMA && r.pid == 6'd3 &&
              r.vaddr == BASE_C + 48'h40 && r.len == 8 && r.remote && r.rdma &&
              r.actv && !r.mode,
              "rdma store: wr_req fields");
    end
    check(netq.size() == 1 && netq[0].data == 64'hDEAD_BEEF_0000_0002 && netq[0].last,
          "rdma store: net beat");
    netq.delete();
    check(hostq.size() == 0, "rdma store: nothing on host");

    // --- 3. DMA local (256 B = 4 beats) ---
    descriptor(4'd1, 28'h100, {16'b0, SRC_VA}, 28'd256, 6'd2);
    fork
        send_beats(4, 64'hAA00);
    join_none
    wait_idle();
    check(rdq.size() == 1, "dma local: one rd_req");
    if (rdq.size() > 0) begin
        r = rdq.pop_front();
        check(r.opcode == LOCAL_READ && r.strm == STRM_HOST && r.pid == 6'd2 &&
              r.vaddr == SRC_VA && r.len == 256, "dma local: rd_req fields");
    end
    check(wrq.size() == 2, "dma local: dma wr_req + completion wr_req");
    if (wrq.size() == 2) begin
        r = wrq.pop_front();
        check(r.opcode == LOCAL_WRITE && r.pid == 6'd1 &&
              r.vaddr == BASE_B + 48'h100 && r.len == 256, "dma local: wr_req fields");
        r = wrq.pop_front();
        check(r.opcode == LOCAL_WRITE && r.pid == 6'd0 && r.vaddr == CPL_VA &&
              r.len == 8, "dma local: completion wr_req fields");
    end else wrq.delete();
    // host stream: 4 data beats then 1 completion beat (value 1)
    check(hostq.size() == 5, "dma local: 4 beats + completion beat");
    if (hostq.size() == 5) begin
        for (int i = 0; i < 4; i++)
            check(hostq[i].data == 64'hAA00 + 64'(i) && (hostq[i].last == (i == 3)),
                  $sformatf("dma local: beat %0d", i));
        check(hostq[4].data == 64'd1 && hostq[4].last, "dma local: completion value 1");
    end
    hostq.delete();

    // --- 4. DMA rdma (128 B = 2 beats) ---
    descriptor(4'd2, 28'h200, {16'b0, SRC_VA}, 28'd128, 6'd0);
    fork
        send_beats(2, 64'hBB00);
    join_none
    wait_idle();
    check(rdq.size() == 1, "dma rdma: one rd_req"); rdq.delete();
    check(wrq.size() == 2, "dma rdma: dma wr_req + completion wr_req");
    if (wrq.size() == 2) begin
        r = wrq.pop_front();
        check(r.opcode == APP_WRITE && r.strm == STRM_RDMA && r.pid == 6'd3 &&
              r.vaddr == BASE_C + 48'h200 && r.len == 128 && r.remote,
              "dma rdma: wr_req fields");
        r = wrq.pop_front();
        check(r.vaddr == CPL_VA, "dma rdma: completion wr_req");
    end else wrq.delete();
    check(netq.size() == 2 && netq[0].data == 64'hBB00 && netq[1].data == 64'hBB01 &&
          netq[1].last, "dma rdma: net beats");
    netq.delete();
    check(hostq.size() == 1 && hostq[0].data == 64'd2, "dma rdma: completion value 2");
    hostq.delete();

    // --- 5. Drops: unprogrammed window; out-of-bounds descriptor ---
    axil_write(16'h5040, 64'hBAD0);                 // window 5 not programmed
    wait_idle();
    descriptor(4'd1, 28'h3F_FF00, {16'b0, SRC_VA}, 28'd512, 6'd2);  // crosses 4MB bound
    wait_idle();
    check(wrq.size() == 0 && rdq.size() == 0, "drops: no requests issued");
    check(hostq.size() == 0 && netq.size() == 0, "drops: no data");
    axil_read(16'((32 + 5) * 8), rdata);            // bounds/invalid drop counter
    check(rdata == 64'd2, $sformatf("drops: counter = %0d, expected 2", rdata));

    // --- 6. Ordering: flag store behind a DMA descriptor ---
    descriptor(4'd1, 28'h300, {16'b0, SRC_VA}, 28'd128, 6'd2);   // 2 beats, not fed yet
    axil_write(16'h1800, 64'hF1A6);                              // flag store, win 1
    repeat (50) @(posedge aclk);
    // engine must be stalled in the DMA stream: rd+wr issued, no flag write yet
    check(rdq.size() == 1 && wrq.size() == 1, "ordering: engine stalled on DMA");
    check(hostq.size() == 0, "ordering: no data before beats fed");
    rdq.delete();
    send_beats(2, 64'hCC00);
    wait_idle();
    // wrq: dma wr (popped above? no - it stayed) ... expect: [dma wr already counted], compl, flag
    check(wrq.size() == 3, "ordering: dma + completion + flag wr_reqs");
    if (wrq.size() == 3) begin
        r = wrq.pop_front();
        check(r.vaddr == BASE_B + 48'h300, "ordering: dma wr first");
        r = wrq.pop_front();
        check(r.vaddr == CPL_VA, "ordering: completion second");
        r = wrq.pop_front();
        check(r.vaddr == BASE_B + 48'h800 && r.len == 8, "ordering: flag wr last");
    end else wrq.delete();
    // host beats: 2 dma, completion (3), then flag beat
    check(hostq.size() == 4, "ordering: beat count");
    if (hostq.size() == 4) begin
        check(hostq[0].data == 64'hCC00 && hostq[1].data == 64'hCC01,
              "ordering: dma beats first");
        check(hostq[2].data == 64'd3, "ordering: completion count 3");
        check(hostq[3].data == 64'hF1A6, "ordering: flag beat last");
    end
    hostq.delete();

    // --- 7. Backpressure matrix ---
    // 7a. wr_ready stalled during a store
    wr_ready = 0;
    axil_write(16'h1040, 64'h7A01);
    repeat (20) @(posedge aclk);
    check(wrq.size() == 0, "bp: no wr_req while wr_ready low");
    @(negedge aclk); wr_ready = 1;
    wait_idle();
    check(wrq.size() == 1 && hostq.size() == 1 && hostq[0].data == 64'h7A01,
          "bp: store completes after wr_ready release");
    wrq.delete(); hostq.delete();
    // 7b. host tready stalled mid-DMA-stream
    descriptor(4'd1, 28'h400, {16'b0, SRC_VA}, 28'd128, 6'd2);
    fork send_beats(2, 64'h7B00); join_none
    do @(posedge aclk); while (hostq.size() < 1);      // first beat through
    @(negedge aclk); m_host_tready = 0;
    repeat (20) @(posedge aclk);
    check(hostq.size() == 1, "bp: stream stalled under host backpressure");
    @(negedge aclk); m_host_tready = 1;
    wait_idle();
    check(hostq.size() == 3 && hostq[1].data == 64'h7B01,
          "bp: stream + completion after release");   // 2 beats + compl
    wrq.delete(); hostq.delete(); rdq.delete();
    // 7c. net tready stalled during an rdma store
    m_net_tready = 0;
    axil_write(16'h2040, 64'h7C01);
    repeat (20) @(posedge aclk);
    check(netq.size() == 0, "bp: net beat held under backpressure");
    @(negedge aclk); m_net_tready = 1;
    wait_idle();
    check(netq.size() == 1 && netq[0].data == 64'h7C01, "bp: net beat after release");
    wrq.delete(); netq.delete();

    // --- 8. Bounds edges (window len 4MB) ---
    descriptor(4'd1, 28'h3F_FFC0, {16'b0, SRC_VA}, 28'd64, 6'd2);  // end == 4MB: pass
    fork send_beats(1, 64'h8A00); join_none
    wait_idle();
    check(wrq.size() == 2 && wrq[0].len == 64, "bounds: end==lim accepted");
    wrq.delete(); hostq.delete(); rdq.delete();
    descriptor(4'd1, 28'h3F_FFC0, {16'b0, SRC_VA}, 28'd72, 6'd2);  // end == 4MB+8: drop
    wait_idle();
    check(wrq.size() == 0 && rdq.size() == 0, "bounds: end==lim+8 dropped");
    axil_write(16'h1FF8, 64'h8B01);                                 // store end == 4KB: pass
    wait_idle();
    check(wrq.size() == 1 && wrq[0].vaddr == BASE_B + 48'hFF8, "bounds: store at window end");
    wrq.delete(); hostq.delete();

    // --- 9. Length not a multiple of 64 B ---
    descriptor(4'd1, 28'h500, {16'b0, SRC_VA}, 28'd100, 6'd2);
    fork send_beats(2, 64'h9A00); join_none
    wait_idle();
    check(wrq.size() == 2 && wrq[0].len == 100, "oddlen: wr_req len 100");
    check(hostq.size() == 3, "oddlen: 2 data beats + completion");
    wrq.delete(); hostq.delete(); rdq.delete();

    // --- 10. Completion disabled ---
    axil_write(16'd136, 64'd0);                     // COMPL_VA = 0
    descriptor(4'd1, 28'h600, {16'b0, SRC_VA}, 28'd64, 6'd2);
    fork send_beats(1, 64'hA100); join_none
    wait_idle();
    check(wrq.size() == 1, "compl-off: single wr_req");
    check(hostq.size() == 1, "compl-off: data beat only, no completion beat");
    wrq.delete(); hostq.delete(); rdq.delete();

    // --- 11. Soak with exact counter deltas (completion still disabled) ---
    begin
        logic [63:0] c_stores0, c_local0, c_drops0, c_stores1, c_local1, c_drops1;
        int exp_stores, exp_local, exp_drops;
        exp_stores = 0; exp_local = 0; exp_drops = 0;
        axil_read(16'((32+0)*8), c_stores0);
        axil_read(16'((32+2)*8), c_local0);
        axil_read(16'((32+5)*8), c_drops0);
        for (int i = 0; i < 60; i++) begin
            case (i % 4)
                0, 1: begin                          // valid store, win 1
                    axil_write(16'h1000 + 16'(8 * (i % 500)), 64'(i));
                    exp_stores++; exp_local++;
                end
                2: begin                             // valid 1-beat desc
                    descriptor(4'd1, 28'(64 * i), {16'b0, SRC_VA}, 28'd64, 6'd2);
                    fork send_beats(1, 64'(i)); join_none
                    exp_local++;
                end
                3: begin                             // invalid window
                    axil_write(16'h7000 + 16'(8 * (i % 500)), 64'(i));
                    exp_stores++; exp_drops++;
                end
            endcase
            wait_idle();                             // serialized: keeps feeder simple
        end
        axil_read(16'((32+0)*8), c_stores1);
        axil_read(16'((32+2)*8), c_local1);
        axil_read(16'((32+5)*8), c_drops1);
        check(c_stores1 - c_stores0 == 64'(exp_stores),
              $sformatf("soak: stores delta %0d, expected %0d", c_stores1 - c_stores0, exp_stores));
        check(c_local1 - c_local0 == 64'(exp_local),
              $sformatf("soak: local_wr delta %0d, expected %0d", c_local1 - c_local0, exp_local));
        check(c_drops1 - c_drops0 == 64'(exp_drops),
              $sformatf("soak: drops delta %0d, expected %0d", c_drops1 - c_drops0, exp_drops));
        wrq.delete(); hostq.delete(); netq.delete(); rdq.delete();
    end

    if (errors == 0) $display("TB PASS (tb_loom_engine)");
    else             $display("TB FAIL (tb_loom_engine): %0d errors", errors);
    $finish;
end

// Watchdog
initial begin
    #500us;
    $display("TB FAIL (tb_loom_engine): timeout");
    $finish;
end

endmodule
