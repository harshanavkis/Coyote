`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_loom_ctrl — block-level test for the ctrl slave:
 * CSR write/readback, table-programming pulse, aperture capture into the
 * order FIFO, descriptor enqueue, arrival ordering, overflow drop, debug
 * counters.
 */
module tb_loom_ctrl;

logic aclk = 0;
logic aresetn = 0;
always #2 aclk = ~aclk;

AXI4L axi_ctrl (.aclk(aclk), .aresetn(aresetn));

// DUT engine-side ports
logic                  tbl_commit;
logic [3:0]            tbl_idx;
logic                  tbl_valid, tbl_route;
logic [PID_BITS-1:0]   tbl_pid;
logic [VADDR_BITS-1:0] tbl_base;
logic [LEN_BITS-1:0]   tbl_len;

logic                  fifo_empty, fifo_is_desc, fifo_is_read;
logic [3:0]            fifo_win;
logic [27:0]           fifo_off, fifo_len;
logic [PID_BITS-1:0]   fifo_src_pid;
logic [VADDR_BITS-1:0] fifo_compl_va;
logic [63:0]           fifo_payload;
logic                  fifo_pop;
logic [63:0]           rd_resp_data;
logic                  rd_resp_valid;
logic [VADDR_BITS-1:0] rdma_staging_va;

logic cnt_local_wr, cnt_rdma_wr, cnt_rx_fwd, cnt_rx_drop, cnt_drop, cnt_compl;
logic cnt_rx_stall = 0;

// Stage counter inputs: driven with distinct constants so the CSR read
// mux (words 50-63) can be checked without an engine
logic [63:0] stage_acc [7];
logic [63:0] stage_cnt [7];

int errors = 0;
int commit_pulses = 0;

loom_ctrl dut (
    .aclk(aclk), .aresetn(aresetn), .axi_ctrl(axi_ctrl),
    .tbl_commit(tbl_commit), .tbl_idx(tbl_idx), .tbl_valid(tbl_valid),
    .tbl_route(tbl_route), .tbl_pid(tbl_pid), .tbl_base(tbl_base),
    .tbl_len(tbl_len),
    .fifo_empty(fifo_empty), .fifo_is_desc(fifo_is_desc),
    .fifo_is_read(fifo_is_read),
    .fifo_win(fifo_win), .fifo_off(fifo_off), .fifo_len(fifo_len),
    .fifo_src_pid(fifo_src_pid), .fifo_compl_va(fifo_compl_va),
    .fifo_payload(fifo_payload), .fifo_pop(fifo_pop),
    .rdma_staging_va(rdma_staging_va),
    .rd_resp_data(rd_resp_data), .rd_resp_valid(rd_resp_valid),
    .cnt_local_wr(cnt_local_wr), .cnt_rdma_wr(cnt_rdma_wr),
    .cnt_rx_fwd(cnt_rx_fwd), .cnt_rx_drop(cnt_rx_drop),
    .cnt_drop(cnt_drop), .cnt_compl(cnt_compl),
    .cnt_rx_stall(cnt_rx_stall),
    .stage_acc(stage_acc), .stage_cnt(stage_cnt)
);

always @(posedge aclk) if (tbl_commit) commit_pulses++;

// CSR word indices (byte address = idx * 8)
localparam int R_TBL_IDX = 0, R_TBL_CFG = 1, R_TBL_PID = 2, R_TBL_BASE = 3,
               R_TBL_LEN = 4, R_TBL_COMMIT = 5, R_DMA_DST = 8,
               R_RDMA_STAGING_TB = 16,
               R_RX_STALL_TB = 44, R_RX_STALL_MAX_TB = 24,
               R_DMA_SRC_VA = 9, R_DMA_LEN = 10, R_DMA_SRC_PID = 11,
               R_DMA_TRIGGER = 12, R_DMA_COMPL_VA = 13, R_DBG = 32;

task check(input bit cond, input string msg);
    if (!cond) begin
        errors++;
        $display("FAIL: %s", msg);
    end
endtask

// AXI4-Lite write (aw+w together, as the XDMA bridge presents them)
task axil_write(input [15:0] addr, input [63:0] data, input [7:0] strb = 8'hFF);
    @(negedge aclk);
    axi_ctrl.awaddr  = {48'b0, addr};
    axi_ctrl.awvalid = 1;
    axi_ctrl.wdata   = data;
    axi_ctrl.wstrb   = strb;
    axi_ctrl.wvalid  = 1;
    axi_ctrl.bready  = 1;
    do @(posedge aclk); while (!(axi_ctrl.awready && axi_ctrl.wready));
    @(negedge aclk);
    axi_ctrl.awvalid = 0;
    axi_ctrl.wvalid  = 0;
    while (!axi_ctrl.bvalid) @(posedge aclk);
    @(negedge aclk);
endtask

task axil_read(input [15:0] addr, output [63:0] data);
    @(negedge aclk);
    axi_ctrl.araddr  = {48'b0, addr};
    axi_ctrl.arvalid = 1;
    axi_ctrl.rready  = 1;
    do @(posedge aclk); while (!axi_ctrl.arready);
    @(negedge aclk);
    axi_ctrl.arvalid = 0;
    while (!axi_ctrl.rvalid) @(posedge aclk);
    data = axi_ctrl.rdata;
    @(negedge aclk);
endtask

task pop_one();
    @(negedge aclk);
    fifo_pop = 1;
    @(negedge aclk);
    fifo_pop = 0;
endtask

logic [63:0] rdata;

initial begin
    axi_ctrl.awvalid = 0; axi_ctrl.wvalid = 0; axi_ctrl.arvalid = 0;
    axi_ctrl.bready = 0; axi_ctrl.rready = 0;
    axi_ctrl.awaddr = 0; axi_ctrl.wdata = 0; axi_ctrl.wstrb = 0; axi_ctrl.araddr = 0;
    fifo_pop = 0;
    rd_resp_data = 0; rd_resp_valid = 0;
    cnt_local_wr = 0; cnt_rdma_wr = 0; cnt_rx_fwd = 0; cnt_rx_drop = 0;
    cnt_drop = 0; cnt_compl = 0;
    for (int i = 0; i < 7; i++) begin
        stage_acc[i] = 64'hA000 + 64'(i);
        stage_cnt[i] = 64'hC000 + 64'(i);
    end

    repeat (5) @(negedge aclk);
    aresetn = 1;
    repeat (2) @(negedge aclk);

    // --- 1. CSR write/readback ---
    axil_write(16'(R_TBL_BASE * 8), 64'h0000_7f1b_d420_0000);
    axil_read(16'(R_TBL_BASE * 8), rdata);
    check(rdata == 64'h0000_7f1b_d420_0000, "TBL_BASE readback");

    // --- 2. Table programming outputs + commit pulse ---
    axil_write(16'(R_TBL_IDX * 8), 64'd3);
    axil_write(16'(R_TBL_CFG * 8), 64'b11);           // valid | rdma
    axil_write(16'(R_TBL_PID * 8), 64'd5);
    axil_write(16'(R_TBL_LEN * 8), 64'h40_0000);
    check(commit_pulses == 0, "commit fired before COMMIT write");
    axil_write(16'(R_TBL_COMMIT * 8), 64'd1);
    check(commit_pulses == 1, "exactly one commit pulse");
    check(tbl_idx == 4'd3 && tbl_valid && tbl_route && tbl_pid == 6'd5 &&
          tbl_base == 48'h7f1b_d420_0000 && tbl_len == 28'h40_0000,
          "table programming outputs");


    // --- 4. Aperture store capture ---
    check(fifo_empty, "FIFO not empty before first store");
    axil_write(16'h2040, 64'hDEAD_BEEF_CAFE_0001);
    check(!fifo_empty, "store not captured");
    check(!fifo_is_desc && fifo_win == 4'd2 && fifo_off == 28'h040 &&
          fifo_payload == 64'hDEAD_BEEF_CAFE_0001 && fifo_len[7:0] == 8'hFF,
          "store entry fields");
    pop_one();
    check(fifo_empty, "FIFO not empty after pop");

    // --- 5. Descriptor enqueue ---
    axil_write(16'(R_DMA_DST * 8),     {4'd1, 32'b0, 28'h001_0000});
    axil_write(16'(R_DMA_SRC_VA * 8),  64'h0000_7f6a_2000_0000);
    axil_write(16'(R_DMA_LEN * 8),     64'h4_0000);
    axil_write(16'(R_DMA_SRC_PID * 8), 64'd2);
    axil_write(16'(R_DMA_COMPL_VA * 8), 64'h0000_7f6a_3000_0000);
    check(fifo_empty, "descriptor enqueued before trigger");
    axil_write(16'(R_DMA_TRIGGER * 8), 64'd1);
    check(!fifo_empty && fifo_is_desc && fifo_win == 4'd1 &&
          fifo_off == 28'h001_0000 && fifo_len == 28'h4_0000 &&
          fifo_src_pid == 6'd2 && fifo_compl_va == 48'h7f6a_3000_0000 &&
          fifo_payload[47:0] == 48'h7f6a_2000_0000,
          "descriptor entry fields");
    pop_one();

    // --- 6. Arrival ordering: store, desc, store ---
    axil_write(16'h1008, 64'h11);                       // store win 1
    axil_write(16'(R_DMA_TRIGGER * 8), 64'd1);          // desc (regs still staged)
    axil_write(16'h3010, 64'h22);                       // store win 3
    check(!fifo_is_desc && fifo_win == 4'd1 && fifo_payload == 64'h11,
          "order[0] should be store win1");
    pop_one();
    check(fifo_is_desc && fifo_win == 4'd1, "order[1] should be descriptor");
    pop_one();
    check(!fifo_is_desc && fifo_win == 4'd3 && fifo_payload == 64'h22,
          "order[2] should be store win3");
    pop_one();
    check(fifo_empty, "FIFO not empty after order test");

    // --- 7. Overflow: fill 64, next two dropped ---
    for (int i = 0; i < 66; i++)
        axil_write(16'h4000 + 16'(8 * (i % 512)), 64'(i));
    axil_read(16'((R_DBG + 6) * 8), rdata);             // overflow counter
    check(rdata == 64'd2, $sformatf("overflow drops = %0d, expected 2", rdata));
    for (int i = 0; i < 64; i++) begin
        check(!fifo_empty && fifo_payload == 64'(i), $sformatf("drain[%0d]", i));
        pop_one();
    end
    check(fifo_empty, "FIFO not empty after drain");

    // --- 8. Debug counters ---
    axil_read(16'((R_DBG + 0) * 8), rdata);             // stores captured
    check(rdata == 64'(1 + 2 + 64), $sformatf("stores captured = %0d", rdata));
    axil_read(16'((R_DBG + 1) * 8), rdata);             // descriptors queued
    check(rdata == 64'd2, $sformatf("descs queued = %0d", rdata));
    @(negedge aclk); cnt_drop = 1; @(negedge aclk); cnt_drop = 0;
    axil_read(16'((R_DBG + 5) * 8), rdata);
    check(rdata == 64'd1, "cnt_drop counter");

    // --- 9. Sub-word wstrb capture ---
    axil_write(16'h1050, 64'hFFFF_FFFF_FFFF_FFFF, 8'h0F);
    check(!fifo_empty && fifo_len[7:0] == 8'h0F, "wstrb 0x0F captured");
    pop_one();

    // --- 10. FIFO wraparound: rolling 5-in/5-out, 30 rounds (crosses 64) ---
    begin
        int seq;
        seq = 0;
        for (int round = 0; round < 30; round++) begin
            for (int k = 0; k < 5; k++) begin
                axil_write(16'h1000 + 16'(8 * ((seq + k) % 512)), 64'(1000 + seq + k));
            end
            for (int k = 0; k < 5; k++) begin
                check(!fifo_empty && fifo_payload == 64'(1000 + seq + k),
                      $sformatf("wrap: round %0d entry %0d", round, k));
                pop_one();
            end
            seq += 5;
        end
        check(fifo_empty, "wrap: empty after rolling test");
    end

    // --- 11. Full RW CSR readback ---
    begin
        int idx_list[11] = '{0, 1, 2, 3, 4, 8, 9, 10, 11, 13, 16};
        foreach (idx_list[j]) begin
            axil_write(16'(idx_list[j] * 8), 64'hC0DE_0000_0000_0000 + 64'(idx_list[j]));
        end
        foreach (idx_list[j]) begin
            axil_read(16'(idx_list[j] * 8), rdata);
            check(rdata == 64'hC0DE_0000_0000_0000 + 64'(idx_list[j]),
                  $sformatf("readback reg %0d", idx_list[j]));
        end
    end

    // --- 12. Aperture read: held-open channel, READ FIFO entry, response ---
    begin
        // begin an AXI read to window 3 offset 0x28 without waiting rvalid
        @(negedge aclk);
        axi_ctrl.araddr = 64'h3028; axi_ctrl.arvalid = 1; axi_ctrl.rready = 1;
        do @(posedge aclk); while (!axi_ctrl.arready);
        @(negedge aclk);
        axi_ctrl.arvalid = 0;
        check(!fifo_empty && fifo_is_read && fifo_win == 4'd3 &&
              fifo_off == 28'h028, "read entry captured");
        repeat (20) @(posedge aclk);
        check(!axi_ctrl.rvalid, "read held open until response");
        pop_one();
        @(negedge aclk);
        rd_resp_data = 64'hCAFE_F00D_0000_0001; rd_resp_valid = 1;
        @(negedge aclk);
        rd_resp_valid = 0;
        while (!axi_ctrl.rvalid) @(posedge aclk);
        check(axi_ctrl.rdata == 64'hCAFE_F00D_0000_0001, "deferred read data");
        @(negedge aclk);
    end

    // --- 13. Read never dropped: arready withheld while FIFO full ---
    begin
        for (int i = 0; i < 64; i++)
            axil_write(16'h1000 + 16'(8 * (i % 512)), 64'(i));   // fill FIFO
        @(negedge aclk);
        axi_ctrl.araddr = 64'h1040; axi_ctrl.arvalid = 1; axi_ctrl.rready = 1;
        repeat (20) @(posedge aclk);
        check(!axi_ctrl.arready, "aperture read blocked while FIFO full");
        pop_one();                                               // make room
        do @(posedge aclk); while (!axi_ctrl.arready);
        @(negedge aclk);
        axi_ctrl.arvalid = 0;
        // drain the 63 stores + our read entry, answering the read
        for (int i = 0; i < 63; i++) pop_one();
        check(!fifo_empty && fifo_is_read, "read entry after the stores");
        pop_one();
        @(negedge aclk);
        rd_resp_data = 64'hCAFE_F00D_0000_0002; rd_resp_valid = 1;
        @(negedge aclk);
        rd_resp_valid = 0;
        while (!axi_ctrl.rvalid) @(posedge aclk);
        check(axi_ctrl.rdata == 64'hCAFE_F00D_0000_0002, "blocked read completes");
        @(negedge aclk);
    end

    // --- 14. Reads-captured counter (idx 40) ---
    axil_read(16'd320, rdata);
    check(rdata == 64'd2, $sformatf("reads captured = %0d, expected 2", rdata));

    // --- 15. Free-running cycle counter (word 48) ---
    begin
        logic [63:0] cyc0, cyc1;
        axil_read(16'(48 * 8), cyc0);
        repeat (10) @(posedge aclk);
        axil_read(16'(48 * 8), cyc1);
        check(cyc1 > cyc0, "cycle counter advances");
    end

    // --- 16. Queue-wait accumulator (word 49): known FIFO residency ---
    begin
        logic [63:0] q0, q1;
        axil_read(16'(49 * 8), q0);
        axil_write(16'h1040, 64'h51A6_0001);       // push one store
        repeat (40) @(posedge aclk);               // let it sit
        pop_one();
        axil_read(16'(49 * 8), q1);
        check(q1 - q0 >= 64'd40 && q1 - q0 <= 64'd120,
              $sformatf("queue-wait delta %0d, expected ~40-120", q1 - q0));
    end

    // --- 17. Stage counter read mux (words 50-63, values from ports) ---
    axil_read(16'(53 * 8), rdata);
    check(rdata == 64'hA003, "stage_acc[3] read mux");
    axil_read(16'(63 * 8), rdata);
    check(rdata == 64'hC006, "stage_cnt[6] read mux");

    // --- 18. A descriptor staging burst must not reach RDMA_STAGING_VA ---
    // A host ctrl write covers its whole 64 B line forward from the target
    // (sim/hw/ctrl_simulation.svh models it, "Write burst which happens in
    // real hardware"). While this register sat at word 14 it was inside the
    // burst of every write at word 8, so dma() zeroed it, the next store
    // went out with RETH = 0, and the far side wrote eight bytes at VA 0.
    // At word 16 it is the first word of its own line and out of reach.
    begin
        logic [47:0] staged;
        staged = 48'h7abc_1234_5000;
        axil_write(16'(R_RDMA_STAGING_TB * 8), {16'b0, staged});
        check(rdma_staging_va == staged, "18: staging vaddr programmed");

        // The burst a single dma() write produces: word 8 and the rest of
        // its line, exactly as the hardware presents it
        for (int w = R_DMA_DST; w < R_DMA_DST + 8; w++)
            axil_write(16'(w * 8), 64'h0);
        check(rdma_staging_va == staged,
              "18: staging vaddr survives a descriptor-line burst");
    end

    // 19: the stall accounting keeps a SUM and a longest unbroken RUN, and
    // they are different numbers. Only the run sizes a buffer - a FIFO
    // swallows a burst up to its depth, so three short stalls are not the
    // same problem as one long one even when they total the same.
    begin
        logic [63:0] sum, mx;
        // runs of 5, 12 and 3, separated by idle cycles: sum 20, longest 12
        for (int r = 0; r < 3; r++) begin
            int len;
            len = (r == 0) ? 5 : (r == 1) ? 12 : 3;
            repeat (len) begin
                @(negedge aclk); cnt_rx_stall = 1;
                @(posedge aclk);
            end
            @(negedge aclk); cnt_rx_stall = 0;
            repeat (4) @(posedge aclk);
        end
        @(negedge aclk);
        axil_read(16'(R_RX_STALL_TB * 8), sum);
        axil_read(16'(R_RX_STALL_MAX_TB * 8), mx);
        check(sum == 64'd20,
              $sformatf("19: stalled cycles sum to 20 (%0d)", sum));
        check(mx == 64'd12,
              $sformatf("19: longest unbroken stall is 12, not the sum (%0d)",
                        mx));
    end

    if (errors == 0) $display("TB PASS (tb_loom_ctrl)");
    else             $display("TB FAIL (tb_loom_ctrl): %0d errors", errors);
    $finish;
end

endmodule
