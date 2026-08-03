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

logic                  fifo_empty, fifo_is_desc;
logic [3:0]            fifo_win;
logic [27:0]           fifo_off, fifo_len;
logic [PID_BITS-1:0]   fifo_src_pid;
logic [VADDR_BITS-1:0] fifo_compl_va;
logic [63:0]           fifo_payload;
logic                  fifo_pop;

logic cnt_local_wr, cnt_rdma_wr, cnt_rx_fwd, cnt_drop, cnt_compl;

int errors = 0;
int commit_pulses = 0;

loom_ctrl dut (
    .aclk(aclk), .aresetn(aresetn), .axi_ctrl(axi_ctrl),
    .tbl_commit(tbl_commit), .tbl_idx(tbl_idx), .tbl_valid(tbl_valid),
    .tbl_route(tbl_route), .tbl_pid(tbl_pid), .tbl_base(tbl_base),
    .tbl_len(tbl_len),
    .fifo_empty(fifo_empty), .fifo_is_desc(fifo_is_desc),
    .fifo_win(fifo_win), .fifo_off(fifo_off), .fifo_len(fifo_len),
    .fifo_src_pid(fifo_src_pid), .fifo_compl_va(fifo_compl_va),
    .fifo_payload(fifo_payload), .fifo_pop(fifo_pop),
    .cnt_local_wr(cnt_local_wr), .cnt_rdma_wr(cnt_rdma_wr),
    .cnt_rx_fwd(cnt_rx_fwd), .cnt_drop(cnt_drop), .cnt_compl(cnt_compl)
);

always @(posedge aclk) if (tbl_commit) commit_pulses++;

// CSR word indices (byte address = idx * 8)
localparam int R_TBL_IDX = 0, R_TBL_CFG = 1, R_TBL_PID = 2, R_TBL_BASE = 3,
               R_TBL_LEN = 4, R_TBL_COMMIT = 5, R_DMA_DST = 8,
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
    cnt_local_wr = 0; cnt_rdma_wr = 0; cnt_rx_fwd = 0; cnt_drop = 0; cnt_compl = 0;

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
        int idx_list[10] = '{0, 1, 2, 3, 4, 8, 9, 10, 11, 13};
        foreach (idx_list[j]) begin
            axil_write(16'(idx_list[j] * 8), 64'hC0DE_0000_0000_0000 + 64'(idx_list[j]));
        end
        foreach (idx_list[j]) begin
            axil_read(16'(idx_list[j] * 8), rdata);
            check(rdata == 64'hC0DE_0000_0000_0000 + 64'(idx_list[j]),
                  $sformatf("readback reg %0d", idx_list[j]));
        end
    end

    if (errors == 0) $display("TB PASS (tb_loom_ctrl)");
    else             $display("TB FAIL (tb_loom_ctrl): %0d errors", errors);
    $finish;
end

endmodule
