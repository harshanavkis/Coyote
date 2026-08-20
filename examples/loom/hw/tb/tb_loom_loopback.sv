`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_loom_loopback — the two-host data plane in one simulation.
 *
 * Client half (real loom_ctrl + loom_table + loom_engine, wired as in
 * vfpga_top) drives its rdma-route windows; every outgoing rdma request
 * and its beats are handed to a model of the shell's RDMA path, which
 * fragments them at PMTU exactly the way ib_transport_protocol does, and
 * the far side (real loom_rx) lands them in a modelled host memory.
 *
 * Why this exists on top of the per-module TBs: loom_engine's header
 * ENCODE and loom_rx's header DECODE are two hand-written bit layouts in
 * two files held together by a "keep in sync" comment, and each block TB
 * checks its own side against literals it writes itself - so both can
 * agree on the same misunderstanding and still pass. Here the bits only
 * ever exist as the engine emits them, and the check is what shows up in
 * far-side memory. The sequence is loom_host's, operation for operation.
 *
 * Shell model (from ib_transport_protocol.cpp + req_t in lynx_pkg):
 *   - a request of len L becomes ceil(L/PMTU) rq_wr's, vaddr advancing by
 *     PMTU (RETH on the first, MSN cursor after that);
 *   - rq_wr.last is high only on the final fragment, and the payload
 *     stream carries exactly one tlast, at the end of the whole message.
 */
module tb_loom_loopback;

localparam int PMTU = 4096;

logic aclk = 0;
logic aresetn = 0;
always #2 aclk = ~aclk;

AXI4L axi_ctrl (.aclk(aclk), .aresetn(aresetn));

// ---- client half: ctrl <-> table <-> engine ----
logic                  tbl_commit;
logic [3:0]            tbl_idx;
logic                  tbl_valid, tbl_route;
logic [PID_BITS-1:0]   tbl_pid;
logic [VADDR_BITS-1:0] tbl_base;
logic [LEN_BITS-1:0]   tbl_len;

logic                  fifo_empty, fifo_is_desc, fifo_is_read, fifo_pop;
logic [3:0]            fifo_win;
logic [27:0]           fifo_off, fifo_len;
logic [PID_BITS-1:0]   fifo_src_pid;
logic [VADDR_BITS-1:0] fifo_compl_va;
logic [63:0]           fifo_payload;

logic [3:0]            lu_idx;
logic                  lu_valid, lu_route;
logic [PID_BITS-1:0]   lu_pid;
logic [VADDR_BITS-1:0] lu_base;
logic [LEN_BITS-1:0]   lu_len;

logic cnt_local_wr, cnt_rdma_wr, cnt_drop, cnt_compl;
logic [63:0] rd_resp_data;
logic        rd_resp_valid;
logic [VADDR_BITS-1:0] rdma_staging_va;
logic [63:0] stage_acc [7];
logic [63:0] stage_cnt [7];

req_t eng_rd_req, eng_wr_req;
logic eng_rd_valid, eng_wr_valid;
logic eng_rd_ready = 1, eng_wr_ready = 1;

logic [AXI_DATA_BITS-1:0]   pull_tdata = 0;
logic [AXI_DATA_BITS/8-1:0] pull_tkeep = 0;
logic pull_tvalid = 0, pull_tready, pull_tlast = 0;

logic [AXI_DATA_BITS-1:0]   m_host_tdata, m_net_tdata;
logic [AXI_DATA_BITS/8-1:0] m_host_tkeep, m_net_tkeep;
logic m_host_tvalid, m_net_tvalid, m_host_tlast, m_net_tlast;
logic m_host_tready = 1, m_net_tready = 1;
logic eng_busy;

int errors = 0;

loom_ctrl inst_ctrl (
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
    .cnt_rx_fwd(1'b0), .cnt_rx_drop(1'b0), .cnt_drop(cnt_drop),
    .cnt_compl(cnt_compl),
    .stage_acc(stage_acc), .stage_cnt(stage_cnt)
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
    .fifo_is_read(fifo_is_read),
    .fifo_win(fifo_win), .fifo_off(fifo_off), .fifo_len(fifo_len),
    .fifo_src_pid(fifo_src_pid), .fifo_compl_va(fifo_compl_va),
    .fifo_payload(fifo_payload), .fifo_pop(fifo_pop),
    .lu_idx(lu_idx), .lu_valid(lu_valid), .lu_route(lu_route),
    .lu_pid(lu_pid), .lu_base(lu_base), .lu_len(lu_len),
    .rdma_staging_va(rdma_staging_va),
    .rd_req(eng_rd_req), .rd_valid(eng_rd_valid), .rd_ready(eng_rd_ready),
    .wr_req(eng_wr_req), .wr_valid(eng_wr_valid), .wr_ready(eng_wr_ready),
    .s_tdata(pull_tdata), .s_tkeep(pull_tkeep), .s_tvalid(pull_tvalid),
    .s_tready(pull_tready), .s_tlast(pull_tlast),
    .m_host_tdata(m_host_tdata), .m_host_tkeep(m_host_tkeep),
    .m_host_tvalid(m_host_tvalid), .m_host_tready(m_host_tready),
    .m_host_tlast(m_host_tlast),
    .m_net_tdata(m_net_tdata), .m_net_tkeep(m_net_tkeep),
    .m_net_tvalid(m_net_tvalid), .m_net_tready(m_net_tready),
    .m_net_tlast(m_net_tlast),
    .rd_resp_data(rd_resp_data), .rd_resp_valid(rd_resp_valid),
    .cnt_local_wr(cnt_local_wr), .cnt_rdma_wr(cnt_rdma_wr),
    .cnt_drop(cnt_drop), .cnt_compl(cnt_compl),
    .stage_acc(stage_acc), .stage_cnt(stage_cnt),
    .busy(eng_busy)
);

// ---- far side: real loom_rx ----
req_t rx_rq_req;
logic rx_rq_valid = 0, rx_rq_ready;
req_t rx_wr_req;
logic rx_wr_valid;
logic rx_wr_ready = 1;

logic [AXI_DATA_BITS-1:0]   rx_s_tdata = 0;
logic [AXI_DATA_BITS/8-1:0] rx_s_tkeep = 0;
logic rx_s_tvalid = 0, rx_s_tready, rx_s_tlast = 0;

logic [AXI_DATA_BITS-1:0]   rx_m_tdata;
logic [AXI_DATA_BITS/8-1:0] rx_m_tkeep;
logic rx_m_tvalid, rx_m_tlast;
logic rx_m_tready = 1;
logic rx_req_arb, rx_busy, rx_fwd, rx_drop;

localparam [47:0] STAGING  = 48'h7d24_8ca0_0000;   // exporter's staging VA
localparam [47:0] BASE1    = 48'h7f1b_d420_0000;   // exported dst1
localparam [47:0] BASE2    = 48'h7f9e_8860_0000;   // exported dst2
localparam [47:0] SRC_VA   = 48'h7f6a_2000_0000;   // importer's source buffer
localparam [47:0] CPL_VA   = 48'h7f6a_3000_0000;   // importer's fence word
localparam [PID_BITS-1:0] QP_OWNER = 6'd1;         // exporter's data ctid

loom_rx inst_rx (
    .aclk(aclk), .aresetn(aresetn),
    .rq_req(rx_rq_req), .rq_valid(rx_rq_valid), .rq_ready(rx_rq_ready),
    .rdma_staging_va(STAGING),
    .wr_req(rx_wr_req), .wr_valid(rx_wr_valid), .wr_ready(rx_wr_ready),
    .s_tdata(rx_s_tdata), .s_tkeep(rx_s_tkeep), .s_tvalid(rx_s_tvalid),
    .s_tready(rx_s_tready), .s_tlast(rx_s_tlast),
    .m_tdata(rx_m_tdata), .m_tkeep(rx_m_tkeep), .m_tvalid(rx_m_tvalid),
    .m_tready(rx_m_tready), .m_tlast(rx_m_tlast),
    .req(rx_req_arb), .grant(1'b1), .busy(rx_busy),
    .cnt_rx_fwd(rx_fwd), .cnt_rx_drop(rx_drop)
);

// -------------------------------------------------------------------------
// Source buffer contents: the pattern loom_host writes into src
// -------------------------------------------------------------------------
function automatic logic [63:0] src_word(input int idx);
    src_word = {32'h5A5A_0000, 32'(idx)};
endfunction

// -------------------------------------------------------------------------
// Pull responder: answers every sq_rd the way the shell's host-read path
// would, with the source buffer's contents
// -------------------------------------------------------------------------
req_t pull_req;
int   pull_beats;
initial forever begin
    @(posedge aclk);
    if (eng_rd_valid && eng_rd_ready) begin
        pull_req   = eng_rd_req;
        pull_beats = (pull_req.len + 63) / 64;
        for (int b = 0; b < pull_beats; b++) begin
            @(negedge aclk);
            for (int l = 0; l < 8; l++)
                pull_tdata[64*l +: 64] =
                    src_word(int'((pull_req.vaddr - SRC_VA) >> 3) + b*8 + l);
            pull_tkeep  = {64{1'b1}};
            pull_tvalid = 1;
            pull_tlast  = (b == pull_beats - 1);
            do @(posedge aclk); while (!pull_tready);
        end
        @(negedge aclk);
        pull_tvalid = 0; pull_tlast = 0;
    end
end

// -------------------------------------------------------------------------
// Shell model: the engine's rdma requests and their beats become RoCE
// packets on the far side. Requests and payload are captured first so the
// fragmenter can present them with the shell's framing.
// -------------------------------------------------------------------------
typedef struct { logic [47:0] vaddr; logic [27:0] len; } wreq_t;
wreq_t net_reqs[$];
logic [AXI_DATA_BITS-1:0] net_beats[$];
int fences = 0;

// Beat accounting. A request claims `len` bytes; the engine must put
// exactly ceil(len/64) beats on the net stream for it and no more. On
// hardware a mismatch does not fail loudly - the shell simply pairs the
// next transaction's beat with this request, so an inline store message
// was seen landing verbatim inside a bulk destination, header and all, and
// every pairing after it is shifted.
int net_owed = 0;                      // beats the posted requests still owe
int net_extra = 0;                     // beats nobody asked for

always @(posedge aclk) begin
    if (eng_wr_valid && eng_wr_ready) begin
        if (eng_wr_req.strm == STRM_RDMA) begin
            net_reqs.push_back('{eng_wr_req.vaddr, eng_wr_req.len});
            net_owed += int((eng_wr_req.len + 63) / 64);
        end else
            fences++;                       // local completion write
    end
    if (m_net_tvalid && m_net_tready) begin
        net_beats.push_back(m_net_tdata);
        if (net_owed > 0) net_owed--;
        else              net_extra++;      // a beat outside any request
    end
end

// Deliver one message: ceil(len/PMTU) fragments, vaddr advancing, last only
// on the final one, and exactly one tlast at the very end of the payload
task deliver_next();
    wreq_t w;
    int nfrag, frag_len, beats, bi, guard;
    w = net_reqs.pop_front();
    nfrag = (w.len + PMTU - 1) / PMTU;
    for (int f = 0; f < nfrag; f++) begin
        frag_len = (f == nfrag - 1) ? (w.len - f*PMTU) : PMTU;
        @(negedge aclk);
        rx_rq_req = '0;
        rx_rq_req.pid   = QP_OWNER;
        rx_rq_req.vaddr = w.vaddr + f*PMTU;
        rx_rq_req.len   = frag_len;
        rx_rq_req.last  = (f == nfrag - 1);
        rx_rq_valid = 1;
        guard = 0;
        do begin @(posedge aclk); guard++; end
        while (!rx_rq_ready && guard < 2000);
        @(negedge aclk);
        rx_rq_valid = 0;
        if (guard >= 2000) begin
            check(1'b0, "far side never accepted the request (stuck)");
            return;
        end

        beats = (frag_len + 63) / 64;
        for (bi = 0; bi < beats; bi++) begin
            // Bounded: a far side that stops taking beats is a framing bug,
            // and it should be reported as one rather than as a timeout
            guard = 0;
            while (net_beats.size() == 0 && guard < 2000) begin
                @(posedge aclk); guard++;
            end
            if (net_beats.size() == 0) begin
                check(1'b0, "shell model: engine never produced the payload");
                return;
            end
            @(negedge aclk);
            rx_s_tdata  = net_beats.pop_front();
            rx_s_tkeep  = {64{1'b1}};
            rx_s_tvalid = 1;
            rx_s_tlast  = (f == nfrag - 1) && (bi == beats - 1);
            guard = 0;
            do begin @(posedge aclk); guard++; end
            while (!rx_s_tready && guard < 2000);
            @(negedge aclk);
            rx_s_tvalid = 0; rx_s_tlast = 0;
            if (guard >= 2000) begin
                check(1'b0, "far side stopped taking beats (stuck mid-transaction)");
                return;
            end
        end
    end
endtask

// -------------------------------------------------------------------------
// Far-side host memory: what loom_rx actually lands, and in what order
// -------------------------------------------------------------------------
logic [63:0] mem [logic [47:0]];      // keyed by vaddr >> 3
logic [47:0] writes[$];               // vaddr of every word written, in order

logic [47:0] wr_cursor;
logic [27:0] wr_left;
initial forever begin
    @(posedge aclk);
    if (rx_wr_valid && rx_wr_ready) begin
        wr_cursor = rx_wr_req.vaddr;
        wr_left   = rx_wr_req.len;
    end
    if (rx_m_tvalid && rx_m_tready) begin
        for (int l = 0; l < 8 && wr_left > 0; l++) begin
            mem[(wr_cursor + l*8) >> 3] = rx_m_tdata[64*l +: 64];
            writes.push_back(wr_cursor + l*8);
            wr_left = (wr_left >= 8) ? wr_left - 8 : 0;
        end
        wr_cursor = wr_cursor + 64;
    end
end

// -------------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------------
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

// importBuf: an rdma window at the exporter's VA, pid = local QP owner
task import_win(input [3:0] idx, input [63:0] base, input [63:0] len);
    axil_write(16'd0,  {60'b0, idx});
    axil_write(16'd8,  64'b11);                  // valid + rdma route
    axil_write(16'd16, {58'b0, QP_OWNER});
    axil_write(16'd24, base);
    axil_write(16'd32, len);
    axil_write(16'd40, 64'd1);
endtask

// releaseWindow: exactly what BundledOrchestrator writes
task release_win(input [3:0] idx);
    axil_write(16'd0,  {60'b0, idx});
    axil_write(16'd8,  64'd0);
    axil_write(16'd40, 64'd1);
endtask

task store(input [3:0] win, input [11:0] off, input [63:0] val);
    axil_write({win, off}, val);
endtask

task copy(input [3:0] win, input [27:0] off, input [63:0] src_va,
          input [27:0] len, input [63:0] compl);
    axil_write(16'd64,  {win, 32'b0, off});
    axil_write(16'd72,  src_va);
    axil_write(16'd80,  {36'b0, len});
    axil_write(16'd88,  {58'b0, QP_OWNER});
    axil_write(16'd104, compl);
    axil_write(16'd96,  64'd1);
endtask

// Run the client until it has nothing left, then hand every message over
task settle();
    int n;
    n = 0;
    while (n < 4000 && (!fifo_empty || eng_busy || net_reqs.size() > 0 ||
                        rx_busy || net_beats.size() > 0)) begin
        if (net_reqs.size() > 0 && !rx_busy) deliver_next();
        else begin @(posedge aclk); n++; end
    end
    repeat (10) @(posedge aclk);
    if (net_reqs.size() > 0 || net_beats.size() > 0)
        check(1'b0, "shell model: message left undelivered");
endtask

task check_payload(input [47:0] at, input int words, input string msg);
    bit ok;
    ok = 1;
    for (int i = 0; i < words; i++)
        if (!mem.exists((at + i*8) >> 3) || mem[(at + i*8) >> 3] != src_word(i))
            ok = 0;
    check(ok, msg);
endtask

int n_writes_before;

initial begin
    axi_ctrl.awvalid = 0; axi_ctrl.wvalid = 0; axi_ctrl.arvalid = 0;
    axi_ctrl.bready = 0; axi_ctrl.rready = 0;
    axi_ctrl.awaddr = 0; axi_ctrl.wdata = 0; axi_ctrl.wstrb = 0;
    axi_ctrl.araddr = 0;
    rx_rq_req = '0;

    repeat (5) @(negedge aclk);
    aresetn = 1;
    repeat (2) @(negedge aclk);

    // attachPeer: staging CSR from the exporter's hello, then two imports
    axil_write(16'd128, {16'b0, STAGING});
    import_win(4'd1, {16'b0, BASE1}, 64'h20_0000);
    import_win(4'd2, {16'b0, BASE2}, 64'h20_0000);

    // --- loom_host: small stores through both windows ---
    store(4'd1, 12'h40, 64'hB00B_0000_0000_0001);
    store(4'd2, 12'h40, 64'hB00B_0000_0000_0002);
    settle();
    check(mem.exists((BASE1 + 48'h40) >> 3) &&
          mem[(BASE1 + 48'h40) >> 3] == 64'hB00B_0000_0000_0001,
          "store via w1 lands at the exporter's VA");
    check(mem.exists((BASE2 + 48'h40) >> 3) &&
          mem[(BASE2 + 48'h40) >> 3] == 64'hB00B_0000_0000_0002,
          "store via w2 lands at the exporter's VA");
    check(writes.size() == 2, "stores wrote one word each, nothing else");

    // --- loom_host: bulk with fence ---
    copy(4'd1, 28'h10000, {16'b0, SRC_VA}, 28'd4096, {16'b0, CPL_VA});
    settle();
    check_payload(BASE1 + 48'h10000, 512, "bulk payload @0x10000 matches src");
    check(fences == 1, "fence 1 written locally");

    // --- loom_host: ordering copy, then the flag through the other window ---
    copy(4'd1, 28'h20000, {16'b0, SRC_VA}, 28'd4096, {16'b0, CPL_VA});
    store(4'd2, 12'h800, 64'hF1A6);
    settle();
    check_payload(BASE1 + 48'h20000, 512, "bulk payload @0x20000 matches src");
    check(mem.exists((BASE2 + 48'h800) >> 3) &&
          mem[(BASE2 + 48'h800) >> 3] == 64'hF1A6, "ordering flag lands");
    check(writes[writes.size()-1] == BASE2 + 48'h800,
          "flag is the LAST word written: payload complete before it");
    check(fences == 2, "fence 2 written locally");

    // --- loom_host: release, then a store to the released window ---
    n_writes_before = writes.size();
    release_win(4'd2);
    store(4'd2, 12'h40, 64'hDEAD);
    settle();
    check(writes.size() == n_writes_before,
          "store to a released window never reaches the far side");

    // --- beyond loom_host: a copy larger than PMTU, so the shell
    //     fragments it into FIRST/MIDDLE/LAST (rq_wr.last low on all but
    //     the last, one tlast for the whole message). T2 and 6.2b get
    //     here the moment a transfer exceeds 4 KB
    copy(4'd1, 28'h30000, {16'b0, SRC_VA}, 28'd12288, {16'b0, CPL_VA});
    settle();
    check_payload(BASE1 + 48'h30000, 1536, "fragmented copy lands intact");

    // A message after a fragmented one must still parse from its own header
    store(4'd1, 12'h80, 64'hAF7E_0001);
    settle();
    check(mem.exists((BASE1 + 48'h80) >> 3) &&
          mem[(BASE1 + 48'h80) >> 3] == 64'hAF7E_0001,
          "store after a fragmented copy parses its own header");

    // --- Boundary sizes. 4096 and 12288 are both whole multiples of PMTU,
    //     so nothing above exercises the minimum transfer or an uneven final
    //     fragment - and the short last fragment is where the beat budget has
    //     to land exactly, on a request whose rq_wr.last is high but which
    //     carries a single beat
    copy(4'd1, 28'h40000, {16'b0, SRC_VA}, 28'd64, {16'b0, CPL_VA});
    settle();
    check_payload(BASE1 + 48'h40000, 8, "minimum bulk: 64 B, one beat, lands");

    copy(4'd1, 28'h50000, {16'b0, SRC_VA}, 28'd4160, {16'b0, CPL_VA});
    settle();
    check_payload(BASE1 + 48'h50000, 520,
                  "PMTU+64 copy lands (short final fragment)");
    check(fences == 5, "every copy released its fence");

    store(4'd1, 12'hC0, 64'hB0DA_0001);
    settle();
    check(mem.exists((BASE1 + 48'hC0) >> 3) &&
          mem[(BASE1 + 48'hC0) >> 3] == 64'hB0DA_0001,
          "store after an unevenly fragmented copy parses its own header");

    check(rx_drop === 1'b0, "no header was ever rejected on the far side");

    // The engine must have delivered exactly what its requests claimed
    check(net_extra == 0,
          $sformatf("no beat outside a request (%0d extra)", net_extra));
    check(net_owed == 0,
          $sformatf("no request left short of payload (%0d owed)", net_owed));

    if (errors == 0) $display("TB PASS (tb_loom_loopback)");
    else             $display("TB FAIL (tb_loom_loopback): %0d errors", errors);
    $finish;
end

initial begin
    #2ms;
    $display("TB FAIL (tb_loom_loopback): timeout");
    $finish;
end

endmodule
