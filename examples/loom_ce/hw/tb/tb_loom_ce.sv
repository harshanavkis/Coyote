`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_loom_ce - the generated wrapper (design_user_logic_c0_0, including
 * vfpga_top.svh) with the TB as the V80 shell: AXI-Lite on the CSR page,
 * sq_rd answered with card data, sq_wr and the host stream captured, cq_wr
 * completing each write.
 *
 * Reads come back in 1 KB tlast-terminated segments, so only the beat count
 * can end a copy. Covers: a copy with a fence and one without, the exact
 * requests, data and fence value; START while busy ignored; a 64 KiB copy
 * under random backpressure on every channel; CSR readback; the cycle
 * counter stopping at the data write's completion; the stall counters (48-50)
 * moving under backpressure. Gets (DIR 1: host stream -> card memory): the
 * requests, data and fence; the fence requested only after the data write
 * completed, and BUSY held until then under slow completions and
 * backpressure; a put after gets.
 * Programs (PROG_CMD): card memory is a model here (a write becomes visible
 * at its completion), so loads, WAIT polls and read-after-write show. P1 a
 * loaded program of puts, a signal and a stamp, its reads issued ahead of
 * the data; P2 run again with another RUN_BASE; P3 WAIT holds every later
 * read until the landed flag reaches its target; P4 REDUCE into card memory
 * under backpressure, then a copy of the result (read only after the
 * REDUCE's writes completed); P5 REDUCE to the host stream with a short last
 * chunk; P6 a card SIGNAL after the card writes before it completed; P7
 * ABORT ends a WAIT, a single copy after it; P8 a get then a put of what it
 * wrote; P9 32 copies of 1 KiB at 100 cycles of read latency, the latency
 * hidden by the read-ahead.
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
bit hold_host = 0;      // the host stream ready one cycle in eight

initial begin
    cq_rd.valid = 0; cq_rd.data = '0;
    axis_host_recv[0].tvalid = 0; axis_host_recv[0].tdata = '0; axis_host_recv[0].tkeep = '0;
    axis_host_recv[0].tlast = 0; axis_host_recv[0].tid = '0;
    axis_card_recv[0].tvalid = 0; axis_card_recv[0].tdata = '0; axis_card_recv[0].tkeep = '0;
    axis_card_recv[0].tlast = 0; axis_card_recv[0].tid = '0;
    notify.ready = 1;
end

always @(posedge aclk) begin
    sq_rd.ready              <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
    sq_wr.ready              <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
    axis_host_send[0].tready <= hold_host ? ($urandom_range(0, 7) == 0) : bp ? ($urandom_range(0, 2) != 0) : 1'b1;
    axis_card_send[0].tready <= bp ? ($urandom_range(0, 2) != 0) : 1'b1;
end

// Reads: each accepted sq_rd is answered with its beats on its stream (card
// or host), in 1 KB segments each ending in tlast
req_t rd_q [$], rd_seen [$];
time  rd_q_t [$];
int   rd_lat = 0;       // cycles from a read request to its first beat (P9)
always @(posedge aclk) if (aresetn && sq_rd.valid && sq_rd.ready) begin
    rd_q.push_back(sq_rd.data); rd_q_t.push_back($time);
    rd_seen.push_back(sq_rd.data);
end

initial forever begin
    req_t r;
    time  t_r;
    wait (rd_q.size() > 0);
    r = rd_q.pop_front(); t_r = rd_q_t.pop_front();
    while ($time < t_r + 4 * rd_lat) @(negedge aclk);
    // one beat a cycle (valid held across beats), gaps only under bp
    for (int k = 0; k < r.len / 64; k++) begin
        @(negedge aclk);
        if (bp) while ($urandom_range(0, 2) == 0) begin
            axis_host_recv[0].tvalid = 0; axis_card_recv[0].tvalid = 0;
            @(negedge aclk);
        end
        if (r.strm == STRM_HOST) begin
            axis_host_recv[0].tdata  = pat(r.vaddr + 64*k);
            axis_host_recv[0].tkeep  = '1;
            axis_host_recv[0].tlast  = (k % 16 == 15) || (k == r.len / 64 - 1);
            axis_host_recv[0].tvalid = 1;
            do @(posedge aclk); while (!axis_host_recv[0].tready);
        end else begin
            axis_card_recv[0].tdata  = card_rd(r.vaddr + 64*k);
            axis_card_recv[0].tkeep  = '1;
            axis_card_recv[0].tlast  = (k % 16 == 15) || (k == r.len / 64 - 1);
            axis_card_recv[0].tvalid = 1;
            do @(posedge aclk); while (!axis_card_recv[0].tready);
        end
    end
    @(negedge aclk);
    axis_host_recv[0].tvalid = 0; axis_card_recv[0].tvalid = 0;
end

// Writes: requests and beats captured (host and card streams); a write
// completes (cq_wr) cq_lat cycles after its last beat. A get's fence request
// must come after its data write's completion: the time of the last card
// write completion, and of each fence request, are kept.
req_t wr_seen [$];
logic [AXI_DATA_BITS-1:0]   h_d [$], c_d [$];
logic [AXI_DATA_BITS/8-1:0] h_k [$], c_k [$];
bit                         h_l [$], c_l [$];
int                         cq_owed = 0, cq_delay = 0, cq_lat = 20;
int                         card_owed = 0, card_done = 0;
time                        t_card_done = 0, t_fence_req = 0;
bit                         cq_card [$];

always @(posedge aclk) if (aresetn) begin
    if (sq_wr.valid && sq_wr.ready) begin
        wr_seen.push_back(sq_wr.data);
        if (sq_wr.data.len == 8) t_fence_req = $time;
    end
    if (axis_host_send[0].tvalid && axis_host_send[0].tready) begin
        h_d.push_back(axis_host_send[0].tdata); h_k.push_back(axis_host_send[0].tkeep);
        h_l.push_back(axis_host_send[0].tlast);
        if (axis_host_send[0].tlast) begin cq_card.push_back(0); cq_owed++; end
    end
    if (axis_card_send[0].tvalid && axis_card_send[0].tready) begin
        c_d.push_back(axis_card_send[0].tdata); c_k.push_back(axis_card_send[0].tkeep);
        c_l.push_back(axis_card_send[0].tlast);
        if (axis_card_send[0].tlast) begin cq_card.push_back(1); cq_owed++; end
    end
end
always @(posedge aclk) begin
    cq_wr.valid <= 1'b0;
    cq_wr.data  <= '0;
    if (cq_owed > 0) begin
        if (cq_delay >= cq_lat) begin
            cq_wr.valid <= 1'b1;
            cq_owed--;
            cq_delay = 0;
            if (cq_card.pop_front()) begin
                card_done++; t_card_done = $time; card_done_t.push_back($time);
                apply_card_write();
            end
        end else cq_delay++;
    end
end


// ---------------------------------------------------------------------------
// Card memory model (programs): lines by 64 B-aligned VA; a line never
// written reads as pat(va). Card writes are kept until their completion,
// then applied (an 8 B write lands at its VA's offset in the line).
// ---------------------------------------------------------------------------
logic [AXI_DATA_BITS-1:0] cmem [longint];
function automatic logic [AXI_DATA_BITS-1:0] card_rd(input longint va);
    longint ln = va & ~64'd63;
    return cmem.exists(ln) ? cmem[ln] : pat(va);
endfunction

typedef struct { longint va; int len; logic [AXI_DATA_BITS-1:0] d [$]; logic [AXI_DATA_BITS/8-1:0] k [$]; } cwr_t;
req_t cwr_q [$];          // card write requests, in order
cwr_t cw_cur, cw_done [$];
bit   cw_open = 0;

// Logs with times (programs)
req_t rd_log [$], wr_log [$];
time  rd_log_t [$], wr_log_t [$], card_done_t [$], h_last_t [$];

always @(posedge aclk) if (aresetn) begin
    if (sq_rd.valid && sq_rd.ready) begin rd_log.push_back(sq_rd.data); rd_log_t.push_back($time); end
    if (sq_wr.valid && sq_wr.ready) begin
        wr_log.push_back(sq_wr.data); wr_log_t.push_back($time);
        if (sq_wr.data.strm == STRM_CARD) cwr_q.push_back(sq_wr.data);
    end
    if (axis_host_send[0].tvalid && axis_host_send[0].tready && axis_host_send[0].tlast) h_last_t.push_back($time);
    if (axis_card_send[0].tvalid && axis_card_send[0].tready) begin
        if (!cw_open) begin
            req_t r = cwr_q.pop_front();
            cw_cur.va = r.vaddr; cw_cur.len = r.len; cw_cur.d = {}; cw_cur.k = {};
            cw_open = 1;
        end
        cw_cur.d.push_back(axis_card_send[0].tdata); cw_cur.k.push_back(axis_card_send[0].tkeep);
        if (axis_card_send[0].tlast) begin cw_done.push_back(cw_cur); cw_open = 0; end
    end
end

task automatic apply_card_write();
    cwr_t w = cw_done.pop_front();
    for (int i = 0; i < w.d.size(); i++) begin
        longint va = w.va + 64*i;
        longint ln = va & ~64'd63;
        int     off = va & 63;
        logic [AXI_DATA_BITS-1:0] line = card_rd(ln);
        for (int b = 0; b < 64; b++)
            if (w.k[i][b] && off + b < 64) line[8*(off+b) +: 8] = w.d[i][8*b +: 8];
        cmem[ln] = line;
    end
endtask

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
                    input logic [47:0] fence, input bit go = 1, input bit get = 0);
    csr_wr(1, 64'(src)); csr_wr(2, 64'(dst)); csr_wr(3, 64'(len));
    csr_wr(4, 64'd3);    csr_wr(5, 64'(fence)); csr_wr(6, 64'(get));
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
// the fence (if any) with the expected count. A put reads the card stream
// and writes the host stream, a get the other way round; the fence is on
// the host stream either way.
task automatic expect_copy(input string name, input logic [47:0] src, input logic [47:0] dst,
                           input int len, input logic [47:0] fence, input int count, input bit get = 0);
    req_t r;
    `CHECK(rd_seen.size() != 0, {name, ": no read request"})
    if (rd_seen.size() != 0) begin
        r = rd_seen.pop_front();
        `CHECK(r.opcode == LOCAL_READ && r.strm == (get ? STRM_HOST : STRM_CARD) && r.pid == 3 && r.vaddr == src &&
               r.len == LEN_BITS'(len) && r.last, $sformatf("%s: read op %0d strm %0d va %h len %0d", name, r.opcode, r.strm, r.vaddr, r.len))
    end
    `CHECK(wr_seen.size() != 0, {name, ": no write request"})
    if (wr_seen.size() != 0) begin
        r = wr_seen.pop_front();
        `CHECK(r.opcode == LOCAL_WRITE && r.strm == (get ? STRM_CARD : STRM_HOST) && r.dest == 0 && r.pid == 3 &&
               r.vaddr == dst && r.len == LEN_BITS'(len) && r.last,
               $sformatf("%s: write op %0d strm %0d va %h len %0d", name, r.opcode, r.strm, r.vaddr, r.len))
    end
    for (int k = 0; k < len / 64; k++) begin
        if (get) begin
            `CHECK(c_d.size() != 0, $sformatf("%s: card beat %0d missing", name, k))
            if (c_d.size() == 0) break;
            `CHECK(c_d.pop_front() == pat(src + 64*k) && c_k.pop_front() == '1 &&
                   c_l.pop_front() == (k == len / 64 - 1), $sformatf("%s: card beat %0d", name, k))
        end else begin
            `CHECK(h_d.size() != 0, $sformatf("%s: beat %0d missing", name, k))
            if (h_d.size() == 0) break;
            `CHECK(h_d.pop_front() == pat(src + 64*k) && h_k.pop_front() == '1 &&
                   h_l.pop_front() == (k == len / 64 - 1), $sformatf("%s: beat %0d", name, k))
        end
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


// ---------------------------------------------------------------------------
// Programs
// ---------------------------------------------------------------------------
localparam int OP_COPY = 1, OP_SIGNAL = 2, OP_WAIT = 3, OP_REDUCE = 4, OP_STAMP = 5;
localparam longint PROG = 64'h7f00_3000_0000;

function automatic logic [AXI_DATA_BITS-1:0] ent(input int op, input bit src_host, input bit dst_card, input int len,
                                                 input longint a, input longint b, input longint dst, input longint val);
    logic [AXI_DATA_BITS-1:0] e = '0;
    e[3:0] = op[3:0]; e[4] = src_host; e[5] = dst_card; e[63:32] = len;
    e[64 +: 64] = a; e[128 +: 64] = b; e[192 +: 64] = dst; e[256 +: 64] = val;
    return e;
endfunction

logic [AXI_DATA_BITS-1:0] prog [$];
task automatic load_prog();
    for (int i = 0; i < prog.size(); i++) cmem[PROG + 64*i] = prog[i];
    csr_wr(4, 64'd3);
    csr_wr(11, PROG); csr_wr(12, 64'(prog.size())); csr_wr(13, 64'd1);
    wait_idle();
endtask
task automatic run_prog(input longint base);
    csr_wr(14, base); csr_wr(13, 64'd2);
endtask
task automatic clear_logs();
    rd_log = {}; rd_log_t = {}; wr_log = {}; wr_log_t = {}; card_done_t = {}; h_last_t = {};
    rd_seen = {}; wr_seen = {}; h_d = {}; h_k = {}; h_l = {}; c_d = {}; c_k = {}; c_l = {};
endtask
// int32 lanes of a line
function automatic logic [AXI_DATA_BITS-1:0] lanes(input int base, input int mul, input int add);
    logic [AXI_DATA_BITS-1:0] d;
    for (int l = 0; l < 16; l++) d[32*l +: 32] = 32'((base + l) * mul + add);
    return d;
endfunction
// The next host write in the logs: its request (va, len), then its beats
task automatic expect_host_write(input string name, input longint va, input int len,
                                 input logic [AXI_DATA_BITS-1:0] want [$]);
    req_t r;
    while (wr_log.size() != 0 && wr_log[0].strm == STRM_CARD) begin   // card writes: checked in card memory
        void'(wr_log.pop_front()); void'(wr_log_t.pop_front());
    end
    `CHECK(wr_log.size() != 0, {name, ": no write request"})
    if (wr_log.size() == 0) return;
    r = wr_log.pop_front(); void'(wr_log_t.pop_front());
    `CHECK(r.opcode == LOCAL_WRITE && r.strm == STRM_HOST && r.pid == 3 && r.vaddr == va && r.len == LEN_BITS'(len),
           $sformatf("%s: write strm %0d va %h len %0d (want %h %0d)", name, r.strm, r.vaddr, r.len, va, len))
    for (int k = 0; k < (len + 63) / 64; k++) begin
        `CHECK(h_d.size() != 0, $sformatf("%s: beat %0d missing", name, k))
        if (h_d.size() == 0) return;
        if (len == 8) begin
            logic [AXI_DATA_BITS-1:0]   d  = h_d.pop_front();
            logic [AXI_DATA_BITS/8-1:0] kk = h_k.pop_front();
            bit                         ll = h_l.pop_front();
            logic [AXI_DATA_BITS-1:0]   w  = want[k];
            `CHECK(d[63:0] == w[63:0] && kk == 64'hFF && ll,
                   $sformatf("%s: 8 B beat %0d keep %h last %0d, want %0d", name, d[63:0], kk, ll, w[63:0]))
        end else begin
            `CHECK(h_d.pop_front() == want[k] && h_k.pop_front() == '1 && h_l.pop_front() == (k == len / 64 - 1),
                   $sformatf("%s: beat %0d", name, k))
        end
    end
endtask
function automatic logic [AXI_DATA_BITS-1:0] word(input longint v);
    return {{(AXI_DATA_BITS-64){1'b0}}, 64'(v)};
endfunction

logic [63:0] v, c0, c1;
logic [AXI_DATA_BITS-1:0] want [$];

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
    csr_rd(6, v); `CHECK(v == 0, "T5: DIR")
    csr_rd(8, v); `CHECK(v == 0, "T5: BUSY")
    $display("ok   T5 readback");

    // --- T6: a 4 KB get with a fence: the fence after the HBM write's completion ---
    cq_lat = 200;
    copy(48'h7e00_0000_0000, 48'h7f00_1300_0000, 4096, 48'h7f00_2000_0080, 1, 1);
    wait_idle();
    expect_copy("T6", 48'h7e00_0000_0000, 48'h7f00_1300_0000, 4096, 48'h7f00_2000_0080, 5, 1);
    `CHECK(card_done == 1, $sformatf("T6: %0d card write completions", card_done))
    `CHECK(t_fence_req > t_card_done, "T6: fence requested before the HBM write completed")
    csr_rd(9, v);  `CHECK(v == 5, $sformatf("T6: COPIES %0d", v))
    csr_rd(6, v);  `CHECK(v == 1, "T6: DIR readback")
    csr_rd(10, c0); `CHECK(c0 > 200, $sformatf("T6: CYCLES %0d (should include the completion)", c0))
    `CHECK(rd_seen.size() == 0 && wr_seen.size() == 0 && h_d.size() == 0 && c_d.size() == 0, "T6: extra traffic")
    $display("ok   T6 get with fence (%0d cycles)", c0);

    // --- T7: a 64 KiB get, no fence, under backpressure: BUSY until the HBM write completes ---
    bp = 1;
    begin
        logic [63:0] p0 [3], p1 [3];
        for (int i = 0; i < 3; i++) csr_rd(48 + i, p0[i]);
        copy(48'h7e00_0001_0000, 48'h7f00_1400_0000, 65536, 48'h0, 1, 1);
        do csr_rd(8, v); while (v[0]);
        `CHECK(card_done == 2, $sformatf("T7: BUSY dropped with %0d of 2 card write completions", card_done))
        csr_rd(9, v); `CHECK(v == 6, $sformatf("T7: COPIES %0d", v))
        wait_idle();
        expect_copy("T7", 48'h7e00_0001_0000, 48'h7f00_1400_0000, 65536, 48'h0, 0, 1);
        `CHECK(wr_seen.size() == 0 && h_d.size() == 0 && c_d.size() == 0, "T7: a fence was written")
        for (int i = 0; i < 3; i++) csr_rd(48 + i, p1[i]);
        // (one write request, so sq_wr may not have waited)
        `CHECK(p1[0] > p0[0] && p1[1] > p0[1],
               $sformatf("T7: stall counters CE out bp %0d, CE in wait %0d",
                         p1[0] - p0[0], p1[1] - p0[1]))
    end
    bp = 0;
    cq_lat = 20;
    $display("ok   T7 64 KiB get under backpressure, BUSY until written");

    // --- T8: a put after gets ---
    copy(48'h7f00_1500_0000, 48'h7e00_0030_0000, 2048, 48'h7f00_2000_00c0, 1, 0);
    wait_idle();
    expect_copy("T8", 48'h7f00_1500_0000, 48'h7e00_0030_0000, 2048, 48'h7f00_2000_00c0, 7, 0);
    `CHECK(rd_seen.size() == 0 && wr_seen.size() == 0 && h_d.size() == 0 && c_d.size() == 0, "T8: extra traffic")
    `CHECK(card_done == 2, "T8: a put wrote card memory")
    $display("ok   T8 put after gets");

    // ======================== programs ========================
    clear_logs();
    // --- P1: three puts, a signal and a stamp; reads issued ahead ---
    prog = {ent(OP_COPY, 0, 0, 1024, 64'h7f00_4000_0000, 0, 64'h7e00_1000_0000, 0),
            ent(OP_COPY, 0, 0, 4096, 64'h7f00_4001_0000, 0, 64'h7e00_1001_0000, 0),
            ent(OP_COPY, 0, 0,   64, 64'h7f00_4002_0000, 0, 64'h7e00_1002_0000, 0),
            ent(OP_SIGNAL, 0, 0, 0, 0, 0, 64'h7e00_2000_0008, 7),
            ent(OP_STAMP,  0, 0, 0, 0, 0, 64'h7e00_2000_0010, 0)};
    load_prog();
    csr_rd(9, c0);
    clear_logs();
    hold_host = 1;                                        // the host stream mostly not ready
    run_prog(100);
    wait_idle();
    hold_host = 0;
    `CHECK(rd_log.size() == 3 && h_last_t.size() == 5, $sformatf("P1: %0d reads, %0d writes", rd_log.size(), h_last_t.size()))
    `CHECK(rd_log.size() >= 3 && h_last_t.size() >= 1 && rd_log_t[2] < h_last_t[0],
           "P1: the third copy's read was not issued before the first copy's data was out")
    for (int i = 0; i < 3; i++) begin
        longint src [3] = '{64'h7f00_4000_0000, 64'h7f00_4001_0000, 64'h7f00_4002_0000};
        int     ln  [3] = '{1024, 4096, 64};
        `CHECK(rd_log.size() > i && rd_log[i].strm == STRM_CARD && rd_log[i].vaddr == src[i] && rd_log[i].len == ln[i],
               $sformatf("P1: read %0d", i))
    end
    want = {}; for (int k = 0; k < 16; k++) want.push_back(pat(64'h7f00_4000_0000 + 64*k));
    expect_host_write("P1 copy 0", 64'h7e00_1000_0000, 1024, want);
    want = {}; for (int k = 0; k < 64; k++) want.push_back(pat(64'h7f00_4001_0000 + 64*k));
    expect_host_write("P1 copy 1", 64'h7e00_1001_0000, 4096, want);
    want = {pat(64'h7f00_4002_0000)};
    expect_host_write("P1 copy 2", 64'h7e00_1002_0000, 64, want);
    want = {word(107)};
    expect_host_write("P1 signal", 64'h7e00_2000_0008, 8, want);
    begin
        logic [63:0] stamp, nw;
        if (h_d.size()) begin logic [AXI_DATA_BITS-1:0] d0 = h_d[0]; stamp = d0[63:0]; end else stamp = 0;
        csr_rd(18, nw);
        `CHECK(stamp != 0 && stamp < nw, $sformatf("P1: stamp %0d, now %0d", stamp, nw))
        want = {word(stamp)};
        expect_host_write("P1 stamp", 64'h7e00_2000_0010, 8, want);
    end
    csr_rd(16, v); `CHECK(v == 1, $sformatf("P1: RUNS %0d", v))
    csr_rd(9, c1); `CHECK(c1 == c0 + 3, $sformatf("P1: COPIES %0d -> %0d", c0, c1))
    csr_rd(21, v); `CHECK(v > 0, "P1: RUN_CYC")
    `CHECK(wr_log.size() == 0 && h_d.size() == 0, "P1: extra writes")
    $display("ok   P1 program: puts read ahead, signal, stamp");

    // --- P2: the same program again, another RUN_BASE ---
    clear_logs();
    run_prog(200);
    wait_idle();
    `CHECK(h_d.size() == 16 + 64 + 1 + 1 + 1, $sformatf("P2: %0d beats", h_d.size()))
    begin
        logic [AXI_DATA_BITS-1:0] d81 = (h_d.size() >= 82) ? h_d[81] : '0;
        `CHECK(d81[63:0] == 207, $sformatf("P2: signal value %0d", d81[63:0]))
    end
    csr_rd(16, v); `CHECK(v == 2, $sformatf("P2: RUNS %0d", v))
    $display("ok   P2 run again, RUN_BASE 200");

    // --- P3: WAIT holds later reads until the flag reaches its target ---
    clear_logs();
    cmem[64'h7f00_5000_0000] = '0;                       // flag line: word 1 is the flag
    prog = {ent(OP_STAMP, 0, 1, 0, 0, 0, 64'h7f00_5100_0000, 0),
            ent(OP_WAIT,  0, 0, 0, 0, 0, 64'h7f00_5000_0008, 5),
            ent(OP_COPY,  0, 0, 2048, 64'h7f00_5200_0000, 0, 64'h7e00_3000_0000, 0),
            ent(OP_SIGNAL, 0, 0, 0, 0, 0, 64'h7e00_3100_0000, 1)};
    load_prog();
    clear_logs();
    run_prog(0);
    repeat (400) @(posedge aclk);
    begin
        bit early = 0;
        foreach (rd_log[i]) if (rd_log[i].vaddr == 64'h7f00_5200_0000) early = 1;
        `CHECK(!early, "P3: the copy's read was issued before the flag")
        `CHECK(h_d.size() == 0, "P3: host writes before the flag")
        csr_rd(20, v); `CHECK(v > 0, "P3: no WAIT polls counted")
        csr_rd(17, v); `CHECK(v == 1, $sformatf("P3: PC %0d while waiting", v))
    end
    for (int k = 0; k < 32; k++) cmem[64'h7f00_5200_0000 + 64*k] = lanes(1000 + k, 1, 0);   // the landed data
    cmem[64'h7f00_5000_0000] = {{(AXI_DATA_BITS-128){1'b0}}, 64'd4, 64'd0};                // short of the target
    repeat (300) @(posedge aclk);
    begin
        bit early = 0;
        foreach (rd_log[i]) if (rd_log[i].vaddr == 64'h7f00_5200_0000) early = 1;
        `CHECK(!early, "P3: the copy's read was issued at flag 4 of 5")
    end
    cmem[64'h7f00_5000_0000] = {{(AXI_DATA_BITS-128){1'b0}}, 64'd5, 64'd0};
    wait_idle();
    want = {}; for (int k = 0; k < 32; k++) want.push_back(lanes(1000 + k, 1, 0));
    expect_host_write("P3 copy", 64'h7e00_3000_0000, 2048, want);
    want = {word(1)};
    expect_host_write("P3 signal", 64'h7e00_3100_0000, 8, want);
    begin
        logic [AXI_DATA_BITS-1:0] st = card_rd(64'h7f00_5100_0000);
        `CHECK(cmem.exists(64'h7f00_5100_0000) && st[63:0] != 0, "P3: no stamp in card memory")
    end
    $display("ok   P3 WAIT holds later reads until the flag");

    // --- P4: REDUCE into card memory under backpressure, then copy it out ---
    clear_logs();
    for (int j = 0; j < 160; j++) begin
        cmem[64'h7f00_6000_0000 + 64*j] = lanes(16*j, 1, 0);
        cmem[64'h7f00_6100_0000 + 64*j] = lanes(16*j, 3, 1);
    end
    prog = {ent(OP_REDUCE, 0, 1, 10240, 64'h7f00_6000_0000, 64'h7f00_6100_0000, 64'h7f00_6200_0000, 0),
            ent(OP_COPY,   0, 0, 10240, 64'h7f00_6200_0000, 0, 64'h7e00_4000_0000, 0)};
    load_prog();
    clear_logs();
    bp = 1; cq_lat = 60;
    run_prog(0);
    wait_idle();
    bp = 0; cq_lat = 20;
    want = {}; for (int j = 0; j < 160; j++) want.push_back(lanes(16*j, 4, 1));
    expect_host_write("P4 result", 64'h7e00_4000_0000, 10240, want);   // (the REDUCE's own write is card)
    begin
        int  n_a = 0, n_b = 0;
        time t_d = 0;
        foreach (rd_log[i]) begin
            if (rd_log[i].vaddr >= 64'h7f00_6000_0000 && rd_log[i].vaddr < 64'h7f00_6100_0000) n_a++;
            if (rd_log[i].vaddr >= 64'h7f00_6100_0000 && rd_log[i].vaddr < 64'h7f00_6200_0000) n_b++;
            if (rd_log[i].vaddr == 64'h7f00_6200_0000) t_d = rd_log_t[i];
        end
        `CHECK(n_a == 3 && n_b == 3, $sformatf("P4: %0d + %0d chunk reads (want 3 + 3)", n_a, n_b))
        `CHECK(card_done_t.size() == 1 && t_d > card_done_t[0], "P4: the result was read before the REDUCE's write completed")
    end
    $display("ok   P4 REDUCE into card memory, then a copy of the result");

    // --- P5: REDUCE to the host stream, a short last chunk ---
    clear_logs();
    prog = {ent(OP_REDUCE, 0, 0, 64*64 + 192, 64'h7f00_6000_0000, 64'h7f00_6100_0000, 64'h7e00_5000_0000, 0)};
    load_prog();
    clear_logs();
    run_prog(0);
    wait_idle();
    want = {}; for (int j = 0; j < 67; j++) want.push_back(lanes(16*j, 4, 1));
    expect_host_write("P5 result", 64'h7e00_5000_0000, 64*64 + 192, want);
    `CHECK(card_done_t.size() == 0, "P5: card memory written")
    $display("ok   P5 REDUCE to the host stream, short last chunk");

    // --- P6: a card SIGNAL only after the card writes before it completed ---
    clear_logs();
    prog = {ent(OP_COPY,   0, 1, 4096, 64'h7f00_6000_0000, 0, 64'h7f00_7000_0000, 0),
            ent(OP_SIGNAL, 0, 1, 0, 0, 0, 64'h7f00_7100_0010, 9)};
    load_prog();
    clear_logs();
    cq_lat = 150;
    run_prog(0);
    wait_idle();
    cq_lat = 20;
    begin
        time t_sig = 0;
        logic [AXI_DATA_BITS-1:0] fl;
        foreach (wr_log[i]) if (wr_log[i].len == 8) t_sig = wr_log_t[i];
        `CHECK(card_done_t.size() == 2 && t_sig > card_done_t[0], "P6: the signal was requested before the copy completed")
        fl = card_rd(64'h7f00_7100_0000);
        `CHECK(fl[128 +: 64] == 9, "P6: signal not in card memory")
        `CHECK(cmem.exists(64'h7f00_7000_0fc0) && cmem[64'h7f00_7000_0fc0] == lanes(16*63, 1, 0), "P6: copy not in card memory")
    end
    $display("ok   P6 card SIGNAL after the card writes completed");

    // --- P7: ABORT ends a WAIT; a single copy afterwards ---
    clear_logs();
    prog = {ent(OP_WAIT, 0, 0, 0, 0, 0, 64'h7f00_5000_0008, 1000),
            ent(OP_SIGNAL, 0, 0, 0, 0, 0, 64'h7e00_3100_0000, 1)};
    load_prog();
    clear_logs();
    run_prog(0);
    repeat (300) @(posedge aclk);
    csr_rd(8, v); `CHECK(v[0], "P7: not busy in the WAIT")
    csr_rd(16, c0);
    csr_wr(15, 64'd1);
    wait_idle();
    csr_rd(19, v); `CHECK(v == 1, "P7: ABORTED not set")
    csr_rd(16, c1); `CHECK(c1 == c0 + 1, "P7: RUNS")
    `CHECK(h_d.size() == 0, "P7: the signal after the WAIT was written")
    rd_seen = {}; wr_seen = {};
    csr_rd(9, c0);
    copy(48'h7f00_1500_0000, 48'h7e00_0030_0000, 2048, 48'h7f00_2000_00c0, 1, 0);
    wait_idle();
    expect_copy("P7 single", 48'h7f00_1500_0000, 48'h7e00_0030_0000, 2048, 48'h7f00_2000_00c0, int'(c0) + 1, 0);
    $display("ok   P7 ABORT ends a WAIT, a single copy after it");

    // --- P8: a get (window -> card), then a put of what it wrote ---
    clear_logs();
    prog = {ent(OP_COPY, 1, 1, 1024, 64'h7e00_6000_0000, 0, 64'h7f00_8000_0000, 0),
            ent(OP_COPY, 0, 0, 1024, 64'h7f00_8000_0000, 0, 64'h7e00_7000_0000, 0)};
    load_prog();
    clear_logs();
    cq_lat = 120;
    run_prog(0);
    wait_idle();
    cq_lat = 20;
    want = {}; for (int k = 0; k < 16; k++) want.push_back(pat(64'h7e00_6000_0000 + 64*k));
    `CHECK(rd_log.size() == 2 && rd_log[0].strm == STRM_HOST && rd_log[1].strm == STRM_CARD &&
           card_done_t.size() == 1 && rd_log_t[1] > card_done_t[0], "P8: the put's read before the get's write completed")
    // (the get's card write is in wr_log first)
    void'(wr_log.pop_front()); void'(wr_log_t.pop_front());
    expect_host_write("P8 put", 64'h7e00_7000_0000, 1024, want);
    $display("ok   P8 get, then a put of what it wrote");

    // --- P9: 32 copies of 1 KiB with 100 cycles of read latency: read-ahead hides it ---
    clear_logs();
    prog = {};
    for (int i = 0; i < 32; i++) prog.push_back(ent(OP_COPY, 0, 0, 1024, 64'h7f00_9000_0000 + 1024*i, 0, 64'h7e00_8000_0000 + 1024*i, 0));
    load_prog();
    clear_logs();
    rd_lat = 100;
    run_prog(0);
    wait_idle();
    rd_lat = 0;
    csr_rd(21, v);
    `CHECK(v < 32 * 16 + 100 + 32 * 6, $sformatf("P9: %0d cycles for 32 x 1 KiB (512 beats)", v))
    `CHECK(h_d.size() == 512, $sformatf("P9: %0d beats", h_d.size()))
    $display("ok   P9 32 x 1 KiB, read latency 100: %0d cycles (512 beats)", v);

    if (errors == 0) $display("TB PASS (tb_loom_ce)");
    else             $display("TB FAIL (tb_loom_ce): %0d errors", errors);
    $finish;
end

initial begin
    #5ms;
    $display("TB FAIL (tb_loom_ce): timeout");
    $finish;
end

endmodule
