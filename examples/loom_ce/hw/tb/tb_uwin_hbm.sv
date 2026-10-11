`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_uwin_hbm - EN_UWIN_HBM's receive path: the user data window straight
 * into HBM, and the copy engine seeing the same bytes.
 *
 * Two paths onto one memory model (HBM behind the NoC), as in the shell:
 *   window: BAR address -> uwin_hbm -> axi_stripe -> memory
 *   card:   card physical address -> axi_stripe -> memory (a vFPGA card
 *           channel: what loom_ce's reads and writes go through)
 * The stripe is the shell's (EN_MEM_STRIPE, the V80's unified HBM), with its
 * register stages at 0 (the register-slice IP is not in xsim; the address
 * logic is the same).
 *
 * Covers: a peer's writes into the window (256 B bursts as PCIe hands them
 * over, single lines, 32 B half lines at line+32, a lone 8 B store) read back
 * by the card path at UWIN_HBM_BASE + offset; the card path writing and the
 * peer reading through the window; the window address with and without the
 * BAR's 0x0800_0000 bit mapping to the same place; uwin_hbm emitting exactly
 * UWIN_HBM_BASE + offset; the window's last line; rlast exact on the window's
 * reads (uwin_hbm makes it: the stripe raises it on a fragment's every beat);
 * DISCARD (T7): switched on and off by counter-page reads, writes answered
 * by uwin_hbm with their own IDs and never reaching memory, a switch held
 * back while a write is in flight. T8: 64 reads in flight (R held back),
 * the 65th held until they are answered. Counters: T5 the writes' (words
 * 0-15), T6 uwin_mon's writes and reads across the clock crossing (16-30),
 * T9 the window's reads (32-39) exact against what its port showed; T6
 * also uwin_ra's counters riding in uwin_mon's snapshot (64-76).
 *
 * READ-AHEAD (T10-T18), through a peer that keeps up to N reads in flight
 * and checks every beat against what each line was last written with: off
 * after reset (pass-through only, words 40-49 stay 0) and switched by lines
 * 61/60 (T10); a 128 KiB stream with R backpressure (T11); random mixes of
 * runs, jumps, starts inside a line and counter-page reads at 1-64 in flight
 * (T12); a window write ends the stream, later reads see the new data, alone
 * and with 32 reads in flight (T13); the idle timeout drops the ring, and
 * within it the documented exposure to card-path writes (T14); switched off
 * and on mid-run (T15); words 40-49 exact (T16); a stream up to the counter
 * page (T17); and what it buys at memory latency 200, 32 x 256 B in flight,
 * 64 KiB (T18). Always: m_axi never has more than 64 reads outstanding, no
 * AR crosses 4 KiB or leaves the window, none reads the counter page but the
 * peer's; read-ahead ARs and lines equal m_axi's traffic beyond the
 * pass-through reads (ra_xcheck).
 */
module tb_uwin_hbm;

logic aclk = 0;
logic aresetn = 0;
always #2 aclk = ~aclk;

localparam longint UWIN_SIZE = 64'h800_0000;   // 128 MB

AXI4 win_in (.aclk(aclk));          // what the shell's interconnect hands the window
AXI4 win_hbm (.aclk(aclk));         // after uwin_hbm (card physical address)
AXI4 win_mem (.aclk(aclk));         // after the stripe, to memory
AXI4 card_in (.aclk(aclk));         // a vFPGA card channel
AXI4 card_mem (.aclk(aclk));

// uwin_mon on its own clock (the shell's xclk), fed a known pattern (T6)
logic xclk = 0;
always #1.5 xclk = ~xclk;
logic m_awvalid = 0, m_awready = 0, m_wvalid = 0, m_wready = 0, m_bvalid = 0, m_bready = 0;
logic m_arvalid = 0, m_arready = 0, m_rvalid = 0, m_rready = 0, m_rlast = 0;
logic [7:0] m_awlen = 0, m_arlen = 0;
logic [64*28-1:0] main_snap;
logic main_tgl;
logic [64*13-1:0] ra_ctr;           // uwin_ra's counters (xclk): a known pattern, words 64-76 (T6)
always_comb for (int i = 0; i < 13; i++) ra_ctr[64*i +: 64] = 64'hC0DE_0000_0000_0000 | (64'(i) << 32) | 64'(i * 7 + 1);
uwin_mon inst_uwin_mon (.xclk(xclk), .xresetn(aresetn), .awvalid(m_awvalid), .awready(m_awready), .awlen(m_awlen),
                        .wvalid(m_wvalid), .wready(m_wready), .bvalid(m_bvalid), .bready(m_bready),
                        .arvalid(m_arvalid), .arready(m_arready), .arlen(m_arlen),
                        .rvalid(m_rvalid), .rready(m_rready), .rlast(m_rlast),
                        .ra_ctr(ra_ctr), .snap(main_snap), .snap_tgl(main_tgl));
uwin_hbm #(.BASE(UWIN_HBM_BASE)) inst_uwin_hbm (.aclk(aclk), .aresetn(aresetn), .s_axi(win_in), .m_axi(win_hbm),
                                               .main_snap(main_snap), .main_tgl(main_tgl));
axi_stripe #(.N_STAGES(0)) inst_strp_win  (.aclk(aclk), .aresetn(aresetn), .s_axi(win_hbm), .m_axi(win_mem));
axi_stripe #(.N_STAGES(0)) inst_strp_card (.aclk(aclk), .aresetn(aresetn), .s_axi(card_in), .m_axi(card_mem));

int errors = 0;
`define CHECK(c, msg) if (!(c)) begin errors++; $display("FAIL [%0t] %s", $time, msg); end

// ---------------------------------------------------------------------------
// Memory: bytes by address, shared by both ports; each port an AXI4 slave
// (AW/W in order, B per AW; AR -> R beats), random ready gaps; reads answer
// mem_lat cycles after their AR at the earliest, beats back to back if
// mem_fast (T18), else an idle cycle after each
// ---------------------------------------------------------------------------
byte unsigned mem [longint];
longint cyc = 0;                    // aclk cycles
always @(posedge aclk) cyc <= cyc + 1;
int mem_lat = 0;                    // cycles from AR taken to its first R beat (T15)
bit mem_fast = 0;                   // R beats back to back (else one idle cycle after each)

`define MEM_PORT(P)                                                                     \
    typedef struct { longint addr; int len; logic [AXI_ID_BITS-1:0] id; longint t; } P``_aw_t; \
    P``_aw_t P``_awq [$], P``_arq [$];                                                  \
    logic [AXI_ID_BITS-1:0] P``_bq [$];                                                 \
    time P``_tr = 0;                                                                    \
    initial begin                                                                       \
        P.awready = 0; P.wready = 0; P.bvalid = 0; P.arready = 0; P.rvalid = 0;         \
        P.bresp = 0; P.rresp = 0; P.rlast = 0; P.rdata = 0; P.bid = 0; P.rid = 0;       \
    end                                                                                 \
    always @(posedge aclk) begin                                                        \
        P.awready <= ($urandom_range(0, 3) != 0);                                      \
        P.arready <= ($urandom_range(0, 3) != 0);                                      \
        if (P.awvalid && P.awready)                                                     \
            P``_awq.push_back('{P.awaddr & ~64'd63, P.awlen + 1, P.awid, cyc});        \
        if (P.arvalid && P.arready)                                                     \
            P``_arq.push_back('{P.araddr & ~64'd63, P.arlen + 1, P.arid, cyc});        \
    end                                                                                 \
    initial forever begin                                                              \
        P``_aw_t t;                                                                     \
        wait (P``_awq.size() > 0);                                                      \
        t = P``_awq.pop_front();                                                        \
        for (int j = 0; j < t.len; j++) begin                                           \
            @(negedge aclk); P.wready = 1;                                              \
            do @(posedge aclk); while (!P.wvalid);                                      \
            for (int b = 0; b < 64; b++)                                                \
                if (P.wstrb[b]) begin                                                   \
                    mem[t.addr + 64*j + b] = P.wdata[8*b +: 8];                         \
                end                                                                     \
            `CHECK(P.wlast == (j == t.len - 1), `"P: wlast`")                          \
            @(negedge aclk); P.wready = 0;                                              \
        end                                                                             \
        P``_bq.push_back(t.id);                                                         \
    end                                                                                 \
    initial forever begin                                                              \
        wait (P``_bq.size() > 0);                                                       \
        @(negedge aclk); P.bid = P``_bq.pop_front(); P.bvalid = 1;                      \
        do @(posedge aclk); while (!P.bready);                                          \
        @(negedge aclk); P.bvalid = 0;                                                  \
    end                                                                                 \
    initial forever begin                                                              \
        P``_aw_t t;                                                                     \
        wait (P``_arq.size() > 0);                                                      \
        t = P``_arq.pop_front();                                                        \
        if (!(mem_fast && $time == P``_tr)) @(negedge aclk);                            \
        while (cyc < t.t + mem_lat) @(negedge aclk);                                    \
        for (int j = 0; j < t.len; j++) begin                                           \
            logic [AXI_DATA_BITS-1:0] d;                                                \
            if (j > 0 && !mem_fast) @(negedge aclk);                                    \
            for (int b = 0; b < 64; b++)                                                \
                d[8*b +: 8] = mem.exists(t.addr + 64*j + b) ? mem[t.addr + 64*j + b] : 8'h00; \
            P.rdata = d; P.rid = t.id; P.rlast = (j == t.len - 1); P.rvalid = 1;         \
            do @(posedge aclk); while (!P.rready);                                      \
            @(negedge aclk); P.rvalid = 0;                                              \
        end                                                                             \
        P``_tr = $time;                                                                 \
    end

`MEM_PORT(win_mem)
`MEM_PORT(card_mem)

// ---------------------------------------------------------------------------
// Masters: one burst at a time (AW, W beats, B; AR, R beats)
// ---------------------------------------------------------------------------
`define MASTER_INIT(M)                                                                  \
    initial begin                                                                       \
        M.awvalid = 0; M.wvalid = 0; M.arvalid = 0; M.bready = 1; M.rready = 1;         \
        M.awburst = 2'b01; M.awsize = 3'd6; M.awcache = 0; M.awlock = 0; M.awprot = 0;  \
        M.awqos = 0; M.awregion = 0; M.awid = 0; M.awlen = 0; M.awaddr = 0;             \
        M.arburst = 2'b01; M.arsize = 3'd6; M.arcache = 0; M.arlock = 0; M.arprot = 0;  \
        M.arqos = 0; M.arregion = 0; M.arid = 0; M.arlen = 0; M.araddr = 0;             \
        M.wdata = 0; M.wstrb = 0; M.wlast = 0;                                          \
    end
`MASTER_INIT(win_in)
`MASTER_INIT(card_in)

// the data a test writes at a given byte address (card side)
function automatic byte unsigned val(input longint a, input int seed);
    return byte'((a * 131) ^ (a >> 7) ^ seed);
endfunction

// A burst of `beats` beats from `addr` (a line-aligned start), first beat
// strobes strb0, others full; data from val(card_base + offset, seed)
task automatic wr_burst(input bit win, input longint addr, input longint card_addr, input int beats,
                        input logic [63:0] strb0, input int seed);
    if (win) begin
        @(negedge aclk); win_in.awaddr = addr; win_in.awlen = 8'(beats - 1); win_in.awvalid = 1;
        do @(posedge aclk); while (!win_in.awready);
        @(negedge aclk); win_in.awvalid = 0;
        for (int j = 0; j < beats; j++) begin
            logic [AXI_DATA_BITS-1:0] d;
            for (int b = 0; b < 64; b++) d[8*b +: 8] = val(card_addr + 64*j + b, seed);
            win_in.wdata = d; win_in.wstrb = (j == 0) ? strb0 : '1; win_in.wlast = (j == beats - 1); win_in.wvalid = 1;
            do @(posedge aclk); while (!win_in.wready);
            @(negedge aclk); win_in.wvalid = 0;
        end
        while (!win_in.bvalid) @(posedge aclk);
        @(negedge aclk);
    end else begin
        @(negedge aclk); card_in.awaddr = addr; card_in.awlen = 8'(beats - 1); card_in.awvalid = 1;
        do @(posedge aclk); while (!card_in.awready);
        @(negedge aclk); card_in.awvalid = 0;
        for (int j = 0; j < beats; j++) begin
            logic [AXI_DATA_BITS-1:0] d;
            for (int b = 0; b < 64; b++) d[8*b +: 8] = val(card_addr + 64*j + b, seed);
            card_in.wdata = d; card_in.wstrb = (j == 0) ? strb0 : '1; card_in.wlast = (j == beats - 1); card_in.wvalid = 1;
            do @(posedge aclk); while (!card_in.wready);
            @(negedge aclk); card_in.wvalid = 0;
        end
        while (!card_in.bvalid) @(posedge aclk);
        @(negedge aclk);
    end
endtask

// Read `beats` beats from `addr`; compare every byte with val(card_addr...),
// bytes where `mask` says nothing was written must be zero
task automatic rd_check(input string name, input bit win, input longint addr, input longint card_addr,
                        input int beats, input int seed);
    int got = 0;
    if (win) begin
        @(negedge aclk); win_in.araddr = addr; win_in.arlen = 8'(beats - 1); win_in.arvalid = 1;
        do @(posedge aclk); while (!win_in.arready);
        @(negedge aclk); win_in.arvalid = 0;
        while (got < beats) begin
            @(posedge aclk);
            if (win_in.rvalid && win_in.rready) begin
                for (int b = 0; b < 64; b++)
                    `CHECK(win_in.rdata[8*b +: 8] == val(card_addr + 64*got + b, seed),
                           $sformatf("%s: byte %0d of beat %0d read %02x, want %02x", name, b, got,
                                     win_in.rdata[8*b +: 8], val(card_addr + 64*got + b, seed)))
                `CHECK(win_in.rlast == (got == beats - 1), {name, ": rlast"})
                got++;
            end
        end
    end else begin
        @(negedge aclk); card_in.araddr = addr; card_in.arlen = 8'(beats - 1); card_in.arvalid = 1;
        do @(posedge aclk); while (!card_in.arready);
        @(negedge aclk); card_in.arvalid = 0;
        while (got < beats) begin
            @(posedge aclk);
            if (card_in.rvalid && card_in.rready) begin
                for (int b = 0; b < 64; b++)
                    `CHECK(card_in.rdata[8*b +: 8] == val(card_addr + 64*got + b, seed),
                           $sformatf("%s: byte %0d of beat %0d read %02x, want %02x", name, b, got,
                                     card_in.rdata[8*b +: 8], val(card_addr + 64*got + b, seed)))
                got++;
            end
        end
    end
endtask

// uwin_hbm must emit exactly UWIN_HBM_BASE + (BAR address mod 128 MB)
longint aw_expect [$];
always @(posedge aclk) if (win_hbm.awvalid && win_hbm.awready && aw_expect.size() != 0) begin
    longint e = aw_expect.pop_front();
    `CHECK(win_hbm.awaddr == e, $sformatf("uwin_hbm awaddr %h, expected %h", win_hbm.awaddr, e))
end

localparam longint BAR = 64'h0800_0000;     // the window's offset in the BAR, as the shell hands it over

// T7: writes uwin_hbm passes on to memory, and the IDs the window must answer
int hbm_aw = 0;
always @(posedge aclk) if (win_hbm.awvalid && win_hbm.awready) hbm_aw++;
bit t7_on = 0;
logic [AXI_ID_BITS-1:0] t7_ids [$];
int t7_b = 0;
always @(posedge aclk) if (t7_on) begin
    if (win_in.awvalid && win_in.awready) t7_ids.push_back(win_in.awid);
    if (win_in.bvalid && win_in.bready) begin
        logic [AXI_ID_BITS-1:0] e;
        `CHECK(t7_ids.size() > 0, "T7: a B with no write outstanding")
        if (t7_ids.size() > 0) begin
            e = t7_ids.pop_front();
            `CHECK(win_in.bid == e && win_in.bresp == 2'b00, $sformatf("T7: bid %0d resp %0d, want %0d OKAY", win_in.bid, win_in.bresp, e))
        end
        t7_b++;
    end
end

// T9: the window's reads as its port (s_axi) shows them, from reset: cycles
// with a read outstanding (AR taken, last R not yet), the sum of reads
// outstanding, AR and R stalls. Nonblocking, so a value read at a posedge has
// what uwin_hbm's counters had
longint rd_out = 0, rd_cyc = 0, rd_sum = 0, rd_ar_stall = 0, rd_r_stall = 0;
always @(posedge aclk) if (aresetn) begin
    rd_out      <= rd_out + (win_in.arvalid && win_in.arready) - (win_in.rvalid && win_in.rready && win_in.rlast);
    rd_cyc      <= rd_cyc + (rd_out != 0);
    rd_sum      <= rd_sum + rd_out;
    rd_ar_stall <= rd_ar_stall + (win_in.arvalid && !win_in.arready);
    rd_r_stall  <= rd_r_stall + (win_in.rvalid && !win_in.rready);
end

// One beat of the counter page's line `line`: its eight words
task automatic ctr_line(input int line, output logic [63:0] c [8]);
    int got = 0;
    @(negedge aclk); win_in.araddr = BAR + UWIN_SIZE - 4096 + 64*line; win_in.arlen = 0; win_in.arvalid = 1;
    do @(posedge aclk); while (!win_in.arready);
    @(negedge aclk); win_in.arvalid = 0;
    while (!got) begin
        @(posedge aclk);
        if (win_in.rvalid && win_in.rready) begin
            for (int j = 0; j < 8; j++) c[j] = win_in.rdata[64*j +: 64];
            got = 1;
        end
    end
endtask
localparam logic [63:0] HI32 = 64'hFFFF_FFFF_0000_0000;
localparam logic [63:0] LO32 = 64'h0000_0000_FFFF_FFFF;

// ---------------------------------------------------------------------------
// READ-AHEAD (T10-T16)
// ---------------------------------------------------------------------------
localparam longint R0 = 64'h40_0000, RSZ = 64'h4_0000;   // 256 KiB the card path fills
localparam int     RA_LINES = 256, RA_BURST = 16;         // uwin_hbm's defaults

// m_axi (uwin_hbm -> stripe): ARs and R beats taken, ARs outstanding (by
// their lengths: the stripe's rlast is not AXI's), never more than 64; no AR
// crosses 4 KiB or leaves the window; ARs into the counter page (at the end:
// only the peer's counter-page reads)
longint hbm_ar = 0, hbm_rb = 0, win_ar = 0, win_rb = 0, hbm_ctr_ar = 0, win_ctr_ar = 0;
int     hbm_len [$];
int     hbm_beat = 0;
always @(posedge aclk) if (aresetn) begin
    if (win_hbm.arvalid && win_hbm.arready) begin
        hbm_ar++;
        hbm_len.push_back(win_hbm.arlen + 1);
        `CHECK(((win_hbm.araddr & 64'hFC0) + 64*(win_hbm.arlen + 1)) <= 4096 && win_hbm.araddr >= UWIN_HBM_BASE
               && win_hbm.araddr < UWIN_HBM_BASE + UWIN_SIZE && win_hbm.arsize == 3'd6 && win_hbm.arburst == 2'b01,
               $sformatf("m_axi AR %h len %0d size %0d burst %0d", win_hbm.araddr, win_hbm.arlen, win_hbm.arsize, win_hbm.arburst))
    end
    if (win_hbm.rvalid && win_hbm.rready) begin
        hbm_rb++;
        `CHECK(hbm_len.size() > 0, "m_axi: an R beat with no read outstanding")
        hbm_beat++;
        if (hbm_len.size() > 0 && hbm_beat == hbm_len[0]) begin
            void'(hbm_len.pop_front());
            hbm_beat = 0;
        end
    end
    `CHECK(hbm_len.size() <= 64, $sformatf("m_axi: %0d reads outstanding", hbm_len.size()))
    if (win_hbm.arvalid && win_hbm.arready && win_hbm.araddr >= UWIN_HBM_BASE + UWIN_SIZE - 4096) hbm_ctr_ar++;
    if (win_in.arvalid && win_in.arready && win_in.araddr[26:0] >= UWIN_SIZE - 4096) win_ctr_ar++;
    if (win_in.arvalid && win_in.arready) win_ar++;
    if (win_in.rvalid && win_in.rready) win_rb++;
end

// the seed each window line was last written with (val(card address, seed))
int seed_of [longint];
function automatic logic [AXI_DATA_BITS-1:0] line_val(input longint ln);
    logic [AXI_DATA_BITS-1:0] d;
    for (int b = 0; b < 64; b++) d[8*b +: 8] = val(UWIN_HBM_BASE + 64*ln + b, seed_of[ln]);
    return d;
endfunction

// the card path (the copy engine) writes `kib` KiB at window offset off
task automatic card_fill(input longint off, input int kib, input int seed);
    for (int k = 0; k < kib; k++) begin
        wr_burst(0, UWIN_HBM_BASE + off + 1024*k, UWIN_HBM_BASE + off + 1024*k, 16, '1, seed);
        for (int j = 0; j < 16; j++) seed_of[(off + 1024*k) / 64 + j] = seed;
    end
endtask

// The peer: reads {window offset, beats} issued in order, at most max_out in
// flight (the CPM allows 32), R checked beat by beat against seed_of, X
// included (counter page: beats and rlast only), random R backpressure while
// peer_rbp. A read in [gate_lo, gate_hi) waits for gate_open. cycles: first
// AR to last R.
typedef struct { longint off; int beats; } rd_t;
bit     peer_rbp = 0;
always @(negedge aclk) if (peer_rbp) win_in.rready = ($urandom_range(0, 2) != 0);
bit     gate_open = 1;
longint gate_lo = -1, gate_hi = -1;
longint peer_front = 0;             // the last read issued
task automatic peer_rd(input string name, input rd_t rds [$], input int max_out, output longint cycles);
    int out = 0;
    longint t0 = cyc, t1;
    fork
        begin
            @(negedge aclk);
            foreach (rds[k]) begin
                while (out >= max_out) @(negedge aclk);
                if (rds[k].off >= gate_lo && rds[k].off < gate_hi) while (!gate_open) @(negedge aclk);
                win_in.araddr = BAR + rds[k].off; win_in.arlen = 8'(rds[k].beats - 1); win_in.arvalid = 1;
                do @(posedge aclk); while (!win_in.arready);
                out++;
                peer_front = rds[k].off;
                @(negedge aclk); win_in.arvalid = 0;
            end
        end
        begin
            foreach (rds[k]) begin
                bit ctrp = (rds[k].off >= UWIN_SIZE - 4096);
                for (int j = 0; j < rds[k].beats; j++) begin
                    do @(posedge aclk); while (!(win_in.rvalid && win_in.rready));
                    if (!ctrp)
                        `CHECK(win_in.rdata === line_val(rds[k].off / 64 + j),
                               $sformatf("%s: read %0d (offset %h) beat %0d: line %h, want seed %0d", name, k, rds[k].off, j,
                                         rds[k].off / 64 + j, seed_of[rds[k].off / 64 + j]))
                    `CHECK(win_in.rlast === (j == rds[k].beats - 1) && win_in.rresp === 2'b00 && win_in.rid === '0,
                           $sformatf("%s: read %0d beat %0d: rlast %0d resp %0d id %0d", name, k, j, win_in.rlast,
                                     win_in.rresp, win_in.rid))
                    if (j == rds[k].beats - 1) out--;
                end
            end
            t1 = cyc;
        end
    join
    cycles = t1 - t0;
endtask

// n sequential reads of `beats` from window offset off
function automatic void seq_rds(ref rd_t rds [$], input longint off, input int n, input int beats);
    for (int k = 0; k < n; k++) rds.push_back('{off + 64*beats*k, beats});
endfunction

// a random mix in [R0, R0 + RSZ): runs of sequential reads (1-16 beats, not
// across 4 KiB), jumps, starts inside a line, counter-page reads (lines 0-7)
function automatic void mix(ref rd_t rds [$], input int n);
    int bl [7] = '{1, 2, 4, 4, 4, 8, 16};
    longint p = R0;
    for (int k = 0; k < n; k++) begin
        int x = $urandom_range(0, 99);
        int b, room;
        longint off;
        if (x < 3) begin
            rds.push_back('{UWIN_SIZE - 4096 + 64*$urandom_range(0, 7), $urandom_range(1, 2)});
            continue;
        end
        if (x < 25) p = R0 + 64*$urandom_range(0, RSZ/64 - 1);
        room = (4096 - (p % 4096)) / 64;
        b = bl[$urandom_range(0, 6)];
        if (b > room) b = room;
        off = (x >= 25 && x < 30) ? p + 8*$urandom_range(1, 7) : p;
        rds.push_back('{off, b});
        p = p + 64*b;
        if (p >= R0 + RSZ) p = R0;
    end
endfunction

// read-ahead on/off (counter page line 61/60), word 14 says so
task automatic ra_set(input bit on);
    logic [63:0] c [8];
    ctr_line(on ? 61 : 60, c);
    ctr_line(1, c);
    `CHECK(c[6] == 64'(on), $sformatf("read-ahead mode %0d, want %0d", c[6], on))
endtask

// words 40-49, and the port counts at the same moment (the window idle)
typedef struct { longint w [10]; longint ar, rb, har, hrb; } ra_snap_t;
task automatic ra_snap(output ra_snap_t s);
    logic [63:0] c [8];
    ctr_line(5, c);
    for (int j = 0; j < 8; j++) s.w[j] = c[j];
    ctr_line(6, c);
    s.w[8] = c[0]; s.w[9] = c[1];
    `CHECK(c[2] == 0, $sformatf("word 50 = %0d, want 0", c[2]))
    repeat (4) @(negedge aclk);
    s.ar = win_ar; s.rb = win_rb; s.har = hbm_ar; s.hrb = hbm_rb;
endtask
localparam int HIT = 0, LATE = 1, PASS = 2, RA_AR = 3, FILLED = 4, SERVED = 5, STREAMS = 6, END_J = 7, END_W = 8, END_I = 9;

// Between two snapshots: the fills and lines read ahead are exactly m_axi's
// traffic beyond the reads that passed through (ARs not answered by the ring,
// their beats)
task automatic ra_xcheck(input string name, input ra_snap_t a, input ra_snap_t b);
    longint ring_rd = (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE]);
    longint served  = b.w[SERVED] - a.w[SERVED];
    `CHECK(b.w[RA_AR] - a.w[RA_AR] == (b.har - a.har) - ((b.ar - a.ar) - ring_rd),
           $sformatf("%s: %0d read-ahead ARs, m_axi shows %0d", name, b.w[RA_AR] - a.w[RA_AR],
                     (b.har - a.har) - ((b.ar - a.ar) - ring_rd)))
    `CHECK(b.w[FILLED] - a.w[FILLED] == (b.hrb - a.hrb) - ((b.rb - a.rb) - served),
           $sformatf("%s: %0d lines read ahead, m_axi shows %0d", name, b.w[FILLED] - a.w[FILLED],
                     (b.hrb - a.hrb) - ((b.rb - a.rb) - served)))
endtask

initial begin
    repeat (10) @(negedge aclk);
    aresetn = 1;
    repeat (10) @(negedge aclk);

    // --- T1: a peer writes 16 KiB into the window as PCIe hands it over
    //     (256 B bursts), the card path reads it at UWIN_HBM_BASE + offset
    for (int k = 0; k < 64; k++) begin
        aw_expect.push_back(UWIN_HBM_BASE + 64'h1_0000 + 256*k);
        wr_burst(1, BAR + 64'h1_0000 + 256*k, UWIN_HBM_BASE + 64'h1_0000 + 256*k, 4, '1, 1);
    end
    for (int k = 0; k < 16; k++)
        rd_check("T1", 0, UWIN_HBM_BASE + 64'h1_0000 + 1024*k, UWIN_HBM_BASE + 64'h1_0000 + 1024*k, 16, 1);
    $display("ok   T1 peer writes 16 KiB into the window, the card path reads it");

    // --- T2: single lines, 32 B half lines at line+0 / line+32 (a CPU's
    //     write-combining flush), a lone 8 B store; without the BAR bit too
    for (int k = 0; k < 8; k++) begin
        wr_burst(1, BAR + 64'h2_0000 + 64*k, UWIN_HBM_BASE + 64'h2_0000 + 64*k, 1, '1, 2);
        wr_burst(1, BAR + 64'h2_1000 + 64*k,      UWIN_HBM_BASE + 64'h2_1000 + 64*k, 1, LO32, 2);
        wr_burst(1, BAR + 64'h2_1000 + 64*k + 32, UWIN_HBM_BASE + 64'h2_1000 + 64*k, 1, HI32, 2);
        wr_burst(1,       64'h2_2000 + 64*k,      UWIN_HBM_BASE + 64'h2_2000 + 64*k, 1, '1, 2);   // no BAR bit
    end
    rd_check("T2 lines", 0, UWIN_HBM_BASE + 64'h2_0000, UWIN_HBM_BASE + 64'h2_0000, 8, 2);
    rd_check("T2 halves", 0, UWIN_HBM_BASE + 64'h2_1000, UWIN_HBM_BASE + 64'h2_1000, 8, 2);
    rd_check("T2 no BAR bit", 0, UWIN_HBM_BASE + 64'h2_2000, UWIN_HBM_BASE + 64'h2_2000, 8, 2);
    begin
        // the lone store: only its 8 bytes land
        wr_burst(1, BAR + 64'h2_3000 + 8, UWIN_HBM_BASE + 64'h2_3000, 1, 64'hFF << 8, 3);
        begin
            int got = 0;
            @(negedge aclk); card_in.araddr = UWIN_HBM_BASE + 64'h2_3000; card_in.arlen = 0; card_in.arvalid = 1;
            do @(posedge aclk); while (!card_in.arready);
            @(negedge aclk); card_in.arvalid = 0;
            while (!got) begin
                @(posedge aclk);
                if (card_in.rvalid && card_in.rready) begin
                    for (int b = 0; b < 64; b++)
                        `CHECK(card_in.rdata[8*b +: 8] == ((b >= 8 && b < 16) ? val(UWIN_HBM_BASE + 64'h2_3000 + b, 3) : 8'h00),
                               $sformatf("T2 store: byte %0d = %02x", b, card_in.rdata[8*b +: 8]))
                    got = 1;
                end
            end
        end
    end
    $display("ok   T2 lines, half lines, a lone 8 B store; with and without the BAR bit");

    // --- T3: the card path (the copy engine) writes, the peer reads through the window
    for (int k = 0; k < 8; k++)
        wr_burst(0, UWIN_HBM_BASE + 64'h4_0000 + 1024*k, UWIN_HBM_BASE + 64'h4_0000 + 1024*k, 16, '1, 4);
    for (int k = 0; k < 32; k++)
        rd_check("T3", 1, BAR + 64'h4_0000 + 256*k, UWIN_HBM_BASE + 64'h4_0000 + 256*k, 4, 4);
    $display("ok   T3 the card path writes, the peer reads it through the window");

    // --- T4: the window's last line and a write that wraps the 128 MB offset
    aw_expect.push_back(UWIN_HBM_BASE + UWIN_SIZE - 64);
    wr_burst(1, BAR + UWIN_SIZE - 64, UWIN_HBM_BASE + UWIN_SIZE - 64, 1, '1, 5);
    rd_check("T4", 0, UWIN_HBM_BASE + UWIN_SIZE - 64, UWIN_HBM_BASE + UWIN_SIZE - 64, 1, 5);
    `CHECK(aw_expect.size() == 0, "uwin_hbm: expected write addresses not all seen")
    $display("ok   T4 the window's last line; uwin_hbm addresses exact");

    // --- T5: the counters, read through the window's last page. T1-T4 wrote
    //     64 x 4-beat bursts + 33 single beats (17 with a partial strobe) + 1,
    //     one at a time (so never more than one outstanding)
    begin
        logic [63:0] c [16];
        int got = 0;
        @(negedge aclk); win_in.araddr = BAR + UWIN_SIZE - 4096; win_in.arlen = 1; win_in.arvalid = 1;
        do @(posedge aclk); while (!win_in.arready);
        @(negedge aclk); win_in.arvalid = 0;
        while (got < 2) begin
            @(posedge aclk);
            if (win_in.rvalid && win_in.rready) begin
                for (int j = 0; j < 8; j++) c[8*got + j] = win_in.rdata[64*j +: 64];
                `CHECK(win_in.rlast == (got == 1), "T5: rlast")
                got++;
            end
        end
        `CHECK(c[0] > 0,     $sformatf("T5: cycles %0d", c[0]))
        `CHECK(c[1] == 98,   $sformatf("T5: AW bursts %0d, want 98", c[1]))
        `CHECK(c[2] == 290,  $sformatf("T5: W beats %0d, want 290", c[2]))
        `CHECK(c[3] > 0,     $sformatf("T5: W stalled %0d (the memory model drops wready)", c[3]))
        `CHECK(c[6] == 98,   $sformatf("T5: B %0d, want 98", c[6]))
        `CHECK(c[8] > 0 && c[9] == c[8], $sformatf("T5: outstanding cycles %0d, sum %0d", c[8], c[9]))
        `CHECK(c[10] == 1,   $sformatf("T5: max outstanding %0d, want 1", c[10]))
        `CHECK(c[11] == 17,  $sformatf("T5: partial beats %0d, want 17", c[11]))
        for (int i = 12; i < 16; i++) `CHECK(c[i] == 0, $sformatf("T5: word %0d = %0d, want 0", i, c[i]))
        // a single beat at the second line: words 8-11, then zeros
        got = 0;
        @(negedge aclk); win_in.araddr = BAR + UWIN_SIZE - 4096 + 64; win_in.arlen = 0; win_in.arvalid = 1;
        do @(posedge aclk); while (!win_in.arready);
        @(negedge aclk); win_in.arvalid = 0;
        while (!got) begin
            @(posedge aclk);
            if (win_in.rvalid && win_in.rready) begin
                `CHECK(win_in.rdata[64*2 +: 64] == 1 && win_in.rdata[64*3 +: 64] == 17 && win_in.rdata[64*4 +: 64] == 0 && win_in.rlast,
                       "T5: second line read alone")
                got = 1;
            end
        end
    end
    // reads elsewhere still return HBM
    rd_check("T5 HBM", 1, BAR + 64'h4_0000, UWIN_HBM_BASE + 64'h4_0000, 4, 4);
    $display("ok   T5 counters through the window's last page; other reads still HBM");

    // --- T6: axi_main's counters (uwin_mon, xclk), across the clock crossing:
    //     10 AWs of 4 beats, 40 W beats against a wready low every third
    //     cycle, 10 Bs; reads: 3 cycles AR stalled, 4 ARs of 2 beats, 5 idle
    //     cycles, then 10 R beats against an rready low every third cycle, a
    //     5th AR taken with the first read's last beat; then the page's words
    //     16-30 after a snapshot
    begin
        int beats = 0, stalls = 0, starve = 0, rb = 0;
        logic [63:0] c [8], c2 [8];
        int got = 0;
        @(negedge xclk); m_awvalid = 1; m_awready = 1; m_awlen = 3;
        repeat (10) @(negedge xclk);
        m_awvalid = 0; m_awready = 0;
        repeat (5) @(negedge xclk);     // beats owed, nothing on W: starved
        starve = 9 + 5;                 // after the first AW (9 cycles) and these 5
        for (int k = 0; beats < 40; k++) begin
            m_wvalid = 1; m_wready = (k % 3 != 0);
            @(posedge xclk);
            if (m_wready) beats++; else stalls++;
            @(negedge xclk);
        end
        m_wvalid = 0; m_wready = 0;
        m_bvalid = 1; m_bready = 1;
        repeat (10) @(negedge xclk);
        m_bvalid = 0; m_bready = 0;
        m_arvalid = 1; m_arlen = 1;
        repeat (3) @(negedge xclk);     // AR stalled
        m_arready = 1;
        repeat (4) @(negedge xclk);     // 4 reads outstanding
        m_arvalid = 0; m_arready = 0;
        repeat (5) @(negedge xclk);
        for (int k = 0; rb < 10; k++) begin
            m_rvalid = 1; m_rready = (k % 3 != 0); m_rlast = (rb % 2 == 1);
            m_arvalid = (k == 2); m_arready = (k == 2);
            @(posedge xclk);
            if (m_rready) rb++;
            @(negedge xclk);
        end
        m_rvalid = 0; m_rready = 0; m_rlast = 0; m_arvalid = 0; m_arready = 0;
        repeat (700) @(negedge xclk);   // two snapshots later
        @(negedge aclk); win_in.araddr = BAR + UWIN_SIZE - 4096 + 128; win_in.arlen = 0; win_in.arvalid = 1;
        do @(posedge aclk); while (!win_in.arready);
        @(negedge aclk); win_in.arvalid = 0;
        while (!got) begin
            @(posedge aclk);
            if (win_in.rvalid && win_in.rready) begin
                for (int j = 0; j < 8; j++) c[j] = win_in.rdata[64*j +: 64];
                got = 1;
            end
        end
        `CHECK(c[0] > 700,      $sformatf("T6: xclk cycles %0d", c[0]))
        `CHECK(c[1] == 10,      $sformatf("T6: AW %0d, want 10", c[1]))
        `CHECK(c[2] == 40,      $sformatf("T6: W beats %0d, want 40", c[2]))
        `CHECK(c[3] == stalls,  $sformatf("T6: W stalled %0d, want %0d", c[3], stalls))
        `CHECK(c[4] == starve,  $sformatf("T6: W starved %0d, want %0d", c[4], starve))
        `CHECK(c[5] == 0,       $sformatf("T6: AW stalled %0d, want 0", c[5]))
        `CHECK(c[6] == 10,      $sformatf("T6: B %0d, want 10", c[6]))
        // reads outstanding: 1,2,3 behind the first three ARs, 4 for the 5
        // idle cycles and 6 R cycles (the 5th AR replaces the first read),
        // then 3,3,3,2,2,2,1,1,1: 23 cycles, sum 6 + 20 + 24 + 18 = 68
        ctr_line(3, c2);
        `CHECK(c[7] == 5,       $sformatf("T6: AR %0d, want 5", c[7]))
        `CHECK(c2[0] == 10,     $sformatf("T6: R beats %0d, want 10", c2[0]))
        `CHECK(c2[1] == 5,      $sformatf("T6: reads completed %0d, want 5", c2[1]))
        `CHECK(c2[2] == 23,     $sformatf("T6: cycles with a read outstanding %0d, want 23", c2[2]))
        `CHECK(c2[3] == 68,     $sformatf("T6: sum of reads outstanding %0d, want 68", c2[3]))
        `CHECK(c2[4] == 4,      $sformatf("T6: most reads outstanding %0d, want 4", c2[4]))
        `CHECK(c2[5] == 3,      $sformatf("T6: AR stalled %0d, want 3", c2[5]))
        `CHECK(c2[6] == 5,      $sformatf("T6: R stalled %0d, want 5", c2[6]))
        `CHECK(c2[7] == 0,      $sformatf("T6: word 31 = %0d, want 0", c2[7]))
        // uwin_ra's counters, in the same snapshot: words 64-76, then zeros
        ctr_line(8, c2);
        for (int j = 0; j < 8; j++)
            `CHECK(c2[j] == ra_ctr[64*j +: 64], $sformatf("T6: word %0d = %h, want uwin_ra's %0d", 64 + j, c2[j], j))
        ctr_line(9, c2);
        for (int j = 0; j < 8; j++)
            `CHECK(c2[j] == ((j < 5) ? ra_ctr[64*(8 + j) +: 64] : 64'd0), $sformatf("T6: word %0d = %h", 72 + j, c2[j]))
    end
    $display("ok   T6 axi_main counters (uwin_mon) through the clock crossing, writes and reads; uwin_ra's (64-76)");

    // --- T7: DISCARD. On (line 63), 40 bursts with AW running ahead of W (the
    //     ID ring fills: 32), IDs 0-3; nothing reaches memory, each B carries
    //     its burst's ID; off (line 62), writes land again; a switch asked for
    //     while a write is in flight waits for its B
    begin
        logic [63:0] c [8], c0 [8];
        int aw0, nb = 40;
        ctr_line(1, c0);
        ctr_line(63, c);                        // discard on
        repeat (5) @(negedge aclk);
        ctr_line(1, c);
        `CHECK(c[7] == 1, $sformatf("T7: mode %0d after the line-63 read, want 1", c[7]))
        aw0 = hbm_aw;
        t7_on = 1;
        fork
            for (int k = 0; k < nb; k++) begin
                @(negedge aclk); win_in.awaddr = BAR + 64'h8_0000 + 256*k; win_in.awlen = 3;
                win_in.awid = AXI_ID_BITS'(k % 4); win_in.awvalid = 1;
                do @(posedge aclk); while (!win_in.awready);
                @(negedge aclk); win_in.awvalid = 0;
            end
            begin
                repeat (60) @(negedge aclk);    // AW runs ahead: the ring fills
                for (int k = 0; k < 4*nb; k++) begin
                    @(negedge aclk); win_in.wdata = '1; win_in.wstrb = '1; win_in.wlast = (k % 4 == 3); win_in.wvalid = 1;
                    do @(posedge aclk); while (!win_in.wready);
                    @(negedge aclk); win_in.wvalid = 0;
                end
            end
        join
        wait (t7_b == nb);
        repeat (5) @(negedge aclk);
        t7_on = 0;
        win_in.awid = 0;
        `CHECK(hbm_aw == aw0, $sformatf("T7: %0d writes reached memory while discarding", hbm_aw - aw0))
        `CHECK(t7_ids.size() == 0, $sformatf("T7: %0d writes never answered", t7_ids.size()))
        ctr_line(1, c);
        `CHECK(c[4] - c0[4] == nb, $sformatf("T7: bursts discarded %0d, want %0d", c[4] - c0[4], nb))
        `CHECK(c[5] - c0[5] == 30, $sformatf("T7: bursts with a nonzero AWID %0d, want 30", c[5] - c0[5]))
        `CHECK(c[2] == 32,         $sformatf("T7: most writes outstanding %0d, want 32 (the ring's depth)", c[2]))
        ctr_line(62, c);                        // discard off
        repeat (5) @(negedge aclk);
        ctr_line(1, c);
        `CHECK(c[7] == 0, $sformatf("T7: mode %0d after the line-62 read, want 0", c[7]))
        // nothing landed where the discarded bursts went (memory reads zeros there)
        begin
            int got = 0;
            @(negedge aclk); card_in.araddr = UWIN_HBM_BASE + 64'h8_0000; card_in.arlen = 8'(4*nb - 1); card_in.arvalid = 1;
            do @(posedge aclk); while (!card_in.arready);
            @(negedge aclk); card_in.arvalid = 0;
            while (got < 4*nb) begin
                @(posedge aclk);
                if (card_in.rvalid && card_in.rready) begin
                    `CHECK(card_in.rdata == '0, $sformatf("T7: beat %0d of the discarded range was written", got))
                    got++;
                end
            end
        end
        // writes land again
        wr_burst(1, BAR + 64'h8_0000, UWIN_HBM_BASE + 64'h8_0000, 4, '1, 7);
        rd_check("T7 HBM again", 0, UWIN_HBM_BASE + 64'h8_0000, UWIN_HBM_BASE + 64'h8_0000, 4, 7);
        // a switch asked for mid-write waits for that write's B
        fork
            wr_burst(1, BAR + 64'hA_0000, UWIN_HBM_BASE + 64'hA_0000, 4, '1, 8);
            begin
                wait (win_hbm.awvalid && win_hbm.awready);
                ctr_line(63, c);
            end
        join
        rd_check("T7 mid-write", 0, UWIN_HBM_BASE + 64'hA_0000, UWIN_HBM_BASE + 64'hA_0000, 4, 8);
        ctr_line(1, c);
        `CHECK(c[7] == 1, $sformatf("T7: mode %0d after the held switch, want 1", c[7]))
        ctr_line(62, c);
        repeat (5) @(negedge aclk);
        ctr_line(1, c);
        `CHECK(c[7] == 0, "T7: back to HBM")
    end
    $display("ok   T7 DISCARD: on/off by counter-page reads, IDs answered, memory untouched, switch held mid-write");

    // --- T8: 64 reads in flight: once the window is idle, with R held back
    //     it takes exactly 64 single-line reads and holds the 65th, then
    //     answers all 64 (on the card the static allows 32) ---
    begin
        int acc = 0, got = 0, w;
        repeat (10) @(negedge aclk);       // T7's last read answered
        win_in.rready = 0;
        for (int k = 0; k < 70; k++) begin
            @(negedge aclk); win_in.araddr = BAR + 64'hE_0000 + 64*k; win_in.arlen = 0; win_in.arvalid = 1;
            w = 0;
            do begin @(posedge aclk); w++; end while (!win_in.arready && w < 300);
            if (win_in.arready) acc++;
            @(negedge aclk); win_in.arvalid = 0;
            if (w >= 300) break;
        end
        `CHECK(acc == 64, $sformatf("T8: %0d reads taken with R held back, want 64", acc))
        win_in.rready = 1;
        w = 0;
        while (got < acc && w < 5000) begin
            @(posedge aclk); w++;
            if (win_in.rvalid && win_in.rready) begin
                `CHECK(win_in.rlast, $sformatf("T8: R %0d without rlast", got))
                got++;
            end
        end
        `CHECK(got == 64, $sformatf("T8: %0d of 64 reads answered", got))
        repeat (20) @(negedge aclk);
        `CHECK(!win_in.rvalid, "T8: an extra R")
    end
    $display("ok   T8 64 reads in flight, the 65th held");

    // --- T9: the window's read counters (words 32-39): 16 reads of 4 beats
    //     taken with R held back for 40 cycles, then answered. Between the
    //     two counter reads: the 16, the first counter read's R, the second's
    //     AR. Most outstanding since reset: T8's 64
    begin
        logic [63:0] c0 [8], c [8];
        longint cyc, sum, ars, rs;
        int acc = 0, got = 0, w = 0;
        ctr_line(4, c0);
        @(negedge aclk); win_in.rready = 0;
        for (int k = 0; k < 16; k++) begin
            @(negedge aclk); win_in.araddr = BAR + 64'h4_0000 + 256*k; win_in.arlen = 3; win_in.arvalid = 1;
            do @(posedge aclk); while (!win_in.arready);
            acc++;
            @(negedge aclk); win_in.arvalid = 0;
        end
        repeat (40) @(negedge aclk);
        win_in.rready = 1;
        while (got < 64 && w < 5000) begin
            @(posedge aclk); w++;
            if (win_in.rvalid && win_in.rready) begin
                for (int b = 0; b < 64; b++)
                    `CHECK(win_in.rdata[8*b +: 8] == val(UWIN_HBM_BASE + 64'h4_0000 + 64*got + b, 4),
                           $sformatf("T9: byte %0d of beat %0d", b, got))
                `CHECK(win_in.rlast == (got % 4 == 3), $sformatf("T9: rlast on beat %0d", got))
                got++;
            end
        end
        `CHECK(acc == 16 && got == 64, $sformatf("T9: %0d reads taken, %0d of 64 beats", acc, got))
        repeat (10) @(negedge aclk);
        ctr_line(4, c);
        cyc = rd_cyc; sum = rd_sum; ars = rd_ar_stall; rs = rd_r_stall;
        `CHECK(c[0] - c0[0] == 17,  $sformatf("T9: AR bursts %0d, want 17", c[0] - c0[0]))
        `CHECK(c[1] - c0[1] == 65,  $sformatf("T9: R beats %0d, want 65", c[1] - c0[1]))
        `CHECK(c[2] - c0[2] == 17,  $sformatf("T9: reads completed %0d, want 17", c[2] - c0[2]))
        `CHECK(c[3] == cyc,         $sformatf("T9: cycles with a read outstanding %0d, port shows %0d", c[3], cyc))
        `CHECK(c[4] == sum,         $sformatf("T9: sum of reads outstanding %0d, port shows %0d", c[4], sum))
        `CHECK(c[3] - c0[3] >= 40 && c[4] - c0[4] >= 16*40,
               $sformatf("T9: %0d cycles, sum %0d outstanding with 16 held 40 cycles", c[3] - c0[3], c[4] - c0[4]))
        `CHECK(c[5] == 64,          $sformatf("T9: most reads outstanding %0d, want 64 (T8)", c[5]))
        `CHECK(c[6] == ars,         $sformatf("T9: AR stalled %0d, port shows %0d", c[6], ars))
        `CHECK(c[7] == rs && c[7] - c0[7] >= 40,
               $sformatf("T9: R stalled %0d (%0d here), port shows %0d", c[7], c[7] - c0[7], rs))
        ctr_line(5, c);
        for (int j = 0; j < 8; j++) `CHECK(c[j] == 0, $sformatf("T9: word %0d = %0d, want 0", 40 + j, c[j]))
    end
    $display("ok   T9 read counters: 16 reads held back then answered, exact against the port");

    // --- T10: read-ahead is off after reset: a sequential stream passes
    //     through (m_axi carries exactly the window's ARs, words 40-49 stay
    //     0); line 61 turns it on, line 60 off (word 14)
    begin
        rd_t rds [$];
        ra_snap_t a, b;
        longint cy;
        card_fill(R0, RSZ / 1024, 10);
        ra_snap(a);
        seq_rds(rds, R0, 64, 4);
        peer_rd("T10 off", rds, 32, cy);
        ra_snap(b);
        for (int i = 0; i < 10; i++) `CHECK(b.w[i] == 0, $sformatf("T10: word %0d = %0d with read-ahead off", 40 + i, b.w[i]))
        `CHECK(b.har - a.har == b.ar - a.ar && b.hrb - a.hrb == b.rb - a.rb,
               $sformatf("T10: m_axi %0d ARs %0d beats, the window %0d ARs %0d beats", b.har - a.har, b.hrb - a.hrb,
                         b.ar - a.ar, b.rb - a.rb))
        ra_set(1);
        ra_set(0);
    end
    $display("ok   T10 read-ahead off after reset: pass-through only; switched on and off");

    // --- T11: on: 512 sequential 256 B reads (128 KiB), 32 in flight, R
    //     backpressure: every byte, rlast, order. One stream (the first read
    //     passes through, the second starts it), ended by the idle timeout;
    //     exactly RA_LINES lines read ahead beyond the last request
    begin
        rd_t rds [$];
        ra_snap_t a, b;
        longint cy;
        ra_set(1);
        ra_snap(a);
        seq_rds(rds, R0, 512, 4);
        peer_rbp = 1;
        peer_rd("T11", rds, 32, cy);
        peer_rbp = 0; win_in.rready = 1;
        repeat (1100) @(negedge aclk);
        ra_snap(b);
        `CHECK(b.w[STREAMS] - a.w[STREAMS] == 1 && b.w[PASS] - a.w[PASS] == 1
               && (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE]) == 511 && b.w[SERVED] - a.w[SERVED] == 4 * 511,
               $sformatf("T11: streams %0d, passed %0d, hits %0d + late %0d, lines served %0d", b.w[STREAMS] - a.w[STREAMS],
                         b.w[PASS] - a.w[PASS], b.w[HIT] - a.w[HIT], b.w[LATE] - a.w[LATE], b.w[SERVED] - a.w[SERVED]))
        `CHECK((b.w[FILLED] - a.w[FILLED]) - (b.w[SERVED] - a.w[SERVED]) == RA_LINES,
               $sformatf("T11: %0d lines read ahead, %0d served", b.w[FILLED] - a.w[FILLED], b.w[SERVED] - a.w[SERVED]))
        `CHECK(b.w[END_I] - a.w[END_I] == 1 && b.w[END_J] == a.w[END_J] && b.w[END_W] == a.w[END_W],
               $sformatf("T11: ended by idle %0d, jump %0d, write %0d", b.w[END_I] - a.w[END_I], b.w[END_J] - a.w[END_J],
                         b.w[END_W] - a.w[END_W]))
        ra_xcheck("T11", a, b);
        $display("     T11 512 reads: %0d hits, %0d late, %0d read-ahead ARs, %0d cycles", b.w[HIT] - a.w[HIT],
                 b.w[LATE] - a.w[LATE], b.w[RA_AR] - a.w[RA_AR], cy);
    end
    $display("ok   T11 a 128 KiB stream with R backpressure: exact data, one stream, RA_LINES ahead, idle end");

    // --- T12: on: random mixes - runs of sequential reads of 1-16 beats,
    //     jumps, starts inside a line, counter-page reads - at 1, 4, 32, 64
    //     in flight, R backpressure: every byte, rlast, order
    begin
        rd_t rds [$];
        ra_snap_t a, b;
        longint cy;
        int nmax [4] = '{1, 4, 32, 64};
        ra_snap(a);
        peer_rbp = 1;
        foreach (nmax[i]) begin
            rds = {};
            mix(rds, 500);
            peer_rd($sformatf("T12 (%0d in flight)", nmax[i]), rds, nmax[i], cy);
        end
        peer_rbp = 0; win_in.rready = 1;
        repeat (1100) @(negedge aclk);
        ra_snap(b);
        ra_xcheck("T12", a, b);
        `CHECK(b.w[STREAMS] - a.w[STREAMS] > 20 && b.w[HIT] - a.w[HIT] > 100 && b.w[END_J] - a.w[END_J] > 20
               && b.w[PASS] - a.w[PASS] > 100,
               $sformatf("T12: streams %0d, hits %0d, ended by a jump %0d, passed %0d: the mix exercised too little",
                         b.w[STREAMS] - a.w[STREAMS], b.w[HIT] - a.w[HIT], b.w[END_J] - a.w[END_J], b.w[PASS] - a.w[PASS]))
        $display("     T12 2000 reads: %0d streams, %0d hits, %0d late, %0d passed through, ended by a jump %0d",
                 b.w[STREAMS] - a.w[STREAMS], b.w[HIT] - a.w[HIT], b.w[LATE] - a.w[LATE], b.w[PASS] - a.w[PASS],
                 b.w[END_J] - a.w[END_J]);
    end
    $display("ok   T12 random mixes at 1-64 in flight: exact data, rlast, order");

    // --- T13: a window write ends the stream. (a) 8 KiB into a stream (read
    //     ahead to +24 KiB) the peer writes 1 KiB at +8 KiB, then continues:
    //     it reads the new data. (b) the same with 32 reads in flight, the
    //     written range read only after the write's last B
    begin
        rd_t rds [$];
        ra_snap_t a, b;
        longint cy;
        longint A = R0 + 64'h1_0000, B = R0 + 64'h2_0000;
        ra_snap(a);
        seq_rds(rds, A, 32, 4);
        peer_rd("T13a before", rds, 32, cy);
        repeat (300) @(negedge aclk);
        for (int k = 0; k < 4; k++) begin
            wr_burst(1, BAR + A + 8192 + 256*k, UWIN_HBM_BASE + A + 8192 + 256*k, 4, '1, 13);
            for (int j = 0; j < 4; j++) seed_of[(A + 8192 + 256*k) / 64 + j] = 13;
        end
        rds = {};
        seq_rds(rds, A + 8192, 64, 4);
        peer_rd("T13a after", rds, 32, cy);
        repeat (1100) @(negedge aclk);
        ra_snap(b);
        `CHECK(b.w[END_W] - a.w[END_W] == 1 && b.w[STREAMS] - a.w[STREAMS] == 2 && b.w[END_I] - a.w[END_I] == 1
               && (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE]) == 95,
               $sformatf("T13: ended by a write %0d, streams %0d, idle %0d, stream reads %0d", b.w[END_W] - a.w[END_W],
                         b.w[STREAMS] - a.w[STREAMS], b.w[END_I] - a.w[END_I], (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE])))
        // (b)
        ra_snap(a);
        rds = {};
        seq_rds(rds, B, 256, 4);
        gate_lo = B + 16384; gate_hi = B + 17408; gate_open = 0;
        fork
            peer_rd("T13b", rds, 32, cy);
            begin
                wait (peer_front >= B + 8192);
                for (int k = 0; k < 4; k++) begin
                    wr_burst(1, BAR + B + 16384 + 256*k, UWIN_HBM_BASE + B + 16384 + 256*k, 4, '1, 14);
                    for (int j = 0; j < 4; j++) seed_of[(B + 16384 + 256*k) / 64 + j] = 14;
                end
                gate_open = 1;
            end
        join
        gate_lo = -1; gate_hi = -1;
        repeat (1100) @(negedge aclk);
        ra_snap(b);
        ra_xcheck("T13b", a, b);
        `CHECK(b.w[END_W] - a.w[END_W] >= 1 && b.w[STREAMS] - a.w[STREAMS] >= 2,
               $sformatf("T13b: ended by a write %0d, streams %0d", b.w[END_W] - a.w[END_W], b.w[STREAMS] - a.w[STREAMS]))
    end
    $display("ok   T13 a window write ends the stream: later reads return the new data (alone, and with 32 in flight)");

    // --- T14: the idle timeout. 8 KiB into a stream, RA_IDLE cycles without
    //     a read, the card path writes 1 KiB at +8 KiB (the ring held it):
    //     continuing reads the new data. Within RA_IDLE the ring still answers
    //     with what it fetched before such a write (the documented exposure);
    //     a re-read (not sequential) returns the new data
    begin
        rd_t rds [$];
        ra_snap_t a, b;
        longint cy;
        longint C = R0 + 64'h3_0000, D = R0 + 64'h3_8000;
        ra_snap(a);
        seq_rds(rds, C, 32, 4);
        peer_rd("T14 before", rds, 32, cy);
        repeat (1100) @(negedge aclk);
        card_fill(C + 8192, 1, 15);
        rds = {};
        seq_rds(rds, C + 8192, 32, 4);
        peer_rd("T14 after idle", rds, 32, cy);
        repeat (1100) @(negedge aclk);
        rds = {};
        seq_rds(rds, D, 32, 4);
        peer_rd("T14 exposure, before", rds, 32, cy);
        repeat (300) @(negedge aclk);
        wr_burst(0, UWIN_HBM_BASE + D + 8192, UWIN_HBM_BASE + D + 8192, 16, '1, 16);   // seed_of: still the old data
        rds = {};
        seq_rds(rds, D + 8192, 4, 4);
        peer_rd("T14 exposure (old data expected)", rds, 32, cy);
        for (int j = 0; j < 16; j++) seed_of[(D + 8192) / 64 + j] = 16;
        rds = '{'{D + 8192, 16}};
        peer_rd("T14 re-read", rds, 32, cy);
        repeat (1100) @(negedge aclk);
        ra_snap(b);
        `CHECK(b.w[END_I] - a.w[END_I] == 2 && b.w[END_J] - a.w[END_J] == 1 && b.w[STREAMS] - a.w[STREAMS] == 3,
               $sformatf("T14: ended by idle %0d, jump %0d, streams %0d", b.w[END_I] - a.w[END_I], b.w[END_J] - a.w[END_J],
                         b.w[STREAMS] - a.w[STREAMS]))
    end
    $display("ok   T14 idle timeout drops the ring (card-path write seen); within RA_IDLE the documented exposure");

    // --- T15: switched off mid-stream (line 60 among 32 reads in flight):
    //     the stream ends, later reads pass through, no read-ahead AR; on
    //     again (line 61) mid-run: a new stream
    begin
        rd_t rds [$];
        ra_snap_t a, m, b;
        logic [63:0] c [8];
        longint cy;
        longint E = R0 + 64'h1_8000;
        seq_rds(rds, E, 16, 4);
        rds.push_back('{UWIN_SIZE - 4096 + 60*64, 1});
        seq_rds(rds, E + 4096, 16, 4);
        peer_rd("T15 off", rds, 32, cy);
        ctr_line(1, c);
        `CHECK(c[6] == 0, "T15: read-ahead still on after line 60")
        repeat (1100) @(negedge aclk);
        ra_snap(a);
        rds = {};
        seq_rds(rds, E + 8192, 32, 4);
        peer_rd("T15 off, then", rds, 32, cy);
        ra_snap(m);
        for (int i = 0; i < 10; i++) `CHECK(m.w[i] == a.w[i], $sformatf("T15: word %0d moved while off", 40 + i))
        `CHECK(m.har - a.har == m.ar - a.ar, "T15: m_axi carried more than the window's reads while off")
        rds = {};
        seq_rds(rds, E + 16384, 8, 4);
        rds.push_back('{UWIN_SIZE - 4096 + 61*64, 1});
        seq_rds(rds, E + 16384 + 2048, 24, 4);
        peer_rd("T15 on", rds, 32, cy);
        repeat (1100) @(negedge aclk);
        ra_snap(b);
        ctr_line(1, c);
        `CHECK(c[6] == 1, "T15: read-ahead off after line 61")
        `CHECK(b.w[STREAMS] - m.w[STREAMS] == 1 && b.w[PASS] - m.w[PASS] == 1
               && (b.w[HIT] - m.w[HIT]) + (b.w[LATE] - m.w[LATE]) == 23 && b.w[END_I] - m.w[END_I] == 1,
               $sformatf("T15: streams %0d, passed %0d, stream reads %0d, idle end %0d", b.w[STREAMS] - m.w[STREAMS],
                         b.w[PASS] - m.w[PASS], (b.w[HIT] - m.w[HIT]) + (b.w[LATE] - m.w[LATE]), b.w[END_I] - m.w[END_I]))
        ra_xcheck("T15", m, b);
    end
    $display("ok   T15 switched off and on mid-run: stream ended, pass-through only while off, a new stream after");

    // --- T16: counters exact. One read at a time: the first passes through,
    //     the second starts the stream (late), then - the read-ahead well
    //     ahead - 62 hits; the idle timeout ends it. Then three streams,
    //     ended by a jump, a counter-page read, a write
    begin
        rd_t rds [$];
        ra_snap_t a, b;
        longint cy;
        longint F = R0 + 64'h2_8000;
        ra_snap(a);
        seq_rds(rds, F, 2, 4);
        peer_rd("T16", rds, 1, cy);
        repeat (200) @(negedge aclk);
        rds = {};
        seq_rds(rds, F + 512, 62, 4);
        peer_rd("T16", rds, 1, cy);
        repeat (1100) @(negedge aclk);
        ra_snap(b);
        `CHECK(b.w[HIT] - a.w[HIT] == 62, $sformatf("T16: hits %0d, want 62", b.w[HIT] - a.w[HIT]))
        `CHECK(b.w[LATE] - a.w[LATE] == 1, $sformatf("T16: late %0d, want 1", b.w[LATE] - a.w[LATE]))
        `CHECK(b.w[PASS] - a.w[PASS] == 1, $sformatf("T16: passed %0d, want 1", b.w[PASS] - a.w[PASS]))
        `CHECK(b.w[STREAMS] - a.w[STREAMS] == 1, $sformatf("T16: streams %0d, want 1", b.w[STREAMS] - a.w[STREAMS]))
        `CHECK(b.w[SERVED] - a.w[SERVED] == 252, $sformatf("T16: lines served %0d, want 252", b.w[SERVED] - a.w[SERVED]))
        `CHECK(b.w[FILLED] - a.w[FILLED] == 252 + RA_LINES,
               $sformatf("T16: lines read ahead %0d, want %0d", b.w[FILLED] - a.w[FILLED], 252 + RA_LINES))
        // 4 + 252 + 256 lines from line 8 of a 16-line burst: 8 + 31 bursts
        `CHECK(b.w[RA_AR] - a.w[RA_AR] == 32, $sformatf("T16: read-ahead ARs %0d, want 32", b.w[RA_AR] - a.w[RA_AR]))
        `CHECK(b.w[END_I] - a.w[END_I] == 1 && b.w[END_J] == a.w[END_J] && b.w[END_W] == a.w[END_W],
               $sformatf("T16: ended by idle %0d, jump %0d, write %0d", b.w[END_I] - a.w[END_I], b.w[END_J] - a.w[END_J],
                         b.w[END_W] - a.w[END_W]))
        ra_xcheck("T16", a, b);
        ra_snap(a);
        rds = {};
        seq_rds(rds, F + 64'h1_0000, 4, 4);
        rds.push_back('{F, 4});                                 // a jump
        peer_rd("T16 jump", rds, 1, cy);
        repeat (1100) @(negedge aclk);
        rds = {};
        seq_rds(rds, F + 64'h1_1000, 4, 4);
        rds.push_back('{UWIN_SIZE - 4096, 1});                  // the counter page
        peer_rd("T16 counter page", rds, 1, cy);
        repeat (1100) @(negedge aclk);
        rds = {};
        seq_rds(rds, F + 64'h1_2000, 4, 4);
        peer_rd("T16 write", rds, 1, cy);
        wr_burst(1, BAR + R0 + RSZ - 256, UWIN_HBM_BASE + R0 + RSZ - 256, 4, '1, 17);
        for (int j = 0; j < 4; j++) seed_of[(R0 + RSZ - 256) / 64 + j] = 17;
        repeat (1100) @(negedge aclk);
        ra_snap(b);
        `CHECK(b.w[END_J] - a.w[END_J] == 2 && b.w[END_W] - a.w[END_W] == 1 && b.w[END_I] == a.w[END_I]
               && b.w[STREAMS] - a.w[STREAMS] == 3 && b.w[PASS] - a.w[PASS] == 4
               && (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE]) == 9,
               $sformatf("T16: ended by a jump %0d, write %0d, idle %0d; streams %0d, passed %0d, stream reads %0d",
                         b.w[END_J] - a.w[END_J], b.w[END_W] - a.w[END_W], b.w[END_I] - a.w[END_I], b.w[STREAMS] - a.w[STREAMS],
                         b.w[PASS] - a.w[PASS], (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE])))
        ra_xcheck("T16 ends", a, b);
    end
    $display("ok   T16 read-ahead counters exact: hits, late, passed, lines, ARs, streams, each end");

    // --- T17: a stream up to the end of the window's HBM part: read-ahead
    //     stops there (nothing fetched beyond the last request), the next
    //     line is the counter page: the stream ends
    begin
        rd_t rds [$];
        ra_snap_t a, b;
        longint cy;
        longint G = UWIN_SIZE - 4096 - 8192;
        card_fill(G, 8, 18);
        ra_snap(a);
        seq_rds(rds, G, 32, 4);
        rds.push_back('{UWIN_SIZE - 4096, 2});
        peer_rd("T17", rds, 32, cy);
        repeat (1100) @(negedge aclk);
        ra_snap(b);
        `CHECK(b.w[FILLED] - a.w[FILLED] == b.w[SERVED] - a.w[SERVED] && b.w[SERVED] - a.w[SERVED] == 124,
               $sformatf("T17: %0d lines read ahead, %0d served, want 124 both", b.w[FILLED] - a.w[FILLED],
                         b.w[SERVED] - a.w[SERVED]))
        `CHECK(b.w[END_J] - a.w[END_J] == 1 && b.w[STREAMS] - a.w[STREAMS] == 1 && b.w[END_I] == a.w[END_I],
               $sformatf("T17: ended by a jump %0d, idle %0d; streams %0d", b.w[END_J] - a.w[END_J], b.w[END_I] - a.w[END_I],
                         b.w[STREAMS] - a.w[STREAMS]))
        ra_xcheck("T17", a, b);
    end
    $display("ok   T17 read-ahead stops at the counter page");

    // --- T18: what read-ahead buys: memory latency 200 cycles, R beats
    //     back to back, the peer 32 x 256 B in flight, 64 KiB sequential
    begin
        rd_t rds [$];
        ra_snap_t a, b;
        longint c_off, c_on;
        mem_lat = 200; mem_fast = 1;
        ra_set(0);
        seq_rds(rds, R0, 256, 4);
        peer_rd("T18 off", rds, 32, c_off);
        ra_set(1);
        ra_snap(a);
        rds = {};
        seq_rds(rds, R0 + 64'h1_0000, 256, 4);
        peer_rd("T18 on", rds, 32, c_on);
        repeat (1100) @(negedge aclk);
        ra_snap(b);
        mem_lat = 0; mem_fast = 0;
        $display("     T18 64 KiB, latency 200, 32 x 256 B in flight: off %0d cycles (%.1f B/cycle), on %0d cycles (%.1f B/cycle), %0d hits of %0d",
                 c_off, 65536.0 / c_off, c_on, 65536.0 / c_on, b.w[HIT] - a.w[HIT], b.w[HIT] - a.w[HIT] + b.w[LATE] - a.w[LATE]);
        `CHECK(4 * c_on < 3 * c_off, $sformatf("T18: on %0d cycles, off %0d: not clearly faster", c_on, c_off))
        ra_xcheck("T18", a, b);
    end
    $display("ok   T18 read-ahead on clearly faster than off at memory latency 200");

    `CHECK(hbm_ctr_ar == win_ctr_ar, $sformatf("m_axi: %0d ARs into the counter page, the peer read it %0d times",
                                               hbm_ctr_ar, win_ctr_ar))
    if (errors == 0) $display("TB PASS (tb_uwin_hbm)");
    else             $display("TB FAIL (tb_uwin_hbm): %0d errors", errors);
    $finish;
end

initial begin
    #5ms;
    $display("TB FAIL (tb_uwin_hbm): timeout");
    $finish;
end

endmodule
