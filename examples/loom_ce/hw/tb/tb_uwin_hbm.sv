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
 * reads (uwin_hbm makes it: the stripe raises it on a fragment's every beat).
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

uwin_hbm #(.BASE(UWIN_HBM_BASE)) inst_uwin_hbm (.aclk(aclk), .aresetn(aresetn), .s_axi(win_in), .m_axi(win_hbm));
axi_stripe #(.N_STAGES(0)) inst_strp_win  (.aclk(aclk), .aresetn(aresetn), .s_axi(win_hbm), .m_axi(win_mem));
axi_stripe #(.N_STAGES(0)) inst_strp_card (.aclk(aclk), .aresetn(aresetn), .s_axi(card_in), .m_axi(card_mem));

int errors = 0;
`define CHECK(c, msg) if (!(c)) begin errors++; $display("FAIL [%0t] %s", $time, msg); end

// ---------------------------------------------------------------------------
// Memory: bytes by address, shared by both ports; each port an AXI4 slave
// (AW/W in order, B per AW; AR -> R beats), random ready gaps
// ---------------------------------------------------------------------------
byte unsigned mem [longint];

`define MEM_PORT(P)                                                                     \
    typedef struct { longint addr; int len; logic [AXI_ID_BITS-1:0] id; } P``_aw_t;    \
    P``_aw_t P``_awq [$], P``_arq [$];                                                  \
    logic [AXI_ID_BITS-1:0] P``_bq [$];                                                 \
    initial begin                                                                       \
        P.awready = 0; P.wready = 0; P.bvalid = 0; P.arready = 0; P.rvalid = 0;         \
        P.bresp = 0; P.rresp = 0; P.rlast = 0; P.rdata = 0; P.bid = 0; P.rid = 0;       \
    end                                                                                 \
    always @(posedge aclk) begin                                                        \
        P.awready <= ($urandom_range(0, 3) != 0);                                      \
        P.arready <= ($urandom_range(0, 3) != 0);                                      \
        if (P.awvalid && P.awready)                                                     \
            P``_awq.push_back('{P.awaddr & ~64'd63, P.awlen + 1, P.awid});             \
        if (P.arvalid && P.arready)                                                     \
            P``_arq.push_back('{P.araddr & ~64'd63, P.arlen + 1, P.arid});             \
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
        for (int j = 0; j < t.len; j++) begin                                           \
            logic [AXI_DATA_BITS-1:0] d;                                                \
            for (int b = 0; b < 64; b++)                                                \
                d[8*b +: 8] = mem.exists(t.addr + 64*j + b) ? mem[t.addr + 64*j + b] : 8'h00; \
            @(negedge aclk); P.rdata = d; P.rid = t.id; P.rlast = (j == t.len - 1); P.rvalid = 1; \
            do @(posedge aclk); while (!P.rready);                                      \
            @(negedge aclk); P.rvalid = 0;                                              \
        end                                                                             \
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
localparam logic [63:0] HI32 = 64'hFFFF_FFFF_0000_0000;
localparam logic [63:0] LO32 = 64'h0000_0000_FFFF_FFFF;

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
