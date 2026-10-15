`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * tb_uwin_ra - uwin_ra on its own, in xclk (333 MHz): the static's axi_main
 * (a peer) -> uwin_ra -> a model of the shell's interconnect and what is
 * behind it.
 *
 * The peer issues reads in order, at most N in flight, a read's slot free
 * again up_dly cycles after its last beat (the time upstream, ~110 ns on the
 * V80), and checks every beat against what each line / register was last
 * written with: data, rlast, rid, rresp, order. Random R backpressure while
 * peer_rbp. Its writes wait for their B.
 * The model answers in AR order (one ID, as the shell's interconnect does for
 * axi_main), every read ds_lat cycles after its AR at the earliest (the round
 * trip through the clock crossing, uwin_hbm and HBM: ~160 ns + memory), its
 * beats at most 3 in 4 cycles while ds_slow (aclk's 250 MHz), random gaps and
 * ready drops while ds_rand. Window: memory by line; below it: registers (8 B,
 * any lane). A read's data is what was there when its AR was taken; a write
 * lands when its B is sent (ds_wlat cycles after its last W), so a fill taken
 * before a write's B returns the old data.
 *
 * Covers: off after reset is exact pass-through - every AR and R signal equal
 * on both ports, every cycle, for a mix of window and register reads and
 * writes at 1-32 in flight, counters 0-11 at 0 (T1); the switch, lines 59/58,
 * word 0, back to pass-through once drained (T2); on: a 128 KiB stream at 8
 * in flight with R backpressure, one stream, RA_LINES ahead, idle end (T3);
 * random mixes of runs, jumps, starts inside a line, counter-page and
 * register reads (IDs 0 and 3), register writes alongside, at 1-64 in flight,
 * random backpressure and gaps on both ports (T4); a window write ends the
 * stream, later reads see it, alone and with 8 in flight; a register write
 * does not end it; no stream starts while a write is outstanding (T5); the
 * idle timeout drops the ring, within it the documented exposure to card-path
 * writes (T6); switched on and off mid-run with reads in flight both ways and
 * R backpressure (T7); counters exact (T8); a constant nonzero ID (T9); 64
 * reads held with R stopped, ARs held beyond (T10); what it buys: off vs on at 8 x 256 B in
 * flight, ds_lat 142 (a read ~441 ns at axi_main off, as measured), 110 ns
 * upstream, 3 beats in 4 cycles (T11); the end of a short stream, the
 * jump's data right behind the fill's last beat (T12); switched on right
 * before a stream, reads still out from PASS when its fill heads mq (T13);
 * W behind AW: back-to-back 4-beat window writes, AW with the first W beat
 * (as the static), W first, AW first, then random backpressure and gaps:
 * one burst every 4 cycles at m_axi, every first beat W_LAG cycles or more
 * after its AW (T14).
 * Always: AW and B are wires; W beats leave in order and unchanged, never
 * dropped or changed while they wait, and a burst's first beat at m_axi
 * comes W_LAG or more cycles after m_axi took its AW; ARs and R never drop a valid or change while waiting (both DUT
 * outputs); in TRACK every AR reaches the decision unchanged and in order,
 * every AR passed through reaches m_axi unchanged and in order, every fill is
 * a read of 1..RA_BURST window lines inside one RA_BURST block, below the
 * counter page, with the stream's ID; m_axi never has more than 64 reads
 * outstanding; fills and lines read ahead equal m_axi's traffic beyond the
 * reads passed through (xcheck); only the peer reads the counter page.
 */
module tb_uwin_ra;

logic xclk = 0;
always #1.5 xclk = ~xclk;
logic xresetn = 0;

localparam longint WIN       = 64'h0800_0000;       // the window on axi_main
localparam longint UWIN_SIZE = 64'h0800_0000;
localparam longint CTRP      = WIN + UWIN_SIZE - 4096;
localparam longint REGR      = 64'h0010_0000;       // registers the peer reads
localparam longint REGW      = 64'h0010_1000;       // ... and writes (read back only after)
localparam int     RA_LINES = 256, RA_BURST = 16, RA_IDLE = 1024, RA_LAG = 192;

AXI4 s (.aclk(xclk));               // axi_main from the static
AXI4 m (.aclk(xclk));               // into the interconnect
logic [64*24-1:0] ctr;
uwin_ra dut (.xclk(xclk), .xresetn(xresetn), .s_axi(s), .m_axi(m), .ctr(ctr));

int errors = 0;
`define CHECK(c, msg) if (!(c)) begin errors++; if (errors <= 40) $display("FAIL [%0t] %s", $time, msg); end

longint cyc = 0;
always @(posedge xclk) cyc <= cyc + 1;

function automatic logic [511:0] line_val(input longint ln, input int seed);
    logic [511:0] d;
    for (int j = 0; j < 8; j++)
        d[64*j +: 64] = (ln * 64'h9E37_79B9_7F4A_7C15) ^ (64'(seed) * 64'hC2B2_AE3D_27D4_EB4F) ^ (64'(j) << 56) ^ ln;
    return d;
endfunction
function automatic logic [63:0] reg_init(input longint a);
    return (a * 64'h1000_0000_01B3) ^ 64'hA5A5_0000_0000_5A5A;
endfunction

// ---------------------------------------------------------------------------
// The model: the interconnect and what is behind it
// ---------------------------------------------------------------------------
logic [511:0] mem [longint];        // window lines written (else line_val(ln, 0))
logic [63:0]  regs [longint];       // registers written (else reg_init)
int  ds_lat = 20, ds_wlat = 10;
bit  ds_slow = 0, ds_rand = 1;

function automatic logic [511:0] mem_rd(input longint a);
    if (a >= WIN && a < WIN + UWIN_SIZE) begin
        longint ln = (a - WIN) / 64;
        return mem.exists(ln) ? mem[ln] : line_val(ln, 0);
    end else begin
        longint r = a & ~64'd7;
        return {8{regs.exists(r) ? regs[r] : reg_init(r)}};
    end
endfunction

typedef struct { logic [511:0] d [$]; logic [AXI_ID_BITS-1:0] id; longint t; } mrd_t;
mrd_t m_rq [$];
int   m_rbeat = 0;
bit   m_r_hs = 0;
initial begin
    m.arready = 0; m.awready = 0; m.wready = 0; m.rvalid = 0; m.bvalid = 0;
    m.rdata = 0; m.rid = 0; m.rlast = 0; m.rresp = 0; m.bid = 0; m.bresp = 0;
end
always @(posedge xclk) begin
    m.arready <= ds_rand ? ($urandom_range(0, 3) != 0) : 1'b1;
    if (m.arvalid && m.arready) begin
        automatic mrd_t e;
        for (int j = 0; j <= m.arlen; j++) e.d.push_back(mem_rd(m.araddr + longint'(j) * (64'd1 << m.arsize)));
        e.id = m.arid;
        e.t = cyc;
        m_rq.push_back(e);
    end
    m_r_hs = m.rvalid && m.rready;
    if (m_r_hs) begin
        m_rbeat++;
        if (m_rbeat == m_rq[0].d.size()) begin
            void'(m_rq.pop_front());
            m_rbeat = 0;
        end
    end
end
always @(negedge xclk) if (!m.rvalid || m_r_hs) begin
    if (m_rq.size() > 0 && cyc >= m_rq[0].t + ds_lat && !(ds_slow && (cyc % 4 == 3))
        && !(ds_rand && $urandom_range(0, 4) == 0)) begin
        m.rdata = m_rq[0].d[m_rbeat]; m.rid = m_rq[0].id; m.rresp = 0;
        m.rlast = (m_rbeat == m_rq[0].d.size() - 1); m.rvalid = 1;
    end else begin
        m.rvalid = 0;
    end
end

typedef struct { longint addr; int len; int size; logic [AXI_ID_BITS-1:0] id; } maw_t;
typedef struct { longint addr; logic [511:0] d; logic [63:0] strb; } mwb_t;
typedef struct { mwb_t b [$]; logic [AXI_ID_BITS-1:0] id; longint t; } mb_t;
maw_t m_awq [$];
mwb_t m_wcur [$];
mb_t  m_bq [$];
int   m_wbeat = 0;
bit   m_b_hs = 0;
always @(posedge xclk) begin
    m.awready <= ds_rand ? ($urandom_range(0, 3) != 0) : 1'b1;
    m.wready  <= ds_rand ? ($urandom_range(0, 3) != 0) : 1'b1;
    if (m.awvalid && m.awready) m_awq.push_back('{m.awaddr, m.awlen + 1, m.awsize, m.awid});
    if (m.wvalid && m.wready) begin
        `CHECK(m_awq.size() > 0, "model: a W beat before its AW")
        if (m_awq.size() > 0) begin
            m_wcur.push_back('{m_awq[0].addr + longint'(m_wbeat) * (64'd1 << m_awq[0].size), m.wdata, m.wstrb});
            m_wbeat++;
            `CHECK(m.wlast == (m_wbeat == m_awq[0].len), "model: wlast")
            if (m.wlast) begin
                automatic mb_t e;
                e.b = m_wcur; e.id = m_awq[0].id; e.t = cyc;
                m_bq.push_back(e);
                m_wcur = {};
                m_wbeat = 0;
                void'(m_awq.pop_front());
            end
        end
    end
    m_b_hs = m.bvalid && m.bready;
    if (m_b_hs) void'(m_bq.pop_front());
end
// a write lands when its B is sent
always @(negedge xclk) if (!m.bvalid || m_b_hs) begin
    if (m_bq.size() > 0 && cyc >= m_bq[0].t + ds_wlat) begin
        foreach (m_bq[0].b[k]) begin
            automatic longint a = m_bq[0].b[k].addr;
            if (a >= WIN && a < WIN + UWIN_SIZE) begin
                automatic logic [511:0] o = mem_rd(a);
                for (int b = 0; b < 64; b++) if (m_bq[0].b[k].strb[b]) o[8*b +: 8] = m_bq[0].b[k].d[8*b +: 8];
                mem[(a - WIN) / 64] = o;
            end else begin
                for (int l = 0; l < 8; l++)
                    if (m_bq[0].b[k].strb[8*l +: 8] == 8'hFF) regs[(a & ~64'd63) + 8*l] = m_bq[0].b[k].d[64*l +: 64];
            end
        end
        m.bid = m_bq[0].id; m.bresp = 0; m.bvalid = 1;
    end else begin
        m.bvalid = 0;
    end
end

// ---------------------------------------------------------------------------
// The peer
// ---------------------------------------------------------------------------
initial begin
    s.arvalid = 0; s.awvalid = 0; s.wvalid = 0; s.rready = 1; s.bready = 1;
    s.araddr = 0; s.arid = 0; s.arlen = 0; s.arsize = 6; s.arburst = 1; s.arcache = 0; s.arlock = 0;
    s.arprot = 0; s.arqos = 0; s.arregion = 0;
    s.awaddr = 0; s.awid = 0; s.awlen = 0; s.awsize = 6; s.awburst = 1; s.awcache = 0; s.awlock = 0;
    s.awprot = 0; s.awqos = 0; s.awregion = 0;
    s.wdata = 0; s.wstrb = 0; s.wlast = 0;
end

int          exp_seed [longint];    // the seed each window line was last written with
logic [63:0] exp_reg [longint];
function automatic logic [511:0] exp_rd(input longint a);
    if (a >= WIN && a < WIN + UWIN_SIZE) begin
        longint ln = (a - WIN) / 64;
        return line_val(ln, exp_seed.exists(ln) ? exp_seed[ln] : 0);
    end else begin
        longint r = a & ~64'd7;
        return {8{exp_reg.exists(r) ? exp_reg[r] : reg_init(r)}};
    end
endfunction

typedef struct { longint addr; int beats; int size; logic [AXI_ID_BITS-1:0] id; } rd_t;
int     up_dly = 0;
bit     peer_rbp = 0;
always @(negedge xclk) if (peer_rbp) s.rready = ($urandom_range(0, 2) != 0);
longint lt_sum = 0, lt_n = 0;       // AR to last R (cycles), reads
bit     gate_open = 1;
longint gate_lo = -1, gate_hi = -1;
longint peer_front = -1;            // the last read issued
longint n_nzid = 0;                 // ARs with a nonzero ID issued

// reads rds in order, at most max_out in flight; cycles: first AR to last R
task automatic peer_rd(input string name, input rd_t rds [$], input int max_out, output longint cycles);
    int out = 0;
    longint t_ar [$], rel [$];          // reads' AR cycles; cycles their slots free again
    longint t0 = cyc, t1;
    fork
        begin
            @(negedge xclk);
            foreach (rds[k]) begin
                while (rel.size() > 0 && rel[0] <= cyc) begin void'(rel.pop_front()); out--; end
                while (out >= max_out) begin
                    @(negedge xclk);
                    while (rel.size() > 0 && rel[0] <= cyc) begin void'(rel.pop_front()); out--; end
                end
                if (rds[k].addr >= gate_lo && rds[k].addr < gate_hi) while (!gate_open) @(negedge xclk);
                s.araddr = rds[k].addr; s.arlen = 8'(rds[k].beats - 1); s.arsize = 3'(rds[k].size);
                s.arid = rds[k].id; s.arvalid = 1;
                do @(posedge xclk); while (!s.arready);
                t_ar.push_back(cyc);
                out++;
                if (rds[k].id != 0) n_nzid++;
                peer_front = rds[k].addr;
                @(negedge xclk); s.arvalid = 0;
            end
        end
        begin
            foreach (rds[k]) begin
                for (int j = 0; j < rds[k].beats; j++) begin
                    logic [511:0] e;
                    do @(posedge xclk); while (!(s.rvalid && s.rready));
                    e = exp_rd(rds[k].addr + longint'(j) * (64'd1 << rds[k].size));
                    `CHECK(s.rdata === e, $sformatf("%s: read %0d (%h) beat %0d: data %h..., want %h...", name, k,
                                                    rds[k].addr, j, s.rdata[63:0], e[63:0]))
                    `CHECK(s.rlast === (j == rds[k].beats - 1) && s.rresp === 2'b00 && s.rid === rds[k].id,
                           $sformatf("%s: read %0d beat %0d: rlast %0d resp %0d id %0d (want %0d)", name, k, j, s.rlast,
                                     s.rresp, s.rid, rds[k].id))
                    if (j == rds[k].beats - 1) begin
                        lt_sum += cyc - t_ar[k];
                        lt_n++;
                        if (up_dly == 0) out--;
                        else rel.push_back(cyc + up_dly);
                    end
                end
            end
            t1 = cyc;
        end
    join
    cycles = t1 - t0;
endtask

// writes `beats` full lines at window address a (line aligned), lines val(ln, seed); waits for B
semaphore wr_sem = new(1);
task automatic wr_win(input longint a, input int beats, input int seed);
    wr_sem.get(1);
    @(negedge xclk); s.awaddr = a; s.awlen = 8'(beats - 1); s.awsize = 6; s.awid = 0; s.awvalid = 1;
    do @(posedge xclk); while (!s.awready);
    @(negedge xclk); s.awvalid = 0;
    for (int j = 0; j < beats; j++) begin
        s.wdata = line_val((a - WIN) / 64 + j, seed); s.wstrb = '1; s.wlast = (j == beats - 1); s.wvalid = 1;
        do @(posedge xclk); while (!s.wready);
        @(negedge xclk); s.wvalid = 0;
    end
    do @(posedge xclk); while (!(s.bvalid && s.bready));
    for (int j = 0; j < beats; j++) exp_seed[(a - WIN) / 64 + j] = seed;
    @(negedge xclk);
    wr_sem.put(1);
endtask
// an 8 B register write
task automatic wr_reg(input longint a, input logic [63:0] v);
    wr_sem.get(1);
    @(negedge xclk); s.awaddr = a; s.awlen = 0; s.awsize = 3; s.awid = 0; s.awvalid = 1;
    do @(posedge xclk); while (!s.awready);
    @(negedge xclk); s.awvalid = 0;
    s.wdata = {8{v}}; s.wstrb = 64'hFF << (8 * a[5:3]); s.wlast = 1; s.wvalid = 1;
    do @(posedge xclk); while (!s.wready);
    @(negedge xclk); s.wvalid = 0;
    do @(posedge xclk); while (!(s.bvalid && s.bready));
    exp_reg[a] = v;
    @(negedge xclk);
    wr_sem.put(1);
endtask
// the card path (the copy engine) writes HBM behind everything: the model's memory
function automatic void card_wr(input longint a, input int lines, input int seed);
    for (int j = 0; j < lines; j++) mem[(a - WIN) / 64 + j] = line_val((a - WIN) / 64 + j, seed);
endfunction

function automatic void seq_rds(ref rd_t rds [$], input longint a, input int n, input int beats,
                                input logic [AXI_ID_BITS-1:0] id = 0);
    for (int k = 0; k < n; k++) rds.push_back('{a + 64*beats*k, beats, 6, id});
endfunction

localparam longint R0 = WIN + 64'h40_0000, RSZ = 64'h4_0000;     // 256 KiB the mixes read
// a random mix in [R0, R0 + RSZ): runs of sequential reads (1-16 beats, not
// across 4 KiB), jumps, starts inside a line, counter-page reads (lines 0-7,
// 8 B or 64 B), register reads (8 B); IDs 0, and 3 one time in ten if ids
function automatic void mix(ref rd_t rds [$], input int n, input bit ids);
    int bl [7] = '{1, 2, 4, 4, 4, 8, 16};
    longint p = R0;
    for (int k = 0; k < n; k++) begin
        int x = $urandom_range(0, 99);
        int b, room;
        logic [AXI_ID_BITS-1:0] id = (ids && $urandom_range(0, 9) == 0) ? 3 : 0;
        if (x < 3) begin
            rds.push_back('{CTRP + 64*$urandom_range(0, 7), 1, $urandom_range(0, 1) ? 6 : 3, id});
            continue;
        end
        if (x < 8) begin
            rds.push_back('{REGR + 8*$urandom_range(0, 63), 1, 3, id});
            continue;
        end
        if (x < 25) p = R0 + 64*$urandom_range(0, RSZ/64 - 1);
        room = (4096 - (p % 4096)) / 64;
        b = bl[$urandom_range(0, 6)];
        if (b > room) b = room;
        rds.push_back('{(x >= 25 && x < 30) ? p + 8*$urandom_range(1, 7) : p, b, 6, id});
        p = p + 64*b;
        if (p >= R0 + RSZ) p = R0;
    end
endfunction

// ---------------------------------------------------------------------------
// Monitors
// ---------------------------------------------------------------------------
// PASS: every AR and R signal equal on both ports; AW, W, B always
bit must_pass = 1;                  // until the first switch: off since reset
longint pass_cyc = 0;
always @(posedge xclk) if (xresetn) begin
    if (must_pass) `CHECK(!dut.trk, "TRACK while it must be off")
    if (!dut.trk) begin
        pass_cyc++;
        `CHECK(m.araddr === s.araddr && m.arid === s.arid && m.arlen === s.arlen && m.arsize === s.arsize
               && m.arburst === s.arburst && m.arcache === s.arcache && m.arlock === s.arlock && m.arprot === s.arprot
               && m.arqos === s.arqos && m.arregion === s.arregion && m.arvalid === s.arvalid && s.arready === m.arready,
               "PASS: AR differs between the ports")
        `CHECK(s.rdata === m.rdata && s.rid === m.rid && s.rlast === m.rlast && s.rresp === m.rresp
               && s.rvalid === m.rvalid && m.rready === s.rready, "PASS: R differs between the ports")
    end
    `CHECK(m.awaddr === s.awaddr && m.awid === s.awid && m.awlen === s.awlen && m.awsize === s.awsize
           && m.awburst === s.awburst && m.awvalid === s.awvalid && s.awready === m.awready
           && s.bid === m.bid && s.bresp === m.bresp && s.bvalid === m.bvalid && m.bready === s.bready,
           "AW/B differ between the ports")
end

// W: every beat taken at s_axi leaves m_axi in order, unchanged; held
// (valid, data) while it waits; a burst's first beat W_LAG or more cycles
// after m_axi took its AW (lag_min/lag_max over the run)
typedef struct packed { logic [511:0] d; logic [63:0] strb; logic last; } wbeat_t;
wbeat_t w_sb [$];
longint m_aw_cyc [$];
bit     m_w_mid = 0, m_w_wait = 0;
wbeat_t m_w_prev;
longint w_beats = 0, lag_min = 1 << 30, lag_max = 0, lag_n = 0;
always @(posedge xclk) if (xresetn) begin
    if (s.wvalid && s.wready) w_sb.push_back('{s.wdata, s.wstrb, s.wlast});
    if (m.awvalid && m.awready) m_aw_cyc.push_back(cyc);
    if (m_w_wait) `CHECK(m.wvalid && wbeat_t'{m.wdata, m.wstrb, m.wlast} == m_w_prev, "W: m_axi dropped or changed a waiting beat")
    m_w_wait = m.wvalid && !m.wready;
    m_w_prev = '{m.wdata, m.wstrb, m.wlast};
    if (m.wvalid && m.wready) begin
        w_beats++;
        `CHECK(w_sb.size() > 0, "W: a beat at m_axi that s_axi never took")
        if (w_sb.size() > 0) begin
            `CHECK(w_sb[0] == wbeat_t'{m.wdata, m.wstrb, m.wlast}, $sformatf("W: beat %0d differs from s_axi's", w_beats))
            void'(w_sb.pop_front());
        end
        if (!m_w_mid) begin
            `CHECK(m_aw_cyc.size() > 0, "W: a first beat before m_axi took its AW")
            if (m_aw_cyc.size() > 0) begin
                automatic longint g = cyc - m_aw_cyc.pop_front();
                `CHECK(g >= dut.W_LAG, $sformatf("W: a first beat %0d cycles after its AW (W_LAG %0d)", g, dut.W_LAG))
                if (g < lag_min) lag_min = g;
                if (g > lag_max) lag_max = g;
                lag_n++;
            end
        end
        m_w_mid = !m.wlast;
    end
end

// the DUT's outputs never drop a valid or change while it waits
typedef struct packed { logic [63:0] addr; logic [AXI_ID_BITS-1:0] id; logic [7:0] len; logic [2:0] size;
                        logic [1:0] burst; logic [3:0] cache; logic [0:0] lock; logic [2:0] prot; logic [3:0] qos;
                        logic [3:0] region; } tar_t;
function automatic tar_t m_ar();
    return '{m.araddr, m.arid, m.arlen, m.arsize, m.arburst, m.arcache, m.arlock, m.arprot, m.arqos, m.arregion};
endfunction
function automatic tar_t s_ar();
    return '{s.araddr, s.arid, s.arlen, s.arsize, s.arburst, s.arcache, s.arlock, s.arprot, s.arqos, s.arregion};
endfunction
bit   ar_w = 0, r_w = 0;
tar_t ar_p;
logic [511:0] r_pd;
logic [AXI_ID_BITS+2:0] r_pm;
always @(posedge xclk) if (xresetn) begin
    if (ar_w) `CHECK(m.arvalid && m_ar() == ar_p, "m_axi: AR dropped or changed while waiting")
    if (r_w) `CHECK(s.rvalid && s.rdata == r_pd && {s.rid, s.rlast, s.rresp} == r_pm, "s_axi: R dropped or changed while waiting")
    ar_w = m.arvalid && !m.arready; ar_p = m_ar();
    r_w = s.rvalid && !s.rready; r_pd = s.rdata; r_pm = {s.rid, s.rlast, s.rresp};
end

// TRACK: ARs taken reach the decision unchanged and in order; ARs passed
// through reach m_axi unchanged and in order; fills are fill-shaped
tar_t   trk_q [$], pas_q [$];
bit     ids_used [int];
longint n_fill = 0, m_ar_n = 0, m_rb = 0, s_ar_n = 0, s_rb = 0, m_ctr_ar = 0, s_ctr_ar = 0;
int     m_out = 0;
bit     m_kind [$];                 // m_axi's reads in order: a fill?
longint tb_fo = 0, tb_fo_sum = 0;   // fills outstanding at m_axi (AR taken, last beat not yet), summed over cycles
always @(posedge xclk) if (xresetn) begin
    tb_fo_sum += tb_fo;
    if (m.rvalid && m.rready && m.rlast) begin
        `CHECK(m_kind.size() > 0, "m_axi: a last R beat with no read outstanding")
        if (m_kind.size() > 0 && m_kind.pop_front()) tb_fo--;
    end
    if (m.arvalid && m.arready) begin
        m_ar_n++;
        m_out++;
        m_kind.push_back(dut.trk && dut.o_q.fill);
        if (dut.trk && dut.o_q.fill) tb_fo++;
        if (m.araddr >= CTRP && m.araddr < WIN + UWIN_SIZE) m_ctr_ar++;
        if (dut.trk) begin
            if (dut.o_q.fill) begin
                automatic longint ln = (m.araddr - WIN) / 64;
                n_fill++;
                `CHECK(m.araddr >= WIN && m.araddr[5:0] == 0 && ln + m.arlen + 1 <= (CTRP - WIN) / 64
                       && m.arsize == 6 && m.arburst == 1 && m.arlen < RA_BURST && (ln % RA_BURST) + m.arlen + 1 <= RA_BURST
                       && ids_used.exists(int'(m.arid)) && m.arcache == 0 && m.arprot == 0 && m.arqos == 0,
                       $sformatf("fill AR %h len %0d size %0d burst %0d id %0d", m.araddr, m.arlen, m.arsize, m.arburst, m.arid))
            end else begin
                `CHECK(pas_q.size() > 0 && m_ar() == pas_q[0],
                       $sformatf("m_axi: AR %h len %0d is not the next AR passed through", m.araddr, m.arlen))
                if (pas_q.size() > 0) void'(pas_q.pop_front());
            end
        end
    end
    if (m.rvalid && m.rready) begin
        m_rb++;
        if (m.rlast) m_out--;
    end
    `CHECK(m_out <= 64, $sformatf("m_axi: %0d reads outstanding", m_out))
    if (dut.q_v && dut.pop_q) begin
        `CHECK(trk_q.size() > 0 && tar_t'(dut.q.ar) == trk_q[0], $sformatf("the decision sees AR %h, not the next taken", dut.q.ar.addr))
        if (trk_q.size() > 0) void'(trk_q.pop_front());
        if (dut.m_peer) pas_q.push_back(dut.q.ar);
    end
    if (s.arvalid && s.arready) begin
        s_ar_n++;
        ids_used[int'(s.arid)] = 1;
        if (s.araddr >= CTRP && s.araddr < WIN + UWIN_SIZE) s_ctr_ar++;
        if (dut.trk) trk_q.push_back(s_ar());
    end
    if (s.rvalid && s.rready) s_rb++;
end

// coverage: a fill at mq's head while reads taken in PASS are still out (T13)
int nb_fill_seen = 0;
always @(posedge xclk) if (dut.nb_nz && dut.mq_hv && dut.mq_hf) nb_fill_seen++;

// counters
function automatic longint cw(input int i);
    return ctr[64*i +: 64];
endfunction
localparam int MODE = 0, HIT = 1, LATE = 2, PASS = 3, RA_AR = 4, FILLED = 5, SERVED = 6, STREAMS = 7, END_J = 8,
               END_W = 9, END_I = 10, FO_SUM = 11, NZ_ID = 12, WM = 13, OOO = 14, H0 = 15, OOO_ACC = 23;
typedef struct { longint w [24]; longint mar, mrb, sar, srb, fill, fo; } snap_t;
task automatic snap(output snap_t x);
    repeat (4) @(negedge xclk);
    for (int i = 0; i < 24; i++) x.w[i] = cw(i);
    x.mar = m_ar_n; x.mrb = m_rb; x.sar = s_ar_n; x.srb = s_rb; x.fill = n_fill; x.fo = tb_fo_sum;
endtask
// between two snapshots (both quiet): fills and lines read ahead are m_axi's
// traffic beyond the reads passed through; the fills seen are word 4
task automatic xcheck(input string name, input snap_t a, input snap_t b);
    longint ring_rd = (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE]);
    longint served  = b.w[SERVED] - a.w[SERVED];
    `CHECK(b.w[RA_AR] - a.w[RA_AR] == (b.mar - a.mar) - ((b.sar - a.sar) - ring_rd) && b.w[RA_AR] - a.w[RA_AR] == b.fill - a.fill,
           $sformatf("%s: %0d read-ahead ARs, m_axi shows %0d (fills seen %0d)", name, b.w[RA_AR] - a.w[RA_AR],
                     (b.mar - a.mar) - ((b.sar - a.sar) - ring_rd), b.fill - a.fill))
    `CHECK(b.w[FILLED] - a.w[FILLED] == (b.mrb - a.mrb) - ((b.srb - a.srb) - served),
           $sformatf("%s: %0d lines read ahead, m_axi shows %0d", name, b.w[FILLED] - a.w[FILLED],
                     (b.mrb - a.mrb) - ((b.srb - a.srb) - served)))
endtask

// read-ahead on (line 59) / off (line 58): word 0; off: back to PASS
task automatic ra_set(input bit on);
    rd_t r [$];
    longint cy;
    int w = 0;
    must_pass = 0;
    r.push_back('{CTRP + 64*(on ? 59 : 58), 1, 3, 0});
    peer_rd(on ? "line 59" : "line 58", r, 1, cy);
    repeat (300) @(negedge xclk);   // the counters' word 0 a cycle later; PASS once drained
    `CHECK(cw(MODE) == 64'(on), $sformatf("read-ahead mode %0d, want %0d", cw(MODE), on))
    while (dut.trk != on && w < 5000) begin @(negedge xclk); w++; end
    `CHECK(dut.trk == on, $sformatf("TRACK %0d after the switch, want %0d", dut.trk, on))
endtask

// WINDOW (line 57) / STRICT (line 56): word 13; a stream keeps the kind it started with
task automatic wm_set(input bit on);
    rd_t r [$];
    longint cy;
    r.push_back('{CTRP + 64*(on ? 57 : 56), 1, 3, 0});
    peer_rd(on ? "line 57" : "line 56", r, 1, cy);
    repeat (300) @(negedge xclk);
    `CHECK(cw(WM) == 64'(on), $sformatf("WINDOW %0d, want %0d", cw(WM), on))
endtask

// An RDMA NIC's reads of a sequential transfer: 4 KiB commands of 16 x 256 B
// reads from a, in order, except that at a fraction pct of the command
// boundaries the previous command's last t1 reads and the next one's first t2
// (1..8 each) interleave at random (each keeps its own order), and one read in
// lag_div is issued up to lag_max reads late
function automatic void nic_rds(ref rd_t rds [$], input longint a, input int ncmd, input int pct,
                                input int lag_div = 0, input int lag_max = 0);
    rd_t x [$];
    for (int k = 0; k < 16 * ncmd; k++) x.push_back('{a + 256*k, 4, 6, 0});
    for (int c = 1; c < ncmd; c++) begin
        if ($urandom_range(0, 99) < pct) begin
            int t1 = $urandom_range(1, 8), t2 = $urandom_range(1, 8);
            rd_t p [$], q [$];
            for (int k = 16*c - t1; k < 16*c; k++) p.push_back(x[k]);
            for (int k = 16*c; k < 16*c + t2; k++) q.push_back(x[k]);
            for (int k = 16*c - t1; k < 16*c + t2; k++)
                if (q.size() == 0 || (p.size() > 0 && $urandom_range(0, 1))) x[k] = p.pop_front();
                else x[k] = q.pop_front();
        end
    end
    if (lag_div > 0)
        for (int k = 0; k + lag_max < x.size(); k++)
            if ($urandom_range(0, lag_div - 1) == 0) begin
                rd_t r = x[k];
                int d = $urandom_range(1, lag_max);
                for (int j = k; j < k + d; j++) x[j] = x[j + 1];
                x[k + d] = r;
            end
    foreach (x[k]) rds.push_back(x[k]);
endfunction

// the out-of-order counters between two snapshots: the bins sum to word 14, those taken (23) at most it
task automatic ooo_check(input string name, input snap_t a, input snap_t b);
    longint nb_sum = 0;
    for (int i = H0; i < H0 + 8; i++) nb_sum += b.w[i] - a.w[i];
    `CHECK(nb_sum == b.w[OOO] - a.w[OOO] && b.w[OOO_ACC] - a.w[OOO_ACC] <= b.w[OOO] - a.w[OOO],
           $sformatf("%s: out-of-order %0d, bins sum %0d, taken %0d", name, b.w[OOO] - a.w[OOO], nb_sum, b.w[OOO_ACC] - a.w[OOO_ACC]))
endtask
function automatic string ooo_str(input snap_t a, input snap_t b);
    return $sformatf("out of order %0d (taken %0d): <-LAG %0d, -LAG..-65 %0d, -64..-17 %0d, -16..-1 %0d, 1..16 %0d, 17..64 %0d, 65..AHEAD %0d, >AHEAD %0d",
                     b.w[OOO] - a.w[OOO], b.w[OOO_ACC] - a.w[OOO_ACC], b.w[H0] - a.w[H0], b.w[H0+1] - a.w[H0+1], b.w[H0+2] - a.w[H0+2],
                     b.w[H0+3] - a.w[H0+3], b.w[H0+4] - a.w[H0+4], b.w[H0+5] - a.w[H0+5], b.w[H0+6] - a.w[H0+6], b.w[H0+7] - a.w[H0+7]);
endfunction

// T14's writer: st_n bursts of st_beats beats from st_a0, back to back, B not
// awaited. st_lag >= 0: W runs ahead (up to 2 bursts), AW k presented st_lag
// cycles after W k's first beat (0: in the same cycle, as the static); < 0:
// AWs run ahead (up to 4), W k's first beat -st_lag cycles after AW k is taken
bit      st_on = 0;
int      st_n, st_beats, st_lag, st_seed, st_aw, st_wk, st_beat, st_b;
longint  st_a0;
longint  st_wpres [int], st_awacc [int];
always @(posedge xclk) if (st_on) begin
    if (s.awvalid && s.awready) begin st_awacc[st_aw] = cyc; st_aw++; end
    if (s.wvalid && s.wready) begin
        if (st_beat == st_beats - 1) begin st_beat = 0; st_wk++; end
        else st_beat++;
    end
    if (s.bvalid && s.bready) st_b++;
end
always @(negedge xclk) if (st_on) begin
    automatic bit w_ok, aw_ok;
    if (st_lag >= 0) w_ok = (st_wk < st_n) && (st_wk - st_aw < 2);
    else             w_ok = (st_wk < st_n) && (st_wk < st_aw) && (st_beat > 0 || cyc - st_awacc[st_wk] >= -st_lag);
    if (w_ok && st_beat == 0 && !st_wpres.exists(st_wk)) st_wpres[st_wk] = cyc;
    s.wvalid = w_ok;
    s.wdata  = line_val((st_a0 - WIN) / 64 + st_wk * st_beats + st_beat, st_seed);
    s.wstrb  = '1;
    s.wlast  = (st_beat == st_beats - 1);
    if (st_lag >= 0) aw_ok = (st_aw < st_n) && st_wpres.exists(st_aw) && (cyc - st_wpres[st_aw] >= st_lag);
    else             aw_ok = (st_aw < st_n) && (st_aw - st_wk < 4);
    s.awvalid = aw_ok;
    s.awaddr  = st_a0 + 64 * longint'(st_aw * st_beats);
    s.awlen   = 8'(st_beats - 1); s.awsize = 6; s.awid = 0; s.awburst = 1;
end
// m_axi's first-beat cycles of T14's bursts, in order
longint st_mfirst [$];
bit     st_m_mid = 0;
always @(posedge xclk) if (st_on && m.wvalid && m.wready) begin
    if (!st_m_mid) st_mfirst.push_back(cyc);
    st_m_mid = !m.wlast;
end
task automatic wr_stream(input string name, input longint a0, input int n, input int beats, input int lag,
                         input int seed, output real cpb);
    st_n = n; st_beats = beats; st_lag = lag; st_seed = seed; st_a0 = a0;
    st_aw = 0; st_wk = 0; st_beat = 0; st_b = 0; st_wpres.delete(); st_awacc.delete(); st_mfirst = {}; st_m_mid = 0;
    @(negedge xclk); st_on = 1;
    for (int t = 0; t < 200 * n && st_b < n; t++) @(posedge xclk);
    @(negedge xclk); st_on = 0; s.awvalid = 0; s.wvalid = 0;
    `CHECK(st_b == n && st_mfirst.size() == n, $sformatf("%s: %0d of %0d B, %0d bursts at m_axi", name, st_b, n, st_mfirst.size()))
    cpb = (st_mfirst.size() == n) ? real'(st_mfirst[n - 1] - st_mfirst[16]) / (n - 17) : 0.0;
endtask

initial begin
    repeat (10) @(negedge xclk);
    xresetn = 1;
    repeat (10) @(negedge xclk);

    // --- T1: off after reset: exact pass-through. Mixes with register
    //     writes alongside, at 1, 8, 32 in flight, random backpressure and
    //     gaps; window writes; counters 0-11 stay 0
    begin
        rd_t rds [$];
        longint cy;
        int nmax [3] = '{1, 8, 32};
        for (int k = 0; k < 4; k++) wr_win(R0 + 1024*k, 16, 1);
        peer_rbp = 1;
        foreach (nmax[i]) begin
            rds = {};
            mix(rds, 300, 1);
            fork
                peer_rd($sformatf("T1 (%0d in flight)", nmax[i]), rds, nmax[i], cy);
                for (int k = 0; k < 8; k++) begin
                    repeat ($urandom_range(20, 80)) @(negedge xclk);
                    wr_reg(REGW + 8*(8*i + k), {32'(i), 32'(k)});
                end
            join
        end
        peer_rbp = 0; s.rready = 1;
        rds = {};
        for (int k = 0; k < 24; k++) rds.push_back('{REGW + 8*k, 1, 3, 0});
        peer_rd("T1 register read-back", rds, 8, cy);
        repeat (20) @(negedge xclk);
        for (int i = 0; i < 24; i++) if (i != NZ_ID) `CHECK(cw(i) == 0, $sformatf("T1: word %0d = %0d while off", i, cw(i)))
        `CHECK(cw(NZ_ID) == n_nzid, $sformatf("T1: word 12 = %0d, %0d ARs with a nonzero ID", cw(NZ_ID), n_nzid))
        `CHECK(!dut.trk && pass_cyc > 1000, $sformatf("T1: %0d cycles checked in PASS", pass_cyc))
    end
    $display("ok   T1 off after reset: exact pass-through (%0d cycles, every AR and R signal equal), counters 0", pass_cyc);

    // --- T2: the switch: line 59 on, line 58 off; PASS again once drained
    ra_set(1);
    ra_set(0);
    begin
        rd_t rds [$];
        longint cy, p0;
        p0 = pass_cyc;
        seq_rds(rds, R0, 32, 4);
        peer_rd("T2 off again", rds, 8, cy);
        `CHECK(!dut.trk && pass_cyc - p0 > 50, "T2: not back to pass-through")
    end
    $display("ok   T2 switched on (line 59) and off (line 58): word 0, back to pass-through");

    // --- T3: on: 512 sequential 256 B reads (128 KiB) at 8 in flight, R
    //     backpressure: one stream (the first read passes, the second starts
    //     it), exactly RA_LINES lines ahead, ended by the idle timeout
    begin
        rd_t rds [$];
        snap_t a, b;
        longint cy;
        ra_set(1);
        snap(a);
        seq_rds(rds, R0, 512, 4);
        peer_rbp = 1;
        peer_rd("T3", rds, 8, cy);
        peer_rbp = 0; s.rready = 1;
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        `CHECK(b.w[STREAMS] - a.w[STREAMS] == 1 && b.w[PASS] - a.w[PASS] == 1
               && (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE]) == 511 && b.w[SERVED] - a.w[SERVED] == 4 * 511,
               $sformatf("T3: streams %0d, passed %0d, hits %0d + late %0d, lines served %0d", b.w[STREAMS] - a.w[STREAMS],
                         b.w[PASS] - a.w[PASS], b.w[HIT] - a.w[HIT], b.w[LATE] - a.w[LATE], b.w[SERVED] - a.w[SERVED]))
        `CHECK((b.w[FILLED] - a.w[FILLED]) - (b.w[SERVED] - a.w[SERVED]) == RA_LINES,
               $sformatf("T3: %0d lines read ahead, %0d served", b.w[FILLED] - a.w[FILLED], b.w[SERVED] - a.w[SERVED]))
        `CHECK(b.w[END_I] - a.w[END_I] == 1 && b.w[END_J] == a.w[END_J] && b.w[END_W] == a.w[END_W],
               $sformatf("T3: ended by idle %0d, jump %0d, write %0d", b.w[END_I] - a.w[END_I], b.w[END_J] - a.w[END_J],
                         b.w[END_W] - a.w[END_W]))
        xcheck("T3", a, b);
        $display("     T3 512 reads: %0d hits, %0d late, %0d read-ahead ARs, %0d cycles", b.w[HIT] - a.w[HIT],
                 b.w[LATE] - a.w[LATE], b.w[RA_AR] - a.w[RA_AR], cy);
    end
    $display("ok   T3 a 128 KiB stream at 8 in flight with R backpressure: exact data, one stream, RA_LINES ahead, idle end");

    // --- T4: on: random mixes (runs, jumps, starts inside a line,
    //     counter-page and register reads, IDs 0/3), register writes
    //     alongside, at 1, 4, 8, 32, 64 in flight, random backpressure
    begin
        rd_t rds [$];
        snap_t a, b;
        longint cy;
        int nmax [5] = '{1, 4, 8, 32, 64};
        snap(a);
        peer_rbp = 1;
        foreach (nmax[i]) begin
            rds = {};
            mix(rds, 500, 1);
            fork
                peer_rd($sformatf("T4 (%0d in flight)", nmax[i]), rds, nmax[i], cy);
                for (int k = 0; k < 6; k++) begin
                    repeat ($urandom_range(50, 300)) @(negedge xclk);
                    wr_reg(REGW + 8*(64 + 8*i + k), {32'(i), 32'(k + 100)});
                end
            join
        end
        peer_rbp = 0; s.rready = 1;
        rds = {};
        for (int k = 0; k < 40; k++) rds.push_back('{REGW + 8*(64 + k), 1, 3, 0});
        peer_rd("T4 register read-back", rds, 8, cy);
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        xcheck("T4", a, b);
        `CHECK(b.w[STREAMS] - a.w[STREAMS] > 50 && b.w[HIT] - a.w[HIT] > 200 && b.w[END_J] - a.w[END_J] > 50
               && b.w[PASS] - a.w[PASS] > 200,
               $sformatf("T4: streams %0d, hits %0d, ended by a jump %0d, passed %0d: the mix exercised too little",
                         b.w[STREAMS] - a.w[STREAMS], b.w[HIT] - a.w[HIT], b.w[END_J] - a.w[END_J], b.w[PASS] - a.w[PASS]))
        `CHECK(b.w[END_W] == a.w[END_W], $sformatf("T4: %0d streams ended by register writes", b.w[END_W] - a.w[END_W]))
        `CHECK(cw(NZ_ID) == n_nzid, $sformatf("T4: word 12 = %0d, %0d ARs with a nonzero ID", cw(NZ_ID), n_nzid))
        $display("     T4 2500 reads: %0d streams, %0d hits, %0d late, %0d passed through, ended by a jump %0d",
                 b.w[STREAMS] - a.w[STREAMS], b.w[HIT] - a.w[HIT], b.w[LATE] - a.w[LATE], b.w[PASS] - a.w[PASS],
                 b.w[END_J] - a.w[END_J]);
    end
    $display("ok   T4 random mixes at 1-64 in flight, register traffic alongside: exact data, rlast, IDs, order");

    // --- T5: staleness. (a) 8 KiB into a stream (read ahead to +24 KiB) the
    //     peer writes 1 KiB at +8 KiB, then continues: it reads the new data;
    //     (b) the same at 8 in flight, the written range read only after the
    //     write's B; (c) a register write mid-stream does not end it; (d) a
    //     write outstanding (B 400 cycles late): a sequential run right below
    //     it starts no stream, so continuing into it after the B reads the
    //     new data
    begin
        rd_t rds [$];
        snap_t a, b;
        longint cy;
        longint A = R0 + 64'h1_0000, B = R0 + 64'h2_0000, C = R0 + 64'h3_0000, D = R0 + 64'h3_8000;
        snap(a);
        seq_rds(rds, A, 32, 4);
        peer_rd("T5a before", rds, 8, cy);
        repeat (300) @(negedge xclk);
        for (int k = 0; k < 4; k++) wr_win(A + 8192 + 256*k, 4, 13);
        rds = {};
        seq_rds(rds, A + 8192, 64, 4);
        peer_rd("T5a after", rds, 8, cy);
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        `CHECK(b.w[END_W] - a.w[END_W] == 1 && b.w[STREAMS] - a.w[STREAMS] == 2 && b.w[END_I] - a.w[END_I] == 1
               && (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE]) == 95,
               $sformatf("T5a: ended by a write %0d, streams %0d, idle %0d, stream reads %0d", b.w[END_W] - a.w[END_W],
                         b.w[STREAMS] - a.w[STREAMS], b.w[END_I] - a.w[END_I], (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE])))
        // (b)
        snap(a);
        rds = {};
        seq_rds(rds, B, 256, 4);
        gate_lo = B + 16384; gate_hi = B + 17408; gate_open = 0;
        fork
            peer_rd("T5b", rds, 8, cy);
            begin
                wait (peer_front >= B + 8192);
                for (int k = 0; k < 4; k++) wr_win(B + 16384 + 256*k, 4, 14);
                gate_open = 1;
            end
        join
        gate_lo = -1; gate_hi = -1;
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        xcheck("T5b", a, b);
        `CHECK(b.w[END_W] - a.w[END_W] >= 1 && b.w[STREAMS] - a.w[STREAMS] >= 2,
               $sformatf("T5b: ended by a write %0d, streams %0d", b.w[END_W] - a.w[END_W], b.w[STREAMS] - a.w[STREAMS]))
        // (c)
        snap(a);
        rds = {};
        seq_rds(rds, C, 64, 4);
        fork
            peer_rd("T5c", rds, 8, cy);
            begin
                wait (peer_front >= C + 4096);
                wr_reg(REGW + 8*200, 64'hC0FFEE);
            end
        join
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        `CHECK(b.w[END_W] == a.w[END_W] && b.w[STREAMS] - a.w[STREAMS] == 1 && b.w[END_I] - a.w[END_I] == 1,
               $sformatf("T5c: register write: ended by a write %0d, streams %0d, idle %0d", b.w[END_W] - a.w[END_W],
                         b.w[STREAMS] - a.w[STREAMS], b.w[END_I] - a.w[END_I]))
        // (d)
        snap(a);
        ds_wlat = 400;
        fork
            wr_win(D + 1024, 16, 15);
            begin
                repeat (30) @(negedge xclk);
                rds = {};
                seq_rds(rds, D, 4, 4);                          // the 1 KiB right below the write
                peer_rd("T5d below", rds, 8, cy);
                while (!(exp_seed.exists((D + 1024 - WIN) / 64) && exp_seed[(D + 1024 - WIN) / 64] == 15)) @(negedge xclk);
                rds = {};
                seq_rds(rds, D + 1024, 16, 4);                  // continues into it after the B
                peer_rd("T5d into the write", rds, 8, cy);
            end
        join
        ds_wlat = 10;
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        `CHECK(b.w[STREAMS] - a.w[STREAMS] == 1 && b.w[PASS] - a.w[PASS] == 4,
               $sformatf("T5d: streams %0d (want 1: started after the B), passed %0d (want 4)",
                         b.w[STREAMS] - a.w[STREAMS], b.w[PASS] - a.w[PASS]))
    end
    $display("ok   T5 a window write ends the stream, later reads see it; a register write does not; no stream while a write is outstanding");

    // --- T6: the idle timeout. 8 KiB into a stream, RA_IDLE cycles without a
    //     read, the card path writes 1 KiB at +8 KiB (the ring held it):
    //     continuing reads the new data. Within RA_IDLE the ring still
    //     answers with what it fetched before such a write (the documented
    //     exposure); a re-read (not sequential) returns the new data
    begin
        rd_t rds [$];
        snap_t a, b;
        longint cy;
        longint E = R0 + 64'h1_8000, F = R0 + 64'h2_8000;
        snap(a);
        seq_rds(rds, E, 32, 4);
        peer_rd("T6 before", rds, 8, cy);
        repeat (RA_IDLE + 100) @(negedge xclk);
        card_wr(E + 8192, 16, 16);
        for (int j = 0; j < 16; j++) exp_seed[(E + 8192 - WIN) / 64 + j] = 16;
        rds = {};
        seq_rds(rds, E + 8192, 32, 4);
        peer_rd("T6 after idle", rds, 8, cy);
        repeat (RA_IDLE + 100) @(negedge xclk);
        rds = {};
        seq_rds(rds, F, 32, 4);
        peer_rd("T6 exposure, before", rds, 8, cy);
        repeat (300) @(negedge xclk);
        card_wr(F + 8192, 16, 17);                              // exp_seed: still the old data
        rds = {};
        seq_rds(rds, F + 8192, 4, 4);
        peer_rd("T6 exposure (old data expected)", rds, 8, cy);
        for (int j = 0; j < 16; j++) exp_seed[(F + 8192 - WIN) / 64 + j] = 17;
        rds = '{'{F + 8192, 16, 6, 0}};
        peer_rd("T6 re-read", rds, 8, cy);
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        `CHECK(b.w[END_I] - a.w[END_I] == 2 && b.w[END_J] - a.w[END_J] == 1 && b.w[STREAMS] - a.w[STREAMS] == 3,
               $sformatf("T6: ended by idle %0d, jump %0d, streams %0d", b.w[END_I] - a.w[END_I], b.w[END_J] - a.w[END_J],
                         b.w[STREAMS] - a.w[STREAMS]))
    end
    $display("ok   T6 idle timeout drops the ring (card-path write seen); within RA_IDLE the documented exposure");

    // --- T7: switched off mid-stream (line 58 among 8 in flight, R
    //     backpressure): the stream ends, later reads pass through, PASS
    //     again, no read-ahead AR; on again (line 59) among reads in flight:
    //     reads still out from PASS answered first, then a new stream
    begin
        rd_t rds [$];
        snap_t a, mm, b;
        longint cy;
        longint G = R0 + 64'h1_0000;
        snap(a);
        peer_rbp = 1;
        seq_rds(rds, G, 16, 4);
        rds.push_back('{CTRP + 58*64, 1, 3, 0});
        seq_rds(rds, G + 4096, 32, 4);
        peer_rd("T7 off", rds, 8, cy);
        `CHECK(cw(MODE) == 0, "T7: read-ahead still on after line 58")
        repeat (RA_IDLE + 300) @(negedge xclk);
        `CHECK(!dut.trk, "T7: not back in PASS")
        snap(mm);
        rds = {};
        seq_rds(rds, G + 12288, 32, 4);
        peer_rd("T7 off, then", rds, 8, cy);
        snap(b);
        for (int i = 1; i < 24; i++) if (i != NZ_ID && i != WM) `CHECK(b.w[i] == mm.w[i], $sformatf("T7: word %0d moved while off", i))
        `CHECK(b.mar - mm.mar == b.sar - mm.sar, "T7: m_axi carried more than the peer's reads while off")
        snap(mm);
        rds = {};
        seq_rds(rds, G + 20480, 8, 4);
        rds.push_back('{CTRP + 59*64, 1, 3, 0});
        seq_rds(rds, G + 20480 + 2048, 24, 4);
        peer_rd("T7 on", rds, 8, cy);
        peer_rbp = 0; s.rready = 1;
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        `CHECK(cw(MODE) == 1 && dut.trk, "T7: read-ahead not on after line 59")
        `CHECK(b.w[STREAMS] - mm.w[STREAMS] == 1 && b.w[END_I] - mm.w[END_I] == 1
               && (b.w[HIT] - mm.w[HIT]) + (b.w[LATE] - mm.w[LATE]) >= 20,
               $sformatf("T7: streams %0d, stream reads %0d, idle end %0d", b.w[STREAMS] - mm.w[STREAMS],
                         (b.w[HIT] - mm.w[HIT]) + (b.w[LATE] - mm.w[LATE]), b.w[END_I] - mm.w[END_I]))
        xcheck("T7", mm, b);
        // switch over and over with reads in flight and R backpressure
        peer_rbp = 1;
        for (int r = 0; r < 6; r++) begin
            rds = {};
            for (int k = 0; k < 6; k++) begin
                seq_rds(rds, G + 64'h8000 + 4096*k, 8, 4);
                rds.push_back('{CTRP + 64*(58 + ((r + k) % 2)), 1, 3, 0});
            end
            peer_rd($sformatf("T7 switching %0d", r), rds, 8, cy);
        end
        peer_rbp = 0; s.rready = 1;
        ra_set(1);
    end
    $display("ok   T7 switched off and on mid-run, reads in flight both ways, R backpressure: data, order, no dropped valid");

    // --- T8: counters exact. One read at a time: the first passes through,
    //     the second starts the stream (late), then - the read-ahead well
    //     ahead - 62 hits; the idle timeout ends it. Then three streams,
    //     ended by a jump, a counter-page read, a window write
    begin
        rd_t rds [$];
        snap_t a, b;
        longint cy;
        longint H = R0 + 64'h2_8000;
        snap(a);
        seq_rds(rds, H, 2, 4);
        peer_rd("T8", rds, 1, cy);
        repeat (200) @(negedge xclk);
        rds = {};
        seq_rds(rds, H + 512, 62, 4);
        peer_rd("T8", rds, 1, cy);
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        `CHECK(b.w[HIT] - a.w[HIT] == 62, $sformatf("T8: hits %0d, want 62", b.w[HIT] - a.w[HIT]))
        `CHECK(b.w[LATE] - a.w[LATE] == 1, $sformatf("T8: late %0d, want 1", b.w[LATE] - a.w[LATE]))
        `CHECK(b.w[PASS] - a.w[PASS] == 1, $sformatf("T8: passed %0d, want 1", b.w[PASS] - a.w[PASS]))
        `CHECK(b.w[STREAMS] - a.w[STREAMS] == 1, $sformatf("T8: streams %0d, want 1", b.w[STREAMS] - a.w[STREAMS]))
        `CHECK(b.w[SERVED] - a.w[SERVED] == 252, $sformatf("T8: lines served %0d, want 252", b.w[SERVED] - a.w[SERVED]))
        `CHECK(b.w[FILLED] - a.w[FILLED] == 252 + RA_LINES,
               $sformatf("T8: lines read ahead %0d, want %0d", b.w[FILLED] - a.w[FILLED], 252 + RA_LINES))
        // 252 + 256 lines from line 4 of a 16-line block: 12, then 31 x 16
        `CHECK(b.w[RA_AR] - a.w[RA_AR] == 32, $sformatf("T8: read-ahead ARs %0d, want 32", b.w[RA_AR] - a.w[RA_AR]))
        `CHECK(b.w[END_I] - a.w[END_I] == 1 && b.w[END_J] == a.w[END_J] && b.w[END_W] == a.w[END_W],
               $sformatf("T8: ended by idle %0d, jump %0d, write %0d", b.w[END_I] - a.w[END_I], b.w[END_J] - a.w[END_J],
                         b.w[END_W] - a.w[END_W]))
        // fills outstanding: from m_axi taking each to its last beat taken
        // in from the R buffer - the port's count, plus at most 2 cycles a fill
        `CHECK(b.w[FO_SUM] - a.w[FO_SUM] >= b.fo - a.fo && b.w[FO_SUM] - a.w[FO_SUM] <= b.fo - a.fo + 2 * 32
               && b.fo - a.fo >= 32 * (ds_lat + 16),
               $sformatf("T8: fills outstanding summed %0d, the port shows %0d (32 fills)", b.w[FO_SUM] - a.w[FO_SUM], b.fo - a.fo))
        xcheck("T8", a, b);
        snap(a);
        rds = {};
        seq_rds(rds, H + 64'h1_0000, 4, 4);
        rds.push_back('{H, 4, 6, 0});                           // a jump
        peer_rd("T8 jump", rds, 1, cy);
        repeat (RA_IDLE + 300) @(negedge xclk);
        rds = {};
        seq_rds(rds, H + 64'h1_1000, 4, 4);
        rds.push_back('{CTRP, 1, 3, 0});                        // the counter page
        peer_rd("T8 counter page", rds, 1, cy);
        repeat (RA_IDLE + 300) @(negedge xclk);
        rds = {};
        seq_rds(rds, H + 64'h1_2000, 4, 4);
        peer_rd("T8 write", rds, 1, cy);
        wr_win(R0 + RSZ - 256, 4, 18);
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        `CHECK(b.w[END_J] - a.w[END_J] == 2 && b.w[END_W] - a.w[END_W] == 1 && b.w[END_I] == a.w[END_I]
               && b.w[STREAMS] - a.w[STREAMS] == 3 && b.w[PASS] - a.w[PASS] == 4
               && (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE]) == 9,
               $sformatf("T8: ended by a jump %0d, write %0d, idle %0d; streams %0d, passed %0d, stream reads %0d",
                         b.w[END_J] - a.w[END_J], b.w[END_W] - a.w[END_W], b.w[END_I] - a.w[END_I], b.w[STREAMS] - a.w[STREAMS],
                         b.w[PASS] - a.w[PASS], (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE])))
        xcheck("T8 ends", a, b);
    end
    $display("ok   T8 counters exact: hits, late, passed, lines, ARs, streams, each end, fills outstanding");

    // --- T9: every AR with ID 5 (a static that ties its IDs to a nonzero
    //     constant): the stream's fills carry 5, every R too; word 12
    begin
        rd_t rds [$];
        snap_t a, b;
        longint cy, n0;
        n0 = cw(NZ_ID);
        snap(a);
        seq_rds(rds, R0 + 64'h2_0000, 128, 4, 5);
        peer_rd("T9", rds, 8, cy);
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        `CHECK(b.w[STREAMS] - a.w[STREAMS] == 1 && (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE]) == 127,
               $sformatf("T9: streams %0d, stream reads %0d", b.w[STREAMS] - a.w[STREAMS], (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE])))
        `CHECK(cw(NZ_ID) - n0 == 128, $sformatf("T9: word 12 moved %0d, want 128", cw(NZ_ID) - n0))
        xcheck("T9", a, b);
    end
    $display("ok   T9 a constant nonzero ID: stream, fills and R carry it; word 12 counts it");

    // --- T10: R stopped mid-stream, the peer allowing 80 in flight: rq
    //     fills (64), ARs held; released, every read answered in order
    begin
        rd_t rds [$];
        longint cy;
        seq_rds(rds, R0, 400, 4);
        fork
            peer_rd("T10", rds, 80, cy);
            begin
                wait (peer_front >= R0 + 256*20);
                @(negedge xclk); peer_rbp = 0; s.rready = 0;
                repeat (600) @(negedge xclk);
                `CHECK(dut.rq_full && s.arvalid && !s.arready, "T10: rq not full, or ARs not held, with R stopped")
                s.rready = 1;
            end
        join
        repeat (RA_IDLE + 300) @(negedge xclk);
    end
    $display("ok   T10 R stopped with 80 allowed in flight: 64 held (rq full), ARs held, then all answered in order");

    // --- T11: what it buys. The model as measured on the V80: a read ~441 ns
    //     at axi_main with read-ahead off (ds_lat 142 + 4 beats), aclk's beat
    //     rate (3 in 4 cycles), ~110 ns upstream before a slot is used again
    //     (up_dly 37), 8 x 256 B in flight; 512 KiB sequential, off then on
    begin
        rd_t rds [$];
        snap_t a, b;
        longint c_off, c_on;
        real l_off, l_on;
        ds_lat = 142; ds_slow = 1; ds_rand = 0; up_dly = 37;
        ra_set(0);
        seq_rds(rds, R0, 2048, 4);
        lt_sum = 0; lt_n = 0;
        peer_rd("T11 off", rds, 8, c_off);
        l_off = real'(lt_sum) / lt_n;
        ra_set(1);
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(a);
        rds = {};
        seq_rds(rds, R0 + 64'h8_0000, 2048, 4);
        lt_sum = 0; lt_n = 0;
        peer_rd("T11 on", rds, 8, c_on);
        l_on = real'(lt_sum) / lt_n;
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        $display("     T11 512 KiB, 8 x 256 B in flight, up 110 ns: off %0d cycles = %.2f GB/s (a read %.0f ns at axi_main), on %0d cycles = %.2f GB/s (%.0f ns); %0d hits of %0d; fill AR to last beat %.0f ns",
                 c_off, 524288.0 / (c_off * 3.0), l_off * 3.0, c_on, 524288.0 / (c_on * 3.0), l_on * 3.0,
                 b.w[HIT] - a.w[HIT], b.w[HIT] - a.w[HIT] + b.w[LATE] - a.w[LATE],
                 3.0 * real'(b.w[FO_SUM] - a.w[FO_SUM]) / (b.w[RA_AR] - a.w[RA_AR]));
        `CHECK(3 * c_on < c_off, $sformatf("T11: on %0d cycles, off %0d: not 3x faster", c_on, c_off))
        xcheck("T11", a, b);
        ds_lat = 20; ds_slow = 0; ds_rand = 1; up_dly = 0;
    end
    $display("ok   T11 read-ahead on vs off at the V80's measured latencies");

    // --- T12: the end of a short stream, as tight as it gets: a read, a
    //     second that starts the stream (one fill, exactly its 16 lines), a
    //     jump right behind it (passed through as soon as that fill is out).
    //     The jump's data follows the fill's last beat back to back while the
    //     ring read still waits for that line: it must not overtake. Beats 3
    //     in 4 cycles, every phase of that pattern, the jump to the window and
    //     to a register
    begin
        rd_t rds [$];
        snap_t a, b;
        longint cy;
        ds_lat = 40; ds_slow = 1; ds_rand = 0;
        snap(a);
        for (int k = 0; k < 32; k++) begin
            longint X = R0 + 64'h1_0000 + 4096*k;
            repeat (RA_IDLE + 50 + (k % 8)) @(negedge xclk);
            rds = {};
            rds.push_back('{X, 16, 6, 0});
            rds.push_back('{X + 1024, 16, 6, 0});
            rds.push_back((k % 2) ? rd_t'('{REGR + 8*k, 1, 3, 0}) : rd_t'('{R0 + 64*k, 1 + k % 4, 6, 0}));
            peer_rd($sformatf("T12 (%0d)", k), rds, 8, cy);
        end
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        `CHECK(b.w[STREAMS] - a.w[STREAMS] == 32 && b.w[RA_AR] - a.w[RA_AR] == 32 && b.w[END_J] - a.w[END_J] == 32,
               $sformatf("T12: streams %0d, read-ahead ARs %0d, ended by a jump %0d (want 32 each)", b.w[STREAMS] - a.w[STREAMS],
                         b.w[RA_AR] - a.w[RA_AR], b.w[END_J] - a.w[END_J]))
        xcheck("T12", a, b);
        ds_lat = 20; ds_slow = 0; ds_rand = 1;
    end
    $display("ok   T12 the end of a short stream: the jump's data does not overtake the ring read");

    // --- T13: switched on right before a stream: line 59, then a read taken
    //     while still in PASS, then one continuing it (TRACK: it starts the
    //     stream at once, its fill first in mq) - at latency 80 the PASS read
    //     is still out when the fill heads mq: its beats are answered first,
    //     not taken for the fill's
    begin
        rd_t rds [$];
        longint cy;
        int seen0 = nb_fill_seen;
        ds_lat = 80; ds_rand = 0;
        for (int k = 0; k < 4; k++) begin
            ra_set(0);
            rds = {};
            rds.push_back('{CTRP + 59*64, 1, 3, 0});
            seq_rds(rds, R0 + 64'h2_0000 + 16384*k, 16, 16);
            peer_rd($sformatf("T13 (%0d)", k), rds, 64, cy);
            repeat (RA_IDLE + 300) @(negedge xclk);
        end
        `CHECK(nb_fill_seen > seen0, "T13: never a fill at mq's head with a PASS read still out")
        ds_lat = 20; ds_rand = 1;
    end
    $display("ok   T13 switched on right before a stream: reads still out from PASS answered first");

    // --- T14: W behind AW. 256 window writes of 4 beats, back to back, m_axi
    //     always ready: one burst every 4 cycles whether AW comes with the
    //     first W beat (the static), after it, or ahead; then random
    //     backpressure and gaps on m_axi (the scoreboard checks every beat)
    begin
        int lags [5] = '{0, 2, 5, -1, -6};
        real cpb;
        ra_set(0);
        ds_rand = 0;
        lag_min = 1 << 30; lag_max = 0;
        foreach (lags[i]) begin
            wr_stream($sformatf("T14 (AW %0d after W)", lags[i]), WIN + 64'h80_0000 + 64'h4_0000 * i, 256, 4, lags[i], 7 + i, cpb);
            `CHECK(cpb > 3.99 && cpb < 4.01, $sformatf("T14: AW %0d cycles after W: %.3f cycles a burst at m_axi (want 4.000)", lags[i], cpb))
            $display("     T14 AW %0d cycles after the first W beat: %.3f cycles a burst at m_axi", lags[i], cpb);
        end
        $display("     T14 first beat after its AW at m_axi: %0d..%0d cycles (W_LAG %0d)", lag_min, lag_max, dut.W_LAG);
        `CHECK(lag_min == dut.W_LAG, $sformatf("T14: shortest AW -> first beat %0d, want W_LAG %0d", lag_min, dut.W_LAG))
        ds_rand = 1;
        for (int k = 0; k < 4; k++)
            wr_stream("T14 (backpressure)", WIN + 64'hA0_0000 + 64'h4_0000 * k, 128, 1 + 5 * (k % 2), k % 2 ? 0 : -2, 20 + k, cpb);
        `CHECK(w_sb.size() == 0, $sformatf("T14: %0d W beats taken at s_axi never left", w_sb.size()))
        ds_rand = 1;
    end
    $display("ok   T14 W behind AW: 4 cycles a burst, first beats W_LAG after their AW, every beat in order");

    // --- T15: WINDOW / STRICT: lines 57 / 56, word 13
    ra_set(1);
    wm_set(1);
    wm_set(0);
    $display("ok   T15 WINDOW selected (line 57) and STRICT again (line 56): word 13");

    // --- T16: a NIC's reordered reads. 64 x 4 KiB commands, half of the
    //     boundaries interleaved, one read in 40 up to 12 reads late; 8 in
    //     flight, R backpressure, model gaps. WINDOW: one stream, exact data,
    //     nothing passed through, every out-of-order read taken; STRICT on
    //     the same pattern: the stream breaks
    begin
        rd_t rds [$];
        snap_t a, b;
        longint cy;
        ds_lat = 20; ds_slow = 0; ds_rand = 1; up_dly = 0;
        wm_set(1);
        repeat (RA_IDLE + 300) @(negedge xclk);
        nic_rds(rds, R0, 64, 50, 40, 12);
        snap(a);
        peer_rbp = 1;
        peer_rd("T16 WINDOW", rds, 8, cy);
        peer_rbp = 0; s.rready = 1;
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        xcheck("T16 WINDOW", a, b);
        ooo_check("T16 WINDOW", a, b);
        `CHECK(b.w[STREAMS] - a.w[STREAMS] == 1 && b.w[END_J] == a.w[END_J] && b.w[END_I] - a.w[END_I] == 1
               && b.w[PASS] - a.w[PASS] <= 2 && (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE]) + (b.w[PASS] - a.w[PASS]) == 16 * 64
               && b.w[OOO] - a.w[OOO] > 20 && b.w[OOO_ACC] - a.w[OOO_ACC] == b.w[OOO] - a.w[OOO],
               $sformatf("T16 WINDOW: streams %0d, jumps %0d, idle %0d, passed %0d, stream reads %0d, %s", b.w[STREAMS] - a.w[STREAMS],
                         b.w[END_J] - a.w[END_J], b.w[END_I] - a.w[END_I], b.w[PASS] - a.w[PASS],
                         (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE]), ooo_str(a, b)))
        $display("     T16 WINDOW: %0d stream reads, %0d late; %s", (b.w[HIT] - a.w[HIT]) + (b.w[LATE] - a.w[LATE]),
                 b.w[LATE] - a.w[LATE], ooo_str(a, b));
        wm_set(0);
        repeat (RA_IDLE + 300) @(negedge xclk);
        rds = {};
        nic_rds(rds, R0 + 64'h1_0000, 64, 50, 40, 12);
        snap(a);
        peer_rd("T16 STRICT", rds, 8, cy);
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        xcheck("T16 STRICT", a, b);
        ooo_check("T16 STRICT", a, b);
        `CHECK(b.w[END_J] - a.w[END_J] > 20 && b.w[OOO_ACC] == a.w[OOO_ACC],
               $sformatf("T16 STRICT: jumps %0d, out-of-order taken %0d", b.w[END_J] - a.w[END_J], b.w[OOO_ACC] - a.w[OOO_ACC]))
        $display("     T16 STRICT: %0d streams, ended by a jump %0d, %0d passed through; %s", b.w[STREAMS] - a.w[STREAMS],
                 b.w[END_J] - a.w[END_J], b.w[PASS] - a.w[PASS], ooo_str(a, b));
    end
    $display("ok   T16 a NIC's interleaved 4 KiB commands: WINDOW keeps one stream, exact data; STRICT breaks");

    // --- T17: WINDOW's edges, each read checked: (a) a read RA_LAG + 4
    //     lines behind s_req ends the stream; (b) a read below the stream's
    //     start (inside RA_LAG) ends it; (c) a read ending beyond s_ahd ends
    //     it; (d) one 16 lines behind and one 16 ahead are taken
    begin
        rd_t rds [$];
        snap_t a, b;
        longint cy;
        longint A = R0 + 64'h2_0000;
        wm_set(1);
        repeat (RA_IDLE + 300) @(negedge xclk);
        // (a)
        snap(a);
        seq_rds(rds, A, 64, 4);                                 // s_req = 256 lines past A
        rds.push_back('{A + 64*(256 - RA_LAG - 4), 4, 6, 0});   // starts RA_LAG + 4 lines behind
        seq_rds(rds, A + 64*256, 8, 4);
        peer_rd("T17a", rds, 1, cy);
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        `CHECK(b.w[END_J] - a.w[END_J] == 1 && b.w[H0] - a.w[H0] == 1,
               $sformatf("T17a: jumps %0d, %s", b.w[END_J] - a.w[END_J], ooo_str(a, b)))
        // (b)
        snap(a);
        rds = {};
        seq_rds(rds, A + 64'h4000, 4, 4);
        rds.push_back('{A + 64'h4000 - 256, 4, 6, 0});           // below the stream's start
        peer_rd("T17b", rds, 1, cy);
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        `CHECK(b.w[END_J] - a.w[END_J] == 1 && b.w[OOO_ACC] == a.w[OOO_ACC],
               $sformatf("T17b: jumps %0d, taken out of order %0d", b.w[END_J] - a.w[END_J], b.w[OOO_ACC] - a.w[OOO_ACC]))
        // (c)
        snap(a);
        rds = {};
        seq_rds(rds, A + 64'h8000, 4, 4);                       // s_req = 16 lines past it
        rds.push_back('{A + 64'h8000 + 64*(16 + 256 - 2), 4, 6, 0});   // ends 2 lines beyond s_ahd
        peer_rd("T17c", rds, 1, cy);
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        `CHECK(b.w[END_J] - a.w[END_J] == 1 && b.w[H0 + 6] - a.w[H0 + 6] == 1,
               $sformatf("T17c: jumps %0d, %s", b.w[END_J] - a.w[END_J], ooo_str(a, b)))
        // (d)
        snap(a);
        rds = {};
        seq_rds(rds, A + 64'hC000, 16, 4);                      // s_req = 64 lines past it
        rds.push_back('{A + 64'hC000 + 64*(64 - 16), 4, 6, 0});
        rds.push_back('{A + 64'hC000 + 64*(64 + 16), 4, 6, 0});
        seq_rds(rds, A + 64'hC000 + 64*64, 8, 4);
        peer_rd("T17d", rds, 1, cy);
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        ooo_check("T17", a, b);
        `CHECK(b.w[END_J] == a.w[END_J] && b.w[STREAMS] - a.w[STREAMS] == 1 && b.w[OOO_ACC] - a.w[OOO_ACC] >= 2
               && b.w[H0 + 3] - a.w[H0 + 3] >= 1 && b.w[H0 + 4] - a.w[H0 + 4] >= 1,
               $sformatf("T17d: jumps %0d, streams %0d, %s", b.w[END_J] - a.w[END_J], b.w[STREAMS] - a.w[STREAMS], ooo_str(a, b)))
    end
    $display("ok   T17 WINDOW's edges: behind s_lag, below the start, beyond s_ahd end the stream; inside it is taken");

    // --- T18: WINDOW under T4's random mixes (1-64 in flight, register
    //     writes alongside) and T5's staleness: a window write ends the
    //     stream, later reads see it
    begin
        rd_t rds [$];
        snap_t a, b;
        longint cy;
        longint A = R0 + 64'h1_8000;
        int nmax [5] = '{1, 4, 8, 32, 64};
        wm_set(1);
        snap(a);
        peer_rbp = 1;
        foreach (nmax[i]) begin
            rds = {};
            mix(rds, 500, 1);
            fork
                peer_rd($sformatf("T18 (%0d in flight)", nmax[i]), rds, nmax[i], cy);
                for (int k = 0; k < 6; k++) begin
                    repeat ($urandom_range(50, 300)) @(negedge xclk);
                    wr_reg(REGW + 8*(160 + 8*i + k), {32'(i), 32'(k + 200)});
                end
            join
        end
        peer_rbp = 0; s.rready = 1;
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        xcheck("T18 mixes", a, b);
        ooo_check("T18 mixes", a, b);
        `CHECK(b.w[STREAMS] - a.w[STREAMS] > 20 && b.w[OOO_ACC] - a.w[OOO_ACC] > 5,
               $sformatf("T18: streams %0d, %s", b.w[STREAMS] - a.w[STREAMS], ooo_str(a, b)))
        $display("     T18 2500 mixed reads: %0d streams, %0d hits, %0d passed; %s", b.w[STREAMS] - a.w[STREAMS],
                 b.w[HIT] - a.w[HIT], b.w[PASS] - a.w[PASS], ooo_str(a, b));
        snap(a);
        rds = {};
        seq_rds(rds, A, 32, 4);
        peer_rd("T18 before", rds, 8, cy);
        for (int k = 0; k < 4; k++) wr_win(A + 8192 + 256*k, 4, 23);
        rds = {};
        nic_rds(rds, A + 8192, 4, 100);
        peer_rd("T18 after", rds, 8, cy);
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        `CHECK(b.w[END_W] - a.w[END_W] == 1, $sformatf("T18: ended by a write %0d", b.w[END_W] - a.w[END_W]))
    end
    $display("ok   T18 WINDOW under random mixes and a window write: exact data, the write ends the stream");

    // --- T19: WINDOW, lagging reads with R stopped: 80 allowed in flight,
    //     one read in 6 up to 20 reads (80 lines) late; R stopped until rq is
    //     full, then released: no deadlock (fills never overrun a read still
    //     to answer), exact data; then the same with R backpressure and the
    //     V80's latencies
    begin
        rd_t rds [$];
        snap_t a, b;
        longint cy;
        snap(a);
        nic_rds(rds, R0 + 64'h2_0000, 100, 50, 6, 20);
        fork
            peer_rd("T19", rds, 80, cy);
            begin
                wait (peer_front >= R0 + 64'h2_0000 + 4096*4);
                @(negedge xclk); s.rready = 0;
                repeat (800) @(negedge xclk);
                `CHECK(dut.rq_full, "T19: rq not full with R stopped")
                s.rready = 1;
            end
        join
        ds_lat = 142; ds_slow = 1; ds_rand = 0; up_dly = 37;
        rds = {};
        nic_rds(rds, R0 + 64'h8_0000, 100, 50, 6, 20);
        peer_rbp = 1;
        peer_rd("T19 V80 latencies", rds, 8, cy);
        peer_rbp = 0; s.rready = 1;
        ds_lat = 20; ds_slow = 0; ds_rand = 1; up_dly = 0;
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        xcheck("T19", a, b);
        ooo_check("T19", a, b);
        `CHECK(b.w[STREAMS] - a.w[STREAMS] <= 4 && b.w[OOO_ACC] - a.w[OOO_ACC] > 100,
               $sformatf("T19: streams %0d, %s", b.w[STREAMS] - a.w[STREAMS], ooo_str(a, b)))
        $display("     T19: %0d streams, ended by a jump %0d; %s", b.w[STREAMS] - a.w[STREAMS], b.w[END_J] - a.w[END_J], ooo_str(a, b));
    end
    $display("ok   T19 WINDOW with lagging reads, R stopped until rq is full: no deadlock, exact data");

    // --- T20: what WINDOW buys on a NIC's pattern at the V80's latencies
    //     (T11's model): 512 KiB of 4 KiB commands, a quarter of the
    //     boundaries interleaved (as measured at 4 KiB: ~0.25 breaks a
    //     command), 8 x 256 B in flight; STRICT, then WINDOW
    begin
        rd_t rds [$];
        snap_t a, b;
        longint c_s, c_w;
        ds_lat = 142; ds_slow = 1; ds_rand = 0; up_dly = 37;
        wm_set(0);
        repeat (RA_IDLE + 300) @(negedge xclk);
        nic_rds(rds, R0, 128, 25);
        snap(a);
        peer_rd("T20 STRICT", rds, 8, c_s);
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        $display("     T20 STRICT: %0d cycles = %.2f GB/s, %0d jumps", c_s, 524288.0 / (c_s * 3.0), b.w[END_J] - a.w[END_J]);
        wm_set(1);
        repeat (RA_IDLE + 300) @(negedge xclk);
        rds = {};
        nic_rds(rds, R0 + 64'h8_0000, 128, 25);
        snap(a);
        peer_rd("T20 WINDOW", rds, 8, c_w);
        repeat (RA_IDLE + 300) @(negedge xclk);
        snap(b);
        xcheck("T20", a, b);
        $display("     T20 WINDOW: %0d cycles = %.2f GB/s, %0d jumps, %0d late", c_w, 524288.0 / (c_w * 3.0), b.w[END_J] - a.w[END_J],
                 b.w[LATE] - a.w[LATE]);
        `CHECK(b.w[END_J] == a.w[END_J] && c_w * 5 < c_s * 4, $sformatf("T20: WINDOW %0d cycles vs STRICT %0d", c_w, c_s))
        ds_lat = 20; ds_slow = 0; ds_rand = 1; up_dly = 0;
        wm_set(0);
    end
    $display("ok   T20 WINDOW vs STRICT on a NIC's pattern at the V80's latencies");

    `CHECK(m_ctr_ar == s_ctr_ar, $sformatf("m_axi: %0d ARs into the counter page, the peer read it %0d times", m_ctr_ar, s_ctr_ar))
    `CHECK(trk_q.size() == 0 && pas_q.size() == 0, $sformatf("ARs never seen: %0d at the decision, %0d at m_axi", trk_q.size(), pas_q.size()))
    if (errors == 0) $display("TB PASS (tb_uwin_ra)");
    else             $display("TB FAIL (tb_uwin_ra): %0d errors", errors);
    $finish;
end

initial begin
    #5ms;
    $display("state: trk %0d ra_on %0d st %0d rq %0d mq %0d nb %0d fl_v %0d q_v %0d o_v %0d ri_v %0d ro_v %0d s_req %0d s_hi %0d s_got %0d g_rd %0d n_old %0d wr_out %0d; peer AR %0d/%0d front %h; model reads %0d writes %0d/%0d",
             dut.trk, dut.ra_on, dut.st, dut.rq_n, dut.mq_n, dut.nb, dut.fl_v, dut.q_v, dut.o_v, dut.ri_v, dut.ro_v,
             dut.s_req, dut.s_hi, dut.s_got, dut.g_rd, dut.n_old, dut.wr_out, s.arvalid, s.arready, peer_front,
             m_rq.size(), m_awq.size(), m_bq.size());
    $display("TB FAIL (tb_uwin_ra): timeout");
    $finish;
end

endmodule
