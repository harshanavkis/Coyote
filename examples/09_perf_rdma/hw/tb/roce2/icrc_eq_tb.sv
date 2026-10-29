`timescale 1ns / 1ps
/**
 * icrc_eq_tb (2026-10-28): icrc_fast (CRC352/CRC448 stages) against icrc (icrc_fixed.sv) on the same input: every
 * 4-byte-multiple packet length 20..4200 B, random data, random gaps, random output back-pressure (the same for both);
 * the two output streams must be identical word by word.
 */
module icrc_eq_tb;
    parameter int GAP = 0;       // 1: one idle input cycle after every packet into icrc_fast (the new one)
    parameter int GAP_REF = 1;   // the same into icrc (the old one: needs it behind a 40 B tail, as the HLS core gives it)
    logic nclk = 0, nresetn = 0;
    always #2 nclk = ~nclk;
    AXI4S #(.AXI4S_DATA_BITS(512)) s0 (.aclk(nclk), .aresetn(nresetn)), s1 (.aclk(nclk), .aresetn(nresetn));
    AXI4S #(.AXI4S_DATA_BITS(512)) m0 (.aclk(nclk), .aresetn(nresetn)), m1 (.aclk(nclk), .aresetn(nresetn));
    icrc      ref_i (.m_axis_rx(s0), .m_axis_tx(m0), .nclk(nclk), .nresetn(nresetn));
    icrc_fast new_i (.m_axis_rx(s1), .m_axis_tx(m1), .nclk(nclk), .nresetn(nresetn));
    // each DUT gets its own input driver (their ready differ), the same packet list; outputs collected and compared
    typedef struct { logic [511:0] d; logic [63:0] k; logic l; } w_t;
    w_t pk[$]; w_t o0[$], o1[$];
    int seed = 7;
    initial begin
        for (int len = 20; len <= 4200; len += 4) begin
            automatic int nb = (len + 63) / 64;
            for (int b = 0; b < nb; b++) begin
                automatic w_t w; automatic int rem = len - b * 64;
                for (int j = 0; j < 16; j++) w.d[j*32 +: 32] = $urandom(seed + len * 131 + b * 17 + j);
                w.k = rem >= 64 ? '1 : ((64'd1 << rem) - 1);
                for (int j = 0; j < 64; j++) if (!w.k[j]) w.d[j*8 +: 8] = 8'h0;
                w.l = (b == nb - 1);
                pk.push_back(w);
            end
        end
        $display("EQ input: %0d words", pk.size());
        repeat (10) @(posedge nclk); nresetn = 1;
    end
    int rr = 0; always @(posedge nclk) rr <= $urandom % 100;
    assign m0.tready = rr < 80; assign m1.tready = rr < 80;
    int i0 = 0, i1 = 0; logic v0 = 0, v1 = 0; logic g0, g1;
    // registered drives (xsim does not re-evaluate continuous assigns on queue elements: they read X)
    logic [511:0] d0 = '0, d1 = '0; logic [63:0] k0 = '0, k1 = '0; logic l0 = 0, l1 = 0, e0 = 0, e1 = 0;
    assign s0.tvalid = v0 && e0; assign s0.tdata = d0; assign s0.tkeep = k0; assign s0.tlast = l0;
    assign s1.tvalid = v1 && e1; assign s1.tdata = d1; assign s1.tkeep = k1; assign s1.tlast = l1;
    int dbg = 0;
    bit h0, h1, x0, x1; longint cyc = 0, ncmp = 0, n448 = 0, n352 = 0, n32r = 0;
    always @(posedge nclk) begin n448 += new_i.stage_crc448_valid; n352 += new_i.stage_crc352_valid; n32r += (ref_i.stage_crc32_valid[0] === 1'b1);
        if (new_i.stage_masking_valid && new_i.stage_masking_last && n448 == 0 && new_i.stage_masking_keep[63:40] != '1 && new_i.stage_masking_keep[3:0] == 4'hf && dbg < 40) begin dbg++; $display("DBG keep %h last word, in-flight %b", new_i.stage_masking_keep, new_i.crc32_in_flight_comb); end end
    always @(posedge nclk) if (cyc >= 3000 && cyc < 3012) $display("PEEK cyc %0d in v%b k%h l%b rdy%b | mask v%b k%h l%b | crc32v0 %b crc512v %b", cyc, s1.tvalid, s1.tkeep, s1.tlast, s1.tready, new_i.stage_masking_valid, new_i.stage_masking_keep, new_i.stage_masking_last, new_i.stage_crc32_valid[0], new_i.stage_crc512_valid);
    always @(posedge nclk) begin
        h0 = s0.tvalid && s0.tready; h1 = s1.tvalid && s1.tready;
        x0 = m0.tvalid && m0.tready; x1 = m1.tvalid && m1.tready;
        if (x0) o0.push_back('{m0.tdata, m0.tkeep, m0.tlast});
        if (x1) o1.push_back('{m1.tdata, m1.tkeep, m1.tlast});
        #0.001;
        cyc++;
        if (nresetn) begin
            if (h0) i0++; if (h1) i1++;
            v0 = ($urandom % 100) < 90; v1 = GAP ? v0 : 1;
            if (GAP_REF && h0 && l0) v0 = 0;
            if (GAP && h1 && l1) v1 = 0;
            e0 = i0 < pk.size(); e1 = i1 < pk.size();
            if (e0) begin d0 = pk[i0].d; k0 = pk[i0].k; l0 = pk[i0].l; end
            if (e1) begin d1 = pk[i1].d; k1 = pk[i1].k; l1 = pk[i1].l; end
        end
        while (o0.size() && o1.size()) begin
            w_t a = o0.pop_front(), b = o1.pop_front(); ncmp++;
            if ($isunknown(a.k) || $isunknown(b.k) || $isunknown(a.l)) begin $display("EQ X in output at cycle %0d", cyc); $finish; end
            if (a.k !== b.k || a.l !== b.l || ((a.d ^ b.d) & {64{8'hff}}) !== 0) begin
                for (int j = 0; j < 64; j++) if (a.k[j] && a.d[j*8 +: 8] !== b.d[j*8 +: 8]) begin
                    $display("EQ output word %0d: bytes 0..63 ref %h", ncmp, a.d); $display("EQ                     new %h", b.d);
                    $display("EQ MISMATCH at cycle %0d (input %0d/%0d): keep %h/%h last %b/%b", cyc, i0, i1, a.k, b.k, a.l, b.l); $finish; end
                if (a.k !== b.k || a.l !== b.l) begin $display("EQ MISMATCH keep/last at cycle %0d: %h/%h %b/%b", cyc, a.k, b.k, a.l, b.l); $finish; end
            end
        end
        if (i0 == pk.size() && i1 == pk.size() && cyc > 2000 && o0.size() == 0 && o1.size() == 0 && !m0.tvalid && !m1.tvalid) begin
            $display("EQ PASS (GAP new %0d ref %0d): %0d input words, %0d packets (20..4200 B), outputs identical (%0d words compared; new CRC448 %0d, CRC352 %0d words; ref CRC32 pipeline entries %0d), cycle %0d", GAP, GAP_REF, pk.size(), (4200 - 20) / 4 + 1, ncmp, n448, n352, n32r, cyc); $finish; end
        if (cyc > 400000) begin $display("EQ TIMEOUT i0 %0d i1 %0d of %0d", i0, i1, pk.size()); $finish; end
    end
endmodule
