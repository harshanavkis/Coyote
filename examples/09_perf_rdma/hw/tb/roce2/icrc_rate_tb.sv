`timescale 1ns / 1ps
/**
 * icrc_rate_tb (2026-10-28, Loom both-way limit): icrc.sv alone, packets back to back at its input (the RoCE HLS core's
 * output), output always ready. Pattern = a list of packet byte lengths repeated: LEN_A then, if LEN_B > 0, LEN_B (a data
 * packet and the ACK the node sends for the peer's packet). Prints input cycles per pattern repeat in the middle half.
 */
module icrc_rate_tb;
    parameter int LEN_A = 4156, LEN_B = 0, NREP = 200, OUT_PCT = 100;
    logic nclk = 0, nresetn = 0;
    always #2 nclk = ~nclk;
    AXI4S #(.AXI4S_DATA_BITS(512)) s (.aclk(nclk), .aresetn(nresetn));
    AXI4S #(.AXI4S_DATA_BITS(512)) m (.aclk(nclk), .aresetn(nresetn));
    icrc dut (.m_axis_rx(s), .m_axis_tx(m), .nclk(nclk), .nresetn(nresetn));

    // output: ready OUT_PCT % of cycles (deterministic pattern)
    int oc = 0;
    always @(posedge nclk) oc <= (oc + 1) % 100;   // (registered: changes after the edge)
    assign m.tready = (oc < OUT_PCT);

    int lens[2]; int npkt; int beat = 0, pi = 0, rep = 0;
    longint cyc = 0, t_rep[$];
    logic v = 0; logic [511:0] d; logic [63:0] k; logic l;
    assign s.tvalid = v; assign s.tdata = d; assign s.tkeep = k; assign s.tlast = l;
    initial begin
        lens[0] = LEN_A; lens[1] = LEN_B; npkt = LEN_B > 0 ? 2 : 1;
        repeat (10) @(posedge nclk); nresetn = 1; repeat (10) @(posedge nclk);
    end
    function automatic void load();
        int len = lens[pi], nb = (len + 63) / 64, rem = len - beat * 64;
        d = {16{beat[31:0] ^ 32'h5a5aa5a5}};
        k = rem >= 64 ? '1 : ((64'd1 << rem) - 1);
        l = (beat == nb - 1);
    endfunction
    bit hs;
    always @(posedge nclk) begin   // sample the handshake at the edge, drive 1 ps later (race-free)
        hs = v && s.tready;
        #0.001;
        cyc = cyc + 1;
        if (nresetn) begin
            if (!v) begin if (rep < NREP) begin beat = 0; pi = 0; load(); v = 1; end end
            else if (hs) begin
                if (l) begin
                    beat = 0; pi = pi + 1;
                    if (pi == npkt) begin pi = 0; t_rep.push_back(cyc); rep = rep + 1; end
                    if (rep < NREP) load(); else v = 0;
                end else begin beat = beat + 1; load(); end
            end
        end
        if (rep == NREP && !v) begin
            automatic int a = NREP / 4, b = 3 * NREP / 4;
            $display("ICRC RATE LEN_A %0d LEN_B %0d OUT_PCT %0d: %.2f input cycles per repeat (beats %0d)", LEN_A, LEN_B, OUT_PCT,
                     real'(t_rep[b] - t_rep[a]) / (b - a), (LEN_A + 63) / 64 + (LEN_B > 0 ? (LEN_B + 63) / 64 : 0));
            $finish;
        end
    end
endmodule
