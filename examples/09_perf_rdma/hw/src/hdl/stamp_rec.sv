import lynxTypes::*;

/**
 * stamp_rec
 *
 * Per-packet timestamps for the latency breakdown (same file in
 * examples/loom_switch/hw/src/hdl). Each of N_PTS event inputs is a
 * one-cycle pulse; every event writes the cycle count since the last arm
 * (aclk) into the point's own ring of DEPTH entries, so a ring holds the
 * point's last DEPTH events. cnt[p] counts every event since the arm; the
 * oldest entry still in the ring is cnt - DEPTH (when cnt > DEPTH), the
 * ring slot of event e is e % DEPTH. Nothing is recorded before the first
 * arm. The events are registered once before they are stamped: every point
 * is late by the same cycle, differences are exact.
 *
 * Readout: software writes (point, index) into sel and reads sel_data; the
 * BRAM read runs every cycle at the selected address, so the data has
 * settled long before the CSR read arrives.
 */
module stamp_rec #(
    parameter integer N_PTS = 9,
    parameter integer DEPTH = 4096
) (
    input  logic                         aclk,
    input  logic                         aresetn,

    input  logic [N_PTS-1:0]             ev,
    input  logic                         arm,
    input  logic [3:0]                   sel_pt,
    input  logic [$clog2(DEPTH)-1:0]     sel_idx,
    output logic [31:0]                  sel_data,
    output logic [N_PTS-1:0][31:0]       cnt,
    output logic [31:0]                  now
);

localparam integer AW = $clog2(DEPTH);

logic [N_PTS-1:0] ev_r;
logic             en;
logic [31:0]      rd [N_PTS];

always_ff @(posedge aclk) begin
    if (!aresetn) begin
        ev_r <= '0;
        en   <= 1'b0;
        now  <= '0;
        cnt  <= '0;
    end else begin
        ev_r <= ev;
        if (arm) begin
            en  <= 1'b1;
            now <= '0;
            cnt <= '0;
        end else begin
            now <= now + 1;
            for (int p = 0; p < N_PTS; p++) if (en && ev_r[p]) cnt[p] <= cnt[p] + 1;
        end
    end
end

for (genvar p = 0; p < N_PTS; p++) begin : g_pt
    (* ram_style = "block" *) logic [31:0] mem [DEPTH];
    always_ff @(posedge aclk) begin
        if (en && !arm && ev_r[p]) mem[cnt[p][AW-1:0]] <= now;
        rd[p] <= mem[sel_idx];
    end
end

always_ff @(posedge aclk) sel_data <= (sel_pt < N_PTS) ? rd[sel_pt] : 32'hFFFF_FFFF;

endmodule
