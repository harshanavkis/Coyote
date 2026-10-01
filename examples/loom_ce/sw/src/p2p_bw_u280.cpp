/**
 * p2p_bw_u280 - how fast can the U280 write peer-to-peer into the V80's
 * window, with nothing of Loom (no loom_rx, no RoCE) in the way? p2p_bw
 * the other way round.
 *
 * Needs example 07 (perf_fpga) on the U280 and loom_ce (EN_UWIN_HBM) on the
 * V80. For each size, 16 back-to-back writes of that size, timed by example
 * 07's cycle counter (250 MHz, START to the last write's completion), run
 * twice with the second (warm) reported, into:
 *   host   a U280 cThread's host buffer (huge pages, the local baseline)
 *   v80    the V80's window (its HBM), exported by the V80 driver and
 *          imported into the U280 cThread, as G6's landing does
 * and the V80 window's counters (uwin_hbm) over the v80 run.
 *
 * With arguments, each LEN:REPS pair is one run of REPS writes of LEN bytes
 * (all to the same buffer, one request each): 4096:4096 is loom_rx's shape,
 * one 4 KiB write per packet, 16 MiB in all.
 *
 * Usage: p2p_bw_u280 [LEN:REPS ...]
 */
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <coyote/cThread.hpp>
#include "land_v80.hpp"
#include "loom_switch.hpp"

namespace {

// Example 07 (perf_fpga) control registers
enum Reg : uint32_t { CTRL = 0, DONE = 1, TIMER = 2, VADDR = 3, LEN = 4, PID = 5, N_REPS = 6, N_BEATS = 7 };
constexpr uint64_t START_WR = 0x2;
constexpr int      REPS     = 16;   // default runs
constexpr double   CLK_HZ   = 250e6;

// One benchmark run: REPS writes of len bytes at va; GB/s or -1 on timeout
double run_once(coyote::cThread &t, void *va, uint64_t len, uint64_t reps) {
    t.setCSR(reinterpret_cast<uint64_t>(va), VADDR);
    t.setCSR(len, LEN);
    t.setCSR(t.getCtid(), PID);
    t.setCSR(reps, N_REPS);
    t.setCSR(reps * len / 64, N_BEATS);
    t.setCSR(START_WR, CTRL);
    const auto t0 = std::chrono::steady_clock::now();
    while (!t.getCSR(DONE))
        if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(10)) return -1;
    const double cycles = double(t.getCSR(TIMER));
    return reps * len / (cycles / CLK_HZ) / 1e9;
}

// Warm: the first pass pays the first-touch translation misses
double run(coyote::cThread &t, void *va, uint64_t len, uint64_t reps) {
    (void) run_once(t, va, len, reps);
    return run_once(t, va, len, reps);
}

}  // namespace

int main(int argc, char **argv) {
    std::vector<std::pair<uint64_t, uint64_t>> runs;
    for (int i = 1; i < argc; i++) runs.push_back({strtoull(argv[i], nullptr, 0), strtoull(strchr(argv[i], ':') + 1, nullptr, 0)});
    if (runs.empty())
        for (uint64_t len : {4096ULL, 65536ULL, 1ULL << 20, 4ULL << 20}) runs.push_back({len, REPS});
    uint64_t max_len = 4096;
    for (auto &r : runs) max_len = r.first > max_len ? r.first : max_len;

    coyote::cThread u280(0, getpid(), 0, nullptr, "coyote_fpga");
    coyote::cThread v80(0, getpid(), 0, nullptr, "coyote_versal_fpga");

    void *host = u280.getMem({coyote::CoyoteAllocType::HPF, max_len});

    // The V80's window, bound to its HBM, imported into the U280
    land_v80::Landing L(v80, max_len);
    void *wva = loom_switch::reserve_va(max_len);
    u280.importDmabuf(L.export_fd(), wva);

    printf("%10s %8s %10s %10s   (GB/s, warm, U280 cycle counter)\n", "bytes", "writes", "host", "v80");
    for (auto &r : runs) {
        const uint64_t len = r.first, reps = r.second;
        const double h = run(u280, host, len, reps);
        const land_v80::Counters u0 = land_v80::Counters::read(L.win);
        const double p = run(u280, wva, len, reps);
        const land_v80::Counters u1 = land_v80::Counters::read(L.win);
        printf("%10lu %8lu %10.2f %10.2f\n", (unsigned long) len, (unsigned long) reps, h, p);
        u1.print(u0);
    }
    return 0;
}
