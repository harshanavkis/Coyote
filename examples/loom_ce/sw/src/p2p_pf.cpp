/**
 * p2p_pf - peer-to-peer writes between the U280 and the V80 with example 07
 * (perf_fpga) on BOTH cards: no Loom anywhere. Each card writes REPS x len
 * bytes, timed by its own perf_fpga cycle counter (250 MHz, START to the
 * last write's completion), run twice with the second (warm) reported, into:
 *   host  its own host buffer (huge pages): the card's write path
 *   peer  the other card's user-window range, exported by the other card's
 *         driver and imported here: with no window in perf_fpga the writes
 *         end at that card's shell interconnect (decode error, data
 *         dropped) - the receiver's PCIe path and nothing else
 * Two directions: V80 -> U280 (into the U280's XDMA bypass) and U280 -> V80
 * (into the V80's PCIe bridge and SmartConnect).
 *
 * Usage: p2p_pf [LEN:REPS ...]   (default 4096:16 65536:16 1048576:16 4194304:16)
 */
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <coyote/cThread.hpp>
#include "loom_switch.hpp"   // reserve_va only

namespace {

// Example 07 (perf_fpga) control registers
enum Reg : uint32_t { CTRL = 0, DONE = 1, TIMER = 2, VADDR = 3, LEN = 4, PID = 5, N_REPS = 6, N_BEATS = 7 };
constexpr uint64_t START_WR = 0x2;
constexpr double   CLK_HZ   = 250e6;

// One run: reps writes of len bytes at va; GB/s, or -1 on a 10 s timeout
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
    return reps * len / (double(t.getCSR(TIMER)) / CLK_HZ) / 1e9;
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
    if (runs.empty()) runs = {{4096, 16}, {65536, 16}, {1 << 20, 16}, {4 << 20, 16}};
    uint64_t max_len = 0;
    for (auto &r : runs) if (r.first > max_len) max_len = r.first;
    max_len = (max_len + (2ULL << 20) - 1) & ~((2ULL << 20) - 1);

    coyote::cThread u280(0, getpid(), 0, nullptr, "loom_fpga");
    coyote::cThread v80(0, getpid(), 0, nullptr, "loom_versal_fpga");
    void *u_host = u280.getMem({coyote::CoyoteAllocType::HPF, max_len});
    void *v_host = v80.getMem({coyote::CoyoteAllocType::HPF, max_len});

    // each card's window range, imported into the other at a 2 MiB-aligned VA
    void *u_win = loom_switch::reserve_va(max_len);
    v80.importDmabuf(u280.exportDmabuf(EXPORT_REGION_UWIN, 0, max_len), u_win);
    void *v_win = loom_switch::reserve_va(max_len);
    u280.importDmabuf(v80.exportDmabuf(EXPORT_REGION_UWIN, 0, max_len), v_win);

    printf("%10s %8s | %10s %12s | %10s %12s   (GB/s, warm, the writer's cycle counter)\n",
           "bytes", "writes", "V80->host", "V80->U280", "U280->host", "U280->V80");
    for (auto &r : runs) {
        const double vh = run(v80, v_host, r.first, r.second);
        const double vu = run(v80, u_win, r.first, r.second);
        const double uh = run(u280, u_host, r.first, r.second);
        const double uv = run(u280, v_win, r.first, r.second);
        printf("%10lu %8lu | %10.2f %12.2f | %10.2f %12.2f\n", (unsigned long) r.first, (unsigned long) r.second,
               vh, vu, uh, uv);
    }
    return 0;
}
