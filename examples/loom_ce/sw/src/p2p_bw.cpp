/**
 * p2p_bw - how fast can the V80 write, to host memory and peer-to-peer to
 * the U280? Isolates the source side of loom_ce's copies.
 *
 * Needs example 07 (perf_fpga) on the V80 and loom_switch on the U280. For
 * each size, 16 back-to-back writes of that size, timed by example 07's
 * cycle counter (250 MHz, START to the last write's completion), run twice
 * with the second (warm: translations and pages in place) reported, into:
 *   host   a V80 cThread's host buffer (the V80's own DMA path)
 *   p2p    the U280's uwin with no window programmed: loom_ingress accepts
 *          and discards every burst (counted as dropped), so this is PCIe
 *          peer-to-peer plus the ingress's accept rate
 *   p2p+   the uwin with a local window: through the ingress into a host
 *          buffer, the path ce_local takes
 *
 * Usage: p2p_bw
 */
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <stdexcept>

#include <coyote/cThread.hpp>
#include "loom_switch.hpp"

namespace {

// Example 07 (perf_fpga) control registers
enum Reg : uint32_t { CTRL = 0, DONE = 1, TIMER = 2, VADDR = 3, LEN = 4, PID = 5, N_REPS = 6, N_BEATS = 7 };
constexpr uint64_t START_WR = 0x2;
constexpr int      REPS     = 16;
constexpr double   CLK_HZ   = 250e6;

// One benchmark run: REPS writes of len bytes at va; GB/s or -1 on timeout
double run_once(coyote::cThread &v80, void *va, uint64_t len) {
    v80.setCSR(reinterpret_cast<uint64_t>(va), VADDR);
    v80.setCSR(len, LEN);
    v80.setCSR(v80.getCtid(), PID);
    v80.setCSR(REPS, N_REPS);
    v80.setCSR(REPS * len / 64, N_BEATS);
    v80.setCSR(START_WR, CTRL);
    const auto t0 = std::chrono::steady_clock::now();
    while (!v80.getCSR(DONE))
        if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(10)) return -1;
    const double cycles = double(v80.getCSR(TIMER));
    return REPS * len / (cycles / CLK_HZ) / 1e9;
}

// Warm: the first pass pays the first-touch translation misses
double run(coyote::cThread &v80, void *va, uint64_t len) {
    (void) run_once(v80, va, len);
    return run_once(v80, va, len);
}

}  // namespace

int main() {
    const uint64_t sizes[] = {4096, 65536, 1 << 20, 16 << 20};
    const uint64_t max_len = 16 << 20;

    coyote::cThread u280(0, getpid(), 0, nullptr, "coyote_fpga");
    coyote::cThread v80(0, getpid(), 0, nullptr, "coyote_versal_fpga");

    void *host = v80.getMem({coyote::CoyoteAllocType::HPF, max_len});

    // The uwin's first max_len bytes, imported into the V80
    const int fd = u280.exportDmabuf(EXPORT_REGION_UWIN, 0, max_len);
    void *uva = mmap(nullptr, max_len, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (uva == MAP_FAILED) throw std::runtime_error("mmap for a reserved address failed");
    v80.importDmabuf(fd, uva);

    // p2p+: a local window over the same uwin range, onto a U280 host buffer
    void *landing = u280.getMem({coyote::CoyoteAllocType::HPF, max_len});

    printf("%10s %10s %10s %10s   (GB/s, %d writes each, warm, V80 cycle counter)\n", "bytes", "host", "p2p", "p2p+", REPS);
    for (uint64_t len : sizes) {
        for (int w = 1; w <= 15; w++) loom_switch::release_window(u280, w);
        (void) loom_switch::csr_read(u280, loom_switch::TBL_IDX);
        const double h = run(v80, host, len);
        loom_switch::IngressCounters c0 = loom_switch::IngressCounters::read(u280);
        const double p = run(v80, uva, len);
        loom_switch::IngressCounters c1 = loom_switch::IngressCounters::read(u280);
        loom_switch::program_window(u280, 1, false, u280.getCtid(), landing, max_len, 0);
        const uint64_t cyc0 = loom_switch::csr_read(u280, loom_switch::CYC);
        const uint64_t wl0  = loom_switch::csr_read(u280, loom_switch::WR_WAIT_LOCAL);
        const double pl = run(v80, uva, len);
        usleep(10000);   // let the landing drain before reading the counters
        const uint64_t cyc1 = loom_switch::csr_read(u280, loom_switch::CYC);
        const uint64_t wl1  = loom_switch::csr_read(u280, loom_switch::WR_WAIT_LOCAL);
        loom_switch::IngressCounters c2 = loom_switch::IngressCounters::read(u280);
        const uint64_t pk = c2.v[2] - c1.v[2];
        printf("%10lu %10.2f %10.2f %10.2f   p2p: %lu bursts dropped; p2p+: %lu bursts, %lu local packets, "
               "%lu cycles a local request waited on sq_wr (%.1f per packet)\n",
               (unsigned long) len, h, p, pl,
               (unsigned long) (c1.v[1] - c0.v[1]),
               (unsigned long) (c2.v[0] - c1.v[0]), (unsigned long) pk,
               (unsigned long) (wl1 - wl0), pk ? double(wl1 - wl0) / pk : 0.0);
        printf("    p2p+ ingress debug over %lu cycles:\n", (unsigned long) (cyc1 - cyc0));
        loom_switch::print_ingress_delta(c1, c2, loom_switch::I_DBG, loom_switch::N_ING - loom_switch::I_DBG);
    }
    loom_switch::release_window(u280, 1);
    return 0;
}
