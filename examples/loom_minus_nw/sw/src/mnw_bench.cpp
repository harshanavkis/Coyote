/**
 * Loom minus-nw: the rdma route with both ends on one vFPGA.
 *
 * examples/loom's two-host runs cannot separate "the host write path is
 * slower than the line" from "the network perturbed the stream" - every
 * corrupt run has a receive-path stall AND retransmissions, and the counters
 * cannot order them. With EN_RDMA 0 the engine's outgoing stream is fed
 * straight back into loom_rx, so nothing can retransmit, reorder or drop,
 * and loom_rx's move/starve/stall split measures the host write path alone.
 *
 * One process, one cThread, one card. The window is rdma-route so the engine
 * takes the same path it takes on the wire (header, then payload), but the
 * header's target is a local buffer and loom_rx lands it under our own pid.
 *
 * What a run answers:
 *   - the sustained rate the receive path can absorb, with no network
 *   - whether its stalls are uniform (structural, and buffering only defers
 *     them) or clustered (a burst a buffer would swallow) - RX_STALL_MAX
 *     against RX_STALL says which
 *   - whether the engine's pull sustains that rate, or gaps it (TX starve)
 */

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include <coyote/cThread.hpp>
#include "loom.hpp"

namespace {

constexpr uint64_t BUF_SIZE = 64ULL * 1024 * 1024;   // src and dst each
constexpr int      WIN      = 1;

// Same ladder as loom_host's remote bench, so the numbers sit next to the
// two-host ones without rescaling. The rdma route is 64 B granular by
// contract, so no ragged lengths here (bulk_sweep covers those locally).
const uint64_t SIZES[] = {64, 256, 1024, 4096, 16384, 65536,
                          262144, 1048576, 4194304};

int failures = 0;

int poll_secs() {
    const char *e = getenv("LOOM_POLL_SECS");
    return e ? atoi(e) : 30;
}

bool poll64(volatile uint64_t *addr, uint64_t want) {
    for (int i = 0; i < poll_secs() * 100; i++) {
        if (*addr == want) return true;
        usleep(10000);
    }
    return false;
}

// Spin rather than sleep: below a megabyte poll64's 10 ms granularity
// reports the sleep, not the transfer.
bool spin64(volatile uint64_t *addr, uint64_t want, double timeout_us) {
    auto t0 = std::chrono::steady_clock::now();
    while (*addr != want) {
        if (std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - t0).count() > timeout_us)
            return false;
    }
    return true;
}

uint64_t only_size() {
    const char *e = getenv("LOOM_BENCH_ONLY");
    return e ? strtoull(e, nullptr, 0) : 0;
}
int iters() {
    const char *e = getenv("LOOM_BENCH_ITERS");
    return e ? atoi(e) : 32;
}

uint64_t src_word(uint64_t i) { return 0x5A5A0000ULL << 32 | i; }

void dump_paths(coyote::cThread &t) {
    const uint64_t tm = loom::csr_read(t, loom::TX_MOVE);
    const uint64_t ts = loom::csr_read(t, loom::TX_STARVE);
    const uint64_t tt = loom::csr_read(t, loom::TX_STALL);
    const uint64_t tot_t = tm + ts + tt;
    printf("transmit path cycles: %lu moving, %lu starved (host pull dry), "
           "%lu stalled (loopback pushing back)\n",
           (unsigned long) tm, (unsigned long) ts, (unsigned long) tt);
    if (tot_t)
        printf("  %.1f%% moving, %.1f%% starved, %.1f%% stalled\n",
               100.0 * tm / tot_t, 100.0 * ts / tot_t, 100.0 * tt / tot_t);

    const uint64_t rm = loom::csr_read(t, loom::RX_MOVE);
    const uint64_t rs = loom::csr_read(t, loom::RX_STARVE);
    const uint64_t rt = loom::csr_read(t, loom::RX_STALL);
    const uint64_t tot_r = rm + rs + rt;
    printf("receive path cycles: %lu moving, %lu starved (ingress had "
           "nothing), %lu stalled (host write not ready)\n",
           (unsigned long) rm, (unsigned long) rs, (unsigned long) rt);
    if (tot_r)
        printf("  %.1f%% moving, %.1f%% starved, %.1f%% stalled\n",
               100.0 * rm / tot_r, 100.0 * rs / tot_r, 100.0 * rt / tot_r);

    const uint64_t hd = loom::csr_read(t, loom::RX_STALL_HEAD);
    const uint64_t bd = loom::csr_read(t, loom::RX_STALL_BODY);
    const uint64_t mx = loom::csr_read(t, loom::RX_STALL_MAX);
    if (rt) {
        printf("  of the stalls: %lu before the packet's first beat, %lu "
               "after -> %s\n", (unsigned long) hd, (unsigned long) bd,
               hd > bd ? "single-outstanding request latency"
                       : "a bursty host write path");
        // The number this build exists for. A sum cannot size a buffer or
        // tell a burst from a deficit; the longest unbroken run can.
        printf("  longest unbroken stall: %lu cycles of %lu total -> %s\n",
               (unsigned long) mx, (unsigned long) rt,
               mx < 512 ? "clustered: a burst, and buffering is the lever"
                        : (mx < 4096 ? "long bursts: buffering defers rather "
                                       "than fixes"
                                     : "sustained: the write path is simply "
                                       "slower than the feed"));
    }
    printf("counters: rx_fwd=%lu rx_hdr_reject=%lu drops=%lu\n",
           (unsigned long) loom::csr_read(t, loom::DBG_BASE + 8 * 4),
           (unsigned long) loom::csr_read(t, loom::DBG_BASE + 8 * 9),
           (unsigned long) loom::csr_read(t, loom::DBG_BASE + 8 * 5));
}

}  // namespace

int main() {
    coyote::cThread t(0, getpid(), 0);
    const uint32_t pid = t.getCtid();
    printf("ctid %u, buffers %lu MB, minus-nw loopback\n",
           pid, (unsigned long) (BUF_SIZE >> 20));

    auto *dst = static_cast<uint64_t *>(
        t.getMem({coyote::CoyoteAllocType::HPF, BUF_SIZE}));
    auto *src = static_cast<uint64_t *>(
        t.getMem({coyote::CoyoteAllocType::HPF, BUF_SIZE}));
    auto *cpl = static_cast<uint64_t *>(
        t.getMem({coyote::CoyoteAllocType::REG, 4096}));
    if (!dst || !src || !cpl) { printf("FAIL: getMem\n"); return 1; }

    for (uint64_t i = 0; i < BUF_SIZE / 8; i++) src[i] = src_word(i);
    memset(cpl, 0, 4096);

    // rdma route: the engine emits a header carrying base+off as the target
    // and then the payload, exactly as it would on the wire. loom_rx takes
    // the target from that header and writes under RX_PID, which here is our
    // own ctid because both ends are this process.
    loom::program_window(t, WIN, /*rdma=*/true, pid, dst, BUF_SIZE);
    loom::csr_write(t, loom::RDMA_STAGING_VA,
                    reinterpret_cast<uint64_t>(dst));
    loom::csr_write(t, loom::RX_PID, pid);

    const uint64_t one = only_size();
    const int n = iters();

    printf("%12s %6s %12s %12s %10s  %s\n",
           "bytes", "iters", "cyc/op", "us/op", "GB/s", "result");

    // The fence is the engine's running completion count and clears only on
    // aresetn, so start from whatever earlier runs left behind.
    uint64_t compl_base = loom::csr_read(t, loom::DBG_BASE + 8 * 7);

    for (uint64_t len : SIZES) {
        if (len > BUF_SIZE) continue;
        if (one && len != one) continue;

        memset(dst, 0, len + 64);

        // Warm the path once so the timed run is a steady state, as the
        // two-host bench does.
        loom::dma(t, WIN, 0, src, len, pid, cpl);
        compl_base++;
        if (!spin64(&cpl[0], compl_base, 5e6)) {
            printf("%12lu  warm-up never fenced - stopping\n",
                   (unsigned long) len);
            failures++;
            continue;
        }

        loom::StageStats a = loom::read_stage_stats(t);
        auto t0 = std::chrono::steady_clock::now();
        for (int k = 0; k < n; k++) {
            loom::dma(t, WIN, 0, src, len, pid, cpl);
            spin64(&cpl[0], compl_base + uint64_t(k + 1), 5e6);
        }
        bool fenced = poll64(&cpl[0], compl_base + uint64_t(n));
        auto t1 = std::chrono::steady_clock::now();
        compl_base += uint64_t(n);
        loom::StageStats b = loom::read_stage_stats(t);

        const double us = std::chrono::duration<double, std::micro>(t1 - t0)
                              .count() / double(n);
        const uint64_t cyc =
            (b.acc[loom::STG_DMA_RDMA] - a.acc[loom::STG_DMA_RDMA]) /
            (n ? uint64_t(n) : 1);

        bool ok = fenced;
        uint64_t first_bad = 0;
        for (uint64_t i = 0; ok && i < len / 8; i++)
            if (dst[i] != src_word(i)) { ok = false; first_bad = i; }
        // Nothing past the transfer may be touched
        for (uint64_t i = len / 8; ok && i < len / 8 + 8; i++)
            if (dst[i] != 0) { ok = false; first_bad = i; }

        printf("%12lu %6d %12lu %12.2f %10.3f  %s\n",
               (unsigned long) len, n, (unsigned long) cyc, us,
               us > 0 ? double(len) / (us * 1000.0) : 0.0,
               ok ? "PASS" : (fenced ? "FAIL payload" : "FAIL no fence"));
        if (!ok) {
            failures++;
            if (fenced)
                printf("  first bad word %lu: got %016lx want %016lx\n",
                       (unsigned long) first_bad,
                       (unsigned long) dst[first_bad],
                       (unsigned long) src_word(first_bad));
        }
    }

    printf("\n");
    dump_paths(t);
    printf(failures == 0 ? "LOOM MNW PASS\n" : "LOOM MNW FAIL (%d)\n",
           failures);
    return failures == 0 ? 0 : 1;
}
