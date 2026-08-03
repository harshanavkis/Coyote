/**
 * Loom end-to-end test (local flows), same binary for simulation
 * (-DEN_SIM=ON, run with COYOTE_SIM_DIR=<hw build>) and hardware.
 *
 * Single cThread (the simulation backend supports exactly one): its own
 * buffers play both the issuer's source and the exporters' destinations,
 * so pid_src == pid_dst == ctid. Cross-pid writes are a hardware gate
 * (G1), exercised in Phase 5 with two processes.
 *
 * Sequence:
 *   1. program windows 1, 2 (local) -> dst1, dst2
 *   2. small aperture stores on both windows -> poll both destinations
 *   3. bulk DMA on both windows -> poll completion, verify contents
 *   4. cross-window global ordering: DMA on win 1, flag store on win 2;
 *      the flag implies the DMA completed (single order FIFO)
 *   5. quiesce, then check counter relations
 *
 * Note (sim): the TB's EN_RANDOMIZATION pads every ctrl write with extra
 * writes in the rest of its 64 B line - polled locations are kept on
 * lines of their own, and counter checks use relations between counters
 * rather than absolute store counts.
 */

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#include <coyote/cThread.hpp>
#include "loom.hpp"

constexpr uint64_t BUF_SIZE   = 2ULL * 1024 * 1024;
constexpr uint64_t DMA_BYTES  = 4096;
constexpr int      POLL_SECS  = 600;   // XSIM is slow; generous wall-clock budget

static int failures = 0;

static void check(bool ok, const char *msg) {
    printf("%s: %s\n", ok ? "PASS" : "FAIL", msg);
    if (!ok) failures++;
}

// Poll a 64-bit word in ordinary memory (never a CSR: CSR reads are
// blocked while the engine pulls in the simulation backend)
static bool poll64(volatile uint64_t *addr, uint64_t want) {
    for (int i = 0; i < POLL_SECS * 100; i++) {
        if (*addr == want) return true;
        usleep(10000);
    }
    return false;
}

int main() {
    coyote::cThread t(0, getpid(), 0);
    const uint32_t pid = t.getCtid();
    printf("ctid %d\n", pid);

    auto *dst1 = static_cast<uint64_t *>(t.getMem({coyote::CoyoteAllocType::HPF, BUF_SIZE}));
    auto *dst2 = static_cast<uint64_t *>(t.getMem({coyote::CoyoteAllocType::HPF, BUF_SIZE}));
    auto *src  = static_cast<uint64_t *>(t.getMem({coyote::CoyoteAllocType::HPF, BUF_SIZE}));
    auto *cpl  = static_cast<uint64_t *>(t.getMem({coyote::CoyoteAllocType::REG, 4096}));
    if (!dst1 || !dst2 || !src || !cpl) { printf("FAIL: getMem\n"); return 1; }
    memset(dst1, 0, BUF_SIZE);
    memset(dst2, 0, BUF_SIZE);
    memset(cpl, 0, 4096);
    for (uint64_t i = 0; i < BUF_SIZE / 8; i++) src[i] = 0xA5A5'0000'0000'0000ULL | i;

    // 1. Two local windows (the fence VA rides in each descriptor)
    loom::program_window(t, 1, false, pid, dst1, BUF_SIZE);
    loom::program_window(t, 2, false, pid, dst2, BUF_SIZE);

    // 2. Small stores, one per window (distinct 64 B lines)
    loom::aperture_store(t, 1, 0x40, 0xDEAD'BEEF'0000'0001ULL);
    loom::aperture_store(t, 2, 0x40, 0xDEAD'BEEF'0000'0002ULL);
    check(poll64(&dst1[8], 0xDEAD'BEEF'0000'0001ULL), "win1 small store lands");
    check(poll64(&dst2[8], 0xDEAD'BEEF'0000'0002ULL), "win2 small store lands");

    // 3. Bulk DMA on both windows
    loom::dma(t, 1, 0x10000, src, DMA_BYTES, pid, cpl);
    loom::dma(t, 2, 0x10000, src, DMA_BYTES, pid, cpl);
    check(poll64(&cpl[0], 2), "DMA completion count 2");
    check(memcmp(reinterpret_cast<uint8_t *>(dst1) + 0x10000, src, DMA_BYTES) == 0,
          "win1 DMA payload matches source");
    check(memcmp(reinterpret_cast<uint8_t *>(dst2) + 0x10000, src, DMA_BYTES) == 0,
          "win2 DMA payload matches source");

    // 4. Cross-window global ordering: descriptor (win 1), flag (win 2)
    loom::dma(t, 1, 0x20000, src, DMA_BYTES, pid, cpl);
    loom::aperture_store(t, 2, 0x800, 0xF1A6ULL);
    check(poll64(&dst2[0x800 / 8], 0xF1A6ULL), "cross-window flag lands");
    check(cpl[0] == 3, "flag implies DMA completed (global order point)");
    check(memcmp(reinterpret_cast<uint8_t *>(dst1) + 0x20000, src, DMA_BYTES) == 0,
          "ordering-DMA payload matches source");

    // 5. Quiesce (counters stable across two reads), then relations
    uint64_t c0[8], c1[8];
    for (int tries = 0; tries < 60; tries++) {
        for (int i = 0; i < 8; i++) c0[i] = loom::csr_read(t, loom::DBG_BASE + 8 * i);
        usleep(200000);
        for (int i = 0; i < 8; i++) c1[i] = loom::csr_read(t, loom::DBG_BASE + 8 * i);
        if (memcmp(c0, c1, sizeof(c0)) == 0) break;
    }
    const char *names[8] = {"stores", "descs", "local_wr", "rdma_wr",
                            "rx_fwd", "drops", "fifo_ovfl", "compl"};
    for (int i = 0; i < 8; i++) printf("dbg[%s] = %lu\n", names[i], c1[i]);
    check(c1[1] == 3, "counters: 3 descriptors");
    check(c1[7] == 3, "counters: 3 completions");
    check(c1[2] == c1[0] + c1[1], "counters: local_wr == stores + descs");
    check(c1[3] == 0 && c1[4] == 0, "counters: no rdma/rx traffic");
    check(c1[5] == 0 && c1[6] == 0, "counters: no drops, no overflow");

    printf(failures == 0 ? "LOOM TEST PASS\n" : "LOOM TEST FAIL (%d)\n", failures);
    return failures == 0 ? 0 : 1;
}
