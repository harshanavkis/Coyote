/**
 * Loom end-to-end test (local flows), same binary for simulation
 * (-DEN_SIM=ON, run with COYOTE_SIM_DIR=<hw build>) and hardware.
 *
 * Single cThread (the simulation backend supports exactly one): its own
 * buffers play both the issuer's source and the exporter's destination,
 * so pid_src == pid_dst == ctid. Cross-pid writes are a hardware gate
 * (G1), exercised in Phase 5 with two processes.
 *
 * Sequence:
 *   1. program window 1 (local) -> destination buffer; set completion word
 *   2. small aperture store -> poll destination
 *   3. bulk DMA (4 KB) -> poll completion word, verify contents
 *   4. ordering: DMA + immediate flag store -> flag implies completion
 *   5. dump debug counters
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

    auto *dst = static_cast<uint64_t *>(t.getMem({coyote::CoyoteAllocType::HPF, BUF_SIZE}));
    auto *src = static_cast<uint64_t *>(t.getMem({coyote::CoyoteAllocType::HPF, BUF_SIZE}));
    auto *cpl = static_cast<uint64_t *>(t.getMem({coyote::CoyoteAllocType::REG, 4096}));
    if (!dst || !src || !cpl) { printf("FAIL: getMem\n"); return 1; }
    memset(dst, 0, BUF_SIZE);
    memset(cpl, 0, 4096);
    for (uint64_t i = 0; i < BUF_SIZE / 8; i++) src[i] = 0xA5A5'0000'0000'0000ULL | i;

    // 1. Window 1 -> dst (local route, this cThread's pid); completion word
    loom::program_window(t, 1, false, pid, dst, BUF_SIZE);
    loom::set_completion(t, pid, cpl);

    // 2. Small store: *(P + 0x40) = v  ->  dst[8]
    loom::aperture_store(t, 1, 0x40, 0xDEAD'BEEF'0000'0001ULL);
    check(poll64(&dst[8], 0xDEAD'BEEF'0000'0001ULL), "small store lands at dst+0x40");

    // 3. Bulk DMA: copy(P + 0x10000, src, 4 KB)
    loom::dma(t, 1, 0x10000, src, DMA_BYTES, pid);
    check(poll64(&cpl[0], 1), "DMA completion count 1");
    check(memcmp(reinterpret_cast<uint8_t *>(dst) + 0x10000, src, DMA_BYTES) == 0,
          "DMA payload matches source");

    // 4. Ordering: descriptor then flag store; flag implies completion done
    loom::dma(t, 1, 0x20000, src, DMA_BYTES, pid);
    loom::aperture_store(t, 1, 0x800, 0xF1A6ULL);
    check(poll64(&dst[0x800 / 8], 0xF1A6ULL), "flag store lands");
    check(cpl[0] == 2, "flag implies DMA completed (order point)");
    check(memcmp(reinterpret_cast<uint8_t *>(dst) + 0x20000, src, DMA_BYTES) == 0,
          "ordering-DMA payload matches source");

    // 5. Debug counters
    const char *names[8] = {"stores", "descs", "local_wr", "rdma_wr",
                            "rx_fwd", "drops", "fifo_ovfl", "compl"};
    for (int i = 0; i < 8; i++)
        printf("dbg[%s] = %lu\n", names[i], loom::csr_read(t, loom::DBG_BASE + 8 * i));

    printf(failures == 0 ? "LOOM TEST PASS\n" : "LOOM TEST FAIL (%d)\n", failures);
    return failures == 0 ? 0 : 1;
}
