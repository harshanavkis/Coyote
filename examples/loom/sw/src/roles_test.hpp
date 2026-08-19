#pragma once

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <unistd.h>

#include <coyote/cThread.hpp>
#include "loom.hpp"
#include "loom_orch.hpp"
#include "loom_xpu.hpp"

/**
 * The role-split test flow, shared by both control-plane transports:
 * roles.cpp runs it against InProcOrchestrator (function calls),
 * roles_sock.cpp against SockOrchClient -> loomd (real Unix socket).
 * Identical checks either way - which is the point: the data plane and
 * the roles cannot tell the transports apart.
 */
namespace loom_test {

constexpr uint64_t BUF_SIZE  = 2ULL * 1024 * 1024;
constexpr uint64_t DMA_BYTES = 4096;
// 600 s made a failing poll look like a hang; override with LOOM_POLL_SECS
// The engine's own account of the descriptor: descs says it was queued,
// drops says it was rejected at the bounds/validity check, local_wr says the
// data write went out, compl says the fence was written. A fence that never
// arrives is one of those four.
inline void dump_counters(coyote::cThread &t, const char *tag) {
    static const char *name[8] = {"stores", "descs", "local_wr", "rdma_wr",
                                  "rx_fwd", "drops", "fifo_ovfl", "compl"};
    printf("counters [%s]:", tag);
    for (int i = 0; i < 8; i++)
        printf(" %s=%lu", name[i],
               (unsigned long) loom::csr_read(t, loom::DBG_BASE + 8 * i));
    printf("\n");
    fflush(stdout);
}

inline int poll_secs() {
    const char *e = getenv("LOOM_POLL_SECS");
    return e ? atoi(e) : 30;
}

inline int  failures = 0;

inline void check(bool ok, const char *msg) {
    printf("%s: %s\n", ok ? "PASS" : "FAIL", msg);
    if (!ok) failures++;
}

inline bool poll64(volatile uint64_t *addr, uint64_t want) {
    for (int i = 0; i < poll_secs() * 100; i++) {
        if (*addr == want) return true;
        usleep(10000);
    }
    return false;
}

// Returns accumulated failures. `release_via` is whichever client should
// issue the revocation (exercises that path over the transport too).
inline int run_roles_flow(coyote::cThread &t_orch, loom::Xpu &A, loom::Xpu &B,
                          loom::OrchClient &release_via) {
    failures = 0;

    // --- B (exporter): allocate + export two segments ---
    auto *dst1 = static_cast<uint64_t *>(B.alloc(BUF_SIZE));
    auto *dst2 = static_cast<uint64_t *>(B.alloc(BUF_SIZE));
    if (!dst1 || !dst2) { printf("FAIL: B alloc\n"); return 1; }
    memset(dst1, 0, BUF_SIZE);
    memset(dst2, 0, BUF_SIZE);
    loom::Handle h1 = B.exportBuf(dst1, BUF_SIZE);
    loom::Handle h2 = B.exportBuf(dst2, BUF_SIZE);
    check(h1 != loom::BAD_HANDLE && h2 != loom::BAD_HANDLE, "export handles");

    // --- A (importer): resolve handles into windows ---
    auto *src = static_cast<uint64_t *>(A.alloc(BUF_SIZE));
    auto *fence = static_cast<uint64_t *>(A.allocSmall(4096));
    if (!src || !fence) { printf("FAIL: A alloc\n"); return failures + 1; }
    memset(fence, 0, 4096);
    for (uint64_t i = 0; i < BUF_SIZE / 8; i++) src[i] = 0x5A5A'0000'0000'0000ULL | i;

    int w1 = A.importBuf(h1);
    int w2 = A.importBuf(h2);
    check(w1 == 1 && w2 == 2, "import -> windows 1, 2");
    check(A.importBuf(1234) == loom::NO_WINDOW, "bogus handle refused");

    // --- peer stores through both windows ---
    A.store(w1, 0x40, 0xB00B'0000'0000'0001ULL);
    A.store(w2, 0x40, 0xB00B'0000'0000'0002ULL);
    check(poll64(&dst1[8], 0xB00B'0000'0000'0001ULL), "B sees store via w1");
    check(poll64(&dst2[8], 0xB00B'0000'0000'0002ULL), "B sees store via w2");

    // --- peer copy with fence ---
    // The engine writes its own running completion count, not 1: compl_cnt
    // only clears on aresetn, so after any earlier run on the same
    // bitstream the next fence carries wherever that counter had got to.
    // Read it and expect the next two.
    const uint64_t f0 = loom::csr_read(t_orch, loom::DBG_BASE + 8 * 7);
    dump_counters(t_orch, "before copy");
    A.copy(w1, 0x10000, src, DMA_BYTES, fence);
    check(poll64(&fence[0], f0 + 1), "fence 1 after copy");
    dump_counters(t_orch, "after copy");
    check(memcmp(reinterpret_cast<uint8_t *>(dst1) + 0x10000, src, DMA_BYTES) == 0,
          "copy payload matches");

    // --- ordering: copy on w1, flag on w2; flag implies fence ---
    A.copy(w1, 0x20000, src, DMA_BYTES, fence);
    A.store(w2, 0x800, 0xF1A6ULL);
    check(poll64(&dst2[0x800 / 8], 0xF1A6ULL), "flag lands");
    check(fence[0] == f0 + 2, "flag implies copy fenced (order point)");
    check(memcmp(reinterpret_cast<uint8_t *>(dst1) + 0x20000, src, DMA_BYTES) == 0,
          "ordering-copy payload matches");

    // --- release: window invalidated, store dropped by the engine ---
    release_via.releaseWindow(w2);
    uint64_t drops0 = loom::csr_read(t_orch, loom::DBG_BASE + 8 * 5);
    A.store(w2, 0x40, 0xDEADULL);
    // ">=": the sim TB's randomization padding multiplies one dropped
    // store into several
    bool dropped = false;
    for (int i = 0; i < poll_secs() * 10 && !dropped; i++) {
        dropped = loom::csr_read(t_orch, loom::DBG_BASE + 8 * 5) >= drops0 + 1;
        if (!dropped) usleep(10000);
    }
    check(dropped, "store to released window dropped");
    check(dst2[8] == 0xB00B'0000'0000'0002ULL, "released window: no new write");

    return failures;
}

} // namespace loom_test
