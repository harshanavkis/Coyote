/**
 * 5.1a: single-binary, role-split demo/test. Client/server structure with
 * an in-process transport:
 *
 *   orchestrator (server) - InProcOrchestrator, sole owner of the CSR
 *       page through "its" cThread
 *   XPU A (client)        - importer: aperture stores + DMA descriptors
 *   XPU B (client)        - exporter: allocates buffers, exports, polls
 *
 * Mode selection at runtime:
 *   - simulation (COYOTE_SIM_DIR set): the mock backend supports exactly
 *     one cThread per process, so all three roles share it (degenerate
 *     mode). The role separation - who calls what - is still exercised
 *     in full; only the ctids collapse to one value.
 *   - hardware (no COYOTE_SIM_DIR): each role gets its own cThread, so
 *     A's imports translate under B's distinct ctid (gate G1). Untested
 *     until Phase 5.3; the code path exists so 5.3 is a rerun, not a
 *     rewrite.
 *
 * Run instructions: README "Role-split demo (Phase 5.1a)".
 */

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <unistd.h>

#include <coyote/cThread.hpp>
#include "loom.hpp"
#include "loom_orch.hpp"
#include "loom_xpu.hpp"

constexpr uint64_t BUF_SIZE  = 2ULL * 1024 * 1024;
constexpr uint64_t DMA_BYTES = 4096;
constexpr int      POLL_SECS = 600;

static int failures = 0;

static void check(bool ok, const char *msg) {
    printf("%s: %s\n", ok ? "PASS" : "FAIL", msg);
    if (!ok) failures++;
}

static bool poll64(volatile uint64_t *addr, uint64_t want) {
    for (int i = 0; i < POLL_SECS * 100; i++) {
        if (*addr == want) return true;
        usleep(10000);
    }
    return false;
}

int main() {
    const bool sim = getenv("COYOTE_SIM_DIR") != nullptr;
    printf("mode: %s\n", sim ? "simulation (single cThread, degenerate)"
                             : "hardware (cThread per role)");

    // --- attach cThreads per mode ---
    std::mutex io_mtx;
    std::unique_ptr<coyote::cThread> t_orch, t_a, t_b;
    t_orch = std::make_unique<coyote::cThread>(0, getpid(), 0);
    if (!sim) {
        t_a = std::make_unique<coyote::cThread>(0, getpid(), 0);
        t_b = std::make_unique<coyote::cThread>(0, getpid(), 0);
    }
    coyote::cThread &ta = sim ? *t_orch : *t_a;
    coyote::cThread &tb = sim ? *t_orch : *t_b;

    // --- roles ---
    loom::InProcOrchestrator orch(*t_orch);
    loom::Xpu A(ta, orch, io_mtx);
    loom::Xpu B(tb, orch, io_mtx);
    printf("ctids: orch %d, A %d, B %d\n",
           t_orch->getCtid(), A.ctid(), B.ctid());

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
    if (!src || !fence) { printf("FAIL: A alloc\n"); return 1; }
    memset(fence, 0, 4096);
    for (uint64_t i = 0; i < BUF_SIZE / 8; i++) src[i] = 0x5A5A'0000'0000'0000ULL | i;

    int w1 = A.importBuf(h1);
    int w2 = A.importBuf(h2);
    check(w1 == 1 && w2 == 2, "import -> windows 1, 2");

    // Server-side validation: a bogus handle must be refused
    check(A.importBuf(1234) == loom::NO_WINDOW, "bogus handle refused");

    // --- peer stores through both windows ---
    A.store(w1, 0x40, 0xB00B'0000'0000'0001ULL);
    A.store(w2, 0x40, 0xB00B'0000'0000'0002ULL);
    check(poll64(&dst1[8], 0xB00B'0000'0000'0001ULL), "B sees store via w1");
    check(poll64(&dst2[8], 0xB00B'0000'0000'0002ULL), "B sees store via w2");

    // --- peer copy with fence ---
    A.copy(w1, 0x10000, src, DMA_BYTES, fence);
    check(poll64(&fence[0], 1), "fence 1 after copy");
    check(memcmp(reinterpret_cast<uint8_t *>(dst1) + 0x10000, src, DMA_BYTES) == 0,
          "copy payload matches");

    // --- ordering: copy on w1, flag on w2; flag implies fence ---
    A.copy(w1, 0x20000, src, DMA_BYTES, fence);
    A.store(w2, 0x800, 0xF1A6ULL);
    check(poll64(&dst2[0x800 / 8], 0xF1A6ULL), "flag lands");
    check(fence[0] == 2, "flag implies copy fenced (order point)");
    check(memcmp(reinterpret_cast<uint8_t *>(dst1) + 0x20000, src, DMA_BYTES) == 0,
          "ordering-copy payload matches");

    // --- release: window invalidated, store is dropped by the engine ---
    // (main is single-threaded here, so plain csr_read calls are safe; the
    // io mutex only matters once roles run on their own threads in 5.3)
    orch.releaseWindow(w2);
    uint64_t drops0 = loom::csr_read(*t_orch, loom::DBG_BASE + 8 * 5);
    A.store(w2, 0x40, 0xDEADULL);
    // ">=" not "==": the sim TB's randomization padding multiplies one
    // store into several aperture writes, all dropped on a dead window
    bool dropped = false;
    for (int i = 0; i < POLL_SECS * 10 && !dropped; i++) {
        dropped = loom::csr_read(*t_orch, loom::DBG_BASE + 8 * 5) >= drops0 + 1;
        if (!dropped) usleep(10000);
    }
    check(dropped, "store to released window dropped");
    check(dst2[8] == 0xB00B'0000'0000'0002ULL, "released window: no new write");

    printf(failures == 0 ? "LOOM ROLES TEST PASS\n" : "LOOM ROLES TEST FAIL (%d)\n",
           failures);
    return failures == 0 ? 0 : 1;
}
