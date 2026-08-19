/**
 * Local-route transfer sweep: correctness across sizes, and the per-stage
 * cycle cost of each one.
 *
 * ./test proves the descriptor path at a single size (4 KB). This walks it
 * from one beat to megabytes, verifies every byte, and reports the engine's
 * own dma-local accumulator delta per transfer - which is the T2 curve the
 * simulator carries a placeholder for, measured rather than assumed.
 *
 * Local route only, one cThread: the point is the transfer size, so the
 * network and the cross-pid split are deliberately out of the picture (both
 * are covered by ./roles and loom_host). Local DMA is byte-granular - the
 * XDMA descriptor bypass, gate G4 - so the sweep includes lengths that are
 * not multiples of 64 B, which the rdma route excludes by contract.
 *
 * Hardware only in practice: the sizes here take far too long under XSIM.
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include <coyote/cThread.hpp>
#include "loom.hpp"

namespace {

constexpr uint64_t BUF_SIZE = 64ULL * 1024 * 1024;   // src and dst each

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

// Sizes: powers of two from one beat up, plus lengths that are deliberately
// not multiples of 64 B and not powers of two
const uint64_t SIZES[] = {
    64, 128, 256, 512, 1024, 4096, 16384, 65536,
    256ULL * 1024, 1024ULL * 1024, 4ULL * 1024 * 1024, 16ULL * 1024 * 1024,
    8, 100, 4095, 4097, 65535
};

} // namespace

int main() {
    coyote::cThread t(0, getpid(), 0);
    const uint32_t pid = t.getCtid();
    printf("ctid %d, buffers %lu MB\n", pid,
           (unsigned long) (BUF_SIZE >> 20));

    auto *dst = static_cast<uint8_t *>(
        t.getMem({coyote::CoyoteAllocType::HPF, BUF_SIZE}));
    auto *src = static_cast<uint8_t *>(
        t.getMem({coyote::CoyoteAllocType::HPF, BUF_SIZE}));
    auto *cpl = static_cast<uint64_t *>(
        t.getMem({coyote::CoyoteAllocType::REG, 4096}));
    if (!dst || !src || !cpl) { printf("FAIL: getMem\n"); return 1; }

    for (uint64_t i = 0; i < BUF_SIZE; i++) src[i] = uint8_t(i * 7 + (i >> 13));
    memset(cpl, 0, 4096);

    loom::program_window(t, 1, false, pid, dst, BUF_SIZE);

    printf("%12s %10s %10s %12s %10s  %s\n",
           "bytes", "cycles", "cyc/KB", "queue_cyc", "beats", "result");

    uint64_t expected_compl = 0;
    for (uint64_t len : SIZES) {
        if (len > BUF_SIZE) continue;
        memset(dst, 0, len + 64);

        loom::StageStats a = loom::read_stage_stats(t);
        loom::dma(t, 1, 0, src, len, pid, cpl);
        expected_compl++;
        bool fenced = poll64(&cpl[0], expected_compl);
        loom::StageStats b = loom::read_stage_stats(t);

        bool ok = fenced && memcmp(dst, src, len) == 0;
        // Nothing beyond the transfer may be touched: a length rounded up to
        // a beat boundary would show here
        for (uint64_t i = len; ok && i < len + 64; i++)
            if (dst[i] != 0) {
                printf("  wrote past the end at +%lu\n",
                       (unsigned long) (i - len));
                ok = false;
            }

        uint64_t cyc = b.acc[loom::STG_DMA_LOCAL] - a.acc[loom::STG_DMA_LOCAL];
        uint64_t ops = b.cnt[loom::STG_DMA_LOCAL] - a.cnt[loom::STG_DMA_LOCAL];
        uint64_t qcyc = b.queue_acc - a.queue_acc;
        printf("%12lu %10lu %10lu %12lu %10lu  %s\n",
               (unsigned long) len, (unsigned long) cyc,
               (unsigned long) (len ? cyc * 1024 / len : 0),
               (unsigned long) qcyc, (unsigned long) ops,
               ok ? "PASS" : (fenced ? "FAIL payload" : "FAIL no fence"));
        if (!ok) failures++;
        if (ops != 1)
            printf("  note: %lu dma-local ops for one descriptor\n",
                   (unsigned long) ops);
    }

    printf(failures == 0 ? "LOOM BULK SWEEP PASS\n"
                         : "LOOM BULK SWEEP FAIL (%d)\n", failures);
    return failures == 0 ? 0 : 1;
}
