/**
 * v80_bisect - V80-only steps for finding what in a loom_ce image resets
 * the host (no U280 involved):
 *
 *   v80_bisect csr SECONDS       read the CSR page (copy, landing and debug
 *                                counters) in a loop, nothing else
 *   v80_bisect host BYTES REPS   loom_ce copies HBM -> a host buffer of the
 *                                V80's own cThread (its own DMA path), then
 *                                a fence; every byte checked
 *   v80_bisect cpu BYTES         the host CPU writes into the V80's window
 *                                (EN_UWIN_HBM: it is HBM), a buffer bound to
 *                                it; reads it back through the window and
 *                                after a sync, both checked
 */
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <immintrin.h>

#include <unistd.h>

#include <coyote/cThread.hpp>
#include "land_v80.hpp"

namespace {

enum CeReg : uint32_t { START = 0, SRC_VA = 1, DST_VA = 2, LEN = 3, PID = 4, FENCE_VA = 5, DIRECTION = 6,
                        BUSY = 8, COPIES = 9, CYCLES = 10 };

uint64_t pattern(uint64_t off, int rep) { return 0xCE00000000000000ULL ^ (off * 0x9E3779B97F4A7C15ULL) ^ rep; }

}  // namespace

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s csr SECONDS | host BYTES REPS\n", argv[0]);
        return 2;
    }
    coyote::cThread v80(0, getpid(), 0, nullptr, "coyote_versal_fpga");

    if (strcmp(argv[1], "csr") == 0) {
        const double secs = atof(argv[2]);
        const uint32_t words[] = {8, 9, 10, 16, 17, 18, 24, 25, 26, 27, 28, 29, 32, 40, 45, 48, 49, 50};
        uint64_t reads = 0, sum = 0;
        const auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < secs)
            for (uint32_t w : words) { sum += v80.getCSR(w); reads++; }
        printf("csr: %lu reads in %.1f s (checksum %lx)\nCSR PASS\n", (unsigned long) reads, secs, (unsigned long) sum);
        return 0;
    }

    if (strcmp(argv[1], "cpu") == 0) {
        // the host writes into the window (it is HBM), reads it back through
        // the window, and through a sync of the bound buffer
        const uint64_t size = strtoull(argv[2], nullptr, 0);
        land_v80::Landing L(v80, size + 4096);
        uint64_t *host = static_cast<uint64_t *>(malloc(size));
        for (uint64_t i = 0; i < size / 8; i++) host[i] = pattern(8 * i, 7);
        const auto t0 = std::chrono::steady_clock::now();
        v80.uwinWrite(0, host, size);
        const uint64_t fv = 0xF00DF00DULL;
        v80.uwinWrite(size, &fv, 8);
        const bool fenced = L.wait(size, fv);
        const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
        uint64_t bad_win = 0, bad_sync = 0;
        for (uint64_t i = 0; i < size / 8; i++) bad_win += (L.win[i] != host[i]);
        L.pull();
        for (uint64_t i = 0; i < size / 8; i++) bad_sync += (L.buf[i] != host[i]);
        const bool ok = fenced && !bad_win && !bad_sync && L.buf[size / 8] == fv;
        printf("%s cpu: %lu bytes + fence through the window into HBM, fence read back after %.0f us; "
               "%lu words wrong read through the window, %lu after a sync\n",
               ok ? "ok  " : "FAIL", (unsigned long) size, us, (unsigned long) bad_win, (unsigned long) bad_sync);
        printf(ok ? "CPU PASS\n" : "CPU FAIL\n");
        return ok ? 0 : 1;
    }

    const uint64_t size = strtoull(argv[2], nullptr, 0);
    const int reps = (argc > 3) ? atoi(argv[3]) : 1;
    uint64_t *src   = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, size}));
    uint64_t *dst   = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, size}));
    uint64_t *fence = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, 4096}));
    volatile uint64_t *vfence = fence;
    int errors = 0;
    for (int r = 0; r < reps; r++) {
        for (uint64_t i = 0; i < size / 8; i++) src[i] = pattern(8 * i, r);
        v80.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{src, size});
        memset(dst, 0, size);
        *vfence = 0;
        const uint64_t want = v80.getCSR(COPIES) + 1;
        v80.setCSR(reinterpret_cast<uint64_t>(src), SRC_VA);
        v80.setCSR(reinterpret_cast<uint64_t>(dst), DST_VA);
        v80.setCSR(size, LEN);
        v80.setCSR(v80.getCtid(), PID);
        v80.setCSR(reinterpret_cast<uint64_t>(fence), FENCE_VA);
        v80.setCSR(0, DIRECTION);   // a put; DIRECTION outlives a ce_get run
        const auto t0 = std::chrono::steady_clock::now();
        v80.setCSR(1, START);
        while (*vfence != want && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5)) _mm_pause();
        const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
        uint64_t bad = 0;
        for (uint64_t i = 0; i < size / 8; i++) bad += (dst[i] != pattern(8 * i, r));
        const bool ok = (*vfence == want) && !bad;
        errors += !ok;
        printf("%s copy %d: %lu bytes, fence %lu (want %lu) after %.1f us (%.2f GB/s), %lu words wrong, CE %lu cycles\n",
               ok ? "ok  " : "FAIL", r, (unsigned long) size, (unsigned long) *vfence, (unsigned long) want,
               us, size / us / 1e3, (unsigned long) bad, (unsigned long) v80.getCSR(CYCLES));
    }
    printf(errors ? "HOST FAIL\n" : "HOST PASS\n");
    return errors ? 1 : 0;
}
