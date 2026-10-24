/**
 * ce_prog - the copy engine's programs on one V80, no U280 needed:
 *
 *   1. REDUCE two HBM buffers into HBM, COPY the result to host memory,
 *      STAMP before and after (card memory); every int32 checked
 *   2. WAIT: COPY into HBM, SIGNAL a flag in HBM, WAIT for it, COPY out;
 *      twice, with RUN_BASE 0 and 1000
 *   3. ABORT: a WAIT that never completes, ended by ABORT
 *   4. rates: N copies of each size from HBM to host memory, in one program
 *      (RUN_CYC) and as single copies started by the host (START, BUSY
 *      polled; BUSY reads per copy), for comparison; `rates_only` runs
 *      only this, and its host-started column also on an image without
 *      programs
 *
 *   ce_prog [rates_only]
 */
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <coyote/cThread.hpp>
#include "ce_prog.hpp"

namespace {

// loom_ce_ctrl.sv, the single copy
enum CeReg : uint32_t { START = 0, SRC_VA = 1, DST_VA = 2, LEN = 3, FENCE_VA = 5, DIRECTION = 6 };

template <typename T> T *alloc(coyote::cThread &t, uint64_t bytes) {
    T *p = static_cast<T *>(t.getMem({coyote::CoyoteAllocType::HPF, bytes}));
    if (!p) { fprintf(stderr, "no memory (%lu B)\n", (unsigned long) bytes); exit(1); }
    std::memset(p, 0, bytes);
    return p;
}
void to_card(coyote::cThread &t, void *p, uint64_t bytes) { t.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{p, bytes}); }
void to_host(coyote::cThread &t, void *p, uint64_t bytes) { t.invoke(coyote::CoyoteOper::LOCAL_SYNC, coyote::syncSg{p, bytes}); }

int fails = 0;
void check(bool ok, const char *what) {
    printf("%s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) fails++;
}

}  // namespace

int main(int argc, char **argv) {
    const bool rates_only = (argc > 1 && strcmp(argv[1], "rates_only") == 0);
    coyote::cThread v80(0, getpid(), 0, nullptr, "loom_versal_fpga");

    // csr: the host's CSR round trip (any loom_ce image, programs or not): a
    // read of BUSY, and a write (DST_VA) followed by that read
    if (argc > 1 && strcmp(argv[1], "csr") == 0) {
        const int n = 20000;
        uint64_t sink = 0;
        for (int r = 0; r < 3; r++) {
            auto t0 = std::chrono::steady_clock::now();
            for (int i = 0; i < n; i++) sink += v80.getCSR(ce_prog::BUSY);
            auto t1 = std::chrono::steady_clock::now();
            for (int i = 0; i < n; i++) { v80.setCSR(uint64_t(i) << 6, DST_VA); sink += v80.getCSR(ce_prog::BUSY); }
            auto t2 = std::chrono::steady_clock::now();
            printf("csr: read %.0f ns, write + read %.0f ns\n",
                   std::chrono::duration<double, std::nano>(t1 - t0).count() / n,
                   std::chrono::duration<double, std::nano>(t2 - t1).count() / n);
        }
        return sink == ~0ULL;
    }
    ce_prog::Program p(v80);

    if (!rates_only) {
        // --- 1. REDUCE in HBM, the result to host memory, stamps ---
        const uint64_t n = 64 * 1024;
        int32_t *a = alloc<int32_t>(v80, n), *b = alloc<int32_t>(v80, n), *d = alloc<int32_t>(v80, n);
        int32_t *h = alloc<int32_t>(v80, n);
        uint64_t *st = alloc<uint64_t>(v80, 4096);
        for (uint64_t i = 0; i < n / 4; i++) { a[i] = int32_t(i * 7 + 1); b[i] = int32_t(i * 3 - 5); }
        to_card(v80, a, n); to_card(v80, b, n); to_card(v80, d, n); to_card(v80, st, 4096);
        p.clear();
        p.add(ce_prog::stamp(&st[0], true));
        p.add(ce_prog::reduce(a, b, d, n, true));
        p.add(ce_prog::copy(d, h, n));
        p.add(ce_prog::stamp(&st[1], true));
        p.load();
        const bool ran = p.run();
        to_host(v80, st, 4096);
        bool same = true;
        for (uint64_t i = 0; i < n / 4; i++) same &= (h[i] == int32_t(i * 10 - 4));
        check(ran && same, "REDUCE 64 KiB in HBM, copied to host memory, every int32");
        check(st[1] > st[0], "STAMP before and after");
        printf("     REDUCE + COPY 64 KiB: %lu cycles between the stamps, run %lu cycles\n",
               (unsigned long) (st[1] - st[0]), (unsigned long) p.run_cycles());

        // --- 2. WAIT on a flag the same program signals ---
        uint64_t *flag = alloc<uint64_t>(v80, 4096);
        int32_t *d2 = alloc<int32_t>(v80, n), *h2 = alloc<int32_t>(v80, n);
        to_card(v80, flag, 4096); to_card(v80, d2, n);
        p.clear();
        p.add(ce_prog::copy(a, d2, n, true));
        p.add(ce_prog::signal(&flag[3], 1, true));
        p.add(ce_prog::wait(&flag[3], 1));
        p.add(ce_prog::copy(d2, h2, n));
        p.load();
        for (uint64_t base : {0ULL, 1000ULL}) {
            std::memset(h2, 0, n);
            const bool ok = p.run(base);
            check(ok && std::memcmp(h2, a, n) == 0, base ? "WAIT, RUN_BASE 1000" : "WAIT, RUN_BASE 0");
        }

        // --- 3. ABORT a WAIT that never completes ---
        p.clear();
        p.add(ce_prog::wait(&flag[5], 1ULL << 40));
        p.load();
        p.start();
        const bool done = p.finish(std::chrono::milliseconds(20));
        check(!done && v80.getCSR(ce_prog::ABORTED) == 1 && v80.getCSR(ce_prog::BUSY) == 0,
              "ABORT ends a WAIT");
        printf("     polls so far %lu\n", (unsigned long) v80.getCSR(ce_prog::POLLS));
    }

    // --- 4. rates: one program vs single copies started by the host ---
    const uint64_t src_bytes = 16ULL << 20;
    uint8_t *src = alloc<uint8_t>(v80, src_bytes);
    uint8_t *hdst = alloc<uint8_t>(v80, src_bytes);
    for (uint64_t i = 0; i < src_bytes; i++) src[i] = uint8_t(i * 13);
    to_card(v80, src, src_bytes);
    printf("rates, GB/s, HBM -> host memory:\n");
    printf("  %10s %6s %12s %12s %12s\n", "size", "copies", "program", "host-started", "BUSY reads");
    for (uint64_t size : {256ULL, 1024ULL, 4096ULL, 12288ULL, 16384ULL, 20480ULL, 65536ULL, 262144ULL, 1048576ULL,
                          4194304ULL}) {
        const uint64_t n = std::min<uint64_t>(ce_prog::N_PROG, std::max<uint64_t>(8, (64ULL << 20) / size));
        double gbs = 0;
        p.clear();
        for (uint64_t i = 0; i < n; i++) {
            const uint64_t off = (i % (src_bytes / size)) * size;   // whole copies inside the buffers
            p.add(ce_prog::copy(src + off, hdst + off, size));
        }
        p.load();
        if (p.run()) gbs = double(n * size) / (double(p.run_cycles()) * ce_prog::CLK_NS);
        else { printf("  run failed at %lu B\n", (unsigned long) size); fails++; }
        // single copies, HBM -> host memory, each started when the last is done
        v80.setCSR(v80.getCtid(), ce_prog::PID);
        v80.setCSR(size, LEN);
        v80.setCSR(0, FENCE_VA);
        v80.setCSR(0, DIRECTION);
        uint64_t reads = 0;
        const auto t0 = std::chrono::steady_clock::now();
        for (uint64_t i = 0; i < n; i++) {
            const uint64_t off = (i % (src_bytes / size)) * size;   // whole copies inside the buffers
            v80.setCSR(ce_prog::va(src + off), SRC_VA);
            v80.setCSR(ce_prog::va(hdst + off), DST_VA);
            v80.setCSR(1, START);
            do reads++; while (v80.getCSR(ce_prog::BUSY));
        }
        const double host_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        printf("  %10lu %6lu %12.2f %12.2f %12.2f\n", (unsigned long) size, (unsigned long) n, gbs,
               double(n * size) / host_s / 1e9, double(reads) / n);
    }

    printf(fails ? "ce_prog: %d FAILED\n" : "ce_prog: all passed\n", fails);
    return fails ? 1 : 0;
}
