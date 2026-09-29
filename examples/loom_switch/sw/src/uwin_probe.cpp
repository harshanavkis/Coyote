/**
 * uwin_probe - gate G2: the host CPU writes into the U280's user data window
 * and the bytes land, through loom_ingress's local route, in a host buffer.
 *
 *   1. bulk: the CPU copies a pattern into window 1 through the
 *      write-combining mapping, fences, then stores an 8 B flag into
 *      window 2; the probe polls the flag in host memory and checks every
 *      byte of the destination
 *   2. stores: scattered 8 B stores, each fenced, then every one checked
 *
 * The ingress counters show how the CPU handed the writes over: full lines
 * become packets, anything the write-combining buffer flushed partially
 * becomes 8 B stores. Both land the same bytes.
 *
 * Needs the loom_switch bitstream (EN_UWIN) and coyote_driver with MMAP_UWIN;
 * or, linked against the simulation library (EN_SIM), COYOTE_SIM_DIR set to
 * a loom_switch simulation build. Writes go through cThread::uwinWrite, so
 * the same code drives both.
 */
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <immintrin.h>
#include <unistd.h>

#include <coyote/cThread.hpp>
#include "loom_switch.hpp"

using namespace loom_switch;

static const char *ING_NAMES[N_ING] = {
    "bursts", "bursts dropped", "local packets", "rdma packets",
    "stores", "partial words", "idle flushes"};

static void print_delta(const IngressCounters &a, const IngressCounters &b) {
    for (int i = 0; i < N_ING; i++)
        printf("    %-15s %lu\n", ING_NAMES[i], (unsigned long) (b.v[i] - a.v[i]));
}

static uint64_t pattern(uint64_t off) { return 0x10AD000000000000ULL ^ (off * 0x9E3779B97F4A7C15ULL); }

int main(int argc, char **argv) {
    const uint64_t size = (argc > 1) ? strtoull(argv[1], nullptr, 0) : (1ULL << 20);
    const int n_stores  = (argc > 2) ? atoi(argv[2]) : 256;
    if (size == 0 || size % 4096 || size > (64ULL << 20)) {
        fprintf(stderr, "size must be a multiple of 4 KiB, at most 64 MiB\n");
        return 2;
    }

    // Simulated time is slow in wall-clock terms
    const auto patience = getenv("COYOTE_SIM_DIR") ? std::chrono::seconds(1800) : std::chrono::seconds(2);

    coyote::cThread t(0, getpid());
    const uint32_t pid = t.getCtid();

    // Destination and flag buffers, and the uwin: window 1 at uwin 0,
    // window 2 (the flag, one page) right after it
    uint64_t *dst  = static_cast<uint64_t *>(t.getMem({coyote::CoyoteAllocType::HPF, size}));
    uint64_t *flag = static_cast<uint64_t *>(t.getMem({coyote::CoyoteAllocType::HPF, 4096}));
    uint64_t *src  = static_cast<uint64_t *>(aligned_alloc(64, size));
    memset(dst, 0, size);
    memset(flag, 0, 4096);
    for (uint64_t i = 0; i < size / 8; i++) src[i] = pattern(8 * i);

    program_window(t, 1, false, pid, dst, size, 0);
    program_window(t, 2, false, pid, flag, 4096, size);
    t.mapUwin(size + 4096);
    volatile uint64_t *vflag = flag;
    volatile uint64_t *vdst = dst;
    int errors = 0;

    // --- 1. bulk, then a flag behind it ---
    IngressCounters c0 = IngressCounters::read(t);
    auto t0 = std::chrono::steady_clock::now();
    const uint64_t one = 1;
    t.uwinWrite(0, src, size);
    t.uwinWrite(size, &one, 8);
    auto deadline = t0 + patience;
    while (*vflag != 1 && std::chrono::steady_clock::now() < deadline) _mm_pause();
    auto t1 = std::chrono::steady_clock::now();
    IngressCounters c1 = IngressCounters::read(t);

    if (*vflag != 1) {
        printf("FAIL bulk: the flag never landed\n");
        errors++;
    }
    uint64_t bad = 0, first_bad = ~0ULL;
    for (uint64_t i = 0; i < size / 8; i++)
        if (dst[i] != src[i]) { if (!bad) first_bad = i; bad++; }
    if (bad) {
        printf("FAIL bulk: %lu of %lu words wrong, first at byte %lu (%016lx, expected %016lx)\n",
               (unsigned long) bad, (unsigned long) (size / 8), (unsigned long) (8 * first_bad),
               (unsigned long) dst[first_bad], (unsigned long) src[first_bad]);
        errors++;
    }
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    printf("%s bulk: %lu bytes, flag seen after %.1f us (%.2f GB/s)\n",
           (errors ? "FAIL" : "ok  "), (unsigned long) size, us, size / us / 1e3);
    print_delta(c0, c1);

    // --- 2. scattered, fenced 8 B stores ---
    memset(dst, 0, size);
    c0 = IngressCounters::read(t);
    for (int k = 0; k < n_stores; k++) {
        uint64_t w = (uint64_t(k) * 7919 * 8) % size / 8;
        uint64_t v = pattern(8 * w) ^ k;
        t.uwinWrite(8 * w, &v, 8);
    }
    deadline = std::chrono::steady_clock::now() + patience;
    do {
        bad = 0;
        for (int k = 0; k < n_stores; k++) {
            uint64_t w = (uint64_t(k) * 7919 * 8) % size / 8;
            if (vdst[w] != (pattern(8 * w) ^ k)) bad++;
        }
    } while (bad && std::chrono::steady_clock::now() < deadline);
    c1 = IngressCounters::read(t);
    if (bad) {
        printf("FAIL stores: %lu of %d wrong\n", (unsigned long) bad, n_stores);
        errors++;
    } else {
        printf("ok   stores: %d fenced 8 B stores landed\n", n_stores);
    }
    print_delta(c0, c1);

    t.unmapUwin();
    release_window(t, 1);
    release_window(t, 2);
    free(src);
    printf(errors ? "G2 FAIL\n" : "G2 PASS\n");
    return errors ? 1 : 0;
}
