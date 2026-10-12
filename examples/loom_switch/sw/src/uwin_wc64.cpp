/**
 * uwin_wc64 - does the U280 pay a cost per packet or per byte for writes
 * into its user data window?
 *
 * Bulk writes from the V80 or the host's PTDMA engine arrive as 256 B
 * packets and stop at ~10.6 GB/s, one packet every 6 cycles at 250 MHz
 * (41.7 M packets/s). Here many CPU threads store full 64 B lines through
 * the write-combining mapping, so each line leaves the CPU as one 64 B
 * posted write. No window is programmed: loom_ingress drops every write and
 * counts it as a 1-beat burst. Packets/s near 41.7 M again means a cost per
 * packet inside the U280; clearly more means the limit is per byte.
 *
 * Usage: uwin_wc64 [threads] [seconds] [lines per sfence]
 * Run under numactl on the U280's node; thread i is pinned to the i-th CPU
 * the process may run on.
 */
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <immintrin.h>
#include <pthread.h>
#include <sched.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <coyote/cThread.hpp>
#include "loom_switch.hpp"

using namespace loom_switch;

namespace {

constexpr uint64_t REGION = 64 << 10;   // uwin bytes each thread cycles over

struct alignas(64) Lines { std::atomic<uint64_t> n{0}; };

void storm(char *base, int fence_every, int cpu, const std::atomic<bool> *stop, Lines *lines) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    const __m256i lo = _mm256_set1_epi64x(0x0C0FFEE000000000LL), hi = _mm256_set1_epi64x(0x0DECAF0000000000LL);
    uint64_t off = 0, n = 0;
    while (!stop->load(std::memory_order_relaxed)) {
        for (int i = 0; i < fence_every; i++) {
            // Both halves of one line: the write-combining buffer leaves full
            _mm256_stream_si256(reinterpret_cast<__m256i *>(base + off), lo);
            _mm256_stream_si256(reinterpret_cast<__m256i *>(base + off + 32), hi);
            off = (off + 64) % REGION;
        }
        _mm_sfence();
        n += fence_every;
        lines->n.store(n, std::memory_order_relaxed);
    }
}

uint64_t total(const std::vector<Lines> &l) {
    uint64_t s = 0;
    for (auto &x : l) s += x.n.load(std::memory_order_relaxed);
    return s;
}

}  // namespace

int main(int argc, char **argv) {
    const int threads     = (argc > 1) ? atoi(argv[1]) : 8;
    const double secs     = (argc > 2) ? atof(argv[2]) : 3.0;
    const int fence_every = (argc > 3) ? atoi(argv[3]) : 16;

    cpu_set_t allowed;
    sched_getaffinity(0, sizeof(allowed), &allowed);
    std::vector<int> cpus;
    for (int c = 0; c < CPU_SETSIZE; c++)
        if (CPU_ISSET(c, &allowed)) cpus.push_back(c);
    if (threads < 1 || threads > (int) cpus.size() || secs <= 0 || fence_every < 1) {
        fprintf(stderr, "usage: uwin_wc64 [threads, at most %zu] [seconds] [lines per sfence]\n", cpus.size());
        return 2;
    }

    coyote::cThread t(0, getpid());
    // Nothing may land: every window invalid, so the ingress drops each write
    for (uint32_t w = 0; w < 16; w++) release_window(t, w);
    (void) csr_read(t, TBL_IDX);
    char *uwin = static_cast<char *>(t.mapUwin(threads * REGION));

    std::atomic<bool> stop{false};
    std::vector<Lines> lines(threads);
    std::vector<std::thread> pool;
    for (int i = 0; i < threads; i++)
        pool.emplace_back(storm, uwin + i * REGION, fence_every, cpus[i], &stop, &lines[i]);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));   // all threads storing
    const IngressCounters c0 = IngressCounters::read(t);
    const uint64_t cyc0 = csr_read(t, CYC), l0 = total(lines);
    const auto t0 = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::duration<double>(secs));
    const IngressCounters c1 = IngressCounters::read(t);
    const uint64_t cyc1 = csr_read(t, CYC), l1 = total(lines);
    const auto t1 = std::chrono::steady_clock::now();
    stop = true;
    for (auto &th : pool) th.join();
    t.unmapUwin();

    const double s = std::chrono::duration<double>(t1 - t0).count();
    // Bursts the ingress took: forwarded (0) plus dropped (1)
    const uint64_t cyc = cyc1 - cyc0, bursts = (c1.v[0] - c0.v[0]) + (c1.v[1] - c0.v[1]), cpu_lines = l1 - l0;
    const uint64_t one_beat = c1.v[17] - c0.v[17], landed = (c1.v[2] - c0.v[2]) + (c1.v[3] - c0.v[3]);
    printf("wc64: %d threads, %.2f s, sfence every %d lines: CPU %.1f M lines/s (%.2f GB/s); "
           "ingress %.1f M bursts/s (%.2f GB/s at 64 B), 1-beat %.1f%%, %.2f cycles/burst, clock %.1f MHz\n",
           threads, s, fence_every, cpu_lines / s / 1e6, cpu_lines * 64 / s / 1e9,
           bursts / s / 1e6, bursts * 64 / s / 1e9, bursts ? 100.0 * one_beat / bursts : 0.0,
           bursts ? double(cyc) / bursts : 0.0, cyc / s / 1e6);
    print_ingress_delta(c0, c1);
    if (landed) printf("FAIL: %lu packets landed - a window was programmed\n", (unsigned long) landed);
    return landed ? 1 : 0;
}
