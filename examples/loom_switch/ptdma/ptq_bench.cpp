/**
 * ptq_bench - ptdma_q on one host, host DRAM -> host DRAM (no FPGA): what one PTDMA engine costs per copy when
 * the hardware gets descriptors straight from user space and completion is polled (no interrupt per descriptor).
 *
 *   ptq_bench --dev /dev/ptq_0000_e8_00_2 [--sizes 64,4096,...] [--iters N] [--batch B] [--mb M] [--user]
 *
 *   lat    one copy, submit -> head says done (p50/p99 us); then that copy + an 8 B flag in one submit (the
 *          collective's step: data, then the signal behind it); "submit" = the ioctl alone (the syscall's share)
 *   check  after each latency copy the destination's last word must hold this iteration's value as soon as
 *          the head register says done (else "head ahead of data")
 *   bw     B copies per submit, kept in flight; GB/s and us per descriptor
 *   order  data + flag back to back; a second thread polls the flag in the destination and checks, when flag i
 *          appears, that the data's first and last words of iteration i are there (a flag that overtook its data)
 * Buffers: 2 MiB huge pages if available, else 4 KiB pages; source and destination each --mb MiB (default 64).
 */
#include <sys/mman.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

#include "../sw/src/ptq.hpp"

namespace {
using clk = std::chrono::steady_clock;
double us(clk::time_point a, clk::time_point b) { return std::chrono::duration<double, std::micro>(b - a).count(); }
double pct(std::vector<double> v, double p) {
    std::sort(v.begin(), v.end());
    return v.empty() ? 0 : v[std::min(v.size() - 1, size_t(p / 100.0 * double(v.size())))];
}
void *alloc(uint64_t len) {
    void *p = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB | MAP_POPULATE, -1, 0);
    if (p == MAP_FAILED) {
        p = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_POPULATE, -1, 0);
        if (p == MAP_FAILED) { perror("mmap"); exit(1); }
        printf("note: 4 KiB pages (no huge pages)\n");
    }
    memset(p, 0, len);
    return p;
}
void spin_until(const ptq::Queue &q, uint64_t s) {
    const auto t0 = clk::now();
    while (!q.done(s))
        if (clk::now() - t0 > std::chrono::seconds(2)) {
            ptq_info i = q.info();
            printf("FAIL: copy did not complete (submitted %llu completed %llu status %#x int %#x)\n",
                   (unsigned long long) i.submitted, (unsigned long long) i.completed, i.status, i.int_status);
            exit(1);
        }
}
}  // namespace

int main(int argc, char **argv) {
    std::string dev;
    std::vector<uint64_t> sizes = {64, 1024, 4096, 16384, 65536, 262144, 1 << 20, 4 << 20};
    int iters = 2000, batch = 16;
    bool user = false;
    uint64_t mb = 64;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--dev" && i + 1 < argc) dev = argv[++i];
        else if (a == "--iters" && i + 1 < argc) iters = atoi(argv[++i]);
        else if (a == "--batch" && i + 1 < argc) batch = atoi(argv[++i]);
        else if (a == "--mb" && i + 1 < argc) mb = strtoull(argv[++i], nullptr, 0);
        else if (a == "--user") user = true;
        else if (a == "--sizes" && i + 1 < argc) {
            sizes.clear();
            for (char *s = argv[++i]; *s;) { sizes.push_back(strtoull(s, &s, 0)); if (*s == ',') s++; }
        } else { dev.clear(); break; }
    }
    if (dev.empty() || batch < 1 || batch > int(PTQ_MAX_BATCH) || iters < 10) {
        printf("usage: %s --dev /dev/ptq_... [--sizes a,b,...] [--iters N >= 10] [--batch B <= %d] [--mb M] [--user]\n", argv[0],
               int(PTQ_MAX_BATCH));
        return 2;
    }
    const uint64_t LEN = mb << 20, FLAG = LEN - 4096;   // the flag word sits in the last page of each buffer
    for (uint64_t s : sizes)
        if (s % 64 || s == 0 || s > FLAG) { printf("size %lu: multiple of 64 and < %lu\n", (unsigned long) s, (unsigned long) FLAG); return 2; }

    ptq::Queue q(dev);
    auto *src = static_cast<uint8_t *>(alloc(LEN)), *dst = static_cast<uint8_t *>(alloc(LEN));
    uint32_t ns = 0, nd = 0;
    const uint32_t hs = q.reg_host(src, LEN, &ns), hd = q.reg_host(dst, LEN, &nd);
    if (user && !q.enable_user()) { printf("FAIL: user mode refused (no translating IOMMU?)\n"); return 1; }
    ptq_info info = q.info();
    printf("ptq_bench %s: ring %u, src %u DMA pieces, dst %u, %d iterations, batch %d, %s\n", dev.c_str(), info.qlen, ns, nd,
           iters, batch, user ? "USER MODE (descriptors + tail from user space)" : "ioctl submit");
    auto *sw = reinterpret_cast<volatile uint64_t *>(src), *dw = reinterpret_cast<volatile uint64_t *>(dst);
    int fails = 0;

    for (uint64_t S : sizes) {
        // lat + check: one copy, then copy + flag
        std::vector<double> one, pair, sub;
        uint64_t late = 0;
        for (int i = 0; i < iters; i++) {
            const uint64_t v = (S << 20) ^ uint64_t(i + 1);
            sw[S / 8 - 1] = v;
            const auto t0 = clk::now();
            const uint64_t s = q.copy(hs, 0, hd, 0, S);
            sub.push_back(us(t0, clk::now()));
            spin_until(q, s);
            const auto t1 = clk::now();
            late += dw[S / 8 - 1] != v;
            one.push_back(us(t0, t1));
        }
        for (int i = 0; i < iters; i++) {
            sw[FLAG / 8] = uint64_t(i + 1);
            ptq_copy c[2] = {{hs, hd, 0, 0, S}, {hs, hd, FLAG, FLAG, 8}};
            const auto t0 = clk::now();
            const uint64_t s = q.submit(c, 2);
            spin_until(q, s);
            pair.push_back(us(t0, clk::now()));
        }
        // bw: B copies per submit, two submits in flight
        const int rounds = std::max(4, int(std::min<uint64_t>(uint64_t(iters), (4ULL << 30) / (S * uint64_t(batch)))));
        std::vector<ptq_copy> cs(batch);
        for (int b = 0; b < batch; b++) cs[b] = {hs, hd, (uint64_t(b) * S) % (FLAG - S), (uint64_t(b) * S) % (FLAG - S), S};
        uint64_t prev = q.seq();
        const auto b0 = clk::now();
        for (int r = 0; r < rounds; r++) {
            const uint64_t s = q.submit(cs.data(), uint32_t(batch));
            spin_until(q, prev);
            prev = s;
        }
        spin_until(q, prev);
        const double t = us(b0, clk::now());
        const double n = double(rounds) * batch;
        printf("size %8lu  submit p50 %5.2f us | one copy p50 %7.2f p99 %7.2f us | copy+flag p50 %7.2f p99 %7.2f us | batched %6.2f GB/s, %6.2f us per copy | head ahead of data %lu\n",
               (unsigned long) S, pct(sub, 50), pct(one, 50), pct(one, 99), pct(pair, 50), pct(pair, 99), n * double(S) / t / 1e3, t / n,
               (unsigned long) late);
        fails += late != 0;
    }

    // order: data + flag back to back, the flag checked against its data by a poller
    {
        const uint64_t S = 1 << 20;
        std::atomic<bool> stop{false};
        std::atomic<uint64_t> bad{0}, seen{0};
        // start from zero: the copy+flag phase left its last flag (iters) in the word, which a poller starting before
        // the first new flag landed took for a flag ahead of its data (2026-10-19 13:54, "2001 flags seen")
        const_cast<uint64_t &>(dw[FLAG / 8]) = 0;
        const_cast<uint64_t &>(dw[0]) = const_cast<uint64_t &>(dw[S / 8 - 1]) = 0;
        std::atomic_thread_fence(std::memory_order_seq_cst);
        std::thread poll([&] {
            uint64_t last = 0;
            while (!stop.load(std::memory_order_relaxed)) {
                const uint64_t f = dw[FLAG / 8];
                if (f != last) {
                    // the data of iteration f (or a later one) must be there: first and last word >= f
                    if (dw[0] < f || dw[S / 8 - 1] < f) bad++;
                    last = f; seen++;
                }
            }
        });
        for (int i = 1; i <= iters; i++) {
            sw[0] = sw[S / 8 - 1] = uint64_t(i);
            sw[FLAG / 8] = uint64_t(i);
            ptq_copy c[2] = {{hs, hd, 0, 0, S}, {hs, hd, FLAG, FLAG, 8}};
            spin_until(q, q.submit(c, 2));
        }
        stop = true;
        poll.join();
        printf("order: %d data+flag pairs, %lu flags seen by the poller, %lu flags ahead of their data\n", iters,
               (unsigned long) seen.load(), (unsigned long) bad.load());
        fails += bad != 0;
    }
    q.unreg(hs);
    q.unreg(hd);
    printf(fails ? "PTQ FAIL\n" : "PTQ PASS\n");
    return fails ? 1 : 0;
}
