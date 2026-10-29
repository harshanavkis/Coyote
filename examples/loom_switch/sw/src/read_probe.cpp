/**
 * read_probe - where the time of a window read goes (2026-10-25): one mode, one size, n transfers,
 * against `get_bench --server` on the far host (window 1 = its export 1, as get_bench's client).
 *
 *     read_probe --client <server_ip> --mode dma|cpu --len BYTES [--port N] [--size BYTES] [--n N]
 *                [--warm N] [--align BYTES] [--gap-us US] [--threads K] [--flag] [--sample]
 *                [--ptq DEV [--ptq-user]] [--window P] [--ra-off] [--ra-dmax N] [--ra-d0 N] [--ra-early]
 *
 *   dma     this host's PTDMA engine copies len bytes from window 1 into a host buffer (get_bench's dma);
 *           the time runs from the submit to the engine's head register (default) or, with --flag, to the
 *           last 8 B of the destination turning non-zero (the flag in the data, as rdma_pingpong --read
 *           polls); --flag then also waits for the head register and reports both
 *   cpu     K threads (default 1) each read len bytes with 32 B streaming loads (get_bench's bulk and
 *           parallel); the time runs to the last thread's end
 *   Every transfer reads a different place, aligned to --align (default 64, as get_bench; 4096 = whole
 *   pages), --gap-us apart (default 50), and every byte is checked.
 *
 *   Card time: the client card's stamps (stamp_rec, CSR 224-240) are armed after the warm-up transfers, so
 *   its rings hold the measured ones; before each transfer the host reads the cycle count (CSR 227) and
 *   prints where the transfer started on the card's time line (cycles since the arm, the read's midpoint
 *   + the time to the submit). `stamps dump` after the run gives the events.
 *   --ra-off: this switch's read-ahead off for the run (RD_CTL bit 8; DESIGN.md 17); --ra-dmax / --ra-d0: the most
 *   pages it fetches ahead / the pages ahead at a stream's start (RD_CTL [21:16] / [29:24]; 0 = the defaults);
 *   --ra-early: streams start on the second consecutive read (RD_CTL bit 9, the first rule).
 *   --local: window 1 is a local window instead: the switch answers every read at once with all ones (checked),
 *   so the time is the reader's own (PTDMA or CPU, PCIe) without the network.
 *   --sample: a thread reads RD_READS, RD_DONE, RD_READS again while a measured transfer runs: reads the
 *   switch has taken and not yet answered (at least / at most).
 *
 * Output: "xfer <i> <us> [<head us>] start_cyc <c>" per transfer, a summary line (median, p10, p90, GB/s),
 * the client's counter deltas over the measured transfers, and the in-flight histogram.
 */
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <immintrin.h>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <coyote/cThread.hpp>
#include "loom_switch.hpp"
#include "ptq.hpp"

namespace {

using namespace loom_switch;
using Clock = std::chrono::steady_clock;

constexpr uint64_t STAGING_SIZE = 1 << 20;
constexpr uint32_t SRC_EXPORT   = 1;
constexpr uint32_t ST_CTL = 224, ST_NOW = 227;
constexpr double   NS_PER_CYC = 4.0;

uint64_t pattern(uint64_t off) { return 0x5EED000000000000ULL ^ (off * 0x9E3779B97F4A7C15ULL); }

__attribute__((target("avx2"))) void copy32(const char *src, char *dst, uint64_t len) {
    for (uint64_t i = 0; i < len; i += 32) {
        const __m256i v = _mm256_stream_load_si256(reinterpret_cast<const __m256i *>(src + i));
        _mm256_storeu_si256(reinterpret_cast<__m256i *>(dst + i), v);
    }
}

bool g_local = false;
uint64_t check(const char *d, uint64_t off, uint64_t len) {
    uint64_t bad = 0;
    const uint64_t *w = reinterpret_cast<const uint64_t *>(d);
    for (uint64_t i = 0; i < len / 8; i++) if (w[i] != (g_local ? ~0ULL : pattern(off + 8 * i))) bad++;
    return bad;
}

double pct(std::vector<double> v, double p) {
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, size_t(p * double(v.size())))];
}

constexpr uint32_t CNT_W[] = {RD_READS, RD_DONE, RD_LINES, RD_SLOT_WAIT, RD_FAILED, GET_SENT, TX_ACKS, TX_WINFULL,
                              TX_REQWAIT, RD_RA_READS, RD_RA_BLOCKS, RD_RA_STARTS, RD_RA_DROP_READ, RD_RA_DROP_STORE,
                              RD_RA_DROP_PKT, RD_RA_DROP_WRITE};
constexpr const char *CNT_N[] = {"reads", "answered", "lines", "slot-wait cycles", "failed", "gets sent", "acks",
                                 "window-wait", "sq-wait", "read-ahead reads", "blocks", "streams", "drops by a read",
                                 "by a store", "by a packet", "by a write"};
constexpr int N_CNT = sizeof(CNT_W) / sizeof(CNT_W[0]);

struct Args {
    std::string ip, mode, ptq_dev;
    uint16_t port = coyote::DEF_PORT;
    uint64_t size = 16ULL << 20, len = 0, align = 64;
    int n = 20, warm = 3, threads = 1, window = -1;
    double gap_us = 50;
    bool flag = false, sample = false, ptq_user = false, local = false, ra_off = false;
    unsigned ra_dmax = 0, ra_d0 = 0;
    bool ra_early = false;
};

int run(const Args &a) {
    coyote::cThread t_qp(0, getpid(), 0, nullptr, "loom_fpga");
    printf("client: QP exchange with %s on port %u ...\n", a.ip.c_str(), a.port);
    if (!t_qp.initRDMA(STAGING_SIZE, a.port, a.ip.c_str())) { printf("FAIL: initRDMA\n"); return 1; }
    if (a.window >= 0) csr_write(t_qp, TX_CTL, uint64_t(a.window));
    printf("client: ack window %lu packets\n", (unsigned long) csr_read(t_qp, TX_CTL));
    const uint64_t rd_ctl = csr_read(t_qp, RD_CTL);
    if (a.ra_off || a.ra_dmax || a.ra_d0 || a.ra_early)
        csr_write(t_qp, RD_CTL, rd_ctl_ra(rd_ctl, a.ra_off, a.ra_dmax, a.ra_d0) | (a.ra_early ? RD_CTL_RA_EARLY : 0));
    const uint64_t rc = csr_read(t_qp, RD_CTL);
    printf("client: read-ahead %s, at most %lu pages ahead, %lu at the start (0 = default), streams start %s\n",
           (rc & RD_CTL_RA_OFF) ? "off" : "on", (unsigned long) ((rc >> 16) & 0x3F), (unsigned long) ((rc >> 24) & 0x3F),
           (rc & RD_CTL_RA_EARLY) ? "on the second read" : "entering a page after a page");
    // --local: window 1 is a local window, whose reads the switch answers at once with all ones (the read path's
    // ceiling without the network)
    program_window(t_qp, 1, !a.local, t_qp.getCtid(), reinterpret_cast<const void *>(export_ref(SRC_EXPORT)), a.size, 0);
    const char *win = static_cast<const char *>(t_qp.mapUwin(a.size));
    t_qp.connSync(true);       // the server is ready

    const uint64_t len = a.len;
    uint64_t next = 0;
    auto pick = [&]() {        // a different place each transfer
        const uint64_t s = (next % (a.size - len + 1)) & ~(a.align - 1);
        next += 4096 + std::max<uint64_t>(64, a.align);
        return s;
    };

    // dma: the engine, a destination buffer
    const bool dma = a.mode == "dma";
    coyote::cThread *t_dst = nullptr;
    char *dst = nullptr;
    ptq::Queue *q = nullptr;
    uint32_t hd = 0, hu = 0;
    std::vector<std::vector<char>> cbuf;
    if (dma) {
        t_dst = new coyote::cThread(0, getpid(), 0, nullptr, "loom_fpga");
        dst = static_cast<char *>(t_dst->getMem({coyote::CoyoteAllocType::HPF, std::max<uint64_t>(len, 1 << 21)}));
        if (!dst) { printf("FAIL: no host memory\n"); return 1; }
        q = new ptq::Queue(a.ptq_dev);
        hd = q->reg_host(dst, std::max<uint64_t>(len, 1 << 21));
        uint64_t ulen = 0;
        hu = q->reg_uwin(&ulen);
        if (a.ptq_user && !q->enable_user()) { printf("FAIL: PTDMA user mode refused\n"); return 1; }
        if (ulen < a.size) { printf("FAIL: the uwin (%lu B) is smaller than window 1\n", (unsigned long) ulen); return 1; }
    } else {
        cbuf.assign(a.threads, std::vector<char>(len + 64));
    }
    printf("mode %s, %lu B, %d threads, align %lu, %d warm + %d transfers, %.0f us apart%s%s\n", a.mode.c_str(),
           (unsigned long) len, dma ? 1 : a.threads, (unsigned long) a.align, a.warm, a.n, a.gap_us,
           a.flag ? ", flag in the data" : "", a.sample ? ", sampling reads in flight" : "");

    // in flight: reads taken, not answered
    std::atomic<bool> stop{false}, active{false};
    std::map<int64_t, uint64_t> hlo, hhi;
    std::thread sampler;
    if (a.sample) {
        sampler = std::thread([&] {
            while (!stop.load()) {
                if (!active.load()) { std::this_thread::yield(); continue; }
                const int64_t r1 = int64_t(csr_read(t_qp, RD_READS));
                const int64_t d  = int64_t(csr_read(t_qp, RD_DONE));
                const int64_t r2 = int64_t(csr_read(t_qp, RD_READS));
                if (!active.load()) continue;
                hlo[std::max<int64_t>(0, r1 - d)]++;
                hhi[r2 - d]++;
            }
        });
    }

    uint64_t c0[N_CNT], c1[N_CNT];
    std::vector<double> us, us_head;
    uint64_t bad = 0, missed_flag = 0;
    int errors = 0;
    for (int r = 0; r < a.warm + a.n; r++) {
        const bool meas = r >= a.warm;
        if (r == a.warm) {
            csr_write(t_qp, ST_CTL, 1);                  // stamps: record from here on
            (void) csr_read(t_qp, ST_CTL);
            for (int i = 0; i < N_CNT; i++) c0[i] = csr_read(t_qp, CNT_W[i]);
        }
        const uint64_t off = pick();
        std::vector<uint64_t> offs;
        if (dma) memset(dst, 0, len);
        else for (int k = 0; k < a.threads; k++) offs.push_back(k == 0 ? off : pick());
        // where this transfer starts on the card's time line
        const auto ta = Clock::now();
        const uint64_t now = csr_read(t_qp, ST_NOW) & 0xFFFFFFFFULL;
        const auto tb = Clock::now();
        _mm_mfence();
        active = meas && a.sample;
        const auto t0 = Clock::now();
        double t_us = 0, h_us = 0;
        if (dma) {
            const uint64_t sq = q->copy(hu, off, hd, 0, len);
            if (a.flag) {
                volatile const uint64_t *f = reinterpret_cast<volatile const uint64_t *>(dst + len - 8);
                while (*f == 0 && Clock::now() - t0 < std::chrono::milliseconds(20)) _mm_pause();
                const auto t1 = Clock::now();
                if (*f == 0) missed_flag++;
                while (!q->done(sq) && Clock::now() - t0 < std::chrono::seconds(5)) _mm_pause();
                const auto t2 = Clock::now();
                t_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
                h_us = std::chrono::duration<double, std::micro>(t2 - t0).count();
            } else {
                while (!q->done(sq) && Clock::now() - t0 < std::chrono::seconds(5)) _mm_pause();
                t_us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
            }
            if (!q->done(sq)) { printf("FAIL: the engine did not complete within 5 s\n"); errors++; break; }
            bad += check(dst, off, len);
        } else if (a.threads == 1) {
            copy32(win + off, cbuf[0].data(), len);
            _mm_lfence();
            t_us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
            bad += check(cbuf[0].data(), off, len);
        } else {
            std::vector<std::thread> th;
            for (int k = 0; k < a.threads; k++) th.emplace_back([&, k] { copy32(win + offs[k], cbuf[k].data(), len); });
            for (auto &x : th) x.join();
            t_us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
            for (int k = 0; k < a.threads; k++) bad += check(cbuf[k].data(), offs[k], len);
        }
        active = false;
        if (meas) {
            const double mid_to_t0 = std::chrono::duration<double, std::nano>(t0 - (ta + (tb - ta) / 2)).count();
            const double rd_ns = std::chrono::duration<double, std::nano>(tb - ta).count();
            us.push_back(t_us);
            if (a.flag) us_head.push_back(h_us);
            printf("xfer %d %.3f", r - a.warm, t_us);
            if (a.flag) printf(" head %.3f", h_us);
            printf(" start_cyc %.0f csr_rd_ns %.0f off %lu\n", double(now) + mid_to_t0 / NS_PER_CYC, rd_ns,
                   (unsigned long) off);
        }
        const auto tg = Clock::now();
        while (std::chrono::duration<double, std::micro>(Clock::now() - tg).count() < a.gap_us) _mm_pause();
    }
    active = false;
    for (int i = 0; i < N_CNT; i++) c1[i] = csr_read(t_qp, CNT_W[i]);
    stop = true;
    if (sampler.joinable()) sampler.join();

    if (!us.empty()) {
        const double m = pct(us, 0.5);
        const double bytes = double(len) * (dma ? 1 : a.threads);
        printf("%s %s %lu B x%d: median %.3f us, p10 %.3f, p90 %.3f, %.3f GB/s", bad || missed_flag ? "FAIL" : "ok  ",
               a.mode.c_str(), (unsigned long) len, dma ? 1 : a.threads, m, pct(us, 0.1), pct(us, 0.9), bytes / m / 1e3);
        if (a.flag) printf("; head register: median %.3f us (flag missed %lu)", pct(us_head, 0.5), (unsigned long) missed_flag);
        printf("%s\n", bad ? " (data wrong)" : "");
        if (bad || missed_flag) errors++;
    }
    printf("counters (%d transfers):", a.n);
    for (int i = 0; i < N_CNT; i++) printf("%s %s %lu", i ? "," : "", CNT_N[i], (unsigned long) (c1[i] - c0[i]));
    printf("\n");
    if (a.sample) {
        uint64_t n = 0;
        for (auto &e : hhi) n += e.second;
        auto show = [&](const char *what, std::map<int64_t, uint64_t> &h) {
            double s = 0; int64_t mx = 0; uint64_t c = 0; int64_t p50 = 0, p90 = 0;
            for (auto &e : h) { s += double(e.first) * double(e.second); mx = std::max(mx, e.first); }
            for (auto &e : h) { c += e.second; if (!p50 && c * 2 >= n) p50 = e.first; if (!p90 && c * 10 >= n * 9) p90 = e.first; }
            printf("in flight (%s): %lu samples, mean %.1f, p50 %ld, p90 %ld, max %ld reads\n", what, (unsigned long) n,
                   n ? s / double(n) : 0.0, (long) p50, (long) p90, (long) mx);
        };
        show("at least", hlo);
        show("at most", hhi);
        printf("in flight histogram (at most): ");
        for (auto &e : hhi) printf("%ld:%lu ", (long) e.first, (unsigned long) e.second);
        printf("\n");
    }

    if (a.ra_off || a.ra_dmax || a.ra_d0 || a.ra_early) csr_write(t_qp, RD_CTL, rd_ctl);
    t_qp.connSync(true);       // done
    t_qp.unmapUwin();
    release_window(t_qp, 1);
    printf(errors ? "READ PROBE FAIL\n" : "READ PROBE PASS\n");
    return errors ? 1 : 0;
}

} // namespace

int main(int argc, char *argv[]) {
    Args a;
    for (int i = 1; i < argc; i++) {
        const std::string s = argv[i];
        if (s == "--client" && i + 1 < argc)        a.ip = argv[++i];
        else if (s == "--mode" && i + 1 < argc)     a.mode = argv[++i];
        else if (s == "--len" && i + 1 < argc)      a.len = strtoull(argv[++i], nullptr, 0);
        else if (s == "--port" && i + 1 < argc)     a.port = uint16_t(atoi(argv[++i]));
        else if (s == "--size" && i + 1 < argc)     a.size = strtoull(argv[++i], nullptr, 0);
        else if (s == "--n" && i + 1 < argc)        a.n = atoi(argv[++i]);
        else if (s == "--warm" && i + 1 < argc)     a.warm = atoi(argv[++i]);
        else if (s == "--align" && i + 1 < argc)    a.align = strtoull(argv[++i], nullptr, 0);
        else if (s == "--gap-us" && i + 1 < argc)   a.gap_us = atof(argv[++i]);
        else if (s == "--threads" && i + 1 < argc)  a.threads = atoi(argv[++i]);
        else if (s == "--window" && i + 1 < argc)   a.window = atoi(argv[++i]);
        else if (s == "--ptq" && i + 1 < argc)      a.ptq_dev = argv[++i];
        else if (s == "--ptq-user")                 a.ptq_user = true;
        else if (s == "--flag")                     a.flag = true;
        else if (s == "--sample")                   a.sample = true;
        else if (s == "--local")                    a.local = g_local = true;
        else if (s == "--ra-off")                   a.ra_off = true;
        else if (s == "--ra-dmax" && i + 1 < argc)  a.ra_dmax = unsigned(atoi(argv[++i]));
        else if (s == "--ra-d0" && i + 1 < argc)    a.ra_d0 = unsigned(atoi(argv[++i]));
        else if (s == "--ra-early")                 a.ra_early = true;
        else { fprintf(stderr, "usage: see the header of read_probe.cpp\n"); return 2; }
    }
    const bool ok = !a.ip.empty() && (a.mode == "cpu" || (a.mode == "dma" && !a.ptq_dev.empty())) && a.len >= 64 &&
                    a.len % 64 == 0 && a.len <= a.size && a.size % 4096 == 0 && a.n >= 1 && a.threads >= 1 &&
                    a.align >= 64 && (a.align & (a.align - 1)) == 0 && a.window <= 255 && !(a.flag && a.mode != "dma") &&
                    !(a.flag && a.local);
    if (!ok) { fprintf(stderr, "usage: see the header of read_probe.cpp\n"); return 2; }
    return run(a);
}
