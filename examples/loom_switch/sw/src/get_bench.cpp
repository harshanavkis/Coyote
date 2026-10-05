/**
 * get_bench - gets through the switch, between two hosts' U280s on the
 * loom_switch bitstream: the client CPU READS a window bound to the server's
 * export, and the switch does the rest:
 *
 *   client CPU load --> client U280 uwin read (loom_read) --get request-->
 *   server loom_rx / loom_rd: read the export, write the lines back -->
 *   client loom_rx --> loom_read --> the load's data
 *
 * Server (the host read from):
 *     get_bench --server [--port N] [--size BYTES] [--window P]
 *   QP exchange (blocks for the client); a source buffer of size bytes with a
 *   known pattern, exported as export 1; answers gets on the QP owner's QP
 *   (RD_CTL); waits for the client, then prints its responder's counters.
 *
 * Client (the reader):
 *     get_bench --client <server_ip> [--port N] [--size BYTES] [--reps N] [--window P]
 *   window 1 onto the server's export 1 (uwin 0 .. size), window 2 just past
 *   its end, then
 *     latency    one 8 B load, and one 32 B load, at a time, reps each:
 *                median, p99, min
 *     bulk       64 B .. 1 MiB (at most size) read with 32 B streaming loads,
 *                one transfer at a time: time, GB/s, and the reads the
 *                switch saw per transfer
 *     parallel   k = 1 .. 32 threads at once, each reading 64 KiB that way
 *                (a core has one load in flight, so k reads in flight)
 *     error      a load from window 2: the server rejects the get, the load
 *                returns all ones
 *   Every load reads a different line and every byte is checked against the
 *   pattern. Both sides use the same --size (default 16 MiB).
 *
 * The QP port is N (Coyote's default if not given). --window sets this
 * host's ack window (TX_CTL: rdma packets unacked, 0 = no limit; 16 after
 * reset): the server's carries the answers, two packets per get.
 */
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <immintrin.h>
#include <string>
#include <thread>
#include <vector>

#include <coyote/cThread.hpp>
#include "loom_switch.hpp"

namespace {

using namespace loom_switch;
using Clock = std::chrono::steady_clock;

constexpr uint64_t STAGING_SIZE = 1 << 20;
constexpr uint32_t SRC_EXPORT   = 1;     // on the server

uint64_t pattern(uint64_t off) { return 0x5EED000000000000ULL ^ (off * 0x9E3779B97F4A7C15ULL); }

struct Counters {
    int n;
    const uint32_t *w;
    const char *const *name;
    uint64_t v[8];
    void read(coyote::cThread &t) { for (int i = 0; i < n; i++) v[i] = csr_read(t, w[i]); }
    void print_delta(const char *who, const Counters &a) const {
        printf("%s", who);
        for (int i = 0; i < n; i++) printf("%s %s %lu", i ? "," : "", name[i], (unsigned long) (v[i] - a.v[i]));
        printf("\n");
    }
};
constexpr uint32_t RESP_W[] = {GET_JOBS, GET_ERRS, GET_PKTS, GET_CMPS, GET_WAIT, GET_STARVE};
constexpr const char *RESP_N[] = {"requests", "errors", "packets", "completions",
                                  "cycles waiting to send", "cycles waiting for data"};
constexpr uint32_t READ_W[] = {RD_READS, RD_DONE, RD_FAILED, RD_FAR_ERR, RD_SLOT_WAIT, RD_STRAY, RD_LINES};
constexpr const char *READ_N[] = {"reads", "answered", "failed", "rejected by the server",
                                  "cycles waiting for a slot", "stray completions", "lines"};

// A 32 B load from the window (a streaming load: the way to read
// write-combining memory)
__attribute__((target("avx2"))) inline void load32(const char *src, char *dst) {
    const __m256i v = _mm256_stream_load_si256(reinterpret_cast<const __m256i *>(src));
    _mm256_storeu_si256(reinterpret_cast<__m256i *>(dst), v);
}
__attribute__((target("avx2"))) void copy32(const char *src, char *dst, uint64_t len) {
    for (uint64_t i = 0; i < len; i += 32) load32(src + i, dst + i);
}

uint64_t check(const char *d, uint64_t off, uint64_t len) {
    uint64_t bad = 0;
    const uint64_t *w = reinterpret_cast<const uint64_t *>(d);
    for (uint64_t i = 0; i < len / 8; i++) if (w[i] != pattern(off + 8 * i)) bad++;
    return bad;
}

double median(std::vector<double> v) { std::sort(v.begin(), v.end()); return v[v.size() / 2]; }

int run_server(uint16_t port, uint64_t size, int window) {
    coyote::cThread t_qp(0, getpid(), 0, nullptr, "coyote_fpga");     // QP owner, CSR page
    coyote::cThread t_data(0, getpid(), 0, nullptr, "coyote_fpga");   // owns the source
    printf("server: QP exchange on port %u ...\n", port);
    if (!t_qp.initRDMA(STAGING_SIZE, port)) { printf("FAIL: initRDMA\n"); return 1; }
    if (window >= 0) csr_write(t_qp, TX_CTL, uint64_t(window));
    printf("server: ack window %lu packets\n", (unsigned long) csr_read(t_qp, TX_CTL));

    uint64_t *src = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, size}));
    for (uint64_t i = 0; i < size / 8; i++) src[i] = pattern(8 * i);
    program_export(t_qp, SRC_EXPORT, t_data.getCtid(), src, size);
    set_response_qp(t_qp, t_qp.getCtid());
    Counters r0{6, RESP_W, RESP_N, {}}, r1 = r0;
    r0.read(t_qp);
    printf("server: %lu bytes exported as %u, answering gets on QP owner %d\n",
           (unsigned long) size, SRC_EXPORT, t_qp.getCtid());

    t_qp.connSync(false);      // the client may start
    t_qp.connSync(false);      // the client is done
    r1.read(t_qp);
    r1.print_delta("server: responder:", r0);
    release_export(t_qp, SRC_EXPORT);
    printf("SERVER DONE\n");
    return 0;
}

int run_client(const std::string &ip, uint16_t port, uint64_t size, int reps, int window) {
    coyote::cThread t_qp(0, getpid(), 0, nullptr, "coyote_fpga");
    printf("client: QP exchange with %s on port %u ...\n", ip.c_str(), port);
    if (!t_qp.initRDMA(STAGING_SIZE, port, ip.c_str())) { printf("FAIL: initRDMA\n"); return 1; }
    if (window >= 0) csr_write(t_qp, TX_CTL, uint64_t(window));
    printf("client: ack window %lu packets\n", (unsigned long) csr_read(t_qp, TX_CTL));

    // Window 1: the server's export 1; window 2: past its end
    program_window(t_qp, 1, true, t_qp.getCtid(), reinterpret_cast<const void *>(export_ref(SRC_EXPORT)), size, 0);
    program_window(t_qp, 2, true, t_qp.getCtid(), reinterpret_cast<const void *>(export_ref(SRC_EXPORT, size)),
                   4096, size);
    const char *win = static_cast<const char *>(t_qp.mapUwin(size + 4096));
    Counters c0{7, READ_W, READ_N, {}}, c1 = c0;
    c0.read(t_qp);
    t_qp.connSync(true);       // the server is ready

    int errors = 0;
    uint64_t next = 0;
    auto pick = [&](uint64_t len) {       // a different line each time
        const uint64_t s = (next % (size - len + 64)) & ~63ULL;
        next += 4096 + 64;
        return s;
    };
    alignas(64) static char buf[1 << 20];

    // --- latency: one load at a time ---
    printf("latency (one load at a time, %d reps; us from issue to data):\n", reps);
    for (int width : {8, 32}) {
        std::vector<double> us;
        uint64_t bad = 0;
        for (int r = 0; r < reps; r++) {
            const uint64_t off = pick(64) + uint64_t(width) * (r % (64 / width));
            _mm_lfence();
            const auto t0 = Clock::now();
            if (width == 8) {
                *reinterpret_cast<uint64_t *>(buf) = *reinterpret_cast<const volatile uint64_t *>(win + off);
            } else {
                load32(win + off, buf);
            }
            _mm_lfence();
            const auto t1 = Clock::now();
            us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
            bad += check(buf, off, width);
        }
        std::sort(us.begin(), us.end());
        printf("  %s %2d B load: median %7.2f us, p99 %7.2f us, min %7.2f us%s\n", bad ? "FAIL" : "ok  ", width,
               us[us.size() / 2], us[std::min(us.size() - 1, us.size() * 99 / 100)], us.front(),
               bad ? " (data wrong)" : "");
        if (bad) errors++;
    }

    // --- bulk: one transfer at a time, 32 B streaming loads ---
    printf("bulk (32 B streaming loads, one transfer at a time):\n");
    for (uint64_t len = 64; len <= (1ULL << 20) && len <= size; len *= 4) {
        const int n = len <= 4096 ? std::max(reps / 10, 3) : 5;
        std::vector<double> us;
        uint64_t bad = 0;
        const uint64_t g0 = csr_read(t_qp, RD_READS);
        for (int r = 0; r < n; r++) {
            const uint64_t off = pick(len);
            _mm_lfence();
            const auto t0 = Clock::now();
            copy32(win + off, buf, len);
            _mm_lfence();
            const auto t1 = Clock::now();
            us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
            bad += check(buf, off, len);
        }
        const uint64_t g1 = csr_read(t_qp, RD_READS);
        const double m = median(us);
        printf("  %s %7lu B: median %9.2f us, %6.3f GB/s, %.1f reads per transfer%s\n", bad ? "FAIL" : "ok  ",
               (unsigned long) len, m, len / m / 1e3, double(g1 - g0) / n, bad ? " (data wrong)" : "");
        if (bad) errors++;
    }

    // --- parallel: k threads loading at once ---
    printf("parallel (k threads at once, each 64 KiB with 32 B streaming loads):\n");
    for (int k = 1; k <= 32; k *= 2) {
        constexpr uint64_t len = 64 << 10;
        std::vector<uint64_t> off(k);
        std::vector<std::vector<char>> dst(k, std::vector<char>(len));
        for (auto &o : off) o = pick(len);
        const uint64_t w0 = csr_read(t_qp, RD_SLOT_WAIT);
        std::vector<std::thread> th;
        const auto t0 = Clock::now();
        for (int i = 0; i < k; i++) th.emplace_back([&, i] { copy32(win + off[i], dst[i].data(), len); });
        for (auto &x : th) x.join();
        const auto t1 = Clock::now();
        const uint64_t w1 = csr_read(t_qp, RD_SLOT_WAIT);
        uint64_t bad = 0;
        for (int i = 0; i < k; i++) bad += check(dst[i].data(), off[i], len);
        const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        printf("  %s %2d threads: %9.1f us, %6.3f GB/s, %lu cycles a read waited for a slot%s\n", bad ? "FAIL" : "ok  ",
               k, us, k * len / us / 1e3, (unsigned long) (w1 - w0), bad ? " (data wrong)" : "");
        if (bad) errors++;
    }

    // --- error: past the server's export ---
    {
        const uint64_t f0 = csr_read(t_qp, RD_FAR_ERR);
        const uint64_t v = *reinterpret_cast<const volatile uint64_t *>(win + size);
        const uint64_t f1 = csr_read(t_qp, RD_FAR_ERR);
        const bool ok = v == ~0ULL && f1 - f0 == 1;
        printf("%s error: a load past the server's export returned %016lx, %lu rejected\n", ok ? "ok  " : "FAIL",
               (unsigned long) v, (unsigned long) (f1 - f0));
        if (!ok) errors++;
    }

    c1.read(t_qp);
    c1.print_delta("client: reader:", c0);
    t_qp.connSync(true);       // done
    t_qp.unmapUwin();
    release_window(t_qp, 1);
    release_window(t_qp, 2);
    printf(errors ? "GET BENCH FAIL\n" : "GET BENCH PASS\n");
    return errors ? 1 : 0;
}

} // namespace

int main(int argc, char *argv[]) {
    bool server = false;
    std::string ip;
    uint16_t port = coyote::DEF_PORT;
    uint64_t size = 16ULL << 20;
    int reps = 1000;
    int window = -1;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--server")                     server = true;
        else if (a == "--client" && i + 1 < argc) ip = argv[++i];
        else if (a == "--port" && i + 1 < argc)  port = uint16_t(atoi(argv[++i]));
        else if (a == "--size" && i + 1 < argc)  size = strtoull(argv[++i], nullptr, 0);
        else if (a == "--reps" && i + 1 < argc)  reps = atoi(argv[++i]);
        else if (a == "--window" && i + 1 < argc) window = atoi(argv[++i]);
        else { fprintf(stderr, "usage: see the header of get_bench.cpp\n"); return 2; }
    }
    if (server == !ip.empty() || size < (64 << 10) || size % 4096 || reps < 1 || window > 255) {
        fprintf(stderr, "usage: see the header of get_bench.cpp (--size: a multiple of 4096, at least 64 KiB)\n");
        return 2;
    }
    return server ? run_server(port, size, window) : run_client(ip, port, size, reps, window);
}
