/**
 * get_bench - gets (a read as two writes) through the switch, between two
 * hosts' U280s on the loom_switch bitstream:
 *
 *   client CPU --8 B store--> client U280 get window --get request--> server
 *   loom_rx / loom_rd: read the server's export, write it back --> client
 *   loom_rx --> client return buffer, then the completion word after it
 *
 * Server (the host read from):
 *     get_bench --server [--port N] [--size BYTES]
 *   QP exchange (blocks for the client); a source buffer of size bytes with a
 *   known pattern, exported as export 1; answers gets on the QP owner's QP
 *   (RD_CTL); waits for the client, then prints its responder's counters.
 *
 * Client (the reader):
 *     get_bench --client <server_ip> [--port N] [--size BYTES] [--reps N]
 *   get window 1 onto the server's export 1 (uwin 0 .. size), a return
 *   buffer exported here as export 3, then
 *     latency    one get at a time, 64 B .. 64 KiB, reps each: median, p99
 *     bandwidth  one get of 64 KiB .. 4 MiB - 64 (at most size)
 *     pipelined  k gets of 256 KiB in flight, k = 1 .. 16
 *     error      a get past the end of the server's export: the completion
 *                must be all ones and no data written
 *   Each get reads a different source offset and every byte is checked
 *   against the pattern; a get is timed from the store to its completion
 *   word. Both sides use the same --size (default 16 MiB).
 *
 * The QP port is N (Coyote's default if not given).
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
#include <vector>

#include <coyote/cThread.hpp>
#include "loom_switch.hpp"

namespace {

using namespace loom_switch;
using Clock = std::chrono::steady_clock;

constexpr uint64_t STAGING_SIZE = 1 << 20;
constexpr uint32_t SRC_EXPORT   = 1;     // on the server
constexpr uint32_t RET_EXPORT   = 3;     // on the client
constexpr uint64_t SLOT         = 256 << 10;
constexpr int      MAX_INFLIGHT = 16;

uint64_t pattern(uint64_t off) { return 0x5EED000000000000ULL ^ (off * 0x9E3779B97F4A7C15ULL); }

struct Responder {
    static constexpr int N = 6;
    static constexpr uint32_t W[N] = {GET_JOBS, GET_ERRS, GET_PKTS, GET_CMPS, GET_WAIT, GET_STARVE};
    static constexpr const char *NAME[N] = {"requests", "errors", "packets", "completions",
                                            "cycles waiting to send", "cycles waiting for data"};
    uint64_t v[N];
    static Responder read(coyote::cThread &t) {
        Responder r;
        for (int i = 0; i < N; i++) r.v[i] = csr_read(t, W[i]);
        return r;
    }
};

int run_server(uint16_t port, uint64_t size) {
    coyote::cThread t_qp(0, getpid(), 0, nullptr, "coyote_fpga");     // QP owner, CSR page
    coyote::cThread t_data(0, getpid(), 0, nullptr, "coyote_fpga");   // owns the source
    printf("server: QP exchange on port %u ...\n", port);
    if (!t_qp.initRDMA(STAGING_SIZE, port)) { printf("FAIL: initRDMA\n"); return 1; }

    uint64_t *src = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, size}));
    for (uint64_t i = 0; i < size / 8; i++) src[i] = pattern(8 * i);
    program_export(t_qp, SRC_EXPORT, t_data.getCtid(), src, size);
    set_response_qp(t_qp, t_qp.getCtid());
    const Responder r0 = Responder::read(t_qp);
    printf("server: %lu bytes exported as %u, answering gets on QP owner %d\n",
           (unsigned long) size, SRC_EXPORT, t_qp.getCtid());

    t_qp.connSync(false);      // the client may start
    t_qp.connSync(false);      // the client is done
    const Responder r1 = Responder::read(t_qp);
    printf("server: responder");
    for (int i = 0; i < Responder::N; i++) printf(", %s %lu", Responder::NAME[i], (unsigned long) (r1.v[i] - r0.v[i]));
    printf("\n");
    release_export(t_qp, SRC_EXPORT);
    printf("SERVER DONE\n");
    return 0;
}

struct Client {
    coyote::cThread &t;
    uint64_t *ret;          // the return buffer (export RET_EXPORT)
    uint64_t size;          // the server's source size
    uint64_t next_src = 0;  // a different source offset per get

    // One get of len bytes into slot k; returns false on a bad completion
    uint64_t pick_src(uint64_t len) {
        const uint64_t span = size - len;
        const uint64_t s = span ? (next_src % (span + 64)) & ~63ULL : 0;
        next_src += 4096 + 64;
        return s;
    }
    volatile uint64_t *cmp(uint64_t k, uint64_t len) { return ret + (k * (SLOT + 4096) + len) / 8; }
    void post(uint64_t k, uint64_t src, uint64_t len) {
        const uint64_t w = get_word(len, export_ref(RET_EXPORT, k * (SLOT + 4096)));
        t.uwinWrite(src, &w, 8);
    }
    bool wait(uint64_t k, uint64_t len, Clock::time_point deadline) {
        const uint64_t w = get_word(len, export_ref(RET_EXPORT, k * (SLOT + 4096)));
        while (*cmp(k, len) != w) {
            if (*cmp(k, len) == GET_ERROR || Clock::now() > deadline) return false;
            _mm_pause();
        }
        return true;
    }
    uint64_t check(uint64_t k, uint64_t src, uint64_t len) {
        uint64_t bad = 0;
        const uint64_t *d = ret + k * (SLOT + 4096) / 8;
        for (uint64_t i = 0; i < len / 8; i++) if (d[i] != pattern(src + 8 * i)) bad++;
        return bad;
    }
};

int run_client(const std::string &ip, uint16_t port, uint64_t size, int reps) {
    coyote::cThread t_qp(0, getpid(), 0, nullptr, "coyote_fpga");
    coyote::cThread t_data(0, getpid(), 0, nullptr, "coyote_fpga");
    printf("client: QP exchange with %s on port %u ...\n", ip.c_str(), port);
    if (!t_qp.initRDMA(STAGING_SIZE, port, ip.c_str())) { printf("FAIL: initRDMA\n"); return 1; }

    const uint64_t ret_len = MAX_INFLIGHT * (SLOT + 4096) + (4ULL << 20) + 4096;
    uint64_t *ret = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, ret_len}));
    memset(ret, 0, ret_len);
    program_export(t_qp, RET_EXPORT, t_data.getCtid(), ret, ret_len);
    program_get_window(t_qp, 1, t_qp.getCtid(), export_ref(SRC_EXPORT), size, 0);
    t_qp.mapUwin(size);
    const uint64_t g0 = csr_read(t_qp, GET_SENT);
    t_qp.connSync(true);       // the server is ready

    Client c{t_qp, ret, size};
    const auto patience = std::chrono::seconds(1);
    int errors = 0;

    // --- latency: one get at a time ---
    printf("latency (one get at a time, %d reps; us from the store to the completion):\n", reps);
    for (uint64_t len = 64; len <= (64 << 10) && len <= size; len *= 4) {
        std::vector<double> us;
        uint64_t bad = 0, prev_src = 0;
        int bad_reps = 0, fast_reps = 0;
        for (int r = 0; r < reps; r++) {
            const uint64_t src = c.pick_src(len);
            *c.cmp(0, len) = 0;
            _mm_sfence();
            const auto t0 = Clock::now();
            c.post(0, src, len);
            const bool ok = c.wait(0, len, t0 + patience);
            const auto t1 = Clock::now();
            if (!ok) { printf("FAIL latency %lu B rep %d: completion %016lx\n", (unsigned long) len, r,
                              (unsigned long) *c.cmp(0, len)); errors++; break; }
            const double t = std::chrono::duration<double, std::micro>(t1 - t0).count();
            const uint64_t b = c.check(0, src, len);
            if (t < 2.0) fast_reps++;
            if (b) {
                // which data is there: the previous get's, and does this one's arrive late?
                const uint64_t stale = c.check(0, prev_src, len);
                usleep(200);
                const uint64_t later = c.check(0, src, len);
                if (!bad_reps)
                    printf("  first bad: rep %d, %.2f us, %lu/%lu words wrong, %lu differ from the previous "
                           "get's data, %lu still wrong 200 us later\n", r, t, (unsigned long) b,
                           (unsigned long) (len / 8), (unsigned long) stale, (unsigned long) later);
                bad_reps++;
            }
            bad += b;
            prev_src = src;
            us.push_back(t);
        }
        if (us.empty()) continue;
        std::sort(us.begin(), us.end());
        const double med = us[us.size() / 2], p99 = us[std::min(us.size() - 1, us.size() * 99 / 100)];
        printf("  %s %7lu B: median %7.2f us, p99 %7.2f us, min %7.2f us%s\n", bad ? "FAIL" : "ok  ",
               (unsigned long) len, med, p99, us.front(), bad ? " (data wrong)" : "");
        if (bad || fast_reps)
            printf("           %d of %d reps with wrong data, %d completed in under 2 us\n", bad_reps, reps, fast_reps);
        if (bad) errors++;
    }

    // --- bandwidth: one large get ---
    printf("bandwidth (one get):\n");
    for (uint64_t len = 64 << 10; len <= size; len *= 4) {
        const uint64_t l = std::min<uint64_t>(len, (4ULL << 20) - 64);
        const uint64_t src = c.pick_src(l);
        memset(ret, 0, l + 64);
        _mm_sfence();
        const auto t0 = Clock::now();
        c.post(0, src, l);
        const bool ok = c.wait(0, l, t0 + patience);
        const auto t1 = Clock::now();
        const uint64_t bad = ok ? c.check(0, src, l) : l / 8;
        const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        printf("  %s %7lu B: %8.1f us, %5.2f GB/s%s\n", (ok && !bad) ? "ok  " : "FAIL", (unsigned long) l, us,
               l / us / 1e3, ok ? (bad ? " (data wrong)" : "") : " (no completion)");
        if (!ok || bad) errors++;
        if (l != len) break;
    }

    // --- pipelined: k gets of SLOT bytes in flight ---
    printf("pipelined (gets of %lu KiB in flight, %d rounds each):\n", (unsigned long) (SLOT >> 10), reps / 10 + 1);
    for (int k = 1; k <= MAX_INFLIGHT; k *= 2) {
        const int rounds = reps / 10 + 1;
        uint64_t bad = 0, bytes = 0;
        bool ok = true;
        double us = 0;
        for (int r = 0; r < rounds && ok; r++) {
            std::vector<uint64_t> src(k);
            for (int i = 0; i < k; i++) { src[i] = c.pick_src(SLOT); *c.cmp(i, SLOT) = 0; }
            _mm_sfence();
            const auto t0 = Clock::now();
            for (int i = 0; i < k; i++) c.post(i, src[i], SLOT);
            for (int i = 0; i < k && ok; i++) ok = c.wait(i, SLOT, t0 + patience);
            const auto t1 = Clock::now();
            us += std::chrono::duration<double, std::micro>(t1 - t0).count();
            for (int i = 0; i < k && ok; i++) bad += c.check(i, src[i], SLOT);
            bytes += k * SLOT;
        }
        printf("  %s %2d in flight: %5.2f GB/s%s\n", (ok && !bad) ? "ok  " : "FAIL", k, bytes / us / 1e3,
               ok ? (bad ? " (data wrong)" : "") : " (no completion)");
        if (!ok || bad) errors++;
    }

    // --- error: past the end of the server's export ---
    {
        const uint64_t len = 128;
        *c.cmp(0, len) = 0;
        ret[0] = 0x1234;
        _mm_sfence();
        c.post(0, size - 64, len);
        const auto deadline = Clock::now() + patience;
        while (*c.cmp(0, len) == 0 && Clock::now() < deadline) _mm_pause();
        const bool ok = (*c.cmp(0, len) == GET_ERROR) && (ret[0] == 0x1234);
        printf("%s error: a get past the export's end completed with %016lx, data %s\n", ok ? "ok  " : "FAIL",
               (unsigned long) *c.cmp(0, len), ret[0] == 0x1234 ? "untouched" : "WRITTEN");
        if (!ok) errors++;
    }

    printf("client: %lu get requests sent\n", (unsigned long) (csr_read(t_qp, GET_SENT) - g0));
    t_qp.connSync(true);       // done
    t_qp.unmapUwin();
    release_window(t_qp, 1);
    release_export(t_qp, RET_EXPORT);
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
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--server")                     server = true;
        else if (a == "--client" && i + 1 < argc) ip = argv[++i];
        else if (a == "--port" && i + 1 < argc)  port = uint16_t(atoi(argv[++i]));
        else if (a == "--size" && i + 1 < argc)  size = strtoull(argv[++i], nullptr, 0);
        else if (a == "--reps" && i + 1 < argc)  reps = atoi(argv[++i]);
        else { fprintf(stderr, "usage: see the header of get_bench.cpp\n"); return 2; }
    }
    if (server == !ip.empty() || size < (64 << 10) || size % 4096) {
        fprintf(stderr, "usage: see the header of get_bench.cpp (--size: a multiple of 4096, at least 64 KiB)\n");
        return 2;
    }
    return server ? run_server(port, size) : run_client(ip, port, size, reps);
}
