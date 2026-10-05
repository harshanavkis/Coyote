/**
 * ce_remote - gate G4: the copy engine on one host writes into a buffer on
 * the other, through both switches:
 *
 *   client V80 HBM --loom_ce, P2P--> client U280 uwin --loom_ingress, rdma
 *   route--> RoCE --> server U280 loom_rx --> server host buffer
 *
 * Server (the landing host; U280, and its V80 with --land-v80):
 *     ce_remote --server [--port N] [--size BYTES] [--land-v80 [--land-offset B]]
 *   (--land-offset: land B bytes into the V80 window, a multiple of 64 below 4096)
 *   1. QP exchange (blocks for the client); the QP owner's staging buffer
 *   2. destination and fence buffers on a data cThread
 *   3. both buffers exported (loom_switch export table: 1 = destination,
 *      2 = fence page, landing under the data cThread), and a TCP hello to
 *      the client with their references
 *   4. per copy: the client announces the fence value to expect; the
 *      server waits for it in the fence page, checks every byte, answers
 *   With --land-v80 the destination and fence are the server's V80 card
 *   memory: the V80's uwin, exported by its driver, is imported into the
 *   data cThread, so loom_rx lands peer-to-peer into the V80's landing
 *   window. The server waits for the copy's fence store to land and every
 *   card write to complete, syncs the buffer back and checks it.
 *
 * Client (the sending host; U280 + V80):
 *     ce_remote --client <server_ip> [--port N] [--reps N] [--window P] [--gap-ms G]
 *   (--gap-ms: idle G ms before each copy, to tell time-periodic effects from
 *   data-periodic ones; the server stamps each copy with its time since the
 *   first)
 *   1. QP exchange, then the hello
 *   2. rdma windows onto the server's destination (uwin 0) and fence page
 *      (right after it), over the QP owner's connection: their bases are
 *      the server's export references, which every packet carries
 *   3. that part of the uwin exported to the V80
 *   4. per copy: fill the source, offload it to HBM, announce it, start
 *      loom_ce with the fence behind the data, report the server's answer
 *
 * Both directions at once (one QP, one process per host, each host both
 * sender and lander):
 *     ce_remote --bidir-server [--port N] [--size BYTES] [--reps N] [--no-send]
 *     ce_remote --bidir-client <server_ip> [--port N] [--reps N] [--window P] [--no-send]
 *   each side lands the peer's copies in its own host buffers and pushes
 *   its own V80's copies into the peer's; per copy both start together
 *   (a TCP barrier), and each side times the peer's copy from the barrier
 *   to the fence landing, checks every byte, and reports its copy engine's
 *   cycles (the outgoing push). --no-send keeps that side's copy engine
 *   idle: the one-way control in the same setup.
 *
 * The QP port is N, the hello's TCP port N + 1 (N defaults to Coyote's).
 * --window sets the client's ack window (packets unacked, TX_CTL; 16 after
 * reset). Each side prints where its switch's cycles went during a copy:
 * the client its waits on the window and on sq_wr, the server loom_rx's
 * moving / starved / stalled cycles; the client also its U280 ingress's
 * debug counters and its copy engine's (V80 words 48-50), a V80-landing
 * server the V80 window's (uwin_hbm, read through the window's last page).
 */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <immintrin.h>
#include <stdexcept>
#include <string>

#include <coyote/cThread.hpp>
#include "land_v80.hpp"
#include "loom_switch.hpp"

namespace {

enum CeReg : uint32_t { START = 0, SRC_VA = 1, DST_VA = 2, LEN = 3, PID = 4, FENCE_VA = 5, DIRECTION = 6,
                        BUSY = 8, COPIES = 9, CYCLES = 10 };

constexpr uint32_t MAGIC        = 0x4C434552;   // "LCER"
constexpr uint64_t STAGING_SIZE = 1 << 20;

struct Hello {
    uint32_t magic;
    uint32_t dst_ctid;     // the server's data cThread: the landing pid
    uint64_t staging_va;   // the server's RDMA staging buffer
    uint64_t dst_va;
    uint64_t fence_va;
    uint64_t size;
};
struct Announce { uint64_t rep; uint64_t fence; };        // client -> server; rep ~0 = done
struct Verdict  { uint64_t bad; uint64_t fence; double wait_us; };

// A set of the switch's counters, read before and after a copy
struct Counters {
    static constexpr int N = 9;
    static constexpr uint32_t W[N] = {loom_switch::CYC, loom_switch::TX_WINFULL, loom_switch::TX_REQWAIT,
                                      loom_switch::WR_WAIT_LOCAL, loom_switch::WR_WAIT_RDMA,
                                      loom_switch::RX_MOVE, loom_switch::RX_STARVE, loom_switch::RX_STALL,
                                      loom_switch::RX_FIFO_FULL};
    uint64_t v[N];
    static Counters read(coyote::cThread &t) {
        Counters c;
        for (int i = 0; i < N; i++) c.v[i] = loom_switch::csr_read(t, W[i]);
        return c;
    }
    uint64_t d(const Counters &before, int i) const { return v[i] - before.v[i]; }
};
enum { C_CYC, C_WINFULL, C_REQWAIT, C_WAIT_LOCAL, C_WAIT_RDMA, C_RX_MOVE, C_RX_STARVE, C_RX_STALL, C_RX_FF };

// The lander's host write path, per copy (loom_ctrl.sv 112-135; longest runs,
// at 144+, are since the bitstream was loaded, so a copy that raises one is
// the copy that had that stall)
struct Shell {
    static constexpr int N = 12;
    static constexpr uint32_t W[N] = {115, 123, 125, 129, 130, 131, 135, 112, 132, 133, 134, 127};
    static constexpr int NM = 4;
    static constexpr uint32_t M[NM] = {155, 157, 163, 166};    // longest runs of 123, 125, 131, 134
    uint64_t v[N], m[NM];
    static Shell read(coyote::cThread &t) {
        Shell c;
        for (int i = 0; i < N; i++) c.v[i] = loom_switch::csr_read(t, W[i]);
        for (int i = 0; i < NM; i++) c.m[i] = loom_switch::csr_read(t, M[i]);
        return c;
    }
    void print(const Shell &b) const {
        auto d = [&](int i) { return (unsigned long) (v[i] - b.v[i]); };
        auto mx = [&](int i) { return m[i] > b.m[i] ? " NEW" : ""; };
        printf("     shell: MMU entry wait %lu (longest %lu%s), data waited on MMU %lu (longest %lu%s), "
               "MMU completion wait %lu (longest %lu%s), %lu completions; credit stage: data %lu, downstream %lu; "
               "rx post wait %lu, %lu packets\n"
               "            MMU write FSM: DMA port wait %lu, mutex wait %lu, miss/invalidate/locked %lu "
               "(longest %lu%s); writebacks %lu\n",
               d(1), (unsigned long) m[0], mx(0), d(2), (unsigned long) m[1], mx(1),
               d(5), (unsigned long) m[2], mx(2), d(6), d(3), d(4), d(0), d(7),
               d(8), d(9), d(10), (unsigned long) m[3], mx(3), d(11));
    }
};

uint64_t pattern(uint64_t off, uint64_t rep) { return 0xCE00000000000000ULL ^ (off * 0x9E3779B97F4A7C15ULL) ^ rep; }

bool read_full(int fd, void *p, size_t n) {
    auto *b = static_cast<char *>(p);
    while (n) { ssize_t r = ::read(fd, b, n); if (r <= 0) return false; b += r; n -= r; }
    return true;
}
bool write_full(int fd, const void *p, size_t n) {
    auto *b = static_cast<const char *>(p);
    while (n) { ssize_t r = ::write(fd, b, n); if (r <= 0) return false; b += r; n -= r; }
    return true;
}

// The V80 landing's counters (loom_ce_ctrl.sv 24-29, 32-45), the copy
// engine's (48-50)
void print_v80_words(coyote::cThread &v80, const uint64_t *before, uint32_t first, int n,
                     const char *const *names) {
    for (int i = 0; i < n; i++) {
        const uint64_t d = v80.getCSR(first + i) - before[i];
        if (d && names[i][0]) printf("    %-17s %lu\n", names[i], (unsigned long) d);
    }
}
const char *const CE_NAMES[3]    = {"CE out bp", "CE in wait", "sq_wr wait"};
void snap(coyote::cThread &v80, uint64_t *out, uint32_t first, int n) {
    for (int i = 0; i < n; i++) out[i] = v80.getCSR(first + i);
}

int run_server(uint16_t port, uint64_t size, bool land, uint64_t off, unsigned poll_us, bool no_touch) {
    coyote::cThread t_qp(0, getpid(), 0, nullptr, "coyote_fpga");     // QP owner
    coyote::cThread t_data(0, getpid(), 0, nullptr, "coyote_fpga");   // owns the landing buffers
    printf("server: waiting for the QP exchange on port %u ...\n", port);
    void *staging = t_qp.initRDMA(STAGING_SIZE, port);
    if (!staging) { printf("FAIL: initRDMA\n"); return 1; }
    loom_switch::csr_write(t_qp, loom_switch::RDMA_STAGING_VA, reinterpret_cast<uint64_t>(staging));

    uint64_t *dst, *fence;
    coyote::cThread *v80 = nullptr;
    land_v80::Landing *L = nullptr;
    if (land) {
        v80 = new coyote::cThread(0, getpid(), 0, nullptr, "coyote_versal_fpga");
        // --land-offset: the copy lands off bytes into the window (64: each
        // message's packets then end on 4 KiB pages, the U280 splits none)
        const uint64_t llen = size + 8192;
        L = new land_v80::Landing(*v80, llen);
        const int lfd = L->export_fd();
        void *lva = loom_switch::reserve_va(llen);
        t_data.importDmabuf(lfd, lva);
        dst   = static_cast<uint64_t *>(lva) + off / 8;
        fence = dst + size / 8;
        printf("server: V80 uwin imported at %p, landing at card VA %p\n", lva, (void *) L->buf);
    } else {
        dst   = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, size}));
        fence = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, 4096}));
        memset(dst, 0, size);
        memset(fence, 0, 4096);
    }
    printf("server: QP up; staging %p, dst %p, fence %p, data ctid %d\n", staging, (void *) dst,
           (void *) fence, t_data.getCtid());

    int lfd = ::socket(AF_INET, SOCK_STREAM, 0), one = 1;
    ::setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in a{};
    a.sin_family = AF_INET; a.sin_addr.s_addr = INADDR_ANY; a.sin_port = htons(port + 1);
    if (::bind(lfd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0 || ::listen(lfd, 1) < 0) {
        printf("FAIL: hello listener on port %u\n", port + 1);
        return 1;
    }
    loom_switch::program_export(t_qp, 1, t_data.getCtid(), dst, size);
    loom_switch::program_export(t_qp, 2, t_data.getCtid(), fence, 4096);
    int c = ::accept(lfd, nullptr, nullptr);
    Hello h{MAGIC, uint32_t(t_data.getCtid()), reinterpret_cast<uint64_t>(staging),
            loom_switch::export_ref(1), loom_switch::export_ref(2), size};
    if (!write_full(c, &h, sizeof(h))) { printf("FAIL: hello\n"); return 1; }

    volatile uint64_t *vfence = fence;
    volatile uint64_t *vdst = dst;
    int errors = 0, copies = 0;
    std::chrono::steady_clock::time_point t_first;
    Announce an;
    while (read_full(c, &an, sizeof(an)) && an.rep != ~0ULL) {
        const Counters k0 = Counters::read(t_qp);
        const Shell s0 = Shell::read(t_qp);
        land_v80::Counters u0{}, u1{};
        if (land) u0 = land_v80::Counters::read(L->win);
        auto t0 = std::chrono::steady_clock::now();
        if (copies == 0) t_first = t0;
        Verdict v{};
        if (land) {
            // the fence, read through the V80's window, then the data synced back
            (void) L->wait(off + size, an.fence, std::chrono::milliseconds(5000), poll_us);
            v.wait_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
            u1 = land_v80::Counters::read(L->win);
            if (!no_touch) L->pull();
            vdst = L->buf + off / 8;
            v.fence = L->buf[(off + size) / 8];
        } else {
            while (*vfence != an.fence && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5))
                _mm_pause();
            v.fence = *vfence;
            v.wait_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
        }
        const Counters k1 = Counters::read(t_qp);
        const Shell s1 = Shell::read(t_qp);
        if (v.fence == an.fence) {
            if (!no_touch)
                for (uint64_t i = 0; i < size / 8; i++) v.bad += (vdst[i] != pattern(8 * i, an.rep));
        } else
            v.bad = size / 8;
        printf("%s copy %lu: fence %lu (expected %lu), %lu of %lu words wrong, fence %.1f us after the announce (%.2f GB/s) at t=%.1f ms\n",
               (v.bad ? "FAIL" : "ok  "), (unsigned long) an.rep, (unsigned long) v.fence,
               (unsigned long) an.fence, (unsigned long) v.bad, (unsigned long) (size / 8),
               v.wait_us, size / v.wait_us / 1e3,
               std::chrono::duration<double, std::milli>(t0 - t_first).count());
        printf("     loom_rx over %lu cycles: %lu moving, %lu starved (nothing arrived), %lu stalled (host write not ready), rx FIFO full %lu\n",
               (unsigned long) k1.d(k0, C_CYC), (unsigned long) k1.d(k0, C_RX_MOVE), (unsigned long) k1.d(k0, C_RX_STARVE),
               (unsigned long) k1.d(k0, C_RX_STALL), (unsigned long) k1.d(k0, C_RX_FF));
        s1.print(s0);
        if (land) u1.print(u0);
        errors += (v.bad != 0);
        copies++;
        // the next copy must write every byte again (--no-touch: the
        // buffer is left alone between copies, and not checked)
        if (!no_touch) {
            if (land) L->clear();
            else      memset(dst, 0, size);
        }
        if (!write_full(c, &v, sizeof(v))) break;
    }
    printf("server: loom_rx landed %lu writes, dropped %lu packets\n",
           (unsigned long) loom_switch::csr_read(t_qp, loom_switch::RX_FWD),
           (unsigned long) loom_switch::csr_read(t_qp, loom_switch::RX_DROP));
    ::close(c);
    ::close(lfd);
    loom_switch::release_export(t_qp, 1);
    loom_switch::release_export(t_qp, 2);
    t_qp.connSync(false);
    delete L;
    delete v80;
    printf(errors || !copies ? "G4 FAIL (server)\n" : "G4 PASS (server)\n");
    return errors || !copies ? 1 : 0;
}

// --passive: the landing half for a plain RDMA sender (perf_rdma's bitstream
// and ~/loom-experiments/perf_rdma_landing/landing_client): the QP exchange
// and the exports as for --server, no hello and no announcements. The sender
// writes export_ref(1) + offset; two barriers frame its runs. Afterwards every
// byte is checked (the sender's buffer holds int i at word i, MSG bytes per
// write, each write MSG further into the export) and the receive path's
// counters are printed for the whole session. Each of the REPS runs is
// timed here, from its first landed packet to its last (loom_rx's landed
// count, size / 4 KiB packets per run): the sender's completions are not used.
int run_passive(uint16_t port, uint64_t size, bool land, uint64_t msg, int reps, bool discard) {
    coyote::cThread t_qp(0, getpid(), 0, nullptr, "coyote_fpga");     // QP owner
    coyote::cThread t_data(0, getpid(), 0, nullptr, "coyote_fpga");   // owns the landing buffer
    printf("passive: waiting for the QP exchange on port %u ...\n", port);
    void *staging = t_qp.initRDMA(STAGING_SIZE, port);
    if (!staging) { printf("FAIL: initRDMA\n"); return 1; }

    uint64_t *dst;
    coyote::cThread *v80 = nullptr;
    land_v80::Landing *L = nullptr;
    if (land) {
        v80 = new coyote::cThread(0, getpid(), 0, nullptr, "coyote_versal_fpga");
        L = new land_v80::Landing(*v80, size + 8192);
        void *lva = loom_switch::reserve_va(size + 8192);
        t_data.importDmabuf(L->export_fd(), lva);
        dst = static_cast<uint64_t *>(lva);
        L->clear();
        printf("passive: V80 uwin imported at %p, landing at card VA %p\n", lva, (void *) L->buf);
        if (discard) {
            // --discard: the V80 window answers the writes without HBM; nothing to check
            if (!L->set_discard(true)) { printf("FAIL: the V80 window did not switch to DISCARD (image without it?)\n"); return 1; }
            printf("passive: V80 window in DISCARD mode: writes are counted and dropped, not landed\n");
        }
    } else {
        dst = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, size}));
        memset(dst, 0, size);
    }
    loom_switch::program_export(t_qp, 1, t_data.getCtid(), dst, size);
    const Counters k0 = Counters::read(t_qp);
    const Shell s0 = Shell::read(t_qp);
    land_v80::Counters u0{}, u1{};
    if (land) u0 = land_v80::Counters::read(L->win);
    const uint64_t fwd0 = loom_switch::csr_read(t_qp, loom_switch::RX_FWD);
    const uint64_t drop0 = loom_switch::csr_read(t_qp, loom_switch::RX_DROP);
    printf("passive: export 1 = %lu B at %p (ctid %d); waiting for the sender\n",
           (unsigned long) size, (void *) dst, t_data.getCtid());
    t_qp.connSync(false);                 // the sender may start
    const uint64_t per_run = size / 4096;
    uint64_t landed = fwd0;
    std::chrono::steady_clock::time_point t_first, t_prev_end;
    for (int r = 0; r < reps; r++) {
        const auto t_wait = std::chrono::steady_clock::now();
        uint64_t f;
        while ((f = loom_switch::csr_read(t_qp, loom_switch::RX_FWD)) == landed)
            if (std::chrono::steady_clock::now() - t_wait > std::chrono::seconds(10)) break;
        if (f == landed) { printf("FAIL run %d: nothing landed in 10 s\n", r); break; }
        const auto t0 = std::chrono::steady_clock::now();
        if (r == 0) t_first = t0;
        while ((f = loom_switch::csr_read(t_qp, loom_switch::RX_FWD)) < landed + per_run)
            if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(10)) break;
        const auto t1 = std::chrono::steady_clock::now();
        const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        printf("%s run %d: %lu of %lu packets landed in %.1f us = %.2f GB/s at t=%.1f ms (idle before: %.1f ms)\n",
               f >= landed + per_run ? "ok  " : "FAIL", r, (unsigned long) (f - landed), (unsigned long) per_run,
               us, double(size) / (us * 1e3), std::chrono::duration<double, std::milli>(t0 - t_first).count(),
               r ? std::chrono::duration<double, std::milli>(t0 - t_prev_end).count() : 0.0);
        t_prev_end = t1;
        landed += per_run;
    }
    t_qp.connSync(false);                 // the sender is done
    const Counters k1 = Counters::read(t_qp);
    const Shell s1 = Shell::read(t_qp);
    if (land) u1 = land_v80::Counters::read(L->win);

    uint64_t bad = 0;
    if (land && discard) {
        if (!L->set_discard(false)) printf("WARN: the V80 window did not switch back to HBM\n");
    } else {
        const uint32_t *w;
        if (land) { L->pull(); w = reinterpret_cast<const uint32_t *>(L->buf); }
        else      w = reinterpret_cast<const uint32_t *>(dst);
        for (uint64_t i = 0; i < size / 4; i++) bad += (w[i] != uint32_t(i % (msg / 4)));
    }
    printf("passive: loom_rx landed %lu writes, dropped %lu packets; %lu of %lu words wrong\n",
           (unsigned long) (loom_switch::csr_read(t_qp, loom_switch::RX_FWD) - fwd0),
           (unsigned long) (loom_switch::csr_read(t_qp, loom_switch::RX_DROP) - drop0),
           (unsigned long) bad, (unsigned long) (size / 4));
    printf("     loom_rx over %lu cycles: %lu moving, %lu starved (nothing arrived), %lu stalled (host write not ready), rx FIFO full %lu\n",
           (unsigned long) k1.d(k0, C_CYC), (unsigned long) k1.d(k0, C_RX_MOVE), (unsigned long) k1.d(k0, C_RX_STARVE),
           (unsigned long) k1.d(k0, C_RX_STALL), (unsigned long) k1.d(k0, C_RX_FF));
    s1.print(s0);
    if (land) u1.print(u0);
    loom_switch::release_export(t_qp, 1);
    delete L;
    delete v80;
    printf(bad ? "PASSIVE FAIL\n" : "PASSIVE PASS\n");
    return bad ? 1 : 0;
}

// --bidir: one process per host, both directions over one QP
struct BiHello { uint32_t magic; uint32_t dst_ctid; uint64_t staging_va, dst_va, fence_va, size; };
struct BiGo    { uint64_t rep, fence; uint32_t send, pad; };                 // my fence value, if I send
struct BiDone  { uint64_t bad; double wait_us; uint64_t ce_cycles; };

uint64_t bi_pattern(uint64_t off, uint64_t rep, int side) { return pattern(off, rep) ^ (uint64_t(side + 1) << 40); }

int run_bidir(const std::string &ip, uint16_t port, uint64_t size, int reps, int window, bool send) {
    const bool server = ip.empty();
    const int me = server ? 0 : 1, peer = 1 - me;
    coyote::cThread t_qp(0, getpid(), 0, nullptr, "coyote_fpga");     // QP owner, CSR page, rdma windows
    coyote::cThread t_data(0, getpid(), 0, nullptr, "coyote_fpga");   // owns the landing buffers
    coyote::cThread v80(0, getpid(), 0, nullptr, "coyote_versal_fpga");
    printf("%s: QP exchange on port %u ...\n", server ? "server" : "client", port);
    void *staging = server ? t_qp.initRDMA(STAGING_SIZE, port) : t_qp.initRDMA(STAGING_SIZE, port, ip.c_str());
    if (!staging) { printf("FAIL: initRDMA\n"); return 1; }

    // Landing buffers for the peer's copies
    uint64_t *dst   = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, size}));
    uint64_t *fence = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, 4096}));
    memset(dst, 0, size);
    memset(fence, 0, 4096);

    // Hello both ways
    int fd = -1;
    if (server) {
        int lfd = ::socket(AF_INET, SOCK_STREAM, 0), one = 1;
        ::setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in a{};
        a.sin_family = AF_INET; a.sin_addr.s_addr = INADDR_ANY; a.sin_port = htons(port + 1);
        if (::bind(lfd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0 || ::listen(lfd, 1) < 0) {
            printf("FAIL: hello listener on port %u\n", port + 1);
            return 1;
        }
        fd = ::accept(lfd, nullptr, nullptr);
        ::close(lfd);
    } else {
        for (int i = 0; i < 300 && fd < 0; i++) {
            fd = ::socket(AF_INET, SOCK_STREAM, 0);
            sockaddr_in a{};
            a.sin_family = AF_INET; a.sin_port = htons(port + 1);
            ::inet_pton(AF_INET, ip.c_str(), &a.sin_addr);
            if (::connect(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0) { ::close(fd); fd = -1; usleep(100000); }
        }
    }
    loom_switch::program_export(t_qp, 1, t_data.getCtid(), dst, size);
    loom_switch::program_export(t_qp, 2, t_data.getCtid(), fence, 4096);
    BiHello mine{MAGIC, uint32_t(t_data.getCtid()), reinterpret_cast<uint64_t>(staging),
                 loom_switch::export_ref(1), loom_switch::export_ref(2), size}, h{};
    if (fd < 0 || !write_full(fd, &mine, sizeof(mine)) || !read_full(fd, &h, sizeof(h)) || h.magic != MAGIC || h.size != size) {
        printf("FAIL: hello (sizes must match on both sides)\n");
        return 1;
    }

    // Outgoing: rdma windows onto the peer's buffers, the uwin exported to the V80
    if (window >= 0) loom_switch::csr_write(t_qp, loom_switch::TX_CTL, uint64_t(window));
    loom_switch::csr_write(t_qp, loom_switch::RDMA_STAGING_VA, h.staging_va);
    loom_switch::program_window(t_qp, 1, true, t_qp.getCtid(), reinterpret_cast<void *>(h.dst_va), size, 0, h.dst_ctid);
    loom_switch::program_window(t_qp, 2, true, t_qp.getCtid(), reinterpret_cast<void *>(h.fence_va), 4096, size, h.dst_ctid);
    const uint64_t win_len = size + 4096;
    const int dfd = t_qp.exportDmabuf(EXPORT_REGION_UWIN, 0, win_len);
    void *uva = loom_switch::reserve_va(win_len);
    v80.importDmabuf(dfd, uva);
    uint64_t *src = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, size}));
    printf("%s: up; sending %s, receiving from a peer that %s\n", server ? "server" : "client",
           send ? "yes" : "no", "announces per copy");

    volatile uint64_t *vfence = fence;
    int errors = 0;
    for (int r = 0; r < reps; r++) {
        if (send) {
            for (uint64_t i = 0; i < size / 8; i++) src[i] = bi_pattern(8 * i, r, me);
            v80.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{src, size});
            v80.setCSR(reinterpret_cast<uint64_t>(src), SRC_VA);
            v80.setCSR(reinterpret_cast<uint64_t>(uva), DST_VA);
            v80.setCSR(size, LEN);
            v80.setCSR(v80.getCtid(), PID);
            v80.setCSR(reinterpret_cast<uint64_t>(uva) + size, FENCE_VA);
            v80.setCSR(0, DIRECTION);   // a put; DIRECTION outlives a ce_get run
        }
        const loom_switch::IngressCounters c0 = loom_switch::IngressCounters::read(t_qp);
        const Counters k0 = Counters::read(t_qp);

        // Barrier: exchange what each side will do, then both start
        BiGo g{uint64_t(r), send ? v80.getCSR(COPIES) + 1 : 0, uint32_t(send), 0}, pg{};
        if (!write_full(fd, &g, sizeof(g)) || !read_full(fd, &pg, sizeof(pg)) || pg.rep != uint64_t(r)) {
            printf("FAIL: barrier\n");
            return 1;
        }
        const auto t0 = std::chrono::steady_clock::now();
        if (send) v80.setCSR(1, START);

        BiDone d{0, 0, 0};
        if (pg.send) {
            while (*vfence != pg.fence && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5))
                _mm_pause();
            d.wait_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
            if (*vfence == pg.fence)
                for (uint64_t i = 0; i < size / 8; i++) d.bad += (dst[i] != bi_pattern(8 * i, r, peer));
            else
                d.bad = size / 8;
        }
        if (send) {
            while (v80.getCSR(BUSY) && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5)) _mm_pause();
            d.ce_cycles = v80.getCSR(CYCLES);
        }
        const loom_switch::IngressCounters c1 = loom_switch::IngressCounters::read(t_qp);
        const Counters k1 = Counters::read(t_qp);

        BiDone pd{};
        if (!write_full(fd, &d, sizeof(d)) || !read_full(fd, &pd, sizeof(pd))) { printf("FAIL: verdict\n"); return 1; }
        const bool bad = (pg.send && d.bad) || (send && pd.bad);
        errors += bad;
        printf("%s copy %d: ", bad ? "FAIL" : "ok  ", r);
        if (pg.send) printf("in %lu B landed %.1f us after the barrier (%.2f GB/s), %lu words wrong; ",
                            (unsigned long) size, d.wait_us, size / d.wait_us / 1e3, (unsigned long) d.bad);
        if (send)    printf("out: CE %lu cycles (%.2f GB/s), peer saw it after %.1f us (%.2f GB/s), %lu words wrong",
                            (unsigned long) d.ce_cycles, size / (d.ce_cycles * 4e-9) / 1e9, pd.wait_us,
                            pd.wait_us > 0 ? size / pd.wait_us / 1e3 : 0.0, (unsigned long) pd.bad);
        printf("\n");
        printf("     ingress: %lu rdma packets, %lu dropped; waits over %lu cycles: ack window %lu, sq_wr rdma %lu / local %lu; "
               "loom_rx moving %lu, starved %lu, stalled %lu\n",
               (unsigned long) (c1.v[3] - c0.v[3]), (unsigned long) (c1.v[1] - c0.v[1]),
               (unsigned long) k1.d(k0, C_CYC), (unsigned long) k1.d(k0, C_WINFULL),
               (unsigned long) k1.d(k0, C_WAIT_RDMA), (unsigned long) k1.d(k0, C_WAIT_LOCAL),
               (unsigned long) k1.d(k0, C_RX_MOVE), (unsigned long) k1.d(k0, C_RX_STARVE), (unsigned long) k1.d(k0, C_RX_STALL));
        memset(dst, 0, size);
        *vfence = 0;
    }
    ::close(fd);
    loom_switch::release_window(t_qp, 1);
    loom_switch::release_window(t_qp, 2);
    loom_switch::release_export(t_qp, 1);
    loom_switch::release_export(t_qp, 2);
    t_qp.connSync(!server);
    printf(errors ? "BIDIR FAIL\n" : "BIDIR PASS\n");
    return errors ? 1 : 0;
}

int run_client(const std::string &ip, uint16_t port, int reps, int window, unsigned gap_ms, bool no_refill) {
    coyote::cThread u280(0, getpid(), 0, nullptr, "coyote_fpga");     // QP owner and CSR page
    coyote::cThread v80(0, getpid(), 0, nullptr, "coyote_versal_fpga");
    if (!u280.initRDMA(STAGING_SIZE, port, ip.c_str())) { printf("FAIL: initRDMA\n"); return 1; }
    printf("client: QP up\n");

    int fd = -1;
    for (int i = 0; i < 300 && fd < 0; i++) {
        fd = ::socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in a{};
        a.sin_family = AF_INET; a.sin_port = htons(port + 1);
        ::inet_pton(AF_INET, ip.c_str(), &a.sin_addr);
        if (::connect(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0) { ::close(fd); fd = -1; usleep(100000); }
    }
    Hello h{};
    if (fd < 0 || !read_full(fd, &h, sizeof(h)) || h.magic != MAGIC) { printf("FAIL: no hello from the server\n"); return 1; }
    const uint64_t size = h.size;
    printf("client: server dst %lx, fence %lx, %lu bytes, landing ctid %u\n",
           (unsigned long) h.dst_va, (unsigned long) h.fence_va, (unsigned long) size, h.dst_ctid);

    if (window >= 0) loom_switch::csr_write(u280, loom_switch::TX_CTL, uint64_t(window));
    printf("client: ack window %lu packets\n", (unsigned long) loom_switch::csr_read(u280, loom_switch::TX_CTL));

    // rdma windows onto the server's buffers, over this QP, landing under its data ctid
    loom_switch::csr_write(u280, loom_switch::RDMA_STAGING_VA, h.staging_va);
    loom_switch::program_window(u280, 1, true, u280.getCtid(), reinterpret_cast<void *>(h.dst_va),
                                size, 0, h.dst_ctid);
    loom_switch::program_window(u280, 2, true, u280.getCtid(), reinterpret_cast<void *>(h.fence_va),
                                4096, size, h.dst_ctid);

    const uint64_t win_len = size + 4096;
    const int dfd = u280.exportDmabuf(EXPORT_REGION_UWIN, 0, win_len);
    void *uva = loom_switch::reserve_va(win_len);
    v80.importDmabuf(dfd, uva);

    uint64_t *src = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, size}));
    int errors = 0;
    for (int r = 0; r < reps; r++) {
        if (gap_ms) usleep(gap_ms * 1000);
        // --no-refill: the source is filled and offloaded once (copy 0's
        // pattern every time)
        if (!no_refill || r == 0) {
            for (uint64_t i = 0; i < size / 8; i++) src[i] = pattern(8 * i, r);
            v80.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{src, size});
        }

        const uint64_t fence = v80.getCSR(COPIES) + 1;
        loom_switch::IngressCounters c0 = loom_switch::IngressCounters::read(u280);
        const uint64_t cut0 = loom_switch::csr_read(u280, 121);   // partial rdma packets cut by a write
        const Counters k0 = Counters::read(u280);
        uint64_t e0[3];
        snap(v80, e0, 48, 3);
        Announce an{no_refill ? 0 : uint64_t(r), fence};
        write_full(fd, &an, sizeof(an));

        v80.setCSR(reinterpret_cast<uint64_t>(src), SRC_VA);
        v80.setCSR(reinterpret_cast<uint64_t>(uva), DST_VA);
        v80.setCSR(size, LEN);
        v80.setCSR(v80.getCtid(), PID);
        v80.setCSR(reinterpret_cast<uint64_t>(uva) + size, FENCE_VA);
        v80.setCSR(0, DIRECTION);   // a put; DIRECTION outlives a ce_get run
        v80.setCSR(1, START);

        Verdict v{};
        if (!read_full(fd, &v, sizeof(v))) { printf("FAIL: the server went away\n"); return 1; }
        loom_switch::IngressCounters c1 = loom_switch::IngressCounters::read(u280);
        const Counters k1 = Counters::read(u280);
        printf("%s copy %d: %lu bytes, server saw the fence after %.1f us, %lu words wrong; CE %lu cycles\n",
               (v.bad ? "FAIL" : "ok  "), r, (unsigned long) size, v.wait_us, (unsigned long) v.bad,
               (unsigned long) v80.getCSR(CYCLES));
        printf("     ingress: %lu bursts, %lu rdma packets (%lu closed by the idle timer, %lu cut by a write), "
               "%lu stores, %lu dropped; acks %lu, unacked %lu\n",
               (unsigned long) (c1.v[0] - c0.v[0]), (unsigned long) (c1.v[3] - c0.v[3]),
               (unsigned long) (c1.v[6] - c0.v[6]),
               (unsigned long) (loom_switch::csr_read(u280, 121) - cut0),
               (unsigned long) (c1.v[4] - c0.v[4]), (unsigned long) (c1.v[1] - c0.v[1]),
               (unsigned long) loom_switch::csr_read(u280, loom_switch::TX_ACKS),
               (unsigned long) loom_switch::csr_read(u280, loom_switch::TX_STATE));
        printf("     switch waits over %lu cycles: ack window %lu, sq_wr (rdma) %lu, sq_wr any rdma %lu / local %lu\n",
               (unsigned long) k1.d(k0, C_CYC), (unsigned long) k1.d(k0, C_WINFULL), (unsigned long) k1.d(k0, C_REQWAIT),
               (unsigned long) k1.d(k0, C_WAIT_RDMA), (unsigned long) k1.d(k0, C_WAIT_LOCAL));
        printf("     U280 ingress debug:\n");
        loom_switch::print_ingress_delta(c0, c1, loom_switch::I_DBG, loom_switch::N_ING - loom_switch::I_DBG);
        printf("     V80 copy engine:\n");
        print_v80_words(v80, e0, 48, 3, CE_NAMES);
        errors += (v.bad != 0);
    }
    Announce done{~0ULL, 0};
    write_full(fd, &done, sizeof(done));
    ::close(fd);
    loom_switch::release_window(u280, 1);
    loom_switch::release_window(u280, 2);
    u280.connSync(true);
    printf(errors ? "G4 FAIL (client)\n" : "G4 PASS (client)\n");
    return errors ? 1 : 0;
}

}  // namespace

int main(int argc, char **argv) {
    bool server = false;
    std::string ip;
    uint16_t port = coyote::DEF_PORT;
    uint64_t size = 1ULL << 20;
    int reps = 3;
    int window = -1;
    bool land = false, bidir = false, no_send = false;
    uint64_t land_off = 0;
    unsigned poll_us = 0, gap_ms = 0;
    bool no_touch = false, no_refill = false;
    bool passive = false, discard = false;
    uint64_t msg = 1ULL << 20;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--server")                     server = true;
        else if (a == "--client" && i + 1 < argc) ip = argv[++i];
        else if (a == "--port" && i + 1 < argc)   port = uint16_t(atoi(argv[++i]));
        else if (a == "--size" && i + 1 < argc)   size = strtoull(argv[++i], nullptr, 0);
        else if (a == "--reps" && i + 1 < argc)   reps = atoi(argv[++i]);
        else if (a == "--window" && i + 1 < argc) window = atoi(argv[++i]);
        else if (a == "--gap-ms" && i + 1 < argc) gap_ms = unsigned(atoi(argv[++i]));
        else if (a == "--no-touch")              no_touch = true;
        else if (a == "--no-refill")             no_refill = true;
        else if (a == "--land-v80")               land = true;
        else if (a == "--land-offset" && i + 1 < argc) land_off = strtoull(argv[++i], nullptr, 0);
        else if (a == "--poll-us" && i + 1 < argc) poll_us = unsigned(atoi(argv[++i]));
        else if (a == "--bidir-server")           { bidir = true; server = true; }
        else if (a == "--bidir-client" && i + 1 < argc) { bidir = true; ip = argv[++i]; }
        else if (a == "--no-send")                no_send = true;
        else if (a == "--passive")                { passive = true; server = true; }
        else if (a == "--msg" && i + 1 < argc)    msg = strtoull(argv[++i], nullptr, 0);
        else if (a == "--discard")                discard = true;
        else { server = false; ip.clear(); break; }
    }
    if (server == !ip.empty() || size == 0 || size % 4096 || size > (64ULL << 20) || land_off % 64 || land_off >= 4096) {
        printf("usage: %s --server [--port N] [--size BYTES] [--land-v80 [--land-offset B] [--poll-us U]] | --client <server_ip> [--port N] [--reps N] [--window P]\n"
               "       | --passive [--port N] [--size BYTES] [--msg BYTES] [--reps N] [--land-v80 [--discard]]  (landing for a plain RDMA sender)\n"
               "       | --bidir-server | --bidir-client <server_ip>  [--port N] [--size BYTES] [--reps N] [--window P] [--no-send]\n"
               "       (size: a multiple of 4 KiB, at most 64 MiB)\n", argv[0]);
        return 2;
    }
    if (passive) return run_passive(port, size, land, msg, reps, discard);
    if (bidir) return run_bidir(server ? std::string() : ip, port, size, reps, window, !no_send);
    return server ? run_server(port, size, land, land_off, poll_us, no_touch) : run_client(ip, port, reps, window, gap_ms, no_refill);
}
