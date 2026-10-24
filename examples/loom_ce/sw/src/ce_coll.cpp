/**
 * ce_coll - collectives at two ranks on Loom (E5): amy's and rose's V80s, data in each V80's HBM, int32.
 * Each rank runs ONE program of K iterations (copies through Loom's window into the peer's HBM, a SIGNAL
 * behind them, a WAIT on its own flag, a REDUCE where the collective has one), started by the CPU, which then
 * polls a SIGNAL the program writes into host memory at its end; per-iteration time = (end - start) / K on the
 * CPU's clock. Back-to-back iterations stay in step through the data (a rank waits for the peer's data of the
 * same iteration); scratch buffers alternate with the iteration's parity so a peer one iteration ahead never
 * overwrites data still being reduced. The local part of a result (e.g. a rank's own block of an all-gather)
 * is not copied, as in rdma_coll.
 *
 *   ag         all-gather: S per rank -> 2S        COPY in -> peer out[k]; SIGNAL; WAIT
 *   a2a        all-to-all: S per rank, S/2 each    COPY in[peer half] -> peer out[k half]; SIGNAL; WAIT
 *   rs         reduce-scatter: S -> S/2            COPY in[peer half] -> peer scratch; SIGNAL; WAIT; REDUCE
 *   ar_ring    all-reduce, ring (= rs + ag)        rs; COPY reduced half -> peer; SIGNAL; WAIT
 *   ar_direct  all-reduce, direct                  COPY in -> peer scratch; SIGNAL; WAIT; REDUCE (all S)
 *   bc         broadcast from rank 0, with the receiver's ack (both systems): rank 0 COPY in -> peer; SIGNAL;
 *              WAIT ack; rank 1 WAIT; SIGNAL ack
 *
 * V80 layout per rank (the window is exported whole; the peer writes scratch, out and flags):
 *   [in S | scratch0 S | scratch1 S | out 2S | flags 4 KiB]   (at most the 128 MiB window: S <= 16 MiB)
 * The receiver checks the last iteration's result.
 *
 * --engine ptdma (2026-10-19): this rank's data is in HOST memory (same layout, exported through Loom the same way)
 * and an AMD PTDMA engine pushes it (ptdma_q, examples/loom_switch/ptdma: one submit per step = the data copy into
 * the window, then an 8 B flag copy behind it on the same queue); the CPU runs the steps itself: it waits for the
 * peer's flag in host memory and does the reduction (int32, one thread). The steps are the V80 program's. Either
 * rank can use either engine (same layout). --verify (ptdma ranks): every pushed block carries the iteration number
 * in its first and last word, and the receiver checks them as soon as the flag arrives (a flag that overtook its
 * data shows); the input changes, so the final result check is skipped and verify runs are not for timing.
 * --acked (ptdma ranks): every push waits until the engine has finished it (the source may be reused then), the
 * counterpart of rdma_coll --acked (every WRITE ACKed by the peer).
 * bc variants (2026-10-19, ptdma ranks, both ranks the same):
 *   --ack-store  (a) rank 1's ack is an 8 B CPU store into the uwin (mapUwin, store + sfence) instead of a PTDMA copy;
 *                only for a signal that follows no data from that rank (bc's ack), so nothing it could overtake
 *   --stamp      (b) no flag behind the data: the payload's last 8 B carry the iteration number and rank 1 polls them
 *                (one QP lands in order); the ack as in (a). --verify stamps the first word of every 64 B line too and
 *                checks them all once the last word shows the iteration; the result check expects the stamp in the last 8 B
 *
 *   amy:   ce_coll --server --coll C --size S [--iters K] [--runs R] [--port P]          (rank 0)
 *   rose:  ce_coll --client <amy ip> --coll C --size S [--iters K] [--runs R] [--port P] [--out FILE]
 *          [--engine v80|ptdma] [--ptq /dev/ptq_0000_e8_00_2] [--verify] [--acked] [--ptq-user] [--ack-store | --stamp]
 */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <coyote/cThread.hpp>
#include "ce_prog.hpp"
#include "land_v80.hpp"
#include "loom_switch.hpp"
#include "ptq.hpp"

namespace {

constexpr uint32_t MAGIC = 0x434F4C4C;      // "COLL"
constexpr uint64_t STAGING_SIZE = 1ULL << 20;

struct Hello { uint32_t magic; uint32_t dst_ctid; uint64_t staging_va, ref, size, iters, coll; int64_t runs; uint64_t bcmode; };

bool write_full(int fd, const void *p, size_t n) {
    const char *c = static_cast<const char *>(p);
    while (n) { ssize_t k = ::write(fd, c, n); if (k <= 0) return false; c += k; n -= size_t(k); }
    return true;
}
bool read_full(int fd, void *p, size_t n) {
    char *c = static_cast<char *>(p);
    while (n) { ssize_t k = ::read(fd, c, n); if (k <= 0) return false; c += k; n -= size_t(k); }
    return true;
}
int connect_or_accept(const std::string &ip, uint16_t port) {
    int fd = -1;
    if (ip.empty()) {
        int lfd = ::socket(AF_INET, SOCK_STREAM, 0), one = 1;
        ::setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in a{};
        a.sin_family = AF_INET; a.sin_addr.s_addr = INADDR_ANY; a.sin_port = htons(port);
        if (::bind(lfd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0 || ::listen(lfd, 1) < 0) return -1;
        fd = ::accept(lfd, nullptr, nullptr);
        ::close(lfd);
    } else {
        for (int i = 0; i < 300 && fd < 0; i++) {
            fd = ::socket(AF_INET, SOCK_STREAM, 0);
            sockaddr_in a{};
            a.sin_family = AF_INET; a.sin_port = htons(port);
            ::inet_pton(AF_INET, ip.c_str(), &a.sin_addr);
            if (::connect(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0) { ::close(fd); fd = -1; usleep(100000); }
        }
    }
    return fd;
}
double pct(std::vector<double> v, double p) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, size_t(p / 100.0 * double(v.size())))];
}
uint64_t pages(uint64_t b) { return std::max<uint64_t>(4096, (b + 4095) & ~4095ULL); }

enum Coll { AG, A2A, RS, AR_RING, AR_DIRECT, BC, NCOLL };
const char *NAMES[NCOLL] = {"ag", "a2a", "rs", "ar_ring", "ar_direct", "bc"};
int32_t input(int rank, uint64_t j) { return int32_t(uint32_t(rank + 1) * 1000003u + uint32_t(j) * 7u); }

// a ptdma rank's REDUCE (the V80's: dst = a + b, int32 lanes), one thread
__attribute__((target("avx2"))) void reduce_cpu(const int32_t *a, const int32_t *b, int32_t *o, uint64_t bytes) {
    for (uint64_t j = 0; j < bytes / 4; j++) o[j] = a[j] + b[j];
}

}  // namespace

int main(int argc, char **argv) {
    std::string ip, out, cname, engine = "v80", ptq_dev = "/dev/ptq_0000_e8_00_2";
    bool server = false, verify = false, acked = false, ptq_user = false, ack_store = false, stamp_mode = false;
    uint16_t port = 19880;
    uint64_t S = 65536;
    int K = 0, runs = 6;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--server")                       server = true;
        else if (a == "--client" && i + 1 < argc)  ip = argv[++i];
        else if (a == "--port" && i + 1 < argc)    port = uint16_t(atoi(argv[++i]));
        else if (a == "--coll" && i + 1 < argc)    cname = argv[++i];
        else if (a == "--size" && i + 1 < argc)    S = strtoull(argv[++i], nullptr, 0);
        else if (a == "--iters" && i + 1 < argc)   K = atoi(argv[++i]);
        else if (a == "--runs" && i + 1 < argc)    runs = atoi(argv[++i]);
        else if (a == "--out" && i + 1 < argc)     out = argv[++i];
        else if (a == "--engine" && i + 1 < argc)  engine = argv[++i];
        else if (a == "--ptq" && i + 1 < argc)     ptq_dev = argv[++i];
        else if (a == "--verify")                  verify = true;
        else if (a == "--acked")                   acked = true;
        else if (a == "--ptq-user")                ptq_user = true;
        else if (a == "--ack-store")               ack_store = true;
        else if (a == "--stamp")                   stamp_mode = ack_store = true;
        else { server = false; ip.clear(); break; }
    }
    const bool cpu = engine == "ptdma";
    int c = int(std::find(NAMES, NAMES + NCOLL, cname) - NAMES);
    const uint64_t IN = 0, SCR0 = S, SCR1 = 2 * S, OUT = 3 * S, FLG = pages(5 * S), VT = FLG + 4096;
    if (!K) K = S <= 65536 ? 200 : S <= (1ULL << 20) ? 50 : 10;
    const int per_iter = 7;                                   // at most (ar_ring)
    if (server == !ip.empty() || c == NCOLL || S % 128 || S == 0 || runs < 2 || K < 1 || K * per_iter + 2 > int(ce_prog::N_PROG) ||
        VT > land_v80::UWIN_SIZE - 4096 || (engine != "v80" && !cpu) || ((verify || acked) && !cpu) || (ack_store && (!cpu || c != BC))) {
        printf("usage: %s --server | --client <ip> --coll ag|a2a|rs|ar_ring|ar_direct|bc --size S (multiple of 128, <= 16 MiB)"
               " [--iters K <= %d] [--runs R >= 2] [--port P] [--out FILE] [--engine v80|ptdma] [--ptq DEV] [--verify (ptdma)]"
               " [--acked (ptdma)] [--ptq-user] [--ack-store | --stamp (ptdma, bc)]\n",
               argv[0], int(ce_prog::N_PROG - 2) / per_iter);
        return 2;
    }
    const int k = server ? 0 : 1, p = 1 - k;
    const uint64_t half = S / 2;

    coyote::cThread t_qp(0, getpid(), 0, nullptr, "loom_fpga");
    coyote::cThread t_data(0, getpid(), 0, nullptr, "loom_fpga");
    std::unique_ptr<coyote::cThread> v80;
    if (!cpu) v80 = std::make_unique<coyote::cThread>(0, getpid(), 0, nullptr, "loom_versal_fpga");
    void *staging = server ? t_qp.initRDMA(STAGING_SIZE, port) : t_qp.initRDMA(STAGING_SIZE, port, ip.c_str());
    if (!staging) { printf("FAIL: initRDMA\n"); return 1; }

    // this rank's region, exported whole through Loom (export 1): V80 memory, or host memory for a ptdma rank
    std::unique_ptr<land_v80::Landing> L;
    uint8_t *hbuf = nullptr;
    if (!cpu) {
        L = std::make_unique<land_v80::Landing>(*v80, VT);
        int32_t *in = reinterpret_cast<int32_t *>(L->buf);
        for (uint64_t j = 0; j < S / 4; j++) in[j] = input(k, j);
        v80->invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{L->buf, VT});
        void *lva = loom_switch::reserve_va(VT);
        t_data.importDmabuf(L->export_fd(), lva);
        loom_switch::program_export(t_qp, 1, t_data.getCtid(), lva, VT);
    } else {
        hbuf = static_cast<uint8_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, VT}));
        if (!hbuf) { printf("FAIL: no host memory\n"); return 1; }
        memset(hbuf, 0, VT);
        int32_t *in = reinterpret_cast<int32_t *>(hbuf);
        for (uint64_t j = 0; j < S / 4; j++) in[j] = input(k, j);
        loom_switch::program_export(t_qp, 1, t_data.getCtid(), hbuf, VT);
    }

    const int fd = connect_or_accept(server ? std::string() : ip, uint16_t(port + 1));
    Hello mine{MAGIC, uint32_t(t_data.getCtid()), reinterpret_cast<uint64_t>(staging), loom_switch::export_ref(1), S,
               uint64_t(K), uint64_t(c), runs, uint64_t(ack_store) + 2 * uint64_t(stamp_mode)}, h{};
    if (fd < 0 || !write_full(fd, &mine, sizeof(mine)) || !read_full(fd, &h, sizeof(h)) || h.magic != MAGIC || h.size != S ||
        h.iters != uint64_t(K) || h.coll != uint64_t(c) || h.runs != runs || h.bcmode != mine.bcmode) {
        printf("FAIL: hello (--coll, --size, --iters, --runs, --ack-store and --stamp must match on both sides)\n");
        return 1;
    }
    // the peer's region, through window 1 = uwin [0, VT): imported into this V80, or the target of this host's PTDMA
    loom_switch::csr_write(t_qp, loom_switch::RDMA_STAGING_VA, h.staging_va);
    loom_switch::program_window(t_qp, 1, true, t_qp.getCtid(), reinterpret_cast<void *>(h.ref), VT, 0, h.dst_ctid);
    if (ack_store && k == 1) t_qp.mapUwin(VT);   // bc (a)/(b): rank 1's ack = a CPU store through window 1

    std::unique_ptr<ce_prog::Program> pr;
    volatile uint64_t *done = nullptr;
    // ptdma rank: the queue, its regions (this rank's host region, the uwin, the flag sources) - destroyed before t_data
    std::unique_ptr<ptq::Queue> q;
    uint32_t hb = 0, hu = 0, hf = 0;
    constexpr uint32_t FSLOTS = 512;            // > the ring: a flag's source word is not rewritten while it is queued
    uint64_t *fsrc = nullptr;
    uint32_t fslot = 0;
    if (!cpu) {
        void *uva = loom_switch::reserve_va(VT);
        v80->importDmabuf(t_qp.exportDmabuf(EXPORT_REGION_UWIN, 0, VT), uva);
        uint8_t *peer = static_cast<uint8_t *>(uva);
        uint8_t *me = reinterpret_cast<uint8_t *>(L->buf);       // card VAs
        uint64_t *my_flag = reinterpret_cast<uint64_t *>(me + FLG), *my_flag2 = my_flag + 1;
        done = static_cast<uint64_t *>(v80->getMem({coyote::CoyoteAllocType::HPF, 4096}));   // host memory
        const_cast<uint64_t &>(done[0]) = 0;

        pr = std::make_unique<ce_prog::Program>(*v80);
        for (int i = 1; i <= K; i++) {
            const uint64_t scr = (i & 1) ? SCR1 : SCR0;
            switch (c) {
            case AG:
                pr->add(ce_prog::copy(me + IN, peer + OUT + k * S, S));
                pr->add(ce_prog::signal(peer + FLG, i));
                pr->add(ce_prog::wait(my_flag, i));
                break;
            case A2A:
                pr->add(ce_prog::copy(me + IN + p * half, peer + OUT + k * half, half));
                pr->add(ce_prog::signal(peer + FLG, i));
                pr->add(ce_prog::wait(my_flag, i));
                break;
            case RS:
            case AR_RING:
                pr->add(ce_prog::copy(me + IN + p * half, peer + scr, half));
                pr->add(ce_prog::signal(peer + FLG, i));
                pr->add(ce_prog::wait(my_flag, i));
                pr->add(ce_prog::reduce(me + IN + k * half, me + scr, me + OUT + (c == AR_RING ? k * half : 0), half, true));
                if (c == AR_RING) {
                    pr->add(ce_prog::copy(me + OUT + k * half, peer + OUT + k * half, half));
                    pr->add(ce_prog::signal(peer + FLG + 8, i));
                    pr->add(ce_prog::wait(my_flag2, i));
                }
                break;
            case AR_DIRECT:
                pr->add(ce_prog::copy(me + IN, peer + scr, S));
                pr->add(ce_prog::signal(peer + FLG, i));
                pr->add(ce_prog::wait(my_flag, i));
                pr->add(ce_prog::reduce(me + IN, me + scr, me + OUT, S, true));
                break;
            case BC:
                if (k == 0) {
                    pr->add(ce_prog::copy(me + IN, peer + OUT, S));
                    pr->add(ce_prog::signal(peer + FLG, i));
                    pr->add(ce_prog::wait(my_flag2, i));
                } else {
                    pr->add(ce_prog::wait(my_flag, i));
                    pr->add(ce_prog::signal(peer + FLG + 8, i));
                }
                break;
            }
        }
        pr->add(ce_prog::signal(const_cast<uint64_t *>(done), K));   // host memory: the CPU's end of a run
        pr->load();
    } else {
        q = std::make_unique<ptq::Queue>(ptq_dev);
        fsrc = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, FSLOTS * 8}));
        if (!fsrc) { printf("FAIL: no host memory\n"); return 1; }
        hb = q->reg_host(hbuf, VT);
        hf = q->reg_host(fsrc, FSLOTS * 8);
        uint64_t ulen = 0;
        hu = q->reg_uwin(&ulen);
        // --ptq-user: descriptors and tail written from user space (no syscall per push; amy, rose)
        if (ptq_user && !q->enable_user()) { printf("FAIL: PTDMA user mode refused (no translating IOMMU?)\n"); return 1; }
        if (ulen < VT) { printf("FAIL: the uwin (%lu B) is smaller than the region\n", (unsigned long) ulen); return 1; }
    }
    printf("%s (rank %d, %s): up; %s, %lu B per rank, %d iterations a run, %d runs (the first a warm-up)%s%s%s\n",
           server ? "server" : "client", k, cpu ? ("ptdma " + ptq_dev).c_str() : "v80", NAMES[c], (unsigned long) S, K, runs,
           pr ? (", " + std::to_string(pr->size()) + " entries").c_str() : "", verify ? ", VERIFY (not for timing)" : acked ? ", ACKED (every push waits for its engine)" : "",
           stamp_mode ? ", bc (b): stamped last word, no flag, ack = CPU store" : ack_store ? ", bc (a): ack = CPU store" : "");

    // ---- the ptdma rank's steps (the V80 program's, run by the CPU) ----
    bool timeout = false;
    uint64_t vbad = 0;
    auto stamp = [&](uint64_t off, uint64_t len, uint64_t v) {
        int32_t *w = reinterpret_cast<int32_t *>(hbuf + off);
        w[0] = w[len / 4 - 1] = int32_t(v);
    };
    auto vcheck = [&](uint64_t off, uint64_t len, uint64_t v) {   // what the peer pushed for iteration v (or later)
        const volatile int32_t *w = reinterpret_cast<int32_t *>(hbuf + off);
        if (stamp_mode) {   // (b): every line's first word (the last word was the signal)
            for (uint64_t j = 0; j < len / 4; j += 16) if (w[j] < int32_t(v)) { vbad++; return; }
            return;
        }
        if (w[0] < int32_t(v) || w[len / 4 - 1] < int32_t(v)) vbad++;
    };
    auto push = [&](uint64_t soff, uint64_t doff, uint64_t len, uint64_t fdoff, uint64_t v) {   // data, then its flag
        if (stamp_mode) {   // bc (b): one copy, its last 8 B = the iteration (the receiver polls them)
            if (verify) for (uint64_t l = 0; l < len; l += 64) *reinterpret_cast<int32_t *>(hbuf + soff + l) = int32_t(v);
            *reinterpret_cast<uint64_t *>(hbuf + soff + len - 8) = v;
            const uint64_t sq_ = q->copy(hb, soff, hu, doff, len);
            if (acked) while (!q->done(sq_)) {}
            return;
        }
        if (verify) stamp(soff, len, v);
        fsrc[fslot] = v;
        ptq_copy cs[2] = {{hb, hu, soff, doff, len}, {hf, hu, uint64_t(fslot) * 8, fdoff, 8}};
        fslot = (fslot + 1) % FSLOTS;
        const uint64_t sq_ = q->submit(cs, 2);
        if (acked) while (!q->done(sq_)) {}   // --acked: the engine has written data and flag out (the source is free)
    };
    auto sig = [&](uint64_t fdoff, uint64_t v) {
        if (ack_store) { t_qp.uwinWrite(fdoff, &v, 8); return; }   // bc (a)/(b): 8 B store + sfence, nothing to complete
        fsrc[fslot] = v;
        const uint64_t sq_ = q->copy(hf, uint64_t(fslot) * 8, hu, fdoff, 8);
        fslot = (fslot + 1) % FSLOTS;
        if (acked) while (!q->done(sq_)) {}
    };
    auto spin = [&](uint64_t off, uint64_t v) {
        const volatile uint64_t *w = reinterpret_cast<uint64_t *>(hbuf + off);
        const auto tw = std::chrono::steady_clock::now();
        while (*w < v)
            if (std::chrono::steady_clock::now() - tw > std::chrono::seconds(10)) { timeout = true; return; }
    };
    auto red = [&](uint64_t a, uint64_t b, uint64_t o, uint64_t len) {
        reduce_cpu(reinterpret_cast<int32_t *>(hbuf + a), reinterpret_cast<int32_t *>(hbuf + b), reinterpret_cast<int32_t *>(hbuf + o), len);
    };
    auto cpu_iter = [&](int i, uint64_t v) {
        const uint64_t scr = (i & 1) ? SCR1 : SCR0;
        switch (c) {
        case AG:
            push(IN, OUT + k * S, S, FLG, v); spin(FLG, v); if (verify) vcheck(OUT + p * S, S, v); break;
        case A2A:
            push(IN + p * half, OUT + k * half, half, FLG, v); spin(FLG, v); if (verify) vcheck(OUT + p * half, half, v); break;
        case RS:
        case AR_RING:
            push(IN + p * half, scr, half, FLG, v); spin(FLG, v);
            if (timeout) return;
            if (verify) vcheck(scr, half, v);
            red(IN + k * half, scr, OUT + (c == AR_RING ? k * half : 0), half);
            if (c == AR_RING) { push(OUT + k * half, OUT + k * half, half, FLG + 8, v); spin(FLG + 8, v); if (verify) vcheck(OUT + p * half, half, v); }
            break;
        case AR_DIRECT:
            push(IN, scr, S, FLG, v); spin(FLG, v);
            if (timeout) return;
            if (verify) vcheck(scr, S, v);
            red(IN, scr, OUT, S);
            break;
        case BC:
            if (k == 0) { push(IN, OUT, S, FLG, v); spin(FLG + 8, v); }
            else        { spin(stamp_mode ? OUT + S - 8 : FLG, v); if (verify) vcheck(OUT, S, v); sig(FLG + 8, v); }
            break;
        }
    };
    auto drain = [&](std::chrono::milliseconds tmo) {      // every queued copy finished
        const auto tw = std::chrono::steady_clock::now();
        while (!q->done(q->seq())) if (std::chrono::steady_clock::now() - tw > tmo) return false;
        return true;
    };

    std::vector<double> per_iter_us;
    int errors = 0;
    for (int r = 0; r < runs; r++) {
        char go = 1;   // both ranks start right after this exchange
        if (server ? !write_full(fd, &go, 1) || !read_full(fd, &go, 1) : !read_full(fd, &go, 1) || !write_full(fd, &go, 1)) {
            printf("FAIL: barrier\n"); return 1;
        }
        const uint64_t base = uint64_t(r) * uint64_t(K);
        const uint64_t want = base + uint64_t(K);
        const auto t0 = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point t1;
        if (cpu) {
            for (int i = 1; i <= K && !timeout; i++) cpu_iter(i, base + uint64_t(i));
            t1 = std::chrono::steady_clock::now();
            if (!drain(std::chrono::milliseconds(2000))) timeout = true;
            if (timeout) {
                const ptq_info qi = q->info();
                printf("FAIL run %d: timed out (flags %lu/%lu, queue %lu of %lu done, status %#x)\n", r,
                       (unsigned long) reinterpret_cast<volatile uint64_t *>(hbuf + FLG)[0],
                       (unsigned long) reinterpret_cast<volatile uint64_t *>(hbuf + FLG)[1], (unsigned long) qi.completed,
                       (unsigned long) qi.submitted, qi.status);
                errors++; break;
            }
        } else {
            pr->start(base);
            while (done[0] < want)
                if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(r ? 20 : 60)) break;
            t1 = std::chrono::steady_clock::now();
            // always end the program before its buffers are touched: on a timeout finish() ABORTs it (a WAIT left polling
            // while L.pull() or the exit drops the card mapping page-faults the V80 and kills its driver, rdma_coll 10-11)
            const bool fin = pr->finish(std::chrono::milliseconds(done[0] < want ? 10 : 1000));
            if (done[0] < want || !fin) {
                printf("FAIL run %d: did not finish (entry %lu, engine %s)\n", r, (unsigned long) v80->getCSR(ce_prog::PC),
                       v80->getCSR(ce_prog::BUSY) ? "STILL BUSY" : "stopped");
                errors++; break;
            }
        }
        const double us = std::chrono::duration<double, std::micro>(t1 - t0).count() / K;
        printf("%s run %d: %.2f us per %s\n", r ? "    " : "warm", r, us, NAMES[c]);
        if (r) per_iter_us.push_back(us);
        char d = 1;
        if (!write_full(fd, &d, 1) || !read_full(fd, &d, 1)) { printf("FAIL: barrier\n"); return 1; }
    }

    // the last iteration's result on this rank (never while an engine runs: see above)
    const uint8_t *res = hbuf;
    if (!cpu) {
        if (v80->getCSR(ce_prog::BUSY)) { printf("FAIL: the engine did not stop; its buffers are left alone\n"); return 1; }
        L->pull();
        res = reinterpret_cast<uint8_t *>(L->buf);
    } else if (!drain(std::chrono::milliseconds(2000))) {
        printf("FAIL: the PTDMA queue did not drain; its buffers are left alone\n"); return 1;
    }
    const int32_t *o = reinterpret_cast<const int32_t *>(res + OUT);
    uint64_t bad = 0, n = 0;
    auto chk = [&](uint64_t idx, int32_t want) { n++; bad += (o[idx] != want); };
    if (verify) {
        printf("%s: verify: %lu blocks seen before their data (flag ahead of data)\n", server ? "server" : "client", (unsigned long) vbad);
        errors += vbad != 0;
    } else {
        switch (c) {
        case AG:        for (uint64_t j = 0; j < S / 4; j++) chk(p * S / 4 + j, input(p, j)); break;
        case A2A:       for (uint64_t j = 0; j < half / 4; j++) chk(p * half / 4 + j, input(p, k * half / 4 + j)); break;
        case RS:        for (uint64_t j = 0; j < half / 4; j++) chk(j, input(k, k * half / 4 + j) + input(p, k * half / 4 + j)); break;
        case AR_RING:
        case AR_DIRECT: for (uint64_t j = 0; j < S / 4; j++) chk(j, input(0, j) + input(1, j)); break;
        case BC:
            if (k != 1) break;
            for (uint64_t j = 0; j < S / 4 - (stamp_mode ? 2 : 0); j++) chk(j, input(0, j));
            if (stamp_mode) { n++; bad += *reinterpret_cast<const uint64_t *>(res + OUT + S - 8) != uint64_t(runs) * uint64_t(K); }
            break;
        }
        printf("%s: %lu of %lu result words wrong\n", server ? "server" : "client", (unsigned long) bad, (unsigned long) n);
        errors += bad != 0;
    }
    if (!per_iter_us.empty()) {
        printf("%s %s %lu B (us per collective, %zu runs, median [min max]): %.2f [%.2f %.2f]\n", server ? "server" : "client",
               NAMES[c], (unsigned long) S, per_iter_us.size(), pct(per_iter_us, 50), pct(per_iter_us, 0), pct(per_iter_us, 100));
        if (!out.empty()) {
            FILE *f = fopen(out.c_str(), "w");
            if (f) { for (double v : per_iter_us) fprintf(f, "%.3f\n", v); fclose(f); }
        }
    }
    ::close(fd);
    q.reset();      // unmaps this rank's buffers from the engine before they are freed
    loom_switch::release_window(t_qp, 1);
    loom_switch::release_export(t_qp, 1);
    t_qp.connSync(!server);
    printf(errors ? "COLL FAIL\n" : "COLL PASS\n");
    return errors ? 1 : 0;
}
