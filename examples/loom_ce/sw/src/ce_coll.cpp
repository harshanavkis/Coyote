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
 *   amy:   ce_coll --server --coll C --size S [--iters K] [--runs R] [--port P]          (rank 0)
 *   rose:  ce_coll --client <amy ip> --coll C --size S [--iters K] [--runs R] [--port P] [--out FILE]
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

namespace {

constexpr uint32_t MAGIC = 0x434F4C4C;      // "COLL"
constexpr uint64_t STAGING_SIZE = 1ULL << 20;

struct Hello { uint32_t magic; uint32_t dst_ctid; uint64_t staging_va, ref, size, iters, coll; int64_t runs; };

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

}  // namespace

int main(int argc, char **argv) {
    std::string ip, out, cname;
    bool server = false;
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
        else { server = false; ip.clear(); break; }
    }
    int c = int(std::find(NAMES, NAMES + NCOLL, cname) - NAMES);
    const uint64_t IN = 0, SCR0 = S, SCR1 = 2 * S, OUT = 3 * S, FLG = pages(5 * S), VT = FLG + 4096;
    if (!K) K = S <= 65536 ? 200 : S <= (1ULL << 20) ? 50 : 10;
    const int per_iter = 7;                                   // at most (ar_ring)
    if (server == !ip.empty() || c == NCOLL || S % 128 || S == 0 || runs < 2 || K < 1 || K * per_iter + 2 > int(ce_prog::N_PROG) ||
        VT > land_v80::UWIN_SIZE - 4096) {
        printf("usage: %s --server | --client <ip> --coll ag|a2a|rs|ar_ring|ar_direct|bc --size S (multiple of 128, <= 16 MiB)"
               " [--iters K <= %d] [--runs R >= 2] [--port P] [--out FILE]\n", argv[0], int(ce_prog::N_PROG - 2) / per_iter);
        return 2;
    }
    const int k = server ? 0 : 1, p = 1 - k;
    const uint64_t half = S / 2;

    coyote::cThread t_qp(0, getpid(), 0, nullptr, "coyote_fpga");
    coyote::cThread t_data(0, getpid(), 0, nullptr, "coyote_fpga");
    coyote::cThread v80(0, getpid(), 0, nullptr, "coyote_versal_fpga");
    void *staging = server ? t_qp.initRDMA(STAGING_SIZE, port) : t_qp.initRDMA(STAGING_SIZE, port, ip.c_str());
    if (!staging) { printf("FAIL: initRDMA\n"); return 1; }

    // this rank's V80 region, exported whole through Loom (export 1)
    land_v80::Landing L(v80, VT);
    int32_t *in = reinterpret_cast<int32_t *>(L.buf);
    for (uint64_t j = 0; j < S / 4; j++) in[j] = input(k, j);
    v80.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{L.buf, VT});
    void *lva = loom_switch::reserve_va(VT);
    t_data.importDmabuf(L.export_fd(), lva);
    loom_switch::program_export(t_qp, 1, t_data.getCtid(), lva, VT);

    const int fd = connect_or_accept(server ? std::string() : ip, uint16_t(port + 1));
    Hello mine{MAGIC, uint32_t(t_data.getCtid()), reinterpret_cast<uint64_t>(staging), loom_switch::export_ref(1), S,
               uint64_t(K), uint64_t(c), runs}, h{};
    if (fd < 0 || !write_full(fd, &mine, sizeof(mine)) || !read_full(fd, &h, sizeof(h)) || h.magic != MAGIC || h.size != S ||
        h.iters != uint64_t(K) || h.coll != uint64_t(c) || h.runs != runs) {
        printf("FAIL: hello (--coll, --size, --iters and --runs must match on both sides)\n");
        return 1;
    }
    // the peer's region, through a window imported into this V80
    loom_switch::csr_write(t_qp, loom_switch::RDMA_STAGING_VA, h.staging_va);
    loom_switch::program_window(t_qp, 1, true, t_qp.getCtid(), reinterpret_cast<void *>(h.ref), VT, 0, h.dst_ctid);
    void *uva = loom_switch::reserve_va(VT);
    v80.importDmabuf(t_qp.exportDmabuf(EXPORT_REGION_UWIN, 0, VT), uva);
    uint8_t *peer = static_cast<uint8_t *>(uva);
    uint8_t *me = reinterpret_cast<uint8_t *>(L.buf);       // card VAs
    uint64_t *my_flag = reinterpret_cast<uint64_t *>(me + FLG), *my_flag2 = my_flag + 1;
    volatile uint64_t *done = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, 4096}));   // host memory
    const_cast<uint64_t &>(done[0]) = 0;

    ce_prog::Program pr(v80);
    for (int i = 1; i <= K; i++) {
        const uint64_t scr = (i & 1) ? SCR1 : SCR0;
        switch (c) {
        case AG:
            pr.add(ce_prog::copy(me + IN, peer + OUT + k * S, S));
            pr.add(ce_prog::signal(peer + FLG, i));
            pr.add(ce_prog::wait(my_flag, i));
            break;
        case A2A:
            pr.add(ce_prog::copy(me + IN + p * half, peer + OUT + k * half, half));
            pr.add(ce_prog::signal(peer + FLG, i));
            pr.add(ce_prog::wait(my_flag, i));
            break;
        case RS:
        case AR_RING:
            pr.add(ce_prog::copy(me + IN + p * half, peer + scr, half));
            pr.add(ce_prog::signal(peer + FLG, i));
            pr.add(ce_prog::wait(my_flag, i));
            pr.add(ce_prog::reduce(me + IN + k * half, me + scr, me + OUT + (c == AR_RING ? k * half : 0), half, true));
            if (c == AR_RING) {
                pr.add(ce_prog::copy(me + OUT + k * half, peer + OUT + k * half, half));
                pr.add(ce_prog::signal(peer + FLG + 8, i));
                pr.add(ce_prog::wait(my_flag2, i));
            }
            break;
        case AR_DIRECT:
            pr.add(ce_prog::copy(me + IN, peer + scr, S));
            pr.add(ce_prog::signal(peer + FLG, i));
            pr.add(ce_prog::wait(my_flag, i));
            pr.add(ce_prog::reduce(me + IN, me + scr, me + OUT, S, true));
            break;
        case BC:
            if (k == 0) {
                pr.add(ce_prog::copy(me + IN, peer + OUT, S));
                pr.add(ce_prog::signal(peer + FLG, i));
                pr.add(ce_prog::wait(my_flag2, i));
            } else {
                pr.add(ce_prog::wait(my_flag, i));
                pr.add(ce_prog::signal(peer + FLG + 8, i));
            }
            break;
        }
    }
    pr.add(ce_prog::signal(const_cast<uint64_t *>(done), K));   // host memory: the CPU's end of a run
    pr.load();
    printf("%s (rank %d): up; %s, %lu B per rank, %d iterations a run, %d runs (the first a warm-up), %zu entries\n",
           server ? "server" : "client", k, NAMES[c], (unsigned long) S, K, runs, pr.size());

    std::vector<double> per_iter_us;
    int errors = 0;
    for (int r = 0; r < runs; r++) {
        char go = 1;   // both ranks start right after this exchange
        if (server ? !write_full(fd, &go, 1) || !read_full(fd, &go, 1) : !read_full(fd, &go, 1) || !write_full(fd, &go, 1)) {
            printf("FAIL: barrier\n"); return 1;
        }
        const uint64_t base = uint64_t(r) * uint64_t(K);
        const auto t0 = std::chrono::steady_clock::now();
        pr.start(base);
        const uint64_t want = base + uint64_t(K);
        while (done[0] < want)
            if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(r ? 20 : 60)) break;
        const auto t1 = std::chrono::steady_clock::now();
        if (done[0] < want || !pr.finish(std::chrono::milliseconds(1000))) {
            printf("FAIL run %d: did not finish (entry %lu)\n", r, (unsigned long) v80.getCSR(ce_prog::PC));
            errors++; break;
        }
        const double us = std::chrono::duration<double, std::micro>(t1 - t0).count() / K;
        printf("%s run %d: %.2f us per %s\n", r ? "    " : "warm", r, us, NAMES[c]);
        if (r) per_iter_us.push_back(us);
        char d = 1;
        if (!write_full(fd, &d, 1) || !read_full(fd, &d, 1)) { printf("FAIL: barrier\n"); return 1; }
    }

    // the last iteration's result on this rank
    L.pull();
    const int32_t *o = reinterpret_cast<const int32_t *>(reinterpret_cast<uint8_t *>(L.buf) + OUT);
    uint64_t bad = 0, n = 0;
    auto chk = [&](uint64_t idx, int32_t want) { n++; bad += (o[idx] != want); };
    switch (c) {
    case AG:        for (uint64_t j = 0; j < S / 4; j++) chk(p * S / 4 + j, input(p, j)); break;
    case A2A:       for (uint64_t j = 0; j < half / 4; j++) chk(p * half / 4 + j, input(p, k * half / 4 + j)); break;
    case RS:        for (uint64_t j = 0; j < half / 4; j++) chk(j, input(k, k * half / 4 + j) + input(p, k * half / 4 + j)); break;
    case AR_RING:
    case AR_DIRECT: for (uint64_t j = 0; j < S / 4; j++) chk(j, input(0, j) + input(1, j)); break;
    case BC:        if (k == 1) for (uint64_t j = 0; j < S / 4; j++) chk(j, input(0, j)); break;
    }
    printf("%s: %lu of %lu result words wrong\n", server ? "server" : "client", (unsigned long) bad, (unsigned long) n);
    errors += bad != 0;
    if (!per_iter_us.empty()) {
        printf("%s %s %lu B (us per collective, %zu runs, median [min max]): %.2f [%.2f %.2f]\n", server ? "server" : "client",
               NAMES[c], (unsigned long) S, per_iter_us.size(), pct(per_iter_us, 50), pct(per_iter_us, 0), pct(per_iter_us, 100));
        if (!out.empty()) {
            FILE *f = fopen(out.c_str(), "w");
            if (f) { for (double v : per_iter_us) fprintf(f, "%.3f\n", v); fclose(f); }
        }
    }
    ::close(fd);
    loom_switch::release_window(t_qp, 1);
    loom_switch::release_export(t_qp, 1);
    t_qp.connSync(!server);
    printf(errors ? "COLL FAIL\n" : "COLL PASS\n");
    return errors ? 1 : 0;
}
