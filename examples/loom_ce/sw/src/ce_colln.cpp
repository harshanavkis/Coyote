/**
 * ce_colln - collectives at N ranks on Loom (E5 at three ranks: amy, rose, clara), int32. ce_coll is the two-rank tool
 * the paper's two-rank numbers come from; ce_colln generalises it and runs the same steps at N = 2 too.
 *
 * Every pair of ranks has its own queue pair: on each U280 one QP-owner cThread per peer, and one window per peer
 * (window 1 + peer index, its QP = that cThread's). Each rank runs its steps with its own engine:
 *   --engine v80    one copy-engine program in V80 HBM (copies through the window, signals, waits, reductions), as ce_coll
 *   --engine ptdma  data in host memory; a PTDMA engine (ptdma_q) pushes each copy and the 8 B signal behind it in one
 *                   submit, the CPU waits for flags in host memory and reduces (int32, one thread), as ce_coll --engine ptdma
 *
 * Region of every rank (exported whole as export 1; the same layout on every rank):
 *   [in S | out S | tmp S | block(w) for every other rank w, in rank order]
 *   block(w) = [area0 B | area1 B | flags 4 KiB]: what rank w writes into this rank. A writer's window covers exactly its
 *   block in the peer (export_ref(1, offset of the block)), so N - 1 windows fit the 128 MiB uwin. area alternates with
 *   the iteration's parity (a peer one iteration ahead never overwrites data still in use).
 *   Chunks: C = S / N rounded up to 128 B; chunk j = [j C, min(S, (j + 1) C)) (the last one shorter or empty).
 *
 *   ag         copy in -> every peer's block(k).area (S); signal; wait for every peer's flag
 *   a2a        copy chunk q of in -> peer q's block(k).area; signal; wait all
 *   rs         copy chunk q of in -> peer q's block(k).area; signal; wait all; out[chunk k] = in[chunk k] + all received
 *   ar_direct  copy in -> every peer's block(k).area (S); signal; wait all; out = in + all received
 *   ar_ring    ring reduce-scatter then ring all-gather, N - 1 steps each, one chunk per step to the next rank only:
 *              block(prev).area = [rs slot 0..N-2 | ag slot 0..N-2] (C each), flags[t] = step t
 *   bc         rank 0: copy in -> every peer's block(0).area; signal; wait for every peer's ack; others: wait; ack
 *   shift      (2026-10-27, a traffic probe, not a collective) copy in -> next's block(k).area (S); signal; wait for
 *              prev's: every card sends to one peer and receives from another (= ag at 2 ranks)
 * Reductions chain through tmp and out (never in place). Time per collective = run time / K on each rank's clock; the
 * script reports the slowest rank. --verify (ptdma ranks): every pushed piece carries the iteration number in its first
 * and last word, checked when its flag arrives; the final result check is skipped then. --acked (ptdma ranks): every
 * push waits until the engine has finished it (the counterpart of rdma_colln --acked). --acked-step (2026-10-25, ptdma
 * ranks): the step's pushes to every peer are submitted first and the engine is waited for once, before the step's first
 * WAIT/REDUCE and at the end of an iteration (the counterpart of rdma_colln --acked-step).
 * bc variants (2026-10-19, ptdma ranks, every rank the same), as ce_coll:
 *   --ack-store  (a) a receiver's ack is an 8 B CPU store into the uwin (mapUwin, store + sfence), not a PTDMA copy; only
 *                for a signal that follows no data from that rank (bc's ack)
 *   --stamp      (b) no flag behind rank 0's data: the payload's last 8 B carry the iteration and the receiver polls them;
 *                the ack as in (a). --verify stamps every 64 B line's first word and checks them once the last word shows
 *                the iteration (and checks a bc receiver's block in every mode, also without --stamp)
 * --flag-in-data (2026-10-25, ptdma ranks, every collective): no SIGNAL behind a COPY; the COPY's last 8 B carry the
 *                iteration (written into the source before the submit, one PTDMA descriptor) and the receiver polls the last
 *                8 B of the block it receives. Blocks keep their length (8 B of payload become the flag); the result check
 *                skips the last 8 B of every block; --verify stamps a block's first word and counts a flag seen before it.
 *                bc's acknowledgement carries no data and stays a SIGNAL. (Same protocol as rdma_colln --flag-in-data.)
 *
 *
 * Several ranks on one host (ptdma ranks; a host's ranks share its U280; 2026-10-22, QP sharing): ranks with the same
 * --hosts entry are on one host; a rank's slot = its index among them, R = its peers on other hosts. Slot s uses export
 * 1 + s, windows 1 + s R + ridx and the uwin from s R BLK (ridx = the peer's index among the remote peers). Its region is a 2 MiB hugetlbfs file (/dev/hugepages/loomcolln_<port>_<rank>,
 * mapped shared), so a peer on the same host writes into it with the CPU: copy, sfence, the last 8 B, sfence, the flag
 * (no QP, no window). Remote peers as above, through the PTDMA engine of the rank (--ptq: one engine per rank).
 *   default     one QP per pair of ranks on different hosts (QP-owner cThread in each of the two processes)
 *   --share-qp  one QP per pair of HOSTS, owned by the first rank of each host; the host's other ranks program their
 *               windows onto that QP (its ctid comes in the hello); the far switch lands every packet by its export
 *   --window P  the card's ack window (TX_CTL: rdma packets unacked over all its QPs, 0 = no limit; 16 after a setup),
 *               written by the card's first rank; it stays until the next setup
 *
 *   ce_colln --rank r --ranks N --hosts ip0,ip1,... (management IPs) --coll C --size S [--iters K] [--runs R] [--port P]
 *            [--engine v80|ptdma] [--ptq DEV] [--verify] [--acked | --acked-step] [--ptq-user] [--ack-store | --stamp] [--share-qp]
 *            [--window P] [--out FILE]
 * Rank 0 is the hub of a TCP control mesh (hello, barriers) on port P; pair (i, j) builds its QP on port P + 2 + 2 * pair.
 */
#include <fcntl.h>
#include <immintrin.h>
#include <sys/mman.h>
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
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <coyote/cThread.hpp>
#include "ce_prog.hpp"
#include "land_v80.hpp"
#include "loom_switch.hpp"
#include "ptq.hpp"

namespace {

constexpr uint32_t MAGIC = 0x434F4C4E;      // "COLN"
constexpr uint64_t STAGING_SIZE = 1ULL << 20;
constexpr int MAXR = 8;

struct Hello { uint32_t magic, rank, dst_ctid, bcmode; uint64_t size, iters, coll; int64_t runs;
               uint32_t exp, share, qp_ctid[MAXR]; };   // export index, --share-qp, ctid of the QP to each rank (NO_QP = none)
constexpr uint32_t NO_QP = 0xFFFFFFFFu;   // ctid 0 is a real cThread
constexpr uint64_t HUGE2M = 2ULL << 20;
std::string region_file(uint16_t port, int rank) { return "/dev/hugepages/loomcolln_" + std::to_string(port) + "_" + std::to_string(rank); }

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
int listen_on(uint16_t port) {
    int lfd = ::socket(AF_INET, SOCK_STREAM, 0), one = 1;
    ::setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in a{};
    a.sin_family = AF_INET; a.sin_addr.s_addr = INADDR_ANY; a.sin_port = htons(port);
    if (::bind(lfd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0 || ::listen(lfd, MAXR) < 0) return -1;
    return lfd;
}
int connect_to(const std::string &ip, uint16_t port) {
    for (int i = 0; i < 600; i++) {
        int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in a{};
        a.sin_family = AF_INET; a.sin_port = htons(port);
        ::inet_pton(AF_INET, ip.c_str(), &a.sin_addr);
        if (::connect(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) == 0) return fd;
        ::close(fd);
        usleep(100000);
    }
    return -1;
}
double pct(std::vector<double> v, double p) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, size_t(p / 100.0 * double(v.size())))];
}
uint64_t pages(uint64_t b) { return std::max<uint64_t>(4096, (b + 4095) & ~4095ULL); }

enum Coll { AG, A2A, RS, AR_RING, AR_DIRECT, BC, SHIFT, NCOLL };
const char *NAMES[NCOLL] = {"ag", "a2a", "rs", "ar_ring", "ar_direct", "bc", "shift"};
int32_t input(int rank, uint64_t j) { return int32_t(uint32_t(rank + 1) * 1000003u + uint32_t(j) * 7u); }   // as ce_coll

__attribute__((target("avx2"))) void reduce_cpu(const int32_t *a, const int32_t *b, int32_t *o, uint64_t bytes) {
    for (uint64_t j = 0; j < bytes / 4; j++) o[j] = a[j] + b[j];
}

// One step of a rank's program. Offsets: local ones in this rank's region; dst of COPY / SIGNAL in peer `peer`'s region.
struct Op {
    enum T { COPY, SIGNAL, WAIT, REDUCE } t;
    int peer;
    uint64_t a, b, o, len;
};

}  // namespace

int main(int argc, char **argv) {
    std::string hosts_s, out, cname, engine = "v80", ptq_dev = "/dev/ptq_0000_e8_00_2";
    int k = -1, N = 0, K = 0, runs = 6, window = -1;
    bool verify = false, acked = false, acked_step = false, ptq_user = false, ack_store = false, stamp_mode = false, share_qp = false, fid = false;
    uint16_t port = 22000;
    uint64_t S = 65536;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--rank" && i + 1 < argc)         k = atoi(argv[++i]);
        else if (a == "--ranks" && i + 1 < argc)   N = atoi(argv[++i]);
        else if (a == "--hosts" && i + 1 < argc)   hosts_s = argv[++i];
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
        else if (a == "--acked-step")              acked_step = true;
        else if (a == "--ptq-user")                ptq_user = true;
        else if (a == "--ack-store")               ack_store = true;
        else if (a == "--stamp")                   stamp_mode = ack_store = true;
        else if (a == "--flag-in-data")            fid = true;
        else if (a == "--share-qp")                share_qp = true;
        else if (a == "--window" && i + 1 < argc)  window = atoi(argv[++i]);
        else { N = 0; break; }
    }
    std::vector<std::string> hosts;
    { std::stringstream ss(hosts_s); std::string h; while (std::getline(ss, h, ',')) if (!h.empty()) hosts.push_back(h); }
    // ranks on one host (the same --hosts entry): slot = index among them, lead = the first of them
    auto same = [&](int q) { return q != k && hosts[size_t(q)] == hosts[size_t(k)]; };
    auto lead = [&](int r) { int j = 0; while (hosts[size_t(j)] != hosts[size_t(r)]) j++; return j; };
    auto need_qp = [&](int i, int j) {   // does pair (i, j) build a QP
        return hosts[size_t(i)] != hosts[size_t(j)] && (!share_qp || (i == lead(i) && j == lead(j)));
    };
    int slot = 0, nslot = 0, nsame = 0;
    if (int(hosts.size()) == N && k >= 0 && k < N)
        for (int j = 0; j < N; j++) if (hosts[size_t(j)] == hosts[size_t(k)]) { nslot++; if (j < k) slot++; if (j != k) nsame++; }
    const int nrem = N - 1 - nsame;                       // peers on other hosts: one window each
    auto ridx = [&](int q) { int r = 0; for (int j = 0; j < q; j++) if (j != k && !same(j)) r++; return r; };
    const int c = int(std::find(NAMES, NAMES + NCOLL, cname) - NAMES);
    const bool cpu = engine == "ptdma";
    const uint64_t C = ((S + uint64_t(N > 0 ? N : 1) * 128 - 1) / (uint64_t(N > 0 ? N : 1) * 128)) * 128;   // chunk
    auto coff = [&](int j) { return std::min(S, uint64_t(j) * C); };
    auto clen = [&](int j) { return std::min(S, uint64_t(j + 1) * C) - coff(j); };
    // per-writer data area B, per collective
    uint64_t B = S;
    if (c == A2A || c == RS) B = C;
    else if (c == AR_RING) B = 2 * uint64_t(N - 1) * C;
    const uint64_t IN = 0, OUT = S, TMP = 2 * S, BLK0 = pages(3 * S), BLK = pages(2 * B + 4096);
    const uint64_t VT = BLK0 + uint64_t(N > 1 ? N - 1 : 0) * BLK;
    const uint64_t FLGS = 2 * B;    // flags' offset in a block
    if (!K) K = S <= 65536 ? 200 : S <= (1ULL << 20) ? 50 : 10;
    if (N < 2 || N > MAXR || k < 0 || k >= N || int(hosts.size()) != N || c == NCOLL || S % 128 || S == 0 || runs < 2 || K < 1 ||
        (engine != "v80" && !cpu) || ((verify || acked || acked_step) && !cpu) || (acked && acked_step) || VT > (cpu ? (1ULL << 30) : land_v80::UWIN_SIZE - 4096) ||
        uint64_t(nslot) * uint64_t(nrem) * BLK > (128ULL << 20) || 2 * uint64_t(N - 1) * 8 > 4096 ||
        (ack_store && (!cpu || c != BC)) || ((nslot > 1 || share_qp) && !cpu) || 1 + nslot * nrem > 16 || nrem < 1 ||
        (fid && (!cpu || stamp_mode))) {
        printf("usage: %s --rank r --ranks N --hosts ip0,ip1,... --coll ag|a2a|rs|ar_ring|ar_direct|bc|shift --size S (multiple of 128)"
               " [--iters K] [--runs R >= 2] [--port P] [--engine v80|ptdma] [--ptq DEV] [--verify] [--acked | --acked-step] [--ptq-user]"
               " [--ack-store | --stamp (ptdma, bc)] [--flag-in-data (ptdma)] [--share-qp (ptdma)] [--window P] [--out FILE] (several ranks on a host: ptdma only;"
               " region %lu B, windows of a card %lu B of 128 MiB)\n", argv[0], (unsigned long) VT,
               (unsigned long) (uint64_t(nslot) * uint64_t(nrem > 0 ? nrem : 0) * BLK));
        return 2;
    }
    const int next = (k + 1) % N, prev = (k + N - 1) % N;
    auto idx = [&](int q, int w) { return w < q ? w : w - 1; };             // w's block index in q's region
    auto blk = [&](int q, int w) { return BLK0 + uint64_t(idx(q, w)) * BLK; };

    // ---- the steps of one iteration i (parity par) ----
    auto program = [&](int i, std::vector<Op> &ops) {
        const uint64_t par = (i & 1) ? B : 0;
        // --flag-in-data: a copy carries its flag in its last 8 B (b = 1) and replaces copy + signal; its receiver waits on the
        // piece's last 8 B instead of the flag slot (bc's ack: sig_, a signal either way)
        auto copy = [&](uint64_t src, int q, uint64_t dst, uint64_t len) { if (len) ops.push_back({Op::COPY, q, src, fid ? 1u : 0u, dst, len}); };
        auto sig_ = [&](int q, uint64_t dst) { ops.push_back({Op::SIGNAL, q, 0, 0, dst, 8}); };
        auto sig  = [&](int q, uint64_t dst) { if (!fid) sig_(q, dst); };
        // a WAIT carries the piece its flag guards (b, len; len 0 = an ack): --verify checks that piece when the wait ends
        auto wait = [&](uint64_t a, uint64_t d, uint64_t dl) { ops.push_back({Op::WAIT, -1, fid && dl ? d + dl - 8 : a, d, 0, dl}); };
        auto red  = [&](uint64_t a, uint64_t b, uint64_t o, uint64_t len) { if (len) ops.push_back({Op::REDUCE, -1, a, b, o, len}); };
        // a chain of reductions acc = first + every received piece, ending in dst (alternating tmp / out, never in place)
        auto chain = [&](uint64_t first, std::vector<uint64_t> pieces, uint64_t dst, uint64_t len) {
            uint64_t acc = first;
            const int n = int(pieces.size());
            for (int j = 0; j < n; j++) {
                const uint64_t o = ((n - 1 - j) % 2 == 0) ? dst : TMP;
                red(acc, pieces[j], o, len);
                acc = o;
            }
        };
        // peers in rotated order (k + 1, k + 2, ...): in step i every rank sends to k + i and receives from k - i, one
        // flow per link; in rank order all ranks would send to the same peer first (an incast on its link)
        std::vector<int> peers;
        for (int i = 1; i < N; i++) peers.push_back((k + i) % N);
        switch (c) {
        case AG:
        case AR_DIRECT:
            for (int q : peers) { copy(IN, q, blk(q, k) + par, S); sig(q, blk(q, k) + FLGS); }
            for (int w : peers) wait(blk(k, w) + FLGS, blk(k, w) + par, S);
            if (c == AR_DIRECT) {
                std::vector<uint64_t> p; for (int w : peers) p.push_back(blk(k, w) + par);
                chain(IN, p, OUT, S);
            }
            break;
        case A2A:
        case RS:
            for (int q : peers) { copy(IN + coff(q), q, blk(q, k) + par, clen(q)); sig(q, blk(q, k) + FLGS); }
            for (int w : peers) wait(blk(k, w) + FLGS, blk(k, w) + par, clen(k));
            if (c == RS) {
                std::vector<uint64_t> p; for (int w : peers) p.push_back(blk(k, w) + par);
                chain(IN + coff(k), p, OUT, clen(k));
            }
            break;
        case AR_RING: {
            const uint64_t mine = blk(k, prev) + par, theirs = blk(next, k) + par;   // what prev writes here / k writes at next
            for (int t = 0; t < N - 1; t++) {        // reduce-scatter: partial sums travel to next
                const int s = (k - t + N) % N, r = (k - t - 1 + 2 * N) % N;
                copy((t == 0 ? IN : OUT) + coff(s), next, theirs + uint64_t(t) * C, clen(s));
                sig(next, blk(next, k) + FLGS + 8 * uint64_t(t));
                wait(blk(k, prev) + FLGS + 8 * uint64_t(t), mine + uint64_t(t) * C, clen(r));
                red(IN + coff(r), mine + uint64_t(t) * C, OUT + coff(r), clen(r));
            }
            for (int t = 0; t < N - 1; t++) {        // all-gather: the reduced chunks travel to next
                const int s = (k + 1 - t + N) % N;
                const uint64_t ag = uint64_t(N - 1) * C;
                copy(t == 0 ? OUT + coff(s) : mine + ag + uint64_t(t - 1) * C, next, theirs + ag + uint64_t(t) * C, clen(s));
                sig(next, blk(next, k) + FLGS + 8 * uint64_t(N - 1 + t));
                wait(blk(k, prev) + FLGS + 8 * uint64_t(N - 1 + t), mine + ag + uint64_t(t) * C, clen((k - t + N) % N));
            }
            break;
        }
        case SHIFT:
            copy(IN, next, blk(next, k) + par, S); sig(next, blk(next, k) + FLGS);
            wait(blk(k, prev) + FLGS, blk(k, prev) + par, S);
            break;
        case BC:   // --stamp: no flag behind the data, the receiver waits on the payload's last 8 B
            if (k == 0) {
                for (int q : peers) { copy(IN, q, blk(q, 0) + par, S); if (!stamp_mode) sig(q, blk(q, 0) + FLGS); }
                for (int w : peers) wait(blk(0, w) + FLGS + 8, 0, 0);
            } else {
                wait(stamp_mode ? blk(k, 0) + par + S - 8 : blk(k, 0) + FLGS, blk(k, 0) + par, S);
                sig_(0, blk(0, k) + FLGS + 8);
            }
            break;
        }
    };

    // ---- cards: a QP-owner cThread per pair that builds a QP, the data cThread (also the CSRs'), the V80 ----
    std::vector<std::unique_ptr<coyote::cThread>> t_qp(N);
    for (int q = 0; q < N; q++) if (q != k && need_qp(k, q)) t_qp[q] = std::make_unique<coyote::cThread>(0, getpid(), 0, nullptr, "loom_fpga");
    coyote::cThread t_data(0, getpid(), 0, nullptr, "loom_fpga");
    coyote::cThread &csr = t_data;
    std::unique_ptr<coyote::cThread> v80;
    if (!cpu) v80 = std::make_unique<coyote::cThread>(0, getpid(), 0, nullptr, "loom_versal_fpga");

    // ---- control mesh: rank 0 is the hub ----
    std::vector<int> cfd(N, -1);
    if (k == 0) {
        const int lfd = listen_on(port);
        if (lfd < 0) { printf("FAIL: control port %u\n", port); return 1; }
        for (int j = 1; j < N; j++) {
            const int fd = ::accept(lfd, nullptr, nullptr);
            uint32_t r = 0;
            if (fd < 0 || !read_full(fd, &r, 4) || r == 0 || int(r) >= N || cfd[r] >= 0) { printf("FAIL: control accept\n"); return 1; }
            cfd[r] = fd;
        }
        ::close(lfd);
    } else {
        cfd[0] = connect_to(hosts[0], port);
        const uint32_t r = uint32_t(k);
        if (cfd[0] < 0 || !write_full(cfd[0], &r, 4)) { printf("FAIL: control connect to %s\n", hosts[0].c_str()); return 1; }
    }
    auto barrier = [&]() {
        char b = 1;
        if (k == 0) {
            for (int j = 1; j < N; j++) if (!read_full(cfd[j], &b, 1)) return false;
            for (int j = 1; j < N; j++) if (!write_full(cfd[j], &b, 1)) return false;
            return true;
        }
        return write_full(cfd[0], &b, 1) && read_full(cfd[0], &b, 1);
    };
    // The window and export tables are programmed through one set of staging CSRs per card (index, fields, commit):
    // ranks sharing a card must not program at the same time, so every rank programs in its own turn
    auto in_turn = [&](const std::function<void()> &fn) {
        for (int r = 0; r < N; r++) { if (r == k) fn(); if (!barrier()) return false; }
        return true;
    };

    // ---- one QP per pair, in a fixed order (pair (i, j), i < j: i serves, j connects, after a barrier) ----
    int pair = 0;
    for (int i = 0; i < N; i++) for (int j = i + 1; j < N; j++, pair++) {
        if (!barrier()) { printf("FAIL: barrier before pair %d-%d\n", i, j); return 1; }
        if (!need_qp(i, j)) continue;
        const uint16_t qport = uint16_t(port + 2 + 2 * pair);
        void *st = nullptr;
        if (k == i) st = t_qp[j]->initRDMA(STAGING_SIZE, qport);
        else if (k == j) {
            for (int a = 0; a < 100 && !st; a++) {
                try { usleep(200000); st = t_qp[i]->initRDMA(STAGING_SIZE, qport, hosts[i].c_str()); }
                catch (const std::exception &e) { if (a == 99) printf("FAIL: QP %d-%d: %s\n", i, j, e.what()); }
            }
        } else continue;
        if (!st) { printf("FAIL: initRDMA pair %d-%d\n", i, j); return 1; }
    }

    // ---- this rank's region, exported whole (export 1 + slot) ----
    std::unique_ptr<land_v80::Landing> L;
    uint8_t *hbuf = nullptr;
    const uint32_t EXP = 1 + uint32_t(slot);
    const uint64_t VTH = (VT + HUGE2M - 1) & ~(HUGE2M - 1);   // the region file, whole 2 MiB pages
    void *ebase = nullptr;                                     // what export EXP lands in
    if (!cpu) {
        L = std::make_unique<land_v80::Landing>(*v80, VT);
        int32_t *in = reinterpret_cast<int32_t *>(L->buf);
        for (uint64_t j = 0; j < S / 4; j++) in[j] = input(k, j);
        v80->invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{L->buf, VT});
        void *lva = loom_switch::reserve_va(VT);
        t_data.importDmabuf(L->export_fd(), lva);
        ebase = lva;
    } else {
        // 2 MiB pages as getMem(HPF), but a shared hugetlbfs file, so a peer on this host can map it
        const std::string fn = region_file(port, k);
        const int rfd = ::open(fn.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0600);
        if (rfd < 0 || ::ftruncate(rfd, off_t(VTH)) < 0) { printf("FAIL: region file %s: %s\n", fn.c_str(), strerror(errno)); return 1; }
        void *m = mmap(nullptr, VTH, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_POPULATE, rfd, 0);
        ::close(rfd);
        if (m == MAP_FAILED) { printf("FAIL: no huge pages for %s (%lu B): %s\n", fn.c_str(), (unsigned long) VTH, strerror(errno)); return 1; }
        hbuf = static_cast<uint8_t *>(m);
        memset(hbuf, 0, VTH);
        t_data.userMap(hbuf, VTH);
        int32_t *in = reinterpret_cast<int32_t *>(hbuf);
        for (uint64_t j = 0; j < S / 4; j++) in[j] = input(k, j);
        ebase = hbuf;
    }
    if (!in_turn([&] { loom_switch::program_export(csr, EXP, t_data.getCtid(), ebase, VT); })) { printf("FAIL: barrier\n"); return 1; }

    // ---- hello: every rank's landing ctid, through rank 0 ----
    std::vector<Hello> hs(N);
    Hello mine{MAGIC, uint32_t(k), uint32_t(t_data.getCtid()), uint32_t(ack_store) + 2 * uint32_t(stamp_mode) + 4 * uint32_t(fid), S, uint64_t(K),
               uint64_t(c), runs, EXP, uint32_t(share_qp), {}};
    for (int q = 0; q < N; q++) mine.qp_ctid[q] = t_qp[q] ? uint32_t(t_qp[q]->getCtid()) : NO_QP;
    if (k == 0) {
        hs[0] = mine;
        for (int j = 1; j < N; j++) if (!read_full(cfd[j], &hs[j], sizeof(Hello))) { printf("FAIL: hello\n"); return 1; }
        for (int j = 1; j < N; j++) if (!write_full(cfd[j], hs.data(), N * sizeof(Hello))) { printf("FAIL: hello\n"); return 1; }
    } else if (!write_full(cfd[0], &mine, sizeof(mine)) || !read_full(cfd[0], hs.data(), N * sizeof(Hello))) {
        printf("FAIL: hello\n"); return 1;
    }
    for (int q = 0; q < N; q++)
        if (hs[q].magic != MAGIC || int(hs[q].rank) != q || hs[q].size != S || hs[q].iters != uint64_t(K) ||
            hs[q].coll != uint64_t(c) || hs[q].runs != runs || hs[q].bcmode != mine.bcmode || hs[q].share != mine.share) {
            printf("FAIL: hello from rank %d (--ranks, --coll, --size, --iters, --runs, --ack-store, --stamp and --share-qp must"
                   " match)\n", q); return 1;
        }
    // ---- peers on this host: their region files, mapped shared ----
    std::vector<uint8_t *> peer(N, nullptr);
    for (int q = 0; q < N; q++) if (same(q)) {
        const int pfd = ::open(region_file(port, q).c_str(), O_RDWR);
        void *m = pfd < 0 ? MAP_FAILED : mmap(nullptr, VTH, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_POPULATE, pfd, 0);
        if (pfd >= 0) ::close(pfd);
        if (m == MAP_FAILED) { printf("FAIL: region of rank %d on this host: %s\n", q, strerror(errno)); return 1; }
        peer[q] = static_cast<uint8_t *>(m);
    }
    // ---- windows: remote peer q's block(k) through window 1 + s R + ridx(q) = uwin [(s R + ridx) BLK, +BLK), on the QP of
    //      this rank and q (--share-qp: of the first ranks of the two hosts) ----
    const uint32_t WIN0 = 1 + uint32_t(slot) * uint32_t(nrem);
    auto ustart = [&](int q) { return (uint64_t(slot) * uint64_t(nrem) + uint64_t(ridx(q))) * BLK; };
    for (int q = 0; q < N; q++) if (q != k && !same(q) && (share_qp ? hs[lead(k)].qp_ctid[lead(q)] : hs[k].qp_ctid[q]) == NO_QP) {
        printf("FAIL: no QP from rank %d to rank %d\n", k, q); return 1;
    }
    if (!in_turn([&] {
            if (window >= 0 && slot == 0) loom_switch::csr_write(csr, loom_switch::TX_CTL, uint64_t(window));
            for (int q = 0; q < N; q++) if (q != k && !same(q))
                loom_switch::program_window(csr, WIN0 + uint32_t(ridx(q)), true, share_qp ? hs[lead(k)].qp_ctid[lead(q)] : hs[k].qp_ctid[q],
                                            reinterpret_cast<void *>(loom_switch::export_ref(hs[q].exp, blk(q, k))), BLK, ustart(q),
                                            hs[q].dst_ctid);
        })) { printf("FAIL: barrier\n"); return 1; }
    auto uoff = [&](int q, uint64_t dst) {     // peer q's region offset -> uwin offset
        if (dst < blk(q, k) || dst - blk(q, k) >= BLK) { fprintf(stderr, "BUG: offset %lu outside block\n", (unsigned long) dst); exit(3); }
        return ustart(q) + (dst - blk(q, k));
    };
    if (!barrier()) { printf("FAIL: barrier\n"); return 1; }   // every window and export in place
    if (ack_store && k != 0) csr.mapUwin(uint64_t(slot + 1) * uint64_t(nrem) * BLK);   // bc (a)/(b): the ack = a CPU store through rank 0's window

    // ---- the engine ----
    std::unique_ptr<ce_prog::Program> pr;
    volatile uint64_t *done = nullptr;
    std::unique_ptr<ptq::Queue> q;
    uint32_t hb = 0, hu = 0, hf = 0;
    constexpr uint32_t FSLOTS = 512;
    uint64_t *fsrc = nullptr;
    uint32_t fslot = 0;
    size_t entries = 0;
    if (!cpu) {
        const uint64_t UW = uint64_t(N - 1) * BLK;
        void *uva = loom_switch::reserve_va(UW);
        v80->importDmabuf(csr.exportDmabuf(EXPORT_REGION_UWIN, 0, UW), uva);
        uint8_t *uw = static_cast<uint8_t *>(uva), *me = reinterpret_cast<uint8_t *>(L->buf);
        done = static_cast<uint64_t *>(v80->getMem({coyote::CoyoteAllocType::HPF, 4096}));
        const_cast<uint64_t &>(done[0]) = 0;
        pr = std::make_unique<ce_prog::Program>(*v80);
        for (int i = 1; i <= K; i++) {
            std::vector<Op> ops;
            program(i, ops);
            for (const Op &o : ops) switch (o.t) {
                case Op::COPY:   pr->add(ce_prog::copy(me + o.a, uw + uoff(o.peer, o.o), o.len)); break;
                case Op::SIGNAL: pr->add(ce_prog::signal(uw + uoff(o.peer, o.o), i)); break;
                case Op::WAIT:   pr->add(ce_prog::wait(me + o.a, i)); break;
                case Op::REDUCE: pr->add(ce_prog::reduce(me + o.a, me + o.b, me + o.o, o.len, true)); break;
            }
        }
        pr->add(ce_prog::signal(const_cast<uint64_t *>(done), K));
        if (pr->size() > ce_prog::N_PROG) { printf("FAIL: %zu program entries (at most %u): fewer --iters\n", pr->size(), ce_prog::N_PROG); return 1; }
        pr->load();
        entries = pr->size();
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
        if (ulen < uint64_t(slot + 1) * uint64_t(nrem) * BLK) { printf("FAIL: the uwin is smaller than the windows\n"); return 1; }
    }
    printf("rank %d of %d (%s): up; %s, %lu B per rank (chunk %lu, block %lu, region %lu), %d iterations a run, %d runs (the first"
           " a warm-up)%s%s\n", k, N, cpu ? ("ptdma " + ptq_dev).c_str() : "v80", NAMES[c], (unsigned long) S, (unsigned long) C,
           (unsigned long) BLK, (unsigned long) VT, K, runs, entries ? (", " + std::to_string(entries) + " entries").c_str() : "",
           (std::string(verify ? ", VERIFY (not for timing)" : acked ? ", ACKED" : acked_step ? ", ACKED PER STEP" : "") +
            (stamp_mode ? ", bc (b): stamped last word, no flag, ack = CPU store" : ack_store ? ", bc (a): ack = CPU store" : "") +
            (nslot > 1 ? ", slot " + std::to_string(slot) + " of " + std::to_string(nslot) + " on this host (" + std::to_string(nsame) +
                         " peer(s) by CPU copy)" : "") + (share_qp ? ", SHARED QP per host pair" : "")).c_str());

    // ---- the CPU's run of the steps (ptdma rank) ----
    bool timeout = false;
    uint64_t vbad = 0;
    std::vector<std::vector<Op>> iters_ops;
    if (cpu) for (int i = 1; i <= K; i++) { iters_ops.emplace_back(); program(i, iters_ops.back()); }
    // 2026-10-27 (owner: the model's reductions from the collectives themselves): every reduce_cpu call of the timed runs,
    // (bytes, ns); printed as the median per call and length at the end
    std::vector<std::pair<uint64_t, double>> red_t;
    bool red_on = false;
    auto run_cpu = [&](int i, uint64_t v) {
        const std::vector<Op> &ops = iters_ops[size_t(i - 1)];
        uint64_t open = 0;   // --acked-step: the last push of this step not yet waited for
        auto settle = [&] { if (open) { while (!q->done(open)) {} open = 0; } };
        for (size_t n = 0; n < ops.size() && !timeout; n++) {
            const Op &o = ops[n];
            if (o.t == Op::WAIT || o.t == Op::REDUCE) settle();
            switch (o.t) {
            case Op::COPY: {
                if (stamp_mode) {   // bc (b): the payload's last 8 B = the iteration (every line's first word too, --verify)
                    if (verify) for (uint64_t l = 0; l < o.len; l += 64) *reinterpret_cast<int32_t *>(hbuf + o.a + l) = int32_t(v);
                    *reinterpret_cast<uint64_t *>(hbuf + o.a + o.len - 8) = v;
                } else if (o.b) {   // --flag-in-data: the last 8 B = the iteration (--verify: the first word too)
                    if (verify) *reinterpret_cast<int32_t *>(hbuf + o.a) = int32_t(v);
                    *reinterpret_cast<uint64_t *>(hbuf + o.a + o.len - 8) = v;
                } else if (verify) { int32_t *w = reinterpret_cast<int32_t *>(hbuf + o.a); w[0] = w[o.len / 4 - 1] = int32_t(v); }
                if (same(o.peer)) {   // a peer on this host: the CPU copies into its region, the last 8 B and the flag after
                    uint8_t *d = peer[size_t(o.peer)] + o.o;
                    memcpy(d, hbuf + o.a, o.len - 8);
                    _mm_sfence();
                    memcpy(d + o.len - 8, hbuf + o.a + o.len - 8, 8);
                    if (n + 1 < ops.size() && ops[n + 1].t == Op::SIGNAL && ops[n + 1].peer == o.peer) {
                        _mm_sfence();
                        *reinterpret_cast<volatile uint64_t *>(peer[size_t(o.peer)] + ops[n + 1].o) = v;
                        n++;
                    }
                    break;
                }
                ptq_copy cs[2] = {{hb, hu, o.a, uoff(o.peer, o.o), o.len}, {}};
                uint32_t nc = 1;
                if (n + 1 < ops.size() && ops[n + 1].t == Op::SIGNAL && ops[n + 1].peer == o.peer) {   // the signal behind it
                    fsrc[fslot] = v;
                    cs[1] = {hf, hu, uint64_t(fslot) * 8, uoff(o.peer, ops[n + 1].o), 8};
                    fslot = (fslot + 1) % FSLOTS; nc = 2; n++;
                }
                const uint64_t sq_ = q->submit(cs, nc);
                if (acked) while (!q->done(sq_)) {}   // --acked: the engine wrote it out (the source is free)
                if (acked_step) open = sq_;
                break;
            }
            case Op::SIGNAL:
                if (same(o.peer)) { _mm_sfence(); *reinterpret_cast<volatile uint64_t *>(peer[size_t(o.peer)] + o.o) = v; break; }
                if (ack_store) { csr.uwinWrite(uoff(o.peer, o.o), &v, 8); break; }   // bc (a)/(b): 8 B store + sfence
                fsrc[fslot] = v;
                {
                    const uint64_t sq_ = q->copy(hf, uint64_t(fslot) * 8, hu, uoff(o.peer, o.o), 8);
                    fslot = (fslot + 1) % FSLOTS;
                    if (acked) while (!q->done(sq_)) {}
                    if (acked_step) open = sq_;
                }
                break;
            case Op::WAIT: {
                const volatile uint64_t *w = reinterpret_cast<uint64_t *>(hbuf + o.a);
                const auto tw = std::chrono::steady_clock::now();
                while (*w < v) if (std::chrono::steady_clock::now() - tw > std::chrono::seconds(10)) { timeout = true; break; }
                if (timeout) {   // which flag: the writer is the rank whose block holds it
                    int wr = -1;
                    for (int q = 0; q < N; q++) if (q != k && o.a >= blk(k, q) && o.a < blk(k, q) + BLK) wr = q;
                    printf("TIMEOUT: wait %zu of iteration %lu: flag at %#lx (from rank %d, %s) is %lu, want %lu\n", n,
                           (unsigned long) v, (unsigned long) o.a, wr, wr >= 0 && same(wr) ? "same host" : "remote",
                           (unsigned long) *w, (unsigned long) v);
                }
                if (verify && o.len && !timeout) {   // the piece this flag guards, of iteration v
                    const volatile int32_t *d = reinterpret_cast<int32_t *>(hbuf + o.b);
                    if (stamp_mode) { for (uint64_t j = 0; j < o.len / 4; j += 16) if (d[j] < int32_t(v)) { vbad++; break; } }
                    else if (fid) { if (d[0] < int32_t(v)) vbad++; }   // the flag (last word) before the first word
                    else if (d[0] < int32_t(v) || d[o.len / 4 - 1] < int32_t(v)) vbad++;
                }
                break;
            }
            case Op::REDUCE: {
                const auto r0 = std::chrono::steady_clock::now();
                reduce_cpu(reinterpret_cast<int32_t *>(hbuf + o.a), reinterpret_cast<int32_t *>(hbuf + o.b),
                           reinterpret_cast<int32_t *>(hbuf + o.o), o.len);
                if (red_on) red_t.push_back({o.len, std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - r0).count()});
                break;
            }
            }
        }
        settle();
    };
    auto drain = [&](std::chrono::milliseconds tmo) {
        const auto tw = std::chrono::steady_clock::now();
        while (!q->done(q->seq())) if (std::chrono::steady_clock::now() - tw > tmo) return false;
        return true;
    };

    std::vector<double> per_iter_us;
    int errors = 0;
    for (int r = 0; r < runs; r++) {
        if (!barrier()) { printf("FAIL: barrier\n"); return 1; }
        const uint64_t base = uint64_t(r) * uint64_t(K), want = base + uint64_t(K);
        red_on = r > 0;
        const auto t0 = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point t1;
        if (cpu) {
            for (int i = 1; i <= K && !timeout; i++) run_cpu(i, base + uint64_t(i));
            t1 = std::chrono::steady_clock::now();
            if (!drain(std::chrono::milliseconds(2000))) timeout = true;
            if (timeout) {
                const ptq_info qi = q->info();
                printf("FAIL run %d: timed out (queue %lu of %lu done, status %#x)\n", r, (unsigned long) qi.completed,
                       (unsigned long) qi.submitted, qi.status);
                errors++; break;
            }
        } else {
            pr->start(base);
            while (done[0] < want)
                if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(r ? 20 : 60)) break;
            t1 = std::chrono::steady_clock::now();
            const bool fin = pr->finish(std::chrono::milliseconds(done[0] < want ? 10 : 1000));   // ABORT on a timeout (ce_coll)
            if (done[0] < want || !fin) {
                printf("FAIL run %d: did not finish (entry %lu, engine %s)\n", r, (unsigned long) v80->getCSR(ce_prog::PC),
                       v80->getCSR(ce_prog::BUSY) ? "STILL BUSY" : "stopped");
                errors++; break;
            }
        }
        const double us = std::chrono::duration<double, std::micro>(t1 - t0).count() / K;
        printf("%s run %d: %.2f us per %s\n", r ? "    " : "warm", r, us, NAMES[c]);
        if (r) per_iter_us.push_back(us);
    }
    const bool synced = errors == 0 && barrier();

    // ---- the last iteration's result on this rank ----
    const uint8_t *res = hbuf;
    if (!cpu) {
        if (v80->getCSR(ce_prog::BUSY)) { printf("FAIL: the engine did not stop; its buffers are left alone\n"); return 1; }
        L->pull();
        res = reinterpret_cast<uint8_t *>(L->buf);
    } else if (!drain(std::chrono::milliseconds(2000))) {
        printf("FAIL: the PTDMA queue did not drain; its buffers are left alone\n"); return 1;
    }
    const uint64_t par = (K & 1) ? B : 0;
    auto word = [&](uint64_t off, uint64_t j) { return reinterpret_cast<const int32_t *>(res + off)[j]; };
    auto sum = [&](uint64_t j) { int32_t s = 0; for (int w = 0; w < N; w++) s += input(w, j); return s; };
    uint64_t bad = 0, n = 0;
    auto chk = [&](int32_t got, int32_t want) { n++; bad += got != want; };
    auto chkb = [&](uint64_t j, uint64_t len, int32_t got, int32_t want) { if (!fid || 4 * j + 8 < len) chk(got, want); };   // flag words
    if (verify) {
        printf("rank %d: verify: %lu pieces seen before their data (flag ahead of data)\n", k, (unsigned long) vbad);
        errors += vbad != 0;
    } else if (errors == 0) {
        for (int w = 0; w < N; w++) {
            if (w == k) continue;
            const uint64_t b = blk(k, w) + par;
            switch (c) {
            case AG:  for (uint64_t j = 0; j < S / 4; j++) chkb(j, S, word(b, j), input(w, j)); break;
            case SHIFT: if (w == prev) for (uint64_t j = 0; j < S / 4; j++) chkb(j, S, word(b, j), input(w, j)); break;
            case A2A: for (uint64_t j = 0; j < clen(k) / 4; j++) chkb(j, clen(k), word(b, j), input(w, coff(k) / 4 + j)); break;
            case BC:
                if (w != 0) break;
                for (uint64_t j = 0; j < S / 4 - (stamp_mode ? 2 : 0); j++) chkb(j, S, word(b, j), input(0, j));
                if (stamp_mode) { n++; bad += *reinterpret_cast<const uint64_t *>(res + b + S - 8) != uint64_t(runs) * uint64_t(K); }
                break;
            default: break;
            }
        }
        if (c == RS) for (uint64_t j = 0; j < clen(k) / 4; j++) chkb(j, clen(k), word(OUT, j), sum(coff(k) / 4 + j));
        if (c == AR_DIRECT) for (uint64_t j = 0; j < S / 4; j++) chk(word(OUT, j), sum(j));
        if (c == AR_RING) {
            const int own = (k + 1) % N;    // reduced here; the others arrived in the all-gather slots
            for (uint64_t j = 0; j < clen(own) / 4; j++) chkb(j, clen(own), word(OUT + coff(own), j), sum(coff(own) / 4 + j));
            const uint64_t ag = blk(k, prev) + par + uint64_t(N - 1) * C;
            for (int t = 0; t < N - 1; t++) {
                const int ch = (k - t + N) % N;
                for (uint64_t j = 0; j < clen(ch) / 4; j++) chkb(j, clen(ch), word(ag + uint64_t(t) * C, j), sum(coff(ch) / 4 + j));
            }
        }
        printf("rank %d: %lu of %lu result words wrong\n", k, (unsigned long) bad, (unsigned long) n);
        errors += bad != 0;
    }
    {
        std::map<uint64_t, std::vector<double>> by;
        for (const auto &[len, ns] : red_t) by[len].push_back(ns);
        for (auto &[len, v] : by)
            printf("rank %d reduce %lu B: %zu calls, median %.1f ns [p10 %.1f p90 %.1f]\n", k, (unsigned long) len, v.size(),
                   pct(v, 50), pct(v, 10), pct(v, 90));
    }
    if (!per_iter_us.empty()) {
        printf("rank %d %s %lu B (us per collective, %zu runs, median [min max]): %.2f [%.2f %.2f]\n", k, NAMES[c],
               (unsigned long) S, per_iter_us.size(), pct(per_iter_us, 50), pct(per_iter_us, 0), pct(per_iter_us, 100));
        if (!out.empty()) {
            FILE *f = fopen(out.c_str(), "w");
            if (f) { for (double v : per_iter_us) fprintf(f, "%.3f\n", v); fclose(f); }
        }
    }
    q.reset();
    auto release = [&] {
        for (int p = 0; p < N; p++) if (p != k && !same(p)) loom_switch::release_window(csr, WIN0 + uint32_t(ridx(p)));
        loom_switch::release_export(csr, EXP);
    };
    if (synced) in_turn(release); else release();
    for (int p = 0; p < N; p++) if (peer[size_t(p)]) munmap(peer[size_t(p)], VTH);
    if (cpu) { t_data.userUnmap(hbuf); munmap(hbuf, VTH); ::unlink(region_file(port, k).c_str()); }
    for (int fd : cfd) if (fd >= 0) ::close(fd);
    printf(errors ? "COLLN FAIL\n" : "COLLN PASS\n");
    return errors ? 1 : 0;
}
