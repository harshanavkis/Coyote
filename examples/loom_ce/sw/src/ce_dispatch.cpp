/**
 * ce_dispatch - the Section 2 dispatch / combine on Loom (E4): rose's V80
 * sends a top-8 MoE dispatch (or combine) into amy's V80 HBM through the two
 * U280 switches, as dispatch_bd does over RDMA on steve. Routing as
 * dispatch_bd.cu: top-8 of 256 experts, R = 8 destinations, one message per
 * (token, destination); the 8 destinations are 8 regions of the one peer's
 * window, as dispatch_bd's 8 destinations are on its one peer. --combine: one
 * message of --H bytes per (token, destination) of the dispatch, back to its
 * destination region.
 *
 *   sender (client), per run:  STAMP t0; COPY every message -> peer; STAMP t1;
 *                              SIGNAL peer flag; STAMP t2; WAIT own flag; STAMP t3
 *   receiver (server), per run: WAIT own flag; SIGNAL peer flag (the ack)
 *
 * t2 - t0: the sender is busy until it has issued the arrival signal (Fig 3's
 * end point), t1 - t0 its copies alone; the switch orders the signal behind
 * the data, so the sender waits for nothing but its own writes being taken.
 * t3 - t0 = delivery + the ack's one-way trip (ce_pingpong --size 0 gives it).
 * --tokens 0 runs no copies: t3 - t0 is the flag round trip. All stamps are
 * the sender V80's clock. RUNS runs (the first a warm-up, not counted); at the
 * end the receiver checks every message of the last run.
 *
 *   amy:   ce_dispatch --server [--port N] [--tokens T] [--H B] [--combine] [--runs R]
 *   rose:  ce_dispatch --client <amy ip> [--port N] [--tokens T] [--H B] [--combine] [--runs R] [--out FILE]
 *
 * --land-host (server): the messages and the flag land in the server's host memory (no V80 needed
 * there); the server checks each run's flag and, at the end, every message, and sends no ack, so the
 * client stops at t2 (no delivery time).
 *
 * Both sides need the same --tokens, --H, --combine and --runs.
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

constexpr uint32_t MAGIC = 0x44495350;      // "DISP"
constexpr uint64_t STAGING_SIZE = 1ULL << 20;
constexpr int R = 8, E = 256, TOPK = 8;

struct Hello { uint32_t magic; uint32_t dst_ctid; uint64_t staging_va, dst_ref, flag_ref, tokens, H, combine; int64_t runs; uint64_t ack; };

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
uint64_t pattern(uint64_t unit, uint64_t off) { return 0x4449000000000000ULL ^ (unit * 0xD6E8FEB86659FD93ULL) ^ (off * 0x9E3779B97F4A7C15ULL); }

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
    const size_t i = std::min(v.size() - 1, size_t(p / 100.0 * double(v.size())));
    return v[i];
}

// dispatch_bd.cu's routing: the destinations holding token t's top-8 experts
unsigned hash(unsigned x) { x ^= x >> 16; x *= 0x7feb352d; x ^= x >> 15; x *= 0x846ca68b; x ^= x >> 16; return x; }
unsigned rank_mask(int t) {
    unsigned mask = 0; int got = 0; unsigned long long used[4] = {0, 0, 0, 0};
    for (int j = 0; got < TOPK; j++) {
        int e = int(hash(unsigned(t * 131 + j)) % E);
        if (used[e >> 6] >> (e & 63) & 1) continue;
        used[e >> 6] |= 1ull << (e & 63); got++;
        mask |= 1u << (e / (E / R));
    }
    return mask;
}

// one message: source unit (dispatch: the token, combine: the message), its slot in the window
struct Msg { uint64_t unit, slot; };

}  // namespace

int main(int argc, char **argv) {
    std::string ip, out;
    bool server = false, combine = false, land_host = false;
    uint16_t port = 19810;
    int tokens = 1, runs = 22;
    uint64_t H = 7168;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--server")                       server = true;
        else if (a == "--client" && i + 1 < argc)  ip = argv[++i];
        else if (a == "--port" && i + 1 < argc)    port = uint16_t(atoi(argv[++i]));
        else if (a == "--tokens" && i + 1 < argc)  tokens = atoi(argv[++i]);
        else if (a == "--H" && i + 1 < argc)       H = strtoull(argv[++i], nullptr, 0);
        else if (a == "--combine")                 combine = true;
        else if (a == "--land-host")               land_host = true;
        else if (a == "--runs" && i + 1 < argc)    runs = atoi(argv[++i]);
        else if (a == "--out" && i + 1 < argc)     out = argv[++i];
        else { server = false; ip.clear(); break; }
    }
    // messages in dispatch_bd's order: by destination, then slot (= token order within a destination)
    std::vector<Msg> msgs;
    std::vector<uint64_t> cnt(R, 0);
    std::vector<std::vector<uint64_t>> by_dest(R);
    for (int t = 0; t < tokens; t++)
        for (unsigned m = rank_mask(t); m; m &= m - 1) by_dest[__builtin_ctz(m)].push_back(uint64_t(t));
    uint64_t slot = 0, unit = 0;
    for (int r = 0; r < R; r++)
        for (uint64_t t : by_dest[r]) msgs.push_back({combine ? unit++ : t, slot++});
    const uint64_t units = combine ? msgs.size() : uint64_t(tokens);
    if (server == !ip.empty() || (land_host && !server) || H % 64 || H == 0 || tokens < 0 || runs < 2 || msgs.size() + 8 > ce_prog::N_PROG ||
        msgs.size() * H > (16ULL << 20)) {
        printf("usage: %s --server | --client <ip>  [--port N] [--tokens T] [--H B, a multiple of 64] [--combine]"
               " [--runs R >= 2] [--out FILE] [--land-host (server)]   (at most %u messages, 16 MiB)\n", argv[0], ce_prog::N_PROG - 8);
        return 2;
    }
    // the message region (whole pages), then the flag page
    const uint64_t mlen = std::max<uint64_t>(4096, (msgs.size() * H + 4095) & ~4095ULL);

    coyote::cThread t_qp(0, getpid(), 0, nullptr, "coyote_fpga");     // QP owner, CSR page, rdma windows
    coyote::cThread t_data(0, getpid(), 0, nullptr, "coyote_fpga");   // owns the landing exports
    std::unique_ptr<coyote::cThread> v80p;   // no V80 on a --land-host server
    if (!land_host) v80p.reset(new coyote::cThread(0, getpid(), 0, nullptr, "coyote_versal_fpga"));
    printf("%s: QP exchange on port %u ...\n", server ? "server" : "client", port);
    void *staging = server ? t_qp.initRDMA(STAGING_SIZE, port) : t_qp.initRDMA(STAGING_SIZE, port, ip.c_str());
    if (!staging) { printf("FAIL: initRDMA\n"); return 1; }

    // Landing: this V80's window (its HBM), imported into the U280 data thread and exported;
    // --land-host: two host buffers of the data thread
    std::unique_ptr<land_v80::Landing> L;
    uint64_t *land = nullptr;                               // the messages; the flag follows at land + mlen
    volatile uint64_t *my_flag = nullptr;
    if (land_host) {
        land = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, mlen}));
        uint64_t *fl = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, 4096}));
        memset(land, 0, mlen); memset(fl, 0, 4096);
        loom_switch::program_export(t_qp, 1, t_data.getCtid(), land, mlen);
        loom_switch::program_export(t_qp, 2, t_data.getCtid(), fl, 4096);
        my_flag = fl;
    } else {
        L.reset(new land_v80::Landing(*v80p, mlen + 4096));
        void *lva = loom_switch::reserve_va(mlen + 4096);
        t_data.importDmabuf(L->export_fd(), lva);
        loom_switch::program_export(t_qp, 1, t_data.getCtid(), lva, mlen);
        loom_switch::program_export(t_qp, 2, t_data.getCtid(), static_cast<uint8_t *>(lva) + mlen, 4096);
        land = L->buf;
        my_flag = L->buf + mlen / 8;                        // card VA, polled by WAIT
    }

    const int fd = connect_or_accept(server ? std::string() : ip, uint16_t(port + 1));
    Hello mine{MAGIC, uint32_t(t_data.getCtid()), reinterpret_cast<uint64_t>(staging), loom_switch::export_ref(1),
               loom_switch::export_ref(2), uint64_t(tokens), H, uint64_t(combine), runs, uint64_t(!land_host)}, h{};
    if (fd < 0 || !write_full(fd, &mine, sizeof(mine)) || !read_full(fd, &h, sizeof(h)) || h.magic != MAGIC ||
        h.tokens != uint64_t(tokens) || h.H != H || h.combine != uint64_t(combine) || h.runs != runs) {
        printf("FAIL: hello (--tokens, --H, --combine and --runs must match on both sides)\n");
        return 1;
    }

    if (land_host) {   // nothing to send: per run, arm, wait for the flag, barrier; then check every message
        int errors = 0;
        for (int r = 0; r < runs && !errors; r++) {
            char go = 1;
            if (!write_full(fd, &go, 1)) { printf("FAIL: armed\n"); return 1; }
            const auto t0 = std::chrono::steady_clock::now();
            while (*my_flag < uint64_t(r) + 1)
                if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(60)) { printf("FAIL run %d: no flag\n", r); errors++; break; }
            printf("server run %d: flag seen after %.1f ms (armed -> flag)\n", r,
                   std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
            char done = 1;
            if (!write_full(fd, &done, 1) || !read_full(fd, &done, 1)) { printf("FAIL: barrier\n"); return 1; }
        }
        uint64_t bad = 0;
        for (const Msg &m : msgs)
            for (uint64_t j = 0; j < H / 8; j++) bad += (land[m.slot * H / 8 + j] != pattern(m.unit, j));
        printf("server (host landing): %lu of %lu words wrong, flag %lu (want %d)\n", (unsigned long) bad,
               (unsigned long) (msgs.size() * H / 8), (unsigned long) *my_flag, runs);
        errors += (bad != 0) + (*my_flag != uint64_t(runs));
        ::close(fd);
        loom_switch::release_export(t_qp, 1);
        loom_switch::release_export(t_qp, 2);
        t_qp.connSync(false);
        printf(errors ? "DISPATCH FAIL\n" : "DISPATCH PASS\n");
        return errors ? 1 : 0;
    }
    coyote::cThread &v80 = *v80p;

    // Sending: rdma windows onto the peer's messages and flag, the uwin exported to this V80
    loom_switch::csr_write(t_qp, loom_switch::RDMA_STAGING_VA, h.staging_va);
    loom_switch::program_window(t_qp, 1, true, t_qp.getCtid(), reinterpret_cast<void *>(h.dst_ref), mlen, 0, h.dst_ctid);
    loom_switch::program_window(t_qp, 2, true, t_qp.getCtid(), reinterpret_cast<void *>(h.flag_ref), 4096, mlen, h.dst_ctid);
    void *uva = loom_switch::reserve_va(mlen + 4096);
    v80.importDmabuf(t_qp.exportDmabuf(EXPORT_REGION_UWIN, 0, mlen + 4096), uva);
    uint8_t *peer_msg = static_cast<uint8_t *>(uva), *peer_flag = peer_msg + mlen;

    // source: one H-byte unit per token (dispatch) or per message (combine), in this V80's HBM
    const uint64_t slen = std::max<uint64_t>(4096, (units * H + 4095) & ~4095ULL);
    uint64_t *src = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, slen}));
    for (uint64_t u = 0; u < units; u++)
        for (uint64_t j = 0; j < H / 8; j++) src[u * H / 8 + j] = pattern(u, j);
    v80.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{src, slen});
    // stamps go to host memory (read without a sync, see ce_pingpong); a card SIGNAL at the end
    // of the program waits for them to complete
    volatile uint64_t *st = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, 4096}));
    uint64_t *drain = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, 4096}));
    v80.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{drain, 4096});

    ce_prog::Program p(v80);
    if (server) {
        p.add(ce_prog::wait(const_cast<uint64_t *>(my_flag), 1));
        p.add(ce_prog::signal(peer_flag, 1));                                     // the ack
    } else {
        auto s = const_cast<uint64_t *>(st);
        p.add(ce_prog::stamp(&s[0]));
        for (const Msg &m : msgs)
            p.add(ce_prog::copy(reinterpret_cast<uint8_t *>(src) + m.unit * H, peer_msg + m.slot * H, H));
        p.add(ce_prog::stamp(&s[1]));
        p.add(ce_prog::signal(peer_flag, 1));
        p.add(ce_prog::stamp(&s[2]));
        if (h.ack) {                                                              // the receiver acks
            p.add(ce_prog::wait(const_cast<uint64_t *>(my_flag), 1));
            p.add(ce_prog::stamp(&s[3]));
        }
        p.add(ce_prog::signal(drain, 0, true));                                  // after every stamp landed
    }
    p.load();
    printf("%s: up; %s, %d tokens, %zu messages of %lu B (%.2f MB), %d runs (the first a warm-up)\n",
           server ? "server" : "client", combine ? "combine" : "dispatch", tokens, msgs.size(), (unsigned long) H,
           double(msgs.size() * H) / 1e6, runs);

    std::vector<double> copies, issued, ack;   // us, counted runs
    FILE *of = (!server && !out.empty()) ? fopen(out.c_str(), "w") : nullptr;
    if (of) fprintf(of, "op,tokens,msgs,H,run,copies_us,signal_issued_us,ack_us\n");
    int errors = 0;
    for (int r = 0; r < runs; r++) {
        const uint64_t base = uint64_t(r);
        char go = 1;
        if (server) {
            p.start(base);                                   // armed: waits for the client's flag
            if (!write_full(fd, &go, 1)) { printf("FAIL: armed\n"); return 1; }
        } else {
            if (!read_full(fd, &go, 1)) { printf("FAIL: the server did not arm\n"); return 1; }
            p.start(base);
        }
        // the first run takes the first touches of the landing pages (on rose ~10 s of driver page faults)
        const bool ok = p.finish(std::chrono::milliseconds(r ? 5000 : 60000));
        if (!ok) { printf("FAIL run %d: the program did not finish (aborted at entry %lu)\n", r,
                          (unsigned long) v80.getCSR(ce_prog::PC)); errors++; break; }
        if (!server) {
            const double c = double(st[1] - st[0]) * ce_prog::CLK_NS / 1e3, i = double(st[2] - st[0]) * ce_prog::CLK_NS / 1e3,
                         a = h.ack ? double(st[3] - st[0]) * ce_prog::CLK_NS / 1e3 : -1;
            printf("%s run %d: copies %.2f us, signal issued %.2f us, ack back %.2f us\n", r ? "    " : "warm", r, c, i, a);
            if (r) { copies.push_back(c); issued.push_back(i); ack.push_back(a); }
            if (of && r) fprintf(of, "%s,%d,%zu,%lu,%d,%.3f,%.3f,%.3f\n", combine ? "combine" : "dispatch", tokens, msgs.size(),
                                 (unsigned long) H, r, c, i, a);
        }
        char done = 1;   // both finished before the next run arms
        if (!write_full(fd, &done, 1) || !read_full(fd, &done, 1)) { printf("FAIL: barrier\n"); return 1; }
    }
    if (of) fclose(of);

    // The receiver checks every message of the last run, and the flag
    if (server) {
        L->pull();
        uint64_t bad = 0;
        for (const Msg &m : msgs)
            for (uint64_t j = 0; j < H / 8; j++) bad += (land[m.slot * H / 8 + j] != pattern(m.unit, j));
        const bool flag_ok = (land[mlen / 8] == uint64_t(runs));
        printf("server: %lu of %lu words wrong, flag %lu (want %d)\n", (unsigned long) bad,
               (unsigned long) (msgs.size() * H / 8), (unsigned long) land[mlen / 8], runs);
        errors += (bad != 0) + !flag_ok;
    } else if (!issued.empty()) {
        printf("%s %d tokens, %zu x %lu B (us, %zu runs, median [min max]): copies %.2f [%.2f %.2f]  signal issued %.2f [%.2f %.2f]"
               "  ack back %.2f [%.2f %.2f]\n", combine ? "combine" : "dispatch", tokens, msgs.size(), (unsigned long) H,
               issued.size(), pct(copies, 50), pct(copies, 0), pct(copies, 100), pct(issued, 50), pct(issued, 0),
               pct(issued, 100), pct(ack, 50), pct(ack, 0), pct(ack, 100));
    }
    ::close(fd);
    loom_switch::release_window(t_qp, 1);
    loom_switch::release_window(t_qp, 2);
    loom_switch::release_export(t_qp, 1);
    loom_switch::release_export(t_qp, 2);
    t_qp.connSync(!server);
    printf(errors ? "DISPATCH FAIL\n" : "DISPATCH PASS\n");
    return errors ? 1 : 0;
}
