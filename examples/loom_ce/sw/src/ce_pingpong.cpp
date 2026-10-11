/**
 * ce_pingpong - one-way latency of Loom puts between two accelerators (E2):
 * rose's V80 and amy's V80, each landing in its own V80 HBM, each sending
 * through its own U280. Both copy engines run programs, so no host is in
 * the loop and the time is one V80's clock:
 *
 *   initiator, per round i:  STAMP t0; COPY msg -> peer; SIGNAL peer flag i;
 *                            WAIT own flag >= i; STAMP t1 (stamps into host memory)
 *   responder, per round i:  WAIT own flag >= i; COPY msg -> peer; SIGNAL
 *                            peer flag i
 *
 * One-way = (t1 - t0) / 2: a message and its flag through both switches and
 * the network into the peer's HBM, the peer engine seeing the flag (its WAIT
 * polls HBM) and reading its own message from HBM to send. --size 0 sends
 * only the 8 B flag (a lone store). ROUNDS rounds per run, RUNS runs (the
 * first is a warm-up and not counted); the programs are loaded once and
 * rerun with RUN_BASE advancing every flag value.
 *
 *   amy:   ce_pingpong --server [--port N] [--size B] [--runs R]
 *   rose:  ce_pingpong --client <amy ip> [--port N] [--size B] [--runs R] [--out FILE]
 *
 * Both sides need the same --size and --runs. The client prints the one-way
 * latency's percentiles (ns) and writes every counted sample to --out (one
 * per line, ns), and both check the last message that landed.
 *
 * --host (both sides): the like-for-like of rdma_pingpong --src-v80 (E2, Fig 8a).
 * The message still comes from this V80's HBM, but it and its flag land in
 * the peer's HOST memory and a CPU drives every round, as a CPU posting an
 * RDMA write does: per round the CPU starts a preloaded two-entry program
 * (COPY msg -> peer, SIGNAL peer flag; two CSR writes) and polls its own
 * flag in host memory; the responder's CPU polls, then starts its program.
 * One-way = round trip / 2 on the client CPU's clock (steady_clock).
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

constexpr uint32_t MAGIC = 0x50494E47;      // "PING"
constexpr uint64_t STAGING_SIZE = 1ULL << 20;
constexpr int ROUNDS = 400;                 // initiator: 5 entries a round, 2048 at most

struct Hello { uint32_t magic; uint32_t dst_ctid; uint64_t staging_va, dst_ref, flag_ref, size; int64_t runs; uint64_t host; };

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
uint64_t pattern(uint64_t off, int side) { return 0x5050000000000000ULL ^ (off * 0x9E3779B97F4A7C15ULL) ^ uint64_t(side); }

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

}  // namespace

int main(int argc, char **argv) {
    std::string ip, out;
    bool server = false, host = false;
    uint16_t port = 19800;
    uint64_t size = 64;
    int runs = 6;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--server")                       server = true;
        else if (a == "--host")                    host = true;
        else if (a == "--client" && i + 1 < argc)  ip = argv[++i];
        else if (a == "--port" && i + 1 < argc)    port = uint16_t(atoi(argv[++i]));
        else if (a == "--size" && i + 1 < argc)    size = strtoull(argv[++i], nullptr, 0);
        else if (a == "--runs" && i + 1 < argc)    runs = atoi(argv[++i]);
        else if (a == "--out" && i + 1 < argc)     out = argv[++i];
        else { server = false; ip.clear(); break; }
    }
    if (server == !ip.empty() || size % 64 || size > (16ULL << 20) || runs < 2) {
        printf("usage: %s --server | --client <ip>  [--port N] [--size B, a multiple of 64, 0 = the flag only]"
               " [--runs R >= 2] [--out FILE] [--host]\n", argv[0]);
        return 2;
    }
    const int side = server ? 0 : 1;
    // the message region (whole pages), then the flag page
    const uint64_t mlen = std::max<uint64_t>(4096, (size + 4095) & ~4095ULL);

    coyote::cThread t_qp(0, getpid(), 0, nullptr, "coyote_fpga");     // QP owner, CSR page, rdma windows
    coyote::cThread t_data(0, getpid(), 0, nullptr, "coyote_fpga");   // owns the landing exports
    coyote::cThread v80(0, getpid(), 0, nullptr, "coyote_versal_fpga");
    printf("%s: QP exchange on port %u ...\n", server ? "server" : "client", port);
    void *staging = server ? t_qp.initRDMA(STAGING_SIZE, port) : t_qp.initRDMA(STAGING_SIZE, port, ip.c_str());
    if (!staging) { printf("FAIL: initRDMA\n"); return 1; }

    // Landing: this V80's window (its HBM), imported into the U280 data thread and exported;
    // --host: host pages of the data thread, exported the same way
    std::unique_ptr<land_v80::Landing> L;
    volatile uint64_t *my_flag;            // card VA polled by WAIT, or (--host) host memory polled by the CPU
    uint64_t *host_msg = nullptr;
    if (host) {
        host_msg = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, mlen}));
        uint64_t *fl = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, 4096}));
        std::memset(host_msg, 0, mlen);
        std::memset(fl, 0, 4096);
        loom_switch::program_export(t_qp, 1, t_data.getCtid(), host_msg, mlen);
        loom_switch::program_export(t_qp, 2, t_data.getCtid(), fl, 4096);
        my_flag = fl;
    } else {
        L.reset(new land_v80::Landing(v80, mlen + 4096));
        void *lva = loom_switch::reserve_va(mlen + 4096);
        t_data.importDmabuf(L->export_fd(), lva);
        loom_switch::program_export(t_qp, 1, t_data.getCtid(), lva, mlen);
        loom_switch::program_export(t_qp, 2, t_data.getCtid(), static_cast<uint8_t *>(lva) + mlen, 4096);
        my_flag = L->buf + mlen / 8;
    }

    const int fd = connect_or_accept(server ? std::string() : ip, uint16_t(port + 1));
    Hello mine{MAGIC, uint32_t(t_data.getCtid()), reinterpret_cast<uint64_t>(staging),
               loom_switch::export_ref(1), loom_switch::export_ref(2), size, runs, uint64_t(host)}, h{};
    if (fd < 0 || !write_full(fd, &mine, sizeof(mine)) || !read_full(fd, &h, sizeof(h)) || h.magic != MAGIC ||
        h.size != size || h.runs != runs || h.host != uint64_t(host)) {
        printf("FAIL: hello (--size, --runs and --host must match on both sides)\n");
        return 1;
    }

    // Sending: rdma windows onto the peer's message and flag, the uwin exported to this V80
    loom_switch::csr_write(t_qp, loom_switch::RDMA_STAGING_VA, h.staging_va);
    loom_switch::program_window(t_qp, 1, true, t_qp.getCtid(), reinterpret_cast<void *>(h.dst_ref), mlen, 0, h.dst_ctid);
    loom_switch::program_window(t_qp, 2, true, t_qp.getCtid(), reinterpret_cast<void *>(h.flag_ref), 4096, mlen, h.dst_ctid);
    void *uva = loom_switch::reserve_va(mlen + 4096);
    v80.importDmabuf(t_qp.exportDmabuf(EXPORT_REGION_UWIN, 0, mlen + 4096), uva);
    uint8_t *peer_msg = static_cast<uint8_t *>(uva), *peer_flag = peer_msg + mlen;

    uint64_t *src = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, mlen}));
    for (uint64_t i = 0; i < mlen / 8; i++) src[i] = pattern(8 * i, side);
    v80.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{src, mlen});
    // stamps go to host memory (read here with no sync: a sync would unmap
    // card pages and the next run's first access would fault, ms); a card
    // SIGNAL at the end of the program waits for them to complete
    volatile uint64_t *st = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, 2 * ROUNDS * 8}));
    uint64_t *drain = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, 4096}));
    v80.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{drain, 4096});

    ce_prog::Program p(v80);
    if (host) {   // one round: the message, then the flag (value 1 + RUN_BASE)
        if (size) p.add(ce_prog::copy(src, peer_msg, size));
        p.add(ce_prog::signal(peer_flag, 1));
    }
    for (int i = 0; i < ROUNDS && !host; i++) {
        const uint64_t v = uint64_t(i) + 1;     // + RUN_BASE
        if (server) {
            p.add(ce_prog::wait(const_cast<uint64_t *>(my_flag), v));
            if (size) p.add(ce_prog::copy(src, peer_msg, size));
            p.add(ce_prog::signal(peer_flag, v));
        } else {
            p.add(ce_prog::stamp(const_cast<uint64_t *>(&st[2 * i])));
            if (size) p.add(ce_prog::copy(src, peer_msg, size));
            p.add(ce_prog::signal(peer_flag, v));
            p.add(ce_prog::wait(const_cast<uint64_t *>(my_flag), v));
            p.add(ce_prog::stamp(const_cast<uint64_t *>(&st[2 * i + 1])));
        }
    }
    if (!server && !host) p.add(ce_prog::signal(drain, 0, true));   // after every stamp landed
    p.load();
    printf("%s: up; %lu B messages, %d rounds a run, %d runs (the first a warm-up)%s\n",
           server ? "server" : "client", (unsigned long) size, ROUNDS, runs,
           host ? "; --host: CPU-started rounds, landing in host memory" : "");

    std::vector<double> oneway;   // ns, counted runs
    int errors = 0;
    for (int r = 0; r < runs && host; r++) {
        char go = 1;   // both sides ready before the run (the server arms by polling)
        if (server ? !write_full(fd, &go, 1) : !read_full(fd, &go, 1)) { printf("FAIL: run barrier\n"); return 1; }
        std::vector<double> run_ns;
        for (int i = 0; i < ROUNDS; i++) {
            const uint64_t v = uint64_t(r) * ROUNDS + uint64_t(i) + 1;
            const auto spin = [&](uint64_t want) {
                const auto t0 = std::chrono::steady_clock::now();
                while (*my_flag < want) {
                    if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(5)) return false;
                }
                return true;
            };
            if (server) {
                if (!spin(v)) { printf("FAIL run %d round %d: no flag %lu\n", r, i, (unsigned long) v); return 1; }
                p.start(v - 1);
                if (!p.finish(std::chrono::milliseconds(1000))) { printf("FAIL: program\n"); return 1; }   // off the timed path
            } else {
                const auto t0 = std::chrono::steady_clock::now();
                p.start(v - 1);
                if (!spin(v)) { printf("FAIL run %d round %d: no reply %lu\n", r, i, (unsigned long) v); return 1; }
                const auto t1 = std::chrono::steady_clock::now();
                run_ns.push_back(std::chrono::duration<double, std::nano>(t1 - t0).count() / 2);
                if (!p.finish(std::chrono::milliseconds(1000))) { printf("FAIL: program\n"); return 1; }  // untimed
            }
        }
        if (!server) {
            printf("%s run %d: one-way median %.0f ns, min %.0f, max %.0f\n", r ? "    " : "warm", r,
                   pct(run_ns, 50), pct(run_ns, 0), pct(run_ns, 100));
            if (r) oneway.insert(oneway.end(), run_ns.begin(), run_ns.end());
        }
        char done = 1;
        if (!write_full(fd, &done, 1) || !read_full(fd, &done, 1)) { printf("FAIL: barrier\n"); return 1; }
    }
    for (int r = 0; r < runs && !host; r++) {
        const uint64_t base = uint64_t(r) * ROUNDS;
        char go = 1;
        if (server) {
            p.start(base);                                   // armed: waits for the client's first flag
            if (!write_full(fd, &go, 1)) { printf("FAIL: armed\n"); return 1; }
        } else {
            if (!read_full(fd, &go, 1)) { printf("FAIL: the server did not arm\n"); return 1; }
            p.start(base);
        }
        const bool ok = p.finish(std::chrono::milliseconds(5000));
        if (!ok) { printf("FAIL run %d: the program did not finish (aborted at entry %lu)\n", r,
                          (unsigned long) v80.getCSR(ce_prog::PC)); errors++; break; }
        if (!server) {
            std::vector<double> run_ns;
            for (int i = 0; i < ROUNDS; i++) run_ns.push_back(double(st[2 * i + 1] - st[2 * i]) * ce_prog::CLK_NS / 2);
            printf("%s run %d: one-way median %.0f ns, min %.0f, max %.0f\n", r ? "    " : "warm", r,
                   pct(run_ns, 50), pct(run_ns, 0), pct(run_ns, 100));
            if (r) oneway.insert(oneway.end(), run_ns.begin(), run_ns.end());
        }
        char done = 1;   // both finished before the next run arms
        if (!write_full(fd, &done, 1) || !read_full(fd, &done, 1)) { printf("FAIL: barrier\n"); return 1; }
    }

    // The last message that landed here, and the flag
    const uint64_t *got = host_msg;
    if (!host) { L->pull(); got = L->buf; }
    uint64_t bad = 0;
    for (uint64_t i = 0; i < size / 8; i++) bad += (got[i] != pattern(8 * i, 1 - side));
    const uint64_t want_flag = uint64_t(runs) * ROUNDS;
    const uint64_t flag = *my_flag;
    const bool flag_ok = (flag == want_flag);
    printf("%s: last message %lu of %lu words wrong, flag %lu (want %lu)\n", server ? "server" : "client",
           (unsigned long) bad, (unsigned long) (size / 8), (unsigned long) flag, (unsigned long) want_flag);
    errors += (bad != 0) + !flag_ok;

    if (!server && !oneway.empty()) {
        printf("one-way %lu B (ns, %zu samples): p0 %.0f  p1 %.0f  p10 %.0f  p50 %.0f  p90 %.0f  p99 %.0f  p99.9 %.0f  max %.0f\n",
               (unsigned long) size, oneway.size(), pct(oneway, 0), pct(oneway, 1), pct(oneway, 10), pct(oneway, 50),
               pct(oneway, 90), pct(oneway, 99), pct(oneway, 99.9), pct(oneway, 100));
        if (!out.empty()) {
            FILE *f = fopen(out.c_str(), "w");
            if (f) { for (double v : oneway) fprintf(f, "%.1f\n", v); fclose(f); }
        }
    }
    ::close(fd);
    loom_switch::release_window(t_qp, 1);
    loom_switch::release_window(t_qp, 2);
    loom_switch::release_export(t_qp, 1);
    loom_switch::release_export(t_qp, 2);
    t_qp.connSync(!server);
    printf(errors ? "PINGPONG FAIL\n" : "PINGPONG PASS\n");
    return errors ? 1 : 0;
}
