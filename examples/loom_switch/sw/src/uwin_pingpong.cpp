/**
 * uwin_pingpong - one-way latency of CPU stores through Loom (E2), host to
 * host: each host lands in its own host buffer and sends through its own
 * U280's user data window (write-combining mapping), so this is the CPU as
 * the endpoint, the like-for-like of an RDMA write ping-pong between hosts.
 *
 *   client, per round i: store the message (8 B stores, sfence), store the
 *                        flag i (sfence); poll own flag == i
 *   server, per round i: poll own flag == i; store the reply and flag i
 *
 * One-way = round trip / 2 on the client's clock. --size 0: the flag only
 * (one 8 B store each way). ROUNDS rounds, the first WARM not counted.
 *
 *   amy:   uwin_pingpong --server [--port N] [--size B] [--rounds R]
 *   rose:  uwin_pingpong --client <amy ip> [--port N] [--size B] [--rounds R] [--out FILE]
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
#include <immintrin.h>
#include <string>
#include <vector>

#include <coyote/cThread.hpp>
#include "loom_switch.hpp"

namespace {

constexpr uint32_t MAGIC = 0x55505047;      // "UPPG"
constexpr uint64_t STAGING_SIZE = 1ULL << 20;
constexpr int WARM = 100;

struct Hello { uint32_t magic; uint32_t dst_ctid; uint64_t staging_va, dst_ref, flag_ref, size; int64_t rounds; };

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
uint64_t pattern(uint64_t off, int side) { return 0x5550000000000000ULL ^ (off * 0x9E3779B97F4A7C15ULL) ^ uint64_t(side); }

}  // namespace

int main(int argc, char **argv) {
    std::string ip, out;
    bool server = false;
    uint16_t port = 19850;
    uint64_t size = 64;
    int rounds = 2100;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--server")                       server = true;
        else if (a == "--client" && i + 1 < argc)  ip = argv[++i];
        else if (a == "--port" && i + 1 < argc)    port = uint16_t(atoi(argv[++i]));
        else if (a == "--size" && i + 1 < argc)    size = strtoull(argv[++i], nullptr, 0);
        else if (a == "--rounds" && i + 1 < argc)  rounds = atoi(argv[++i]);
        else if (a == "--out" && i + 1 < argc)     out = argv[++i];
        else { server = false; ip.clear(); break; }
    }
    if (server == !ip.empty() || size % 8 || size > (1ULL << 20) || rounds <= WARM) {
        printf("usage: %s --server | --client <ip>  [--port N] [--size B, a multiple of 8, 0 = the flag only]"
               " [--rounds R > %d] [--out FILE]\n", argv[0], WARM);
        return 2;
    }
    const int side = server ? 0 : 1;
    const uint64_t mlen = std::max<uint64_t>(4096, (size + 4095) & ~4095ULL);

    coyote::cThread t_qp(0, getpid(), 0, nullptr, "coyote_fpga");     // QP owner, rdma windows, the uwin mapping
    coyote::cThread t_data(0, getpid(), 0, nullptr, "coyote_fpga");   // owns the landing buffers
    printf("%s: QP exchange on port %u ...\n", server ? "server" : "client", port);
    void *staging = server ? t_qp.initRDMA(STAGING_SIZE, port) : t_qp.initRDMA(STAGING_SIZE, port, ip.c_str());
    if (!staging) { printf("FAIL: initRDMA\n"); return 1; }

    uint64_t *dst  = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, mlen}));
    uint64_t *flag = static_cast<uint64_t *>(t_data.getMem({coyote::CoyoteAllocType::HPF, 4096}));
    memset(dst, 0, mlen);
    memset(flag, 0, 4096);
    loom_switch::program_export(t_qp, 1, t_data.getCtid(), dst, mlen);
    loom_switch::program_export(t_qp, 2, t_data.getCtid(), flag, 4096);

    const int fd = connect_or_accept(server ? std::string() : ip, uint16_t(port + 1));
    Hello mine{MAGIC, uint32_t(t_data.getCtid()), reinterpret_cast<uint64_t>(staging),
               loom_switch::export_ref(1), loom_switch::export_ref(2), size, rounds}, h{};
    if (fd < 0 || !write_full(fd, &mine, sizeof(mine)) || !read_full(fd, &h, sizeof(h)) || h.magic != MAGIC ||
        h.size != size || h.rounds != rounds) {
        printf("FAIL: hello (--size and --rounds must match on both sides)\n");
        return 1;
    }
    loom_switch::csr_write(t_qp, loom_switch::RDMA_STAGING_VA, h.staging_va);
    loom_switch::program_window(t_qp, 1, true, t_qp.getCtid(), reinterpret_cast<void *>(h.dst_ref), mlen, 0, h.dst_ctid);
    loom_switch::program_window(t_qp, 2, true, t_qp.getCtid(), reinterpret_cast<void *>(h.flag_ref), 4096, mlen, h.dst_ctid);
    t_qp.mapUwin(mlen + 4096);

    std::vector<uint64_t> msg(std::max<uint64_t>(size / 8, 1));
    for (uint64_t i = 0; i < size / 8; i++) msg[i] = pattern(8 * i, side);
    char go = 1;
    if (!write_full(fd, &go, 1) || !read_full(fd, &go, 1)) { printf("FAIL: barrier\n"); return 1; }

    volatile uint64_t *vflag = flag;
    std::vector<double> oneway;
    bool timeout = false;
    for (int i = 1; i <= rounds && !timeout; i++) {
        const uint64_t v = uint64_t(i);
        if (server) {
            const auto tw = std::chrono::steady_clock::now();
            while (*vflag != v) if (std::chrono::steady_clock::now() - tw > std::chrono::seconds(5)) { timeout = true; break; }
            if (timeout) break;
            if (size) t_qp.uwinWrite(0, msg.data(), size);
            t_qp.uwinWrite(mlen, &v, 8);
        } else {
            const auto t0 = std::chrono::steady_clock::now();
            if (size) t_qp.uwinWrite(0, msg.data(), size);
            t_qp.uwinWrite(mlen, &v, 8);
            while (*vflag != v) if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(5)) { timeout = true; break; }
            const auto t1 = std::chrono::steady_clock::now();
            if (i > WARM) oneway.push_back(std::chrono::duration<double, std::nano>(t1 - t0).count() / 2);
        }
    }
    if (timeout) printf("FAIL: round timed out (flag %lu)\n", (unsigned long) *vflag);

    uint64_t bad = 0;
    for (uint64_t i = 0; i < size / 8; i++) bad += (dst[i] != pattern(8 * i, 1 - side));
    printf("%s: last message %lu of %lu words wrong, flag %lu (want %d)\n", server ? "server" : "client",
           (unsigned long) bad, (unsigned long) (size / 8), (unsigned long) *vflag, rounds);
    const int errors = timeout + (bad != 0) + (*vflag != uint64_t(rounds));
    if (!server && !oneway.empty()) {
        printf("one-way %lu B (ns, %zu samples): p0 %.0f  p1 %.0f  p10 %.0f  p50 %.0f  p90 %.0f  p99 %.0f  p99.9 %.0f  max %.0f\n",
               (unsigned long) size, oneway.size(), pct(oneway, 0), pct(oneway, 1), pct(oneway, 10), pct(oneway, 50),
               pct(oneway, 90), pct(oneway, 99), pct(oneway, 99.9), pct(oneway, 100));
        if (!out.empty()) {
            FILE *f = fopen(out.c_str(), "w");
            if (f) { for (double x : oneway) fprintf(f, "%.1f\n", x); fclose(f); }
        }
    }
    if (!write_full(fd, &go, 1) || !read_full(fd, &go, 1)) printf("(final barrier lost)\n");
    ::close(fd);
    t_qp.unmapUwin();
    loom_switch::release_window(t_qp, 1);
    loom_switch::release_window(t_qp, 2);
    loom_switch::release_export(t_qp, 1);
    loom_switch::release_export(t_qp, 2);
    t_qp.connSync(!server);
    printf(errors ? "UWIN PINGPONG FAIL\n" : "UWIN PINGPONG PASS\n");
    return errors ? 1 : 0;
}
