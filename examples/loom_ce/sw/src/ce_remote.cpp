/**
 * ce_remote - gate G4: the copy engine on one host writes into a buffer on
 * the other, through both switches:
 *
 *   client V80 HBM --loom_ce, P2P--> client U280 uwin --loom_ingress, rdma
 *   route--> RoCE --> server U280 loom_rx --> server host buffer
 *
 * Server (the landing host; U280, and its V80 with --land-v80):
 *     ce_remote --server [--port N] [--size BYTES] [--land-v80]
 *   1. QP exchange (blocks for the client); the QP owner's staging buffer
 *   2. destination and fence buffers on a data cThread
 *   3. a TCP hello to the client: staging VA, both buffers, the data ctid
 *   4. per copy: the client announces the fence value to expect; the
 *      server waits for it in the fence page, checks every byte, answers
 *   With --land-v80 the destination and fence are the server's V80 card
 *   memory: the V80's uwin, exported by its driver, is imported into the
 *   data cThread, so loom_rx lands peer-to-peer into the V80's landing
 *   window. The server waits for the copy's fence store to land and every
 *   card write to complete, syncs the buffer back and checks it.
 *
 * Client (the sending host; U280 + V80):
 *     ce_remote --client <server_ip> [--port N] [--reps N] [--window P]
 *   1. QP exchange, then the hello
 *   2. staging CSR = the server's staging VA; rdma windows onto the
 *      server's destination (uwin 0) and fence page (right after it),
 *      over the QP owner's connection, landing under the server's data ctid
 *   3. that part of the uwin exported to the V80
 *   4. per copy: fill the source, offload it to HBM, announce it, start
 *      loom_ce with the fence behind the data, report the server's answer
 *
 * The QP port is N, the hello's TCP port N + 1 (N defaults to Coyote's).
 * --window sets the client's ack window (packets unacked, TX_CTL; 16 after
 * reset). Each side prints where its switch's cycles went during a copy:
 * the client its waits on the window and on sq_wr, the server loom_rx's
 * moving / starved / stalled cycles; the client also its U280 ingress's
 * debug counters and its copy engine's (V80 words 48-50), a V80-landing
 * server the landing's (24-29, 32-45).
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

enum CeReg : uint32_t { START = 0, SRC_VA = 1, DST_VA = 2, LEN = 3, PID = 4, FENCE_VA = 5,
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
const char *const LAND_NAMES[6]  = {"land reqs", "land done", "land bursts", "land drops", "land stores", "land partial"};
const char *const LDBG_NAMES[14] = {"land out bp", "", "land fifo empty", "land W: no aw", "land W: B slot",
                                    "land W: fifo", "land W: queue", "land W: stores", "land drop: no win",
                                    "land drop: end", "land 1 beat", "land 2-4", "land >4", "land misaligned"};
const char *const CE_NAMES[3]    = {"CE out bp", "CE in wait", "sq_wr wait"};
void snap(coyote::cThread &v80, uint64_t *out, uint32_t first, int n) {
    for (int i = 0; i < n; i++) out[i] = v80.getCSR(first + i);
}

int run_server(uint16_t port, uint64_t size, bool land) {
    coyote::cThread t_qp(0, getpid(), 0, nullptr, "coyote_fpga");     // QP owner
    coyote::cThread t_data(0, getpid(), 0, nullptr, "coyote_fpga");   // owns the landing buffers
    printf("server: waiting for the QP exchange on port %u ...\n", port);
    void *staging = t_qp.initRDMA(STAGING_SIZE, port);
    if (!staging) { printf("FAIL: initRDMA\n"); return 1; }
    loom_switch::csr_write(t_qp, loom_switch::RDMA_STAGING_VA, reinterpret_cast<uint64_t>(staging));

    uint64_t *dst, *fence;
    coyote::cThread *v80 = nullptr;
    land_v80::Landing *L = nullptr;
    uint64_t stores_base = 0;
    if (land) {
        v80 = new coyote::cThread(0, getpid(), 0, nullptr, "coyote_versal_fpga");
        L = new land_v80::Landing(*v80, size + 4096);
        const int lfd = L->export_fd();
        void *lva = loom_switch::reserve_va(size + 4096);
        t_data.importDmabuf(lfd, lva);
        dst   = static_cast<uint64_t *>(lva);
        fence = dst + size / 8;
        stores_base = L->csr(land_v80::LAND_STORES);
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
    int c = ::accept(lfd, nullptr, nullptr);
    Hello h{MAGIC, uint32_t(t_data.getCtid()), reinterpret_cast<uint64_t>(staging),
            reinterpret_cast<uint64_t>(dst), reinterpret_cast<uint64_t>(fence), size};
    if (!write_full(c, &h, sizeof(h))) { printf("FAIL: hello\n"); return 1; }

    volatile uint64_t *vfence = fence;
    volatile uint64_t *vdst = dst;
    int errors = 0, copies = 0;
    Announce an;
    uint64_t l0[6], d0[14];
    while (read_full(c, &an, sizeof(an)) && an.rep != ~0ULL) {
        const Counters k0 = Counters::read(t_qp);
        if (land) { snap(*v80, l0, land_v80::LAND_REQS, 6); snap(*v80, d0, 32, 14); }
        auto t0 = std::chrono::steady_clock::now();
        Verdict v{};
        if (land) {
            // one fence store per copy so far, then every card write complete
            (void) L->wait(stores_base + copies);
            v.wait_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
            L->pull();
            vdst = L->buf;
            v.fence = L->buf[size / 8];
        } else {
            while (*vfence != an.fence && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5))
                _mm_pause();
            v.fence = *vfence;
            v.wait_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
        }
        const Counters k1 = Counters::read(t_qp);
        if (v.fence == an.fence)
            for (uint64_t i = 0; i < size / 8; i++) v.bad += (vdst[i] != pattern(8 * i, an.rep));
        else
            v.bad = size / 8;
        printf("%s copy %lu: fence %lu (expected %lu), %lu of %lu words wrong, fence %.1f us after the announce (%.2f GB/s)\n",
               (v.bad ? "FAIL" : "ok  "), (unsigned long) an.rep, (unsigned long) v.fence,
               (unsigned long) an.fence, (unsigned long) v.bad, (unsigned long) (size / 8),
               v.wait_us, size / v.wait_us / 1e3);
        printf("     loom_rx over %lu cycles: %lu moving, %lu starved (nothing arrived), %lu stalled (host write not ready), rx FIFO full %lu\n",
               (unsigned long) k1.d(k0, C_CYC), (unsigned long) k1.d(k0, C_RX_MOVE), (unsigned long) k1.d(k0, C_RX_STARVE),
               (unsigned long) k1.d(k0, C_RX_STALL), (unsigned long) k1.d(k0, C_RX_FF));
        if (land) {
            printf("     V80 landing:\n");
            print_v80_words(*v80, l0, land_v80::LAND_REQS, 6, LAND_NAMES);
            print_v80_words(*v80, d0, 32, 14, LDBG_NAMES);
        }
        errors += (v.bad != 0);
        copies++;
        // the next copy must write every byte again
        if (land) L->clear();
        else      memset(dst, 0, size);
        if (!write_full(c, &v, sizeof(v))) break;
    }
    printf("server: loom_rx forwarded %lu writes, rejected %lu headers\n",
           (unsigned long) loom_switch::csr_read(t_qp, loom_switch::RX_FWD),
           (unsigned long) loom_switch::csr_read(t_qp, loom_switch::RX_DROP));
    ::close(c);
    ::close(lfd);
    t_qp.connSync(false);
    delete L;
    delete v80;
    printf(errors || !copies ? "G4 FAIL (server)\n" : "G4 PASS (server)\n");
    return errors || !copies ? 1 : 0;
}

int run_client(const std::string &ip, uint16_t port, int reps, int window) {
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
        for (uint64_t i = 0; i < size / 8; i++) src[i] = pattern(8 * i, r);
        v80.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{src, size});

        const uint64_t fence = v80.getCSR(COPIES) + 1;
        loom_switch::IngressCounters c0 = loom_switch::IngressCounters::read(u280);
        const Counters k0 = Counters::read(u280);
        uint64_t e0[3];
        snap(v80, e0, 48, 3);
        Announce an{uint64_t(r), fence};
        write_full(fd, &an, sizeof(an));

        v80.setCSR(reinterpret_cast<uint64_t>(src), SRC_VA);
        v80.setCSR(reinterpret_cast<uint64_t>(uva), DST_VA);
        v80.setCSR(size, LEN);
        v80.setCSR(v80.getCtid(), PID);
        v80.setCSR(reinterpret_cast<uint64_t>(uva) + size, FENCE_VA);
        v80.setCSR(1, START);

        Verdict v{};
        if (!read_full(fd, &v, sizeof(v))) { printf("FAIL: the server went away\n"); return 1; }
        loom_switch::IngressCounters c1 = loom_switch::IngressCounters::read(u280);
        const Counters k1 = Counters::read(u280);
        printf("%s copy %d: %lu bytes, server saw the fence after %.1f us, %lu words wrong; CE %lu cycles\n",
               (v.bad ? "FAIL" : "ok  "), r, (unsigned long) size, v.wait_us, (unsigned long) v.bad,
               (unsigned long) v80.getCSR(CYCLES));
        printf("     ingress: %lu bursts, %lu rdma packets, %lu stores, %lu dropped; acks %lu, unacked %lu\n",
               (unsigned long) (c1.v[0] - c0.v[0]), (unsigned long) (c1.v[3] - c0.v[3]),
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
    bool land = false;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--server")                     server = true;
        else if (a == "--client" && i + 1 < argc) ip = argv[++i];
        else if (a == "--port" && i + 1 < argc)   port = uint16_t(atoi(argv[++i]));
        else if (a == "--size" && i + 1 < argc)   size = strtoull(argv[++i], nullptr, 0);
        else if (a == "--reps" && i + 1 < argc)   reps = atoi(argv[++i]);
        else if (a == "--window" && i + 1 < argc) window = atoi(argv[++i]);
        else if (a == "--land-v80")               land = true;
        else { server = false; ip.clear(); break; }
    }
    if (server == !ip.empty() || size == 0 || size % 4096 || size > (64ULL << 20)) {
        printf("usage: %s --server [--port N] [--size BYTES] [--land-v80] | --client <server_ip> [--port N] [--reps N] [--window P]\n"
               "       (size: a multiple of 4 KiB, at most 64 MiB)\n", argv[0]);
        return 2;
    }
    return server ? run_server(port, size, land) : run_client(ip, port, reps, window);
}
