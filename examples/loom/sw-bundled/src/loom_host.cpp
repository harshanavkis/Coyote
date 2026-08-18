/**
 * 6.2a: the bundled two-host binary - ONE process per host carrying the
 * daemon role (InProcOrchestrator-equivalent backend + local loomd on a
 * Unix socket + loomd<->loomd TCP peering) and that side's app roles as
 * threads. Two processes total across the cluster: the cross-host
 * bring-up vehicle.
 *
 *   server (exporter side, host 2):
 *     loom_host --server [--qp-port N] [--peer-port N] [--sock PATH]
 *   client (importer side, host 1):
 *     loom_host --client <server_ip> [--qp-port N] [--peer-port N] [--sock PATH]
 *
 * Setup sequence (order matters - initRDMA blocks on the QP exchange):
 *   server: initRDMA(qp_port) [blocks] -> program local staging CSR
 *           (loom_rx dispatch compare) -> export dst segments -> start
 *           local loomd + PeerServer -> poll for the client's writes ->
 *           wait DONE -> verify, report.
 *   client: initRDMA(qp_port, ip) [unblocks server] -> PeerClient
 *           connect (retry; hello carries the server's staging VA) ->
 *           program staging CSR + import handles as rdma windows ->
 *           stores/DMAs/fence/ordering flow -> DONE -> report.
 *
 * QP topology (bring-up scope): ONE RC connection - the client's data
 * cThread paired with the server's data cThread (initRDMA on each; the
 * QP rides their ctids). Both imported windows use that connection.
 * Per-binding QPs and multiple exporter processes are 6.2b/full 6.1.
 * The data buffers live on the server's data cThread: incoming RETH VAs
 * and message-header VAs translate under the QP owner's pid, so the
 * exporter cThread must own the destination memory.
 *
 * EXECUTION is hardware-only (two hosts; sim has no networking - the
 * mock's initRDMA asserts). The binary compiles under EN_SIM but exits
 * early with a message if COYOTE_SIM_DIR is set. FPGA-free coverage of
 * the peering protocol lives in test_peering.cpp.
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>

#include <coyote/cThread.hpp>
#include "loom.hpp"
#include "loom_orch.hpp"
#include "loom_xpu.hpp"
#include "loomd.hpp"
#include "loom_peer.hpp"
#include "loom_bundle.hpp"

namespace {

constexpr uint64_t BUF_SIZE      = 2ULL * 1024 * 1024;
constexpr uint64_t DMA_BYTES     = 4096;        // rdma bulk: len % 64 == 0
constexpr uint32_t STAGING_BYTES = 4096;
// Bring-up: a poll that is going to fail should fail fast enough that the
// counter dump after it is worth reading. LOOM_POLL_SECS overrides.
int poll_secs() {
    const char *e = getenv("LOOM_POLL_SECS");
    return e ? atoi(e) : 20;
}

int failures = 0;

// Bring-up switch: run the store path with no bulk copy in front of it.
// The far side stops forwarding at the transaction after the first copy, so
// this splits "the store path is broken" from "a preceding direct write
// wedges the receiver". Both sides must be started with it set.
bool skip_bulk() { return getenv("LOOM_SKIP_BULK") != nullptr; }

// The vFPGA's own account of what it did, which is the only first-hand
// evidence when a write does not arrive: dbg[4] counts transactions loom_rx
// forwarded (this test should produce exactly 5 - three wire messages and
// two direct bulk writes), dbg[9] counts headers it refused to translate,
// and on the issuing side dbg[0]/dbg[3]/dbg[5] say what the engine captured,
// put on the wire, and dropped.
void dump_counters(coyote::cThread &t, const char *tag) {
    static const char *name[10] = {
        "stores", "descs", "local_wr", "rdma_wr", "rx_fwd",
        "drops", "fifo_ovfl", "compl", "reads", "rx_hdr_reject"
    };
    printf("counters [%s]:", tag);
    for (int i = 0; i < 10; i++)
        printf(" %s=%lu", name[i],
               (unsigned long) loom::csr_read(t, loom::DBG_BASE + 8 * i));
    printf("\n");
    fflush(stdout);
}

void check(bool ok, const char *msg) {
    printf("%s: %s\n", ok ? "PASS" : "FAIL", msg);
    if (!ok) failures++;
}

bool poll64(volatile uint64_t *addr, uint64_t want) {
    for (int i = 0; i < poll_secs() * 100; i++) {
        if (*addr == want) return true;
        usleep(10000);
    }
    return false;
}

uint64_t src_word(uint64_t i) { return 0x5A5A'0000'0000'0000ULL | i; }

bool payload_matches(const uint64_t *dst_words) {
    for (uint64_t i = 0; i < DMA_BYTES / 8; i++)
        if (dst_words[i] != src_word(i)) return false;
    return true;
}

// Poll until the DMA payload has fully landed (RC delivers in order, but
// the poll may catch a partially-written buffer - wait for the last word
// first, then verify the whole range)
bool poll_payload(volatile uint64_t *dst_words) {
    if (!poll64(&dst_words[DMA_BYTES / 8 - 1], src_word(DMA_BYTES / 8 - 1)))
        return false;
    return payload_matches(const_cast<const uint64_t *>(dst_words));
}

int run_server(uint16_t qp_port, uint16_t peer_port, const std::string &sock) {
    coyote::cThread t_ctrl(0, getpid(), 0);
    coyote::cThread t_data(0, getpid(), 0);
    printf("ctids: ctrl %d, data %d\n", t_ctrl.getCtid(), t_data.getCtid());

    // QP exchange (blocks until the client's initRDMA connects); the
    // returned buffer is this host's RDMA staging area
    printf("server: waiting for QP exchange on port %u ...\n", qp_port);
    void *staging = t_data.initRDMA(STAGING_BYTES, qp_port);
    if (!staging) { printf("FAIL: initRDMA\n"); return 1; }
    printf("server: QP up, staging %p\n", staging);

    // loom_rx recognizes wire messages by RETH == staging
    loom::set_rdma_staging(t_ctrl, staging);

    // Exporter role: two destination segments under the QP owner's ctid
    auto *dst1 = static_cast<uint64_t *>(
        t_data.getMem({coyote::CoyoteAllocType::HPF, BUF_SIZE}));
    auto *dst2 = static_cast<uint64_t *>(
        t_data.getMem({coyote::CoyoteAllocType::HPF, BUF_SIZE}));
    if (!dst1 || !dst2) { printf("FAIL: getMem\n"); return 1; }
    memset(dst1, 0, BUF_SIZE);
    memset(dst2, 0, BUF_SIZE);

    loom::BundledOrchestrator orch(t_ctrl);
    loom::Handle h1 = orch.exportBuf(t_data.getCtid(), dst1, BUF_SIZE);
    loom::Handle h2 = orch.exportBuf(t_data.getCtid(), dst2, BUF_SIZE);
    printf("server: exported handles %u, %u\n", h1, h2);

    // Daemon role: local loomd (external attachers) + the peering server
    loom::Loomd loomd(orch, sock);
    if (!loomd.start()) { printf("FAIL: loomd start\n"); return 1; }
    std::thread loomd_thr([&] { loomd.run(); });

    loom::PeerServer peer(orch, reinterpret_cast<uint64_t>(staging), peer_port);
    if (!peer.start()) { printf("FAIL: peer server start\n"); return 1; }
    std::thread peer_thr([&] { peer.run(); });
    printf("server: peering on port %u, loomd on %s\n", peer.port(), sock.c_str());

    // Verify the client's traffic as it lands
    dump_counters(t_ctrl, "server idle");
    check(poll64(&dst1[8], 0xB00B'0000'0000'0001ULL), "store via w1 lands");
    dump_counters(t_ctrl, "after store w1");
    check(poll64(&dst2[8], 0xB00B'0000'0000'0002ULL), "store via w2 lands");
    dump_counters(t_ctrl, "after store w2");
    if (!skip_bulk()) {
        check(poll_payload(&dst1[0x10000 / 8]), "bulk payload @0x10000 matches");
        dump_counters(t_ctrl, "after bulk @0x10000");
    }
    // Ordering: when the flag (issued AFTER the second copy) is visible,
    // the second copy's payload must already be complete - RC in-order
    // delivery extends the order-FIFO guarantee across hosts
    check(poll64(&dst2[0x800 / 8], 0xF1A6ULL), "ordering flag lands");
    dump_counters(t_ctrl, "after ordering flag");
    if (!skip_bulk())
        check(payload_matches(&dst1[0x20000 / 8]),
              "flag implies bulk payload @0x20000 complete (cross-host order)");

    // Where did the second copy actually land? A framing slip shows up as a
    // shift, so report the first mismatching word rather than just "no"
    {
        const uint64_t *p = &dst1[0x20000 / 8];
        for (uint64_t i = 0; i < DMA_BYTES / 8; i++)
            if (p[i] != src_word(i)) {
                printf("  @0x20000 first mismatch at word %lu: got %016lx, "
                       "want %016lx\n", (unsigned long) i,
                       (unsigned long) p[i], (unsigned long) src_word(i));
                break;
            }
        printf("  dst2[0x800] = %016lx (flag), dst2[0x40] = %016lx\n",
               (unsigned long) dst2[0x800 / 8], (unsigned long) dst2[8]);
        // One aperture store becomes eight wire messages: the real one plus
        // seven padding writes to +8..+56 of the same 64 B line. Whether
        // those carry zeros or garbage decides whether every peer store is
        // clobbering its neighbours (which G5 claims it does not)
        printf("  dst2[0x800..0x838] =");
        for (int i = 0; i < 8; i++)
            printf(" %016lx", (unsigned long) dst2[0x800 / 8 + i]);
        printf("\n");
        fflush(stdout);
    }

    // End-of-run barrier, then teardown
    for (int i = 0; i < poll_secs() * 100 && peer.doneCount() < 1; i++)
        usleep(10000);
    check(peer.doneCount() >= 1, "client DONE received");
    dump_counters(t_ctrl, "server final");

    t_data.connSync(false);
    peer.stop(); peer_thr.join();
    loomd.stop(); loomd_thr.join();

    printf(failures == 0 ? "LOOM HOST SERVER PASS\n"
                         : "LOOM HOST SERVER FAIL (%d)\n", failures);
    return failures == 0 ? 0 : 1;
}

int run_client(const std::string &ip, uint16_t qp_port, uint16_t peer_port,
               const std::string &sock) {
    coyote::cThread t_ctrl(0, getpid(), 0);
    coyote::cThread t_data(0, getpid(), 0);
    printf("ctids: ctrl %d, data %d\n", t_ctrl.getCtid(), t_data.getCtid());

    // QP exchange (the server is already blocking in its initRDMA)
    void *staging_local = t_data.initRDMA(STAGING_BYTES, qp_port, ip.c_str());
    if (!staging_local) { printf("FAIL: initRDMA\n"); return 1; }
    printf("client: QP up\n");

    // Peering: retry while the server brings its listener up
    loom::PeerClient peer;
    bool connected = false;
    for (int i = 0; i < 300 && !connected; i++) {
        connected = peer.connect(ip, peer_port);
        if (!connected) usleep(100000);
    }
    check(connected, "peering connected (hello received)");
    if (!connected) return failures;
    check(peer.stagingVa() != 0, "hello carries server staging VA");

    // Daemon role backend: remote imports through the peer, staging CSR
    // programmed from the hello; local loomd for external attachers
    loom::BundledOrchestrator orch(t_ctrl);
    orch.attachPeer(peer, t_data.getCtid());
    loom::Loomd loomd(orch, sock);
    if (!loomd.start()) { printf("FAIL: loomd start\n"); return 1; }
    std::thread loomd_thr([&] { loomd.run(); });

    // App role: the importer Xpu on the data cThread
    std::mutex io_mtx;
    loom::Xpu A(t_data, orch, io_mtx);

    int w1 = A.importBuf(1);
    int w2 = A.importBuf(2);
    check(w1 == 1 && w2 == 2, "remote import -> rdma windows 1, 2");
    check(A.importBuf(1234) == loom::NO_WINDOW, "bogus handle refused remotely");

    auto *src = static_cast<uint64_t *>(A.alloc(BUF_SIZE));
    auto *fence = static_cast<uint64_t *>(A.allocSmall(4096));
    if (!src || !fence) { printf("FAIL: alloc\n"); return failures + 1; }
    memset(fence, 0, 4096);
    for (uint64_t i = 0; i < BUF_SIZE / 8; i++) src[i] = src_word(i);

    // Small stores through both windows (64 B inline wire messages)
    A.store(w1, 0x40, 0xB00B'0000'0000'0001ULL);
    A.store(w2, 0x40, 0xB00B'0000'0000'0002ULL);

    // Bulk with fence (direct RDMA WRITE; fence = local posted completion)
    if (!skip_bulk()) {
        A.copy(w1, 0x10000, src, DMA_BYTES, fence);
        check(poll64(&fence[0], 1), "fence 1 after copy (posted completion)");
    } else {
        printf("LOOM_SKIP_BULK: no copies, stores only\n");
    }

    // Ordering across hosts: copy, then flag through the other window;
    // both ride the same QP, RC keeps them in order at the far side
    dump_counters(t_ctrl, "client before ordering copy");
    if (!skip_bulk()) {
        A.copy(w1, 0x20000, src, DMA_BYTES, fence);
        A.store(w2, 0x800, 0xF1A6ULL);
        check(poll64(&fence[0], 2), "fence 2 after ordering copy");
    } else {
        A.store(w2, 0x800, 0xF1A6ULL);
    }
    dump_counters(t_ctrl, "client after ordering copy + flag store");

    // Release: window invalidated at the SOURCE - the engine drops the
    // store before anything reaches the wire (counted in dbg[drops])
    uint64_t drops0 = loom::csr_read(t_ctrl, loom::DBG_BASE + 8 * 5);
    orch.releaseWindow(w2);
    A.store(w2, 0x40, 0xDEADULL);
    bool dropped = false;
    for (int i = 0; i < poll_secs() * 10 && !dropped; i++) {
        dropped = loom::csr_read(t_ctrl, loom::DBG_BASE + 8 * 5) >= drops0 + 1;
        if (!dropped) usleep(10000);
    }
    check(dropped, "store to released window dropped at source");
    dump_counters(t_ctrl, "client final");

    check(peer.done(), "DONE barrier acknowledged");
    t_data.connSync(true);
    loomd.stop(); loomd_thr.join();

    printf(failures == 0 ? "LOOM HOST CLIENT PASS\n"
                         : "LOOM HOST CLIENT FAIL (%d)\n", failures);
    return failures == 0 ? 0 : 1;
}

} // namespace

int main(int argc, char **argv) {
    if (getenv("COYOTE_SIM_DIR")) {
        printf("loom_host is hardware-only: the simulation backend has no "
               "networking (initRDMA asserts). FPGA-free protocol coverage: "
               "./test_peering\n");
        return 2;
    }

    bool server = false;
    std::string ip, sock = "/tmp/loomd-bundled.sock";
    uint16_t qp_port = coyote::DEF_PORT;
    uint16_t peer_port = coyote::DEF_PORT + 1;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--server")                       server = true;
        else if (a == "--client" && i + 1 < argc)  ip = argv[++i];
        else if (a == "--qp-port" && i + 1 < argc) qp_port = static_cast<uint16_t>(atoi(argv[++i]));
        else if (a == "--peer-port" && i + 1 < argc) peer_port = static_cast<uint16_t>(atoi(argv[++i]));
        else if (a == "--sock" && i + 1 < argc)    sock = argv[++i];
        else {
            printf("usage: %s --server | --client <server_ip> "
                   "[--qp-port N] [--peer-port N] [--sock PATH]\n", argv[0]);
            return 2;
        }
    }
    if (server != ip.empty()) {   // exactly one of --server / --client
        printf("usage: %s --server | --client <server_ip> "
               "[--qp-port N] [--peer-port N] [--sock PATH]\n", argv[0]);
        return 2;
    }

    return server ? run_server(qp_port, peer_port, sock)
                  : run_client(ip, qp_port, peer_port, sock);
}
