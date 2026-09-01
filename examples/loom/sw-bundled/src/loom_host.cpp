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
#include <chrono>
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

// 16 MB: the benchmark sweep packs transfers up to 4 MB nose to tail from
// 0x40000, which ends at 5.6 MB, and the window bounds check drops anything
// past the segment length
constexpr uint64_t BUF_SIZE      = 16ULL * 1024 * 1024;
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

// -------------------------------------------------------------------------
// Remote benchmark plan. Both sides compile the same table, so the exporter
// knows where every transfer should have landed without being told.
//
// What this can and cannot measure. The fence an rdma descriptor releases is
// a LOCAL posted completion: it fires when the engine has finished streaming
// the payload to the network, not when the far side has it. There is no
// return path in 6.2a - the peering is one-directional and remote reads are
// 6.2b - so round-trip latency is not available. What is available is the
// transmit pipeline: the engine's own per-stage cycles (t-encap for stores,
// dma-rdma for bulk) and the achieved bytes per second through it. Those are
// exactly the T2/T3 quantities the simulator carries placeholders for; a
// remote-read RTT (T6) has to wait for 6.2b.
//
// Sizes are multiples of 64 B because the rdma bulk route requires it by
// contract (loom_engine drops the rest at the source).
constexpr uint64_t BENCH_SIZES[] = {
    64, 256, 1024, 4096, 16384, 65536, 262144, 1048576, 4194304
};
constexpr int BENCH_ITERS = 32;      // per size, issued back to back
constexpr uint64_t BENCH_MAX_BURST = 4ULL * 1024 * 1024;   // bytes in flight

// Bound the burst by bytes, not by count: 32 x 4 MB is 128 MB staged as
// fast as the CSR path allows, far past anything the far side sustains.
int bench_iters(uint64_t len) {
    // LOOM_BENCH_ITERS=1 issues ONE descriptor per size. Credit pacing only
    // controls the gap between descriptors; inside a single 256 KB message
    // the shell still emits 64 back-to-back PMTU packets, and 64 KB - which
    // works - is 16. If one descriptor on its own fails, the limit is the
    // burst within a message and no software pacing can reach it.
    const char *e = getenv("LOOM_BENCH_ITERS");
    if (e) { int v = atoi(e); return v > 0 ? v : 1; }
    uint64_t n = BENCH_MAX_BURST / (len ? len : 1);
    if (n > BENCH_ITERS) n = BENCH_ITERS;
    return n ? int(n) : 1;
}

// Cap on descriptors in flight. loom_engine tracks nothing: vfpga_top ties
// cq_wr.ready high and discards every completion, so the engine drains the
// order FIFO as fast as sq_wr is accepted and the number of outstanding
// RDMA writes is bounded by nothing at all. The shell's budget is
// RDMA_N_WR_OUTSTANDING = 16. Pacing here in software says whether that is
// what the large sizes hit: if they come back clean under a cap, the fix is
// credit tracking in the engine, not bandwidth and not the receive path.
int bench_gap_us() {
    const char *e = getenv("LOOM_BENCH_GAP_US");
    return e ? atoi(e) : 0;
}
int bench_credit() {
    const char *e = getenv("LOOM_BENCH_CREDIT");
    return e ? atoi(e) : 0;                  // 0 = unlimited, as today
}
constexpr int BENCH_STORES = 256;    // inline messages for the store rate

// Where each size's last iteration lands, packed nose to tail
uint64_t bench_offset(int idx) {
    uint64_t off = 0x40000;          // clear of the correctness checks
    for (int i = 0; i < idx; i++) off += BENCH_SIZES[i];
    return (off + 63) & ~63ULL;
}
uint64_t bench_word(int idx, uint64_t i) {
    return (uint64_t(0xBE0 + idx) << 48) | i;
}
constexpr int BENCH_N = int(sizeof(BENCH_SIZES) / sizeof(BENCH_SIZES[0]));

bool bench_mode() { return getenv("LOOM_BENCH") != nullptr; }

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

// Spin, no sleeping: the benchmark's unit of time is microseconds, and
// poll64's 10 ms usleep granularity swamped every size below a megabyte -
// it reported the sleep, not the transfer.
// Wait for the fence to reach AT LEAST `want`: pacing needs the window to
// have drained enough, not to have drained exactly
bool spin64_ge(volatile uint64_t *addr, uint64_t want, double timeout_us) {
    auto t0 = std::chrono::steady_clock::now();
    while (*addr < want) {
        if (std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - t0).count() > timeout_us)
            return false;
    }
    return true;
}

bool spin64(volatile uint64_t *addr, uint64_t want, double timeout_us) {
    auto t0 = std::chrono::steady_clock::now();
    while (*addr != want) {
        if (std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - t0).count() > timeout_us)
            return false;
    }
    return true;
}

// One field out of the shell's network counters. The receive path losing
// packets shows up here and nowhere else in this program: a drop is a
// packet loom_rx did not forward, and RC then retransmits it.
long net_stat(const char *field) {
    FILE *f = fopen("/sys/kernel/coyote_sysfs_0/cyt_attr_nstats", "r");
    if (!f) return -1;
    char line[256];
    long v = -1;
    while (fgets(line, sizeof(line), f))
        if (strstr(line, field)) { 
            const char *c = strchr(line, ':');
            if (c) v = atol(c + 1);
            break;
        }
    fclose(f);
    return v;
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

// Issue BENCH_ITERS descriptors of each size back to back and wait for the
// last fence, so the measurement covers a pipeline in steady state rather
// than a series of round trips through software.
void run_bench(coyote::cThread &t_ctrl, loom::Xpu &A, int win,
               uint64_t *src, volatile uint64_t *fence) {
    printf("\n== remote transmit benchmark (<=%d iters/size, <=%lu MB burst, "
           "credit %d)\n", BENCH_ITERS,
           (unsigned long) (BENCH_MAX_BURST >> 20), bench_credit());
    printf("%10s %6s %10s %10s %12s %10s %8s %8s %10s\n",
           "bytes", "iters", "cyc/op", "queue_cyc", "us/op", "GB/s",
           "retrans", "psndrop", "landed");
    // The per-size columns below bracket only the timed loop, and an RC
    // retransmit timer fires well after the payload has been streamed - so
    // they read zero on runs that go on to retransmit over a thousand
    // packets. Bracket the whole benchmark as well, and sample after a
    // settle so the timers have actually run.
    const long rt_all0 = net_stat("Retrans cnt"), pd_all0 = net_stat("PSN drop cnt");

    // LOOM_BENCH_ONLY=<bytes> runs one size and nothing else. The sweep
    // runs sizes in order, so a failure at 256 KB may be that size or may
    // be everything before it having degraded the QP - this separates them.
    const char *only = getenv("LOOM_BENCH_ONLY");
    const uint64_t only_len = only ? strtoull(only, nullptr, 0) : 0;

    for (int i = 0; i < BENCH_N; i++) {
        const uint64_t len = BENCH_SIZES[i];
        const uint64_t off = bench_offset(i);
        if (only_len && len != only_len) continue;

        // Distinct pattern per size so the exporter can tell them apart
        for (uint64_t w = 0; w < len / 8; w++) src[w] = bench_word(i, w);

        const uint64_t warm = loom::csr_read(t_ctrl, loom::DBG_BASE + 8 * 7);
        A.copy(win, uint32_t(off), src, len, fence);      // warm the path
        if (!spin64(fence, warm + 1, 5e6)) {
            printf("%10lu   warm-up never fenced - stopping\n",
                   (unsigned long) len);
            printf("  a single descriptor of this size does not complete; "
                   "its recovery has been seen splattering into the region "
                   "below, so later rows and the store rate would both "
                   "measure a wrecked QP\n");
            break;
        }

        const uint64_t base = warm + 1;
        const long rt0 = net_stat("Retrans cnt"), pd0 = net_stat("PSN drop cnt");
        loom::StageStats a = loom::read_stage_stats(t_ctrl);
        auto t0 = std::chrono::steady_clock::now();
        const int iters  = bench_iters(len);
        const int credit = bench_credit();
        const int gap_us = bench_gap_us();
        for (int k = 0; k < iters; k++) {
            A.copy(win, uint32_t(off), src, len, fence);
            if (credit && k + 1 > credit)          // at most `credit` unretired
                spin64_ge(fence, base + uint64_t(k + 1 - credit), 5e6);
            // Pace against the RECEIVER, which nothing else here does. The
            // fence is a local posted completion, so with credit 0 the
            // descriptors go out back to back at whatever rate the engine
            // sustains - 12.3 GB/s on hardware, above the ~11.8 GB/s
            // perf_rdma settles at because ITS benchmark is throttled by
            // waiting for the server to echo. Idling between messages tests
            // whether the loss is back-to-back pressure: ITERS=1 is clean at
            // the same instantaneous rate, so a gap should make ITERS=N look
            // like N separate clean runs. It does NOT reduce the rate within
            // a message, so loss occurring inside one will survive it.
            if (gap_us) {
                spin64_ge(fence, base + uint64_t(k + 1), 5e6);
                usleep(useconds_t(gap_us));
            }
        }
        // Generous but bounded: a size that cannot keep up should report,
        // not hold the run for the poll timeout
        bool ok = spin64(fence, base + iters, 5e6);
        auto t1 = std::chrono::steady_clock::now();
        loom::StageStats b = loom::read_stage_stats(t_ctrl);
        const long rt1 = net_stat("Retrans cnt"), pd1 = net_stat("PSN drop cnt");

        double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        uint64_t cyc = b.acc[loom::STG_DMA_RDMA] - a.acc[loom::STG_DMA_RDMA];
        uint64_t ops = b.cnt[loom::STG_DMA_RDMA] - a.cnt[loom::STG_DMA_RDMA];
        uint64_t q   = b.queue_acc - a.queue_acc;
        printf("%10lu %6d %10lu %10lu %12.2f %10.3f %8ld %8ld %10s\n",
               (unsigned long) len, iters,
               (unsigned long) (ops ? cyc / ops : 0),
               (unsigned long) (ops ? q / ops : 0),
               us / iters,
               (double(len) * iters) / (us * 1e3),
               rt1 - rt0, pd1 - pd0,
               ok ? "yes" : "NO FENCE");

        // Once the wire has lost a packet the QP is compromised: stop.
        // Every later row would measure the recovery rather than the
        // pipeline, and a replayed write has been seen landing at the
        // wrong offset - the exporter reports that as a corrupt region
        // for a size that was never the problem.
        if (rt1 > rt0 || pd1 > pd0 || !ok) {
            printf("  receive path gave out here; later sizes would "
                   "measure RC recovery, not the pipeline\n");
            break;
        }
    }

    // The corrupting write seen at 256 KB carries store #255's payload and
    // lands at word 0 of the bulk region - the START, so a request holding
    // the bulk's vaddr took it having received none of its own beats. That
    // is a bulk-to-store transition, not a short bulk transfer. Skip the
    // stores and the region should stay clean if that reading is right.
    if (getenv("LOOM_BENCH_NO_STORES")) {
        printf("LOOM_BENCH_NO_STORES: skipping the inline store phase\n");
    {
        // Where the transmit stream's cycles went. Only the rdma route is
        // counted. A gap here is one Loom put into the outgoing packet
        // stream; a stall is the shell declining to take a beat.
        const uint64_t mv = loom::csr_read(t_ctrl, loom::TX_MOVE);
        const uint64_t sv = loom::csr_read(t_ctrl, loom::TX_STARVE);
        const uint64_t st = loom::csr_read(t_ctrl, loom::TX_STALL);
        const uint64_t tot = mv + sv + st;
        printf("transmit path cycles: %lu moving, %lu starved (host pull dry), "
               "%lu stalled (network pushing back)\n",
               (unsigned long) mv, (unsigned long) sv, (unsigned long) st);
        if (tot)
            printf("  %.1f%% moving, %.1f%% starved, %.1f%% stalled -> %s\n",
                   100.0 * double(mv) / double(tot),
                   100.0 * double(sv) / double(tot),
                   100.0 * double(st) / double(tot),
                   sv > st ? "the pull is gapping the outgoing stream"
                           : "the fabric is pushing back, which is expected");
    }

    usleep(500000);      // let RC retransmit timers fire before sampling
    {
        const long rt = net_stat("Retrans cnt") - rt_all0;
        const long pd = net_stat("PSN drop cnt") - pd_all0;
        printf("whole run: %ld retransmissions, %ld PSN drops%s\n", rt, pd,
               (rt || pd) ? "  <-- the per-size columns above sample too "
                            "early to show these" : "");
    }
        fflush(stdout);
        return;
    }

    // Inline stores have no fence of their own; the engine's rdma-write
    // counter advancing by the number issued is what says they are gone
    {
        const uint64_t w0 = loom::csr_read(t_ctrl, loom::DBG_BASE + 8 * 3);
        // The order FIFO is 64 deep and DROPS a push when it is full rather
        // than stalling the AXI-Lite bridge (loom_ctrl, deliberate: aperture
        // stores stay posted toward the host). The engine drains nothing
        // while it streams a bulk, so this phase - issued back to back right
        // after the sweep's last transfer - is exactly where the depth can
        // be the whole budget. A drop is INVISIBLE in the data here, because
        // every store below targets the same offset; only dbg[6] shows it.
        const uint64_t ovf0 = loom::csr_read(t_ctrl, loom::DBG_BASE + 8 * 6);
        loom::StageStats a = loom::read_stage_stats(t_ctrl);
        auto t0 = std::chrono::steady_clock::now();
        for (int k = 0; k < BENCH_STORES; k++)
            A.store(win, 0x100, 0x5709'0000'0000'0000ULL | uint64_t(k));
        // Bounded: after a size has wrecked the QP these may never retire,
        // and an unbounded spin here hangs the whole run. A store that was
        // dropped never retires either, so the wait counts the dropped ones
        // as accounted for - otherwise the 5 s guard lands inside the
        // measurement and every rate below it is meaningless.
        auto guard = std::chrono::steady_clock::now();
        while ((loom::csr_read(t_ctrl, loom::DBG_BASE + 8 * 3) - w0) +
               (loom::csr_read(t_ctrl, loom::DBG_BASE + 8 * 6) - ovf0)
                   < uint64_t(BENCH_STORES)) {
            if (std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - guard).count() > 5.0) {
                printf("  store rate: gave up waiting for the engine\n");
                break;
            }
        }
        auto t1 = std::chrono::steady_clock::now();
        loom::StageStats b = loom::read_stage_stats(t_ctrl);
        double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        uint64_t cyc = b.acc[loom::STG_STORE_RDMA] - a.acc[loom::STG_STORE_RDMA];
        uint64_t ops = b.cnt[loom::STG_STORE_RDMA] - a.cnt[loom::STG_STORE_RDMA];
        const uint64_t ovf = loom::csr_read(t_ctrl, loom::DBG_BASE + 8 * 6) - ovf0;
        printf("8 B store x%d: t-encap %lu cyc/op, %.2f us/op, %lu ops\n",
               BENCH_STORES, (unsigned long) (ops ? cyc / ops : 0),
               us / BENCH_STORES, (unsigned long) ops);
        printf("  order FIFO: %lu of %d dropped on a full FIFO (depth 64)%s\n",
               (unsigned long) ovf, BENCH_STORES,
               ovf ? "  <-- the store phase outran the engine" : "");
    }
    usleep(500000);      // let RC retransmit timers fire before sampling
    {
        const long rt = net_stat("Retrans cnt") - rt_all0;
        const long pd = net_stat("PSN drop cnt") - pd_all0;
        printf("whole run: %ld retransmissions, %ld PSN drops%s\n", rt, pd,
               (rt || pd) ? "  <-- the per-size columns above sample too "
                            "early to show these" : "");
    }
    {
        // Where the transmit stream's cycles went. Only the rdma route is
        // counted. A gap here is one Loom put into the outgoing packet
        // stream; a stall is the shell declining to take a beat.
        const uint64_t mv = loom::csr_read(t_ctrl, loom::TX_MOVE);
        const uint64_t sv = loom::csr_read(t_ctrl, loom::TX_STARVE);
        const uint64_t st = loom::csr_read(t_ctrl, loom::TX_STALL);
        const uint64_t tot = mv + sv + st;
        printf("transmit path cycles: %lu moving, %lu starved (host pull dry), "
               "%lu stalled (network pushing back)\n",
               (unsigned long) mv, (unsigned long) sv, (unsigned long) st);
        if (tot)
            printf("  %.1f%% moving, %.1f%% starved, %.1f%% stalled -> %s\n",
                   100.0 * double(mv) / double(tot),
                   100.0 * double(sv) / double(tot),
                   100.0 * double(st) / double(tot),
                   sv > st ? "the pull is gapping the outgoing stream"
                           : "the fabric is pushing back, which is expected");
    }

    dump_counters(t_ctrl, "client after bench");
    fflush(stdout);
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

    // Whose address space incoming writes land in. loom_rx no longer reads a
    // pid off each request - it drains rq_wr without inspecting it, the way
    // jigsaw's controller does - so the QP owner's ctid is programmed once
    // here. It is fixed for the life of the connection.
    loom::set_rx_pid(t_ctrl, t_data.getCtid());

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
        check(poll64(&dst2[0xC00 / 8], 0xB0BE'0000'0000'0001ULL),
              "store after a descriptor lands (staging survives dma())");
        dump_counters(t_ctrl, "after probe store");
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
        // Did the write land at the WRONG address rather than nowhere? If
        // the value turns up at some other offset, the transformation is
        // readable; if it turns up in neither buffer it went outside them,
        // which is consistent with the page-0 fault
        auto scan = [](const char *nm, const uint64_t *b, uint64_t want) {
            for (uint64_t i = 0; i < BUF_SIZE / 8; i++)
                if (b[i] == want) {
                    printf("  found %016lx in %s at byte offset 0x%lx\n",
                           (unsigned long) want, nm, (unsigned long) (i * 8));
                    return;
                }
            printf("  %016lx NOT PRESENT anywhere in %s\n",
                   (unsigned long) want, nm);
        };
        scan("dst1", dst1, 0xF1A6ULL);
        scan("dst2", dst2, 0xF1A6ULL);
        if (!skip_bulk()) {
            scan("dst1", dst1, 0xB0BE'0000'0000'0001ULL);
            scan("dst2", dst2, 0xB0BE'0000'0000'0001ULL);
        }
        printf("  dst1 = %p, dst2 = %p, staging = %p\n", (void *) dst1,
               (void *) dst2, staging);
        printf("  dst2[0xC00] = %016lx (probe)\n",
               (unsigned long) dst2[0xC00 / 8]);
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
    // The importer's benchmark can spend seconds per size before it gives
    // up on one, so DONE has to outlast it rather than expire underneath
    const int done_secs = bench_mode() ? poll_secs() * 10 : poll_secs();

    // Time the receive path against its own clock while the importer runs.
    // Above roughly 9 GB/s the offered rate outruns this side, the shell
    // drops what it cannot deliver, and one drop turns every packet after it
    // into a PSN mismatch - so what matters is how many cycles loom_rx spends
    // per packet, measured HERE rather than inferred from the sender.
    //
    // Sampling on a wall-clock tick does NOT measure this. A 32x64 KB phase
    // is ~200 us end to end while any sane poll interval is milliseconds, so
    // every window is almost entirely idle and the answer comes back as
    // "interval / packets" - 31464 cycles per packet for a 64-beat packet,
    // which is just 10 ms divided by 80. Bracket the ACTIVE phase instead:
    // spin on the counters, stamp the cycle counter at the first packet and
    // at the last, and divide by the packets actually forwarded between them.
    // A CSR read is a PCIe round trip (~1 us), which bounds the edges but not
    // the interior - over a 200 us phase that is about 1%.
    //
    // The floor is structural: a PMTU packet is 4096 B = 64 beats, so 64
    // cycles is the best any receiver can do. Near it, loom_rx is at its
    // limit and the ceiling belongs to the host write path; well above it,
    // the per-transaction cost - ST_IDLE, ST_WR_REQ, the arbiter handover,
    // paid per packet - is the ceiling and pipelining loom_rx is the fix.
    //
    // Run with LOOM_BENCH_NO_STORES=1 to read this: the store phase forwards
    // 64 B packets that cost one beat each, and mixing them in makes the
    // cycles-per-packet figure meaningless.
    // OPT-IN (LOOM_RX_PROBE=1), because it PERTURBS what it measures. Each
    // sample is a CSR read, i.e. a PCIe transaction into the same card that
    // is landing the payload, and spinning on it costs the receiver enough
    // that the SENDER slows down with it: the identical 4-iteration run
    // measured 8.797 GB/s unprobed and 2.311 GB/s probed. Numbers taken
    // while this is on describe a loaded receiver, not the real ceiling.
    //
    // The unperturbed way to get the same answer is the cliff itself - the
    // last intact rate divided into the packet size gives the per-packet
    // budget with no instrument in the path at all. A direct measurement
    // wants an RTL cycle accumulator gated on loom_rx being busy, read once
    // at the end; that costs a build, and this exists to avoid guessing in
    // the meantime.
    uint64_t c_first = 0, c_last = 0, f_first = 0, f_last = 0;
    bool started = false;
    int idle_polls = 0;
    const bool probe = getenv("LOOM_RX_PROBE") != nullptr;
    const uint64_t f0 = probe ? loom::csr_read(t_ctrl, loom::DBG_BASE + 8 * 4) : 0;
    for (int i = 0; probe && i < done_secs * 1000000 && peer.doneCount() < 1; i++) {
        const uint64_t f = loom::csr_read(t_ctrl, loom::DBG_BASE + 8 * 4);
        if (!started) {
            if (f != f0) {          // first packet of the phase
                c_first = loom::csr_read(t_ctrl, loom::STG_CYC);
                f_first = f;
                started = true;
            }
        } else if (f != f_last) {   // still moving
            c_last = loom::csr_read(t_ctrl, loom::STG_CYC);
            f_last = f;
            idle_polls = 0;
        } else if (++idle_polls > 200000) {
            break;                  // phase over, stop burning PCIe reads
        }
        if (!started) f_last = f;
    }
    if (!probe)
        for (int i = 0; i < done_secs * 100 && peer.doneCount() < 1; i++)
            usleep(10000);
    check(peer.doneCount() >= 1, "client DONE received");
    if (bench_mode() && f_last > f_first && c_last > c_first)
        printf("receive path: %lu cycles per packet over the active phase "
               "(%lu packets, %lu cycles; 64 = the beat floor for a 4096 B "
               "packet)\n",
               (unsigned long) ((c_last - c_first) / (f_last - f_first)),
               (unsigned long) (f_last - f_first),
               (unsigned long) (c_last - c_first));

    if (bench_mode()) {
        printf("\n== exporter check of the benchmark regions\n");
        for (int i = 0; i < BENCH_N; i++) {
            const uint64_t len = BENCH_SIZES[i], off = bench_offset(i);

            // The importer stops at the first size that loses packets, so
            // the sizes after it were never sent. An untouched region is
            // not a failure, and reporting it as one buries the size that
            // actually broke.
            bool touched = false;
            for (uint64_t w = 0; !touched && w < len / 8; w++)
                if (dst1[off / 8 + w] != 0) touched = true;
            if (!touched) {
                printf("  %8lu B at 0x%-8lx never written (importer stopped "
                       "before this size)\n",
                       (unsigned long) len, (unsigned long) off);
                continue;
            }

            // Characterize the damage instead of stopping at the first bad
            // word. bench_word is (0xBE0+idx)<<48 | word_index, so any word
            // that landed says exactly WHICH source word it is - the
            // displacement is read off, not guessed. That separates the
            // three things the counters cannot: a region most of which never
            // arrived (zeros), one that arrived whole but shifted (a
            // consistent nonzero displacement), and one clobbered by
            // something that is not this size's payload at all.
            uint64_t bad = 0, zeros = 0, foreign = 0;
            uint64_t first_bad = ~0ULL, last_bad = 0;
            int64_t shift = 0;
            bool shift_seen = false, shift_same = true;
            for (uint64_t w = 0; w < len / 8; w++) {
                const uint64_t got = dst1[off / 8 + w];
                if (got == bench_word(i, w)) continue;
                bad++;
                if (first_bad == ~0ULL) first_bad = w;
                last_bad = w;
                if (got == 0) { zeros++; continue; }
                if ((got >> 48) == uint64_t(0xBE0 + i)) {
                    const int64_t d = int64_t(got & 0xFFFF'FFFF'FFFFULL) - int64_t(w);
                    if (!shift_seen) { shift = d; shift_seen = true; }
                    else if (d != shift) shift_same = false;
                } else {
                    foreign++;
                }
            }
            if (bad) {
                printf("  %lu B at 0x%lx: %lu of %lu words wrong "
                       "(%lu never written, %lu not this payload), "
                       "first %lu last %lu\n",
                       (unsigned long) len, (unsigned long) off,
                       (unsigned long) bad, (unsigned long) (len / 8),
                       (unsigned long) zeros, (unsigned long) foreign,
                       (unsigned long) first_bad, (unsigned long) last_bad);
                if (shift_seen)
                    printf("    displaced payload is offset by %+ld words "
                           "(%+ld x 64 B beats)%s\n",
                           (long) shift, (long) (shift / 8),
                           shift_same ? ", the same everywhere"
                                      : ", NOT consistent across the region");
                if (zeros == bad)
                    printf("    every wrong word is zero: this payload never "
                           "arrived, it was not misplaced\n");
            }

            bool ok = true;
            for (uint64_t w = 0; ok && w < len / 8; w++)
                if (dst1[off / 8 + w] != bench_word(i, w)) {
                    printf("  %lu B at 0x%lx: word %lu is %016lx, want %016lx\n",
                           (unsigned long) len, (unsigned long) off,
                           (unsigned long) w,
                           (unsigned long) dst1[off / 8 + w],
                           (unsigned long) bench_word(i, w));
                    // A whole 64 B line here says what overwrote it. An
                    // inline wire message is {op|len, target VA, data} in
                    // the first three lanes and zero after: seeing that
                    // means loom_rx forwarded a MESSAGE down the direct
                    // path instead of parsing it, so the far side compared
                    // its RETH against the staging address and missed.
                    const uint64_t base = (off / 8 + w) & ~7ULL;
                    printf("    line at 0x%lx:",
                           (unsigned long) (base * 8));
                    for (int l = 0; l < 8; l++)
                        printf(" %016lx", (unsigned long) dst1[base + l]);
                    printf("\n");
                    if ((dst1[base] & 0xFF) == 2 && (dst1[base] >> 8) == 8)
                        printf("    ^ that is a Loom inline message header "
                               "(op 2, len 8): a message was written as "
                               "bulk payload\n");
                    ok = false;
                }
            check(ok, ok ? "bench region landed intact" : "bench region CORRUPT");
        }
    }

    if (bench_mode()) {
        // Where the receive path's cycles went. loom_rx holds no buffer, so
        // a beat moves only when the RoCE ingress and the host write path
        // are ready in the same cycle; these three partition the forwarding
        // cycles and say which side owns the ceiling. Unlike a host-side
        // poll this costs nothing while it is being measured.
        const uint64_t mv = loom::csr_read(t_ctrl, loom::RX_MOVE);
        const uint64_t sv = loom::csr_read(t_ctrl, loom::RX_STARVE);
        const uint64_t st = loom::csr_read(t_ctrl, loom::RX_STALL);
        const uint64_t tot = mv + sv + st;
        printf("receive path cycles: %lu moving, %lu starved (ingress had "
               "nothing), %lu stalled (host write not ready)\n",
               (unsigned long) mv, (unsigned long) sv, (unsigned long) st);
        if (tot)
            printf("  %.1f%% moving, %.1f%% starved, %.1f%% stalled;"
                   " 64 beats of data per 4096 B packet is the floor\n",
                   100.0 * double(mv) / double(tot),
                   100.0 * double(sv) / double(tot),
                   100.0 * double(st) / double(tot));
        const uint64_t acc = loom::csr_read(t_ctrl, loom::RX_REQ);
        const uint64_t don = loom::csr_read(t_ctrl, loom::DBG_BASE + 8 * 4);
        const uint64_t spn = loom::csr_read(t_ctrl, loom::RX_SPAN);
        // accepted counts PACKETS (every request is drained), completed
        // counts MESSAGES. The ratio is packets per message; there is no
        // identity between them to check.
        printf("rq_wr: %lu accepted (packets), %lu completed (messages), "
               "%lu arrived mid-stream\n",
               (unsigned long) acc, (unsigned long) don, (unsigned long) spn);
        const uint64_t hd = loom::csr_read(t_ctrl, loom::RX_STALL_HEAD);
        const uint64_t bd = loom::csr_read(t_ctrl, loom::RX_STALL_BODY);
        if (st)
            printf("  of the stalls: %lu before the packet's first beat, "
                   "%lu after -> %s\n", (unsigned long) hd,
                   (unsigned long) bd,
                   hd > bd ? "single-outstanding request latency; overlap the "
                             "next sq_wr with the current stream"
                           : "a bursty host write path; buffer it");
        // Sum vs worst run: the ingress FIFO is 512 beats in the shell plus
        // 512 in user logic, so a worst run well under that is a burst the
        // buffer can swallow. A worst run comparable to the total means the
        // write path is simply slower than the link and no depth fixes it.
        const uint64_t pd = loom::csr_read(t_ctrl, loom::PULL_DESYNC);
        if (pd)
            printf("  *** pull desync: %lu beat(s) left on the pull stream "
                   "when a read was issued - payload displaced ***\n",
                   (unsigned long) pd);
        const uint64_t mx = loom::csr_read(t_ctrl, loom::RX_STALL_MAX);
        if (st)
            printf("  longest unbroken stall: %lu cycles of %lu total -> %s\n",
                   (unsigned long) mx, (unsigned long) st,
                   mx < 512 ? "burst, inside one FIFO's depth"
                            : (mx < 1024 ? "burst, needs the 1024 both FIFOs give"
                                         : "sustained: deeper buffering will not "
                                           "fix this"));
    }

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

    // Remote reads do not exist yet, and the contract is that a load through
    // an rdma-route window ANSWERS with poison rather than hanging the
    // issuing CPU: loom_engine's validity check has (l_is_read ? !l_route)
    // so the read takes the same path an invalid window does. Serving rq_rd
    // like 09_perf_rdma, and the T6 round trip that follows, are 6.2b. Pin
    // it here so a half-finished 6.2b cannot quietly turn loads into
    // something that neither poisons nor returns.
    check(A.load(w1, 0x40) == loom::READ_POISON,
          "load through an rdma window answers with poison (remote reads are 6.2b)");

    auto *src = static_cast<uint64_t *>(A.alloc(BUF_SIZE));
    auto *fence = static_cast<uint64_t *>(A.allocSmall(4096));
    if (!src || !fence) { printf("FAIL: alloc\n"); return failures + 1; }
    memset(fence, 0, 4096);
    for (uint64_t i = 0; i < BUF_SIZE / 8; i++) src[i] = src_word(i);

    // Small stores through both windows (64 B inline wire messages)
    A.store(w1, 0x40, 0xB00B'0000'0000'0001ULL);
    A.store(w2, 0x40, 0xB00B'0000'0000'0002ULL);

    // The fence word carries the engine's RUNNING completion count, and
    // compl_cnt clears only on aresetn - so a second run against the same
    // card starts wherever the previous one left off. Read the baseline
    // instead of hardcoding 1, exactly as roles_test.hpp does; poll64 is an
    // equality wait, so a hardcoded value burns the whole timeout and then
    // reports a failure that is really just a warm card.
    const uint64_t f0 = loom::csr_read(t_ctrl, loom::DBG_BASE + 8 * 7);

    // Bulk with fence (direct RDMA WRITE; fence = local posted completion)
    if (!skip_bulk()) {
        A.copy(w1, 0x10000, src, DMA_BYTES, fence);
        check(poll64(&fence[0], f0 + 1),
              "fence 1 after copy (posted completion)");
        // Probe: the same window and the same kind of store as the ordering
        // flag, but right after a descriptor rather than after two. If this
        // one lands and the flag does not, position in the sequence is not
        // what matters; if both fail, any store following a descriptor does
        A.store(w2, 0xC00, 0xB0BE'0000'0000'0001ULL);
    } else {
        printf("LOOM_SKIP_BULK: no copies, stores only\n");
    }

    // Ordering across hosts: copy, then flag through the other window;
    // both ride the same QP, RC keeps them in order at the far side
    dump_counters(t_ctrl, "client before ordering copy");
    if (!skip_bulk()) {
        A.copy(w1, 0x20000, src, DMA_BYTES, fence);
        A.store(w2, 0x800, 0xF1A6ULL);
        check(poll64(&fence[0], f0 + 2), "fence 2 after ordering copy");
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

    if (bench_mode()) run_bench(t_ctrl, A, w1, src, fence);

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
