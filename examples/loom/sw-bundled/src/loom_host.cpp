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
#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
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
constexpr uint64_t BUF_SIZE      = 192ULL * 1024 * 1024;
constexpr uint64_t DMA_BYTES     = 4096;        // rdma bulk: len % 64 == 0
constexpr uint32_t STAGING_BYTES = 4096;
// Bring-up: a poll that is going to fail should fail fast enough that the
// counter dump after it is worth reading. LOOM_POLL_SECS overrides.
// Which /dev/coyote_fpga_<device>_v0 to open. The driver's device index
// increments on every load within a boot, so a host whose driver has been
// reloaded a few times is not device 0 any more - amy came up as
// coyote_fpga_6_v0 while clara, freshly rebooted, was 0. Passing 0
// regardless fails with "cThread instance could not be obtained, vfid: 0"
// even though the card and driver are perfectly healthy.
uint32_t coyote_device() {
    if (const char *e = getenv("LOOM_DEVICE")) return uint32_t(atoi(e));
    // Discover it: whichever node exists is the one to use.
    for (uint32_t d = 0; d < 64; d++) {
        char path[64];
        snprintf(path, sizeof(path), "/dev/coyote_fpga_%u_v0", d);
        if (access(path, F_OK) == 0) return d;
    }
    return 0;
}

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
    64, 256, 1024, 4096, 16384, 65536, 262144, 1048576,
    2097152, 4194304, 6291456, 8388608,
    16777216, 33554432, 67108864,
    // Bisecting the ceiling: 32 MB passes, 64 MB does not, and 64 MB fails
    // the same way at the start of the buffer as at 69.58 MB into it, so it
    // is the LENGTH not the address. These three are APPENDED, never
    // inserted, so indices 0..14 and every packed offset above stay exactly
    // what the banked measurements were taken with. A --size 0 sweep stops
    // at 64 MB and never reaches them; they are for --size with --offset.
    41943040, 50331648, 58720256,         // 40, 48, 56 MB
    // Cold, 2 MB passes and 4 MB does not. src is MAP_HUGETLB and 2 MB
    // aligned, so 2 MB reads exactly one huge page and 4 MB reads two.
    // 3 MB reads two as well: if it fails, the boundary is the huge page,
    // not a byte count somewhere between 2 and 4 MB. Run it with --offset.
    3145728,                              // 3 MB
    // The 1-2 MB bisect (2026-09-08..11), every point pinned to --offset
    // 0x400000 so only the size varied, unchunked, gap 20:
    //   1.00 / 1.25 / 1.50 / 1.75 MB   INTACT, 0 retransmissions
    //   1.8125 MB   12 retrans  4 lost   1.875 MB   99 retrans  20 lost
    //   1.9375 MB   43 retrans 11 lost   2.0 MB     64 retrans  15 lost
    // So a SINGLE unchunked message is clean up to 1.75 MiB and corrupt from
    // 1.8125 MiB on this bitstream. Read it as a practical limit, not a
    // mechanism: the retransmission counts are not monotonic in size and
    // the first bad word wanders 194882-232258, and the same 4 MB delivered
    // as 4 x 1 MB separate descriptors (credit 1, fully serialised) is
    // CORRUPT while a standalone 1 MB is clean. What tracks the loss
    // monotonically is the sustained RATE: 4 MB software-chunked is clean
    // at 65472/73728/81920 B (9.835/10.180/10.347 GB/s, 0 retrans) and
    // corrupt from 90112 B up (10.552 GB/s, 144 retrans, rising with rate).
    // Destination address (three offsets) and source address (src-skew 64K
    // and 1M, identical results) are both irrelevant.
    // APPENDED, never inserted - indices 0..18 and every packed offset above
    // keep the values the banked measurements were taken with. Like 3 MB
    // these sit past BUF_SIZE in the packed layout and are SKIPPED unless
    // run with --offset.
    1310720, 1572864, 1835008,            // 1.25, 1.5, 1.75 MB
    1900544, 1966080, 2031616             // 1.8125, 1.875, 1.9375 MB
};
// 8 MB exists so a run can send the whole thing as ONE descriptor rather
// than N iterations of a smaller one. Iterating was only ever a way to reach
// steady state for a RATE, and it is actively misleading for CORRECTNESS:
// every iteration writes the SAME bytes to the SAME destination offset (the
// offset comes from the size index, not the loop counter), so a later clean
// write silently repairs an earlier corrupted one and the exporter - which
// checks the region once, at the end - cannot tell. More iterations mask
// faults rather than expose them. One large transfer writes every
// destination byte exactly once, so the exporter's check covers the entire
// transfer, and there are no inter-message gaps in the measurement at all.
// Offsets: 0x40000 + sum(previous) = 5853184, + 8 MB = 14241792 < BUF_SIZE.
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
    // LOOM_BENCH_ITERS=0 runs the warm-up descriptor and NOTHING after it,
    // so the destination region is written by exactly ONE message. Every
    // run to date wrote it twice - warm-up plus the timed copy - so a
    // corrupt region was a mix of two messages and the forensics were
    // ambiguous. There is no rate from such a run, only a clean picture of
    // what one message did.
    const char *e = getenv("LOOM_BENCH_ITERS");
    if (e) { int v = atoi(e); return v >= 0 ? v : 1; }
    uint64_t n = BENCH_MAX_BURST / (len ? len : 1);
    if (n > BENCH_ITERS) n = BENCH_ITERS;
    return n ? int(n) : 1;
}

int bench_gap_us() {
    const char *e = getenv("LOOM_BENCH_GAP_US");
    return e ? atoi(e) : 0;
}
// Descriptors the bench leaves unretired at once. loom_ctrl's order FIFO
// is 64 deep and DROPS a push when full (deliberate: aperture stores stay
// posted toward the host), so a long --iters burst would lose descriptors
// silently. Held under the depth; not a knob, not a mechanism - what is in
// flight on the wire is bounded by the engine's window (LOOM_TX_WINDOW).
constexpr int BENCH_UNRETIRED = 48;
constexpr int BENCH_STORES = 256;    // inline messages for the store rate

// Where each size's last iteration lands, packed nose to tail
uint64_t bench_offset(int idx) {
    // LOOM_BENCH_OFF=<bytes> puts the transfer at one fixed offset instead
    // of the packed layout. The packed layout confounds size with address:
    // every size that passes lives in the first 69.58 MB of the buffer and
    // 64 MB is the only one that reaches past it, so "64 MB fails" and "the
    // region past 69.58 MB fails" cannot be told apart from a sweep. Use it
    // with LOOM_BENCH_ONLY so exactly one size goes to exactly one address.
    // Both hosts must be given the same value - they compile the same table
    // and the exporter derives the destination from it.
    if (const char *e = getenv("LOOM_BENCH_OFF"))
        return strtoull(e, nullptr, 0) & ~63ULL;
    uint64_t off = 0x40000;          // clear of the correctness checks
    for (int i = 0; i < idx; i++) off += BENCH_SIZES[i];
    return (off + 63) & ~63ULL;
}
uint64_t bench_word(int idx, uint64_t i) {
    return (uint64_t(0xBE0 + idx) << 48) | i;
}
constexpr int BENCH_N = int(sizeof(BENCH_SIZES) / sizeof(BENCH_SIZES[0]));

bool bench_mode() { return getenv("LOOM_BENCH") != nullptr; }
// LOOM_BENCH_PINGPONG=1: the delivery-based benchmark (perf_rdma's
// definition) instead of the push benchmark. See run_pingpong / serve_pong.
bool pingpong_mode() { return getenv("LOOM_BENCH_PINGPONG") != nullptr; }
// LOOM_XPUS=2: a second XPU (its own data cThread and buffer) per host.
// LOOM_BENCH_MATRIX=1: the 2 local + 2 remote exchange test (needs XPUS=2).
int  n_xpus() { const char *e = getenv("LOOM_XPUS"); return e ? atoi(e) : 1; }
bool matrix_mode() { return getenv("LOOM_BENCH_MATRIX") != nullptr; }

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

// The engine's transmit-side words, for a wedge post-mortem in the matrix
// modes: is the engine held by its window, by sq_wr, or by its own pull?
// The receive path's cycle accounting (loom_rx), for the client too: over a
// run, was its ingress mostly STARVED (the stack delivered nothing) or
// STALLED (the host write path would not take a beat)?
void dump_rx(coyote::cThread &t, const char *tag) {
    const uint64_t mv = loom::csr_read(t, loom::RX_MOVE), sv = loom::csr_read(t, loom::RX_STARVE),
                   st = loom::csr_read(t, loom::RX_STALL), bp = loom::csr_read(t, loom::RX_BP),
                   ff = loom::csr_read(t, loom::RX_FIFO_FULL), rq = loom::csr_read(t, loom::RX_REQ),
                   fw = loom::csr_read(t, loom::DBG_BASE + 8 * 4);
    const double tot = double(mv + sv + st);
    printf("rx [%s]: moving %lu, starved %lu, stalled %lu (%.1f%% / %.1f%% / %.1f%%), longest stall %lu, "
           "backpressure %lu, ingress FIFO full %lu, rq_wr %lu, forwarded %lu\n", tag,
           (unsigned long) mv, (unsigned long) sv, (unsigned long) st,
           tot ? 100.0 * mv / tot : 0, tot ? 100.0 * sv / tot : 0, tot ? 100.0 * st / tot : 0,
           (unsigned long) loom::csr_read(t, loom::RX_STALL_MAX), (unsigned long) bp, (unsigned long) ff,
           (unsigned long) rq, (unsigned long) fw);
    fflush(stdout);
}

// The shared request port: where its time went. See loom.hpp.
void dump_port(coyote::cThread &t, const char *tag) {
    const uint64_t wl = loom::csr_read(t, loom::WR_WAIT_LOCAL),
                   wr = loom::csr_read(t, loom::WR_WAIT_RDMA),
                   be = loom::csr_read(t, loom::WR_BLK_ENG),
                   br = loom::csr_read(t, loom::WR_BLK_RX);
    printf("port [%s]: sq_wr waits - local %lu cyc, rdma %lu cyc; engine blocked by rx %lu cyc; "
           "rx blocked by engine %lu cyc (must be 0); rx_chunk %lu\n", tag,
           (unsigned long) wl, (unsigned long) wr, (unsigned long) be, (unsigned long) br,
           (unsigned long) loom::csr_read(t, loom::RX_CHUNK));
    fflush(stdout);
}

void dump_tx(coyote::cThread &t, const char *tag) {
    printf("tx [%s]: window %lu, unacked now %lu, acks %lu, window-full %lu cyc, sq_wr-wait %lu cyc, "
           "tx FIFO held the pull %lu cyc, pull desync %lu, stage: move %lu starve %lu stall %lu\n", tag,
           (unsigned long) (loom::csr_read(t, loom::TX_CTL) & 0xFF),
           (unsigned long) (loom::csr_read(t, loom::TX_STATE) & 0xFFFF),
           (unsigned long) loom::csr_read(t, loom::TX_ACKS),
           (unsigned long) loom::csr_read(t, loom::TX_WINFULL),
           (unsigned long) loom::csr_read(t, loom::TX_REQWAIT),
           (unsigned long) loom::csr_read(t, loom::TX_FIFO_FULL),
           (unsigned long) loom::csr_read(t, loom::PULL_DESYNC),
           (unsigned long) loom::csr_read(t, loom::TX_MOVE),
           (unsigned long) loom::csr_read(t, loom::TX_STARVE),
           (unsigned long) loom::csr_read(t, loom::TX_STALL));
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
// LOOM_TX_PACE is "NUM/DEN" (payload beats may move NUM out of every DEN
// cycles on the rdma route, e.g. 41/64 ~= 10.25 GB/s at 250 MHz x 64 B) or
// a bare N, kept for the old 1-in-N meaning: N/(N+1). Returns the CSR word
// {den[15:8], num[7:0]}, 0 = off.
static uint64_t parse_pace(const char *e) {
    unsigned num = 0, den = 0;
    if (sscanf(e, "%u/%u", &num, &den) == 2) { /* explicit fraction */ }
    else { num = unsigned(strtoul(e, nullptr, 0)); den = num ? num + 1 : 0; }
    if (num == 0 || den == 0 || num >= den || num > 255 || den > 255) return 0;
    return (uint64_t(den) << 8) | num;
}
// LOOM_TX_WINDOW=N packets posted and not yet acked (0 = no window). Unset
// = bitstream default (16). Returns the TX_CTL word.
static uint64_t tx_ctl_from_env(uint64_t cur) {
    uint64_t v = cur;
    if (const char *e = getenv("LOOM_TX_WINDOW"))
        v = (v & ~0xFFull) | (strtoul(e, nullptr, 0) & 0xFF);
    return v;
}
// loom_rx's host-write granularity (CSR 76), on BOTH hosts: each lands its
// own incoming packets, so the knob has to be armed on each side.
static void arm_rx_chunk(coyote::cThread &t, const char *when) {
    if (const char *e = getenv("LOOM_RX_CHUNK")) {
        const uint64_t k = strtoull(e, nullptr, 0);
        loom::csr_write(t, loom::RX_CHUNK, k);
        const uint64_t got = loom::csr_read(t, loom::RX_CHUNK);
        printf("engine config: rx_chunk %s: %lu packets per host write%s\n", when,
               (unsigned long) got, got == k ? "" : " DID NOT TAKE");
        fflush(stdout);
    }
}

static void print_tx_ctl(uint64_t v, const char *when) {
    printf("engine config: tx window %s: %lu packets unacked%s\n",
           when, (unsigned long) (v & 0xFF), (v & 0xFF) ? "" : " (no window)");
    fflush(stdout);
}
static void print_pace(uint64_t v, const char *when) {
    const unsigned num = v & 0xFF, den = (v >> 8) & 0xFF;
    if (num && den)
        printf("engine config: tx_pace %s: %u/%u (%.1f%% of burst, ~%.2f GB/s cap)\n",
               when, num, den, 100.0 * num / den, 16.0 * num / den);
    else
        printf("engine config: tx_pace %s: off\n", when);
    fflush(stdout);
}

// -------------------------------------------------------------------------
// PING-PONG: the delivery-based benchmark, perf_rdma's definition.
//
// The push benchmark above stops its clock at the sender's fence - the
// last beat handed to the RoCE stack - which is exact to 0.01% for 64 MiB
// and meaningless for 4 KB. This one stops it when the bytes have LANDED
// on the far host and come back:
//
//   clara: copy(len) -> store(LEN) -> store(FLAG = k)   [order FIFO keeps
//          the flag behind the data; RC delivers in order; loom_rx writes
//          in order - the flag cannot become visible before the payload]
//   amy:   spin on FLAG == k in its OWN memory (the GPU model: no interrupt,
//          no device register, a peer-written word), then copy the same
//          bytes back into clara's pong buffer, LEN, FLAG = k
//   clara: spin on its pong FLAG == k
//
// One round trip = two deliveries; the reported one-way time is RTT/2 and
// the rate is len / (RTT/2). Both sides use the same rdma windows, the
// same engine, the same window/pacing knobs. The reverse path is set up
// through the peering socket (PeerOp::PONG): clara hands amy its staging
// VA and a buffer to write into, amy programs an rdma window onto it.
// -------------------------------------------------------------------------
// The flag and length words are aperture STORES, which address one 4 KB
// page per window ((win << 12) | off), so they live inside it, clear of
// the offsets the functional phase uses. The payload is a descriptor
// offset (28 bits) and sits at 64 MiB.
constexpr uint64_t PP_FLAG_OFF = 0xF00;              // flag word, both sides
constexpr uint64_t PP_LEN_OFF  = 0xF08;              // length word, both sides
constexpr uint64_t PP_DATA_OFF = 64ULL * 1024 * 1024; // payload, both sides
constexpr uint64_t PP_MAGIC    = 0x5049'4E47'0000'0000ULL;   // "PING" | k

// Server side: answer every ping until the client's DONE arrives.
template <class DoneFn>
void serve_pong(loom::Xpu &S, int win, uint64_t *buf,
                volatile uint64_t *fence, DoneFn done) {
    volatile uint64_t *flag = buf + PP_FLAG_OFF / 8;
    volatile uint64_t *lenw = buf + PP_LEN_OFF / 8;
    uint64_t k = 0, served = 0, bytes = 0;
    printf("server: ping-pong service on window %d\n", win);
    fflush(stdout);
    while (!done()) {
        // poll the flag, checking for DONE every ~1 ms of spinning
        bool got = false;
        for (int i = 0; i < 20000 && !got; i++)
            if (*flag == (PP_MAGIC | (k + 1))) got = true;
        if (!got) continue;
        k++;
        const uint64_t len = *lenw;
        const uint64_t c = *fence;
        S.copy(win, uint32_t(PP_DATA_OFF), buf + PP_DATA_OFF / 8, len, fence);
        if (!spin64(fence, c + 1, 5e6)) {
            printf("server: pong %lu never fenced - stopping\n", (unsigned long) k);
            break;
        }
        S.store(win, uint32_t(PP_LEN_OFF), len);
        S.store(win, uint32_t(PP_FLAG_OFF), PP_MAGIC | k);
        served++; bytes += len;
    }
    printf("server: ping-pong served %lu rounds, %lu bytes each way\n",
           (unsigned long) served, (unsigned long) bytes);
    fflush(stdout);
}

// Client side: for every size, a warm-up round then ITERS timed rounds.
void run_pingpong(loom::Xpu &A, int win, uint64_t *src, uint64_t *pong,
                  volatile uint64_t *fence) {
    volatile uint64_t *pflag = pong + PP_FLAG_OFF / 8;
    uint64_t k = 0;
    const long rt0 = net_stat("Retrans cnt"), pd0 = net_stat("PSN drop cnt");
    std::vector<uint64_t> only_set;             // LOOM_BENCH_ONLY, as run_bench
    if (const char *o = getenv("LOOM_BENCH_ONLY"))
        for (const char *q = o; *q; ) {
            only_set.push_back(strtoull(q, nullptr, 0));
            while (*q && *q != ',') q++;
            if (*q == ',') q++;
        }
    auto wanted = [&](uint64_t l) {
        if (only_set.empty()) return true;
        for (uint64_t v : only_set) if (v == l) return true;
        return false;
    };
    printf("\n== ping-pong benchmark (delivery-based: amy lands it, writes it "
           "back, clara lands that; one way = RTT/2)\n");
    printf("%10s %6s %12s %12s %10s %8s\n",
           "bytes", "rounds", "rtt_us", "one_way_us", "GB/s", "landed");
    fflush(stdout);
    for (size_t i = 0; i < sizeof(BENCH_SIZES) / sizeof(BENCH_SIZES[0]); i++) {
        const uint64_t len = BENCH_SIZES[i];
        if (!wanted(len)) continue;
        if (PP_DATA_OFF + len > BUF_SIZE) continue;
        for (uint64_t w = 0; w < len / 8; w++) src[w] = bench_word(i, w);
        memset(pong + PP_DATA_OFF / 8, 0, len);
        const int rounds = bench_iters(len) > 0 ? bench_iters(len) : 1;
        bool ok = true;
        double total_us = 0.0;
        // warm-up round + timed rounds; each round is one full trip
        for (int r = 0; r <= rounds && ok; r++) {
            k++;
            const uint64_t c = *fence;
            auto t0 = std::chrono::steady_clock::now();
            A.copy(win, uint32_t(PP_DATA_OFF), src, len, fence);
            A.store(win, uint32_t(PP_LEN_OFF), len);
            A.store(win, uint32_t(PP_FLAG_OFF), PP_MAGIC | k);
            if (!spin64(pflag, PP_MAGIC | k, 5e6)) {
                printf("%10lu   round %d: pong never arrived (fence %s)\n",
                       (unsigned long) len, r,
                       (*fence >= c + 1) ? "retired" : "NOT retired");
                ok = false;
                break;
            }
            const double us = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - t0).count();
            if (r > 0) total_us += us;
        }
        // The bytes that came back are the bytes that landed there
        bool same = ok;
        for (uint64_t w = 0; same && w < len / 8; w++)
            if (pong[PP_DATA_OFF / 8 + w] != bench_word(i, w)) same = false;
        if (ok) {
            const double rtt = total_us / rounds;
            printf("%10lu %6d %12.2f %12.2f %10.3f %8s\n",
                   (unsigned long) len, rounds, rtt, rtt / 2,
                   double(len) / (rtt / 2 * 1e3), same ? "yes" : "NO");
        }
        fflush(stdout);
        if (!ok || !same) { printf("ping-pong: stopping at this size\n"); break; }
    }
    usleep(500000);      // let RC retransmit timers fire before sampling
    printf("whole run: %ld retransmissions, %ld PSN drops (this side)\n",
           net_stat("Retrans cnt") - rt0, net_stat("PSN drop cnt") - pd0);
    fflush(stdout);
}

void run_bench(coyote::cThread &t_ctrl, loom::Xpu &A, int win,
               uint64_t *src, volatile uint64_t *fence) {
    printf("\n== remote transmit benchmark (<=%d iters/size, <=%lu MB burst)\n",
           BENCH_ITERS, (unsigned long) (BENCH_MAX_BURST >> 20));
    // us/op and GB/s are the TRANSFER alone - the deliberate idle of
    // LOOM_BENCH_GAP_US is excluded, so they answer "how fast does Loom move
    // a message" and are directly comparable to perf_rdma's throughput
    // column. offered_* include that idle: the sustained rate a paced sender
    // actually offers. Reporting only the latter is what made a pacing sweep
    // look like a throughput collapse - at 1 MB the wall clock is
    // size/(transfer + gap), so it tracked the gap almost exactly (gap 1000
    // us predicted 0.967 and measured 0.965 GB/s; 160 us predicted 4.30 and
    // measured 4.257) while the transfer underneath never moved.
    printf("%10s %6s %10s %10s %12s %10s %12s %12s %8s %8s %10s\n",
           "bytes", "iters", "cyc/op", "queue_cyc", "us/op", "GB/s",
           "offered_us", "offered_GB/s",
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
    // LOOM_BENCH_ONLY takes a comma-separated LIST now, so a run can be
    // exactly "one 2 MB descriptor, then one 64 MB descriptor, nothing
    // else". A single preceding 2 MB message is the only thing found that
    // moves the cold failure rate (1590 retransmissions -> 17), and the
    // question this answers is whether it is a fix or only an improvement.
    std::vector<uint64_t> only_set;
    if (const char *o = getenv("LOOM_BENCH_ONLY"))
        for (const char *q = o; *q; ) {
            only_set.push_back(strtoull(q, nullptr, 0));
            while (*q && *q != ',') q++;
            if (*q == ',') q++;
        }
    auto wanted = [&](uint64_t l) {
        if (only_set.empty()) return true;
        for (uint64_t v : only_set) if (v == l) return true;
        return false;
    };
    const uint64_t only_len = only_set.size() == 1 ? only_set[0] : 0;
    (void) only_len;

    // LOOM_BENCH_FROM=<bytes> starts the sweep at that size instead of at
    // 64 B, and runs everything above it. This is how much ramp a size
    // gets: the sweep passes at every size and a lone size of 4 MB or more
    // does not, so the question is how much of the ramp is load-bearing.
    // Unlike a synthetic warm-up it uses real bench regions, so every byte
    // the ramp writes is checked by the exporter too - a ramp that quietly
    // corrupts cannot be mistaken for one that worked.
    const char *from = getenv("LOOM_BENCH_FROM");
    const uint64_t from_len = from ? strtoull(from, nullptr, 0) : 0;

    for (int i = 0; i < BENCH_N; i++) {
        const uint64_t len = BENCH_SIZES[i];
        const uint64_t off = bench_offset(i);
        if (!wanted(len)) continue;
        if (from_len && len < from_len) continue;
        if (off + len > BUF_SIZE) continue;   // bisect sizes, packed layout

        // Distinct pattern per size so the exporter can tell them apart.
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
        const int gap_us = bench_gap_us();
        double idle_us = 0.0;   // deliberate pacing idle, excluded below
        for (int k = 0; k < iters; k++) {
            A.copy(win, uint32_t(off), src, len, fence);
            if (k + 1 > BENCH_UNRETIRED)           // stay under the order FIFO
                spin64_ge(fence, base + uint64_t(k + 1 - BENCH_UNRETIRED), 5e6);
            // Pace against the RECEIVER, which nothing else here does. The
            // fence is a local posted completion, so the
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
                // Spin, do not usleep. The kernel timer bounds usleep's
                // resolution, so usleep(5) and usleep(20) both idle for
                // hundreds of microseconds - a 5, 10 and 20 us sweep came
                // back 515.67, 509.64 and 506.77 us/op, which is one
                // experiment run three times. A message is ~85 us, so the
                // interesting range needs single-microsecond resolution.
                auto g0 = std::chrono::steady_clock::now();
                while (std::chrono::duration<double, std::micro>(
                           std::chrono::steady_clock::now() - g0).count()
                       < double(gap_us)) { }
                // The fence wait above is transfer time and stays counted;
                // only this idle comes back out.
                idle_us += std::chrono::duration<double, std::micro>(
                    std::chrono::steady_clock::now() - g0).count();
            }
        }
        // Generous but bounded: a size that cannot keep up should report,
        // not hold the run for the poll timeout
        bool ok = iters == 0 ? true : spin64(fence, base + iters, 5e6);
        auto t1 = std::chrono::steady_clock::now();
        loom::StageStats b = loom::read_stage_stats(t_ctrl);
        const long rt1 = net_stat("Retrans cnt"), pd1 = net_stat("PSN drop cnt");

        const double wall_us = std::chrono::duration<double, std::micro>(
            t1 - t0).count();
        double us = wall_us - idle_us;      // transfer alone
        if (us < 1.0) us = wall_us;         // never divide by ~0
        uint64_t cyc = b.acc[loom::STG_DMA_RDMA] - a.acc[loom::STG_DMA_RDMA];
        uint64_t ops = b.cnt[loom::STG_DMA_RDMA] - a.cnt[loom::STG_DMA_RDMA];
        uint64_t q   = b.queue_acc - a.queue_acc;
        printf("%10lu %6d %10lu %10lu %12.2f %10.3f %12.2f %12.3f "
               "%8ld %8ld %10s\n",
               (unsigned long) len, iters,
               (unsigned long) (ops ? cyc / ops : 0),
               (unsigned long) (ops ? q / ops : 0),
               us / iters,
               (double(len) * iters) / (us * 1e3),
               wall_us / iters,
               (double(len) * iters) / (wall_us * 1e3),
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
        // The engine's own residue detector. It was read only in the
        // receive-path report, which runs on the SERVER - where the engine
        // never streams a bulk descriptor, so it could only ever read zero.
        // The pull is the SENDER's, and this is the sender.
        const uint64_t pdes = loom::csr_read(t_ctrl, loom::PULL_DESYNC);
        printf("  pull desync: %lu   (beats left on the pull stream when a "
               "read was issued; nonzero means the engine forwarded a beat "
               "that was not its payload, and the message is displaced)\n",
               (unsigned long) pdes);
        {
            const uint64_t pv = loom::csr_read(t_ctrl, loom::TX_PACE);
            const uint64_t pc = loom::csr_read(t_ctrl, loom::TX_PACED);
            const unsigned num = pv & 0xFF, den = (pv >> 8) & 0xFF;
            if (num && den)
                // moving beats cost den each and every cycle earns num, so
                // the pacer must hold ~moving*(den-num)/num cycles
                printf("  tx pacing: %u/%u, pacer held %lu cycles (expect ~%lu; "
                       "cap %.2f GB/s)\n", num, den, (unsigned long) pc,
                       (unsigned long) (mv * (den - num) / num), 16.0 * num / den);
            else
                printf("  tx pacing: off\n");
        }
        {
            const uint64_t ctl  = loom::csr_read(t_ctrl, loom::TX_CTL);
            const uint64_t st   = loom::csr_read(t_ctrl, loom::TX_STATE);
            const uint64_t acks = loom::csr_read(t_ctrl, loom::TX_ACKS);
            const uint64_t wf   = loom::csr_read(t_ctrl, loom::TX_WINFULL);
            const uint64_t rw   = loom::csr_read(t_ctrl, loom::TX_REQWAIT);
            const uint64_t ff   = loom::csr_read(t_ctrl, loom::TX_FIFO_FULL);
            printf("  tx window: %lu packets; %lu unacked at the end\n",
                   (unsigned long) (ctl & 0xFF), (unsigned long) (st & 0xFFFF));
            printf("  tx acks: %lu packets acked; window full %lu cycles; sq_wr wait %lu cycles; "
                   "tx FIFO held the pull %lu cycles\n",
                   (unsigned long) acks, (unsigned long) wf, (unsigned long) rw,
                   (unsigned long) ff);
            printf("    -> %s\n",
                   wf ? "the far side's acks bounded the sender (window binding)"
                      : "the window never bound: the sender never had more unacked "
                        "than the window allows");
        }
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

// -------------------------------------------------------------------------
// MATRIX: 2 local + 2 remote XPUs exchanging data every way.
//
// Per host: XPU 1 and XPU 2, each on its own cThread with its own buffer,
// plus the dedicated QP owner. The client XPUs exchange locally (A1<->A2,
// the cross-pid write through the TLB; the server side is the same code
// and is not repeated), every A->B pair remotely (A1->B1, A2->B2, A1->B2,
// A2->B1, each answered), then A1<->B1 BIDIRECTIONALLY (both push at
// once, one issuer per engine), then A1<->B1 and A2<->B2 concurrently
// (both directions AND two issuers per engine). What makes the remote
// cases land in the right address space is the destination pid the
// sender's window carries into the message header - one QP serves all.
//
// The fence carries the issuing XPU's own completion count (per source pid
// in the engine); a fence word is waited on for >= its old value + 1.
//
// Protocol: the client drives; each server XPU runs a service thread that
// polls a COMMAND word in its own buffer (the GPU model again):
//   CMD_REMOTE {seq, from}: verify DATA against pattern(from -> me), copy
//     it back into the sender's buffer through the window onto it, store
//     RFLAG = seq there
//   CMD_BIDIR  {seq, from}: push pattern(me -> from) at the sender NOW (the
//     sender's own push follows its command by one wire latency, so the
//     two overlap for nearly the whole transfer), store RFLAG = seq; then
//     wait for the sender's BFLAG = seq, verify DATA against
//     pattern(from -> me), store ACK = seq back
// -------------------------------------------------------------------------
constexpr uint64_t MX_RFLAG  = 0xF00;   // remote reply landed (seq)      [importer side]
constexpr uint64_t MX_LFLAG  = 0xF10;   // local copy landed (seq)
constexpr uint64_t MX_CMD    = 0xF20;   // {code[63:56], from/to[55:48], seq[47:0]}
constexpr uint64_t MX_LEN    = 0xF28;   // bytes of the command's payload
constexpr uint64_t MX_ACK    = 0xF30;   // receiver verified the sender's bidir push (seq)
constexpr uint64_t MX_BFLAG  = 0xF38;   // sender's bidir push fenced (seq)   [server side]
constexpr uint64_t MX_VFLAG  = 0xF40;   // receiver finished verifying the bidir push (seq)
constexpr uint64_t MX_DATA   = 4ULL << 20;   // remote landing region
constexpr uint64_t MX_LOCAL  = 8ULL << 20;   // local landing region
constexpr uint64_t MX_SRC    = 12ULL << 20;  // where a sender keeps its pattern
constexpr uint64_t MX_BUF    = 16ULL << 20;  // per-XPU buffer for XPU 2
constexpr uint64_t MX_LEN_R  = 4ULL << 20;   // the remote transfer size
constexpr uint64_t CMD_REMOTE = 1, CMD_BIDIR = 3, CMD_STORM = 6, CMD_PUSHME = 7;

inline uint64_t mx_cmd(uint64_t code, uint64_t who, uint64_t seq) {
    return (code << 56) | (who << 48) | (seq & 0xFFFF'FFFF'FFFFULL);
}
// One pattern per (sender, receiver) pair, both sides can compute it
inline uint64_t mx_word(int from, int to, uint64_t w) {
    return (0x4D58ULL << 48) | (uint64_t(from) << 40) | (uint64_t(to) << 32) | (w & 0xFFFF'FFFFULL);
}
static bool mx_check(const uint64_t *at, int from, int to, uint64_t len, const char *what) {
    for (uint64_t w = 0; w < len / 8; w++)
        if (at[w] != mx_word(from, to, w)) {
            printf("FAIL: %s: word %lu got %016lx want %016lx\n", what,
                   (unsigned long) w, (unsigned long) at[w],
                   (unsigned long) mx_word(from, to, w));
            failures++;
            return false;
        }
    printf("PASS: %s\n", what);
    return true;
}

// Server side: one per server XPU. `me` is 1 or 2 (B1/B2); `to_a[j]` is
// the remote window onto client XPU j+1.
struct MxServerXpu {
    loom::Xpu *xpu; uint64_t *buf; volatile uint64_t *fence; int me;
    int to_a[2];
};
template <class DoneFn>
void mx_serve(MxServerXpu x, DoneFn done) {
    volatile uint64_t *cmd  = x.buf + MX_CMD / 8;
    volatile uint64_t *lenw = x.buf + MX_LEN / 8;
    // The bidir source, filled ahead of time: the push has to start the
    // moment the command lands or it does not overlap the client's
    for (uint64_t w = 0; w < MX_LEN_R / 8; w++) x.buf[MX_SRC / 8 + w] = mx_word(x.me + 2, x.me, w);
    uint64_t last_cmd = 0;
    while (!done()) {
        bool any = false;
        for (int i = 0; i < 20000 && !any; i++) {
            const uint64_t c = *cmd;
            if (c != last_cmd && c != 0) {
                last_cmd = c; any = true;
                const uint64_t code = c >> 56, who = (c >> 48) & 0xFF, seq = c & 0xFFFF'FFFF'FFFFULL;
                const uint64_t len = *lenw;
                if (code == CMD_REMOTE) {
                    char what[96];
                    snprintf(what, sizeof what, "matrix: A%lu -> B%d landed (%lu B, seq %lu)",
                             (unsigned long) who, x.me, (unsigned long) len, (unsigned long) seq);
                    mx_check(x.buf + MX_DATA / 8, int(who), x.me + 2, len, what);
                    const uint64_t f = *x.fence;
                    x.xpu->copy(x.to_a[who - 1], uint32_t(MX_DATA), x.buf + MX_DATA / 8, len, x.fence);
                    if (!spin64_ge(x.fence, f + 1, 5e6)) { printf("FAIL: B%d reply never fenced\n", x.me); failures++; }
                    x.xpu->store(x.to_a[who - 1], uint32_t(MX_LEN), len);
                    x.xpu->store(x.to_a[who - 1], uint32_t(MX_RFLAG), seq);
                } else if (code == CMD_STORM) {
                    // len = how many 64 B stores to fire back-to-back at A
                    // while A pushes 4 MiB at us: small PACKETS, no bytes.
                    // Then the usual landing handshake.
                    for (uint64_t k = 0; k < len; k++) x.xpu->store(x.to_a[who - 1], uint32_t(MX_RFLAG), seq);
                    if (!spin64(x.buf + MX_BFLAG / 8, seq, 5e6)) { printf("FAIL: matrix storm: A%lu -> B%d flag never landed\n", (unsigned long) who, x.me); failures++; continue; }
                    x.xpu->store(x.to_a[who - 1], uint32_t(MX_ACK), seq);
                    char what[96];
                    snprintf(what, sizeof what, "matrix storm: A%lu -> B%d landed (seq %lu)", (unsigned long) who, x.me, (unsigned long) seq);
                    mx_check(x.buf + MX_DATA / 8, int(who), x.me + 2, MX_LEN_R, what);
                    x.xpu->store(x.to_a[who - 1], uint32_t(MX_VFLAG), seq);
                } else if (code == CMD_PUSHME) {
                    // Push at the sender and nothing else: the sender is busy
                    // with a LOCAL copy, not with a push of its own. Used by
                    // --rxlocal to give the sender's vFPGA the same number of
                    // incoming packets as a bidirectional round while all of
                    // its OWN requests are local (STRM_HOST) - see the
                    // head-of-line-blocking question in HANDOVER-bidir.md.
                    const uint64_t f = *x.fence;
                    x.xpu->copy(x.to_a[who - 1], uint32_t(MX_DATA), x.buf + MX_SRC / 8, len, x.fence);
                    if (!spin64_ge(x.fence, f + 1, 5e6)) { printf("FAIL: matrix pushme: B%d -> A%lu never fenced\n", x.me, (unsigned long) who); failures++; }
                    x.xpu->store(x.to_a[who - 1], uint32_t(MX_LEN), len);
                    x.xpu->store(x.to_a[who - 1], uint32_t(MX_RFLAG), seq);
                } else if (code == CMD_BIDIR) {
                    if (int(who) != x.me || len > MX_LEN_R) { printf("FAIL: B%d bidir from A%lu, %lu B: unsupported\n", x.me, (unsigned long) who, (unsigned long) len); failures++; continue; }
                    const uint64_t f = *x.fence;
                    x.xpu->copy(x.to_a[who - 1], uint32_t(MX_DATA), x.buf + MX_SRC / 8, len, x.fence);
                    if (!spin64_ge(x.fence, f + 1, 5e6)) { printf("FAIL: matrix bidir: B%d -> A%lu never fenced\n", x.me, (unsigned long) who); failures++; }
                    x.xpu->store(x.to_a[who - 1], uint32_t(MX_LEN), len);
                    x.xpu->store(x.to_a[who - 1], uint32_t(MX_RFLAG), seq);
                    if (!spin64(x.buf + MX_BFLAG / 8, seq, 5e6)) { printf("FAIL: matrix bidir: A%lu -> B%d flag never landed\n", (unsigned long) who, x.me); failures++; continue; }
                    // ACK the landing first, verify after: the sender times
                    // the round up to this ACK, and checking 4 MiB word by
                    // word takes longer than moving it
                    x.xpu->store(x.to_a[who - 1], uint32_t(MX_ACK), seq);
                    char what[96];
                    snprintf(what, sizeof what, "matrix bidir: A%lu -> B%d landed (%lu B, seq %lu)",
                             (unsigned long) who, x.me, (unsigned long) len, (unsigned long) seq);
                    mx_check(x.buf + MX_DATA / 8, int(who), x.me + 2, len, what);
                    // ...and only now may the sender reuse this landing region
                    x.xpu->store(x.to_a[who - 1], uint32_t(MX_VFLAG), seq);
                }
            }
        }
    }
}

// Client side. A[0], A[1] are the two client XPUs with their buffers;
// wins: to_b[i][j] remote window from client XPU i+1 onto server XPU j+1's
// buffer, to_a[i] local window onto client XPU i+1's buffer.
struct MxClientXpu { loom::Xpu *xpu; uint64_t *buf; uint64_t *src; volatile uint64_t *fence; };
void run_matrix(MxClientXpu A[2], int to_b[2][2], int to_a[2]) {
    uint64_t seq = 0x100;
    const uint64_t LEN_R = MX_LEN_R, LEN_L = 1ULL << 20;
    // A_i -> B_i and B_i -> A_i at once. Our command starts B's push; ours
    // follows it by one wire latency (~8 us against ~370 us of transfer).
    // One issuer per engine on each side - the two-issuer case is the
    // concurrent phase.
    // Per-round timing, from the command store: our push fenced (issue,
    // fence-clocked), the far push landed here (the reply flag), our push
    // landed there (the ACK, stored before the far side verifies). The
    // round is the later landing; both directions carry a flag latency or
    // two of overhead (~8 us each against ~370 us of transfer). Verification
    // on both sides happens after the last timestamp.
    struct BidirRound { double fence_us, far_landed_us, own_landed_us; };
    std::vector<BidirRound> rounds;
    // sequential = the far push first, ours only after it has landed: the
    // same buffers and pulls touched, nothing concurrent on either host.
    // Used as a warm-up to test whether the round-1 host-DMA stall is a
    // first-touch cost.
    // LOOM_BIDIR_LIGHT=1: touch the landing region with the CPU as little as
    // possible. Normally each round memsets 4 MiB of it and verifies 4 MiB
    // word by word - 8 MiB of CPU traffic per round on the CLIENT only (the
    // server neither memsets nor is timed the same way), and every dirty
    // line makes the landing DMA write do coherency work. The client's engine
    // is the one starved 63% of the run waiting for its pull, so this asks
    // how much of that is our own harness rather than the hardware.
    // Sentinels at both ends still catch a landing that never happened, and
    // the full check still runs on the last round.
    const bool light = getenv("LOOM_BIDIR_LIGHT") != nullptr;
    auto bidir = [&](int i, uint64_t len, uint64_t sq, const char *tag, bool sequential = false,
                     bool full_check = true) {
        using clk = std::chrono::steady_clock;
        auto us = [](clk::time_point a, clk::time_point b) {
            return std::chrono::duration<double, std::micro>(b - a).count();
        };
        if (light) {
            A[i].buf[MX_DATA / 8] = 0;                       // sentinels only
            A[i].buf[MX_DATA / 8 + len / 8 - 1] = 0;
        } else {
            memset(A[i].buf + MX_DATA / 8, 0, len);
        }
        const uint64_t f = *A[i].fence;
        A[i].xpu->store(to_b[i][i], uint32_t(MX_LEN), len);
        const auto t0 = clk::now();
        A[i].xpu->store(to_b[i][i], uint32_t(MX_CMD), mx_cmd(CMD_BIDIR, i + 1, sq));
        if (sequential && !spin64(A[i].buf + MX_RFLAG / 8, sq, 5e6)) { printf("FAIL: %s: B%d -> A%d never arrived (sequential)\n", tag, i + 1, i + 1); failures++; return; }
        A[i].xpu->copy(to_b[i][i], uint32_t(MX_DATA), A[i].src, len, A[i].fence);
        if (!spin64_ge(A[i].fence, f + 1, 5e6)) { printf("FAIL: %s: A%d -> B%d never fenced\n", tag, i + 1, i + 1); failures++; return; }
        const auto t_fence = clk::now();
        A[i].xpu->store(to_b[i][i], uint32_t(MX_BFLAG), sq);
        // The two landings complete in either order; take both timestamps
        // as each is seen, then verify.
        clk::time_point t_far{}, t_own{};
        bool far_ok = false, own_ok = false;
        volatile uint64_t *rflag = A[i].buf + MX_RFLAG / 8, *ack = A[i].buf + MX_ACK / 8;
        const auto deadline = clk::now() + std::chrono::seconds(5);
        while (!(far_ok && own_ok) && clk::now() < deadline) {
            if (!far_ok && *rflag == sq) { t_far = clk::now(); far_ok = true; }
            if (!own_ok && *ack == sq)   { t_own = clk::now(); own_ok = true; }
        }
        if (!far_ok) { printf("FAIL: %s: B%d -> A%d never arrived\n", tag, i + 1, i + 1); failures++; return; }
        if (!own_ok) { printf("FAIL: %s: B%d never acked A%d's data\n", tag, i + 1, i + 1); failures++; return; }
        rounds.push_back({us(t0, t_fence), us(t0, t_far), us(t0, t_own)});
        char what[96];
        snprintf(what, sizeof what, "%s: B%d -> A%d landed (%lu B)", tag, i + 1, i + 1, (unsigned long) len);
        if (light && !full_check) {
            // the two words the sentinels zeroed must now hold the pattern
            const bool ok = A[i].buf[MX_DATA / 8] == mx_word(i + 3, i + 1, 0) &&
                            A[i].buf[MX_DATA / 8 + len / 8 - 1] == mx_word(i + 3, i + 1, len / 8 - 1);
            if (!ok) { printf("FAIL: %s: sentinels say the push did not land\n", tag); failures++; }
        } else {
            mx_check(A[i].buf + MX_DATA / 8, i + 3, i + 1, len, what);
        }
        // The far side is still checking what we pushed; the next round
        // overwrites that region, so wait for it (outside the timing)
        if (!spin64(A[i].buf + MX_VFLAG / 8, sq, 5e6)) { printf("FAIL: %s: B%d never finished verifying A%d's data\n", tag, i + 1, i + 1); failures++; }
    };
    auto bidir_report = [&](uint64_t len) {
        if (rounds.empty()) return;
        auto med = [&](auto get) {
            std::vector<double> v; for (auto &r : rounds) v.push_back(get(r));
            std::sort(v.begin(), v.end()); return v[v.size() / 2];
        };
        auto lo = [&](auto get) { double m = 1e30; for (auto &r : rounds) m = std::min(m, get(r)); return m; };
        auto hi = [&](auto get) { double m = 0;    for (auto &r : rounds) m = std::max(m, get(r)); return m; };
        auto round_us = [](const BidirRound &r) { return std::max(r.far_landed_us, r.own_landed_us); };
        const double rm = med(round_us);
        printf("bidir timing: %zu rounds of %lu B each way, from the command store (us)\n", rounds.size(), (unsigned long) len);
        printf("bidir timing:   A -> B fenced   median %8.1f  min %8.1f  max %8.1f\n",
               med([](const BidirRound &r) { return r.fence_us; }), lo([](const BidirRound &r) { return r.fence_us; }), hi([](const BidirRound &r) { return r.fence_us; }));
        printf("bidir timing:   A -> B landed   median %8.1f  min %8.1f  max %8.1f   (%.2f GB/s)\n",
               med([](const BidirRound &r) { return r.own_landed_us; }), lo([](const BidirRound &r) { return r.own_landed_us; }), hi([](const BidirRound &r) { return r.own_landed_us; }),
               len / med([](const BidirRound &r) { return r.own_landed_us; }) / 1e3);
        printf("bidir timing:   B -> A landed   median %8.1f  min %8.1f  max %8.1f   (%.2f GB/s)\n",
               med([](const BidirRound &r) { return r.far_landed_us; }), lo([](const BidirRound &r) { return r.far_landed_us; }), hi([](const BidirRound &r) { return r.far_landed_us; }),
               len / med([](const BidirRound &r) { return r.far_landed_us; }) / 1e3);
        printf("bidir timing:   round (later landing) median %8.1f  min %8.1f  max %8.1f   -> %.2f GB/s aggregate, both directions\n",
               rm, lo(round_us), hi(round_us), 2.0 * len / rm / 1e3);
        for (size_t k = 0; k < rounds.size(); k++)
            if (round_us(rounds[k]) > 3 * rm)
                printf("bidir timing:   OUTLIER round %zu: fenced %.1f, far landed %.1f, own landed %.1f us\n",
                       k + 1, rounds[k].fence_us, rounds[k].far_landed_us, rounds[k].own_landed_us);
        fflush(stdout);
    };
    if (const char *e = getenv("LOOM_MATRIX_LOCAL")) {
        // Only local copies, N rounds: A1's engine pulls 4 MiB from host
        // memory and writes it back into A2's buffer - one host doing a
        // DMA read and a DMA write at the same time, no network. The
        // control for the bidirectional rate: the host DMA path does both
        // at ~9.8 GB/s each (2026-09-17), so it is not what the two
        // directions share.
        using clk = std::chrono::steady_clock;
        const int n_rounds = atoi(e);
        printf("\n== matrix local only: A1 -> A2, %d rounds of %lu B\n", n_rounds, (unsigned long) LEN_R);
        fflush(stdout);
        for (uint64_t w = 0; w < LEN_R / 8; w++) A[0].src[w] = mx_word(1, 2, w);
        memset(A[1].buf + MX_DATA / 8, 0, LEN_R);
        std::vector<double> t;
        for (int r = 0; r < n_rounds + 1; r++) {          // +1: the first is the warm-up
            const uint64_t f = *A[0].fence;
            const auto t0 = clk::now();
            A[0].xpu->copy(to_a[1], uint32_t(MX_DATA), A[0].src, LEN_R, A[0].fence);
            if (!spin64_ge(A[0].fence, f + 1, 5e6)) { printf("FAIL: local A1 -> A2 round %d never fenced\n", r); failures++; break; }
            const double us = std::chrono::duration<double, std::micro>(clk::now() - t0).count();
            if (r == 0) printf("local timing: warm-up round %.1f us\n", us); else t.push_back(us);
        }
        if (!t.empty()) {
            std::vector<double> v = t; std::sort(v.begin(), v.end());
            printf("local timing: %zu rounds of %lu B: fenced median %.1f  min %.1f  max %.1f us   -> %.2f GB/s (read + write of that many bytes each, on one host)\n",
                   v.size(), (unsigned long) LEN_R, v[v.size() / 2], v.front(), v.back(), LEN_R / v[v.size() / 2] / 1e3);
        }
        mx_check(A[1].buf + MX_DATA / 8, 1, 2, LEN_R, "matrix local: A1 -> A2 landed (last round)");
        printf("== matrix done\n");
        fflush(stdout);
        return;
    }
    if (const char *e = getenv("LOOM_MATRIX_RXLOCAL")) {
        // A1 copies 4 MiB into A2 (LOCAL route) while B1 pushes 4 MiB into A1
        // (arriving as local writes out of loom_rx). A's vFPGA therefore
        // issues only STRM_HOST requests while receiving as many packets as a
        // bidirectional round - the same load, without a remote request ever
        // sitting at the head of the shared sq_wr port. If the halving is
        // head-of-line blocking between the local and remote paths in the
        // shell's request demux, both directions here stay near full rate; if
        // both drop to ~5.9 GB/s, that story is wrong.
        using clk = std::chrono::steady_clock;
        const int n_rounds = atoi(e);
        printf("\n== matrix rxlocal: A1 -> A2 local 4 MiB while B1 -> A1 pushes 4 MiB, %d rounds\n", n_rounds);
        fflush(stdout);
        for (uint64_t w = 0; w < LEN_R / 8; w++) A[0].src[w] = mx_word(1, 2, w);
        std::vector<double> t_loc, t_in;
        const int f0 = failures;
        for (int r = 0; r < n_rounds + 1 && failures == f0; r++) {   // +1: warm-up, not counted
            memset(A[0].buf + MX_DATA / 8, 0, LEN_R);
            memset(A[1].buf + MX_LOCAL / 8, 0, LEN_R);
            const uint64_t sq = ++seq, f = *A[0].fence;
            A[0].xpu->store(to_b[0][0], uint32_t(MX_LEN), LEN_R);
            const auto t0 = clk::now();
            A[0].xpu->store(to_b[0][0], uint32_t(MX_CMD), mx_cmd(CMD_PUSHME, 1, sq));
            // ...and now our own 4 MiB, on the local route
            A[0].xpu->copy(to_a[1], uint32_t(MX_LOCAL), A[0].src, LEN_R, A[0].fence);
            if (!spin64_ge(A[0].fence, f + 1, 5e6)) { printf("FAIL: rxlocal: A1 -> A2 never fenced\n"); failures++; break; }
            const double loc = std::chrono::duration<double, std::micro>(clk::now() - t0).count();
            if (!spin64(A[0].buf + MX_RFLAG / 8, sq, 5e6)) { printf("FAIL: rxlocal: B1 -> A1 never arrived\n"); failures++; break; }
            const double in = std::chrono::duration<double, std::micro>(clk::now() - t0).count();
            if (r) { t_loc.push_back(loc); t_in.push_back(in); }
            mx_check(A[1].buf + MX_LOCAL / 8, 1, 2, LEN_R, "rxlocal: A1 -> A2 local landed");
            mx_check(A[0].buf + MX_DATA / 8, 3, 1, LEN_R, "rxlocal: B1 -> A1 landed");
            if (failures != f0) break;
        }
        if (!t_loc.empty()) {
            auto med = [](std::vector<double> v) { std::sort(v.begin(), v.end()); return v[v.size() / 2]; };
            const double ml = med(t_loc), mi = med(t_in);
            printf("rxlocal timing: %zu rounds of %lu B each: A1 -> A2 local median %.1f us (%.2f GB/s), "
                   "B1 -> A1 incoming median %.1f us (%.2f GB/s) -> %.2f GB/s aggregate on A\n",
                   t_loc.size(), (unsigned long) LEN_R, ml, LEN_R / ml / 1e3, mi, LEN_R / mi / 1e3,
                   2.0 * LEN_R / std::max(ml, mi) / 1e3);
        }
        printf("== matrix done\n");
        fflush(stdout);
        return;
    }
    if (const char *e = getenv("LOOM_MATRIX_BIDIR")) {
        // Only the bidirectional exchange, N rounds
        const int n_rounds = atoi(e);
        printf("\n== matrix bidir only: A1 <-> B1, %d rounds of %lu B each way\n", n_rounds, (unsigned long) LEN_R);
        fflush(stdout);
        for (uint64_t w = 0; w < LEN_R / 8; w++) A[0].src[w] = mx_word(1, 3, w);
        const int f0 = failures;
        if (!getenv("LOOM_BIDIR_NOWARMUP")) {
            // One SEQUENTIAL exchange first (B -> A, then A -> B), not
            // counted. The first exchange after setup costs ~30 ms on the
            // server side, whatever its shape (its host DMA and engine do
            // nothing, then everything completes); paid once per process.
            // Concurrent with the client's first push, that freeze outlives
            // the RC retransmit timer and wedged ~1 run in 5 (2026-09-16).
            // Sequential, it is harmless. LOOM_BIDIR_NOWARMUP=1 reproduces
            // the stall (the outlier report names the round).
            bidir(0, LEN_R, ++seq, "matrix bidir warm-up", true);
            if (failures != f0) { printf("matrix bidir: warm-up failed\n"); }
            else printf("matrix bidir: warm-up (sequential exchange) done, %.1f us\n",
                        std::max(rounds.back().far_landed_us, rounds.back().own_landed_us));
            rounds.clear();
        }
        for (int r = 0; r < n_rounds && failures == f0; r++) {
            bidir(0, LEN_R, ++seq, "matrix bidir", false, !light || (r == n_rounds - 1));
            if (failures != f0) { printf("matrix bidir: stopping after the first failure (round %d of %d)\n", r + 1, n_rounds); break; }
        }
        bidir_report(LEN_R);
        if (const char *k = getenv("LOOM_STORM_STORES")) {
            // Same rounds, but B answers with K 64 B stores instead of 4 MiB:
            // does A's push slow down per PACKET B sends (rx cost per
            // packet) or per BYTE? Fence-clocked: 394 us alone, ~700 us
            // against a 4 MiB push.
            const uint64_t K = strtoull(k, nullptr, 0);
            std::vector<double> t;
            for (int r = 0; r < n_rounds && failures == f0; r++) {
                using clk = std::chrono::steady_clock;
                memset(A[0].buf + MX_DATA / 8, 0, 8);
                const uint64_t sq = ++seq;
                const uint64_t f = *A[0].fence;
                A[0].xpu->store(to_b[0][0], uint32_t(MX_LEN), K);
                const auto t0 = clk::now();
                A[0].xpu->store(to_b[0][0], uint32_t(MX_CMD), mx_cmd(CMD_STORM, 1, sq));
                A[0].xpu->copy(to_b[0][0], uint32_t(MX_DATA), A[0].src, LEN_R, A[0].fence);
                if (!spin64_ge(A[0].fence, f + 1, 5e6)) { printf("FAIL: storm: A1 -> B1 never fenced\n"); failures++; break; }
                t.push_back(std::chrono::duration<double, std::micro>(clk::now() - t0).count());
                A[0].xpu->store(to_b[0][0], uint32_t(MX_BFLAG), sq);
                if (!spin64(A[0].buf + MX_ACK / 8, sq, 5e6)) { printf("FAIL: storm: B1 never acked\n"); failures++; break; }
                if (!spin64(A[0].buf + MX_VFLAG / 8, sq, 5e6)) { printf("FAIL: storm: B1 never verified\n"); failures++; break; }
            }
            if (!t.empty()) {
                std::vector<double> v = t; std::sort(v.begin(), v.end());
                printf("storm timing: A -> B 4 MiB fenced while B fires %lu x 64 B stores: median %.1f  min %.1f  max %.1f us\n",
                       (unsigned long) K, v[v.size() / 2], v.front(), v.back());
            }
        }
        printf("== matrix done\n");
        fflush(stdout);
        return;
    }
    printf("\n== matrix: 2 local + 2 remote XPUs\n");
    fflush(stdout);
    auto remote = [&](int i, int j, uint64_t len, uint64_t sq, const char *tag) {
        for (uint64_t w = 0; w < len / 8; w++) A[i].src[w] = mx_word(i + 1, j + 3, w);
        memset(A[i].buf + MX_DATA / 8, 0, len);
        const uint64_t f = *A[i].fence;
        A[i].xpu->copy(to_b[i][j], uint32_t(MX_DATA), A[i].src, len, A[i].fence);
        if (!spin64_ge(A[i].fence, f + 1, 5e6)) { printf("FAIL: %s: never fenced\n", tag); failures++; return; }
        A[i].xpu->store(to_b[i][j], uint32_t(MX_LEN), len);
        A[i].xpu->store(to_b[i][j], uint32_t(MX_CMD), mx_cmd(CMD_REMOTE, i + 1, sq));
        if (!spin64(A[i].buf + MX_RFLAG / 8, sq, 5e6)) { printf("FAIL: %s: reply never arrived\n", tag); failures++; return; }
        char what[96];
        snprintf(what, sizeof what, "%s: B%d -> A%d reply landed (%lu B)", tag, j + 1, i + 1, (unsigned long) len);
        mx_check(A[i].buf + MX_DATA / 8, i + 1, j + 3, len, what);
    };
    // --- local, client side: A1 -> A2, A2 -> A1 ---
    for (int i = 0; i < 2; i++) {
        const int k = 1 - i;
        for (uint64_t w = 0; w < LEN_L / 8; w++) A[i].src[w] = mx_word(i + 1, k + 1, w);
        memset(A[k].buf + MX_LOCAL / 8, 0, LEN_L);
        const uint64_t f = *A[i].fence;
        A[i].xpu->copy(to_a[k], uint32_t(MX_LOCAL), A[i].src, LEN_L, A[i].fence);
        if (!spin64_ge(A[i].fence, f + 1, 5e6)) { printf("FAIL: local A%d->A%d never fenced\n", i+1, k+1); failures++; }
        A[i].xpu->store(to_a[k], uint32_t(MX_LFLAG), ++seq);
        if (!spin64(A[k].buf + MX_LFLAG / 8, seq, 5e6)) { printf("FAIL: local A%d->A%d flag never landed\n", i+1, k+1); failures++; continue; }
        char what[64]; snprintf(what, sizeof what, "matrix: A%d -> A%d local landed", i + 1, k + 1);
        mx_check(A[k].buf + MX_LOCAL / 8, i + 1, k + 1, LEN_L, what);
    }
    // --- remote, every pair, one at a time ---
    remote(0, 0, LEN_R, ++seq, "matrix: A1 -> B1");
    remote(1, 1, LEN_R, ++seq, "matrix: A2 -> B2");
    remote(0, 1, LEN_R, ++seq, "matrix: A1 -> B2");
    remote(1, 0, LEN_R, ++seq, "matrix: A2 -> B1");
    // --- bidirectional: A1 -> B1 and B1 -> A1 at once, one issuer per engine ---
    for (uint64_t w = 0; w < LEN_R / 8; w++) A[0].src[w] = mx_word(1, 3, w);
    for (int r = 0, f0 = failures; r < 8 && failures == f0; r++) bidir(0, LEN_R, ++seq, "matrix bidir");
    // --- concurrent: A1<->B1 and A2<->B2 at once, several rounds ---
    {
        const int rounds = 8;
        uint64_t s1 = seq + 0x1000, s2 = seq + 0x2000;
        std::thread t1([&] { for (int r = 0; r < rounds; r++) remote(0, 0, LEN_R, s1 + r, "matrix concurrent: A1 -> B1"); });
        std::thread t2([&] { for (int r = 0; r < rounds; r++) remote(1, 1, LEN_R, s2 + r, "matrix concurrent: A2 -> B2"); });
        t1.join(); t2.join();
        seq += 0x3000;
    }
    printf("== matrix done\n");
    fflush(stdout);
}

int run_server(uint16_t qp_port, uint16_t peer_port, const std::string &sock) {
    const uint32_t dev = coyote_device();
    coyote::cThread t_ctrl(0, getpid(), dev);
    // The QP owner: a cThread of its own that holds the RC connection and
    // the staging buffer and never moves data. The XPUs are the data
    // cThreads; every XPU on this host shares the one QP, and the message
    // header names which XPU an incoming write lands in.
    coyote::cThread t_qp(0, getpid(), dev);
    coyote::cThread t_data(0, getpid(), dev);              // XPU 1 (B1)
    std::unique_ptr<coyote::cThread> t_x2;                 // XPU 2 (B2), LOOM_XPUS=2
    if (n_xpus() >= 2) t_x2 = std::make_unique<coyote::cThread>(0, getpid(), dev);
    printf("ctids: ctrl %d, qp %d, xpu1 %d%s\n", t_ctrl.getCtid(), t_qp.getCtid(),
           t_data.getCtid(), t_x2 ? (", xpu2 " + std::to_string(t_x2->getCtid())).c_str() : "");

    // QP exchange (blocks until the client's initRDMA connects); the
    // returned buffer is this host's RDMA staging area
    printf("server: waiting for QP exchange on port %u ...\n", qp_port);
    void *staging = t_qp.initRDMA(STAGING_BYTES, qp_port);
    if (!staging) { printf("FAIL: initRDMA\n"); return 1; }
    printf("server: QP up, staging %p\n", staging);

    // The engine's RETH for messages going out (the far staging comes with
    // the far side's PONG); loom_rx ignores the RETH and lands by header.
    loom::set_rdma_staging(t_ctrl, staging);

    // Exporter role: XPU 1's two destination segments, XPU 2's one
    auto *dst1 = static_cast<uint64_t *>(
        t_data.getMem({coyote::CoyoteAllocType::HPF, BUF_SIZE}));
    auto *dst2 = static_cast<uint64_t *>(
        t_data.getMem({coyote::CoyoteAllocType::HPF, BUF_SIZE}));
    if (!dst1 || !dst2) { printf("FAIL: getMem\n"); return 1; }
    memset(dst1, 0, BUF_SIZE);
    memset(dst2, 0, BUF_SIZE);
    uint64_t *dst3 = nullptr;
    if (t_x2) {
        dst3 = static_cast<uint64_t *>(t_x2->getMem({coyote::CoyoteAllocType::HPF, MX_BUF}));
        if (!dst3) { printf("FAIL: getMem xpu2\n"); return 1; }
        memset(dst3, 0, MX_BUF);
    }

    loom::BundledOrchestrator orch(t_ctrl);
    orch.setQpOwner(t_qp.getCtid());         // the wire for replies
    // Engine knobs, read back and printed: a knob that silently did not
    // take is how a whole experiment gets misread, and the defaults live in
    // the bitstream, so an unset variable is not the same as a zero.
    // LOOM_TX_WINDOW is the flow control (packets unacked); LOOM_TX_PACE is
    // an optional manual cap on top, off by default.
    if (const char *e = getenv("LOOM_TX_PACE"))
        loom::csr_write(t_ctrl, loom::TX_PACE, parse_pace(e));
    print_pace(loom::csr_read(t_ctrl, loom::TX_PACE), "at init");
    loom::csr_write(t_ctrl, loom::TX_CTL, tx_ctl_from_env(loom::csr_read(t_ctrl, loom::TX_CTL)));
    print_tx_ctl(loom::csr_read(t_ctrl, loom::TX_CTL), "at init");
    arm_rx_chunk(t_ctrl, "at init");
    loom::Handle h1 = orch.exportBuf(t_data.getCtid(), dst1, BUF_SIZE);
    loom::Handle h2 = orch.exportBuf(t_data.getCtid(), dst2, BUF_SIZE);
    printf("server: exported handles %u, %u\n", h1, h2);
    if (t_x2) {
        loom::Handle h3 = orch.exportBuf(t_x2->getCtid(), dst3, MX_BUF);
        printf("server: exported handle %u (xpu2)\n", h3);
    }

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
    if (pingpong_mode() || matrix_mode()) {
        // This side transmits too: an Xpu per XPU cThread, one fence each,
        // one io mutex for all of them (the doorbell words are shared)
        std::mutex io_mtx_s;
        loom::Xpu S1(t_data, orch, io_mtx_s);
        auto *fence_s1 = static_cast<uint64_t *>(S1.allocSmall(4096));
        if (!fence_s1) { printf("FAIL: alloc fence\n"); return 1; }
        memset(fence_s1, 0, 4096);
        const int want_pongs = matrix_mode() ? 2 : 1;
        auto t0 = std::chrono::steady_clock::now();
        while (orch.pongWindow(want_pongs - 1) == loom::NO_WINDOW && peer.doneCount() < 1 &&
               std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < done_secs)
            usleep(1000);
        if (orch.pongWindow(want_pongs - 1) == loom::NO_WINDOW)
            printf("server: PONG setup never arrived\n");
        else if (matrix_mode()) {
            if (!t_x2) { printf("FAIL: matrix needs LOOM_XPUS=2\n"); failures++; }
            else {
                loom::Xpu S2(*t_x2, orch, io_mtx_s);
                auto *fence_s2 = static_cast<uint64_t *>(S2.allocSmall(4096));
                if (!fence_s2) { printf("FAIL: alloc fence xpu2\n"); return 1; }
                memset(fence_s2, 0, 4096);
                MxServerXpu x1{&S1, dst1, fence_s1, 1, {orch.pongWindow(0), orch.pongWindow(1)}};
                MxServerXpu x2{&S2, dst3, fence_s2, 2, {orch.pongWindow(0), orch.pongWindow(1)}};
                auto done = [&] { return peer.doneCount() >= 1; };
                std::thread th1([&] { mx_serve(x1, done); });
                std::thread th2([&] { mx_serve(x2, done); });
                th1.join(); th2.join();
            }
        } else
            serve_pong(S1, orch.pongWindow(0), dst1, fence_s1,
                       [&] { return peer.doneCount() >= 1; });
    } else if (!probe)
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

    if (bench_mode() && !pingpong_mode() && !matrix_mode()) {
        printf("\n== exporter check of the benchmark regions\n");
        std::vector<uint64_t> only_e_set;
        if (const char *o = getenv("LOOM_BENCH_ONLY"))
            for (const char *q = o; *q; ) {
                only_e_set.push_back(strtoull(q, nullptr, 0));
                while (*q && *q != ',') q++;
                if (*q == ',') q++;
            }
        const uint64_t only_l = only_e_set.empty() ? 0 : 1;   // "a list was given"
        auto e_wanted = [&](uint64_t l) {
            if (only_e_set.empty()) return true;
            for (uint64_t v : only_e_set) if (v == l) return true;
            return false;
        };
        const char *from_e = getenv("LOOM_BENCH_FROM");
        const uint64_t from_l = from_e ? strtoull(from_e, nullptr, 0) : 0;
        for (int i = 0; i < BENCH_N; i++) {
            const uint64_t len = BENCH_SIZES[i], off = bench_offset(i);

            // A focused run sent ONE size; the others are not "untouched
            // regions", they were never asked for. Skipping them by name
            // beats inferring it from a region reading zero - and with
            // LOOM_BENCH_OFF every size shares an address, so the zero test
            // would report the sent bytes under fourteen wrong labels.
            if (!e_wanted(len)) continue;
            if (from_l && len < from_l) continue;
            if (off + len > BUF_SIZE) continue;

            // The importer stops at the first size that loses packets, so
            // the sizes after it were never sent. An untouched region is
            // not a failure, and reporting it as one buries the size that
            // actually broke.
            bool touched = false;
            for (uint64_t w = 0; !touched && w < len / 8; w++)
                if (dst1[off / 8 + w] != 0) touched = true;
            if (!touched) {
                // With LOOM_BENCH_ONLY there is no "before this size" - the
                // one size asked for arrived as nothing at all, and calling
                // that untouched is how a run that transferred NOTHING used
                // to print SERVER PASS.
                if (only_l) {
                    printf("  %8lu B at 0x%-8lx: nothing arrived\n",
                           (unsigned long) len, (unsigned long) off);
                    check(false, "bench region CORRUPT");
                    continue;
                }
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

                    // Characterise the HOLE, not just the first bad word.
                    // Each word carries its own source index, so every word
                    // says exactly which source word landed there and the
                    // damage can be read off rather than guessed. The whole
                    // question is how many bytes went missing and at what
                    // alignment: a 64 B hole at 16 B alignment is a very
                    // different fault from one at 64 B alignment.
                    printf("    --- context, word index : source index "
                           "landed there (delta) ---\n");
                    const uint64_t lo = (w >= 24) ? ((w - 24) & ~7ULL) : 0;
                    for (uint64_t v = lo; v < w + 40 && v < len / 8; v += 8) {
                        printf("    +%-8lu", (unsigned long) (v * 8));
                        for (int l = 0; l < 8; l++) {
                            const uint64_t got = dst1[off / 8 + v + l];
                            const uint64_t idx = got & 0xFFFFFFFFFFFFULL;
                            const long d = (long) idx - (long) (v + l);
                            if (got == 0)            printf("      ZERO");
                            else if ((got >> 48) != (0xBE0 + i))
                                                     printf("     ALIEN");
                            else if (d == 0)         printf("         .");
                            else                     printf(" %+9ld", d);
                        }
                        printf("\n");
                    }
                    printf("    ('.' = correct; a number is how many source "
                           "words LATER the word that landed here came "
                           "from, so it is the size of the hole in words; "
                           "byte alignment of the first nonzero tells you "
                           "the datapath width that lost it)\n");
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
        // accepted counts PACKETS (every request is drained), completed
        // counts MESSAGES. The ratio is packets per message; there is no
        // identity between them to check.
        printf("rq_wr: %lu accepted (packets), %lu completed (messages)\n",
               (unsigned long) acc, (unsigned long) don);
        // Sum vs worst run: the ingress FIFO is 512 beats in the shell plus
        // 512 in user logic, so a worst run well under that is a burst the
        // buffer can swallow. A worst run comparable to the total means the
        // write path is simply slower than the link and no depth fixes it.
        const uint64_t pd = loom::csr_read(t_ctrl, loom::PULL_DESYNC);
        if (pd)
            printf("  *** pull desync: %lu beat(s) left on the pull stream "
                   "when a read was issued - payload displaced ***\n",
                   (unsigned long) pd);
        // The receive path's own account of beats nobody announced. Printed
        // unconditionally: a zero here is a result, not an absence.
        const uint64_t orph = loom::csr_read(t_ctrl, loom::RX_ORPHAN);
        printf("  rx orphan beats: %lu   (beats no rq_wr request accounted "
               "for; nonzero means the receive path handed up a packet it "
               "did not deliver)\n", (unsigned long) orph);
        {
            const uint64_t ff  = loom::csr_read(t_ctrl, loom::RX_FIFO_FULL);
            const uint64_t ffm = loom::csr_read(t_ctrl, loom::RX_FIFO_FULL_MAX);
            printf("  ingress FIFO FULL: %lu cycles, longest run %lu -> %s\n",
                   (unsigned long) ff, (unsigned long) ffm,
                   ff == 0 ? "never; the shell never had a beat we could not take"
                           : "the shell offered a beat and our ingress FIFO was "
                             "full. From here backpressure walks into the shell "
                             "(stack input, rx_crossing) and the CMAC, which "
                             "has no tready, DROPS - before every counter. "
                             "This is the loss");
        }
        const uint64_t bp  = loom::csr_read(t_ctrl, loom::RX_BP);
        const uint64_t bpm = loom::csr_read(t_ctrl, loom::RX_BP_MAX);
        printf("  ingress backpressure: %lu cycles, longest run %lu -> %s\n",
               (unsigned long) bp, (unsigned long) bpm,
               "NOT a fault signal - measured INVERSELY correlated with "
               "corruption. The clean 9.84 GB/s run carries the HIGHEST "
               "backpressure seen (28198 cycles, RX stalled 27.6%) while both "
               "corrupt ~12.5 GB/s runs sit near 17000 / 7%. High values mean "
               "the sender is being throttled, which is the safe state. Read "
               "the RATE, not this.");
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
    if (matrix_mode()) { dump_tx(t_ctrl, "server final"); dump_port(t_ctrl, "server final"); }

    t_qp.connSync(false);
    peer.stop(); peer_thr.join();
    loomd.stop(); loomd_thr.join();

    printf(failures == 0 ? "LOOM HOST SERVER PASS\n"
                         : "LOOM HOST SERVER FAIL (%d)\n", failures);
    return failures == 0 ? 0 : 1;
}

int run_client(const std::string &ip, uint16_t qp_port, uint16_t peer_port,
               const std::string &sock) {
    const uint32_t dev = coyote_device();
    coyote::cThread t_ctrl(0, getpid(), dev);
    coyote::cThread t_qp(0, getpid(), dev);                // the QP owner
    coyote::cThread t_data(0, getpid(), dev);              // XPU 1 (A1)
    std::unique_ptr<coyote::cThread> t_x2;                 // XPU 2 (A2), LOOM_XPUS=2
    if (n_xpus() >= 2) t_x2 = std::make_unique<coyote::cThread>(0, getpid(), dev);
    printf("ctids: ctrl %d, qp %d, xpu1 %d%s\n", t_ctrl.getCtid(), t_qp.getCtid(),
           t_data.getCtid(), t_x2 ? (", xpu2 " + std::to_string(t_x2->getCtid())).c_str() : "");

    // QP exchange (the server is already blocking in its initRDMA)
    void *staging_local = t_qp.initRDMA(STAGING_BYTES, qp_port, ip.c_str());
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
    orch.attachPeer(peer, t_qp.getCtid());
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

    // The chunk register lives on the SENDER's engine, so the client needs
    // it too - the server write above only covers the receive side.
    // Engine knobs, read back and printed: a knob that silently did not
    // take is how a whole experiment gets misread, and the defaults live in
    // the bitstream, so an unset variable is not the same as a zero.
    // LOOM_TX_WINDOW is the flow control (packets unacked); LOOM_TX_PACE is
    // an optional manual cap on top, off by default.
    if (const char *e = getenv("LOOM_TX_PACE"))
        loom::csr_write(t_ctrl, loom::TX_PACE, parse_pace(e));
    print_pace(loom::csr_read(t_ctrl, loom::TX_PACE), "at init");
    loom::csr_write(t_ctrl, loom::TX_CTL, tx_ctl_from_env(loom::csr_read(t_ctrl, loom::TX_CTL)));
    print_tx_ctl(loom::csr_read(t_ctrl, loom::TX_CTL), "at init");
    arm_rx_chunk(t_ctrl, "at init");

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
    dump_counters(t_ctrl, "client after the functional tests");

    // Re-arm the pacing knob right before the bench. On hardware a write to
    // any table register (TBL_IDX/CFG/LEN/COMMIT - words 0-5) also clobbers
    // words 4 and 6 of the same 64-byte line with stale or address-derived
    // data; releaseWindow above does exactly that and wiped CSR 6 in every
    // run of the first sweep. Simulation does not reproduce it, so it is the
    // ctrl mapping or the shell's AXI-Lite bridge, not loom_ctrl (see the
    // README TODO on pgprot_writecombine for MMAP_CTRL). Nothing in line 0
    // is written after this point, so a rewrite here holds for the bench.
    if (const char *e = getenv("LOOM_TX_PACE")) {
        const uint64_t want = parse_pace(e);
        loom::csr_write(t_ctrl, loom::TX_PACE, want);
        const uint64_t got = loom::csr_read(t_ctrl, loom::TX_PACE);
        print_pace(got, got == want ? "re-armed before bench" : "re-armed before bench DID NOT TAKE");
    }
    arm_rx_chunk(t_ctrl, "re-armed before bench");
    {
        const uint64_t want = tx_ctl_from_env(loom::csr_read(t_ctrl, loom::TX_CTL));
        loom::csr_write(t_ctrl, loom::TX_CTL, want);
        const uint64_t got = loom::csr_read(t_ctrl, loom::TX_CTL);
        print_tx_ctl(got, got == want ? "re-armed before bench" : "re-armed before bench DID NOT TAKE");
    }
    if (matrix_mode()) {
        if (!t_x2) { printf("FAIL: matrix needs LOOM_XPUS=2\n"); return failures + 1; }
        loom::Xpu A2(*t_x2, orch, io_mtx);
        auto *buf1 = static_cast<uint64_t *>(A.alloc(MX_BUF));
        auto *buf2 = static_cast<uint64_t *>(A2.alloc(MX_BUF));
        auto *src2 = static_cast<uint64_t *>(A2.alloc(MX_BUF));
        auto *fence2 = static_cast<uint64_t *>(A2.allocSmall(4096));
        if (!buf1 || !buf2 || !src2 || !fence2) { printf("FAIL: alloc xpu2\n"); return failures + 1; }
        memset(buf1, 0, MX_BUF); memset(buf2, 0, MX_BUF); memset(fence2, 0, 4096);
        // Reverse path, one window per client XPU (each lands under its own
        // ctid on the far side - the header's dst pid)
        const int pw1 = peer.pong(reinterpret_cast<uint64_t>(staging_local),
                                  reinterpret_cast<uint64_t>(buf1), MX_BUF, t_data.getCtid());
        const int pw2 = peer.pong(reinterpret_cast<uint64_t>(staging_local),
                                  reinterpret_cast<uint64_t>(buf2), MX_BUF, t_x2->getCtid());
        check(pw1 > 0 && pw2 > 0, "server accepted both PONG windows");
        // Windows onto the server XPUs (shared by both client XPUs) and
        // local windows onto each other
        const int w3 = A.importBuf(3);
        check(w3 != loom::NO_WINDOW, "remote import -> rdma window onto B2");
        loom::Handle ha1 = orch.exportBuf(t_data.getCtid(), buf1, MX_BUF);
        loom::Handle ha2 = orch.exportBuf(t_x2->getCtid(), buf2, MX_BUF);
        const int la1 = orch.importLocal(ha1), la2 = orch.importLocal(ha2);
        check(la1 != loom::NO_WINDOW && la2 != loom::NO_WINDOW, "local windows onto both client XPUs");
        MxClientXpu X[2] = {{&A, buf1, src, fence}, {&A2, buf2, src2, fence2}};
        int to_b[2][2] = {{w1, w3}, {w1, w3}};
        int to_a[2] = {la1, la2};
        run_matrix(X, to_b, to_a);
        dump_counters(t_ctrl, "client after the matrix");
        dump_tx(t_ctrl, "client after the matrix");
        dump_rx(t_ctrl, "client after the matrix");
        dump_port(t_ctrl, "client after the matrix");
    } else if (pingpong_mode()) {
        // Reverse path: our own receive buffer, landing under XPU 1's ctid
        // (the header's dst pid), and amy told where to write and which
        // RETH to use
        auto *pong = static_cast<uint64_t *>(A.alloc(BUF_SIZE));
        if (!pong) { printf("FAIL: alloc pong\n"); return failures + 1; }
        memset(pong, 0, BUF_SIZE);
        check(peer.pong(reinterpret_cast<uint64_t>(staging_local),
                        reinterpret_cast<uint64_t>(pong), BUF_SIZE, t_data.getCtid()) > 0,
              "server accepted the PONG window");
        run_pingpong(A, w1, src, pong, fence);
    } else if (bench_mode())
        run_bench(t_ctrl, A, w1, src, fence);

    check(peer.done(), "DONE barrier acknowledged");
    t_qp.connSync(true);
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
