/**
 * ce_get - copy-engine gets: the client's V80 copy engine READS a window of
 * its U280 that is bound to the server's export, into its own HBM, and the
 * switches do the rest:
 *
 *   client V80 loom_ce --P2P reads--> client U280 uwin (loom_read) --get
 *   requests--> server loom_rx / loom_rd: read the export, write the lines
 *   back --> client loom_rx --> loom_read --> the reads' data --> V80 HBM
 *
 * Server (the host read from; U280 only):
 *     get_bench --server [--port N] [--size BYTES]     (examples/loom_switch/sw)
 *   a source buffer with get_bench's pattern, exported as export 1, gets
 *   answered on its QP owner's QP.
 *
 * Client (the reader; U280 + V80):
 *     ce_get --client <server_ip> [--port N] [--size BYTES] [--reps N] [--window P]
 *   window 1 onto the server's export 1 (uwin 0 .. size), that part of the
 *   uwin exported to the V80, an HBM buffer of size bytes; then for each
 *   transfer of 4 KiB, 16 KiB, ... up to size, reps times: clear the HBM
 *   buffer, one get (DIR 1, no fence), wait for BUSY to drop (the HBM write
 *   has completed), sync the buffer back and check every byte. Prints the
 *   copy engine's time (START to the HBM write's completion, 250 MHz), and
 *   the reads the U280 saw per get (their count and mean size: the V80's
 *   PCIe max read request size) and the cycles they waited for a read slot.
 *   Both sides use the same --size (get_bench's default 16 MiB).
 *
 * Local (one host, no network, no server):
 *     ce_get --local [--size BYTES] [--reps N]
 *   the same gets from a part of the uwin with no window bound: loom_read
 *   answers every read on the card with all ones (the get_bench error path),
 *   so this times the peer-to-peer read path alone (V80 -> U280 shell ->
 *   loom_read) and checks every byte is all ones.
 *
 * The QP port is N (Coyote's default if not given). --window sets this
 * host's ack window (TX_CTL: rdma packets unacked, 0 = no limit; 16 after
 * reset); the server's is get_bench --window.
 */
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <immintrin.h>
#include <string>

#include <coyote/cThread.hpp>
#include "loom_switch.hpp"

namespace {

using namespace loom_switch;
using Clock = std::chrono::steady_clock;

enum CeReg : uint32_t { START = 0, SRC_VA = 1, DST_VA = 2, LEN = 3, PID = 4, FENCE_VA = 5, DIRECTION = 6,
                        BUSY = 8, COPIES = 9, CYCLES = 10 };

constexpr uint64_t STAGING_SIZE = 1 << 20;
constexpr uint32_t SRC_EXPORT   = 1;     // on the server, as get_bench

// get_bench's source pattern
uint64_t pattern(uint64_t off) { return 0x5EED000000000000ULL ^ (off * 0x9E3779B97F4A7C15ULL); }

// local: no window bound, every read answered with all ones on the card
int run_client(const std::string &ip, uint16_t port, uint64_t size, int reps, int window) {
    const bool local = ip.empty();
    coyote::cThread u280(0, getpid(), 0, nullptr, "coyote_fpga");     // QP owner, CSR page
    coyote::cThread v80(0, getpid(), 0, nullptr, "coyote_versal_fpga");
    if (local) {
        for (int w = 1; w <= 2; w++) release_window(u280, w);
    } else {
        printf("client: QP exchange with %s on port %u ...\n", ip.c_str(), port);
        if (!u280.initRDMA(STAGING_SIZE, port, ip.c_str())) { printf("FAIL: initRDMA\n"); return 1; }
        if (window >= 0) csr_write(u280, TX_CTL, uint64_t(window));
        printf("client: ack window %lu packets\n", (unsigned long) csr_read(u280, TX_CTL));
        program_window(u280, 1, true, u280.getCtid(), reinterpret_cast<const void *>(export_ref(SRC_EXPORT)), size, 0);
    }
    const int dfd = u280.exportDmabuf(EXPORT_REGION_UWIN, 0, size);
    void *uva = reserve_va(size);
    v80.importDmabuf(dfd, uva);
    uint64_t *dst = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, size}));
    if (local)
        printf("client: uwin 0 .. %lu, no window bound, imported into the V80 at %p; HBM buffer %p\n",
               (unsigned long) size, uva, (void *) dst);
    else {
        printf("client: window 1 onto the server's export %u (%lu bytes), imported into the V80 at %p; HBM buffer %p\n",
               SRC_EXPORT, (unsigned long) size, uva, (void *) dst);
        u280.connSync(true);   // the server is ready
    }

    int errors = 0;
    for (uint64_t len = 4096; len <= size; len *= 4) {
        for (int r = 0; r < reps; r++) {
            memset(dst, 0, len);
            v80.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{dst, len});
            const uint64_t rd0 = csr_read(u280, RD_READS), ln0 = csr_read(u280, RD_LINES);
            const uint64_t sw0 = csr_read(u280, RD_SLOT_WAIT), f0 = csr_read(u280, RD_FAILED);

            v80.setCSR(reinterpret_cast<uint64_t>(uva), SRC_VA);
            v80.setCSR(reinterpret_cast<uint64_t>(dst), DST_VA);
            v80.setCSR(len, LEN);
            v80.setCSR(v80.getCtid(), PID);
            v80.setCSR(0, FENCE_VA);
            v80.setCSR(1, DIRECTION);
            const auto t0 = Clock::now();
            v80.setCSR(1, START);
            while (v80.getCSR(BUSY) && Clock::now() - t0 < std::chrono::seconds(5)) _mm_pause();
            if (v80.getCSR(BUSY)) {
                printf("FAIL get %lu bytes: still busy after 5 s (U280 reads %lu, failed %lu)\n", (unsigned long) len,
                       (unsigned long) (csr_read(u280, RD_READS) - rd0), (unsigned long) (csr_read(u280, RD_FAILED) - f0));
                return 1;
            }
            const uint64_t cyc = v80.getCSR(CYCLES);
            const uint64_t reads = csr_read(u280, RD_READS) - rd0, lines = csr_read(u280, RD_LINES) - ln0;
            const uint64_t swait = csr_read(u280, RD_SLOT_WAIT) - sw0, failed = csr_read(u280, RD_FAILED) - f0;

            v80.invoke(coyote::CoyoteOper::LOCAL_SYNC, coyote::syncSg{dst, len});
            uint64_t bad = 0;
            for (uint64_t i = 0; i < len / 8; i++) bad += (dst[i] != (local ? ~0ULL : pattern(8 * i)));
            // local: every read fails by design, and no lines come back
            const bool ok = !bad && (local ? failed == reads : failed == 0);
            errors += !ok;
            const double us = cyc * 4e-3;
            printf("%s get %8lu bytes: %9.1f us (%6.2f GB/s); %lu of %lu words wrong; U280 %lu reads (%.0f B each), "
                   "%lu failed, %lu cycles waiting for a slot\n",
                   ok ? "ok  " : "FAIL", (unsigned long) len, us, len / us / 1e3,
                   (unsigned long) bad, (unsigned long) (len / 8), (unsigned long) reads,
                   reads ? (local ? double(len) : 64.0 * lines) / reads : 0.0, (unsigned long) failed, (unsigned long) swait);
        }
    }

    v80.setCSR(0, DIRECTION);
    if (!local) {
        u280.connSync(true);   // done
        release_window(u280, 1);
    }
    printf(errors ? "CE GET FAIL\n" : "CE GET PASS\n");
    return errors ? 1 : 0;
}

} // namespace

int main(int argc, char *argv[]) {
    std::string ip;
    bool local = false;
    uint16_t port = coyote::DEF_PORT;
    uint64_t size = 16ULL << 20;
    int reps = 3;
    int window = -1;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--client" && i + 1 < argc)     ip = argv[++i];
        else if (a == "--local")                 local = true;
        else if (a == "--port" && i + 1 < argc)  port = uint16_t(atoi(argv[++i]));
        else if (a == "--size" && i + 1 < argc)  size = strtoull(argv[++i], nullptr, 0);
        else if (a == "--reps" && i + 1 < argc)  reps = atoi(argv[++i]);
        else if (a == "--window" && i + 1 < argc) window = atoi(argv[++i]);
        else { fprintf(stderr, "usage: see the header of ce_get.cpp\n"); return 2; }
    }
    if (local == !ip.empty() || size < 4096 || size % 4096 || size > (64ULL << 20) || reps < 1 || window > 255) {
        fprintf(stderr, "usage: see the header of ce_get.cpp (--size: a multiple of 4096, at most 64 MiB)\n");
        return 2;
    }
    return run_client(ip, port, size, reps, window);
}
