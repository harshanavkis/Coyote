/**
 * Aperture loads and stores on hardware: the path that is NOT the
 * descriptor engine.
 *
 * Everything run on silicon so far stored at offset 0x40 or 0x800 - both
 * 64 B aligned, both lane 0 - and never issued a single aperture READ.
 * Reads deadlock the C++ interactive simulation (a blocking ctrl read parks
 * the generator that has to serve the engine's pull), so ./test cannot
 * contain them and their only coverage is the block TBs and the Python
 * framework. That leaves two things unmeasured on real hardware:
 *
 *   G2, sub-64 B store alignment. A store carries one 8 B beat LSB-aligned
 *   and the shell writes `len` bytes at the request's vaddr. Whether that
 *   lands at the right byte offset for a lane other than 0 has been assumed,
 *   not seen.
 *
 *   The read path end to end: the AXI read channel is held open while the
 *   engine pulls the containing 64 B line under the DESTINATION's pid and
 *   selects the lane.
 *
 * The lane sweep writes DESCENDING on purpose. A host ctrl write covers its
 * whole 64 B line forward from the target, so ascending order survives even
 * without the strobe gate in loom_ctrl - each store's padding only reaches
 * lanes written later, which are then written correctly. Descending order
 * does not: without the gate, storing lane 6 pads over lane 7 and destroys
 * it. So this test also tells you which bitstream you are on.
 *
 * Hardware only: refuses to run under COYOTE_SIM_DIR.
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <cstring>
#include <unistd.h>

#include <coyote/cThread.hpp>
#include "loom.hpp"

namespace {

constexpr uint64_t BUF_SIZE  = 2ULL * 1024 * 1024;
constexpr uint64_t SMALL_LEN = 0x100;      // bounds-check window

int failures = 0;

void check(bool ok, const char *msg) {
    printf("%s: %s\n", ok ? "PASS" : "FAIL", msg);
    if (!ok) failures++;
}

// A store is posted, so the value arrives shortly after the write returns.
// Poll for it rather than sleeping a fixed amount: it is there in
// microseconds and a fixed sleep is either wasteful or wrong.
bool settle(volatile uint64_t *addr, uint64_t want) {
    for (int i = 0; i < 200000; i++) {          // ~2 s at 10 us
        if (*addr == want) return true;
        usleep(10);
    }
    return false;
}

uint64_t lane_val(int lane) {
    return 0x1A2E'0000'0000'0000ULL | uint64_t(lane);
}

void dump_counters(coyote::cThread &t, const char *tag) {
    static const char *name[10] = {"stores", "descs", "local_wr", "rdma_wr",
                                   "rx_fwd", "drops", "fifo_ovfl", "compl",
                                   "reads", "rx_hdr_reject"};
    printf("counters [%s]:", tag);
    for (int i = 0; i < 10; i++)
        printf(" %s=%lu", name[i],
               (unsigned long) loom::csr_read(t, loom::DBG_BASE + 8 * i));
    printf("\n");
    fflush(stdout);
}

} // namespace

int main() {
    if (getenv("COYOTE_SIM_DIR")) {
        printf("aperture_test is hardware-only: a blocking aperture read "
               "deadlocks the interactive simulation\n");
        return 2;
    }

    coyote::cThread t(0, getpid(), 0);
    const uint32_t pid = t.getCtid();
    printf("ctid %d\n", pid);

    auto *dst = static_cast<uint64_t *>(
        t.getMem({coyote::CoyoteAllocType::HPF, BUF_SIZE}));
    auto *small = static_cast<uint64_t *>(
        t.getMem({coyote::CoyoteAllocType::HPF, BUF_SIZE}));
    if (!dst || !small) { printf("FAIL: getMem\n"); return 1; }
    memset(dst, 0, BUF_SIZE);
    memset(small, 0, BUF_SIZE);

    loom::program_window(t, 1, false, pid, dst, BUF_SIZE);
    loom::program_window(t, 2, false, pid, small, SMALL_LEN);
    dump_counters(t, "start");

    // --- 1. G2: every lane of a 64 B line, written descending ---
    for (int lane = 7; lane >= 0; lane--)
        loom::aperture_store(t, 1, uint32_t(lane * 8), lane_val(lane));
    settle(&dst[0], lane_val(0));               // last one issued
    {
        bool ok = true;
        for (int lane = 0; lane < 8; lane++)
            if (dst[lane] != lane_val(lane)) {
                printf("  lane %d: got %016lx want %016lx\n", lane,
                       (unsigned long) dst[lane],
                       (unsigned long) lane_val(lane));
                ok = false;
            }
        check(ok, "G2: all 8 lanes of a line land at their own offset");
        check(dst[8] == 0, "G2: the next line is untouched");
    }

    // --- 2. Reads: the same lanes back through the aperture ---
    {
        bool ok = true;
        for (int lane = 0; lane < 8; lane++) {
            uint64_t got = loom::aperture_read(t, 1, uint32_t(lane * 8));
            if (got != lane_val(lane)) {
                printf("  read lane %d: got %016lx want %016lx\n", lane,
                       (unsigned long) got, (unsigned long) lane_val(lane));
                ok = false;
            }
        }
        check(ok, "read: lane select returns the right 8 bytes");
    }

    // --- 3. Reads across the window, away from lane 0 ---
    {
        const uint32_t offs[] = {0x8, 0x3F8, 0x400, 0x7F8, 0xFF8};
        bool ok = true;
        for (uint32_t off : offs) {
            uint64_t v = 0xC0DE'0000'0000'0000ULL | off;
            loom::aperture_store(t, 1, off, v);
            settle(&dst[off / 8], v);
            uint64_t got = loom::aperture_read(t, 1, off);
            if (dst[off / 8] != v || got != v) {
                printf("  off 0x%x: mem %016lx read %016lx want %016lx\n",
                       off, (unsigned long) dst[off / 8],
                       (unsigned long) got, (unsigned long) v);
                ok = false;
            }
        }
        check(ok, "store+read agree across the window, including 0xFF8");
    }

    // --- 4. Read-after-write through the order FIFO ---
    {
        loom::aperture_store(t, 1, 0x200, 0x9A70'0000'0000'0001ULL);
        uint64_t got = loom::aperture_read(t, 1, 0x200);
        check(got == 0x9A70'0000'0000'0001ULL,
              "read issued after a store to the same offset sees it");
    }

    // --- 5. Poison: an unprogrammed window answers, never hangs ---
    check(loom::aperture_read(t, 15, 0x40) == loom::READ_POISON,
          "read of an unprogrammed window returns poison");

    // --- 6. Bounds: window 2 is only SMALL_LEN long ---
    {
        uint64_t d0 = loom::csr_read(t, loom::DBG_BASE + 8 * 5);
        loom::aperture_store(t, 2, 0xF8, 0xB0'0000'0000'0001ULL);
        settle(&small[0xF8 / 8], 0xB0'0000'0000'0001ULL);
        check(small[0xF8 / 8] == 0xB0'0000'0000'0001ULL,
              "bounds: the last word inside the window lands");

        // Expected to be dropped, so there is no value to wait for: poll the
        // drop counter, which is what actually says the engine has seen it
        loom::aperture_store(t, 2, 0x800, 0xBAD0'0000'0000'0001ULL);
        uint64_t d1 = d0;
        for (int i = 0; i < 200000 && d1 == d0; i++) {
            d1 = loom::csr_read(t, loom::DBG_BASE + 8 * 5);
            if (d1 == d0) usleep(10);
        }
        check(d1 > d0, "bounds: a store past the window is dropped");
        check(small[0x800 / 8] == 0,
              "bounds: nothing was written past the window");
        check(loom::aperture_read(t, 2, 0x800) == loom::READ_POISON,
              "bounds: a read past the window returns poison");
    }

    // --- T6: what a load actually costs ---
    // The read holds the AXI-Lite read channel open until the engine has
    // pulled the containing 64 B line and lane-selected, so a load is a
    // blocking round trip whatever the route - local here, a network RTT
    // once 6.2b lands. Two numbers: the engine's own read-stage
    // accumulator, which is the device-side line pull, and wall clock,
    // which is what an XPU issuing the load would actually see (MMIO out,
    // pull, MMIO completion back).
    //
    // No counter reads inside the loop: CSR reads are single-outstanding,
    // so one in here would serialize behind every load and inflate it.
    {
        constexpr int N = 1000;
        loom::StageStats a = loom::read_stage_stats(t);
        auto t0 = std::chrono::steady_clock::now();
        uint64_t sink = 0;
        for (int i = 0; i < N; i++)
            sink ^= loom::aperture_read(t, 1, uint32_t((i % 64) * 8));
        auto t1 = std::chrono::steady_clock::now();
        loom::StageStats b = loom::read_stage_stats(t);

        double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        uint64_t cyc = b.acc[loom::STG_READ] - a.acc[loom::STG_READ];
        uint64_t ops = b.cnt[loom::STG_READ] - a.cnt[loom::STG_READ];
        printf("\nT6 local load, %d reads: %lu cyc/read in the engine, "
               "%.3f us/read end to end (sink %016lx)\n",
               N, (unsigned long) (ops ? cyc / ops : 0), us / N,
               (unsigned long) sink);
        check(ops == N, "T6: every load reached the engine's read stage");
    }

    dump_counters(t, "end");
    printf(failures == 0 ? "LOOM APERTURE TEST PASS\n"
                         : "LOOM APERTURE TEST FAIL (%d)\n", failures);
    return failures == 0 ? 0 : 1;
}
