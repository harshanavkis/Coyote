/**
 * ce_selfping - the V80's peer-to-peer self-loop latency on the V80's own
 * clock: loom_ce writes a message and a flag out over PCIe, through the root
 * complex, back into its own window (HBM), and waits for the flag there. No
 * U280 and no host in the loop; one program, loaded once:
 *
 *   loop, per round i:   STAMP t0; COPY msg -> own window; SIGNAL own window
 *                        flag i; WAIT flag >= i (polls HBM); STAMP t1
 *   local, per round i:  the same with the message and flag written into HBM
 *                        directly (the card path, no PCIe)
 *
 * Each sample is t1 - t0 (one trip out and back in, not halved); loop - local
 * leaves the trip itself: the engine's write out over PCIe, the root complex
 * turning it around, the static and shell back to HBM. --size 0 sends only
 * the 8 B flag. The window's counters over the loop runs (writes AW->B at the
 * window and at axi_main) split off the part inside the V80 on the way in.
 * ROUNDS rounds per run, RUNS runs per mode (the first a warm-up, not counted).
 *
 *   ce_selfping [--size B] [--runs R]      (on amy or rose; V80 only)
 */
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <coyote/cThread.hpp>
#include "ce_prog.hpp"
#include "land_v80.hpp"
#include "loom_switch.hpp"

namespace {

constexpr int ROUNDS = 400;                 // 5 entries a round, 2048 at most

uint64_t pattern(uint64_t off, int side) { return 0x5E1F000000000000ULL ^ (off * 0x9E3779B97F4A7C15ULL) ^ uint64_t(side); }

double pct(std::vector<double> v, double p) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    const size_t i = std::min(v.size() - 1, size_t(p / 100.0 * double(v.size())));
    return v[i];
}

}  // namespace

int main(int argc, char **argv) {
    uint64_t size = 64;
    int runs = 6;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string a = argv[i];
        if (a == "--size")      size = strtoull(argv[i + 1], nullptr, 0);
        else if (a == "--runs") runs = atoi(argv[i + 1]);
        else { fprintf(stderr, "usage: %s [--size B] [--runs R]\n", argv[0]); return 2; }
    }
    if (size % 8 || size > (1ULL << 20) || runs < 2) { fprintf(stderr, "--size: a multiple of 8, at most 1 MiB; --runs >= 2\n"); return 2; }
    const uint64_t mlen = std::max<uint64_t>(4096, (size + 4095) & ~4095ULL);

    coyote::cThread v80(0, getpid(), 0, nullptr, "loom_versal_fpga");

    // The window's HBM: message at [0, mlen), flag at mlen; the same window
    // imported into the V80's own MMU is the way out over PCIe and back in
    land_v80::Landing L(v80, mlen + 4096);
    void *sva = loom_switch::reserve_va(mlen + 4096);
    v80.importDmabuf(L.export_fd(), sva);
    uint8_t *self_msg = static_cast<uint8_t *>(sva), *self_flag = self_msg + mlen;
    uint64_t *my_msg = L.buf, *my_flag = L.buf + mlen / 8;

    uint64_t *src = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, mlen}));
    for (uint64_t i = 0; i < mlen / 8; i++) src[i] = pattern(8 * i, 0);
    v80.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{src, mlen});
    // stamps go to host memory (read with no sync); a card SIGNAL at the end
    // of the program waits for them to complete
    volatile uint64_t *st = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, 2 * ROUNDS * 8}));
    uint64_t *drain = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, 4096}));
    v80.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{drain, 4096});
    printf("selfping: %lu B messages, %d rounds a run, %d runs a mode (the first a warm-up); window imported at %p\n",
           (unsigned long) size, ROUNDS, runs, sva);

    int errors = 0;
    std::vector<std::vector<double>> all;         // per mode, the counted samples
    for (int mode = 0; mode < 2; mode++) {        // 0 loop over PCIe, 1 local (card path)
        const bool loop = (mode == 0);
        L.clear();
        ce_prog::Program p(v80);
        for (int i = 0; i < ROUNDS; i++) {
            const uint64_t v = uint64_t(i) + 1;   // + RUN_BASE
            p.add(ce_prog::stamp(const_cast<uint64_t *>(&st[2 * i])));
            if (size) p.add(loop ? ce_prog::copy(src, self_msg, size) : ce_prog::copy(src, my_msg, size, true));
            p.add(loop ? ce_prog::signal(self_flag, v) : ce_prog::signal(my_flag, v, true));
            p.add(ce_prog::wait(my_flag, v));
            p.add(ce_prog::stamp(const_cast<uint64_t *>(&st[2 * i + 1])));
        }
        p.add(ce_prog::signal(drain, 0, true));   // after every stamp landed
        p.load();

        const land_v80::Counters c0 = land_v80::Counters::read(L.win);
        std::vector<double> ns;
        for (int r = 0; r < runs; r++) {
            p.start(uint64_t(r) * ROUNDS);
            if (!p.finish(std::chrono::milliseconds(5000))) {
                printf("FAIL %s run %d: the program did not finish (aborted at entry %lu)\n", loop ? "loop" : "local", r,
                       (unsigned long) v80.getCSR(ce_prog::PC));
                errors++;
                break;
            }
            std::vector<double> run_ns;
            for (int i = 0; i < ROUNDS; i++) run_ns.push_back(double(st[2 * i + 1] - st[2 * i]) * ce_prog::CLK_NS);
            printf("%s %-5s run %d: median %.0f ns, min %.0f, max %.0f\n", r ? "    " : "warm", loop ? "loop" : "local", r,
                   pct(run_ns, 50), pct(run_ns, 0), pct(run_ns, 100));
            if (r) ns.insert(ns.end(), run_ns.begin(), run_ns.end());
        }
        if (loop) land_v80::Counters::read(L.win).print(c0);

        // The last message and flag that landed
        L.pull();
        uint64_t bad = 0;
        for (uint64_t i = 0; i < size / 8; i++) bad += (L.buf[i] != pattern(8 * i, 0));
        const uint64_t want = uint64_t(runs) * ROUNDS;
        printf("%s: message %lu of %lu words wrong, flag %lu (want %lu)\n", loop ? "loop" : "local", (unsigned long) bad,
               (unsigned long) (size / 8), (unsigned long) L.buf[mlen / 8], (unsigned long) want);
        errors += (bad != 0) + (L.buf[mlen / 8] != want);
        if (!ns.empty()) {
            printf("%s %lu B (ns, %zu samples): p0 %.0f  p1 %.0f  p10 %.0f  p50 %.0f  p90 %.0f  p99 %.0f  max %.0f\n",
                   loop ? "loop " : "local", (unsigned long) size, ns.size(), pct(ns, 0), pct(ns, 1), pct(ns, 10),
                   pct(ns, 50), pct(ns, 90), pct(ns, 99), pct(ns, 100));
            all.push_back(ns);
        }
    }
    // The WAIT polls HBM, so a sample ends a poll after its flag landed: the
    // local mode's samples split by one poll; compare like percentiles
    if (all.size() == 2)
        printf("self-loop trip %lu B (loop - local): p0 %.0f ns, p10 %.0f, p50 %.0f\n", (unsigned long) size,
               pct(all[0], 0) - pct(all[1], 0), pct(all[0], 10) - pct(all[1], 10), pct(all[0], 50) - pct(all[1], 50));
    printf(errors ? "SELFPING FAIL\n" : "SELFPING PASS\n");
    return errors ? 1 : 0;
}
