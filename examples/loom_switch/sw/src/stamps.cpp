/**
 * stamps - arm and read the per-packet stamps (stamp_rec, CSR words 224-240)
 * of the U280 on this host; the same words in loom_switch and perf_rdma.
 *
 *   stamps arm          clear the counts, restart the cycle count, record from now on
 *   stamps counts       events per point since the arm, and the cycles since it
 *   stamps dump FILE    every point's events still in its ring (the last DEPTH), oldest
 *                       first, as "point event cycle" lines (event = its number since the
 *                       arm; cycle = aclk cycles since the arm, 4 ns, 32 bits)
 *
 * Points: 0 data in (Loom: first W beat of a uwin burst; perf_rdma: first beat back from
 * the host read), 1 Loom's rdma request posted, 2 first beat to the stack, 3 remote ack,
 * 4 rq_wr, 5 first beat from the stack, 6 landing write posted, 7 first landing beat out,
 * 8 landing write completed. Run arm before and dump after the measured program, never
 * while it runs (each read is an MMIO round trip).
 */
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <unistd.h>

#include <coyote/cThread.hpp>

namespace {
constexpr uint32_t ST_CTL = 224, ST_SEL = 225, ST_DATA = 226, ST_NOW = 227, ST_CNT = 232;
const char *NAMES[] = {"data_in", "post", "to_stack", "ack", "rq_wr", "from_stack", "land_wr", "land_beat", "land_done"};
}

int main(int argc, char **argv) {
    const std::string cmd = argc > 1 ? argv[1] : "";
    if (cmd != "arm" && cmd != "counts" && !(cmd == "dump" && argc > 2)) {
        fprintf(stderr, "usage: %s arm | counts | dump FILE\n", argv[0]);
        return 2;
    }
    coyote::cThread t(0, getpid(), 0, nullptr, "loom_fpga");
    const uint64_t info = t.getCSR(ST_CTL);
    const uint32_t n = uint32_t(info), depth = uint32_t(info >> 32);
    if (n == 0 || n > 16 || depth == 0) {
        fprintf(stderr, "no stamps on this image (word %u reads %#lx)\n", ST_CTL, (unsigned long) info);
        return 1;
    }
    if (cmd == "arm") {
        t.setCSR(1, ST_CTL);
        printf("stamps armed (%u points, %u events each)\n", n, depth);
        return 0;
    }
    uint64_t cnt[16];
    for (uint32_t p = 0; p < n; p++) cnt[p] = t.getCSR(ST_CNT + p);
    const uint64_t now = t.getCSR(ST_NOW);
    if (cmd == "counts") {
        for (uint32_t p = 0; p < n; p++) printf("%u %-10s %lu\n", p, p < 9 ? NAMES[p] : "", (unsigned long) cnt[p]);
        printf("cycles since the arm %lu\n", (unsigned long) now);
        return 0;
    }
    FILE *f = fopen(argv[2], "w");
    if (!f) { perror(argv[2]); return 1; }
    fprintf(f, "# stamps: %u points, ring %u, cycles since the arm %lu; counts", n, depth, (unsigned long) now);
    for (uint32_t p = 0; p < n; p++) fprintf(f, " %lu", (unsigned long) cnt[p]);
    fprintf(f, "\npoint\tevent\tcycle\n");
    for (uint32_t p = 0; p < n; p++) {
        for (uint64_t e = cnt[p] > depth ? cnt[p] - depth : 0; e < cnt[p]; e++) {
            t.setCSR((uint64_t(p) << 16) | (e % depth), ST_SEL);
            fprintf(f, "%u\t%lu\t%lu\n", p, (unsigned long) e, (unsigned long) t.getCSR(ST_DATA));
        }
    }
    fclose(f);
    printf("stamps: wrote %s\n", argv[2]);
    return 0;
}
