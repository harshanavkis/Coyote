/**
 * loom_csr - print loom_switch's receive-side counters on this host's U280
 * (read-only), for diffing around a run: the run-length maxima (since the
 * bitstream was loaded) say whether loom_rx's stalls are short or long.
 *
 * Usage: loom_csr
 */
#include <unistd.h>

#include <cstdio>

#include <coyote/cThread.hpp>
#include "loom_switch.hpp"

int main() {
    coyote::cThread u280(0, getpid(), 0, nullptr, "coyote_fpga");
    const struct { uint32_t w; const char *name; } regs[] = {
        {48, "cycles"}, {36, "rx fwd (writes done)"}, {47, "rx req (rq_wr taken)"}, {41, "rx drop (bad header)"},
        {26, "rx orphan beats"}, {42, "rx move"}, {43, "rx starve"}, {44, "rx stall (host write not ready)"},
        {24, "rx stall MAX run"}, {14, "rx fifo full"}, {15, "rx fifo full MAX run"}, {30, "rx bp"}, {31, "rx bp MAX run"},
        {72, "wr wait local (sq_wr not ready)"}, {73, "wr wait rdma (sq_wr not ready)"},
        {74, "wr blocked by ingress"}, {75, "wr blocked by rx"}, {66, "tx ctl (ack window)"}, {68, "tx acks"},
        {81, "host out: beats moved"}, {82, "host out: beat waited on DMA"}, {83, "host out: DMA wait MAX run"},
        {84, "host out: request waited on DMA"}};
    for (auto &r : regs) printf("%-34s %lu\n", r.name, (unsigned long) loom_switch::csr_read(u280, r.w));
    return 0;
}
