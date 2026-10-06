/**
 * loom_csr - print loom_switch's receive-side counters on this host's U280
 * (read-only), for diffing around a run: the run-length maxima (since the
 * bitstream was loaded) say whether loom_rx's stalls are short or long.
 *
 * Usage: loom_csr            (the receive-side list)
 *        loom_csr WORD...    (those CSR words, raw)
 */
#include <unistd.h>

#include <cstdio>
#include <cstdlib>

#include <coyote/cThread.hpp>
#include "loom_switch.hpp"

int main(int argc, char **argv) {
    coyote::cThread u280(0, getpid(), 0, nullptr, "coyote_fpga");
    if (argc > 1) {
        for (int i = 1; i < argc; i++) {
            const uint32_t w = uint32_t(strtoul(argv[i], nullptr, 0));
            printf("word %u: %lu\n", w, (unsigned long) loom_switch::csr_read(u280, w));
        }
        return 0;
    }
    const struct { uint32_t w; const char *name; } regs[] = {
        {48, "cycles"}, {36, "rx fwd (writes done)"}, {47, "rx req (rq_wr taken)"}, {41, "rx drop (bad header)"},
        {26, "rx orphan beats"}, {42, "rx move"}, {43, "rx starve"}, {44, "rx stall (host write not ready)"},
        {24, "rx stall MAX run"}, {14, "rx fifo full"}, {15, "rx fifo full MAX run"}, {30, "rx bp"}, {31, "rx bp MAX run"},
        {72, "wr wait local (sq_wr not ready)"}, {73, "wr wait rdma (sq_wr not ready)"},
        {74, "wr blocked by ingress"}, {75, "wr blocked by rx"}, {66, "tx ctl (ack window)"}, {68, "tx acks"},
        {81, "host out: beats moved"}, {82, "host out: beat waited on DMA"}, {83, "host out: DMA wait MAX run"},
        {84, "host out: request waited on DMA"}, {41, "rx drop"},
        // loom_rx: self-describing packets
        {112, "rx packets landed"}, {113, "rx stores landed"}, {114, "rx packets dropped"},
        {115, "rx write waited on sq_wr"}, {147, "rx write wait on sq_wr MAX run"},
        {116, "rx at outstanding limit"}, {117, "rx beat waited for its write"},
        {149, "rx beat wait for write MAX run"}, {118, "rx rq_wr lost (must be 0)"},
        // loom_ingress
        {119, "ingress full rdma packets"}, {120, "ingress partial: idle timer"},
        {121, "ingress partial: cut by a write"},
        // the shell's host write path, user region to DMA engine
        {122, "shell: DMA write reqs issued"}, {123, "shell: req waited to enter MMU"},
        {155, "shell: MMU entry wait MAX run"}, {124, "shell: reqs taken by MMU"},
        {125, "shell: data waited on MMU order"}, {157, "shell: data wait MAX run"},
        {126, "shell: page-fault irqs"}, {127, "shell: writebacks"},
        {128, "shell: writeback waited"}, {160, "shell: writeback wait MAX run"},
        {129, "shell: rx req waited for data"}, {161, "shell: rx req data wait MAX run"},
        {130, "shell: rx req waited downstream"}, {162, "shell: rx req downstream MAX run"},
        // the shell MMU's write FSM (region 0)
        {131, "mmu: waited for a completion"}, {163, "mmu: completion wait MAX run"},
        {132, "mmu: waited on DMA req port"}, {164, "mmu: DMA port wait MAX run"},
        {133, "mmu: waited for TLB mutex"}, {165, "mmu: mutex wait MAX run"},
        {134, "mmu: miss/invalidate/locked"}, {166, "mmu: miss/inv MAX run"},
        {135, "mmu: DMA completions"}};
    for (auto &r : regs) printf("%-34s %lu\n", r.name, (unsigned long) loom_switch::csr_read(u280, r.w));
    return 0;
}
