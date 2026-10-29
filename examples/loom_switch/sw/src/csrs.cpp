/*
 * csrs - print the Loom switch's counter words of the U280 on this host (2026-10-28: where both-way traffic waits).
 *
 *   csrs [WORD ...]     one "word name value" line per word; without words the transmit / receive / ingress counters
 *
 * Read before and after a measured program, never while it runs (each read is an MMIO round trip); the counters run
 * since the card's setup, so the difference of two readings is what the program did.
 */
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>
#include <unistd.h>

#include <coyote/cThread.hpp>

#include "loom_switch.hpp"

using namespace loom_switch;

int main(int argc, char **argv) {
    const std::vector<std::pair<uint32_t, const char *>> dflt = {
        {CYC, "cycles"}, {TX_CTL, "tx_window"}, {TX_ACKS, "tx_acks"}, {TX_WINFULL, "tx_winfull"},
        {TX_REQWAIT, "tx_reqwait"}, {WR_WAIT_LOCAL, "wr_wait_local"}, {WR_WAIT_RDMA, "wr_wait_rdma"},
        {ING_BURSTS, "ing_bursts"}, {ING_DROPS, "ing_drops"}, {ING_PKT_RDMA, "ing_pkt_rdma"}, {ING_STORES, "ing_stores"},
        {RX_FIFO_FULL, "rx_fifo_full"}, {RX_BP, "rx_bp"}, {RX_FWD, "rx_fwd"}, {RX_DROP, "rx_drop"},
        {RX_MOVE, "rx_move"}, {RX_STARVE, "rx_starve"}, {RX_STALL, "rx_stall"}};
    std::vector<std::pair<uint32_t, const char *>> w;
    for (int i = 1; i < argc; i++) w.push_back({uint32_t(std::strtoul(argv[i], nullptr, 0)), ""});
    if (w.empty()) w = dflt;
    coyote::cThread t(0, getpid(), 0, nullptr, "loom_fpga");
    for (const auto &[word, name] : w)
        printf("%u %s %lu\n", word, name, (unsigned long) t.getCSR(word));
    return 0;
}
