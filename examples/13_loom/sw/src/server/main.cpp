/**
 * Loom server (destination node).
 * Passive: the RoCE stack raises rq_wr for each inbound RDMA WRITE and the
 * router lands the payload on host stream 1. Verifies the buffer and prints
 * the destination-side counters.
 */

#include <iostream>
#include <cstring>
#include <boost/program_options.hpp>
#include <coyote/cThread.hpp>

#include "loom_regs.hpp"

using namespace loom;
constexpr int DEFAULT_VFPGA_ID = 0;

int main(int argc, char* argv[]) {
    uint32_t rdma_size = 4096;
    boost::program_options::options_description opts("Loom server");
    opts.add_options()("size,z", boost::program_options::value<uint32_t>(&rdma_size), "RDMA transfer size");
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opts), vm);
    boost::program_options::notify(vm);

    coyote::cThread thr(DEFAULT_VFPGA_ID, getpid());

    char* r = (char*) thr.initRDMA(rdma_size, coyote::DEF_PORT);
    if (!r) { std::cerr << "initRDMA failed\n"; return 1; }

    thr.setCSR(1, ROLE_REG);                    // destination node
    thr.setCSR(thr.getCtid(), COYOTE_PID_REG);
    clear_ranges(thr);
    clear_counters(thr);

    std::memset(r, 0, rdma_size);
    std::cout << "server ready, waiting for RDMA WRITE (" << rdma_size << " B)\n";

    thr.connSync(false);                        // client fills + sends
    thr.connSync(false);                        // client done

    // Region outside the offset probe must hold the streamed pattern.
    bool ok = true;
    for (uint32_t i = 0; i < rdma_size; i++) {
        const bool in_probe = (rdma_size >= XLAT_OFF + XLAT_LEN)
                              && (i >= XLAT_OFF) && (i < XLAT_OFF + XLAT_LEN);
        if (in_probe) continue;
        if (r[i] != (char)((i * 11) & 0xFF)) {
            std::cout << "  mismatch at " << i << "\n"; ok = false; break;
        }
    }
    std::cout << (ok ? "PASS" : "FAIL") << ": bulk transfer landed\n";

    // The offset transfer must land at peer_base+XLAT_OFF. If translation were
    // a fixed base, the marker would have overwritten offset 0 instead.
    if (rdma_size >= XLAT_OFF + XLAT_LEN) {
        bool xok = true;
        for (uint32_t i = 0; i < XLAT_LEN; i++)
            if ((uint8_t)r[XLAT_OFF + i] != XLAT_MARK) { xok = false; break; }
        // If translation were a fixed base the marker would sit at offset 0.
        // (Checking != MARK, not == 0: memset also leaves 0 there.)
        bool base_intact = ((uint8_t)r[0] != XLAT_MARK);
        std::cout << (xok ? "PASS" : "FAIL") << ": address translation (marker at +"
                  << XLAT_OFF << ")\n";
        std::cout << (base_intact ? "PASS" : "FAIL")
                  << ": offset 0 untouched (translation is not a fixed base)\n";
        ok = ok && xok && base_intact;
    }

    print_counters(thr, "DESTINATION side (push)");

    // Third barrier: the client now pulls this buffer back with an RDMA READ.
    // Serving that is pure hardware (rq_rd -> local read -> rrsp_send); this
    // process only has to stay alive and keep the memory mapped.
    thr.connSync(false);
    std::cout << "serving pull requests...\n";
    sleep(2);
    print_counters(thr, "DESTINATION side (after pull)");
    return ok ? 0 : 2;
}
