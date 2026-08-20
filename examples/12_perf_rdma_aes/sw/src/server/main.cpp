/**
 * Example 12 - AES-GCM over RDMA: server (fixed reps).
 *
 * Waits for the client's batch to land, then writes the same buffer back. That
 * buffer already went through this node's decrypt bank, so it holds plaintext
 * plus the locally computed tag in each fragment's tag slot; writing it back
 * re-encrypts it, the tag slot being treated as pad. Also owns the hardware
 * counters, being the node whose receive path is under test.
 */

#include <iostream>
#include <iomanip>
#include <cstdlib>

#include <boost/program_options.hpp>

#include <coyote/cThread.hpp>
#include <constants.hpp>

constexpr bool const IS_CLIENT = false;

void run_bench(
    coyote::cThread &coyote_thread, coyote::rdmaSg &sg,
    int *mem, uint transfers, uint n_runs
) {
    for (uint i = 0; i < n_runs; i++) {
        coyote_thread.clearCompleted();
        coyote_thread.connSync(IS_CLIENT);

        while (coyote_thread.checkCompleted(coyote::CoyoteOper::LOCAL_WRITE) != transfers) {}

        for (uint j = 0; j < transfers; j++) {
            coyote_thread.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);
        }
    }
}

int main(int argc, char *argv[]) {
    unsigned int min_size, max_size, n_runs;
    bool crypto;

    boost::program_options::options_description runtime_options("Coyote Example 12: AES-GCM RDMA Server");
    runtime_options.add_options()
        ("crypto,c", boost::program_options::value<bool>(&crypto)->default_value(true), "Enable the AES-GCM banks (0 = bypass, for the baseline); MUST match the client")
        ("runs,r", boost::program_options::value<unsigned int>(&n_runs)->default_value(N_RUNS_DEFAULT), "Number of times to repeat the test")
        ("min_size,x", boost::program_options::value<unsigned int>(&min_size)->default_value(MIN_TRANSFER_SIZE_DEFAULT), "Starting (minimum) transfer size")
        ("max_size,X", boost::program_options::value<unsigned int>(&max_size)->default_value(MAX_TRANSFER_SIZE_DEFAULT), "Ending (maximum) transfer size");
    boost::program_options::variables_map command_line_arguments;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, runtime_options), command_line_arguments);
    boost::program_options::notify(command_line_arguments);

    HEADER("CLI PARAMETERS:");
    std::cout << "AES-GCM: " << (crypto ? "ENABLED" : "BYPASSED (baseline)") << std::endl;
    std::cout << "Number of test runs: " << n_runs << std::endl;
    std::cout << "Transfer size range: " << min_size << " - " << max_size << " B" << std::endl << std::endl;

    coyote::cThread coyote_thread(DEFAULT_VFPGA_ID, getpid());
    int *mem = (int *) coyote_thread.initRDMA(max_size, coyote::DEF_PORT);
    if (!mem) { throw std::runtime_error("Could not allocate memory; exiting..."); }

    // Role first, enable second: the engines sample their IV space when they
    // leave reset, and the enable bit is what releases reset. Server = role 1.
    coyote_thread.connSync(IS_CLIENT);
    coyote_thread.setCSR(CTRL_ROLE_BIT, (uint32_t) AesRegs::CTRL);
    if (crypto) {
        coyote_thread.setCSR(CTRL_ROLE_BIT | CTRL_ENABLE_BIT, (uint32_t) AesRegs::CTRL);
    }
    coyote_thread.setCSR(0, (uint32_t) AesRegs::MEAS_CTRL);
    coyote_thread.setCSR(1, (uint32_t) AesRegs::MEAS_CTRL);
    coyote_thread.connSync(IS_CLIENT);

    HEADER("RDMA WRITE BENCHMARK: SERVER");

    // Frames the PMTU assumption predicts, to check against RX_FRAMES at the end
    uint64_t expected_frames = 0;

    unsigned int curr_size = min_size;
    while (curr_size <= max_size) {
        if (!valid_transfer_size(curr_size)) { curr_size *= 2; continue; }

        coyote::rdmaSg sg = { .len = curr_size };
        run_bench(coyote_thread, sg, mem, N_THROUGHPUT_REPS, n_runs);
        run_bench(coyote_thread, sg, mem, N_LATENCY_REPS, n_runs);

        expected_frames += (uint64_t) n_runs
                         * (N_THROUGHPUT_REPS + N_LATENCY_REPS)
                         * fragments_for(curr_size);

        const unsigned int bad = check_payload(mem, curr_size);
        std::cout << "Size " << std::setw(8) << curr_size
                  << "  payload " << std::setw(8) << payload_bytes(curr_size)
                  << "  decrypt check: " << (bad == 0 ? "ok" : "FAIL")
                  << std::endl;

        curr_size *= 2;
    }

    // ---- hardware view of the receive path ----
    const uint64_t rx_frames = coyote_thread.getCSR((uint32_t) AesRegs::RX_FRAMES);
    const uint64_t tx_frames = coyote_thread.getCSR((uint32_t) AesRegs::TX_FRAMES);
    const uint64_t tag_ok    = coyote_thread.getCSR((uint32_t) AesRegs::TAG_OK);
    const uint64_t status    = coyote_thread.getCSR((uint32_t) AesRegs::STATUS);

    const uint64_t first = coyote_thread.getCSR((uint32_t) AesRegs::FIRST_CYCLE_LO)
                         | (coyote_thread.getCSR((uint32_t) AesRegs::FIRST_CYCLE_HI) << 32);
    const uint64_t last  = coyote_thread.getCSR((uint32_t) AesRegs::LAST_CYCLE_LO)
                         | (coyote_thread.getCSR((uint32_t) AesRegs::LAST_CYCLE_HI) << 32);

    std::cout << std::endl;
    HEADER("HARDWARE COUNTERS (RECEIVE PATH)");
    std::cout << "Fragments decrypted: " << rx_frames << std::endl;
    std::cout << "Fragments encrypted: " << tx_frames << std::endl;
    std::cout << "Tags verified:       " << tag_ok << std::endl;

    /* What framing did the shell actually use?
     *
     * The last 16 bytes of every FRAME are the tag slot, so usable payload
     * depends on how many frames a transfer becomes -- a property of the
     * shell's fragmentation, not of this code. RX_FRAMES counts frames leaving
     * the decrypt bank, so comparing it against the frames the PMTU assumption
     * predicts tells us on the very first run whether the assumption holds.
     * The datapath is correct either way (axis_strip_tail_beat reserves the
     * last 16 bytes of whatever a frame is); only payload accounting depends
     * on this, so a mismatch means the reported Goodput is off, not that the
     * crypto is wrong.
     */
    if (crypto && rx_frames != 0 && expected_frames != 0) {
        if (rx_frames == expected_frames) {
            std::cout << "Framing:             as assumed (" << RDMA_PMTU_BYTES
                      << " B fragments); payload accounting is correct" << std::endl;
        } else {
            const double ratio = (double) rx_frames / (double) expected_frames;
            std::cout << "Framing:             MISMATCH -- " << rx_frames
                      << " frames decrypted, PMTU fragmentation predicts "
                      << expected_frames << " (ratio " << std::fixed
                      << std::setprecision(3) << ratio << ")" << std::endl
                      << "  The tag slot is per FRAME, so Goodput is misreported by"
                      << " this factor." << std::endl
                      << "  If the ratio is ~1 the shell delivers one frame per"
                      << " message: set RDMA_PMTU_BYTES high enough that"
                      << std::endl
                      << "  fragments_for() returns 1, and re-run." << std::endl;
        }
    }

    if (crypto && rx_frames != 0 && tag_ok != rx_frames) {
        std::cout << "WARNING: " << (rx_frames - tag_ok)
                  << " decrypted fragments did not verify" << std::endl;
    }
    if (status & 0x1) {
        std::cout << "WARNING: a decrypt engine is quarantining a frame (tag never verified)"
                  << std::endl;
    }

    if (last > first) {
        const double cycles  = (double) (last - first);
        const double seconds = cycles / USER_CLOCK_FREQ_HZ;
        std::cout << "Span: " << (uint64_t) cycles << " cycles ("
                  << std::fixed << std::setprecision(3) << seconds * 1e3 << " ms)"
                  << std::endl;
        // Spans the whole sweep including barriers, so this is a floor on the
        // receive rate. Arm MEAS_CTRL around one size for a clean figure.
        std::cout << "Span: covers the full sweep incl. barriers (floor, not line rate)"
                  << std::endl;
    }

    coyote_thread.setCSR(0, (uint32_t) AesRegs::CTRL);
    coyote_thread.connSync(IS_CLIENT);
    return EXIT_SUCCESS;
}
