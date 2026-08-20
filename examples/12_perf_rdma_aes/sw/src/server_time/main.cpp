/**
 * Example 12 - AES-GCM over RDMA: server (time-based).
 *
 * Beat-for-beat mirror of client_time; round counts arrive via readAck. Ported
 * from the retired AES echo example's server_og_time
 * (Coyotev2/Coyote/examples/13_aes_gcm_rdma), with the crypto CSRs and tag-slot
 * payload check added.
 */

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cstdlib>

#include <boost/program_options.hpp>

#include <coyote/cThread.hpp>
#include <constants.hpp>

constexpr bool const IS_CLIENT = false;

class cThreadBench : public coyote::cThread {
public:
    using cThread::cThread;
    using cThread::sendAck;
    using cThread::readAck;
};

/// Mirror of the client's run_throughput_rounds.
void server_throughput_rounds(cThreadBench &ct, coyote::rdmaSg &sg,
                              uint32_t n_rounds, uint32_t batch_size) {
    uint32_t cumul = 0;
    for (uint32_t r = 0; r < n_rounds; r++) {
        cumul += batch_size;
        while (ct.checkCompleted(coyote::CoyoteOper::LOCAL_WRITE) < cumul) {}

        for (uint32_t i = 0; i < batch_size; i++) {
            ct.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);
        }

        if ((r + 1) % SYNC_INTERVAL == 0 && r + 1 < n_rounds) {
            ct.connSync(IS_CLIENT);
            ct.clearCompleted();
            ct.connSync(IS_CLIENT);
            cumul = 0;
        }
    }
}

/// Mirror of the client's run_latency_rounds, cBench prep_fn included.
void server_latency_rounds(cThreadBench &ct, coyote::rdmaSg &sg, uint32_t n_pings) {
    for (uint32_t p = 0; p < n_pings; p++) {
        ct.clearCompleted();
        ct.connSync(IS_CLIENT);
        while (ct.checkCompleted(coyote::CoyoteOper::LOCAL_WRITE) != 1) {}
        ct.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);
    }
}

static void barrier_reset(cThreadBench &ct) {
    ct.connSync(IS_CLIENT);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ct.clearCompleted();
    ct.connSync(IS_CLIENT);
}

int main(int argc, char *argv[]) {
    unsigned int n_runs, min_size, max_size;
    std::string mode_str;
    bool crypto;

    boost::program_options::options_description opts("Coyote Example 12: AES-GCM RDMA Server (time-based)");
    opts.add_options()
        ("crypto,c", boost::program_options::value<bool>(&crypto)->default_value(true), "Enable the AES-GCM banks (0 = bypass, for the baseline); MUST match the client")
        ("mode,m", boost::program_options::value<std::string>(&mode_str)->default_value("both"), "Mode: latency, throughput, or both (must match the client)")
        ("runs,r", boost::program_options::value<unsigned int>(&n_runs)->default_value(N_RUNS_DEFAULT), "Measurement runs per size")
        ("min_size,x", boost::program_options::value<unsigned int>(&min_size)->default_value(MIN_TRANSFER_SIZE_DEFAULT), "Starting (minimum) transfer size")
        ("max_size,X", boost::program_options::value<unsigned int>(&max_size)->default_value(MAX_TRANSFER_SIZE_DEFAULT), "Ending (maximum) transfer size");
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opts), vm);
    boost::program_options::notify(vm);

    const bool run_throughput = (mode_str == "throughput" || mode_str == "both");
    const bool run_latency    = (mode_str == "latency"    || mode_str == "both");
    if (!run_throughput && !run_latency) {
        std::cerr << "Error: --mode must be 'latency', 'throughput', or 'both'" << std::endl;
        return EXIT_FAILURE;
    }

    HEADER("CLI PARAMETERS:");
    std::cout << "Mode: " << mode_str << std::endl;
    std::cout << "AES-GCM: " << (crypto ? "ENABLED" : "BYPASSED (baseline)") << std::endl;
    std::cout << "Number of runs: " << n_runs << std::endl;
    std::cout << "Transfer size range: " << min_size << " - " << max_size << " B" << std::endl << std::endl;

    cThreadBench ct(DEFAULT_VFPGA_ID, getpid());
    int *mem = (int *) ct.initRDMA(max_size, coyote::DEF_PORT);
    if (!mem) { throw std::runtime_error("Could not allocate memory; exiting..."); }

    // Role first, enable second (see client_time). Server is role 1.
    ct.connSync(IS_CLIENT);
    ct.setCSR(CTRL_ROLE_BIT, (uint32_t) AesRegs::CTRL);
    if (crypto) { ct.setCSR(CTRL_ROLE_BIT | CTRL_ENABLE_BIT, (uint32_t) AesRegs::CTRL); }
    ct.setCSR(0, (uint32_t) AesRegs::MEAS_CTRL);
    ct.setCSR(1, (uint32_t) AesRegs::MEAS_CTRL);
    ct.connSync(IS_CLIENT);

    // Frames the PMTU assumption predicts, to check against RX_FRAMES at the end
    uint64_t expected_frames = 0;

    unsigned int curr_size;

    // ---- THROUGHPUT ----
    if (run_throughput) {
        HEADER("RDMA WRITE THROUGHPUT: SERVER (TIME-BASED)");
        curr_size = min_size;
        while (curr_size <= max_size) {
            if (!valid_transfer_size(curr_size)) { curr_size *= 2; continue; }

            coyote::rdmaSg sg = { .len = curr_size };
            const uint32_t batch = compute_batch(curr_size);

            barrier_reset(ct);

            // Calibration burst
            uint32_t rounds = ct.readAck();
            server_throughput_rounds(ct, sg, rounds, batch);
            expected_frames += (uint64_t) rounds * batch * fragments_for(curr_size);

            barrier_reset(ct);

            for (unsigned int run = 0; run < n_runs; run++) {
                rounds = ct.readAck();
                server_throughput_rounds(ct, sg, rounds, batch);
                expected_frames += (uint64_t) rounds * batch * fragments_for(curr_size);
                barrier_reset(ct);
            }

            const unsigned int bad = check_payload(mem, curr_size);
            std::cout << "Size " << std::setw(8) << curr_size
                      << "  payload " << std::setw(8) << payload_bytes(curr_size)
                      << "  decrypt check: " << (bad == 0 ? "ok" : "FAIL") << std::endl;

            curr_size *= 2;
        }
    }

    ct.connSync(IS_CLIENT);

    // ---- LATENCY ----
    if (run_latency) {
        HEADER("RDMA WRITE LATENCY: SERVER (TIME-BASED)");
        curr_size = min_size;
        while (curr_size <= max_size) {
            if (!valid_transfer_size(curr_size)) { curr_size *= 2; continue; }

            coyote::rdmaSg sg = { .len = curr_size };

            barrier_reset(ct);
            uint32_t pings = ct.readAck();
            server_latency_rounds(ct, sg, pings);
            expected_frames += (uint64_t) pings * fragments_for(curr_size);
            barrier_reset(ct);

            for (unsigned int run = 0; run < n_runs; run++) {
                pings = ct.readAck();
                server_latency_rounds(ct, sg, pings);
                expected_frames += (uint64_t) pings * fragments_for(curr_size);
                barrier_reset(ct);
            }

            std::cout << "Size " << std::setw(8) << curr_size << ": done" << std::endl;
            curr_size *= 2;
        }
    }

    // ---- hardware view of the receive path ----
    const uint64_t rx_frames = ct.getCSR((uint32_t) AesRegs::RX_FRAMES);
    const uint64_t tx_frames = ct.getCSR((uint32_t) AesRegs::TX_FRAMES);
    const uint64_t tag_ok    = ct.getCSR((uint32_t) AesRegs::TAG_OK);
    const uint64_t status    = ct.getCSR((uint32_t) AesRegs::STATUS);

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
        std::cout << "WARNING: a decrypt engine is quarantining a frame (tag never verified)" << std::endl;
    }

    ct.setCSR(0, (uint32_t) AesRegs::CTRL);
    ct.connSync(IS_CLIENT);
    return EXIT_SUCCESS;
}
