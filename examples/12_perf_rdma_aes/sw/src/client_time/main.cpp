/**
 * Example 12 - AES-GCM over RDMA: client (time-based). Prefer this over
 * src/client for numbers you intend to quote.
 *
 * Harness ported from the retired AES echo example's client_og_time
 * (Coyotev2/Coyote/examples/13_aes_gcm_rdma): adaptive batching,
 * calibration against a target duration, periodic barriers so the completion
 * counters cannot overflow, round counts exchanged out of band via sendAck.
 * Every barrier here has a matching one in server_time -- change the cadence
 * on one side and you must change it on the other.
 *
 * The server serialises (waits for the whole batch, then echoes), so elapsed
 * covers two legs and the one-way figure is elapsed/2.
 */

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cstdlib>

#include <boost/program_options.hpp>

#include <coyote/cBench.hpp>
#include <coyote/cThread.hpp>
#include <constants.hpp>

constexpr bool const IS_CLIENT = true;

// Subclass to expose sendAck/readAck for the out-of-band round-count exchange
class cThreadBench : public coyote::cThread {
public:
    using cThread::cThread;
    using cThread::sendAck;
    using cThread::readAck;
};

/// Send `batch_size` writes, wait for that many echoes to land.
double run_throughput_rounds(cThreadBench &ct, coyote::rdmaSg &sg,
                             uint32_t n_rounds, uint32_t batch_size) {
    auto t_start = std::chrono::high_resolution_clock::now();

    uint32_t cumul = 0;
    for (uint32_t r = 0; r < n_rounds; r++) {
        for (uint32_t i = 0; i < batch_size; i++) {
            ct.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);
        }

        cumul += batch_size;
        while (ct.checkCompleted(coyote::CoyoteOper::LOCAL_WRITE) < cumul) {}

        // Cadence must match the server exactly
        if ((r + 1) % SYNC_INTERVAL == 0 && r + 1 < n_rounds) {
            ct.connSync(IS_CLIENT);
            ct.clearCompleted();
            ct.connSync(IS_CLIENT);
            cumul = 0;
        }
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double>(t_end - t_start).count();
}

/// Ping-pong latency; prep_fn is outside the timer. Returns ns per round trip.
double run_latency_rounds(cThreadBench &ct, coyote::rdmaSg &sg, uint32_t n_pings) {
    auto prep_fn = [&]() {
        ct.clearCompleted();
        ct.connSync(IS_CLIENT);
    };
    auto bench_fn = [&]() {
        ct.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);
        while (ct.checkCompleted(coyote::CoyoteOper::LOCAL_WRITE) != 1) {}
    };

    coyote::cBench bench(n_pings, 0);
    bench.execute(bench_fn, prep_fn);
    return bench.getAvg();
}

/// The reset dance between phases; the server mirrors it beat for beat.
static void barrier_reset(cThreadBench &ct) {
    ct.connSync(IS_CLIENT);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ct.clearCompleted();
    ct.connSync(IS_CLIENT);
}

int main(int argc, char *argv[]) {
    std::string server_ip, mode_str;
    unsigned int n_runs, target_time, min_size, max_size;
    bool crypto;

    boost::program_options::options_description opts("Coyote Example 12: AES-GCM RDMA Client (time-based)");
    opts.add_options()
        ("ip_address,i", boost::program_options::value<std::string>(&server_ip), "Server's IP address")
        ("crypto,c", boost::program_options::value<bool>(&crypto)->default_value(true), "Enable the AES-GCM banks (0 = bypass, for the baseline); MUST match the server")
        ("mode,m", boost::program_options::value<std::string>(&mode_str)->default_value("both"), "Mode: latency, throughput, or both (must match the server)")
        ("runs,r", boost::program_options::value<unsigned int>(&n_runs)->default_value(N_RUNS_DEFAULT), "Measurement runs per size")
        ("target_time,t", boost::program_options::value<unsigned int>(&target_time)->default_value(TARGET_TIME_DEFAULT), "Target time per measurement (seconds)")
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
    std::cout << "Server's TCP address: " << server_ip << std::endl;
    std::cout << "AES-GCM: " << (crypto ? "ENABLED" : "BYPASSED (baseline)") << std::endl;
    std::cout << "Number of runs: " << n_runs << std::endl;
    std::cout << "Target time: " << target_time << " s" << std::endl;
    std::cout << "Transfer size range: " << min_size << " - " << max_size << " B" << std::endl << std::endl;

    cThreadBench ct(DEFAULT_VFPGA_ID, getpid(), 0);
    int *mem = (int *) ct.initRDMA(max_size, coyote::DEF_PORT, server_ip.c_str());
    if (!mem) { throw std::runtime_error("Could not allocate memory; exiting..."); }

    // Role first, enable second: the engines latch their IV space when they
    // leave reset, and the enable bit releases reset. Client is role 0.
    ct.connSync(IS_CLIENT);
    ct.setCSR(0, (uint32_t) AesRegs::CTRL);
    if (crypto) { ct.setCSR(CTRL_ENABLE_BIT, (uint32_t) AesRegs::CTRL); }
    ct.setCSR(0, (uint32_t) AesRegs::MEAS_CTRL);
    ct.setCSR(1, (uint32_t) AesRegs::MEAS_CTRL);
    ct.connSync(IS_CLIENT);

    unsigned int curr_size;

    // ---- THROUGHPUT ----
    if (run_throughput) {
        HEADER("RDMA WRITE THROUGHPUT: CLIENT (TIME-BASED)");
        std::cout << std::setw(10) << "Size"
                  << std::setw(10) << "Payload"
                  << std::setw(9)  << "Batch"
                  << std::setw(10) << "Rounds"
                  << std::setw(13) << "Wire(Gb/s)"
                  << std::setw(15) << "Goodput(Gb/s)"
                  << std::setw(8)  << "Check" << std::endl;
        std::cout << std::string(75, '-') << std::endl;

        curr_size = min_size;
        while (curr_size <= max_size) {
            if (!valid_transfer_size(curr_size)) {
                std::cout << std::setw(10) << curr_size << "   skipped (not a legal tag-slot size)" << std::endl;
                curr_size *= 2;
                continue;
            }

            const uint32_t batch = compute_batch(curr_size);
            coyote::rdmaSg sg = { .len = curr_size };
            fill_payload(mem, curr_size);

            barrier_reset(ct);

            // Calibrate the per-round cost
            const uint32_t cal_rounds = 5;
            ct.sendAck(cal_rounds);
            const double cal_time = run_throughput_rounds(ct, sg, cal_rounds, batch);
            const double time_per_round = cal_time / cal_rounds;

            barrier_reset(ct);

            const uint32_t n_rounds = std::max(10u, (uint32_t) (target_time / time_per_round));

            double total_wire_gbps = 0.0, total_good_gbps = 0.0;
            for (unsigned int run = 0; run < n_runs; run++) {
                ct.sendAck(n_rounds);
                const double elapsed = run_throughput_rounds(ct, sg, n_rounds, batch);

                const uint64_t total_ops = (uint64_t) n_rounds * batch;
                const double one_way = elapsed / 2.0;   // serialised send + echo legs
                total_wire_gbps += ((double) total_ops * (double) curr_size * 8.0) / one_way / 1e9;
                total_good_gbps += ((double) total_ops * (double) payload_bytes(curr_size) * 8.0) / one_way / 1e9;

                barrier_reset(ct);
            }

            const unsigned int bad = check_payload(mem, curr_size);

            std::cout << std::setw(10) << curr_size
                      << std::setw(10) << payload_bytes(curr_size)
                      << std::setw(9)  << batch
                      << std::setw(10) << n_rounds
                      << std::setw(13) << std::fixed << std::setprecision(2) << total_wire_gbps / n_runs
                      << std::setw(15) << std::fixed << std::setprecision(2) << total_good_gbps / n_runs
                      << std::setw(8)  << (bad == 0 ? "ok" : "FAIL") << std::endl;

            curr_size *= 2;
        }
    }

    ct.connSync(IS_CLIENT);

    // ---- LATENCY ----
    if (run_latency) {
        HEADER("RDMA WRITE LATENCY: CLIENT (TIME-BASED)");
        std::cout << std::setw(10) << "Size"
                  << std::setw(10) << "Payload"
                  << std::setw(12) << "Pings"
                  << std::setw(12) << "RTT(us)" << std::endl;
        std::cout << std::string(44, '-') << std::endl;

        curr_size = min_size;
        while (curr_size <= max_size) {
            if (!valid_transfer_size(curr_size)) { curr_size *= 2; continue; }

            coyote::rdmaSg sg = { .len = curr_size };
            fill_payload(mem, curr_size);

            barrier_reset(ct);

            const uint32_t cal_pings = 10;
            ct.sendAck(cal_pings);
            const double time_per_ping = run_latency_rounds(ct, sg, cal_pings) * 1e-9;

            barrier_reset(ct);

            const uint32_t n_pings = std::max(100u, (uint32_t) (target_time / time_per_ping));

            double total_lat_us = 0.0;
            for (unsigned int run = 0; run < n_runs; run++) {
                ct.sendAck(n_pings);
                total_lat_us += run_latency_rounds(ct, sg, n_pings) / 1e3;
                barrier_reset(ct);
            }

            std::cout << std::setw(10) << curr_size
                      << std::setw(10) << payload_bytes(curr_size)
                      << std::setw(12) << n_pings
                      << std::setw(12) << std::fixed << std::setprecision(2) << total_lat_us / n_runs
                      << std::endl;

            curr_size *= 2;
        }
    }

    std::cout << std::endl;
    std::cout << "Local banks: verified tags = " << ct.getCSR((uint32_t) AesRegs::TAG_OK)
              << ", decrypted = " << ct.getCSR((uint32_t) AesRegs::RX_FRAMES)
              << ", encrypted = " << ct.getCSR((uint32_t) AesRegs::TX_FRAMES) << std::endl;
    if (ct.getCSR((uint32_t) AesRegs::STATUS) & 0x1) {
        std::cout << "WARNING: a decrypt engine is quarantining a frame (tag never verified)" << std::endl;
    }

    ct.setCSR(0, (uint32_t) AesRegs::CTRL);
    ct.connSync(IS_CLIENT);
    return EXIT_SUCCESS;
}
