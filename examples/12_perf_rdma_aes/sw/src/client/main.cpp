/**
 * Example 12 - AES-GCM over RDMA: client (fixed reps).
 *
 * 09_perf_rdma's WRITE benchmark, except the payload now crosses two AES-GCM
 * banks each way: this node's encrypt on the way out, the peer's decrypt on
 * the way in. --crypto 0 bypasses both on the same bitstream, so the AES cost
 * is a straight delta between two runs of this binary.
 *
 * Wire = bytes on the link, including the per-fragment tag slot; Goodput =
 * usable payload. For numbers you intend to quote, prefer client_time.
 */

#include <iostream>
#include <iomanip>
#include <cstdlib>

#include <boost/program_options.hpp>

#include <coyote/cBench.hpp>
#include <coyote/cThread.hpp>
#include <constants.hpp>

constexpr bool const IS_CLIENT = true;

double run_bench(
    coyote::cThread &coyote_thread, coyote::rdmaSg &sg,
    int *mem, uint transfers, uint n_runs
) {
    auto prep_fn = [&]() {
        fill_payload(mem, sg.len);
        coyote_thread.clearCompleted();
        coyote_thread.connSync(IS_CLIENT);
    };

    auto bench_fn = [&]() {
        for (uint i = 0; i < transfers; i++) {
            coyote_thread.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);
        }
        while (coyote_thread.checkCompleted(coyote::CoyoteOper::LOCAL_WRITE) != transfers) {}
    };

    coyote::cBench bench(n_runs, 0);
    bench.execute(bench_fn, prep_fn);

    // Halve: the message travelled client -> server -> client
    return bench.getAvg() / 2.0;
}

int main(int argc, char *argv[]) {
    std::string server_ip;
    unsigned int min_size, max_size, n_runs;
    bool crypto;

    boost::program_options::options_description runtime_options("Coyote Example 12: AES-GCM RDMA Client");
    runtime_options.add_options()
        ("ip_address,i", boost::program_options::value<std::string>(&server_ip), "Server's IP address")
        ("crypto,c", boost::program_options::value<bool>(&crypto)->default_value(true), "Enable the AES-GCM banks (0 = bypass, for the baseline)")
        ("runs,r", boost::program_options::value<unsigned int>(&n_runs)->default_value(N_RUNS_DEFAULT), "Number of times to repeat the test")
        ("min_size,x", boost::program_options::value<unsigned int>(&min_size)->default_value(MIN_TRANSFER_SIZE_DEFAULT), "Starting (minimum) transfer size")
        ("max_size,X", boost::program_options::value<unsigned int>(&max_size)->default_value(MAX_TRANSFER_SIZE_DEFAULT), "Ending (maximum) transfer size");
    boost::program_options::variables_map command_line_arguments;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, runtime_options), command_line_arguments);
    boost::program_options::notify(command_line_arguments);

    HEADER("CLI PARAMETERS:");
    std::cout << "Server's TCP address: " << server_ip << std::endl;
    std::cout << "AES-GCM: " << (crypto ? "ENABLED" : "BYPASSED (baseline)") << std::endl;
    std::cout << "Number of test runs: " << n_runs << std::endl;
    std::cout << "Transfer size range: " << min_size << " - " << max_size << " B" << std::endl << std::endl;

    coyote::cThread coyote_thread(DEFAULT_VFPGA_ID, getpid(), 0);
    int *mem = (int *) coyote_thread.initRDMA(max_size, coyote::DEF_PORT, server_ip.c_str());
    if (!mem) { throw std::runtime_error("Could not allocate memory; exiting..."); }

    // Role first, enable second: the engines latch their IV space when they
    // leave reset, and the enable bit releases reset. Client is role 0.
    coyote_thread.connSync(IS_CLIENT);
    coyote_thread.setCSR(0, (uint32_t) AesRegs::CTRL);
    if (crypto) {
        coyote_thread.setCSR(CTRL_ENABLE_BIT, (uint32_t) AesRegs::CTRL);
    }
    coyote_thread.setCSR(0, (uint32_t) AesRegs::MEAS_CTRL);
    coyote_thread.setCSR(1, (uint32_t) AesRegs::MEAS_CTRL);
    coyote_thread.connSync(IS_CLIENT);

    HEADER("RDMA WRITE BENCHMARK: CLIENT");
    std::cout << std::setw(10) << "Size"
              << std::setw(12) << "Payload"
              << std::setw(14) << "Wire(Gb/s)"
              << std::setw(15) << "Goodput(Gb/s)"
              << std::setw(12) << "Lat(us)"
              << std::setw(10) << "Check" << std::endl;
    std::cout << std::string(73, '-') << std::endl;

    unsigned int curr_size = min_size;
    while (curr_size <= max_size) {
        if (!valid_transfer_size(curr_size)) {
            std::cout << std::setw(10) << curr_size << "   skipped (not a legal tag-slot size)" << std::endl;
            curr_size *= 2;
            continue;
        }

        coyote::rdmaSg sg = { .len = curr_size };

        double tput_ns = run_bench(coyote_thread, sg, mem, N_THROUGHPUT_REPS, n_runs);
        const unsigned int bad = check_payload(mem, curr_size);

        const double total_wire    = (double) N_THROUGHPUT_REPS * (double) curr_size;
        const double total_payload = (double) N_THROUGHPUT_REPS * (double) payload_bytes(curr_size);
        const double wire_gbps     = (total_wire * 8.0) / tput_ns;       // bytes*8/ns == Gb/s
        const double good_gbps     = (total_payload * 8.0) / tput_ns;

        double lat_ns = run_bench(coyote_thread, sg, mem, N_LATENCY_REPS, n_runs);

        std::cout << std::setw(10) << curr_size
                  << std::setw(12) << payload_bytes(curr_size)
                  << std::setw(14) << std::fixed << std::setprecision(2) << wire_gbps
                  << std::setw(15) << std::fixed << std::setprecision(2) << good_gbps
                  << std::setw(12) << std::fixed << std::setprecision(2) << lat_ns / 1e3
                  << std::setw(10) << (bad == 0 ? "ok" : "FAIL")
                  << std::endl;

        if (bad != 0) {
            std::cout << "         ^ " << bad << " payload words differ after the round trip"
                      << std::endl;
        }

        curr_size *= 2;
    }

    std::cout << std::endl;
    std::cout << "Local decrypt bank: verified tags = "
              << coyote_thread.getCSR((uint32_t) AesRegs::TAG_OK)
              << ", frames in = " << coyote_thread.getCSR((uint32_t) AesRegs::RX_FRAMES)
              << ", frames out = " << coyote_thread.getCSR((uint32_t) AesRegs::TX_FRAMES)
              << std::endl;
    if (coyote_thread.getCSR((uint32_t) AesRegs::STATUS) & 0x1) {
        std::cout << "WARNING: a decrypt engine is quarantining a frame (tag never verified)"
                  << std::endl;
    }

    // Leave the banks disabled so a following run starts from a clean reset
    coyote_thread.setCSR(0, (uint32_t) AesRegs::CTRL);
    coyote_thread.connSync(IS_CLIENT);
    return EXIT_SUCCESS;
}
