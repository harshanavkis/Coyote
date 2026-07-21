/**
 * Jigsaw Host Controller — Software
 *
 * Sets up an RDMA connection to the device-side FPGA, configures the
 * host-controller AXI-Lite registers, and exercises the jigsaw protocol
 * (MMIO read/write and DMA) over the RDMA link.
 *
 * The --ip_address flag is the TCP/IP address of the device node used
 * for the out-of-band QP exchange.  The actual RoCE IP is read from the
 * Coyote driver (set at insmod time) and exchanged automatically.
 *
 * Usage:
 *   ./test -i <device_oob_ip>
 */

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>

#include <boost/program_options.hpp>

#include <coyote/cThread.hpp>

#include "shmem.hpp"

// SIGUSR1 → dump host_controller's 24 local debug counters. Lets us snapshot
// pipeline state when the daemon appears wedged inside mmio_read polling, so
// we can see whether DMA is in flight, whether the parser FIFO has dropped
// requests, etc. Mirrors the dump in sw_no_vm/main.cpp's --dump_host_debug.
static constexpr uint32_t HC_DEBUG_BASE  = 10;
static constexpr uint32_t HC_DEBUG_COUNT = 24;
static constexpr uint32_t HC_FIFO_DROPPED_REG = HC_DEBUG_BASE + HC_DEBUG_COUNT; // 34

static const std::array<const char *, HC_DEBUG_COUNT> HC_DEBUG_NAMES = {
    "live_status", "mmio_start", "mmio_read_req_sent", "mmio_write_req_sent",
    "mmio_resp_recv", "mmio_write_done", "d2h_header_recv", "d2h_payload_beats",
    "d2h_payload_done", "sq_write_fire", "h2d_read_req_recv", "sq_read_fire",
    "h2d_reply_header_sent", "h2d_payload_beats_sent", "rdma_meta_fire",
    "rdma_final_beats", "rdma_sq_stall_cycles", "network_out_backpressure",
    "network_in_backpressure", "host_out_backpressure", "host_in_backpressure",
    "last_mmio_resp_data", "last_d2h_len", "last_h2d_len",
};

static std::atomic<bool> g_dump_pending{false};

static void dump_hc_debug(coyote::cThread &ct, const char *label)
{
    std::cout << "\n--- HC DEBUG CSRS [" << label << "] ---" << std::endl;
    for (uint32_t i = 0; i < HC_DEBUG_COUNT; i++) {
        uint64_t v = ct.getCSR(HC_DEBUG_BASE + i);
        std::cout << "HC_DBG[" << std::setw(2) << i << "] "
                  << std::left << std::setw(26) << HC_DEBUG_NAMES[i]
                  << " = 0x" << std::hex << v << std::dec
                  << " (" << v << ")" << std::endl;
    }
    uint64_t dropped = ct.getCSR(HC_FIFO_DROPPED_REG);
    std::cout << "MMIO_FIFO_DROPPED               = " << dropped
              << (dropped ? "  <-- WARNING: requests were dropped" : "")
              << std::endl;
    std::cout << std::right << std::flush;
}

static void on_sigusr1(int) { g_dump_pending.store(true); }

static void run_app(coyote::cThread &ct, void *mem, bool debug_watcher)
{
    uint64_t mem_addr = reinterpret_cast<uint64_t>(mem);

    // --- Configure host-controller registers ---
    ct.setCSR(ct.getCtid(), static_cast<uint32_t>(HCReg::COYOTE_PID));
    ct.setCSR(mem_addr,     static_cast<uint32_t>(HCReg::MMIO_VADDR));

    // Background watcher (opt-in via --debug; measurement runs stay clean):
    // every 250 ms, check MMIO_FIFO_DROPPED and dump full counters if it
    // ever becomes non-zero (silent request drop). Also serves SIGUSR1 for
    // on-demand snapshots when the daemon looks wedged.
    if (debug_watcher) {
        std::signal(SIGUSR1, on_sigusr1);
        std::cout << "HC debug watcher armed (PID " << getpid()
                  << " — kill -USR1 " << getpid() << " for snapshot)" << std::endl;
        std::thread dumper([&ct] {
            bool drop_reported = false;
            while (true) {
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
                if (g_dump_pending.exchange(false)) {
                    dump_hc_debug(ct, "SIGUSR1");
                }
                if (!drop_reported &&
                    ct.getCSR(HC_FIFO_DROPPED_REG) != 0) {
                    dump_hc_debug(ct, "FIFO_DROPPED detected");
                    drop_reported = true;
                }
            }
        });
        dumper.detach();
    }

    void *ret = run_shmem_app(ct);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[]) {
    std::string device_ip;
    bool debug_watcher = false;

    boost::program_options::options_description opts("Jigsaw Host Controller Options");
    opts.add_options()
        ("ip_address,i",
            boost::program_options::value<std::string>(&device_ip),
            "Device-side OOB TCP/IP address (for QP exchange)")
        ("debug,d",
            boost::program_options::bool_switch(&debug_watcher),
            "Enable the HC debug watcher (SIGUSR1 snapshots + FIFO-drop autodump); off by default");

    boost::program_options::variables_map vm;
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, argv, opts), vm);
    boost::program_options::notify(vm);

    if (device_ip.empty()) {
        std::cout << "ERROR: --ip_address (-i) is required\n" << opts << std::endl;
        return EXIT_FAILURE;
    }

    HEADER("JIGSAW HOST CONTROLLER");
    std::cout << "Device OOB IP : " << device_ip << std::endl;
    std::cout << std::endl;

    // Create Coyote thread
    coyote::cThread ct(DEFAULT_VFPGA_ID, getpid(), 0);

    // Initialise RDMA — allocates hugepage buffer, exchanges QPs
    // (including the driver-configured RoCE IP), writes QP context
    // to HW, does ARP lookup.  The returned buffer doubles as our
    // jigsaw DMA buffer.
    void *mem = ct.initRDMA(RDMA_BUFFER_SIZE, coyote::DEF_PORT, device_ip.c_str());
    if (!mem) {
        throw std::runtime_error("initRDMA failed — could not allocate memory");
    }

    std::cout << "RDMA connection established (using RDMA WRITE)." << std::endl;

    // Write remote buffer address to HW for RDMA WRITE targeting
    uint64_t remote_vaddr = (uint64_t)ct.getQpair()->remote.vaddr;
    ct.setCSR(remote_vaddr, static_cast<uint32_t>(HCReg::REMOTE_VADDR));
    std::cout << "  Remote VADDR = 0x" << std::hex << remote_vaddr << std::dec << std::endl;
    std::cout << std::endl;

    // Init shared memory and tell coyote via register write
    void *shmem = init_shared_memory();
    if (!shmem) {
        printf("SHMEM: init_shared_memory failed\n");
    }
    ct.userMap(reinterpret_cast<char *>(shmem), SHMEM_SIZE);
    
    // Sync with device before starting
    ct.connSync(true);

    // Run the application
    run_app(ct, shmem, debug_watcher);

    // Sync with device after finishing
    ct.connSync(true);

    std::cout << "All tests completed." << std::endl;
    ct.closeConn();

    return EXIT_SUCCESS;
}
