/**
 * Jigsaw Software Forwarder — host-side VM daemon
 *
 * Software-only replacement for the jigsaw host controller: serves the
 * guest's MMIO requests from the ivshmem region (same doorbell protocol as
 * jigsaw_host_controller/sw, so the VM stack runs unchanged), but instead of
 * writing them into the host-controller vFPGA it encapsulates them into wire
 * messages and forwards them over the Coyote RDMA stack (perf_rdma as a dumb
 * NIC) to the device-side replayer.
 *
 * DMA payloads bounce between ivshmem and the NIC (QP) buffer on this side
 * and between the NIC buffer and the device buffer on the device side — the
 * two staging copies of a software forwarder on commodity hardware.
 *
 * Meant to busy-poll on one dedicated core (run under taskset).
 *
 * Usage:
 *   ./test -i <device_oob_ip>
 */

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>

#include <boost/program_options.hpp>

#include <coyote/cThread.hpp>

#include "shmem.hpp"
#include "wire.hpp"

using namespace jsfwd;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
#define DEFAULT_VFPGA_ID 0

static_assert(SHMEM_SIZE == jsfwd::BUF_BYTES,
              "ivshmem and QP buffer must mirror each other");
static_assert(DMA_REGION_OFFSET == jsfwd::PAYLOAD_OFF,
              "ivshmem DMA region and QP payload region must line up");

static std::atomic<bool> g_stop{false};
static void on_sigint(int) { g_stop.store(true); }

// ---------------------------------------------------------------------------
// Forwarding loop — same structure as run_shmem_app() in
// jigsaw_host_controller/sw, with the backend swapped for the wire protocol.
// ---------------------------------------------------------------------------
static void run_forwarder(HostForwarder &fw)
{
    std::cout << "SHMEM application started. Waiting for messages..." << std::endl;

    while (!g_stop.load(std::memory_order_relaxed))
    {
        shmem_arm_write_doorbell();

        shmem_wait_write_doorbell();

        mmio_message_header header;
        shmem_read_header(&header);

        switch (header.operation)
        {
        case OP_READ:
            shmem_complete_read(fw.mmio_read(header.address));
            continue;

        case OP_WRITE:
            fw.mmio_write(header.address, header.value);
            continue;

        default:
            fprintf(stderr, "Unknown operation: %d\n", header.operation);
            continue;
        }
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[]) {
    std::string device_ip;

    boost::program_options::options_description opts("Jigsaw SW Forwarder Options");
    opts.add_options()
        ("ip_address,i",
            boost::program_options::value<std::string>(&device_ip),
            "Device-side OOB TCP/IP address (for QP exchange)");

    boost::program_options::variables_map vm;
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, argv, opts), vm);
    boost::program_options::notify(vm);

    if (device_ip.empty()) {
        std::cout << "ERROR: --ip_address (-i) is required\n" << opts << std::endl;
        return EXIT_FAILURE;
    }

    HEADER("JIGSAW SW FORWARDER — HOST");
    std::cout << "Device OOB IP : " << device_ip << std::endl;
    std::cout << std::endl;

    coyote::cThread ct(DEFAULT_VFPGA_ID, getpid());

    char *nic = static_cast<char *>(
        ct.initRDMA(BUF_BYTES, coyote::DEF_PORT, device_ip.c_str()));
    if (!nic) {
        std::cerr << "initRDMA failed" << std::endl;
        return EXIT_FAILURE;
    }
    memset(nic, 0, CTRL_BYTES);

    // Init shared memory with the guest; publishes the proxy DMA base the
    // guest hands out as DMA pointers (ivshmem base + DMA_REGION_OFFSET).
    char *shmem = static_cast<char *>(init_shared_memory());
    if (!shmem) {
        std::cerr << "init_shared_memory failed" << std::endl;
        return EXIT_FAILURE;
    }

    // Barrier: both sides have zeroed their control pages
    ct.connSync(true);
    std::cout << "RDMA connection established." << std::endl;

    HostForwarder fw(ct, nic, shmem);
    fw.send_setup();

    std::signal(SIGINT, on_sigint);
    run_forwarder(fw);

    std::cout << "Stopping..." << std::endl;
    fw.send_stop();
    ct.connSync(true);
    ct.closeConn();

    return EXIT_SUCCESS;
}
