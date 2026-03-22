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

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <boost/program_options.hpp>

#include <coyote/cThread.hpp>

#include "shmem.hpp"

static void run_app(coyote::cThread &ct, void *mem)
{
    uint64_t mem_addr = reinterpret_cast<uint64_t>(mem);

    // --- Configure host-controller registers ---
    ct.setCSR(ct.getCtid(), static_cast<uint32_t>(HCReg::COYOTE_PID));
    ct.setCSR(mem_addr,     static_cast<uint32_t>(HCReg::MMIO_VADDR));

    void *ret = run_shmem_app(ct);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[]) {
    std::string device_ip;

    boost::program_options::options_description opts("Jigsaw Host Controller Options");
    opts.add_options()
        ("ip_address,i",
            boost::program_options::value<std::string>(&device_ip),
            "Device-side OOB TCP/IP address (for QP exchange)");

    boost::program_options::variables_map vm;
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, argv, opts), vm);
    boost::program_options::notify(vm);

    if (device_ip.empty()) {
        std::cerr << "ERROR: --ip_address (-i) is required\n" << opts << std::endl;
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
    run_app(ct, shmem);

    // Sync with device after finishing
    ct.connSync(true);

    std::cout << "All tests completed." << std::endl;
    ct.closeConn();

    return EXIT_SUCCESS;
}
