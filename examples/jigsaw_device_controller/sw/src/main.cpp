/**
 * Jigsaw Device Controller — Software
 *
 * Acts as the RDMA server side.  Sets up the RDMA connection (QP
 * exchange, ARP lookup), configures the device-controller AXI-Lite
 * registers (PID), then waits for the host to drive all traffic.
 * Uses RDMA SEND (no remote vaddr needed).  The device HW
 * (txn_generator) handles everything autonomously once the
 * connection is live.
 *
 * The RoCE IP address is configured at driver-load time (insmod) and
 * is automatically read from the driver and exchanged during initRDMA.
 *
 * Usage:
 *   ./test
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include <coyote/cThread.hpp>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
#define DEFAULT_VFPGA_ID     0
#define RDMA_BUFFER_SIZE     (1024 * 1024)  // 1 MiB

// ---------------------------------------------------------------------------
// AXI-Lite register map — jigsaw_dc_axi_ctrl_parser
// ---------------------------------------------------------------------------
enum class DCReg : uint32_t {
    COYOTE_PID          = 0,
    REMOTE_VADDR        = 1,
};

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[]) {
    HEADER("JIGSAW DEVICE CONTROLLER");

    // Create Coyote thread
    coyote::cThread ct(DEFAULT_VFPGA_ID, getpid());

    // Initialise RDMA as server (no IP address).
    // Blocks until the host-side client connects, then:
    //  - Exchanges QP info (QPN, PSN, rkey, GID/RoCE-IP, vaddr)
    //  - Writes QP context to HW config registers
    //  - Does ARP lookup for the remote (host) node
    //  - Allocates a hugepage buffer
    std::cout << "Waiting for host connection on port "
              << coyote::DEF_PORT << " ..." << std::endl;

    void *mem = ct.initRDMA(RDMA_BUFFER_SIZE, coyote::DEF_PORT);
    if (!mem) {
        throw std::runtime_error("initRDMA failed — could not allocate memory");
    }
    std::cout << "RDMA connection established." << std::endl;

    // Configure device-controller AXI-Lite registers
    ct.setCSR(ct.getCtid(), static_cast<uint32_t>(DCReg::COYOTE_PID));

    // Write remote buffer address to HW for RDMA WRITE targeting
    uint64_t remote_vaddr = (uint64_t)ct.getQpair()->remote.vaddr;
    ct.setCSR(remote_vaddr, static_cast<uint32_t>(DCReg::REMOTE_VADDR));

    std::cout << "Device controller configured:" << std::endl;
    std::cout << "  PID          = " << ct.getCtid() << std::endl;
    std::cout << "  Remote VADDR = 0x" << std::hex << remote_vaddr << std::dec << std::endl;
    std::cout << "  Using RDMA WRITE" << std::endl;
    std::cout << std::endl;

    // Signal host that we are ready
    ct.connSync(false);
    std::cout << "Host connected — device ready, HW handles all traffic." << std::endl;

    // Wait for host to signal completion
    ct.connSync(false);
    std::cout << "Host signalled completion." << std::endl;

    ct.closeConn();
    std::cout << "Connection closed. Exiting." << std::endl;

    return EXIT_SUCCESS;
}
