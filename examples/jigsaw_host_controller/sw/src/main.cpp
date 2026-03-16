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

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
#define CLOCK_PERIOD_NS      4
#define DEFAULT_VFPGA_ID     0
#define RDMA_BUFFER_SIZE     (2048 * 1024)  // 1 MiB

// ---------------------------------------------------------------------------
// AXI-Lite register map — jigsaw_hc_axi_ctrl_parser
// ---------------------------------------------------------------------------
enum class HCReg : uint32_t {
    MMIO_VADDR          = 0,
    MMIO_CTRL           = 1,
    MMIO_WRITE_STATUS   = 2,
    MMIO_READ_STATUS    = 3,
    COYOTE_PID          = 4,
    REMOTE_VADDR        = 5,
};

// ---------------------------------------------------------------------------
// Device register map — payload_to_mmio on the device side
// ---------------------------------------------------------------------------
enum class DevReg : uint64_t {
    DMA_CMD        = 0x00,
    DMA_SRC_ADDR   = 0x08,
    DMA_DST_ADDR   = 0x10,
    DMA_LEN        = 0x18,
    DMA_STATUS     = 0x20,
    START_COMPUTE  = 0x28,
    CYCLES_COMPUTE = 0x30,
    DMA_TX_LEN     = 0x38,
};

// ---------------------------------------------------------------------------
// MMIO helpers
// ---------------------------------------------------------------------------

/**
 * @brief Read a device register over the jigsaw protocol.
 *
 * Packs a read request at mem+24, triggers the host controller via
 * MMIO_CTRL, polls MMIO_READ_STATUS, then reads the 8-byte result
 * from mem+16.
 */
static uint64_t read_mmio(coyote::cThread &ct, void *mem, uint64_t addr) {
    ct.setCSR(0, static_cast<uint32_t>(HCReg::MMIO_READ_STATUS));

    volatile unsigned char *base = static_cast<volatile unsigned char *>(mem);

    // Diagnostic: write sentinel so we can tell if the response DMA actually arrived
    *reinterpret_cast<volatile uint64_t *>(base + 16) = 0xDEADBEEFDEADBEEFULL;

    std::cout << "  Reading deadbeef" << std::hex << *reinterpret_cast<volatile uint64_t *>(base + 16)<< std::dec << std::endl;

    base[24] = 0;                                              // Opcode 0 = Read
    *reinterpret_cast<volatile uint64_t *>(base + 25) = addr;
    *reinterpret_cast<volatile uint64_t *>(base + 33) = 0;

    ct.setCSR(1, static_cast<uint32_t>(HCReg::MMIO_CTRL));

    while (ct.getCSR(static_cast<uint32_t>(HCReg::MMIO_READ_STATUS)) != 1) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(CLOCK_PERIOD_NS));
    }

    return *reinterpret_cast<volatile uint64_t *>(base + 16);
}

/**
 * @brief Write a device register over the jigsaw protocol.
 */
static void write_mmio(coyote::cThread &ct, void *mem, uint64_t addr, uint64_t data) {
    ct.setCSR(0, static_cast<uint32_t>(HCReg::MMIO_WRITE_STATUS));

    volatile unsigned char *base = static_cast<volatile unsigned char *>(mem);
    base[24] = 1;                                              // Opcode 1 = Write
    *reinterpret_cast<volatile uint64_t *>(base + 25) = addr;
    *reinterpret_cast<volatile uint64_t *>(base + 33) = 0;
    *reinterpret_cast<volatile uint64_t *>(base + 41) = data;

    ct.setCSR(1, static_cast<uint32_t>(HCReg::MMIO_CTRL));

    while (ct.getCSR(static_cast<uint32_t>(HCReg::MMIO_WRITE_STATUS)) != 1) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(CLOCK_PERIOD_NS));
    }
}

// ---------------------------------------------------------------------------
// Test routine
// ---------------------------------------------------------------------------
static void run_test(coyote::cThread &ct, void *mem) {
    uint64_t mem_addr = reinterpret_cast<uint64_t>(mem);

    // --- Configure host-controller registers ---
    ct.setCSR(ct.getCtid(), static_cast<uint32_t>(HCReg::COYOTE_PID));
    ct.setCSR(mem_addr,     static_cast<uint32_t>(HCReg::MMIO_VADDR));

    std::cout << "Host controller configured:" << std::endl;
    std::cout << "  PID         = " << ct.getCtid() << std::endl;
    std::cout << "  MMIO VADDR  = 0x" << std::hex << mem_addr << std::dec << std::endl;
    std::cout << std::endl;

    // --- MMIO read tests ---
    std::cout << "=== MMIO Read Tests ===" << std::endl;

    uint64_t v;
    v = read_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_SRC_ADDR));
    std::cout << "  DMA Source Address : 0x" << std::hex << v << std::dec << std::endl;

    v = read_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_DST_ADDR));
    std::cout << "  DMA Dest Address   : 0x" << std::hex << v << std::dec << std::endl;

    v = read_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_STATUS));
    std::cout << "  DMA Status         : " << v << std::endl;

    v = read_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_TX_LEN));
    std::cout << "  DMA TX Length      : " << v << std::endl;
    std::cout << std::endl;

    // --- DMA test (H2D read, 1 KiB) ---
    uint64_t xfer = 1024*1024;
    std::cout << "=== DMA Test (H2D Read, " << xfer << " B) ===" << std::endl;

    uint64_t dma_dst = mem_addr + 4096;
    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_DST_ADDR), dma_dst);
    std::cout << "  Set DMA DST  = 0x" << std::hex << dma_dst << std::dec << std::endl;

    v = read_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_DST_ADDR));
    std::cout << "  Read DMA DST = 0x" << std::hex << v << std::dec << std::endl;

    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_SRC_ADDR), dma_dst);
    std::cout << "  Set DMA SRC  = 0x" << std::hex << dma_dst << std::dec << std::endl;

    v = read_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_SRC_ADDR));
    std::cout << "  Read DMA SRC = 0x" << std::hex << v << std::dec << std::endl;

    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_LEN), xfer);
    std::cout << "  Set DMA LEN  = " << xfer << std::endl;

    v = read_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_LEN));
    std::cout << "  Read DMA LEN = " << v << std::endl;

    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_STATUS), 0);

    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_CMD), 1);
    std::cout << "  Started DMA (CMD=1)" << std::endl;

    int polls = 0;
    uint64_t status = 0;
    while (((status = read_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_STATUS))) & 0x1) != 1) {
        if (polls % 1000 == 0) {
            std::cout << "  Polling DMA status... count=" << polls << " val=" << status << std::endl;
        }
        polls++;
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    std::cout << "  DMA Completed!" << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(5));

    status = read_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_STATUS));
    std::cout << "  DMA Status   = " << status << std::endl;

    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_STATUS), 0);

    v = read_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_TX_LEN));
    std::cout << "  DMA TX LEN   = " << v << std::endl;
    std::cout << std::endl;
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

    // Sync with device before starting
    ct.connSync(true);

    int* jigsaw_mem =  (int *) ct.getMem({coyote::CoyoteAllocType::HPF, 1024*1024});
    if (!jigsaw_mem) { throw std::runtime_error("Could not allocate memory; exiting..."); }

    // Run tests
    for (int i = 0; i < 10; i++) {
        std::cout << "=== RUN " << i << " ===" << std::endl;
        run_test(ct, jigsaw_mem);
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    // Sync with device after finishing
    ct.connSync(true);

    std::cout << "All tests completed." << std::endl;
    ct.closeConn();

    return EXIT_SUCCESS;
}
