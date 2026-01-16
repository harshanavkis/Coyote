// Includes
#include <chrono>
#include <limits>
#include <thread>
#include <iostream>
#include <boost/program_options.hpp>

#include "cThread.hpp"

// Constants
#define CLOCK_PERIOD_NS 4
#define DEFAULT_VFPGA_ID 0

#define N_LATENCY_REPS 1
#define N_THROUGHPUT_REPS 32

// Registers, corresponding to registers defined the vFPGA
enum class DMAEngineRegisters: uint32_t {
    DMA_CMD_REG = 0x00,
    DMA_SRC_ADDR_REG = 0x08,
    DMA_DST_ADDR_REG = 0x10,
    DMA_LEN_REG = 0x18,
    DMA_STATUS_REG = 0x20,
    DMA_TX_LEN_REG = 0x38
};

// Registers for jigsaw_host_controller based on jigsaw_minus_nw_axi_ctrl_parser
enum class JigsawHostControlRegisters : uint32_t {
    MMIO_VADDR_REG = 0,
    MMIO_CTRL_REG = 1,
    MMIO_WRITE_STATUS_REG = 2,
    MMIO_READ_STATUS_REG = 3,
    COYOTE_PID_REG = 4
};

// Note, how the Coyote thread is passed by reference; to avoid creating a copy of 
// the thread object which can lead to undefined behaviour and bugs. 
void run_bench(
    coyote::cThread &coyote_thread, int *mem
) {
    // Single iteration of transfers (reads or writes)
    auto benchmark_run = [&]() {
        // Set the host controller registers from SW
        coyote_thread.setCSR(coyote_thread.getCtid(), static_cast<uint32_t>(JigsawHostControlRegisters::COYOTE_PID_REG));

        // MMIO Read Helper

        auto read_mmio = [&](uint64_t addr) -> uint64_t {
             // Clear read status
             coyote_thread.setCSR(0, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_READ_STATUS_REG));
             
             // Prepare request: Opcode 0 (Read) at offset 24
             // Structure at mem + 24: [Opcode (1B) | Address (8B) | Length (8B) ...]
             // We use volatile to ensure writes happen before we trigger HW
             volatile unsigned char* base = (unsigned char*)mem;
             base[24] = 0; // Opcode 0 = Read
             *(volatile uint64_t*)(base + 25) = addr; // Address
             *(volatile uint64_t*)(base + 33) = 0; // Length (unused for read)
             
             // Trigger MMIO
             coyote_thread.setCSR(1, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_CTRL_REG));
             
             // Poll for completion
             while (coyote_thread.getCSR(static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_READ_STATUS_REG)) != 1) {
                std::this_thread::sleep_for(std::chrono::nanoseconds(CLOCK_PERIOD_NS));
             }
             
             // Read result from offset 16 (where response is written)
             // Structure at mem + 16: [Data (8B)]
             return *(volatile uint64_t*)(base + 16);
        };

        // MMIO Write Helper
        auto write_mmio = [&](uint64_t addr, uint64_t data) {
             // Clear write status
             coyote_thread.setCSR(0, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_WRITE_STATUS_REG));

             // Prepare request: Opcode 1 (Write) at offset 24
             // Structure at mem + 24: [Opcode (1B) | Address (8B) | Length (8B) | Data (8B)]
             volatile unsigned char* base = (unsigned char*)mem;
             base[24] = 1; // Opcode 1 = Write
             *(volatile uint64_t*)(base + 25) = addr; // Address
             *(volatile uint64_t*)(base + 33) = 0; // Length
             *(volatile uint64_t*)(base + 41) = data; // Data

             // Trigger MMIO
             coyote_thread.setCSR(1, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_CTRL_REG));

             // Poll for completion
             while (coyote_thread.getCSR(static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_WRITE_STATUS_REG)) != 1) {
                std::this_thread::sleep_for(std::chrono::nanoseconds(CLOCK_PERIOD_NS));
             }
        };

        uint64_t mem_addr = reinterpret_cast<uint64_t>(mem);

        coyote_thread.setCSR(mem_addr, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_VADDR_REG));

        // Read DMA SRC ADDR (Reg 1 -> Address 0x8)
        uint64_t src_addr = read_mmio(static_cast<uint32_t>(DMAEngineRegisters::DMA_SRC_ADDR_REG));
        std::cout << "DMA Source Address: 0x" << std::hex << src_addr << std::dec << std::endl;

        // Read DMA DST ADDR (Reg 2 -> Address 0x10)
        uint64_t dst_addr = read_mmio(static_cast<uint32_t>(DMAEngineRegisters::DMA_DST_ADDR_REG));
        std::cout << "DMA Destination Address: 0x" << std::hex << dst_addr << std::dec << std::endl;

        // Read DMA TX LEN (Reg 3 -> Address 0x38)
        uint64_t tx_len = read_mmio(static_cast<uint32_t>(DMAEngineRegisters::DMA_TX_LEN_REG));
        std::cout << "DMA TX Length: " << std::dec << tx_len << std::endl;

        // Read DMA STATUS (Reg 4 -> Address 0x38)
        uint64_t status = read_mmio(static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG));
        std::cout << "DMA Status: " << std::dec << status << std::endl;


        // DMA Tests
        ////////////////////////////////////////////////////////

        // --- H2D DMA TEST ---
        std::cout << "\nStarting H2D DMA Test..." << std::endl;

        std::cout << "MMIO VADDR: 0x" << std::hex << mem_addr << std::dec << std::endl;

        std::cout << "Reading MMIO VADDR: 0x" << std::hex << coyote_thread.getCSR(static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_VADDR_REG)) << std::dec << std::endl;
        std::cout << "Reading coyote PID: " << coyote_thread.getCSR(static_cast<uint32_t>(JigsawHostControlRegisters::COYOTE_PID_REG)) << std::endl;

        // 1. Setup Destination Address (mem + 4KiB)
        uint64_t h2d_dst_addr = mem_addr + 4096;
        write_mmio(static_cast<uint32_t>(DMAEngineRegisters::DMA_SRC_ADDR_REG), h2d_dst_addr);
        std::cout << "Set DMA Source Address: 0x" << std::hex << h2d_dst_addr << std::dec << std::endl;
        std::cout << "Reading DMA Source Address: 0x" << std::hex << read_mmio(static_cast<uint32_t>(DMAEngineRegisters::DMA_SRC_ADDR_REG)) << std::dec << std::endl;

        // 2. Setup Length
        uint64_t len = 1024; // 1KiB
        write_mmio(static_cast<uint32_t>(DMAEngineRegisters::DMA_LEN_REG), len);
        std::cout << "Set DMA Length: " << std::dec << len << std::endl;
        std::cout << "Reading DMA LEN: " << read_mmio(static_cast<uint32_t>(DMAEngineRegisters::DMA_LEN_REG)) << std::endl;

        // 3. Start DMA (Direction = 1 (D2H), Start = 1) -> 0x3
        write_mmio(static_cast<uint32_t>(DMAEngineRegisters::DMA_CMD_REG), 1);
        std::cout << "Started DMA (Cmd: 1)" << std::endl;

        int polling_count = 0;
        status = 0;
        while ((status = read_mmio(static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG)) & 0x1) != 1) {
             if (polling_count % 1000 == 0) {
                 std::cout << "Polling DMA status... count: " << polling_count << " val: " << status << std::endl;
             }
             polling_count++;
             std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
        std::cout << "DMA Completed!" << std::endl;

        status = read_mmio(static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG));
        std::cout << "DMA Status: " << std::dec << status << std::endl;

        write_mmio(static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG), 0);

        // 5. Verify DMA TX Length
        tx_len = read_mmio(static_cast<uint32_t>(DMAEngineRegisters::DMA_TX_LEN_REG));
        std::cout << "DMA TX Length: " << std::dec << tx_len << std::endl;
    };

    benchmark_run();
}

int main(int argc, char *argv[]) {
    // Create Coyote thread and allocate memory for the transfer
    coyote::cThread coyote_thread(DEFAULT_VFPGA_ID, getpid());
    int* mem =  (int *) coyote_thread.getMem({coyote::CoyoteAllocType::HPF, 16*1024});
    if (!mem) { throw std::runtime_error("Could not allocate memory; exiting..."); }

    // Benchmark sweep
    HEADER("JIGSAW BASELINE");
    
    // Run benchmark twice without restart to reproduce second-run failure
    std::cout << "=== RUN 1 ===" << std::endl;
    run_bench(coyote_thread, mem);
    
    // std::cout << "Press [Enter] to continue to RUN 2..." << std::endl;
    // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // clear buffer
    // std::cin.get(); // wait for enter

    // std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "=== RUN 2 ===" << std::endl;
    run_bench(coyote_thread, mem);

    return EXIT_SUCCESS;
}

