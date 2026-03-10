// Includes
#include <chrono>
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
enum class JigsawRegisters: uint32_t {
    DMA_CMD_REG = 0,
    DMA_SRC_ADDR_REG = 1,
    DMA_DST_ADDR_REG = 2,
    DMA_LEN_REG = 3,
    DMA_STATUS_REG = 4,
    START_COMPUTATION_REG = 5,
    CYCLES_PER_COMPUTATION_REG = 6,
    COYOTE_PID_REG = 7,
    COYOTE_DMA_TX_LEN_REG = 8
};

// Note, how the Coyote thread is passed by reference; to avoid creating a copy of 
// the thread object which can lead to undefined behaviour and bugs. 
void run_bench(
    coyote::cThread &coyote_thread, int *mem
) {
    // Single iteration of transfers (reads or writes)
    auto benchmark_run = [&]() {
        // Set the required registers from SW
        coyote_thread.setCSR(reinterpret_cast<uint64_t>(mem), static_cast<uint32_t>(JigsawRegisters::DMA_SRC_ADDR_REG));
        coyote_thread.setCSR(reinterpret_cast<uint64_t>(mem), static_cast<uint32_t>(JigsawRegisters::DMA_DST_ADDR_REG));
        coyote_thread.setCSR(static_cast<uint64_t>(32768), static_cast<uint32_t>(JigsawRegisters::DMA_LEN_REG));
        coyote_thread.setCSR(coyote_thread.getCtid(), static_cast<uint32_t>(JigsawRegisters::COYOTE_PID_REG));

        // Start DMA transfer
        coyote_thread.setCSR(static_cast<uint64_t>(3), static_cast<uint32_t>(JigsawRegisters::DMA_CMD_REG));

        // coyote_thread.setCSR(1, static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG));
        // coyote_thread.setCSR(2, static_cast<uint32_t>(JigsawRegisters::START_COMPUTATION_REG));
        // coyote_thread.setCSR(100, static_cast<uint32_t>(JigsawRegisters::CYCLES_PER_COMPUTATION_REG));

        // std:: cout << "DMA_CMD_REG: " << coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_CMD_REG)) << std::endl;
        // std:: cout << "DMA_STATUS_REG: " << coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG)) << std::endl;
        // std:: cout << "DMA_SRC_ADDR_REG: " << coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_SRC_ADDR_REG)) << std::endl;
        // std:: cout << "DMA_DST_ADDR_REG: " << coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_DST_ADDR_REG)) << std::endl;
        // std:: cout << "DMA_LEN_REG: " << coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_LEN_REG)) << std::endl;
        // std:: cout << "START_COMPUTATION_REG: " << coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::START_COMPUTATION_REG)) << std::endl;
        // std:: cout << "CYCLES_PER_COMPUTATION_REG: " << coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::CYCLES_PER_COMPUTATION_REG)) << std::endl;
        // std:: cout << "COYOTE_PID_REG: " << coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::COYOTE_PID_REG)) << std::endl;

        // Wait for DMA transfer to complete
        while (coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG)) != 1) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(CLOCK_PERIOD_NS));
        }

        std:: cout << "DMA_STATUS_REG: " << coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG)) << std::endl;
        std:: cout << "DMA_CMD_REG: " << coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_CMD_REG)) << std::endl;
        std:: cout << "COYOTE_DMA_TX_LEN_REG: " << coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::COYOTE_DMA_TX_LEN_REG)) << std::endl;

        unsigned char* ptr = (unsigned char*)mem;
        for (int i = 0; i < 64; i++) {
            printf("%02x ", ptr[i]);
        }
    };

    benchmark_run();
}

int main(int argc, char *argv[]) {
    // Create Coyote thread and allocate memory for the transfer
    coyote::cThread coyote_thread(DEFAULT_VFPGA_ID, getpid());
    int* mem =  (int *) coyote_thread.getMem({coyote::CoyoteAllocType::HPF, 4*1024*1024});
    if (!mem) { throw std::runtime_error("Could not allocate memory; exiting..."); }

    // Benchmark sweep
    HEADER("JIGSAW BASELINE");
    
    run_bench(coyote_thread, mem);

    return EXIT_SUCCESS;
}

