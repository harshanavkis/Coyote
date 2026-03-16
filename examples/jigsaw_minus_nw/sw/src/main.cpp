// Includes
#include <chrono>
#include <limits>
#include <thread>
#include <iostream>
#include <boost/program_options.hpp>

#include <coyote/cThread.hpp>

#include "shmem.hpp"

// Note, how the Coyote thread is passed by reference; to avoid creating a copy of 
// the thread object which can lead to undefined behaviour and bugs. 
void run_bench(
    coyote::cThread &coyote_thread, void *mem
) {
    // Single iteration of transfers (reads or writes)
    auto benchmark_run = [&]() {
        // Set the host controller registers from SW
        coyote_thread.setCSR(coyote_thread.getCtid(), static_cast<uint32_t>(JigsawHostControlRegisters::COYOTE_PID_REG));

        uint64_t mem_addr = reinterpret_cast<uint64_t>(mem);

        coyote_thread.setCSR(mem_addr, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_VADDR_REG));

        void *ret = run_shmem_app(coyote_thread);
    };

    benchmark_run();
}

int main(int argc, char *argv[]) {
    // Create Coyote thread and allocate memory for the transfer
    void *shmem = init_shared_memory();
    if (!shmem) {
        printf("SHMEM: init_shared_memory failed\n");
    }

    coyote::cThread coyote_thread(DEFAULT_VFPGA_ID, getpid());

    coyote_thread.userMap(reinterpret_cast<char *>(shmem) + DMA_REGION_OFFSET, DMA_SIZE);

    // Benchmark sweep
    HEADER("JIGSAW BASELINE");
    
    run_bench(coyote_thread, shmem);
    return EXIT_SUCCESS;
}

