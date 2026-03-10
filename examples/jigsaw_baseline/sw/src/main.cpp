// Includes
#include <unistd.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <boost/program_options.hpp>

#include "cThread.hpp"

#include "shmem.hpp"

// Constants
#define CLOCK_PERIOD_NS 4
#define DEFAULT_VFPGA_ID 0

#define N_LATENCY_REPS 1
#define N_THROUGHPUT_REPS 32

// Note, how the Coyote thread is passed by reference; to avoid creating a copy of 
// the thread object which can lead to undefined behaviour and bugs. 
void run_bench( coyote::cThread &coyote_thread)
{
    // Single iteration of transfers (reads or writes)
    auto benchmark_run = [&]() {
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
    
    run_bench(coyote_thread);

    return EXIT_SUCCESS;
}

