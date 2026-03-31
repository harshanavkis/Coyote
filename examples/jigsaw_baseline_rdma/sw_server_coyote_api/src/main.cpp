#include <thread>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>

#include <coyote/cThread.hpp>

// Constants
#define CLOCK_PERIOD_NS 4
#define DEFAULT_VFPGA_ID 0
#define DMA_SIZE (1024*1024) 
#define CONTROL_SIZE 4096
#define DEF_PORT 18488

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

// Message structure for control plane
struct msg {
    uint64_t type;      // 0 for Request, 1 for Completion, 2 for Failure
    uint64_t size;      // Size of the buffer to transfer
    uint64_t direction; // 0 for D2H, 1 for H2D
    uint64_t ready;     // Signaling flag: 1 means message is valid
    uint64_t padding[4];// Pad to 64 bytes
};

void device_d2h(coyote::cThread &coyote_thread, void *dst, size_t size) {
    coyote_thread.setCSR(reinterpret_cast<uint64_t>(dst), static_cast<uint32_t>(JigsawRegisters::DMA_DST_ADDR_REG));
    coyote_thread.setCSR(static_cast<uint64_t>(size), static_cast<uint32_t>(JigsawRegisters::DMA_LEN_REG));
    coyote_thread.setCSR(coyote_thread.getCtid(), static_cast<uint32_t>(JigsawRegisters::COYOTE_PID_REG));

    // Start DMA transfer (3 for D2H in original code)
    coyote_thread.setCSR(static_cast<uint64_t>(3), static_cast<uint32_t>(JigsawRegisters::DMA_CMD_REG));

    while (coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG)) != 1) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(CLOCK_PERIOD_NS));
    }
}

void device_h2d(coyote::cThread &coyote_thread, void *src, size_t size) {
    coyote_thread.setCSR(reinterpret_cast<uint64_t>(src), static_cast<uint32_t>(JigsawRegisters::DMA_SRC_ADDR_REG));
    coyote_thread.setCSR(static_cast<uint64_t>(size), static_cast<uint32_t>(JigsawRegisters::DMA_LEN_REG));
    coyote_thread.setCSR(coyote_thread.getCtid(), static_cast<uint32_t>(JigsawRegisters::COYOTE_PID_REG));

    // Start DMA transfer (1 for H2D in original code)
    coyote_thread.setCSR(static_cast<uint64_t>(1), static_cast<uint32_t>(JigsawRegisters::DMA_CMD_REG));

    while (coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG)) != 1) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(CLOCK_PERIOD_NS));
    }
}

int main(int argc, char *argv[]) {
    std::cout << "Starting Combined Jigsaw + Perf RDMA Server (Coyote API)..." << std::endl;

    // vFPGA 0: perf_rdma (Networked benchmark)
    coyote::cThread coyote_perf(0, getpid());
    // vFPGA 1: jigsaw_baseline (Local application)
    coyote::cThread coyote_jigsaw(1, getpid());

    // Allocate and initialize RDMA ONLY for vFPGA 0 (benchmark)
    size_t total_size = DMA_SIZE + CONTROL_SIZE;
    void *mem_shared = coyote_perf.initRDMA(total_size, DEF_PORT);
    if (!mem_shared) {
        std::cerr << "Failed to initialize RDMA on vFPGA 0" << std::endl;
        return EXIT_FAILURE;
    }

    // Share the same physical memory with vFPGA 1 by mapping it into its TLB
    coyote_jigsaw.userMap(mem_shared, static_cast<uint32_t>(total_size));

    volatile struct msg *mailbox = static_cast<volatile struct msg *>(mem_shared);
    void *dma_buf = static_cast<char *>(mem_shared) + CONTROL_SIZE;

    std::cout << "Server ready on port " << DEF_PORT << " (vFPGA 0: RDMA, vFPGA 1: Local / Shared Memory)" << std::endl;

    // Initial sync with client
    coyote_perf.connSync(false);

    while (true) {
        // Poll mailbox on vFPGA 0 (RDMA connection)
        if (mailbox->ready == 1 && mailbox->type == 0) {
            uint64_t transfer_size = mailbox->size;
            uint64_t direction = mailbox->direction;
            
            // Clear local ready flag immediately to prevent re-processing
            mailbox->ready = 0;

            if (direction == 2) { // Special SYNC mode for perf_rdma style queue flushing
                std::cout << "[SYNC] Clearing queues and synchronizing network... " << std::flush;
                
                mailbox->type = 1;
                mailbox->direction = 2;
                mailbox->ready = 1;
                coyote::rdmaSg compl_sg = {.local_offs = 0, .remote_offs = 0, .len = 64};
                coyote_perf.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, compl_sg);
                
                // Extremely short hardware settlement delay before tearing down via reset
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                
                coyote_perf.clearCompleted();
                coyote_perf.connSync(true);
                
                std::cout << "Done." << std::endl;
                continue;
            }

            if (direction == 0) { // D2H (FPGA -> Network)
                std::cout << "[D2H] Processing size: " << transfer_size << " bytes..." << std::endl;

                // 1. Talk to device: Pull data from Jigsaw (vFPGA 1) to shared buffer
                std::cout << "  -> Talking to device (vFPGA 1)... " << std::flush;
                device_d2h(coyote_jigsaw, dma_buf, transfer_size);
                std::cout << "Done. (Data ready for client push)" << std::endl;
                
                // 2. Push payload to Client!
                std::cout << "  -> Pushing payload to Client (vFPGA 0)... " << std::flush;
                coyote::rdmaSg push_sg = {
                    .local_offs = CONTROL_SIZE,
                    .remote_offs = CONTROL_SIZE,
                    .len = static_cast<uint32_t>(transfer_size)};
                coyote_perf.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, push_sg);
                std::cout << "Done." << std::endl;

            } else { // H2D (Network -> FPGA)
                std::cout << "[H2D] Processing size: " << transfer_size << " bytes..." << std::endl;

                // 1. Data completely pushed by Client already! Talk to device: Push data from shared buffer to Jigsaw (vFPGA 1)
                std::cout << "  -> Talking to device (vFPGA 1)... " << std::flush;
                device_h2d(coyote_jigsaw, dma_buf, transfer_size);
                std::cout << "Done." << std::endl;
            }

            // 3. Signal completion back to client
            std::cout << "  -> Signaling completion (vFPGA 0)... " << std::flush;
            mailbox->type = 1;
            mailbox->ready = 1;

            coyote::rdmaSg compl_sg = {
                .local_offs = 0,
                .remote_offs = 0,
                .len = 64 // Coyote RDMA might require lengths to be a multiple of 64 bytes (cacheline)
            };
            
            coyote_perf.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, compl_sg);
            // Wait: outbound RDMA WRITE does not generate a reliable completion to poll on. 
            // The next incoming operation will synchronize via LOCAL_WRITE.
            std::cout << " Done. Request completed." << std::endl;
        }
    }

    return EXIT_SUCCESS;
}
