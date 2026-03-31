#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <iomanip>
#include <getopt.h>
#include <unistd.h>

#include <coyote/cThread.hpp>

// Constants
#define DEFAULT_VFPGA_ID 0
#define DMA_SIZE (1024 * 1024)
#define CONTROL_SIZE 4096
#define DEF_PORT 18488

// Message structure for control plane
struct msg
{
    uint64_t type;       // 0 for Request, 1 for Completion, 2 for Failure
    uint64_t size;       // Size of the buffer to transfer
    uint64_t direction;  // 0 for D2D, 1 for H2D
    uint64_t ready;      // Signaling flag: 1 means message is valid
    uint64_t padding[4]; // Pad to 64 bytes
};

int main(int argc, char *argv[])
{
    std::string server_ip = "127.0.0.1";
    uint16_t port = DEF_PORT;
    int iterations = 10;
    int op;

    while ((op = getopt(argc, argv, "i:n:")) != -1)
    {
        switch (op)
        {
        case 'i':
            server_ip = optarg;
            break;
        case 'n':
            iterations = std::stoi(optarg);
            break;
        default:
            std::cerr << "Usage: " << argv[0] << " [-i server_ip] [-n iterations]" << std::endl;
            return EXIT_FAILURE;
        }
    }

    std::cout << "Starting Jigsaw Baseline RDMA Client (Coyote API)..." << std::endl;
    std::cout << "Server: " << server_ip << ":" << port << std::endl;
    std::cout << "Iterations per size: " << iterations << std::endl;

    // Create Coyote thread
    coyote::cThread coyote_thread(DEFAULT_VFPGA_ID, getpid());

    // Initialize RDMA as client
    size_t total_size = DMA_SIZE + CONTROL_SIZE;
    void *mem = coyote_thread.initRDMA(total_size, port, server_ip.c_str());
    if (!mem)
    {
        std::cerr << "Failed to initialize RDMA" << std::endl;
        return EXIT_FAILURE;
    }

    volatile struct msg *mailbox = static_cast<volatile struct msg *>(mem);
    void *dma_buf = static_cast<char *>(mem) + CONTROL_SIZE;

    // Initial sync with server
    coyote_thread.connSync(true);

    // Results storage
    struct Result
    {
        size_t size;
        std::string direction;
        int iteration;
        double latency_us;
        double throughput_gbps;
    };
    std::vector<Result> results;

    // --- D2H Phase ---
    std::cout << "\n--- D2H Sweep ---" << std::endl;
    for (size_t size = 4096; size <= DMA_SIZE; size *= 2)
    {
        std::cout << "Size: " << (size / 1024) << " KiB... " << std::flush;
        
        for (int i = 0; i < iterations; ++i)
        {
            // --- Structural Synchronization & Queue Flush (perf_rdma barrier mode) ---
            mailbox->ready = 0;
            mailbox->type = 0;
            mailbox->direction = 2; // Special SYNC flag
            mailbox->size = 0;
            mailbox->ready = 1;
            
            coyote::rdmaSg sync_sg = {.local_offs = 0, .remote_offs = 0, .len = 64};
            coyote_thread.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sync_sg);
            
            while (!(mailbox->ready == 1 && mailbox->type == 1 && mailbox->direction == 2)) {
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }
            
            // Network quiet, remote side entering sync. Flush and TCP Barrier!
            coyote_thread.clearCompleted();
            coyote_thread.connSync(false);

            std::cout << "Iteration: " << i << "... " << std::flush;
            auto start = std::chrono::high_resolution_clock::now();
            mailbox->ready = 0;
            mailbox->type = 0;
            mailbox->size = size;
            mailbox->direction = 0; // D2H
            mailbox->ready = 1;

            coyote::rdmaSg d2h_sg = {.local_offs = 0, .remote_offs = 0, .len = 64};
            coyote_thread.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, d2h_sg);

            // The Server will PUSH the payload directly to our RAM via REMOTE_RDMA_WRITE, 
            // and then PUSH the completion mailbox. We just wait for the completion mailbox!
            while (!(mailbox->ready == 1 && mailbox->type == 1))
                ;

            auto end = std::chrono::high_resolution_clock::now();

            std::chrono::duration<double> diff = end - start;
            double time_s = diff.count();
            double latency_us = time_s * 1000000.0;
            double data_gb = (static_cast<double>(size) * 8.0) / (1024.0 * 1024.0 * 1024.0);
            double throughput_gbps = data_gb / time_s;

            results.push_back({size, "d2h", i, latency_us, throughput_gbps});
        }
        std::cout << "Done." << std::endl;
    }

    // --- H2D Phase ---
    std::cout << "\n--- H2D Sweep ---" << std::endl;
    for (size_t size = 4096; size <= DMA_SIZE; size *= 2)
    {
        std::cout << "Size: " << (size / 1024) << " KiB... " << std::flush;

        for (int i = 0; i < iterations; ++i)
        {
            // --- Structural Synchronization & Queue Flush (perf_rdma barrier mode) ---
            mailbox->ready = 0;
            mailbox->type = 0;
            mailbox->direction = 2; // Special SYNC flag
            mailbox->size = 0;
            mailbox->ready = 1;
            
            coyote::rdmaSg sync_sg = {.local_offs = 0, .remote_offs = 0, .len = 64};
            coyote_thread.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sync_sg);
            
            while (!(mailbox->ready == 1 && mailbox->type == 1 && mailbox->direction == 2)) {
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }
            
            // Network quiet, remote side entering sync. Flush and TCP Barrier!
            coyote_thread.clearCompleted();
            coyote_thread.connSync(false);

            auto start = std::chrono::high_resolution_clock::now();
            mailbox->ready = 0;
            mailbox->type = 0;
            mailbox->size = size;
            mailbox->direction = 1; // H2D
            mailbox->ready = 1;

            // Client PUSHES the payload to the Server!
            coyote::rdmaSg push_sg = {
                .local_offs = CONTROL_SIZE,
                .remote_offs = CONTROL_SIZE,
                .len = static_cast<uint32_t>(size)};
            coyote_thread.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, push_sg);

            // Then Client PUSHES the notification mailbox
            coyote::rdmaSg h2d_sg = {.local_offs = 0, .remote_offs = 0, .len = 64};
            coyote_thread.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, h2d_sg);

            // Wait for Server completion notification
            while (!(mailbox->ready == 1 && mailbox->type == 1))
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }
            auto end = std::chrono::high_resolution_clock::now();

            std::chrono::duration<double> diff = end - start;
            double time_s = diff.count();
            double latency_us = time_s * 1000000.0;
            double data_gb = (static_cast<double>(size) * 8.0) / (1024.0 * 1024.0 * 1024.0);
            double throughput_gbps = data_gb / time_s;

            results.push_back({size, "h2d", i, latency_us, throughput_gbps});
        }
        std::cout << "Done." << std::endl;
    }

    std::cout << "\nAll sweeps finished successfully." << std::endl;

    // CSV Output
    std::cout << "\nResults (size, direction, iteration, latency [us], throughput [Gbps]):" << std::endl;
    for (const auto &res : results)
    {
        std::cout << res.size << ", " << res.direction << ", " << res.iteration << ", "
                  << std::fixed << std::setprecision(3) << res.latency_us << ", "
                  << res.throughput_gbps << std::endl;
    }

    return EXIT_SUCCESS;
}
// test comment
// incremental test
