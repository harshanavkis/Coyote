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

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <coyote/cThread.hpp>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
#define DEFAULT_VFPGA_ID     0
#define RDMA_BUFFER_SIZE     (2048 * 1024)  // 1 MiB
static constexpr uint16_t DEBUG_PORT_OFFSET = 1;

// ---------------------------------------------------------------------------
// AXI-Lite register map — jigsaw_dc_axi_ctrl_parser
// ---------------------------------------------------------------------------
enum class DCReg : uint32_t {
    COYOTE_PID          = 0,
    REMOTE_VADDR        = 1,
};

static constexpr uint32_t DC_DEBUG_BASE = 2;
static constexpr uint32_t DC_DEBUG_COUNT = 24;

static const std::array<const char *, DC_DEBUG_COUNT> DC_DEBUG_NAMES = {
    "live_status",
    "mmio_rd_req_recv",
    "mmio_wr_req_recv",
    "mmio_resp_pending_cycles",
    "mmio_resp_sent",
    "h2d_reply_header_recv",
    "h2d_payload_beats_recv",
    "h2d_payload_done",
    "dma_start",
    "h2d_dma_start",
    "d2h_dma_start",
    "dma_clear_status",
    "dma_done_status",
    "start_compute_seen",
    "compute_done_status",
    "rdma_meta_fire",
    "rdma_first_beats",
    "rdma_final_beats",
    "rdma_sq_stall_cycles",
    "output_backpressure",
    "input_backpressure",
    "last_dma_len",
    "last_dma_tx_len",
    "last_rdma_pkt_len",
};

static void dump_debug_counters(const coyote::cThread &ct, const std::string &label = "") {
    std::cout << "\n--- DEVICE LOCAL DEBUG CSRS";
    if (!label.empty()) {
        std::cout << " [" << label << "]";
    }
    std::cout << " ---" << std::endl;
    for (uint32_t i = 0; i < DC_DEBUG_COUNT; i++) {
        uint64_t value = ct.getCSR(DC_DEBUG_BASE + i);
        std::cout << "DC_DBG[" << std::setw(2) << i << "] "
                  << std::left << std::setw(26) << DC_DEBUG_NAMES[i]
                  << " = 0x" << std::hex << value << std::dec
                  << " (" << value << ")" << std::endl;
    }
    std::cout << std::right;
}

static bool starts_with(const std::string &value, const char *prefix) {
    return value.rfind(prefix, 0) == 0;
}

static int make_debug_server_socket(uint16_t port) {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        throw std::runtime_error("debug socket creation failed");
    }

    int one = 1;
    if (::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) < 0) {
        ::close(fd);
        throw std::runtime_error("debug socket setsockopt failed");
    }

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        throw std::runtime_error("debug socket bind failed");
    }
    if (::listen(fd, 1) < 0) {
        ::close(fd);
        throw std::runtime_error("debug socket listen failed");
    }
    return fd;
}

static void process_debug_message(const coyote::cThread &ct,
                                  const std::string &message,
                                  bool &run_active,
                                  std::string &active_label,
                                  std::chrono::steady_clock::time_point &next_watchdog,
                                  std::chrono::microseconds watchdog_interval,
                                  std::atomic<bool> &keep_running) {
    if (starts_with(message, "START ")) {
        active_label = message.substr(6);
        run_active = true;
        next_watchdog = std::chrono::steady_clock::now() + watchdog_interval;
        std::cout << "\n--- DEVICE DEBUG RUN START [" << active_label << "] ---" << std::endl;
    } else if (starts_with(message, "DONE ")) {
        std::string label = message.substr(5);
        dump_debug_counters(ct, "DONE " + label);
        run_active = false;
    } else if (starts_with(message, "DUMP ")) {
        dump_debug_counters(ct, message.substr(5));
    } else if (message == "STOP") {
        keep_running.store(false);
    }
}

static void debug_server_loop(const coyote::cThread &ct,
                              std::atomic<bool> &keep_running,
                              std::atomic<bool> &server_ready,
                              std::atomic<bool> &server_failed,
                              uint16_t port,
                              uint64_t watchdog_us) {
    int listen_fd = -1;
    int client_fd = -1;
    std::string rx_buffer;
    bool run_active = false;
    std::string active_label;
    const auto watchdog_interval = std::chrono::microseconds(watchdog_us);
    auto next_watchdog = std::chrono::steady_clock::now() + watchdog_interval;

    try {
        listen_fd = make_debug_server_socket(port);
        std::cout << "Device debug server listening on port " << port << std::endl;
        server_ready.store(true);

        while (keep_running.load()) {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(listen_fd, &readfds);
            int max_fd = listen_fd;
            if (client_fd >= 0) {
                FD_SET(client_fd, &readfds);
                if (client_fd > max_fd) {
                    max_fd = client_fd;
                }
            }

            timeval tv = {};
            tv.tv_sec = 0;
            tv.tv_usec = 100000;
            int rc = ::select(max_fd + 1, &readfds, nullptr, nullptr, &tv);
            if (rc < 0) {
                if (errno == EINTR) {
                    continue;
                }
                throw std::runtime_error("debug socket select failed");
            }

            if (run_active && std::chrono::steady_clock::now() >= next_watchdog) {
                dump_debug_counters(ct, "WATCHDOG " + active_label);
                next_watchdog = std::chrono::steady_clock::now() + watchdog_interval;
            }

            if (rc == 0) {
                continue;
            }

            if (FD_ISSET(listen_fd, &readfds)) {
                int new_fd = ::accept(listen_fd, nullptr, nullptr);
                if (new_fd >= 0) {
                    if (client_fd >= 0) {
                        ::close(client_fd);
                    }
                    client_fd = new_fd;
                    rx_buffer.clear();
                    std::cout << "Device debug client connected." << std::endl;
                }
            }

            if (client_fd >= 0 && FD_ISSET(client_fd, &readfds)) {
                char data[512];
                ssize_t n = ::recv(client_fd, data, sizeof(data), 0);
                if (n <= 0) {
                    ::close(client_fd);
                    client_fd = -1;
                    rx_buffer.clear();
                    continue;
                }

                rx_buffer.append(data, static_cast<size_t>(n));
                size_t pos = 0;
                while ((pos = rx_buffer.find('\n')) != std::string::npos) {
                    std::string line = rx_buffer.substr(0, pos);
                    rx_buffer.erase(0, pos + 1);
                    if (!line.empty() && line.back() == '\r') {
                        line.pop_back();
                    }
                    process_debug_message(ct, line, run_active, active_label,
                                          next_watchdog, watchdog_interval, keep_running);
                }
            }
        }
    } catch (const std::exception &e) {
        server_failed.store(true);
        std::cerr << "Device debug server error: " << e.what() << std::endl;
    }

    if (client_fd >= 0) {
        ::close(client_fd);
    }
    if (listen_fd >= 0) {
        ::close(listen_fd);
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[]) {
    bool dump_debug = false;
    uint64_t dump_debug_us = 1000000;

    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--dump-debug") == 0) {
            dump_debug = true;
        } else if (std::strcmp(argv[i], "--dump-debug-us") == 0) {
            dump_debug = true;
            if (i + 1 < argc && std::strncmp(argv[i + 1], "--", 2) != 0) {
                dump_debug_us = std::stoull(argv[++i]);
                if (dump_debug_us == 0) {
                    dump_debug_us = 1;
                }
            }
        } else if (std::strncmp(argv[i], "--dump-debug-us=", 16) == 0) {
            dump_debug = true;
            dump_debug_us = std::stoull(argv[i] + 16);
            if (dump_debug_us == 0) {
                dump_debug_us = 1;
            }
        } else {
            std::cerr << "Usage: " << argv[0]
                      << " [--dump-debug] [--dump-debug-us <us>]"
                      << std::endl;
            return EXIT_FAILURE;
        }
    }

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

    std::atomic<bool> keep_dumping{dump_debug};
    std::atomic<bool> debug_server_ready{!dump_debug};
    std::atomic<bool> debug_server_failed{false};
    std::thread debug_thread;
    if (dump_debug) {
        debug_thread = std::thread(debug_server_loop,
                                   std::cref(ct),
                                   std::ref(keep_dumping),
                                   std::ref(debug_server_ready),
                                   std::ref(debug_server_failed),
                                   static_cast<uint16_t>(coyote::DEF_PORT + DEBUG_PORT_OFFSET),
                                   dump_debug_us);
        while (!debug_server_ready.load() && !debug_server_failed.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (debug_server_failed.load()) {
            keep_dumping.store(false);
            if (debug_thread.joinable()) {
                debug_thread.join();
            }
            throw std::runtime_error("device debug server failed to start");
        }
    }

    // Signal host that we are ready
    ct.connSync(false);
    std::cout << "Host connected — device ready, HW handles all traffic." << std::endl;

    // Wait for host to signal completion
    ct.connSync(false);
    keep_dumping.store(false);
    if (debug_thread.joinable()) {
        debug_thread.join();
    }
    std::cout << "Host signalled completion." << std::endl;

    ct.closeConn();
    std::cout << "Connection closed. Exiting." << std::endl;

    return EXIT_SUCCESS;
}
