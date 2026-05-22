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

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <boost/program_options.hpp>

#include <coyote/cThread.hpp>
#include <immintrin.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
#define CLOCK_PERIOD_NS 4
#define DEFAULT_VFPGA_ID 0
#define RDMA_BUFFER_SIZE (2 * 1024 * 1024) // 2 MiB
static constexpr uint16_t DEBUG_PORT_OFFSET = 1;

// Trace replay buffer must hold the largest single transfer in traces.hpp
// (JIGSAW_TRACE_MAX_BYTES) plus the 4 KiB offset reserved at the start of
// the buffer for the protocol header. Rounded up to a 2 MiB hugepage
// multiple so HPF gets a clean allocation.
static constexpr uint64_t TRACE_DMA_OFFSET = 4096;

// ---------------------------------------------------------------------------
// AXI-Lite register map — jigsaw_hc_axi_ctrl_parser
// ---------------------------------------------------------------------------
// MMIO_FIFO_DROPPED is a sticky overflow flag exposed by the HW request
// FIFO in jigsaw_hc_axi_ctrl_parser.sv. Because write_mmio() no longer
// polls per request, SW has no per-push feedback if the FIFO ever fills
// and drops a trigger. HW sets bit 0 on any push that arrives while the
// FIFO is full, and SW clears by writing 0. With FIFO_DEPTH=32 and the
// current ≤7-deep MMIO bursts, this should never fire — read it once at
// end-of-test as a sanity check; nonzero means a lost setup write and
// unreliable results above. Index 34 = DEBUG_BASE_REG (10) + DEBUG_COUNT
// (24), placed after the read-only debug-counter region.
enum class HCReg : uint32_t
{
    MMIO_VADDR = 0,
    MMIO_CTRL = 1,
    MMIO_WRITE_STATUS = 2,
    MMIO_READ_STATUS = 3,
    COYOTE_PID = 4,
    REMOTE_VADDR = 5,
    MMIO_OP = 6,
    MMIO_ADDR = 7,
    MMIO_DATA = 8,
    MMIO_READ_DATA = 9,
    MMIO_FIFO_DROPPED = 34
};

// ---------------------------------------------------------------------------
// Results structures
// ---------------------------------------------------------------------------
struct DmaResult
{
    size_t size;
    std::string direction;
    int iteration;
    double latency_us;
    double mmio_setup_us;
    double throughput_gibps;
};

struct CompResult
{
    size_t size;
    uint64_t cycles;
    int iteration;
    double latency_us;
    double mmio_setup_us;
    double throughput_gibps;
};

// ---------------------------------------------------------------------------
// Device register map — payload_to_mmio on the device side
// ---------------------------------------------------------------------------
enum class DevReg : uint64_t
{
    DMA_CMD = 0x00,
    DMA_SRC_ADDR = 0x08,
    DMA_DST_ADDR = 0x10,
    DMA_H2D_LEN = 0x18,
    DMA_STATUS = 0x20,
    START_COMPUTE = 0x28,
    CYCLES_COMPUTE = 0x30,
    DMA_TX_LEN = 0x38,
    DMA_D2H_LEN = 0x40,
};

static bool g_trace_mmio = false;
static bool g_dump_host_debug = false;

static constexpr uint32_t HC_DEBUG_BASE = 10;
static constexpr uint32_t HC_DEBUG_COUNT = 24;

static const std::array<const char *, HC_DEBUG_COUNT> HC_DEBUG_NAMES = {
    "live_status",
    "mmio_start",
    "mmio_read_req_sent",
    "mmio_write_req_sent",
    "mmio_resp_recv",
    "mmio_write_done",
    "d2h_header_recv",
    "d2h_payload_beats",
    "d2h_payload_done",
    "sq_write_fire",
    "h2d_read_req_recv",
    "sq_read_fire",
    "h2d_reply_header_sent",
    "h2d_payload_beats_sent",
    "rdma_meta_fire",
    "rdma_final_beats",
    "rdma_sq_stall_cycles",
    "network_out_backpressure",
    "network_in_backpressure",
    "host_out_backpressure",
    "host_in_backpressure",
    "last_mmio_resp_data",
    "last_d2h_len",
    "last_h2d_len",
};

static void dump_host_debug_counters(coyote::cThread &ct, const std::string &label = "")
{
    std::cerr << "\n--- HOST LOCAL DEBUG CSRS";
    if (!label.empty())
    {
        std::cerr << " [" << label << "]";
    }
    std::cerr << " ---" << std::endl;
    for (uint32_t i = 0; i < HC_DEBUG_COUNT; i++)
    {
        uint64_t value = ct.getCSR(HC_DEBUG_BASE + i);
        std::cerr << "HC_DBG[" << std::setw(2) << i << "] "
                  << std::left << std::setw(26) << HC_DEBUG_NAMES[i]
                  << " = 0x" << std::hex << value << std::dec
                  << " (" << value << ")" << std::endl;
    }
    std::cerr << std::right;
}

static void dump_host_debug_after_run(coyote::cThread &ct, const std::string &label)
{
    if (g_dump_host_debug)
    {
        dump_host_debug_counters(ct, "DONE " + label);
    }
}

static void maybe_dump_host_debug(coyote::cThread &ct, uint64_t polls)
{
    if (g_dump_host_debug && (polls % 1000000 == 0))
    {
        dump_host_debug_counters(ct, "POLL WAIT");
    }
}

class DeviceDebugClient
{
public:
    ~DeviceDebugClient()
    {
        close();
    }

    void connect_to(const std::string &host, uint16_t port)
    {
        std::string port_str = std::to_string(port);
        addrinfo hints = {};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;

        addrinfo *result = nullptr;
        int rc = ::getaddrinfo(host.c_str(), port_str.c_str(), &hints, &result);
        if (rc != 0)
        {
            throw std::runtime_error(std::string("debug getaddrinfo failed: ") + gai_strerror(rc));
        }

        for (int attempt = 0; attempt < 100 && fd_ < 0; attempt++)
        {
            for (addrinfo *rp = result; rp != nullptr && fd_ < 0; rp = rp->ai_next)
            {
                int candidate = ::socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
                if (candidate < 0)
                {
                    continue;
                }
                if (::connect(candidate, rp->ai_addr, rp->ai_addrlen) == 0)
                {
                    fd_ = candidate;
                    break;
                }
                ::close(candidate);
            }

            if (fd_ < 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }

        ::freeaddrinfo(result);
        if (fd_ < 0)
        {
            throw std::runtime_error("could not connect to device debug server");
        }
    }

    bool connected() const
    {
        return fd_ >= 0;
    }

    void send_line(const std::string &line)
    {
        if (fd_ < 0)
        {
            return;
        }

        std::string payload = line + "\n";
        const char *ptr = payload.data();
        size_t remaining = payload.size();
        while (remaining != 0)
        {
            ssize_t written = ::send(fd_, ptr, remaining, MSG_NOSIGNAL);
            if (written <= 0)
            {
                throw std::runtime_error("could not write to device debug socket");
            }
            ptr += written;
            remaining -= static_cast<size_t>(written);
        }
    }

    void start_run(const std::string &label)
    {
        send_line("START " + label);
    }

    void done_run(const std::string &label)
    {
        send_line("DONE " + label);
    }

    void close()
    {
        if (fd_ >= 0)
        {
            try
            {
                send_line("STOP");
            }
            catch (...)
            {
            }
            ::close(fd_);
            fd_ = -1;
        }
    }

private:
    int fd_ = -1;
};

static std::string make_dma_debug_label(const char *direction, uint32_t size, int iteration)
{
    std::ostringstream os;
    os << "DMA direction=" << direction
       << " size=" << size
       << " iteration=" << iteration;
    return os.str();
}

static std::string make_comp_debug_label(uint32_t size, uint64_t cycles, int iteration)
{
    std::ostringstream os;
    os << "COMP size=" << size
       << " cycles=" << cycles
       << " iteration=" << iteration;
    return os.str();
}

static void trace_wait(const char *op, const char *phase, uint64_t addr, uint64_t polls)
{
    if (g_trace_mmio && (polls % 1000000 == 0))
    {
        std::cerr << op << " addr=0x" << std::hex << addr << std::dec
                  << " waiting in " << phase << ", polls=" << polls << std::endl;
    }
}

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
static uint64_t read_mmio(coyote::cThread &ct, void *mem, uint64_t addr)
{
    (void)mem;
    if (g_trace_mmio)
    {
        std::cerr << "MMIO READ begin addr=0x" << std::hex << addr << std::dec << std::endl;
    }

    // The pre-clear poll on READ_STATUS, the MMIO_CTRL idle wait, and the
    // post-trigger flush read have all been removed: with the parser
    // request FIFO providing ordering, the only load-bearing poll is the
    // final wait on READ_STATUS=1, which is set when the network response
    // packet arrives (mmio_read_done).
    ct.setCSR(0, static_cast<uint32_t>(HCReg::MMIO_READ_STATUS));
    ct.setCSR(0, static_cast<uint32_t>(HCReg::MMIO_OP)); // 0 = Read
    ct.setCSR(addr, static_cast<uint32_t>(HCReg::MMIO_ADDR));
    ct.setCSR(1, static_cast<uint32_t>(HCReg::MMIO_CTRL));

    uint64_t wait_polls = 0;
    while (ct.getCSR(static_cast<uint32_t>(HCReg::MMIO_READ_STATUS)) != 1)
    {
        trace_wait("MMIO READ", "read-done", addr, ++wait_polls);
        maybe_dump_host_debug(ct, wait_polls);
    }

    uint64_t value = ct.getCSR(static_cast<uint32_t>(HCReg::MMIO_READ_DATA));
    if (g_trace_mmio)
    {
        std::cerr << "MMIO READ done addr=0x" << std::hex << addr
                  << " value=0x" << value << std::dec << std::endl;
    }
    return value;
}

/**
 * @brief Write a device register over the jigsaw protocol.
 *
 * The HW-side request FIFO in jigsaw_hc_axi_ctrl_parser.sv decouples
 * submission from emission, so SW does not need to wait for completion
 * of each individual write: the FIFO preserves order and the
 * host_controller drains in order. The previous poll on
 * MMIO_WRITE_STATUS was the dominant cost (one ~2.5 us PCIe read per
 * write_mmio); dropping it — along with the pre-clear status poll, the
 * MMIO_CTRL idle wait, and the post-write flush read — collapses each
 * call to 4 posted PCIe writes.
 *
 * Reads still synchronize via read_mmio() because SW genuinely needs
 * the returned data, and the final DMA-completion poll on DMA_STATUS
 * implicitly waits for the entire enqueued write sequence to drain (the
 * status read can't be dispatched until all preceding FIFO entries are
 * consumed by the host_controller).
 */
static void write_mmio(coyote::cThread &ct, void *mem, uint64_t addr, uint64_t data)
{
    (void)mem;
    if (g_trace_mmio)
    {
        std::cerr << "MMIO WRITE begin addr=0x" << std::hex << addr
                  << " data=0x" << data << std::dec << std::endl;
    }

    ct.setCSR(1, static_cast<uint32_t>(HCReg::MMIO_OP)); // 1 = Write
    ct.setCSR(addr, static_cast<uint32_t>(HCReg::MMIO_ADDR));
    ct.setCSR(data, static_cast<uint32_t>(HCReg::MMIO_DATA));
    ct.setCSR(1, static_cast<uint32_t>(HCReg::MMIO_CTRL));

    if (g_trace_mmio)
    {
        std::cerr << "MMIO WRITE done addr=0x" << std::hex << addr << std::dec << std::endl;
    }
}

// ---------------------------------------------------------------------------
// Test routine (raw DMA benchmark)
// ---------------------------------------------------------------------------
static std::pair<double, double> run_test(coyote::cThread &ct, void *mem, uint32_t size, bool d2h)
{
    uint64_t mem_addr = reinterpret_cast<uint64_t>(mem);

    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_STATUS), 0);

    // Start timing before configuration MMIO calls
    auto start = std::chrono::high_resolution_clock::now();

    // --- Configure host-controller registers ---
    ct.setCSR(mem_addr, static_cast<uint32_t>(HCReg::MMIO_VADDR));

    uint64_t dma_dst = mem_addr + 4096;
    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_DST_ADDR), dma_dst);
    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_SRC_ADDR), dma_dst);
    write_mmio(ct, mem, static_cast<uint64_t>(d2h ? DevReg::DMA_D2H_LEN : DevReg::DMA_H2D_LEN), size);

    uint64_t cmd = d2h ? 3 : 1;
    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_CMD), cmd);

    // Capture time after MMIO setup is done
    auto mmio_done = std::chrono::high_resolution_clock::now();

    uint64_t status = 0;
    auto poll_count = 0;
    while (((status = read_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_STATUS))) & 0x1) != 1)
    {
        // Fast polling for benchmarks
        poll_count++;
        if (poll_count % 100000 == 0)
        {
            std::cout << "Polling DMA status, poll count: " << poll_count << std::endl;
        }
    }
    auto end = std::chrono::high_resolution_clock::now();

    uint64_t tx_len = read_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_TX_LEN));
    if (tx_len != size + 64 /* header */)
    {
        std::cerr << "DMA bytes mismatch: expected " << size
                  << ", got " << tx_len << std::endl;
    }

    std::chrono::duration<double> diff = end - start;
    std::chrono::duration<double> setup_diff = mmio_done - start;
    return {diff.count(), setup_diff.count()};
}

// ---------------------------------------------------------------------------
// Computation test: H2D → compute N cycles → D2H (all orchestrated by FPGA)
// ---------------------------------------------------------------------------
static std::pair<double, double> run_computation_test(coyote::cThread &ct, void *mem, uint32_t size, uint64_t compute_cycles)
{
    uint64_t mem_addr = reinterpret_cast<uint64_t>(mem);

    // Start timing before configuration MMIO calls
    auto start = std::chrono::high_resolution_clock::now();

    // --- Configure host-controller registers ---
    ct.setCSR(mem_addr, static_cast<uint32_t>(HCReg::MMIO_VADDR));

    uint64_t dma_dst = mem_addr + 4096;
    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_DST_ADDR), dma_dst);
    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_SRC_ADDR), dma_dst);
    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_H2D_LEN), size);
    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_D2H_LEN), size);

    // Set cycles per computation
    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::CYCLES_COMPUTE), compute_cycles);

    // Clear DMA status register (both bit 0 and bit 1)
    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_STATUS), 0);

    // Start computation: write 1 to START_COMPUTATION_REG
    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::START_COMPUTE), 1);

    // Capture time after MMIO setup is done
    auto mmio_done = std::chrono::high_resolution_clock::now();

    // Poll DMA_STATUS_REG for computation AND D2H DMA completion (both bits 0 and 1)
    uint64_t status = 0;
    uint64_t poll_count = 0;
    while (((status = read_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_STATUS))) & 0x3) != 0x3)
    {
        poll_count++;
        if (poll_count % 100000 == 0)
        {
            std::cout << "Polling computation status:" << status
                      << ", poll count: " << poll_count << std::endl;
        }
    }
    std::cout << "Computation status:" << status << std::endl;
    auto end = std::chrono::high_resolution_clock::now();

    // Clear status after polling
    write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_STATUS), 0);

    std::chrono::duration<double> diff = end - start;
    std::chrono::duration<double> setup_diff = mmio_done - start;
    return {diff.count(), setup_diff.count()};
}

// ---------------------------------------------------------------------------
// Trace Replay (bundled, embedded via traces.hpp)
//
// Mirrors the trace replay in jigsaw_baseline/sw_no_vm and jigsaw_minus_nw/sw_no_vm.
// Differences here: device registers are accessed over the jigsaw protocol
// via write_mmio/read_mmio (RDMA round-trip), and DevReg offsets are byte
// offsets into the device-side AXI-Lite map.
// ---------------------------------------------------------------------------
#include "traces.hpp"
using namespace jigsaw_traces_ns;

static constexpr uint32_t MIN_DMA_BYTES = 64;

// Round JIGSAW_TRACE_MAX_BYTES + TRACE_DMA_OFFSET up to a 2 MiB hugepage
// multiple so the HPF allocator can serve it as N contiguous hugepages.
static constexpr uint64_t HUGEPAGE_BYTES = 2ULL * 1024 * 1024;
static constexpr uint64_t TRACE_MEM_BYTES =
    ((JIGSAW_TRACE_MAX_BYTES + TRACE_DMA_OFFSET + HUGEPAGE_BYTES - 1) /
     HUGEPAGE_BYTES) * HUGEPAGE_BYTES;

static constexpr int TRACE_N_RUNS = 5;

struct TraceResult
{
    std::string app;
    int run_idx;
    int event_idx;
    uint8_t kind;
    uint64_t h2d_size, d2h_size, cycles;
    double total_us;
    int original_count;
};

static uint64_t clamp_dma(uint64_t raw, uint64_t mem_cap)
{
    uint64_t sz = (raw > mem_cap) ? mem_cap : raw;
    // Keep raw==0 as 0 so computation_engine skips that phase entirely.
    // Non-zero sizes are rounded UP to a cacheline multiple — host RDMA
    // fetches in 64-byte beats while the device DMA truncates to 64-byte
    // multiples, so a non-aligned length desyncs the meta/payload pipeline
    // (observed: h2d=74 wedges the device RDMA SQ).
    if (sz != 0)
    {
        sz = (sz + MIN_DMA_BYTES - 1) & ~static_cast<uint64_t>(MIN_DMA_BYTES - 1);
        if (sz > mem_cap)
        {
            sz = mem_cap & ~static_cast<uint64_t>(MIN_DMA_BYTES - 1);
        }
    }
    return sz;
}

static void run_trace_benchmark(coyote::cThread &ct, void *mem, uint64_t mem_bytes,
                                DeviceDebugClient &device_debug,
                                std::vector<TraceResult> &results)
{
    uint64_t mem_addr = reinterpret_cast<uint64_t>(mem);
    uint64_t dma_addr = mem_addr + TRACE_DMA_OFFSET;
    uint64_t dma_capacity = mem_bytes - TRACE_DMA_OFFSET;

    ct.setCSR(mem_addr, static_cast<uint32_t>(HCReg::MMIO_VADDR));

    using clk = std::chrono::high_resolution_clock;
    auto secs = [](auto a, auto b) { return std::chrono::duration<double>(b - a).count(); };

    // CYCLES_COMPUTE is a benchmark-only register (no analogue on a real
    // accelerator), so it's staged outside the timed window per BUNDLE event.
    // Per-event addresses and lengths stay inside since they would be set per
    // request in a real workload too.
    auto do_bulk = [&](uint32_t size, bool d2h) -> double {
        auto start = clk::now();
        write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_SRC_ADDR), dma_addr);
        write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_DST_ADDR), dma_addr);
        write_mmio(ct, mem,
                   static_cast<uint64_t>(d2h ? DevReg::DMA_D2H_LEN : DevReg::DMA_H2D_LEN),
                   size);
        write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_STATUS), 0);

        uint64_t cmd = d2h ? 3 : 1;
        write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_CMD), cmd);

        uint64_t poll_count = 0;
        while ((read_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_STATUS)) & 0x1) != 1)
        {
            maybe_dump_host_debug(ct, ++poll_count);
        }
        auto end = clk::now();
        return secs(start, end);
    };

    auto do_bundle = [&](uint32_t h2d, uint32_t d2h, uint64_t cycles) -> double {
        // Synthetic compute-cycle knob: not a real-device register, stage outside.
        write_mmio(ct, mem, static_cast<uint64_t>(DevReg::CYCLES_COMPUTE), cycles);
        const uint64_t cycles_readback = read_mmio(ct, mem, static_cast<uint64_t>(DevReg::CYCLES_COMPUTE));
        if (cycles_readback != cycles) {
            std::cerr << "[ERROR] CYCLES_COMPUTE readback mismatch: wrote "
                      << cycles << ", read " << cycles_readback << std::endl;
            throw std::runtime_error("CYCLES_COMPUTE readback mismatch");
        }

        auto start = clk::now();
        write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_SRC_ADDR), dma_addr);
        write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_DST_ADDR), dma_addr);
        write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_H2D_LEN), h2d);
        write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_D2H_LEN), d2h);
        write_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_STATUS), 0);

        write_mmio(ct, mem, static_cast<uint64_t>(DevReg::START_COMPUTE), 1);
        uint64_t poll_count = 0;
        while ((read_mmio(ct, mem, static_cast<uint64_t>(DevReg::DMA_STATUS)) & 0x3) != 0x3)
        {
            maybe_dump_host_debug(ct, ++poll_count);
        }
        auto end = clk::now();
        return secs(start, end);
    };

    auto kind_str = [](uint8_t k) -> const char * {
        switch (k) {
            case TRACE_BULK_H2D: return "BULK_H2D";
            case TRACE_BULK_D2H: return "BULK_D2H";
            case TRACE_BUNDLE:   return "BUNDLE  ";
            default:             return "UNKNOWN ";
        }
    };

    auto trace_debug_label = [&](const char *app_name, size_t event_idx, const char *kind,
                                 uint64_t h2d, uint64_t d2h, uint64_t cycles) {
        std::ostringstream os;
        os << "TRACE app=" << app_name
           << " event=" << event_idx
           << " kind=" << kind
           << " h2d=" << h2d
           << " d2h=" << d2h
           << " cycles=" << cycles;
        return os.str();
    };

    constexpr size_t n_apps = sizeof(jigsaw_traces) / sizeof(jigsaw_traces[0]);
    for (size_t a = 0; a < n_apps; a++)
    {
        const auto &app = jigsaw_traces[a];
        std::cerr << "[trace] " << app.name << " (" << app.n << " events, "
                  << TRACE_N_RUNS << " runs)" << std::endl;

      for (int run = 0; run < TRACE_N_RUNS; run++)
      {
        std::cerr << "[trace] " << app.name << " run " << run << "/" << TRACE_N_RUNS << std::endl;
        for (size_t i = 0; i < app.n; i++)
        {
            const auto &ev = app.events[i];
            double total_s = 0;
            uint64_t h2d = 0, d2h = 0;

            std::cerr << "[trace]   " << app.name
                      << " ev=" << i << "/" << app.n
                      << " kind=" << kind_str(ev.kind)
                      << " h2d_raw=" << ev.h2d_size
                      << " d2h_raw=" << ev.d2h_size
                      << " cycles=" << ev.cycles
                      << " ... " << std::flush;

            switch (ev.kind)
            {
            case TRACE_BULK_H2D:
                h2d = clamp_dma(ev.h2d_size, dma_capacity);
                std::cerr << "(h2d=" << h2d << ") " << std::flush;
                {
                    std::string debug_label = trace_debug_label(app.name, i, kind_str(ev.kind), h2d, 0, ev.cycles);
                    if (device_debug.connected())
                    {
                        device_debug.start_run(debug_label);
                    }
                    total_s = do_bulk(static_cast<uint32_t>(h2d), false);
                    if (device_debug.connected())
                    {
                        device_debug.done_run(debug_label);
                    }
                    dump_host_debug_after_run(ct, debug_label);
                }
                break;
            case TRACE_BULK_D2H:
                d2h = clamp_dma(ev.d2h_size, dma_capacity);
                std::cerr << "(d2h=" << d2h << ") " << std::flush;
                {
                    std::string debug_label = trace_debug_label(app.name, i, kind_str(ev.kind), 0, d2h, ev.cycles);
                    if (device_debug.connected())
                    {
                        device_debug.start_run(debug_label);
                    }
                    total_s = do_bulk(static_cast<uint32_t>(d2h), true);
                    if (device_debug.connected())
                    {
                        device_debug.done_run(debug_label);
                    }
                    dump_host_debug_after_run(ct, debug_label);
                }
                break;
            case TRACE_BUNDLE:
                h2d = clamp_dma(ev.h2d_size, dma_capacity);
                d2h = clamp_dma(ev.d2h_size, dma_capacity);
                // Device fires d2h_dma_start regardless of DMA_D2H_LEN, so a
                // 0-byte D2H still emits a meta packet through an already-
                // congested RDMA SQ and wedges the pipeline. Same for h2d.
                // Floor to a cacheline so the DMA carries real payload.
                // if (d2h == 0) d2h = MIN_DMA_BYTES;
                // if (h2d == 0) h2d = MIN_DMA_BYTES;
                std::cerr << "(h2d=" << h2d << " d2h=" << d2h << ") " << std::flush;
                {
                    std::string debug_label = trace_debug_label(app.name, i, kind_str(ev.kind), h2d, d2h, ev.cycles);
                    if (device_debug.connected())
                    {
                        device_debug.start_run(debug_label);
                    }
                    total_s = do_bundle(static_cast<uint32_t>(h2d), static_cast<uint32_t>(d2h), ev.cycles);
                    if (device_debug.connected())
                    {
                        device_debug.done_run(debug_label);
                    }
                    dump_host_debug_after_run(ct, debug_label);
                }
                break;
            default:
                std::cerr << "[warn] unknown kind " << (int)ev.kind << std::endl;
                continue;
            }

            std::cerr << "done " << std::fixed << std::setprecision(3)
                      << (total_s * 1e6) << " us" << std::endl;

            results.push_back({app.name, run, static_cast<int>(i), ev.kind,
                               ev.h2d_size, ev.d2h_size, ev.cycles,
                               total_s * 1e6,
                               ev.original_count});

        }
      }
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    std::string device_ip;

    bool skip_comp = false;
    bool trace_mmio = false;
    bool dump_host_debug = false;
    bool dump_device_debug = false;
    bool dump_device_debug_alias = false;
    boost::program_options::options_description opts("Jigsaw Host Controller Options");
    opts.add_options()("ip_address,i",
                       boost::program_options::value<std::string>(&device_ip),
                       "Device-side OOB TCP/IP address (for QP exchange)")("skip_comp,s",
                                                                           boost::program_options::bool_switch(&skip_comp),
                                                                           "Skip computation benchmarks")("trace_mmio",
                                                                                                           boost::program_options::bool_switch(&trace_mmio),
                                                                                                           "Trace each host-driven jigsaw MMIO transaction")("dump_host_debug",
                                                                                                                                                            boost::program_options::bool_switch(&dump_host_debug),
                                                                                                                                                            "Dump host-controller local debug CSRs while polling MMIO")("dump-device-debug",
                                                                                                                                                                                                                         boost::program_options::bool_switch(&dump_device_debug),
                                                                                                                                                                                                                         "Ask device software to dump debug CSRs after each benchmark run")("dump_device_debug",
                                                                                                                                                                                                                                                                                         boost::program_options::bool_switch(&dump_device_debug_alias),
                                                                                                                                                                                                                                                                                         "Alias for --dump-device-debug");

    boost::program_options::variables_map vm;
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, argv, opts), vm);
    boost::program_options::notify(vm);

    if (device_ip.empty())
    {
        std::cerr << "ERROR: --ip_address (-i) is required\n"
                  << opts << std::endl;
        return EXIT_FAILURE;
    }
    dump_device_debug = dump_device_debug || dump_device_debug_alias;
    g_trace_mmio = trace_mmio;
    g_dump_host_debug = dump_host_debug;

    // HEADER("JIGSAW HOST CONTROLLER");
    std::cout << "Device OOB IP : " << device_ip << std::endl;
    std::cout << std::endl;

    // Create Coyote thread
    coyote::cThread ct(DEFAULT_VFPGA_ID, getpid());

    // Initialise RDMA — allocates hugepage buffer, exchanges QPs
    void *mem = ct.initRDMA(RDMA_BUFFER_SIZE, coyote::DEF_PORT, device_ip.c_str());
    if (!mem)
    {
        throw std::runtime_error("initRDMA failed — could not allocate memory");
    }

    // Configure hardware PID
    ct.setCSR(ct.getCtid(), static_cast<uint32_t>(HCReg::COYOTE_PID));

    std::cout << "RDMA connection established (PID " << ct.getCtid() << ")." << std::endl;

    // Write remote buffer address to HW for RDMA WRITE targeting
    uint64_t remote_vaddr = (uint64_t)ct.getQpair()->remote.vaddr;
    ct.setCSR(remote_vaddr, static_cast<uint32_t>(HCReg::REMOTE_VADDR));
    std::cout << "  Remote VADDR = 0x" << std::hex << remote_vaddr << std::dec << std::endl;
    std::cout << std::endl;

    DeviceDebugClient device_debug;
    if (dump_device_debug)
    {
        device_debug.connect_to(device_ip, static_cast<uint16_t>(coyote::DEF_PORT + DEBUG_PORT_OFFSET));
        std::cout << "Device debug sideband connected." << std::endl;
    }

    // Sync with device before starting
    ct.connSync(true);

    // Size jigsaw_mem to fit the largest single transfer in the embedded
    // trace set (JIGSAW_TRACE_MAX_BYTES) plus the 4 KiB DMA offset. The
    // earlier 2 MiB allocation forced clamp_dma() to truncate every event
    // larger than 2 MiB, so the trace benchmark was not actually exercising
    // the full per-event sizes recorded in traces.hpp.
    int *jigsaw_mem = (int *)ct.getMem({coyote::CoyoteAllocType::HPF,
                                        static_cast<uint32_t>(TRACE_MEM_BYTES)});
    if (!jigsaw_mem)
    {
        throw std::runtime_error("Could not allocate memory; exiting...");
    }

    const uint64_t dma_offset = TRACE_DMA_OFFSET;

    const int n_runs = 5;
    std::vector<DmaResult> dma_results;
    std::vector<CompResult> comp_results;

    // =========================================================================
    // DMA-only benchmarks (backward compatible)
    // =========================================================================
    std::cout << "=== DMA BENCHMARKS ===" << std::endl;

    // H2D Tests
    for (uint32_t size = 4096; size <= 1048576; size *= 2)
    {
        std::cout << "H2D Size: " << (size / 1024) << " KiB... " << std::flush;
        for (int i = 0; i < n_runs; i++)
        {
            std::string debug_label = make_dma_debug_label("h2d", size, i);
            if (device_debug.connected())
            {
                device_debug.start_run(debug_label);
            }
            auto [time_s, setup_s] = run_test(ct, jigsaw_mem, size, false); // H2D
            if (device_debug.connected())
            {
                device_debug.done_run(debug_label);
            }
            dump_host_debug_after_run(ct, debug_label);
            double latency_us = time_s * 1000000.0;
            double setup_us = setup_s * 1000000.0;
            double throughput_gibps = (static_cast<double>(size) / (1024.0 * 1024.0 * 1024.0)) / time_s;
            dma_results.push_back({size, "h2d", i, latency_us, setup_us, throughput_gibps});
        }
        std::cout << "Done." << std::endl;
    }

    // D2H Tests
    for (uint32_t size = 4096; size <= 1048576; size *= 2)
    {
        std::cout << "D2H Size: " << (size / 1024) << " KiB... " << std::flush;
        for (int i = 0; i < n_runs; i++)
        {
            std::string debug_label = make_dma_debug_label("d2h", size, i);
            if (device_debug.connected())
            {
                device_debug.start_run(debug_label);
            }
            auto [time_s, setup_s] = run_test(ct, jigsaw_mem, size, true); // D2H
            if (device_debug.connected())
            {
                device_debug.done_run(debug_label);
            }
            dump_host_debug_after_run(ct, debug_label);
            double latency_us = time_s * 1000000.0;
            double setup_us = setup_s * 1000000.0;
            double throughput_gibps = (static_cast<double>(size) / (1024.0 * 1024.0 * 1024.0)) / time_s;
            dma_results.push_back({size, "d2h", i, latency_us, setup_us, throughput_gibps});

            // Memory verification
            unsigned char *check_ptr = reinterpret_cast<unsigned char *>(jigsaw_mem) + dma_offset;
            bool all_ones = true;
            for (uint64_t j = 0; j < size; j++)
            {
                if (check_ptr[j] != 0xFF)
                {
                    all_ones = false;
                    break;
                }
            }
            if (!all_ones)
            {
                std::cout << "  ERROR: Memory mismatch for size " << size << " on run " << i << std::endl;
            }
            std::memset(check_ptr, 0, size);
        }
        std::cout << "Done." << std::endl;
    }

    std::cout << std::endl;

    if (!skip_comp)
    {
        // =========================================================================
        // Full computation pipeline benchmarks (H2D → compute → D2H)
        // =========================================================================
        std::cout << "=== COMPUTATION BENCHMARKS ===" << std::endl;

        const uint64_t cycle_counts[] = {
            100,    // 100 cycles  = 400 ns @ 250 MHz
            1000,   // 1K cycles   = 4 us
            10000,  // 10K cycles  = 40 us
            100000, // 100K cycles = 400 us
            1000000 // 1M cycles   = 4 ms
        };
        const int n_cycle_counts = sizeof(cycle_counts) / sizeof(cycle_counts[0]);

        for (uint32_t size = 4096; size <= 1024 * 1024; size *= 2)
        {
            std::cout << "Comp Size: " << (size / 1024) << " KiB... " << std::flush;
            for (int c = 0; c < n_cycle_counts; c++)
            {
                uint64_t cycles = cycle_counts[c];
                for (int i = 0; i < n_runs; i++)
                {
                    if (g_trace_mmio)
                    {
                        std::cerr << "BEGIN computation size=" << size
                                  << " cycles=" << cycles
                                  << " iteration=" << i << std::endl;
                    }
                    std::string debug_label = make_comp_debug_label(size, cycles, i);
                    if (device_debug.connected())
                    {
                        device_debug.start_run(debug_label);
                    }
                    auto [time_s, setup_s] = run_computation_test(ct, jigsaw_mem, size, cycles);
                    if (device_debug.connected())
                    {
                        device_debug.done_run(debug_label);
                    }
                    dump_host_debug_after_run(ct, debug_label);
                    double latency_us = time_s * 1000000.0;
                    double setup_us = setup_s * 1000000.0;
                    double throughput_gibps = (static_cast<double>(size) / (1024.0 * 1024.0 * 1024.0)) / time_s;
                    std::cout << "Size: " << (size) << " bytes, Cycles: " << cycles << ", Iteration: " << i << ", Latency: " << latency_us << " us, Throughput: " << throughput_gibps << " GiBps" << std::endl;
                    comp_results.push_back({size, cycles, i, latency_us, setup_us, throughput_gibps});
                }
            }
            std::cout << "Done." << std::endl;
        }
    }

    // =========================================================================
    // Trace Benchmark (embedded apps from traces.hpp)
    // =========================================================================
    std::cout << "\n=== TRACE BENCHMARK ===" << std::endl;
    {
        std::vector<TraceResult> trace_results;
        run_trace_benchmark(ct, jigsaw_mem, TRACE_MEM_BYTES, device_debug, trace_results);

        struct AppAgg
        {
            int n_events = 0;
            int n_bulk_h2d = 0, n_bulk_d2h = 0, n_bundle = 0;
            double total_h2d_us = 0, total_d2h_us = 0, total_bundle_us = 0;
            uint64_t total_h2d_bytes = 0, total_d2h_bytes = 0, total_cycles = 0;
        };
        struct AppKey
        {
            std::string app;
            int run_idx;
        };
        std::vector<std::pair<AppKey, AppAgg>> ordered;
        std::map<std::pair<std::string, int>, size_t> idx;
        for (const auto &r : trace_results)
        {
            auto key = std::make_pair(r.app, r.run_idx);
            auto it = idx.find(key);
            if (it == idx.end())
            {
                idx[key] = ordered.size();
                ordered.push_back({{r.app, r.run_idx}, AppAgg{}});
                it = idx.find(key);
            }
            auto &a = ordered[it->second].second;
            a.n_events++;
            switch (r.kind)
            {
            case TRACE_BULK_H2D:
                a.n_bulk_h2d++;
                a.total_h2d_us += r.total_us;
                a.total_h2d_bytes += r.h2d_size;
                break;
            case TRACE_BULK_D2H:
                a.n_bulk_d2h++;
                a.total_d2h_us += r.total_us;
                a.total_d2h_bytes += r.d2h_size;
                break;
            case TRACE_BUNDLE:
                a.n_bundle++;
                a.total_bundle_us += r.total_us;
                a.total_h2d_bytes += r.h2d_size;
                a.total_d2h_bytes += r.d2h_size;
                a.total_cycles += r.cycles;
                break;
            }
        }

        std::cout << std::endl
                  << "# per-app summary (one row per (app, run))" << std::endl;
        std::cout << "app, run, n_events, n_bulk_h2d, n_bulk_d2h, n_bundle, "
                  << "total_h2d_us, total_d2h_us, total_bundle_us, total_us, "
                  << "total_h2d_bytes, total_d2h_bytes, total_cycles" << std::endl;
        for (const auto &p : ordered)
        {
            const auto &a = p.second;
            double total_us = a.total_h2d_us + a.total_d2h_us + a.total_bundle_us;
            std::cout << p.first.app << ", " << p.first.run_idx << ", "
                      << a.n_events << ", "
                      << a.n_bulk_h2d << ", " << a.n_bulk_d2h << ", " << a.n_bundle << ", "
                      << std::fixed << std::setprecision(3)
                      << a.total_h2d_us << ", " << a.total_d2h_us << ", "
                      << a.total_bundle_us << ", " << total_us << ", "
                      << a.total_h2d_bytes << ", " << a.total_d2h_bytes << ", "
                      << a.total_cycles << std::endl;
        }
    }

    // =========================================================================
    // Final Results Summary
    // =========================================================================
    std::cout << "\nResults (DMA-only) (size, direction, iteration, latency [us], mmio_setup [us], throughput [GiBps]):" << std::endl;
    for (const auto &res : dma_results)
    {
        std::cout << res.size << ", " << res.direction << ", " << res.iteration << ", "
                  << std::fixed << std::setprecision(3) << res.latency_us << ", "
                  << res.mmio_setup_us << ", "
                  << res.throughput_gibps << std::endl;
    }

    if (!skip_comp)
    {
        std::cout << "\nResults (Computation) (size, cycles, iteration, latency [us], mmio_setup [us], throughput [GiBps]):" << std::endl;
        for (const auto &res : comp_results)
        {
            std::cout << res.size << ", " << res.cycles << ", " << res.iteration << ", "
                      << std::fixed << std::setprecision(3) << res.latency_us << ", "
                      << res.mmio_setup_us << ", "
                      << res.throughput_gibps << std::endl;
        }
    }

    device_debug.close();

    // FIFO overflow sanity check (see HCReg comment). A nonzero value
    // means at least one MMIO push was dropped during this run —
    // measurements above are unreliable.
    uint64_t fifo_dropped = ct.getCSR(static_cast<uint32_t>(HCReg::MMIO_FIFO_DROPPED));
    if (fifo_dropped != 0) {
        std::cerr << "[WARNING] MMIO FIFO overflow detected (MMIO_FIFO_DROPPED="
                  << fifo_dropped << "). Some setup writes were dropped; "
                  << "results above are unreliable." << std::endl;
    }

    // Sync with device after finishing
    ct.connSync(true);

    // std::cout << "All tests completed." << std::endl;
    ct.closeConn();

    return EXIT_SUCCESS;
}
