/**
 * Jigsaw Software Forwarder — host-side trace harness (no VM)
 *
 * Drives the same device interactions as jigsaw_host_controller/sw_no_vm
 * (Vortex OpenCL trace replay), but through the software
 * forwarding path instead of the jigsaw host-controller hardware: every
 * MMIO access is encapsulated into a wire message and carried over the
 * Coyote RDMA stack (perf_rdma as a dumb NIC) to the device-side replayer,
 * which replays it on the unmodified jigsaw_baseline accelerator.
 *
 * DMA payloads bounce through two staging copies (application buffer ->
 * NIC buffer here, NIC buffer -> device buffer on the device node), the
 * cost a software forwarder on commodity hardware cannot avoid.
 *
 * Usage:
 *   ./test -i <device_oob_ip> [-r <trace runs>]
 */

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>

#include <boost/program_options.hpp>

#include <coyote/cThread.hpp>

#include "mailbox.hpp"

using namespace jsfwd;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
#define DEFAULT_VFPGA_ID 0

// ---------------------------------------------------------------------------
// Trace Replay (bundled, embedded via traces.hpp)
// ---------------------------------------------------------------------------
#include "traces.hpp"
using namespace jigsaw_traces_ns;

static constexpr uint32_t MIN_DMA_BYTES = 64;

struct TraceResult {
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
    // Keep raw==0 as 0 so the computation engine skips that phase entirely.
    if (sz != 0) {
        sz = (sz + MIN_DMA_BYTES - 1) & ~static_cast<uint64_t>(MIN_DMA_BYTES - 1);
        if (sz > mem_cap) {
            sz = mem_cap & ~static_cast<uint64_t>(MIN_DMA_BYTES - 1);
        }
    }
    return sz;
}

// ---------------------------------------------------------------------------
// Benchmarks — same MMIO sequences as jigsaw_host_controller/sw_no_vm, with
// write_mmio/read_mmio going through the forwarder instead of the HC vFPGA.
// ---------------------------------------------------------------------------
using clk = std::chrono::high_resolution_clock;
static double secs(clk::time_point a, clk::time_point b)
{
    return std::chrono::duration<double>(b - a).count();
}

// Bulk transfers are sliced into 1 MiB chunks with a full MMIO sequence per
// chunk — mirroring the guest driver (my_qemu_edu.c TRACE_CHUNK_BYTES): the
// system's established convention, since >1 MiB single transfers are not
// reliable on either forwarding path. Timing covers the whole chunked loop;
// bundles are never chunked (their transfers are tiny by construction).
static constexpr uint64_t TRACE_CHUNK_BYTES = 1ULL << 20;

static double do_bulk(HostForwarder &fw, uint64_t dma_addr, uint32_t size, bool d2h)
{
    auto start = clk::now();
    uint64_t remaining = size, off = 0;
    while (remaining > 0) {
        uint64_t chunk = remaining > TRACE_CHUNK_BYTES ? TRACE_CHUNK_BYTES : remaining;
        fw.mmio_write(static_cast<uint64_t>(DevReg::DMA_SRC_ADDR), dma_addr + off);
        fw.mmio_write(static_cast<uint64_t>(DevReg::DMA_DST_ADDR), dma_addr + off);
        fw.mmio_write(static_cast<uint64_t>(d2h ? DevReg::DMA_D2H_LEN : DevReg::DMA_H2D_LEN),
                      chunk);
        fw.mmio_write(static_cast<uint64_t>(DevReg::DMA_STATUS), 0);
        fw.mmio_write(static_cast<uint64_t>(DevReg::DMA_CMD),
                      d2h ? DMA_CMD_D2H : DMA_CMD_H2D);
        while ((fw.mmio_read(static_cast<uint64_t>(DevReg::DMA_STATUS)) & 0x1) != 1) {
        }
        fw.mmio_write(static_cast<uint64_t>(DevReg::DMA_STATUS), 0);
        off += chunk;
        remaining -= chunk;
    }
    auto end = clk::now();
    return secs(start, end);
}

static double do_bundle(HostForwarder &fw, uint64_t dma_addr, uint32_t h2d,
                        uint32_t d2h, uint64_t cycles)
{
    // Synthetic compute-cycle knob: not a real-device register, stage outside
    // the timed window (same as sw_no_vm).
    fw.mmio_write(static_cast<uint64_t>(DevReg::CYCLES_COMPUTE), cycles);
    const uint64_t cycles_readback =
        fw.mmio_read(static_cast<uint64_t>(DevReg::CYCLES_COMPUTE));
    if (cycles_readback != cycles) {
        std::cerr << "[ERROR] CYCLES_COMPUTE readback mismatch: wrote " << cycles
                  << ", read " << cycles_readback << std::endl;
        throw std::runtime_error("CYCLES_COMPUTE readback mismatch");
    }

    auto start = clk::now();
    fw.mmio_write(static_cast<uint64_t>(DevReg::DMA_SRC_ADDR), dma_addr);
    fw.mmio_write(static_cast<uint64_t>(DevReg::DMA_DST_ADDR), dma_addr);
    fw.mmio_write(static_cast<uint64_t>(DevReg::DMA_H2D_LEN), h2d);
    fw.mmio_write(static_cast<uint64_t>(DevReg::DMA_D2H_LEN), d2h);
    fw.mmio_write(static_cast<uint64_t>(DevReg::DMA_STATUS), 0);

    fw.mmio_write(static_cast<uint64_t>(DevReg::START_COMPUTE), 1);
    while ((fw.mmio_read(static_cast<uint64_t>(DevReg::DMA_STATUS)) & 0x3) != 0x3) {
    }
    auto end = clk::now();
    return secs(start, end);
}

static void run_traces(HostForwarder &fw, uint64_t dma_addr, uint64_t dma_capacity,
                       int n_runs, std::vector<TraceResult> &results)
{
    auto kind_str = [](uint8_t k) -> const char * {
        switch (k) {
            case TRACE_BULK_H2D: return "BULK_H2D";
            case TRACE_BULK_D2H: return "BULK_D2H";
            case TRACE_BUNDLE:   return "BUNDLE  ";
            default:             return "UNKNOWN ";
        }
    };

    constexpr size_t n_apps = sizeof(jigsaw_traces) / sizeof(jigsaw_traces[0]);
    for (size_t a = 0; a < n_apps; a++) {
        const auto &app = jigsaw_traces[a];
        std::cerr << "[trace] " << app.name << " (" << app.n << " events, "
                  << n_runs << " runs)" << std::endl;

        for (int run = 0; run < n_runs; run++) {
            for (size_t i = 0; i < app.n; i++) {
                const auto &ev = app.events[i];
                double total_s = 0;
                uint64_t h2d = 0, d2h = 0;

                std::cerr << "[trace]   " << app.name << " ev=" << i << "/" << app.n
                          << " kind=" << kind_str(ev.kind)
                          << " h2d=" << ev.h2d_size << " d2h=" << ev.d2h_size
                          << " cycles=" << ev.cycles << " ... " << std::flush;

                switch (ev.kind) {
                case TRACE_BULK_H2D:
                    h2d = clamp_dma(ev.h2d_size, dma_capacity);
                    total_s = do_bulk(fw, dma_addr, static_cast<uint32_t>(h2d), false);
                    break;
                case TRACE_BULK_D2H:
                    d2h = clamp_dma(ev.d2h_size, dma_capacity);
                    total_s = do_bulk(fw, dma_addr, static_cast<uint32_t>(d2h), true);
                    break;
                case TRACE_BUNDLE:
                    h2d = clamp_dma(ev.h2d_size, dma_capacity);
                    d2h = clamp_dma(ev.d2h_size, dma_capacity);
                    total_s = do_bundle(fw, dma_addr, static_cast<uint32_t>(h2d),
                                        static_cast<uint32_t>(d2h), ev.cycles);
                    break;
                default:
                    std::cerr << "[warn] unknown kind " << (int)ev.kind << std::endl;
                    continue;
                }

                std::cerr << "done " << (total_s * 1e6) << " us" << std::endl;

                results.push_back({app.name, run, static_cast<int>(i), ev.kind,
                                   ev.h2d_size, ev.d2h_size, ev.cycles,
                                   total_s * 1e6, ev.original_count});

                // full quiesce between events, outside the timed window —
                // the May-baseline discipline that keeps the stack healthy
                fw.sync();
            }
            std::cerr << "[trace] " << app.name << " run " << run << " done"
                      << std::endl;
        }
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    std::string device_ip;
    int trace_runs = 5;

    boost::program_options::options_description opts("Jigsaw SW Forwarder (no VM) Options");
    opts.add_options()
        ("ip_address,i",
            boost::program_options::value<std::string>(&device_ip),
            "Device-side OOB TCP/IP address (for QP exchange)")
        ("trace_runs,r",
            boost::program_options::value<int>(&trace_runs),
            "Runs per trace application");

    boost::program_options::variables_map vm;
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, argv, opts), vm);
    boost::program_options::notify(vm);

    if (device_ip.empty()) {
        std::cout << "ERROR: --ip_address (-i) is required\n" << opts << std::endl;
        return EXIT_FAILURE;
    }

    HEADER("JIGSAW SW FORWARDER — HOST (NO VM)");
    std::cout << "Device OOB IP : " << device_ip << std::endl;

    coyote::cThread ct(DEFAULT_VFPGA_ID, getpid());

    char *nic = static_cast<char *>(
        ct.initRDMA(BUF_BYTES, coyote::DEF_PORT, device_ip.c_str()));
    if (!nic) {
        std::cerr << "initRDMA failed" << std::endl;
        return EXIT_FAILURE;
    }
    memset(nic, 0, CTRL_BYTES);

    // Application buffer standing in for the guest/ivshmem memory the
    // forwarder serves; payload region mirrors the QP buffer layout.
    char *app = nullptr;
    if (posix_memalign(reinterpret_cast<void **>(&app), 4096, BUF_BYTES) != 0) {
        std::cerr << "application buffer allocation failed" << std::endl;
        return EXIT_FAILURE;
    }
    memset(app, 0xAB, BUF_BYTES);  // pre-fault + recognizable payload

    // Barrier: both sides have zeroed their control pages
    ct.connSync(true);
    std::cout << "RDMA connection established." << std::endl;

    HostForwarder fw(ct, nic, app);
    fw.send_setup();

    uint64_t dma_addr = reinterpret_cast<uint64_t>(app) + PAYLOAD_OFF;
    uint64_t dma_capacity = BUF_BYTES - PAYLOAD_OFF;

    std::vector<TraceResult> trace_results;

    run_traces(fw, dma_addr, dma_capacity, trace_runs, trace_results);

    fw.send_stop();

    // CSV output
    if (!trace_results.empty()) {
        std::cout << "\nTrace results (app, run, event, kind, h2d, d2h, cycles, total [us], original_count):"
                  << std::endl;
        for (const auto &r : trace_results) {
            std::cout << r.app << ", " << r.run_idx << ", " << r.event_idx << ", "
                      << static_cast<int>(r.kind) << ", " << r.h2d_size << ", "
                      << r.d2h_size << ", " << r.cycles << ", "
                      << std::fixed << std::setprecision(3) << r.total_us << ", "
                      << r.original_count << std::endl;
        }
    }

    ct.connSync(true);
    ct.closeConn();

    std::cout << "All benchmarks completed." << std::endl;
    return EXIT_SUCCESS;
}
