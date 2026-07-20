/**
 * Jigsaw Software Forwarder — device selftest (local, no RDMA)
 *
 * Replays the full-size Vortex trace directly on the jigsaw_baseline vFPGA
 * of the CURRENT bitstream through plain setCSR/getCSR — the exact register
 * sequences, 1 MiB chunking, buffer size and staging buffer type the
 * forwarder's device replayer uses, but single-node with no NIC or RDMA
 * involvement whatsoever.
 *
 * Purpose: discriminate the spmv ev8 wedge. The existing
 * jigsaw_baseline/sw_no_vm "normal case" runs a scaled-down trace (events
 * capped at 1.2 MB, 4 MiB buffer) on the standalone bitstream at vFPGA 0;
 * the full-size trace has never been driven into jigsaw_baseline locally on
 * the two-vFPGA build_may11 bitstream. If THIS harness also wedges (stuck
 * wait on DMA_STATUS, register dump printed after 3 s), the fault is in the
 * device path itself under this stimulus; if it completes, the wedge needs
 * concurrent NIC activity and the forwarder's cross-vFPGA interaction is
 * the suspect.
 *
 * Usage (on the device node, current bitstream flashed):
 *   ./test [-v <vfpga_id, default 1>] [-r <runs, default 1>]
 *   (-v 0 for the standalone jigsaw_baseline bitstream)
 */

#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

#include <boost/program_options.hpp>

#include <coyote/cThread.hpp>

#include "mailbox.hpp"

using namespace jsfwd;

#define CLOCK_PERIOD_NS 4

#include "traces.hpp"
using namespace jigsaw_traces_ns;

static constexpr uint32_t MIN_DMA_BYTES = 64;
// Same chunking convention as the forwarder harness / guest driver.
static constexpr uint64_t TRACE_CHUNK_BYTES = 1ULL << 20;

static coyote::cThread *g_jig = nullptr;

static uint64_t rd(DevReg r) {
    return g_jig->getCSR(dev_reg_index(static_cast<uint64_t>(r)));
}
static void wr(DevReg r, uint64_t v) {
    g_jig->setCSR(v, dev_reg_index(static_cast<uint64_t>(r)));
}

static uint64_t clamp_dma(uint64_t raw, uint64_t mem_cap)
{
    uint64_t sz = (raw > mem_cap) ? mem_cap : raw;
    if (sz != 0) {
        sz = (sz + MIN_DMA_BYTES - 1) & ~static_cast<uint64_t>(MIN_DMA_BYTES - 1);
        if (sz > mem_cap) {
            sz = mem_cap & ~static_cast<uint64_t>(MIN_DMA_BYTES - 1);
        }
    }
    return sz;
}

// Same stuck-operation diagnostics as the device replayer's wait_status.
static void wait_status(uint64_t mask)
{
    uint64_t start = now_ms();
    bool dumped = false;
    while ((rd(DevReg::DMA_STATUS) & mask) != mask) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(CLOCK_PERIOD_NS));
        if (!dumped && now_ms() - start > 3000) {
            dumped = true;
            std::cerr << "[selftest] wait_status(0x" << std::hex << mask
                      << ") stuck 3s: STATUS=0x" << rd(DevReg::DMA_STATUS)
                      << " CMD=0x" << rd(DevReg::DMA_CMD)
                      << " TX_LEN=0x" << rd(DevReg::DMA_TX_LEN)
                      << " H2D_LEN=0x" << rd(DevReg::DMA_H2D_LEN)
                      << " D2H_LEN=0x" << rd(DevReg::DMA_D2H_LEN)
                      << std::dec << std::endl;
        }
    }
}

// Identical MMIO sequence to the forwarder's do_bulk, executed locally.
static double do_bulk(uint64_t dma_addr, uint32_t size, bool d2h)
{
    auto start = std::chrono::high_resolution_clock::now();
    uint64_t remaining = size, off = 0;
    while (remaining > 0) {
        uint64_t chunk = remaining > TRACE_CHUNK_BYTES ? TRACE_CHUNK_BYTES : remaining;
        wr(DevReg::DMA_SRC_ADDR, dma_addr + off);
        wr(DevReg::DMA_DST_ADDR, dma_addr + off);
        wr(d2h ? DevReg::DMA_D2H_LEN : DevReg::DMA_H2D_LEN, chunk);
        wr(DevReg::DMA_STATUS, 0);
        wr(DevReg::DMA_CMD, d2h ? DMA_CMD_D2H : DMA_CMD_H2D);
        wait_status(STATUS_DMA_DONE_MASK);
        wr(DevReg::DMA_STATUS, 0);
        off += chunk;
        remaining -= chunk;
    }
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double>(end - start).count();
}

// Identical MMIO sequence to the forwarder's do_bundle, executed locally.
static double do_bundle(uint64_t dma_addr, uint32_t h2d, uint32_t d2h,
                        uint64_t cycles)
{
    wr(DevReg::CYCLES_COMPUTE, cycles);
    const uint64_t cycles_readback = rd(DevReg::CYCLES_COMPUTE);
    if (cycles_readback != cycles) {
        std::cerr << "[ERROR] CYCLES_COMPUTE readback mismatch: wrote " << cycles
                  << ", read " << cycles_readback << std::endl;
        throw std::runtime_error("CYCLES_COMPUTE readback mismatch");
    }

    auto start = std::chrono::high_resolution_clock::now();
    wr(DevReg::DMA_SRC_ADDR, dma_addr);
    wr(DevReg::DMA_DST_ADDR, dma_addr);
    wr(DevReg::DMA_H2D_LEN, h2d);
    wr(DevReg::DMA_D2H_LEN, d2h);
    wr(DevReg::DMA_STATUS, 0);
    wr(DevReg::START_COMPUTE, 1);
    wait_status(STATUS_BUNDLE_DONE_MASK);
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double>(end - start).count();
}

int main(int argc, char *argv[])
{
    int vfpga_id = 1;
    int trace_runs = 1;

    boost::program_options::options_description opts("Jigsaw Device Selftest Options");
    opts.add_options()
        ("vfpga,v", boost::program_options::value<int>(&vfpga_id),
            "vFPGA id of jigsaw_baseline (1 on build_may11, 0 standalone)")
        ("trace_runs,r", boost::program_options::value<int>(&trace_runs),
            "Runs per trace application");

    boost::program_options::variables_map vm;
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, argv, opts), vm);
    boost::program_options::notify(vm);

    HEADER("JIGSAW SW FORWARDER — DEVICE SELFTEST (LOCAL)");
    std::cout << "jigsaw_baseline vFPGA id : " << vfpga_id << std::endl;

    coyote::cThread jig(vfpga_id, getpid());
    g_jig = &jig;

    // Same staging buffer type and size as the device replayer.
    char *device_buf = static_cast<char *>(
        jig.getMem({coyote::CoyoteAllocType::HPF, BUF_BYTES}));
    if (!device_buf) {
        std::cerr << "device staging buffer allocation failed" << std::endl;
        return EXIT_FAILURE;
    }
    memset(device_buf, 0xAB, BUF_BYTES);  // pre-fault

    jig.setCSR(jig.getCtid(), COYOTE_PID_REG);

    uint64_t dma_addr = reinterpret_cast<uint64_t>(device_buf) + PAYLOAD_OFF;
    uint64_t dma_capacity = BUF_BYTES - PAYLOAD_OFF;

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
                  << trace_runs << " runs)" << std::endl;

        for (int run = 0; run < trace_runs; run++) {
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
                    total_s = do_bulk(dma_addr, static_cast<uint32_t>(h2d), false);
                    break;
                case TRACE_BULK_D2H:
                    d2h = clamp_dma(ev.d2h_size, dma_capacity);
                    total_s = do_bulk(dma_addr, static_cast<uint32_t>(d2h), true);
                    break;
                case TRACE_BUNDLE:
                    h2d = clamp_dma(ev.h2d_size, dma_capacity);
                    d2h = clamp_dma(ev.d2h_size, dma_capacity);
                    total_s = do_bundle(dma_addr, static_cast<uint32_t>(h2d),
                                        static_cast<uint32_t>(d2h), ev.cycles);
                    break;
                default:
                    std::cerr << "[warn] unknown kind " << (int)ev.kind << std::endl;
                    continue;
                }

                std::cerr << "done " << (total_s * 1e6) << " us" << std::endl;
            }
            std::cerr << "[trace] " << app.name << " run " << run << " done"
                      << std::endl;
        }
    }

    std::cout << "Selftest completed: full trace replayed locally without a wedge."
              << std::endl;
    return EXIT_SUCCESS;
}
