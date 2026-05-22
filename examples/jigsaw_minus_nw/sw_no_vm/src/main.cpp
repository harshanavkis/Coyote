// Includes
#include <chrono>
#include <iomanip>
#include <cstring>
#include <limits>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <stdexcept>
#include <thread>
#include <iostream>
#include <boost/program_options.hpp>

#include <coyote/cThread.hpp>

// Constants
#define CLOCK_PERIOD_NS 4
#define DEFAULT_VFPGA_ID 0

#define N_LATENCY_REPS 1
#define N_THROUGHPUT_REPS 32

// Registers, corresponding to registers defined the vFPGA
//
// These are the byte offsets the *device* MMIO registers map to in
// payload_to_mmio.sv. They must stay in sync with the HW case statements
// (search for `64'h40:` etc. in that file). Don't add new entries here that
// don't have a matching HW handler — the HW will silently drop writes to
// unrecognized addresses, which manifests as zero-valued LEN/STATUS at
// the DC, and the DMA "completes" after one beat with bogus tx_len.
// COYOTE_PID is set on the HC parser via JigsawHostControlRegisters,
// not via this enum.
enum class DMAEngineRegisters: uint32_t {
    DMA_CMD_REG = 0x00,
    DMA_SRC_ADDR_REG = 0x08,
    DMA_DST_ADDR_REG = 0x10,
    DMA_H2D_LEN_REG = 0x18,
    DMA_STATUS_REG = 0x20,
    START_COMPUTATION_REG = 0x28,
    CYCLES_PER_COMP_REG = 0x30,
    DMA_TX_LEN_REG = 0x38,
    DMA_D2H_LEN_REG = 0x40
};

// Registers for jigsaw_host_controller based on jigsaw_minus_nw_axi_ctrl_parser
//
// MMIO_FIFO_DROPPED_REG is a sticky overflow flag exposed by the HW request
// FIFO in the AXI-Lite parser. Because write_mmio() no longer polls per
// request, SW has no per-push feedback if the FIFO ever fills and drops a
// trigger. HW sets bit 0 of this register on any push that arrives while
// the FIFO is full, and SW clears it by writing 0. With FIFO_DEPTH=32 and
// the current ≤7-deep MMIO bursts in benchmark_run/computation_run/trace,
// this should never fire — read it once at end-of-test as a sanity check;
// nonzero indicates a lost setup write and an unreliable result.
enum class JigsawHostControlRegisters : uint32_t {
    MMIO_VADDR_REG = 0,
    MMIO_CTRL_REG = 1,
    MMIO_WRITE_STATUS_REG = 2,
    MMIO_READ_STATUS_REG = 3,
    COYOTE_PID_REG = 4,
    MMIO_OP_REG = 5,
    MMIO_ADDR_REG = 6,
    MMIO_DATA_REG = 7,
    MMIO_READ_DATA_REG = 8,
    MMIO_FIFO_DROPPED_REG = 9
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
    uint32_t size;
    uint64_t cycles;
    int iteration;
    double latency_us;
    double mmio_setup_us;
    double throughput_gibps;
};

// ---------------------------------------------------------------------------
// Benchmarking Logic
// ---------------------------------------------------------------------------

// Global MMIO Read Helper
inline uint64_t read_mmio(coyote::cThread &coyote_thread, uint64_t addr) {
    // Clear read status
    coyote_thread.setCSR(0, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_READ_STATUS_REG));
    
    // Write directly to AXI-Lite registers
    coyote_thread.setCSR(0, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_OP_REG));
    coyote_thread.setCSR(addr, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_ADDR_REG));
    
    // Trigger MMIO
    coyote_thread.setCSR(1, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_CTRL_REG));
    
    // Poll for completion
    while (coyote_thread.getCSR(static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_READ_STATUS_REG)) != 1) {
    }
    
    // Read result directly from AXI-Lite register
    return coyote_thread.getCSR(static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_READ_DATA_REG));
}

// Global MMIO Write Helper
//
// The HW-side request FIFO in jigsaw_minus_nw_axi_ctrl_parser.sv decouples
// submission from emission, so SW does not need to wait for completion of
// each individual write: the FIFO preserves order and the host_controller
// drains in order. The previous poll on MMIO_WRITE_STATUS_REG was the
// dominant cost (one ~2.5 us PCIe read per write_mmio); dropping it
// collapses 5-MMIO setup sequences to ~5 posted PCIe writes total.
//
// Reads still synchronize via read_mmio() because SW genuinely needs the
// returned data, and the final DMA-completion poll on DMA_STATUS_REG
// implicitly waits for the entire enqueued write sequence to drain (the
// status read can't be dispatched until all preceding FIFO entries are
// consumed by the host_controller).
inline void write_mmio(coyote::cThread &coyote_thread, uint64_t addr, uint64_t data) {
    coyote_thread.setCSR(1, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_OP_REG));
    coyote_thread.setCSR(addr, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_ADDR_REG));
    coyote_thread.setCSR(data, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_DATA_REG));
    coyote_thread.setCSR(1, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_CTRL_REG));
}

void run_bench(
    coyote::cThread &coyote_thread, int *mem, std::vector<DmaResult> &results
) {
    // Set the host controller registers from SW
    coyote_thread.setCSR(coyote_thread.getCtid(), static_cast<uint32_t>(JigsawHostControlRegisters::COYOTE_PID_REG));

    uint64_t mem_addr = reinterpret_cast<uint64_t>(mem);

    coyote_thread.setCSR(mem_addr, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_VADDR_REG));

        auto benchmark_run = [&](uint32_t size, bool d2h) -> std::pair<double, double> {
        // Offset by 4KiB to avoid overwriting MMIO request space in SW
        uint64_t dma_addr = mem_addr + 4096;

        if (d2h) {
            // Clear memory region before D2H to verify contents later
            std::memset(reinterpret_cast<void*>(dma_addr), 0, size);
        }

        // Start timing before configuration MMIO calls
        auto start = std::chrono::high_resolution_clock::now();
        write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_SRC_ADDR_REG), dma_addr);
        write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_DST_ADDR_REG), dma_addr);
        
        // Set correct length register based on direction
        if (d2h) {
            write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_D2H_LEN_REG), size);
        } else {
            write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_H2D_LEN_REG), size);
        }

        // Clear DMA status before starting
        write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG), 0);

        // Start DMA transfer. cmd bit 0 = start, bit 1 = direction (0 for H2D, 1 for D2H)
        uint64_t cmd = d2h ? 3 : 1; 
        write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_CMD_REG), cmd);

        // Capture time after MMIO setup is done
        auto mmio_done = std::chrono::high_resolution_clock::now();

        // Wait for DMA transfer to complete
        uint64_t status = 0;
        while ((status = read_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG)) & 0x1) != 1) {
             // Polling for completion
        }
        auto end = std::chrono::high_resolution_clock::now();

        // std::cout << "DMA tx length: " << read_mmio(coyote_thread, static_cast<uint64_t>(DMAEngineRegisters::DMA_TX_LEN_REG)) << std::endl;

        if (d2h) {
            // Verify destination buffer contains all bits set to 1
            volatile uint8_t* check_ptr = reinterpret_cast<volatile uint8_t*>(dma_addr);
            for (uint32_t c = 0; c < size; ++c) {
                if (check_ptr[c] != 0xFF) {
                    std::cerr << "\n[ERROR] DMA D2H mismatch at offset " << c << " (expected 0xFF, got 0x" 
                              << std::hex << (int)check_ptr[c] << std::dec << ")" << std::endl;
                    break;
                }
            }
        }

        std::chrono::duration<double> diff = end - start;
        std::chrono::duration<double> setup_diff = mmio_done - start;
        return {diff.count(), setup_diff.count()}; 
    };

    const int n_runs = 5;

    for (uint32_t size = 4096; size <= 1048576; size *= 2) {
        // H2D
        std::cout << "H2D Size: " << (size / 1024) << " KiB... " << std::flush;
        for (int i = 0; i < n_runs; i++) {
            auto [time_s, m_setup_s] = benchmark_run(size, false);
            double latency_us = time_s * 1000000.0;
            double setup_us = m_setup_s * 1000000.0;
            double throughput_gibps = (static_cast<double>(size) / (1024.0 * 1024.0 * 1024.0)) / time_s;
            results.push_back({size, "h2d", i, latency_us, setup_us, throughput_gibps});
        }
        std::cout << "Done." << std::endl;

        // D2H
        std::cout << "D2H Size: " << (size / 1024) << " KiB... " << std::flush;
        for (int i = 0; i < n_runs; i++) {
            auto [time_s, m_setup_s] = benchmark_run(size, true);
            double latency_us = time_s * 1000000.0;
            double setup_us = m_setup_s * 1000000.0;
            double throughput_gibps = (static_cast<double>(size) / (1024.0 * 1024.0 * 1024.0)) / time_s;
            results.push_back({size, "d2h", i, latency_us, setup_us, throughput_gibps});
        }
        std::cout << "Done." << std::endl;
    }
}

// ---------------------------------------------------------------------------
// Trace Replay (bundled, embedded via traces.hpp)
//
// Three event kinds: BULK_H2D / BULK_D2H / BUNDLE. See REPLAY.md.
// Bitstream quirks: 64-B minimum + no-shrink — mitigated by floor + running
// max per direction.
// ---------------------------------------------------------------------------
#include "traces.hpp"
using namespace jigsaw_traces_ns;

static constexpr uint32_t MIN_DMA_BYTES = 64;
static constexpr int TRACE_N_RUNS = 5;

struct TraceResult {
    std::string app;
    int run_idx;
    int event_idx;
    uint8_t kind;
    uint64_t h2d_size, d2h_size, cycles;
    double total_us;
    int original_count;
};

static uint64_t clamp_dma(uint64_t raw, uint64_t mem_cap) {
    uint64_t sz = (raw > mem_cap) ? mem_cap : raw;
    // Keep raw==0 as 0 so computation_engine skips that phase entirely.
    // Non-zero sizes are rounded UP to a cacheline multiple — Coyote's
    // 512-bit AXI streams move data one 64-byte beat at a time, and
    // non-aligned lengths desync the meta/payload pipeline.
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

static void run_trace_benchmark(coyote::cThread &coyote_thread, int *mem,
                                uint64_t mem_bytes,
                                std::vector<TraceResult> &results)
{
    uint64_t mem_addr = reinterpret_cast<uint64_t>(mem);
    uint64_t dma_addr = mem_addr + 4096;
    uint64_t dma_capacity = mem_bytes - 4096;

    coyote_thread.setCSR(coyote_thread.getCtid(), static_cast<uint32_t>(JigsawHostControlRegisters::COYOTE_PID_REG));
    coyote_thread.setCSR(mem_addr, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_VADDR_REG));

    using clk = std::chrono::high_resolution_clock;
    auto secs = [](auto a, auto b) { return std::chrono::duration<double>(b - a).count(); };

    // CYCLES_PER_COMP is a benchmark-only register (no analogue on a real
    // accelerator), so it's staged outside the timed window per BUNDLE event.
    auto do_bulk = [&](uint32_t size, bool d2h) -> double {
        auto start = clk::now();
        write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_SRC_ADDR_REG), dma_addr);
        write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_DST_ADDR_REG), dma_addr);
        if (d2h) write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_D2H_LEN_REG), size);
        else     write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_H2D_LEN_REG), size);
        write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG), 0);

        uint64_t cmd = d2h ? 3 : 1;
        write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_CMD_REG), cmd);
        while ((read_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG)) & 0x1) != 1) {}
        auto end = clk::now();
        return secs(start, end);
    };

    auto do_bundle = [&](uint32_t h2d, uint32_t d2h, uint64_t cycles) -> double {
        // Synthetic compute-cycle knob: not a real-device register, stage outside.
        write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::CYCLES_PER_COMP_REG), cycles);
        const uint64_t cycles_readback = read_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::CYCLES_PER_COMP_REG));
        if (cycles_readback != cycles) {
            std::cerr << "[ERROR] CYCLES_PER_COMP_REG readback mismatch: wrote "
                      << cycles << ", read " << cycles_readback << std::endl;
            throw std::runtime_error("CYCLES_PER_COMP_REG readback mismatch");
        }

        auto start = clk::now();
        write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_SRC_ADDR_REG), dma_addr);
        write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_DST_ADDR_REG), dma_addr);
        write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_H2D_LEN_REG), h2d);
        write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_D2H_LEN_REG), d2h);
        write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG), 0);

        write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::START_COMPUTATION_REG), 1);
        while ((read_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG)) & 0x3) != 0x3) {}
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

    constexpr size_t n_apps = sizeof(jigsaw_traces) / sizeof(jigsaw_traces[0]);
    for (size_t a = 0; a < n_apps; a++) {
        const auto &app = jigsaw_traces[a];
        std::cerr << "[trace] " << app.name << " (" << app.n << " events, "
                  << TRACE_N_RUNS << " runs)" << std::endl;

      for (int run = 0; run < TRACE_N_RUNS; run++) {
        std::cerr << "[trace] " << app.name << " run " << run << "/" << TRACE_N_RUNS << std::endl;
        for (size_t i = 0; i < app.n; i++) {
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

            switch (ev.kind) {
            case TRACE_BULK_H2D:
                h2d = clamp_dma(ev.h2d_size, dma_capacity);
                std::cerr << "(h2d=" << h2d << ") " << std::flush;
                total_s = do_bulk(static_cast<uint32_t>(h2d), false);
                break;
            case TRACE_BULK_D2H:
                d2h = clamp_dma(ev.d2h_size, dma_capacity);
                std::cerr << "(d2h=" << d2h << ") " << std::flush;
                total_s = do_bulk(static_cast<uint32_t>(d2h), true);
                break;
            case TRACE_BUNDLE:
                h2d = clamp_dma(ev.h2d_size, dma_capacity);
                d2h = clamp_dma(ev.d2h_size, dma_capacity);
                std::cerr << "(h2d=" << h2d << " d2h=" << d2h << ") " << std::flush;
                total_s = do_bundle(static_cast<uint32_t>(h2d), static_cast<uint32_t>(d2h), ev.cycles);
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

int main(int argc, char *argv[]) {
    bool skip_comp = false;

    // Command line options
    boost::program_options::options_description opts("Jigsaw Minus-NW Options");
    opts.add_options()
        ("skip_comp,s", boost::program_options::bool_switch(&skip_comp), "Skip computation benchmarks (No-op in Minus-NW)");

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opts), vm);
    boost::program_options::notify(vm);

    // Create Coyote thread and allocate memory for the transfer
    coyote::cThread coyote_thread(DEFAULT_VFPGA_ID, getpid());
    const uint64_t mem_bytes = 4ULL * 1024 * 1024;
    int* mem =  (int *) coyote_thread.getMem({coyote::CoyoteAllocType::HPF, mem_bytes});
    if (!mem) { throw std::runtime_error("Could not allocate memory; exiting..."); }

    // Results vector
    std::vector<DmaResult> results;

    // Benchmark sweeps
    run_bench(coyote_thread, mem, results);
    run_bench(coyote_thread, mem, results);

    std::vector<CompResult> comp_results;

    if (!skip_comp) {
        std::cout << "\n=== COMPUTATION BENCHMARKS ===" << std::endl;

        const uint64_t cycle_counts[] = {
            100,    // 100 cycles  = 400 ns @ 250 MHz
            1000,   // 1K cycles   = 4 us
            10000,  // 10K cycles  = 40 us
            100000, // 100K cycles = 400 us
            1000000 // 1M cycles   = 4 ms
        };
        const int n_cycle_counts = sizeof(cycle_counts) / sizeof(cycle_counts[0]);

        auto benchmark_comp_run = [&](uint32_t size, uint64_t compute_cycles) -> std::pair<double, double> {
            uint64_t mem_addr = reinterpret_cast<uint64_t>(mem);
            uint64_t dma_addr = mem_addr + 4096;

            // Clear memory to zero before verifying output
            std::memset(reinterpret_cast<void*>(dma_addr), 0, size);

            auto start = std::chrono::high_resolution_clock::now();

            // Set Virtual Address explicitly
            coyote_thread.setCSR(mem_addr, static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_VADDR_REG));

            write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_SRC_ADDR_REG), dma_addr);
            write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_DST_ADDR_REG), dma_addr);
            
            // Set both lengths
            write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_H2D_LEN_REG), size);
            write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_D2H_LEN_REG), size);

            write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::CYCLES_PER_COMP_REG), compute_cycles);

            write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG), 0);

            write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::START_COMPUTATION_REG), 1);

            auto mmio_done = std::chrono::high_resolution_clock::now();

            uint64_t status = 0;
            while (((status = read_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG))) & 0x3) != 0x3) {
                // std::cout << "Polling for computation completion" << std::endl;
            }

            auto end = std::chrono::high_resolution_clock::now();

            // Verify output is all 1s (0xFF)
            volatile uint8_t* check_ptr = reinterpret_cast<volatile uint8_t*>(dma_addr);
            for (uint32_t cx = 0; cx < size; ++cx) {
                if (check_ptr[cx] != 0xFF) {
                    std::cerr << "\n[ERROR] Comp D2H mismatch at offset " << cx << " (expected 0xFF, got 0x" 
                              << std::hex << (int)check_ptr[cx] << std::dec << ")" << std::endl;
                    break;
                }
            }

            std::chrono::duration<double> diff = end - start;
            std::chrono::duration<double> setup_diff = mmio_done - start;
            return {diff.count(), setup_diff.count()};
        };

        const int n_runs = 5;

        for (uint32_t size = 4096; size <= 1024 * 1024; size *= 2) {
            std::cout << "Comp Size: " << (size / 1024) << " KiB... " << std::flush;
            for (int c = 0; c < n_cycle_counts; c++) {
                uint64_t cycles = cycle_counts[c];
                for (int i = 0; i < n_runs; i++) {
                    auto [time_s, setup_s] = benchmark_comp_run(size, cycles);
                    double latency_us = time_s * 1000000.0;
                    double setup_us = setup_s * 1000000.0;
                    double throughput_gibps = (static_cast<double>(size) / (1024.0 * 1024.0 * 1024.0)) / time_s;
                    comp_results.push_back({size, cycles, i, latency_us, setup_us, throughput_gibps});
                }
            }
            std::cout << "Done." << std::endl;
        }
    }

    // =========================================================================
    // Final Results Summary
    // =========================================================================
    std::cout << "\nResults (DMA-only) (size, direction, iteration, latency [us], mmio_setup [us], throughput [GiBps]):" << std::endl;
    for (const auto &res : results)
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

    // =========================================================================
    // Asymmetric DMA Length Test
    // =========================================================================
    // =========================================================================
    // Trace Benchmark (embedded apps from traces.hpp)
    // =========================================================================
    std::cout << "\n=== TRACE BENCHMARK ===" << std::endl;
    {
        std::vector<TraceResult> trace_results;
        run_trace_benchmark(coyote_thread, mem, mem_bytes, trace_results);

        struct AppAgg {
            int n_events = 0;
            int n_bulk_h2d = 0, n_bulk_d2h = 0, n_bundle = 0;
            double total_h2d_us = 0, total_d2h_us = 0, total_bundle_us = 0;
            uint64_t total_h2d_bytes = 0, total_d2h_bytes = 0, total_cycles = 0;
        };
        struct AppKey {
            std::string app;
            int run_idx;
        };
        std::vector<std::pair<AppKey, AppAgg>> ordered;
        std::map<std::pair<std::string, int>, size_t> idx;
        for (const auto &r : trace_results) {
            auto key = std::make_pair(r.app, r.run_idx);
            auto it = idx.find(key);
            if (it == idx.end()) {
                idx[key] = ordered.size();
                ordered.push_back({{r.app, r.run_idx}, AppAgg{}});
                it = idx.find(key);
            }
            auto &a = ordered[it->second].second;
            a.n_events++;
            switch (r.kind) {
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

        std::cout << std::endl << "# per-app summary (one row per (app, run))" << std::endl;
        std::cout << "app, run, n_events, n_bulk_h2d, n_bulk_d2h, n_bundle, "
                  << "total_h2d_us, total_d2h_us, total_bundle_us, total_us, "
                  << "total_h2d_bytes, total_d2h_bytes, total_cycles" << std::endl;
        for (const auto &p : ordered) {
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

    std::cout << "\n=== ASYMMETRIC DMA TEST ===" << std::endl;
    uint64_t h2d_test_len = 4096;
    uint64_t d2h_test_len = 8192;
    
    // H2D
    write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_H2D_LEN_REG), h2d_test_len);
    write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG), 0);
    write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_CMD_REG), 1); // H2D Start
    while ((read_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG)) & 0x1) != 1);
    uint64_t h2d_actual = read_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_TX_LEN_REG));
    
    // D2H
    write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_D2H_LEN_REG), d2h_test_len);
    write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG), 0);
    write_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_CMD_REG), 3); // D2H Start
    while ((read_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_STATUS_REG)) & 0x1) != 1);
    uint64_t d2h_actual = read_mmio(coyote_thread, static_cast<uint32_t>(DMAEngineRegisters::DMA_TX_LEN_REG));
    
    std::cout << "Asymmetric Test Result:" << std::endl;
    // Note: H2D in Minus-NW includes the 64B header in the tx_len count
    std::cout << "H2D: Target " << h2d_test_len << ", Actual " << h2d_actual << (h2d_actual == h2d_test_len + 64 ? " [PASS]" : " [FAIL]") << std::endl;
    std::cout << "D2H: Target " << d2h_test_len << ", Actual " << d2h_actual << (d2h_actual == d2h_test_len ? " [PASS]" : " [FAIL]") << std::endl;

    // FIFO overflow sanity check (see JigsawHostControlRegisters comment).
    // A nonzero value here means at least one MMIO push was dropped during
    // this run — measurements above are unreliable.
    uint64_t fifo_dropped = coyote_thread.getCSR(static_cast<uint32_t>(JigsawHostControlRegisters::MMIO_FIFO_DROPPED_REG));
    if (fifo_dropped != 0) {
        std::cerr << "[WARNING] MMIO FIFO overflow detected (MMIO_FIFO_DROPPED_REG=" << fifo_dropped
                  << "). Some setup writes were dropped; results above are unreliable." << std::endl;
    }

    return EXIT_SUCCESS;
}
