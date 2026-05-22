// Includes
#include <chrono>
#include <iomanip>
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
enum class JigsawRegisters: uint32_t {
    DMA_CMD_REG = 0,
    DMA_SRC_ADDR_REG = 1,
    DMA_DST_ADDR_REG = 2,
    DMA_H2D_LEN_REG = 3,
    DMA_STATUS_REG = 4,
    START_COMPUTATION_REG = 5,
    CYCLES_PER_COMPUTATION_REG = 6,
    COYOTE_DMA_TX_LEN_REG = 7,
    DMA_D2H_LEN_REG = 8,
    COYOTE_PID_REG = 9
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
// Benchmarking Logic
// ---------------------------------------------------------------------------
void run_bench(
    coyote::cThread &coyote_thread, int *mem, std::vector<DmaResult> &results
) {
    auto benchmark_run = [&](uint32_t size, bool d2h) -> std::pair<double, double> {
        // Start timing before configuration MMIO calls
        auto start = std::chrono::high_resolution_clock::now();

        // Set the required registers from SW
        coyote_thread.setCSR(reinterpret_cast<uint64_t>(mem), static_cast<uint32_t>(JigsawRegisters::DMA_SRC_ADDR_REG));
        coyote_thread.setCSR(reinterpret_cast<uint64_t>(mem), static_cast<uint32_t>(JigsawRegisters::DMA_DST_ADDR_REG));
        
        // Set the correct length register based on direction
        if (d2h) {
            coyote_thread.setCSR(static_cast<uint64_t>(size), static_cast<uint32_t>(JigsawRegisters::DMA_D2H_LEN_REG));
        } else {
            coyote_thread.setCSR(static_cast<uint64_t>(size), static_cast<uint32_t>(JigsawRegisters::DMA_H2D_LEN_REG));
        }

        coyote_thread.setCSR(coyote_thread.getCtid(), static_cast<uint32_t>(JigsawRegisters::COYOTE_PID_REG));

        // Clear DMA status before starting
        coyote_thread.setCSR(static_cast<uint64_t>(0), static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG));

        // Start DMA transfer. cmd bit 0 = start, bit 1 = direction (0 for H2D, 1 for D2H)
        uint64_t cmd = d2h ? 3 : 1; 
        coyote_thread.setCSR(cmd, static_cast<uint32_t>(JigsawRegisters::DMA_CMD_REG));

        // Capture time after MMIO setup is done
        auto mmio_done = std::chrono::high_resolution_clock::now();

        // Wait for DMA transfer to complete
        while (coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG)) != 1) {
            // Polling
        }
        auto end = std::chrono::high_resolution_clock::now();

        // Clear DMA status after polling
        coyote_thread.setCSR(static_cast<uint64_t>(0), static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG));

        std::chrono::duration<double> diff = end - start;
        std::chrono::duration<double> setup_diff = mmio_done - start;
        return {diff.count(), setup_diff.count()}; 
    };

    const int n_runs = 5;

    for (uint32_t size = 4096; size <= 1048576; size *= 2) {
        // H2D
        std::cout << "H2D Size: " << (size / 1024) << " KiB... " << std::flush;
        for (int i = 0; i < n_runs; i++) {
            auto [time_s, setup_s] = benchmark_run(size, false);
            double latency_us = time_s * 1000000.0;
            double setup_us = setup_s * 1000000.0;
            double throughput_gibps = (static_cast<double>(size) / (1024.0 * 1024.0 * 1024.0)) / time_s;
            results.push_back({size, "h2d", i, latency_us, setup_us, throughput_gibps});
        }
        std::cout << "Done." << std::endl;

        // D2H
        std::cout << "D2H Size: " << (size / 1024) << " KiB... " << std::flush;
        for (int i = 0; i < n_runs; i++) {
            auto [time_s, setup_s] = benchmark_run(size, true);
            double latency_us = time_s * 1000000.0;
            double setup_us = setup_s * 1000000.0;
            double throughput_gibps = (static_cast<double>(size) / (1024.0 * 1024.0 * 1024.0)) / time_s;
            results.push_back({size, "d2h", i, latency_us, setup_us, throughput_gibps});
        }
        std::cout << "Done." << std::endl;
    }
}

/**
 * run_computation_bench - benchmarks the full computation pipeline:
 *   H2D DMA → compute (spin for N cycles on FPGA) → D2H DMA
 *
 * Sweeps across multiple data sizes and computation cycle counts.
 * Polls DMA_STATUS_REG bit 1 for completion.
 */
void run_computation_bench(
    coyote::cThread &coyote_thread, int *mem, std::vector<CompResult> &results
) {
    auto computation_run = [&](uint32_t size, uint64_t compute_cycles) -> std::pair<double, double> {
        // Start timing before configuration MMIO calls
        auto start = std::chrono::high_resolution_clock::now();

        // Set addresses and length
        coyote_thread.setCSR(reinterpret_cast<uint64_t>(mem), static_cast<uint32_t>(JigsawRegisters::DMA_SRC_ADDR_REG));
        coyote_thread.setCSR(reinterpret_cast<uint64_t>(mem), static_cast<uint32_t>(JigsawRegisters::DMA_DST_ADDR_REG));
        
        // Set both lengths (currently same, but hardware supports different)
        coyote_thread.setCSR(static_cast<uint64_t>(size), static_cast<uint32_t>(JigsawRegisters::DMA_H2D_LEN_REG));
        coyote_thread.setCSR(static_cast<uint64_t>(size), static_cast<uint32_t>(JigsawRegisters::DMA_D2H_LEN_REG));
        
        coyote_thread.setCSR(coyote_thread.getCtid(), static_cast<uint32_t>(JigsawRegisters::COYOTE_PID_REG));

        // Set cycles per computation
        coyote_thread.setCSR(compute_cycles, static_cast<uint32_t>(JigsawRegisters::CYCLES_PER_COMPUTATION_REG));

        // Clear DMA status register (both bit 0 and bit 1)
        coyote_thread.setCSR(static_cast<uint64_t>(0), static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG));

        // Start computation: write 1 to START_COMPUTATION_REG
        coyote_thread.setCSR(static_cast<uint64_t>(1), static_cast<uint32_t>(JigsawRegisters::START_COMPUTATION_REG));

        // Capture time after MMIO setup is done
        auto mmio_done = std::chrono::high_resolution_clock::now();

        // Poll DMA_STATUS_REG bit 1 for computation completion
        while ((coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG)) & 0x2) == 0) {
            // Polling
        }
        auto end = std::chrono::high_resolution_clock::now();

        // Clear DMA status after polling
        coyote_thread.setCSR(static_cast<uint64_t>(0), static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG));

        std::chrono::duration<double> diff = end - start;
        std::chrono::duration<double> setup_diff = mmio_done - start;
        return {diff.count(), setup_diff.count()}; 
    };

    const int n_runs = 5;

    // Computation cycle counts to sweep
    const uint64_t cycle_counts[] = {
        100,        // 100 cycles  = 400 ns @ 250 MHz
        1000,       // 1K cycles   = 4 us
        10000,      // 10K cycles  = 40 us
        100000,     // 100K cycles = 400 us
        1000000     // 1M cycles   = 4 ms
    };
    const int n_cycle_counts = sizeof(cycle_counts) / sizeof(cycle_counts[0]);

    for (uint32_t size = 4096; size <= 1048576; size *= 2) {
        std::cout << "Comp Size: " << (size / 1024) << " KiB... " << std::flush;
        for (int c = 0; c < n_cycle_counts; c++) {
            uint64_t cycles = cycle_counts[c];
            for (int i = 0; i < n_runs; i++) {
                auto [time_s, setup_s] = computation_run(size, cycles);
                double latency_us = time_s * 1000000.0;
                double setup_us = setup_s * 1000000.0;
                double throughput_gibps = (static_cast<double>(size) / (1024.0 * 1024.0 * 1024.0)) / time_s;
                results.push_back({size, cycles, i, latency_us, setup_us, throughput_gibps});
            }
        }
        std::cout << "Done." << std::endl;
    }
}

// ---------------------------------------------------------------------------
// Trace Replay (bundled, embedded via traces.hpp)
//
// Each app's trace is a sequence of three event kinds:
//   BULK_H2D : standalone H2D DMA (data preload, kernel binary).
//   BULK_D2H : standalone D2H DMA (final result readback).
//   BUNDLE   : one START_COMPUTATION_REG kick — the device fuses H2D-in
//              (kernel args + flag resets) + busy-wait compute + D2H-out
//              (flag readbacks) atomically.
//
// Bitstream quirks (still relevant for the BULK paths):
//   1. Minimum length is one cacheline (64 B).
//   2. Length register cannot decrease between same-direction transfers.
// Mitigations: floor at MIN_DMA_BYTES, pad up to running max per direction.
//
// Timing: parameter MMIOs staged OUTSIDE the timed window. total_us covers
// only kick MMIO + completion poll.
// ---------------------------------------------------------------------------
#include "traces.hpp"
using namespace jigsaw_traces_ns;

static constexpr uint32_t MIN_DMA_BYTES = 64;

static constexpr int TRACE_N_RUNS = 5;

struct TraceResult {
    std::string app;
    int run_idx;
    int event_idx;
    uint8_t kind;       // BULK_H2D / BULK_D2H / BUNDLE
    uint64_t h2d_size;
    uint64_t d2h_size;
    uint64_t cycles;
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
    using clk = std::chrono::high_resolution_clock;
    auto secs = [](auto a, auto b) { return std::chrono::duration<double>(b - a).count(); };

    // PID is set once at the start of the trace run instead of being re-asserted
    // per event. In real Coyote use the PID is bound at vFPGA acquisition; the
    // per-event re-assertion in the original benchmark_run lambda was inherited
    // defensively without evidence the bitstream actually requires it. If a
    // future bitstream proves otherwise, move it back inside do_bulk/do_bundle.
    coyote_thread.setCSR(coyote_thread.getCtid(), static_cast<uint32_t>(JigsawRegisters::COYOTE_PID_REG));

    // CYCLES_PER_COMPUTATION is a benchmark-only register (no analogue on a real
    // accelerator), so it's staged outside the timed window per BUNDLE event.
    auto do_bulk = [&](uint32_t size, bool d2h) -> double {
        auto start = clk::now();
        coyote_thread.setCSR(reinterpret_cast<uint64_t>(mem),  static_cast<uint32_t>(JigsawRegisters::DMA_SRC_ADDR_REG));
        coyote_thread.setCSR(reinterpret_cast<uint64_t>(mem),  static_cast<uint32_t>(JigsawRegisters::DMA_DST_ADDR_REG));
        if (d2h) coyote_thread.setCSR(static_cast<uint64_t>(size), static_cast<uint32_t>(JigsawRegisters::DMA_D2H_LEN_REG));
        else     coyote_thread.setCSR(static_cast<uint64_t>(size), static_cast<uint32_t>(JigsawRegisters::DMA_H2D_LEN_REG));
        coyote_thread.setCSR(static_cast<uint64_t>(0),         static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG));

        uint64_t cmd = d2h ? 3 : 1;
        coyote_thread.setCSR(cmd,                              static_cast<uint32_t>(JigsawRegisters::DMA_CMD_REG));
        while (coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG)) != 1) {}
        auto end = clk::now();
        coyote_thread.setCSR(static_cast<uint64_t>(0),         static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG));
        return secs(start, end);
    };

    auto do_bundle = [&](uint32_t h2d, uint32_t d2h, uint64_t cycles) -> double {
        // Synthetic compute-cycle knob: not a real-device register, stage outside.
        coyote_thread.setCSR(cycles,                           static_cast<uint32_t>(JigsawRegisters::CYCLES_PER_COMPUTATION_REG));
        const uint64_t cycles_readback = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::CYCLES_PER_COMPUTATION_REG));
        if (cycles_readback != cycles) {
            std::cerr << "[ERROR] CYCLES_PER_COMPUTATION_REG readback mismatch: wrote "
                      << cycles << ", read " << cycles_readback << std::endl;
            throw std::runtime_error("CYCLES_PER_COMPUTATION_REG readback mismatch");
        }

        auto start = clk::now();
        coyote_thread.setCSR(reinterpret_cast<uint64_t>(mem),  static_cast<uint32_t>(JigsawRegisters::DMA_SRC_ADDR_REG));
        coyote_thread.setCSR(reinterpret_cast<uint64_t>(mem),  static_cast<uint32_t>(JigsawRegisters::DMA_DST_ADDR_REG));
        coyote_thread.setCSR(static_cast<uint64_t>(h2d),       static_cast<uint32_t>(JigsawRegisters::DMA_H2D_LEN_REG));
        coyote_thread.setCSR(static_cast<uint64_t>(d2h),       static_cast<uint32_t>(JigsawRegisters::DMA_D2H_LEN_REG));
        coyote_thread.setCSR(static_cast<uint64_t>(0),         static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG));

        coyote_thread.setCSR(static_cast<uint64_t>(1),         static_cast<uint32_t>(JigsawRegisters::START_COMPUTATION_REG));
        while ((coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG)) & 0x2) == 0) {}
        auto end = clk::now();
        coyote_thread.setCSR(static_cast<uint64_t>(0),         static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG));
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
                h2d = clamp_dma(ev.h2d_size, mem_bytes);
                std::cerr << "(h2d=" << h2d << ") " << std::flush;
                total_s = do_bulk(static_cast<uint32_t>(h2d), false);
                break;
            case TRACE_BULK_D2H:
                d2h = clamp_dma(ev.d2h_size, mem_bytes);
                std::cerr << "(d2h=" << d2h << ") " << std::flush;
                total_s = do_bulk(static_cast<uint32_t>(d2h), true);
                break;
            case TRACE_BUNDLE:
                h2d = clamp_dma(ev.h2d_size, mem_bytes);
                d2h = clamp_dma(ev.d2h_size, mem_bytes);
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
    boost::program_options::options_description opts("Jigsaw Baseline Options");
    opts.add_options()
        ("skip_comp,s", boost::program_options::bool_switch(&skip_comp), "Skip computation benchmarks");

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opts), vm);
    boost::program_options::notify(vm);

    // Create Coyote thread and allocate memory for the transfer
    coyote::cThread coyote_thread(DEFAULT_VFPGA_ID, getpid());
    const uint64_t mem_bytes = 4ULL * 1024 * 1024;
    int* mem =  (int *) coyote_thread.getMem({coyote::CoyoteAllocType::HPF, mem_bytes});
    if (!mem) { throw std::runtime_error("Could not allocate memory; exiting..."); }

    // Results vectors
    std::vector<DmaResult> dma_results;
    std::vector<CompResult> comp_results;

    // DMA-only benchmarks (backward compatible)
    std::cout << "=== DMA BENCHMARKS ===" << std::endl;
    run_bench(coyote_thread, mem, dma_results);

    std::cout << std::endl;

    if (!skip_comp) {
        // Full computation pipeline benchmarks (H2D → compute → D2H)
        std::cout << "=== COMPUTATION BENCHMARKS ===" << std::endl;
        run_computation_bench(coyote_thread, mem, comp_results);
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

    // =========================================================================
    // Trace Benchmark (embedded apps from traces.hpp)
    // =========================================================================
    std::cout << "\n=== TRACE BENCHMARK ===" << std::endl;
    {
        std::vector<TraceResult> trace_results;
        run_trace_benchmark(coyote_thread, mem, mem_bytes, trace_results);

        // Per-app aggregate summary
        struct AppAgg {
            int n_events = 0;
            int n_bulk_h2d = 0, n_bulk_d2h = 0, n_bundle = 0;
            double total_h2d_us = 0;     // time in BULK_H2D events
            double total_d2h_us = 0;     // time in BULK_D2H events
            double total_bundle_us = 0;  // time in BUNDLE events
            uint64_t total_h2d_bytes = 0;
            uint64_t total_d2h_bytes = 0;
            uint64_t total_cycles = 0;
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

    // =========================================================================
    // Asymmetric DMA Length Test
    // =========================================================================
    std::cout << "\n=== ASYMMETRIC DMA TEST ===" << std::endl;
    uint64_t h2d_test_len = 4096;
    uint64_t d2h_test_len = 8192;
    
    // H2D
    coyote_thread.setCSR(h2d_test_len, static_cast<uint32_t>(JigsawRegisters::DMA_H2D_LEN_REG));
    coyote_thread.setCSR(static_cast<uint64_t>(0), static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG));
    coyote_thread.setCSR(1, static_cast<uint32_t>(JigsawRegisters::DMA_CMD_REG)); // H2D Start
    while (coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG)) != 1);
    uint64_t h2d_actual = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::COYOTE_DMA_TX_LEN_REG));
    
    // D2H
    coyote_thread.setCSR(d2h_test_len, static_cast<uint32_t>(JigsawRegisters::DMA_D2H_LEN_REG));
    coyote_thread.setCSR(static_cast<uint64_t>(0), static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG));
    coyote_thread.setCSR(3, static_cast<uint32_t>(JigsawRegisters::DMA_CMD_REG)); // D2H Start
    while (coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::DMA_STATUS_REG)) != 1);
    uint64_t d2h_actual = coyote_thread.getCSR(static_cast<uint32_t>(JigsawRegisters::COYOTE_DMA_TX_LEN_REG));
    
    std::cout << "Asymmetric Test Result:" << std::endl;
    std::cout << "H2D: Target " << h2d_test_len << ", Actual " << h2d_actual << (h2d_actual == h2d_test_len ? " [PASS]" : " [FAIL]") << std::endl;
    std::cout << "D2H: Target " << d2h_test_len << ", Actual " << d2h_actual << (d2h_actual == d2h_test_len ? " [PASS]" : " [FAIL]") << std::endl;

    return EXIT_SUCCESS;
}
