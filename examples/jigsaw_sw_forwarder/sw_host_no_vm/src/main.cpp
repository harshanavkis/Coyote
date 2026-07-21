/**
 * Jigsaw Software Forwarder — host-side trace harness (no VM)
 *
 * One cThread on the perf_rdma vFPGA (dumb NIC), one 64 B mailbox slot per
 * direction (requests at offset 0, responses at offset 64 — each slot has
 * exactly one writer, see messages.hpp), payloads pushed into the region
 * behind the control page, strict ping-pong.
 *
 * Drives the same device interactions as jigsaw_host_controller/sw_no_vm
 * (Vortex OpenCL trace replay), but every MMIO access is encapsulated into
 * a mailbox request and replayed VERBATIM by the device-side replayer on
 * the unmodified jigsaw_baseline accelerator — the device never waits on
 * the accelerator; this harness polls STATUS over the wire exactly as the
 * guest driver polls it locally. D2H data is pushed by the device ahead of
 * the first STATUS response that reports "done" (same QP, placed first),
 * so it is present in the NIC buffer the moment "done" is visible.
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

#include "messages.hpp"

using namespace jsfwd;

// Constants
#define DEFAULT_VFPGA_ID 0

// Globals (plain file-scope state, logic in helpers)
static coyote::cThread *ct;
static char *nic_buf;
static char *app_buf;
static struct msg *req_slot;             // this node is its only writer
static volatile struct msg *resp_slot;   // written by the device

// Shadow copies of the last-written DMA parameter registers, needed to
// stage payloads at trigger time (same role as on the device side).
static uint64_t sh_src, sh_dst, sh_h2d, sh_d2h;

// Armed D2H: set at the trigger write; consumed by the first STATUS read
// that observes the done bits (the device pushed the payload ahead of
// that read's response on the same QP, so the data is already local).
static struct { bool armed; uint64_t off, len, mask; } d2h_pending;

static uint64_t req_seq = 0;

static const uint64_t RESP_TIMEOUT_MS = 5000;

// ---------------------------------------------------------------------------
// Mailbox primitives: one slot per direction, monotonic publish counter
// (see messages.hpp for why not a clear-and-wait ready flag)
// ---------------------------------------------------------------------------
static uint64_t request(uint64_t op, uint64_t addr, uint64_t value)
{
    uint64_t seq = ++req_seq;
    req_slot->op = op;
    req_slot->addr = addr;
    req_slot->value = value;
    req_slot->seq = seq;  // publish flag, written last
    coyote::rdmaSg sg = {.local_offs = REQ_OFF, .remote_offs = REQ_OFF,
                         .len = SLOT_BYTES};
    ct->invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);

    uint64_t deadline = now_ms() + RESP_TIMEOUT_MS;
    while (resp_slot->seq < seq) {
        if (now_ms() > deadline) {
            std::cerr << "[mbox] FATAL: no response for seq=" << seq
                      << " op=" << op << " addr=0x" << std::hex << addr
                      << std::dec << std::endl;
            abort();
        }
    }
    uint64_t result = resp_slot->value;
    // Cheap local flush every round trip, the baseline pair's cadence
    ct->clearCompleted();
    return result;
}

static bool payload_range_ok(uint64_t off, uint64_t len)
{
    return off >= PAYLOAD_OFF && off < BUF_BYTES && len <= BUF_BYTES - off;
}

// H2D: bounce application memory into the NIC buffer and push it before
// the trigger request is sent (same QP => payload placed first).
static void stage_in(uint64_t src_ptr, uint64_t len)
{
    uint64_t off = src_ptr - reinterpret_cast<uint64_t>(app_buf);
    if (!payload_range_ok(off, len)) return;
    memcpy(nic_buf + off, app_buf + off, len);
    uint64_t aligned = (len + 63) & ~uint64_t(63);
    if (off + aligned > BUF_BYTES)
        aligned = BUF_BYTES - off;
    coyote::rdmaSg sg = {.local_offs = off, .remote_offs = off,
                         .len = static_cast<uint32_t>(aligned)};
    ct->invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);
}

// Forward one MMIO access, in the exact order the guest driver issues
// them. An H2D trigger is preceded by its payload push (same QP, placed
// first); a trigger with a D2H phase arms the copy-out consumed by the
// STATUS poll below.
static void mmio_write(uint64_t addr, uint64_t value)
{
    switch (static_cast<DevReg>(addr)) {
    case DevReg::DMA_SRC_ADDR: sh_src = value; break;
    case DevReg::DMA_DST_ADDR: sh_dst = value; break;
    case DevReg::DMA_H2D_LEN:  sh_h2d = value; break;
    case DevReg::DMA_D2H_LEN:  sh_d2h = value; break;
    case DevReg::DMA_CMD:
        if (value == DMA_CMD_H2D) {
            stage_in(sh_src, sh_h2d);
        } else if (value == DMA_CMD_D2H) {
            uint64_t off = sh_dst - reinterpret_cast<uint64_t>(app_buf);
            d2h_pending = {true, off, sh_d2h, STATUS_DMA_DONE_MASK};
        }
        break;
    case DevReg::START_COMPUTE:
        if (value == 1) {
            if (sh_h2d) stage_in(sh_src, sh_h2d);
            if (sh_d2h) {
                uint64_t off = sh_dst - reinterpret_cast<uint64_t>(app_buf);
                d2h_pending = {true, off, sh_d2h, STATUS_BUNDLE_DONE_MASK};
            }
        }
        break;
    default:
        break;
    }
    (void)request(OP_MMIO_WRITE, addr, value);
}

static uint64_t mmio_read(uint64_t addr)
{
    uint64_t value = request(OP_MMIO_READ, addr, 0);
    // The device pushes an armed D2H payload before the STATUS response
    // that first reports "done" (same QP, placed first), so the data is in
    // the NIC buffer by the time "done" is visible — bounce it out.
    if (static_cast<DevReg>(addr) == DevReg::DMA_STATUS && d2h_pending.armed &&
        (value & d2h_pending.mask) == d2h_pending.mask) {
        if (payload_range_ok(d2h_pending.off, d2h_pending.len))
            memcpy(app_buf + d2h_pending.off, nic_buf + d2h_pending.off,
                   d2h_pending.len);
        d2h_pending.armed = false;
    }
    return value;
}

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

using clk = std::chrono::high_resolution_clock;
static double secs(clk::time_point a, clk::time_point b)
{
    return std::chrono::duration<double>(b - a).count();
}

// Diagnose a stuck completion poll: after 3 s dump the device's
// engine-visible registers once, read over the wire (the replayer answers
// MMIO_READs as long as the transport is alive).
static void poll_status(uint64_t mask, const char *what)
{
    auto start = clk::now();
    bool dumped = false;
    while ((mmio_read(static_cast<uint64_t>(DevReg::DMA_STATUS)) & mask) != mask) {
        if (!dumped && secs(start, clk::now()) > 3.0) {
            dumped = true;
            std::cerr << "\n[host] " << what << " poll(0x" << std::hex << mask
                      << ") stuck 3s: STATUS=0x"
                      << mmio_read(static_cast<uint64_t>(DevReg::DMA_STATUS))
                      << " CMD=0x"
                      << mmio_read(static_cast<uint64_t>(DevReg::DMA_CMD))
                      << " TX_LEN=0x"
                      << mmio_read(static_cast<uint64_t>(DevReg::DMA_TX_LEN))
                      << " H2D_LEN=0x"
                      << mmio_read(static_cast<uint64_t>(DevReg::DMA_H2D_LEN))
                      << " D2H_LEN=0x"
                      << mmio_read(static_cast<uint64_t>(DevReg::DMA_D2H_LEN))
                      << std::dec << std::endl;
        }
    }
}

// Bulk transfers are sliced into 1 MiB chunks with a full MMIO sequence per
// chunk — mirroring the guest driver (my_qemu_edu.c TRACE_CHUNK_BYTES).
// Timed exactly as the driver's replay_one: one window around the whole
// chunked loop, including the per-chunk trailing STATUS clear.
static double do_bulk(uint64_t dma_addr, uint32_t size, bool d2h)
{
    auto start = clk::now();
    uint64_t remaining = size, off = 0;
    while (remaining > 0) {
        uint64_t chunk = remaining > TRACE_CHUNK_BYTES ? TRACE_CHUNK_BYTES : remaining;
        mmio_write(static_cast<uint64_t>(DevReg::DMA_SRC_ADDR), dma_addr + off);
        mmio_write(static_cast<uint64_t>(DevReg::DMA_DST_ADDR), dma_addr + off);
        mmio_write(static_cast<uint64_t>(d2h ? DevReg::DMA_D2H_LEN : DevReg::DMA_H2D_LEN),
                   chunk);
        mmio_write(static_cast<uint64_t>(DevReg::DMA_STATUS), 0);
        mmio_write(static_cast<uint64_t>(DevReg::DMA_CMD),
                   d2h ? DMA_CMD_D2H : DMA_CMD_H2D);
        poll_status(STATUS_DMA_DONE_MASK, d2h ? "BULK_D2H" : "BULK_H2D");
        mmio_write(static_cast<uint64_t>(DevReg::DMA_STATUS), 0);
        off += chunk;
        remaining -= chunk;
    }
    return secs(start, clk::now());
}

static double do_bundle(uint64_t dma_addr, uint32_t h2d, uint32_t d2h,
                        uint64_t cycles)
{
    // Synthetic compute-cycle knob: not a real-device register, stage outside
    // the timed window (same as sw_no_vm).
    mmio_write(static_cast<uint64_t>(DevReg::CYCLES_COMPUTE), cycles);
    const uint64_t cycles_readback =
        mmio_read(static_cast<uint64_t>(DevReg::CYCLES_COMPUTE));
    if (cycles_readback != cycles) {
        std::cerr << "[ERROR] CYCLES_COMPUTE readback mismatch: wrote " << cycles
                  << ", read " << cycles_readback << std::endl;
        throw std::runtime_error("CYCLES_COMPUTE readback mismatch");
    }

    auto start = clk::now();
    mmio_write(static_cast<uint64_t>(DevReg::DMA_SRC_ADDR), dma_addr);
    mmio_write(static_cast<uint64_t>(DevReg::DMA_DST_ADDR), dma_addr);
    mmio_write(static_cast<uint64_t>(DevReg::DMA_H2D_LEN), h2d);
    mmio_write(static_cast<uint64_t>(DevReg::DMA_D2H_LEN), d2h);
    mmio_write(static_cast<uint64_t>(DevReg::DMA_STATUS), 0);

    mmio_write(static_cast<uint64_t>(DevReg::START_COMPUTE), 1);
    poll_status(STATUS_BUNDLE_DONE_MASK, "BUNDLE");
    auto end = clk::now();
    // Trailing STATUS clear outside the timed window, as in the driver
    mmio_write(static_cast<uint64_t>(DevReg::DMA_STATUS), 0);
    return secs(start, end);
}

// The embedded traces.hpp is byte-identical to the guest driver's
// long-traces.h, so the driver's `set` column value applies directly and
// TRACE_EVENT / TRACE_SUMMARY lines are comparable with driver dmesg logs.
static constexpr const char *TRACE_SET_LABEL = "long";

// cycles_scale divides every bundle's compute cycles, modeling a faster
// accelerator (traces captured on Vortex-on-FPGA at 250 MHz; a
// Vortex-family ASIC clocks ~1.5 GHz, i.e. ~6x — arXiv:2512.00053).
// Mirrors the guest driver's cycles_scale module parameter; scaled cycles
// are programmed AND reported, so logs stay self-consistent. Sweep points:
// 1 (as captured) and 6.
static void run_traces(uint64_t dma_addr, uint64_t dma_capacity,
                       int n_runs, uint64_t cycles_scale,
                       std::vector<TraceResult> &results)
{
    if (cycles_scale == 0) cycles_scale = 1;
    std::cout << "TRACE_CSV: cycles_scale=" << cycles_scale << std::endl;
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
            // Per-(app, run) aggregates, exactly as the driver's replay_one
            int agg_n_bulk_h2d = 0, agg_n_bulk_d2h = 0, agg_n_bundle = 0;
            uint64_t agg_h2d_ns = 0, agg_d2h_ns = 0, agg_bundle_ns = 0;
            uint64_t agg_h2d_bytes = 0, agg_d2h_bytes = 0, agg_cycles = 0;

            for (size_t i = 0; i < app.n; i++) {
                const auto &ev = app.events[i];
                double total_s = 0;
                uint64_t h2d = 0, d2h = 0;
                const uint64_t cycles = ev.cycles / cycles_scale;

                std::cerr << "[trace]   " << app.name << " ev=" << i << "/" << app.n
                          << " kind=" << kind_str(ev.kind)
                          << " h2d=" << ev.h2d_size << " d2h=" << ev.d2h_size
                          << " cycles=" << cycles << " ... " << std::flush;

                switch (ev.kind) {
                case TRACE_BULK_H2D:
                    h2d = clamp_dma(ev.h2d_size, dma_capacity);
                    break;
                case TRACE_BULK_D2H:
                    d2h = clamp_dma(ev.d2h_size, dma_capacity);
                    break;
                case TRACE_BUNDLE:
                    h2d = clamp_dma(ev.h2d_size, dma_capacity);
                    d2h = clamp_dma(ev.d2h_size, dma_capacity);
                    break;
                default:
                    std::cerr << "[warn] unknown kind " << (int)ev.kind << std::endl;
                    continue;
                }

                // Same schema as the driver:
                // TRACE_EVENT: set,app,run,event,kind,h2d_bytes,d2h_bytes,cycles
                std::cout << "TRACE_EVENT: " << TRACE_SET_LABEL << ","
                          << app.name << "," << run << "," << i << ","
                          << static_cast<unsigned>(ev.kind) << "," << h2d << ","
                          << d2h << "," << cycles << std::endl;

                switch (ev.kind) {
                case TRACE_BULK_H2D:
                    total_s = do_bulk(dma_addr, static_cast<uint32_t>(h2d), false);
                    break;
                case TRACE_BULK_D2H:
                    total_s = do_bulk(dma_addr, static_cast<uint32_t>(d2h), true);
                    break;
                case TRACE_BUNDLE:
                    total_s = do_bundle(dma_addr, static_cast<uint32_t>(h2d),
                                        static_cast<uint32_t>(d2h), cycles);
                    break;
                }

                std::cerr << "done " << (total_s * 1e6) << " us" << std::endl;

                uint64_t total_ns = static_cast<uint64_t>(total_s * 1e9);
                switch (ev.kind) {
                case TRACE_BULK_H2D:
                    agg_n_bulk_h2d++;
                    agg_h2d_ns    += total_ns;
                    agg_h2d_bytes += ev.h2d_size;  // raw size, as the driver
                    break;
                case TRACE_BULK_D2H:
                    agg_n_bulk_d2h++;
                    agg_d2h_ns    += total_ns;
                    agg_d2h_bytes += ev.d2h_size;
                    break;
                case TRACE_BUNDLE:
                    agg_n_bundle++;
                    agg_bundle_ns += total_ns;
                    agg_h2d_bytes += ev.h2d_size;
                    agg_d2h_bytes += ev.d2h_size;
                    agg_cycles    += cycles;
                    break;
                }

                results.push_back({app.name, run, static_cast<int>(i), ev.kind,
                                   ev.h2d_size, ev.d2h_size, cycles,
                                   total_s * 1e6, ev.original_count});
            }

            // Same schema as the driver:
            // TRACE_SUMMARY: set,app,run,n_events,n_bulk_h2d,n_bulk_d2h,
            //                n_bundle,total_h2d_ns,total_d2h_ns,
            //                total_bundle_ns,total_ns,total_h2d_bytes,
            //                total_d2h_bytes,total_cycles
            std::cout << "TRACE_SUMMARY: " << TRACE_SET_LABEL << ","
                      << app.name << "," << run << "," << app.n << ","
                      << agg_n_bulk_h2d << "," << agg_n_bulk_d2h << ","
                      << agg_n_bundle << "," << agg_h2d_ns << ","
                      << agg_d2h_ns << "," << agg_bundle_ns << ","
                      << (agg_h2d_ns + agg_d2h_ns + agg_bundle_ns) << ","
                      << agg_h2d_bytes << "," << agg_d2h_bytes << ","
                      << agg_cycles << std::endl;
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
    uint64_t cycles_scale = 1;

    boost::program_options::options_description opts("Jigsaw SW Forwarder (no VM) Options");
    opts.add_options()
        ("ip_address,i",
            boost::program_options::value<std::string>(&device_ip),
            "Device-side OOB TCP/IP address (for QP exchange)")
        ("trace_runs,r",
            boost::program_options::value<int>(&trace_runs),
            "Runs per trace application")
        ("cycles_scale,c",
            boost::program_options::value<uint64_t>(&cycles_scale),
            "Divide bundle compute cycles by this factor (default 1; 6 = Vortex-ASIC-class accelerator)");

    boost::program_options::variables_map vm;
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, argv, opts), vm);
    boost::program_options::notify(vm);

    if (device_ip.empty()) {
        std::cout << "ERROR: --ip_address (-i) is required\n" << opts << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Starting Jigsaw SW Forwarder — Host (no VM)..." << std::endl;
    std::cout << "Device OOB IP : " << device_ip << std::endl;

    coyote::cThread coyote_thread(DEFAULT_VFPGA_ID, getpid());
    ct = &coyote_thread;

    nic_buf = static_cast<char *>(
        coyote_thread.initRDMA(BUF_BYTES, coyote::DEF_PORT, device_ip.c_str()));
    if (!nic_buf) {
        std::cerr << "initRDMA failed" << std::endl;
        return EXIT_FAILURE;
    }
    memset(nic_buf, 0, CONTROL_SIZE);
    req_slot = reinterpret_cast<struct msg *>(nic_buf + REQ_OFF);
    resp_slot = reinterpret_cast<volatile struct msg *>(nic_buf + RESP_OFF);

    // Application buffer standing in for the guest/ivshmem memory the
    // forwarder serves; payload region mirrors the QP buffer layout.
    if (posix_memalign(reinterpret_cast<void **>(&app_buf), 4096, BUF_BYTES) != 0) {
        std::cerr << "application buffer allocation failed" << std::endl;
        return EXIT_FAILURE;
    }
    memset(app_buf, 0xAB, BUF_BYTES);  // pre-fault + recognizable payload

    // Initial sync with the device
    coyote_thread.connSync(true);
    std::cout << "RDMA connection established." << std::endl;

    (void)request(OP_SETUP, 0, reinterpret_cast<uint64_t>(app_buf));

    uint64_t dma_addr = reinterpret_cast<uint64_t>(app_buf) + PAYLOAD_OFF;
    uint64_t dma_capacity = BUF_BYTES - PAYLOAD_OFF;

    std::vector<TraceResult> trace_results;
    run_traces(dma_addr, dma_capacity, trace_runs, cycles_scale, trace_results);

    (void)request(OP_STOP, 0, 0);

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

    coyote_thread.connSync(true);
    coyote_thread.closeConn();

    std::cout << "All benchmarks completed." << std::endl;
    return EXIT_SUCCESS;
}
