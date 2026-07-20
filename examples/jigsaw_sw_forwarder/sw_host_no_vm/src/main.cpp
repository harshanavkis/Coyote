/**
 * Jigsaw Software Forwarder — host-side trace harness (no VM)
 *
 * Structured exactly like jigsaw_baseline_rdma/sw_client_coyote_api: one
 * cThread on the perf_rdma vFPGA, a volatile 64 B mailbox at offset 0 of
 * the QP buffer with a clear-and-wait ready flag, payloads pushed into the
 * region behind the control page, and — the keystone of the proven May
 * discipline — a full quiesce (SYNC exchange + clearCompleted + TCP
 * connSync barrier) before EVERY payload-bearing transfer, so the wire
 * never carries more than a handful of writes between barriers.
 *
 * Drives the same device interactions as jigsaw_host_controller/sw_no_vm
 * (Vortex OpenCL trace replay), but every MMIO access is encapsulated into
 * a mailbox request and replayed by the device-side replayer on the
 * unmodified jigsaw_baseline accelerator. The device executes triggers
 * synchronously and completes only afterwards (any D2H payload pushed
 * ahead of the completion on the same QP), so a completion means done.
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

// Globals (May-client style: plain file-scope state, logic in helpers)
static coyote::cThread *ct;
static char *nic_buf;
static char *app_buf;
static volatile struct msg *mailbox;

// Shadow copies of the last-written DMA parameter registers, needed to
// stage payloads at trigger time (same role as on the device side).
static uint64_t sh_src, sh_dst, sh_h2d, sh_d2h;

static const uint64_t RESP_TIMEOUT_MS = 5000;

// ---------------------------------------------------------------------------
// Mailbox primitives (single slot, clear-and-wait ready flag, as in the
// jigsaw_baseline_rdma client)
// ---------------------------------------------------------------------------
static void mbox_request(uint64_t op, uint64_t addr, uint64_t value)
{
    mailbox->ready = 0;
    mailbox->type = MSG_REQUEST;
    mailbox->op = op;
    mailbox->addr = addr;
    mailbox->value = value;
    mailbox->ready = 1;
    coyote::rdmaSg sg = {.local_offs = 0, .remote_offs = 0, .len = 64};
    ct->invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);
}

static uint64_t wait_completion(uint64_t op, uint64_t addr)
{
    uint64_t deadline = now_ms() + RESP_TIMEOUT_MS;
    while (!(mailbox->ready == 1 && mailbox->type == MSG_COMPLETION)) {
        if (now_ms() > deadline) {
            std::cerr << "[mbox] FATAL: no completion for op=" << op
                      << " addr=0x" << std::hex << addr << std::dec
                      << std::endl;
            abort();
        }
    }
    return mailbox->value;
}

static uint64_t request(uint64_t op, uint64_t addr, uint64_t value)
{
    mbox_request(op, addr, value);
    return wait_completion(op, addr);
}

// Full quiesce before every payload-bearing transfer — the May client's
// "Structural Synchronization & Queue Flush": SYNC exchange, then both
// sides flush completion state and meet in the TCP barrier with an idle
// wire. Between two barriers the wire carries only a handful of writes,
// the envelope in which this stack is proven reliable.
static void quiesce()
{
    (void)request(OP_SYNC, 0, 0);
    ct->clearCompleted();
    ct->connSync(false);
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

// D2H: by the time the trigger's completion arrived, the device's payload
// push has landed (same QP, placed first) — bounce it into the app buffer.
static void stage_out(uint64_t dst_ptr, uint64_t len)
{
    uint64_t off = dst_ptr - reinterpret_cast<uint64_t>(app_buf);
    if (!payload_range_ok(off, len)) return;
    memcpy(app_buf + off, nic_buf + off, len);
}

// Forward one MMIO access. Writes to trigger registers carry their payload
// dance; the device executes triggers synchronously, so the completion of
// the trigger write itself means the operation is done.
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
            (void)request(OP_MMIO_WRITE, addr, value);
            return;
        }
        if (value == DMA_CMD_D2H) {
            (void)request(OP_MMIO_WRITE, addr, value);
            stage_out(sh_dst, sh_d2h);
            return;
        }
        break;
    case DevReg::START_COMPUTE:
        if (value == 1) {
            if (sh_h2d) stage_in(sh_src, sh_h2d);
            (void)request(OP_MMIO_WRITE, addr, value);
            if (sh_d2h) stage_out(sh_dst, sh_d2h);
            return;
        }
        break;
    default:
        break;
    }
    (void)request(OP_MMIO_WRITE, addr, value);
}

static uint64_t mmio_read(uint64_t addr)
{
    return request(OP_MMIO_READ, addr, 0);
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
// Each chunk cycle is bracketed by a quiesce, exactly one May-client
// iteration; the quiesce sits outside the timed window, as in the May
// benchmarks. Timing accumulates over the chunks of an event.
static double do_bulk(uint64_t dma_addr, uint32_t size, bool d2h)
{
    double total_s = 0;
    uint64_t remaining = size, off = 0;
    while (remaining > 0) {
        uint64_t chunk = remaining > TRACE_CHUNK_BYTES ? TRACE_CHUNK_BYTES : remaining;

        quiesce();

        auto start = clk::now();
        mmio_write(static_cast<uint64_t>(DevReg::DMA_SRC_ADDR), dma_addr + off);
        mmio_write(static_cast<uint64_t>(DevReg::DMA_DST_ADDR), dma_addr + off);
        mmio_write(static_cast<uint64_t>(d2h ? DevReg::DMA_D2H_LEN : DevReg::DMA_H2D_LEN),
                   chunk);
        mmio_write(static_cast<uint64_t>(DevReg::DMA_STATUS), 0);
        mmio_write(static_cast<uint64_t>(DevReg::DMA_CMD),
                   d2h ? DMA_CMD_D2H : DMA_CMD_H2D);
        poll_status(STATUS_DMA_DONE_MASK, d2h ? "BULK_D2H" : "BULK_H2D");
        mmio_write(static_cast<uint64_t>(DevReg::DMA_STATUS), 0);
        total_s += secs(start, clk::now());

        off += chunk;
        remaining -= chunk;
    }
    return total_s;
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

    quiesce();

    auto start = clk::now();
    mmio_write(static_cast<uint64_t>(DevReg::DMA_SRC_ADDR), dma_addr);
    mmio_write(static_cast<uint64_t>(DevReg::DMA_DST_ADDR), dma_addr);
    mmio_write(static_cast<uint64_t>(DevReg::DMA_H2D_LEN), h2d);
    mmio_write(static_cast<uint64_t>(DevReg::DMA_D2H_LEN), d2h);
    mmio_write(static_cast<uint64_t>(DevReg::DMA_STATUS), 0);

    mmio_write(static_cast<uint64_t>(DevReg::START_COMPUTE), 1);
    poll_status(STATUS_BUNDLE_DONE_MASK, "BUNDLE");
    auto end = clk::now();
    return secs(start, end);
}

static void run_traces(uint64_t dma_addr, uint64_t dma_capacity,
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

                results.push_back({app.name, run, static_cast<int>(i), ev.kind,
                                   ev.h2d_size, ev.d2h_size, ev.cycles,
                                   total_s * 1e6, ev.original_count});
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
    mailbox = reinterpret_cast<volatile struct msg *>(nic_buf);

    // Application buffer standing in for the guest/ivshmem memory the
    // forwarder serves; payload region mirrors the QP buffer layout.
    if (posix_memalign(reinterpret_cast<void **>(&app_buf), 4096, BUF_BYTES) != 0) {
        std::cerr << "application buffer allocation failed" << std::endl;
        return EXIT_FAILURE;
    }
    memset(app_buf, 0xAB, BUF_BYTES);  // pre-fault + recognizable payload

    // Initial sync with the device (May-client parity)
    coyote_thread.connSync(true);
    std::cout << "RDMA connection established." << std::endl;

    (void)request(OP_SETUP, 0, reinterpret_cast<uint64_t>(app_buf));

    uint64_t dma_addr = reinterpret_cast<uint64_t>(app_buf) + PAYLOAD_OFF;
    uint64_t dma_capacity = BUF_BYTES - PAYLOAD_OFF;

    std::vector<TraceResult> trace_results;
    run_traces(dma_addr, dma_capacity, trace_runs, trace_results);

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

    coyote_thread.connSync(false);
    coyote_thread.closeConn();

    std::cout << "All benchmarks completed." << std::endl;
    return EXIT_SUCCESS;
}
