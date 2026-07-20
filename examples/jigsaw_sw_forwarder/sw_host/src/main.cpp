/**
 * Jigsaw Software Forwarder — host-side VM daemon
 *
 * Software-only replacement for the jigsaw host controller: serves the
 * guest's MMIO requests from the ivshmem region (same doorbell protocol as
 * jigsaw_host_controller/sw, so the QEMU/guest stack runs unchanged), but
 * instead of writing them into the host-controller vFPGA it forwards them
 * over the Coyote RDMA stack (perf_rdma as a dumb NIC) to the device-side
 * replayer, which replays them verbatim on the accelerator.
 *
 * Identical wire protocol and primitives to sw_host_no_vm (see
 * messages.hpp): one 64 B slot per direction, monotonic publish counter,
 * strict ping-pong, clearCompleted per round trip, no retries. The
 * application buffer served here is the ivshmem region itself: guest DMA
 * pointers are proxy-shmem vaddrs (published at OFFSET_PROXY_SHMEM by
 * init_shared_memory), so they translate to payload offsets with the same
 * single base subtraction as the no-VM harness.
 *
 * DMA payloads bounce between ivshmem and the NIC (QP) buffer on this side
 * and between the NIC buffer and the device staging buffer on the device
 * side — the two staging copies of a software forwarder on commodity
 * hardware. An H2D payload is pushed before its trigger request; a D2H
 * payload is pushed by the device ahead of the first STATUS response that
 * reports "done" and bounced into ivshmem at that poll, before the guest
 * sees the doorbell.
 *
 * Meant to busy-poll on one dedicated core (run under taskset).
 *
 * Usage:
 *   taskset -c <core> ./test -i <device_oob_ip>
 */

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>

#include <boost/program_options.hpp>

#include <coyote/cThread.hpp>

#include "shmem.hpp"
#include "messages.hpp"

using namespace jsfwd;

// Constants
#define DEFAULT_VFPGA_ID 0

static_assert(SHMEM_SIZE == jsfwd::BUF_BYTES,
              "ivshmem and QP buffer must mirror each other");
static_assert(DMA_REGION_OFFSET == jsfwd::PAYLOAD_OFF,
              "ivshmem DMA region and QP payload region must line up");

// Globals (plain file-scope state, logic in helpers — same as sw_host_no_vm)
static coyote::cThread *ct;
static char *nic_buf;
static char *app_buf;                    // the ivshmem region
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

static std::atomic<bool> g_stop{false};
static void on_sigint(int) { g_stop.store(true); }

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
    coyote::rdmaSg sg = {.local_offs = REQ_OFF, .remote_offs = REQ_OFF, .len = 64};
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

// H2D: bounce guest memory (ivshmem) into the NIC buffer and push it before
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

// Forward one guest MMIO write, in the exact order the guest driver issues
// them. An H2D trigger is preceded by its payload push; a trigger with a
// D2H phase arms the copy-out consumed by the guest's STATUS poll below.
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
    // the NIC buffer by the time "done" is visible — bounce it into
    // ivshmem before the guest can observe the completed poll.
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
// Forwarding loop — same doorbell structure as run_shmem_app() in
// jigsaw_host_controller/sw, with the backend swapped for the wire protocol.
// ---------------------------------------------------------------------------
static void run_forwarder()
{
    std::cout << "SHMEM application started. Waiting for messages..." << std::endl;

    while (!g_stop.load(std::memory_order_relaxed))
    {
        shmem_arm_write_doorbell();

        shmem_wait_write_doorbell();

        mmio_message_header header;
        shmem_read_header(&header);

        switch (header.operation)
        {
        case OP_READ:
            shmem_complete_read(mmio_read(header.address));
            continue;

        case OP_WRITE:
            mmio_write(header.address, header.value);
            continue;

        default:
            fprintf(stderr, "Unknown operation: %d\n", header.operation);
            continue;
        }
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[]) {
    std::string device_ip;

    boost::program_options::options_description opts("Jigsaw SW Forwarder Options");
    opts.add_options()
        ("ip_address,i",
            boost::program_options::value<std::string>(&device_ip),
            "Device-side OOB TCP/IP address (for QP exchange)");

    boost::program_options::variables_map vm;
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, argv, opts), vm);
    boost::program_options::notify(vm);

    if (device_ip.empty()) {
        std::cout << "ERROR: --ip_address (-i) is required\n" << opts << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Starting Jigsaw SW Forwarder — Host (VM daemon)..." << std::endl;
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

    // Init shared memory with the guest; publishes the proxy DMA base the
    // guest hands out as DMA pointers (ivshmem base + DMA_REGION_OFFSET).
    app_buf = static_cast<char *>(init_shared_memory());
    if (!app_buf) {
        std::cerr << "init_shared_memory failed" << std::endl;
        return EXIT_FAILURE;
    }

    // Initial sync with the device
    coyote_thread.connSync(true);
    std::cout << "RDMA connection established." << std::endl;

    (void)request(OP_SETUP, 0, reinterpret_cast<uint64_t>(app_buf));

    std::signal(SIGINT, on_sigint);
    run_forwarder();

    std::cout << "Stopping..." << std::endl;
    (void)request(OP_STOP, 0, 0);

    coyote_thread.connSync(true);
    coyote_thread.closeConn();

    return EXIT_SUCCESS;
}
