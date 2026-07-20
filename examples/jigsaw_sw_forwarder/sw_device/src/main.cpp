/**
 * Jigsaw Software Forwarder — device-side replayer
 *
 * Two cThreads as in jigsaw_baseline_rdma/sw_server_coyote_api: vFPGA 0 =
 * perf_rdma dumb NIC, vFPGA 1 = unmodified jigsaw_baseline accelerator.
 *
 * MMIO accesses are replayed VERBATIM, exactly as the guest driver issues
 * them: a write is setCSR + immediate response, a read is getCSR — the
 * device never waits on the accelerator itself; the host's forwarded
 * STATUS polls do the waiting, preserving the driver's contract.
 *
 * The only forwarding additions around the raw replay:
 *  - H2D triggers bounce the payload (pushed by the host ahead of the
 *    trigger on the same QP) from the NIC buffer into the device staging
 *    buffer before the CSR write.
 *  - D2H data is pushed lazily: when a STATUS read observes the armed
 *    completion bits, the payload is bounced device buffer -> NIC buffer
 *    and pushed BEFORE that read's response (same QP, placed first), so
 *    the host can never see "done" before the data has landed.
 *  - Guest DMA pointers in MMIO writes are rewritten to the device
 *    staging buffer at the same payload offset. NIC buffer and device
 *    buffer are deliberately separate (the staging copy a software
 *    forwarder on commodity hardware cannot avoid).
 *
 * Wire discipline: strict ping-pong, one 64 B slot per direction with a
 * monotonic publish counter (see messages.hpp) so hardware-level replays
 * are recognized and never re-executed. Completion state is cleared on
 * both threads after every request (the baseline server's cadence).
 *
 * Usage:
 *   ./test            (waits for the host forwarder to connect)
 */

#include <cstring>
#include <iostream>
#include <unistd.h>

#include <coyote/cThread.hpp>

#include "messages.hpp"

using namespace jsfwd;

// Constants
#define NIC_VFPGA_ID    0
#define JIGSAW_VFPGA_ID 1

// Globals (server style: plain file-scope state, logic in main)
static coyote::cThread *nic;
static coyote::cThread *jig;
static char *nic_buf;
static char *device_buf;
static uint64_t app_base;

// Shadow copies of the last-written DMA parameter registers, needed to
// stage payloads and rewrite guest pointers at trigger time.
static uint64_t sh_src, sh_dst, sh_h2d, sh_d2h;

// Armed D2H: set when a trigger with a D2H phase is written; consumed by
// the first STATUS read that observes the corresponding done bits.
static struct { bool armed; uint64_t dst, len, mask; } d2h_pending;

// Guest DMA pointer -> device buffer address at the same payload offset
static uint64_t rewrite(uint64_t guest_ptr) {
    return reinterpret_cast<uint64_t>(device_buf) + (guest_ptr - app_base);
}

static bool payload_range_ok(uint64_t off, uint64_t len) {
    return off >= PAYLOAD_OFF && off < BUF_BYTES && len <= BUF_BYTES - off;
}

// H2D: the payload was pushed into our NIC buffer before the trigger
// request (same QP, placed in order) — bounce it into the device buffer.
static void stage_in(uint64_t src_ptr, uint64_t len) {
    uint64_t off = src_ptr - app_base;
    if (!payload_range_ok(off, len)) {
        std::cerr << "stage_in: payload out of range (off=0x" << std::hex
                  << off << ", len=0x" << len << std::dec << ")" << std::endl;
        return;
    }
    memcpy(device_buf + off, nic_buf + off, len);
}

// D2H: bounce the accelerator's output into the NIC buffer and push it to
// the host; the pending response follows on the same QP, so the host can
// never see completion before the data has landed.
static void stage_out(uint64_t dst_ptr, uint64_t len) {
    uint64_t off = dst_ptr - app_base;
    if (!payload_range_ok(off, len)) {
        std::cerr << "stage_out: payload out of range (off=0x" << std::hex
                  << off << ", len=0x" << len << std::dec << ")" << std::endl;
        return;
    }
    memcpy(nic_buf + off, device_buf + off, len);
    uint64_t aligned = (len + 63) & ~uint64_t(63);
    if (off + aligned > BUF_BYTES)
        aligned = BUF_BYTES - off;
    coyote::rdmaSg sg = {.local_offs = off, .remote_offs = off,
                         .len = static_cast<uint32_t>(aligned)};
    nic->invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);
}

// Replay one MMIO write on the accelerator, verbatim: every path ends in
// exactly the setCSR the driver would have issued, with no waiting.
static void handle_write(uint64_t addr, uint64_t value) {
    switch (static_cast<DevReg>(addr)) {
    case DevReg::DMA_SRC_ADDR:
        sh_src = value;
        jig->setCSR(rewrite(value), dev_reg_index(addr));
        return;
    case DevReg::DMA_DST_ADDR:
        sh_dst = value;
        jig->setCSR(rewrite(value), dev_reg_index(addr));
        return;
    case DevReg::DMA_H2D_LEN:
        sh_h2d = value;
        break;
    case DevReg::DMA_D2H_LEN:
        sh_d2h = value;
        break;
    case DevReg::DMA_CMD:
        if (value == DMA_CMD_H2D) {
            stage_in(sh_src, sh_h2d);
        } else if (value == DMA_CMD_D2H) {
            d2h_pending = {true, sh_dst, sh_d2h, STATUS_DMA_DONE_MASK};
        }
        break;
    case DevReg::START_COMPUTE:
        if (value == 1) {
            if (sh_h2d) stage_in(sh_src, sh_h2d);
            if (sh_d2h) d2h_pending = {true, sh_dst, sh_d2h,
                                       STATUS_BUNDLE_DONE_MASK};
        }
        break;
    default:
        break;
    }
    jig->setCSR(value, dev_reg_index(addr));
}

// Replay one MMIO read; a STATUS read that first observes the armed done
// bits pushes the pending D2H payload before its response is sent.
static uint64_t handle_read(uint64_t addr) {
    uint64_t value = jig->getCSR(dev_reg_index(addr));
    if (static_cast<DevReg>(addr) == DevReg::DMA_STATUS && d2h_pending.armed &&
        (value & d2h_pending.mask) == d2h_pending.mask) {
        stage_out(d2h_pending.dst, d2h_pending.len);
        d2h_pending.armed = false;
    }
    return value;
}

int main(int argc, char *argv[]) {
    std::cout << "Starting Jigsaw SW Forwarder — Device Replayer..." << std::endl;

    // vFPGA 0: perf_rdma (dumb NIC), vFPGA 1: jigsaw_baseline (accelerator)
    coyote::cThread coyote_nic(NIC_VFPGA_ID, getpid());
    coyote::cThread coyote_jigsaw(JIGSAW_VFPGA_ID, getpid());
    nic = &coyote_nic;
    jig = &coyote_jigsaw;

    std::cout << "Waiting for host connection on port " << coyote::DEF_PORT
              << " ..." << std::endl;
    nic_buf = static_cast<char *>(
        coyote_nic.initRDMA(BUF_BYTES, coyote::DEF_PORT));
    if (!nic_buf) {
        std::cerr << "initRDMA failed" << std::endl;
        return EXIT_FAILURE;
    }
    memset(nic_buf, 0, CONTROL_SIZE);

    // Staging buffer for the accelerator, mapped only into vFPGA 1's TLB
    device_buf = static_cast<char *>(
        coyote_jigsaw.getMem({coyote::CoyoteAllocType::HPF, BUF_BYTES}));
    if (!device_buf) {
        std::cerr << "device staging buffer allocation failed" << std::endl;
        return EXIT_FAILURE;
    }
    memset(device_buf, 0, BUF_BYTES);  // pre-fault

    coyote_jigsaw.setCSR(coyote_jigsaw.getCtid(), COYOTE_PID_REG);

    volatile struct msg *req  = reinterpret_cast<volatile struct msg *>(nic_buf + REQ_OFF);
    struct msg *resp = reinterpret_cast<struct msg *>(nic_buf + RESP_OFF);

    // Initial sync with the host
    coyote_nic.connSync(false);
    std::cout << "Host connected." << std::endl;

    uint64_t last_seq = 0;
    uint64_t served = 0;
    bool running = true;
    while (running) {
        // Poll for a new request: the monotonic publish counter advances
        // exactly once per request, so a hardware-level replay of an
        // already-seen message is ignored here.
        if (req->seq <= last_seq) {
            continue;
        }
        last_seq = req->seq;
        uint64_t op = req->op;
        uint64_t addr = req->addr;
        uint64_t value = req->value;
        served++;

        uint64_t result = 0;
        switch (op) {
        case OP_SETUP:
            app_base = value;
            std::cout << "Setup received: app_base = 0x" << std::hex
                      << app_base << std::dec << std::endl;
            break;
        case OP_MMIO_WRITE:
            handle_write(addr, value);
            break;
        case OP_MMIO_READ:
            result = handle_read(addr);
            break;
        case OP_STOP:
            running = false;
            break;
        default:
            std::cerr << "Unknown op: " << op << std::endl;
            break;
        }

        // Respond from the dedicated response slot (this node is its only
        // writer, so a pending retransmission always re-reads the bytes it
        // originally sent). seq mirrors the request's and is written last.
        resp->op = op;
        resp->addr = addr;
        resp->value = result;
        resp->seq = last_seq;
        coyote::rdmaSg sg = {.local_offs = RESP_OFF, .remote_offs = RESP_OFF,
                             .len = SLOT_BYTES};
        coyote_nic.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);

        // Cheap local flush every request, the baseline server's cadence
        coyote_nic.clearCompleted();
        coyote_jigsaw.clearCompleted();
    }

    std::cout << "Host posted STOP after " << served << " requests." << std::endl;

    coyote_nic.connSync(false);
    coyote_nic.closeConn();

    return EXIT_SUCCESS;
}
