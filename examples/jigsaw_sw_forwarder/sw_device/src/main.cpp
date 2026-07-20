/**
 * Jigsaw Software Forwarder — device-side replayer
 *
 * Structured exactly like jigsaw_baseline_rdma/sw_server_coyote_api: two
 * cThreads (vFPGA 0 = perf_rdma dumb NIC, vFPGA 1 = unmodified
 * jigsaw_baseline accelerator), one volatile 64 B mailbox at offset 0 of
 * the QP buffer polled with a clear-and-wait ready flag, requests executed
 * synchronously and completed in place, and a full quiesce (clearCompleted
 * + connSync) whenever the host asks for one — which the host does before
 * every payload-bearing transfer, the cadence under which this stack is
 * proven reliable.
 *
 * The host forwards the guest's MMIO accesses as mailbox requests; they are
 * replayed on the accelerator through its CSRs. DMA triggers execute fully
 * before the completion is sent (the accelerator's STATUS is polled here,
 * like the May server's device_h2d/device_d2h), so by the time the host
 * sees the completion, the operation — including any D2H payload push,
 * which travels the same QP ahead of the completion — has finished.
 *
 * DMA payloads are staged through a dedicated device buffer (NIC buffer and
 * accelerator buffer deliberately separate, memcpy between them — the copy
 * a software forwarder on commodity hardware cannot avoid; cf. AvA shadow
 * buffers, rCUDA pinned pools). Guest DMA pointers arriving in MMIO writes
 * are rewritten to the device buffer at the same payload offset.
 *
 * Usage:
 *   ./test            (waits for the host forwarder to connect)
 */

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <coyote/cThread.hpp>

#include "messages.hpp"

using namespace jsfwd;

// Constants
#define CLOCK_PERIOD_NS 4
#define NIC_VFPGA_ID    0
#define JIGSAW_VFPGA_ID 1
#define DEF_PORT        coyote::DEF_PORT

// Globals (May-server style: plain file-scope state, logic in main)
static coyote::cThread *nic;
static coyote::cThread *jig;
static char *nic_buf;
static char *device_buf;
static uint64_t app_base;

// Shadow copies of the last-written DMA parameter registers, needed to
// stage payloads and rewrite guest pointers at trigger time.
static uint64_t sh_src, sh_dst, sh_h2d, sh_d2h;

// Guest DMA pointer -> device buffer address at the same payload offset
static uint64_t rewrite(uint64_t guest_ptr) {
    return reinterpret_cast<uint64_t>(device_buf) + (guest_ptr - app_base);
}

static bool payload_range_ok(uint64_t off, uint64_t len) {
    return off >= PAYLOAD_OFF && off < BUF_BYTES && len <= BUF_BYTES - off;
}

// Poll the accelerator's STATUS until `mask` is set, like the May server's
// device_h2d/device_d2h. Diagnose a wedged operation: dump the
// engine-visible registers once after 3 s stuck.
static void wait_status(uint64_t mask) {
    uint64_t start = now_ms();
    bool dumped = false;
    while ((jig->getCSR(dev_reg_index(
               static_cast<uint64_t>(DevReg::DMA_STATUS))) & mask) != mask) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(CLOCK_PERIOD_NS));
        if (!dumped && now_ms() - start > 3000) {
            dumped = true;
            std::cerr << "[dev] wait_status(0x" << std::hex << mask
                      << ") stuck 3s: STATUS=0x"
                      << jig->getCSR(dev_reg_index(static_cast<uint64_t>(DevReg::DMA_STATUS)))
                      << " CMD=0x"
                      << jig->getCSR(dev_reg_index(static_cast<uint64_t>(DevReg::DMA_CMD)))
                      << " TX_LEN=0x"
                      << jig->getCSR(dev_reg_index(static_cast<uint64_t>(DevReg::DMA_TX_LEN)))
                      << " H2D_LEN=0x"
                      << jig->getCSR(dev_reg_index(static_cast<uint64_t>(DevReg::DMA_H2D_LEN)))
                      << " D2H_LEN=0x"
                      << jig->getCSR(dev_reg_index(static_cast<uint64_t>(DevReg::DMA_D2H_LEN)))
                      << std::dec << std::endl;
        }
    }
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
// the host; the completion follows on the same QP, so the host can never
// see it before the data has landed.
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

// Replay one MMIO write on the accelerator. Triggers execute fully
// (STATUS polled) before this returns, so the completion sent afterwards
// means the operation is done.
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
            jig->setCSR(value, dev_reg_index(addr));
            wait_status(STATUS_DMA_DONE_MASK);
            return;
        }
        if (value == DMA_CMD_D2H) {
            jig->setCSR(value, dev_reg_index(addr));
            wait_status(STATUS_DMA_DONE_MASK);
            stage_out(sh_dst, sh_d2h);
            return;
        }
        break;
    case DevReg::START_COMPUTE:
        if (value == 1) {
            if (sh_h2d) stage_in(sh_src, sh_h2d);
            jig->setCSR(value, dev_reg_index(addr));
            wait_status(STATUS_BUNDLE_DONE_MASK);
            if (sh_d2h) stage_out(sh_dst, sh_d2h);
            return;
        }
        break;
    default:
        break;
    }
    jig->setCSR(value, dev_reg_index(addr));
}

int main(int argc, char *argv[]) {
    std::cout << "Starting Jigsaw SW Forwarder — Device Replayer..." << std::endl;

    // vFPGA 0: perf_rdma (dumb NIC), vFPGA 1: jigsaw_baseline (accelerator)
    coyote::cThread coyote_nic(NIC_VFPGA_ID, getpid());
    coyote::cThread coyote_jigsaw(JIGSAW_VFPGA_ID, getpid());
    nic = &coyote_nic;
    jig = &coyote_jigsaw;

    std::cout << "Waiting for host connection on port " << DEF_PORT << " ..."
              << std::endl;
    nic_buf = static_cast<char *>(coyote_nic.initRDMA(BUF_BYTES, DEF_PORT));
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

    volatile struct msg *mailbox = reinterpret_cast<volatile struct msg *>(nic_buf);

    // Initial sync with the host (May-server parity)
    coyote_nic.connSync(false);
    std::cout << "Host connected." << std::endl;

    uint64_t served = 0;
    bool running = true;
    while (running) {
        // Poll the mailbox for a request (clear-and-wait ready flag)
        if (!(mailbox->ready == 1 && mailbox->type == MSG_REQUEST)) {
            continue;
        }
        uint64_t op = mailbox->op;
        uint64_t addr = mailbox->addr;
        uint64_t value = mailbox->value;

        // Clear local ready flag immediately to prevent re-processing
        mailbox->ready = 0;
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
            result = jig->getCSR(dev_reg_index(addr));
            break;
        case OP_SYNC:
        case OP_STOP:
            // completion sent below; quiesce/exit handled after the send
            break;
        default:
            std::cerr << "Unknown op: " << op << std::endl;
            break;
        }

        // Signal completion back to the host: compose in place, write the
        // single 64 B mailbox into the host's slot (offset 0, same QP as
        // any D2H payload pushed above, so it is placed after the data).
        mailbox->type = MSG_COMPLETION;
        mailbox->value = result;
        mailbox->ready = 1;
        coyote::rdmaSg compl_sg = {.local_offs = 0, .remote_offs = 0, .len = 64};
        coyote_nic.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, compl_sg);

        if (op == OP_SYNC) {
            // Full quiesce, exactly as the May server does: flush completion
            // state on both threads and meet the host in the TCP barrier
            // with an idle wire.
            coyote_nic.clearCompleted();
            coyote_jigsaw.clearCompleted();
            coyote_nic.connSync(true);
        } else if (op == OP_STOP) {
            running = false;
        } else {
            // Cheap local flush every request, the baseline server's cadence
            coyote_nic.clearCompleted();
            coyote_jigsaw.clearCompleted();
        }
    }

    std::cout << "Host posted STOP after " << served << " requests." << std::endl;

    coyote_nic.connSync(true);
    coyote_nic.closeConn();

    return EXIT_SUCCESS;
}
