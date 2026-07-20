/**
 * Jigsaw Software Forwarder — device-side replayer
 *
 * Software-only counterpart of the jigsaw device controller: receives the
 * host's MMIO accesses as mailbox requests over the Coyote RDMA stack
 * (perf_rdma on vFPGA 0 acting as a dumb NIC) and replays them on the
 * unmodified jigsaw_baseline accelerator (vFPGA 1) through its CSRs.
 * Strict ping-pong: one request, one response, nothing else in flight —
 * the proven jigsaw_baseline_rdma coyote_api discipline, including the
 * periodic full quiesce (SYNC: clearCompleted + connSync).
 *
 * DMA payloads are staged through a dedicated device buffer: the NIC (QP)
 * buffer and the buffer the accelerator DMAs from/to are deliberately
 * separate, with a memcpy between them (a software forwarder on commodity
 * hardware cannot point a device's DMA engine at the NIC's buffers).
 * Guest DMA pointers arriving in MMIO writes are rewritten to the device
 * buffer at the same payload offset.
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

#include "mailbox.hpp"

using namespace jsfwd;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
#define CLOCK_PERIOD_NS 4
#define NIC_VFPGA_ID    0
#define JIGSAW_VFPGA_ID 1

// ---------------------------------------------------------------------------
// Replayer
// ---------------------------------------------------------------------------
class Replayer {
public:
    Replayer(coyote::cThread &nic_thread, coyote::cThread &jigsaw_thread,
             char *nic_buf, char *device_buf)
        : nic(nic_thread), jig(jigsaw_thread), nic_buf(nic_buf),
          device_buf(device_buf) {}

    // Serve one request; returns false once the host sent MBOX_STOP.
    bool serve_one() {
        mbox_msg m;
        mbox_wait(nic_buf, MBOX_REQ_OFF, last_rid, m);
        last_rid = m.req_id;
        served++;

        switch (m.type) {
        case MBOX_SETUP:
            app_base = m.addr;
            std::cout << "Setup received: app_base = 0x" << std::hex
                      << app_base << std::dec << ", payload = " << m.value
                      << " bytes" << std::endl;
            respond(m.req_id, 0);
            break;
        case MBOX_MMIO_WRITE:
            // respond only after the write fully took effect: for DMA and
            // compute triggers that is after the device finished and any
            // D2H payload was pushed — the response can never overtake data
            handle_write(m.addr, m.value);
            respond(m.req_id, 0);
            break;
        case MBOX_MMIO_READ:
            respond(m.req_id, jig.getCSR(dev_reg_index(m.addr)));
            break;
        case MBOX_SYNC:
            // full quiesce, as the May software does around every
            // iteration: ack first, then both sides clear completion
            // state and meet in the TCP barrier with an idle wire
            respond(m.req_id, 0);
            nic.clearCompleted();
            nic.connSync(false);
            break;
        case MBOX_STOP:
            respond(m.req_id, 0);
            return false;
        default:
            std::cerr << "Unknown mailbox type: " << m.type << std::endl;
            respond(m.req_id, 0);
            break;
        }
        return true;
    }

    uint64_t requests_served() const { return served; }

private:
    void respond(uint64_t req_id, uint64_t value) {
        mbox_send(nic, nic_buf, MBOX_RESP_OFF, 0, 0, value, req_id);
    }

    void handle_write(uint64_t addr, uint64_t value) {
        switch (static_cast<DevReg>(addr)) {
        case DevReg::DMA_SRC_ADDR:
            sh_src = value;
            jig.setCSR(rewrite(value), dev_reg_index(addr));
            return;
        case DevReg::DMA_DST_ADDR:
            sh_dst = value;
            jig.setCSR(rewrite(value), dev_reg_index(addr));
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
                jig.setCSR(value, dev_reg_index(addr));
                return;
            }
            if (value == DMA_CMD_D2H) {
                jig.setCSR(value, dev_reg_index(addr));
                wait_status(STATUS_DMA_DONE_MASK);
                stage_out(sh_dst, sh_d2h);
                return;
            }
            break;
        case DevReg::START_COMPUTE:
            if (value == 1) {
                if (sh_h2d) stage_in(sh_src, sh_h2d);
                jig.setCSR(value, dev_reg_index(addr));
                wait_status(STATUS_BUNDLE_DONE_MASK);
                if (sh_d2h) stage_out(sh_dst, sh_d2h);
                return;
            }
            break;
        default:
            break;
        }
        jig.setCSR(value, dev_reg_index(addr));
    }

    // Guest DMA pointer -> device buffer address at the same payload offset
    uint64_t rewrite(uint64_t guest_ptr) const {
        return reinterpret_cast<uint64_t>(device_buf) + (guest_ptr - app_base);
    }

    bool payload_range_ok(uint64_t off, uint64_t len) const {
        return off >= PAYLOAD_OFF && off < BUF_BYTES && len <= BUF_BYTES - off;
    }

    // H2D: the payload was pushed into our NIC buffer before the trigger
    // request (same QP, placed in order) — bounce it into the device buffer.
    void stage_in(uint64_t src_ptr, uint64_t len) {
        uint64_t off = src_ptr - app_base;
        if (!payload_range_ok(off, len)) {
            std::cerr << "stage_in: payload out of range (off=0x" << std::hex
                      << off << ", len=0x" << len << std::dec << ")" << std::endl;
            return;
        }
        memcpy(device_buf + off, nic_buf + off, len);
    }

    // D2H: bounce the accelerator's output into the NIC buffer and push it;
    // the trigger's response follows on the same QP, so the host can never
    // see completion before the data has landed.
    void stage_out(uint64_t dst_ptr, uint64_t len) {
        uint64_t off = dst_ptr - app_base;
        if (!payload_range_ok(off, len)) {
            std::cerr << "stage_out: payload out of range (off=0x" << std::hex
                      << off << ", len=0x" << len << std::dec << ")" << std::endl;
            return;
        }
        memcpy(nic_buf + off, device_buf + off, len);
        push_payload(nic, off, len);
    }

    void wait_status(uint64_t mask) {
        while ((jig.getCSR(dev_reg_index(
                   static_cast<uint64_t>(DevReg::DMA_STATUS))) & mask) != mask) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(CLOCK_PERIOD_NS));
        }
    }

    coyote::cThread &nic;
    coyote::cThread &jig;
    char *nic_buf;
    char *device_buf;

    uint64_t app_base = 0;
    uint64_t last_rid = 0;
    uint64_t served = 0;
    uint64_t sh_src = 0, sh_dst = 0, sh_h2d = 0, sh_d2h = 0;
};

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[]) {
    HEADER("JIGSAW SW FORWARDER — DEVICE REPLAYER");

    // vFPGA 0: perf_rdma (dumb NIC), vFPGA 1: jigsaw_baseline (accelerator)
    coyote::cThread nic_thread(NIC_VFPGA_ID, getpid());
    coyote::cThread jigsaw_thread(JIGSAW_VFPGA_ID, getpid());

    std::cout << "Waiting for host connection on port " << coyote::DEF_PORT
              << " ..." << std::endl;
    char *nic_buf = static_cast<char *>(
        nic_thread.initRDMA(BUF_BYTES, coyote::DEF_PORT));
    if (!nic_buf) {
        std::cerr << "initRDMA failed" << std::endl;
        return EXIT_FAILURE;
    }
    memset(nic_buf, 0, CTRL_BYTES);

    // Device staging buffer, mapped only into the accelerator's TLB
    char *device_buf = static_cast<char *>(
        jigsaw_thread.getMem({coyote::CoyoteAllocType::HPF, BUF_BYTES}));
    if (!device_buf) {
        std::cerr << "device staging buffer allocation failed" << std::endl;
        return EXIT_FAILURE;
    }
    memset(device_buf, 0, BUF_BYTES);  // pre-fault

    jigsaw_thread.setCSR(jigsaw_thread.getCtid(), COYOTE_PID_REG);

    Replayer replayer(nic_thread, jigsaw_thread, nic_buf, device_buf);

    // Barrier: both sides have zeroed their control pages
    nic_thread.connSync(false);
    std::cout << "Host connected." << std::endl;

    while (replayer.serve_one()) {
    }
    std::cout << "Host posted STOP after " << replayer.requests_served()
              << " requests." << std::endl;

    nic_thread.connSync(false);
    nic_thread.closeConn();

    return EXIT_SUCCESS;
}
