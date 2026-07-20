/**
 * Jigsaw Software Forwarder — device-side replayer
 *
 * Software-only counterpart of the jigsaw device controller: receives the
 * host's encapsulated MMIO accesses over the Coyote RDMA stack (perf_rdma on
 * vFPGA 0 acting as a dumb NIC) and replays them on the unmodified
 * jigsaw_baseline accelerator (vFPGA 1) through its AXI-Lite CSRs.
 *
 * DMA payloads are staged through a dedicated device buffer: the NIC (QP)
 * buffer and the buffer the accelerator DMAs from/to are deliberately
 * separate, with a memcpy between them, because a software forwarder on
 * commodity hardware cannot point a device's DMA engine at the NIC's
 * buffers (separate DMA/IOMMU domains, buffers owned by different stacks).
 * Guest DMA pointers arriving in MMIO writes are rewritten to the device
 * buffer at the same payload offset, so alignment is preserved and no
 * translation tables are needed.
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

#include "wire.hpp"

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
          device_buf(device_buf),
          rx(nic_buf, REQ_RING_OFF, REQ_SLOTS),
          tx(nic, nic_buf, RESP_RING_OFF, RESP_SLOTS) {}

    void wait_setup() {
        volatile setup_msg *s =
            reinterpret_cast<volatile setup_msg *>(nic_buf + SETUP_OFF);
        while (s->magic != SETUP_MAGIC) {
            _mm_pause();
        }
        app_base = s->app_base;
        std::cout << "Setup received: app_base = 0x" << std::hex << app_base
                  << std::dec << ", payload = " << s->payload_bytes
                  << " bytes" << std::endl;
    }

    // Returns false once the host posted WIRE_STOP.
    bool serve_one() {
        wire_msg m = rx.wait();

        switch (m.op) {
        case WIRE_MMIO_WRITE:
            handle_write(m.addr, m.value);
            // ack after the write fully took effect (for DMA/compute
            // triggers that means after the device finished and any D2H
            // payload was pushed — the ack can then never overtake data)
            tx.post(WIRE_WRITE_ACK, m.addr, rx.consumed(), 0);
            last_credit = rx.consumed();
            break;
        case WIRE_MMIO_READ: {
            uint64_t value = jig.getCSR(dev_reg_index(m.addr));
            // len piggybacks the consumed count as a credit update
            tx.post(WIRE_READ_RESP, m.addr, rx.consumed(), value);
            last_credit = rx.consumed();
            break;
        }
        case WIRE_STOP:
            return false;
        default:
            std::cerr << "Unknown wire op: " << m.op << std::endl;
            break;
        }

        // Posted writes generate no responses, so return credits standalone
        // before the host's request window can fill up.
        if (rx.consumed() - last_credit >= REQ_SLOTS / 2) {
            send_credit();
        }
        return true;
    }

private:
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
    // arrived (same QP, ordered) — bounce it into the device buffer.
    void stage_in(uint64_t src_ptr, uint64_t len) {
        uint64_t off = src_ptr - app_base;
        if (!payload_range_ok(off, len)) {
            std::cerr << "stage_in: payload out of range (off=0x" << std::hex
                      << off << ", len=0x" << len << std::dec << ")" << std::endl;
            return;
        }
        memcpy(device_buf + off, nic_buf + off, len);
    }

    // D2H: bounce the accelerator's output into the NIC buffer and push it
    // to the host. The push precedes any later status-read response on the
    // same QP, so the host never sees "done" before the data has landed.
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

    void send_credit() {
        wire_msg *c = reinterpret_cast<wire_msg *>(nic_buf + CREDIT_OFF);
        memset(c, 0, sizeof(*c));
        c->seq = rx.consumed();  // last word: placed last at the host
        coyote::rdmaSg sg = {.local_offs = CREDIT_OFF, .remote_offs = CREDIT_OFF,
                             .len = WIRE_BYTES};
        nic.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);
        last_credit = rx.consumed();
    }

    coyote::cThread &nic;
    coyote::cThread &jig;
    char *nic_buf;
    char *device_buf;
    RxRing rx;
    TxRing tx;

    uint64_t app_base = 0;
    uint64_t sh_src = 0, sh_dst = 0, sh_h2d = 0, sh_d2h = 0;
    uint64_t last_credit = 0;
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

    replayer.wait_setup();

    uint64_t served = 0;
    while (replayer.serve_one()) {
        served++;
    }
    std::cout << "Host posted STOP after " << served << " requests." << std::endl;

    nic_thread.connSync(false);
    nic_thread.closeConn();

    return EXIT_SUCCESS;
}
