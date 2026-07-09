#ifndef JIGSAW_SW_FORWARDER_WIRE_HPP
#define JIGSAW_SW_FORWARDER_WIRE_HPP

/**
 * Jigsaw Software Forwarder — wire protocol
 *
 * Software-only forwarding of jigsaw device interactions over the Coyote
 * RDMA stack used as a dumb NIC (perf_rdma vFPGA, REMOTE_RDMA_WRITE only).
 * The host encapsulates the guest's MMIO accesses into 64-byte messages and
 * posts them into a request ring in the device node's QP buffer; the device
 * replays them on the local jigsaw_baseline vFPGA and answers reads through
 * a response ring. Bulk DMA payloads travel through the payload region of
 * the QP buffers, staged with an explicit memcpy on both ends (application
 * buffer <-> NIC buffer on the host, NIC buffer <-> device buffer on the
 * device) — the two bounce copies a software forwarder on commodity
 * hardware cannot avoid.
 *
 * Ordering relies on the single RC QP: ring slots, payload pushes and
 * responses are placed at the target in posting order, and within one
 * 64-byte write data is placed in increasing address order, so the trailing
 * `seq` word publishes the rest of the slot.
 */

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <immintrin.h>

#include <coyote/cThread.hpp>

namespace jsfwd {

// ---------------------------------------------------------------------------
// QP buffer layout (identical on both nodes)
//
// The payload region mirrors the ivshmem layout one-to-one: ivshmem offset X
// corresponds to QP buffer offset X on both nodes, so guest DMA pointers
// translate with a single base subtraction and no lookup tables.
// ---------------------------------------------------------------------------
constexpr uint32_t WIRE_BYTES   = 64;
constexpr uint32_t REQ_RING_OFF = 0x000;   // host -> device requests
constexpr uint32_t REQ_SLOTS    = 32;
constexpr uint32_t RESP_RING_OFF= 0x800;   // device -> host responses
constexpr uint32_t RESP_SLOTS   = 16;
constexpr uint32_t CREDIT_OFF   = 0xC00;   // device -> host consumed counter
constexpr uint32_t SETUP_OFF    = 0xC40;   // host -> device one-time setup
constexpr uint32_t CTRL_BYTES   = 0x1000;
constexpr uint32_t PAYLOAD_OFF  = 0x1000;  // == ivshmem DMA_REGION_OFFSET
constexpr uint32_t BUF_BYTES    = 16U * 1024 * 1024;  // == ivshmem SHMEM_SIZE

constexpr uint64_t SETUP_MAGIC  = 0x4a53465744313641ULL;

enum WireOp : uint64_t {
    WIRE_MMIO_WRITE = 1,  // posted, no reply
    WIRE_MMIO_READ  = 2,
    WIRE_READ_RESP  = 3,  // len carries the device's consumed request count
    WIRE_STOP       = 4,
};

// `seq` is the last word on purpose: RDMA WRITE payload is placed in
// increasing address order, so a slot is complete once its seq matches.
struct wire_msg {
    uint64_t op;
    uint64_t addr;
    uint64_t len;
    uint64_t value;
    uint64_t pad[3];
    uint64_t seq;
};
static_assert(sizeof(wire_msg) == WIRE_BYTES, "wire_msg must be one slot");

struct setup_msg {
    uint64_t app_base;       // host vaddr the guest's DMA pointers live in
    uint64_t payload_bytes;
    uint64_t pad[5];
    uint64_t magic;          // written last (highest address)
};
static_assert(sizeof(setup_msg) == WIRE_BYTES, "setup_msg must be one slot");

// ---------------------------------------------------------------------------
// Device register map of the jigsaw_baseline vFPGA, as byte offsets in the
// guest-visible BAR (same map as jigsaw_host_controller/sw_no_vm). The
// replayer maps an offset to a Coyote CSR index with a single shift.
// ---------------------------------------------------------------------------
enum class DevReg : uint64_t {
    DMA_CMD       = 0x00,
    DMA_SRC_ADDR  = 0x08,
    DMA_DST_ADDR  = 0x10,
    DMA_H2D_LEN   = 0x18,
    DMA_STATUS    = 0x20,
    START_COMPUTE = 0x28,
    CYCLES_COMPUTE= 0x30,
    DMA_TX_LEN    = 0x38,
    DMA_D2H_LEN   = 0x40,
};
constexpr uint32_t dev_reg_index(uint64_t addr) {
    return static_cast<uint32_t>(addr >> 3);
}
constexpr uint32_t COYOTE_PID_REG = 9;

constexpr uint64_t DMA_CMD_H2D = 1;
constexpr uint64_t DMA_CMD_D2H = 3;
constexpr uint64_t STATUS_DMA_DONE_MASK    = 0x1;
constexpr uint64_t STATUS_BUNDLE_DONE_MASK = 0x3;

// ---------------------------------------------------------------------------
// Rings over REMOTE_RDMA_WRITE
// ---------------------------------------------------------------------------

// Sender side: compose in the local QP buffer, write to the mirrored slot in
// the peer's QP buffer. Flow control (request ring only) via the consumed
// counter the peer maintains in our credit slot and piggybacks on responses.
class TxRing {
public:
    TxRing(coyote::cThread &ct, char *local, uint32_t ring_off, uint32_t nslots,
           volatile uint64_t *credit_word = nullptr)
        : ct(ct), local(local), ring_off(ring_off), nslots(nslots),
          credit_word(credit_word) {}

    void post(uint64_t op, uint64_t addr, uint64_t len, uint64_t value) {
        uint64_t seq = next_seq + 1;
        if (credit_word) {
            while (seq - credit_seen > nslots - 2) {
                uint64_t c = *credit_word;
                if (c > credit_seen) credit_seen = c;
                _mm_pause();
            }
        }

        uint32_t slot = ring_off + static_cast<uint32_t>((seq - 1) % nslots) * WIRE_BYTES;
        wire_msg *m = reinterpret_cast<wire_msg *>(local + slot);
        m->op = op;
        m->addr = addr;
        m->len = len;
        m->value = value;
        m->pad[0] = m->pad[1] = m->pad[2] = 0;
        m->seq = seq;

        coyote::rdmaSg sg = {.local_offs = slot, .remote_offs = slot, .len = WIRE_BYTES};
        ct.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);
        next_seq = seq;
    }

    void note_credit(uint64_t c) { if (c > credit_seen) credit_seen = c; }

private:
    coyote::cThread &ct;
    char *local;
    uint32_t ring_off;
    uint32_t nslots;
    volatile uint64_t *credit_word;
    uint64_t next_seq = 0;
    uint64_t credit_seen = 0;
};

// Receiver side: poll the next expected slot in the local QP buffer.
class RxRing {
public:
    RxRing(char *local, uint32_t ring_off, uint32_t nslots)
        : local(local), ring_off(ring_off), nslots(nslots) {}

    wire_msg wait() {
        uint64_t seq = seen + 1;
        volatile wire_msg *m = reinterpret_cast<volatile wire_msg *>(
            local + ring_off + static_cast<uint32_t>((seq - 1) % nslots) * WIRE_BYTES);
        while (m->seq != seq) {
            _mm_pause();
        }
        std::atomic_thread_fence(std::memory_order_acquire);
        wire_msg out;
        out.op = m->op;
        out.addr = m->addr;
        out.len = m->len;
        out.value = m->value;
        out.seq = m->seq;
        seen = seq;
        return out;
    }

    uint64_t consumed() const { return seen; }

private:
    char *local;
    uint32_t ring_off;
    uint32_t nslots;
    uint64_t seen = 0;
};

// Push [off, off+len) of the local payload region to the same offsets on the
// peer. Segmented into 1 MiB invocations; segmentation is at the network
// level only — callers finish their staging memcpy before pushing.
inline void push_payload(coyote::cThread &ct, uint64_t off, uint64_t len) {
    constexpr uint64_t SEG = 1UL << 20;
    uint64_t end = off + ((len + WIRE_BYTES - 1) & ~uint64_t(WIRE_BYTES - 1));
    if (end > BUF_BYTES) end = BUF_BYTES;
    for (uint64_t o = off; o < end; o += SEG) {
        coyote::rdmaSg sg = {.local_offs = o, .remote_offs = o,
                             .len = static_cast<uint32_t>(std::min(SEG, end - o))};
        ct.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);
    }
}

// ---------------------------------------------------------------------------
// Host-side forwarder core, shared between the VM daemon (app = ivshmem) and
// the no-VM trace harness (app = a local stand-in buffer). Encapsulates MMIO
// accesses, shadows the DMA registers to know when payloads must move, and
// performs the host-side staging copies.
// ---------------------------------------------------------------------------
class HostForwarder {
public:
    HostForwarder(coyote::cThread &ct, char *nic, char *app)
        : ct(ct), nic(nic), app(app),
          tx(ct, nic, REQ_RING_OFF, REQ_SLOTS,
             reinterpret_cast<volatile uint64_t *>(nic + CREDIT_OFF + WIRE_BYTES - 8)),
          rx(nic, RESP_RING_OFF, RESP_SLOTS) {}

    void send_setup() {
        setup_msg *s = reinterpret_cast<setup_msg *>(nic + SETUP_OFF);
        s->app_base = reinterpret_cast<uint64_t>(app);
        s->payload_bytes = BUF_BYTES - PAYLOAD_OFF;
        memset(s->pad, 0, sizeof(s->pad));
        s->magic = SETUP_MAGIC;
        coyote::rdmaSg sg = {.local_offs = SETUP_OFF, .remote_offs = SETUP_OFF,
                             .len = WIRE_BYTES};
        ct.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);
    }

    void mmio_write(uint64_t addr, uint64_t value) {
        switch (static_cast<DevReg>(addr)) {
        case DevReg::DMA_SRC_ADDR: sh_src = value; break;
        case DevReg::DMA_DST_ADDR: sh_dst = value; break;
        case DevReg::DMA_H2D_LEN:  sh_h2d = value; break;
        case DevReg::DMA_D2H_LEN:  sh_d2h = value; break;
        case DevReg::DMA_CMD:
            if (value == DMA_CMD_H2D) {
                stage_in(sh_src, sh_h2d);
            } else if (value == DMA_CMD_D2H) {
                arm_d2h(sh_dst, sh_d2h, STATUS_DMA_DONE_MASK);
            }
            break;
        case DevReg::START_COMPUTE:
            if (sh_h2d) stage_in(sh_src, sh_h2d);
            if (sh_d2h) arm_d2h(sh_dst, sh_d2h, STATUS_BUNDLE_DONE_MASK);
            break;
        default:
            break;
        }
        tx.post(WIRE_MMIO_WRITE, addr, 8, value);
    }

    uint64_t mmio_read(uint64_t addr) {
        tx.post(WIRE_MMIO_READ, addr, 8, 0);
        wire_msg m = rx.wait();
        tx.note_credit(m.len);
        // The guest must never observe a completed D2H before its payload is
        // in application memory: the device pushes the payload before this
        // response on the same QP, so it has landed — copy it out now.
        if (static_cast<DevReg>(addr) == DevReg::DMA_STATUS && d2h.armed &&
            (m.value & d2h.mask) == d2h.mask) {
            memcpy(app + d2h.off, nic + d2h.off, d2h.len);
            d2h.armed = false;
        }
        return m.value;
    }

    void send_stop() { tx.post(WIRE_STOP, 0, 0, 0); }

private:
    // H2D: copy application memory into the NIC buffer and push it, before
    // the trigger itself is posted (same QP => ordered).
    void stage_in(uint64_t src_ptr, uint64_t len) {
        uint64_t off = src_ptr - reinterpret_cast<uint64_t>(app);
        if (!payload_range_ok(off, len)) return;
        memcpy(nic + off, app + off, len);
        push_payload(ct, off, len);
    }

    void arm_d2h(uint64_t dst_ptr, uint64_t len, uint64_t mask) {
        uint64_t off = dst_ptr - reinterpret_cast<uint64_t>(app);
        if (!payload_range_ok(off, len)) return;
        d2h = {true, off, len, mask};
    }

    static bool payload_range_ok(uint64_t off, uint64_t len) {
        return off >= PAYLOAD_OFF && off < BUF_BYTES && len <= BUF_BYTES - off;
    }

    coyote::cThread &ct;
    char *nic;
    char *app;
    TxRing tx;
    RxRing rx;

    uint64_t sh_src = 0, sh_dst = 0, sh_h2d = 0, sh_d2h = 0;
    struct { bool armed = false; uint64_t off = 0, len = 0, mask = 0; } d2h;
};

} // namespace jsfwd

#endif // JIGSAW_SW_FORWARDER_WIRE_HPP
