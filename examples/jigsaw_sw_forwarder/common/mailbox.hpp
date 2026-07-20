#ifndef JIGSAW_SW_FORWARDER_MAILBOX_HPP
#define JIGSAW_SW_FORWARDER_MAILBOX_HPP

/**
 * Jigsaw Software Forwarder — mailbox protocol
 *
 * Modeled directly on the proven jigsaw_baseline_rdma coyote_api pattern:
 * one 64 B mailbox per direction, strict ping-pong request/response (at
 * most ONE control message in flight, ever), bulk payloads written into a
 * payload region that mirrors the ivshmem layout, and a periodic full
 * quiesce (clearCompleted + connSync on both sides) — the queue-flush
 * discipline the working May software performs around every iteration.
 *
 * The host encapsulates the guest's MMIO accesses as mailbox requests; the
 * device replays them on the jigsaw_baseline vFPGA and responds. Requests
 * carry a monotonic req_id which doubles as the mailbox publish flag
 * (written last; RDMA WRITE places bytes in increasing address order).
 *
 * Payload ordering needs no extra machinery: transfers are chunked at
 * 1 MiB by the guest driver / harness, a payload write posted before the
 * trigger request travels the same QP and is placed first, and the device
 * responds to a trigger only after the operation fully completed (any D2H
 * payload pushed), so a response can never overtake data.
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <immintrin.h>

#include <coyote/cThread.hpp>

namespace jsfwd {

// ---------------------------------------------------------------------------
// QP buffer layout (identical on both nodes). The payload region mirrors the
// ivshmem layout one-to-one: ivshmem offset X == QP buffer offset X on both
// nodes, so guest DMA pointers translate with a single base subtraction.
// ---------------------------------------------------------------------------
constexpr uint32_t MBOX_BYTES   = 64;
constexpr uint32_t MBOX_REQ_OFF = 0;    // host -> device requests
constexpr uint32_t MBOX_RESP_OFF= 64;   // device -> host responses
constexpr uint32_t CTRL_BYTES   = 0x1000;
constexpr uint32_t PAYLOAD_OFF  = 0x1000;  // == ivshmem DMA_REGION_OFFSET
constexpr uint32_t BUF_BYTES    = 16U * 1024 * 1024;  // == ivshmem SHMEM_SIZE

enum MboxType : uint64_t {
    MBOX_SETUP      = 1,  // addr = app_base (host vaddr of the ivshmem/app buffer)
    MBOX_MMIO_WRITE = 2,
    MBOX_MMIO_READ  = 3,
    MBOX_SYNC       = 4,  // both sides quiesce: clearCompleted + connSync
    MBOX_STOP       = 5,
};

// req_id is the last word on purpose: it publishes the rest of the message.
// orig names the first attempt of a request: retries use a fresh req_id (so
// the publish flag advances and the receiver wakes) but keep orig, letting
// the device execute each logical request exactly once without caching.
struct mbox_msg {
    uint64_t type;
    uint64_t addr;
    uint64_t value;
    uint64_t orig;
    uint64_t pad[3];
    uint64_t req_id;
};
static_assert(sizeof(mbox_msg) == MBOX_BYTES, "mbox_msg must be 64 B");

// ---------------------------------------------------------------------------
// Device register map of the jigsaw_baseline vFPGA (byte offsets in the
// guest-visible BAR; CSR index = offset >> 3, as in jigsaw_host_controller).
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
// Mailbox primitives
// ---------------------------------------------------------------------------
inline uint64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Compose a message in the local mailbox and write it into the peer's.
inline void mbox_send(coyote::cThread &ct, char *buf, uint32_t off,
                      uint64_t type, uint64_t addr, uint64_t value,
                      uint64_t orig, uint64_t req_id) {
    mbox_msg *m = reinterpret_cast<mbox_msg *>(buf + off);
    m->type = type;
    m->addr = addr;
    m->value = value;
    m->orig = orig;
    m->pad[0] = m->pad[1] = m->pad[2] = 0;
    m->req_id = req_id;
    coyote::rdmaSg sg = {.local_offs = off, .remote_offs = off, .len = MBOX_BYTES};
    ct.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);
}

// Wait until the local mailbox at `off` publishes a req_id above
// `last_seen`; returns a copy. deadline_ms == 0 means wait forever.
inline bool mbox_wait(char *buf, uint32_t off, uint64_t last_seen,
                      mbox_msg &out, uint64_t deadline_ms = 0) {
    volatile mbox_msg *m = reinterpret_cast<volatile mbox_msg *>(buf + off);
    uint32_t spins = 0;
    while (m->req_id <= last_seen) {
        _mm_pause();
        if (deadline_ms && ++spins == 4096) {
            spins = 0;
            if (now_ms() > deadline_ms)
                return false;
        }
    }
    std::atomic_thread_fence(std::memory_order_acquire);
    out.type = m->type;
    out.addr = m->addr;
    out.value = m->value;
    out.orig = m->orig;
    out.req_id = m->req_id;
    return true;
}

// Push [off, off+len) of the local payload region to the same offsets on
// the peer. Transfers are <= 1 MiB by the driver/harness chunking
// convention, so this is a single WQE — the proven May-pattern profile of
// one payload plus one mailbox write in flight.
inline void push_payload(coyote::cThread &ct, uint64_t off, uint64_t len) {
    uint64_t aligned = (len + MBOX_BYTES - 1) & ~uint64_t(MBOX_BYTES - 1);
    if (off + aligned > BUF_BYTES)
        aligned = BUF_BYTES - off;
    coyote::rdmaSg sg = {.local_offs = off, .remote_offs = off,
                         .len = static_cast<uint32_t>(aligned)};
    ct.invoke(coyote::CoyoteOper::REMOTE_RDMA_WRITE, sg);
}

// ---------------------------------------------------------------------------
// Host-side forwarder: shared between the VM daemon (app = ivshmem) and the
// no-VM trace harness (app = a local stand-in buffer).
// ---------------------------------------------------------------------------
class HostForwarder {
public:
    // Generous: a trigger's response arrives only after the device finished
    // the operation (largest compute bundle ~214 ms).
    static const uint64_t RESP_TIMEOUT_MS = 5000;

    HostForwarder(coyote::cThread &ct, char *nic, char *app)
        : ct(ct), nic(nic), app(app) {}

    void send_setup() {
        (void)request(MBOX_SETUP, reinterpret_cast<uint64_t>(app),
                      BUF_BYTES - PAYLOAD_OFF);
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
        (void)request(MBOX_MMIO_WRITE, addr, value);
    }

    uint64_t mmio_read(uint64_t addr) {
        uint64_t value = request(MBOX_MMIO_READ, addr, 0);
        // The device pushes a D2H payload before responding to the poll
        // that reports completion (same QP, placed in order), so the data
        // is in the NIC buffer by the time "done" is visible — stage it out.
        if (static_cast<DevReg>(addr) == DevReg::DMA_STATUS && d2h.armed &&
            (value & d2h.mask) == d2h.mask) {
            memcpy(app + d2h.off, nic + d2h.off, d2h.len);
            d2h.armed = false;
        }
        return value;
    }

    // Full quiesce, exactly as the May software does around every
    // iteration: the device acks the SYNC, then both sides clear their
    // completion state and meet in a TCP barrier with an idle wire.
    void sync() {
        (void)request(MBOX_SYNC, 0, 0);
        ct.clearCompleted();
        ct.connSync(true);
    }

    void send_stop() {
        (void)request(MBOX_STOP, 0, 0);
        ct.connSync(true);
    }

private:
    // A request or its response can vanish: the stack's go-back-N
    // retransmission does not reliably replay 64 B writes (observed with a
    // trigger posted behind a payload as well as in 64 B-only phases). On
    // timeout, retry with a FRESH req_id (so the mailbox publish flag
    // advances and the device wakes even if the original was delivered)
    // carrying the same `orig`: the device executes each orig exactly once
    // — a retry can never re-fire a trigger — and simply responds again
    // (reads re-execute; they are idempotent, nothing is cached).
    static const uint32_t MAX_ATTEMPTS = 10;

    uint64_t request(uint64_t type, uint64_t addr, uint64_t value) {
        uint64_t orig = ++next_req_id;
        uint64_t rid = orig;
        mbox_msg resp;
        for (uint32_t attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
            mbox_send(ct, nic, MBOX_REQ_OFF, type, addr, value, orig, rid);
            uint64_t deadline = now_ms() + RESP_TIMEOUT_MS;
            while (mbox_wait(nic, MBOX_RESP_OFF, last_resp_rid, resp, deadline)) {
                last_resp_rid = resp.req_id;
                if (resp.req_id == rid)
                    return resp.value;
                // stale response to an earlier attempt — skip it
            }
            std::cerr << "[mbox] no response for req_id=" << rid
                      << " (orig=" << orig << ") type=" << type
                      << " addr=0x" << std::hex << addr << std::dec
                      << " — retrying (attempt " << attempt << ")" << std::endl;
            rid = ++next_req_id;
        }
        std::cerr << "[mbox] FATAL: request orig=" << orig
                  << " unrecoverable after " << MAX_ATTEMPTS
                  << " attempts" << std::endl;
        abort();
    }

    // H2D: bounce application memory into the NIC buffer and push it before
    // the trigger request is sent (same QP => payload placed first).
    void stage_in(uint64_t src_ptr, uint64_t len) {
        uint64_t off = src_ptr - reinterpret_cast<uint64_t>(app);
        if (!payload_range_ok(off, len)) return;
        memcpy(nic + off, app + off, len);
        push_payload(ct, off, len);
    }

    void arm_d2h(uint64_t dst_ptr, uint64_t len, uint64_t mask) {
        uint64_t off = dst_ptr - reinterpret_cast<uint64_t>(app);
        if (!payload_range_ok(off, len)) return;
        d2h.armed = true;
        d2h.off = off;
        d2h.len = len;
        d2h.mask = mask;
    }

    static bool payload_range_ok(uint64_t off, uint64_t len) {
        return off >= PAYLOAD_OFF && off < BUF_BYTES && len <= BUF_BYTES - off;
    }

    coyote::cThread &ct;
    char *nic;
    char *app;

    uint64_t next_req_id = 0;
    uint64_t last_resp_rid = 0;
    uint64_t sh_src = 0, sh_dst = 0, sh_h2d = 0, sh_d2h = 0;
    struct { bool armed = false; uint64_t off = 0, len = 0, mask = 0; } d2h;
};

} // namespace jsfwd

#endif // JIGSAW_SW_FORWARDER_MAILBOX_HPP
