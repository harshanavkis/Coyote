#pragma once

#include <cstdint>
#include <coyote/cThread.hpp>

/**
 * Loom CSR map (byte offsets in the vFPGA's 64 KB user ctrl region) and
 * access helpers. Keep in sync with hw/src/hdl/loom_ctrl.sv.
 *
 * Programming model in one paragraph: every participating process holds
 * a cThread (its identity toward the shell TLB) and, through it, the
 * mmap'd ctrl region. Byte 0x0000-0x0FFF of that region is the CSR page
 * (control plane: window-table programming, DMA descriptor staging,
 * debug counters); bytes 0x1000-0xFFFF are 15 aperture windows of 4 KB
 * (data plane: an 8-byte store at window w, offset o becomes a peer
 * write of 8 bytes at offset o of whatever segment the control plane
 * bound to w). Multi-word structures (table entries, descriptors) are
 * staged register-by-register and committed atomically by the final
 * COMMIT/TRIGGER write. Bulk transfers are descriptors: the engine pulls
 * the source buffer (named by the issuer's own VA + pid) and writes the
 * destination; completion is a fence - the engine writes an incrementing
 * count to the descriptor's compl VA, and software polls that word in
 * ordinary memory (never a CSR: memory polls are cache-cheap on
 * hardware, and in simulation CSR reads block behind the generator).
 */
namespace loom {

// CSR page (byte 0x0000-0x0FFF)
constexpr uint32_t TBL_IDX     = 0x00;
constexpr uint32_t TBL_CFG     = 0x08;   // bit0 valid, bit1 route (0 local, 1 rdma)
constexpr uint32_t TBL_PID     = 0x10;
constexpr uint32_t TBL_BASE    = 0x18;
constexpr uint32_t TBL_LEN     = 0x20;
constexpr uint32_t TBL_COMMIT  = 0x28;
constexpr uint32_t DMA_DST     = 0x40;   // [63:60] window, [27:0] segment offset
constexpr uint32_t DMA_SRC_VA  = 0x48;
constexpr uint32_t DMA_LEN     = 0x50;
constexpr uint32_t DMA_SRC_PID = 0x58;
constexpr uint32_t DMA_TRIGGER = 0x60;
constexpr uint32_t DMA_COMPL_VA = 0x68;
// Word 16, on its own 64 B line. NOT word 14 (0x70): a host ctrl write
// covers its whole line forward from the target, so there it sat inside the
// burst of every descriptor staging write at 0x40 - see loom_ctrl.sv.
constexpr uint32_t RDMA_STAGING_VA = 0x80;  // RETH vaddr for outgoing wire messages
constexpr uint32_t DBG_BASE    = 0x100;  // 10 x RO counters (see loom_ctrl.sv)

// Stage cycle counters (RO, words 48-63; see loom_ctrl.sv header). Feed
// the T3 per-stage latency measurements: average cycles = acc / cnt,
// scaled by the vFPGA clock period.
// Receive-path cycle accounting (RO words 42-44, loom_rx). The three
// partition every cycle loom_rx spends forwarding a payload: moving (both
// sides ready), starved (RoCE ingress had no beat), stalled (host write path
// not ready). loom_rx holds no buffer, so a cycle lost to either side is a
// cycle the ingress is not drained, and above ~9 GB/s that becomes a dropped
// packet and then a PSN storm. The FSM itself costs ~6 cycles per packet
// against 64 beats of data, so a per-packet cost far above the 64-cycle
// floor is stall, not overhead - these say which side to fix.
// Transmit-side cycle accounting (RO words 17-19, loom_engine). Same three
// buckets as the receive side, for the rdma route only. starved means the
// host pull left a gap in the outgoing packet stream - a gap Loom created;
// stalled means the shell pushed back, which is the fabric behaving. The
// engine's cyc/op cannot tell those apart, and they have opposite fixes.
constexpr uint32_t TX_MOVE       = 0x88;
constexpr uint32_t TX_STARVE     = 0x90;
constexpr uint32_t TX_STALL      = 0x98;

constexpr uint32_t RX_MOVE       = 0x150;
constexpr uint32_t RX_STARVE     = 0x158;
constexpr uint32_t RX_STALL      = 0x160;
// Longest unbroken run of stalled cycles (RX_STALL is their sum). Only this
// one sizes a buffer: a FIFO absorbs a burst up to its depth, so the worst
// single run is the number that matters, not the total.
constexpr uint32_t RX_STALL_MAX  = 0x0C0;   // word 24
// The stall split. loom_rx is single-outstanding on the write side, so if
// the shell withholds m_tready until it has accepted and translated the
// request, every packet pays that latency serially and the stalls land
// BEFORE its first beat. Head-heavy: overlap the next request with the
// current stream. Body-heavy: the host write path is bursty, buffer it.
// Neither: its sustained bandwidth is the ceiling and neither fix helps.
constexpr uint32_t RX_STALL_HEAD = 0x168;
constexpr uint32_t RX_STALL_BODY = 0x170;
// Requests accepted off rq_wr, against DBG rx_fwd (completed). A gap says
// requests arrived and were not finished; equality with a shortfall against
// the packets the sender must have sent says they never arrived at all.
constexpr uint32_t RX_REQ        = 0x178;
// Continuation requests absorbed by a spanning message (RO word 17). A bulk
// transfer is ONE write across many packets, so most of its rq_wr's are
// swallowed rather than becoming transactions. Expect, per bulk message,
// ceil((len + 64) / PMTU) - 1. Anything else means the receive side is
// pairing requests with payload differently than the sender framed it.
// Whose address space incoming rdma writes land in (RW word 21). The QP
// owner's cThread is fixed for the connection, so the exporter writes it
// once at QP setup and the receive path never reads a pid off a request -
// jigsaw's controller uses a configured pid the same way.
constexpr uint32_t RX_PID        = 0xA8;
constexpr uint32_t RX_SPAN       = 0xA0;

constexpr uint32_t STG_CYC       = 0x180;  // free-running cycle counter
constexpr uint32_t STG_QUEUE_ACC = 0x188;  // order-FIFO residency sum (t-queue)
constexpr uint32_t STG_ACC_BASE  = 0x190;  // 7 words, cycles per stage
constexpr uint32_t STG_CNT_BASE  = 0x1C8;  // 7 words, completed-op counts
constexpr int      STG_N         = 7;

// Stage indices for STG_ACC_BASE/STG_CNT_BASE
enum Stage : int {
    STG_LOOKUP      = 0,  // pop + check, 2 cycles/entry (t-lookup; cnt = pops)
    STG_STORE_LOCAL = 1,  // t-forward
    STG_STORE_RDMA  = 2,  // t-encap
    STG_DMA_LOCAL   = 3,
    STG_DMA_RDMA    = 4,
    STG_READ        = 5,  // aligned-line pull service (shell round trip)
    STG_FENCE       = 6,
};

// Aperture: windows 1..15, 4 KB each (byte 0x1000-0xFFFF)
constexpr uint32_t APERTURE_WIN_SIZE = 4096;

// The offset field is 12 bits. An offset that does not fit is not clamped
// or refused by the hardware - it ORs into the window index and the store
// silently lands in a DIFFERENT window, where it is dropped if that window
// is unprogrammed and, worse, honoured if it is not.
constexpr uint32_t aperture(uint32_t win, uint32_t off) {
    return (win << 12) | (off & (APERTURE_WIN_SIZE - 1));
}

inline bool aperture_off_ok(uint32_t off) { return off < APERTURE_WIN_SIZE; }

/**
 * setCSR/getCSR take a 64-bit word index in BOTH backends: the real
 * cThread does `ctrl_reg[offs] = val` and the simulation generator
 * multiplies offs by 8 onto the AXI-Lite address (verified in
 * simulate.log: byte 0x1040 requested as index arrives at 0x8200 when
 * passed verbatim). Aperture offsets must therefore be 8 B aligned.
 */
inline void csr_write(coyote::cThread &t, uint32_t byte_off, uint64_t val) {
    t.setCSR(val, byte_off / 8);
}

inline uint64_t csr_read(coyote::cThread &t, uint32_t byte_off) {
    return t.getCSR(byte_off / 8);
}

// Program one window-table entry
inline void program_window(coyote::cThread &t, uint32_t win, bool rdma,
                           uint32_t pid, const void *base, uint64_t len) {
    csr_write(t, TBL_IDX,  win);
    csr_write(t, TBL_CFG,  0b01 | (rdma ? 0b10 : 0b00));
    csr_write(t, TBL_PID,  pid);
    csr_write(t, TBL_BASE, reinterpret_cast<uint64_t>(base));
    csr_write(t, TBL_LEN,  len);
    csr_write(t, TBL_COMMIT, 1);
}

// Small write through the aperture (the emulated peer store)
inline void aperture_store(coyote::cThread &t, uint32_t win, uint32_t off,
                           uint64_t val) {
    csr_write(t, aperture(win, off), val);
}

// Aperture load: an 8 B peer read through the window. Non-posted: blocks
// until the switch returns the data (the engine pulls the destination's
// 64 B line and lane-selects). Invalid windows return POISON (all-ones)
// rather than hanging. NOTE: unusable in the C++ interactive simulation
// (the blocking ctrl read parks the sim generator, which then cannot
// service the engine's pull - the documented interactive-mode deadlock);
// covered by block TBs + the Python framework + hardware.
constexpr uint64_t READ_POISON = ~0ULL;
inline uint64_t aperture_read(coyote::cThread &t, uint32_t win, uint32_t off) {
    return csr_read(t, aperture(win, off));
}

// Program the RDMA staging vaddr (per-host; exchanged at QP setup). All
// outgoing rdma wire messages carry this as the RETH vaddr; the true
// target rides the message header (op-len-vaddr).
inline void set_rdma_staging(coyote::cThread &t, const void *va) {
    csr_write(t, RDMA_STAGING_VA, reinterpret_cast<uint64_t>(va));
}

inline void set_rx_pid(coyote::cThread &t, uint32_t ctid) {
    csr_write(t, RX_PID, ctid);
}

// Bulk transfer: configure the DMA engine, then it moves the data.
// compl_va is the per-descriptor completion (fence) address: when the
// descriptor retires, the engine writes an incrementing count there under
// src_pid (the copy-engine semaphore-release pattern). nullptr = none.
inline void dma(coyote::cThread &t, uint32_t win, uint32_t seg_off,
                const void *src, uint64_t len, uint32_t src_pid,
                const void *compl_va = nullptr) {
    csr_write(t, DMA_DST,      (uint64_t(win) << 60) | seg_off);
    csr_write(t, DMA_SRC_VA,   reinterpret_cast<uint64_t>(src));
    csr_write(t, DMA_LEN,      len);
    csr_write(t, DMA_SRC_PID,  src_pid);
    csr_write(t, DMA_COMPL_VA, reinterpret_cast<uint64_t>(compl_va));
    csr_write(t, DMA_TRIGGER,  1);
}

// Snapshot of the stage cycle counters. Counters are never cleared;
// measure an interval by taking two snapshots and differencing.
struct StageStats {
    uint64_t cyc;               // free-running cycle counter
    uint64_t queue_acc;         // FIFO-residency cycle sum (divisor: cnt[STG_LOOKUP])
    uint64_t acc[STG_N];        // per-stage cycle sums
    uint64_t cnt[STG_N];        // per-stage completed-op counts
};

inline StageStats read_stage_stats(coyote::cThread &t) {
    StageStats s;
    s.cyc       = csr_read(t, STG_CYC);
    s.queue_acc = csr_read(t, STG_QUEUE_ACC);
    for (int i = 0; i < STG_N; i++) s.acc[i] = csr_read(t, STG_ACC_BASE + 8 * i);
    for (int i = 0; i < STG_N; i++) s.cnt[i] = csr_read(t, STG_CNT_BASE + 8 * i);
    return s;
}

// Average cycles per op for one stage over the interval [a, b]
inline double stage_avg(const StageStats &a, const StageStats &b, Stage st) {
    uint64_t ops = b.cnt[st] - a.cnt[st];
    return ops ? double(b.acc[st] - a.acc[st]) / double(ops) : 0.0;
}

} // namespace loom
