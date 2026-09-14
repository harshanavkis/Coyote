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
// Beats left on the pull stream when a new read was issued. Nothing should
// be waiting there, and a surplus beat is forwarded as the payload of the
// transfer about to start - displacing it for the rest of its length. This
// must read 0; anything else is the corruption signature's root.
constexpr uint32_t PULL_DESYNC   = 0x0C8;   // word 25
// Beats the receive path handed loom_rx that no rq_wr request announced.
// Nonzero says the shell streamed a packet to the user and then did not
// deliver it; zero says that never happened and the leak theory is wrong.
constexpr uint32_t RX_ORPHAN     = 0x0D0;   // word 26
// Word 28 was the rdma chunk size; the engine packetises now and the word is
// unused.
// Cycles loom_rx refused a beat the shell was offering, in ANY state, and the
// longest unbroken run of them. RX_STALL above is ST_STREAM-only, so it is
// blind to the grant wait and the request handshake - exactly where Loom
// differs from perf_rdma, whose receive path is a pass-through that cannot
// backpressure at all. This matters beyond throughput: the shell emits the
// host write and the packet's ACK from the SAME FSM step, so backpressure
// here stops ACKs, and a QP that goes 1 ms without one retransmits
// (transport_timer.hpp). These must read ~0.
// Word 64/65, on an otherwise-empty CSR line: a write to any table register
// clobbers words 4 and 6 of line 0 on hardware (see loom_ctrl.sv R_TX_PACE).
constexpr uint32_t TX_PACE       = 0x200;   // word 64: [7:0] num, [15:8] den; off if 0
constexpr uint32_t TX_PACED      = 0x208;   // word 65: cycles the pacer held
// Transmit window (loom_engine.sv header). Same CSR line.
constexpr uint32_t TX_CTL        = 0x210;   // word 66: [7:0] window (packets unacked), reset 16, 0 = none
constexpr uint32_t TX_STATE      = 0x218;   // word 67 RO: [15:0] packets unacked right now
constexpr uint32_t TX_ACKS       = 0x220;   // word 68 RO: packet acks received
constexpr uint32_t TX_WINFULL    = 0x228;   // word 69 RO: cycles a packet waited on the window
constexpr uint32_t TX_REQWAIT    = 0x230;   // word 70 RO: cycles a packet waited on sq_wr.ready
constexpr uint32_t TX_FIFO_FULL  = 0x238;   // word 71 RO: cycles the pull was held by the tx FIFO
constexpr uint32_t RX_FIFO_FULL  = 0x070;   // word 14: ingress FIFO refused a beat
constexpr uint32_t RX_FIFO_FULL_MAX = 0x078; // word 15: longest run of that
constexpr uint32_t RX_BP         = 0x0F0;   // word 30
constexpr uint32_t RX_BP_MAX     = 0x0F8;   // word 31
// Requests accepted off rq_wr, against DBG rx_fwd (completed). A gap says
// requests arrived and were not finished; equality with a shortfall against
// the packets the sender must have sent says they never arrived at all.
constexpr uint32_t RX_REQ        = 0x178;
// Word 21 was RX_PID; the landing pid travels in the message header now
// (window table word 2 [15:8] on the sender).

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
// pid: local route - the destination cThread (the TLB translates base+off
// under it); rdma route - the QP owner (selects the wire). dst_pid: rdma
// route only - the exporter's ctid on the FAR host, carried in the message
// header so the far loom_rx lands the bytes in that XPU's address space
// (one QP serves every XPU on a host).
inline void program_window(coyote::cThread &t, uint32_t win, bool rdma,
                           uint32_t pid, const void *base, uint64_t len,
                           uint32_t dst_pid = 0) {
    csr_write(t, TBL_IDX,  win);
    csr_write(t, TBL_CFG,  0b01 | (rdma ? 0b10 : 0b00));
    csr_write(t, TBL_PID,  (uint64_t(dst_pid & 0xFF) << 8) | (pid & 0xFF));
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
