#pragma once

#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <sys/mman.h>
#include <coyote/cThread.hpp>

/**
 * The switch's CSR page (hw/src/hdl/loom_ctrl.sv) and window programming.
 * Word numbers follow examples/loom where the register survives.
 */
namespace loom_switch {

// Word indices
constexpr uint32_t TBL_IDX         = 0;
constexpr uint32_t TBL_CFG         = 1;    // bit0 valid, bit1 route (0 local, 1 rdma), bit2 get
constexpr uint32_t TBL_PID         = 2;    // [5:0] pid (local: destination; rdma: QP owner), [13:8] far pid
constexpr uint32_t TBL_BASE        = 3;
constexpr uint32_t TBL_LEN         = 4;
constexpr uint32_t TBL_COMMIT      = 5;
constexpr uint32_t TBL_USTART      = 80;   // the window's start in the uwin
// This host's export table: where peers' self-describing packets may land
constexpr uint32_t EXP_IDX         = 176;
constexpr uint32_t EXP_CFG         = 177;  // bit0 valid
constexpr uint32_t EXP_PID         = 178;  // the landing address space
constexpr uint32_t EXP_BASE        = 179;  // the landing VA
constexpr uint32_t EXP_LEN         = 180;
constexpr uint32_t EXP_COMMIT      = 181;
constexpr uint32_t RD_CTL          = 184;  // [5:0] the QP owner the get responses go out on
// Gets (loom_ctrl.sv 136-143)
constexpr uint32_t GET_SENT        = 136;  // get requests sent
constexpr uint32_t GET_FULL_DROP   = 137;  // full lines written to a get window, dropped
constexpr uint32_t GET_JOBS        = 138;  // responder: requests taken
constexpr uint32_t GET_ERRS        = 139;  // responder: error completions (bad requests)
constexpr uint32_t GET_PKTS        = 140;  // responder: response packets
constexpr uint32_t GET_CMPS        = 141;  // responder: completions
constexpr uint32_t GET_WAIT        = 142;  // responder: cycles a request waited on the window or sq_wr
constexpr uint32_t GET_STARVE      = 143;  // responder: cycles a packet waited for its read data
constexpr uint32_t RDMA_STAGING_VA = 16;
constexpr uint32_t RX_FWD          = 36;
constexpr uint32_t RX_DROP         = 41;
constexpr uint32_t CYC             = 48;
constexpr uint32_t TX_CTL          = 66;
constexpr uint32_t TX_STATE        = 67;
constexpr uint32_t TX_ACKS         = 68;
constexpr uint32_t RX_CHUNK        = 76;
// Where the cycles go (loom_ctrl.sv)
constexpr uint32_t TX_WINFULL      = 69;   // an rdma packet waited on the ack window
constexpr uint32_t TX_REQWAIT      = 70;   // an rdma packet waited on sq_wr.ready
constexpr uint32_t WR_WAIT_LOCAL   = 72;   // a local request waited on sq_wr.ready
constexpr uint32_t WR_WAIT_RDMA    = 73;   // an rdma request waited on sq_wr.ready
constexpr uint32_t RX_FIFO_FULL    = 14;   // loom_rx's ingress FIFO refused a beat
constexpr uint32_t RX_BP           = 30;   // loom_rx refused a beat the FIFO offered
constexpr uint32_t RX_MOVE         = 42;   // loom_rx forwarded a beat
constexpr uint32_t RX_STARVE       = 43;   // loom_rx had nothing to forward
constexpr uint32_t RX_STALL        = 44;   // loom_rx had a beat, the host write was not ready
// Ingress counters, 88-94, then its debug counters, 95-108 (loom_ctrl.sv)
constexpr uint32_t ING_BURSTS      = 88;
constexpr uint32_t ING_DROPS       = 89;
constexpr uint32_t ING_PKT_LOCAL   = 90;
constexpr uint32_t ING_PKT_RDMA    = 91;
constexpr uint32_t ING_STORES      = 92;
constexpr uint32_t ING_STORE_DROPS = 93;
constexpr uint32_t ING_FLUSHES     = 94;
constexpr uint32_t ING_DBG         = 95;
constexpr int      N_ING           = 21;
constexpr int      I_DBG           = ING_DBG - ING_BURSTS;   // first debug counter in IngressCounters

inline const char *const ING_NAMES[N_ING] = {
    "bursts", "bursts dropped", "local packets", "rdma packets",
    "stores", "partial words", "idle flushes",
    "host out bp", "net out bp", "send fifo empty",
    "W wait: no aw", "W wait: B slot", "W wait: fifo", "W wait: queue", "W wait: stores",
    "drop: no window", "drop: past end",
    "bursts 1 beat", "bursts 2-4", "bursts >4", "misaligned"};

// An address range to import a peer's dma-buf at: 2 MiB-aligned, so the
// driver can map the peer's contiguous BAR pages as huge TLB entries
// (vfpga_gup.c coalesces 4 KiB pages only at 2 MiB-aligned VAs). At a
// 4 KiB-aligned VA every page takes a small-TLB entry, and a window larger
// than the small TLB is served through page faults.
inline void *reserve_va(uint64_t len) {
    constexpr uint64_t HUGE = 2ULL << 20;
    void *p = mmap(nullptr, len + HUGE, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (p == MAP_FAILED) throw std::runtime_error("mmap for a reserved address failed");
    const uint64_t a = (reinterpret_cast<uint64_t>(p) + HUGE - 1) & ~(HUGE - 1);
    return reinterpret_cast<void *>(a);
}

inline void csr_write(coyote::cThread &t, uint32_t word, uint64_t val) { t.setCSR(val, word); }
inline uint64_t csr_read(coyote::cThread &t, uint32_t word) { return t.getCSR(word); }

// A remote reference: what an rdma window's base holds, and what every
// packet's RETH carries - export idx of the far host, offset into it
inline uint64_t export_ref(uint32_t idx, uint64_t off = 0) { return (uint64_t(idx) << 40) | off; }

// One window: uwin bytes [ustart, ustart + len) land at base + offset under
// pid (local), or, over pid's QP (rdma), at the far host's export
// reference base + offset (base = export_ref(idx, off) from that host)
inline void program_window(coyote::cThread &t, uint32_t win, bool rdma, uint32_t pid,
                           const void *base, uint64_t len, uint64_t ustart,
                           uint32_t dst_pid = 0) {
    csr_write(t, TBL_IDX,    win);
    csr_write(t, TBL_CFG,    0b01 | (rdma ? 0b10 : 0b00));
    csr_write(t, TBL_PID,    (uint64_t(dst_pid & 0x3F) << 8) | (pid & 0x3F));
    csr_write(t, TBL_BASE,   reinterpret_cast<uint64_t>(base));
    csr_write(t, TBL_LEN,    len);
    csr_write(t, TBL_USTART, ustart);
    csr_write(t, TBL_COMMIT, 1);
    // The table writes are posted, and they reach the vFPGA on a different
    // port than data written to the uwin afterwards: read back so the entry
    // is in place before anything is written through it
    (void) csr_read(t, TBL_IDX);
}

// A get window: each 8 B store at window offset off (8 B aligned) asks the
// far host for get_word's bytes from src_ref + off, over pid's QP. src_ref is
// export_ref(idx, off) from that host; the far host answers with
// set_response_qp done.
inline void program_get_window(coyote::cThread &t, uint32_t win, uint32_t pid,
                               uint64_t src_ref, uint64_t len, uint64_t ustart) {
    csr_write(t, TBL_IDX,    win);
    csr_write(t, TBL_CFG,    0b111);
    csr_write(t, TBL_PID,    pid & 0x3F);
    csr_write(t, TBL_BASE,   src_ref);
    csr_write(t, TBL_LEN,    len);
    csr_write(t, TBL_USTART, ustart);
    csr_write(t, TBL_COMMIT, 1);
    (void) csr_read(t, TBL_IDX);
}

// The word stored into a get window: len bytes (a multiple of 64, at most
// 65535 * 64) back to ret_ref, a reference into an export of THIS host that
// holds len + 8 bytes: the data lands at ret_ref, then the word itself at
// ret_ref + len (the completion; all ones if the far host rejected the
// request: no export, out of bounds, misaligned)
inline uint64_t get_word(uint64_t len, uint64_t ret_ref) {
    if (len == 0 || len % 64 || len / 64 > 0xFFFF || ret_ref % 64 || ret_ref >> 48)
        throw std::runtime_error("get_word: len must be a multiple of 64 below 4 MiB, ret_ref 64 B aligned");
    return ((len / 64) << 48) | ret_ref;
}
constexpr uint64_t GET_ERROR = ~0ULL;

// This host answers peers' gets on pid's QP (the QP owner's ctid)
inline void set_response_qp(coyote::cThread &t, uint32_t pid) {
    csr_write(t, RD_CTL, pid & 0x3F);
    (void) csr_read(t, RD_CTL);
}

inline void release_window(coyote::cThread &t, uint32_t win) {
    csr_write(t, TBL_IDX,    win);
    csr_write(t, TBL_CFG,    0);
    csr_write(t, TBL_COMMIT, 1);
}

// Export idx (0..15): peers' packets for export_ref(idx, off) land at
// base + off under pid, if off + their length <= len; anything else is
// dropped (and counted) by loom_rx
inline void program_export(coyote::cThread &t, uint32_t idx, uint32_t pid, const void *base, uint64_t len) {
    csr_write(t, EXP_IDX,    idx);
    csr_write(t, EXP_CFG,    1);
    csr_write(t, EXP_PID,    pid & 0x3F);
    csr_write(t, EXP_BASE,   reinterpret_cast<uint64_t>(base));
    csr_write(t, EXP_LEN,    len);
    csr_write(t, EXP_COMMIT, 1);
    (void) csr_read(t, EXP_IDX);     // in place before a peer can send
}

inline void release_export(coyote::cThread &t, uint32_t idx) {
    csr_write(t, EXP_IDX,    idx);
    csr_write(t, EXP_CFG,    0);
    csr_write(t, EXP_COMMIT, 1);
}

struct IngressCounters {
    uint64_t v[N_ING];
    static IngressCounters read(coyote::cThread &t) {
        IngressCounters c;
        for (int i = 0; i < N_ING; i++) c.v[i] = csr_read(t, ING_BURSTS + i);
        return c;
    }
};

// Counters [first, first + n) that moved from a to b, as "name delta" pairs
inline void print_ingress_delta(const IngressCounters &a, const IngressCounters &b,
                                int first = 0, int n = N_ING) {
    for (int i = first; i < first + n; i++)
        if (b.v[i] != a.v[i])
            printf("    %-17s %lu\n", ING_NAMES[i], (unsigned long) (b.v[i] - a.v[i]));
}

} // namespace loom_switch
