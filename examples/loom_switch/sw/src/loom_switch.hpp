#pragma once

#include <cstdint>
#include <coyote/cThread.hpp>

/**
 * The switch's CSR page (hw/src/hdl/loom_ctrl.sv) and window programming.
 * Word numbers follow examples/loom where the register survives.
 */
namespace loom_switch {

// Word indices
constexpr uint32_t TBL_IDX         = 0;
constexpr uint32_t TBL_CFG         = 1;    // bit0 valid, bit1 route (0 local, 1 rdma)
constexpr uint32_t TBL_PID         = 2;    // [5:0] pid (local: destination; rdma: QP owner), [13:8] far pid
constexpr uint32_t TBL_BASE        = 3;
constexpr uint32_t TBL_LEN         = 4;
constexpr uint32_t TBL_COMMIT      = 5;
constexpr uint32_t TBL_USTART      = 80;   // the window's start in the uwin
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
// Ingress counters, 88-94
constexpr uint32_t ING_BURSTS      = 88;
constexpr uint32_t ING_DROPS       = 89;
constexpr uint32_t ING_PKT_LOCAL   = 90;
constexpr uint32_t ING_PKT_RDMA    = 91;
constexpr uint32_t ING_STORES      = 92;
constexpr uint32_t ING_STORE_DROPS = 93;
constexpr uint32_t ING_FLUSHES     = 94;
constexpr int      N_ING           = 7;

inline void csr_write(coyote::cThread &t, uint32_t word, uint64_t val) { t.setCSR(val, word); }
inline uint64_t csr_read(coyote::cThread &t, uint32_t word) { return t.getCSR(word); }

// One window: uwin bytes [ustart, ustart + len) land at base + offset under
// pid (local), or go to the far pid's base + offset over pid's QP (rdma)
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

inline void release_window(coyote::cThread &t, uint32_t win) {
    csr_write(t, TBL_IDX,    win);
    csr_write(t, TBL_CFG,    0);
    csr_write(t, TBL_COMMIT, 1);
}

struct IngressCounters {
    uint64_t v[N_ING];
    static IngressCounters read(coyote::cThread &t) {
        IngressCounters c;
        for (int i = 0; i < N_ING; i++) c.v[i] = csr_read(t, ING_BURSTS + i);
        return c;
    }
};

} // namespace loom_switch
