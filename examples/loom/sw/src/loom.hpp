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
constexpr uint32_t DBG_BASE    = 0x100;  // 8 x RO counters (see loom_ctrl.sv)

// Aperture: windows 1..15, 4 KB each (byte 0x1000-0xFFFF)
constexpr uint32_t aperture(uint32_t win, uint32_t off) {
    return (win << 12) | off;
}

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

} // namespace loom
