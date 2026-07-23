/**
 * Loom CSR map + shared helpers. Must match hw/src/hdl/loom_pkg.sv.
 * 8 B stride; the aperture window is fixed at 16..31 (HW traps that range).
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <coyote/cThread.hpp>

namespace loom {

enum Reg : uint32_t {
    CMD_REG = 0, STATUS_REG = 1, DONE_CNT_REG = 2,
    SRC_PID_REG = 3, SRC_ADDR_REG = 4, SRC_STRM_REG = 5,
    DST_PID_REG = 6, DST_ADDR_REG = 7, DST_STRM_REG = 8, LEN_REG = 9,
    AP_PID_REG = 10, AP_BASE_LO_REG = 11, AP_BASE_HI_REG = 12, AP_STRM_REG = 13,
    COYOTE_PID_REG = 14,        // 15 reserved (was REMOTE_VADDR; now per-range)
    AP_LO = 16,                       // aperture window base (16 regs = 128 B peer space)
    RANGE_BASE_REG = 32,              // 8 entries x 3 regs -> 32..55
    WIN_BASE_REG = 56, WIN_LIMIT_REG = 57,
    DROP_CNT_REG = 58, ROLE_REG = 59, CNT_CTRL_REG = 60,
    CNT_BASE_REG = 61,                // 8 counters x {acc, n} -> 61..76
    TICK_REG = 77
};

constexpr uint32_t N_RANGE = 8;
constexpr uint32_t RANGE_W = 3;       // words per entry
constexpr uint32_t N_CNT   = 8;
constexpr uint32_t THRESH  = 8;       // <= 8 B (one AXI-Lite txn) -> aperture
constexpr double   NS_PER_CYCLE = 4.0; // 250 MHz

// Offset-transfer probe: proves the range table translates
// remote = remote_base + (addr - local_base), rather than using a fixed base.
constexpr uint32_t XLAT_OFF  = 2048;
constexpr uint32_t XLAT_LEN  = 64;
constexpr uint8_t  XLAT_MARK = 0xA5;

// RT_PULL is inferred by HW from a remote SOURCE, not programmed via set_range;
// listed for parity with loom_pkg.sv.
enum Route : uint64_t { RT_LOCAL = 0, RT_REMOTE = 1, RT_DROP = 2, RT_PULL = 3 };
enum Ingress : uint64_t { ING_APERTURE = 1, ING_DMA = 2, ING_BOTH = 3 };

// Counter indices, same order as loom_pkg.sv
enum Cnt : uint32_t {
    C_TRANSLATE = 0, C_LOOKUP = 1, C_FORWARD = 2, C_QUEUE = 3,
    C_ENCAP = 4, C_ROCE_TX = 5, C_RX_LAND = 6, C_RX_FORWARD = 7
};

static const char* CNT_NAME[N_CNT] = {
    "t_translate", "t_lookup", "t_forward", "t_queue",
    "t_encap",     "t_roce_tx", "t_rx_land", "t_rx_forward"
};

// Program one range-table entry.
//   base        local virtual address the app will target
//   remote_base peer address the window maps onto (RT_REMOTE only; HW computes
//               remote_base + (addr - base), so the whole window is transparent)
inline void set_range(coyote::cThread& ct, uint32_t idx, uint64_t base, uint64_t len,
                      Route rt, uint64_t dest, uint64_t src_pid, Ingress ing,
                      uint64_t remote_base = 0) {
    ct.setCSR(base, RANGE_BASE_REG + idx * RANGE_W);
    uint64_t w = (len & 0xFFFFFFFULL)
               | ((dest    & 0xFULL)  << 28)
               | ((uint64_t(rt) & 0x3ULL) << 32)
               | ((src_pid & 0x3FULL) << 34)
               | ((uint64_t(ing) & 0x3ULL) << 40)
               | (1ULL << 42);                        // valid
    ct.setCSR(w,           RANGE_BASE_REG + idx * RANGE_W + 1);
    ct.setCSR(remote_base, RANGE_BASE_REG + idx * RANGE_W + 2);
}

inline void clear_ranges(coyote::cThread& ct) {
    for (uint32_t i = 0; i < N_RANGE * RANGE_W; i++)
        ct.setCSR(0, RANGE_BASE_REG + i);
}

inline void clear_counters(coyote::cThread& ct) {
    ct.setCSR(1, CNT_CTRL_REG);
    ct.setCSR(0, CNT_CTRL_REG);
}

// Mean cycles -> ns for each counter, plus the derived pipeline sums.
inline void print_counters(coyote::cThread& ct, const char* title) {
    double ns[N_CNT] = {0};
    printf("\n== %s ==\n", title);
    printf("%-14s %12s %10s %12s\n", "counter", "acc(cycles)", "n", "mean(ns)");
    for (uint32_t i = 0; i < N_CNT; i++) {
        uint64_t acc = ct.getCSR(CNT_BASE_REG + i * 2);
        uint64_t n   = ct.getCSR(CNT_BASE_REG + i * 2 + 1);
        ns[i] = n ? (double(acc) / double(n)) * NS_PER_CYCLE : 0.0;
        printf("%-14s %12lu %10lu %12.1f\n", CNT_NAME[i], acc, n, ns[i]);
    }
    printf("  local adder      (lookup+translate+forward) = %.1f ns\n",
           ns[C_LOOKUP] + ns[C_TRANSLATE] + ns[C_FORWARD]);
    printf("  source remote    (lookup+queue+encap)       = %.1f ns\n",
           ns[C_LOOKUP] + ns[C_QUEUE] + ns[C_ENCAP]);
    printf("  destination      (rx_land+rx_forward)       = %.1f ns\n",
           ns[C_RX_LAND] + ns[C_RX_FORWARD]);
    printf("  roce tx observed (stack+wire+remote land)   = %.1f ns\n", ns[C_ROCE_TX]);
    printf("  drops = %lu\n", ct.getCSR(DROP_CNT_REG));
}

// ---- mechanisms ----
inline void dma(coyote::cThread& ct,
                uint64_t s_pid, void* s_addr, uint64_t s_strm,
                uint64_t d_pid, void* d_addr, uint64_t d_strm, uint32_t len) {
    uint64_t before = ct.getCSR(DONE_CNT_REG);
    ct.setCSR(s_pid, SRC_PID_REG); ct.setCSR((uint64_t)s_addr, SRC_ADDR_REG);
    ct.setCSR(s_strm, SRC_STRM_REG);
    ct.setCSR(d_pid, DST_PID_REG); ct.setCSR((uint64_t)d_addr, DST_ADDR_REG);
    ct.setCSR(d_strm, DST_STRM_REG);
    ct.setCSR(len, LEN_REG);
    ct.setCSR(0x1, CMD_REG);
    while (ct.getCSR(DONE_CNT_REG) == before) {}
}

inline void ap_config(coyote::cThread& ct, uint64_t peer_ctid, void* peer_base, uint64_t peer_strm) {
    ct.setCSR(peer_ctid, AP_PID_REG);
    ct.setCSR((uint64_t)peer_base & 0xFFFFFFFF, AP_BASE_LO_REG);
    ct.setCSR((uint64_t)peer_base >> 32,        AP_BASE_HI_REG);
    ct.setCSR(peer_strm, AP_STRM_REG);
}

// Trapped 8 B accesses: writes posted, reads blocking round trips.
inline void ap_write(coyote::cThread& ct, void* src, uint32_t size) {
    uint64_t* w = (uint64_t*) src;
    for (uint32_t k = 0; k < size / 8; k++) ct.setCSR(w[k], AP_LO + k);
}
inline void ap_read(coyote::cThread& ct, void* dst, uint32_t size) {
    uint64_t* w = (uint64_t*) dst;
    for (uint32_t k = 0; k < size / 8; k++) w[k] = ct.getCSR(AP_LO + k);
}

} // namespace loom
