#pragma once
/**
 * Landing in the V80's HBM (EN_UWIN_HBM): the V80's user data window is its
 * last HBM block, so what the U280 writes into the exported window - loom_rx
 * landings, or a local window - goes peer-to-peer straight into card memory.
 * A buffer's card pages are bound to that region (uwinHbmBind), so buffer
 * offset x is uwin offset x: the copy engine reads it by VA on its card
 * stream, and the host reads it back by syncing the buffer.
 *
 * A copy lands its data, then its fence (an 8 B write behind the data; the
 * V80's NoC keeps same-ID writes in order). wait() polls the fence word
 * through the window itself (reads return HBM), pull() syncs the buffer.
 *
 * The window's last 4 KiB page reads back uwin_hbm's counters instead of HBM
 * (where the peer's writes, and its reads, spend their cycles, and what its
 * read-ahead did, and uwin_ra's on axi_main); the whole window is mapped for
 * that, and Counters prints what one copy cost.
 */
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <immintrin.h>

#include <coyote/cThread.hpp>

namespace land_v80 {

constexpr uint64_t UWIN_SIZE = 1ULL << 27;    // the window (uwin_hbm UWIN_BITS)

// uwin_hbm's counters (hw/hdl/common/uwin/uwin_hbm.sv), read before and after a copy
struct Counters {
    static constexpr int N = 16;
    enum { CYC, AW, W, W_STALL, W_STARVE, AW_STALL, B, B_STALL, OUT_CYC, OUT_SUM, OUT_MAX, PARTIAL,
           DISCARDED, NZ_ID, RA_MODE, MODE };
    // words 16-30: uwin_mon's axi_main counts (xclk), where the writes and reads enter the shell
    static constexpr int N_MAIN = 15;
    enum { M_CYC, M_AW, M_W, M_W_STALL, M_W_STARVE, M_AW_STALL, M_B,
           M_AR, M_R, M_RD_DONE, M_RD_OUT_CYC, M_RD_OUT_SUM, M_RD_OUT_MAX, M_AR_STALL, M_R_STALL };
    // words 32-39: uwin_hbm's own counts of the window's reads (aclk)
    static constexpr int N_R = 8;
    enum { RD_AR, RD_R, RD_DONE, RD_OUT_CYC, RD_OUT_SUM, RD_OUT_MAX, RD_AR_STALL, RD_R_STALL };
    // words 40-49: uwin_hbm's read-ahead (on/off: counter page line 61/60; RA_MODE says which)
    static constexpr int N_RA = 10;
    enum { RA_HIT, RA_LATE, RA_PASS, RA_AR, RA_FILLED, RA_SERVED, RA_STREAMS, RA_END_JUMP, RA_END_WRITE, RA_END_IDLE };
    // words 64-76: uwin_ra's read-ahead on axi_main, before the clock crossing (xclk; on/off: counter page line
    // 59/58, X_MODE says which; a snapshot like words 16-30). X_FILL_OUT_SUM / X_AR = cycles a read-ahead AR is out
    static constexpr int N_X = 13;
    enum { X_MODE, X_HIT, X_LATE, X_PASS, X_AR, X_FILLED, X_SERVED, X_STREAMS, X_END_JUMP, X_END_WRITE, X_END_IDLE,
           X_FILL_OUT_SUM, X_NZ_ID };
    // uwin_hbm counts in the shell's aclk (ACLK_F, 250 MHz), uwin_mon and uwin_ra in the static's xclk (SCLK_F, 333 MHz on the V80)
    static constexpr double ACLK_NS = 1e3 / 250, XCLK_NS = 1e3 / 333;
    uint64_t v[N], m[N_MAIN], r[N_R], a[N_RA], x[N_X];
    static Counters read(volatile uint64_t *win) {
        Counters c;
        for (int i = 0; i < N; i++) c.v[i] = win[(UWIN_SIZE - 4096) / 8 + i];
        for (int i = 0; i < N_MAIN; i++) c.m[i] = win[(UWIN_SIZE - 4096) / 8 + 16 + i];
        for (int i = 0; i < N_R; i++) c.r[i] = win[(UWIN_SIZE - 4096) / 8 + 32 + i];
        for (int i = 0; i < N_RA; i++) c.a[i] = win[(UWIN_SIZE - 4096) / 8 + 40 + i];
        for (int i = 0; i < N_X; i++) c.x[i] = win[(UWIN_SIZE - 4096) / 8 + 64 + i];
        return c;
    }
    // One side's reads: AR, beats per read, completed, mean outstanding while any is, mean AR-to-last-R
    // (cycles, ns), max outstanding since reset, AR stalled, R stalled
    static void print_reads(const char *side, unsigned long cyc, unsigned long ar, unsigned long beats,
                            unsigned long done, unsigned long out_cyc, unsigned long out_sum, uint64_t out_max,
                            unsigned long ar_stall, unsigned long r_stall, double clk_ns) {
        const double life = done ? double(out_sum) / done : 0.0;
        printf("     %s reads over %lu cycles: %lu AR (%.1f beats), %lu completed; outstanding %lu cycles, "
               "mean %.1f, AR->last R %.0f cycles (%.0f ns), max since reset %lu; AR stalled %lu, R stalled %lu\n",
               side, cyc, ar, ar ? double(beats) / ar : 0.0, done, out_cyc, out_cyc ? double(out_sum) / out_cyc : 0.0,
               life, life * clk_ns, (unsigned long) out_max, ar_stall, r_stall);
    }
    void print(const Counters &b) const {
        auto d = [&](int i) { return (unsigned long) (v[i] - b.v[i]); };
        const double aw = d(AW) ? double(d(AW)) : 1.0;
        printf("     V80 window over %lu cycles: %lu W beats, %lu stalled (HBM side), %lu starved (PCIe side), "
               "AW stalled %lu; %lu bursts (%.1f beats), %lu partial beats; B %lu, stalled %lu; "
               "writes outstanding %lu cycles, mean %.1f, AW->B %.0f cycles, max since reset %lu\n",
               d(CYC), d(W), d(W_STALL), d(W_STARVE), d(AW_STALL), d(AW), d(W) / aw, d(PARTIAL), d(B), d(B_STALL),
               d(OUT_CYC), d(OUT_CYC) ? double(d(OUT_SUM)) / d(OUT_CYC) : 0.0, d(OUT_SUM) / aw,
               (unsigned long) v[OUT_MAX]);
        if (d(DISCARDED) || d(NZ_ID) || v[MODE])
            printf("     V80 window: %lu bursts discarded, %lu with a nonzero AWID; mode now %s\n",
                   d(DISCARDED), d(NZ_ID), v[MODE] ? "DISCARD" : "HBM");
        auto e = [&](int i) { return (unsigned long) (m[i] - b.m[i]); };
        printf("     axi_main (static -> shell, xclk) over %lu cycles: %lu W beats, %lu stalled (shell not ready), "
               "%lu starved (static side), %lu AW, AW stalled %lu, B %lu (snapshots every 256 cycles)\n",
               e(M_CYC), e(M_W), e(M_W_STALL), e(M_W_STARVE), e(M_AW), e(M_AW_STALL), e(M_B));
        // reads, counter-page reads among them (read() makes 49): AR stalled = memory side slow (or 64 in
        // flight) at the window, shell not ready at axi_main; R stalled = PCIe side slow at the window,
        // static side at axi_main
        auto f = [&](int i) { return (unsigned long) (r[i] - b.r[i]); };
        print_reads("V80 window (aclk)", d(CYC), f(RD_AR), f(RD_R), f(RD_DONE), f(RD_OUT_CYC), f(RD_OUT_SUM),
                    r[RD_OUT_MAX], f(RD_AR_STALL), f(RD_R_STALL), ACLK_NS);
        print_reads("axi_main (xclk)", e(M_CYC), e(M_AR), e(M_R), e(M_RD_DONE), e(M_RD_OUT_CYC), e(M_RD_OUT_SUM),
                    m[M_RD_OUT_MAX], e(M_AR_STALL), e(M_R_STALL), XCLK_NS);
        // read-ahead: stream reads (hits: every line already requested), reads passed through while on,
        // read-ahead ARs and lines (lines - served = fetched, unused), streams and what ended them
        auto g = [&](int i) { return (unsigned long) (a[i] - b.a[i]); };
        if (v[RA_MODE] || g(RA_STREAMS) || g(RA_PASS))
            printf("     V80 read-ahead (now %s): %lu stream reads, %lu hits, %lu late; %lu passed through; %lu ARs, "
                   "%lu lines read ahead, %lu served; %lu streams, ended by a jump %lu, a write %lu, idle %lu\n",
                   v[RA_MODE] ? "on" : "off", g(RA_HIT) + g(RA_LATE), g(RA_HIT), g(RA_LATE), g(RA_PASS), g(RA_AR),
                   g(RA_FILLED), g(RA_SERVED), g(RA_STREAMS), g(RA_END_JUMP), g(RA_END_WRITE), g(RA_END_IDLE));
        // uwin_ra (axi_main): the same, and how long its read-ahead ARs are out (the interconnect, uwin_hbm and HBM)
        auto h = [&](int i) { return (unsigned long) (x[i] - b.x[i]); };
        if (x[X_MODE] || h(X_STREAMS) || h(X_PASS) || h(X_NZ_ID))
            printf("     axi_main read-ahead (now %s): %lu stream reads, %lu hits, %lu late; %lu passed through; %lu ARs "
                   "(out %.0f ns each), %lu lines read ahead, %lu served; %lu streams, ended by a jump %lu, a write %lu, "
                   "idle %lu; %lu ARs with a nonzero ID\n",
                   x[X_MODE] ? "on" : "off", h(X_HIT) + h(X_LATE), h(X_HIT), h(X_LATE), h(X_PASS), h(X_AR),
                   h(X_AR) ? double(h(X_FILL_OUT_SUM)) / h(X_AR) * XCLK_NS : 0.0, h(X_FILLED), h(X_SERVED), h(X_STREAMS),
                   h(X_END_JUMP), h(X_END_WRITE), h(X_END_IDLE), h(X_NZ_ID));
    }
};

struct Landing {
    coyote::cThread &v80;
    uint64_t        *buf;
    uint64_t         len;
    volatile uint64_t *win;          // the whole window: the fence, the counter page

    // A buffer of len bytes (a multiple of 4 KiB, at most the window) bound
    // to the window's HBM region, zeroed
    Landing(coyote::cThread &t, uint64_t n) : v80(t), len(n) {
        buf = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, len}));
        v80.uwinHbmBind(buf, len);
        win = static_cast<volatile uint64_t *>(v80.mapUwin(UWIN_SIZE));
        clear();
    }
    ~Landing() { v80.unmapUwin(); }

    // The window, as a dma-buf for the U280 to import
    int export_fd() { return v80.exportDmabuf(EXPORT_REGION_UWIN, 0, len); }

    // Zero the region in HBM, so the next copy must write every byte again
    void clear() {
        memset(buf, 0, len);
        v80.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{buf, len});
    }

    // Wait for the 8 B word at byte offset off to read val; false on timeout
    // poll_us > 0: read the window only every poll_us microseconds
    bool wait(uint64_t off, uint64_t val, std::chrono::milliseconds timeout = std::chrono::milliseconds(5000),
              unsigned poll_us = 0) {
        const auto t0 = std::chrono::steady_clock::now();
        while (win[off / 8] != val) {
            if (std::chrono::steady_clock::now() - t0 > timeout) return false;
            if (poll_us) {
                const auto t1 = std::chrono::steady_clock::now() + std::chrono::microseconds(poll_us);
                while (std::chrono::steady_clock::now() < t1) _mm_pause();
            } else {
                _mm_pause();
            }
        }
        return true;
    }

    // uwin_hbm's DISCARD mode (diagnostic): writes into the window answered
    // without reaching HBM. A counter-page read of line 63 asks for it, line
    // 62 for HBM; it applies once no write is in flight (word 15 says)
    bool set_discard(bool on) {
        (void) win[(UWIN_SIZE - (on ? 64 : 128)) / 8];
        const auto t0 = std::chrono::steady_clock::now();
        while (win[(UWIN_SIZE - 4096) / 8 + Counters::MODE] != uint64_t(on))
            if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(1)) return false;
        return true;
    }

    // HBM back to host memory
    void pull() { v80.invoke(coyote::CoyoteOper::LOCAL_SYNC, coyote::syncSg{buf, len}); }
};

}  // namespace land_v80
