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
 * (where the peer's writes spend their cycles); the whole window is mapped
 * for that, and Counters prints what one copy cost.
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
    static constexpr int N = 12;
    enum { CYC, AW, W, W_STALL, W_STARVE, AW_STALL, B, B_STALL, OUT_CYC, OUT_SUM, OUT_MAX, PARTIAL };
    uint64_t v[N];
    static Counters read(volatile uint64_t *win) {
        Counters c;
        for (int i = 0; i < N; i++) c.v[i] = win[(UWIN_SIZE - 4096) / 8 + i];
        return c;
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
    bool wait(uint64_t off, uint64_t val, std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
        const auto t0 = std::chrono::steady_clock::now();
        while (win[off / 8] != val) {
            if (std::chrono::steady_clock::now() - t0 > timeout) return false;
            _mm_pause();
        }
        return true;
    }

    // HBM back to host memory
    void pull() { v80.invoke(coyote::CoyoteOper::LOCAL_SYNC, coyote::syncSg{buf, len}); }
};

}  // namespace land_v80
