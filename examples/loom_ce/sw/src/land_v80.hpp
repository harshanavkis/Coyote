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
 */
#include <chrono>
#include <cstdint>
#include <cstring>
#include <immintrin.h>

#include <coyote/cThread.hpp>

namespace land_v80 {

struct Landing {
    coyote::cThread &v80;
    uint64_t        *buf;
    uint64_t         len;
    volatile uint64_t *win;          // the window, mapped for reading the fence

    // A buffer of len bytes (a multiple of 4 KiB, at most the window) bound
    // to the window's HBM region, zeroed
    Landing(coyote::cThread &t, uint64_t n) : v80(t), len(n) {
        buf = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, len}));
        v80.uwinHbmBind(buf, len);
        win = static_cast<volatile uint64_t *>(v80.mapUwin(len));
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
