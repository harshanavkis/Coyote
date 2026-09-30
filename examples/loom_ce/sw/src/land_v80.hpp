#pragma once
/**
 * The V80's landing window (loom_ce_ctrl.sv words 16-29): writes into the
 * V80's uwin land in its card memory (HBM) at a buffer's VA. The U280 imports
 * the exported uwin, so what its switch writes to that VA - loom_rx landings,
 * or a local window - goes peer-to-peer into the V80 and on into HBM.
 *
 * A copy lands its data, then its fence: an 8 B store behind the data, the
 * copy's only store. wait() waits for that store to be posted and for every
 * posted card write to be complete, then pull() brings the buffer back to
 * host memory to be checked.
 */
#include <chrono>
#include <cstdint>
#include <cstring>

#include <coyote/cThread.hpp>

namespace land_v80 {

enum LandReg : uint32_t { LAND_BASE = 16, LAND_LEN = 17, LAND_PID = 18,
                          LAND_REQS = 24, LAND_DONE = 25, LAND_BURSTS = 26, LAND_DROPS = 27,
                          LAND_STORES = 28, LAND_PARTIAL = 29 };

struct Landing {
    coyote::cThread &v80;
    uint64_t        *buf;
    uint64_t         len;

    // A buffer of len bytes (a multiple of 4 KiB) in HBM, zeroed, and the
    // window [0, len) of the uwin onto it
    Landing(coyote::cThread &t, uint64_t n) : v80(t), len(n) {
        buf = static_cast<uint64_t *>(v80.getMem({coyote::CoyoteAllocType::HPF, len}));
        clear();
        v80.setCSR(reinterpret_cast<uint64_t>(buf), LAND_BASE);
        v80.setCSR(v80.getCtid(), LAND_PID);
        v80.setCSR(len, LAND_LEN);
    }
    ~Landing() { v80.setCSR(0, LAND_LEN); }

    // The window's part of the uwin, as a dma-buf for the U280 to import
    int export_fd() { return v80.exportDmabuf(EXPORT_REGION_UWIN, 0, len); }

    // Zero the buffer in HBM, so the next copy must write every byte again
    void clear() {
        memset(buf, 0, len);
        v80.invoke(coyote::CoyoteOper::LOCAL_OFFLOAD, coyote::syncSg{buf, len});
    }

    uint64_t csr(uint32_t w) { return v80.getCSR(w); }

    // Wait until a store beyond `stores_before` has been posted and every
    // posted card write is complete; false on timeout
    bool wait(uint64_t stores_before, std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
        const auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - t0 < timeout) {
            if (csr(LAND_STORES) > stores_before) {
                const uint64_t reqs = csr(LAND_REQS);
                if (csr(LAND_DONE) == reqs) return true;
            }
        }
        return false;
    }

    // HBM back to host memory
    void pull() { v80.invoke(coyote::CoyoteOper::LOCAL_SYNC, coyote::syncSg{buf, len}); }
};

}  // namespace land_v80
