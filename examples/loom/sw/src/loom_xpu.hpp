#pragma once

#include <cstdint>
#include <mutex>

#include <coyote/cThread.hpp>
#include "loom.hpp"
#include "loom_orch.hpp"

/**
 * Client role: an emulated XPU. Pure data-plane object plus calls into
 * the OrchClient interface for control-plane operations. It never
 * touches table CSRs itself - only its own aperture windows, its own
 * doorbell registers, and its own memory.
 *
 * In the hardware multi-cThread mode each Xpu owns a distinct cThread
 * (distinct ctid -> real cross-pid destination translation). In the
 * simulation degenerate mode all roles share the process's single
 * cThread; `io_mtx` serializes their CSR traffic in that case (the
 * mock backend is not reentrant).
 */
namespace loom {

class Xpu {
public:
    Xpu(coyote::cThread &t, OrchClient &orch, std::mutex &io_mtx)
        : t_(t), orch_(orch), io_(io_mtx) {}

    uint32_t ctid() const { return t_.getCtid(); }

    // Data-plane memory: pinned, TLB-mapped under this Xpu's ctid
    void *alloc(uint64_t len) {
        std::lock_guard<std::mutex> g(io_);
        return t_.getMem({coyote::CoyoteAllocType::HPF, len});
    }
    void *allocSmall(uint64_t len) {
        std::lock_guard<std::mutex> g(io_);
        return t_.getMem({coyote::CoyoteAllocType::REG, len});
    }

    // Control plane, via the orchestrator only
    Handle exportBuf(const void *va, uint64_t len) {
        return orch_.exportBuf(ctid(), va, len);
    }
    int importBuf(Handle h) { return orch_.importBuf(h); }

    // Peer store: <= 8 B write through the aperture window
    void store(int win, uint32_t off, uint64_t v) {
        std::lock_guard<std::mutex> g(io_);
        aperture_store(t_, win, off, v);
    }

    // Peer copy: descriptor to the engine; fence released to `fence`
    // (a word in THIS Xpu's memory) when the transfer retires
    void copy(int win, uint32_t seg_off, const void *src, uint64_t len,
              const volatile uint64_t *fence) {
        std::lock_guard<std::mutex> g(io_);
        dma(t_, win, seg_off, src, len, ctid(),
            const_cast<const void *>(reinterpret_cast<const volatile void *>(fence)));
    }

private:
    coyote::cThread &t_;
    OrchClient &orch_;
    std::mutex &io_;
};

} // namespace loom
