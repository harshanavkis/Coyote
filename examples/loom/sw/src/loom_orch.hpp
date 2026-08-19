#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

#include <coyote/cThread.hpp>
#include "loom.hpp"

/**
 * Control-plane roles (client/server structure, transport-agnostic).
 *
 * The SERVER role (orchestrator) is the only code that touches the CSR
 * page: it keeps the segment registry, allocates aperture windows, and
 * programs the window table through its own cThread.
 *
 * The CLIENT role (an XPU process/thread, see loom_xpu.hpp) only ever
 * calls the narrow OrchClient interface below. Applications never see
 * table CSRs, window allocation, or other processes' addresses - they
 * put in {my buffer, my ctid} and get back an opaque handle; they put in
 * a handle and get back a window they can store/copy through.
 *
 * Transports:
 *   5.1a (this file): InProcOrchestrator - plain function calls behind
 *        the interface; client and server live in one binary.
 *   5.1b: the same OrchClient implemented over a Unix socket to a
 *        standalone loomd; roles.cpp and loom_xpu.hpp stay unchanged.
 *   6.x: loomd additionally talks TCP to its peer loomd and sets up the
 *        per-binding QPs; importBuf then may program rdma-route windows.
 */
namespace loom {

using Handle = uint32_t;
constexpr Handle BAD_HANDLE = 0;
constexpr int NO_WINDOW = -1;

struct Segment {
    uint32_t ctid;      // exporter's cThread id (the pid the TLB translates under)
    uint64_t va;        // exporter's own buffer VA
    uint64_t len;       // bounds
};

class OrchClient {
public:
    virtual ~OrchClient() = default;

    // Register a buffer as an exported segment. Returns an opaque handle
    // the exporter passes to its peers over any channel it likes.
    virtual Handle exportBuf(uint32_t ctid, const void *va, uint64_t len) = 0;

    // Resolve a handle and bind it to a free aperture window (the server
    // programs the window table). Returns the window index, or NO_WINDOW.
    virtual int importBuf(Handle h) = 0;

    // Unbind a window (table entry invalidated).
    virtual void releaseWindow(int win) = 0;
};

/**
 * In-process orchestrator (5.1a transport). Thread-safe: multiple client
 * threads may call in concurrently; every CSR access goes through the
 * orchestrator's own cThread under one lock.
 */
class InProcOrchestrator : public OrchClient {
public:
    // `ctrl` is the orchestrator's cThread. In the hardware multi-cThread
    // mode this is a dedicated attachment whose only job is the CSR page;
    // in the simulation degenerate mode it is the process's single
    // cThread, shared with the XPU roles.
    explicit InProcOrchestrator(coyote::cThread &ctrl) : ctrl_(ctrl) {}

    Handle exportBuf(uint32_t ctid, const void *va, uint64_t len) override {
        std::lock_guard<std::mutex> g(m_);
        segs_.push_back({ctid, reinterpret_cast<uint64_t>(va), len});
        return static_cast<Handle>(segs_.size());   // handle = 1-based index
    }

    int importBuf(Handle h) override {
        std::lock_guard<std::mutex> g(m_);
        if (h == BAD_HANDLE || h > segs_.size())
            return NO_WINDOW;                        // unknown handle: refuse
        const Segment &s = segs_[h - 1];
        const int win = alloc_win();
        if (win == NO_WINDOW) return NO_WINDOW;      // aperture exhausted
        program_window(ctrl_, win, /*rdma=*/false, s.ctid,
                       reinterpret_cast<const void *>(s.va), s.len);
        return win;
    }

    void releaseWindow(int win) override {
        std::lock_guard<std::mutex> g(m_);
        if (win < 1 || win > 15) return;
        win_used_[win] = false;                      // reusable again
        // Invalidate: commit an entry with valid=0
        csr_write(ctrl_, TBL_IDX, static_cast<uint64_t>(win));
        csr_write(ctrl_, TBL_CFG, 0);
        csr_write(ctrl_, TBL_COMMIT, 1);
    }

private:
    coyote::cThread &ctrl_;
    std::mutex m_;
    std::vector<Segment> segs_;
    bool win_used_[16] = {};   // 0 is the CSR page, never a window

    // Lowest free window. Windows are a fixed resource of 15, and loomd is
    // long-lived: handing out a fresh index per import and never taking it
    // back exhausts the aperture after a handful of client runs, which is
    // exactly what a daemon serving run after run does.
    int alloc_win() {
        for (int w = 1; w <= 15; w++)
            if (!win_used_[w]) { win_used_[w] = true; return w; }
        return NO_WINDOW;
    }
};

} // namespace loom
