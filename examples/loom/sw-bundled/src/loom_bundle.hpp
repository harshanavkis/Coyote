#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

#include <coyote/cThread.hpp>
#include "loom.hpp"
#include "loom_orch.hpp"
#include "loom_peer.hpp"

/**
 * The bundled host's orchestrator (6.2a): one OrchClient that serves
 * BOTH sides of the two-host topology, depending on how it is wired:
 *
 *   - exporter side (server): exportBuf registers segments locally and
 *     the registry doubles as the SegmentResolver behind the PeerServer,
 *     so the far daemon can resolve handles. importBuf without a peer
 *     behaves like InProcOrchestrator (local windows) - the single-host
 *     bring-up configuration.
 *   - importer side (client): a PeerClient is attached; importBuf then
 *     resolves the handle CROSS-HOST and programs the window with
 *     route=rdma, pid = the LOCAL QP-owner ctid (whose RC connection
 *     carries the traffic; the far ctid from the resolution is the far
 *     TLB's business), base = the exporter's VA, len = the bounds.
 *
 * The rdma-route programming realizes the design's binding compile:
 * segment resolution (far VA) and route selection (local QP) are folded
 * into the window entry at install time; the wire then carries only
 * base+offset in the RETH (bulk) or the message header (sub-64 B).
 *
 * Duplicates the small local-import path of InProcOrchestrator rather
 * than modifying sw/ (5.x code stays frozen; this folder is 6.2a).
 */
namespace loom {

class BundledOrchestrator : public OrchClient, public SegmentResolver {
public:
    explicit BundledOrchestrator(coyote::cThread &ctrl) : ctrl_(ctrl) {}

    // Importer side: attach the peering client and the local QP owner.
    // All subsequent imports are cross-host rdma windows.
    void attachPeer(PeerClient &peer, uint32_t qp_owner_ctid) {
        std::lock_guard<std::mutex> g(m_);
        peer_ = &peer;
        qp_owner_ = qp_owner_ctid;
        set_rdma_staging(ctrl_,
            reinterpret_cast<const void *>(peer.stagingVa()));
    }

    // --- OrchClient (local clients / app roles / the local loomd) ---

    Handle exportBuf(uint32_t ctid, const void *va, uint64_t len) override {
        std::lock_guard<std::mutex> g(m_);
        segs_.push_back({ctid, reinterpret_cast<uint64_t>(va), len});
        return static_cast<Handle>(segs_.size());   // handle = 1-based index
    }

    int importBuf(Handle h) override {
        std::lock_guard<std::mutex> g(m_);

        Segment s{};
        bool rdma = false;
        if (peer_) {
            // Cross-host: the handle lives in the far daemon's registry
            if (!peer_->resolve(h, s))
                return NO_WINDOW;                    // refused remotely
            s.ctid = qp_owner_;                      // local QP selects the wire
            rdma = true;
        } else {
            if (h == BAD_HANDLE || h > segs_.size())
                return NO_WINDOW;                    // unknown handle: refuse
            s = segs_[h - 1];
        }

        const int win = alloc_win();
        if (win == NO_WINDOW) return NO_WINDOW;      // aperture exhausted
        program_window(ctrl_, win, rdma, s.ctid,
                       reinterpret_cast<const void *>(s.va), s.len);
        return win;
    }

    void releaseWindow(int win) override {
        std::lock_guard<std::mutex> g(m_);
        if (win < 1 || win > 15) return;
        win_used_[win] = false;                      // reusable again
        csr_write(ctrl_, TBL_IDX, static_cast<uint64_t>(win));
        csr_write(ctrl_, TBL_CFG, 0);
        csr_write(ctrl_, TBL_COMMIT, 1);
    }

    // --- SegmentResolver (the far daemon, via PeerServer) ---

    bool resolve(Handle h, Segment &out) override {
        std::lock_guard<std::mutex> g(m_);
        if (h == BAD_HANDLE || h > segs_.size()) return false;
        out = segs_[h - 1];
        return true;
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
    PeerClient *peer_ = nullptr;
    uint32_t qp_owner_ = 0;
};

} // namespace loom
