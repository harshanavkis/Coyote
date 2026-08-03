#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "loom_proto.hpp"   // read_full / write_full
#include "loom_orch.hpp"    // Handle / Segment

/**
 * loomd <-> loomd TCP peering (the 6.1 control plane).
 *
 * One fixed-size request -> one fixed-size response, same discipline as
 * the local loomd protocol (loom_proto.hpp): the control plane is off
 * the data path, so simplicity beats efficiency.
 *
 * Connection setup: the server (exporter side's daemon) sends one
 * PeerHello immediately after accept. It carries the exporter side's
 * RDMA STAGING vaddr - the RETH address the importer side must program
 * into its RDMA_STAGING_VA CSR so its sub-64 B stores arrive as wire
 * messages the exporter's loom_rx recognizes. (The staging CSR is
 * currently global per vFPGA, which serves the asymmetric two-host
 * topology - client side only TX, server side only RX; symmetric
 * traffic needs the per-window staging planned for 6.1.)
 *
 * RESOLVE is cross-host handle resolution: the importer side's daemon
 * asks the exporter side's daemon to turn an opaque handle into the
 * segment triple {exporter ctid, exporter VA, len}. The ctid is
 * meaningful only on the exporter's host (its TLB pid); the importer
 * programs its window with the LOCAL QP-owner ctid and route=rdma, and
 * the VA/len go into the window base/bounds (the RETH carries
 * base+offset; translation finishes at the exporter's TLB).
 *
 * DONE is the end-of-run barrier for the bring-up flow: the client side
 * signals it is finished issuing traffic; the server side's main waits
 * for it before final verification/teardown.
 */
namespace loom {

constexpr uint32_t PEER_MAGIC   = 0x4C4F4F4D;   // "LOOM"
constexpr uint32_t PEER_VERSION = 1;

enum class PeerOp : uint32_t {
    RESOLVE = 1,   // handle -> {ctid, va, len}
    DONE    = 2,   // end-of-run barrier
};

struct PeerHello {
    uint32_t magic;
    uint32_t version;
    uint64_t staging_va;   // exporter side's RDMA staging buffer VA
};

struct PeerReq {
    uint32_t op;       // PeerOp
    uint32_t handle;   // RESOLVE: handle to resolve
};

struct PeerResp {
    int32_t  status;   // 0 = ok, -1 = unknown handle, -2 = unknown op
    uint32_t ctid;     // RESOLVE: exporter's cThread id (far-host pid)
    uint64_t va;       // RESOLVE: exporter's buffer VA
    uint64_t len;      // RESOLVE: segment length
};

/**
 * What the peer server needs from its backend: handle -> Segment.
 * Implemented by the bundled orchestrator (its export registry); the
 * FPGA-free smoke test provides a fake.
 */
class SegmentResolver {
public:
    virtual ~SegmentResolver() = default;
    virtual bool resolve(Handle h, Segment &out) = 0;
};

/**
 * The exporter side's peering endpoint. Single-threaded poll loop over
 * a TCP listener, multiple concurrent peer connections, same shape as
 * the local Loomd. Protocol errors kill the one connection; unknown ops
 * are answered and survived.
 */
class PeerServer {
public:
    // port 0 = ephemeral (query with port() after start; used by tests)
    PeerServer(SegmentResolver &resolver, uint64_t staging_va, uint16_t port)
        : resolver_(resolver), staging_va_(staging_va), port_(port) {}

    bool start() {
        lfd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (lfd_ < 0) return false;
        int one = 1;
        ::setsockopt(lfd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port_);
        if (::bind(lfd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
            return false;
        if (::listen(lfd_, 8) < 0) return false;
        socklen_t alen = sizeof(addr);
        if (::getsockname(lfd_, reinterpret_cast<sockaddr *>(&addr), &alen) == 0)
            port_ = ntohs(addr.sin_port);
        return true;
    }

    uint16_t port() const { return port_; }

    // Serve until stop(); safe to run on a dedicated thread
    void run() {
        while (!stop_.load()) {
            std::vector<pollfd> fds;
            fds.push_back({lfd_, POLLIN, 0});
            for (int c : peers_) fds.push_back({c, POLLIN, 0});

            int rc = ::poll(fds.data(), fds.size(), 100 /*ms: stop_ poll*/);
            if (rc <= 0) continue;

            if (fds[0].revents & POLLIN) {
                int c = ::accept(lfd_, nullptr, nullptr);
                if (c >= 0) {
                    PeerHello hello{PEER_MAGIC, PEER_VERSION, staging_va_};
                    if (write_full(c, &hello, sizeof(hello)))
                        peers_.push_back(c);
                    else
                        ::close(c);
                }
            }
            for (size_t i = 1; i < fds.size(); i++) {
                if (!(fds[i].revents & (POLLIN | POLLHUP | POLLERR))) continue;
                if (!serve_one(fds[i].fd)) {
                    ::close(fds[i].fd);
                    peers_.erase(std::find(peers_.begin(), peers_.end(),
                                           fds[i].fd));
                }
            }
        }
        for (int c : peers_) ::close(c);
        peers_.clear();
        if (lfd_ >= 0) ::close(lfd_);
    }

    void stop() { stop_.store(true); }

    // End-of-run barrier: how many DONEs have arrived
    int doneCount() const { return done_.load(); }

private:
    bool serve_one(int fd) {
        PeerReq req{};
        if (!read_full(fd, &req, sizeof(req))) return false;

        PeerResp resp{};
        switch (static_cast<PeerOp>(req.op)) {
            case PeerOp::RESOLVE: {
                Segment s{};
                if (resolver_.resolve(req.handle, s)) {
                    resp.status = 0;
                    resp.ctid = s.ctid;
                    resp.va = s.va;
                    resp.len = s.len;
                } else {
                    resp.status = -1;
                }
                break;
            }
            case PeerOp::DONE:
                done_.fetch_add(1);
                resp.status = 0;
                break;
            default:
                resp.status = -2;      // unknown op: answer, do not kill conn
                break;
        }
        return write_full(fd, &resp, sizeof(resp));
    }

    SegmentResolver &resolver_;
    uint64_t staging_va_;
    uint16_t port_;
    int lfd_ = -1;
    std::vector<int> peers_;
    std::atomic<bool> stop_{false};
    std::atomic<int> done_{0};
};

/**
 * The importer side's peering endpoint: connect, take the hello, then
 * resolve handles on demand. Synchronous (one request in flight) - all
 * uses are control-plane setup calls.
 */
class PeerClient {
public:
    ~PeerClient() { if (fd_ >= 0) ::close(fd_); }

    bool connect(const std::string &ip, uint16_t port) {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0) return false;
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) return false;
        if (::connect(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
            return false;
        PeerHello hello{};
        if (!read_full(fd_, &hello, sizeof(hello))) return false;
        if (hello.magic != PEER_MAGIC || hello.version != PEER_VERSION)
            return false;
        staging_va_ = hello.staging_va;
        return true;
    }

    uint64_t stagingVa() const { return staging_va_; }

    // Cross-host handle resolution; false on refusal or transport error
    bool resolve(Handle h, Segment &out) {
        PeerResp resp{};
        if (!rpc({static_cast<uint32_t>(PeerOp::RESOLVE), h}, resp)) return false;
        if (resp.status != 0) return false;
        out = {resp.ctid, resp.va, resp.len};
        return true;
    }

    bool done() {
        PeerResp resp{};
        return rpc({static_cast<uint32_t>(PeerOp::DONE), 0}, resp) &&
               resp.status == 0;
    }

    // Raw request escape hatch (tests: unknown-op survival)
    bool rpc(const PeerReq &req, PeerResp &resp) {
        if (fd_ < 0) return false;
        if (!write_full(fd_, &req, sizeof(req))) return false;
        return read_full(fd_, &resp, sizeof(resp));
    }

private:
    int fd_ = -1;
    uint64_t staging_va_ = 0;
};

} // namespace loom
