#pragma once

#include <cstdio>
#include <mutex>
#include <string>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "loom_proto.hpp"
#include "loom_orch.hpp"

/**
 * SockOrchClient: the OrchClient interface over a Unix socket to loomd.
 * Drop-in replacement for InProcOrchestrator - roles and data-plane
 * code (loom_xpu.hpp) compile against OrchClient and never notice the
 * transport. Synchronous request/response; a mutex serializes calls
 * from multiple threads sharing one connection.
 */
namespace loom {

class SockOrchClient : public OrchClient {
public:
    explicit SockOrchClient(const std::string &path) {
        fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd_ < 0) return;
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path.c_str());
        if (::connect(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
    ~SockOrchClient() override {
        if (fd_ >= 0) ::close(fd_);
    }

    bool connected() const { return fd_ >= 0; }

    Handle exportBuf(uint32_t ctid, const void *va, uint64_t len) override {
        OrchReq req{};
        req.op   = static_cast<uint32_t>(OrchOp::EXPORT);
        req.ctid = ctid;
        req.va   = reinterpret_cast<uint64_t>(va);
        req.len  = len;
        OrchResp resp{};
        if (!rpc(req, resp) || resp.status != 0) return BAD_HANDLE;
        return resp.handle;
    }

    int importBuf(Handle h) override {
        OrchReq req{};
        req.op     = static_cast<uint32_t>(OrchOp::IMPORT);
        req.handle = h;
        OrchResp resp{};
        if (!rpc(req, resp) || resp.status != 0) return NO_WINDOW;
        return resp.win;
    }

    void releaseWindow(int win) override {
        OrchReq req{};
        req.op  = static_cast<uint32_t>(OrchOp::RELEASE);
        req.win = win;
        OrchResp resp{};
        rpc(req, resp);
    }

private:
    bool rpc(const OrchReq &req, OrchResp &resp) {
        std::lock_guard<std::mutex> g(m_);
        if (fd_ < 0) return false;
        return write_full(fd_, &req, sizeof(req)) &&
               read_full(fd_, &resp, sizeof(resp));
    }

    int fd_ = -1;
    std::mutex m_;
};

} // namespace loom
