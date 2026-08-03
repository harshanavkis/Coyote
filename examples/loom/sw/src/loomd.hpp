#pragma once

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "loom_proto.hpp"
#include "loom_orch.hpp"

/**
 * loomd: the control daemon - a Unix-socket front end over an
 * OrchClient backend.
 *
 * Layering is the point: loomd itself is transport only. In production
 * the backend is InProcOrchestrator (the sole owner of the CSR page,
 * through the daemon's cThread); in the FPGA-free protocol tests the
 * backend is a fake. Clients never see the difference between talking
 * to loomd and holding the in-process orchestrator - both are
 * OrchClient implementations (loom_sock.hpp provides the client side).
 *
 * Single-threaded poll loop, multiple concurrent client connections.
 * Requests are served in arrival order; the backend serializes CSR
 * access internally.
 */
namespace loom {

class Loomd {
public:
    Loomd(OrchClient &backend, std::string sock_path)
        : backend_(backend), path_(std::move(sock_path)) {}

    // Bind + listen; returns false on setup failure
    bool start() {
        lfd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (lfd_ < 0) return false;
        ::unlink(path_.c_str());
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path_.c_str());
        if (::bind(lfd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
            return false;
        if (::listen(lfd_, 8) < 0) return false;
        return true;
    }

    // Serve until stop(); safe to run on a dedicated thread
    void run() {
        while (!stop_.load()) {
            std::vector<pollfd> fds;
            fds.push_back({lfd_, POLLIN, 0});
            for (int c : clients_) fds.push_back({c, POLLIN, 0});

            int rc = ::poll(fds.data(), fds.size(), 100 /*ms: stop_ poll*/);
            if (rc <= 0) continue;

            if (fds[0].revents & POLLIN) {
                int c = ::accept(lfd_, nullptr, nullptr);
                if (c >= 0) clients_.push_back(c);
            }
            for (size_t i = 1; i < fds.size(); i++) {
                if (!(fds[i].revents & (POLLIN | POLLHUP | POLLERR))) continue;
                if (!serve_one(fds[i].fd)) {
                    ::close(fds[i].fd);
                    clients_.erase(std::find(clients_.begin(), clients_.end(),
                                             fds[i].fd));
                }
            }
        }
        for (int c : clients_) ::close(c);
        clients_.clear();
        if (lfd_ >= 0) ::close(lfd_);
        ::unlink(path_.c_str());
    }

    void stop() { stop_.store(true); }

private:
    // One request -> one response; false on protocol error / disconnect
    bool serve_one(int fd) {
        OrchReq req{};
        if (!read_full(fd, &req, sizeof(req))) return false;

        OrchResp resp{};
        switch (static_cast<OrchOp>(req.op)) {
            case OrchOp::EXPORT:
                resp.handle = backend_.exportBuf(
                    req.ctid, reinterpret_cast<const void *>(req.va), req.len);
                resp.status = (resp.handle == BAD_HANDLE) ? -1 : 0;
                break;
            case OrchOp::IMPORT:
                resp.win    = backend_.importBuf(req.handle);
                resp.status = (resp.win == NO_WINDOW) ? -1 : 0;
                break;
            case OrchOp::RELEASE:
                backend_.releaseWindow(req.win);
                resp.status = 0;
                break;
            default:
                resp.status = -2;      // unknown op: answer, do not kill conn
                break;
        }
        return write_full(fd, &resp, sizeof(resp));
    }

    OrchClient &backend_;
    std::string path_;
    int lfd_ = -1;
    std::vector<int> clients_;
    std::atomic<bool> stop_{false};
};

} // namespace loom
