#pragma once

#include <cstdint>
#include <cerrno>
#include <unistd.h>

/**
 * Wire protocol between libloom clients and the loomd control daemon
 * (Unix domain socket, one fixed-size request -> one fixed-size
 * response). Deliberately minimal: the control plane is off the data
 * path, so simplicity beats efficiency here.
 */
namespace loom {

enum class OrchOp : uint32_t {
    EXPORT  = 1,   // register {ctid, va, len} -> handle
    IMPORT  = 2,   // resolve handle -> window (server programs the table)
    RELEASE = 3,   // invalidate a window
};

struct OrchReq {
    uint32_t op;       // OrchOp
    uint32_t ctid;     // EXPORT: exporter's cThread id
    uint64_t va;       // EXPORT: buffer VA
    uint64_t len;      // EXPORT: buffer length
    uint32_t handle;   // IMPORT: handle to resolve
    int32_t  win;      // RELEASE: window to invalidate
};

struct OrchResp {
    int32_t  status;   // 0 = ok, <0 = refused
    uint32_t handle;   // EXPORT result
    int32_t  win;      // IMPORT result (NO_WINDOW on refusal)
};

// Robust full-buffer read/write (handles EINTR and short transfers).
// Return true on success, false on error/EOF.
inline bool read_full(int fd, void *buf, size_t n) {
    auto *p = static_cast<uint8_t *>(buf);
    while (n > 0) {
        ssize_t r = ::read(fd, p, n);
        if (r == 0) return false;                    // peer closed
        if (r < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        p += r; n -= static_cast<size_t>(r);
    }
    return true;
}

inline bool write_full(int fd, const void *buf, size_t n) {
    auto *p = static_cast<const uint8_t *>(buf);
    while (n > 0) {
        ssize_t r = ::write(fd, p, n);
        if (r < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        p += r; n -= static_cast<size_t>(r);
    }
    return true;
}

} // namespace loom
