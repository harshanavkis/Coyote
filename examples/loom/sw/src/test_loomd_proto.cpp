/**
 * FPGA-free protocol test for loomd + SockOrchClient: no cThread is
 * ever constructed, so this runs on any machine (no device, no
 * COYOTE_SIM_DIR, no Vivado). The daemon's backend is a fake that
 * records calls and returns canned values - exactly the seam the
 * Loomd(backend) layering exists for.
 *
 * Covers: field pass-through, refusals, release, unknown-op survival,
 * two concurrent clients, disconnect resilience.
 */

#include <cstdio>
#include <cstring>
#include <thread>
#include <unistd.h>
#include <vector>

#include "loom_orch.hpp"
#include "loomd.hpp"
#include "loom_sock.hpp"

static int failures = 0;
static void check(bool ok, const char *msg) {
    printf("%s: %s\n", ok ? "PASS" : "FAIL", msg);
    if (!ok) failures++;
}

// Recording fake backend
struct FakeOrch : loom::OrchClient {
    struct Exp { uint32_t ctid; uint64_t va, len; };
    std::vector<Exp> exports;
    std::vector<loom::Handle> imports;
    std::vector<int> releases;

    loom::Handle exportBuf(uint32_t ctid, const void *va, uint64_t len) override {
        exports.push_back({ctid, reinterpret_cast<uint64_t>(va), len});
        return static_cast<loom::Handle>(exports.size());     // 1, 2, ...
    }
    int importBuf(loom::Handle h) override {
        imports.push_back(h);
        if (h == 0 || h > exports.size()) return loom::NO_WINDOW;
        return static_cast<int>(h);                           // win = handle
    }
    void releaseWindow(int win) override { releases.push_back(win); }
};

int main() {
    char path[64];
    snprintf(path, sizeof(path), "/tmp/loomd_proto_%d.sock", getpid());

    FakeOrch fake;
    loom::Loomd daemon(fake, path);
    if (!daemon.start()) { printf("FAIL: daemon start\n"); return 1; }
    std::thread thr([&] { daemon.run(); });

    {
        loom::SockOrchClient c1(path);
        check(c1.connected(), "client 1 connects");

        // Field pass-through
        loom::Handle h = c1.exportBuf(7, reinterpret_cast<void *>(0x7f12'3456'7000ULL),
                                      0x200000);
        check(h == 1, "export returns handle 1");
        check(fake.exports.size() == 1 && fake.exports[0].ctid == 7 &&
              fake.exports[0].va == 0x7f12'3456'7000ULL &&
              fake.exports[0].len == 0x200000,
              "export fields pass through verbatim");

        // Import: valid + refused
        check(c1.importBuf(h) == 1, "import resolves");
        check(c1.importBuf(999) == loom::NO_WINDOW, "bogus import refused");

        // Release
        c1.releaseWindow(4);
        check(fake.releases.size() == 1 && fake.releases[0] == 4,
              "release passes through");

        // Unknown op: daemon answers status -2 and the connection survives
        loom::OrchReq raw{};
        raw.op = 99;
        loom::OrchResp resp{};
        // (reach into the same socket via a scratch client)
        loom::SockOrchClient c_raw(path);
        int fd = -1;
        {
            // send the raw request over a fresh connection
            fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
            sockaddr_un a{};
            a.sun_family = AF_UNIX;
            snprintf(a.sun_path, sizeof(a.sun_path), "%s", path);
            check(::connect(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) == 0,
                  "raw connect");
            check(loom::write_full(fd, &raw, sizeof(raw)) &&
                  loom::read_full(fd, &resp, sizeof(resp)) && resp.status == -2,
                  "unknown op answered with -2");
            // same connection still serves a valid request
            loom::OrchReq ok{};
            ok.op = static_cast<uint32_t>(loom::OrchOp::IMPORT);
            ok.handle = 1;
            check(loom::write_full(fd, &ok, sizeof(ok)) &&
                  loom::read_full(fd, &resp, sizeof(resp)) && resp.win == 1,
                  "connection survives unknown op");
            ::close(fd);
        }

        // Two concurrent clients, interleaved
        loom::SockOrchClient c2(path);
        check(c2.connected(), "client 2 connects");
        loom::Handle h2 = c2.exportBuf(9, reinterpret_cast<void *>(0x7fab'0000'0000ULL), 4096);
        check(h2 == 2, "client 2 export while client 1 open");
        check(c1.importBuf(h2) == 2, "client 1 resolves client 2's handle");
    }   // c1/c2 disconnect here

    // Daemon survives client disconnects: a fresh client still works
    {
        loom::SockOrchClient c3(path);
        check(c3.connected() && c3.importBuf(1) == 1,
              "daemon serves after client disconnects");
    }

    daemon.stop();
    thr.join();

    printf(failures == 0 ? "LOOMD PROTO TEST PASS\n"
                         : "LOOMD PROTO TEST FAIL (%d)\n", failures);
    return failures == 0 ? 0 : 1;
}
