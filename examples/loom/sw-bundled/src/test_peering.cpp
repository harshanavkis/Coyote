/**
 * 6.2a FPGA-free smoke test: the loomd<->loomd TCP peering protocol,
 * no cThread, no vFPGA - runs natively anywhere (like test_loomd_proto
 * for the local protocol).
 *
 * Covers: hello handshake (magic/version/staging passthrough), handle
 * resolution field passthrough, remote refusal of unknown handles,
 * unknown-op survival (connection stays alive), concurrent peers,
 * abrupt-disconnect resilience, the DONE barrier, and clean failure
 * against a dead port.
 */

#include <cstdio>
#include <cstring>
#include <thread>
#include <unistd.h>

#include "loom_peer.hpp"

namespace {

int failures = 0;

void check(bool ok, const char *msg) {
    printf("%s: %s\n", ok ? "PASS" : "FAIL", msg);
    if (!ok) failures++;
}

// Registry stand-in: two segments with distinctive field values
class FakeResolver : public loom::SegmentResolver {
public:
    bool resolve(loom::Handle h, loom::Segment &out) override {
        if (h == 1) { out = {7,  0x7F1B'D420'0000ULL, 0x40'0000ULL}; return true; }
        if (h == 2) { out = {12, 0x7F9E'8860'0000ULL, 0x20'0000ULL}; return true; }
        return false;
    }
};

constexpr uint64_t STAGING = 0x7F00'AB00'0000ULL;

} // namespace

int main() {
    FakeResolver resolver;
    loom::PeerServer server(resolver, STAGING, 0 /*ephemeral*/);
    if (!server.start()) { printf("FAIL: server start\n"); return 1; }
    std::thread server_thr([&] { server.run(); });
    printf("peer server on 127.0.0.1:%u\n", server.port());

    // --- 1. Connect + hello ---
    loom::PeerClient c1;
    check(c1.connect("127.0.0.1", server.port()), "connect + hello accepted");
    check(c1.stagingVa() == STAGING, "hello staging VA passthrough");

    // --- 2. Handle resolution: exact field passthrough ---
    loom::Segment s{};
    check(c1.resolve(1, s) && s.ctid == 7 && s.va == 0x7F1B'D420'0000ULL &&
          s.len == 0x40'0000ULL, "resolve h1 fields");
    check(c1.resolve(2, s) && s.ctid == 12 && s.va == 0x7F9E'8860'0000ULL &&
          s.len == 0x20'0000ULL, "resolve h2 fields");

    // --- 3. Unknown handle refused (connection intact) ---
    check(!c1.resolve(1234, s), "unknown handle refused");
    check(c1.resolve(1, s), "connection alive after refusal");

    // --- 4. Unknown op answered, not fatal ---
    {
        loom::PeerResp resp{};
        check(c1.rpc({0xDEAD, 0}, resp) && resp.status == -2,
              "unknown op answered with -2");
        check(c1.resolve(2, s), "connection alive after unknown op");
    }

    // --- 5. Concurrent second peer ---
    {
        loom::PeerClient c2;
        check(c2.connect("127.0.0.1", server.port()), "second peer connects");
        loom::Segment s2{};
        check(c2.resolve(1, s2) && s2.ctid == 7, "second peer resolves");
        check(c1.resolve(2, s), "first peer unaffected");
        // c2 goes out of scope here: abrupt close
    }

    // --- 6. Disconnect resilience: server keeps serving after a close ---
    usleep(200000);   // let the poll loop reap the closed connection
    check(c1.resolve(1, s), "server serves on after peer disconnect");

    // --- 7. DONE barrier ---
    check(server.doneCount() == 0, "no DONE before the client sends it");
    check(c1.done(), "DONE acknowledged");
    for (int i = 0; i < 100 && server.doneCount() < 1; i++) usleep(10000);
    check(server.doneCount() == 1, "DONE counted server-side");
    check(c1.resolve(1, s), "connection alive after DONE");

    // --- 8. Dead port: clean failure, no hang ---
    {
        loom::PeerClient c3;
        check(!c3.connect("127.0.0.1", 1 /*closed port*/), "dead port fails cleanly");
    }

    server.stop();
    server_thr.join();

    printf(failures == 0 ? "LOOM PEERING TEST PASS\n"
                         : "LOOM PEERING TEST FAIL (%d)\n", failures);
    return failures == 0 ? 0 : 1;
}
