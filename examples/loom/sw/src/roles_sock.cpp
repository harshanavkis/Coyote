/**
 * 5.3: the SAME role-split flow as roles.cpp, but with the control
 * plane behind a REAL Unix domain socket: a loomd instance serves the
 * socket (backend = InProcOrchestrator, the sole CSR-page owner), and
 * the A/B roles each hold their own SockOrchClient connection - the
 * multi-client case a real deployment has.
 *
 * In simulation (COYOTE_SIM_DIR set) the daemon runs as a thread and
 * all roles share the single mock cThread (degenerate process
 * isolation) - but the socket, the protocol, and the daemon state
 * machine are the production code paths, exercised for real.
 *
 * Hardware two-process mode: set LOOMD_SOCK=<path> to SKIP the internal
 * daemon and connect to an externally running `./loomd <path>` instead -
 * the true client/server split (control plane in another process).
 */

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <thread>
#include <unistd.h>

#include <coyote/cThread.hpp>
#include "roles_test.hpp"
#include "loomd.hpp"
#include "loom_sock.hpp"

int main() {
    const bool sim = getenv("COYOTE_SIM_DIR") != nullptr;
    printf("mode: %s\n", sim ? "simulation (single cThread, degenerate)"
                             : "hardware (cThread per role)");

    std::mutex io_mtx;
    std::unique_ptr<coyote::cThread> t_orch, t_a, t_b;
    t_orch = std::make_unique<coyote::cThread>(0, getpid(), 0);
    if (!sim) {
        t_a = std::make_unique<coyote::cThread>(0, getpid(), 0);
        t_b = std::make_unique<coyote::cThread>(0, getpid(), 0);
    }
    coyote::cThread &ta = sim ? *t_orch : *t_a;
    coyote::cThread &tb = sim ? *t_orch : *t_b;

    // Server role: external loomd (LOOMD_SOCK set) or internal thread
    const char *extern_sock = getenv("LOOMD_SOCK");
    char sock_path[108];
    std::unique_ptr<loom::InProcOrchestrator> backend;
    std::unique_ptr<loom::Loomd> daemon;
    std::thread daemon_thr;
    if (extern_sock) {
        snprintf(sock_path, sizeof(sock_path), "%s", extern_sock);
        printf("using external loomd at %s\n", sock_path);
    } else {
        snprintf(sock_path, sizeof(sock_path), "/tmp/loomd_test_%d.sock", getpid());
        backend = std::make_unique<loom::InProcOrchestrator>(*t_orch);
        daemon  = std::make_unique<loom::Loomd>(*backend, sock_path);
        if (!daemon->start()) { printf("FAIL: loomd start\n"); return 1; }
        daemon_thr = std::thread([&] { daemon->run(); });
    }

    // Client roles: one socket connection EACH
    loom::SockOrchClient orchA(sock_path), orchB(sock_path);
    if (!orchA.connected() || !orchB.connected()) {
        printf("FAIL: socket connect\n");
        if (daemon) { daemon->stop(); daemon_thr.join(); }
        return 1;
    }
    printf("PASS: two clients connected to %s\n", sock_path);

    loom::Xpu A(ta, orchA, io_mtx);
    loom::Xpu B(tb, orchB, io_mtx);
    printf("ctids: orch %d, A %d, B %d\n", t_orch->getCtid(), A.ctid(), B.ctid());

    int failures = loom_test::run_roles_flow(*t_orch, A, B, orchA);

    if (daemon) {
        daemon->stop();
        daemon_thr.join();
    }

    printf(failures == 0 ? "LOOM ROLES-SOCK TEST PASS\n"
                         : "LOOM ROLES-SOCK TEST FAIL (%d)\n", failures);
    return failures == 0 ? 0 : 1;
}
