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
 * machine are the production code paths, exercised for real. On
 * hardware the same binary uses a cThread per role; the standalone
 * `loomd` binary + separate client processes are the 5.4/5.5 rerun.
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

    // Server role: orchestrator behind loomd on a real socket
    char sock_path[64];
    snprintf(sock_path, sizeof(sock_path), "/tmp/loomd_test_%d.sock", getpid());
    loom::InProcOrchestrator backend(*t_orch);
    loom::Loomd daemon(backend, sock_path);
    if (!daemon.start()) { printf("FAIL: loomd start\n"); return 1; }
    std::thread daemon_thr([&] { daemon.run(); });

    // Client roles: one socket connection EACH
    loom::SockOrchClient orchA(sock_path), orchB(sock_path);
    if (!orchA.connected() || !orchB.connected()) {
        printf("FAIL: socket connect\n");
        daemon.stop(); daemon_thr.join();
        return 1;
    }
    printf("PASS: two clients connected to %s\n", sock_path);

    loom::Xpu A(ta, orchA, io_mtx);
    loom::Xpu B(tb, orchB, io_mtx);
    printf("ctids: orch %d, A %d, B %d\n", t_orch->getCtid(), A.ctid(), B.ctid());

    int failures = loom_test::run_roles_flow(*t_orch, A, B, orchA);

    daemon.stop();
    daemon_thr.join();

    printf(failures == 0 ? "LOOM ROLES-SOCK TEST PASS\n"
                         : "LOOM ROLES-SOCK TEST FAIL (%d)\n", failures);
    return failures == 0 ? 0 : 1;
}
