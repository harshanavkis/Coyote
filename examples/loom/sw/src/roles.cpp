/**
 * 5.1a: single-binary, role-split demo/test with the IN-PROCESS
 * control-plane transport (function calls behind OrchClient).
 * roles_sock.cpp runs the identical flow over a real Unix socket to
 * loomd (5.3). Shared flow: roles_test.hpp.
 *
 * Mode selection at runtime:
 *   - simulation (COYOTE_SIM_DIR set): the mock backend supports exactly
 *     one cThread per process, so all three roles share it (degenerate
 *     mode). The role separation - who calls what - is still exercised
 *     in full; only the ctids collapse to one value.
 *   - hardware (no COYOTE_SIM_DIR): each role gets its own cThread, so
 *     A's imports translate under B's distinct ctid.
 */

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <unistd.h>

#include <coyote/cThread.hpp>
#include "roles_test.hpp"

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

    loom::InProcOrchestrator orch(*t_orch);
    loom::Xpu A(ta, orch, io_mtx);
    loom::Xpu B(tb, orch, io_mtx);
    printf("ctids: orch %d, A %d, B %d\n", t_orch->getCtid(), A.ctid(), B.ctid());

    int failures = loom_test::run_roles_flow(*t_orch, A, B, orch);

    printf(failures == 0 ? "LOOM ROLES TEST PASS\n"
                         : "LOOM ROLES TEST FAIL (%d)\n", failures);
    return failures == 0 ? 0 : 1;
}
