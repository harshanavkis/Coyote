/**
 * Standalone loomd binary (hardware mode): the per-host control daemon.
 * Owns a cThread (thereby the CSR page) and serves the OrchClient
 * protocol on a Unix socket for separate application processes.
 *
 * Usage: ./loomd [socket-path]     (default /tmp/loomd.sock)
 *
 * Not for the simulation backend: each mock cThread spawns its own
 * simulator instance, so a separate daemon process would not share the
 * simulated vFPGA with its clients - use roles_sock (threaded daemon,
 * shared cThread) there instead.
 */

#include <csignal>
#include <cstdio>
#include <unistd.h>

#include <coyote/cThread.hpp>
#include "loom_orch.hpp"
#include "loomd.hpp"

static loom::Loomd *g_daemon = nullptr;

static void on_sigint(int) {
    if (g_daemon) g_daemon->stop();
}

int main(int argc, char **argv) {
    const char *path = (argc > 1) ? argv[1] : "/tmp/loomd.sock";

    coyote::cThread t(0, getpid(), 0);
    loom::InProcOrchestrator backend(t);
    loom::Loomd daemon(backend, path);
    if (!daemon.start()) {
        fprintf(stderr, "loomd: failed to bind %s\n", path);
        return 1;
    }
    g_daemon = &daemon;
    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    printf("loomd: serving on %s (ctid %d)\n", path, t.getCtid());
    daemon.run();
    printf("loomd: stopped\n");
    return 0;
}
