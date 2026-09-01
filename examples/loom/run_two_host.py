#!/usr/bin/env python3
"""Two-host Loom run, driven entirely from clara.

Both sides' output is captured HERE: the server's stdout is streamed back
over the ssh connection rather than written on amy and fetched afterwards,
so a run that dies mid-way still leaves whatever the server managed to say.

--flash programs both cards first, in the order the card needs:

    teardown_coyote.sh   rmmod, PCI remove, rescan - endpoint goes down
    program_loom.tcl     JTAG program, selecting the U280 by PART
    setup_coyote.sh      PCI remove/rescan, insmod with this host's ip/mac

The teardown BEFORE programming is what lets the rescan afterwards
re-enumerate the endpoint, so no reboot is needed and the run continues in
the same pass.

    ./run_two_host.py                        # 1 MB x2, back to back
    ./run_two_host.py --iters 8 --gap 15     # 15 us between messages
    ./run_two_host.py --size 4194304 --iters 1
    ./run_two_host.py --flash --iters 8      # reprogram both, then run
"""

import argparse
import os
import re
import subprocess
import sys
import threading
import time
from datetime import datetime

SERVER_HOST = os.environ.get("SERVER_HOST", "amy.dos.cit.tum.de")
SERVER_IP   = os.environ.get("SERVER_IP", "131.159.102.20")  # what the client dials
COYOTE      = os.environ.get("COYOTE", "/scratch/harshanavkis/loom-proj/Coyote")
BIT         = os.environ.get("BIT", os.path.expanduser(
                  "~/coyote-bitstreams/loom/hw/bitstreams/cyt_top.bit"))
BUILD       = f"{COYOTE}/examples/loom/sw-bundled/build"
# The sysfs index is NOT stable: it varies per host and per driver load -
# amy came up as coyote_sysfs_4 while clara was coyote_sysfs_0 - so a
# hardcoded path silently reads nothing and the counters look empty.
SYSFS_GLOB  = "/sys/kernel/coyote_sysfs_*"
SSH         = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10"]

SETTLE = 5   # seconds to let a PCI rescan finish
BOLD, DIM, RED, RESET = "\033[1m", "\033[2m", "\033[31m", "\033[0m"


def say(msg):
    print(f"\n{BOLD}=== {msg}{RESET}", flush=True)


def die(msg):
    print(f"\n{RED}ERROR: {msg}{RESET}", file=sys.stderr, flush=True)
    sys.exit(1)


def local(cmd, check=True, quiet=True):
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if check and r.returncode != 0:
        die(f"clara: {cmd}\n{r.stderr.strip()}")
    if not quiet and r.stdout.strip():
        print(r.stdout.rstrip(), flush=True)
    return r


def remote(cmd, check=True, quiet=True, timeout=600):
    r = subprocess.run(SSH + [SERVER_HOST, cmd],
                       capture_output=True, text=True, timeout=timeout)
    if check and r.returncode != 0:
        die(f"amy: {cmd}\n{r.stderr.strip()}")
    if not quiet and r.stdout.strip():
        print(r.stdout.rstrip(), flush=True)
    return r


def card_programmed(remote_host=False):
    """The Coyote shell enumerates as 10ee:903f; the unprogrammed U280 as d00c."""
    cmd = "lspci -d 10ee: -nn"
    out = (remote(cmd, check=False) if remote_host else local(cmd, check=False)).stdout
    return "10ee:903f" in out


def sysfs_dir(remote_host=False):
    """The coyote sysfs node, whatever index it landed on."""
    cmd = f"ls -d {SYSFS_GLOB} 2>/dev/null | head -1"
    r = remote(cmd, check=False) if remote_host else local(cmd, check=False)
    return r.stdout.strip()


def driver_loaded(remote_host=False):
    return bool(sysfs_dir(remote_host))


def read_nstats(remote_host=False):
    d = sysfs_dir(remote_host)
    if not d:
        return ""
    cmd = f"cat {d}/cyt_attr_nstats"
    r = remote(cmd, check=False) if remote_host else local(cmd, check=False)
    return r.stdout


def roce(nstats, direction):
    m = re.search(rf"ROCE {direction} pkgs:\s*(\d+)", nstats)
    return int(m.group(1)) if m else None


def flash(args):
    if not os.path.isfile(BIT):
        die(f"no bitstream at {BIT}")
    # vivado exists only inside xilinx-shell, and a non-interactive ssh does
    # not get it on PATH - without this the programming step silently does
    # nothing and reports no error of its own.
    prog = (f'xilinx-shell -c "cd {COYOTE}/examples/loom/hw && vivado -mode batch '
            f'-nolog -nojournal -notrace -source program_loom.tcl -tclargs {BIT}"')

    for host, run in (("amy", remote), ("clara", local)):
        # e1:00.0 is the U280 on both hosts. If it is anything else the
        # card has ALREADY fallen off the bus and the BDF has been taken by
        # whatever the renumbering put there - tearing down that device
        # would remove the wrong subtree, so stop instead.
        is_remote = host == "amy"
        vendor = (remote if is_remote else local)(
            "cat /sys/bus/pci/devices/0000:e1:00.0/vendor 2>/dev/null",
            check=False).stdout.strip()
        if vendor != "0x10ee":
            die(f"{host}: the U280 is not at 0000:e1:00.0 (found "
                f"{vendor or 'nothing'}), so it is already off the bus. "
                f"Recover it with a cold power cycle before flashing.")

        say(f"{host}: teardown -> program -> setup")
        run(f"cd {COYOTE} && sudo bash teardown_coyote.sh", check=False)
        # The rescan inside teardown is asynchronous. Programming a card the
        # kernel is still re-enumerating is how clara ended up off the bus
        # while amy - whose steps are spaced by ssh round trips - survived.
        time.sleep(SETTLE)
        r = run(prog, check=False)
        sel = [l for l in r.stdout.splitlines()
               if "SELECTED" in l or "PROGRAMMED" in l or l.startswith("ERROR")]
        for l in sel:
            print("   " + l.strip(), flush=True)
        if not any("PROGRAMMED" in l for l in sel):
            die(f"{host}: programming did not complete")
        # The endpoint is down straight after programming; give it time
        # before setup's remove/rescan tries to bring it back.
        time.sleep(SETTLE)
        run(f"cd {COYOTE} && sudo bash setup_coyote.sh", check=False)

        for _ in range(10):
            time.sleep(2)
            if card_programmed(is_remote) and driver_loaded(is_remote):
                break
        if not card_programmed(is_remote):
            die(f"{host}: card did not come back after programming "
                f"(expected 10ee:903f). It needs a cold power cycle.")
        if not driver_loaded(is_remote):
            die(f"{host}: card is up but the driver did not attach")
        print(f"   {host}: card programmed and driver up", flush=True)


def preflight():
    say("preflight")
    if remote("true", check=False).returncode != 0:
        die(f"cannot ssh to {SERVER_HOST}")

    for host, is_remote in (("clara", False), ("amy", True)):
        if not card_programmed(is_remote):
            die(f"{host}: card is not running the Coyote shell (expected 10ee:903f). "
                f"Run with --flash.")
        if not driver_loaded(is_remote):
            print(f"   {host}: loading driver", flush=True)
            (remote if is_remote else local)(
                f"cd {COYOTE} && sudo bash setup_coyote.sh", check=False)
            if not driver_loaded(is_remote):
                die(f"{host}: driver still not loaded")
        print(f"   {host}: card programmed, driver up", flush=True)

    if not os.access(f"{BUILD}/loom_host", os.X_OK):
        die(f"no loom_host at {BUILD}")
    say("syncing loom_host to amy")
    local(f"rsync -a {BUILD}/loom_host {SERVER_HOST}:{BUILD}/")

    # A server left by an interrupted run holds port 18488 ("Could not bind a
    # socket") and pushes the ctids up on every attempt. -x matches the
    # process NAME: -f would match the ssh command line below, which contains
    # "loom_host --server", and the script would kill its own child.
    remote("sudo pkill -x loom_host; true", check=False)
    local("sudo pkill -x loom_host || true", check=False)
    time.sleep(1)


def stream(proc, sink, echo_prefix=None):
    """Drain a process's stdout into a list, optionally echoing."""
    for line in proc.stdout:
        sink.append(line)
        if echo_prefix is not None:
            print(echo_prefix + line.rstrip(), flush=True)


def main():
    ap = argparse.ArgumentParser(
        description="Two-host Loom run, all logs captured on clara",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--size", type=int, default=1048576)
    ap.add_argument("--iters", type=int, default=2)
    ap.add_argument("--gap", type=int, default=0, help="microseconds between messages")
    ap.add_argument("--flash", action="store_true", help="teardown, program, setup, then run")
    ap.add_argument("--skip-bulk", action="store_true")
    ap.add_argument("--out", default=f"{COYOTE}/examples/loom/experiments-log.txt")
    args = ap.parse_args()

    if args.flash:
        flash(args)
    preflight()

    env = (f"LOOM_BENCH=1 LOOM_BENCH_NO_STORES=1 "
           f"LOOM_BENCH_ONLY={args.size} LOOM_BENCH_ITERS={args.iters}")
    if args.gap:
        env += f" LOOM_BENCH_GAP_US={args.gap}"
    if args.skip_bulk:
        env += " LOOM_SKIP_BULK=1"

    srv_n0, cli_n0 = read_nstats(True), read_nstats(False)

    # stdbuf: over ssh stdout is not a tty, so libc block-buffers it and the
    # readiness line never arrives for the poll below.
    say(f"starting server on {SERVER_HOST}")
    srv_cmd = f"cd {BUILD} && sudo stdbuf -oL -eL env {env} ./loom_host --server"
    srv = subprocess.Popen(SSH + [SERVER_HOST, srv_cmd], stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, text=True, bufsize=1)
    srv_out = []
    threading.Thread(target=stream, args=(srv, srv_out, f"{DIM}[amy] {RESET}"),
                     daemon=True).start()

    for _ in range(60):
        if any("waiting for QP exchange" in l for l in srv_out):
            break
        if srv.poll() is not None:
            print("".join(srv_out), file=sys.stderr)
            die("server exited before it was ready")
        time.sleep(1)
    else:
        print("".join(srv_out), file=sys.stderr)
        srv.kill()
        die("server never became ready")

    say("running client")
    cli_cmd = f"cd {BUILD} && sudo stdbuf -oL -eL env {env} ./loom_host --client {SERVER_IP}"
    cli = subprocess.Popen(cli_cmd, shell=True, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, text=True, bufsize=1)
    cli_out = []
    stream(cli, cli_out, "")
    cli.wait()

    try:
        srv.wait(timeout=60)
    except subprocess.TimeoutExpired:
        srv.kill()
    remote("sudo pkill -x loom_host; true", check=False)

    srv_n1, cli_n1 = read_nstats(True), read_nstats(False)

    with open(args.out, "w") as f:
        f.write(f"# {datetime.now():%F %T}  size={args.size} iters={args.iters} "
                f"gap={args.gap}us\n# server {SERVER_HOST}, client {os.uname().nodename}\n\n")
        f.write(f"sudo env {env} ./loom_host --server\n\n")
        f.writelines(srv_out)
        f.write(f"\nserver side stats (before):\n{srv_n0}")
        f.write(f"\nserver side stats (after):\n{srv_n1}")
        f.write(f"\nsudo env {env} ./loom_host --client {SERVER_IP}\n\n")
        f.writelines(cli_out)
        f.write(f"\nclient side stats (before):\n{cli_n0}")
        f.write(f"\nclient side stats (after):\n{cli_n1}")

    # ------------------------------------------------------------- summary
    cli_txt, srv_txt = "".join(cli_out), "".join(srv_out)
    say("summary")
    rate = re.search(rf"^\s*{args.size}\s+\d+\s+\d+\s+\d+\s+[\d.]+\s+([\d.]+)",
                     cli_txt, re.M)
    rt = re.search(r"whole run: (\d+) retransmissions", cli_txt)
    tx = roce(cli_n1, "TX"), roce(cli_n0, "TX")
    rx = roce(srv_n1, "RX"), roce(srv_n0, "RX")
    d_tx = tx[0] - tx[1] if None not in tx else None
    d_rx = rx[0] - rx[1] if None not in rx else None

    print(f"  rate           : {rate.group(1) if rate else '?'} GB/s")
    print(f"  retransmits    : {rt.group(1) if rt else '?'}")
    print(f"  client ROCE TX : {d_tx}")
    print(f"  server ROCE RX : {d_rx}")
    if d_tx is not None and d_rx is not None:
        lost = d_tx - d_rx
        print(f"  packets lost   : {lost}" + ("  <-- receiver overrun" if lost > 0 else ""))
    # NO FENCE means the descriptor never retired, so nothing was
    # transferred - "intact" and "0 lost" below would be true and useless.
    if "NO FENCE" in cli_txt:
        print(f"  {RED}bench WEDGED: no fence, nothing transferred{RESET}")
        for l in cli_txt.splitlines():
            if "stalled (network pushing back)" in l or "% moving" in l:
                print("                   " + l.strip())
        print(f"\nfull log: {args.out}")
        return 2

    if "CORRUPT" in srv_txt:
        print(f"  payload        : {RED}CORRUPT{RESET}")
        for l in srv_txt.splitlines():
            if "displaced payload" in l or "words wrong" in l:
                print("                   " + l.strip())
    else:
        print("  payload        : intact")
    print(f"\nfull log: {args.out}")
    return 1 if "CORRUPT" in srv_txt else 0


if __name__ == "__main__":
    sys.exit(main())
