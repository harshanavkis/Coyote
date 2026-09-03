#!/usr/bin/env python3
"""Two-host Loom run, driven entirely from clara.

Both sides' output is captured HERE: the server's stdout is streamed back
over the ssh connection rather than written on amy and fetched afterwards,
so a run that dies mid-way still leaves whatever the server managed to say.

BEFORE EVERY SWEEP POINT both cards go through teardown -> flash -> setup,
in the order the card needs, WAITING FOR EACH STAGE TO FINISH before starting
the next. The bench does not return to a comparable state on its own, so a
point run on what the previous point left behind measures that damage rather
than the pacing under test. --no-flash skips it, for debugging this script
only.

    teardown_coyote.sh   rmmod, PCI remove, rescan - endpoint goes down
                         wait: endpoint re-enumerated, PCIe link trained
    program_loom.tcl     JTAG program, selecting the U280 by PART
                         wait: --settle, untouched (see settle_after_program)
    setup_coyote.sh      PCI remove/rescan, insmod with this host's ip/mac
                         wait: driver bound and the sysfs node published

The teardown BEFORE programming is what lets the rescan afterwards
re-enumerate the endpoint, so no reboot is needed and the run continues in
the same pass.

Earlier revisions slept a flat 5s between the three and moved on regardless.
That is what dropped clara's U280 off the bus: a stage that had not finished
was followed by one that assumed it had. amy only looked reliable because its
steps go over ssh and the round trips spaced them by accident.

    ./run_two_host.py                          # 1 MB x2, back to back
    ./run_two_host.py --iters 8 --gap 15       # 15 us between messages
    ./run_two_host.py --gap 180,160,140        # a sweep, flashed per point
    ./run_two_host.py --gap 180,180,180        # the same point three times
"""

import argparse
import io
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
BDF         = os.environ.get("BDF", "e1:00.0")   # the U280 on clara and amy
# Absolute, because numactl lives in the nix store and is NOT on the PATH that
# `sudo env ...` gets - neither locally nor over ssh. Same store path on both
# hosts. This is the same trap as vivado and cmake.
NUMACTL     = os.environ.get("NUMACTL",
    "/nix/store/gdni20c8009xdz8gms6yn1r2hfhmk1jk-numactl-2.0.18/bin/numactl")
SSH         = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10"]

# Each stage is waited for by POLLING THE CARD until it reports the state that
# stage was supposed to produce - never by sleeping a guess. Generous, because
# these are upper bounds on a wait that normally ends in a second or two, not
# delays that are actually spent.
WAIT_TEARDOWN = 60    # the rescan done re-enumerating, link trained
WAIT_SETUP    = 60    # insmod returned and the sysfs node published
WAIT_VIVADO   = 900   # JTAG programming, start to finish
SETUP_TRIES   = 3     # setup_coyote.sh runs that did not take are re-run
BOLD, DIM, RED, RESET = "\033[1m", "\033[2m", "\033[31m", "\033[0m"


def say(msg):
    print(f"\n{BOLD}=== {msg}{RESET}", flush=True)


def die(msg):
    print(f"\n{RED}ERROR: {msg}{RESET}", file=sys.stderr, flush=True)
    sys.exit(1)


def local(cmd, check=True, quiet=True, timeout=None):
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                       timeout=timeout)
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


def card_state(is_remote):
    """Everything the flash stages wait on, in ONE round trip.

    One command rather than six probes: over ssh every probe is a fresh
    connection, and it was exactly that accidental spacing that made amy look
    reliable while clara, probed locally, did not get it. Timing must not
    depend on how a host happens to be reached.
    """
    d = f"/sys/bus/pci/devices/0000:{BDF}"
    cmd = "; ".join([
        f"echo vendor=$(cat {d}/vendor 2>/dev/null)",
        f"echo device=$(cat {d}/device 2>/dev/null)",
        f"echo width=$(cat {d}/current_link_width 2>/dev/null)",
        f"echo driver=$(readlink {d}/driver 2>/dev/null | sed 's:.*/::')",
        "echo module=$(grep -c '^coyote_driver ' /proc/modules)",
        f"echo sysfs=$(ls -d {SYSFS_GLOB} 2>/dev/null | head -1)",
        f"echo numa=$(cat {d}/numa_node 2>/dev/null)",
    ])
    out = (remote(cmd, check=False) if is_remote else local(cmd, check=False)).stdout
    st = {}
    for line in out.splitlines():
        k, _, v = line.partition("=")
        st[k.strip()] = v.strip()
    return st


def numactl_prefix(args, is_remote, host):
    """Pin this host's process to the NUMA node ITS card is on.

    Measured 2026-09-02, and it is the whole ballgame. The U280 sits on node
    1 on both hosts; unpinned, the scheduler puts the process wherever it
    likes and its source buffer follows. Land on node 0 and every DMA pull
    crosses the inter-socket link, the engine starves (84% starved, 16%
    moving), it emits a gappy stream, packets are lost, the RC layer
    retransmits, and a retransmit lands displaced - the corruption this
    project spent weeks attributing to Loom's logic.

    Same sweep, same bitstream, same minute, only the node differing:
      node 1: every size PASS, 0 retransmissions, 12.5 GB/s at 1 MB
      node 0: 2.4 GB/s at 256 KB, then 1 MB wedges with 479 retransmissions
              and the exporter reports CORRUPT
    """
    if args.numa == "off":
        return ""
    node = (args.numa if args.numa != "auto"
            else card_state(is_remote).get("numa"))
    if node in (None, "", "-1"):
        die(f"{host}: cannot read the card's NUMA node; pass --numa <n> "
            f"or --numa off")
    if not os.access(NUMACTL, os.X_OK):
        die(f"no numactl at {NUMACTL}; set NUMACTL=<path> or --numa off")
    return f"{NUMACTL} --cpunodebind={node} --membind={node} "


def show(st):
    return (f"vendor={st.get('vendor') or '-'} device={st.get('device') or '-'} "
            f"link_width={st.get('width') or '-'} "
            f"driver={st.get('driver') or '-'} "
            f"module={st.get('module') or '-'} "
            f"sysfs={st.get('sysfs') or '-'}")


def poll_for(done, is_remote, timeout):
    """Poll until done(state); return the state, or None on timeout."""
    deadline = time.time() + timeout
    st = {}
    while time.time() < deadline:
        st = card_state(is_remote)
        if done(st):
            return st
        time.sleep(1)
    return None


def wait_for(what, is_remote, done, timeout, host):
    """Poll the card until `done(state)` holds, or fail loudly.

    Returning early on a timeout would put us straight back in the failure
    this replaces - the next stage running against a card still in motion -
    so a timeout is fatal and prints what the card actually reported.
    """
    deadline = time.time() + timeout
    st = {}
    while time.time() < deadline:
        st = card_state(is_remote)
        if done(st):
            return st
        time.sleep(1)
    die(f"{host}: waited {timeout}s for {what}, and it did not happen.\n"
        f"  card reports: {show(st)}")


def settle_after_program(host, seconds):
    """The one wait that is a sleep, and the reason it has to be.

    After JTAG programming there is nothing on this host worth polling. The
    kernel still holds the PRE-programming device: vendor/device in sysfs are
    values it cached at enumeration, so they answer happily and tell us
    nothing, while current_link_width is a live config read to an endpoint
    that is mid-reset - the probe is a hazard, not a measurement. So do what
    is done by hand: leave it alone, then let setup_coyote.sh remove and
    rescan, which rebuilds the kernel's view from scratch. Only after that is
    there anything true to poll.
    """
    print(f"   {host}: letting the link retrain, untouched, for {seconds}s",
          flush=True)
    time.sleep(seconds)


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


def flash(settle, hosts=("amy", "clara")):
    if not os.path.isfile(BIT):
        die(f"no bitstream at {BIT}")
    # vivado exists only inside xilinx-shell, and a non-interactive ssh does
    # not get it on PATH - without this the programming step silently does
    # nothing and reports no error of its own.
    prog = (f'xilinx-shell -c "cd {COYOTE}/examples/loom/hw && vivado -mode batch '
            f'-nolog -nojournal -notrace -source program_loom.tcl -tclargs {BIT}"')

    for host, run in (("amy", remote), ("clara", local)):
        if host not in hosts:
            continue
        is_remote = host == "amy"
        say(f"{host}: teardown -> program -> setup")

        # e1:00.0 is the U280 on both hosts. If it is anything else the
        # card has ALREADY fallen off the bus and the BDF has been taken by
        # whatever the renumbering put there - tearing down that device
        # would remove the wrong subtree, so stop instead.
        st = card_state(is_remote)
        if st.get("vendor") != "0x10ee":
            die(f"{host}: the U280 is not at 0000:{BDF} (found "
                f"{st.get('vendor') or 'nothing'}), so it is already off the "
                f"bus. Recover it with a warm reboot before flashing.\n"
                f"  card reports: {show(st)}")

        # ---- 1/3 teardown: rmmod, PCI remove, rescan -------------------
        print(f"   {host}: teardown", flush=True)
        run(f"cd {COYOTE} && sudo bash teardown_coyote.sh",
            check=False, quiet=False)
        # teardown_coyote.sh returns as soon as it has WRITTEN to
        # .../rescan; the kernel then enumerates in the background, and that
        # enumeration is the only part of the stage that outlives the script.
        # Wait for the endpoint to be back with a trained link.
        #
        # Deliberately NOT waiting for the module to be gone: the driver has a
        # MODULE_DEVICE_TABLE, so the rescan can trigger a udev auto-load and
        # bring it straight back (setup_coyote.sh says as much, and rmmods a
        # second time for exactly this reason). Requiring module=0 would hang
        # for the full timeout on a perfectly healthy card. Programming with
        # it loaded is what happens by hand too, so let it be - just say so.
        #
        # Probing here is safe in a way it is not after programming: until the
        # device node exists there is nothing to read, and once it exists the
        # kernel has already enumerated it.
        st = wait_for("the rescan to re-enumerate the card", is_remote,
                      lambda s: (s.get("vendor") == "0x10ee"
                                 and s.get("width") not in ("", "0", None)),
                      WAIT_TEARDOWN, host)
        back = " (udev re-loaded the driver)" if st.get("sysfs") else ""
        print(f"   {host}: torn down, endpoint back at {BDF}{back}", flush=True)

        # ---- 2/3 program: JTAG, selecting the U280 by PART --------------
        print(f"   {host}: programming over JTAG (vivado, minutes)", flush=True)
        r = run(prog, check=False, quiet=True, timeout=WAIT_VIVADO)
        sel = [l for l in r.stdout.splitlines()
               if "SELECTED" in l or "PROGRAMMED" in l or l.startswith("ERROR")]
        for l in sel:
            print("   " + l.strip(), flush=True)
        if r.returncode != 0 or not any("PROGRAMMED" in l for l in sel):
            die(f"{host}: programming did not complete (vivado exit "
                f"{r.returncode})\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}")
        settle_after_program(host, settle)

        # ---- 3/3 setup: PCI remove/rescan, insmod with ip/mac -----------
        # RE-RUN, not merely waited on. If setup ran while the card was still
        # settling its insmod does not take, and no amount of waiting fixes
        # that - the remedy is to run setup again, which is what is done by
        # hand. amy hit exactly this: one setup left driver=- module=0, and
        # the identical command by hand a minute later worked first try.
        up = lambda s: (s.get("device") == "0x903f"
                        and s.get("driver") == "coyote_driver"
                        and bool(s.get("sysfs")))
        st = None
        for attempt in range(1, SETUP_TRIES + 1):
            print(f"   {host}: setup ({attempt}/{SETUP_TRIES})", flush=True)
            run(f"cd {COYOTE} && sudo bash setup_coyote.sh",
                check=False, quiet=False)
            st = poll_for(up, is_remote, WAIT_SETUP // SETUP_TRIES)
            if st:
                break
            print(f"   {host}: setup did not take, running it again",
                  flush=True)
        if not st:
            die(f"{host}: the driver did not attach after {SETUP_TRIES} "
                f"setup_coyote.sh runs.\n"
                f"  card reports: {show(card_state(is_remote))}")
        print(f"   {host}: card programmed and driver up ({show(st)})",
              flush=True)


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
            # insmod returns before the sysfs node is published; checking
            # straight away reports a failure that has not happened yet.
            wait_for("the driver to attach", is_remote,
                     lambda s: bool(s.get("sysfs")), WAIT_SETUP, host)
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



def one_run(args, env):
    """One server+client pass. Returns None if the bench wedged."""
    srv_n0, cli_n0 = read_nstats(True), read_nstats(False)

    remote("sudo pkill -x loom_host; true", check=False)
    local("sudo pkill -x loom_host || true", check=False)
    time.sleep(1)

    # stdbuf: over ssh stdout is not a tty, so libc block-buffers it and the
    # readiness line never arrives for the poll below.
    say(f"starting server on {SERVER_HOST}")
    srv_cmd = (f"cd {BUILD} && sudo stdbuf -oL -eL env {env} "
               f"{numactl_prefix(args, True, 'amy')}./loom_host --server")
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
    cli_cmd = (f"cd {BUILD} && sudo stdbuf -oL -eL env {env} "
               f"{numactl_prefix(args, False, 'clara')}./loom_host "
               f"--client {SERVER_IP}")
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

    # Returned even on a wedge: the wedge is the thing under investigation,
    # so its output has to reach the log rather than being dropped.
    return (srv_out, cli_out, srv_n0, read_nstats(True), cli_n0,
            read_nstats(False))


def verdict(result):
    """What this run actually proved. "intact" requires POSITIVE proof.

    An earlier version called a run intact whenever the server output simply
    lacked the word CORRUPT. A run that aborted at warm-up - nothing sent,
    amy's counters all zero, six FAIL lines on the server - therefore scored
    as "intact, 0 retransmissions". A verified-correct rate is the headline
    number this sweep exists to produce, so a false clean is the worst thing
    this script can emit. Nothing counts as intact now unless the server says
    in as many words that it passed.

    Fence wording differs by where the bench gave up: the per-size table
    prints NO FENCE in its landed column, while an abort during warm-up
    prints "warm-up never fenced - stopping" and no table row at all.
    """
    srv_txt, cli_txt = "".join(result[0]), "".join(result[1])
    if "never fenced" in cli_txt or "NO FENCE" in cli_txt:
        return "WEDGED"
    if "CORRUPT" in srv_txt:
        return "CORRUPT"
    if "LOOM HOST SERVER PASS" in srv_txt:
        return "intact"
    return "FAIL"


def wedged(result):
    """Nothing was transferred, so "intact" and "0 lost" would both be true
    and both meaningless."""
    return verdict(result) == "WEDGED"


def bench_env(args, gap):
    # size 0 means "every size in BENCH_SIZES": the bench sweeps its whole
    # list when LOOM_BENCH_ONLY is unset. It stops at the first size that
    # retransmits, since later rows would measure RC recovery instead.
    env = "LOOM_BENCH=1 LOOM_BENCH_NO_STORES=1 "
    if args.size:
        env += f"LOOM_BENCH_ONLY={args.size} "
    env += f"LOOM_BENCH_ITERS={args.iters}"
    if gap:
        env += f" LOOM_BENCH_GAP_US={gap}"
    if args.credit:
        env += f" LOOM_BENCH_CREDIT={args.credit}"
    if args.skip_bulk:
        env += " LOOM_SKIP_BULK=1"
    return env


def log_run(path, args, gap, env, result):
    """Append both sides of one run to the log.

    Previous revisions printed "full log: <path>" and never wrote it - the
    file on disk had been made by hand with a shell redirect.
    """
    srv_out, cli_out, srv_n0, srv_n1, cli_n0, cli_n1 = result
    with open(path, "a", encoding="utf-8") as f:
        f.write(f"\n\n{'='*72}\n")
        f.write(f"# {datetime.now():%Y-%m-%d %H:%M:%S}  size={args.size} "
                f"iters={args.iters} gap={gap}us\n\n")
        f.write(f"sudo env {env} ./loom_host --server\n")
        f.writelines(srv_out)
        f.write(f"\nsudo env {env} ./loom_host --client {SERVER_IP}\n")
        f.writelines(cli_out)
        for name, txt in (("server before", srv_n0), ("server after", srv_n1),
                          ("client before", cli_n0), ("client after", cli_n1)):
            f.write(f"\n{name} stats:\n{txt}")


# Worst-first. A CORRUPT attempt is a REAL MEASUREMENT and must never be
# retried away; only a WEDGE is an infrastructure failure worth re-running.
VERDICT_RANK = {"CORRUPT": 0, "FAIL": 1, "WEDGED": 2, "intact": 3}


def summarize(args, gap, result, verdicts=None):
    """One row of the sweep."""
    srv_out, cli_out, srv_n0, srv_n1, cli_n0, cli_n1 = result
    cli_txt, srv_txt = "".join(cli_out), "".join(srv_out)
    m = re.search(rf"^\s*{args.size}\s+\d+\s+\d+\s+\d+\s+[\d.]+\s+([\d.]+)",
                  cli_txt, re.M)
    rt = re.search(r"whole run: (\d+) retransmissions", cli_txt)
    tx = roce(cli_n1, "TX"), roce(cli_n0, "TX")
    rx = roce(srv_n1, "RX"), roce(srv_n0, "RX")
    d_tx = tx[0] - tx[1] if None not in tx else None
    d_rx = rx[0] - rx[1] if None not in rx else None
    lost = d_tx - d_rx if None not in (d_tx, d_rx) else None
    # Report the WORST verdict across attempts, not the last one. Retrying
    # until a run comes back clean and printing only that run turns "corrupt
    # on 2 of 3 attempts" into "intact" - the same false clean this script
    # already had once, arrived at by a different route.
    vs = list(verdicts) if verdicts else [verdict(result)]
    # A WEDGE is the infrastructure failure the retry exists to get past, so
    # it is discarded once any attempt produced a real verdict. CORRUPT and
    # FAIL are real results and always survive.
    real = [v for v in vs if v != "WEDGED"] or vs
    worst = min(real, key=lambda v: VERDICT_RANK.get(v, 3))
    return {"gap": gap,
            "rate": m.group(1) if m else "?",
            "retrans": rt.group(1) if rt else "?",
            "tx": d_tx, "rx": d_rx, "lost": lost,
            "payload": worst,
            "attempts": vs}


def recover(settle=15):
    """Clear a wedge by REFLASHING. A driver reload is not enough.

    Owner, from experience, confirmed by this bench's own counters: after a
    wedge only the first run following a FLASH puts real traffic on the wire
    (2017 ROCE packets); every run after a mere driver reload sends 2. So a
    reload leaves the shell's send window filled and never drained, and every
    later sweep point inherits the wrecked QP and wedges in turn - which is
    how a sweep silently turns into one failure repeated N times.

    Reflashing costs minutes per point, which is the price of a real
    measurement; the alternative is a fast sweep of meaningless rows.
    """
    flash(settle)


def run_gap(args, gap):
    """One sweep point, retried through a wedge.

    Returns (last result, every attempt's verdict). The caller reports the
    worst of those verdicts - a run that corrupted still counts even if a
    later attempt came back clean.
    """
    env = bench_env(args, gap)
    verdicts = []
    result = None
    for attempt in range(1, args.retries + 2):
        result = one_run(args, env)
        log_run(args.out, args, gap, env, result)
        v = verdict(result)
        verdicts.append(v)
        if v != "WEDGED":
            return result, verdicts
        if attempt > args.retries:
            say(f"gap={gap}us stayed wedged after {args.retries} retries - "
                f"recording it and moving on")
            return result, verdicts
        say(f"bench wedged - reflashing both cards and retrying "
            f"({attempt}/{args.retries})")
        recover(args.settle)
    return result, verdicts


def main():
    ap = argparse.ArgumentParser(
        description="Two-host Loom run, all logs captured on clara",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--size", type=int, default=1048576,
                    help="one transfer size in bytes, or 0 to sweep them all")
    ap.add_argument("--iters", type=int, default=2)
    ap.add_argument("--gap", default="0",
                    help="microseconds between messages; comma-separated for "
                         "a sweep, e.g. --gap 0,10,20,40,80")
    ap.add_argument("--no-flash", action="store_true",
                    help="skip the per-point teardown/flash/setup. FOR "
                         "DEBUGGING THE SCRIPT ONLY - the measurements it "
                         "produces are not comparable (see main)")
    ap.add_argument("--numa", default="auto",
                    help="NUMA node to pin each side to: 'auto' (default) "
                         "pins each host to the node ITS card reports, a "
                         "number forces one, 'off' disables pinning. Leaving "
                         "it unpinned is what made runs randomly slow AND "
                         "corrupt - see numactl_prefix()")
    ap.add_argument("--settle", type=int, default=15,
                    help="seconds to leave the card alone after programming, "
                         "while the PCIe link retrains (default 15)")
    ap.add_argument("--credit", type=int, default=0,
                    help="cap unretired descriptors (LOOM_BENCH_CREDIT). "
                         "0 = uncapped. NOTE: this was added believing "
                         "outstanding-write count caused the iters=32 wedge. "
                         "It does not - iters 16 and 20 pass, and iters=32 "
                         "passes too once paced. That wedge is receiver "
                         "overrun escalating, the same bug as the corruption. "
                         "The shell already enforces its own window in "
                         "rdma_flow.sv and the engine already waits on "
                         "sq_wr.ready. Kept as a knob; do not read a "
                         "mechanism into it")
    ap.add_argument("--skip-bulk", action="store_true")
    ap.add_argument("--retries", type=int, default=2,
                    help="reflash both cards and retry after a wedge")
    ap.add_argument("--out", default=f"{COYOTE}/examples/loom/experiments-log.txt")
    args = ap.parse_args()

    try:
        gaps = [int(g) for g in args.gap.split(",") if g.strip()]
    except ValueError:
        die(f"--gap wants integers, comma-separated; got {args.gap!r}")
    if not gaps:
        die("--gap: no sweep points given")

    preflight()

    rows = []
    for i, gap in enumerate(gaps, 1):
        say(f"sweep point {i}/{len(gaps)}: gap={gap}us")
        # TEARDOWN -> FLASH -> SETUP ON BOTH HOSTS BEFORE EVERY POINT.
        # Owner mandate, and the reason for it is measured: the bench does
        # not return to a comparable state by itself. Only the first run
        # after a flash puts real traffic on the wire (2017 ROCE packets vs 2
        # after a driver reload), so a point measured on the bench the
        # previous point left behind is not a measurement of that point - it
        # is a measurement of the previous point's damage. This is what made
        # gap=160us read "wedged" once and "intact" once.
        #
        # It costs minutes per point. That is the price of every point being
        # independent, and it is cheaper than a table of numbers that cannot
        # be compared to each other.
        if not args.no_flash:
            say("teardown -> flash -> setup, both hosts")
            flash(args.settle)
        result, verdicts = run_gap(args, gap)
        row = summarize(args, gap, result, verdicts)
        rows.append(row)
        extra = ("  [attempts: " + ", ".join(row["attempts"]) + "]"
                 if len(row["attempts"]) > 1 else "")
        print(f"   gap={gap}us -> {row['rate']} GB/s, {row['retrans']} retrans, "
              f"{row['payload']}{extra}", flush=True)
        # "a single descriptor of this size does not complete; its recovery
        # has been seen splattering into the region below" - the bench's own
        # words. A point that ended badly leaves the QP wrecked, and only a
        # reflash clears it, so pay for one before measuring the next point.
        if row["payload"] != "intact" and i < len(gaps):
            say("reflashing to clear the wedge before the next point")
            recover(args.settle)

    # ------------------------------------------------------------- summary
    say("sweep summary")
    print(f"  {'gap us':>7} {'GB/s':>8} {'retrans':>8} {'TX':>7} {'RX':>7} "
          f"{'lost':>6}  payload")
    for r in rows:
        n = lambda v: "?" if v is None else str(v)
        mark = ""
        if r["payload"] == "CORRUPT":
            mark = f"  {RED}<-- corrupt{RESET}"
        elif r["payload"] == "WEDGED":
            mark = f"  {RED}<-- wedged, nothing transferred{RESET}"
        elif r["payload"] == "FAIL":
            mark = f"  {RED}<-- server checks failed{RESET}"
        elif r["lost"]:
            mark = "  <-- receiver overrun"
        att = ("  (" + "/".join(r["attempts"]) + ")"
               if len(r["attempts"]) > 1 else "")
        print(f"  {r['gap']:>7} {r['rate']:>8} {r['retrans']:>8} "
              f"{n(r['tx']):>7} {n(r['rx']):>7} {n(r['lost']):>6}  "
              f"{r['payload']}{att}{mark}")

    # The headline number: the fastest point that was verified byte-correct.
    # "verified correct" means EVERY attempt at that gap was intact.
    clean = [r for r in rows
             if r["retrans"] == "0" and all(v == "intact" for v in r["attempts"])]
    if clean:
        best = max(clean, key=lambda r: float(r["rate"]) if r["rate"] != "?" else -1)
        print(f"\n  {BOLD}fastest verified-correct point: {best['rate']} GB/s "
              f"at gap={best['gap']}us, 0 retransmissions{RESET}")
    else:
        print(f"\n  {RED}no point was both intact and retransmission-free{RESET}")

    print(f"\nfull log: {args.out}")
    return 0 if clean else 1


if __name__ == "__main__":
    sys.exit(main())
