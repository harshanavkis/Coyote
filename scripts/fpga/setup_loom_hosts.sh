#!/usr/bin/env bash
# Bring Loom hosts' cards up from any state (after a reboot, or whatever the
# last user left): the U280 programmed and its driver loaded with the host's
# identity, then the V80 programmed and its driver loaded, at x16.
#
#   scripts/fpga/setup_loom_hosts.sh <u280.bit> <v80.pdi> <host>...   (on clara)
#
#   u280.bit, v80.pdi  images on the shared home (every host must see them),
#                      e.g. ~/coyote-bitstreams/loom-switch-coalesce/hw/bitstreams/cyt_top.bit
#                      and ~/coyote-bitstreams/loom-ce-get/cyt_top.pdi
#   host               amy, rose or clara (clara has no V80)
#
# Runs every host at once, each in a tmux session `loom_setup` ON that host
# (an interrupted ssh cannot leave a half-done or second programming run
# behind), and waits for them. The checkouts on amy and rose are older than
# clara's, so this script, its helpers and this checkout's drivers are copied
# to the shared home first (~/.cache/loom-setup/<time>/, logs <host>.log).
#
# On each host, in this order (each step avoids a way it failed before):
#   1. stop this user's hw_server: one left by Vivado 2023.2 (U280) makes
#      2025.1 (V80) refuse to connect
#   2. unload both Coyote drivers, whoever loaded them: a V80 driver loaded
#      at boot takes /sys/kernel/coyote_sysfs_0, which the U280 driver needs
#   3. U280: off the bus, JTAG program, link retrain, rescan, insmod with the
#      host's IP and MAC, check the identity and the link
#   4. V80: off the bus (its root port), JTAG program, rescan, insmod; it
#      often trains x8 the first time, so up to 3 tries for x16
# A U280 that does not come back after the rescan needs a warm reboot (the
# programmed image survives it); the script says so and stops.
set -uo pipefail

declare -A U280_IP=([clara]=0a000002 [amy]=0a000001 [rose]=0a000003)
declare -A U280_MAC=([clara]=000A350E24F2 [amy]=000A350E24D6 [rose]=000A350E24E6)
declare -A V80_BDF=([amy]=0000:81:00.0 [rose]=0000:c1:00.0)
U280_BDF=0000:e1:00.0

# ---------------------------------------------------------------------------
# On one host: setup_loom_hosts.sh --here <stage dir> <u280.bit> <v80.pdi>
# ---------------------------------------------------------------------------
if [ "${1:-}" = --here ]; then
    STAGE=$2; BIT=$3; PDI=$4
    H=$(hostname)
    source "$STAGE/vivado_env.sh"
    fail() { echo "SETUP FAILED $H: $*"; exit 1; }
    link() { echo "$(cat $1/current_link_speed 2>/dev/null) x$(cat $1/current_link_width 2>/dev/null)"; }
    jtag() {   # <image> <part pattern> <device>
        local out
        out=$(xilinx-shell -c "$(vivado_env "$(vivado_version_for $3)") cd $STAGE && vivado -mode batch -nolog -nojournal -notrace -source $STAGE/program_loom.tcl -tclargs $1 '$2'" 2>&1)
        echo "$out" | grep -E 'SELECTED|PROGRAMMED|ERROR' | sed 's/^/   /'
        echo "$out" | grep -q '^PROGRAMMED:'
    }
    [ -n "${U280_IP[$H]:-}" ] || fail "unknown host"
    [ -f "$BIT" ] || fail "no U280 image $BIT"
    [ -z "${V80_BDF[$H]:-}" ] || [ -f "$PDI" ] || fail "no V80 image $PDI"
    echo "== $H $(date -u +%H:%M:%S): U280 $BIT ($(md5sum < $BIT | cut -c1-8))${V80_BDF[$H]:+, V80 $PDI ($(md5sum < $PDI | cut -c1-8))}"

    # 1, 2
    pkill -u "$(id -u)" -x hw_server && sleep 1
    for m in coyote_driver coyote_driver_versal; do
        if lsmod | grep "^$m " >/dev/null; then
            sudo rmmod $m || fail "cannot unload $m (in use? $(sudo fuser /dev/coyote* 2>&1 | tr -s ' '))"
            echo "   unloaded $m"
        fi
    done

    # 3. U280
    D=/sys/bus/pci/devices/$U280_BDF
    for f in /sys/bus/pci/devices/${U280_BDF%.*}.*; do [ -e $f ] && echo 1 | sudo tee $f/remove >/dev/null; done
    echo "   U280: programming over JTAG"
    jtag "$BIT" 'xcu280*' u280 || fail "U280 programming failed"
    sleep 15
    [ -e $D ] && echo 1 | sudo tee $D/remove >/dev/null
    echo 1 | sudo tee /sys/bus/pci/rescan >/dev/null
    sleep 2
    [ -e $D ] || fail "the U280 did not come back at $U280_BDF: warm-reboot $H (the image stays programmed) and run this again"
    sudo rmmod coyote_driver 2>/dev/null   # a udev auto-load on the rescan, without identity
    sudo insmod $STAGE/coyote_driver.ko ip_addr=0x${U280_IP[$H]} mac_addr=${U280_MAC[$H]} || fail "U280 driver insmod failed (dmesg)"
    IP=""
    for i in $(seq 1 20); do
        IP=$(sudo cat /sys/kernel/coyote_sysfs_0/cyt_attr_ip 2>/dev/null | grep -oE '[0-9a-f]{8}' | head -1)
        [ -n "$IP" ] && break; sleep 1
    done
    echo "   U280: driver $(basename "$(readlink -f $D/driver)"), IP ${IP:-none}, link $(link $D)"
    [ "$IP" = "${U280_IP[$H]}" ] || fail "U280 identity ${IP:-none}, want ${U280_IP[$H]}"
    [ "$(cat $D/current_link_width)" = 16 ] || fail "U280 link $(link $D), want x16"
    pkill -u "$(id -u)" -x hw_server && sleep 1

    # 4. V80
    if [ -n "${V80_BDF[$H]:-}" ]; then
        V=/sys/bus/pci/devices/${V80_BDF[$H]}
        for try in 1 2 3; do
            sudo rmmod coyote_driver_versal 2>/dev/null
            if [ -e $V ]; then
                # its root port when the card is alone under it: only
                # re-enumerating the port re-sizes the bridge windows for
                # the Coyote image's BARs
                P=$(basename "$(dirname "$(readlink -f $V)")")
                [ "$(ls -d /sys/bus/pci/devices/$P/0000:??:??.? 2>/dev/null | wc -l)" -eq 1 ] || P=${V80_BDF[$H]}
                echo 1 | sudo tee /sys/bus/pci/devices/$P/remove >/dev/null
            fi
            echo "   V80 (try $try): programming over JTAG"
            jtag "$PDI" 'xcv80*' v80 || fail "V80 programming failed"
            sleep 5
            for i in $(seq 1 30); do
                echo 1 | sudo tee /sys/bus/pci/rescan >/dev/null
                [ -e $V ] && grep -qE '0x(b0|b1|b2|b3)3f' $V/device && break
                sleep 2
            done
            [ -e $V ] || fail "the V80 did not come back at ${V80_BDF[$H]}"
            sudo rmmod coyote_driver_versal 2>/dev/null
            sudo insmod $STAGE/coyote_driver_versal.ko || fail "V80 driver insmod failed (dmesg)"
            for i in $(seq 1 30); do ls -d /sys/kernel/coyote_versal_sysfs_* >/dev/null 2>&1 && break; sleep 1; done
            echo "   V80: driver $(basename "$(readlink -f $V/driver)"), link $(link $V)"
            [ "$(cat $V/current_link_width)" = 16 ] && break
            [ $try = 3 ] && fail "V80 link $(link $V) after 3 tries, want x16"
        done
        ls -d /sys/kernel/coyote_versal_sysfs_* >/dev/null 2>&1 || fail "V80 driver up but no coyote_versal_sysfs node (dmesg)"
        pkill -u "$(id -u)" -x hw_server
    fi
    echo "SETUP OK $H: U280 ${U280_IP[$H]} $(link $D)${V80_BDF[$H]:+, V80 $(link $V)} ($(date -u +%H:%M:%S))"
    exit 0
fi

# ---------------------------------------------------------------------------
# On clara: stage, start every host, wait
# ---------------------------------------------------------------------------
[ $# -ge 3 ] || { sed -n '2,13p' "$0" >&2; exit 1; }
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(git -C "$HERE" rev-parse --show-toplevel)
BIT=$(readlink -f "$1"); PDI=$(readlink -f "$2"); shift 2
for f in "$BIT" "$PDI"; do
    case $f in "$HOME"/*) ;; *) echo "$f is not on the shared home; the hosts cannot see it" >&2; exit 1 ;; esac
    [ -f "$f" ] || { echo "no file $f" >&2; exit 1; }
done
for h in "$@"; do [ -n "${U280_IP[$h]:-}" ] || { echo "unknown host $h" >&2; exit 1; }; done

STAGE=$HOME/.cache/loom-setup/$(date -u +%Y%m%d-%H%M%S)
mkdir -p "$STAGE"
cp "$0" "$HERE/vivado_env.sh" "$REPO/examples/loom/hw/program_loom.tcl" \
   "$REPO/driver/build/coyote_driver.ko" "$REPO/driver/build_versal/coyote_driver_versal.ko" "$STAGE/" || exit 1
echo "commit $(git -C "$REPO" rev-parse --short HEAD), drivers $(cd $STAGE && md5sum *.ko | cut -c1-8 | tr '\n' ' ')" > "$STAGE/INFO.txt"
echo "staged in $STAGE ($(cat $STAGE/INFO.txt))"

run_on() {   # <host> <command>
    if [ "$1" = "$(hostname)" ]; then bash -c "$2"; else ssh -o ConnectTimeout=15 -n "$1.dos.cit.tum.de" "$2"; fi
}
for h in "$@"; do
    if run_on $h "tmux has-session -t loom_setup 2>/dev/null"; then
        echo "$h: a loom_setup session is still running there (tmux attach -t loom_setup); not starting another" >&2
        exit 1
    fi
    run_on $h "tmux new-session -d -s loom_setup 'bash $STAGE/$(basename $0) --here $STAGE $BIT $PDI > $STAGE/$h.log 2>&1'" \
        || { echo "$h: could not start (ssh?)" >&2; exit 1; }
    echo "$h: started (log $STAGE/$h.log)"
done

rc=0
for h in "$@"; do
    for _ in $(seq 1 180); do grep -qE '^SETUP (OK|FAILED)' $STAGE/$h.log 2>/dev/null && break; sleep 10; done
    cat $STAGE/$h.log
    grep -q '^SETUP OK' $STAGE/$h.log || rc=1
done
exit $rc
