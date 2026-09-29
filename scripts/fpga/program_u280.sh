#!/usr/bin/env bash
# Program a U280 with a Coyote bitstream (.bit) and load its driver.
#
#   scripts/fpga/program_u280.sh <image.bit> [bdf]
#
#   image.bit  e.g. examples/loom/hw/build_sep29_ctrl/bitstreams/cyt_top.bit
#   bdf        the U280's PCIe address (default per host: clara/amy
#              0000:e1:00.0, rose 0000:c1:00.0)
#
# Steps: unload coyote_driver, remove the card from PCIe, program it over JTAG
# (Vivado 2023.2, device selected by part xcu280), let the link retrain, then
# setup_coyote.sh (PCI remove/rescan, insmod with this host's FPGA IP/MAC), and
# check the identity the driver reports. The V80 (coyote_driver_versal) is not
# touched. run_two_host.py has its own, more defensive, flash for Loom runs.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/vivado_env.sh"
REPO=$(git -C "$HERE" rev-parse --show-toplevel)

[ $# -ge 1 ] || { sed -n '2,15p' "$0" >&2; exit 1; }
IMG=$(readlink -f "$1")
BDF=${2:-$(default_bdf u280)}
TCL=$REPO/examples/loom/hw/program_loom.tcl     # generic: takes a part pattern
DEV=/sys/bus/pci/devices/$BDF
declare -A WANT_IP=([clara]=0a000002 [amy]=0a000001 [rose]=0a000003)

[ -f "$IMG" ] || { echo "no image at $IMG" >&2; exit 1; }
[ -f "$REPO/driver/build/coyote_driver.ko" ] || { echo "no driver/build/coyote_driver.ko; build the driver first" >&2; exit 1; }
case $IMG in *.bit) ;; *) echo "$IMG is not a .bit" >&2; exit 1 ;; esac

echo "== U280 at $BDF: $IMG"

# 1. driver out, card off the bus
if lsmod | grep -q '^coyote_driver '; then
    echo "   unloading coyote_driver"
    sudo rmmod coyote_driver
fi
if [ -e "$DEV" ]; then
    echo "   removing $BDF from PCIe (was $(cat $DEV/vendor):$(cat $DEV/device))"
    echo 1 | sudo tee "$DEV/remove" >/dev/null
fi

# 2. program over JTAG
echo "   programming over JTAG (Vivado $(vivado_version_for u280), minutes)"
ENV=$(vivado_env "$(vivado_version_for u280)")
OUT=$(xilinx-shell -c "$ENV cd $(dirname "$TCL") && vivado -mode batch -nolog -nojournal -notrace -source $TCL -tclargs $IMG 'xcu280*'" 2>&1) || true
echo "$OUT" | grep -E 'SELECTED|PROGRAMMED|ERROR' | sed 's/^/   /' || true
echo "$OUT" | grep -q '^PROGRAMMED:' || { echo "programming failed; full Vivado output:" >&2; echo "$OUT" | tail -30 >&2; exit 1; }

# 3. let the link retrain untouched, then the usual setup (rescan + insmod)
echo "   letting the link retrain for 15 s"
sleep 15
(cd "$REPO" && sudo bash setup_coyote.sh) | sed 's/^/   /'

# 4. check what the driver reports
for i in $(seq 1 20); do
    IP=$(sudo cat /sys/kernel/coyote_sysfs_*/cyt_attr_ip 2>/dev/null | grep -o -E '[0-9a-f]{8}' | head -1 || true)
    [ -n "$IP" ] && break
    sleep 1
done
WANT=${WANT_IP[$(hostname)]:-}
echo "   driver up: $(basename $(readlink -f $DEV/driver 2>/dev/null) 2>/dev/null), FPGA IP ${IP:-none}, link $(cat $DEV/current_link_speed 2>/dev/null) x$(cat $DEV/current_link_width 2>/dev/null)"
if [ -n "$WANT" ] && [ "$IP" != "$WANT" ]; then
    echo "WRONG IDENTITY: FPGA IP ${IP:-none}, want $WANT - reload with: sudo rmmod coyote_driver && sudo bash setup_coyote.sh" >&2
    exit 1
fi
