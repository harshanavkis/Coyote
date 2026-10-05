#!/usr/bin/env bash
# Program a V80 with a Coyote image (.pdi) and load its driver.
#
#   scripts/fpga/program_v80.sh <image.pdi> [bdf]
#
#   image.pdi  e.g. examples/07_perf_fpga/hw/build_v80/bitstreams/cyt_top.pdi
#   bdf        the V80's PCIe address (default per host: amy 0000:81:00.0,
#              rose 0000:c1:00.0)
#
# Steps: unload coyote_driver_versal if loaded, remove the card (its root port
# if the card is alone under it) from PCIe,
# program it over JTAG (Vivado 2025.1, device selected by part xcv80), rescan,
# wait for the Coyote PCI function, load driver/build_versal/
# coyote_driver_versal.ko, wait for its sysfs node and device files. The U280
# (coyote_driver) is not touched.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/vivado_env.sh"
REPO=$(git -C "$HERE" rev-parse --show-toplevel)

[ $# -ge 1 ] || { sed -n '2,15p' "$0" >&2; exit 1; }
IMG=$(readlink -f "$1")
BDF=${2:-$(default_bdf v80)}
KO=$REPO/driver/build_versal/coyote_driver_versal.ko
TCL=$REPO/examples/loom/hw/program_loom.tcl     # generic: takes a part pattern
DEV=/sys/bus/pci/devices/$BDF

[ -f "$IMG" ] || { echo "no image at $IMG" >&2; exit 1; }
[ -f "$KO" ]  || { echo "no driver at $KO; build it: cd driver && make TARGET_PLATFORM=versal KERNELDIR=..." >&2; exit 1; }
case $IMG in *.pdi) ;; *) echo "$IMG is not a .pdi (V80 images are PDIs)" >&2; exit 1 ;; esac

echo "== V80 at $BDF: $IMG"

# 1. driver out, card off the bus
# grep -q under pipefail: lsmod dies of SIGPIPE and the test reads false
if lsmod | grep '^coyote_driver_versal ' >/dev/null; then
    echo "   unloading coyote_driver_versal"
    sudo rmmod coyote_driver_versal
fi
# Remove the card's root port, not just the card, when the card is its only
# device: the Coyote image asks for other BARs (BAR0 1 MB, BAR2 512 KB, BAR4
# 256 MB) than whatever was loaded at boot, and only re-enumerating the port
# lets the kernel re-size its bridge windows. Removing just the card left
# BAR2 unassigned and the probe failed enabling MSI-X (-12).
PORT=""
if [ -e "$DEV" ]; then
    PARENT=$(basename "$(dirname "$(readlink -f "$DEV")")")
    # PCI functions only: the port's own service devices (0000:..:pcie001)
    # live in the same directory
    NDEV=$(ls -d "/sys/bus/pci/devices/$PARENT"/0000:??:??.? 2>/dev/null | wc -l)
    if [[ $PARENT == 0000:* ]] && [ "$NDEV" -eq 1 ]; then PORT=$PARENT; fi
    echo "   removing ${PORT:-$BDF} from PCIe (card was $(cat $DEV/vendor):$(cat $DEV/device))"
    echo 1 | sudo tee "/sys/bus/pci/devices/${PORT:-$BDF}/remove" >/dev/null
fi

# 2. program over JTAG
echo "   programming over JTAG (Vivado $(vivado_version_for v80), minutes)"
ENV=$(vivado_env "$(vivado_version_for v80)")
OUT=$(xilinx-shell -c "$ENV cd $(dirname "$TCL") && vivado -mode batch -nolog -nojournal -notrace -source $TCL -tclargs $IMG 'xcv80*'" 2>&1) || true
echo "$OUT" | grep -E 'SELECTED|PROGRAMMED|ERROR' | sed 's/^/   /' || true
echo "$OUT" | grep -q '^PROGRAMMED:' || { echo "programming failed; full Vivado output:" >&2; echo "$OUT" | tail -30 >&2; exit 1; }

# 3. rescan and wait for the Coyote PCI function (driver ID table: b03f, b13f, b23f, b33f)
sleep 5
for i in $(seq 1 30); do
    echo 1 | sudo tee /sys/bus/pci/rescan >/dev/null
    if [ -e "$DEV" ] && grep -q -E '0x(b0|b1|b2|b3)3f' "$DEV/device"; then break; fi
    sleep 2
done
[ -e "$DEV" ] || { echo "the V80 did not come back at $BDF after the rescan" >&2; exit 1; }
echo "   back on the bus: $(cat $DEV/vendor):$(cat $DEV/device), link $(cat $DEV/current_link_speed) x$(cat $DEV/current_link_width)"
grep -q -E '0x(b0|b1|b2|b3)3f' "$DEV/device" || { echo "not a Coyote V80 function (device $(cat $DEV/device)); was the image a Coyote shell?" >&2; exit 1; }

# 4. driver in
sudo insmod "$KO"
for i in $(seq 1 30); do
    ls -d /sys/kernel/coyote_versal_sysfs_* >/dev/null 2>&1 && ls /dev/coyote_versal_fpga_*_v0 >/dev/null 2>&1 && break
    sleep 1
done
ls -d /sys/kernel/coyote_versal_sysfs_* >/dev/null 2>&1 || { echo "driver loaded but no coyote_versal_sysfs node; see dmesg" >&2; exit 1; }
echo "   driver up: $(basename $(readlink -f $DEV/driver)), $(ls -d /sys/kernel/coyote_versal_sysfs_* | tr '\n' ' ')$(ls /dev/coyote_versal_fpga_* | tr '\n' ' ')"
