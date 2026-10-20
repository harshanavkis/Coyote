#!/usr/bin/env bash
# Give PTDMA functions to ptdma_q (claim) or back to the stock ptdma driver (release), on THIS host.
#   ptq_claim.sh claim   <bdf>...   loads ptdma_q (uwin_bdf = this host's U280) if needed -> /dev/ptq_<bdf>
#   ptq_claim.sh release <bdf>...   and unloads ptdma_q once it has no function left
# Module: ~/loom-experiments/ptdma_q/ptdma_q.ko.k$(uname -r) (built from Coyote examples/loom_switch/ptdma).
# Every sysfs write runs under a timeout: an unbind that hangs is reported, not waited for.
set -u
OP=$1; shift
KO=$HOME/loom-experiments/ptdma_q/ptdma_q.ko.k$(uname -r)
U280_BDF=0000:e1:00.0
w() { timeout 20 sudo sh -c "echo '$1' > '$2'" || { echo "PTQ CLAIM FAILED: echo $1 > $2"; exit 1; }; }
case $OP in
claim)
    if ! lsmod | grep -q '^ptdma_q '; then
        [ -f "$KO" ] || { echo "PTQ CLAIM FAILED: no $KO"; exit 1; }
        timeout 30 sudo insmod "$KO" uwin_bdf=$U280_BDF || { echo "PTQ CLAIM FAILED: insmod"; exit 1; }
    fi
    for b in "$@"; do
        D=/sys/bus/pci/devices/$b
        w ptdma_q $D/driver_override
        [ -e $D/driver ] && [ "$(basename $(readlink $D/driver))" = ptdma ] && w $b /sys/bus/pci/drivers/ptdma/unbind
        w $b /sys/bus/pci/drivers_probe
        dev=/dev/ptq_$(echo $b | tr ':.' '__')
        for i in $(seq 50); do [ -e $dev ] && break; sleep 0.1; done
        [ -e $dev ] || { echo "PTQ CLAIM FAILED: no $dev"; exit 1; }
        echo "claimed $b -> $dev"
    done ;;
release)
    for b in "$@"; do
        D=/sys/bus/pci/devices/$b
        w '' $D/driver_override
        [ -e $D/driver ] && [ "$(basename $(readlink $D/driver))" = ptdma_q ] && w $b /sys/bus/pci/drivers/ptdma_q/unbind
        w $b /sys/bus/pci/drivers_probe
        echo "released $b -> $(basename $(readlink $D/driver 2>/dev/null) 2>/dev/null)"
    done
    if lsmod | grep -q '^ptdma_q ' && [ -z "$(ls /sys/bus/pci/drivers/ptdma_q 2>/dev/null | grep '^0000:')" ]; then
        timeout 30 sudo rmmod ptdma_q || { echo "PTQ RELEASE FAILED: rmmod"; exit 1; }
    fi ;;
*) echo "usage: $0 claim|release <bdf>..."; exit 2 ;;
esac
