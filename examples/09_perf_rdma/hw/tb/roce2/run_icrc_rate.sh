#!/usr/bin/env bash
# 2026-10-28: icrc.sv input cycles per packet pattern (icrc_rate_tb.sv). ICRC = which icrc (default the checkout's).
#   ./run_icrc_rate.sh "LEN_A=4156 LEN_B=48" ...   (one xsim run per quoted parameter set)
set -u
cd "$(dirname "$0")"
if ! command -v xvlog >/dev/null 2>&1; then exec xilinx-shell "$(pwd)/$(basename "$0")" "$@"; fi
C=$(cd ../../../../.. && pwd); H=$C/hw/hdl
ICRC=${ICRC:-$H/network/rdma/icrc.sv}
W=${WORK:-work_icrc}; mkdir -p $W && cd $W
xvlog -sv -d SYNTHESIS ../pkg/lynx_pkg_w32.sv $H/pkg/axi_intf.sv $H/network/rdma/buffer_fifo.sv $ICRC ../icrc_rate_tb.sv -i $H/pkg -i $H > xvlog.log 2>&1 || { grep ERROR xvlog.log | head; exit 1; }
for ps in "$@"; do
    GEN=""; for kv in $ps; do GEN="$GEN -generic_top $kv"; done
    xelab -timescale 1ns/1ps $GEN icrc_rate_tb -s r > xelab.log 2>&1 || { grep ERROR xelab.log | head; exit 1; }
    xsim r -R 2>&1 | grep -E "ICRC RATE|Error|FATAL"
done
