#!/usr/bin/env bash
# 2026-10-28: icrc_fast.sv vs icrc_fixed.sv output equality (icrc_eq_tb.sv)
set -u
cd "$(dirname "$0")"
if ! command -v xvlog >/dev/null 2>&1; then exec xilinx-shell "$(pwd)/$(basename "$0")" "$@"; fi
C=$(cd ../../../../.. && pwd); H=$C/hw/hdl
W=work_icrc_eq; mkdir -p $W && cd $W
sed 's/^module icrc(/module icrc_fast(/' ../icrc_fast.sv > icrc_fast_mod.sv
xvlog -sv -d SYNTHESIS ../pkg/lynx_pkg_w32.sv $H/pkg/axi_intf.sv $H/network/rdma/buffer_fifo.sv ../icrc_fixed.sv icrc_fast_mod.sv ../icrc_eq_tb.sv -i $H/pkg -i $H > xvlog.log 2>&1 || { grep ERROR xvlog.log | head; exit 1; }
xelab -timescale 1ns/1ps ${GEN:-} icrc_eq_tb -s e > xelab.log 2>&1 || { grep ERROR xelab.log | head; exit 1; }
xsim e -R 2>&1 | grep -E "EQ |DBG|PEEK|Error|FATAL"
