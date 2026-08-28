#!/usr/bin/env bash
# Drives the minus-nw top end to end THROUGH its arbiter, which is the one
# thing tb_loom_loopback cannot do (it ties loom_rx's grant high).
set -e
[ -z "$XILINX_VIVADO" ] && exec xilinx-shell -c "$0 $*"
HERE=$(cd "$(dirname "$0")" && pwd)
C=$HERE/../../../..
L=$C/examples/loom
mkdir -p "$HERE/work" && cd "$HERE/work"
xvlog -sv -i "$HERE/../src" \
  $L/hw/build_sim/sim/lynx_pkg.sv $C/hw/hdl/pkg/axi_intf.sv $C/hw/hdl/pkg/lynx_intf.sv \
  "$HERE/../src/hdl/loom_table.sv" "$HERE/../src/hdl/loom_ctrl.sv" \
  "$HERE/../src/hdl/loom_engine.sv" "$HERE/../src/hdl/loom_rx.sv" \
  "$HERE/../src/hdl/loom_mnw_pktzr.sv" \
  $L/hw/tb/sim_axisr_register_slice_512.sv $C/hw/hdl/common/regs/axisr_reg.sv \
  "$HERE/tb_loom_mnw.sv" > xvlog_mnw.log 2>&1
xelab -debug typical tb_loom_mnw -s mnw_tb_snap > xelab_mnw.log 2>&1
xsim mnw_tb_snap -runall > xsim_mnw.log 2>&1
grep -E 'PASS:|FAIL:|deadlocked|first bad' xsim_mnw.log || true
grep -q 'TB PASS' xsim_mnw.log && echo "PASS: tb_loom_mnw" \
  || { echo "FAIL: tb_loom_mnw (see hw/tb/work/xsim_mnw.log)"; exit 1; }
