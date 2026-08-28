#!/usr/bin/env bash
# Smoke test for the minus-nw top: compile and elaborate it against the same
# interfaces the shell presents, and run long enough to trip the interface
# assertions. It does NOT exercise the data plane - tb_loom_loopback in
# examples/loom already covers engine -> wire format -> loom_rx, and this top
# uses the same modules.
set -e
[ -z "$XILINX_VIVADO" ] && exec xilinx-shell -c "$0 $*"
HERE=$(cd "$(dirname "$0")" && pwd)
C=$HERE/../../../..
L=$C/examples/loom
mkdir -p "$HERE/work" && cd "$HERE/work"
xvlog -sv -i "$HERE/../src" \
  $L/hw/build_sim/sim/lynx_pkg.sv $C/hw/hdl/pkg/axi_intf.sv $C/hw/hdl/pkg/lynx_intf.sv \
  $L/hw/src/hdl/loom_table.sv $L/hw/src/hdl/loom_ctrl.sv \
  $L/hw/src/hdl/loom_engine.sv $L/hw/src/hdl/loom_rx.sv \
  "$HERE/../src/hdl/loom_mnw_pktzr.sv" \
  $L/hw/tb/sim_axisr_register_slice_512.sv $C/hw/hdl/common/regs/axisr_reg.sv \
  "$HERE/tb_mnw_elab.sv" > xvlog.log 2>&1
xelab -debug typical tb_mnw_elab -s mnw_snap > xelab.log 2>&1
xsim mnw_snap -runall > xsim.log 2>&1
grep -q 'MNW ELABORATION OK' xsim.log && echo "PASS: tb_mnw_elab" \
  || { echo "FAIL: tb_mnw_elab (see hw/tb/work/xsim.log)"; exit 1; }
