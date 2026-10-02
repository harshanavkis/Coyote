#!/usr/bin/env bash
# Block-level testbench runner (XSIM), as examples/loom/hw/tb/run_tbs.sh.
# Vivado tools live behind xilinx-shell; the script re-execs itself inside it.
set -u
cd "$(dirname "$0")"

if ! command -v xvlog >/dev/null 2>&1; then
    exec xilinx-shell "$(pwd)/$(basename "$0")" "$@"
fi

COYOTE_ROOT=../../../..
LOOM_TB=$COYOTE_ROOT/examples/loom/hw/tb
LYNX_PKG=../build_sim/sim/lynx_pkg.sv
USER_LOGIC=../build_sim/sim/user_logic_c0_0.sv

if [ ! -f "$LYNX_PKG" ]; then
    echo "ERROR: $LYNX_PKG not found."
    echo "Generate it once with:"
    echo "  cd ../ && mkdir -p build_sim && cd build_sim"
    echo "  xilinx-shell -c \"nix-shell -p cmake --run 'cmake .. -DFDEV_NAME=u280'\""
    echo "  mkdir -p sim && nix-shell -p 'python3.withPackages(ps: [ps.jinja2])' --run 'python3 write_hdl.py 3 0 0'"
    exit 1
fi

TBS="${TBS:-tb_loom_ingress tb_loom_switch_top}"
# The shell primitives vfpga_top uses have behavioural stand-ins in
# examples/loom's tb directory (the register slice and the rx FIFO IPs)
SRCS="$LYNX_PKG $COYOTE_ROOT/hw/hdl/pkg/axi_intf.sv $COYOTE_ROOT/hw/hdl/pkg/lynx_intf.sv \
      ../src/hdl/loom_table.sv ../src/hdl/loom_ingress.sv ../src/hdl/loom_ctrl.sv ../src/hdl/loom_rx.sv ../src/hdl/loom_exports.sv \
      $LOOM_TB/sim_axisr_register_slice_512.sv $LOOM_TB/sim_axis_data_fifo_512.sv sim_axis_data_fifo_rx4096.sv \
      $COYOTE_ROOT/hw/hdl/common/regs/axisr_reg.sv $USER_LOGIC"

mkdir -p work && cd work

# The data FIFO is an XPM primitive: xelab links it from the precompiled xpm
# library, which needs glbl.
GLBL=/share/xilinx/Vivado/2023.2/data/verilog/src/glbl.v

echo "== xvlog =="
xvlog $GLBL > xvlog_glbl.log 2>&1 || { tail -5 xvlog_glbl.log; echo "COMPILE FAILED (glbl)"; exit 1; }
xvlog -sv $(for f in $SRCS; do echo ../$f; done) -i ../../src -i ../$COYOTE_ROOT/hw/hdl/pkg \
    ../tb_loom_ingress.sv ../tb_loom_switch_top.sv \
    > xvlog.log 2>&1 || { tail -30 xvlog.log; echo "COMPILE FAILED"; exit 1; }

fail=0
for tb in $TBS; do
    echo "== $tb =="
    xelab -debug typical -L xpm "$tb" glbl -s "${tb}_sim" > "xelab_${tb}.log" 2>&1 \
        || { tail -30 "xelab_${tb}.log"; echo "ELAB FAILED: $tb"; fail=1; continue; }
    xsim -R "${tb}_sim" > "xsim_${tb}.log" 2>&1
    if grep -q "TB PASS ($tb)" "xsim_${tb}.log"; then
        grep -E '^(ok|FAIL) ' "xsim_${tb}.log"
        echo "PASS: $tb"
    else
        grep -E "FAIL|Error" "xsim_${tb}.log" | head -30
        echo "FAIL: $tb (see hw/tb/work/xsim_${tb}.log)"
        fail=1
    fi
done

exit $fail
