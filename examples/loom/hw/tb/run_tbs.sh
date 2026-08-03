#!/usr/bin/env bash
# Block-level testbench runner (XSIM). Vivado tools live behind xilinx-shell
# on this system; the script re-execs itself inside it if needed.
set -u
cd "$(dirname "$0")"

if ! command -v xvlog >/dev/null 2>&1; then
    exec xilinx-shell "$0" "$@"
fi

COYOTE_ROOT=../../../..
LYNX_PKG=../build_sim/sim/lynx_pkg.sv
AXI_INTF=$COYOTE_ROOT/hw/hdl/pkg/axi_intf.sv

if [ ! -f "$LYNX_PKG" ]; then
    echo "ERROR: $LYNX_PKG not found."
    echo "Generate it once with:"
    echo "  cd ../ && mkdir -p build_sim && cd build_sim"
    echo "  xilinx-shell -c \"nix-shell -p cmake --run 'cmake .. -DFDEV_NAME=u280'\""
    echo "  mkdir -p sim && nix-shell -p 'python3.withPackages(ps: [ps.jinja2])' --run 'python3 write_hdl.py 3 0 0'"
    exit 1
fi

TBS="tb_loom_table tb_loom_ctrl tb_loom_engine tb_loom_rx tb_loom_top"
SRCS="$LYNX_PKG $AXI_INTF $COYOTE_ROOT/hw/hdl/pkg/lynx_intf.sv \
      ../src/hdl/loom_table.sv ../src/hdl/loom_ctrl.sv \
      ../src/hdl/loom_engine.sv ../src/hdl/loom_rx.sv \
      ../build_sim/sim/user_logic_c0_0.sv"

mkdir -p work && cd work

echo "== xvlog =="
xvlog -sv $(for f in $SRCS; do echo ../$f; done) \
    -i ../../src -i ../$COYOTE_ROOT/hw/hdl/pkg \
    ../tb_loom_table.sv ../tb_loom_ctrl.sv ../tb_loom_engine.sv \
    ../tb_loom_rx.sv ../tb_loom_top.sv \
    > xvlog.log 2>&1 || { tail -30 xvlog.log; echo "COMPILE FAILED"; exit 1; }

fail=0
for tb in $TBS; do
    echo "== $tb =="
    xelab -debug typical "$tb" -s "${tb}_sim" > "xelab_${tb}.log" 2>&1 \
        || { tail -30 "xelab_${tb}.log"; echo "ELAB FAILED: $tb"; fail=1; continue; }
    xsim -R "${tb}_sim" > "xsim_${tb}.log" 2>&1
    if grep -q "TB PASS ($tb)" "xsim_${tb}.log"; then
        echo "PASS: $tb"
    else
        grep -E "FAIL|Error" "xsim_${tb}.log" | head -20
        echo "FAIL: $tb (see hw/tb/work/xsim_${tb}.log)"
        fail=1
    fi
done

exit $fail
