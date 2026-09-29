#!/usr/bin/env bash
# Block-level testbench runner (XSIM), as examples/loom/hw/tb/run_tbs.sh.
# Vivado tools live behind xilinx-shell; the script re-execs itself inside it.
#
# Until loom_switch has its own build_sim, lynx_pkg comes from examples/loom's
# (same shell parameters for everything these blocks use).
set -u
cd "$(dirname "$0")"

if ! command -v xvlog >/dev/null 2>&1; then
    exec xilinx-shell "$(pwd)/$(basename "$0")" "$@"
fi

COYOTE_ROOT=../../../..
LYNX_PKG=$COYOTE_ROOT/examples/loom/hw/build_sim/sim/lynx_pkg.sv
AXI_INTF=$COYOTE_ROOT/hw/hdl/pkg/axi_intf.sv

if [ ! -f "$LYNX_PKG" ]; then
    echo "ERROR: $LYNX_PKG not found; generate it as examples/loom/hw/tb/run_tbs.sh says."
    exit 1
fi

TBS="${TBS:-tb_loom_ingress}"
SRCS="$LYNX_PKG $AXI_INTF ../src/hdl/loom_table.sv ../src/hdl/loom_ingress.sv"

mkdir -p work && cd work

# The data FIFO is an XPM primitive: xelab links it from the precompiled xpm
# library, which needs glbl.
GLBL=/share/xilinx/Vivado/2023.2/data/verilog/src/glbl.v

echo "== xvlog =="
xvlog $GLBL > xvlog_glbl.log 2>&1 || { tail -5 xvlog_glbl.log; echo "COMPILE FAILED (glbl)"; exit 1; }
xvlog -sv $(for f in $SRCS; do echo ../$f; done) ../tb_loom_ingress.sv \
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
