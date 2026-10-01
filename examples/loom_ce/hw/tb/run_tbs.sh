#!/usr/bin/env bash
# Block-level testbench runner (XSIM), as examples/loom_switch/hw/tb/run_tbs.sh.
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
    echo "  xilinx-shell -c \"nix-shell -p cmake --run 'cmake .. -DFDEV_NAME=v80'\""
    echo "  mkdir -p sim && xilinx-shell -c 'python3 write_hdl.py 3 0 0'"
    exit 1
fi

TBS="${TBS:-tb_loom_ce tb_uwin_hbm}"
# axisr_reg's register slice has a behavioural stand-in in examples/loom's tb
SRCS="$LYNX_PKG $COYOTE_ROOT/hw/hdl/pkg/axi_intf.sv $COYOTE_ROOT/hw/hdl/pkg/lynx_intf.sv \
      ../src/hdl/loom_ce_ctrl.sv ../src/hdl/loom_ce.sv \
      $LOOM_TB/sim_axisr_register_slice_512.sv \
      $COYOTE_ROOT/hw/hdl/common/regs/axisr_reg.sv $USER_LOGIC \
      $COYOTE_ROOT/hw/hdl/common/regs/axi_reg_array.sv $COYOTE_ROOT/hw/hdl/common/queues/fifo.sv \
      $COYOTE_ROOT/hw/hdl/common/queues/queue_meta.sv $COYOTE_ROOT/hw/hdl/stripe/axi_stripe_rd.sv \
      $COYOTE_ROOT/hw/hdl/stripe/axi_stripe_wr.sv $COYOTE_ROOT/hw/hdl/stripe/axi_stripe.sv \
      $COYOTE_ROOT/hw/hdl/common/uwin/uwin_hbm.sv $COYOTE_ROOT/hw/hdl/common/uwin/uwin_mon.sv"

mkdir -p work && cd work

echo "== xvlog =="
xvlog -sv $(for f in $SRCS; do echo ../$f; done) -i ../../src -i ../$COYOTE_ROOT/hw/hdl/pkg \
    ../tb_loom_ce.sv ../tb_uwin_hbm.sv > xvlog.log 2>&1 || { grep -E 'ERROR' xvlog.log | head -20; echo "COMPILE FAILED"; exit 1; }

fail=0
for tb in $TBS; do
    echo "== $tb =="
    xelab -debug typical "$tb" -s "${tb}_sim" > "xelab_${tb}.log" 2>&1 \
        || { grep -E 'ERROR' "xelab_${tb}.log" | head -20; echo "ELAB FAILED: $tb"; fail=1; continue; }
    xsim -R "${tb}_sim" > "xsim_${tb}.log" 2>&1
    if grep -q "TB PASS ($tb)" "xsim_${tb}.log"; then
        grep -E '^(ok|FAIL) ' "xsim_${tb}.log"
        echo "PASS: $tb"
    else
        grep -E "FAIL|Error|Fatal" "xsim_${tb}.log" | head -30
        echo "FAIL: $tb (see hw/tb/work/xsim_${tb}.log)"
        fail=1
    fi
done

exit $fail
