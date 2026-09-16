#!/usr/bin/env bash
# Out-of-context synthesis of one Loom module: a 4-minute check that the
# RTL synthesizes (XPM primitives, latches, widths - the errors a 4.5 h
# build would otherwise find late), what it costs, and whether it plausibly
# makes 250 MHz. Not a substitute for the full build: no shell, no routing.
#
#   ./ooc_synth.sh              # loom_engine
#   ./ooc_synth.sh loom_rx      # any of loom_engine loom_rx loom_ctrl loom_table
#   ./ooc_synth.sh top          # the whole vFPGA (vfpga_top.svh + every
#                               # module, under ooc_top.sv which types the
#                               # interfaces; the ingress FIFO IP as a
#                               # black box from its stub)
#
# Needs the generated lynx_pkg.sv from build_sim/sim (setup_sim.sh, step 2)
# and Vivado (re-execs itself inside xilinx-shell). Reports land in
# build_ooc/<module>/: utilization (hierarchical, one level) and a timing
# estimate at 250 MHz (post-synthesis, so an estimate).
set -euo pipefail
cd "$(dirname "$0")"
if ! command -v vivado >/dev/null 2>&1; then
    exec xilinx-shell "$0" "$@"
fi

MOD=${1:-loom_engine}
COYOTE=$(realpath ../../..)
PKG=build_sim/sim/lynx_pkg.sv
[ -f "$PKG" ] || { echo "ERROR: $PKG missing - run ./setup_sim.sh first (steps 1-2 render it)"; exit 1; }
OUT=build_ooc/$MOD
mkdir -p "$OUT"

# Sources: the package and interfaces, then the module and anything it
# instantiates (loom_engine stands alone; the others too). 'top' is the
# generated wrapper around vfpga_top.svh with all four modules, the
# shell's register slice and the FIFO IP's stub (a black box: its
# internals are the IP's, only the glue around it is under test here).
if [ "$MOD" = top ]; then
    TOP=ooc_top
    IPDIR=$(ls -d build_*/example_loom_config_0/user_c0_0/example_loom.gen/sources_1/ip 2>/dev/null | tail -1)
    STUB=$IPDIR/axis_data_fifo_512/axis_data_fifo_512_stub.v
    STUB2=$IPDIR/axisr_register_slice_512/axisr_register_slice_512_stub.v
    [ -f "$STUB" ] && [ -f "$STUB2" ] || { echo "ERROR: IP stubs missing under build_*/.../sources_1/ip - a full build's IP output is needed for the black boxes"; exit 1; }
    SRC_LINES="read_verilog -sv $PWD/src/hdl/loom_table.sv
read_verilog -sv $PWD/src/hdl/loom_ctrl.sv
read_verilog -sv $PWD/src/hdl/loom_engine.sv
read_verilog -sv $PWD/src/hdl/loom_rx.sv
read_verilog -sv $COYOTE/hw/hdl/common/regs/axisr_reg.sv
read_verilog $PWD/$STUB
read_verilog $PWD/$STUB2
read_verilog -sv $PWD/build_sim/sim/user_logic_c0_0.sv
read_verilog -sv $PWD/ooc_top.sv"
    INC="-include_dirs $COYOTE/hw/hdl/pkg -include_dirs $PWD/src"
else
    TOP=$MOD
    SRC_LINES="read_verilog -sv $PWD/src/hdl/$MOD.sv"
    INC="-include_dirs $COYOTE/hw/hdl/pkg"
fi
cat > "$OUT/ooc.tcl" <<TCL
read_verilog -sv $PWD/$PKG
read_verilog -sv $COYOTE/hw/hdl/pkg/axi_intf.sv
read_verilog -sv $COYOTE/hw/hdl/pkg/lynx_intf.sv
$SRC_LINES
synth_design -top $TOP -part xcu280-fsvh2892-2L-e -mode out_of_context $INC
create_clock -period 4.000 -name aclk [get_ports aclk]
report_utilization -hierarchical -hierarchical_depth 1 -file $PWD/$OUT/util.rpt
report_timing_summary -max_paths 3 -file $PWD/$OUT/timing.rpt
TCL

echo "== OOC synthesis of $MOD (reports in $OUT/)"
if ! vivado -mode batch -nolog -nojournal -notrace -source "$OUT/ooc.tcl" > "$OUT/vivado.log" 2>&1; then
    echo "FAILED - see $OUT/vivado.log"; grep -E '^ERROR' "$OUT/vivado.log" | head; exit 1
fi
if grep -qE '^ERROR|CRITICAL WARNING' "$OUT/vivado.log"; then
    echo "-- errors / critical warnings:"; grep -E '^ERROR|CRITICAL WARNING' "$OUT/vivado.log" | head
fi
echo "-- utilization (top and one level down):"
grep -E '^\| +(Instance|'"$TOP"' |  +inst_)' "$OUT/util.rpt" | sed 's/ \+/ /g' | cut -c1-140
echo "-- timing estimate at 250 MHz (post-synthesis, WNS >= 0 means it fits):"
grep -A3 'WNS(ns)' "$OUT/timing.rpt" | sed -n '1,4p'
