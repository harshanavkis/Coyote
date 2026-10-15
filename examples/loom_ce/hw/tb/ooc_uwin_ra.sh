#!/usr/bin/env bash
# Out-of-context place and route of uwin_ra alone on the V80 part at 333 MHz
# (xclk, 3.0 ns), Vivado 2025.1 as the V80 build uses: post-route WNS of its
# internal paths (ports unconstrained) and utilization. ~15-30 min. Not a
# substitute for the build. Needs ../build_sim/sim/lynx_pkg.sv (as run_tbs.sh).
# Reports in work_ooc/.
set -euo pipefail
cd "$(dirname "$0")"
if [ -z "${OOC_IN_SHELL:-}" ]; then
    NCURSES=$(nix-build '<nixpkgs>' -A ncurses --no-out-link 2>/dev/null | tail -1)
    exec xilinx-shell -c "unset XILINX_VIVADO XILINX_HLS XILINX_VITIS XILINX_SDX XILINX_PATH MYVIVADO MYXILINX RDI_BINROOT RDI_APPROOT RDI_BASEROOT RDI_INSTALLROOT RDI_INSTALLVER RDI_BASELINE RDI_PATCHROOT RDI_SHARED_DATA HDI_APPROOT _RDI_SETENV_RUN; \
PATH=\$(printf '%s' \"\$PATH\" | tr ':' '\n' | awk '\$0 !~ \"^/share/xilinx/\"' | paste -sd: -); export PATH; \
source /share/xilinx/2025.1/Vivado/.settings64-Vivado.sh >/dev/null 2>&1; export XILINX_LOCAL_USER_DATA=no; \
export LD_LIBRARY_PATH=$NCURSES/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}; OOC_IN_SHELL=1 $PWD/$(basename "$0")"
fi
C=$(realpath ../../../..)
mkdir -p work_ooc && cd work_ooc
cat > ooc.tcl <<TCL
read_verilog -sv $C/examples/loom_ce/hw/build_sim/sim/lynx_pkg.sv
read_verilog -sv $C/hw/hdl/pkg/axi_intf.sv
read_verilog -sv $C/hw/hdl/common/uwin/uwin_ra.sv
read_verilog -sv $C/examples/loom_ce/hw/tb/ooc_uwin_ra_top.sv
synth_design -mode out_of_context -top ooc_uwin_ra_top -part xcv80-lsva4737-2MHP-e-S -include_dirs $C/hw/hdl/pkg
create_clock -period 3.000 -name xclk [get_ports xclk]
opt_design
place_design
phys_opt_design
route_design
report_timing_summary -max_paths 10 -file timing.rpt
report_utilization -hierarchical -hierarchical_depth 2 -file util.rpt
report_timing -max_paths 5 -nworst 1 -file worst.rpt
TCL
vivado -mode batch -nojournal -log ooc.log -source ooc.tcl > /dev/null 2>&1 || { tail -20 ooc.log; echo "OOC FAILED"; exit 1; }
grep -A4 -E 'Design Timing Summary' timing.rpt | tail -2
grep -E '^\| (Total|Slice LUTs|LUT as Logic|Registers|Block RAM)' util.rpt | head -5 || true
echo "OOC done: $(pwd)"
