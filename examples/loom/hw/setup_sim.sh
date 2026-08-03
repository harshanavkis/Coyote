#!/usr/bin/env bash
# One-time simulation setup for the loom example (NixOS testbed).
#
# Does four things, in order:
#   1. CMake-configures the hw build (Vivado from xilinx-shell, modern
#      cmake from nix - Vivado's bundled cmake 3.3 is too old).
#   2. Renders the generated RTL (lynx_pkg.sv, user_logic wrapper) that
#      the block TBs and the XSIM project compile against.
#   3. Builds the DPI library (coyote_sim.so) with the SYSTEM toolchain.
#      This replaces Coyote's `make sim` DPI step (xsc), which fails on
#      this host: Vivado 2023.2 ships binutils 2.37, whose ld cannot link
#      against NixOS glibc 2.42 ("unknown type [0x13] section `.relr.dyn'",
#      "skipping incompatible libmvec"). XSIM only dlopens the .so at
#      runtime, so a system-gcc build works fine.
#   4. Creates the XSIM project (cr_sim.tcl) in Vivado batch mode.
#
# Idempotent: safe to re-run; step 4 recreates the project.
set -euo pipefail
cd "$(dirname "$0")"

VIVADO_INC=/share/xilinx/Vivado/2023.2/data/xsim/include
COYOTE=$(realpath ../../..)

mkdir -p build_sim
cd build_sim

echo "== [1/4] cmake configure =="
xilinx-shell -c "nix-shell -p cmake --run 'cmake .. -DFDEV_NAME=u280'"

echo "== [2/4] render generated RTL =="
mkdir -p sim
nix-shell -p 'python3.withPackages(ps: [ps.jinja2])' \
    --run 'python3 write_hdl.py 3 0 0'

echo "== [3/4] DPI library (system gcc; replaces the broken xsc step) =="
nix-shell -p gcc --run \
    "gcc -shared -fPIC -I$VIVADO_INC $COYOTE/sim/hw/dpi/file_io.c -o sim/coyote_sim.so"

echo "== [4/4] XSIM project (vivado batch) =="
xilinx-shell -c "vivado -mode batch -source cr_sim.tcl" > cr_sim_run.log 2>&1
ls sim/*.xpr

echo "setup_sim.sh: done"
