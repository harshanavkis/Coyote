#!/usr/bin/env bash
# Build a Coyote bitstream for one FPGA, reproducibly.
#
#   scripts/fpga/build_bitstream.sh <device> <hw_dir> <build_name> [--fg] [cmake args...]
#
#   device      u280 | u55c | u250 | v80
#   hw_dir      the example's hw directory, e.g. examples/loom/hw
#   build_name  build directory created inside hw_dir, e.g. build_sep29_ctrl
#   --fg        run in the foreground instead of a tmux session
#   cmake args  passed through, e.g. -DEN_MEM=1
#
# The Vivado version follows the device: UltraScale+ uses 2023.2 (Coyote's
# U280/U55C/U250 static checkpoints are Vivado 2022.1 files, which 2025.1
# cannot open); the V80 uses 2025.1 (its checkpoint is 2024.2, and Coyote
# requires >= 2024.2). Output: <hw_dir>/<build_name>/bitstreams/cyt_top.bit
# (UltraScale+) or cyt_top.pdi (V80), log in bitgen.log, provenance in
# BUILD_INFO.txt. The tmux session is bit_<build_name>.
set -euo pipefail

usage() { sed -n '2,20p' "$0" >&2; exit 1; }
[ $# -ge 3 ] || usage
DEV=$1; HW=$2; NAME=$3; shift 3
FG=0
if [ "${1:-}" = "--fg" ]; then FG=1; shift; fi
CMAKE_ARGS=("$@")

REPO=$(git -C "$(dirname "$0")" rev-parse --show-toplevel)
HW=$(cd "$HW" && pwd)
BUILD=$HW/$NAME

case $DEV in
    u280|u55c|u250) VIVADO=2023.2 ;;
    v80)            VIVADO=2025.1 ;;
    *) echo "unknown device '$DEV'" >&2; exit 1 ;;
esac

# Tools that are in the nix store, not on the xilinx-shell PATH
CMAKE_BIN=$(nix-shell -p cmake --run 'dirname $(command -v cmake)' 2>/dev/null | tail -1)
[ -x "$CMAKE_BIN/cmake" ] || { echo "could not resolve cmake via nix" >&2; exit 1; }

# The environment each Vivado version needs inside xilinx-shell. xilinx-shell
# sets up 2023.2; 2025.1 needs that environment cleared, its own settings
# sourced (Vivado and Vitis, for vitis-run), and ncurses 6 (libtinfo.so.6).
if [ "$VIVADO" = 2025.1 ]; then
    NCURSES=$(nix-build '<nixpkgs>' -A ncurses --no-out-link 2>/dev/null | tail -1)
    [ -e "$NCURSES/lib/libtinfo.so.6" ] || { echo "could not resolve ncurses 6 via nix" >&2; exit 1; }
    ENV_SETUP="unset XILINX_VIVADO XILINX_HLS XILINX_VITIS XILINX_SDX XILINX_PATH MYVIVADO MYXILINX RDI_BINROOT RDI_APPROOT RDI_BASEROOT RDI_INSTALLROOT RDI_INSTALLVER RDI_BASELINE RDI_PATCHROOT RDI_SHARED_DATA HDI_APPROOT _RDI_SETENV_RUN; \
PATH=\$(printf '%s' \"\$PATH\" | tr ':' '\n' | awk '\$0 !~ \"^/share/xilinx/\"' | paste -sd: -); export PATH; \
source /share/xilinx/2025.1/Vivado/.settings64-Vivado.sh >/dev/null 2>&1; \
source /share/xilinx/2025.1/Vitis/.settings64-Vitis.sh >/dev/null 2>&1; \
export XILINX_LOCAL_USER_DATA=no; \
export LD_LIBRARY_PATH=$NCURSES/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH};"
else
    ENV_SETUP=""
fi
ENV_SETUP="$ENV_SETUP export TERM=\${TERM:-xterm}; export PATH=$CMAKE_BIN:\$PATH;"

run_xs() { xilinx-shell -c "$ENV_SETUP $1"; }

GOT=$(run_xs "vivado -version | head -1" 2>/dev/null | grep -o 'v20[0-9.]*' | head -1)
[ "$GOT" = "v$VIVADO" ] || { echo "expected Vivado $VIVADO, got '$GOT'" >&2; exit 1; }

[ -e "$BUILD" ] && { echo "$BUILD exists; pick a new build_name" >&2; exit 1; }
mkdir -p "$BUILD"

# Provenance: what this bitstream was built from
{
    echo "built:      $(date -Is) on $(hostname)"
    echo "device:     $DEV"
    echo "vivado:     $VIVADO"
    echo "hw_dir:     ${HW#$REPO/}"
    echo "cmake args: -DFDEV_NAME=$DEV ${CMAKE_ARGS[*]:-}"
    echo "commit:     $(git -C "$REPO" rev-parse HEAD) ($(git -C "$REPO" branch --show-current))"
    echo "network:    $(git -C "$REPO" ls-tree HEAD hw/services/network | awk '{print $3}') (pinned), $(git -C "$REPO/hw/services/network" rev-parse HEAD) (checked out)"
    echo "dirty files:"
    git -C "$REPO" status --short --untracked-files=no | sed 's/^/  /'
} > "$BUILD/BUILD_INFO.txt"

cd "$BUILD"
run_xs "cmake .. -DFDEV_NAME=$DEV ${CMAKE_ARGS[*]:-}" > cmake.log 2>&1 \
    || { echo "cmake failed, see $BUILD/cmake.log" >&2; exit 1; }

STEPS="cd $BUILD && make project && make bitgen"
if [ $FG = 1 ]; then
    run_xs "$STEPS" 2>&1 | tee bitgen.log
else
    SESSION=bit_$NAME
    # The session re-enters xilinx-shell with the same environment
    printf '%s\n' "$ENV_SETUP $STEPS" > "$BUILD/.build_cmd.sh"
    tmux new-session -d -s "$SESSION" "xilinx-shell -c 'bash $BUILD/.build_cmd.sh' > $BUILD/bitgen.log 2>&1"
    echo "started tmux session $SESSION"
    echo "  log:  $BUILD/bitgen.log"
    echo "  info: $BUILD/BUILD_INFO.txt"
fi
