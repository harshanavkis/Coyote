#!/usr/bin/env bash
# Resume a build that died (host reset, killed session) in its own directory:
# `make bitgen` again, which reuses every step already finished. Never delete
# a dead build to start over. Runs in tmux session bit_<build_name>, appending
# to the build's bitgen.log, like build_bitstream.sh.
#
#   scripts/fpga/resume_build.sh <device> <build_dir> [--fg]
#
#   device     u280 | u55c | u250 | v80 (picks the Vivado version)
#   build_dir  the build directory build_bitstream.sh created
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/vivado_env.sh"
[ $# -ge 2 ] || { sed -n '2,10p' "$0" >&2; exit 1; }
DEV=$1; BUILD=$(readlink -f "$2"); FG=0; [ "${3:-}" = --fg ] && FG=1
[ -f "$BUILD/Makefile" ] || { echo "$BUILD has no Makefile: not a configured build directory" >&2; exit 1; }
SESSION=bit_$(basename "$BUILD")
CMAKE_BIN=$(nix-shell -p cmake --run 'dirname $(command -v cmake)' 2>/dev/null | tail -1)
CMD="$(vivado_env "$(vivado_version_for "$DEV")") export PATH=$CMAKE_BIN:\$PATH; cd $BUILD && make bitgen"
echo "--- resumed $(date -Is) on $(hostname)" >> "$BUILD/bitgen.log"
if [ $FG = 1 ]; then
    xilinx-shell -c "$CMD" 2>&1 | tee -a "$BUILD/bitgen.log"
else
    tmux has-session -t "$SESSION" 2>/dev/null && { echo "tmux session $SESSION exists: the build may still be running" >&2; exit 1; }
    printf '%s\n' "$CMD" > "$BUILD/.resume_cmd.sh"
    tmux new-session -d -s "$SESSION" "xilinx-shell -c 'bash $BUILD/.resume_cmd.sh' 2>&1 | tee -a $BUILD/bitgen.log"
    echo "resumed in tmux session $SESSION (tmux attach -t $SESSION); log $BUILD/bitgen.log"
fi
