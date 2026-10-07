#!/usr/bin/env bash
# Start a Loom build in tmux with the standard arguments (build_bitstream.sh
# underneath: provenance in BUILD_INFO.txt, tmux session bit_<build_name>).
#
#   scripts/fpga/build_loom.sh switch <build_name> [cmake args...]   # U280, examples/loom_switch
#   scripts/fpga/build_loom.sh ce     <build_name> [cmake args...]   # V80,  examples/loom_ce
#
# Extra cmake args come after the standard ones, so they override them.
# Then: scripts/fpga/wait_build.sh <switch|ce> <build_name> [stage_name]
set -euo pipefail
REPO=$(realpath "$(dirname "$0")/../..")
[ $# -ge 2 ] || { sed -n '2,9p' "$0"; exit 2; }
WHAT=$1 NAME=$2; shift 2

case $WHAT in
    # STATIC_PATH defaults to examples/loom_switch/hw/static (32 reads in
    # flight, timing closed)
    switch) DEV=u280; HW=examples/loom_switch/hw
            STD=(-DACK_GAP_CYCLES=16 -DBUILD_SHELL=1 -DBUILD_STATIC=0) ;;
    ce)     DEV=v80;  HW=examples/loom_ce/hw
            STD=(-DBUILD_SHELL=1 -DBUILD_STATIC=0
                 -DSTATIC_PATH=/home/harshanavkis/coyote-bitstreams/loom-ce-static_ot16/static -DBUILD_OPT=1) ;;
    *) echo "first argument: switch or ce" >&2; exit 2 ;;
esac

[ -z "$(git -C "$REPO" status --short --untracked-files=no)" ] \
    || echo "WARNING: uncommitted changes; the image will not match HEAD (BUILD_INFO lists them)" >&2
exec "$REPO/scripts/fpga/build_bitstream.sh" "$DEV" "$REPO/$HW" "$NAME" "${STD[@]}" "$@"
