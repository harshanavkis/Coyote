#!/usr/bin/env bash
# Start a Loom build in tmux with the standard arguments (build_bitstream.sh
# underneath: provenance in BUILD_INFO.txt, tmux session bit_<build_name>).
#
#   scripts/fpga/build_loom.sh switch <build_name> [cmake args...]   # U280, examples/loom_switch
#   scripts/fpga/build_loom.sh ce     <build_name> [cmake args...]   # V80,  examples/loom_ce
#   scripts/fpga/build_loom.sh rdma   <build_name> [cmake args...]   # U280, examples/09_perf_rdma (the RDMA baseline)
#
# Extra cmake args come after the standard ones, so they override them.
# Then: scripts/fpga/wait_build.sh <switch|ce|rdma> <build_name> [stage_name]
set -euo pipefail
REPO=$(realpath "$(dirname "$0")/../..")
[ $# -ge 2 ] || { sed -n '2,9p' "$0"; exit 2; }
WHAT=$1 NAME=$2; shift 2

case $WHAT in
    # Both U280 images link into the QDMA static of loom-switch-qdma64 (the
    # standard images since 2026-10-15; examples/loom_switch/hw/static is the
    # older XDMA static)
    switch) DEV=u280; HW=examples/loom_switch/hw
            STD=(-DACK_GAP_CYCLES=16 -DBUILD_SHELL=1 -DBUILD_STATIC=0
                 -DSTATIC_PATH=/home/harshanavkis/coyote-bitstreams/loom-switch-qdma64/static -DSTATIC_DMA=qdma) ;;
    rdma)   DEV=u280; HW=examples/09_perf_rdma/hw
            STD=(-DACK_GAP_CYCLES=16 -DBUILD_SHELL=1 -DBUILD_STATIC=0
                 -DSTATIC_PATH=/home/harshanavkis/coyote-bitstreams/loom-switch-qdma64/static -DSTATIC_DMA=qdma) ;;
    ce)     DEV=v80;  HW=examples/loom_ce/hw
            STD=(-DBUILD_SHELL=1 -DBUILD_STATIC=0
                 -DSTATIC_PATH=/home/harshanavkis/coyote-bitstreams/loom-ce-static_ot16/static -DBUILD_OPT=1) ;;
    *) echo "first argument: switch, ce or rdma" >&2; exit 2 ;;
esac

[ -z "$(git -C "$REPO" status --short --untracked-files=no)" ] \
    || echo "WARNING: uncommitted changes; the image will not match HEAD (BUILD_INFO lists them)" >&2
exec "$REPO/scripts/fpga/build_bitstream.sh" "$DEV" "$REPO/$HW" "$NAME" "${STD[@]}" "$@"
