#!/usr/bin/env bash
# Wait for a Loom build started by build_loom.sh, then report: the image and
# its md5, post-route timing, the first errors. With a stage name the image is
# copied where the hosts and setup_loom_hosts.sh take it from (never over an
# existing stage directory):
#   U280: ~/coyote-bitstreams/<stage_name>/hw/bitstreams/cyt_top.bit (+ .ltx)
#   V80:  ~/coyote-bitstreams/<stage_name>/cyt_top.pdi
#
#   scripts/fpga/wait_build.sh <switch|ce|rdma> <build_name> [stage_name]
#
# Exits 0 if the image exists. Run it in the background (it polls the tmux
# session once a minute).
set -uo pipefail
REPO=$(realpath "$(dirname "$0")/../..")
[ $# -ge 2 ] || { sed -n '2,12p' "$0"; exit 2; }
WHAT=$1 NAME=$2 STAGE=${3:-}
case $WHAT in
    switch) B=$REPO/examples/loom_switch/hw/$NAME; IMG=$B/bitstreams/cyt_top.bit ;;
    ce)     B=$REPO/examples/loom_ce/hw/$NAME;     IMG=$B/bitstreams/cyt_top.pdi ;;
    rdma)   B=$REPO/examples/09_perf_rdma/hw/$NAME; IMG=$B/bitstreams/cyt_top.bit ;;
    *) echo "first argument: switch, ce or rdma" >&2; exit 2 ;;
esac
[ -d "$B" ] || { echo "no build directory $B" >&2; exit 2; }

while tmux has-session -t "bit_$NAME" 2>/dev/null; do sleep 60; done

echo "build $NAME ended $(date -Is)"
grep -m1 '^commit' "$B/BUILD_INFO.txt"
grep 'Post Routing Timing Summary' "$B/bitgen.log" | tail -1 | sed 's/.*Summary | /post-route: /'
grep -E '^ERROR' "$B/bitgen.log" | head -5
if [ ! -f "$IMG" ]; then
    echo "NO IMAGE ($IMG); log tail:"; tail -5 "$B/bitgen.log"
    exit 1
fi
echo "image: $IMG md5 $(md5sum "$IMG" | cut -c1-8)"
if [ -n "$STAGE" ]; then
    if [ -e "$HOME/coyote-bitstreams/$STAGE" ]; then
        echo "NOT staged: ~/coyote-bitstreams/$STAGE exists (pick a new stage name)"; exit 0
    fi
    if [ "$WHAT" != ce ]; then
        D=$HOME/coyote-bitstreams/$STAGE/hw/bitstreams
        mkdir -p "$D" && cp "$IMG" "$B/bitstreams/cyt_top.ltx" "$D/" && echo "staged: $D/cyt_top.bit"
    else
        D=$HOME/coyote-bitstreams/$STAGE
        mkdir -p "$D" && cp "$IMG" "$D/" && echo "staged: $D/cyt_top.pdi"
    fi
fi
