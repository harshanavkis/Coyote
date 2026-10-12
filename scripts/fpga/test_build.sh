#!/usr/bin/env bash
# Test a Loom build end to end (run it in tmux on clara; ~10 min after the
# build ends): wait for the build and stage its image (wait_build.sh), bring
# amy and rose up with it (the other card keeps the standard image), run the
# full regression (examples/loom_switch/regress.sh) against a baseline, then
# ce_prog (the copy engine's program self-test and rates) on both hosts.
#
#   scripts/fpga/test_build.sh <switch|ce> <build_name> <stage_name> [baseline metrics.tsv]
#
# Standard images (override with U280_IMG / V80_IMG):
U280_IMG=${U280_IMG:-$HOME/coyote-bitstreams/loom-switch-coalesce-clean/hw/bitstreams/cyt_top.bit}
V80_IMG=${V80_IMG:-$HOME/coyote-bitstreams/loom-ce-uwinra/cyt_top.pdi}
# Results: ~/loom-experiments/regress/<date>_<stage_name>/ (metrics.tsv, logs,
# ce_prog_<host>.log). ssh to the hosts needs a forwarded agent (VS Code's or
# an ssh -A login's); a live one is picked before each phase, waiting while
# there is none.
set -u
REPO=$(realpath "$(dirname "$0")/../..")
[ $# -ge 3 ] || { sed -n '2,10p' "$0"; exit 2; }
WHAT=$1 NAME=$2 STAGE=$3 BASE=${4:-}
OUT=$HOME/loom-experiments/regress/$(date -u +%Y%m%d)_$STAGE
NUMA="/nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1"

pick_agent() {
    for i in $(seq 1 720); do
        for s in $(ls -t /run/user/$(id -u)/vscode-ssh-auth-sock-* $HOME/.ssh/agent/s.* 2>/dev/null); do
            if SSH_AUTH_SOCK=$s timeout 5 ssh-add -l > /dev/null 2>&1; then export SSH_AUTH_SOCK=$s; return 0; fi
        done
        [ $i = 1 ] && echo "TEST: no live ssh agent; waiting for one (reconnect VS Code or log in with ssh -A)"
        sleep 60
    done
    return 1
}

"$REPO/scripts/fpga/wait_build.sh" "$WHAT" "$NAME" "$STAGE" || { echo "TEST: no image, stopping"; exit 1; }
case $WHAT in
    switch) U280_IMG=$HOME/coyote-bitstreams/$STAGE/hw/bitstreams/cyt_top.bit ;;
    ce)     V80_IMG=$HOME/coyote-bitstreams/$STAGE/cyt_top.pdi ;;
esac
[ -f "$U280_IMG" ] && [ -f "$V80_IMG" ] || { echo "TEST: missing $U280_IMG or $V80_IMG"; exit 1; }

pick_agent || { echo "TEST: no ssh agent, stopping"; exit 1; }
COMPARE=$BASE "$REPO/examples/loom_switch/regress.sh" "$OUT" --setup "$U280_IMG" "$V80_IMG"
echo "TEST: regress.sh exit $?"

pick_agent
for h in amy rose; do
    echo "=== ce_prog on $h ($(date -u +%H:%M:%S))"
    ssh -o ConnectTimeout=15 -n $h.dos.cit.tum.de \
        "cd $REPO/examples/loom_ce/sw/build && sudo timeout 300 $NUMA ./ce_prog" 2>&1 | tee "$OUT/ce_prog_$h.log"
done
echo "TEST: done $(date -Is), results in $OUT"
