#!/usr/bin/env bash
# Every Loom experiment on amy and rose, for checking a hardware change: run
# it after every new image, then compare with the last results in PLAN.md
# (Running). From clara:
#
#   examples/loom_switch/regress.sh OUTDIR [--setup <u280.bit> <v80.pdi>]
#
#   --setup  first bring both hosts' cards up with these images
#            (scripts/fpga/setup_loom_hosts.sh); without it the cards must
#            already be up (U280 loom_switch image, V80 loom_ce image)
#
# The software is rebuilt on clara and copied to both hosts first, so the
# tools match this checkout. 16 MiB copies unless noted, every byte checked:
#   1. puts, V80 HBM -> network -> peer V80 HBM (ce_remote --land-v80), each
#      direction, 16 MiB x 8 and 64 MiB x 4
#   2. puts, V80 HBM -> network -> peer host (ce_remote --bidir): both ways
#      at once, then one way each direction
#   3. CPU puts through the window (uwin_probe), each host
#   4. gets: copy engine without the network (ce_get --local), each host;
#      rose's copy engine reading amy over the network (ce_get, window 128)
#      with reads per get; CPU gets (get_bench)
#   5. local puts (ce_local) on each host: V80 self-loop, V80 -> host,
#      V80 -> U280 -> host, V80 -> U280 -> V80 HBM. Last: ce_local is what
#      triggered the driver's page-pinning oops before (2026-10-05/06).
# Logs in OUTDIR, one per step; the summary goes to stdout. STEPS=15 (for
# example) runs only those steps.
set -u
STEPS=${STEPS:-12345}
OUT=${1:?usage: regress.sh OUTDIR [--setup <u280.bit> <v80.pdi>]}; shift
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(git -C "$HERE" rev-parse --show-toplevel)
mkdir -p "$OUT"; OUT=$(readlink -f "$OUT")

NUMA="/nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1"
CE=$REPO/examples/loom_ce/sw/build
SW=$REPO/examples/loom_switch/sw/build
A=amy.dos.cit.tum.de; R=rose.dos.cit.tum.de
declare -A IP=([amy]=131.159.102.20 [rose]=131.159.102.21)
PORT=19700
nextport() { PORT=$((PORT + 2)); }   # in this shell: a $(...) subshell would lose the count
on() { ssh -o ConnectTimeout=15 -n "$1.dos.cit.tum.de" "$2"; }
ns() { on $1 "sudo cat /sys/kernel/coyote_sysfs_0/cyt_attr_nstats" | grep -E "^(ROCE TX pkgs|PSN drop cnt|Retrans cnt):" | tr -s ' ' | tr '\n' ' '; }
step() { echo; echo "=== $* ($(date -u +%H:%M:%S))"; }
# CE cycles (4 ns) per copy -> GB/s, from a ce_remote client log
ce_rate() { grep -E '^(ok|FAIL) +copy' "$1" | sed -nE 's/.*CE ([0-9]+) cycles.*/\1/p' | awk -v s=$2 '{printf "%.2f ", s / ($1 * 4e-9) / 1e9}'; }

echo "regress.sh: $(git -C "$REPO" log --oneline -1), $(date -u +%F)"

if [ "${1:-}" = --setup ]; then
    step "setup: $2 / $3"
    "$REPO/scripts/fpga/setup_loom_hosts.sh" "$2" "$3" amy rose > "$OUT/setup.log" 2>&1 \
        || { grep -E '^SETUP|could not|still running' "$OUT/setup.log"; echo "SETUP FAILED, see $OUT/setup.log"; exit 1; }
    grep -E '^SETUP' "$OUT/setup.log"
fi

step "software: rebuild on clara, copy to amy and rose"
for d in $CE $SW; do
    (cd $d && nix-shell $REPO/shell.nix --run "cmake .. > /dev/null && make -j16 2>&1 | tail -1") || { echo "build failed in $d"; exit 1; }
    for h in $A $R; do rsync -a $d/ $h:$d/ || { echo "rsync to $h failed"; exit 1; }; done
done

step "health"
T0=$(date +%s)
for h in amy rose; do
    echo "$h: $(on $h 'uptime | sed "s/.*up/up/"; echo "U280 IP $(sudo cat /sys/kernel/coyote_sysfs_0/cyt_attr_ip | grep -oE "[0-9a-f]{8}" | head -1),"; echo "V80 driver nodes $(ls -d /sys/kernel/coyote_versal_sysfs_* 2>/dev/null | wc -l)"' | tr '\n' ' ') | $(ns $h)"
done

[[ $STEPS == *1* ]] && {
step "1. puts, V80 HBM -> network -> peer V80 HBM (ce_remote --land-v80)"
for pair in "amy rose" "rose amy"; do
    S=${pair%% *}; C=${pair##* }   # server (lands), client (sends)
    for sz in "16777216 8" "67108864 4"; do
        size=${sz% *}; reps=${sz#* }; nextport; p=$PORT; L=$OUT/hbm_${C}_to_${S}_$size
        on $S "cd $CE && sudo timeout 300 $NUMA ./ce_remote --server --land-v80 --size $size --port $p" > $L.server.log 2>&1 & sp=$!
        sleep 4
        on $C "cd $CE && sudo timeout 280 $NUMA ./ce_remote --client ${IP[$S]} --reps $reps --port $p" > $L.client.log 2>&1
        wait $sp
        echo "-- $C -> $S V80 HBM, $((size >> 20)) MiB x $reps: fence $(grep -E '^(ok|FAIL) +copy' $L.server.log | sed -nE 's/^(ok|FAIL).*\(([0-9.]+) GB\/s\).*/\1:\2/p' | tr '\n' ' ')| CE $(ce_rate $L.client.log $size)"
        grep -hE '^FAIL' $L.*.log | head -3
    done
done
}

[[ $STEPS == *2* ]] && {
step "2. puts, V80 HBM -> network -> peer host (ce_remote --bidir)"
for mode in "both::" "rose_to_amy:--no-send:" "amy_to_rose::--no-send"; do
    name=${mode%%:*}; rest=${mode#*:}; SX=${rest%%:*}; CX=${rest#*:}; nextport; p=$PORT; L=$OUT/pair_$name
    on amy "cd $CE && sudo timeout 120 $NUMA ./ce_remote --bidir-server --size 16777216 --reps 8 --port $p $SX" > $L.server.log 2>&1 & sp=$!
    sleep 3
    on rose "cd $CE && sudo timeout 120 $NUMA ./ce_remote --bidir-client ${IP[amy]} --size 16777216 --reps 8 --port $p $CX" > $L.client.log 2>&1
    wait $sp
    into_rose=$(grep -E '^ok +copy' $L.client.log | grep -oE 'in [0-9]+ B landed [0-9.]+ us after the barrier \([0-9.]+' | grep -oE '[0-9.]+$' | tr '\n' ' ')
    into_amy=$(grep -E '^ok +copy' $L.server.log | grep -oE 'in [0-9]+ B landed [0-9.]+ us after the barrier \([0-9.]+' | grep -oE '[0-9.]+$' | tr '\n' ' ')
    echo "-- $name: into rose ${into_rose:-none} | into amy ${into_amy:-none}"
    grep -hE '^FAIL' $L.*.log | head -3
done
}

[[ $STEPS == *3* ]] && {
step "3. CPU puts through the window (uwin_probe 1 MiB)"
for h in amy rose; do
    echo "-- $h: $(on $h "cd $SW && sudo timeout 120 $NUMA ./uwin_probe 1048576 256" 2>&1 | tee $OUT/uwin_probe_$h.log | grep -E 'bulk|PASS|FAIL' | tr '\n' ' ' | cut -c1-200)"
done
}

[[ $STEPS == *4* ]] && {
step "4. gets"
for h in amy rose; do
    on $h "cd $CE && sudo timeout 240 $NUMA ./ce_get --local --reps 1" > $OUT/ceget_local_$h.log 2>&1
    echo "-- $h ce_get --local: $(grep -E '^(ok|FAIL) +get' $OUT/ceget_local_$h.log | sed -nE 's/.*get +([0-9]+) bytes:.*\( *([0-9.]+) GB\/s\).*/\1:\2/p' | tail -3 | tr '\n' ' ')$(grep -oE 'CE GET (PASS|FAIL)' $OUT/ceget_local_$h.log)"
done
nextport; p=$PORT; a0=$(ns amy); r0=$(ns rose)
on amy "cd $SW && sudo timeout 400 $NUMA ./get_bench --server --port $p --window 128" > $OUT/ceget_net_server.log 2>&1 & sp=$!
sleep 4
on rose "cd $CE && sudo timeout 380 $NUMA ./ce_get --client ${IP[amy]} --port $p --window 128 --reps 3" > $OUT/ceget_net_client.log 2>&1
wait $sp
reads=$(grep -E '^(ok|FAIL) +get' $OUT/ceget_net_client.log | sed -nE 's/.*U280 ([0-9]+) reads.*/\1/p' | awk '{s += $1} END {print s + 0}')
reqs=$(grep -oE 'requests [0-9]+' $OUT/ceget_net_server.log | awk '{print $2}')
echo "-- rose's copy engine reads amy (window 128): $(grep -E '^(ok|FAIL) +get' $OUT/ceget_net_client.log | awk 'NR % 3 == 0' | sed -nE 's/^(ok|FAIL) +get +([0-9]+) bytes:.*\( *([0-9.]+) GB\/s\).*/\1 \2:\3/p' | tr '\n' ' ')$(grep -oE 'CE GET (PASS|FAIL)' $OUT/ceget_net_client.log)"
echo "   reads $reads, gets answered ${reqs:-?}, reads per get $(awk -v a=$reads -v b=${reqs:-0} 'BEGIN { if (b) printf "%.1f", a / b; else print "?" }'); $(grep -oE 'cycles waiting for data [0-9]+' $OUT/ceget_net_server.log)"
nextport; p=$PORT
on amy "cd $SW && sudo timeout 600 $NUMA ./get_bench --server --port $p --window 128" > $OUT/cpuget_server.log 2>&1 & sp=$!
sleep 4
on rose "cd $SW && sudo timeout 580 $NUMA ./get_bench --client ${IP[amy]} --port $p --window 128" > $OUT/cpuget_client.log 2>&1
wait $sp
echo "-- CPU gets (rose reads amy): $(grep -E '8 B load' $OUT/cpuget_client.log | sed -E 's/ +/ /g'); $(grep -E '16384 B:' $OUT/cpuget_client.log | sed -E 's/ +/ /g' | cut -c1-60)"
grep -E 'threads:' $OUT/cpuget_client.log | sed -E 's/ +/ /g; s/^/   /'
echo "   $(grep -oE 'GET BENCH (PASS|FAIL)' $OUT/cpuget_client.log)"
echo "-- RoCE counters during gets: amy $a0 -> $(ns amy) | rose $r0 -> $(ns rose)"
}

[[ $STEPS == *5* ]] && {
step "5. local puts (ce_local, 16 MiB x 3)"
for h in amy rose; do
    for m in --self --host "" --land-v80; do
        L=$OUT/ce_local_${h}${m:-_u280host}.log
        on $h "cd $CE && sudo timeout 240 $NUMA ./ce_local $m 16777216 3" > $L 2>&1
        echo "-- $h ce_local ${m:-(V80 -> U280 -> host)}: $(sed -nE 's/^(ok|FAIL).*fence (seen )?after [0-9.]+ us \(([0-9.]+) GB\/s\).*/\1:\3/p' $L | tr '\n' ' ')$(grep -oE '\b(PASS|FAIL)\b' $L | tail -1)"
    done
done
}

step "health after"
for h in amy rose; do
    echo "$h: $(on $h "uptime | sed 's/.*up/up/'; echo 'kernel BUG/Oops/AER since the start:' \$(sudo journalctl -k --since @$T0 --no-pager | grep -ciE '\\bBUG\\b|Oops|\\bAER\\b'), 'IOMMU faults:' \$(sudo journalctl -k --since @$T0 --no-pager | grep -c 'IO_PAGE_FAULT')" | tr '\n' ' ') | $(ns $h)"
done
echo; echo "REGRESS DONE (logs $OUT)"
