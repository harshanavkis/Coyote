#!/usr/bin/env bash
# Every Loom experiment on amy and rose, for checking a hardware change: run
# it after every new image. From clara:
#
#   examples/loom_switch/regress.sh OUTDIR [--setup <u280.bit> <v80.pdi>]
#
#   --setup  first bring both hosts' cards up with these images
#            (scripts/fpga/setup_loom_hosts.sh); without it the cards must
#            already be up (U280 loom_switch image, V80 loom_ce image)
#
# The software is rebuilt on clara and copied to both hosts first, so the
# tools match this checkout. 16 MiB copies unless noted, every byte checked;
# put rates are timed on the landing host from the first data to the fence:
#   1. puts, V80 HBM -> network -> peer V80 HBM (ce_remote --land-v80), each
#      direction, 16 MiB x 8 and 64 MiB x 4; the rate is the copy engine's
#      (a V80 landing cannot be timed on the landing host: reads through the
#      V80's window wait for its writes)
#   2. puts, V80 HBM -> network -> peer host (ce_remote --bidir): both ways
#      at once, then one way each direction
#   3. CPU puts through the window (uwin_probe), each host
#   4. gets: copy engine without the network (ce_get --local), each host;
#      rose's copy engine reading amy over the network (ce_get, window 128)
#      with reads per get; CPU gets (get_bench)
#   5. local puts (ce_local, 7 copies: one slow copy in three is common) on each host: V80 self-loop, V80 -> host,
#      V80 -> U280 -> host, V80 -> U280 -> V80 HBM (ce_local triggered the
#      driver's page-pinning oops on 2026-10-05/06)
#   6. put sizes: rose's V80 -> amy's host memory, 4 KiB .. 64 MiB copies,
#      each measurement a burst of back-to-back copies moving 64 MiB (host
#      memory, so the landing host can time it; bursts of 1 MiB copies used
#      to stall, fixed in the driver, de45b0c6)
# Logs in OUTDIR, one per step; the summary goes to stdout. STEPS=15 (for
# example) runs only those steps. Every result also goes to OUTDIR/metrics.tsv
# (name, value, tolerance in %: the median of the warm copies), and at the end
# it is compared with the newest other metrics.tsv next to OUTDIR (or
# COMPARE=<file>): a change beyond the tolerance is marked CHECK.
set -u
STEPS=${STEPS:-123456}
OUT=${1:?usage: regress.sh OUTDIR [--setup <u280.bit> <v80.pdi>]}; shift
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(git -C "$HERE" rev-parse --show-toplevel)
mkdir -p "$OUT"; OUT=$(readlink -f "$OUT")
METRICS=$OUT/metrics.tsv; : > "$METRICS"

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
# A run that hits its timeout leaves the cards stuck (a copy engine waiting on
# a dead peer, an ack window that never reopens): stop, the cards need setup
hung() { echo; echo "ABORT: $1 failed or hit its timeout; the cards need setup_loom_hosts.sh before more runs (logs $OUT)"; finish; exit 3; }
# The health check and the comparison: at the end, and after an abort
finish() {
    step "health after"
    for h in amy rose; do
        errs=$(on $h "sudo journalctl -k --since @$T0 --no-pager | grep -ciE '\\bBUG\\b|Oops|\\bAER\\b'")
        faults=$(on $h "sudo journalctl -k --since @$T0 --no-pager | grep -c 'IO_PAGE_FAULT'")
        echo "$h: $(on $h "uptime | sed 's/.*up/up/'") kernel BUG/Oops/AER since the start: $errs, IOMMU faults: $faults | $(ns $h)"
        metric kernel_errors_$h "$errs" 0
    done

    PREV=${COMPARE:-$(ls -t "$(dirname "$OUT")"/*/metrics.tsv 2>/dev/null | grep -v "^$OUT/metrics.tsv$" | head -1)}
    if [ -n "$PREV" ] && [ -f "$PREV" ]; then
        step "compared with $PREV"
        awk -F'\t' 'NR == FNR { p[$1] = $2; next }
            {
                name = $1; cur = $2; tol = $3; prev = (name in p) ? p[name] : "NA"; mark = ""; ch = ""
                if (cur == "NA") mark = "CHECK (no result)"
                else if (prev == "NA") mark = "(new)"
                else if (prev + 0 == 0) { if (cur + 0 != 0) mark = "CHECK" }
                else {
                    ch = sprintf("%+.1f%%", (cur - prev) / prev * 100)
                    if ((cur - prev) / prev * 100 > tol || (cur - prev) / prev * 100 < -tol) mark = "CHECK"
                }
                n += (mark ~ /^CHECK/)
                printf "%-36s %10s %10s %8s  %s\n", name, prev, cur, ch, mark
            }
            END { printf "%d metric(s) to CHECK\n", n }' "$PREV" "$METRICS"
    else
        step "no earlier metrics.tsv next to $OUT to compare with"
    fi
}

# metric <name> <value> [tolerance %, default 5]
metric() { printf '%s\t%s\t%s\n' "$1" "${2:-NA}" "${3:-5}" >> "$METRICS"; }
# the median of the numbers on stdin, one per line
median() { grep -E '^[0-9.]+$' | sort -g | awk '{a[NR] = $1} END { if (NR) print (NR % 2 ? a[(NR + 1) / 2] : (a[NR / 2] + a[NR / 2 + 1]) / 2); else print "NA" }'; }
warm() { tail -n +2; }               # every copy but the first (cold)
words() { tr ' ' '\n' | grep .; }
# CE cycles (4 ns) per copy -> GB/s, from a ce_remote client log
ce_rate() { grep -E '^(ok|FAIL) +copy' "$1" | sed -nE 's/.*CE ([0-9]+) cycles.*/\1/p' | awk -v s=$2 '{printf "%.2f ", s / ($1 * 4e-9) / 1e9}'; }
# server-side rate (first data -> fence) per copy, from a ce_remote server log
landed() { grep -E '^(ok|FAIL) +copy' "$1" | sed -nE 's/.*after the first data \(([0-9.]+) GB\/s\).*/\1/p' | tr '\n' ' '; }

echo "regress.sh: $(git -C "$REPO" log --oneline -1), $(date -u +%F)"

if [ "${1:-}" = --setup ]; then
    step "setup: $2 / $3"
    "$REPO/scripts/fpga/setup_loom_hosts.sh" "$2" "$3" amy rose > "$OUT/setup.log" 2>&1 \
        || { grep -E '^SETUP|could not|still running' "$OUT/setup.log"; echo "SETUP FAILED, see $OUT/setup.log"; exit 1; }
    grep -E '^SETUP' "$OUT/setup.log"
fi

step "software: rebuild on clara, copy to amy and rose"
for d in $CE $SW; do
    (cd $d && nix-shell $REPO/shell.nix --run "cmake .. > /dev/null 2>&1 && make -j16 2>&1 | grep -E 'error|Built target' | tail -1") || { echo "build failed in $d"; exit 1; }
    for h in $A $R; do rsync -a $d/ $h:$d/ || { echo "rsync to $h failed"; exit 1; }; done
done

step "health"
T0=$(date +%s)
for h in amy rose; do
    echo "$h: $(on $h 'uptime | sed "s/.*up/up/"; echo "U280 IP $(sudo cat /sys/kernel/coyote_sysfs_0/cyt_attr_ip | grep -oE "[0-9a-f]{8}" | head -1),"; echo "V80 driver nodes $(ls -d /sys/kernel/coyote_versal_sysfs_* 2>/dev/null | wc -l)"' | tr '\n' ' ') | $(ns $h)"
    up=$(on $h "cut -d. -f1 /proc/uptime")
    [ "${up:-0}" -lt 900 ] && echo "WARNING: $h booted $((up / 60)) min ago: CPU-side numbers (uwin_probe, CPU gets) have been off in the first ~15 min after a boot"
done

[[ $STEPS == *1* ]] && {
step "1. puts, V80 HBM -> network -> peer V80 HBM (ce_remote --land-v80)"
for pair in "amy rose" "rose amy"; do
    S=${pair%% *}; C=${pair##* }   # server (lands), client (sends)
    for sz in "16777216 8" "67108864 4"; do
        size=${sz% *}; reps=${sz#* }; nextport; p=$PORT; L=$OUT/hbm_${C}_to_${S}_$size
        on $S "cd $CE && sudo stdbuf -oL timeout 300 $NUMA ./ce_remote --server --land-v80 --size $size --port $p" > $L.server.log 2>&1 & sp=$!
        sleep 4
        on $C "cd $CE && sudo stdbuf -oL timeout 280 $NUMA ./ce_remote --client ${IP[$S]} --reps $reps --port $p" > $L.client.log 2>&1 || hung "$L.client"
        wait $sp || hung "$L.server"
        echo "-- $C -> $S V80 HBM, $((size >> 20)) MiB x $reps: CE $(ce_rate $L.client.log $size)$(grep -qE '^FAIL' $L.server.log || echo '(all byte-exact)')"
        metric hbm_${C}_to_${S}_$((size >> 20))M_ce "$(ce_rate $L.client.log $size | words | warm | median)"
        grep -hE '^FAIL' $L.*.log | head -3
    done
done
}

[[ $STEPS == *2* ]] && {
step "2. puts, V80 HBM -> network -> peer host (ce_remote --bidir)"
for mode in "both::" "rose_to_amy:--no-send:" "amy_to_rose::--no-send"; do
    name=${mode%%:*}; rest=${mode#*:}; SX=${rest%%:*}; CX=${rest#*:}; nextport; p=$PORT; L=$OUT/pair_$name
    on amy "cd $CE && sudo stdbuf -oL timeout 120 $NUMA ./ce_remote --bidir-server --size 16777216 --reps 8 --port $p $SX" > $L.server.log 2>&1 & sp=$!
    sleep 3
    on rose "cd $CE && sudo stdbuf -oL timeout 120 $NUMA ./ce_remote --bidir-client ${IP[amy]} --size 16777216 --reps 8 --port $p $CX" > $L.client.log 2>&1 || hung "$L.client"
    wait $sp || hung "$L.server"
    into_rose=$(grep -E '^ok +copy' $L.client.log | sed -nE 's/.*in [0-9]+ B landed [0-9.]+ us after its first data \(([0-9.]+) GB\/s\).*/\1/p' | tr '\n' ' ')
    into_amy=$(grep -E '^ok +copy' $L.server.log | sed -nE 's/.*in [0-9]+ B landed [0-9.]+ us after its first data \(([0-9.]+) GB\/s\).*/\1/p' | tr '\n' ' ')
    echo "-- $name: into rose ${into_rose:-none} | into amy ${into_amy:-none}"
    [ -n "$into_rose" ] && metric pair_${name}_into_rose "$(echo $into_rose | words | warm | median)"
    [ -n "$into_amy" ]  && metric pair_${name}_into_amy "$(echo $into_amy | words | warm | median)"
    grep -hE '^FAIL' $L.*.log | head -3
done
}

[[ $STEPS == *3* ]] && {
step "3. CPU puts through the window (uwin_probe 1 MiB)"
for h in amy rose; do
    on $h "cd $SW && sudo stdbuf -oL timeout 120 $NUMA ./uwin_probe 1048576 256" > $OUT/uwin_probe_$h.log 2>&1
    echo "-- $h: $(grep -E 'bulk|PASS|FAIL' $OUT/uwin_probe_$h.log | tr '\n' ' ' | cut -c1-200)"
    metric uwin_probe_$h "$(sed -nE 's/.*bulk.*\(([0-9.]+) GB\/s\).*/\1/p' $OUT/uwin_probe_$h.log | head -1)" 40
done
}

[[ $STEPS == *4* ]] && {
step "4. gets"
for h in amy rose; do
    on $h "cd $CE && sudo stdbuf -oL timeout 240 $NUMA ./ce_get --local --reps 1" > $OUT/ceget_local_$h.log 2>&1
    echo "-- $h ce_get --local: $(grep -E '^(ok|FAIL) +get' $OUT/ceget_local_$h.log | sed -nE 's/.*get +([0-9]+) bytes:.*\( *([0-9.]+) GB\/s\).*/\1:\2/p' | tail -3 | tr '\n' ' ')$(grep -oE 'CE GET (PASS|FAIL)' $OUT/ceget_local_$h.log)"
    metric ceget_local_${h}_16M "$(sed -nE 's/.*get +16777216 bytes:.*\( *([0-9.]+) GB\/s\).*/\1/p' $OUT/ceget_local_$h.log | median)"
done
nextport; p=$PORT; a0=$(ns amy); r0=$(ns rose)
on amy "cd $SW && sudo stdbuf -oL timeout 400 $NUMA ./get_bench --server --port $p --window 128" > $OUT/ceget_net_server.log 2>&1 & sp=$!
sleep 4
on rose "cd $CE && sudo stdbuf -oL timeout 380 $NUMA ./ce_get --client ${IP[amy]} --port $p --window 128 --reps 3" > $OUT/ceget_net_client.log 2>&1
wait $sp
reads=$(grep -E '^(ok|FAIL) +get' $OUT/ceget_net_client.log | sed -nE 's/.*U280 ([0-9]+) reads.*/\1/p' | awk '{s += $1} END {print s + 0}')
reqs=$(grep -oE 'requests [0-9]+' $OUT/ceget_net_server.log | awk '{print $2}')
rpg=$(awk -v a=$reads -v b=${reqs:-0} 'BEGIN { if (b) printf "%.1f", a / b; else print "NA" }')
echo "-- rose's copy engine reads amy (window 128): $(grep -E '^(ok|FAIL) +get' $OUT/ceget_net_client.log | awk 'NR % 3 == 0' | sed -nE 's/^(ok|FAIL) +get +([0-9]+) bytes:.*\( *([0-9.]+) GB\/s\).*/\1 \2:\3/p' | tr '\n' ' ')$(grep -oE 'CE GET (PASS|FAIL)' $OUT/ceget_net_client.log)"
echo "   reads $reads, gets answered ${reqs:-?}, reads per get $rpg; $(grep -oE 'cycles waiting for data [0-9]+' $OUT/ceget_net_server.log)"
metric ceget_net_16M "$(sed -nE 's/.*get +16777216 bytes:.*\( *([0-9.]+) GB\/s\).*/\1/p' $OUT/ceget_net_client.log | median)"
metric ceget_net_reads_per_get "$rpg" 50   # flips between ~16 and ~11 (V80 read gaps vs the 32-cycle timer), same rate
nextport; p=$PORT
on amy "cd $SW && sudo stdbuf -oL timeout 600 $NUMA ./get_bench --server --port $p --window 128" > $OUT/cpuget_server.log 2>&1 & sp=$!
sleep 4
on rose "cd $SW && sudo stdbuf -oL timeout 580 $NUMA ./get_bench --client ${IP[amy]} --port $p --window 128" > $OUT/cpuget_client.log 2>&1
wait $sp
echo "-- CPU gets (rose reads amy): $(grep -E '8 B load' $OUT/cpuget_client.log | sed -E 's/ +/ /g'); $(grep -E '16384 B:' $OUT/cpuget_client.log | sed -E 's/ +/ /g' | cut -c1-70)"
grep -E 'threads:' $OUT/cpuget_client.log | sed -E 's/ +/ /g; s/^/   /'
echo "   $(grep -oE 'GET BENCH (PASS|FAIL)' $OUT/cpuget_client.log)"
echo "-- RoCE counters during gets: amy $a0 -> $(ns amy) | rose $r0 -> $(ns rose)"
metric cpuget_load8_us "$(sed -nE 's/.*8 B load: median +([0-9.]+) us.*/\1/p' $OUT/cpuget_client.log)" 5
metric cpuget_16K_reads_per_transfer "$(sed -nE 's/.*16384 B:.* ([0-9.]+) reads per transfer.*/\1/p' $OUT/cpuget_client.log)" 20
metric cpuget_16K "$(sed -nE 's/.*16384 B:.*, +([0-9.]+) GB\/s.*/\1/p' $OUT/cpuget_client.log)" 40
metric cpuget_4threads "$(sed -nE 's/.* 4 threads:.*, +([0-9.]+) GB\/s.*/\1/p' $OUT/cpuget_client.log)" 40
}

[[ $STEPS == *5* ]] && {
step "5. local puts (ce_local, 16 MiB x 7)"
for h in amy rose; do
    for m in --self --host "" --land-v80; do
        L=$OUT/ce_local_${h}${m:-_u280host}.log
        on $h "cd $CE && sudo stdbuf -oL timeout 240 $NUMA ./ce_local $m 16777216 7" > $L 2>&1
        rates=$(sed -nE 's/^(ok|FAIL).*fence (seen )?after [0-9.]+ us \(([0-9.]+) GB\/s\).*/\3/p' $L | tr '\n' ' ')
        echo "-- $h ce_local ${m:-(V80 -> U280 -> host)}: $rates$(grep -oE '\b(PASS|FAIL)\b' $L | tail -1)"
        metric ce_local_${h}${m:-_u280host} "$(echo $rates | words | warm | median)"
    done
done
}

[[ $STEPS == *6* ]] && {
step "6. put sizes: rose's V80 -> amy's host memory, bursts of 64 MiB (landed | issued by the copy engine)"
for size in 4096 16384 65536 262144 1048576 4194304 16777216 67108864; do
    burst=$((67108864 / size)); nextport; p=$PORT; L=$OUT/putsize_$size
    on amy "cd $CE && sudo stdbuf -oL timeout 300 $NUMA ./ce_remote --server --size $size --port $p" > $L.server.log 2>&1 & sp=$!
    sleep 4
    on rose "cd $CE && sudo stdbuf -oL timeout 280 $NUMA ./ce_remote --client ${IP[amy]} --reps 3 --burst $burst --port $p" > $L.client.log 2>&1 || hung "$L.client"
    wait $sp || hung "$L.server"
    issued=$(grep -E '^(ok|FAIL) +copy' $L.client.log | sed -nE 's/.*after the first start \(([0-9.]+) GB\/s\).*/\1/p' | tr '\n' ' ')
    echo "-- $size B x $burst: landed $(landed $L.server.log)| issued $issued"
    metric putsize_${size}_landed "$(landed $L.server.log | words | median)"
    metric putsize_${size}_issued "$(echo $issued | words | median)"
    grep -hE '^FAIL' $L.*.log | head -3
done
}

finish
echo; echo "REGRESS DONE (logs $OUT)"
