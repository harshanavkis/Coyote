#!/usr/bin/env bash
# 2026-10-27 (owner: the RoCE stack loses rate both ways and more with several QPs; reproduce 1 and 2 QPs both ways vs one
# way): 2 MiB per direction (1 QP: 2 x 1 MiB WRs; 2 QPs: one 1 MiB WR per QP), 16 B flags (no 64 B-multiple packet), link
# delay DELAY cycles each way, prints off. RATE/WIN/LAT/STALL lines per node. Run from bash.
#   R1 1 QP one way   R2 1 QP both ways   R3 2 QPs one way   R4 2 QPs both ways
cd "$(dirname "$0")"
D=${DELAY:-500}
C="DELAY=$D FLAG_BYTES=16 VERBOSE=0 TIMEOUT=400000 WR_BYTES=1048576"
run() { local n=$1; shift; WORK=work_$n bash ./run_roce_tb.sh "$@" > run_$n.out 2>&1; }
P=${PFX:-R}
run ${P}1 NQ=1 NWR=2 SEND_B=0 $C &
run ${P}2 NQ=1 NWR=2 $C &
run ${P}3 NQ=2 NWR=1 SEND_B=0 $C &
run ${P}4 NQ=2 NWR=1 $C &
wait
for n in 1 2 3 4; do
    echo "== ${P}$n: $(grep -m1 -oE 'TB [A-Z]+[^|]*\| cycle [0-9]+' work_${P}$n/xsim.log 2>/dev/null || tail -1 run_${P}$n.out)"
    grep -A20 -E 'TB (PASS|FAIL|DONE)' work_${P}$n/xsim.log | grep -E 'RATE|WIN |LAT |STALL|counters'
done
echo RATE_TESTS_DONE
