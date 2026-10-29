#!/usr/bin/env bash
# 2026-10-28 03:35 (night): does window 32 leave the receive path too little headroom when the host stalls landing writes?
# 2 QPs both ways, 2 x 1 MiB per QP and direction, the build's HLS, random landing stalls (LANDSTALL_PCT), window 16 vs 32.
cd "$(dirname "$0")"; T=$(pwd)
C="NQ=2 NWR=2 WR_BYTES=1048576 DELAY=500 FLAG_BYTES=16 VERBOSE=0 TIMEOUT=600000 STALL_CYC=40000 SEED=3"
run() { local n=$1 w=$2; shift 2; WORK=work_$n PKG=$T/pkg/lynx_pkg_w$w.sv bash ./run_roce_tb.sh "$@" > run_$n.out 2>&1; }
run L16s5 16 $C LANDSTALL_PCT=5 & run L32s5 32 $C LANDSTALL_PCT=5 &
run L16s15 16 $C LANDSTALL_PCT=15 & run L32s15 32 $C LANDSTALL_PCT=15 &
wait
for n in L16s5 L32s5 L16s15 L32s15; do
    echo "== $n: $(grep -a -m1 -oE 'TB [A-Z]+[^|]*\| cycle [0-9]+' work_$n/xsim.log 2>/dev/null || tail -n 1 run_$n.out)"
    grep -a -A22 -E 'TB (PASS|FAIL|DONE)' work_$n/xsim.log | grep -E 'TB |counters|RATE A|STALL A'
done
echo LANDSTALL_DONE
