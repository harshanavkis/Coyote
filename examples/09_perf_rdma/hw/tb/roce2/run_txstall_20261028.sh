#!/usr/bin/env bash
# 2026-10-28 03:30 (night): window 32 on hardware corrupted ~1 in 350k packets in amy's transmit path (losshunt p10). Does
# irregular MAC back-pressure corrupt packets in the stack's TX (trimmer / icrc)? Window 32, the build's HLS, current icrc,
# 2 QPs both ways, 2 x 1 MiB per QP and direction, 16 B flags, random m_tx stalls (TXSTALL_PCT) with two seeds.
cd "$(dirname "$0")"; T=$(pwd)
C="NQ=2 NWR=2 WR_BYTES=1048576 DELAY=500 FLAG_BYTES=16 VERBOSE=0 TIMEOUT=600000 STALL_CYC=40000"
run() { local n=$1; shift; WORK=work_$n PKG=$T/pkg/lynx_pkg_w32.sv bash ./run_roce_tb.sh "$@" > run_$n.out 2>&1; }
run X10a $C TXSTALL_PCT=10 SEED=1 & run X10b $C TXSTALL_PCT=10 SEED=7 &
run X30a $C TXSTALL_PCT=30 SEED=3 & run X30b $C TXSTALL_PCT=30 SEED=11 &
wait
for n in X10a X10b X30a X30b; do
    echo "== $n: $(grep -a -m1 -oE 'TB [A-Z]+[^|]*\| cycle [0-9]+' work_$n/xsim.log 2>/dev/null || tail -n 1 run_$n.out)"
    grep -a -A20 -E 'TB (PASS|FAIL|DONE)' work_$n/xsim.log | grep -E 'check:|counters|RATE A|DATA WRONG'
done
echo TXSTALL_DONE
