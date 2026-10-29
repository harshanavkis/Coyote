#!/usr/bin/env bash
# 2026-10-28: icrc_fast.sv (CRC352/CRC448 stages + in_first + roce-fixes 64 B fix) in the two-node stack TB, window-32 package.
#   F1 4 KiB WRITE_ONLY x 512 both ways (Loom-like)   F2 the same one way   F3 one 2 MiB WR both ways (RDMA-like)
#   F4 2 QPs both ways 4 x 64 KiB + 8 B flags (64 B packets: the 10-27 icrc fix)   + icrc rate per packet pattern
cd "$(dirname "$0")"
P=$(pwd)/pkg/lynx_pkg_w32.sv; I=$(pwd)/icrc_fast.sv
run() { local n=$1; shift; WORK=work_$n PKG=$P ICRC=$I bash ./run_roce_tb.sh "$@" > run_$n.out 2>&1; }
WORK=work_icrc_fast ICRC=$I bash ./run_icrc_rate.sh "LEN_A=4152" "LEN_A=4136" "LEN_A=44" "LEN_A=4152 LEN_B=44" "LEN_A=4136 LEN_B=44" "LEN_A=64 LEN_B=44" > icrc_rate_fast.out 2>&1 &
run F1 NQ=1 NWR=512 WR_BYTES=4096 DELAY=500 FLAG_BYTES=0 VERBOSE=0 TIMEOUT=400000 &
run F2 NQ=1 NWR=512 WR_BYTES=4096 DELAY=500 FLAG_BYTES=0 VERBOSE=0 TIMEOUT=400000 SEND_B=0 &
run F3 NQ=1 NWR=1 WR_BYTES=2097152 DELAY=500 FLAG_BYTES=0 VERBOSE=0 TIMEOUT=400000 &
run F4 NQ=2 NWR=4 WR_BYTES=65536 DELAY=500 FLAG_BYTES=8 VERBOSE=0 TIMEOUT=400000 &
wait
for n in F1 F2 F3 F4; do
    echo "== $n: $(grep -a -m1 -oE 'TB [A-Z]+[^|]*\| cycle [0-9]+' work_$n/xsim.log 2>/dev/null || tail -n 1 run_$n.out)"
    grep -a -A20 -E 'TB (PASS|FAIL|DONE)' work_$n/xsim.log | grep -E 'check:|counters|RATE A|RATE B|STALL A'
done
echo "== icrc rate (icrc_fast)"; cat icrc_rate_fast.out
echo FAST_DONE
