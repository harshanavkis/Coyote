#!/usr/bin/env bash
# 2026-10-28 (Loom both-way limit): hardware Loom both ways = one 4 KiB packet per 95 cycles at every stage, one way 83.5
# (line); RDMA (multi-packet WRs) both ways 11.9 GB/s. Loom sends every packet as its own WRITE_ONLY (one request per 4 KiB).
# Does the stack alone slow down with WRITE_ONLY packets both ways? Window 32 package (= the w32 images), no flags.
#   A1 4 KiB WRITE_ONLY x 512, 1 QP, both ways   A2 the same one way   B1 one 2 MiB WR (FIRST/MIDDLE/LAST), 1 QP, both ways
cd "$(dirname "$0")"
P=$(pwd)/pkg/lynx_pkg_w32.sv
run() { local n=$1; shift; WORK=work_$n PKG=$P bash ./run_roce_tb.sh "$@" > run_$n.out 2>&1; }
run A1 NQ=1 NWR=512 WR_BYTES=4096 DELAY=500 FLAG_BYTES=0 VERBOSE=0 TIMEOUT=400000 &
run A2 NQ=1 NWR=512 WR_BYTES=4096 DELAY=500 FLAG_BYTES=0 VERBOSE=0 TIMEOUT=400000 SEND_B=0 &
run B1 NQ=1 NWR=1 WR_BYTES=2097152 DELAY=500 FLAG_BYTES=0 VERBOSE=0 TIMEOUT=400000 &
wait
for n in A1 A2 B1; do
    echo "== $n: $(grep -a -m1 -oE 'TB [A-Z]+[^|]*\| cycle [0-9]+' work_$n/xsim.log 2>/dev/null || tail -n 1 run_$n.out)"
    grep -a -A20 -E 'TB (PASS|FAIL|DONE)' work_$n/xsim.log | grep -E 'check:|counters|RATE A|RATE B|WIN  A|LAT  A'
done
echo ONLY_DONE
