#!/usr/bin/env bash
# 2026-10-27 (owner: the 10-26/27 window-64 builds failed timing with the HLS change; can window 32 do without it?): the
# build's HLS rocev2 RTL (4-bit retransmitter slot) + window 32 + the current icrc, 16 B flags (no 64 B packets).
#   O1 1 QP both ways 2 x 1 MiB   O2 2 QPs both ways 1 MiB each   O4 4 QPs both ways 3 x 64 KiB (stress)
cd "$(dirname "$0")"
T=$(pwd)
P=$T/pkg/lynx_pkg_w32.sv
run() { local n=$1; shift; WORK=work_$n PKG=$P bash ./run_roce_tb.sh "$@" > run_$n.out 2>&1; }
run O1 NQ=1 NWR=2 WR_BYTES=1048576 DELAY=500 FLAG_BYTES=16 VERBOSE=0 TIMEOUT=400000 &
run O2 NQ=2 NWR=1 WR_BYTES=1048576 DELAY=500 FLAG_BYTES=16 VERBOSE=0 TIMEOUT=400000 &
run O4 NQ=4 NWR=3 WR_BYTES=65536 DELAY=500 FLAG_BYTES=16 VERBOSE=0 TIMEOUT=300000 &
wait
for n in O1 O2 O4; do
    echo "== $n: $(grep -a -m1 -oE 'TB [A-Z]+[^|]*\| cycle [0-9]+' work_$n/xsim.log 2>/dev/null || tail -n 1 run_$n.out)"
    grep -a -A20 -E 'TB (PASS|FAIL|DONE)' work_$n/xsim.log | grep -E 'check:|counters|RATE A|WIN  A|LAT  A'
done
echo W32_OLDHLS_DONE
