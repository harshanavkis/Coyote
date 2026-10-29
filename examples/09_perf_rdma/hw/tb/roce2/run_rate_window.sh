#!/usr/bin/env bash
# 2026-10-27: does a larger rdma_flow window remove the both-way loss (run_rate_tests.sh: window-bound, 8.45 / 8.79 GB/s)?
# Both directions, 2 MiB each, as R2 (1 QP) / R4 (2 QPs), with the 6-bit-slot HLS (hls_offs6, 2026-10-26) and window 32 / 64.
cd "$(dirname "$0")"
T=$(pwd)
NEW=$T/hls_offs6/rocev2_prj/solution1/syn/verilog
D=${DELAY:-500}
C="DELAY=$D FLAG_BYTES=16 VERBOSE=0 TIMEOUT=400000 WR_BYTES=1048576"
run() { local n=$1 p=$2; shift 2; WORK=work_$n PKG=$p HLSV=$NEW bash ./run_roce_tb.sh "$@" > run_$n.out 2>&1; }
run W32a $T/pkg/lynx_pkg_w32.sv NQ=1 NWR=2 $C &
run W32b $T/pkg/lynx_pkg_w32.sv NQ=2 NWR=1 $C &
run W64a $T/pkg/lynx_pkg_w64.sv NQ=1 NWR=2 $C &
run W64b $T/pkg/lynx_pkg_w64.sv NQ=2 NWR=1 $C &
wait
for n in W32a W32b W64a W64b; do
    echo "== $n: $(grep -m1 -oE 'TB [A-Z]+[^|]*\| cycle [0-9]+' work_$n/xsim.log 2>/dev/null || tail -1 run_$n.out)"
    grep -A20 -E 'TB (PASS|FAIL|DONE)' work_$n/xsim.log | grep -E 'check:|RATE A|WIN  A|LAT  A|STALL A|counters'
done
echo WINDOW_RATE_DONE
