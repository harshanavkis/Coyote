#!/usr/bin/env bash
# rdma_flow window 16 -> 64 (owner 2026-10-26), lossless traffic as on the testbed since the flag-in-data fix: no packet a
# multiple of 64 B at IP level (icrc.sv's bug) and no forced drops (the stack does not recover from a loss: HLS defects
# D1/D2, 2026-10-25). Every landed byte checked.
#   L1 window 16, build HLS:   1 QP, 1 KiB WRs, long link (2,000 cycles each way)   control: throughput bound by the window
#   L2 window 64, new HLS:     same                                                 expect ~64 outstanding, sooner done
#   L3 window 64, new HLS:     2 QPs both ways, 64 KiB WRs (multi-packet)
#   L4 window 16, build HLS:   same as L3 (control)
#   L5 window 64, new HLS:     4 QPs both ways, 4 KiB WRs, long link (stress: up to 4 x 64 packets in flight per node)
#   L6 window 16, build HLS:   same as L5 (control)
# Each test in its own work dir (work_L*); summary at the end. Run from bash.
cd "$(dirname "$0")"
T=$(pwd)
B=/scratch/harshanavkis/loom-proj/Coyote/examples/09_perf_rdma/hw/build_oct23_rdma_stamp_noila
OLD=$B/network/hls/rocev2/rocev2_prj/solution1/syn/verilog
NEW=$T/hls_offs6/rocev2_prj/solution1/syn/verilog
W16=$T/pkg/lynx_pkg_w16.sv
W64=$T/pkg/lynx_pkg_w64.sv
ONE="NQ=1 NWR=60 WR_BYTES=1024 DELAY=2000 SEND_B=0 FLAG_BYTES=16 TIMEOUT=600000"
TWO="NQ=2 NWR=2 WR_BYTES=65536 DELAY=500 FLAG_BYTES=16 TIMEOUT=400000"
FOUR="NQ=4 NWR=20 WR_BYTES=4096 DELAY=2000 FLAG_BYTES=16 TIMEOUT=600000"
run() {   # name pkg hls args...
    local n=$1 p=$2 h=$3; shift 3
    WORK=work_$n PKG=$p HLSV=$h bash ./run_roce_tb.sh "$@" > run_$n.out 2>&1
}
run L1 $W16 $OLD $ONE &
run L2 $W64 $NEW $ONE &
run L3 $W64 $NEW $TWO &
run L4 $W16 $OLD $TWO &
run L5 $W64 $NEW $FOUR &
run L6 $W16 $OLD $FOUR &
wait
for n in L1 L2 L3 L4 L5 L6; do
    echo "== $n: $(grep -m1 -oE 'TB [A-Z]+[^|]*\| cycle [0-9]+' work_$n/xsim.log 2>/dev/null || tail -1 run_$n.out)"
    grep -m1 'check:' work_$n/xsim.log
    grep -m2 'counters' work_$n/xsim.log
    grep -m2 'DATA WRONG' work_$n/xsim.log
done
echo LOSSLESS_TESTS_DONE
