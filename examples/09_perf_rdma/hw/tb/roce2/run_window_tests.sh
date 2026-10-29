#!/usr/bin/env bash
# rdma_flow window 16 -> 64 (owner 2026-10-26): the RoCE two-node bench with forced drops and a check of every landed byte.
#   T1 window 16, build HLS (4-bit retransmitter slot)      control: today's stack, expect PASS
#   T2 window 64, HLS with 6-bit slot fields (hls_offs6)     the change, expect PASS and ~64 outstanding
#   T3 window 64, build HLS                                  mutation: slot truncated to 4 bits, expect TB FAIL: data
#   T4a/b as T1 / T2 with the buffer decoding 33 address bits (the U280's HBM port)
#   T5 window 64, new HLS, both directions, 2 QPs, 64 KiB WRs, no drops (regression of the plain path)
#   T6 window 64, new HLS, 2 QPs, drops on both QPs' traffic
# Each test in its own work dir (work_T*), output run_T*.out; summary at the end. Run from bash (zsh does not split "$a").
cd "$(dirname "$0")"
T=$(pwd)
B=/scratch/harshanavkis/loom-proj/Coyote/examples/09_perf_rdma/hw/build_oct23_rdma_stamp_noila
OLD=$B/network/hls/rocev2/rocev2_prj/solution1/syn/verilog
NEW=$T/hls_offs6/rocev2_prj/solution1/syn/verilog
W16=$T/pkg/lynx_pkg_w16.sv
W64=$T/pkg/lynx_pkg_w64.sv
DROPS="DROP_FIRST=40 DROP_EVERY=37 MAX_DROPS=4"
ONE="NQ=1 NWR=60 WR_BYTES=1024 DELAY=2000 SEND_B=0 TIMEOUT=600000"
run() {   # name pkg hls args...
    local n=$1 p=$2 h=$3; shift 3
    WORK=work_$n PKG=$p HLSV=$h bash ./run_roce_tb.sh "$@" > run_$n.out 2>&1
}
run T1 $W16 $OLD $ONE $DROPS &
run T2 $W64 $NEW $ONE $DROPS &
run T3 $W64 $OLD $ONE $DROPS &
run T4a $W16 $OLD $ONE $DROPS MEM_BITS=33 &
run T4b $W64 $NEW $ONE $DROPS MEM_BITS=33 &
run T5 $W64 $NEW NQ=2 NWR=2 WR_BYTES=65536 DELAY=500 TIMEOUT=400000 &
run T6 $W64 $NEW NQ=2 NWR=30 WR_BYTES=1024 DELAY=2000 SEND_B=0 TIMEOUT=600000 $DROPS &
wait
for n in T1 T2 T3 T4a T4b T5 T6; do
    echo "== $n: $(grep -m1 -oE 'TB [A-Z]+[^|]*' work_$n/xsim.log 2>/dev/null || tail -1 run_$n.out)"
    grep -m1 -E 'TB ' work_$n/xsim.log | grep -oE 'cycle [0-9]+'
    grep -m1 'check:' work_$n/xsim.log
    grep -m2 'counters' work_$n/xsim.log
    grep -m2 'DATA WRONG' work_$n/xsim.log
done
echo WINDOW_TESTS_DONE
