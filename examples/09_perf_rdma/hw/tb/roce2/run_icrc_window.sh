#!/usr/bin/env bash
# 2026-10-27: why did perf_rdma-w64 lose one-way streams on hardware (10-27, e5_ptdma/20261027_1016_fig9s_w64rdma) when
# window 16 never did? Hypothesis: icrc.sv drops the first word behind a packet with a full last word (8 B WRITE = 64 B);
# with a large window the sender's packets queue and follow each other back to back, so the packet behind a flag loses
# its header; no recovery (HLS D1/D2). One way, 4 x 256 KiB WRs + 8 B flags; then the proposed fix both ways.
#   I16  window 16, build HLS, current icrc         I64  window 64, 6-bit HLS, current icrc
#   I64f window 64, 6-bit HLS, fixed icrc            I32f window 32, 6-bit HLS, fixed icrc, BOTH ways
cd "$(dirname "$0")"
T=$(pwd)
B=/scratch/harshanavkis/loom-proj/Coyote/examples/09_perf_rdma/hw/build_oct23_rdma_stamp_noila
OLD=$B/example_09_perf_rdma_shell/example_09_perf_rdma.gen/sources_1/ip/rocev2_ip/hdl/verilog
NEW=$T/hls_offs6/rocev2_prj/solution1/syn/verilog
C="NQ=1 NWR=4 WR_BYTES=262144 DELAY=500 FLAG_BYTES=8 VERBOSE=0 TIMEOUT=300000"
run() { local n=$1 p=$2 h=$3 i=$4; shift 4; WORK=work_$n PKG=$p HLSV=$h ICRC=$i bash ./run_roce_tb.sh "$@" > run_$n.out 2>&1; }
run I16  $T/pkg/lynx_pkg_w16.sv $OLD /scratch/harshanavkis/loom-proj/Coyote/hw/hdl/network/rdma/icrc.sv SEND_B=0 $C &
run I64  $T/pkg/lynx_pkg_w64.sv $NEW /scratch/harshanavkis/loom-proj/Coyote/hw/hdl/network/rdma/icrc.sv SEND_B=0 $C &
run I64f $T/pkg/lynx_pkg_w64.sv $NEW $T/icrc_fixed.sv SEND_B=0 $C &
run I32f $T/pkg/lynx_pkg_w32.sv $NEW $T/icrc_fixed.sv $C &
wait
for n in I16 I64 I64f I32f; do
    echo "== $n: $(grep -a -m1 -oE 'TB [A-Z]+[^|]*\| cycle [0-9]+' work_$n/xsim.log 2>/dev/null || tail -1 run_$n.out)"
    grep -a -A20 -E 'TB (PASS|FAIL|DONE)' work_$n/xsim.log | grep -E 'TB |check:|RATE B|WIN  A|counters|probes'
done
echo ICRC_WINDOW_DONE
