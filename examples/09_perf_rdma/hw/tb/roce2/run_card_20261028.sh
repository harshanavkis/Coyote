#!/usr/bin/env bash
# 2026-10-28 06:00 (night): rdma_flow card-wide cap (branch window32card): window 32 per QP, 32 over all QPs. Both ways:
# 1 QP (expect the TB link rate, 12.12), 2 QPs (12.12, card total <= 32), 4 QPs x 64 KiB stress; every landed byte checked.
cd "$(dirname "$0")"; T=$(pwd); P=$T/pkg/lynx_pkg_w32c.sv
run() { local n=$1; shift; WORK=work_$n PKG=$P bash ./run_roce_tb.sh "$@" > run_$n.out 2>&1; }
run K1 NQ=1 NWR=2 WR_BYTES=1048576 DELAY=500 FLAG_BYTES=16 VERBOSE=0 TIMEOUT=400000 &
run K2 NQ=2 NWR=1 WR_BYTES=1048576 DELAY=500 FLAG_BYTES=16 VERBOSE=0 TIMEOUT=400000 &
run K4 NQ=4 NWR=3 WR_BYTES=65536 DELAY=500 FLAG_BYTES=16 VERBOSE=0 TIMEOUT=300000 &
wait
for n in K1 K2 K4; do
    echo "== $n: $(grep -a -m1 -oE 'TB [A-Z]+[^|]*\| cycle [0-9]+' work_$n/xsim.log 2>/dev/null || tail -n 1 run_$n.out)"
    grep -a -A20 -E 'TB (PASS|FAIL|DONE)' work_$n/xsim.log | grep -E 'check:|counters|RATE A|WIN  A'
done
echo CARD_DONE
