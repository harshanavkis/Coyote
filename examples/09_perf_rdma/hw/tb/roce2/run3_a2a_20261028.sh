#!/usr/bin/env bash
# 2026-10-28 ~21:00: the 3-rank a2a 1 MiB pattern (rdma_colln --acked-step --flag-in-data, perf_rdma-w32 RTL) on three
# nodes behind the switch model: 20 steps of 349,568 B per peer; A no jitter, B-D random post jitter (CPU), longer host gap.
cd "$(dirname "$0")"
run() { local n=$1; shift; WORK=work3_$n bash ./run_roce3_tb.sh "$@" > run3_$n.out 2>&1; }
C="NWR=20 WR_BYTES=349568 TIMEOUT=3000000"
run A $C POSTGAP=2000 &
run B $C POSTGAP=2000 JITTER=4000 SEED=1 &
run C $C POSTGAP=10000 JITTER=10000 SEED=2 &
run D $C POSTGAP=500 JITTER=2000 SEED=3 &
wait
for n in A B C D; do echo "== $n: $(grep -a -m1 -E 'TB (PASS|FAIL|DONE)' work3_$n/xsim.log | cut -c1-260)"; done
echo A2A3_DONE
