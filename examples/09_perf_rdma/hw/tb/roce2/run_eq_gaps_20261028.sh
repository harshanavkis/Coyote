#!/usr/bin/env bash
# 2026-10-28: icrc_fast (in_first) vs icrc_fixed: (1) both with a gap after each packet, (2) icrc_fast back to back, valid
# every cycle (icrc_fixed keeps its gaps), (3) negative control: (2) with one CRC448 term dropped -> must MISMATCH
cd "$(dirname "$0")"
{
echo "== (1) GAP new 1 ref 1"; GEN="-generic_top GAP=1 -generic_top GAP_REF=1" bash ./run_icrc_eq.sh 2>&1 | grep -E "EQ (PASS|MISM|X|TIME)"
echo "== (2) GAP new 0 ref 1"; GEN="-generic_top GAP=0 -generic_top GAP_REF=1" bash ./run_icrc_eq.sh 2>&1 | grep -E "EQ (PASS|MISM|X|TIME)"
cp icrc_fast.sv icrc_fast.sv.ctl_keep
sed -i 's/stage_crc448_crc\[5\] <= d\[447-446\] ^ /stage_crc448_crc[5] <= /' icrc_fast.sv
cmp -s icrc_fast.sv icrc_fast.sv.ctl_keep && echo "control NOT applied"
echo "== (3) control (CRC448 bit 5 corrupted), GAP new 0 ref 1"; GEN="-generic_top GAP=0 -generic_top GAP_REF=1" bash ./run_icrc_eq.sh 2>&1 | grep -E "EQ (PASS|MISM|X|TIME)"
mv icrc_fast.sv.ctl_keep icrc_fast.sv
echo EQGAPS_DONE
} > eq_gaps.out 2>&1
