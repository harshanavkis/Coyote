#!/usr/bin/env bash
# Simulate the generated shell_ctrl block design with EN_UWIN=1 (XSIM, U280
# part, Vivado 2023.2 like the U280 builds). `./run.sh 2` for two regions.
set -u
cd "$(dirname "$0")"
if ! command -v vivado >/dev/null 2>&1; then
    exec xilinx-shell "$(pwd)/$(basename "$0")" "$@"
fi
N=${1:-1}
rm -rf work_n$N && mkdir -p work_n$N && cd work_n$N
vivado -mode batch -nojournal -source ../sim_shell_ctrl.tcl -tclargs $N > vivado.log 2>&1
LOG=$(find . -name simulate.log | head -1)
if [ -n "$LOG" ] && grep -q "TB PASS" "$LOG"; then
    grep -E '^(ok|reset|TB PASS)' "$LOG"
    exit 0
fi
[ -n "$LOG" ] && grep -E 'FAIL|ERROR|Error' "$LOG" | head -30
grep -E 'ERROR' vivado.log | head -20
echo "FAIL (see $(pwd)/vivado.log${LOG:+ and $LOG})"
exit 1
