#!/usr/bin/env bash
# qdma_hw rev2 bitstream build (run create_rev2.tcl first, from hw/); run inside tmux: tmux new-session -d -s qdma_rev2_build <this script>
cd "$(dirname "$(readlink -f "$0")")/.." || exit 1
mkdir -p logs
log=logs/build_$(date +%Y%m%d_%H%M).log
ln -sf "$(basename "$log")" logs/build.log
xilinx-shell -c "vivado -mode batch -nojournal -log logs/build.vivado.log -source scripts/build_rev2.tcl" > "$log" 2>&1
echo "BUILD_SCRIPT_EXIT $?" >> "$log"
