#!/usr/bin/env bash
# Re-place and re-route a U280 static build (BUILD_STATIC=1) that missed timing
# in the XDMA PCIe core, then cut its static region out as the locked
# checkpoint shell-only builds link against. Runs in a tmux session.
#
#   scripts/fpga/close_static_timing.sh <build_dir> <out_dir> [--fg]
#
#   build_dir  a finished static build: its checkpoints/shell_opted.dcp is
#              re-placed (e.g. examples/loom_switch/hw/build_oct05_rd32)
#   out_dir    results: close.log, shell_routed.dcp, timing reports,
#              cyt_top.bit (only if the whole design meets timing) and
#              static_routed_locked_u280.dcp (always, with the static
#              region's clocks summarized: use it as STATIC_PATH only if
#              they meet timing)
#   --fg       run in the foreground instead of tmux session static_<out_dir name>
#
# Why: the first U280 static rebuild (build_oct05_rd32) placed the two ends of
# a flip-flop pair in the XDMA's 512b completion interface far apart
# (transceiver TXOUTCLK -> xclk, -0.427 ns); phys_opt and re-routing did not
# move it. Placing with those crossings over-constrained by 0.6 ns keeps the
# pair close; the over-constraint is removed before routing. On
# build_oct05_rd32: xclk +0.076, PCIe transceiver clocks +0.067, ~5.5 h.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/vivado_env.sh"
[ $# -ge 2 ] || { sed -n '2,18p' "$0" >&2; exit 1; }
BUILD=$(readlink -f "$1"); mkdir -p "$2"; OUT=$(readlink -f "$2")
FG=0; [ "${3:-}" = --fg ] && FG=1
DCP=$BUILD/checkpoints/shell_opted.dcp
[ -f "$DCP" ] || { echo "no $DCP: not a finished static build?" >&2; exit 1; }
SESSION=static_$(basename "$OUT")

cat > "$OUT/close.tcl" <<TCL
set_param general.maxThreads 32
open_checkpoint $DCP
set gt [get_clocks -quiet -filter {NAME =~ GTYE4_CHANNEL_TXOUTCLK*}]
puts "GT clocks: [llength \$gt]"
set_clock_uncertainty -setup 0.6 -from \$gt -to [get_clocks xclk]
set_clock_uncertainty -setup 0.6 -from [get_clocks xclk] -to \$gt
place_design -directive ExtraNetDelay_high
phys_opt_design -directive AggressiveExplore
write_checkpoint -force $OUT/placed.dcp
set_clock_uncertainty -setup 0.0 -from \$gt -to [get_clocks xclk]
set_clock_uncertainty -setup 0.0 -from [get_clocks xclk] -to \$gt
report_timing_summary -no_detailed_paths -file $OUT/placed_timing.rpt
route_design -directive AggressiveExplore
phys_opt_design -directive AggressiveExplore
set wns [get_property SLACK [get_timing_paths -max_paths 1 -setup]]
set whs [get_property SLACK [get_timing_paths -max_paths 1 -hold]]
report_timing_summary -max_paths 5 -file $OUT/final_timing.rpt
report_route_status -file $OUT/route_status.rpt
write_checkpoint -force $OUT/shell_routed.dcp
# The static region's clocks: what a shell build linked against it inherits
foreach c {xclk pipe_clk} {
    set p [get_timing_paths -quiet -max_paths 1 -setup -to [get_clocks -quiet \$c]]
    if {[llength \$p]} { puts "STATIC_CLOCK \$c [get_property SLACK \$p]" }
}
set p [get_timing_paths -quiet -max_paths 1 -setup -to \$gt]
if {[llength \$p]} { puts "STATIC_CLOCK GTYE4_TXOUTCLK* [get_property SLACK \$p]" }
puts "RESULT_WNS \$wns WHS \$whs"
if {\$wns >= 0 && \$whs >= 0} {
    write_bitstream -force -no_partial_bitfile $OUT/cyt_top.bit
    write_debug_probes -no_partial_ltxfile -force $OUT/cyt_top.ltx
    puts "BITSTREAM WRITTEN"
} else {
    puts "TIMING NOT MET: no bitstream"
}
update_design -cell inst_shell -black_box
lock_design -level routing
write_checkpoint -force $OUT/static_routed_locked_u280.dcp
puts "STATIC CHECKPOINT WRITTEN"
TCL

CMD="$(vivado_env "$(vivado_version_for u280)") cd $OUT && vivado -mode batch -nojournal -log close.log -source close.tcl > close.out 2>&1; echo vivado exit \$? >> close.out; grep -E '^(GT clocks|STATIC_CLOCK|RESULT_WNS|BITSTREAM|TIMING NOT|STATIC CHECKPOINT|ERROR)' close.log"
echo "re-placing $DCP into $OUT"
if [ $FG = 1 ]; then
    xilinx-shell -c "$CMD"
else
    tmux has-session -t "$SESSION" 2>/dev/null && { echo "tmux session $SESSION exists; not starting another" >&2; exit 1; }
    printf '%s\n' "$CMD" > "$OUT/.close_cmd.sh"
    tmux new-session -d -s "$SESSION" "xilinx-shell -c 'bash $OUT/.close_cmd.sh' 2>&1 | tee $OUT/summary.txt"
    echo "started tmux session $SESSION (~5.5 h); result lines in $OUT/summary.txt, log $OUT/close.log"
fi
