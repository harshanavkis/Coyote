#!/usr/bin/env bash
# run_roce3_tb.sh (copy of run_roce_tb.sh, 2026-10-28): THREE RoCE nodes behind a switch model (tb_roce_three_node.sv), default
# RTL = the perf_rdma-w32 image (BUILD build_oct27_rdma_w32, glue of branch window32 in RTL=rtl_w32 for icrc.sv and rdma_flow.sv).
# Two RoCE nodes back to back in XSIM (tb_roce_two_node.sv): the roce_stack RTL of a perf_rdma build (BUILD, default
# build_oct23_rdma_stamp_noila: its shell package, its HLS rocev2 RTL and its generated FIFO IPs) and the glue from this
# checkout (hw/hdl/network/rdma). Parameters go to xelab as -generic_top, e.g.
#   ./run_roce_tb.sh NQ=2 NWR=2 WR_BYTES=1048576 SEND_A=1 SEND_B=1
# On a receive stall the bench stops and dump_state.tcl prints every HLS FIFO's full/empty flags; the HLS deadlock
# detector (rocev2_top_hls_deadlock_detector.vh) prints its own report if the HLS processes deadlock.
# 2026-10-26: PKG = another shell package (e.g. pkg/lynx_pkg_w64.sv: rdma_flow window 64), HLSV = another HLS rocev2 RTL
# directory (e.g. hls_offs6/rocev2_prj/solution1/syn/verilog: the retransmitter's slot fields 6 bits wide).
set -u
cd "$(dirname "$0")"
if ! command -v xvlog >/dev/null 2>&1; then
    exec xilinx-shell "$(pwd)/$(basename "$0")" "$@"
fi
C=$(cd ../../../../.. && pwd)
H=$C/hw/hdl
BUILD=${BUILD:-$C/examples/09_perf_rdma/hw/build_oct27_rdma_w32}
IP=$BUILD/example_09_perf_rdma_shell/example_09_perf_rdma.gen/sources_1/ip
PKG=${PKG:-$BUILD/example_09_perf_rdma_shell/hdl/lynx_pkg.sv}
HLSV=${HLSV:-$IP/rocev2_ip/hdl/verilog}
RTL=${RTL:-$(pwd)/rtl_w32}
ICRC=${ICRC:-$RTL/icrc.sv}   # 2026-10-27: e.g. icrc_fixed.sv (branch roce-fixes)
[ -f "$PKG" ] || { echo "ERROR: $PKG not found"; exit 1; }
W=${WORK:-work}
mkdir -p $W && cd $W
GLUE="$H/common/queues/fifo.sv $H/common/queues/queue_meta.sv $H/common/queues/meta_queue.sv $H/common/queues/queue_stream.sv
      $H/common/ram/sp_ram_nc.sv $H/network/rdma/buffer_fifo.sv $ICRC $H/network/rdma/read_request_trimmer.sv
      $H/network/rdma/ack_gap_enforcer.sv ${FLOW:-$RTL/rdma_flow.sv} $H/network/rdma/rdma_mux_retrans.sv
      $H/network/rdma/roce_stack.sv"
IPS="$IP/axis_data_fifo_512_cc_tx/axis_data_fifo_512_cc_tx_sim_netlist.v $IP/axis_data_fifo_512/axis_data_fifo_512_sim_netlist.v
     $(ls $IP/axis_data_fifo_meta_*/axis_data_fifo_meta_*_sim_netlist.v)"
cp -f $HLSV/*.dat . 2>/dev/null
cp -f $HLSV/*.vh . 2>/dev/null
{
xvlog $IPS $HLSV/*.v $IP/rocev2_ip/synth/rocev2_ip.v $XILINX_VIVADO/data/verilog/src/glbl.v -i $HLSV &&
xvlog -sv -d SYNTHESIS $PKG $H/pkg/axi_intf.sv $H/pkg/lynx_intf.sv $GLUE ../tb_roce_three_node.sv -i $H/pkg -i $H
} > xvlog.log 2>&1 || { grep -E 'ERROR' xvlog.log | head -20; echo "COMPILE FAILED (see $(pwd)/xvlog.log)"; exit 1; }
GEN=""; for kv in "$@"; do GEN="$GEN -generic_top $kv"; done
xelab -debug typical -timescale 1ns/1ps -L unisims_ver -L unimacro_ver -L secureip $GEN tb_roce_three_node glbl -s sim > xelab.log 2>&1 \
    || { grep -E 'ERROR' xelab.log | head -20; echo "ELAB FAILED (see $(pwd)/xelab.log)"; exit 1; }
xsim sim -tclbatch ${XTCL:-../dump_state3.tcl} > xsim.log 2>&1   # XTCL: another batch script (2026-10-28: vcd_fifo.tcl)
grep -E 'node [0-9]|sw drops|progress|probes|retrans |counters|RATE|WIN |LAT |STALL|check:|DATA WRONG|set up|POWERUP|TB |DEADLOCK|deadlock|unexpected|FIFO |state ' xsim.log | head -400
grep -q 'TB PASS' xsim.log && echo "PASS" || echo "NOT PASS (see $(pwd)/xsim.log)"
