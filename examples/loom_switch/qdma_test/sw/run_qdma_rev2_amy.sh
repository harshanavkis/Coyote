#!/usr/bin/env bash
# 2026-10-14 soft-QDMA test image rev2 on amy's U280 (qdma_hw/rev2: 10ee:9f3f, IP = the U280 static's QDMA config with
# pfch_cache_depth 64, C2H/H2C = the static's qdma_wr/rd_wrapper + reg slices (Coyote 7cb6023d) behind a shell stand-in,
# BAR4 = rev1 sink + control/status window at 0x0FF00000). Gate for the static rebuild: every QDMA path the static uses.
#   run_qdma_rev2_amy_20261014.sh prog     JTAG-program rev2, rescan; exit 2 = no 9f3f after rescan (warm reboot amy)
#   run_qdma_rev2_amy_20261014.sh test     insmod qdma_c2h_test.ko (setup as pci_qdma.c + C2H/H2C tests), dmesg -> $OUT
#   run_qdma_rev2_amy_20261014.sh unload   rmmod qdma_c2h_test (refused if the module reported a stuck transfer)
#   run_qdma_rev2_amy_20261014.sh bridge   PTDMA host -> BAR4 sink, 8 ch (rev1's 12.25 GB/s on this IP config) + counters
#   run_qdma_rev2_amy_20261014.sh restore  standard U280 (images.sh, V80 '-') + sanity discard
# Output: e1/20261014_qdma_rev2/ (OUT env overrides). Run on clara.
set -u
C=/scratch/harshanavkis/loom-proj/Coyote; source /home/harshanavkis/loom-experiments/images.sh
Q=/home/harshanavkis/loom-experiments/qdma_hw          # shared home: bitstream + helpers seen by amy
BIT=$Q/qdma_hw_rev2.bit; TCL=$Q/program_loom.tcl; CNT=$Q/axib_cnt.py
TK=$Q/qdma_c2h_test/qdma_c2h_test.ko
PK=/home/harshanavkis/loom-experiments/ptdma_bw/ptdma_bw.ko
CE=$C/examples/loom_ce/sw/build
NUMA="/nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1"
U=0000:e1:00.0
OUT=${OUT:-/home/harshanavkis/loom-experiments/e1/20261014_qdma_rev2}; mkdir -p $OUT
for s in $(ls -t /run/user/$(id -u)/vscode-ssh-auth-sock-* $HOME/.ssh/agent/s.* 2>/dev/null); do
    SSH_AUTH_SOCK=$s timeout 5 ssh-add -l >/dev/null 2>&1 && { export SSH_AUTH_SOCK=$s; break; }; done
on() { timeout ${T:-300} ssh -o ConnectTimeout=15 -n amy.dos.cit.tum.de "$1"; }

case ${1:-} in
prog)
    [ -f $BIT ] || { echo "no $BIT"; exit 1; }
    md5sum $BIT | tee $OUT/bit.md5
    on "who; lsmod | command grep -E '^(coyote|ptdma|qdma)'; lspci -nn -s $U" | tee $OUT/amy_before.txt
    on "lsmod | command grep -q '^qdma_c2h_test '" && { echo "qdma_c2h_test loaded on amy: unload first"; exit 1; }
    echo "== $(date +%T) amy: U280 off the bus, JTAG program $BIT"
    T=900 on "sudo pkill -u harshanavkis -x hw_server; lsmod | command grep -q '^coyote_driver ' && sudo rmmod coyote_driver; \
        [ -e /sys/bus/pci/devices/$U ] && echo 1 | sudo tee /sys/bus/pci/devices/$U/remove >/dev/null; \
        xilinx-shell -c \"export LM_LICENSE_FILE=/share/xilinx/Xilinx.lic; cd $Q && vivado -mode batch -nolog -nojournal -notrace -source $TCL -tclargs $BIT 'xcu280*'\" 2>&1 \
        | command grep -E 'SELECTED|PROGRAMMED|ERROR'" | tee $OUT/program.log
    grep -q '^PROGRAMMED:' $OUT/program.log || { echo "PROGRAMMING FAILED"; exit 1; }
    echo "== $(date +%T) link retrain 15 s, rescan"
    sleep 15
    on "echo 1 | sudo tee /sys/bus/pci/rescan >/dev/null; sleep 3; lspci -nn -s $U; sudo lspci -vv -s $U" > $OUT/after_prog.txt
    head -1 $OUT/after_prog.txt
    if grep -q '10ee:9f3f' $OUT/after_prog.txt && grep -q 'Region 4' $OUT/after_prog.txt && ! grep -q 'Region .*unassigned\|<unassigned>' $OUT/after_prog.txt; then
        grep -E 'LnkSta:|Region' $OUT/after_prog.txt | sed 's/^/   /'; echo "PROG OK"
    else
        grep -E 'LnkSta:|Region' $OUT/after_prog.txt | sed 's/^/   /'
        echo "NO 9f3f (or BARs unassigned) AFTER RESCAN: warm reboot needed"; exit 2
    fi ;;
test)
    VM=$(modinfo -F vermagic $TK | cut -d' ' -f1); KR=$(on "uname -r")
    [ "$VM" = "$KR" ] || { echo "qdma_c2h_test.ko is for $VM, amy runs $KR"; exit 1; }
    on "lspci -nn -s $U" | grep -q '10ee:9f3f' || { echo "amy's U280 is not the 9f3f test image"; exit 1; }
    on "lsmod | command grep -q '^qdma_c2h_test '" && { echo "qdma_c2h_test already loaded"; exit 1; }
    tag=run_$(date +%H%M%S)
    on "echo 'qdma_c2h_test_marker $tag' | sudo tee /dev/kmsg >/dev/null"
    echo "== $(date +%T) insmod qdma_c2h_test ${TARGS:-} ($tag)"
    T=180 on "sudo timeout 150 insmod $TK ${TARGS:-}" > $OUT/$tag.insmod 2>&1; echo "   insmod rc=$?"
    on "sudo dmesg" | awk -v t="qdma_c2h_test_marker $tag" 'index($0, t) {p=1} p && /qdma_c2h_test/' > $OUT/$tag.dmesg
    ln -sf $tag.dmesg $OUT/last.dmesg
    grep -E 'RESULT|RATE|DONE|TIMEOUT|STUCK|not granted|failed|magic' $OUT/$tag.dmesg | sed 's/^.*qdma_c2h_test: /   /'
    grep -q 'DONE fails 0$' $OUT/$tag.dmesg && echo "TEST PASS" || echo "TEST FAIL (see $OUT/$tag.dmesg)" ;;
unload)
    grep -q 'STUCK' $OUT/last.dmesg 2>/dev/null && { echo "module reported a STUCK transfer: not unloading"; exit 1; }
    T=60 on "sudo timeout 30 rmmod qdma_c2h_test; lsmod | command grep -c '^qdma_c2h_test '"; echo "   rmmod done" ;;
bridge)
    on "lsmod | command grep -q '^qdma_c2h_test '" && { echo "qdma_c2h_test loaded (holds BAR4): unload first"; exit 1; }
    on "sudo setpci -s $U COMMAND=0x0006"
    BAR4=$(on "sed -n 5p /sys/bus/pci/devices/$U/resource" | cut -d' ' -f1)
    echo "   BAR4 $BAR4"
    on "sudo python3 $CNT --bdf $U --clear" > /dev/null
    t=qr2_$(date +%s)
    T=180 on "sudo timeout 120 insmod $PK allow_write=1 dst_phys=$BAR4 nchan=8 numa=1 size=67108864 chunk=1048576 iters=3 tag=$t" \
        > $OUT/bridge.insmod 2>&1
    on "sudo dmesg" | awk -v t="start $t:" 'index($0, t) {p=1} p && /ptdma_bw/' > $OUT/bridge_ptdma.dmesg
    echo "   ptdma 8 ch: $(grep -E 'SUMMARY' $OUT/bridge_ptdma.dmesg | sed 's/.*GB\/s/GB\/s/')"
    on "lsmod | command grep -c '^ptdma_bw '" | grep -q '^0' || echo "   ! ptdma_bw STAYED LOADED"
    on "sudo python3 $CNT --bdf $U" > $OUT/bridge_cnt.txt
    grep -E '^(aw|w_last|bytes|gap4|gap5|gap6|gap_ge16|w_bubble|outst_max) |busy gaps|mean burst|over the span' $OUT/bridge_cnt.txt | sed 's/^/      /' ;;
restore)
    on "lsmod | command grep -q '^qdma_c2h_test '" && grep -q 'STUCK' $OUT/last.dmesg 2>/dev/null && \
        echo "   note: qdma_c2h_test still loaded with a stuck transfer; its remove() skips the QDMA and keeps its buffers"
    echo "== $(date +%T) amy: standard U280 $U280_LOOM (V80 '-')"
    for try in 1 2; do
        T=1800 $C/scripts/fpga/setup_loom_hosts.sh $U280_LOOM - amy > $OUT/restore_try$try.log 2>&1
        grep -q '^SETUP OK' $OUT/restore_try$try.log && break
    done
    grep -E '^SETUP|FAILED|warm reboot' $OUT/restore_try*.log | sed 's/^/   /'
    grep -q '^SETUP OK' $OUT/restore_try$try.log || { echo "RESTORE FAILED"; exit 1; }
    echo "== $(date +%T) sanity: ce_local --discard 3 16 MiB on amy (standard 10.63-10.64)"
    on "cd $CE && sudo timeout 60 $NUMA ./ce_local --discard 3 16777216" > $OUT/discard_sanity.log 2>&1
    echo "   rc=$? $(grep -E '^discard:|FAIL|rror' $OUT/discard_sanity.log)"
    echo "RESTORE DONE" ;;
*) sed -n '2,10p' $0; exit 1 ;;
esac
