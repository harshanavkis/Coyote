#!/usr/bin/env bash
# Check the U280 static block design (hw/bd/ultrascale_plus/cr_pci.tcl) builds,
# validates and gives the XDMA bypass master 32 reads in flight (Vivado 2023.2,
# as the U280 builds). run.sh qdma: the same for the soft-QDMA static
# (cr_pci_qdma.tcl, STATIC_DMA=qdma).
set -u
cd "$(dirname "$0")"
if ! command -v vivado >/dev/null 2>&1; then
    exec xilinx-shell "$(pwd)/$(basename "$0")" "$@"
fi
CHECK=check_static_pci.tcl; [ "${1:-}" = qdma ] && CHECK=check_static_pci_qdma.tcl
rm -rf work && mkdir -p work && cd work
vivado -mode batch -nojournal -source ../$CHECK > vivado.log 2>&1
grep -E '^(ok|FAIL|info|STATIC PCI)' vivado.log
grep -E '^ERROR' vivado.log | head -10
grep -q 'STATIC PCI PASS' vivado.log
