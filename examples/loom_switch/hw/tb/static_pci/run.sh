#!/usr/bin/env bash
# Check the U280 static block design (hw/bd/ultrascale_plus/cr_pci.tcl) builds,
# validates and gives the XDMA bypass master 32 reads in flight (Vivado 2023.2,
# as the U280 builds).
set -u
cd "$(dirname "$0")"
if ! command -v vivado >/dev/null 2>&1; then
    exec xilinx-shell "$(pwd)/$(basename "$0")" "$@"
fi
rm -rf work && mkdir -p work && cd work
vivado -mode batch -nojournal -source ../check_static_pci.tcl > vivado.log 2>&1
grep -E '^(ok|FAIL|info|STATIC PCI)' vivado.log
grep -E '^ERROR' vivado.log | head -10
grep -q 'STATIC PCI PASS' vivado.log
