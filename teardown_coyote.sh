#!/usr/bin/env bash

sudo rmmod coyote_driver
# sudo bash sw/util/hot_reset.sh "e1:00.0"

echo 1 | sudo tee /sys/bus/pci/devices/0000:e1:00.0/remove

echo 1 | sudo tee /sys/bus/pci/rescan

# xilinx-shell "vivado -mode batch -nolog -nojournal -source program_fpga.tcl -tclargs $1"

