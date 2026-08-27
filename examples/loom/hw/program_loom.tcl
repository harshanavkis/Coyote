# Program the Coyote U280 by PART, not by position in the target list.
#
# clara carries two JTAG targets: a Versal V80 (arm_dap_0 + xcv80_1, PCIe
# 81:00.0) and the Alveo U280 Coyote runs on (xcu280_u55c_0, PCIe e1:00.0).
# The stock program_fpga.tcl opens a bare open_hw_target and takes device 0,
# which on the V80 target is arm_dap_0 - an ARM debug port, not an FPGA.
#
#   vivado -mode batch -notrace -source program_loom.tcl -tclargs <bit> [part] [-dry]
#
# part defaults to xcu280*; -dry reports the selection without programming.

set bitpath  [lindex $argv 0]
set part_pat "xcu280*"
set dry      [expr {[lsearch -exact $argv "-dry"] >= 0}]
if {[llength $argv] > 1 && [string index [lindex $argv 1] 0] ne "-"} {
    set part_pat [lindex $argv 1]
}

open_hw_manager
connect_hw_server -allow_non_jtag

set dev ""
foreach t [get_hw_targets] {
    current_hw_target $t
    open_hw_target
    foreach d [get_hw_devices] {
        if {[string match $part_pat [get_property PART $d]]} {
            set dev $d
            break
        }
    }
    if {$dev ne ""} break
    close_hw_target
}

if {$dev eq ""} {
    puts "ERROR: no device matching '$part_pat' on any JTAG target"
    exit 1
}
puts "SELECTED: $dev (part [get_property PART $dev]) on target [current_hw_target]"

if {$dry} {
    puts "DRY RUN: not programming"
    close_hw_target
    exit 0
}
if {![file exists $bitpath]} {
    puts "ERROR: bitstream not found: $bitpath"
    exit 1
}

current_hw_device $dev
refresh_hw_device -update_hw_probes false $dev
set_property PROBES.FILE {} $dev
set_property FULL_PROBES.FILE {} $dev
set_property PROGRAM.FILE $bitpath $dev
program_hw_devices $dev
refresh_hw_device $dev
puts "PROGRAMMED: $bitpath"
close_hw_target
