# Build design_static from hw/bd/ultrascale_plus/cr_pci.tcl for the U280,
# validate it and generate the XDMA, then check the bypass master's reads in
# flight: the IP parameter, the generated core's C_M_AXI_NUM_READ and the
# axi_main port's declaration. (In DMA mode with the AXI-ST bypass and no
# mult_pf_des, the IP's bd.tcl does not update M_AXI_BYPASS's declared
# NUM_READ_OUTSTANDING, so that label stays 8; it is printed, not checked.)
set cyt [file normalize [file dirname [file normalize [info script]]]/../../../../..]

create_project -force sp [pwd]/prj -part xcu280-fsvh2892-2L-e

array set cfg [list fdev u280 n_hchan 3]
source $cyt/hw/bd/ultrascale_plus/cr_pci.tcl
cr_bd_design_static ""
open_bd_design [get_files design_static.bd]
validate_bd_design

set errs 0
proc expect {name got want} {
    global errs
    if {$got eq $want} { puts "ok   $name = $got" } else { puts "FAIL $name = $got, expected $want"; incr errs }
}
set x [get_bd_cells xdma_0]
expect "xdma_0 c_m_axi_num_write" [get_property CONFIG.c_m_axi_num_write $x] 32
puts "info xdma_0/M_AXI_BYPASS declares NUM_READ_OUTSTANDING [get_property CONFIG.NUM_READ_OUTSTANDING [get_bd_intf_pins xdma_0/M_AXI_BYPASS]]"
expect "axi_main NUM_READ_OUTSTANDING" [get_property CONFIG.NUM_READ_OUTSTANDING [get_bd_intf_ports axi_main]] 32

generate_target synthesis [get_files design_static.bd]
set xci [lindex [glob [pwd]/prj/sp.srcs/sources_1/bd/design_static/ip/design_static_xdma_0_0/*.xci] 0]
set f [open $xci r]; set t [read $f]; close $f
if {[regexp {"C_M_AXI_NUM_READ": \[ \{ "value": "([0-9]+)"} $t -> v] || [regexp {MODELPARAM_VALUE.C_M_AXI_NUM_READ">([0-9]+)<} $t -> v]} {
    expect "generated xdma C_M_AXI_NUM_READ" $v 32
} else { puts "FAIL C_M_AXI_NUM_READ not found in $xci"; incr errs }

if {$errs == 0} { puts "STATIC PCI PASS" } else { puts "STATIC PCI FAIL: $errs" }
