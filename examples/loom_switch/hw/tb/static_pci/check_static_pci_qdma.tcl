# Build design_static from hw/bd/ultrascale_plus/cr_pci_qdma.tcl (the soft-QDMA U280 static, STATIC_DMA=qdma),
# validate it, generate the QDMA and check what the driver and the window rely on: device 913F, descriptor
# bypass both ways, and 32 writes and 32 reads in flight on the AXI bridge master (BAR4, axi_main; the QDMA
# derives C_M_AXI_NUM_READ from c_m_axi_num_write, as XDMA does).
set cyt [file normalize [file dirname [file normalize [info script]]]/../../../../..]

create_project -force sp [pwd]/prj -part xcu280-fsvh2892-2L-e

array set cfg [list fdev u280 n_hchan 3 static_dma qdma]
source $cyt/hw/bd/ultrascale_plus/cr_pci_qdma.tcl
cr_bd_design_static ""
open_bd_design [get_files design_static.bd]
validate_bd_design

set errs 0
proc expect {name got want} {
    global errs
    if {$got eq $want} { puts "ok   $name = $got" } else { puts "FAIL $name = $got, expected $want"; incr errs }
}
set q [get_bd_cells qdma_0]
expect "qdma_0 c_m_axi_num_write" [get_property CONFIG.c_m_axi_num_write $q] 32
expect "qdma_0 pf0_device_id" [get_property CONFIG.pf0_device_id $q] 913F
expect "qdma_0 dsc_byp_mode" [get_property CONFIG.dsc_byp_mode $q] Descriptor_bypass_and_internal
expect "qdma_0 pfch_cache_depth" [get_property CONFIG.pfch_cache_depth $q] 64
expect "axi_main NUM_READ_OUTSTANDING" [get_property CONFIG.NUM_READ_OUTSTANDING [get_bd_intf_ports axi_main]] 32

generate_target synthesis [get_files design_static.bd]
set xci [lindex [glob [pwd]/prj/sp.srcs/sources_1/bd/design_static/ip/design_static_qdma_0_0/*.xci] 0]
set f [open $xci r]; set t [read $f]; close $f
foreach {p want} {C_M_AXI_NUM_READ 32 C_M_AXI_NUM_WRITE 32 PF0_DEVICE_ID 0x913F DSC_BYPASS_RD 1 DSC_BYPASS_WR 1 MDMA_PFCH_CACHE_DEPTH 64} {
    if {[regexp "\"$p\": \\\[ \\\{ \"value\": \"(\[^\"\]+)\"" $t -> v]} {
        expect "generated qdma $p" $v $want
    } else { puts "FAIL $p not found in $xci"; incr errs }
}

if {$errs == 0} { puts "STATIC PCI PASS" } else { puts "STATIC PCI FAIL: $errs" }
