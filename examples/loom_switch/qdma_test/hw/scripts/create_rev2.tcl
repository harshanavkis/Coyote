# qdma_hw rev2: QDMA test image whose IP config and C2H/H2C logic are the U280 soft-QDMA static's (Coyote 7cb6023d).
# QDMA 5.0 = cr_pci_qdma.tcl's property list (incl. pfch_cache_depth 64) except pf0_device_id 9F3F (no Coyote driver
# binds it) and en_ext_ch_gt_drp (static-only DRP clocking). Reg slices + qdma_ecc = scripts/ip_inst/pci_infrastructure.tcl.
# Run from qdma_hw/rev2: vivado -mode batch -source scripts/create_rev2.tcl
set R [pwd]
create_project -force rev2 $R/prj -part xcu280-fsvh2892-2L-e
set_property target_language Verilog [current_project]

create_ip -name qdma -vendor xilinx.com -library ip -version 5.0 -module_name qdma_0
set_property -dict [list \
  CONFIG.mode_selection {Advanced} \
  CONFIG.pcie_blk_locn {PCIE4C_X1Y1} \
  CONFIG.select_quad {GTY_Quad_227} \
  CONFIG.pl_link_cap_max_link_speed {8.0_GT/s} \
  CONFIG.pl_link_cap_max_link_width {X16} \
  CONFIG.axi_data_width {512_bit} \
  CONFIG.axisten_freq {250} \
  CONFIG.axi_id_width {4} \
  CONFIG.dma_intf_sel_qdma {AXI_Stream_with_Completion} \
  CONFIG.dsc_byp_mode {Descriptor_bypass_and_internal} \
  CONFIG.pf0_device_id {9F3F} \
  CONFIG.pf0_bar0_type_qdma {AXI_Lite_Master} \
  CONFIG.pf0_bar0_64bit_qdma {true} \
  CONFIG.pf0_bar0_prefetchable_qdma {true} \
  CONFIG.pf0_bar0_scale_qdma {Megabytes} \
  CONFIG.pf0_bar0_size_qdma {1} \
  CONFIG.pf0_bar2_type_qdma {DMA} \
  CONFIG.pf0_bar2_64bit_qdma {true} \
  CONFIG.pf0_bar4_enabled_qdma {true} \
  CONFIG.pf0_bar4_type_qdma {AXI_Bridge_Master} \
  CONFIG.pf0_bar4_64bit_qdma {true} \
  CONFIG.pf0_bar4_prefetchable_qdma {true} \
  CONFIG.pf0_bar4_scale_qdma {Megabytes} \
  CONFIG.pf0_bar4_size_qdma {256} \
  CONFIG.PF0_MSIX_CAP_TABLE_SIZE_qdma {01F} \
  CONFIG.c_m_axi_num_write {32} \
  CONFIG.pfch_cache_depth {64} \
] [get_ips qdma_0]
foreach p {mode_selection pcie_blk_locn select_quad pl_link_cap_max_link_speed pl_link_cap_max_link_width axi_data_width axisten_freq axi_id_width dma_intf_sel_qdma dsc_byp_mode pf0_device_id pf0_bar0_type_qdma pf0_bar0_size_qdma pf0_bar2_type_qdma pf0_bar4_type_qdma pf0_bar4_size_qdma PF0_MSIX_CAP_TABLE_SIZE_qdma c_m_axi_num_write c_m_axi_num_read pfch_cache_depth num_queues} {
  puts "QCFG $p = [get_property CONFIG.$p [get_ips qdma_0]]"
}

create_ip -name axis_register_slice -vendor xilinx.com -library ip -version 1.1 -module_name register_slice_qdma_data
set_property -dict [list CONFIG.TDATA_NUM_BYTES {75} CONFIG.REG_CONFIG {8} CONFIG.HAS_TKEEP {0} CONFIG.HAS_TLAST {1}] [get_ips register_slice_qdma_data]
create_ip -name axis_register_slice -vendor xilinx.com -library ip -version 1.1 -module_name register_slice_qdma_h2c_cmd
set_property -dict [list CONFIG.TDATA_NUM_BYTES {17} CONFIG.REG_CONFIG {8} CONFIG.HAS_TKEEP {0} CONFIG.HAS_TLAST {0}] [get_ips register_slice_qdma_h2c_cmd]
create_ip -name axis_register_slice -vendor xilinx.com -library ip -version 1.1 -module_name register_slice_qdma_c2h_cmd
set_property -dict [list CONFIG.TDATA_NUM_BYTES {13} CONFIG.REG_CONFIG {8} CONFIG.HAS_TKEEP {0} CONFIG.HAS_TLAST {0}] [get_ips register_slice_qdma_c2h_cmd]
create_ip -name ecc -vendor xilinx.com -library ip -version 2.0 -module_name qdma_ecc
set_property CONFIG.C_DATA_WIDTH {57} [get_ips qdma_ecc]

generate_target {instantiation_template synthesis simulation} [get_ips]
puts "CREATE_IP_DONE"
