######################################################################################
# LOOM LOCAL CHANGE: the U280 static with the soft QDMA 5.0 in place of XDMA 4.1 (STATIC_DMA=qdma).
#
# Why: XDMA's bypass (target) path holds its CQ intake one cycle per 256 B write (6 cycles per write =
# 10.66 GB/s, simulated and measured); the soft QDMA's AXI bridge master took 256 B writes at 5.19 cycles
# (12.33 GB/s while busy, measured on amy's U280 with an always-ready sink, 2026-10-13).
#
# Same PCIe block, quad and link as cr_pci.tcl (PCIE4C_X1Y1, GTY_Quad_227, Gen3 x16, 512 bit at 250 MHz)
# and the same BAR order, so the driver's BAR indices hold: BAR0 AXI-Lite master -> axi_cnfg (static
# config), BAR2 DMA (queues, MSI-X), BAR4 256 MB AXI bridge master -> axi_main (the shell). Device ID 913F
# (the XDMA static is 903F). Data path as on the V80 (CPM5 QDMA): AXI-ST H2C/C2H with descriptor bypass,
# H2C queue status (qsts_out) and C2H status (axis_c2h_drop) to the qdma_* wrappers in static_top. The
# completion, credit, descriptor-status and bypass-out interfaces are exported and tied off in static_top.
# Prefetch cache 64 deep: every C2H queue in simple bypass holds a prefetch tag, and at the default 16 the QDMA
# hands out 15 (measured 2026-10-14: qid 271+ got no tag, 0x140C read 0, TRQ_ERR 0x8).
######################################################################################

proc cr_bd_design_static { parentCell } {
  upvar #0 cfg cnfg
  set design_name design_static
  create_bd_design $design_name

  if { $parentCell eq "" } {
     set parentCell [get_bd_cells /]
  }
  set parentObj [get_bd_cells $parentCell]
  set oldCurInst [current_bd_instance .]
  current_bd_instance $parentObj

  # ---------------------------------------------------------------------------------
  # Ports
  # ---------------------------------------------------------------------------------
  set axi_cnfg [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:aximm_rtl:1.0 axi_cnfg ]
  set_property -dict [ list CONFIG.ADDR_WIDTH {32} CONFIG.DATA_WIDTH {32} CONFIG.PROTOCOL {AXI4LITE} ] $axi_cnfg

  set axi_main [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:aximm_rtl:1.0 axi_main ]
  set_property -dict [ list \
    CONFIG.ADDR_WIDTH {64} \
    CONFIG.DATA_WIDTH {512} \
    CONFIG.HAS_BRESP {1} \
    CONFIG.HAS_BURST {1} \
    CONFIG.HAS_CACHE {1} \
    CONFIG.HAS_LOCK {1} \
    CONFIG.HAS_PROT {1} \
    CONFIG.HAS_QOS {1} \
    CONFIG.HAS_REGION {1} \
    CONFIG.HAS_RRESP {1} \
    CONFIG.HAS_WSTRB {1} \
    CONFIG.NUM_READ_OUTSTANDING {32} \
    CONFIG.NUM_WRITE_OUTSTANDING {32} \
    CONFIG.PROTOCOL {AXI4} \
    CONFIG.READ_WRITE_MODE {READ_WRITE} \
  ] $axi_main

  set pcie_clk [ create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 pcie_clk ]
  set_property -dict [ list CONFIG.FREQ_HZ {100000000} ] $pcie_clk
  set pcie_x16 [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:pcie_7x_mgt_rtl:1.0 pcie_x16 ]

  # QDMA data path (names as the QDMA's own interfaces)
  create_bd_intf_port -mode Master -vlnv xilinx.com:display_eqdma:m_axis_h2c_rtl:1.0 m_axis_h2c
  create_bd_intf_port -mode Slave  -vlnv xilinx.com:display_eqdma:s_axis_c2h_rtl:1.0 s_axis_c2h
  create_bd_intf_port -mode Slave  -vlnv xilinx.com:display_eqdma:s_axis_c2h_cmpt_rtl:1.0 s_axis_c2h_cmpt
  create_bd_intf_port -mode Slave  -vlnv xilinx.com:display_eqdma:h2c_byp_in_st_rtl:1.0 h2c_byp_in_st
  create_bd_intf_port -mode Slave  -vlnv xilinx.com:display_eqdma:c2h_byp_in_st_rtl:1.0 c2h_byp_in_st
  create_bd_intf_port -mode Master -vlnv xilinx.com:display_eqdma:h2c_byp_out_rtl:1.0 h2c_byp_out
  create_bd_intf_port -mode Master -vlnv xilinx.com:display_eqdma:c2h_byp_out_rtl:1.0 c2h_byp_out
  create_bd_intf_port -mode Master -vlnv xilinx.com:display_eqdma:qsts_out_rtl:1.0 qsts_out
  create_bd_intf_port -mode Master -vlnv xilinx.com:display_eqdma:axis_c2h_drop_rtl:1.0 axis_c2h_drop
  create_bd_intf_port -mode Master -vlnv xilinx.com:display_eqdma:tm_dsc_sts_rtl:1.0 tm_dsc_sts
  create_bd_intf_port -mode Slave  -vlnv xilinx.com:display_eqdma:dsc_crdt_in_rtl:1.0 dsc_crdt_in
  create_bd_intf_port -mode Slave  -vlnv xilinx.com:display_eqdma:usr_irq_rtl:1.0 usr_irq

  # Clocks and resets (as cr_pci.tcl)
  set xresetn [ create_bd_port -dir O -type rst xresetn ]
  set xclk [ create_bd_port -dir O -type clk xclk ]
  set_property -dict [ list \
    CONFIG.ASSOCIATED_BUSIF {m_axis_h2c:s_axis_c2h:s_axis_c2h_cmpt:h2c_byp_in_st:c2h_byp_in_st:h2c_byp_out:c2h_byp_out:qsts_out:axis_c2h_drop:tm_dsc_sts:dsc_crdt_in:usr_irq:axi_main:axi_cnfg} \
    CONFIG.ASSOCIATED_RESET {xresetn:sresetn:eos_resetn} \
  ] $xclk
  set presetn [ create_bd_port -dir O -type rst presetn ]
  set pclk [ create_bd_port -dir O -type clk pclk ]
  set_property -dict [ list CONFIG.ASSOCIATED_RESET {presetn} CONFIG.FREQ_HZ {200000000} ] $pclk
  set dresetn [ create_bd_port -dir O -type rst dresetn ]
  set dclk [ create_bd_port -dir O -type clk dclk ]
  set_property -dict [ list CONFIG.ASSOCIATED_RESET {dresetn} CONFIG.FREQ_HZ {100000000} ] $dclk
  set sresetn [ create_bd_port -dir O -type rst sresetn ]
  set lckresetn [ create_bd_port -dir O -type rst lckresetn ]
  create_bd_port -dir I -type rst eos_resetn
  set_property CONFIG.POLARITY ACTIVE_LOW [get_bd_ports eos_resetn]

  # ---------------------------------------------------------------------------------
  # Cells
  # ---------------------------------------------------------------------------------
  set clk_wiz_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz:6.0 clk_wiz_0 ]
  set_property -dict [list \
   CONFIG.PRIM_IN_FREQ {100.000} \
   CONFIG.USE_LOCKED {true} \
   CONFIG.USE_RESET {false} \
   CONFIG.CLKIN1_JITTER_PS {40.0} \
   CONFIG.MMCM_DIVCLK_DIVIDE {5} \
   CONFIG.MMCM_CLKFBOUT_MULT_F {24.000} \
   CONFIG.MMCM_CLKIN1_PERIOD {4.000} \
   CONFIG.MMCM_CLKIN2_PERIOD {10.0} \
   CONFIG.CLKOUT1_JITTER {134.506} \
   CONFIG.CLKOUT1_PHASE_ERROR {154.678} \
  ] $clk_wiz_0
  set_property -dict [list CONFIG.OPTIMIZE_CLOCKING_STRUCTURE_EN {true} CONFIG.CLKOUT2_USED {true} CONFIG.NUM_OUT_CLKS {2} CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {200.000} CONFIG.CLKOUT2_REQUESTED_OUT_FREQ {100.000}] $clk_wiz_0

  set proc_sys_reset_1 [ create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_1 ]
  set proc_sys_reset_p [ create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_p ]
  set proc_sys_reset_d [ create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_d ]
  set proc_sys_reset_x [ create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_x ]
  create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic:2.0 util_vector_logic_0
  set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {and} CONFIG.LOGO_FILE {data/sym_orgate.png}] [get_bd_cells util_vector_logic_0]
  set util_ds_buf [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf util_ds_buf ]
  set_property -dict [ list CONFIG.C_BUF_TYPE {IBUFDSGTE} ] $util_ds_buf
  create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 xlconstant_0
  set_property -dict [list CONFIG.CONST_VAL {1}] [get_bd_cells xlconstant_0]
  create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 xlconstant_1
  set_property -dict [list CONFIG.CONST_VAL {1}] [get_bd_cells xlconstant_1]

  set qdma_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:qdma:5.0 qdma_0 ]
  set_property -dict [ list \
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
    CONFIG.pf0_device_id {913F} \
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
    CONFIG.en_ext_ch_gt_drp {true} \
  ] $qdma_0

  # ---------------------------------------------------------------------------------
  # Interface connections
  # ---------------------------------------------------------------------------------
  connect_bd_intf_net [get_bd_intf_ports pcie_clk] [get_bd_intf_pins util_ds_buf/CLK_IN_D]
  connect_bd_intf_net [get_bd_intf_ports pcie_x16] [get_bd_intf_pins qdma_0/pcie_mgt]
  connect_bd_intf_net [get_bd_intf_ports axi_cnfg] [get_bd_intf_pins qdma_0/M_AXI_LITE]
  connect_bd_intf_net [get_bd_intf_ports axi_main] [get_bd_intf_pins qdma_0/M_AXI_BRIDGE]
  foreach i {m_axis_h2c s_axis_c2h s_axis_c2h_cmpt h2c_byp_in_st c2h_byp_in_st h2c_byp_out c2h_byp_out qsts_out axis_c2h_drop tm_dsc_sts dsc_crdt_in usr_irq} {
    connect_bd_intf_net [get_bd_intf_ports $i] [get_bd_intf_pins qdma_0/$i]
  }

  # ---------------------------------------------------------------------------------
  # Port connections (clocks and resets as cr_pci.tcl)
  # ---------------------------------------------------------------------------------
  connect_bd_net [get_bd_pins util_ds_buf/IBUF_DS_ODIV2] [get_bd_pins qdma_0/sys_clk]
  connect_bd_net [get_bd_pins util_ds_buf/IBUF_OUT] [get_bd_pins qdma_0/sys_clk_gt]
  # No PERST on the U280 (as the XDMA static): the core comes out of reset after configuration
  connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins qdma_0/sys_rst_n] [get_bd_pins qdma_0/soft_reset_n]

  connect_bd_net [get_bd_pins proc_sys_reset_p/ext_reset_in] [get_bd_pins xlconstant_1/dout]
  connect_bd_net [get_bd_pins proc_sys_reset_p/slowest_sync_clk] [get_bd_pins clk_wiz_0/clk_out1]
  connect_bd_net [get_bd_ports presetn] [get_bd_pins proc_sys_reset_p/peripheral_aresetn]

  connect_bd_net [get_bd_pins proc_sys_reset_d/ext_reset_in] [get_bd_pins xlconstant_1/dout]
  connect_bd_net [get_bd_pins proc_sys_reset_d/slowest_sync_clk] [get_bd_pins clk_wiz_0/clk_out2]
  connect_bd_net [get_bd_ports dresetn] [get_bd_pins proc_sys_reset_d/peripheral_aresetn]

  connect_bd_net [get_bd_pins util_vector_logic_0/Op1] [get_bd_pins xlconstant_1/dout]
  connect_bd_net [get_bd_pins util_vector_logic_0/Res] [get_bd_pins proc_sys_reset_x/ext_reset_in]
  connect_bd_net [get_bd_ports xresetn] [get_bd_pins proc_sys_reset_x/peripheral_aresetn]
  connect_bd_net [get_bd_ports eos_resetn] [get_bd_pins util_vector_logic_0/Op2]

  connect_bd_net [get_bd_pins qdma_0/ext_ch_gt_drpclk] [get_bd_pins clk_wiz_0/clk_in1]
  connect_bd_net [get_bd_ports pclk] [get_bd_pins clk_wiz_0/clk_out1]
  connect_bd_net [get_bd_ports dclk] [get_bd_pins clk_wiz_0/clk_out2]
  connect_bd_net [get_bd_ports lckresetn] [get_bd_pins clk_wiz_0/locked]

  connect_bd_net [get_bd_ports xclk] [get_bd_pins qdma_0/axi_aclk]
  connect_bd_net [get_bd_pins proc_sys_reset_x/slowest_sync_clk] [get_bd_pins qdma_0/axi_aclk]
  connect_bd_net [get_bd_pins proc_sys_reset_1/slowest_sync_clk] [get_bd_pins qdma_0/axi_aclk]
  connect_bd_net [get_bd_pins qdma_0/axi_aresetn] [get_bd_pins proc_sys_reset_1/ext_reset_in]
  connect_bd_net [get_bd_ports sresetn] [get_bd_pins proc_sys_reset_1/peripheral_aresetn]

  # ---------------------------------------------------------------------------------
  # Address segments
  # ---------------------------------------------------------------------------------
  assign_bd_address -offset 0x00000000 -range 256M [get_bd_addr_segs axi_main/Reg]
  assign_bd_address -offset 0x00000000 -range 1M [get_bd_addr_segs axi_cnfg/Reg]

  current_bd_instance $oldCurInst
  validate_bd_design
  save_bd_design
  close_bd_design $design_name

  return 0
}
# End of cr_bd_design_static()
