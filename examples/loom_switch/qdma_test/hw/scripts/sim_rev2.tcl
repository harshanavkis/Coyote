# qdma_hw rev2 unit TB (tb/tb_core.sv) in xsim: core + Coyote wrappers + the static's reg-slice/ECC IPs, no QDMA IP.
# Run from qdma_hw/rev2: vivado -mode batch -source scripts/sim_rev2.tcl
set R [pwd]
create_project -force tbprj $R/tbprj -part xcu280-fsvh2892-2L-e
set_property target_language Verilog [current_project]
create_ip -name axis_register_slice -vendor xilinx.com -library ip -version 1.1 -module_name register_slice_qdma_data
set_property -dict [list CONFIG.TDATA_NUM_BYTES {75} CONFIG.REG_CONFIG {8} CONFIG.HAS_TKEEP {0} CONFIG.HAS_TLAST {1}] [get_ips register_slice_qdma_data]
create_ip -name axis_register_slice -vendor xilinx.com -library ip -version 1.1 -module_name register_slice_qdma_h2c_cmd
set_property -dict [list CONFIG.TDATA_NUM_BYTES {17} CONFIG.REG_CONFIG {8} CONFIG.HAS_TKEEP {0} CONFIG.HAS_TLAST {0}] [get_ips register_slice_qdma_h2c_cmd]
create_ip -name axis_register_slice -vendor xilinx.com -library ip -version 1.1 -module_name register_slice_qdma_c2h_cmd
set_property -dict [list CONFIG.TDATA_NUM_BYTES {13} CONFIG.REG_CONFIG {8} CONFIG.HAS_TKEEP {0} CONFIG.HAS_TLAST {0}] [get_ips register_slice_qdma_c2h_cmd]
create_ip -name ecc -vendor xilinx.com -library ip -version 2.0 -module_name qdma_ecc
set_property CONFIG.C_DATA_WIDTH {57} [get_ips qdma_ecc]
generate_target {simulation} [get_ips]

add_files -norecurse [list \
  $R/coyote/lynx_pkg.sv $R/coyote/axi_intf.sv $R/coyote/lynx_intf.sv \
  $R/coyote/qdma_wr_wrapper.sv $R/coyote/qdma_rd_wrapper.sv $R/coyote/qdma_reg_slice.sv \
  $R/src/qdma_gen.sv $R/src/bar4_ctl.sv $R/src/qdma_hw2_core.sv]
add_files -norecurse [list $R/coyote/lynx_macros.svh $R/coyote/assert_macros.svh $R/coyote/axi_macros.svh]
set_property file_type {Verilog Header} [get_files *.svh]
set_property is_global_include true [get_files *.svh]
add_files -fileset sim_1 -norecurse [list $R/tb/tb_core.sv \
  /scratch/harshanavkis/loom-proj/qdma_hw/ex/qdma_0_ex/imports/qdma_ecc_enc.sv]
set_property top qdma_hw2_core [get_filesets sources_1]
set_property top tb_core [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {all} -objects [get_filesets sim_1]
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
launch_simulation -simset sim_1 -mode behavioral
puts "SIM_DONE"
