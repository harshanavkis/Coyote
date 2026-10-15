# qdma_hw rev2 bitstream (project + IPs from create_rev2.tcl). Run from qdma_hw/rev2.
set R [pwd]
open_project $R/prj/rev2.xpr
if {[llength [get_files -quiet */src/qdma_hw2_top.sv]] == 0} {
  add_files -norecurse [list \
    $R/coyote/lynx_pkg.sv $R/coyote/axi_intf.sv $R/coyote/lynx_intf.sv \
    $R/coyote/qdma_wr_wrapper.sv $R/coyote/qdma_rd_wrapper.sv $R/coyote/qdma_reg_slice.sv \
    $R/src/qdma_gen.sv $R/src/bar4_ctl.sv $R/src/qdma_hw2_core.sv $R/src/qdma_hw2_top.sv]
  add_files -norecurse [list $R/coyote/lynx_macros.svh $R/coyote/assert_macros.svh $R/coyote/axi_macros.svh]
  set_property file_type {Verilog Header} [get_files *.svh]
  set_property is_global_include true [get_files *.svh]
  add_files -fileset constrs_1 -norecurse $R/src/rev2.xdc
}
set_property top qdma_hw2_top [get_filesets sources_1]
update_compile_order -fileset sources_1
reset_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 16
wait_on_run impl_1
puts "IMPL_STATUS [get_property STATUS [get_runs impl_1]] PROGRESS [get_property PROGRESS [get_runs impl_1]]"
puts "TIMING WNS [get_property STATS.WNS [get_runs impl_1]] TNS [get_property STATS.TNS [get_runs impl_1]] WHS [get_property STATS.WHS [get_runs impl_1]] THS [get_property STATS.THS [get_runs impl_1]]"
puts "BIT [glob -nocomplain [get_property DIRECTORY [get_runs impl_1]]/*.bit]"
puts "BUILD_DONE"
