# rerun tb_core with the sim-only qdma_rd_wrapper copy (tb/simfix) in place of coyote/qdma_rd_wrapper.sv
open_project [pwd]/tbprj/tbprj.xpr
set f [get_files -quiet */coyote/qdma_rd_wrapper.sv]
if {[llength $f]} { remove_files $f; add_files -norecurse [pwd]/tb/simfix/qdma_rd_wrapper.sv }
update_compile_order -fileset sim_1
launch_simulation -simset sim_1 -mode behavioral
puts "SIM_DONE"
