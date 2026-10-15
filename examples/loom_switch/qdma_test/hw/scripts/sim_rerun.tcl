# rerun tb_core in the existing tbprj (after sim_rev2.tcl created it)
open_project [pwd]/tbprj/tbprj.xpr
launch_simulation -simset sim_1 -mode behavioral
puts "SIM_DONE"
