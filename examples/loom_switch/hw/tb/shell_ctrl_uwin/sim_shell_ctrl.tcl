# Build design_ctrl from hw/bd/ultrascale_plus/cr_ctrl.tcl with EN_UWIN=1 and
# simulate it with tb_shell_ctrl_uwin. Arg: number of regions (1 or 2).
set n_reg  [lindex $argv 0]
set tb_dir [file dirname [file normalize [info script]]]
set cyt    [file normalize $tb_dir/../../../../..]

create_project -force sc [pwd]/prj -part xcu280-fsvh2892-2L-e

array set cfg [list n_reg $n_reg en_avx 1 en_uwin 1 aclk_f 250 nclk_f 250 uclk_f 250]
source $cyt/hw/bd/ultrascale_plus/cr_ctrl.tcl
cr_bd_design_ctrl ""

set bd [get_files design_ctrl.bd]
generate_target simulation $bd

add_files -fileset sim_1 $tb_dir/tb_shell_ctrl_uwin.sv
if {$n_reg == 2} { set_property verilog_define {N2} [get_filesets sim_1] }
set_property top tb_shell_ctrl_uwin [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {all} -objects [get_filesets sim_1]
launch_simulation
close_sim
