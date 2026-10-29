# 2026-10-28: VCD of node A's HLS rocev2 FIFO flags (*_full_n, *_empty_n) for the whole run -> fifo.vcd (fifo_stats.py)
set top [lindex [get_scopes /*] 0]
foreach s [get_scopes /*] { if {[string match *tb_roce_two_node* $s]} { set top $s } }
current_scope $top
current_scope na/inst_roce/rocev2_inst/inst
set objs [concat [get_objects *_full_n] [get_objects *_empty_n]]
puts "VCD objects [llength $objs]"
open_vcd fifo.vcd
log_vcd $objs
run all
close_vcd
quit
