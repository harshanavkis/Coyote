# Power-up like the FPGA: every flop the glue leaves without a reset starts at 0 (in simulation it would start as X;
# icrc.sv's pipeline registers made the HLS output register slice desynchronise). Then run tb_roce_two_node; when it
# stops ($stop on a stall or timeout), print for both stacks every HLS stream FIFO that is full or holds data, and the
# FSM state of every HLS process, then quit.
set top ""
foreach s [get_scopes /*] { if {[string match *tb_roce_two_node* $s]} { set top $s } }
run 1ns
set nz 0
foreach node {na nb} {
    current_scope $top
    current_scope $node/inst_roce
    foreach o [get_objects -r *] {
        if {[catch {set v [get_value -radix bin $o]}]} { continue }
        if {[string first x [string tolower $v]] >= 0 && [string length $v] <= 4096} {
            if {[catch {set_value -radix bin $o [string repeat 0 [string length $v]]}] == 0} { incr nz }
        }
    }
}
puts "POWERUP zeroed $nz X-valued glue signals"
run all
if {$top ne ""} {
    foreach node {na nb} {
        current_scope $top
        current_scope $node/inst_roce/rocev2_inst/inst
        foreach o [get_objects *_full_n] { if {[get_value $o] == 0} { puts "FIFO full  $node [lindex [split $o /] end]" } }
        foreach o [get_objects *_empty_n] { if {[get_value $o] == 1} { puts "FIFO data  $node [lindex [split $o /] end]" } }
    }
}
quit
