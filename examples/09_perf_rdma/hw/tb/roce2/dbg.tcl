run all
set top ""
foreach s [get_scopes /*] { if {[string match *tb_roce_two_node* $s]} { set top $s } }
current_scope $top
current_scope nb/inst_roce/rocev2_inst/inst
foreach o [get_objects *_full_n] { set v [get_value $o]; if {$v == 0} { puts "NBFULL [lindex [split $o /] end]" } }
foreach o [get_objects *_empty_n] { set v [get_value $o]; if {$v == 1} { puts "NBDATA [lindex [split $o /] end]" } }
foreach f {rx_process2dropLengthFifo_U rx_ip2udpMetaFifo_U rx_ip2udpFifo_U rx_udpMetaFifo_U rx_udp2shiftFifo_U rx_udp2ibFifo_U rx_crc2ipFifo_U rx_process2dropFifo_U} {
    foreach o [get_objects $f/*] { set n [lindex [split $o /] end]; if {[string match *full_n* $n] || [string match *empty_n* $n] || [string match *mOutPtr* $n] || [string match *num_data* $n]} { puts "F $f $n = [get_value -radix dec $o]" } }
}
quit
