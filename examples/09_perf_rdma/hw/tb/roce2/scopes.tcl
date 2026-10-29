run 2000ns
foreach s [get_scopes /*] { puts "TOP $s" }
foreach s [get_scopes -r /*] { if {[string match *rocev2* $s]} { puts "SC $s" } }
quit
