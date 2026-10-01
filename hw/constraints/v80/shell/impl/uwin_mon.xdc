# uwin_mon (EN_UWIN_HBM): its axi_main counter snapshot crosses from xclk to
# aclk. The bus is held for 256 xclk cycles and uwin_hbm captures it only
# after the toggle (raised after the snapshot) has been synchronized, so the
# bus needs no timing; the toggle goes into a 3-flop synchronizer.
set_false_path -quiet -from [get_cells -quiet -hier -filter {NAME =~ *inst_uwin_mon/snap_reg*}] -to [get_cells -quiet -hier -filter {NAME =~ *inst_uwin_hbm/main_ctr_reg*}]
set_max_delay -quiet -datapath_only 3.0 -from [get_cells -quiet -hier -filter {NAME =~ *inst_uwin_mon/snap_tgl_reg*}] -to [get_cells -quiet -hier -filter {NAME =~ *inst_uwin_hbm/main_tgl_s_reg[0]*}]
