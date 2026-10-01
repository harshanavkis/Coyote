# The ingress FIFO in front of loom_rx: 4096 beats (256 KiB, 64 packets), so
# the sender's ack window (16 packets, up to ~60) fits in it even while the
# host write path is slower than the wire - otherwise the RoCE stack loses
# packets and the sender retransmits (go-back-N).
create_ip -name axis_data_fifo -vendor xilinx.com -library ip -version 2.0 -module_name axis_data_fifo_rx4096
set_property -dict [list CONFIG.TDATA_NUM_BYTES {64} CONFIG.FIFO_DEPTH {4096} CONFIG.HAS_TKEEP {1} CONFIG.HAS_TLAST {1} CONFIG.FIFO_MEMORY_TYPE {ultra}] [get_ips axis_data_fifo_rx4096]
