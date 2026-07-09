# Loom switch: 512-bit AXI4-Stream FIFO buffering data read from the sender's
# buffer (LOCAL_READ) before it is written to the receiver's buffer (LOCAL_WRITE).
create_ip -name axis_data_fifo -vendor xilinx.com -library ip -version 2.0 -module_name axis_data_fifo_loom
set_property -dict [list \
    CONFIG.TDATA_NUM_BYTES {64} \
    CONFIG.FIFO_DEPTH {512} \
    CONFIG.HAS_TKEEP {1} \
    CONFIG.HAS_TLAST {1} \
] [get_ips axis_data_fifo_loom]
