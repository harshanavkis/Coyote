######################################################################################
# Example 12: AES-GCM over RDMA (split encrypt / decrypt)
#
# ILA on the two crypto stream crossings, so a hung datapath can be told
# apart from a stalled one on hardware.
######################################################################################

create_ip -name ila -vendor xilinx.com -library ip -version 6.2 -module_name ila_perf_rdma_aes
set_property -dict [list CONFIG.C_NUM_OF_PROBES {16} CONFIG.C_EN_STRG_QUAL {1} CONFIG.ALL_PROBE_SAME_MU_CNT {2}] [get_ips ila_perf_rdma_aes]
