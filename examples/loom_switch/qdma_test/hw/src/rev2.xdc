# qdma_hw rev2 (U280): as rev1 (qdma_hw/ex/.../xilinx_qdma_pcie_x1y1.xdc) without the example's waivers.
create_clock -name sys_clk -period 10 [get_ports sys_clk_p]
# U280 PCIe refclk = Coyote hw/constraints/u280/static/impl/u280_static_pcie.xdc; no PERST pin (reset held high)
set_property PACKAGE_PIN AR15 [get_ports sys_clk_p]
set_property PACKAGE_PIN AR14 [get_ports sys_clk_n]
set_property CONFIG_VOLTAGE 1.8 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
