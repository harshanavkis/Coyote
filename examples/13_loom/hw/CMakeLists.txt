cmake_minimum_required(VERSION 3.5)
set(CYT_DIR ${CMAKE_SOURCE_DIR}/../../../)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CYT_DIR}/cmake)
find_package(CoyoteHW REQUIRED)

project(example_13_loom)
message("*** Coyote Example 13: Loom Hardware ***")

set(EN_STRM 1)
set(N_STRM_AXI 2)
set(EN_RDMA 1)
set(BUILD_OPT 1)   

set(N_REGIONS 1)

validation_checks_hw()

load_apps (
    VFPGA_C0_0 "src"
)

create_hw()
