# CMake configuration
cmake_minimum_required(VERSION 3.5)
set(CYT_DIR ${CMAKE_SOURCE_DIR}/../../../)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CYT_DIR}/cmake)
find_package(CoyoteHW REQUIRED)

project(example_loom_switch)
message("*** Loom switch, single data path [Hardware] ***")

# Infrastructure settings
set(EN_STRM 1)
# Two host streams: [0] the ingress's local writes, [1] loom_rx's landings
set(N_STRM_AXI 2)
set(EN_RDMA 1)
# The user data window: every byte of data enters here (loom_ingress)
set(EN_UWIN 1)

# Single vFPGA: every emulated XPU is a binding (a window) in it
set(N_REGIONS 1)

set(BUILD_OPT 0)

validation_checks_hw()

load_apps (
    VFPGA_C0_0 "src"
)

create_hw()
