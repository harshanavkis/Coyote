# CMake configuration
cmake_minimum_required(VERSION 3.5)
set(CYT_DIR ${CMAKE_SOURCE_DIR}/../../../)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CYT_DIR}/cmake)
find_package(CoyoteHW REQUIRED)

project(example_loom)
message("*** Loom switch prototype [Hardware] ***")

# Infrastructure settings
set(EN_STRM 1)
set(N_STRM_AXI 1)
set(EN_RDMA 1)

# Single vFPGA: all emulated XPU processes attach to it as cThreads
set(N_REGIONS 1)

# Build with optimization, since timing closure with RDMA can be difficult
set(BUILD_OPT 1)

validation_checks_hw()

load_apps (
    VFPGA_C0_0 "src"
)

create_hw()
