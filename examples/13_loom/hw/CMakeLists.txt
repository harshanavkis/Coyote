cmake_minimum_required(VERSION 3.5)
set(CYT_DIR ${CMAKE_SOURCE_DIR}/../../../)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CYT_DIR}/cmake)
find_package(CoyoteHW REQUIRED)

project(example_13_loom)
message("*** Coyote Example 13: Loom 2-thread switch [Hardware] ***")

# Host streams; FPGA-initiated LOCAL_READ (stream 0) + LOCAL_WRITE (stream 1).
# Two streams: stream index = thread identity (sender on 0, receiver on 1).
set(EN_STRM 1)
set(N_STRM_AXI 2)

set(N_REGIONS 1)

validation_checks_hw()

load_apps (
    VFPGA_C0_0 "src"
)

create_hw()
