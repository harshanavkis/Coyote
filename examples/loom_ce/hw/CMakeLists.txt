# CMake configuration
cmake_minimum_required(VERSION 3.5)
set(CYT_DIR ${CMAKE_SOURCE_DIR}/../../../)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CYT_DIR}/cmake)
find_package(CoyoteHW REQUIRED)

project(example_loom_ce)
message("*** Loom copy engine (V80) [Hardware] ***")

# Host stream: the copy's destination writes (peer-to-peer into the U280's
# uwin) and the fence. Card memory: the copy's source (HBM), and where the
# user data window (written peer-to-peer by the U280) lands.
set(EN_STRM 1)
set(EN_MEM 1)
set(N_CARD_AXI 1)
set(EN_UWIN 1)
# The window lands straight in HBM (the receive side): the U280 writes into
# the exported window, and it is the last HBM block, not vFPGA logic
set(EN_UWIN_HBM 1)

set(N_REGIONS 1)

validation_checks_hw()

load_apps (
    VFPGA_C0_0 "src"
)

create_hw()
