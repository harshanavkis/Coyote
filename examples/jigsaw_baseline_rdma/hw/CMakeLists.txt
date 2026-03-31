# CMake configuration
cmake_minimum_required(VERSION 3.5)
set(CYT_DIR ${CMAKE_SOURCE_DIR}/../../../)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CYT_DIR}/cmake)
find_package(CoyoteHW REQUIRED)

project(example_jigsaw_combined_rdma)
message("*** Jigsaw Baseline RDMA + Perf RDMA [Hardware] ***")

# Infrastructure settings (Global for all vFPGAs)
set(EN_STRM 1)
set(N_STRM_AXI 2)
set(EN_RDMA 1)

# Number of vFPGAs (user applications)
set(N_REGIONS 2)

# Build with optimization, since timing closure with RDMA can be difficult
set(BUILD_OPT 1)

# Confirm that the selected options are allowed
validation_checks_hw()

# Load user applications from existing directories to avoid duplication:
# Region 0: perf_rdma
# Region 1: jigsaw_baseline
load_apps (
    VFPGA_C0_0 "../../09_perf_rdma/hw/src"
    VFPGA_C0_1 "../../jigsaw_baseline/hw/src"
)

# Create the hardware project
create_hw()
