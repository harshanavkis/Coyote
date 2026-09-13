# CMake configuration
cmake_minimum_required(VERSION 3.5)
set(CYT_DIR ${CMAKE_SOURCE_DIR}/../../../)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CYT_DIR}/cmake)
find_package(CoyoteHW REQUIRED)

project(example_loom)
message("*** Loom switch prototype [Hardware] ***")

# Infrastructure settings
set(EN_STRM 1)
# Two host streams, as perf_rdma builds: [0] is the engine's (stores, DMA
# writes, fences, the pull), [1] is loom_rx's landing stream for incoming
# rdma writes. Each dest has its own queue, credits and FIFO in the shell
# (local_credits_host_wr), so the receive path no longer shares the
# engine's stream, its arbiter slot or its EN_CRED_LOCAL accounting.
set(N_STRM_AXI 2)
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
