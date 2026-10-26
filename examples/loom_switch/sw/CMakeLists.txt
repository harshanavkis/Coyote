cmake_minimum_required(VERSION 3.5)
project(example_loom_switch_sw)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(EN_SIM "Enable simulation mode for Coyote" OFF)
if(EN_SIM)
    add_subdirectory(../../../sim/sw ${CMAKE_BINARY_DIR}/coyote)
else()
    add_subdirectory(../../../sw ${CMAKE_BINARY_DIR}/coyote)
endif()

set(TARGET_DIR "${CMAKE_SOURCE_DIR}/src/")

# G2: host CPU writes through the user data window
add_executable(uwin_probe ${TARGET_DIR}/uwin_probe.cpp)
target_link_libraries(uwin_probe PUBLIC Coyote pthread)

# Gets: the CPU reads a window bound to the far host's export
add_executable(get_bench ${TARGET_DIR}/get_bench.cpp)
target_link_libraries(get_bench PUBLIC Coyote pthread)

# E2: one-way latency of CPU stores through Loom, host to host (ping-pong)
add_executable(uwin_pingpong ${TARGET_DIR}/uwin_pingpong.cpp)
target_link_libraries(uwin_pingpong PUBLIC Coyote pthread)

# PTDMA (the host CPU's DMA engines) as Loom's copy engine, host DRAM -> window -> far host
add_executable(uwin_ptdma ${TARGET_DIR}/uwin_ptdma.cpp)
target_link_libraries(uwin_ptdma PUBLIC Coyote pthread)

# Push ceiling: packets/s of 64 B CPU writes into the uwin (per-packet or per-byte limit?)
add_executable(uwin_wc64 ${TARGET_DIR}/uwin_wc64.cpp)
target_link_libraries(uwin_wc64 PUBLIC Coyote pthread)
target_compile_options(uwin_wc64 PRIVATE -mavx2)

# Per-packet stamps (stamp_rec, CSR 224-240): arm / counts / dump, Loom and perf_rdma images
add_executable(stamps ${TARGET_DIR}/stamps.cpp)
target_link_libraries(stamps PUBLIC Coyote pthread)

# Where a window read's time goes: one size, PTDMA or CPU loads, card time line + reads in flight
add_executable(read_probe ${TARGET_DIR}/read_probe.cpp)
target_link_libraries(read_probe PUBLIC Coyote pthread)
