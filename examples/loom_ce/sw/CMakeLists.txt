cmake_minimum_required(VERSION 3.5)
project(example_loom_ce_sw)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_subdirectory(../../../sw ${CMAKE_BINARY_DIR}/coyote)

set(TARGET_DIR "${CMAKE_SOURCE_DIR}/src/")
# The switch's CSR page and window programming
include_directories("${CMAKE_SOURCE_DIR}/../../loom_switch/sw/src")

# G4, local half: V80 HBM -> U280 uwin -> host buffer
add_executable(ce_local ${TARGET_DIR}/ce_local.cpp)
target_link_libraries(ce_local PUBLIC Coyote pthread)

# G4: V80 HBM -> U280 uwin -> RoCE -> far U280 -> far host buffer
add_executable(ce_remote ${TARGET_DIR}/ce_remote.cpp)
target_link_libraries(ce_remote PUBLIC Coyote pthread)

# The V80's write bandwidth to host memory vs peer-to-peer to the U280 (example 07 on the V80)
add_executable(p2p_bw ${TARGET_DIR}/p2p_bw.cpp)
target_link_libraries(p2p_bw PUBLIC Coyote pthread)

# Bisecting a host reset: V80-only steps (CSR reads, copy to host memory)
add_executable(p2p_bw_u280 ${TARGET_DIR}/p2p_bw_u280.cpp)
target_link_libraries(p2p_bw_u280 PUBLIC Coyote pthread)

add_executable(loom_csr ${TARGET_DIR}/loom_csr.cpp)
target_link_libraries(loom_csr PUBLIC Coyote pthread)

add_executable(v80_bisect ${TARGET_DIR}/v80_bisect.cpp)
target_link_libraries(v80_bisect PUBLIC Coyote pthread)
