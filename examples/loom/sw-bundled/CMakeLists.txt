# 6.2a: the bundled two-host binary (one process per host = daemon +
# app roles) and the FPGA-free peering smoke test. Reuses the 5.x
# headers from ../sw/src unchanged.
cmake_minimum_required(VERSION 3.5)
project(example_loom_bundled_sw)

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
include_directories("${CMAKE_SOURCE_DIR}/../sw/src")

# The bundled host binary (execution: hardware only; links everywhere)
add_executable(loom_host ${TARGET_DIR}/loom_host.cpp)
target_link_libraries(loom_host PUBLIC Coyote pthread)

# FPGA-free peering protocol test (no cThread; runs anywhere)
add_executable(test_peering ${TARGET_DIR}/test_peering.cpp)
target_link_libraries(test_peering PUBLIC Coyote pthread)
