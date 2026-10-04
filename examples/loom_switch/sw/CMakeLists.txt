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

# Gets: the CPU reads the far host's export through a get window (two writes)
add_executable(get_bench ${TARGET_DIR}/get_bench.cpp)
target_link_libraries(get_bench PUBLIC Coyote pthread)
