# G1 probe: a V80 vFPGA writes into a U280 vFPGA's BAR, peer-to-peer
cmake_minimum_required(VERSION 3.5)
project(loom_p2p_probe)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_subdirectory(../../../sw ${CMAKE_BINARY_DIR}/coyote)

add_executable(p2p_probe ${CMAKE_SOURCE_DIR}/src/main.cpp)
target_link_libraries(p2p_probe PUBLIC Coyote)
