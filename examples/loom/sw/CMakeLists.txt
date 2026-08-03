# CMake configuration
cmake_minimum_required(VERSION 3.5)
project(example_loom_sw)

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

set(EXEC test)
add_executable(${EXEC} ${TARGET_DIR}/main.cpp)
target_link_libraries(${EXEC} PUBLIC Coyote)

# 5.1a role-split demo (client/server structure, in-process transport)
add_executable(roles ${TARGET_DIR}/roles.cpp)
target_link_libraries(roles PUBLIC Coyote)

# 5.3: same flow over a real Unix socket to a threaded loomd
add_executable(roles_sock ${TARGET_DIR}/roles_sock.cpp)
target_link_libraries(roles_sock PUBLIC Coyote pthread)

# 5.3: standalone control daemon (hardware mode)
add_executable(loomd ${TARGET_DIR}/loomd_main.cpp)
target_link_libraries(loomd PUBLIC Coyote pthread)

# 5.3: FPGA-free protocol test (no cThread; runs anywhere)
add_executable(test_loomd_proto ${TARGET_DIR}/test_loomd_proto.cpp)
target_link_libraries(test_loomd_proto PUBLIC Coyote pthread)

find_package(Boost REQUIRED COMPONENTS program_options)
target_link_libraries(${EXEC} PUBLIC Boost::program_options)
target_link_libraries(roles PUBLIC Boost::program_options)
