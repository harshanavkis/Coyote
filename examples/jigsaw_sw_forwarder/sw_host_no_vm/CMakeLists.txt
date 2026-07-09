# CMake configuration
cmake_minimum_required(VERSION 3.5)
project(jigsaw_sw_forwarder_host_no_vm)

set(CYT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CYT_DIR}/cmake)
find_package(CoyoteSW REQUIRED)

message("*** Jigsaw SW Forwarder — Host (no VM) ***")

# Directory containing the executable(s) to be compiled
set(TARGET_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/")

# Create build targets and link against required libraries
set(EXEC test)
add_executable(${EXEC} ${TARGET_DIR}/main.cpp)
target_link_libraries(${EXEC} PUBLIC Coyote)

# Additional includes if needed; traces.hpp is shared with the
# jigsaw_host_controller no-VM harness so both replay identical events.
target_include_directories(${EXEC} PRIVATE
    ${CYT_DIR}/sw/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${CMAKE_CURRENT_SOURCE_DIR}/../../jigsaw_host_controller/sw_no_vm/src
)

target_compile_options(${EXEC} PUBLIC -std=c++17 -O3)
