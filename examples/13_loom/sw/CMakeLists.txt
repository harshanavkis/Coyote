cmake_minimum_required(VERSION 3.5)
project(example_13_loom)

set(CYT_DIR ${CMAKE_SOURCE_DIR}/../../../)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CYT_DIR}/cmake)
find_package(CoyoteSW REQUIRED)

# Build target: client (source node) or server (destination node).
# Local-only tests run from the client without --server_ip.
set(INSTANCE "client" CACHE STRING "Loom software build target: client or server")
if(INSTANCE STREQUAL "server")
    set(TARGET_DIR "${CMAKE_SOURCE_DIR}/src/server")
    message("*** Coyote Example 13: Loom Server [Software] ***")
else()
    set(TARGET_DIR "${CMAKE_SOURCE_DIR}/src/client")
    message("*** Coyote Example 13: Loom Client [Software] ***")
endif()
include_directories("${CMAKE_SOURCE_DIR}/src/include")

set(EXEC test)
add_executable(${EXEC} ${TARGET_DIR}/main.cpp)
target_link_libraries(${EXEC} PUBLIC Coyote)
target_link_directories(${EXEC} PUBLIC /usr/local/lib)

find_package(Boost REQUIRED COMPONENTS program_options)
target_link_libraries(${EXEC} PUBLIC Boost::program_options)
