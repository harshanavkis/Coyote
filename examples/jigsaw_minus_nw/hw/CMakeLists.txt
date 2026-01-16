# CMake configuration
cmake_minimum_required(VERSION 3.5)
set(CYT_DIR ${CMAKE_SOURCE_DIR}/../../../)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CYT_DIR}/cmake)
find_package(CoyoteHW REQUIRED)

project(example_jigsaw_minus_nw)
message("*** Jigsaw minus NW ***")

# Enables host stream
set(EN_STRM 1)

# Number of vFPGAs (user applications)
set(N_REGIONS 1)

# Build with optimizations for easier timing closure
set(BUILD_OPT 1)

# Confirm that the selected options are allowed
validation_checks_hw()

# Load a user application in Configuration #0, Region #0
load_apps (
    VFPGA_C0_0 "src"
)

# Create the hardware project
create_hw()
