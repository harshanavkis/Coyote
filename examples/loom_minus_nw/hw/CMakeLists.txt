cmake_minimum_required(VERSION 3.5)
set(CYT_DIR ${CMAKE_SOURCE_DIR}/../../../)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CYT_DIR}/cmake)
find_package(CoyoteHW REQUIRED)

project(example_loom_minus_nw)
message("*** Loom minus NW: both ends on one vFPGA, no RoCE stack ***")

# EN_RDMA 0 is the whole point. It drops the RoCE stack, so the receive path
# is fed by loom_engine directly and nothing between the two ends can
# retransmit, reorder or drop. It also forces EN_DCARD 0, which sidesteps the
# u280 DDR/HBM reference-clock conflict that killed build_aug3.
set(EN_STRM 1)
set(N_STRM_AXI 1)
set(EN_RDMA 0)
set(N_REGIONS 1)
set(BUILD_OPT 1)

# The four loom_*.sv are examples/loom's, not copies kept in step by hand.
# They cannot be symlinks: cr_user.tcl does `file copy -force` of src/hdl into
# the build tree, which copies a symlink VERBATIM, and a relative target does
# not resolve from there - synthesis then fails with "module 'loom_ctrl' not
# found". Materialising them at configure time gives cr_user.tcl real files
# while keeping examples/loom the only place they are edited.
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../../loom/hw/src/hdl/
     DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/src/hdl)

validation_checks_hw()
load_apps (
    VFPGA_C0_0 "src"
)
create_hw()
