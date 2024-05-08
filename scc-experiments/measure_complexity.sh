#!/usr/bin/env bash

cd ..

# Host code
scc --format json sw/examples/coyote_host_app_template/main.cpp
scc --format json sw/examples/vfpio_host_app_template/main.cpp
###############################################################

# FPGA code: cyt_examples is for coyote style applications, vfpio otherwise

scc --format json hw/hdl/operators/examples/service_aes
scc --format json hw/hdl/operators/examples/cyt_examples/service_aes

scc --format json hw/hdl/operators/examples/sha256
scc --format json hw/hdl/operators/examples/cyt_examples/sha256

scc --format json hw/hdl/operators/examples/nw
scc --format json hw/hdl/operators/examples/cyt_examples/nw

scc --format json hw/hdl/operators/examples/matmul
scc --format json hw/hdl/operators/examples/cyt_examples/matmul

scc --format json hw/hdl/operators/examples/keccak
scc --format json hw/hdl/operators/examples/cyt_examples/keccak

scc --format json hw/hdl/operators/examples/rng
scc --format json hw/hdl/operators/examples/cyt_examples/rng

# scc --format json hw/hdl/operators/examples/gzip
# scc --format json hw/hdl/operators/examples/cyt_examples/gzip

# scc --format json hw/hdl/operators/examples/hls4ml
# scc --format json hw/hdl/operators/examples/cyt_examples/hls4ml

scc --format json hw/hdl/operators/examples/md5
scc --format json hw/hdl/operators/examples/cyt_examples/md5
###############################################################
