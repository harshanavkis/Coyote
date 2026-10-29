#!/usr/bin/env bash
cd /scratch/harshanavkis/loom-proj/Coyote/examples/09_perf_rdma/hw/tb/roce2/work_smoke && xsim sim -tclbatch ../scopes.tcl > scopes.log 2>&1
