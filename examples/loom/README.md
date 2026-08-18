# Loom example (vFPGA switch prototype)

A single vFPGA per host acts as the switch. All emulated-XPU processes and
the control daemon attach to it as cThreads. Changes are confined to vFPGA
user logic and user-space software; the shell and driver stay stock.

- [HW-DESIGN.md](HW-DESIGN.md) — components, the stock-shell gates with
  their source evidence, GPU-equivalence argument, deployment topology.
- [WORKFLOW.md](WORKFLOW.md) — worked example of all five flows with
  concrete addresses.

> Every commit that runs anything documents how to run it in "Running"
> below. Keep it current.

## Ctrl-region layout (64 KB, AXI4-Lite)

- `0x0000-0x0FFF` — CSR page: window-table programming, DMA descriptor
  staging (incl. per-descriptor fence VA) + trigger, RO debug counters
  (words 32-41), stage cycle counters for T3 (words 48-63). Register map
  in the `loom_ctrl.sv` header.
- `0x1000-0xFFFF` — aperture: 15 windows of 4 KB. Window = `addr[15:12]`,
  offset = `addr[11:0]`. Every write beat is captured as a posted
  small-write transaction; every read is a held-open non-posted one.

## Status

| Phase | Content | State |
|---|---|---|
| 1 | loom_ctrl (CSR page + aperture capture + order FIFO), loom_table, block TBs | done |
| 2 | loom_engine (store + DMA branches, completion) + block TB | done |
| 3 | loom_rx + block TB | done |
| 4 | vfpga_top wiring + Coyote integration sim (EN_SIM) | done |
| 4.5 | test hardening: tb_loom_top (arbitration), engine/ctrl corner cases, extended integration sim, Python RDMA TX test | done |
| 5.0 | per-descriptor completion (fence VA in descriptor, CE semaphore-release model) | done |
| 5.1a | single-process software: client/server role split (OrchClient iface, in-process transport) | done |
| 5.2 | aperture reads, local path (READ order-FIFO entry, held-open AXI-Lite read, 64B line pull + lane select, poison on invalid) | done |
| 5.2b | rdma wire-message format: 64B header ⟨op·len·vaddr⟩, RETH = staging vaddr (CSR 14), loom_rx parses + issues exact writes | done |
| 5.2c | hybrid wire scheme: bulk reverts to DIRECT RDMA WRITE (zero overhead); inline message only for sub-64B stores | done |
| 5.3 | loomd control daemon: socket<->OrchClient adapter, SockOrchClient, standalone binary | done |
| 5.3b | stage cycle counters (T3 enabler): FIFO residency + per-stage cycles/op counts, RO CSR words 48-63, `StageStats`/`stage_avg` readout | done |
| 5.4 | hardware gate tests G1/G2/G4 on stock examples | pending |
| 5.5 | run on U280 (cross-pid); measure the sim's FPGA-owned constants: T3 per-stage latencies, T2 coalescing curve (needs coalescer RTL), substrate floors, B2 rdma-init, local read RTT | pending |
| 5.4a | deployment binaries: app_export/app_import | pending (AFTER 6.2a, owner reorder) |
| 6.1 | loomd-loomd TCP, QP setup, remote export/import | partial: peering + QP setup + staging exchange shipped with 6.2a; per-binding QPs + per-window staging pending |
| 6.2a | two-host BUNDLED configuration (one process per host) — the cross-host bring-up vehicle | code done, peering test 17x PASS; EXECUTION pending two-host testbed |
| 6.2b | full deployment topology (per side: loomd + 2 apps); remote reads via shell RDMA READ (T6) | pending |

**Bitstream: BUILDS on U280** (2026-08-04, Vivado 2023.2 → `cyt_top.bit`).
u280 must be an HBM device for any `EN_RDMA` build; see
`cmake/FindCoyoteHW.cmake` and the bitstream section below.

## Deferred optimization TODO (owner decision point)

**Write-combining aperture mapping (AVX-512 stores).** The ctrl region is
mapped `pgprot_noncached` (driver `vfpga_ops.c:595`), so UC serializes an
AVX-512 store into 8 B strongly-ordered accesses. Two pieces are needed,
because AXI4-Lite is single-beat at bus width regardless of mapping:
(1) a one-line driver change (`pgprot_writecombine` for `MMAP_CTRL`) so
the CPU emits 64 B TLPs at all — this breaches the "vFPGA + user space
only" rule and needs explicit owner sign-off; (2) a line-reassembly buffer
in loom_ctrl (8 beats -> one 64 B store; pure user logic, allowed), with
the inline message as the fallback for WC partial flushes. Payoff: much
better small-store issue rate plus a 64 B-line direct remote-store fast
path. Revisit at 5.5; until then latency numbers include UC store costs.

## Running

### Environment (NixOS testbed)

- Vivado tools (`vivado`, `xvlog`, `xelab`, `xsim`, `xsc`) are on PATH only
  inside `xilinx-shell` (non-interactive: `xilinx-shell -c "cmd"`).
  Default 2023.2; others under `/share/xilinx/Vivado/`.
- `cmake` inside xilinx-shell is Vivado's 3.3.2, too old for Coyote — get
  it from nix: `nix-shell -p cmake --run "..."`, nested inside
  xilinx-shell when the CMake run must find Vivado.
- Coyote's `sim/README.md` calls 2023.2 broken for their `tb_user`
  testbench; everything here ran fine on it, and no newer version is
  installed.

### One-time simulation setup

```bash
examples/loom/hw/setup_sim.sh
```

Configures the hw build, renders the generated RTL, builds the DPI library
and creates the XSIM project. It works around two host quirks (details in
the script header): cmake comes from nix, and Coyote's `make sim` DPI step
fails here because Vivado 2023.2's bundled `ld` (binutils 2.37) cannot
link against NixOS glibc 2.42 (`unknown type [0x13] section '.relr.dyn'`),
so the script builds `sim/coyote_sim.so` with the system gcc instead —
equivalent, since XSIM only dlopens it at runtime.

### Block-level testbenches

```bash
examples/loom/hw/tb/run_tbs.sh      # expect PASS x5, logs in hw/tb/work/
```

Re-execs itself inside `xilinx-shell`, compiles `lynx_pkg.sv` +
`axi_intf.sv` + the loom modules, runs `tb_loom_table`, `tb_loom_ctrl`,
`tb_loom_engine`, `tb_loom_rx`, `tb_loom_top`, `tb_loom_loopback`. Between
them they cover CSR readback, aperture capture incl. sub-word `wstrb`,
arrival ordering, FIFO wraparound and overflow, backpressure and bounds
edges, exact stage-counter relations (2-cycle unstalled stores,
`acc == 2*pops`), engine/rx arbitration (mutual-exclusion assertions,
starvation recovery, soaks), and both routes end to end.

`tb_loom_loopback` is the two-host data plane in one simulation: the real
ctrl/table/engine drive rdma windows, a model of the shell's RDMA path
fragments each request at PMTU the way `ib_transport_protocol` does
(`rq_wr.last` low on every fragment but the last, one `tlast` per message),
and the real `loom_rx` lands the result in a modelled host memory checked
against what software asked for. It runs `loom_host`'s sequence operation
for operation, plus a copy larger than PMTU. It exists because the header
ENCODE and DECODE are two hand-written layouts in two files: each block TB
checks its own side against literals it writes itself, so both can agree on
the same misunderstanding and still pass.

### Coyote integration sim

Prerequisite: `setup_sim.sh`. The run spawns Vivado/XSIM, so it goes
through xilinx-shell; use tmux, it takes minutes.

```bash
cd examples/loom/sw && mkdir -p build_sim && cd build_sim
nix-shell -p cmake gcc boost --run 'cmake .. -DEN_SIM=ON && make -j8'
tmux new-session -d -s loom_run \
  "xilinx-shell -c 'export COYOTE_SIM_DIR=$PWD/../../hw/build_sim/; ./test' \
   > run_test.log 2>&1"
tail -f run_test.log     # expect 19x PASS, then LOOM TEST PASS
# waveform: hw/build_sim/sim/sim_dump.vcd
```

Sim-backend gotchas:

- `setCSR/getCSR` take 64-bit **word indices** in both backends (the sim
  generator multiplies by 8 onto the AXI address); `loom.hpp` handles it.
- `EN_RANDOMIZATION` pads every ctrl write with random writes up to the
  next 64 B boundary. So `dbg[stores]` counts padding stores that land in
  valid windows (keep test offsets clear of each other), and CSR staging
  sequences must end with the meaningful write (ours end with
  COMMIT/TRIGGER). Counters read right after a poll may still be draining.
- Poll destination/completion **memory**, never CSRs, while a DMA is in
  flight — a blocking sim CSR read parks the generator.

### Python unit tests

The Coyote Python framework (`sim/unit_test`) drives the same XSIM project
non-interactively with barebones RDMA mocks.

```bash
cd examples/loom/hw/unit-tests
tmux new-session -d -s loom_py \
  "xilinx-shell -c 'nix-shell -p python3 --run \
   \"PYTHONPATH=../build_sim python3 -m unittest test_loom_rdma test_loom_read -v\"' \
   > pytest_run.log 2>&1"
tail -f pytest_run.log     # expect OK (skipped=1)
```

`test_loom_rdma` verifies the TX rdma path into the TB's RDMA-REMOTE mock
via exact debug counters; `test_loom_read` checks window loads return the
right qwords and that an unprogrammed window returns POISON. Aperture
reads cannot run in the C++ interactive sim at all — the blocking ctrl
read parks the generator that must service the engine's pull.

The RX test is skipped by design: the stock TB delivers only the `rq_wr`
request of an incoming write and discards the payload
(`memory_simulation.svh`, `rdmaLocalWrite`), so a forwarder waiting on
`axis_rrsp_recv` cannot complete. RX coverage is `tb_loom_rx` +
`tb_loom_loopback` + gate G3.

Framework quirks: register values pack as signed 64-bit (keep payloads
below 2^63); the RDMA-REMOTE segment must be allocated
(`remote_rdma_write`) before TX writes land; live register reads need a
long sim window (the test sets 1 ms; the 4 µs default closes too early).

### Simulation harnesses

NOT the deployment shape — sim allows one cThread per process, so true
process splits cannot run there (see HW-DESIGN "Deployment topology").

```bash
cd examples/loom/sw/build_sim
./test_loomd_proto      # FPGA-free, any machine: 13x PASS

tmux new-session -d -s loom_roles \
  "xilinx-shell -c 'export COYOTE_SIM_DIR=$PWD/../../hw/build_sim/; ./roles' \
   > run_roles.log 2>&1"        # expect 12x PASS, LOOM ROLES TEST PASS

tmux new-session -d -s loom_sock \
  "xilinx-shell -c 'export COYOTE_SIM_DIR=$PWD/../../hw/build_sim/; ./roles_sock' \
   > run_roles_sock.log 2>&1"   # expect "two clients connected", 12x PASS
```

- `./test` — monolithic regression (one cThread doing everything).
- `./roles` — role split: `loom_orch.hpp` (OrchClient + InProcOrchestrator,
  the only code touching the CSR page), `loom_xpu.hpp` (client), shared
  flow in `roles_test.hpp`. Covers export/import, bogus-handle refusal,
  stores + fenced copies + the order point, and window release. With
  `COYOTE_SIM_DIR` all roles share the one sim cThread; on hardware each
  role gets its own (real cross-pid).
- `./roles_sock` — the same flow with `loomd` on a thread over a REAL Unix
  socket (`loom_sock.hpp` implements OrchClient over it, so roles cannot
  tell the transports apart). A separate daemon process would spawn its
  own simulator, which is why the daemon is a thread here; only process
  isolation is degenerate, the protocol paths are production.

### 6.2a: the bundled two-host binary (`sw-bundled/`)

One process per host carrying the daemon role (BundledOrchestrator + local
loomd on a Unix socket + loomd<->loomd TCP peering) and that side's app
roles as threads — two processes total across the cluster.

- `loom_peer.hpp` — the 6.1 control plane. The hello at accept carries the
  exporter side's staging VA (the importer programs it into
  `RDMA_STAGING_VA`); RESOLVE turns a handle into `{exporter ctid, VA,
  len}` cross-host; DONE is the end-of-run barrier.
- `loom_bundle.hpp` — `BundledOrchestrator`: imports go remote when a peer
  is attached (route=rdma, pid = the LOCAL QP-owner ctid, base = the
  exporter's VA); its export registry doubles as the server-side resolver.
- `loom_host.cpp` — the binary. QP setup rides `cThread::initRDMA`
  (server first, then client with the server IP). Client flow = the roles
  flow across hosts; the server verifies the landings.

```bash
cd examples/loom/sw-bundled && mkdir -p build && cd build
nix-shell -p cmake gcc boost --run 'cmake .. && make -j8'
./test_peering                       # FPGA-free, any machine: 17x PASS

# two-host run (hardware), server side first:
./loom_host --server                 # waits in the QP exchange
./loom_host --client <host2_ip>      # from host 1
# expect LOOM HOST CLIENT PASS (7x) / LOOM HOST SERVER PASS (6x)
# options: --qp-port (18488), --peer-port (18489), --sock <loomd socket>
```

Builds everywhere but refuses to RUN under `COYOTE_SIM_DIR` (the sim has
no networking). Scope limits, deliberate: one RC connection, both exported
segments owned by the server's data cThread, and the global staging CSR
serves the asymmetric topology only (client TX / server RX; symmetric
traffic needs per-window staging, 6.1).

### Hardware

**Bitstream** (hours; tmux). U280 needs the HBM shell — with `EN_RDMA` on
a DDR-configured u280 the DDR4 MIG and the dangling HBM clock both claim
the board's single 100 MHz reference (BJ43/BJ44) and `opt_design` dies with
`[Mig 66-99] ... c0_sys_clk_p ... not placed`. `cmake/FindCoyoteHW.cmake`
therefore lists u280 in `HBM_DEV`, matching its own comment.

```bash
cd examples/loom/hw && mkdir -p build_hw && cd build_hw
CMAKE_BIN=$(nix-shell -p cmake --run 'dirname $(command -v cmake)')
xilinx-shell -c "export PATH=$CMAKE_BIN:\$PATH; cmake .. -DFDEV_NAME=u280"
tmux new-session -d -s loom_bit \
  "xilinx-shell -c 'export PATH=$CMAKE_BIN:\$PATH; make project && make bitgen' \
   > bitgen.log 2>&1"
# -> bitstreams/cyt_top.bit   (~4.5 h end to end on this host)
# sanity: base.tcl should show en_hcard 1, en_dcard 0, ddr_0 0, en_rdma 1
```

**Program + driver**, then the software in hardware mode (no `EN_SIM`, no
`COYOTE_SIM_DIR`):

```bash
cd <coyote-root>
xilinx-shell -c "vivado -mode batch -source program_fpga.tcl ..."
sudo bash setup_coyote.sh

cd examples/loom/sw && mkdir -p build_hw && cd build_hw
nix-shell -p cmake gcc boost --run 'cmake .. && make -j8'
./test     # same checks as sim
./roles    # each role gets its OWN cThread -> imports translate under the
           # exporter's distinct ctid (G1)
```

Do the 5.4 gate tests (G1/G2/G4) on stock examples before the first run.
Deployment binaries (`app_export`/`app_import`) are 5.4a; until they exist
the hardware multi-process stopgap is `./loomd /tmp/loomd.sock` plus
`LOOMD_SOCK=/tmp/loomd.sock ./roles_sock`, and the cross-host vehicle is
the 6.2a bundled binary above.

### Two things that silently break the sim

**Switching between the C++ and Python harnesses.** They compile the XSIM
project with different defines; the stale snapshot makes the *other*
harness crash at t=0 (XSIM kernel FATAL). Clean once after switching:

```bash
rm -rf examples/loom/hw/build_sim/sim/example_loom.sim
```

**After editing `hw/src/vfpga_top.svh`.** Setup copies it into
`hw/build_sim/sim/`, and the generated wrapper resolves its `include`
against that copy — so the sim silently builds the stale top-level wiring
(symptom: modules referenced in place are current, wiring changes are not):

```bash
cp examples/loom/hw/src/vfpga_top.svh examples/loom/hw/build_sim/sim/
rm -rf examples/loom/hw/build_sim/sim/example_loom.sim
```
