# Loom example (vFPGA switch prototype)

Single vFPGA per host acts as the switch. All emulated-XPU processes and the
control daemon attach to it as cThreads. Changes are confined to vFPGA user
logic and user-space software; the shell and driver stay stock.

> **Running things**: every commit that runs anything documents how to run
> it in the "Running" section at the bottom of this file. Keep it current.

## Ctrl-region layout (64 KB, AXI4-Lite)

- `0x0000-0x0FFF` — CSR page: window-table programming, DMA descriptor
  staging (incl. per-descriptor fence VA) + trigger, RO debug counters.
- `0x1000-0xFFFF` — aperture: 15 windows of 4 KB. Window = `addr[15:12]`,
  offset = `addr[11:0]`. Every write beat here is captured as a small-write
  transaction (posted).

Detailed hardware description and the GPU-equivalence argument:
[HW-DESIGN.md](HW-DESIGN.md).

## Components (hw/src/hdl/)

- **loom_ctrl** — AXI4-Lite slave: CSR page + aperture capture. Captured
  stores and triggered DMA descriptors are pushed into one arrival-ordered
  FIFO (the order point), so a store cannot overtake an earlier descriptor.
- **loom_table** — per-window entry `{valid, route local|rdma, pid, VA base,
  len}`. `base` is always the exporter's own VA. Programmed via CSRs only.
- **loom_engine** — pops the order FIFO.
  - Store, local: `sq_wr {LOCAL_WRITE, STRM_HOST, pid, base+off, 8}` + one
    beat on `axis_host_send`.
  - Store, rdma: `sq_wr {APP_WRITE, STRM_RDMA, pid = QP owner, base+off, 8}`
    + beat on `axis_rreq_send` (RETH vaddr = exporter's VA).
  - Descriptor: pull `sq_rd {src_pid, src_va, len}` via `axis_host_recv`,
    forward the stream to the local or rdma write side; then release the
    descriptor's fence: an incrementing count written to the descriptor's
    completion VA under `src_pid` (CE semaphore-release model).
- **loom_rx** — incoming RDMA writes: forward `(pid, vaddr, len)` from
  `rq_wr` + payload to a local `sq_wr` write.

## Workflow (VAs only)

The full worked example (concrete addresses, all four flows, translation
chain) is in [WORKFLOW.md](WORKFLOW.md). Summary:

Processes A, B on host 1 (pids 0, 1); C on host 2 (pid 0).
`buf_B`/`buf_C` are `getMem` buffers at B's/C's own VAs. Daemon programs:
window 1 -> `{local, pid 1, buf_B, 4MB}`, window 2 -> `{rdma, qp, buf_C, 4MB}`.

1. **Small local** `*(P_B+0x40)=v`: A's MMU -> ctrl `0x1040` -> table ->
   `sq_wr {pid 1, buf_B+0x40}` -> TLB under pid 1 -> B polls `buf_B[0x40]`.
2. **Small remote** `*(P_C+0x40)=v`: same until table -> RDMA write with
   vaddr `buf_C+0x40` on the binding's QP -> host 2 TLB under C's pid.
3. **Bulk local** `copy(P_B+off, src, 1MB)`: descriptor `{win 1/off, src, len}`
   -> pull under pid 0 -> write under pid 1 -> completion word into A's memory.
4. **Bulk remote**: same pull, write side is the RDMA path (vaddr
   `buf_C+off`); shell fragments to PMTU.

Every address is some process's ordinary VA: A's pointers in, exporter's VA
out; the wire carries the exporter's VA (offset in affine encoding).

## Gates to verify first

1. Cross-pid write: `sq_wr` under another attached cThread's pid lands in
   that process's buffer.
2. Sub-line writes: alignment/keep semantics of an 8 B `LOCAL_WRITE`.
3. RX interposition: where incoming RDMA writes surface (`rq_wr` +
   `axis_rrsp_recv`, per jigsaw) vs. direct-to-memory.
4. QP selection from user logic when more than one binding/QP exists.
5. Minimum RDMA payload: jigsaw needed 64 B (it pads every network
   message to one full beat, `rdma_wr_len = 64`). Our 8 B `APP_WRITE`
   remote stores likely violate this; naive padding would clobber 56
   neighbor bytes at the destination. Expected fix (Phase 6): 64 B wire
   message = header + payload, far-side loom_rx issues the exact 8 B
   local write. Verify the constraint on hardware first.

Shell config (when hw is added): `EN_STRM 1, N_STRM_AXI 1, EN_RDMA 1,
N_REGIONS 1`; cf. `examples/jigsaw_baseline_rdma`.

## Status

| Phase | Content | State |
|---|---|---|
| 1 | loom_ctrl (CSR page + aperture capture + order FIFO), loom_table, block TBs | done, TBs pass |
| 2 | loom_engine (store + DMA branches, completion) + block TB | done, TBs pass |
| 3 | loom_rx + block TB | done, TBs pass |
| 4 | vfpga_top wiring + Coyote integration sim (EN_SIM) | done, LOOM TEST PASS |
| 4.5 | test hardening: tb_loom_top (arbitration), engine/ctrl corner cases, extended integration sim, Python RDMA TX test | done, all pass |
| 5.0 | per-descriptor completion (fence VA in descriptor, CE semaphore-release model) | done, all sims pass |
| 5.1a | single-process software: client/server role split (OrchClient iface, in-process transport) | done, ROLES TEST PASS (sim) |
| 5.2 | aperture reads, local path (READ order-FIFO entry, held-open AXI-Lite read, 64B line pull + lane select, poison on invalid) + TBs + Python sim test | done, all pass |
| 5.3 | multi-process split: libloom + loomd control daemon (Unix socket) | pending |
| 5.4 | hardware gate tests G1/G2/G4 on stock examples | pending |
| 5.5 | synthesize + run on U280 (cross-pid); measure the sim's FPGA-owned constants: T3 per-stage latencies (needs stage cycle counters), T2 coalescing curve (needs coalescer RTL, on/off), substrate floors, B2 rdma-init, local read RTT | pending |
| 6.1 | loomd-loomd TCP, QP setup via Coyote RDMA API, remote export/import | pending |
| 6.2 | two-host run (resolves G3), remote measurements incl. remote reads via shell RDMA READ (T6 remote RTT) | pending |

## Running

### Environment (NixOS testbed)

- Vivado tools (vivado, xvlog, xelab, xsim, xsc) are only on PATH inside
  `xilinx-shell` (non-interactive: `xilinx-shell -c "cmd"`). Default is
  Vivado 2023.2; others under `/share/xilinx/Vivado/`.
- `cmake` inside xilinx-shell is Vivado's ancient 3.3.2; use nix instead:
  `nix-shell -p cmake --run "..."` (nested inside xilinx-shell when the
  CMake run needs to find Vivado).
- Coyote's `sim/README.md` flags Vivado 2023.2 as broken for their full
  `tb_user` testbench (mailbox regression); in practice the Phase 4
  integration sim ran fine on 2023.2 here. (No newer version is currently
  installed under `/share/xilinx/Vivado/`.)

### One-time simulation setup (script)

```bash
examples/loom/hw/setup_sim.sh
```

The script configures the hw build, renders the generated RTL
(`build_sim/sim/lynx_pkg.sv` + wrapper), builds the DPI library, and
creates the XSIM project. Two host quirks it works around (details in
the script header):

- Vivado's bundled cmake 3.3 is too old for Coyote -> cmake comes from
  `nix-shell`, Vivado from `xilinx-shell`.
- **Coyote's `make sim` DPI step (xsc) fails on this host**: Vivado
  2023.2's bundled `ld` (binutils 2.37) cannot link against NixOS glibc
  2.42 - the signature is `unknown type [0x13] section \`.relr.dyn'` /
  `skipping incompatible ... libmvec.so.1`, ending in
  `Linking failed for "coyote_sim.so"`. The script builds
  `sim/coyote_sim.so` with the system gcc instead; XSIM only dlopens the
  library at runtime, so this is equivalent.

### Block-level testbenches (Phase 1+)

```bash
examples/loom/hw/tb/run_tbs.sh
```

The script re-execs itself inside `xilinx-shell` if needed, compiles
`lynx_pkg.sv` + `hw/hdl/pkg/axi_intf.sv` + the loom modules, then runs each
`tb_*` in XSIM. Expected output: `PASS: <tb>` per testbench (currently
`tb_loom_table`, `tb_loom_ctrl`, `tb_loom_engine`, `tb_loom_rx`,
`tb_loom_top`). Logs land in `hw/tb/work/`.

Coverage (hardened in Phase 4.5):
- `tb_loom_ctrl`: CSR readback (all RW regs), commit pulse, aperture
  capture fields incl. sub-word wstrb, descriptor enqueue, arrival
  ordering, overflow drops, FIFO wraparound (rolling 5-in/5-out across
  the 64-entry boundary), counters.
- `tb_loom_engine` (composite ctrl+table+engine, shell mocked):
  local/rdma stores, DMA local/rdma with completion values,
  descriptor-then-flag ordering, backpressure matrix (wr_ready, host and
  net tready, mid-stream), bounds edges (end==lim vs end==lim+8, store at
  window end), non-64B-multiple lengths, completion-disabled path, and a
  60-op soak checked against exact counter deltas.
- `tb_loom_rx`: grant gating, forwarding, backpressure, back-to-back.
- `tb_loom_top`: the generated `design_user_logic_c0_0` wrapper as DUT
  (vfpga_top.svh verbatim) - engine/rx arbitration: continuous mutual-
  exclusion assertions, races in both directions, starvation recovery
  after a store burst, and a mixed 40-op soak (stores/descs/rx) with
  exact wr_req/beat/counter accounting.

### Coyote integration sim (Phase 4)

Prerequisite: `setup_sim.sh` above (project + DPI lib in place).

Software build and run (the run spawns Vivado/XSIM, so it goes through
xilinx-shell; use tmux, it takes minutes):

```bash
cd examples/loom/sw && mkdir -p build_sim && cd build_sim
nix-shell -p cmake gcc boost --run 'cmake .. -DEN_SIM=ON && make -j8'
tmux new-session -d -s loom_run \
  "xilinx-shell -c 'export COYOTE_SIM_DIR=$PWD/../../hw/build_sim/; ./test' \
   > run_test.log 2>&1"
tail -f run_test.log     # expect: 13x PASS (2 windows, interleaved stores/
                         # DMAs, cross-window ordering, counter relations),
                         # then LOOM TEST PASS
# waveform: hw/build_sim/sim/sim_dump.vcd
```

Sim-backend notes (learned the hard way):
- `setCSR/getCSR` take 64-bit **word indices** in both backends (the sim
  generator multiplies by 8 onto the AXI address); `loom.hpp` handles it.
- The TB's `EN_RANDOMIZATION` pads every ctrl write with random writes up
  to the next 64 B boundary. Consequences: `dbg[stores]` counts padding
  stores landing in the aperture (random data written at nearby offsets in
  valid windows - keep test offsets clear of each other), and CSR staging
  sequences must end with the meaningful write (ours do: COMMIT/TRIGGER
  last). Counters read immediately after a poll may still be draining.
- Poll destination/completion *memory*, never CSRs, while a DMA is in
  flight (sim CSR reads block behind the generator).

### Python RDMA unit test (Phase 4.5)

The Coyote Python sim framework (`sim/unit_test`) drives the same XSIM
project non-interactively and provides barebones RDMA mocks. Our test
verifies the TX rdma path end-to-end into the TB's RDMA-REMOTE mock,
asserted via the loom debug counters (exact counts: randomization is
disabled through the framework's own knob):

```bash
cd examples/loom/hw/unit-tests
tmux new-session -d -s loom_py \
  "xilinx-shell -c 'nix-shell -p python3 --run \
   \"PYTHONPATH=../build_sim python3 -m unittest test_loom_rdma -v\"' \
   > pytest_run.log 2>&1"
tail -f pytest_run.log     # expect: test_tx_store_takes_rdma_path ... ok
                           #         OK (skipped=1)
```

The local aperture-read test runs the same way (or together:
`python3 -m unittest test_loom_rdma test_loom_read -v`):
`test_loom_read.py` writes a patterned buffer into the sim's memory mock,
programs a window, and checks that window loads return the right qwords
(aligned 64 B line pull + lane select) and that an unprogrammed window
returns POISON. Aperture reads cannot run in the C++ interactive sim
(the blocking ctrl read parks the generator that would have to service
the engine's pull - the documented interactive-mode deadlock).

The RX test is skipped by design: the stock TB delivers only the rq_wr
request of an incoming RDMA write and discards the payload
(memory_simulation.svh, rdmaLocalWrite), so a forwarder waiting on
axis_rrsp_recv cannot complete in simulation. RX data-path coverage:
tb_loom_rx (block level) + hardware gate G3 (Phase 6).

Framework quirks: register values are packed as signed 64-bit (keep test
payloads below 2^63); the RDMA-REMOTE segment must be allocated first
(remote_rdma_write); live register reads need a long simulation window
(the test sets 1 ms; the 4 us default closes before responses arrive).

### Role-split demo (Phase 5.1a)

Single binary, client/server structure: `loom_orch.hpp` (OrchClient
interface + InProcOrchestrator, the only code touching the CSR page) and
`loom_xpu.hpp` (client role: data plane + control calls through the
interface); `roles.cpp` runs exporter/importer/orchestrator roles.
Transport is in-process function calls; 5.1b swaps it for a Unix socket
to loomd behind the same interface. Mode is runtime-selected: with
COYOTE_SIM_DIR set, all roles share the single sim cThread (degenerate);
on hardware each role gets its own cThread (untested until 5.3).

```bash
cd examples/loom/sw/build_sim   # after the cmake/make above
nix-shell -p cmake gcc boost --run 'make -j8 roles'
tmux new-session -d -s loom_roles \
  "xilinx-shell -c 'export COYOTE_SIM_DIR=$PWD/../../hw/build_sim/; ./roles' \
   > run_roles.log 2>&1"
tail -f run_roles.log   # expect 12x PASS, LOOM ROLES TEST PASS
```

Covers: export/import through the orchestrator (windows allocated
server-side), bogus-handle refusal, stores + fenced copies + the order
point through the role API, and window release (subsequent stores dropped,
observed via the drops counter).

### Running on hardware (Phase 5.3 flow - NOT yet validated)

The expected flow once testbed time is available; commands follow the
standard Coyote flow and this repo's helper scripts:

```bash
# 1. Bitstream (hours; tmux). Same build dir layout as sim, own dir:
cd examples/loom/hw && mkdir -p build_hw && cd build_hw
xilinx-shell -c "nix-shell -p cmake --run 'cmake .. -DFDEV_NAME=u280'"
tmux new-session -d -s loom_bit \
  "xilinx-shell -c 'nix-shell -p cmake --run \"make project && make bitgen\"' \
   > bitgen.log 2>&1"
# -> bitstreams/cyt_top.bit

# 2. Program the FPGA + load the driver (repo root; needs sudo; the
#    script hot-resets the PCIe device and insmods with per-host ip/mac):
cd ../../../..   # Coyote root
xilinx-shell -c "vivado -mode batch -source program_fpga.tcl ..."  # or hw_server flow
sudo bash setup_coyote.sh

# 3. Software, hardware mode (no EN_SIM, no COYOTE_SIM_DIR):
cd examples/loom/sw && mkdir -p build_hw && cd build_hw
nix-shell -p cmake gcc boost --run 'cmake .. && make -j8'
./test    # single-process regression (same checks as sim)
./roles   # role-split demo: on hardware each role gets its OWN cThread,
          # so imports translate under the exporter's distinct ctid (G1)
```

Before first hardware run, do the Phase 5.2 gate tests (G1/G2/G4, see
"Gates to verify first" above) on stock examples.

### Switching between the C++ and Python sim harnesses

The two harnesses compile the XSIM project with different defines
(interactive vs. non-interactive). The stale snapshot makes the *other*
harness crash at t=0 (XSIM kernel FATAL). After switching, clean once:

```bash
rm -rf examples/loom/hw/build_sim/sim/example_loom.sim
```
