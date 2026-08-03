# Loom example (vFPGA switch prototype)

Single vFPGA per host acts as the switch. All emulated-XPU processes and the
control daemon attach to it as cThreads. Changes are confined to vFPGA user
logic and user-space software; the shell and driver stay stock.

> **Running things**: every commit that runs anything documents how to run
> it in the "Running" section at the bottom of this file. Keep it current.

## Ctrl-region layout (64 KB, AXI4-Lite)

- `0x0000-0x0FFF` — CSR page: window-table programming, DMA descriptor
  staging + trigger, completion config, RO debug counters.
- `0x1000-0xFFFF` — aperture: 15 windows of 4 KB. Window = `addr[15:12]`,
  offset = `addr[11:0]`. Every write beat here is captured as a small-write
  transaction (posted).

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
    forward the stream to the local or rdma write side; on the last beat,
    write an incrementing completion count to `(COMPL_PID, COMPL_VA)`.
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
| 5 | hardware bring-up, local paths | pending |
| 6 | hardware, RDMA path (two hosts) | pending |

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

### One-time: configure the hw build and render lynx_pkg.sv

The block TBs compile against the generated `lynxTypes` package:

```bash
cd examples/loom/hw && mkdir -p build_sim && cd build_sim
xilinx-shell -c "nix-shell -p cmake --run 'cmake .. -DFDEV_NAME=u280'"
mkdir -p sim
nix-shell -p 'python3.withPackages(ps: [ps.jinja2])' \
    --run 'python3 write_hdl.py 3 0 0'
# -> build_sim/sim/lynx_pkg.sv (+ user_logic_c0_0.sv)
```

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

One-time hw project setup (after the lynx_pkg step above). Note: `make sim`'s
DPI step is broken on this NixOS host (Vivado's bundled binutils cannot link
against glibc 2.42), so the DPI lib is built with the system toolchain and
the project created directly:

```bash
cd examples/loom/hw/build_sim
nix-shell -p gcc --run \
  'gcc -shared -fPIC -I/share/xilinx/Vivado/2023.2/data/xsim/include \
   ../../../../sim/hw/dpi/file_io.c -o sim/coyote_sim.so'
tmux new-session -d -s loom_crsim \
  "xilinx-shell -c 'vivado -mode batch -source cr_sim.tcl' > cr_sim_run.log 2>&1"
# wait for the session to exit; expect sim/example_loom.xpr
```

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

The RX test is skipped by design: the stock TB delivers only the rq_wr
request of an incoming RDMA write and discards the payload
(memory_simulation.svh, rdmaLocalWrite), so a forwarder waiting on
axis_rrsp_recv cannot complete in simulation. RX data-path coverage:
tb_loom_rx (block level) + hardware gate G3 (Phase 6).

Framework quirks: register values are packed as signed 64-bit (keep test
payloads below 2^63); the RDMA-REMOTE segment must be allocated first
(remote_rdma_write); live register reads need a long simulation window
(the test sets 1 ms; the 4 us default closes before responses arrive).
