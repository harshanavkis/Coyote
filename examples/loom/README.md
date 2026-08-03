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

## Components (planned hw/src/hdl/)

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
| 3 | loom_rx + block TB | pending |
| 4 | vfpga_top wiring + Coyote integration sim (EN_SIM) | pending |
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
- Note: Coyote's `sim/README.md` flags Vivado 2023.2 as broken for their
  full `tb_user` testbench (mailbox regression). The block TBs below do
  not use that testbench and work on 2023.2; for Phase 4 integration sim,
  try 2025.1 if 2023.2 fails.

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
`tb_loom_table`, `tb_loom_ctrl`, `tb_loom_engine`). Logs land in
`hw/tb/work/`.

`tb_loom_engine` is a composite test (real loom_ctrl + loom_table +
loom_engine, shell side mocked): local/rdma stores, DMA local/rdma with
completion writes, invalid-window and bounds drops, and the ordering
property (flag store behind a DMA descriptor issues only after the DMA
stream + completion).
