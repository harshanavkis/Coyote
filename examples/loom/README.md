# Loom example (vFPGA switch prototype)

Single vFPGA per host acts as the switch. All emulated-XPU processes and the
control daemon attach to it as cThreads. Changes are confined to vFPGA user
logic and user-space software; the shell and driver stay stock.

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
