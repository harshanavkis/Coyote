# Loom hardware design

What the vFPGA implements, component by component, and why each piece is
the moral equivalent of something a GPU already does. Companion documents:
[README.md](README.md) (layout, status, how to run) and
[WORKFLOW.md](WORKFLOW.md) (worked example of all four flows).

## Big picture

The single vFPGA on each host is a switch for peer-memory traffic. Every
process that participates (emulated XPUs, the control daemon) attaches as
a Coyote cThread. Processes reach peers through two producer paths that
converge in one pipeline:

```
              64 KB ctrl region (AXI4-Lite)
   CSR page (0x0000-0x0FFF)        aperture (0x1000-0xFFFF, 15 x 4 KB)
   table programming, DMA          direct stores: address = window+offset
   descriptor staging + trigger              |
              |                              |
              +--------> ORDER FIFO <--------+        (loom_ctrl)
                             |
                     table lookup (loom_table)
                             |
                       loom_engine ------ sq_rd pull (DMA path)
                       /          \
              local write        rdma write
         sq_wr {pid, VA+off}   sq_wr {RETH vaddr = exporter VA + off}
         + axis_host_send      + axis_rreq_send
                       \          /
                     shared-path arbiter also serving
                       loom_rx (incoming rdma writes ->
                       local write under destination pid)
```

## Components

### loom_ctrl — aperture capture, CSRs, the order point

The AXI4-Lite slave splits the 64 KB user ctrl region: a CSR page (window
table programming, DMA descriptor staging + trigger, RO debug counters)
and 15 aperture windows of 4 KB, where the *address itself* names the
destination (`addr[15:12]` = window, `addr[11:0]` = offset). Every write
beat landing in a window is captured as a posted small-write transaction;
every descriptor trigger is captured as a bulk transaction. Both go into
one arrival-ordered FIFO. That single FIFO is the design's ordering
guarantee: a flag store issued after a DMA descriptor sits behind it and
can never overtake it.

### loom_table — the compiled route

One entry per window: `{valid, route local|rdma, pid, VA base, len}`.
`base` is always the *exporter's own* virtual address; `len` bounds the
segment. Programmed only through the CSR page, by the control daemon.
The table is consulted once per transaction; nothing on the data path
computes a route.

### loom_engine — the two consumers folded into one

Transaction-serialized consumer of the order FIFO.

- STORE entry: one write request + one data beat. Local:
  `sq_wr {LOCAL_WRITE, STRM_HOST, pid, base+off, 8}` — the shell TLB
  translates the exporter's VA under the exporter's pid. Rdma:
  `sq_wr {APP_WRITE, STRM_RDMA, base+off, 8}` — the RETH virtual address
  *is* the exporter's VA plus offset; the far host's TLB finishes the job.
- DESC entry: pull `sq_rd {LOCAL_READ, src_pid, src_va, len}` (the shell
  TLB translates the issuer's source buffer during the pull), forward the
  stream to the local or rdma write side, then release the fence: if the
  descriptor's completion VA is nonzero, write an incrementing count to
  `(src_pid, compl_va)`.
- Invalid window or bounds violation: the entry is dropped and counted.

### loom_rx — the receive side

Accepts one incoming RDMA write at a time — request on `rq_wr`
`{pid, vaddr, len}`, payload on `axis_rrsp_recv` — and forwards it as a
plain local write under the destination pid. It only starts when granted
the shared write path.

### The arbiter (vfpga_top)

`loom_engine` and `loom_rx` share `sq_wr` and `axis_host_send[0]`. A
registered arbiter grants the pair to one of them for whole transactions,
engine priority; the engine is gated by masking its FIFO-empty input, rx
by an explicit grant. Mutual exclusion is assertion-tested in
`tb_loom_top`.

## Why this is what GPUs already do

| Loom piece | GPU equivalent | Notes |
|---|---|---|
| imported pointer -> mmap'd aperture window | peer pointer from `cudaIpcOpenMemHandle` -> GPU-MMU PTEs to the peer's BAR | the host MMU plays the GPU MMU; dereference is an ordinary store either way |
| aperture store captured by the switch | fine-grained peer store crossing PCIe to a peer BAR | posted semantics; this is the path NCCL LL flags ride |
| window table entry | the compiled peer mapping behind the BAR window (which device, which offset) | installed at import time, consulted per transaction, invisible to the app |
| doorbell + descriptor CSRs | copy-engine ring / pushbuffer command | `loom_memcpy` hides descriptor mechanics exactly as `cudaMemcpy` hides CE programming |
| engine pull -> write | the copy engine executing the command | fidelity note: a real CE *pushes* through the fabric port; the engine *pulls* via host streams and injects at the same pipeline point — same bytes, same ordering point |
| per-descriptor completion write (incrementing count to a memory word) | CE **semaphore release**: the engine writes a monotonic fence value to the address the command names; `cudaStreamSynchronize`/events spin on that word | our descriptor carries its fence VA like a CE command carries its release address |
| polling the destination / fence in ordinary memory | polling mapped fence memory (spin) — the GPU fast path | never a register read on the data path |
| order FIFO + serialized engine | ordering of one issuer's stores and CE ops through one fabric port / stream | our single global FIFO is *stricter* than required (orders across issuers too); per-binding relaxation is future work alongside per-destination queues |
| RETH vaddr = exporter's VA | an RDMA NIC's virtually-addressed wire format | the "global name" on the wire is just the exporter's process-local VA, meaningless anywhere else |
| debug counters (stores, descs, writes, drops, fences) | engine performance counters | read via CSRs off the data path |

## Deliberate simplifications (and where they go next)

- Aperture ops are <= 8 B (one AXI-Lite beat); larger transfers use the
  DMA path. Sub-word `wstrb` is captured but writes issue as full 8 B
  (hardware gate G2).
- One engine, fully serialized. Of the classic switch machinery, only
  the TX **coalescer** is still planned (it produces the T2
  coalescing/goodput curve the simulation calibrates against, coalescer
  on/off). Per-destination queues + scheduler and the error containment
  unit are **descoped** (2026-08 eval review: the paper makes no
  isolation claim, the victim-flow experiment is dead, failure
  containment dropped).
- Aperture reads (loads through the window) are **not implemented yet -
  planned, scoped to T6 calibration**: today a CPU load from the
  aperture returns CSR values (window 0) or zeros; the data path is
  write-only. The planned read path adds a READ entry kind to the order
  FIFO - the engine pulls 8 B from the destination under its pid and
  the ctrl slave holds the AXI-Lite read channel open until the data
  returns (local reads sim-testable via getCSR; remote read RTT via the
  shell's RDMA READ path is hardware-only, PCIe completion timeout
  watched). The credit *model* stays in the simulation (sweep_credits);
  AXI-Lite's single-outstanding reads make the hardware tracker
  trivially depth-1.
- The completion count is a single monotonic counter per engine. Apps that
  serialize their own DMAs can poll for change/expected values; carrying a
  caller-chosen fence *payload* in the descriptor (full CE semantics) is a
  small future extension.
- Source identity is by partition convention (no attacker model);
  destination isolation is real (pid-scoped TLB translation).
