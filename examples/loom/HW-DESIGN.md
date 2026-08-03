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
  translates the exporter's VA under the exporter's pid. Rdma: ONE full
  64 B wire message at the staging RETH vaddr — beat =
  `{lane0 = ⟨op WRITE_INLINE · len⟩, lane1 = exporter VA + off,
  lane2 = data}`; the far loom_rx issues the exact 8 B write. Nothing
  sub-64 B ever goes on the wire (gate G5), and the wire format is the
  design's ⟨op·len·vaddr⟩ message.
- DESC entry: pull `sq_rd {LOCAL_READ, src_pid, src_va, len}` (the shell
  TLB translates the issuer's source buffer during the pull), forward the
  stream to the local or rdma write side, then release the fence: if the
  descriptor's completion VA is nonzero, write an incrementing count to
  `(src_pid, compl_va)`.
- Invalid window or bounds violation: the entry is dropped and counted.

### loom_rx — the receive side

Hybrid dispatch on one compare: an incoming write whose RETH vaddr
equals the STAGING address is a Loom inline message — parse the header
beat `⟨op · len · target VA⟩` and issue the exact (8 B) write it
describes; any other vaddr is a direct bulk write — forward request and
every beat verbatim. Local writes run under the QP owner's pid; unknown
ops drain harmlessly; starts only when granted the shared write path.

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
| bulk: direct RDMA WRITE (RETH = exporter VA + off); sub-64 B stores: 64 B inline message at a staging RETH vaddr | direct: an RNIC's ordinary virtually-addressed write; inline: RNIC inline-send / NCCL LL fixed-size lines | op·len·vaddr on the wire is RDMA's own BTH/RETH for bulk; the inline header exists only because a RETH cannot say "envelope 64, true write 8" |
| reachability = a TLB entry under the owner's pid (no MR registration, no keys) | GPU scale-up model: a peer write is valid iff the owning GPU's MMU translates it; nothing registered with the fabric, no key on any bus | deliberately NOT the verbs model - matches the design's "no addresses, keys, or QPs visible" claim; bounds at the source switch, translation at the destination |
| staging buffer: small ordinary getMem of the QP owner, address exchanged at QP setup | NCCL per-connection transport buffer (plain cudaMalloc, address exchanged at connect/accept, remote LL/FIFO writes land there for interpretation) | shell contract (per 09_perf_rdma): incoming writes surface to USER LOGIC (rq_wr + rrsp_recv) and land only if user logic issues the write - so the shell never writes staging memory itself; the staging vaddr is addressing, its allocation exists so the vaddr is honest/mappable and future-proof |
| debug counters (stores, descs, writes, drops, fences) | engine performance counters | read via CSRs off the data path |

## Deployment topology (target process architecture)

There is ONE real deployment shape - the two-host client/server
topology; everything else is a degenerate of it:

```
CLIENT side (host 1)                    SERVER side (host 2)
  loomd  (control daemon, owns            loomd  (control daemon, owns
         the CSR page via its cThread)          the CSR page)
  app_import x2 (importer processes,      app_export x2 (exporter processes,
         own cThreads; stores/copies/            own cThreads; allocate,
         reads via their windows)                export, poll their buffers)
         |___ SockOrchClient (Unix socket) to their side's loomd ___|
              loomd <-> loomd TCP (6.x): cross-host handle
              resolution + QP setup (QPs owned by exporter cThreads)
```

- **Hardware bring-up configuration** (Phase 5.4/5.5, local route): the
  SAME binaries on one host - one loomd (one vFPGA = one CSR-page
  owner) + the exporter and importer processes together; client and
  server sides collapse onto the single host. A stepping stone, not a
  separate variant.
- **Two-host bundled configuration** (Phase 6.2a, remote-route
  bring-up): one process PER HOST, bundling the daemon role (thread) +
  that side's app roles - two processes total across the cluster.
  Exercises everything new in 6.x (loomd<->loomd TCP, QP setup, staging
  exchange, the real wire) with minimal moving parts; the full per-side
  split (6.2b) is then a process-topology rerun on a proven data path.
- **Simulation degenerate**: the mock backend allows one cThread per
  process (each spawns its own simulator), so true process splits
  cannot run in sim - the threaded harnesses (`roles`, `roles_sock`)
  exist solely to exercise the protocol, daemon, and data-plane logic
  there. They are NOT the deployment shape.

Binaries: `loomd` (exists) + `app_export`/`app_import` (to be written,
Phase 5.4a; two instances each per side; handle exchange between
exporter and importer via a simple side channel, as real IPC handles
travel).

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
- Aperture reads (loads through a window) are **implemented for local
  windows** (Phase 5.2): a READ entry rides the same order FIFO, the
  engine pulls the full 64 B aligned line containing the target under
  the destination pid and lane-selects the 8 B, and the ctrl slave
  holds the AXI-Lite read channel open until the data returns. Safety:
  reads are never dropped (arready withheld while the FIFO is full) and
  every read answers - invalid/dead/rdma-route windows return POISON
  (all-ones) instead of wedging the CPU. The full-line pull is
  deliberate: it avoids sub-line DMA entirely (cf. the 64 B minimum
  RDMA payload jigsaw hit, gate G5). Remote loads arrive with the
  two-host phase (shell RDMA READ, T6 remote RTT). The credit *model*
  stays in the simulation (sweep_credits); AXI-Lite's single-outstanding
  reads make the hardware tracker trivially depth-1. GPU-equivalence
  note: the stall-until-answer load matches the non-posted peer read a
  GPU issues over PCIe P2P today - slow by nature, which is exactly why
  the fast path stays push-only.
- The completion count is a single monotonic counter per engine. Apps that
  serialize their own DMAs can poll for change/expected values; carrying a
  caller-chosen fence *payload* in the descriptor (full CE semantics) is a
  small future extension.
- Source identity is by partition convention (no attacker model);
  destination isolation is real (pid-scoped TLB translation).
