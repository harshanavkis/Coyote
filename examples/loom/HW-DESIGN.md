# Loom hardware design

What the vFPGA implements, why each piece is the moral equivalent of
something a GPU already does, and the source evidence that it works on a
stock Coyote shell. Companions: [README.md](README.md) (status, how to
run), [WORKFLOW.md](WORKFLOW.md) (worked example of all five flows).

## Big picture

The single vFPGA on each host is a switch for peer-memory traffic. Every
participating process (emulated XPUs, the control daemon) attaches as a
Coyote cThread. Two producer paths converge in one pipeline:

```
              64 KB ctrl region (AXI4-Lite)
   CSR page (0x0000-0x0FFF)        aperture (0x1000-0xFFFF, 15 x 4 KB)
   table programming, DMA          direct stores/loads: address = window+offset
   descriptor staging + trigger              |
              |                              |
              +--------> ORDER FIFO <--------+        (loom_ctrl)
                             |
                     table lookup (loom_table)
                             |
                       loom_engine ------ sq_rd pull (DMA + read paths)
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

**loom_ctrl** — AXI4-Lite slave. Splits the ctrl region into a CSR page
(table programming, descriptor staging + trigger, RO counters) and 15
aperture windows where the *address itself* names the destination
(`addr[15:12]` = window, `addr[11:0]` = offset). Stores, descriptor
triggers and reads all enter one arrival-ordered FIFO. That FIFO is the
ordering guarantee: a flag store issued after a DMA descriptor sits behind
it and can never overtake it.

**loom_table** — one entry per window, `{valid, route local|rdma, pid, VA
base, len}`. `base` is always the *exporter's own* VA; `len` bounds the
segment. Programmed only through the CSR page, by the control daemon.
Consulted once per transaction; nothing on the data path computes a route.

**loom_engine** — transaction-serialized consumer of the order FIFO.

- STORE, local: `sq_wr {LOCAL_WRITE, STRM_HOST, pid, base+off, 8}` + one
  beat; the shell TLB translates the exporter's VA under its pid.
- STORE, rdma: ONE 64 B wire message at the staging RETH vaddr, beat =
  `{lane0 = ⟨op WRITE_INLINE · len⟩, lane1 = exporter VA + off,
  lane2 = data}`; the far loom_rx issues the exact 8 B write. Nothing
  sub-64 B goes on the wire (G5 below).
- DESC: pull `sq_rd {LOCAL_READ, src_pid, src_va, len}`, forward the
  stream to the local or rdma write side, then release the fence — an
  incrementing count to `(src_pid, compl_va)` if the descriptor named one.
- READ: pull the full 64 B aligned line under the destination pid, lane-
  select 8 B, answer the held-open AXI read.
- Invalid window or bounds violation: entry dropped and counted.

**loom_rx** — hybrid dispatch on one compare. An incoming write whose RETH
vaddr equals STAGING is a Loom inline message: parse `⟨op · len · target
VA⟩` and issue exactly the write it describes. Any other vaddr is direct
bulk: forward request and every beat verbatim. Writes run under the QP
owner's pid.

Two rules keep a parsed target from ever being garbage, because this side
hands it straight to the shell TLB. **Transactions are bounded by the
request's length** (`ceil(len/64)` beats), not by `tlast`: `rq_wr.last` is
low whenever the shell ends a stream without one (`req_t` in `lynx_pkg`,
which `ib_transport_protocol` emits for every `RDMA_WRITE_FIRST/MIDDLE`),
so a tlast-only rule reads the next message's payload as a header. Beats a
request still owns after its write are drained, never left behind.
**Headers are checked against the contract** `loom_engine` emits — inline
is `len == 8` with an 8 B-aligned target, bulk is a nonzero 64 B multiple,
reserved bits zero — and anything else is dropped and counted (CSR word
41) rather than translated.

**The arbiter (vfpga_top)** — `loom_engine` and `loom_rx` share `sq_wr`
and `axis_host_send[0]`. A registered arbiter grants the pair for whole
transactions, engine priority; the engine is gated by masking its
FIFO-empty input, rx by explicit grant. Mutual exclusion is assertion-
tested in `tb_loom_top`.

## Gates: why this works on a stock shell (all closed from source)

Every mechanism Loom needs was confirmed in the Coyote sources rather than
deferred to hardware. Hardware owes bring-up validation and performance
numbers only.

| Gate | Verdict | Evidence (Coyote sources) |
|---|---|---|
| **G1** cross-pid write | CLOSED | `hw/hdl/mmu/tlb_controller.sv:400` — a TLB hit needs `tag match && entry.pid == request.pid`. Entries are pid-tagged and coexist; the request's pid IS the address-space selector and nothing checks who set it. `09_perf_rdma` forwards foreign pids routinely; `08_multithreading` runs several cThreads' DMAs on one vFPGA |
| **G2** sub-line writes | CLOSED | `hw/hdl/static/xdma_wrapper.sv:49` — host writes drive XDMA descriptor-bypass with `{c2h_addr, c2h_len}`, i.e. byte-granular descriptors, payload consumed from stream lane 0. An 8 B `LOCAL_WRITE` = a `{PA, 8}` descriptor + our LSB-aligned beat |
| **G3** RX interposition | CLOSED | `09_perf_rdma/hw/src/vfpga_top.svh` — an incoming RDMA WRITE surfaces as `rq_wr` (request) + `axis_rrsp_recv` (payload) and **user logic must land it**; there is no silent-to-memory path, which is why `loom_rx` is required rather than optional. `ib_transport_protocol.cpp:628` — the RX parser puts the RETH vaddr in verbatim (no MR table, no filter), MIDDLE/LAST continue from an MSN-table cursor seeded from it. Incoming READs likewise surface on `rq_rd` for user logic to serve (`rq_rd -> sq_rd`, `host_recv -> rrsp_send`) — the 6.x remote-read template |
| **G4** QP selection | CLOSED | `sw/src/cThread.cpp:178` — `local.qpn = (vfid << PID_BITS) \| ctid`. The QPN encodes the cThread id, so one QP per cThread by construction and the request's pid field selects the QP because they are the same number. Our `wr_req.pid = entry.pid (QP owner)` is that mechanism |
| **G5** minimum RDMA payload | RESOLVED BY DESIGN | No explicit `len >= 64` check exists (`rdma_req_parser.sv` handles any length; `udpLen = 12+16+payloadLen+4` is length-agnostic), but sub-64 B sits outside the exercised envelope: perf floors at `MIN_TRANSFER_SIZE_DEFAULT = 64`, `ib_transport_protocol.cpp:append_payload` carries a "TODO align this stuff!!", `lenToKeep` is `ap_uint<6>`, and jigsaw pads to 64 unconditionally. Hence the hybrid wire scheme: 64 B inline message for sub-64 B stores, direct RDMA WRITE for bulk, `len % 64 == 0` enforced for rdma bulk. Padding was never an option — it would clobber 56 neighbour bytes |

Reachability everywhere is a TLB entry under the QP owner's pid at
write-issue time. No verbs MR registration exists in this path.

Shell config: `EN_STRM 1, N_STRM_AXI 1, EN_RDMA 1, N_REGIONS 1`
(cf. `examples/jigsaw_baseline_rdma`).

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
| order FIFO + serialized engine | ordering of one issuer's stores and CE ops through one fabric port / stream | our single global FIFO is *stricter* than required (orders across issuers too); per-binding relaxation is future work |
| bulk: direct RDMA WRITE (RETH = exporter VA + off); sub-64 B stores: 64 B inline message at a staging RETH vaddr | direct: an RNIC's ordinary virtually-addressed write; inline: RNIC inline-send / NCCL LL fixed-size lines | op·len·vaddr on the wire is RDMA's own BTH/RETH for bulk; the inline header exists only because a RETH cannot say "envelope 64, true write 8" |
| reachability = a TLB entry under the owner's pid (no MR registration, no keys) | GPU scale-up model: a peer write is valid iff the owning GPU's MMU translates it; nothing registered with the fabric, no key on any bus | deliberately NOT the verbs model — matches the design's "no addresses, keys, or QPs visible" claim; bounds at the source switch, translation at the destination |
| staging buffer: small ordinary getMem of the QP owner, address exchanged at QP setup | NCCL per-connection transport buffer (plain cudaMalloc, address exchanged at connect/accept) | per G3 the shell never writes staging memory itself; the staging vaddr is addressing, and the allocation exists so the vaddr is honest and mappable |
| debug counters (stores, descs, writes, drops, fences) | engine performance counters | read via CSRs off the data path |
| stage cycle counters (free-running clock, order-FIFO residency, per-stage cycle sums + op counts) | GPU/NIC pipeline profiling counters (SM `clock64` / RNIC port latency histograms) | words 48-63; feed the T3 per-stage constants as acc/cnt deltas scaled by the vFPGA clock; measurement runs use homogeneous traffic per class |

## Deployment topology (target process architecture)

ONE real deployment shape — the two-host client/server topology;
everything else is a degenerate of it:

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

- **Hardware bring-up** (5.4/5.5, local route): the SAME binaries on one
  host under one loomd (one vFPGA = one CSR-page owner); the two sides
  collapse. A stepping stone, not a separate variant.
- **Two-host bundled** (6.2a, remote-route bring-up): one process per
  host bundling the daemon role (thread) + that side's app roles — two
  processes total. Exercises everything new in 6.x with minimal moving
  parts; the full per-side split (6.2b) is then a process-topology rerun
  on a proven data path.
- **Simulation is degenerate**: the mock backend allows one cThread per
  process, so true process splits cannot run there. The threaded
  harnesses (`roles`, `roles_sock`) exercise protocol, daemon and
  data-plane logic only — they are NOT the deployment shape.

Binaries: `loomd` exists; `app_export`/`app_import` are Phase 5.4a.

## Deliberate simplifications (and where they go next)

- **Aperture ops are <= 8 B** (one AXI-Lite beat); larger transfers use
  the DMA path. Sub-word `wstrb` is captured but writes issue as full 8 B.
- **One engine, fully serialized.** Of the classic switch machinery only
  the TX **coalescer** is still planned — it produces the T2 goodput curve
  the simulation calibrates against. Per-destination queues + scheduler
  and the error containment unit are **descoped** (2026-08 eval review:
  no isolation claim, victim experiment dead, failure containment dropped).
- **Aperture reads are local-only** (5.2). A READ rides the same order
  FIFO; the engine pulls the full 64 B aligned line and lane-selects,
  which avoids sub-line DMA entirely (cf. G5). Reads are never dropped
  (arready withheld when the FIFO is full) and always answer — invalid,
  dead or rdma-route windows return POISON rather than wedging the CPU.
  Remote loads arrive with the two-host phase (shell RDMA READ, T6). The
  credit *model* stays in simulation; AXI-Lite's single-outstanding read
  makes the hardware tracker trivially depth-1. The stall-until-answer
  load matches a GPU's non-posted peer read over PCIe P2P today — slow by
  nature, which is why the fast path stays push-only.
- **The completion count is one monotonic counter per engine.** Apps that
  serialize their own DMAs can poll for change; a caller-chosen fence
  *payload* (full CE semantics) is a small future extension.
- **Source identity is by partition convention** (no attacker model);
  destination isolation is real (pid-scoped TLB translation).
