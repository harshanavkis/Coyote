# Loom example workflow (concrete, virtual addresses only)

Worked end-to-end example of the five data flows. Every address is an
ordinary virtual address in some process's page tables; the vFPGA tables
and the Coyote shell TLB carry transactions between address spaces. No
physical address appears anywhere, and none crosses a host boundary.

## Setup phase

Processes A, B on host 1; C on host 2, all cThreads of their host's single
vFPGA. Coyote pids: A=0, B=1 (host 1); C=0 (host 2).

1. **Attach** — each process opens the vFPGA and mmaps the ctrl region.
   Its ctid is its identity toward the shell TLB for everything below.
2. **Memory "registration"** — `getMem` pins the pages and installs
   shell-TLB entries under the caller's ctid; that IS the registration
   step (no verbs MR/rkey machinery exists — a buffer is remotely
   reachable iff its owner's TLB translates it, the GPU-MMU model). Every
   QP owner additionally allocates a small **staging buffer**, the landing
   zone for inline messages.
3. **QP setup** (remote bindings only) — one RC connection per host
   pair, owned by a **dedicated QP-owner cThread** on each host that holds
   the connection and the staging buffer and never moves data (the "NIC
   context"); every XPU on the host shares it. Each side's QP owner calls
   `initRDMA` (exporter first, importer with the exporter's IP; Coyote's
   own out-of-band TCP exchanges QPNs and PSNs), and the loomd<->loomd
   hello (`loom_peer.hpp`) carries the far side's staging VA, which is
   programmed into `RDMA_STAGING_VA` — as NCCL peers exchange
   transport-buffer addresses at connect/accept. Which XPU an incoming
   write lands in is not the QP's business: the message header names it
   (step 4). Per-binding QPs remain 6.1/6.2b.
4. **Switch programming** (daemon only) — a window-table entry
   `{route, pid, dst_pid, VA base, len}` per binding, plus the staging CSR.
   Local binding: `{local, pid = exporter ctid, base, len}` — the shell TLB
   translates under that pid (the cross-pid write). Remote binding: the
   importer's daemon programs `{rdma, pid = LOCAL QP-owner ctid, dst_pid =
   exporter ctid ON THE FAR HOST, base = exporter VA, len}`: the pid
   selects whose RC connection carries the write, `dst_pid` travels in the
   64 B message header and the far `loom_rx` issues its host writes under
   it, so the far TLB translates in the exporter XPU's address space
   (`loom_bundle.hpp`). The staging CSR is per host (one QP).
5. **Handle exchange** (application level) — the exporter passes an opaque
   handle over any channel; the importer resolves it through the control
   plane and receives its window pointer. Cross-host this is the peering
   RESOLVE (6.2a): the importer's daemon asks the exporter's daemon to
   turn the handle into `{exporter ctid, VA, len}`, then programs step 4.

After step 5, no software touches a transfer.

Concretely:

- B: `buf_B = getMem(4MB)` at `0x7f1b_d420_0000` (B's own VA) — host-1
  shell TLB maps (pid 1, that VA).
- C: `buf_C = getMem(4MB)` at `0x7f9e_8860_0000` (C's own VA) — host-2
  shell TLB maps (pid 0, that VA).
- A's source buffer: `src = getMem(...)` at `0x7f6a_2000_0000` (pid 0).

| Window (ctrl byte offset) | Entry |
|---|---|
| 1 (`0x1000`) | `{local, pid 1, base buf_B, len 4MB}` |
| 2 (`0x2000`) | `{rdma, pid = QP owner, dst_pid 0 (= C), base buf_C, len 4MB}` |

A's pointers: `P_B = ctrl_map + 0x1000`, `P_C = ctrl_map + 0x2000`.

## Flow 1 — small write, local: `*(P_B + 0x40) = v`

1. A's CPU MMU translates `P_B+0x40` into the ctrl BAR → AXI-Lite write
   beat at `awaddr = 0x1040`.
2. loom_ctrl captures it into the order FIFO (win 1, off `0x40`, data v).
3. loom_engine pops, looks up window 1 → local → issues
   `sq_wr {LOCAL_WRITE, STRM_HOST, pid 1, vaddr buf_B+0x40, len 8}` plus
   one beat on `axis_host_send`.
4. Shell TLB translates under pid 1 → the write lands in B's buffer.
5. B polls `buf_B[0x40]` — its own unmodified VA.

## Flow 2 — small write, remote: `*(P_C + 0x40) = v`

Steps 1-2 identical (`awaddr = 0x2040` → win 2). The lookup says rdma, so
the switch emits ONE full 64 B wire message (never a sub-beat RDMA
payload, gate G5): `sq_wr {APP_WRITE, STRM_RDMA, pid = QP owner, vaddr =
STAGING, len 64}` with the beat `{lane0 = ⟨op WRITE_INLINE · len 8⟩,
lane1 = buf_C+0x40, lane2 = data}`. **The wire carries op·len·vaddr — the
design's message format**; the RETH staging vaddr is a data-meaningless
per-host address exchanged at QP setup. Host 2's loom_rx parses the header
and issues the EXACT 8 B write at `buf_C+0x40` under C's pid, so padding
never clobbers neighbours. Same instruction as Flow 1 on A's side;
divergence only at the table lookup.

## Flow 3 — bulk, local: `copy(P_B + 0x10000, src, 1MB)`

1. The library writes a descriptor to the CSR page: `DMA_DST = {win 1, off
   0x10000}`, `DMA_SRC_VA = src`, `DMA_LEN = 0x100000`, `DMA_SRC_PID = 0`,
   then `DMA_TRIGGER = 1` → it enters the same order FIFO as the stores.
2. loom_engine pops it and pulls `sq_rd {pid 0, vaddr src, len}` — payload
   streams in on `axis_host_recv`, the TLB translating A's pages.
3. Write side: `sq_wr {LOCAL_WRITE, pid 1, vaddr buf_B+0x10000, len}`, the
   stream forwarded through. A host-to-host copy through the vFPGA.
4. The descriptor retires when the last beat is accepted, and the engine
   releases its fence: an incrementing count written to the completion VA
   the descriptor carried (a word in A's own memory, under A's pid). The
   library polls that word — never a CSR during a DMA.

## Flow 4 — bulk, remote: `copy(P_C + 0x10000, src, 1MB)`

Same pull as Flow 3 — one `LOCAL_READ` for the whole 1 MB, the
copy-engine model; its beats land in the engine's 256 KB buffer at
whatever rate the host delivers. The write side is a Loom WRITE message
at the staging RETH vaddr: a header beat first — `⟨dst_pid 0 · op WRITE ·
len 1 MB⟩` in lane 0, `buf_C+0x10000` in lane 1 — then the payload. The
engine packetises it itself: one RoCE request per PMTU (257 packets), each
posted only while fewer than `tx_window` packets are unacked by host 2's
stack (the ACK-clocked window, CSR 66). Beats leave the buffer as packets
are admitted; the far side's acks open the window again.

At host 2 the packets arrive as 257 `rq_wr`s on the vFPGA's own host
stream (dest 1). loom_rx reads the target, **the length and the landing
pid** out of the header and issues one `sq_wr {LOCAL_WRITE, pid 0,
buf_C+0x10000+k*4KB, 4 KB}` per packet, pipelined eight deep, positioning
by the header's length — never by the RETH, which names staging. Landing
it is not optional: per gate G3 incoming writes reach memory only if user
logic issues them.

The header is what makes that one write possible. Addressing bulk by RETH —
which is RDMA's own form, and what this did until 2026-08-24 — leaves the
receiver with nothing but a per-packet cursor to write from: 256 separate
host writes for this transfer, and any packet arriving outside its message
naming its own destination. The 64 B buys one write per message and leaves
no path by which payload can address memory.

## Flow 5 — peer load, local: `v = *(P_B + 0x48)`

Reads are non-posted: the issuing CPU stalls until the switch answers.

1. A's CPU MMU turns the load into an AXI-Lite READ at `araddr = 0x1048`.
2. loom_ctrl pushes a READ entry into the same order FIFO and HOLDS the
   read channel open (rvalid deferred). Two safety rules: `arready` is
   withheld while the FIFO is full (dropping a read would wedge the CPU
   until PCIe completion timeout), and reads are single-outstanding.
3. loom_engine pops, looks up window 1, bounds-checks, and pulls the full
   64 B ALIGNED line containing the target —
   `sq_rd {LOCAL_READ, pid 1, vaddr buf_B+0x40, len 64}`, which avoids
   sub-line DMA entirely (cf. G5) — then lane-selects qword 1.
4. The 8 B travels back (rd_resp) and loom_ctrl completes the AXI read.
5. Because the READ sits in the same FIFO as stores and descriptors, a
   load issued after a store to the same location returns the fresh value.

Invalid reads ALWAYS answer: a dead window, an out-of-bounds offset or
(for now) an rdma-route window returns POISON (all-ones). Remote loads
arrive with the two-host phase via the shell's RDMA READ. The asymmetry
with Flows 1-4 is inherited from the design — writes are posted, loads
stall the issuer for the round trip, which is why the fast path stays
push-only and loads exist as a correctness/debug facility.

## Flow 6 — two XPUs per host

Add D on host 2 (pid 1) and A2 on host 1 (pid 2), each with its own
buffer. Nothing new is needed on the wire: A2's window onto D carries
`{rdma, pid = QP owner, dst_pid 1, base buf_D, len}`, so its messages
reach host 2 on the same QP as everything else and land under pid 1; C's
and D's replies come back through windows on host 2 onto A's and A2's
buffers, each with the right `dst_pid`. Locally, A2 -> A is a window
`{local, pid 0, base buf_A, len}` — the same cross-pid write as Flow 3.
Both XPUs issue concurrently: the order FIFO interleaves their entries,
each descriptor carries its issuer's ctid as the source pid, and the
transmit window is shared per host because it bounds what the far stack
can absorb regardless of sender. `run_two_host.py --matrix` exercises
every pair (`sw-bundled/loom_host.cpp`, `run_matrix`/`mx_serve`).

## Ordering across flows

Stores, descriptors and reads share one arrival-ordered FIFO, so
`copy(P_B+off, src, len); *(P_B+flag) = 1` can never show the flag before
the data, and a load issued after either observes their effects — the
engine is transaction-serialized. Across hosts the guarantee extends over
the RC connection: both windows of the 6.2a flow ride one QP, so a flag
stored after a bulk copy is delivered after the copy's payload.

## Observability

The RO stage counters (CSR words 48-63) time every flow above: word 49
accumulates order-FIFO residency (t-queue), word 50 the lookup/check
cycles (t-lookup, exactly 2 per entry), words 51-56 the per-class engine
cycles — Flow 1 in store-local (t-forward), Flow 2 in store-rdma
(t-encap), Flows 3/4 in dma-local/dma-rdma, Flow 5 in read, fences in
their own class — with completed-op counts in words 57-63. Averages are
acc/cnt deltas; T3 runs read them after homogeneous traffic per class.

## The translation chain in one table

| Address the app used | Translated by | Into |
|---|---|---|
| `P_B`/`P_C` (+offset), on dereference | A's CPU page tables (mmap of ctrl region) | ctrl window + offset |
| ctrl window + offset | loom_table entry (installed by the daemon) | `{local: (pid, base)}` or `{rdma: qp, base}` |
| `src` in the descriptor | shell TLB under A's pid, during the pull | A's memory |
| `base + offset` at the destination | shell TLB under the exporter's pid — locally the window's pid, remotely the header's `dst_pid` on the far host | exporter's memory |

Every name is one of the participants' existing VAs: A's pointers going
in, the exporter's own buffer VA coming out; the wire carries the
exporter's VA (offset in affine encoding). Nothing is invented, nothing
global.
