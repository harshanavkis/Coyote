# Loom example workflow (concrete, virtual addresses only)

Worked end-to-end example of all four data flows through the Loom vFPGA.
Every address below is an ordinary virtual address in some process's page
tables; the vFPGA tables and the Coyote shell TLB carry transactions
between address spaces. No physical address appears anywhere, and none
crosses a host boundary.

## Setup phase (everything that happens before the first byte moves)

Processes A, B on host 1; C on host 2. All are cThreads of their host's
single vFPGA. Coyote pids: A=0, B=1 (host 1); C=0 (host 2).

The control-plane sequence, in order:

1. **Attach**: each participating process opens the vFPGA (a cThread)
   and mmaps the ctrl region. The ctid it receives is its identity
   toward the shell TLB for everything below.
2. **Memory "registration"**: allocating with `getMem` pins the pages
   and installs shell-TLB entries under the caller's ctid - that IS the
   registration step (Coyote has no verbs MR/rkey machinery; a buffer
   is remotely reachable iff its owner's TLB translates it, the GPU-MMU
   model). Exporters register their data buffers this way; every QP
   owner additionally allocates a small **staging buffer** (real,
   TLB-mapped memory - the landing zone for inline messages).
3. **QP setup** (remote bindings only): the exporter's cThread creates
   the RC QP - it must be the exporter's, because only its ctid makes
   its buffers TLB-reachable for incoming writes. The endpoints
   exchange QP metadata out of band (Coyote's TCP exchange: QPNs, PSNs)
   plus the staging vaddr, exactly like NCCL peers exchange transport-
   buffer addresses at connect/accept. In 6.1 the loomd daemons
   coordinate this exchange.
4. **Switch programming** (control plane only - loomd/orchestrator
   role): window-table entries `{route local|rdma, pid, VA base, len}`
   per binding, and the staging CSR (per host; moves into the window
   table when hosts have multiple QP owners).
5. **Handle exchange** (application level): the exporter passes the
   opaque handle to the importer over any channel it likes; the
   importer resolves it through the control plane and receives its
   window pointer.

After step 5, no software touches a transfer.

- B: `buf_B = getMem(4MB)` at `0x7f1b_d420_0000` (B's own VA) — host-1
  shell TLB maps (pid 1, that VA).
- C: `buf_C = getMem(4MB)` at `0x7f9e_8860_0000` (C's own VA) — host-2
  shell TLB maps (pid 0, that VA).
- A's source buffer: `src = getMem(...)` at `0x7f6a_2000_0000` (pid 0).
- A mmaps the vFPGA ctrl region; a control daemon programs two window-table
  entries via the CSR page:

| Window (ctrl byte offset) | Entry |
|---|---|
| 1 (`0x1000`) | `{local, pid 1, base buf_B, len 4MB}` |
| 2 (`0x2000`) | `{rdma, qp X (owner pid), base buf_C, len 4MB}` |

- A's pointers: `P_B = ctrl_map + 0x1000`, `P_C = ctrl_map + 0x2000`.

## Flow 1 — small write, local: `*(P_B + 0x40) = v`

1. A's CPU MMU translates `P_B+0x40` into the ctrl BAR → AXI-Lite write
   beat arrives at `axi_ctrl` with `awaddr = 0x1040`.
2. loom_ctrl captures it into the order FIFO (win 1, off `0x40`, data v).
3. loom_engine pops, looks up window 1 → local → issues
   `sq_wr {LOCAL_WRITE, STRM_HOST, pid 1, vaddr buf_B+0x40, len 8}` with
   one data beat on `axis_host_send`.
4. Shell TLB translates under pid 1 → write lands in B's pinned buffer.
5. B polls `buf_B[0x40]` — its own unmodified VA.

## Flow 2 — small write, remote: `*(P_C + 0x40) = v`

Steps 1–2 identical (`awaddr = 0x2040` → win 2). Lookup says rdma, and
the switch emits ONE full 64 B wire message (never a sub-beat RDMA
payload — gate G5): `sq_wr {APP_WRITE, STRM_RDMA, pid = QP owner,
vaddr = STAGING, len 64}` with the beat
`{lane0 = ⟨op WRITE_INLINE · len 8⟩, lane1 = buf_C+0x40, lane2 = data}`.
**The wire carries op·len·vaddr — the design's message format**; the
RETH staging vaddr is a data-meaningless per-host address exchanged at
QP setup (jigsaw's remote_vaddr pattern). Host 2's loom_rx parses the
header and issues the EXACT 8 B local write at `buf_C+0x40` under C's
pid — padding never clobbers neighbors. C polls `buf_C[0x40]`. Same
instruction as Flow 1 on A's side; divergence only at the table lookup.

## Flow 3 — bulk, local: `copy(P_B + 0x10000, src, 1MB)`

The CPU configures the DMA engine, then the engine moves the data:

1. Library writes a descriptor to the CSR page:
   `DMA_DST = {win 1, off 0x10000}`, `DMA_SRC_VA = src`,
   `DMA_LEN = 0x100000`, `DMA_SRC_PID = 0`, then `DMA_TRIGGER = 1` →
   descriptor enters the same order FIFO as the stores.
2. loom_engine pops it: pull `sq_rd {pid 0, vaddr src, len}` — payload
   streams in on `axis_host_recv`, TLB translating A's pages.
3. Write side: `sq_wr {LOCAL_WRITE, pid 1, vaddr buf_B+0x10000, len}` —
   the stream is forwarded through. A host-to-host copy through the vFPGA.
4. Descriptor retires when the last beat is accepted; the engine releases
   the descriptor's fence: an incrementing count written to the completion
   VA the descriptor carried (a word in A's own memory, written under A's
   pid) - the library polls it (never poll CSRs during DMA).

## Flow 4 — bulk, remote: `copy(P_C + 0x10000, src, 1MB)`

Same pull as Flow 3; the write side is a DIRECT RDMA WRITE — RETH
vaddr = `buf_C+0x10000` (the true target), len = 1 MB, raw payload, no
Loom framing (op·len·vaddr on the wire is RDMA's own BTH/RETH; the shell
fragments to PMTU). On host 2 the write is landed verbatim —
`sq_wr {LOCAL_WRITE, pid 0, buf_C+0x10000, 1MB}` via loom_rx's
pass-through (or the stock path directly; gate G3). Only sub-64 B
stores (Flow 2) use the staging-addressed inline message, because a
RETH cannot express "64 B envelope, 8 B true write".

## Flow 5 — peer load (local): `v = *(P_B + 0x48)`

Reads are non-posted: the issuing CPU stalls until the switch answers.

1. A's CPU MMU turns the load into an AXI-Lite READ at `araddr = 0x1048`.
2. loom_ctrl pushes a READ entry (win 1, off `0x48`) into the same order
   FIFO and HOLDS the read channel open (rvalid deferred). Two safety
   rules: arready is withheld while the FIFO is full (a read is never
   dropped - dropping one would wedge the CPU until PCIe completion
   timeout), and reads are single-outstanding by construction.
3. loom_engine pops it, looks up window 1, bounds-checks, then pulls the
   full 64 B ALIGNED line containing the target:
   `sq_rd {LOCAL_READ, pid 1, vaddr buf_B+0x40, len 64}` - a full-line
   pull avoids sub-line DMA entirely (cf. the 64 B minimum RDMA payload
   jigsaw ran into) - and lane-selects qword 1 of the returned beat.
4. The 8 B travels back (rd_resp) and loom_ctrl completes the AXI read;
   A's load retires with B's data.
5. Because the READ entry sits in the same FIFO as stores and
   descriptors, a load issued after a store to the same location returns
   the fresh value (read-after-write in program order).

Invalid reads ALWAYS answer: a dead window, an out-of-bounds offset, or
(for now) an rdma-route window returns POISON (all-ones) instead of
hanging the CPU. Remote loads arrive with the two-host phase via the
shell's RDMA READ. Note the asymmetry with Flow 1-4 is inherited from
the design: writes are posted (fire-and-forget), loads stall the issuer
for the round trip - which is why peer communication stays push-only on
the fast path and loads exist as a correctness/debug facility.

## Ordering across flows

Stores, descriptors AND reads share one arrival-ordered FIFO, so
`copy(P_B+off, src, len); *(P_B+flag) = 1` can never show the flag before
the data (the flag store sits behind the descriptor), and a load issued
after either observes their effects: the engine is
transaction-serialized.

## The translation chain in one table

| Address the app used | Translated by | Into |
|---|---|---|
| `P_B`/`P_C` (+offset), on dereference (store or load) | A's CPU page tables (mmap of ctrl region) | ctrl window + offset |
| ctrl window + offset | loom_table entry (installed by the daemon) | `{local: (pid, base)}` or `{rdma: qp, base}` |
| `src` in the descriptor | shell TLB under A's pid, during the pull | A's memory |
| `base + offset` at the destination | shell TLB under the exporter's pid (locally or on the remote host via the QP) | exporter's memory |

Every name is one of the participants' existing VAs: A's pointers going
in, the exporter's own buffer VA coming out; the wire carries the
exporter's VA (offset in affine encoding). Nothing is invented, nothing
global.

## Incoming RDMA (receive side)

loom_rx accepts the incoming request from `rq_wr` (pid = QP owner,
staging vaddr, wire len), parses the header beat from `axis_rrsp_recv`
(`⟨op · len · target VA⟩`), and issues exactly what it describes: the
inline 8 B write, or the header-stripped payload stream — the same
egress as Flow 1, under the QP owner's pid. (Exact interposition
semantics of the stock shell are hardware gate G3, see README.)
