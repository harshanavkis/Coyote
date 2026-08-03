# Loom example workflow (concrete, virtual addresses only)

Worked end-to-end example of all four data flows through the Loom vFPGA.
Every address below is an ordinary virtual address in some process's page
tables; the vFPGA tables and the Coyote shell TLB carry transactions
between address spaces. No physical address appears anywhere, and none
crosses a host boundary.

## Setup

Processes A, B on host 1; C on host 2. All are cThreads of their host's
single vFPGA. Coyote pids: A=0, B=1 (host 1); C=0 (host 2).

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

Steps 1–2 identical (`awaddr = 0x2040` → win 2). Lookup says rdma:
`sq_wr {APP_WRITE, STRM_RDMA, pid = QP owner, vaddr buf_C+0x40, len 8}`,
payload on `axis_rreq_send`. **The RDMA RETH vaddr is C's own VA** — the
Loom offset in affine encoding (`vaddr = buf_C + offset`; the per-binding
QP makes them isomorphic). Host 2's RoCE stack + TLB (QP belongs to C,
pid 0) write into `buf_C`; C polls `buf_C[0x40]`. Same instruction as
Flow 1 on A's side; divergence only at the table lookup.

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
4. Descriptor retires when the last beat is accepted; the engine writes an
   incrementing count to `(COMPL_PID, COMPL_VA)` — a word in A's own
   memory that the library polls (never poll CSRs during DMA).

## Flow 4 — bulk, remote: `copy(P_C + 0x10000, src, 1MB)`

Same pull as Flow 3; the write side is the RDMA request
(`vaddr = buf_C+0x10000` on C's QP); the shell fragments to PMTU. On
host 2, identical landing as Flow 2.

## Ordering across flows

Stores and descriptors share one arrival-ordered FIFO, so
`copy(P_B+off, src, len); *(P_B+flag) = 1` can never show the flag before
the data: the flag store sits behind the descriptor in the FIFO and the
engine is transaction-serialized.

## The translation chain in one table

| Address the app used | Translated by | Into |
|---|---|---|
| `P_B`/`P_C` (+offset), on dereference | A's CPU page tables (mmap of ctrl region) | ctrl window + offset |
| ctrl window + offset | loom_table entry (installed by the daemon) | `{local: (pid, base)}` or `{rdma: qp, base}` |
| `src` in the descriptor | shell TLB under A's pid, during the pull | A's memory |
| `base + offset` at the destination | shell TLB under the exporter's pid (locally or on the remote host via the QP) | exporter's memory |

Every name is one of the participants' existing VAs: A's pointers going
in, the exporter's own buffer VA coming out; the wire carries the
exporter's VA (offset in affine encoding). Nothing is invented, nothing
global.

## Incoming RDMA (receive side)

loom_rx accepts the incoming write request from `rq_wr` (pid, vaddr, len)
and forwards the payload from `axis_rrsp_recv` to a local
`sq_wr {LOCAL_WRITE}` — the same egress as Flow 1. (Exact interposition
semantics of the stock shell are hardware gate G3, see README.)
