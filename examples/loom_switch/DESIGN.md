# Loom switch: design decisions

Why the switch and the copy engine are built the way they are: each
decision with its reason, the evidence behind it, and what was rejected.
How things work end to end is in WORKFLOW.md; how to run them, and the
results, in PLAN.md (Running).

## 1. Gets are reads of the window

An endpoint only loads and stores; the switch does the rest. A get is an
endpoint READ of an rdma window: the switch looks the address up, sends a get
request to the far switch, and answers the read with the data that comes
back (`loom_read`, WORKFLOW 6.5).

- Rejected: the endpoint stores a request word {length, return reference}
  into a get window and polls a completion word (5f710592). It reached
  11.65 GB/s for one 4 MiB get, but it puts protocol work on the endpoint.
- Gets read memory when they are issued and the switch keeps no copies, as
  over PCIe: nothing can be stale.

## 2. The copy engine reads too (DIR)

`loom_ce` CSR 6 (DIR) = 1 turns a copy around: it reads the window imported
into the V80's address space (host stream, peer-to-peer reads) and writes
HBM (card stream). A get waits until every write it issued has completed
before it writes the fence or drops BUSY, so BUSY, COPIES and the fence mean
the data is in HBM. The shell sends one write completion per request (only
for its last chunk), so the count cannot run negative. The put programs
write DIR = 0 before every copy: the register outlives a run. (ccf9ecfe)

## 3. What PCIe does to peer-to-peer traffic

Measured on amy and rose (AMD hosts, cards on socket 1):

- Every hop of the V80 -> U280 path (both root ports, the V80, the U280)
  is set to 512 B max payload and 512 B max read request; the AMD root ports
  cannot go higher.
- Yet peer-to-peer writes AND reads arrive in 256 B pieces (4-beat bursts),
  at the V80's window and at the U280's. Something below the PCIe settings
  splits them, most likely the root complex's peer-to-peer forwarding
  (inferred). Raising MPS or MRRS cannot change it.
- The V80's max read request decides only below 256 B: at 128 B the reads
  arrive as 128 B; at 512 B or 4096 B they arrive as 256 B.
- Peer-to-peer writes top out at ~10.9 GB/s (the V80 writing its own window
  through the root complex), DMA to host memory at ~12.5 GB/s. So for data
  in accelerator memory Loom runs at the platform's peer-to-peer write rate
  (V80 -> V80 across the network 11.4-11.8 GB/s at 16 MiB, 10.7-11.0
  sustained), and RDMA from host memory (12.2 GB/s) is faster because it
  does no peer-to-peer. The fair baseline is RDMA moving V80-resident data
  (not measured).

## 4. Writes become 4 KiB packets, reads cannot

A write is posted: the switch acknowledges it locally, so the V80 keeps
sending and `loom_ingress` gathers 16 consecutive 256 B writes into one
4 KiB packet (a 16 MiB copy: 65,537 bursts -> 4,096 rdma packets).

A read cannot be acknowledged until its data is there. Read bandwidth is
reads open x read size / round trip, and every 256 B read of a remote window
waits a network round trip (~6.6 us).

## 5. Reads in flight: 8 -> 32 (Phase 1)

Three places let only a few reads be open at once:

| Where | Was | Now | Change |
|---|---|---|---|
| XDMA bypass master (static region) | 8 | 32 (its max) | `cr_pci.tcl`: `c_m_axi_num_write`, which sets `C_M_AXI_NUM_READ` |
| shell crossbar into the window | 8 | 32 (its max) | `cr_ctrl.tcl`: `axi_main` and `axim_udata` `NUM_READ_OUTSTANDING` |
| far `loom_rd`, host reads ahead | 4 jobs, 8 KiB | 32 jobs, 32 KiB | `vfpga_top.svh` |

Evidence for the 8: copy-engine gets over the network ran at 0.31 GB/s =
8 x 256 B / ~6.6 us (1 MiB in 3,396 us = 512 rounds of 8 reads). Locally
(no network, reads answered on the card) 5.88 GB/s, ~43 ns per read. One
CPU thread reading the window kept ~8 lines in flight, more threads added
nothing.

- Safe: more reads open, each still fetched when issued.
- Expected (inferred): network gets ~1.2 GB/s (32 x 256 B / 6.6 us); a
  multi-threaded CPU memcpy out of the window ~0.28 GB/s at most
  (32 x 64 B / 7.3 us); one get's latency unchanged (7.3 us).
- The XDMA's own declaration of its bypass port stays 8 (its block-design
  script only updates it in another mode); the generated core has
  `C_M_AXI_NUM_READ` = 32. That the parameter governs the bypass master is
  inferred (its signals are `m_axib_*`, the bridge master's) and the HDL is
  encrypted: the local get test after the build is the proof.
- The shell's 512-deep packet-mode input FIFO (32 KiB of read data in
  flight) is left as is: 32 x 256 B = 8 KiB does not reach it.
- Needs a U280 static rebuild (`BUILD_STATIC=1`), the first in this repo.
- Tests: `tb/shell_ctrl_uwin` counts the reads that reach the window while
  none is answered (8 before, 32 after); `tb/static_pci` builds the static
  block design and checks the XDMA's parameter. (a9898fd1)

Even at 32, gets stay ~10x below puts. Reaching RDMA's 12.2 GB/s across a
6.6 us round trip needs ~80 KB in flight: ~315 reads of 256 B (the V80's
size), ~1,260 CPU lines. The XDMA allows 32; even without that limit the V80
can keep at most 256 requests open (8-bit PCIe tags; larger tag spaces need
Gen4 at both ends and the U280 is Gen3), ~9.9 GB/s. Only a shorter round
trip helps, in proportion (inferred).

## 6. A get of bytes the same host just put waits for the put's completion

Neither switch orders a read behind an earlier write to the same bytes, so
the rule is: before reading back through a window what this host just put
there, wait for that put's completion (as WORKFLOW 6.5 states). Two places
rely on it:

- Here: `loom_ingress` keeps a get behind writes whose beats it has taken,
  not behind a write whose address was accepted but whose beats are still
  in its lookup stages or behind the previous burst, so the get can leave
  first.
- At the far switch: `loom_rx` lands the put with a host write and `loom_rd`
  serves the get with a host read on another DMA engine, which can pass the
  unfinished write.

Push-based use never reads back its own puts (consumers read what was pushed
into their memory; a producer polling a far ack or credit only sees an older
value), and no benchmark does. Read-your-writes in hardware would need both
places to hold reads behind earlier writes; not built.

## 7. Read coalescer (option, not built)

A timer, restarted by each consecutive read that arrives, sends one get for
all of them when it expires. Safe (nothing fetched before it is asked for),
but it cannot raise bandwidth: at most 32 reads are open (8 KiB), and read
33 cannot come before one of them is answered. It cuts get requests and
completions 32x; build it only if Phase 1's gets fall short of ~1.2 GB/s for
packet-rate reasons.

## 8. Bulk data is moved by puts

Puts run at the platform's peer-to-peer limit; gets are for small,
latency-bound reads (7.3 us) and moderate bulk reads (up to ~1.2 GB/s with
32 in flight, section 5). Collectives already push; KV-cache transfer can be
pushed by the prefill side.

## 9. Ingress gathering of out-of-order CPU lines (deferred)

CPU write-combining buffers drain lines out of address order, so CPU puts
make ~1.5-line packets (~1 GB/s). Copy engines write in order and already
make 4 KiB packets; gathering would matter only for bulk data made of many
threads' stores. Deferred; stated as a limitation.
