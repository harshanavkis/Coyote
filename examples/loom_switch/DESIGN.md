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
- Measured (2026-10-06, PLAN Running): network copy-engine gets 0.45 GB/s
  (0.31 at 8), not the ~1.2 GB/s 32 in flight allow: the far responder
  now waits on its host reads, one 256 B DMA read per get (~563 ns each).
  Local gets 6.09 GB/s (5.88 at 8): ~42 ns per read either way, not bound
  by reads in flight. CPU gets do not scale past 2-4 threads (same far-side
  cost). Latency unchanged (7.3 us), puts unchanged.
- The XDMA's own declaration of its bypass port stays 8 (its block-design
  script only updates it in another mode); the generated core has
  `C_M_AXI_NUM_READ` = 32. That the parameter governs the bypass master is
  inferred (its signals are `m_axib_*`, the bridge master's) and the HDL is
  encrypted: the local get test after the build is the proof.
- The shell's 512-deep packet-mode input FIFO (32 KiB of read data in
  flight) is left as is: 32 x 256 B = 8 KiB does not reach it.
- Needs a U280 static rebuild (`BUILD_STATIC=1`), the first in this repo.
  It missed timing by 0.427 ns in the XDMA's 512b completion interface (two
  flip-flops placed far apart); re-placing it with those transceiver-clock
  crossings over-constrained (`scripts/fpga/close_static_timing.sh`) met it
  (xclk +0.076 ns). That static, locked, is
  `hw/static/static_routed_locked_u280.dcp` (md5 7ac672e9), this example's
  default `STATIC_PATH`: shell builds link against it (the coalescer image
  on it meets timing, WNS +0.001).
- Tests: `tb/shell_ctrl_uwin` counts the reads that reach the window while
  none is answered (8 before, 32 after); `tb/static_pci` builds the static
  block design and checks the XDMA's parameter. (a9898fd1)

32 is the most the XDMA allows: its master bridge (our bypass path)
"can support up to 32 active PCIe MemRd request TLPs with pending
completions" and its AXI master up to 32 outstanding reads (PG194 v3.0,
Master Bridge; `c_m_axi_num_write` choices 2..32); the shell crossbar also
tops out at 32. More reads in flight would need our own PCIe completer in
place of the XDMA's bridge in the static region.

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
  in its lookup stages, in its W buffer (section 12) or behind the
  previous burst, so the get can leave first.
- At the far switch: `loom_rx` lands the put with a host write and `loom_rd`
  serves the get with a host read on another DMA engine, which can pass the
  unfinished write.

Push-based use never reads back its own puts (consumers read what was pushed
into their memory; a producer polling a far ack or credit only sees an older
value), and no benchmark does. Read-your-writes in hardware would need both
places to hold reads behind earlier writes; not built.

## 7. Reads that continue each other share one get (read coalescer)

`loom_read` gathers reads into the open get: a read of the same window and
page that starts at the byte where the last one ended joins it. The get is
sent when a read that does not continue it arrives (failed reads included),
when it reaches the end of its page, or after `CO_WAIT` (32) cycles with no
read joining.

- Why: Phase 1's network gets (0.45 GB/s) are bound by the far responder's
  host read per get (one 256 B DMA read, ~563 ns; section 5). A page of the
  V80's 256 B reads is now one get and one 4 KiB host read: 16x fewer get
  requests, completions and far-side host reads (64x for CPU lines).
- Safe: a get only gathers reads that have arrived; nothing is fetched
  before it is asked for. It cannot raise bandwidth past the 32 reads in
  flight: ~32 x 256 B per round trip, ~1.2 GB/s (inferred).
- At most one page (4 KiB): `loom_rx` lands an answer only in the first
  4 KiB of a slot's 8 KiB return region. Larger gets would need that region
  changed; not done unless measurements ask for it.
- The answer lands in the get's first read's slot buffer and every read of
  the get takes its lines from there; done and error are per get, so a
  rejected get answers all its reads with all ones. That buffer is kept
  until the get's last read is answered: a slot is taken only if it is not
  the buffer of a read still waiting.
- `CO_WAIT` = 32 cycles (128 ns): the V80's reads arrive ~42 ns (~10.5
  cycles) apart (`ce_get --local`), so a get is not cut by ordinary jitter;
  a cut only costs one more get. A lone read (a CPU load) leaves 128 ns
  later, ~2% of the 7.3 us get latency.
- Tests: `tb_loom_switch_top` T17-T23 (a page of reads, reads that must not
  join, the timer, a page boundary, CPU loads inside lines, a rejected
  shared get, the buffer kept); 19 mutations of the new logic all caught.
  Out of context at 250 MHz: +0.95 ns, 1,422 LUTs (+240), 8 URAM (same).
- Measured (2026-10-06, PLAN Running): network copy-engine gets 0.98 GB/s
  (0.45), 16.0 reads per get, the far responder's wait for host data down
  from 129 to 14 ms of a run; a CPU load 7.39 us (7.28); puts unchanged.
- Next limit (inferred): reads now wait for free slots, which they never
  did before. With 32 reads in flight and 16-read gets, at most ~47 slots
  could be taken, so more than 32 256 B reads reach the window at once:
  likely the XDMA's 32 counts PCIe requests and splits the V80's 512 B
  requests into two 256 B reads (up to 64). Then the 64 slots, with a
  get's buffer held until its last read, bound gets now.

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

## 10. The copy engine runs programs (V80, for the evaluation)

`loom_ce` stays the emulated accelerator, but it no longer needs the host
for every copy: a program of up to 2,048 64 B entries in card memory is
loaded once into on-chip RAM (8 URAM) and run with one CSR write, as often
as wanted (`RUN_BASE` shifts every signal value and wait target, so a
program reruns without a reload). Ops: COPY, SIGNAL, WAIT, REDUCE (int32
lanes), STAMP. The single-copy CSR mode is kept, so every existing tool runs
unchanged; it runs straight from the CSRs (no fetch, no read-ahead).

- Why: a host-started copy costs a CSR round trip each (a 4 KiB put took
  3.4 us of which 0.4 us is data: puts reached 1.22 GB/s at 4 KiB), so small
  puts, a dispatch of many tokens and any multi-step collective were
  host-driven. The paper's endpoint issues its own stores; a program is
  that: the kernel's routing output in device memory, executed in order.
  WAIT gives the kernel's spin on a peer's signal, so ring algorithms run
  on the device; REDUCE is the accelerator's own compute (Loom moves data,
  it does not reduce), which makes ReduceScatter/AllReduce possible; STAMP
  gives latency on one clock (the V80's) with no host in the loop.
- Single copies cycle for cycle as before: the first image ran a START as
  a two-entry program (COPY, SIGNAL), ~7 cycles more a copy (CE cycles 459
  -> 466) and BUSY ~7 cycles later. Its 16 KiB host-started puts measured
  4.18-4.30 GB/s against 4.53 for the old image, so a START now reads its
  own source and writes the fence right after the data, the old state
  sequence. With that, a back-to-back A/B (2026-10-07) gives the same 4.60-
  4.68 GB/s and 2.1 BUSY reads a copy on both images. The metric itself
  swings 4.2-4.7 GB/s on one image: the host polls BUSY (~1.47 us a CSR
  read, the same on both images) before each next copy, so a copy's time
  comes in whole reads, and 16 KiB sits where a copy takes about two
  (12 KiB: 2.00 reads, 16 KiB: 3.00 reads host -> local memory). regress.sh
  now records BUSY reads a copy next to each put size.
- Read-ahead: card-memory sources are requested up to 8 entries ahead,
  the card stream returns them in request order. 32 x 1 KiB at 100 cycles
  of read latency: 744 cycles for 512 beats (TB P9), ~4 cycles per entry.
- Barriers (the issuer stops until the executor is past the entry): WAIT
  (later reads may need what it waits for), a write into card memory (later
  reads may read it; the executor finishes such an entry only once its
  writes completed), a host-stream read (it would queue behind read-ahead
  requests whose data is only taken after it). A card SIGNAL waits for all
  earlier writes' completions; a host-stream SIGNAL is posted behind the data
  (loom_ingress keeps the order).
- Landing order, not enforced: a WAIT polls its flag on the copy engine's
  card channel, while the peer's data and flag land through the window
  (`uwin_hbm`, one AXI ID, in order) on another HBM port. Bursts to other
  HBM channels could in principle still be in flight when the flag is
  visible. A hold in `uwin_hbm` (a single-beat write waits for every earlier
  write's B) was written and tested, then rolled back (owner, 2026-10-07):
  a flag ahead of its data has never been seen in practice (every landing
  checked byte-exact after its fence). If a WAIT ever reads stale data, this
  is the first suspect.
- REDUCE reads a and b on the one card stream, alternating 4 KiB chunks
  (a's chunk held in LUTRAM), so it outputs at most half the card read
  rate (~8 GB/s), below the link. A second card channel would lift it;
  not needed for the collectives' rates.
- Rejected: repeating one descriptor (`--burst` already does, the host
  round trip per copy stays); a descriptor ring the host keeps feeding (the
  host stays in the loop); fetching entries from HBM one by one (an HBM read
  per entry, ~0.5 us); in-network reduction (not Loom's claim).
- Tests: `tb_loom_ce` P1-P9 (card memory modelled, writes visible only at
  completion), T1-T8 unchanged; mutations of the barriers caught (P4, P8).
  Out of context
  (V80, 250 MHz): +0.92 ns (the REDUCE chunk arithmetic registered: -0.83
  before), 2,997 LUTs, 8 URAM.
- Software: `sw/src/ce_prog.hpp` (entries, load, run); `ce_prog` tests it on
  one V80 (REDUCE, WAIT, ABORT, rates program vs host-started).

## 11. Peers' reads of the V80's window: 8 -> 64 in flight (shell)

The fair RDMA baseline moves V80-resident data: the U280's RDMA engine
reads its payload peer-to-peer from the V80's window. It ran 3.7 GB/s at
every write size (2026-10-07), against 11.9 from host memory.

- Peer-to-peer reads reach the V80 as 256 B pieces whatever the requester's
  max read request (section 3), so their bandwidth is pieces in flight x
  256 B / round trip. Test (runtime max read request on the sending U280,
  1 MiB writes): from V80 HBM 512 B 3.73, 256 B 3.72, 128 B 2.34 GB/s; from
  host memory (control) 11.9, 8.0, 4.0. A bandwidth limit would not fall
  with smaller reads; a fixed count with a fixed round trip would halve
  (37% here, the round trip ~20% shorter for smaller pieces): a count.
- The count, along the window's read path (resolved IP parameters of the
  built images): the static's NoC entry from the PCIe block 64, its NoC exit
  and both SmartConnects 32 (its `axi_main` port declaration of 8 is not
  what the SmartConnect uses); the shell's interconnect 8 (from
  `cr_ctrl.tcl`'s `axi_main` and `axim_udata` declarations); `uwin_hbm`'s
  read queue 16. The shell's 8 bound it.
- Now: the shell's declarations 64, `uwin_hbm` N_RD 64 (TB T8: 64 taken,
  the 65th held). The static (not rebuilt) caps the path at 32; raising it
  needs a static build (`cr_pci.tcl`, and its NoC read_bw for the window,
  1000 MB/s, writes 12800). With 32 x 256 B in flight the round trip
  allows ~7.4 GB/s at 1.1 us, line rate at <= 0.7 us (unmeasured).
- The requester's max read request goes to 4 KiB for the baseline (runtime
  setpci): the U280 has ~32 requests in flight (from host memory: 31 per us
  of round trip), so larger requests put more 256 B pieces in flight.

## 12. W beats are taken ahead of their burst's lookup (W buffer)

`loom_ingress` takes W beats from the bus into an 8-beat buffer whether or
not their burst has been looked up, and its burst stage takes them from the
buffer's head. Before, a beat was taken only once its burst was loaded.

- Why: bulk writes reach the ingress as 256 B bursts (4 beats, section 3),
  and the ingress counter "W wait: no aw" (a beat presented, no burst looked
  up yet) was ~2.0 cycles per burst in every bulk run: PTDMA 64 MiB x 5
  2,622,210 cycles for 1,310,725 bursts, chunk sweep 5,358,220 / 2,621,450,
  the V80 engine 1.85. 4 + 2 cycles per 256 B at 250 MHz is 10.67 GB/s,
  Loom's put ceiling (V80 engine 10.4-10.9, PTDMA 10.0-10.5) against 12.2
  for RDMA on the same link; the output-side waits were small and packets
  full 4 KiB.
- Cause: an AW reaches the burst stage four cycles after its handshake
  (three lookup stages, then the load), and wready needed the burst loaded.
  The shell's interconnect hands AW over about two cycles before the
  burst's first beat (its S00 data FIFO is in packet mode, so it passes a
  burst on once it holds all of its data; inferred), so every burst waited
  about two cycles for its own lookup. The header's "bursts follow each
  other with no gap" held only for AWs running ahead of their data, as the
  TB's master sent them.
- Hardware (2026-10-12, image loom-switch-wbuf, 36a57a37; full regress
  passes): the rate did NOT change. ce_local V80 -> U280 -> host 10.64
  GB/s and -> V80 HBM 10.62 (both before and after), the V80 engine's
  remote puts 10.71, PTDMA into the window 10.45-10.52 (4 MiB chunks) and
  10.03-10.13 (1 MiB) as before. The ~2 cycles a burst were real but did not
  set the pace: the upstream delivers a 256 B burst about every 6 cycles
  anyway, so 4 + 2 = 6 matched the ceiling by coincidence. Writes into the
  U280 stop at ~10.5-10.7 GB/s for both senders upstream of loom_ingress
  (the U280's PCIe receive path, the shell's interconnect or the root
  complex's peer-to-peer forwarding; not split yet); the V80 writing its own
  window stops at 10.93 (section 3).
- TB (`tb_loom_ingress` rate tests, 64 back-to-back 4-beat bursts, cycles
  per burst, before -> after): AW with the first beat 8 -> 4, 1 beat ahead
  7 -> 4, 2 ahead 6 -> 4 (the hardware's case), 4 ahead 4 -> 4; AW only
  1 / 2 / 4 cycles after the previous burst's last beat was taken 8 / 9 /
  11 -> 4. With W up to three cycles ahead of its AW the buffer keeps one
  beat a cycle; further ahead it fills and the bus waits (12 cycles: 6.67).
- Order and B, as a producer sees them, unchanged: taken now means taken by
  the burst stage from the head, B is answered there, and a get still goes
  behind every write whose beats were taken before it, so behind every
  write answered before it. Beats in the buffer whose burst is still in the
  lookup are not taken; a get may pass them, as it could pass beats held on
  the bus before (section 6). A lone burst's beats are taken by the burst
  stage in the same cycles as before, or one cycle later if its AW led by
  four cycles or more, and B comes as many cycles after AW as before; in a
  stream both come earlier (no gaps).
- Debug counters 98-102 now count cycles the head beat waits; the bus waits
  only while the buffer is full. T24's "W ->" latencies read 3 cycles more
  (W is taken on arrival); from AW they are unchanged.
- The strobes are kept as two bits per 8 B word (529 bits a beat, not 577),
  which also takes the 64-bit strobe AND off the path into the data FIFO's
  write enable (before: the critical path).
- Rejected: a fast path for an AW that continues the previous burst (same
  binding, next offset) past the table: address and bounds comparators,
  still a cycle late when AW comes with its data, and nothing for bursts
  that do not continue. Answering B when the last beat enters the buffer:
  needs a queue of AW IDs and the get order over the buffer, for no gain
  (B is not what limits the rate). Fewer lookup stages: still a gap per
  burst unless the lookup took zero cycles.
- Tests: the suite runs with the free master, AW with its first W beat, and
  AW two cycles after the previous last beat (W ahead of AW), each with and
  without backpressure; new runs: a discarded burst inside a run, get
  requests in a stream (each checked behind every write answered and every
  beat taken before it arrived; packets cut only where a get left), the
  rates. 8 mutations of the new logic all caught (overflow past a full
  buffer, popping untaken beats, the bus's wlast or wdata for the head's,
  the B slot on the bus's wlast, the strobe decode, a buffer two short,
  a get not waiting for the open packet). Out of context at 250 MHz
  (loom_table + loom_ingress, every port registered, placed and routed):
  +0.50 ns (+0.61 before; post-synthesis +1.00, +1.46 before), 1,074 LUTs
  (+324, of them 314 LUTRAM for the buffer), 710 FFs (+15), 9 RAMB36 and
  0 URAM (same). The critical path is now the buffer's read pointer through
  the head's take into the lookup stages' enables (8 levels).
