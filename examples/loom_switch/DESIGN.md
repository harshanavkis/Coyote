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
- Peer-to-peer writes stopped at ~10.9 GB/s into the V80 and ~10.6 GB/s into
  the U280, DMA to host memory at ~12.5 GB/s. Neither is the root complex:
  - V80: its shell's interconnect allowed 16 writes in flight (section 13);
    with 32 the V80 takes what its sender gives (12.35-12.56 GB/s).
  - U280: a 256 B piece every 6 cycles at 250 MHz from every sender. In
    simulation (the static's XDMA with AMD's root-port model) XDMA's bypass
    master holds the request interface one idle cycle per write on top of
    the 5 cycles a 256 B request occupies (16 B descriptor + 256 B); 6.0
    cycles, as on hardware. Not fixed: the window path has to leave XDMA's
    bypass (e.g. QDMA's bridge master).
  Until the U280 is fixed, Loom V80 -> V80 runs at the U280's receive rate
  (10.7 GB/s sustained).

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

## 13. Peers' writes into the V80's window: 16 -> 32 in flight (shell)

The V80 shell's SmartConnect (`axi_main` -> `axim_udata_0`,
hw/bd/versal/cr_ctrl.tcl) is declared with 32 writes in flight on both
ports (was 16, 6aad1a7a). The static is unchanged (loom-ce-static_ot16; its
own SmartConnect already allows 32).

- Why: every write into the V80's window stopped at 10.9-11.0 GB/s (V80
  self-loop 10.93, RDMA U280 -> V80 11.02, Loom -> V80 HBM 10.6). The window
  counters (uwin_hbm) showed 9.6 writes in flight on average, at most 16,
  AW->B 56 cycles; at `axi_main` AW waited 83% and W 46% of the cycles.
- Cause (simulated, ic_rate_sim v80_path: uwin_ra, the generated
  SmartConnect, the 3 + 3 register slices before uwin_hbm, B 49 cycles after
  the last beat = the measured AW->B 56): the SmartConnect issues at most 16
  writes and each holds its slot ~94 cycles (56 to HBM and back, 12 in the
  slices, ~25 inside the SmartConnect); 16 x 256 B / 94 cycles = 10.9 GB/s.
  The model gives the hardware's rate, writes in flight, AW->B and both
  `axi_main` stalls on the uwin_ra images with W_LAG 0 and 3 alike (with W
  leading its AW by up to 2 bursts, as the static sends it), so W_LAG (W
  three cycles behind AW, 0dedea02) could not help: W/AW timing was never
  the limit. 6aad1a7a's sizing (16 cover ~11 GB/s at 236 ns a write) used too
  short a round trip. Without the slices the model ran at 12.6 GB/s.
- Simulated with 32 (and 64): 4.0 cycles a burst (16 GB/s at the window),
  also with the static limited to 24 or 32 writes in flight.
- Hardware (2026-10-13, image loom-ce-sc32, 01ff1973, WNS +0.013; full
  regression passes): V80 self-loop 10.93 -> 12.35 (amy) / 12.56 (rose)
  GB/s, the same as the engine writing host memory (12.37 / 12.57): the
  sender now sets the rate. Window: 12.3 writes in flight on average (26 at
  most), AW->B 62; `axi_main` stalls 0%, waits for the static 25%. That is
  5.08 cycles per 256 B write through the V80's own QDMA bridge master.
  Program puts landing in V80 HBM +6-14%; paths through the U280 unchanged
  (section 3).
- Rejected: uwin_hbm answering B at a write's last beat (simulated also 16
  GB/s, even with only 16 writes in flight to HBM): it changes ordering at
  the window (reads would have to wait for earlier writes' HBM B) for
  nothing a declaration change does not give. 32 in the static as well
  (9aad9e58): failed static timing at -1.75 ns and is not needed.

## 14. The U280 static on the soft QDMA (STATIC_DMA=qdma)

The U280 static can be built with the soft QDMA 5.0 in place of XDMA 4.1
(`-DBUILD_STATIC=1 -DSTATIC_DMA=qdma`; hw/bd/ultrascale_plus/cr_pci_qdma.tcl,
hw/templates/ultrascale_plus/static_top_qdma_tmplt.txt). Same PCIe block,
quad and link (PCIE4C_X1Y1, Gen3 x16, 512 bit at 250 MHz), same BAR order
(BAR0 AXI-Lite -> static config, BAR2 DMA, BAR4 256 MB AXI bridge master ->
`axi_main`, 32 writes and 32 reads in flight), same static -> shell ports, so
shells link against it unchanged. Device ID 913F; the driver for it is built
with `make TARGET_PLATFORM=ultrascale_plus DMA=qdma` (driver/build_qdma,
pci_qdma.c) and setup_loom_hosts.sh loads it when the card shows 913F.

- Why: writes into the U280's window stopped at 10.66 GB/s from every sender
  (6.0 cycles per 256 B write). Simulated (xdma_sim: the static's exact XDMA
  4.1 with AMD's root-port model, an always-ready sink on the bypass master):
  XDMA's bypass intake takes 5 CQ beats per 256 B TLP and holds its tready low
  one more cycle per TLP = 6.000 cycles; the hard block, link, shell and
  ingress are not it.
- Evidence for the QDMA (hardware, 2026-10-13, amy's U280 with the QDMA
  example design, the bridge master on an always-ready sink with counters,
  qdma_hw/): PTDMA host -> BAR4 at 12.25 GB/s (8 channels; XDMA image, same
  runs: 10.61), every write a 256 B burst, 5.19 cycles per burst while busy
  (81% 5 cycles, 19% 6, the same for 1 and 8 sender channels) = 12.33 GB/s;
  XDMA 6.00. The V80's CPM5 QDMA bridge took 5.08 (section 13).
- Data path as on the V80: AXI-ST H2C/C2H queues with descriptor bypass
  through qdma_rd_wrapper / qdma_wr_wrapper / qdma_reg_slice /
  qdma_interrupt_wrapper; the PR channel is an ordinary H2C stream channel
  (queue 3) to the ICAP, so there is no MM engine and no PMC path. The driver
  skips the CPM5-only host profile, MM PR queue and H2C MM engine on
  UltraScale+.
- Not simulated: AMD's soft-QDMA example design hangs at t=0 in xsim 2023.2
  (unmodified, too), so the static is checked by its block design
  (tb/static_pci/run.sh qdma), synthesis and hardware.
- Prefetch cache 64 deep (7cb6023d): every C2H queue the static writes with
  needs its own prefetch tag (simple bypass); qdma_wr_wrapper uses
  N_OUTSTANDING (8) queues per host channel, 3 channels = 24 tags. The IP's
  default 16-entry cache grants 15 (measured), so the first QDMA image never
  completed a C2H write. The driver now enables 32 C2H queues and refuses to
  load if a tag is not granted for its own queue.
- Checked before the static was rebuilt (2026-10-14, qdma_hw/rev2: the
  static's QDMA configuration and its unchanged wrappers behind a shell
  stand-in, measured on amy): all 32 tags granted, C2H on all 24 queues with
  the granted tag, C2H/H2C 64 B / 4 KiB / 100 B byte-exact, C2H 4 KiB 14.8
  GB/s, H2C 14.2, both at once 13.0 + 13.0, bridge 12.23. C2H costs ~7.4
  cycles per descriptor: 64 B writes 34 M/s (2.2 GB/s), 256 B 7.3 GB/s.
- The driver maps only the part of BAR4 below the user data window into the
  kernel (f8f43110, as pci_xdma.c): an uncached kernel mapping over the window
  made PAT downgrade the CPU's write-combining window mapping, and CPU stores
  reached the window as 8 B writes (CPU puts 0.08 GB/s, gets 0.004).
- Result (loom-switch-qdma64, standard U280 image since 2026-10-15; full
  regression regress/20261015_loom-switch-qdma64-kmap against the XDMA image,
  measured): V80 HBM -> peer V80 HBM 10.7 -> 12.26 GB/s, V80 -> peer host one
  way 10.9-11.2 -> 12.4-12.8, V80 -> U280 -> host 10.64 -> 12.30, program puts
  +8..25%, both directions at once unchanged (~10.8-11.0). Timing: WNS -0.464
  ns, worst paths inside the QDMA IP's PCIe interface (xclk -> PCIe core);
  no errors in either regression.

## 15. Peers' reads of the V80's window may come out of order (uwin_ra WINDOW)

The RDMA baseline pulling from V80 memory (U280 perf_rdma on the QDMA static,
`landing_client --contig 1 --read-ahead 2`) reached line rate from 256 KiB but
not below: V80 -> V80 7.1 / 8.5 / 9.7 / 11.5 GB/s at 4 / 8 / 16 / 64 KiB
(best of 10, Loom 12.2 at every size).

- Cause (measured, e1/20261015_2037_ra_modes and _2043_ra_pace): uwin_ra's
  stream (and uwin_hbm's) ends on any read that does not continue the
  previous one, and the U280's reads reach the V80 out of order 0.24 / 0.29 /
  0.32-0.43 / 0.50-0.56 times per write. Each break costs ~1.2 us (the
  passed-through reads wait behind the fills in flight; a fill is out ~1.6 us):
  time per write = size / 12.25 GB/s + breaks x 1.2 us fits the measured
  rates within ~8% at every size (computed). With one 4 KiB read command in
  flight at a time (writes paced >= 1.5 us apart) the breaks vanish (0.001
  per write); with 16 or 64 KiB writes (4 or 16 commands back to back) they
  stay: the requester's concurrent 4 KiB commands interleave, within a write
  or across writes. A 4 KiB max read request on the U280 (runtime setpci)
  cuts the breaks 2.5-4x but caps 64 KiB at 10.5 (e1/20261015_2057_ra_mrrs).
- WINDOW streams (counter-page line 57; 56 = STRICT, the reset default): a
  window read of the stream's ID whose lines lie between s_req - RA_LAG (192
  lines, not below the stream's start) and s_req + RA_AHEAD (256) belongs to
  the stream in any order (s_req: the furthest end of its reads). The ring is
  read per read (gq: first line, end, floor; a walker in AR order) instead of
  as a FIFO; fills land at most RA_RING lines beyond the floor of the oldest
  read not yet read out (its first line, or the s_lag it was taken against),
  so a lagging read's lines are never overwritten (TB mutation without it:
  wrong data in T19) and RA_AHEAD + RA_LAG + RA_BURST <= RA_RING keeps the
  fills a read waits for always allowed.
- Counters 77-87 (uwin_ra words 13-23): WINDOW selected; out-of-order stream
  reads; by distance from s_req (8 bins); taken. They show the real pattern
  in either mode.
- TB (tb_uwin_ra T15-T20, model): a NIC's pattern (half the 4 KiB boundaries
  interleaved, reads up to 20 late) stays one stream in WINDOW; at the V80's
  latencies with a quarter of the boundaries interleaved STRICT runs 7.8
  GB/s, WINDOW 14.7 (the 8-in-flight model's ceiling; STRICT sequential 14.7,
  was 15.7 before the walker: ~2 cycles more a hit).
- Rejected: STRICT with a longer idle or a larger ring (the break is the
  order, not the depth); several strict streams (a single stream reordered
  is not several sequential ones); fewer reads in flight on the U280 (one
  command at a time = 2.7 GB/s at 4 KiB, measured).
- Prediction (computed): 4-64 KiB from a V80 source at ~12.2 GB/s best, if the
  real offsets fit the window (the counters say).
- Result (loom-ce-rawin, 0fe84ab0, WNS +0.042; full regression
  regress/20261015_loom-ce-rawin passes, Loom paths unchanged; measured
  e1/20261015_2354_rawin, U280_RDMA, best / median of 10, byte-exact):
  V80 -> V80 WINDOW 12.21/12.20, 12.23/12.23, 12.23/12.05, 12.25/12.25 GB/s
  at 4/8/16/64 KiB (STRICT on the same image 6.22/4.79, 8.86/7.95,
  9.35/9.28, 11.28/11.22), 12.25-12.32 from 256 KiB (STRICT 12.10-12.26);
  V80 -> host the same (WINDOW 12.21-12.25 best at every size). The real
  pattern (counters, 10 x 64 MiB in 4 KiB writes): 154,379 out-of-order
  reads (0.94 a write), all within 4 KiB of s_req (87,704 at -16..-1
  lines, 62,008 at 1..16, 4,049 at -64..-17, 618 at 17..64), all taken;
  2,621,365 of 2,621,412 stream reads hits, axi_main read 36 ns, 1 break
  in 10 runs (STRICT ~3,500 a run). The pull from V80 memory now runs at
  line rate at every size, as Loom's push (12.16-12.22).

## 16. Per-packet stamps in both U280 images; rdma_flow's window per QP

The 8 B breakdown of 2026-10-22 (e2/20261022_1443_breakdown) split one-way
times by subtraction of host-side measurements (READ - WRITE = the network,
an even split of Loom's store write + landing): the U280 only counted events
and waits, nothing timed a packet. RoCE BALBOA's breakdown (arXiv version,
Fig 5) times each block first byte in -> first byte out; the same idea at the
vFPGA's edges:

- stamp_rec (examples/loom_switch/hw/src/hdl, the same file in
  examples/09_perf_rdma/hw/src/hdl): 9 points, each a ring of 4,096 32-bit
  cycle counts (since the last arm, aclk 250 MHz) = the point's last 4,096
  events; counts since the arm; CSR words 224-240 in both images (loom_ctrl
  here, stamp_csr in perf_rdma). The same points in both: 0 the packet's
  data enters (Loom: first W beat of a uwin burst; perf_rdma: first beat back
  from the shell's host read), 1 Loom's rdma request posted (perf_rdma: the
  post is in the shell, never fires), 2 first beat to the stack, 3 remote
  ack, 4 rq_wr, 5 first beat from the stack, 6 landing write posted, 7 first
  landing beat out, 8 landing write completed (cq_wr, local). Landing beats
  carry no tlast in Loom: they are paired with the landing writes in post
  order (len / 64 beats each).
- vFPGA only, no shell change for stamps: the stacks, CMAC and wire are one
  segment, measured from a ping-pong without a common clock as
  ((A_rx - A_tx) - (B_tx - B_rx)) / 2; PCIe + CPU stay one residual.
- TB: tb_stamp_rec (rings wrap, re-arm, events in the same cycle), T25 in
  tb_loom_switch_top (every stamp difference = the cycles the TB saw, 5
  landings in a row), tb_perf_rdma_stamp. OOC: 36 BRAM tiles, 138 LUT, WNS
  +2.0 ns at 250 MHz (post-synthesis).
- rdma_flow (shell, hw/hdl/network/rdma): its pointer RAM reads one cycle
  late, and *_LUP_WAIT drove the address to 0, so every QP's lookup used QP
  entry 0's head / tail / issued (and its offs). TB (xsim, 20 WRITEs, no
  ACKs): QP 1 alone passed 20 (no window at all while QP 0 is idle), 25 after
  one ACK; QP 0 16. Fix: hold the looked-up address through *_LUP_WAIT
  (16 / 17 / 16). QP 0 reads the same entry as before, so every 2-rank result
  stands; ranks with several QPs per card (3 ranks, 2 per host) need it.
  RoCE BALBOA's rdma_flow.sv is the same file (its multi-QP figure used
  32 KiB READs, 8 packets, under the 16-packet window).

## 17. Reads: read-ahead, one packet per get, the slot count (2026-10-25)

Measured first (read_probe + stamps, e2/20261025_2254_rdprobe and
_2300_rdlocal; amy's PTDMA reads rose's host memory through window 1, the
stamp image): a PTDMA window read of 64 B took 9.34 us = submit -> get out
1.55 + the get's round trip 6.33 (rose's host read 1.21, wires and stacks
5.05) + completion -> engine done 1.45. Large reads ran at 1.0 GB/s: at most
3 gets out at once (~2.7 KiB each) x 7.3 us = ~8 KiB = 32 reads x 256 B, the
U280 bridge's reads in flight (sections 5, 14). With the switch answering
at once (a local window: all ones), the same engine reads the window at
12.3-12.7 GB/s (1-16 MiB), RDMA READ's rate: the engine and PCIe are not the
limit, the round trip in front of every 32 reads is.

- Read-ahead (loom_read, revived from the 2026-10-05 version that was set
  aside as too complex, ~/loom-experiments/readahead-archive): byte-
  consecutive INCR reads of one rdma window form a stream where, a whole
  page of them already, they enter the next page; the window is fetched
  ahead in pages (blocks, a 32-page ring next to the 64 read slots in the
  answer buffer) and the reads are answered from the blocks. The
  prefetcher starts 4 pages ahead and doubles that each time the reads
  enter a new page (a short transfer fetches little past its end; a long
  one keeps up to 32 pages, ~22 needed at 12 GB/s x 7.3 us).
  Staleness: the stream is dropped by any read that does not continue it
  (the same bytes again included: a polling reader always gets a fresh
  get), any packet or store landing on this host, and any write into the
  uwin; fetches after a drop are fresh, reads attached before it are
  answered from their blocks (issued before the reader could know). A new
  stream waits for the old blocks to drain and, after a landing or a uwin
  write, 512 quiet cycles. CPU loads (lines twice, out of order) rarely form
  streams.
  The first build (loom-switch-reads, 2026-10-26; measured
  e2/20261026_0830_rdprobe_ra, _0836_reads, _0843_rdprobe_tune) started a
  stream on the second consecutive read, 2 pages ahead, and a drop by a
  read also started the quiet time. Two costs: a PTDMA read of 256 B-1 KiB
  waited for a whole page (and the next) behind its own first get, 0.7-1.0
  us slower than without read-ahead (a read of a page or less is all in
  one burst of the engine's 32 reads: read-ahead cannot help it, the
  coalescer fetches it exactly); and every transfer's first read drops the
  last transfer's stream, so the engine's whole first burst passed in the
  quiet time and read-ahead began a round trip late (16 KiB equally slow
  2-16 pages ahead; stamps: 3 gets in the first 2 us, the rest after the
  first answers). RD_CTL bit 9 restores the first start rule.
  Still open: after a long stream up to 32 pages fetched ahead are in
  flight, and the stream of a transfer started right after it waits for
  them (a new stream reusing their ring entries would need a generation per
  block).
  RD_CTL bit 8 turns read-ahead off (A/B on one image); [21:16] / [29:24]
  lower the most pages ahead / the pages ahead at a stream's start; counters
  199-205.
- One packet per get: a get answering a read (return export RRET_EXP) is at
  most 4 KiB, one PMTU, so loom_rx completes it at the packet's last beat and
  loom_rd sends no completion store (a rejected get still gets the all-ones
  store; gets back into an export keep theirs). Why: the 16-packet ack window
  is card-wide and an ack returns 4.7-5.2 us after its packet (stamps), so
  two packets per 4 KiB get would cap any read-ahead at 8 x 4 KiB / 5.2 us =
  ~6.3 GB/s; one packet allows ~12.6 GB/s. It also halves the packets of
  CPU-load gets (1 KiB of loads: 16 gets x 2 packets > 16 waited ~5 us for
  acks) and removes the 64 B completion packets.
- The slot count: `(SLOT_W+2)'(wp - rp)` is evaluated in the cast's width
  (8 bits), so once the 7-bit wp wrapped and rp had not, 128 + the reads out
  counted as used and AR stopped until every older read was answered: a
  pipeline drain every 128 reads (the slot waits counted even at 1 KiB, 5
  reads; ~16% of a 1 MiB read). Fixed with a 7-bit intermediate (xsim showed
  the one-line cast fix computes the same wrong value).
- Software: get_bench times a PTDMA read to the last 8 B of the destination
  (the flag in the data, as RDMA READ is polled), 0.5-0.6 us before the
  engine's head register; read_probe (one size, card time line, --local,
  --ra-off).
- Tests: tb_loom_switch_top T11-T25 with read-ahead off (as before; the
  loopback now answers each get with one packet); T26-T36 with the first
  build's start (RD_CTL bit 9, 2 pages ahead): a stream to the window's
  end; drops by a store, a packet, a write, a repeated read; rejected
  blocks and a partial page; the ring wrapping with 256 B reads and a
  reader ahead of it with 4 KiB reads; pages wanted after a drop; the
  ramp's bounds from RD_CTL; no stream while a dropped one's blocks are out
  (a late old block would answer a new page's reads with stale data: the
  mutation without that rule fails T36 on data); bit 8 dropping a running
  stream. T37 the default start (one page, 2 KiB across a page, 4 KiB
  reached mid-page: no stream before the next page), T38 no quiet time
  after a drop by a read, T34 16 reads in flight across the pointers' wrap.
  Mutations: 21 of 21 caught (the slot count, the packet-end completion,
  each drop, the room check, the attached-read count, the ramp and its
  bound, ring idle, pages wanted after a drop, the same bytes again, bit 8;
  the start rule without its page, its run, the run's reset, the first
  rule; the quiet time after a read; the start depth). T25's ack check
  compared with the packet's first beat, which the mock may ack in the same
  cycle; it now compares with the post.
- Rejected: more reads in flight at the bridge (a static rebuild; even 256
  x 256 B per 7.3 us is ~9 GB/s, computed); bigger PTDMA reads (P2P reads
  arrive in 256 B pieces whatever MRRS, section 3); a larger card-wide ack
  window (32 costs the collectives, 10-25 lwin).

## 18. rdma_flow's window per QP: 16 -> 64 packets (shell + HLS, 2026-10-26)

- Why: Loom answers every get with its own WRITE request, so rdma_flow's 16
  outstanding requests per QP bound a reader at 16 x 4 KiB per ack round trip.
  With the round trip measured under two-way load (~6.1 us, e2/
  20261026_0843_rdprobe_tune) that is 10.7 GB/s (computed), and read build 3
  measures 10.7 GB/s at 16 MiB (e2/20261026_1855_reads); a 64 KiB transfer
  takes 48 gets (read_probe counters), three windows. RDMA's own results are
  not window-bound: its 4 KiB+ WRITEs reach line rate (5.2 us round trip ->
  12.5 GB/s allowed), READ responses bypass rdma_flow, and 256 B READs are
  post-bound (0.24 GB/s, half of what 16 would allow).
- Change: RDMA_N_WR_OUTSTANDING 64 (hw/templates/common/lynx_pkg_tmplt.txt).
  The slot index travels in OFFS_BITS = 6 everywhere in the RTL and in the
  HLS memCmd / txMeta, but the HLS retransmitter kept it in 4 bits
  (retransAddrLen, retransEntry, its meta table, retransEvent): with more than
  16 slots a retransmission would resend slot n mod 16, and the receiver would
  accept it (the ICRC covers what is sent). Widened to 6 bits (hw/services/
  network, ib_transport_protocol.hpp, retransmitter.hpp). rdma_flow's pointer
  RAM (16 bits) holds the 13 it needs; the retransmitter's table has 2,000
  entries.
- TB (examples/09_perf_rdma/hw/tb/roce2, run_window_lossless.sh): every
  landed byte checked against its address; lossless traffic as on the testbed
  (no 64 B-multiple packets, the icrc.sv bug). Window 16 vs 64: 1 QP over a
  2,000-cycle link 34,306 -> 9,916 cycles (16 -> 64 outstanding); 2 QPs x 64 KiB
  both ways 8,400 -> 7,092; 4 QPs x 4 KiB both ways 15,760 -> 13,450; all PASS,
  0 wrong bytes, 0 drops. A corrupted landed byte fails the check (control).
- Not testable yet: a retransmission. After any loss the stack freezes at
  every window (HLS D1/D2, 2026-10-25), so the 6-bit slot cannot be shown
  working under loss until D1/D2 are fixed. Also open (code reading,
  unconfirmed): rdma_mux_retrans shifts the buffer address left by
  RDMA_MEM_SHIFT (27), which puts the slot at bit 39+, while the U280's HBM
  port takes ARADDR[32:0]: every slot may alias.
