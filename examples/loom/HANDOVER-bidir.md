# Handover: the bidirectional bandwidth problem

Written 2026-09-20 for whoever takes over the Loom prototype's performance
work. Everything below was measured on the two-host testbed (clara =
client, amy = server; U280 on each; see README "Running" and
`run_two_host.py --help`). The deployed bitstream is
`hw/build_sep16_arbiter_fixed` (md5 4bb3b847), built from commit
ad88e9f0's RTL, which is the RTL at HEAD (nothing in `hw/src` changed
since). Software is HEAD on both hosts.

## The problem in one paragraph

A single direction of Loom traffic moves 11.8 GB/s (`--size 67108864
--iters 20 --tx-window 32`), the same as stock perf_rdma and ~96% of the
100 GbE payload rate at 4 KB packets. Two directions at once move 5.9
GB/s **each** (`--bidir 64`): the two directions share the one-direction
ceiling although the link, the PCIe bus and the host DMA are all full
duplex. That is not a Loom limit - stock perf_rdma with both sides
writing at once (`PERF_RDMA_BIDIR=1`) gets 4.1 + 6.3 GB/s - and it is not
a hardware limit either; it is a per-packet cost somewhere in the shell's
request path, measured at ~100 cycles (0.4 us) per packet **regardless of
the packet's size**. Fixing that cost lifts both the single-direction
ceiling (toward the DMA's 12.8/14 GB/s) and the bidirectional one.

## What is established (each line is a measurement, not a guess)

| # | measurement | tool | number |
|---|---|---|---|
| 1 | one direction, 64 MiB x20 | push bench | 11.84-11.90 GB/s, 0 retransmissions |
| 2 | both directions, 4 MiB each way, 64 rounds | `--bidir 64` | 5.9 + 5.9 GB/s = 11.76 aggregate, symmetric, byte-exact |
| 3 | same at W=40 and W=48 | `--bidir 64 --tx-window 40/48` | identical: the ACK window is not binding |
| 4 | stock pass-through vFPGA, both sides writing | 09_perf_rdma `PERF_RDMA_BIDIR=1` (both sides timed) | 4.1 + 6.3 GB/s: the shell shares the pipe too |
| 5 | host DMA read + write at once on one host, no network | `--local 64` (4 MiB local copy) | 9.8 GB/s each way concurrently: the host DMA is full duplex |
| 6 | A pushes 4 MiB while B fires K x 64 B stores at A | `--storm K` | A's push = 394 + 0.42 us x K (K = 512/1024/2048 -> 582/808/1258 us): **cost per packet, bytes irrelevant** |
| 7 | the RoCE stack's HLS synthesis reports (rocev2_prj/solution1/syn/report in any build) | | every rx/tx stage II=1, latency <= 3 cycles: the cost is not inside the stack's FSMs |
| 8 | the client's own path counters during a storm | `tx [client after the matrix]`, `rx [...]` | engine waited 3.0 M cycles on `sq_wr.ready`; loom_rx refused shell beats 3.4 M cycles, only 0.43 M of them because the host stream was not ready |

Line 6 says what the cost is per; line 8 says where it is paid: the vFPGA
side waits for the SHELL TO ACCEPT ITS REQUESTS (`sq_wr.ready`). Every
request from the vFPGA - a host write for a landed packet (loom_rx), an
RDMA request for a packet to send (loom_engine), a 64 B store - is taken
at roughly one per ~90-100 cycles. Check: one direction = the receiver
issues one host-write request per 4 KB packet and the sender one RDMA
request per 4 KB packet -> 4096 B / ~95 cycles at 250 MHz ~= 11 GB/s. Two
directions = each host issues both kinds per 4 KB per direction -> half.
The ACK matters only indirectly: the receiving stack emits the ACK in the
same FSM step in which loom_rx's request is accepted, so slow acceptance
delays ACKs, which throttles the other direction's sender.

## What has been ruled out (do not re-derive)

- The Loom arbiter (fixed in ad88e9f0: whole-transaction ownership of
  `sq_wr` deadlocked bidirectional traffic; now per request). Deadlock
  is gone: 0 packets stuck in loom_rx in every run since.
- The ACK window (line 3), retransmissions (0 in every run), the wire
  (100 GbE, full duplex), PCIe/host DMA (line 5), hugepages/TLB (getMem
  maps eagerly through IOCTL_MAP_USER_MEM), ARP (both sides resolve at
  initRDMA), the RoCE stack's FSM latencies (line 7).
- A first-exchange cost of ~30 ms per process on the receiving host's
  first host write through loom_rx (dest 1). Cause unknown, workaround
  in: `--bidir` runs one sequential exchange first (`--bidir-no-warmup`
  reproduces it). Any application must warm up once. Separate item.

## Where to look next

1. **The shell's request-acceptance path behind `sq_wr`.** Start from the
   vFPGA wrapper's `sq_wr` port and follow it into the shell: the request
   demux by opcode (`hw/hdl/user/dreq/` - `dreq_rdma_parser_wr.sv` splits
   RDMA requests at PMTU; the local-write path goes to the host DMA
   request logic and the TLB). Find what limits it to one request per
   ~100 cycles: an un-pipelined TLB lookup per request, a credit that is
   returned only on completion (`N_OUTSTANDING = 8` in the generated
   lynx_pkg), or a serialized parser FSM. The HLS reports (line 7) cover
   only the stack; this part is SystemVerilog and has no such report -
   read it, then confirm with a counter (the ctrl page has RO words; add
   one for "cycles sq_wr.valid && !sq_wr.ready by opcode").
2. **The Loom-side arbiter's priority.** `vfpga_top.svh` gives rx
   priority on `sq_wr`. Under a flood of incoming packets the engine's
   requests wait behind every landing (line 8: 3.0 M cycles). Once the
   shell side is understood, this may need round-robin - a one-line
   change, testbench T8 in `hw/tb/tb_loom_top.sv` covers the deadlock
   property, `./hw/ooc_synth.sh top` checks timing.
3. **If the shell path cannot be made faster**, the mitigation is fewer
   packets per byte: build with `-DPMTU_BYTES=8192`. Checked: the
   parameter reaches base.tcl, lynx_pkg (request parser, loom_engine,
   loom_rx, ingress FIFO all scale with it), rocev2_config.hpp
   (PMTU_WORDS 128 -> 256, the 8-bit beat counter in tx_pkg_arbiter
   still fits; 16 KB would not); the CMAC IP sets no max frame (default
   9600 B); the switch (FS N8550-32C, the FPGAs and the hosts' E810s are
   on it) passes 9000 B frames between the E810s. Keep `--tx-window` at
   16 with 8 KB packets (the receiver buffers ~24 of them). Halves the
   per-byte cost: expect ~9-10 GB/s each way bidirectionally.
4. Only after the above: the two RoCE-stack changes considered and set
   aside - pipelining rx_ibh_fsm/rx_exh_fsm (they are already II=1, so
   this buys nothing unless line 7 is wrong), and ACK coalescing (would
   need loom_engine's window to count PSNs instead of completions).

## Reproducing every line above

README "Running" -> "Reproducing the bidirectional-bandwidth experiments"
has the exact command and the expected output for each measurement in
the table, plus the perf_rdma control and how to tell which bitstream a
card holds. The rest of "Running" covers the environment, the
testbenches, out-of-context synthesis, building a bitstream, programming
and the driver, and building the software on both hosts.

## Tools you have

- `./run_two_host.py` drives both hosts from clara, flashes before every
  point, logs everything to `experiments-log.txt`. Modes: push (default),
  `--pingpong`, `--matrix`, `--bidir N`, `--local N`, `--storm K`,
  `--tx-window W`, `--no-flash`. Verdict line per run; the exit code is
  meaningless for the matrix modes (known cosmetic crash).
- `loom_host` prints the engine's transmit words (`tx [...]`), loom_rx's
  cycle accounting (`rx [...]`) and the DBG counters after the matrix.
- `09_perf_rdma` with `PERF_RDMA_BIDIR=1` on the server: the stock stack
  under bidirectional load, both sides timed. Its bitstream is at
  `~/coyote-bitstreams/perf_rdma/`; flash with
  `BIT=$HOME/coyote-bitstreams/perf_rdma/hw/bitstreams/cyt_top.bit python3 -c "import run_two_host as r; r.flash(15)"`.
- `hw/tb/run_tbs.sh` (6 testbenches), `hw/ooc_synth.sh top` (whole vFPGA
  out of context, 4 min, utilization + WNS).

## Bench traps (each cost hours once)

- After ANY host reboot the cards must be reflashed before a run (the
  runner does it unless `--no-flash`). If a card is missing from `lspci`
  after a reboot: program it over JTAG by hand (`xilinx-shell -c "vivado
  -mode batch -source hw/program_loom.tcl -tclargs <bit>"`), THEN warm
  reboot that host.
- udev can auto-load the Coyote driver with its DEFAULT identity (ip
  0x0b01d4d1) and win over setup_coyote.sh: the sysfs node looks fine,
  nothing is delivered. The runner's preflight now checks
  `cyt_attr_ip`; fix = `sudo rmmod coyote_driver && sudo bash setup_coyote.sh`.
- Never put a host NIC on 10.0.0.0/8: the stack's ARP table is indexed by
  the first octet, one entry per /8, and the host's ARP overwrites the
  peer FPGA's entry. The FPGAs answer only tiny pings (their ICMP echo
  fails at 56 B payloads), so ping cannot test jumbo on the FPGA path.
- The runner rsyncs clara's `loom_host` to amy on every run; a special
  build on clara silently replaces amy's binary. Check md5 on both.
- Aperture stores reach only the window's first 4 KB page (the flags at
  0xF00); anything further needs a descriptor.
- `/tmp` is cleared by a reboot; put scratch scripts elsewhere.
