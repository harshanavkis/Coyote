# PTDMA: the host CPU's DMA engines as a copy engine for Loom

AMD EPYC CPUs have DMA engines in their I/O die, *PTDMA* (Linux driver `ptdma`,
`drivers/dma/amd/ptdma`). Each one appears as a PCI function on the CPU's internal
buses (`0000:xx:00.2`); amy and rose (2x EPYC 7413) have 16, 8 per socket. The V80 and
the U280 sit on socket 1 (NUMA node 1), so the tests use that socket's engines.

We use PTDMA for two things:

1. **A host-side copy engine for Loom.** With host DRAM as the data source for both
   systems, RDMA's NIC pulls host DRAM and PTDMA pushes host DRAM into Loom's user data
   window on the U280 - the role the V80's copy engine plays in the V80 experiments
   (and a GPU's copy engine in the paper's setting). No staging copy, no dma-buf.
2. **A diagnostic reader of V80 memory.** PTDMA reading the V80's window shows how fast
   the V80 serves reads to a host-side reader, without the U280 or the network.

## What is here

| File | What it is |
|---|---|
| `ptdma_bw.c`, `Makefile` | kernel module: runs one copy test in `module_init` and then fails with `-EAGAIN` on purpose, so it never stays loaded (except after a DMA timeout, see below). Results go to the kernel log (`ptdma_bw: ...`). |
| `../sw/src/uwin_ptdma.cpp` | Loom tool: the receiver exports a host buffer and checks it; the sender binds its U280 window to it (as `uwin_pingpong`) and tells you how to load `ptdma_bw` to push into that window. |
| `~/loom-experiments/e1/run_ptdma_20261011.sh` | local tests on rose: host->host and V80 window->host, 1/4/8 engines. |
| `~/loom-experiments/e1/run_uwin_ptdma_20261011.sh` | PTDMA over Loom amy -> rose (1/16/64 MiB) and the same copy with CPU stores. |
| `~/loom-experiments/e1/run_uwin_ptdma_chunks_20261011.sh` | PTDMA over Loom, 64 MiB, descriptor (chunk) sizes 4 MiB / 1 MiB / 64 KiB / 7 KiB. |

## How `ptdma_bw` works

- dmaengine client: takes `nchan` PTDMA channels (one per PTDMA device, on NUMA node
  `numa`, or the devices listed in `devs`) with `dma_request_channel`.
- Per iteration, `size` bytes are cut into descriptors of at most `chunk` bytes, dealt
  round robin over the channels, submitted (`DMA_PREP_INTERRUPT` on every descriptor:
  the ptdma driver keeps ONE descriptor in the hardware and starts the next from the
  completion interrupt), and waited for. Time = first prep to last completion.
- Source and destination are a host DRAM buffer (node-local, 4 MiB pieces, source filled
  with `0x5054444d00000000 ^ byte offset`, destination poisoned and verified after each
  iteration when both are host memory) or an MMIO physical address (`src_phys`,
  `dst_phys`, e.g. a BAR), mapped for each channel's device with `dma_map_resource`
  (the IOMMU translates PTDMA's DMA too).
- Flag mode (`flag_off`, needs `dst_phys` and `nchan=1`): after each iteration's data,
  one more 8 B descriptor on the same channel writes `flag_val + iteration + 1` to
  `dst_phys + flag_off`. One requester's posted writes stay in order, so the flag lands
  after the data - the copy engine's SIGNAL. Time runs to the flag's completion.
- Safety: `dst_phys != 0` is refused unless `allow_write=1` (a write below Loom's window
  hits the shell's registers and can hang the card). If a DMA does not finish within
  `timeout_ms`, the module STAYS LOADED holding every buffer, mapping and channel (the
  driver cannot stop a running descriptor); `rmmod ptdma_bw` later.

Parameters: `size` (default 64 MiB), `chunk` (1 MiB, max 4 MiB), `iters` (3), `numa` (1;
-1 = any), `nchan` (1, max 16), `devs`, `src_phys`, `dst_phys`, `allow_write` (0),
`timeout_ms` (10000), `flag_off` (0 = off), `flag_val` (0), `tag` (a word printed in the
run's first log line, so a script can find its lines in a full kernel log).

## Build

The hosts run NixOS kernel 7.2.2; clara has the matching kernel tree in its nix store.
Build inside Coyote's nix shell (a gcc version warning is normal), then put the module
where amy and rose can load it (`/home` is shared over NFS, `/scratch` is not):

```
cd Coyote/examples/loom_switch/ptdma
nix-shell ../../../shell.nix --run "make KDIR=/nix/store/rvrswvmlrss37pnzavczhijwza4cb3q8-linux-7.2.2-dev/lib/modules/7.2.2/build"
modinfo -F vermagic ptdma_bw.ko                      # 7.2.2 SMP preempt mod_unload
mkdir -p ~/loom-experiments/ptdma_bw && cp ptdma_bw.ko ~/loom-experiments/ptdma_bw/
```

`uwin_ptdma` builds with the other `sw` targets (`examples/loom_switch/sw/build`:
`cmake . && make uwin_ptdma`); the run scripts copy it and `libcoyote.so` to amy and rose.

## Run

Local (on rose; the module prints to the kernel log, read it with `dmesg | grep ptdma_bw`
- do not count log lines, the ring is full of Coyote messages):

```
sudo insmod ~/loom-experiments/ptdma_bw/ptdma_bw.ko numa=1 nchan=1                 # host -> host
sudo insmod ~/loom-experiments/ptdma_bw/ptdma_bw.ko numa=1 nchan=4 src_phys=0x58128000000   # rose's V80 window -> host
```

(rose's V80 window = its BAR4 base + 0x0800_0000; BAR4 = line 5 of
`/sys/bus/pci/devices/0000:c1:00.0/resource`.) `insmod` reporting "Resource temporarily
unavailable" is the intended `-EAGAIN`.

Over Loom (amy -> rose, U280s on the Loom image): run the scripts above in tmux on clara;
they set the cards up, start `uwin_ptdma --server` on rose and `--client` on amy, check
the printed `insmod` line (destination = amy's U280 BAR4 + 0x0800_0000, the uwin, checked
against sysfs and the kernel's write-combining list), load the module, and restore the
standard images at the end.

## Results (measured 2026-10-11)

| Test | 1 engine | 4 engines | 8 engines |
|---|---|---|---|
| host DRAM -> host DRAM (rose) | 17.4-20.2 GB/s | 22.9-23.9 | 22.1-24.3 |
| rose's V80 window -> host DRAM | 3.60-3.64 GB/s | 3.72-3.73 | 3.71-3.72 |

PTDMA over Loom, amy host DRAM -> U280 window -> rose host DRAM, 1 engine, flag mode,
every word checked on rose (all pass, flag never ahead of the data):

| Chunk (64 MiB per iteration) | 4 MiB | 1 MiB | 64 KiB | 7 KiB |
|---|---|---|---|---|
| GB/s | 10.50-10.51 | 10.04-10.10 | 5.72-5.97 | 1.03-1.26 |

CPU stores through the same window (`uwinWrite`, write-combining): 0.17 GB/s.

## Known limits

- **~5.4 us per descriptor.** The Linux ptdma driver keeps one descriptor in the engine
  and starts the next from the completion interrupt, so small descriptors leave the
  engine idle: model = ~10.6 GB/s streaming + ~5.4 us per descriptor fits every chunk
  size above. E4-sized pushes (7 KiB) reach only ~1.1 GB/s. Fix: queue many descriptors
  per doorbell (a minimal own driver for the PTDMA queue) - not done.
- **~10.5 GB/s over Loom, with full packets.** The ingress counters show 4 KiB packets
  for bulk PTDMA (64 MiB x 10: 164,827 packets = 4,071 B each; 64 MiB x 5: 81,960 =
  4,094 B; idle flushes 80-1,900) and almost no backpressure (`W wait: fifo` 35-78k
  cycles). The V80 engine, also 4 KiB packets, tops out at the same 10.4-10.7 GB/s (RDMA
  from host memory: 12.2). The limit was `loom_ingress` itself: every write arrives as a
  256 B burst and waited ~2 cycles for its address lookup (`W wait: no aw` 2.0 per burst),
  6 cycles per 256 B at 250 MHz = 10.67 GB/s. Fixed by a W buffer ahead of the lookup
  (`loom_ingress.sv`, DESIGN.md section 12; 4 cycles per burst in simulation, not yet
  measured on hardware). The idle timer only splits real gaps: 7 KiB chunks end in a
  3 KiB tail each (309,575 flushes), followed by the next descriptor ~5.4 us later.
- **A lone 1 MiB transfer takes ~347 us (3 GB/s)**, ~240 us more than the model; seen only
  when the whole transfer is 1 MiB. Unexplained.
- The V80 serves every reader (PTDMA as much as the U280) at ~3.7 GB/s: 8 reads of 256 B
  in flight inside the V80 static (see the window counters, `hw/hdl/common/uwin`).
  uwin_ra's read-ahead (counter page line 59 on) lifts the U280's RDMA reads of V80
  memory to 11.4-11.6 GB/s.
