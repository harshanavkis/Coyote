# Loom split: V80 copy engine + U280 switch

Branch `loom-v80`. Two new apps next to the existing `examples/loom/`, which
stays as it is:

- `examples/loom_ce/` (V80): the emulated accelerator's copy engine. It reads
  its source from V80 HBM and writes it to a destination virtual address. The
  V80's Coyote MMU maps that address, via a dma-buf, onto the U280's ingress
  window, so the writes travel peer-to-peer over PCIe.
- `examples/loom_switch/` (U280): Loom's switch. It turns writes arriving on
  the ingress window (uwin) into local or RoCE writes by binding, and
  `loom_rx` lands what arrives from the far side.

**One data path.** Everything that writes into a window - the copy engine's
bulk writes and the host CPU's stores - is a write to a uwin address, and
the address names the binding. There is no aperture, no order FIFO, no
descriptor and no `loom_engine`: `examples/loom` needed those only because
its aperture was AXI-Lite (at most 8 B per uncached beat), so bulk had to be
a descriptor and a pull. The CPU maps the uwin write-combining; a full line
joins packets like bulk, a lone 8 B word becomes one store (below). A flag
written after data stays behind it (one queue); on the CPU side the data and
the flag are separated by a store fence, as with GPU peer writes. No loads
for now: uwin reads return zeros.

Hosts: clara ↔ rose (a V80 and a U280 each). Builds go through
`scripts/fpga/build_bitstream.sh` (U280: Vivado 2023.2; V80: 2025.1).

## Data path

```
V80 loom_ce:  HBM --read--> CE --sq_wr LOCAL_WRITE to dst VA-->  V80 MMU/TLB
                                   (dst VA mapped via dma-buf to U280 BAR)
PCIe P2P:     MWr TLPs  ------------------------------------->  U280 bypass BAR
host CPU:     WC stores into the mmap'd uwin ------------------>  U280 bypass BAR
U280 shell:   bypass BAR 0x0800_0000.. -> new AXI4 512-bit port -> vFPGA
U280 loom_switch: uwin range -> binding (loom_table) -> packets: local sq_wr,
                  or single-packet Loom messages (ACK window) -> RoCE
far U280:     loom_rx lands the data (unchanged)
```

## 1. Shell: a user data window (U280)

The XDMA's AXI bypass BAR (256 MB, 64-bit prefetchable, 512-bit AXI) already
enters the `shell_ctrl` block design (`hw/bd/ultrascale_plus/cr_ctrl.tcl`),
which splits it:

| Offset | Size | Port |
|---|---|---|
| `0x0000_0000` | 32 KB | shell config (AXI-Lite) |
| `0x0010_0000 + i·256 KB` | 256 KB | `axi_ctrl_i`, the vFPGA's AXI-Lite control (`loom_ctrl`'s CSR page) |
| `0x0100_0000 + i·256 KB` | 256 KB | `axim_ctrl_i`, 256-bit AVX to the shell registers |
| **`0x0800_0000`** | **128 MB** | **new: `axim_udata_0`, 512-bit AXI4 into vFPGA 0** |

The uwin takes the rest of the bypass BAR (the 256 MB BAR itself is fixed
by the XDMA configuration in the static region). With `N_REGIONS 1` the
uwin is the largest aligned hole, 128 MB; with more regions it is split
equally (`128 MB / N_REGIONS` each). Windows are not fixed slices of it:
each table entry holds its own range in the uwin (see RTL), so one binding
can be up to 128 MB (a 64 MiB push fits one binding).

- **Config flag (done):** `EN_UWIN` (default 0; UltraScale+ only, not with
  `EN_UCLK`). With it off, every generated file is identical to before.
- **Path (done):** `cr_ctrl.tcl` adds `axim_udata_<i>` (512-bit AXI4, 6-bit
  IDs) as interconnect masters after the control ports, and their address
  segments. `shell_ctrl` → `shell_top` → `dynamic_top` (two register slices
  and the decoupler, all on aclk) → `user_wrapper` → `user_logic`, where
  `vfpga_top` sees it as `axi_udata`.
- **IDs:** the uwin port carries the host's AXI IDs (the AVX port has none:
  its width converter drops them). The vFPGA must answer B and R with the ID
  it was given, or the interconnect's response routing wedges; `loom_ingress`
  echoes them.
- **Writes:** reach the vFPGA as AXI4 bursts. The payload is posted, so the
  B channel must be answered promptly.
- **Test (passes):** `hw/tb/shell_ctrl_uwin/run.sh` (`run.sh 2` for two
  regions) builds `design_ctrl` from `cr_ctrl.tcl` in Vivado 2023.2 and
  simulates it: uwin bursts arrive whole with their IDs, up to the uwin's
  last byte; shell config, `axi_ctrl` and `axim_ctrl` still decode where
  they did; holes and past-the-uwin addresses get DECERR; uwin reads come
  back; back-to-back 256 B writes cross at one beat per cycle.

## 2. Driver (U280) and user library (done, untested on hardware)

- **Addresses:** the driver's region offsets are relative to
  `bar_phys_addr[BAR_SHELL_CONFIG]`, the bypass BAR, and equal the block
  design's AXI addresses, so vFPGA *i*'s uwin is at
  `+ VFPGA_UWIN_OFFS (0x0800_0000) + i * uwin_size`, `uwin_size` being the
  same power-of-two share `cr_ctrl.tcl` gives it (`coyote_setup.c`).
- **mmap:** `MMAP_UWIN` (page offset 4) maps up to `uwin_size` bytes
  write-combining (`vfpga_ops.c`); it is refused if the BAR is too small to
  hold the uwin. Nothing tells the driver whether the shell was built with
  `EN_UWIN`: on one that was not, writes are dropped with DECERR, and reads
  must not be issued.
- **dma-buf:** `EXPORT_REGION_UWIN` (1) in `vfpga_export.c`: the G1 exporter,
  whose importer maps the BAR range with `dma_map_resource`.
- **User library:** `cThread::mapUwin(len)` / `unmapUwin()` (unmapped with
  the thread); `exportDmabuf(EXPORT_REGION_UWIN, off, len)` for the V80, which
  imports it with `importDmabuf(fd, vaddr)` as in G1.

## 3. RTL

- **`loom_switch` (U280):** `vfpga_top` = `loom_ctrl` (the CSR page only:
  table programming including each window's `ustart`, staging VA, ack
  window, counters), `loom_table`, `loom_ingress` and `loom_rx`. `sq_wr` is
  shared per request by the ingress and `loom_rx`, nothing else.
  - **`loom_table`** (the switch's copy): each entry gains `ustart`, its
    range in the uwin being `[ustart, ustart + len)`; a second lookup port
    matches an address against the ranges (lowest index wins).
  - **`loom_ingress`** (done, `hw/src/hdl/loom_ingress.sv`): AXI4 write
    slave, 512-bit.
    - **Packets:** consecutive beats continuing one binding at the next
      offset are gathered into a packet of up to PMTU on the wire. A packet
      closes when full, on a beat that does not continue it, or after
      `FLUSH_CYCLES` (16) with no beat presented. It is stored before it is
      sent, since the request and the header both state its length.
    - **Local route:** `LOCAL_WRITE` of up to 4 KB, on host stream 0.
    - **RDMA route:** a single-packet Loom message: `RC_RDMA_WRITE_ONLY`,
      RAW, at the staging vaddr, header beat `{op WRITE, len, dst_pid,
      base + off}` + up to 63 payload beats, on RDMA stream 1. That is what
      `loom_engine` sends for a one-packet message, so the far `loom_rx` is
      unchanged (it takes the destination from the header, never the RETH).
      Posted only while the ack window allows (`win_ok`, `rdma_post`).
    - **Stores:** a beat whose strobe is not full but covers whole,
      aligned 8 B words is one store per word: an 8 B `LOCAL_WRITE`, or the
      64 B `WRITE_INLINE` message loom_rx already lands. Anything smaller is
      dropped and counted.
    - **Order:** one data FIFO and one packet queue, so packets leave in
      arrival order across bindings.
    - **Contract:** 64 B-aligned INCR bursts. A burst with no window, a
      misaligned address or an end past its window is discarded and
      counted. B once the last beat is in the FIFO. Reads return zeros.
  - **Streams:** the ingress is the only sender: host stream 0 and RDMA
    stream 0; `loom_rx` lands on host stream 1 (`N_STRM_AXI 2`,
    `N_RDMA_AXI 1`). The ack window counts only the ingress's posts.
- **`loom_ce` (V80):** example 07's structure with source and destination
  CSRs, `sq_rd` from card memory (`STRM_CARD`, `EN_MEM=1`), the read stream
  forwarded to `sq_wr` on the host stream (dst VA), and a completion fence
  write.

## 4. Testbenches

- `tb_loom_ingress` (passes): AXI4 bursts → the exact packets (requests,
  rdma headers, payload beats, tlast), against a reference packetiser:
  aligned packets on both routes, a run starting mid-packet, back-to-back
  bursts over several packets, alternating windows, offset and idle gaps,
  drops, a 64 MiB window, 8 B stores (lone words on both routes, several in
  one beat, an empty beat, partial words dropped, a store inside a run, a
  flag behind bulk), reads; then all of it again under random
  backpressure on `sq_wr`, the window, both send streams, B, and W bubbles.
  Back-to-back bursts are taken at one beat per cycle. Run:
  `examples/loom_switch/hw/tb/run_tbs.sh` (uses `examples/loom`'s
  `build_sim/sim/lynx_pkg.sv` until this app has its own).
- `tb_loom_switch_top`: ingress + `loom_rx` sharing `sq_wr`.
- `tb_loom_ce`: descriptor → `sq_rd`/`sq_wr` sequence, stream forwarding,
  fence.
- Same style and runner as `examples/loom/hw/tb/` (`run_tbs.sh`).

## 5. Gates

| Gate | Check |
|---|---|
| G0 | clara drives the U280 and the V80 at once (two driver modules); example 07 runs on the V80 |
| G1 | P2P spike: V80 example 07 writes its counter pattern into an exported U280 region; the host reads it back |
| G2 | host CPU writes into the U280 window (WC mmap) appear on `loom_ingress` counters: lines and 8 B stores |
| G3 | TBs green, `ooc_synth` timing met on both apps |
| G4 | clara V80 → clara U280 → rose U280 → rose host, byte-exact, 0 retransmits; then the local route |
| G5 | performance rerun of PERFORMANCE.md (push, DMA ping-pong, store latency through the uwin) |

## Builds in flight (2026-09-29)

- `examples/loom/hw/build_sep29_ctrl`: control rebuild of today's Loom on the
  merged stack (U280, 2023.2).
- `examples/07_perf_fpga/hw/build_v80`: example 07 for the V80 (2025.1), for
  G0/G1.
