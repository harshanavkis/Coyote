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

- **`loom_switch` (U280, done: `hw/src/vfpga_top.svh`, `hw/CMakeLists.txt`):**
  `vfpga_top` = `loom_ctrl` (the CSR page only: table programming including
  each window's `ustart` at word 80, staging VA, ack window, `RX_CHUNK`,
  counters; ingress counters at words 88-94, its debug counters at 95-108),
  `loom_table`, `loom_ingress`
  and `loom_rx` (copied unchanged from `examples/loom`). `sq_wr` is shared
  per request by the ingress and `loom_rx`, nothing else; the arbiter and
  its two known defects are carried over as they are. `RX_CHUNK` is wired
  to `loom_rx` here (in `examples/loom` it is not, see below).
  - **`loom_table`** (the switch's copy): each entry gains `ustart`, its
    range in the uwin being `[ustart, ustart + len)`; a second lookup port
    matches an address against the ranges (lowest index wins), in two
    register stages.
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
    - **Contract:** INCR bursts of full-width beats. The address is rounded
      down to 64 B and the strobes say which bytes are written, so a Zen 3
      host's 32 B half-line flushes and 8 B stores at their own address are
      stores. A burst with no window or whose lines end past its window is
      discarded and counted. B once the last beat is in the FIFO. Reads
      return zeros.
    - **Lookup:** three register stages (the table's two, then offset and
      bounds) that advance whenever the next has room, so bursts follow
      each other with no gap, at one a cycle for 1-beat bursts.
    - **Debug counters (words 95-108):** output backpressure, the data
      FIFO empty while sending, cycles a W beat waited by reason, drops by
      reason, bursts by size, misaligned bursts. `uwin_probe` prints them
      all, `p2p_bw` those of each landed run.
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
  flag behind bulk), writes starting inside a line (32 B halves, 8 B stores
  at their own address, a burst from line+32), 1-beat bursts at one a cycle,
  the debug counters (each W wait under exactly one reason, every cycle),
  reads; then all of it again under random
  backpressure on `sq_wr`, the window, both send streams, B, and W bubbles.
  Back-to-back bursts are taken at one beat per cycle. Run:
  `examples/loom_switch/hw/tb/run_tbs.sh` (uses `examples/loom`'s
  `build_sim/sim/lynx_pkg.sv` until this app has its own).
- `tb_loom_switch_top` (passes): the generated wrapper with the TB as the
  shell. Table programming and readback; bulk and stores through the uwin
  on both routes, exact; the ack window holding rdma packets until acks
  return; `loom_rx` landings, and `RX_CHUNK` 1 vs 2 changing a two-packet
  message from two host writes to one; the ingress and `loom_rx` racing for
  `sq_wr` under backpressure with the arbitration invariants checked every
  cycle; the counters. Under `sq_wr` backpressure the shell interface's own
  stability assertion catches the arbiter's known defect 2 (a presented
  request replaced by `loom_rx`'s); it is switched off for that test only,
  and the test checks that no request is lost.
- `tb_loom_ce`: descriptor → `sq_rd`/`sq_wr` sequence, stream forwarding,
  fence.
- Same style as `examples/loom/hw/tb/`. `hw/tb/run_tbs.sh` runs both block
  TBs against this app's own generated package and wrapper (`hw/build_sim`,
  generated once as the script says).

## Running

- Block TBs (any host with Vivado):
  `/scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/hw/tb/run_tbs.sh`
- The shell window's block design:
  `/scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/hw/tb/shell_ctrl_uwin/run.sh`
- Software:
  `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/sw && mkdir -p build && cd build && nix-shell ../../../../shell.nix --run "cmake .. && make -j16"`
- G2 in Coyote's simulation (the whole vFPGA under `sim/hw/tb_user.sv`,
  which drives `axi_udata` for `cThread::uwinWrite`; passes, 16 KiB bulk and
  16 stores). Once: build the software with `-DEN_SIM=ON` in `sw/build_sim`,
  and create the sim project in `hw/build_sim` with `xilinx-shell -c "make sim"`.
  Vivado 2023.2's bundled binutils cannot link the DPI library against this
  host's glibc any more, so that step fails; copy
  `examples/loom/hw/build_sim/sim/coyote_sim.so` (same source) into
  `hw/build_sim/sim/` and run
  `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/hw/build_sim && xilinx-shell -c "vivado -mode batch -nojournal -source cr_sim.tcl -notrace"`.
  Then:
  `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/sw/build_sim && xilinx-shell -c "COYOTE_SIM_DIR=/scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/hw/build_sim stdbuf -oL ./uwin_probe 16384 16"`
  (compiles, elaborates and runs; minutes of wall clock)
- **On hardware (clara, and rose for G4).** Images:
  `examples/loom_switch/hw/build_sep29_uwin/bitstreams/cyt_top.bit` (U280) and
  `examples/loom_ce/hw/build_sep29_ce/bitstreams/cyt_top.pdi` (V80), both from
  `492e2a25`. The drivers come from this tree (`driver/build`,
  `driver/build_versal`, built with the uwin mmap and export).
  1. Sync the U280 driver to rose:
     `rsync -a /scratch/harshanavkis/loom-proj/Coyote/driver/build/ rose.dos.cit.tum.de:/scratch/harshanavkis/loom-proj/Coyote/driver/build/`
  2. Flash both U280s and load their drivers with each host's IP and MAC
     (from `examples/loom`, on clara). Each host programs from its own view
     of `BIT`, and `/scratch` is per host, so the image goes to the shared
     home first:
     `mkdir -p ~/coyote-bitstreams/loom-switch-sep29/hw/bitstreams && cp /scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/hw/build_sep29_uwin/bitstreams/cyt_top.bit /scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/hw/build_sep29_uwin/bitstreams/cyt_top.ltx ~/coyote-bitstreams/loom-switch-sep29/hw/bitstreams/`
     `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom && BIT=/home/harshanavkis/coyote-bitstreams/loom-switch-sep29/hw/bitstreams/cyt_top.bit SERVER_HOST=rose.dos.cit.tum.de SERVER_IP=131.159.102.21 SERVER_BDF=c1:00.0 SERVER_FPGA_IP=0a000003 python3 -c "import run_two_host as r; r.flash(15)"`
  3. Program clara's V80 (never rose's, it is another user's):
     `cd /scratch/harshanavkis/loom-proj/Coyote && scripts/fpga/program_v80.sh examples/loom_ce/hw/build_sep29_ce/bitstreams/cyt_top.pdi 0000:81:00.0`
  4. G2 on clara (1 MiB bulk, 256 stores):
     `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/sw/build && sudo ./uwin_probe 1048576 256`
  5. G4, local half, on clara (1 MiB, 3 copies):
     `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && sudo ./ce_local 1048576 3`
  6. G4: sync the binaries and library to rose, start the server there, then
     the client on clara:
     - clara: `rsync -a /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build/ rose.dos.cit.tum.de:/scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build/`
     - rose: `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && sudo ./ce_remote --server --size 1048576`
     - clara: `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && sudo ./ce_remote --client 131.159.102.21 --reps 3`
  7. The V80's write bandwidth, to host memory and peer-to-peer into the
     uwin, on clara. It needs example 07 on the V80 instead of `loom_ce`
     (step 3 puts `loom_ce` back):
     `cd /scratch/harshanavkis/loom-proj/Coyote && scripts/fpga/program_v80.sh examples/07_perf_fpga/hw/build_v80/bitstreams/cyt_top.pdi 0000:81:00.0`
     `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && sudo ./p2p_bw`

## Found on the way: `RX_CHUNK` is not wired in `examples/loom`

`examples/loom/hw/src/vfpga_top.svh` connects neither `loom_ctrl`'s
`rx_chunk` output nor `loom_rx`'s `rx_chunk` input (since `40c7620c`), and
synthesis does not warn. The CSR (word 76) therefore has no effect on the
card. `loom-portcnt` and `build_sep29_ctrl` both have it.

What synthesis made of it (read from `loom-portcnt`'s synthesized user
checkpoint, `build_sep27_port_counters/.../synth_1/design_user_wrapper_0.dcp`):
`rx_chunk` = 0. The comparators decode to `q_left > 0` and
`hdr_len > 0xFFFFFC0`, so `chunk_bytes` = 0 and `loom_rx` lands every message
with ONE host write of its whole length (a 64 MiB push is one 64 MiB write),
not one per packet. The PERFORMANCE.md numbers were taken in that mode; the
control build compares like with like.

`loom_switch` differs: the ingress sends only single-packet messages, so the
far `loom_rx` lands one host write per packet whatever `RX_CHUNK` says. A
receive-side difference in G5 against `loom-portcnt` has that as a
candidate cause. `examples/loom` is left as it is.

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
