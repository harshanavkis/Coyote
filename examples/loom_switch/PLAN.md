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
  - **Receive side (`EN_UWIN_HBM`):** the V80's BAR4 reaches the shell as
    `axi_main` like the U280's bypass BAR, so it has the same uwin; with
    `EN_UWIN_HBM` the uwin does not go to the vFPGA but through its own HBM
    channel (`uwin_hbm` + `axi_stripe`, striped like the card channels)
    into the last HBM block: uwin offset x is card physical address
    `UWIN_HBM_BASE + x` (`0x47_E000_0000`). The driver keeps that block out
    of its allocator, and `IOCTL_UWIN_HBM_BIND` (`cThread::uwinHbmBind`)
    points a buffer's card pages at it. So the far U280's `loom_rx` (or a
    local window) writes peer-to-peer straight into HBM, like into a GPU's
    BAR, and the copy engine reads the same bytes by VA on its card stream.
    Reads through the window return HBM too. The V80's NoC keeps same-ID
    writes in order (first-generation NoC: strict write ordering), so a
    fence written after the data lands after it. (The earlier vFPGA
    landing path - a `loom_ingress` copy issuing card writes and sharing
    `sq_wr` with the copy engine - reset the hosts and is gone.)

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
  fence, the stall counters. `tb_uwin_hbm`: the window path (`uwin_hbm` +
  `axi_stripe`) and a card channel (`axi_stripe`) on one memory model - a
  peer's writes through the window (256 B bursts, lines, 32 B halves, an
  8 B store) are what the card path reads at `UWIN_HBM_BASE` + offset, and
  the reverse; exact `rlast` on window reads. Run:
  `examples/loom_ce/hw/tb/run_tbs.sh`.
- Same style as `examples/loom/hw/tb/`. `hw/tb/run_tbs.sh` runs both block
  TBs against this app's own generated package and wrapper (`hw/build_sim`,
  generated once as the script says).

## Running

- Block TBs (any host with Vivado):
  `/scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/hw/tb/run_tbs.sh`
- The shell window's block design:
  `/scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/hw/tb/shell_ctrl_uwin/run.sh`
  (includes reads in flight into the window: 32)
- The U280 static block design's XDMA (32 reads in flight on the bypass
  master): `/scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/hw/tb/static_pci/run.sh`
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
- **On hardware (clara, and rose for G4).** Every card sits on NUMA node 1
  on both hosts; unpinned runs get buffers on node 0 at random and lose
  up to 3x, so every run below is pinned with numactl (from nix; the same
  store path on clara and rose). Images:
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
     `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/sw/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./uwin_probe 1048576 256`
  5. G4, local half, on clara (1 MiB, 3 copies):
     `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./ce_local 1048576 3`
  6. G4: sync the binaries and library to rose, start the server there, then
     the client on clara:
     - clara: `rsync -a /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build/ rose.dos.cit.tum.de:/scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build/`
     - rose: `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./ce_remote --server --size 1048576`
     - clara: `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./ce_remote --client 131.159.102.21 --reps 3`
  7. The V80's write bandwidth, to host memory and peer-to-peer into the
     uwin, on clara. It needs example 07 on the V80 instead of `loom_ce`
     (step 3 puts `loom_ce` back):
     `cd /scratch/harshanavkis/loom-proj/Coyote && scripts/fpga/program_v80.sh examples/07_perf_fpga/hw/build_v80/bitstreams/cyt_top.pdi 0000:81:00.0`
     `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./p2p_bw`
- **The receive side into V80 HBM (G6), clara and rose.** Images:
  `examples/loom_switch/hw/build_sep30_uwin2/bitstreams/cyt_top.bit` (U280)
  and `examples/loom_ce/hw/build_sep30_uwinhbm/bitstreams/cyt_top.pdi` (V80,
  `EN_UWIN_HBM`). Both V80s run `loom_ce` and need the V80 driver from this
  tree (the uwin HBM bind).
  1. Flash both U280s as in step 2 above, from
     `~/coyote-bitstreams/loom-switch-sep30` (copy `build_sep30_uwin2`'s
     `cyt_top.bit` and `cyt_top.ltx` there first).
  2. The V80 image to the shared home, the V80 driver and the software to
     rose (on clara):
     `mkdir -p ~/coyote-bitstreams/loom-ce-uwinhbm && cp /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/hw/build_sep30_uwinhbm/bitstreams/cyt_top.pdi ~/coyote-bitstreams/loom-ce-uwinhbm/`
     `rsync -a /scratch/harshanavkis/loom-proj/Coyote/driver/build_versal/ rose.dos.cit.tum.de:/scratch/harshanavkis/loom-proj/Coyote/driver/build_versal/`
     `rsync -a /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build/ rose.dos.cit.tum.de:/scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build/`
     `rsync -a /scratch/harshanavkis/loom-proj/Coyote/scripts/fpga/ rose.dos.cit.tum.de:/scratch/harshanavkis/loom-proj/Coyote/scripts/fpga/`
  3. Program both V80s:
     - clara: `cd /scratch/harshanavkis/loom-proj/Coyote && scripts/fpga/program_v80.sh /home/harshanavkis/coyote-bitstreams/loom-ce-uwinhbm/cyt_top.pdi 0000:81:00.0`
     - rose: `cd /scratch/harshanavkis/loom-proj/Coyote && scripts/fpga/program_v80.sh /home/harshanavkis/coyote-bitstreams/loom-ce-uwinhbm/cyt_top.pdi 0000:61:00.0`
  4. V80 → U280 → V80 on clara (the U280 → V80 hop; compare its rate with
     the same size landing in host memory, the second line):
     `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./ce_local --land-v80 16777216 3`
     `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./ce_local 16777216 3`
  5. clara V80 → rose V80 HBM, a fresh port each run:
     - rose: `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./ce_remote --server --size 1048576 --land-v80 --port 18600`
     - clara: `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./ce_remote --client 131.159.102.21 --reps 3 --port 18600`

- **The U280 -> V80 hop at full network rate, from a plain RDMA sender
  (clara receives, rose sends).** rose's U280 runs perf_rdma
  (`~/coyote-bitstreams/perf_rdma`, 3894367a) as the sender, with a client
  kept out of tree (nothing is committed in `09_perf_rdma`):
  `~/loom-experiments/perf_rdma_landing/` (`landing_client`; build:
  `cd /home/harshanavkis/loom-experiments/perf_rdma_landing/build && nix-shell /scratch/harshanavkis/loom-proj/Coyote/shell.nix --run "cmake .. && make -j16"`).
  clara's U280 runs loom_switch, clara's V80 `loom_ce` with the window's
  discard mode (`~/coyote-bitstreams/loom-ce-discard`, 34dea8cf). The
  receiver programs export 1, times each run from its first landed packet
  to its last, and checks every byte; the perf_rdma image reports no write
  completions to its sender, so the client does not wait for any.
  1. clara (start first; `--discard`: the V80 window answers the writes
     without HBM, nothing is checked; without `--land-v80` they land in
     host memory):
     `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./ce_remote --passive --land-v80 --size 16777216 --msg 1048576 --reps 8 --port 19590`
  2. rose:
     `cd /home/harshanavkis/loom-experiments/perf_rdma_landing/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./landing_client 131.159.102.22 --size 16777216 --msg 1048576 --reps 8 --gap-ms 200 --port 19590`
  2026-10-03: 8.83-8.86 GB/s into HBM, 8.85-8.88 discarded (host memory
  11.2-11.8), byte-exact. `ce_local --land-v80` prints the V80 window's
  counters per copy (the burst sizes of the U280's writes into the V80).

- **Gets (WORKFLOW 6.5): rose's CPU reads amy's export.** A get is a read of
  an rdma window (`loom_read`). Both U280s run
  `~/coyote-bitstreams/loom-switch-read` (build_oct04_read, b5a6b36d,
  `-DACK_GAP_CYCLES=16`, md5 9fb3dbdf; WNS -0.371 in the shell's HBM and
  rdma_mem_intf paths). The reader is rose: both hosts' root ports treat a
  PCIe completion timeout as fatal (`UESvrt CmpltTO+`, 65-210 ms), so a
  load that was never answered could reset the reader's host, and amy is
  shared. amy and rose have older trees, so they are flashed with an
  out-of-tree helper that JTAG-programs by part and loads
  `~/coyote-drivers/coyote_driver-u280-0930.ko` with the host's IP and MAC:
  1. On clara:
     `ssh amy.dos.cit.tum.de "bash ~/loom-experiments/get/flash_u280.sh /home/harshanavkis/coyote-bitstreams/loom-switch-read/hw/bitstreams/cyt_top.bit"`
     `ssh rose.dos.cit.tum.de "bash ~/loom-experiments/get/flash_u280.sh /home/harshanavkis/coyote-bitstreams/loom-switch-read/hw/bitstreams/cyt_top.bit"`
  2. The binary and library to both (on clara):
     `for h in amy rose; do rsync -a /scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/sw/build/get_bench $h.dos.cit.tum.de:/scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/sw/build/ && rsync -a /scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/sw/build/coyote/libcoyote.so $h.dos.cit.tum.de:/scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/sw/build/coyote/; done`
  3. A read with no network first, on rose (a load where no window is
     bound: all ones, from the card): `sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 /home/harshanavkis/loom-experiments/get/read_probe`
  4. amy, then rose (`SRV=amy CLI=rose ~/loom-experiments/get/run_get.sh TAG
     --port N` does both and the nstats deltas; a fresh port per run, the
     last one's stays bound for a while):
     - amy: `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/sw/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./get_bench --server --port 18501`
     - rose: `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/sw/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./get_bench --client 131.159.102.20 --port 18501`
  2026-10-05: GET BENCH PASS three times (300k reads, every byte checked,
  0 retransmissions, 0 stray completions). One CPU load at a time, 8 B or
  32 B: 7.26 us median (p99 7.5-7.9). The CPU keeps one uncached load in
  flight per core, so a core reads 32 B per 7.3 us (0.004 GB/s; every 32 B
  load is one read, nothing combines). k threads at once: 1, 2, 4, 8 take
  the same 15.3 ms for 64 KiB each (k reads in flight, no slowdown); 16
  take 27 ms (0.039 GB/s; likely the responder's 4 host reads ahead,
  inferred). A load past amy's export returns all ones. Reads at bandwidth
  need a DMA reader; the V80 copy engine reads only its own HBM today.
  The first form (5f710592, the CPU stored a request word into a get
  window and polled a completion word; amy reading rose: 7.8 us, 11.6 GB/s
  with 16 x 256 KiB in flight) is replaced.
  Writes on the same image, 2026-10-05 (clara, rose, all byte-exact, 0
  retransmissions): `uwin_probe` (G2) PASS on clara and rose; `ce_local`
  6.69-6.82 GB/s and `ce_local --land-v80` 4.43-4.72 (as before);
  `ce_remote` clara -> rose 11.08-11.24, rose -> clara 9.8-10.1 with the
  same two slow copies as before (rose's copy engine); both ways at once
  (`~/loom-experiments/bidir/loom_bidir.sh`) 6.5-6.6 each way. That last
  one was 7.3 on 10-03, but today the 10-03 image (loom-switch-ackgap16)
  gives 6.5-6.6 too (one run 7.1), so it is not the RTL. Nor is it the
  flash: the read image flashed 3 times, 2 runs each
  (`~/loom-experiments/bidir/flashvar.sh`), gives 6.5-6.7 every run, with
  identical PCIe settings and shell counters; the limit is clara's shell
  write path (landing writes wait 1.6-2.6 M cycles a run to enter the
  MMU, rose's ~0; the DMA engine never pushes back). What made the 7.1 and
  7.3 runs faster is not known. The window's CPU mapping was uncached-minus, not
  write-combining: the driver `pci_iomap`ed the whole bypass BAR, and PAT
  downgraded the window's write-combining mmap inside it, so every CPU
  store was its own PCIe write (`uwin_probe` bulk 0.02 GB/s on clara, 0.08
  on rose). The driver now maps only the BAR below the window
  (`coyote_driver-u280-uwinwc.ko`; PAT shows the window write-combining):
  `uwin_probe` bulk 0.15-0.21 GB/s on clara, 1.06-1.27 on rose. Whole
  lines arrive, but not in address order (~1.5 lines a packet, the
  ingress queue full most of the run), and clara takes each host write
  request slowly. CPU loads: same 7.3 us each, a thread's bulk read
  0.044-0.076 GB/s (was 0.004; ~1 read per 64 B line up to 16 KiB), but
  more threads add nothing (~0.045 in total, with free read slots: a cap
  upstream of the switch, not identified). Copy-engine runs unchanged.

- **amy and rose, both with a V80 and a U280 (since 2026-10-05).** clara's
  V80 moved to amy (`81:00.0`); rose's cards to `e1:00.0` / `c1:00.0`; all on
  socket 1. U280s on `loom-switch-read` (write-combining driver), V80s on
  `loom-ce-discard` (`scripts/fpga/program_v80.sh <pdi> <bdf>`; a V80 that
  trains Gen4 x8 comes up x16 when programmed again). Runners in
  `~/loom-experiments/bidir/`: `loom_pair.sh CLI SRV` (`ce_remote --bidir`
  between any two hosts, SX/CX `--no-send` for one way), `bidir_pair.sh` /
  `run_pr.sh` (stock perf_rdma). 2026-10-05, 16 MiB copies, byte-exact, 0
  retransmissions:
  - local (amy / rose): V80 self-loop 10.92 / 10.91 GB/s; V80 -> host
    12.57 / 12.40; V80 -> U280 -> host 10.63 / 10.64; V80 -> U280 -> V80
    HBM 10.62 / 10.62 (clara: 6.7 and 4.5).
  - remote: amy -> rose 11.09-11.15, rose -> amy 10.83-11.23, both ways at
    once 10.7 / 10.5 each way (clara <-> rose: 6.6).
  - gets (rose reads amy): 7.28 us a load; CPU puts 0.87-1.03 GB/s.
  - stock perf_rdma on the same U280s: one way 12.25, both ways 11.49 /
    11.58; with clara both ways is winner-take-all (10.5-10.6 into clara,
    5.5-5.6 out), whichever host is its partner.

- **Copy-engine gets (WORKFLOW 6.5): rose's V80 reads amy's export into
  its HBM.** `loom_ce` with CSR 6 (DIR) = 1 reads the imported window and
  writes HBM. V80 image: build_oct05_get (`examples/loom_ce/hw`, same
  arguments as `loom-ce-discard`), staged to
  `~/coyote-bitstreams/loom-ce-get`. U280s stay on `loom-switch-read`. The
  put programs (`ce_local`, `ce_remote`, `v80_bisect`) write DIR = 0 before
  every copy, so they run on either V80 image.
  1. Build the software on clara and sync it to both hosts:
     `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && nix-shell ../../../../shell.nix --run "cmake .. && make -j16"`
     `for h in amy rose; do rsync -a /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build/ $h.dos.cit.tum.de:/scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build/; done`
  2. Program rose's V80 (on rose; program again if it trains Gen4 x8):
     `cd /scratch/harshanavkis/loom-proj/Coyote && scripts/fpga/program_v80.sh /home/harshanavkis/coyote-bitstreams/loom-ce-get/cyt_top.pdi 0000:c1:00.0`
  3. No network first, on rose: reads where no window is bound, which the
     U280 answers on the card with all ones, so this times the
     peer-to-peer read path alone:
     `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./ce_get --local --reps 3`
  4. amy serves, rose reads (a fresh port per run):
     - amy: `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_switch/sw/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./get_bench --server --port 18601`
     - rose: `cd /scratch/harshanavkis/loom-proj/Coyote/examples/loom_ce/sw/build && sudo /nix/store/4sjg21zq04394ci22lj67vgpw7ykw9bg-numactl-2.0.18/bin/numactl -N 1 -m 1 ./ce_get --client 131.159.102.20 --port 18601 --reps 3`
  Each get prints the copy engine's time, every byte checked, and the
  reads the U280 saw and their mean size.
  2026-10-05, both V80s on `loom-ce-get` (md5 4ce86b05), CE GET PASS,
  byte-exact, 0 retransmissions:
  - `--local` (rose): 5.88 GB/s from 4 MiB. The V80 reads at most 256 B
    (max read request 512 or 4096 B; 128 B reads at 128, 3.05 GB/s), and
    each read costs ~43 ns whatever its size: a per-read limit upstream
    of `loom_read` (whose answer path takes 3-4 cycles a read).
  - over the network (rose reads amy): 0.31 GB/s from 1 MiB, 0.23 at
    4 KiB; 8 x 256 B per ~6.6 us round trip, the shell's 8 reads in flight
    into the window (`cr_ctrl.tcl`: `axi_main` / `axim_udata`
    `NUM_READ_OUTSTANDING 8`).
  - puts on the new image, unchanged: rose self-loop 10.91, V80 -> host
    12.57, V80 -> U280 -> host 10.64 (amy 10.64), -> V80 HBM 10.62;
    `uwin_probe` 1.22 (amy 0.96); `loom_pair.sh` rose -> amy 10.79-10.86,
    amy -> rose 11.01-11.31, both ways 10.4-10.5 into rose and 10.7-10.8
    into amy.
  amy had crashed at 15:25 (kernel oopses in slab code with the Coyote
  drivers loaded) and came back without its U280 on the bus: JTAG-program
  the U280, warm reboot, then insmod the driver with amy's IP and MAC.

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
| G6 | the receive side into V80 HBM: V80 → U280 → V80 on one host (`ce_local --land-v80`, also the U280 → V80 rate), then clara V80 → rose U280 → rose V80 HBM (`ce_remote --server --land-v80`), byte-exact |

## Builds in flight (2026-09-29)

- `examples/loom/hw/build_sep29_ctrl`: control rebuild of today's Loom on the
  merged stack (U280, 2023.2).
- `examples/07_perf_fpga/hw/build_v80`: example 07 for the V80 (2025.1), for
  G0/G1.
