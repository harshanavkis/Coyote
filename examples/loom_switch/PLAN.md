# Loom split: V80 copy engine + U280 switch

Branch `loom-v80`. Two new apps next to the existing `examples/loom/`, which
stays as it is:

- `examples/loom_ce/` (V80): the emulated accelerator's copy engine. It reads
  its source from V80 HBM and writes it to a destination virtual address. The
  V80's Coyote MMU maps that address, via a dma-buf, onto the U280's ingress
  window, so the writes travel peer-to-peer over PCIe.
- `examples/loom_switch/` (U280): Loom's switch. It turns writes arriving on
  the ingress window into local or RoCE writes by binding. There is no
  descriptor path. CPU stores through the AXI-Lite aperture and `loom_rx`
  landing stay.

Hosts: clara ↔ rose (a V80 and a U280 each). Builds go through
`scripts/fpga/build_bitstream.sh` (U280: Vivado 2023.2; V80: 2025.1).

## Data path

```
V80 loom_ce:  HBM --read--> CE --sq_wr LOCAL_WRITE to dst VA-->  V80 MMU/TLB
                                   (dst VA mapped via dma-buf to U280 BAR)
PCIe P2P:     MWr TLPs  ------------------------------------->  U280 bypass BAR
U280 shell:   bypass BAR 0x0800_0000.. -> new AXI4 512-bit port -> vFPGA
U280 loom_switch: decode window -> binding (loom_table) -> local sq_wr, or
                  RDMA WRITE (PMTU segments, ACK window) -> RoCE
far U280:     loom_rx lands the data (unchanged)
```

## 1. Shell: a user data window (U280)

The XDMA's AXI bypass BAR (256 MB, 64-bit prefetchable, 512-bit AXI) already
enters the `shell_ctrl` block design (`hw/bd/ultrascale_plus/cr_ctrl.tcl`),
which splits it:

| Offset | Size | Port |
|---|---|---|
| `0x0000_0000` | 32 KB | shell config (AXI-Lite) |
| `0x0010_0000 + i·256 KB` | 256 KB | `axi_ctrl_i`, the vFPGA's AXI-Lite control (Loom's aperture) |
| `0x0100_0000 + i·256 KB` | 256 KB | `axim_ctrl_i`, 256-bit AVX to the shell registers |
| **`0x0800_0000 + i·64 MB`** | **64 MB** | **new: `axim_udata_i`, 512-bit AXI4 into vFPGA *i*** |

- **Config flag:** a new `EN_UWIN` (default 0), so no existing example changes.
- **Templates to change:** `cr_ctrl.tcl` (port and address segment),
  `shell_top` / `dynamic_top` / `user_wrapper` / `user_logic` (route the port
  to `vfpga_top` as `axi_udata`), `FindCoyoteHW.cmake` (the flag).
- **Writes:** reach the vFPGA as AXI4 bursts. The payload is posted, so the
  B channel must be answered promptly.
- **Reads:** answered with zeros, since Loom has no remote reads (6.2b).

## 2. Driver (U280) and user library

- **mmap:** a new `MMAP_UWIN` offset in `vfpga_ops.c` mapping the region
  write-combining, so the host CPU can also write it (gate G2).
- **dma-buf exporter:** an ioctl that exports the region as a dma-buf, so the
  V80's driver can import it through its existing path (`vfpga_gup.c`, as in
  `06_gpu_p2p`). The exporter returns an sg_table holding the BAR's bus
  address (`dma_map_resource` in the importer's attach).
- **User library:** `cThread` maps the window (`getUwin()`) and exports its
  fd (`exportUwin()`). On the V80 side, an existing-style import maps the fd
  at a virtual address.

## 3. RTL

- **`loom_switch` (U280):** starts from `examples/loom` and removes the DESC
  path from `loom_engine`. A new `loom_ingress`:
  - AXI4 write slave (512-bit)
  - window decode (offset → window 1..15 → binding) using `loom_table`
  - per binding, turns the write stream into requests: `LOCAL_WRITE` (local
    route) or `APP_WRITE` over `STRM_RDMA` (RETH vaddr = binding base +
    offset), one request per PMTU with the 7b ACK window
  - arbitrates with the store path and `loom_rx` on `sq_wr` using the
    existing per-request arbiter
  - ordering per binding follows arrival order; the B response is sent once
    a beat is accepted into the queue
- **`loom_ce` (V80):** example 07's structure with source and destination
  CSRs, `sq_rd` from card memory (`STRM_CARD`, `EN_MEM=1`), the read stream
  forwarded to `sq_wr` on the host stream (dst VA), and a completion fence
  write.

## 4. Testbenches

- `tb_loom_ingress`: AXI4 bursts (aligned, unaligned, back-to-back, mixed
  windows) → the exact request sequence (opcodes, vaddr, len, PMTU split),
  with backpressure on `sq_wr` and the window.
- `tb_loom_switch_top`: ingress + stores + `loom_rx` arbitration.
- `tb_loom_ce`: descriptor → `sq_rd`/`sq_wr` sequence, stream forwarding,
  fence.
- Same style and runner as `examples/loom/hw/tb/` (`run_tbs.sh`).

## 5. Gates

| Gate | Check |
|---|---|
| G0 | clara drives the U280 and the V80 at once (two driver modules); example 07 runs on the V80 |
| G1 | P2P spike: V80 example 07 writes its counter pattern into an exported U280 region; the host reads it back |
| G2 | host CPU writes into the U280 window (WC mmap) appear on `loom_ingress` counters |
| G3 | TBs green, `ooc_synth` timing met on both apps |
| G4 | clara V80 → clara U280 → rose U280 → rose host, byte-exact, 0 retransmits; then the local route |
| G5 | performance rerun of PERFORMANCE.md (push, DMA ping-pong) |

## Builds in flight (2026-09-29)

- `examples/loom/hw/build_sep29_ctrl`: control rebuild of today's Loom on the
  merged stack (U280, 2023.2).
- `examples/07_perf_fpga/hw/build_v80`: example 07 for the V80 (2025.1), for
  G0/G1.
