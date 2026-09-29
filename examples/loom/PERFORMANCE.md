# Performance: Loom vs perf_rdma vs ConnectX-7 vs E810

Measured 2026-09-28/29. All four systems use RDMA WRITE over one QP. The
Loom numbers in the comparison tables use **only the copy-engine (DMA)
path**, with no aperture stores. The store path is reported separately at
the end.

## Setup

| System | Hosts | Device | Link | NUMA | Software |
|---|---|---|---|---|---|
| **Loom** | clara ↔ amy | Alveo U280, Coyote shell, `loom-portcnt` bitstream (md5 `36de2752`, HEAD RTL = `40c7620c`) | 100G through the FS N8550-32C switch (flow control on the FPGA ports) | 1 (card) | `loom_host` @ `88853322`, ACK window 32 |
| **perf_rdma** | clara ↔ amy | same U280s, `perf_rdma` bitstream (md5 `3894367a`; build commit not recorded) | same | 1 | stock `examples/09_perf_rdma` (unmodified, built from this tree at `20f5f14d`) |
| **E810** | clara ↔ amy | Intel E810-C, irdma, RoCE v2 | 100G through the same switch (flow control on ports 1/0/7-8 since 2026-09-28), MTU 9000 (RoCE 4096) | 0 (NIC) | perftest 26.07.7, rdma-core 62.0 |
| **ConnectX-7** | jamie ↔ ian | ConnectX-7, fw 28.44.1036, RoCE v2 | **200G direct cable**, MTU 1500 (RoCE 1024) | 0 / 1 (NIC) | perftest 26.07.7, rdma-core 62.0 |

Every run reported here had 0 retransmits and 0 PSN drops, and every Loom
run verified the landed bytes.

## Latency, one way (µs)

| Size | Loom (DMA) | perf_rdma | ConnectX-7 | E810 |
|---|---|---|---|---|
| 64 B | 6.42 | 5.45 | 1.12 | 5.53 |
| 256 B | 6.38 | 5.58 | 1.65 | 6.24 |
| 1 KiB | 6.71 | 5.74 | 1.78 | 6.63 |
| 4 KiB | 7.31 | 6.68 | 1.95 | 7.49 |
| 16 KiB | 8.67 | 7.74 | 2.46 | 9.48 |
| 64 KiB | 12.45 | 11.81 | 4.63 | 17.21 |
| 256 KiB | 29.40 | 28.69 | 13.11 | 47.70 |
| 1 MiB | 97.76 | 98.77 | 47.12 | 171.01 |
| 4 MiB | 375.33 | 373.63 | 183.04 | 654.84 |

- **Loom vs perf_rdma:** on the same cards and stack, Loom adds about
  1 µs at small sizes (the engine pulls the source, then sends it). The two
  meet from 256 KiB up, where the link sets the pace.
- **ConnectX-7:** about 5× lower at small sizes. At large sizes its 200G
  link halves the wire time, so compare against it per link rate there.
- **E810:** close to perf_rdma at small sizes and slowest at large ones. It
  has a per-QP receive limit of about 50 Gb/s (see Throughput).

## Throughput (GB/s, 10⁹ B/s)

| Size | Loom (push) | perf_rdma | ConnectX-7, 1 QP | E810, 1 QP |
|---|---|---|---|---|
| 64 B | 0.037 | 0.101 | 0.339 | 0.220 |
| 256 B | 0.145 | 0.450 | 1.354 | 0.904 |
| 1 KiB | 0.600 | 1.727 | 5.43 | 3.55 |
| 4 KiB | 2.16 | 5.76 | 20.59 | 6.86 |
| 16 KiB | 5.75 | 9.44 | 23.10 | 6.30 |
| 64 KiB | 9.89 | 10.54 | 23.14 | 6.22 |
| 256 KiB | 11.75 | 11.25 | 23.14 | 6.19 |
| 1 MiB | 11.87 | 11.41 | 23.15 | 6.21 |
| 4 MiB | 11.90 | 11.48 | 23.15 | 6.21 |

- **Large sizes:** Loom reaches about 95% of 100G (11.9 GB/s), slightly
  above perf_rdma. The ConnectX-7 figure is 185 Gb/s of payload on a 200G
  link, and one QP is enough to reach it. The E810 is capped near 50 Gb/s
  per QP; with 2 QPs it reaches 11.6 GB/s, and with 4 it reaches 12.3.
- **Small sizes** measure the host's cost of *starting* transfers, not the
  network. A Loom `copy()` is six uncached control-register writes (about
  1.75 µs each at 64 B), a perf_rdma `invoke` is fewer, and a verbs post is
  cheaper still. The three methods also time different things (see
  Methodology).

## Bidirectional, one QP, 4 MiB (GB/s per direction)

| System | One-way | Both ways at once |
|---|---|---|
| Loom | 11.8 | 5.9 + 5.9 (halves) |
| perf_rdma | 11.3–11.7 | 5.22 incoming (halves; lossy at 32 × 4 MiB) |
| ConnectX-7 | 24.5 (wire) | 24.5 + 24.5 |
| E810 | 6.33 (1 QP) | 6.0 + 6.3 |

The halving is Coyote-specific. One RC QP carrying both directions doesn't
halve on either ASIC NIC. The cause is still open; see
`HANDOVER-bidir.md` and the ACK-gap lead in the project notes.

## Small writes and reads, 8–64 B (µs)

| Size | perf_rdma write | ConnectX-7 write (inline) | ConnectX-7 read | E810 write (inline) | E810 read |
|---|---|---|---|---|---|
| 8 B | 5.28* | 1.06 | 2.19 | 5.40 | 10.48 |
| 16 B | 5.23* | 1.07 | 2.13 | 5.43 | 10.47 |
| 32 B | 5.29* | 1.08 | 2.21 | 5.52 | 10.51 |
| 64 B | 5.45 | 1.13 | 2.19 | 5.48 | 10.60 |

- **Write** is one way (half a ping-pong). **Read** is the full round trip
  of one RDMA READ.
- **\*** From a perf_rdma run started at 8 B, which later hung at 128 B;
  not re-measured. perf_rdma and Loom's copy engine start at 64 B. That's
  Loom's gate G5: no RDMA payload smaller than one 64 B beat.
- **Inline:** payloads up to 101 B (E810) or 220 B (ConnectX-7) travel
  inside the work request. With inline off, a write costs about 0.45 µs
  more on both NICs.
- **Loom** has no DMA transfer below 64 B and no remote reads yet (item
  6.2b).

## Why the ConnectX-7 is faster

The software is essentially the same on all three RDMA stacks:
- posting is an uncached write to the device (verbs doorbell, or Coyote's
  control registers)
- completion is detected by polling host memory (perftest polls the
  payload; Coyote writes its landing counter back into host memory)

So the gap is in the device. On the ConnectX-7, the one-way latency of an
8 B write breaks down cleanly (`MLX5_SHUT_UP_BF=1` stops the CPU pushing the
work request):

| CPU pushes the work request | Payload inline | One way | What the NIC fetches from host memory |
|---|---|---|---|
| yes | yes | 1.06 µs | nothing |
| no | yes | 1.50 µs | the work request |
| yes | no | 1.51 µs | the payload |
| no | no | 1.94 µs | both |

Each fetch from host memory costs about 0.44 µs, one PCIe round trip.
Coyote always fetches the payload, so its closest ConnectX-7 equivalent is
the 1.51 µs row. The remaining ~4 µs is the FPGA path on both sides: the
250 MHz HLS RoCE stack and the shell. Experiment E4 (stage counters) will
attribute Loom's share.

## Methodology

**Latency.** All four are ping-pongs reported as RTT/2.

| System | One round | Completion detected by | Rounds | Statistic |
|---|---|---|---|---|
| Loom `--pingpong-dma` | client `copy()`; server sees the payload's last word, `copy()`s it back; client sees its last word | polling the payload's last word (the flag) | 1 warm-up + 32 per size | mean |
| perf_rdma | client `invoke` WRITE; server waits for its landing counter, writes back; client waits for its counter | Coyote's landing counter, written back to host memory | 50 per size, no warm-up, TCP sync before each | mean |
| perftest `ib_write_lat` | client posts WRITE; server polls the buffer's last byte, posts back | polling the payload | 1,000 per size (`-a` sweep); 10,000 for the 8–64 B runs | median ("typical") |

**Throughput:**
- **Loom push:** 32 `copy()`s back to back per size (at most 48 in flight),
  timed until the last descriptor's sender-side fence, i.e. its last beat
  handed to the RoCE stack.
- **perf_rdma:** 32 writes, then the server writes 32 back; time ÷ 2 per
  direction. Reported in MiB/s, converted here.
- **perftest `ib_write_bw`:** 1 QP, 5,000 iterations, send-queue depth 128,
  timed by send completions.

**Caveats:**
- The ConnectX-7s are on a different host pair (jamie/ian), on a direct
  cable, at 200G, with RoCE MTU 1024. The other three are clara/amy through
  a switch at 100G.
- The perf_rdma bitstream's build commit isn't recorded.
- In `loom-portcnt`, `loom_rx`'s `rx_chunk` input is unconnected
  (`vfpga_top.svh`) and synthesis tied it to 0, so the receiver lands each
  message with one host write of its whole length, not one per packet, and
  the `RX_CHUNK` CSR (word 76) has no effect. The Loom numbers here are in
  that mode.
- Means (Loom, perf_rdma) and medians (perftest) are reported as the tools
  give them.

## Loom store path (AXI-Lite aperture), not part of the comparison

A CPU memcpy onto the peer pointer, sent as uncached 8 B aperture stores
(`--store-pingpong`). One way, payload plus length and flag stores:

| Bytes | Stores | One way |
|---|---|---|
| 8 | 1 | 4.78 µs |
| 64 | 8 | 7.34 µs |
| 128 | 16 | 10.66 µs |
| 256 | 32 | 17.05 µs |
| 512 | 64 | 30.09 µs |
| 1024 | 128 | 56.43 µs |
| 2048 | 256 | fails: the 64-deep order FIFO drops stores |

- **Per store:** about 0.41 µs each after the first, set by the host's
  uncached write path. The engine itself takes 2 cycles (8 ns) per store.
- **64 back-to-back stores** (`--stores 64`): 0.39 µs per store on the host,
  with none dropped.

## Reproducing

**Loom** (from `examples/loom`, `BIT=` pointing at the `loom-portcnt`
bitstream):

```bash
./run_two_host.py --pingpong-dma --size 0 --iters 32 --gap 0 --retries 0 --tx-window 32
./run_two_host.py --size 0 --iters 32 --gap 0 --retries 0 --tx-window 32
./run_two_host.py --store-pingpong --size 0 --iters 32 --gap 0 --retries 0 --tx-window 32 --no-flash
```

**perf_rdma:** flash its bitstream with the runner's `flash()` (see
README), then start the server on amy first:

```bash
/home/harshanavkis/loom-experiments/perf_rdma_stock/server/test -o 1 -x 64 -X 4194304 -r 50
/home/harshanavkis/loom-experiments/perf_rdma_stock/client/test -i 131.159.102.20 -o 1 -x 64 -X 4194304 -r 50
```

Both run under `sudo`, pinned with `numactl -N 1 -m 1`. The binaries are
built from this tree's unmodified `examples/09_perf_rdma`.

**E810 and ConnectX-7:** `~/loom-experiments/perftest-build/`
(`run_lat.sh`, `run_e810.sh`, `run_lat_cx7.sh`, `run_bw_cx7.sh`; its
README has the raw tables). The `-a` sweeps are in `runs/sweep/`.
