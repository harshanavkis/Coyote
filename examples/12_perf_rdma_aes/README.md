# Example 12 — AES-GCM over RDMA, encrypt on one node, decrypt on the other

`09_perf_rdma` with the RDMA WRITE payload routed through Jigsaw AES-256-GCM
engine banks. Both nodes run the **same bitstream**; a CSR bit picks the role.

```
   TX   axis_host_recv[0] → aes_gcm_bank(ENCRYPT) → axis_rreq_send[0]
   RX   axis_rrsp_recv[0] → aes_gcm_bank(DECRYPT) → axis_host_send[1]
```

Control path is stock `09_perf_rdma` — the shell keeps descriptors,
packetisation and completions. RDMA READ streams are plain pass-throughs.

## Source

`hw/src/hdl/` is vendored from `harshanavkis/corundum` branch **`sec-enc-dec`**,
`fpga/app/jigsaw/rtl/jigsaw_modules` — the *streaming* engine
(`aes_gcm_stream.v`), not the older per-packet-controller version. Upstream's
`jigsaw_pkt_processor.v` is unused: it fuses decrypt + `txn_generator` +
encrypt for a NIC-resident pipeline. `aes_gcm_bank.v` is that cut to one
direction.

| File | Change vs upstream |
|---|---|
| `aes_gcm_stream.v` | new `iv_dir` input drives IV bit 95 — upstream bakes it into `IV_INIT` at elaboration, needing one bitstream per role |
| `aes_gcm_encryption.v` | threads `iv_dir` through |
| `aes_gcm_decryption_taglast.v` | new: tag-**last** variant. Upstream wants the tag first, but our peer is an FPGA whose encrypt engine emits the tag only after the packet |

## Length convention

`dreq_rdma_parser_wr.sv` fragments every WRITE at `PMTU_BYTES` (4096) and
commits each fragment's length to the RoCE header before the vFPGA sees the
payload, so **the user region cannot change a fragment's length**. The last 16
bytes of every fragment are the tag slot:

```
TX  [ payload  n−16 ][ 16 pad ]  → strip pad, encrypt, append tag → n bytes
RX  [ ciphertext n−16 ][ 16 tag ] → strip tag, decrypt, append computed → n bytes
```

A transfer of `L` bytes carries `L − 16·⌈L/4096⌉` bytes of payload; the
benchmark reports **Wire** and **Goodput** separately. `L` must be a multiple
of 16 with every fragment ≥ 32 B — any multiple of 64 works, and
`valid_transfer_size()` skips the rest.

## IV spaces and the role bit

Each engine derives a packet's IV from its position in that engine's frame
sequence since reset (engine *n* of *N*: `IV_INIT = n`, `IV_STRIDE = N`), so
encrypt and decrypt stay in lockstep with no key exchange — given the in-order
lossless delivery RoCE RC provides.

Both directions share the zero PoC key, so node A's transmit must not share an
IV space with node B's — `(key, IV)` reuse breaks GCM. Bit 95 separates them:
TX gets `iv_dir = role`, RX gets `~role`. It is sampled when a bank leaves
reset and `CTRL.enable` releases reset, so **write role first, enable second**.

## CSR map

| # | Name | Meaning |
|---|---|---|
| 0 | `CTRL` | `[0]` enable (0 = bypass), `[1]` role |
| 1 | `STATUS` | `[0]` rx quarantine, `[1]` sticky "a tag verified" |
| 2–4 | `TAG_OK`, `TX_FRAMES`, `RX_FRAMES` | fragment counters |
| 5 | `MEAS_CTRL` | `[0]` arm — clears counters and stamps |
| 6–9 | `FIRST_CYCLE`, `LAST_CYCLE` | 48-bit receive-side stamps |

## Build and run

```bash
cd hw && mkdir build_hw && cd build_hw
cmake ../ -DFDEV_NAME=u280 && make project && make bitgen

cd sw && mkdir build_sw && cd build_sw
cmake ../ -DINSTANCE=client   # client | server | client_time | server_time
make
```

```bash
./test --crypto 1                          # server node
./test --ip_address <server> --crypto 1    # client node
```

`--crypto` must match on both sides. Run the pair twice, `--crypto 1` and
`--crypto 0`, and the delta is the AES cost on one bitstream with nothing else
changed — that is the number worth quoting.

`client`/`server` are fixed-reps, fine for a smoke test. **`client_time`/
`server_time` are what you quote**: adaptive batching, calibration against a
target duration, barriers against counter overflow. They mirror each other
barrier for barrier — change the cadence on one side, change it on the other.

## Sizing `N_AES_ENGINES`

One engine is 128 bit/cycle at 250 MHz = **32 Gb/s**, sustained one beat per
cycle (no per-packet flush — that was the old `aes_gcm_controller`). So N
engines per direction ≈ 32·N Gb/s.

The default is **2, and that is a resource decision.** The retired AES echo
build (`Coyotev2/Coyote/examples/13_aes_gcm_rdma/hw/build/reports/`) measured
238,573 LUTs for two cores ⇒ **~119k LUTs/core**, against a 287k-LUT RDMA
shell on a 1.30M-LUT U280.

| per direction | total | verdict |
|---|---|---|
| 1 | 0.53 M | fits |
| 2 | 0.76 M | should fit |
| 4 (upstream default) | **1.24 M** | will not place and route |

That build also missed timing at **−2.567 ns** / 164k endpoints with only two
cores, so expect timing work even at 2. Small messages lose further to the
per-packet J0 block and tag beat: upstream measures 50% duty at 33 B, 97% at
1500 B.

## Verification status

- `verilator --lint-only` clean over all six `NUM_AES_ENGINES × DIRECTION`
  combinations. Needs `-fno-gate` (a Verilator internal assertion, not a design
  fault) and a stub for the 34 MB generated `top_aes_gcm.v`, whose 131k-token
  line Verilator cannot parse but Vivado can.
- Instantiation ports and parameters cross-checked against every module.
- `constants.hpp` tag-slot arithmetic unit-tested against an independent
  per-fragment model over the whole 64 B – 1 MiB sweep, boundaries included
  (4112 B rejected, its tail fragment would be tag-only; 4160 B accepted).
- **Simulated**: `hw/tb/bank_loopback/` runs the encrypt bank into the decrypt
  bank under cocotb + Icarus and checks three things — payload round-trips
  byte-for-byte, the ciphertext between the banks matches Python's AES-256-GCM
  under the IV the positional scheme should have used, and `tag_ok` pulses
  once per frame with no quarantine. Passes at `ENGINES=1`, `2` and `4`. The software
  cross-check matters: it is what distinguishes a correct pipeline from one
  that is merely self-consistent.
- **Coyote co-sim: attempted, blocked by the toolchain.** `make sim` fails at
  the DPI step in *both* installed Vivados: 2023.2 and 2025.1 each bundle
  `binutils-2.37`, whose `ld` cannot parse this host's glibc 2.42 `.relr.dyn`
  sections, so `coyote_sim.so` never links. (2025.1 additionally needs
  `LD_LIBRARY_PATH=/usr/lib64` for `libtinfo.so.6`, and both need a cmake newer
  than the 3.3.2 on the shell's PATH.) Working around it means hand-building the
  DPI library with a modern linker.

  Worth knowing before anyone retries: **the co-sim could not have settled the
  framing question anyway.** It mocks the shell rather than instantiating it,
  and `sim/hw/stream_simulation.svh` emits one `tlast` per *transfer*
  (`last = current_block + 1 == n_blocks`) with no PMTU concept anywhere in
  `sim/`. It would have replayed a different assumption, not tested this one.
  What it *would* still validate is the vFPGA integration: CSR decode, the
  role-before-enable sequencing, stream plumbing and tie-offs.

- **Not** synthesised or run on hardware. First bring-up should confirm
  `TAG_OK == RX_FRAMES`, the client's payload check, and the `Framing:` line
  the server now prints (see below) before any throughput number is believed.

## Two bugs the simulation caught

Worth recording, because neither is visible to lint and both would have cost a
bitstream:

1. **Collector select advanced on the wrong handshake.** `axis_mux` samples
   `select` when it *internally* starts a frame, and its output sits a register
   stage behind that. Advancing `coll_sel` on the mux's *output* `tlast` meant
   that for back-to-back frames the mux picked the next frame before the
   counter had moved, re-selecting the same engine. Frames left the bank as
   0,1,3,2,5,4,7,6. Because the IV is positional, a frame in the wrong slot is
   encrypted under the wrong IV, corrupting its ciphertext and its tag. Fixed
   by advancing on the selected port's own end-of-frame.

   The dispatcher never had this problem: `axis_demux` samples `select` at the
   input start-of-frame, which is the same handshake `disp_sel` already counts.

   **`jigsaw_pkt_processor.v` upstream drives its `axis_mux` the same way**, so
   `NUM_AES_ENGINES > 1` there is likely affected. Upstream's testbenches drive
   a single wrapper rather than a bank, so it would not show up.

2. **`tag_quarantine` was asserted whenever idle.** `pause = (tag_credits == 0)`
   is also the resting state, so `STATUS[0]` would have read 1 permanently and
   the software would have printed a quarantine warning on every healthy run.
   Now gated on an outstanding-frame count.

3. **`tag_ok` was an OR across engines rather than a count.** With several
   engines in parallel two can verify on the same cycle, so `TAG_OK` undercounted
   and the server's `tag_ok != rx_frames` check would report failures on a
   healthy run. Only reproduced at `ENGINES=4` — at 2 the collision never
   happened. The bank now emits how many engines verified this cycle and the
   vFPGA accumulates that.

Bugs 2 and 3 share a shape: neither breaks the datapath, both corrupt the
evidence. On hardware they would have made a working design look broken —
and bug 3 specifically would have blocked the `TAG_OK == RX_FRAMES` gate this
README tells you to check before trusting a throughput number.

## The framing assumption, and how it is checked

The tag slot is the last 16 bytes of every **frame**, so usable payload depends
on how many frames a transfer becomes — which is a property of the shell's
fragmentation, not of this code. `dreq_rdma_parser_wr.sv` splits a WRITE into
`PMTU_BYTES` sub-requests, so `fragments_for()` assumes `ceil(len/4096)`.

That assumption is **inferred from reading the RTL, not verified**. Rather than
leave it silent, the server derives the truth at runtime: it accumulates the
frames the assumption predicts and compares them against the `RX_FRAMES`
counter, printing either

    Framing:             as assumed (4096 B fragments); payload accounting is correct

or a `MISMATCH` line with the ratio. Note what is and is not at stake — the
datapath is correct either way, because `axis_strip_tail_beat` reserves the last
16 bytes of whatever a frame turns out to be. A mismatch means the reported
**Goodput is scaled wrong**, not that the crypto is broken.

## Limitations

- All-zero key, no runtime key loading (upstream PoC limitation).
- Positional IVs assume lossless in-order delivery. A dropped or reordered
  fragment desynchronises the counters permanently: tags stop verifying and the
  decrypt bank quarantines. Recovery is a reset via `CTRL`.
- A frame whose tag never verifies stalls its engine (upstream's policy).
- `CTRL.enable` must not change while payload is in flight.
- `OUT_FIFO_DEPTH` (8 KiB) bounds the max fragment; raising `PMTU_BYTES` means
  raising it too.
