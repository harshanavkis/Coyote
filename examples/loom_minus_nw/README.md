# loom_minus_nw

Both ends of the Loom link on one vFPGA, with the RoCE stack removed.

`examples/loom` cannot separate "the host write path is slower than the line"
from "the network perturbed the stream": every corrupt two-host run has both
a receive-path stall and retransmissions, and the counters cannot say which
came first. Here there is no transport at all - `loom_engine`'s outgoing
stream is fed straight back into `loom_rx` - so `loom_rx`'s move/starve/stall
split measures the host write path and nothing else.

The RTL is the same. `hw/src/hdl/loom_{ctrl,table,engine,rx}.sv` are symlinks
into `examples/loom`, so there is one copy and no drift.

## What it can and cannot answer

Can: the receive path's sustained host-write rate; whether its stalls are
uniform (structural) or bursty (bufferable); whether the engine's pull
sustains line rate; whether the 2.5-vs-12 GB/s bimodality of the two-host
runs reproduces with no network involved.

Cannot: retransmission. There is no transport, so the beat displacement seen
on hardware cannot arise on its own. `loom_mnw_pktzr` has a drop injection
for asking that question deliberately instead (tied off until it is wired to
a CSR).

## Differences from examples/loom's top

- `EN_RDMA 0`. Drops the RoCE stack, and with it `EN_DCARD`, which also
  sidesteps the u280 DDR/HBM reference-clock conflict that killed build_aug3.
- `eng_net_*` goes to `loom_mnw_pktzr` and on to `loom_rx`, rather than to
  `axis_rreq_send`. The packetiser puts a tlast every 64 beats (4096 B at
  PMTU) so the branch in `loom_rx` that ignores intermediate tlasts is still
  exercised.
- `rq_wr` is gone. `loom_rx` ties `rq_ready` high and takes target and length
  from the message header, so tying valid low is the whole change.
- The engine's `STRM_RDMA` write request is acked and dropped: it waits on
  `wr_ready` in `ST_DMA_WR_REQ`, but the request has no queue with
  `EN_RDMA 0`, and the fence is a local posted completion driven by the
  engine's own streaming rather than by the shell completing that request.
- No ingress FIFO. `examples/loom` puts one in front of `loom_rx` to keep
  host-write stalls off the RoCE ingress; with no RoCE stack there is nothing
  to protect, and feeding `loom_rx` with zero slack is what makes its stall
  counter a measurement of the host write path rather than of the buffer in
  front of it.

## Running

Smoke test (compile + elaborate against the shell's interfaces):

```bash
examples/loom_minus_nw/hw/tb/run_elab.sh      # expect PASS: tb_mnw_elab
```

It does not exercise the data plane. `tb_loom_loopback` in `examples/loom`
already covers engine -> wire format -> loom_rx against a modelled link, and
this top instantiates the same modules.

Bitstream:

```bash
cd examples/loom_minus_nw/hw && mkdir -p build && cd build
CMAKE_BIN=$(nix-shell -p cmake --run 'dirname $(command -v cmake)')
xilinx-shell -c "export PATH=$CMAKE_BIN:\$PATH; cmake .. -DFDEV_NAME=u280"
tmux new-session -d -s mnw_bit \
  "xilinx-shell -c 'export PATH=$CMAKE_BIN:\$PATH; make project && make bitgen' \
   > bitgen.log 2>&1"
```

Sanity check in the generated `base.tcl`: `en_rdma 0`, `en_dcard 0`.

## Software

One process, one cThread, one card. `loom.hpp` is shared with
`examples/loom` (include path, not a copy).

```bash
cd examples/loom_minus_nw/sw && mkdir -p build && cd build
nix-shell -p cmake gcc boost --run 'cmake .. && make -j8'
sudo ./mnw_bench
```

Options, matching loom_host so numbers sit next to the two-host ones:
`LOOM_BENCH_ONLY=<bytes>` for a single size, `LOOM_BENCH_ITERS=N` for the
descriptors per size (default 32), `LOOM_POLL_SECS` for the fence timeout.

It programs window 1 rdma-route with `base = dst`, sets `RDMA_STAGING_VA`
and `RX_PID` to this cThread's ctid, then sweeps 64 B to 4 MB verifying
every word and checking nothing lands past the end.

### Reading a run

The table gives cyc/op from the engine's dma-rdma stage accumulator, wall
clock per op and the achieved rate. Below it:

- `transmit path cycles` - starved means the engine's pull left gaps in
  the outgoing stream. On the two-host runs this swung between 22% and 84%
  and took throughput with it; if that reproduces here it is a host-side
  effect and has nothing to do with the network.
- `receive path cycles` - stalled means the host write path would not take
  a beat. This is the number the whole example exists to isolate.
- `longest unbroken stall` (RO word 24) against the stall total. A sum
  cannot tell a burst from a sustained deficit and cannot size a buffer;
  the longest run can. Under ~512 cycles the stalls are clustered and
  buffering is the right lever; comparable to the total and the write path
  is simply slower than the feed, in which case no depth fixes it.
- `rx_hdr_reject` must stay 0. It has been 0 on every two-host run except
  the one that wedged, and a nonzero value here means the framing
  desynchronised with no network in the picture at all.
