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

Software is not written yet.
