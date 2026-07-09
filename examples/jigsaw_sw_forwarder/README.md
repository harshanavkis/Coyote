# Jigsaw Software Forwarder

End-to-end **software** forwarding baseline for jigsaw: device interactions
are trapped on the host, encapsulated into wire messages, carried over the
network, and replayed in software against the unmodified `jigsaw_baseline`
accelerator — no jigsaw host/device controller hardware anywhere on the path.

The network is the Coyote RDMA stack used as a dumb NIC (`perf_rdma` vFPGA,
`REMOTE_RDMA_WRITE` only), i.e. the same network hardware as the jigsaw
end-to-end setup, so the comparison isolates software vs. hardware
forwarding.

DMA payloads pay the two staging copies of a real software forwarder
(cf. AvA's shadow buffers, rCUDA's pinned pools):

```
guest/app buffer --memcpy--> NIC buffer --RDMA--> NIC buffer --memcpy--> device buffer --DMA--> accelerator
      (host)                  (host)              (device)                (device)
```

## Components

- `common/wire.hpp` — wire protocol: 64 B request/response rings over
  `REMOTE_RDMA_WRITE` mailboxes with credit flow control (MMIO writes are
  posted, reads round-trip), payload region mirroring the ivshmem layout,
  and the shared host-side forwarder core.
- `sw_device` — device-node replayer: `perf_rdma` on vFPGA 0,
  `jigsaw_baseline` on vFPGA 1. Replays MMIO through the accelerator CSRs
  (CSR index = BAR offset >> 3), rewrites guest DMA pointers to a dedicated
  device staging buffer, and pushes D2H payloads before answering status
  reads so completion is never visible before the data.
- `sw_host` — VM-path daemon: identical ivshmem/doorbell protocol to
  `jigsaw_host_controller/sw`, so the QEMU/guest stack runs unchanged.
  Run pinned to one core (`taskset -c <core>`).
- `sw_host_no_vm` — bring-up/benchmark harness: same raw DMA sweep and
  Vortex trace replay as `jigsaw_host_controller/sw_no_vm` (shares its
  `traces.hpp`), through the forwarding path.

## Hardware

Both nodes are flashed with the `jigsaw_baseline_rdma/hw` bitstream
(vFPGA 0: perf_rdma, vFPGA 1: jigsaw_baseline). The host node only uses
vFPGA 0.

## Running

Device node first:

```
cd sw_device/build && ./test
```

Then, on the host node, either the no-VM harness:

```
cd sw_host_no_vm/build && ./test -i <device_oob_ip> [-n <iters>] [-r <trace runs>]
```

or the VM daemon (with the VM started as in the jigsaw e2e setup):

```
cd sw_host/build && taskset -c <core> ./test -i <device_oob_ip>
```
