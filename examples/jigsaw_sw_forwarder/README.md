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

## Protocol

- One 64 B mailbox slot **per direction** (requests at offset 0, responses
  at offset 64), so every slot has exactly one writer and a node's
  send-source bytes are never overwritten by incoming traffic — the
  stack's retransmitter re-reads local memory on replay, and under strict
  ping-pong this guarantees any replayable packet has stable source bytes.
- The publish flag is a **monotonic counter** (written last; RDMA WRITE
  places bytes in increasing address order): a replayed message is
  re-placed identically at the receiver and recognized as already seen —
  duplicate detection for hardware-level replays, not a retry layer.
- Strict ping-pong: at most one request in flight, ever. `clearCompleted`
  after every round trip on both nodes (the baseline pair's cadence).
- The device replays MMIO **verbatim**, like the guest driver: writes are
  `setCSR` + immediate response, reads are `getCSR`; the host's forwarded
  STATUS polls do the waiting. The device never waits on the accelerator.
- Payloads live behind the control page at offsets mirroring the ivshmem
  layout; ≤1 MiB per transfer (guest-driver chunking convention). An H2D
  payload is pushed before its trigger request; a D2H payload is pushed
  before the response of the first STATUS read that reports "done" — same
  QP, so data is always placed before the signal that announces it.
- No retries: a missing response after 5 s dumps diagnostics and aborts.

## Components

- `common/messages.hpp` — wire layout only (message struct, ops, register
  map, buffer layout); protocol logic sits inline in each `main.cpp`.
- `sw_device` — device-node replayer: `perf_rdma` on vFPGA 0,
  `jigsaw_baseline` on vFPGA 1. Replays MMIO through the accelerator CSRs
  (CSR index = BAR offset >> 3), rewrites guest DMA pointers to a dedicated
  device staging buffer.
- `sw_device_selftest` — single-node debug tool: replays the full-size
  Vortex trace directly on the accelerator vFPGA (no RDMA), with the
  forwarder's exact register sequences, chunking and staging buffer.
- `sw_host` — VM-path daemon: identical ivshmem/doorbell protocol to
  `jigsaw_host_controller/sw`, so the QEMU/guest stack runs unchanged.
  Run pinned to one core (`taskset -c <core>`). (Pending rewrite onto
  `messages.hpp`; currently not building.)
- `sw_host_no_vm` — bring-up/benchmark harness: same Vortex trace replay
  as `jigsaw_host_controller/sw_no_vm` (shares its `traces.hpp`), through
  the forwarding path.

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
cd sw_host_no_vm/build && ./test -i <device_oob_ip> [-r <trace runs>]
```

or the VM daemon (with the VM started as in the jigsaw e2e setup):

```
cd sw_host/build && taskset -c <core> ./test -i <device_oob_ip>
```
