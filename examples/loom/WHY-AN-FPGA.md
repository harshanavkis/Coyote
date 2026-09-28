# Why an FPGA, and not a commodity NIC or DPU?

A likely reviewer question: *"Why build Loom on a Coyote FPGA? Couldn't you do
the same with a ConnectX-7, or on a BlueField DPU?"* This note is the answer,
written 2026-09-28. Vendor facts are cited at the end. The Loom numbers are
our own measurements on the U280 prototype.

## Short answer

Loom's contribution is where the translation happens. An endpoint issues
plain loads, stores and copy-engine descriptors to a mapped peer address. A
network element between the endpoint and the wire turns them into local
forwarding or reliable RDMA, so the endpoint never builds work requests,
holds queue pairs or polls completions.

- A **ConnectX-7** has no programmable element on its MMIO path. It
  implements the verbs contract; it can't absorb it. Anything built on it
  alone is one of our baselines, not Loom.
- A **BlueField-3** can host Loom's contract, but only as software per
  transaction on the DPU. That makes it a slower, software implementation of
  the same design, not a counterexample. BlueField-2 can't.
- The **FPGA** gives the translation a hardware pipeline. That's the
  property being evaluated, and it's the closest available stand-in for the
  switch silicon the design targets.

## What Loom needs from the device

1. **A data aperture.** A store to any offset of a mapped window has to
   become a write to a bound remote (or local) address, with no endpoint
   software per transfer.
2. **A transport-agnostic copy descriptor.** `copy(dst, src, len)` must not
   require the endpoint to know QPs, keys or a NIC's work-request format.
3. **Bindings owned by the network element.** The orchestrator programs
   route and translation state (window to destination, pid, bounds) into the
   device. The endpoint never holds it.
4. **Landing in the destination's address space, and reliable in-order
   delivery.** Every RDMA NIC already provides these.

## ConnectX-7

- It covers requirement 4 natively: RC transport, memory regions and rkeys.
  It does so better than the Coyote stack does: ASIC latency and full-duplex
  line rate. On our testbed the E810 doesn't halve in bidirectional traffic,
  while Coyote does.
- It doesn't cover 1–3. Its BAR exposes doorbell and BlueFlame pages that
  take work requests in the NIC's own format. A store of data to an
  arbitrary offset isn't an operation it understands. Flow steering matches
  packets, not host MMIO.
- It has no programmable data-path cores. The DPA ships only on BlueField-3
  ([NVIDIA][bf3-blog]).
- **What you can build on it is our baselines.**
  - A library that turns `copy()` into verbs, with a proxy thread for
    stores, is B2 (CPU-proxy).
  - An accelerator that builds work requests and rings doorbells itself
    (IBGDA/DeepEP) is B1 (GPU-initiated).
  - Both put the transport on the endpoint, which is the cost Loom removes
    (for example, DeepSeek-V3 reserves up to 20 SMs for communication).

## BlueField-3

What it adds over a ConnectX-7 is a programmable element that sits between
the host and the wire:

- **DOCA DevEmu PCI TLP:** the DPU presents a custom PCIe function. "Any
  read or write to an address within this region by the host driver is
  passed to the application running on the DPU in the form of a memory read
  or memory write ... TLP request" ([DevEmu PCI TLP][tlp]). This meets
  requirement 1: every aperture store becomes visible.
  - It is software on the Arm cores, per TLP.
  - It needs BlueField-3 firmware 32.49.0288+ and a recent DOCA 3.x.
  - NVIDIA publishes no latency or throughput numbers for it.
- **DOCA DevEmu PCI Generic:** stateful BAR regions notify Arm software
  "that a write has occurred", and doorbell regions notify DPA handlers per
  doorbell. The documented example allows 64 doorbells per device. The DPU
  can DMA and RDMA host memory through an endpoint mmap
  ([DevEmu PCI Generic][generic]). That covers requirement 2 (a descriptor
  doorbell becomes an RDMA write from host memory) and requirement 3 (tables
  on the DPU).
- **Device emulation is BlueField-3 and later only** ([DevEmu][generic]).
  BlueField-2 has no DPA and no generic emulation, so there Loom collapses
  to a proxy thread running on the DPU.

**Where it falls short of the FPGA: the store path is software.**

- On the U280 prototype, wrapping a store for the network takes the engine
  2 cycles (8 ns at 250 MHz, measured with the T3 stage counters).
- On a BlueField-3, each store is a TLP handled by Arm software, or a
  doorbell handled by a DPA thread.
- Measurements of the DPA find single-thread performance "up to 26× lower"
  than host or Arm cores, and aggregate throughput 7.5× below the host CPU
  ([Chen et al.][dpa-paper]).
- Stores are exactly the fine-grained, high-rate operation where
  per-transaction software cost dominates.
- Bulk is different: one descriptor becomes one RDMA write, which should
  run at line rate on either device.

**Where it would beat the prototype:** it has an ASIC transport at
200/400 Gb/s, it's commercially available, and the transport has none of
Coyote's issues.

## Likely follow-up: "your stores cost 0.4 µs anyway"

Measured on the prototype (2026-09-28):
- a back-to-back store costs about 0.4 µs
- a 1 KiB store memcpy takes 56 µs one way
- a single store takes 4.8 µs one way

The 0.4 µs is the **host's** issue path: uncached 8 B MMIO writes (the
Coyote driver maps the region `pgprot_noncached`) entering through a 64-bit
AXI-Lite port. Every PCIe device pays it, a DPU included. A DPU then adds
its per-store software on top. Loom's own share is the 8 ns. The prototype's
store cost therefore bounds the PCIe emulation, not the design. A real
switch port, or a write-combining mapping with a wide AXI4 window, removes
most of it.

## A hybrid, if a reviewer asks for an ASIC transport

Keep Loom's aperture, bindings and translation on the FPGA, and use a
ConnectX-7 only as the transport. The FPGA posts work requests to it over
PCIe peer-to-peer, as a GPU does under IBGDA. The endpoint contract stays
Loom's and the wire becomes an ASIC. The cost is implementing the NIC's
work-request, doorbell and completion formats in the FPGA. It's a credible
"future work", not a hole in the argument.

## One-paragraph rebuttal

> Commodity RDMA NICs implement the verbs contract rather than absorbing it:
> a ConnectX-7 exposes doorbell and work-request pages, not an address window
> whose stores become remote writes, so any NIC-only construction places the
> transport back on the endpoint (our B1/B2 baselines). A BlueField-3 can
> emulate a custom PCIe function whose MMIO writes reach the DPU as TLPs
> (DOCA DevEmu), so Loom's contract is expressible there, but as software per
> transaction on Arm or DPA cores, whose single-thread performance is up to
> 26x below a host core. Loom's premise is that the translation belongs in
> the network element's datapath; we use an FPGA because it is the available
> device where that datapath is hardware (8 ns per store in our prototype),
> as it would be in switch silicon.

## Sources

- [bf3-blog]: NVIDIA Technical Blog, "Power the Next Wave of Applications
  with NVIDIA BlueField-3 DPUs",
  https://developer.nvidia.com/blog/power-the-next-wave-of-applications-with-nvidia-bluefield-3-dpus/
- [tlp]: DOCA DevEmu PCI TLP,
  https://networking-docs.nvidia.com/doca/archive/3-3-0/doca-devemu-pci-tlp
- [generic]: DOCA DevEmu PCI Generic,
  https://networking-docs.nvidia.com/doca/archive/3-4-0/doca-devemu-pci-generic
- [dpa-paper]: X. Chen et al., "Demystifying Datapath Accelerator Enhanced
  Off-path SmartNIC", arXiv:2402.03041, https://arxiv.org/abs/2402.03041

[bf3-blog]: https://developer.nvidia.com/blog/power-the-next-wave-of-applications-with-nvidia-bluefield-3-dpus/
[tlp]: https://networking-docs.nvidia.com/doca/archive/3-3-0/doca-devemu-pci-tlp
[generic]: https://networking-docs.nvidia.com/doca/archive/3-4-0/doca-devemu-pci-generic
[dpa-paper]: https://arxiv.org/abs/2402.03041
