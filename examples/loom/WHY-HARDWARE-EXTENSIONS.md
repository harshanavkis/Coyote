# Why does Loom need hardware extensions?

A likely reviewer question: *"Why does Loom need new hardware? Couldn't the
same be done on existing NICs, or on a programmable SmartNIC or DPU?"* This
note is the answer, written 2026-09-28. Vendor facts are cited at the end.
The Loom numbers are our own measurements on the U280 prototype.

## Short answer

Loom's contribution is where the translation happens. An endpoint issues
plain loads, stores and copy-engine descriptors to a mapped peer address. A
network element between the endpoint and the wire turns each one into local
forwarding or reliable RDMA, so the endpoint never builds work requests,
holds queue pairs or polls completions.

That needs one thing no shipping NIC or SmartNIC provides in hardware: **a
translation stage on the device's host interface.** Writes to a decoded
address window are treated as transactions and routed, at per-transaction
hardware speed, by bindings the orchestrator owns.

- **Fixed-function RDMA NICs** don't have the stage at all. They implement
  the verbs contract; they can't absorb it. Anything built on them alone is
  one of our baselines.
- **Programmable SmartNICs and DPUs** put their programmability in packet
  pipelines and embedded cores. A host store reaches that programmability
  only as a TLP or doorbell handed to software. So they can emulate Loom's
  contract, but in software per transaction, which is a slower
  implementation of the same design, not a counterexample.
- **FPGA SmartNICs** are where the stage can be added as hardware today.
  The prototype uses one as the extension, not as an alternative to it.

The extension itself is small: an address decode, a table lookup and
encapsulation in front of a standard RDMA transport. It's the same kind of
change industry makes in silicon to get memory semantics across a fabric
(UALink/SUE endpoints, Huawei's UnifiedBus). Loom puts it in the network
element instead of the accelerator.

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

Requirement 4 exists today; requirements 1–3 are the extension.

## Existing device classes

| Class | Examples | Host-facing interface | Programmability | Loom requirements 1–3 |
|---|---|---|---|---|
| Fixed-function RDMA NIC | ConnectX-7, Intel E810 | verbs: doorbell and work-request pages in the NIC's own format | none on the host path | no. Only the B1/B2 baselines can be built |
| SoC SmartNIC / DPU | NVIDIA BlueField-3, AMD Pensando Elba, Intel IPU E2000/E2100 | fixed emulations (NVMe, virtio) or generic PCIe device emulation in software | P4 packet pipelines, Arm cores; BlueField-3 also DPA cores | emulated in software, per transaction |
| FPGA SmartNIC | Alveo U280 + Coyote (this prototype) | whatever the logic implements | hardware on the host path | yes. This is the extension |

**Fixed-function NICs.**
- A store of data to an arbitrary BAR offset isn't an operation they
  understand, and flow steering matches packets, not host MMIO.
- What you can build is our baselines:
  - a library turning `copy()` into verbs, with a proxy thread for stores,
    is B2 (CPU-proxy)
  - an accelerator building work requests and ringing doorbells itself
    (IBGDA/DeepEP) is B1 (GPU-initiated)
- Both put the transport on the endpoint, which is the cost Loom removes
  (for example, DeepSeek-V3 reserves up to 20 SMs for communication).
- They do provide requirement 4 well. On our testbed the E810 doesn't halve
  in bidirectional traffic, while the Coyote stack does.

**SoC SmartNICs and DPUs.**
- Their programmable blocks act on packets or run as embedded software:
  - P4 match-action pipelines: Pensando Elba has 144 match processing units
    ([AMD][elba]); the Intel IPU has a P4 pipeline ([Intel][ipu])
  - Arm core complexes (16 on Elba, up to 16 Neoverse N1 on the IPU E2000)
- A P4 pipeline processes packets after they exist; it doesn't sit on the
  host's MMIO writes.
- The host-facing side is fixed device emulation (NVMe, virtio) or, on
  BlueField-3, generic emulation in software.
- So a host store reaches the programmable part only by being handed to
  software. That covers requirements 1–3 functionally but not at hardware
  speed.

## The most capable documented case: BlueField-3

BlueField-3 is the strongest SoC case because it documents generic PCIe
device emulation:

- **DOCA DevEmu PCI TLP:** the DPU presents a custom PCIe function. "Any
  read or write to an address within this region by the host driver is
  passed to the application running on the DPU in the form of a memory read
  or memory write ... TLP request" ([DevEmu PCI TLP][tlp]). This meets
  requirement 1 functionally: every aperture store becomes visible.
  - It is software on the Arm cores, per TLP.
  - It needs BlueField-3 firmware 32.49.0288+ and a recent DOCA 3.x.
  - NVIDIA publishes no latency or throughput numbers for it.
- **DOCA DevEmu PCI Generic:** stateful BAR regions notify Arm software
  "that a write has occurred", and doorbell regions notify DPA handlers. The
  documented example allows 64 doorbells per device. The DPU can DMA and
  RDMA host memory through an endpoint mmap ([DevEmu PCI Generic][generic]).
  That covers requirements 2 and 3 in software.
- **Device emulation is BlueField-3 and later only**, and the DPA ships only
  on BlueField-3 ([NVIDIA][bf3-blog]).
- **Measured DPA performance:** single-thread performance "up to 26× lower"
  than host or Arm cores, and aggregate throughput 7.5× below the host CPU
  ([Chen et al.][dpa-paper]).

## Why software emulation isn't enough

- On the U280 prototype, the translation stage wraps a store for the network
  in 2 cycles (8 ns at 250 MHz, measured with the T3 stage counters).
- On an emulating SmartNIC, each store is a TLP or doorbell handled by an
  embedded core.
- Stores are exactly the fine-grained, high-rate operation where
  per-transaction software cost dominates.
- Bulk is different: one descriptor becomes one RDMA write, which runs at
  line rate on either kind of device. A software emulation would therefore
  look fine on bulk and fall behind on stores.

## Likely follow-up: "your stores cost 0.4 µs anyway"

Measured on the prototype (2026-09-28):
- a back-to-back store costs about 0.4 µs
- a 1 KiB store memcpy takes 56 µs one way
- a single store takes 4.8 µs one way

The 0.4 µs is the **host's** issue path: uncached 8 B MMIO writes (the
Coyote driver maps the region `pgprot_noncached`) entering through a 64-bit
AXI-Lite port. Every PCIe device pays it, an emulating SmartNIC included,
which then adds its per-store software on top. Loom's own share is the 8 ns.
The prototype's store cost therefore bounds the PCIe emulation, not the
design. A real switch port, or a write-combining mapping with a wide AXI4
window, removes most of it.

## "What about CXL?"

Some newer devices list a CXL host interface; the Intel IPU E2100 lists
"PCIe/CXL" ([Intel][ipu]). CXL makes the host link memory-semantic: 64 B
lines, many requests in flight. That would remove most of the host issue
cost above. It doesn't remove the need for the translation stage. Each line
the device receives still has to be matched to a binding and sent to the
bound peer, and on these devices that would again be embedded software.
Also, a store miss to write-back CXL memory first reads the line
(read-for-ownership); for a line that lives on another host, that read is
a remote round trip. So CXL is a better host link for the extension, not a
replacement for it.

## A hybrid, if a reviewer asks for an ASIC transport

Keep the translation stage (aperture, bindings, encapsulation) on the FPGA,
and use a commodity RDMA NIC only as the transport. The FPGA posts work
requests to it over PCIe peer-to-peer, as a GPU does under IBGDA. The
endpoint contract stays Loom's and the wire becomes an ASIC. The cost is
implementing the NIC's work-request, doorbell and completion formats in the
FPGA. It's a credible "future work", and it shows that the extension is the
translation stage, not the transport.

## One-paragraph rebuttal

> Loom requires one capability no shipping NIC provides in hardware: a
> translation stage on the host interface that treats writes to a decoded
> address window as transactions and routes them by orchestrator-installed
> bindings. Fixed-function RDMA NICs implement the verbs contract rather
> than absorbing it; any construction on them places the transport back on
> the endpoint (our B1/B2 baselines). Programmable SmartNICs place their
> programmability in packet pipelines and embedded cores: a host store
> reaches it only as a TLP or doorbell handed to software (e.g., BlueField-3
> DOCA DevEmu), so Loom's contract is expressible there, but as software per
> transaction, on cores whose single-thread performance is up to 26x below a
> host core. The extension itself is small (address decode, table lookup,
> encapsulation in front of a standard RDMA transport, 8 ns per store in our
> FPGA prototype), and it is the same class of change industry makes in
> silicon to obtain memory semantics across fabrics.

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
- [elba]: AMD Pensando Elba product brief,
  https://www.amd.com/content/dam/amd/en/documents/pensando-technical-docs/product-briefs/pensando-elba-product-brief.pdf
- [ipu]: Intel IPU Adapter E2100 product page,
  https://www.intel.com/content/www/us/en/products/details/network-io/ipu/adapter-e2100.html

[bf3-blog]: https://developer.nvidia.com/blog/power-the-next-wave-of-applications-with-nvidia-bluefield-3-dpus/
[tlp]: https://networking-docs.nvidia.com/doca/archive/3-3-0/doca-devemu-pci-tlp
[generic]: https://networking-docs.nvidia.com/doca/archive/3-4-0/doca-devemu-pci-generic
[dpa-paper]: https://arxiv.org/abs/2402.03041
[elba]: https://www.amd.com/content/dam/amd/en/documents/pensando-technical-docs/product-briefs/pensando-elba-product-brief.pdf
[ipu]: https://www.intel.com/content/www/us/en/products/details/network-io/ipu/adapter-e2100.html
