# Can Loom run on existing SmartNICs? Why does it need hardware extensions?

A likely reviewer question: *"Why does Loom need new hardware? Couldn't it
run on existing NICs, or on a current or upcoming SmartNIC/DPU?"* This note
is the answer, written 2026-09-28. Vendor facts are cited at the end. The
Loom numbers are our own measurements on the U280 prototype.

## Short answer

**Loom's contract:** an accelerator reaches any peer, in its rack or across
the datacenter, through the same loads, stores and copy-engine writes it
uses for a local peer. A network element between the peers routes each
transaction: over the scale-up fabric if the peer is local, over reliable
RDMA if it's remote. The endpoint never builds work requests, holds queue
pairs or polls completions.

**Running that needs two things from hardware:**
1. **A translation stage on the host interface.** Writes to a decoded
   address window are treated as transactions and routed, at
   per-transaction hardware speed, by bindings the orchestrator owns.
2. **The right placement.** The element has to sit on the path of every peer
   transaction, local and remote, because that's where the two domains
   meet. In Loom's design it's the ToR switch: on the rack's scale-up fabric,
   with RoCE uplinks.

**What existing devices can do:**
- **Fixed-function RDMA NICs** (ConnectX-7, E810) have neither. Only our
  baselines can be built on them.
- **The most capable SmartNICs** (BlueField-3/4) can run Loom's contract for
  remote peers, but as software per transaction. They also sit at one
  host's edge of the scale-out network, not on the scale-up path, so they
  can unify the two domains only partially.
- **The closest silicon to Loom's placement** (Enfabrica ACF-S: a PCIe
  switch and an Ethernet NIC on one chip) keeps verbs as the remote
  interface. It consolidates the hardware without unifying the interface.

So Loom runs on existing SmartNICs only as a degraded, per-host software
version of itself. The hardware extension Loom needs is the translation
stage, placed where both fabrics meet. It is small: address decode, table
lookup and encapsulation in front of a standard RDMA transport.

## What Loom needs from the device

1. **A data aperture.** A store or copy-engine write to any offset of a
   mapped window has to become a write to the bound peer address, with no
   endpoint software per transfer.
2. **One route decision for both domains.** The same window mechanism
   serves a local peer (short-circuited over the scale-up fabric) and a
   remote one (over RDMA). The endpoint can't tell which.
3. **Bindings owned by the network element.** The orchestrator programs
   route and translation state (window to destination, pid, bounds) into the
   device. The endpoint never holds it.
4. **Landing in the destination's address space, and reliable in-order
   delivery.** Every RDMA NIC already provides these.

Requirement 4 exists today; requirements 1–3 are the extension.

## Existing device classes

| Class | Examples | Host-facing interface | Programmability | Loom on it |
|---|---|---|---|---|
| Fixed-function RDMA NIC | ConnectX-7, Intel E810 | verbs: doorbell and work-request pages in the NIC's own format | none on the host path | no. Only the B1/B2 baselines |
| SoC SmartNIC / DPU | NVIDIA BlueField-3/4, AMD Pensando, Intel IPU E2000/E2100 | fixed emulations (NVMe, virtio) or, on BlueField-3+, generic PCIe device emulation in software | P4 packet pipelines, Arm cores; BlueField-3 also DPA cores | remote peers only, in software per transaction, per host |
| NIC + PCIe switch on one chip | Enfabrica ACF-S | PCIe switch ports to GPUs plus Ethernet NIC ports | not for the host path | right placement; interface is still verbs |
| FPGA SmartNIC | Alveo U280 + Coyote (this prototype) | whatever the logic implements | hardware on the host path | yes, per host. This is the extension |

**Fixed-function NICs.**
- A write of data to an arbitrary BAR offset isn't an operation they
  understand, and flow steering matches packets, not host MMIO.
- What you can build is our baselines:
  - a library turning copies into verbs, with a proxy thread, is B2
    (CPU-proxy)
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
  host's writes.
- The host-facing side is fixed device emulation, or generic emulation in
  software on BlueField-3 and later. So a host write reaches the
  programmable part only by being handed to software.

## The most capable case: BlueField-3 (and BlueField-4)

- **DOCA DevEmu PCI TLP:** the DPU presents a custom PCIe function. "Any
  read or write to an address within this region by the host driver is
  passed to the application running on the DPU in the form of a memory read
  or memory write ... TLP request" ([DevEmu PCI TLP][tlp]). That meets
  requirement 1 functionally.
  - It is software on the Arm cores, per TLP.
  - It needs BlueField-3 firmware 32.49.0288+ and a recent DOCA 3.x.
  - NVIDIA publishes no latency or throughput numbers for it.
- **DOCA DevEmu PCI Generic:** stateful BAR regions notify Arm software
  "that a write has occurred", and doorbell regions notify DPA handlers. The
  documented example allows 64 doorbells per device. The DPU can DMA and
  RDMA host memory through an endpoint mmap ([DevEmu PCI Generic][generic]).
  Bindings could live on the DPU (requirement 3).
- **Device emulation is BlueField-3 and later only**, and the DPA ships only
  on BlueField-3 ([NVIDIA][bf3-blog]). Measurements of the DPA find
  single-thread performance "up to 26× lower" than host or Arm cores, and
  aggregate throughput 7.5× below the host CPU ([Chen et al.][dpa-paper]).
- **BlueField-4** (announced for 2026) scales the same model rather than
  changing it:
  - a 64-core Grace CPU (Neoverse V2), ConnectX-9 at 800 Gb/s, and a
    PCIe Gen6 host link
  - SNAP, VirtIO and NVMe device emulation ([STH][bf4])
  - more and faster cores for the software path, but still a host-edge
    device with software emulation

## Why software emulation isn't enough

- **Stores.** On the U280 prototype, the translation stage wraps a store
  for the network in 2 cycles (8 ns at 250 MHz, measured with the T3 stage
  counters). On an emulating SmartNIC, each store is a TLP handled by an
  embedded core.
- **Bulk data, too.** In Loom's design, an unmodified accelerator's copy
  engine writes bulk data to the aperture as ordinary PCIe writes, and the
  switch segments them. On an emulating SmartNIC, those writes also arrive
  as TLPs for software.
  - At 400 Gb/s with 256 B writes, that's on the order of 200 million TLPs
    per second. Our arithmetic, not measured.
  - The alternative is a descriptor doorbell the DPU acts on (as our
    prototype's copy engine does). But then the endpoint uses a different
    operation for remote peers than for local ones, which breaks
    requirement 2.
- In short, software emulation keeps the contract but pays software per
  transaction on both the store and the bulk path.

## Unifying the domains: why placement matters

Loom's goal isn't only removing transport work from the accelerator. It's
**one contract for scale-up and scale-out**, so that where a peer sits
decides nothing in the application. That needs one element that sees every
peer transaction and chooses the route per binding. In the design, the ToR
switch is that element: it sits on the rack's scale-up fabric and on the
Ethernet uplinks, so it short-circuits local peers and encapsulates remote
ones.

A SmartNIC sits at one host's edge of the scale-out network. Scale-up
traffic (NVLink, PCIe peer-to-peer between accelerators) never passes
through it. That leaves two options, and both lose something:

1. **Local peers stay on the native fabric; the SmartNIC handles only
   remote peers.**
   - The endpoint sees pointers in both cases, but the route is decided in
     two places: the accelerator's own peer mappings for local peers, and
     the SmartNIC for remote ones.
   - Isolation, bindings and failure handling each exist twice, managed by
     different software.
   - This is close to what communication libraries do today (NVSHMEM:
     peer-to-peer locally, IB remotely). It removes the endpoint's transport
     cost for remote peers, but the divide remains in the infrastructure.
2. **Local traffic also goes through the SmartNIC (loopback).**
   - One element decides every route, but local peers lose the scale-up
     fabric: their traffic crosses the NIC's PCIe link and its software path
     instead of NVLink or the rack's PCIe fabric.
   - That breaks the other half of Loom's question: preserving the
     performance of local peer access.
   - It also covers only peers in the same host, not the rest of the rack.

The one existing chip whose placement matches Loom's is **Enfabrica ACF-S**.
It combines PCIe switch ports to GPUs with multi-port 800 GbE on one die.
But its remote path is "based on InfiniBand Verbs" ([Business Wire][acfs]):
it consolidates the hardware and keeps both programming contracts. That
makes it evidence for Loom's point rather than against it. Even the most
integrated silicon keeps the divide in the interface. It's also where
Loom's extension would naturally go: add the translation stage to a chip
like that.

Our FPGA prototype is itself a per-host element (the "per-host controller"
variant in the implementation plan), so we state the same placement caveat
in the paper. What the prototype demonstrates is the translation stage and
the unified contract. The switch placement is carried by the simulation.

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

## A hybrid, if a reviewer asks for an ASIC transport

Keep the translation stage (aperture, bindings, encapsulation) on the FPGA,
and use a commodity RDMA NIC only as the transport. The FPGA posts work
requests to it over PCIe peer-to-peer, as a GPU does under IBGDA. The
endpoint contract stays Loom's and the wire becomes an ASIC. The cost is
implementing the NIC's work-request, doorbell and completion formats in the
FPGA. It's a credible "future work", and it shows that the extension is the
translation stage, not the transport.

## One-paragraph rebuttal

> Loom needs a translation stage on the host interface, one that routes
> writes to a decoded address window by orchestrator-installed bindings, and
> it needs that stage where the scale-up and scale-out fabrics meet. No
> shipping device provides both. Fixed-function RDMA NICs implement the verbs
> contract rather than absorbing it (our B1/B2 baselines). Programmable
> SmartNICs such as BlueField-3/4 can emulate a custom PCIe function whose
> writes reach embedded cores as TLPs, so Loom's contract is expressible
> there, but as software per transaction, on stores and on copy-engine
> writes alike, and only at one host's edge of the scale-out network:
> scale-up traffic never passes through it, so the domains are unified only
> for remote peers or at the cost of local performance. The one chip placed
> on both fabrics, Enfabrica's ACF-S, keeps verbs as its remote interface.
> Loom's extension is small (address decode, table lookup, encapsulation in
> front of a standard RDMA transport; 8 ns per store in our FPGA prototype)
> and belongs in exactly such a device.

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
- [bf4]: ServeTheHome, "NVIDIA BlueField-4 with 64 Arm Cores and 800G
  Networking Announced for 2026",
  https://www.servethehome.com/nvidia-bluefield-4-with-64-arm-cores-and-800g-networking-announced-for-2026/
- [elba]: AMD Pensando Elba product brief,
  https://www.amd.com/content/dam/amd/en/documents/pensando-technical-docs/product-briefs/pensando-elba-product-brief.pdf
- [ipu]: Intel IPU Adapter E2100 product page,
  https://www.intel.com/content/www/us/en/products/details/network-io/ipu/adapter-e2100.html
- [acfs]: Business Wire, "Enfabrica Unveils Industry's First Ethernet-Based
  AI Memory Fabric System ...", 2025-07-29,
  https://www.businesswire.com/news/home/20250729711298/en/

[bf3-blog]: https://developer.nvidia.com/blog/power-the-next-wave-of-applications-with-nvidia-bluefield-3-dpus/
[tlp]: https://networking-docs.nvidia.com/doca/archive/3-3-0/doca-devemu-pci-tlp
[generic]: https://networking-docs.nvidia.com/doca/archive/3-4-0/doca-devemu-pci-generic
[dpa-paper]: https://arxiv.org/abs/2402.03041
[bf4]: https://www.servethehome.com/nvidia-bluefield-4-with-64-arm-cores-and-800g-networking-announced-for-2026/
[elba]: https://www.amd.com/content/dam/amd/en/documents/pensando-technical-docs/product-briefs/pensando-elba-product-brief.pdf
[ipu]: https://www.intel.com/content/www/us/en/products/details/network-io/ipu/adapter-e2100.html
[acfs]: https://www.businesswire.com/news/home/20250729711298/en/
