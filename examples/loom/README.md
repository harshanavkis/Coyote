# Loom example (vFPGA switch prototype)

Single vFPGA per host acts as the switch. All emulated-XPU processes and the
control daemon attach to it as cThreads. Changes are confined to vFPGA user
logic and user-space software; the shell and driver stay stock.

> **Running things**: every commit that runs anything documents how to run
> it in the "Running" section at the bottom of this file. Keep it current.

## Ctrl-region layout (64 KB, AXI4-Lite)

- `0x0000-0x0FFF` — CSR page: window-table programming, DMA descriptor
  staging (incl. per-descriptor fence VA) + trigger, RO debug counters
  (words 32-40) and stage cycle counters for T3 (words 48-63: free-running
  cycle counter, order-FIFO residency accumulator, per-stage cycle
  accumulators + completed-op counts; see `loom_ctrl.sv` header).
- `0x1000-0xFFFF` — aperture: 15 windows of 4 KB. Window = `addr[15:12]`,
  offset = `addr[11:0]`. Every write beat here is captured as a small-write
  transaction (posted).

Detailed hardware description and the GPU-equivalence argument:
[HW-DESIGN.md](HW-DESIGN.md).

## Components (hw/src/hdl/)

- **loom_ctrl** — AXI4-Lite slave: CSR page + aperture capture. Captured
  stores and triggered DMA descriptors are pushed into one arrival-ordered
  FIFO (the order point), so a store cannot overtake an earlier descriptor.
- **loom_table** — per-window entry `{valid, route local|rdma, pid, VA base,
  len}`. `base` is always the exporter's own VA. Programmed via CSRs only.
- **loom_engine** — pops the order FIFO.
  - Store, local: `sq_wr {LOCAL_WRITE, STRM_HOST, pid, base+off, 8}` + one
    beat on `axis_host_send`.
  - Store, rdma: `sq_wr {APP_WRITE, STRM_RDMA, pid = QP owner, base+off, 8}`
    + beat on `axis_rreq_send` (RETH vaddr = exporter's VA).
  - Descriptor: pull `sq_rd {src_pid, src_va, len}` via `axis_host_recv`,
    forward the stream to the local or rdma write side; then release the
    descriptor's fence: an incrementing count written to the descriptor's
    completion VA under `src_pid` (CE semaphore-release model).
- **loom_rx** — incoming RDMA writes: forward `(pid, vaddr, len)` from
  `rq_wr` + payload to a local `sq_wr` write.

## Workflow (VAs only)

The full worked example (concrete addresses, all four flows, translation
chain) is in [WORKFLOW.md](WORKFLOW.md). Summary:

Processes A, B on host 1 (pids 0, 1); C on host 2 (pid 0).
`buf_B`/`buf_C` are `getMem` buffers at B's/C's own VAs. Daemon programs:
window 1 -> `{local, pid 1, buf_B, 4MB}`, window 2 -> `{rdma, qp, buf_C, 4MB}`.

1. **Small local** `*(P_B+0x40)=v`: A's MMU -> ctrl `0x1040` -> table ->
   `sq_wr {pid 1, buf_B+0x40}` -> TLB under pid 1 -> B polls `buf_B[0x40]`.
2. **Small remote** `*(P_C+0x40)=v`: same until table -> RDMA write with
   vaddr `buf_C+0x40` on the binding's QP -> host 2 TLB under C's pid.
3. **Bulk local** `copy(P_B+off, src, 1MB)`: descriptor `{win 1/off, src, len}`
   -> pull under pid 0 -> write under pid 1 -> completion word into A's memory.
4. **Bulk remote**: same pull, write side is the RDMA path (vaddr
   `buf_C+off`); shell fragments to PMTU.

Every address is some process's ordinary VA: A's pointers in, exporter's VA
out; the wire carries the exporter's VA (offset in affine encoding).

## Gates (all closed from source, 2026-08-03; hardware owes only
## bring-up validation and performance numbers)

1. Cross-pid write - **CLOSED** (`hw/hdl/mmu/tlb_controller.sv:400`):
   a TLB hit requires `tag match && entry.pid == request.pid` - entries
   are pid-tagged and coexist; the request's pid IS the address-space
   selector and nothing checks who set it (perf_rdma forwards foreign
   pids routinely; 08_multithreading runs multiple cThreads' DMAs on
   one vFPGA). Residual hardware item: none functional (TLB
   pre-population at getMem is a performance behavior).
2. Sub-line writes - **CLOSED** (`hw/hdl/static/xdma_wrapper.sv:49`):
   host writes drive XDMA descriptor-bypass directly with
   `{c2h_addr, c2h_len}` - byte-granular descriptors, payload consumed
   from stream lane 0 (XDMA C2H contract). An 8 B LOCAL_WRITE = a
   {PA, 8} descriptor + our LSB-aligned beat.
3. RX interposition - RESOLVED FROM SOURCE (09_perf_rdma/hw/src/
   vfpga_top.svh): the shell's contract is that an incoming RDMA WRITE
   surfaces as rq_wr (request) + axis_rrsp_recv (payload) and USER
   LOGIC must land it (perf_rdma forwards rq_wr -> sq_wr {STRM_HOST}
   and rrsp_recv -> host_send; nothing lands without user logic). There
   is no silent-to-memory path; loom_rx is the required component, and
   the staging vaddr is pure addressing (the shell never writes it
   itself). Incoming remote READs likewise surface on rq_rd for user
   logic to serve (rq_rd -> sq_rd, host_recv -> rrsp_send) - the 6.x
   remote-read template. The vaddr semantics are ALSO source-confirmed
   (ib_transport_protocol.cpp:628): the RX parser puts
   rdmaHeader.getVirtualAddress() - the RETH vaddr, verbatim, no MR
   table or filter - into the memory command that becomes rq_wr, and
   MIDDLE/LAST packets continue from an MSN-table cursor initialized
   from it; perf_rdma forwards exactly this field into sq_wr and is the
   hardware-validated example. G3 CLOSED - the staging-compare dispatch
   works by construction; nothing deferred to hardware beyond ordinary
   bring-up. (No verbs MR registration exists anywhere in this path;
   validity = a TLB entry under the QP owner's pid at write-issue time.)
4. QP selection - **CLOSED** (`sw/src/cThread.cpp:178`):
   `local.qpn = (vfid << PID_BITS) | ctid` - the QPN literally encodes
   the cThread id, one QP per cThread by construction; the request's
   pid field selects the QP because they are the same number. Our
   engine's `wr_req.pid = entry.pid (QP owner)` is the mechanism itself.
5. Minimum RDMA payload, shell-side evidence (checked in the Coyote
   sources, 2026-08-03): there is NO explicit `len >= 64` check anywhere
   in the TX path - `rdma_req_parser.sv` handles any length (len <= PMTU
   -> WRITE_ONLY with plen = len) and the transport's header math
   (`udpLen = 12+16+payloadLen+4`) is length-agnostic. BUT sub-64 B
   writes sit outside the exercised envelope: Coyote's own perf example
   floors at 64 B (`MIN_TRANSFER_SIZE_DEFAULT = 64`), the header/payload
   merge in `ib_transport_protocol.cpp:append_payload` carries an
   explicit "TODO align this stuff!!", and the sub-word keep helper is
   `lenToKeep(ap_uint<6>)` (< 64 only). jigsaw's unconditional padding
   (`rdma_wr_len = 64`) corroborates empirically. Treat 64 B as the
   de-facto minimum until this gate tests an 8 B `APP_WRITE` on
   hardware. If it fails (expected): Phase-6 fix is a 64 B wire message
   (header + payload); the far-side loom_rx issues the exact 8 B local
   write - naive padding is NOT an option (it would clobber 56 neighbor
   bytes at the destination).

Shell config (when hw is added): `EN_STRM 1, N_STRM_AXI 1, EN_RDMA 1,
N_REGIONS 1`; cf. `examples/jigsaw_baseline_rdma`.

## Deferred optimization TODOs (owner decision points)

- **Write-combining aperture mapping (AVX-512 stores).** The ctrl region
  is mapped `pgprot_noncached` (driver `vfpga_ops.c:595`): UC serializes
  an AVX-512 store into 8 B strongly-ordered accesses, so 64 B never
  arrives as one unit and small-store issue rate pays ~8 UC round trips.
  Two pieces would be needed, because AXI4-Lite itself is single-beat
  at bus width (8 B, no bursts - a 64 B TLP is chopped into 8 beats by
  the XDMA bridge regardless of mapping type):
  (1) a ONE-LINE driver change (`pgprot_writecombine` for `MMAP_CTRL`)
  so the CPU emits 64 B TLPs at all - breaches the "vFPGA + user space
  only" rule, needs explicit owner sign-off;
  (2) a line-reassembly buffer in loom_ctrl (8 consecutive beats -> one
  64 B store; pure user logic, allowed) with the inline message kept as
  the fallback for WC partial flushes.
  Payoff: much better small-store issue rate, and a 64 B-line *direct*
  remote-store fast path (NCCL-LL-style producer-owned lines). Revisit
  at Phase 5.5; until then, latency measurements include UC store costs.

## Status

| Phase | Content | State |
|---|---|---|
| 1 | loom_ctrl (CSR page + aperture capture + order FIFO), loom_table, block TBs | done, TBs pass |
| 2 | loom_engine (store + DMA branches, completion) + block TB | done, TBs pass |
| 3 | loom_rx + block TB | done, TBs pass |
| 4 | vfpga_top wiring + Coyote integration sim (EN_SIM) | done, LOOM TEST PASS |
| 4.5 | test hardening: tb_loom_top (arbitration), engine/ctrl corner cases, extended integration sim, Python RDMA TX test | done, all pass |
| 5.0 | per-descriptor completion (fence VA in descriptor, CE semaphore-release model) | done, all sims pass |
| 5.1a | single-process software: client/server role split (OrchClient iface, in-process transport) | done, ROLES TEST PASS (sim) |
| 5.2 | aperture reads, local path (READ order-FIFO entry, held-open AXI-Lite read, 64B line pull + lane select, poison on invalid) + TBs + Python sim test | done, all pass |
| 5.2b | rdma wire-message format: 64B header ⟨op·len·vaddr⟩ (+inline data for stores), RETH = staging vaddr (CSR 14), loom_rx parses + issues exact writes | done, all pass |
| 5.2c | hybrid wire scheme: bulk reverts to DIRECT RDMA WRITE (RETH = true target, zero overhead; op·len·vaddr = RDMA's own headers); inline message kept only for sub-64B stores; loom_rx dispatches on staging-vaddr compare | done, all pass |
| 5.3 | loomd control daemon: socket<->OrchClient adapter, SockOrchClient, standalone loomd binary; FPGA-free protocol test + full flow over a real socket in sim | done, all pass |
| 5.3b | stage cycle counters (T3 enabler, owner move-up 2026-08-03): free-running cycle counter + order-FIFO residency accumulator (t-queue) + per-stage cycle accumulators/op counts in the engine (lookup, store-local = t-forward, store-rdma = t-encap, dma-local/rdma, read, fence), RO CSR words 48-63; sw readout (`StageStats`/`stage_avg`) | done, all sims pass |
| 5.4 | hardware gate tests G1/G2/G4 on stock examples | pending |
| 5.5 | synthesize + run on U280 (cross-pid); measure the sim's FPGA-owned constants: T3 per-stage latencies (stage counters in RTL since 5.3b - read deltas, divide by op counts, scale by clock), T2 coalescing curve (needs coalescer RTL, on/off), substrate floors, B2 rdma-init, local read RTT | pending |
| 5.4a | deployment binaries: app_export/app_import (per side: loomd + 2 app processes; single-host bring-up = same binaries, one loomd) | pending (AFTER 6.2a, owner reorder) |
| 6.1 | loomd-loomd TCP, QP setup via Coyote RDMA API (QP owned by the exporter's cThread; staging = a small getMem buffer of the QP owner), remote export/import; move staging from the global CSR into the window table if hosts have multiple QP owners | partial: peering protocol + QP setup + staging exchange shipped with 6.2a (`sw-bundled/`); per-binding QPs + per-window staging pending |
| 6.2a | two-host BUNDLED configuration: one process per host (daemon thread + that side's app roles), loomd-loomd TCP + QP setup + real wire - the cross-host bring-up vehicle (2 processes total) | code DONE (`sw-bundled/`: loom_peer/loom_bundle/loom_host), FPGA-free peering test 17x PASS; EXECUTION pending two-host testbed |
| 6.2b | two-host FULL deployment topology (per side: loomd + 2 app processes); remote measurements incl. remote reads via shell RDMA READ (T6 remote RTT) | pending |

## Running

### Environment (NixOS testbed)

- Vivado tools (vivado, xvlog, xelab, xsim, xsc) are only on PATH inside
  `xilinx-shell` (non-interactive: `xilinx-shell -c "cmd"`). Default is
  Vivado 2023.2; others under `/share/xilinx/Vivado/`.
- `cmake` inside xilinx-shell is Vivado's ancient 3.3.2; use nix instead:
  `nix-shell -p cmake --run "..."` (nested inside xilinx-shell when the
  CMake run needs to find Vivado).
- Coyote's `sim/README.md` flags Vivado 2023.2 as broken for their full
  `tb_user` testbench (mailbox regression); in practice the Phase 4
  integration sim ran fine on 2023.2 here. (No newer version is currently
  installed under `/share/xilinx/Vivado/`.)

### One-time simulation setup (script)

```bash
examples/loom/hw/setup_sim.sh
```

The script configures the hw build, renders the generated RTL
(`build_sim/sim/lynx_pkg.sv` + wrapper), builds the DPI library, and
creates the XSIM project. Two host quirks it works around (details in
the script header):

- Vivado's bundled cmake 3.3 is too old for Coyote -> cmake comes from
  `nix-shell`, Vivado from `xilinx-shell`.
- **Coyote's `make sim` DPI step (xsc) fails on this host**: Vivado
  2023.2's bundled `ld` (binutils 2.37) cannot link against NixOS glibc
  2.42 - the signature is `unknown type [0x13] section \`.relr.dyn'` /
  `skipping incompatible ... libmvec.so.1`, ending in
  `Linking failed for "coyote_sim.so"`. The script builds
  `sim/coyote_sim.so` with the system gcc instead; XSIM only dlopens the
  library at runtime, so this is equivalent.

### Block-level testbenches (Phase 1+)

```bash
examples/loom/hw/tb/run_tbs.sh
```

The script re-execs itself inside `xilinx-shell` if needed, compiles
`lynx_pkg.sv` + `hw/hdl/pkg/axi_intf.sv` + the loom modules, then runs each
`tb_*` in XSIM. Expected output: `PASS: <tb>` per testbench (currently
`tb_loom_table`, `tb_loom_ctrl`, `tb_loom_engine`, `tb_loom_rx`,
`tb_loom_top`). Logs land in `hw/tb/work/`.

Coverage (hardened in Phase 4.5):
- `tb_loom_ctrl`: CSR readback (all RW regs), commit pulse, aperture
  capture fields incl. sub-word wstrb, descriptor enqueue, arrival
  ordering, overflow drops, FIFO wraparound (rolling 5-in/5-out across
  the 64-entry boundary), counters, stage-counter plumbing (cycle
  counter advances, queue-wait accumulator vs. a known FIFO residency,
  words 50-63 read mux).
- `tb_loom_engine` (composite ctrl+table+engine, shell mocked):
  local/rdma stores, DMA local/rdma with completion values,
  descriptor-then-flag ordering, backpressure matrix (wr_ready, host and
  net tready, mid-stream), bounds edges (end==lim vs end==lim+8, store at
  window end), non-64B-multiple lengths, completion-disabled path, a
  60-op soak checked against exact counter deltas, and stage cycle
  counters (exact 2-cycle unstalled stores, stall attribution,
  acc==2*pops lookup invariant, count relations vs. debug counters).
- `tb_loom_rx`: grant gating, forwarding, backpressure, back-to-back.
- `tb_loom_top`: the generated `design_user_logic_c0_0` wrapper as DUT
  (vfpga_top.svh verbatim) - engine/rx arbitration: continuous mutual-
  exclusion assertions, races in both directions, starvation recovery
  after a store burst, and a mixed 40-op soak (stores/descs/rx) with
  exact wr_req/beat/counter accounting.

### Coyote integration sim (Phase 4)

Prerequisite: `setup_sim.sh` above (project + DPI lib in place).

Software build and run (the run spawns Vivado/XSIM, so it goes through
xilinx-shell; use tmux, it takes minutes):

```bash
cd examples/loom/sw && mkdir -p build_sim && cd build_sim
nix-shell -p cmake gcc boost --run 'cmake .. -DEN_SIM=ON && make -j8'
tmux new-session -d -s loom_run \
  "xilinx-shell -c 'export COYOTE_SIM_DIR=$PWD/../../hw/build_sim/; ./test' \
   > run_test.log 2>&1"
tail -f run_test.log     # expect: 19x PASS (2 windows, interleaved stores/
                         # DMAs, cross-window ordering, counter relations,
                         # stage-cycle-counter relations), then LOOM TEST PASS
# waveform: hw/build_sim/sim/sim_dump.vcd
```

Sim-backend notes (learned the hard way):
- `setCSR/getCSR` take 64-bit **word indices** in both backends (the sim
  generator multiplies by 8 onto the AXI address); `loom.hpp` handles it.
- The TB's `EN_RANDOMIZATION` pads every ctrl write with random writes up
  to the next 64 B boundary. Consequences: `dbg[stores]` counts padding
  stores landing in the aperture (random data written at nearby offsets in
  valid windows - keep test offsets clear of each other), and CSR staging
  sequences must end with the meaningful write (ours do: COMMIT/TRIGGER
  last). Counters read immediately after a poll may still be draining.
- Poll destination/completion *memory*, never CSRs, while a DMA is in
  flight (sim CSR reads block behind the generator).

### Python RDMA unit test (Phase 4.5)

The Coyote Python sim framework (`sim/unit_test`) drives the same XSIM
project non-interactively and provides barebones RDMA mocks. Our test
verifies the TX rdma path end-to-end into the TB's RDMA-REMOTE mock,
asserted via the loom debug counters (exact counts: randomization is
disabled through the framework's own knob):

```bash
cd examples/loom/hw/unit-tests
tmux new-session -d -s loom_py \
  "xilinx-shell -c 'nix-shell -p python3 --run \
   \"PYTHONPATH=../build_sim python3 -m unittest test_loom_rdma -v\"' \
   > pytest_run.log 2>&1"
tail -f pytest_run.log     # expect: test_tx_store_takes_rdma_path ... ok
                           #         OK (skipped=1)
```

The local aperture-read test runs the same way (or together:
`python3 -m unittest test_loom_rdma test_loom_read -v`):
`test_loom_read.py` writes a patterned buffer into the sim's memory mock,
programs a window, and checks that window loads return the right qwords
(aligned 64 B line pull + lane select) and that an unprogrammed window
returns POISON. Aperture reads cannot run in the C++ interactive sim
(the blocking ctrl read parks the generator that would have to service
the engine's pull - the documented interactive-mode deadlock).

The RX test is skipped by design: the stock TB delivers only the rq_wr
request of an incoming RDMA write and discards the payload
(memory_simulation.svh, rdmaLocalWrite), so a forwarder waiting on
axis_rrsp_recv cannot complete in simulation. RX data-path coverage:
tb_loom_rx (block level) + hardware gate G3 (Phase 6).

Framework quirks: register values are packed as signed 64-bit (keep test
payloads below 2^63); the RDMA-REMOTE segment must be allocated first
(remote_rdma_write); live register reads need a long simulation window
(the test sets 1 ms; the 4 us default closes before responses arrive).

### Simulation harnesses (NOT the deployment shape - see HW-DESIGN
### "Deployment topology"; sim allows one cThread per process, so true
### process splits cannot run here)

#### Single-binary harnesses

Two binaries, no sockets, no daemon - everything in one process:

- `./test` - the monolithic regression (one cThread doing everything;
  run instructions in the integration-sim section above; 19x PASS,
  `LOOM TEST PASS`).
- `./roles` - the role-split demo: `loom_orch.hpp` (OrchClient
  interface + InProcOrchestrator, the only code touching the CSR page),
  `loom_xpu.hpp` (client role), shared flow in `roles_test.hpp`.
  Transport = in-process function calls behind the OrchClient
  interface. Mode is runtime-selected: with COYOTE_SIM_DIR set, all
  roles share the single sim cThread (degenerate); on hardware each
  role gets its own cThread (real cross-pid).

```bash
cd examples/loom/sw/build_sim   # after the cmake/make above
tmux new-session -d -s loom_roles \
  "xilinx-shell -c 'export COYOTE_SIM_DIR=$PWD/../../hw/build_sim/; ./roles' \
   > run_roles.log 2>&1"
tail -f run_roles.log   # expect 12x PASS, LOOM ROLES TEST PASS
# hardware: same binary, no COYOTE_SIM_DIR -> cThread per role
```

Covers: export/import through the orchestrator (windows allocated
server-side), bogus-handle refusal, stores + fenced copies + the order
point through the role API, and window release (subsequent stores dropped,
observed via the drops counter).

### Running on hardware (Phase 5.4+ flow - NOT yet validated)

The expected flow once testbed time is available; commands follow the
standard Coyote flow and this repo's helper scripts:

```bash
# 1. Bitstream (hours; tmux). Same build dir layout as sim, own dir:
cd examples/loom/hw && mkdir -p build_hw && cd build_hw
xilinx-shell -c "nix-shell -p cmake --run 'cmake .. -DFDEV_NAME=u280'"
tmux new-session -d -s loom_bit \
  "xilinx-shell -c 'nix-shell -p cmake --run \"make project && make bitgen\"' \
   > bitgen.log 2>&1"
# -> bitstreams/cyt_top.bit

# 2. Program the FPGA + load the driver (repo root; needs sudo; the
#    script hot-resets the PCIe device and insmods with per-host ip/mac):
cd ../../../..   # Coyote root
xilinx-shell -c "vivado -mode batch -source program_fpga.tcl ..."  # or hw_server flow
sudo bash setup_coyote.sh

# 3. Software, hardware mode (no EN_SIM, no COYOTE_SIM_DIR):
cd examples/loom/sw && mkdir -p build_hw && cd build_hw
nix-shell -p cmake gcc boost --run 'cmake .. && make -j8'
./test    # single-process regression (same checks as sim)
./roles   # role-split demo: on hardware each role gets its OWN cThread,
          # so imports translate under the exporter's distinct ctid (G1)
```

Before first hardware run, do the Phase 5.2 gate tests (G1/G2/G4, see
"Gates to verify first" above) on stock examples.

#### Socket harness (loomd on a thread, real Unix socket)

`loomd.hpp` is a Unix-socket front end over any OrchClient backend
(production: InProcOrchestrator through the daemon's cThread; tests: a
fake). `loom_sock.hpp` implements OrchClient over the socket - roles and
data-plane code cannot tell the transports apart (`roles_test.hpp` holds
the shared flow). `loomd_main.cpp` is the standalone daemon for
hardware; in simulation a separate daemon process would spawn its own
simulator, so `roles_sock` runs the daemon as a thread over a REAL
socket instead (protocol/daemon are the production code paths; only
process isolation is degenerate).

FPGA-free protocol test (no cThread constructed - runs on any machine,
no COYOTE_SIM_DIR, no Vivado):

```bash
cd examples/loom/sw/build_sim
./test_loomd_proto              # expect 13x PASS, LOOMD PROTO TEST PASS
```

Full client/server flow in simulation (daemon on a thread serving a
real socket; tmux + xilinx-shell like every sim run):

```bash
cd examples/loom/sw/build_sim
tmux new-session -d -s loom_sock \
  "xilinx-shell -c 'export COYOTE_SIM_DIR=$PWD/../../hw/build_sim/; ./roles_sock' \
   > run_roles_sock.log 2>&1"
tail -f run_roles_sock.log
# expect: "two clients connected", 12x PASS, LOOM ROLES-SOCK TEST PASS
```

### 6.2a: the bundled two-host binary (sw-bundled/)

`sw-bundled/` is the cross-host bring-up vehicle: ONE process per host
carrying the daemon role (BundledOrchestrator backend + local loomd on
a Unix socket + loomd<->loomd TCP peering) and that side's app roles as
threads - two processes total across the cluster. New pieces:

- `loom_peer.hpp` - the 6.1 control plane: fixed-size TCP protocol
  between the two daemons. Hello at accept carries the exporter side's
  RDMA staging VA (the importer programs it into RDMA_STAGING_VA);
  RESOLVE turns a handle into {exporter ctid, VA, len} cross-host;
  DONE is the end-of-run barrier.
- `loom_bundle.hpp` - `BundledOrchestrator`: OrchClient whose imports
  go remote when a peer is attached (window programmed route=rdma,
  pid = the LOCAL QP-owner ctid, base = the exporter's VA), and whose
  export registry doubles as the peering resolver on the server side.
- `loom_host.cpp` - the binary. QP setup rides Coyote's
  `cThread::initRDMA` (server side first, then client with the server
  IP; ONE RC connection between the two data cThreads carries both
  windows for bring-up - per-binding QPs are 6.1/6.2b). Client flow =
  the roles flow across hosts: remote imports, inline-message stores,
  direct bulk with fence, the cross-host order point (flag after copy
  on the same QP; RC in-order delivery extends the FIFO guarantee),
  release + source-side drop. Server verifies the landings.

Build (compiles everywhere; EXECUTION needs the two-host testbed - the
sim has no networking, the binary refuses under COYOTE_SIM_DIR):

```bash
cd examples/loom/sw-bundled && mkdir -p build && cd build
nix-shell -p cmake gcc boost --run 'cmake .. && make -j8'
```

FPGA-free peering protocol test (no cThread, runs on any machine):

```bash
./test_peering    # expect 17x PASS, LOOM PEERING TEST PASS
```

Two-host run (hardware; server side first):

```bash
# host 2 (exporter side):
./loom_host --server                 # waits in the QP exchange
# host 1 (importer side):
./loom_host --client <host2_ip>
# expect: LOOM HOST CLIENT PASS (7x) / LOOM HOST SERVER PASS (6x)
# options: --qp-port (default 18488), --peer-port (default 18489),
#          --sock <local loomd Unix socket path>
```

Known scope limits (deliberate, documented in loom_host.cpp): one RC
connection, both exported segments owned by the server's data cThread,
and the global staging CSR serves the asymmetric topology only (client
TX / server RX; symmetric traffic needs per-window staging, 6.1).

### Deployment (hardware, Phase 5.4a+ - binaries to be written)

The real topology (HW-DESIGN "Deployment topology"): per side, one
`loomd` + two app processes; two hosts for the remote route. Planned
binaries: `app_export` / `app_import` (two instances each, connecting
via `LOOMD_SOCK`), with the single-host bring-up configuration running
the same binaries under one loomd (local route). Until they exist, the
hardware-runnable multi-process check is the stopgap:
`./loomd /tmp/loomd.sock` in one terminal and
`LOOMD_SOCK=/tmp/loomd.sock ./roles_sock` in another (client roles
still bundled in one process); the cross-host configuration is the
6.2a bundled binary above.

### Switching between the C++ and Python sim harnesses

The two harnesses compile the XSIM project with different defines
(interactive vs. non-interactive). The stale snapshot makes the *other*
harness crash at t=0 (XSIM kernel FATAL). After switching, clean once:

```bash
rm -rf examples/loom/hw/build_sim/sim/example_loom.sim
```

### After editing `hw/src/vfpga_top.svh`

The setup step copies `vfpga_top.svh` into `hw/build_sim/sim/`, and the
generated `user_logic_c0_0.sv` wrapper resolves its `include` against
that copy - the integration sims silently build the stale version
(symptom: modules referenced in place are current, but top-level wiring
changes never take effect). After editing, refresh the copy and clean
the snapshot:

```bash
cp examples/loom/hw/src/vfpga_top.svh examples/loom/hw/build_sim/sim/
rm -rf examples/loom/hw/build_sim/sim/example_loom.sim
```
