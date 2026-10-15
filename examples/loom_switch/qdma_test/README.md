# QDMA test image (U280, 10ee:9f3f)

Checks every QDMA path the U280 soft-QDMA static (DESIGN.md section 14) uses, on a small image that builds in
about 1.5 h, before the 8 h static build. The QDMA IP has the static's configuration (cr_pci_qdma.tcl; only the
device ID 9F3F differs, so no Coyote driver binds it). Its C2H/H2C logic is the static's own: qdma_wr_wrapper,
qdma_rd_wrapper, qdma_reg_slice and qdma_ecc, unchanged. A shell stand-in drives them. BAR4 holds an always-ready
sink for the bridge path, plus a control and status window at 0x0FF0_0000.

- `hw/src`: `qdma_hw2_top` (QDMA IP + core), `qdma_hw2_core` (wrappers wired as in static_top_qdma, counters at
  the IP's ports), `qdma_gen` (shell stand-in: C2H requests + pattern data, H2C reads + byte check), `bar4_ctl`
  (bridge sink, control registers, status words).
- `hw/coyote`: copies of the Coyote sources used (7cb6023d), plus `lynx_pkg.sv` from the build_oct13_qdma
  loom_switch build (`SOURCE.txt`).
- `hw/scripts`, run from `hw/`:
  - `create_rev2.tcl`: creates the project and IPs.
  - `build_rev2.sh`: builds the bitstream; run it in tmux.
  - `sim_rev2.tcl`: unit TB against a behavioural QDMA model. It needs `qdma_ecc_enc.sv` from AMD's QDMA example
    design; its path is set in the script. The xsim QDMA model itself hangs at t=0, so the TB doesn't use it.
- `hw/tb/simfix/qdma_rd_wrapper.sv`: simulation-only copy with the ready assignment moved ahead of its read. In
  xsim's `always_comb` ordering the unchanged file issues extra H2C commands; on hardware it doesn't.
- `sw/qdma_c2h_test.c`: kernel module, `make KDIR=<kernel build>`. It sets up the queues as
  `driver/src/platform/pci_qdma.c` does: 64 H2C queues, 32 C2H queues, all 32 prefetch tags must be granted. Then
  it runs the C2H singles on all 24 wrapper queues (tag checked), C2H/H2C back-to-back runs with byte checks, and
  the throughput runs. Every wait is bounded. Results go to dmesg as `RESULT`/`RATE` lines.
- `sw/run_qdma_rev2_amy.sh`: `prog | test | unload | bridge | restore` on amy; expects the bitstream and module
  under `~/loom-experiments/qdma_hw/`.

Measured 2026-10-14 on amy (e1/20261014_qdma_rev2, two runs): all tests pass.

| Path | Result |
|---|---|
| C2H 4 KiB | 14.8 GB/s |
| C2H 256 B | 7.3 GB/s |
| C2H 64 B | 2.2 GB/s (about 7.4 cycles per descriptor) |
| H2C 4 KiB | 14.2 GB/s |
| C2H and H2C at once | 13.0 + 13.0 GB/s |
| Bridge (PTDMA, 8 channels) | 12.23 GB/s, 5.19 cycles per 256 B burst |
