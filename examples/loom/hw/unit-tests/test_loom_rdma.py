"""
Loom RDMA tests on the Coyote Python sim framework (best-effort; the
framework's RDMA support is self-described as barebones).

TX: program an rdma window, issue an aperture store, assert via the loom
debug counters that exactly one write took the RDMA path (payload lands in
the TB's RDMA-REMOTE mock, which has no read-back API - content checks for
the RDMA path live in tb_loom_engine/tb_loom_top and hardware Phase 6).

RX: skipped. The stock TB's local_rdma_write delivers only the rq_wr
request; the payload bytes are discarded (memory_simulation.svh,
rdmaLocalWrite) and axis_rrsp_recv is never driven, so a forwarder that
waits for payload cannot complete in simulation. Covered by tb_loom_rx
(block level) and hardware gate G3.

Run instructions: see ../../README.md ("Python RDMA unit test").
"""

import unittest

import coyote_test  # noqa: F401  (generated in hw/build_sim; sets paths/constants)
from unit_test.fpga_test_case import FPGATestCase
from unit_test.fpga_register import vFPGARegister
from unit_test.simulation_time import FixedSimulationTime


def reg64(idx: int, value: int) -> vFPGARegister:
    return vFPGARegister(idx, bytearray(int(value).to_bytes(8, "little")))


class LoomRdmaTx(FPGATestCase):
    # Exact counter assertions need deterministic input (no random
    # write-padding bursts from the ctrl driver)
    disable_input_timing_randomization = True

    def _poll_register(self, idx, want, stop, tries=30):
        val = None
        for _ in range(tries):
            val = self.read_register(idx, stop)
            if val == want:
                return val
        return val

    def test_tx_store_takes_rdma_path(self):
        # Default sim window (4us) can close before the live register
        # reads below get their responses
        self.overwrite_simulation_time(FixedSimulationTime.from_string("1ms"))
        stop = self.simulate_fpga_non_blocking()

        # Allocate the STAGING segment in the TB's RDMA-REMOTE mock: all
        # wire messages land there (RETH vaddr is data-meaningless; the
        # true target rides the message header)
        self.remote_rdma_write(0x7F9E_8860_0000, bytearray(4096))
        self.write_register(reg64(14, 0x7F9E_8860_0000))   # RDMA_STAGING_VA

        # Window 2 -> rdma route, QP-owner pid 0, base = exporter's VA
        # (header content only - never dereferenced on this side)
        self.write_register(reg64(0, 2))
        self.write_register(reg64(1, 0b11))
        self.write_register(reg64(2, 0))
        self.write_register(reg64(3, 0x7FAA_0000_0000))
        self.write_register(reg64(4, 0x40_0000))
        self.write_register(reg64(5, 1))
        # Aperture store: window 2, offset 0x40 (register ids are 64-bit
        # word indices; byte 0x2040 -> id 0x408)
        self.write_register(reg64(0x2040 // 8, 0x1EAD_BEEF_0000_0002))

        # Counters: exactly one rdma write, nothing local, no drops
        self.assertEqual(self._poll_register(35, 1, stop), 1, "dbg[rdma_wr]")
        self.assertEqual(self.read_register(34, stop), 0, "dbg[local_wr]")
        self.assertEqual(self.read_register(37, stop), 0, "dbg[drops]")

        self.finish_fpga_simulation()

    @unittest.skip(
        "stock TB drops the payload of incoming RDMA writes (rq_wr request "
        "only; axis_rrsp_recv never driven) - RX data path not completable "
        "in simulation; covered by tb_loom_rx and hardware gate G3"
    )
    def test_rx_incoming_write(self):
        pass


if __name__ == "__main__":
    unittest.main()
