"""
Local aperture-read test (Phase 5.2) on the Coyote Python sim framework.

Why here and not the C++ integration sim: an aperture read is a blocking
ctrl read; in the interactive (C++) backend it parks the sim generator,
which then cannot service the engine's line pull - the documented
interactive-mode deadlock. The non-interactive Python backend serves
pulls from the on-sim memory mock, so the round trip completes.

Covers: read returns the destination memory's qword (engine pulls the
aligned 64 B line under the destination pid and lane-selects), and an
unprogrammed window returns POISON (all-ones) instead of hanging.

Run instructions: see ../../README.md ("Python RDMA unit test" section).
"""

import unittest

import coyote_test  # noqa: F401  (generated in hw/build_sim; sets paths/constants)
from unit_test.fpga_test_case import FPGATestCase
from unit_test.fpga_register import vFPGARegister
from unit_test.simulation_time import FixedSimulationTime

POISON = (1 << 64) - 1


def reg64(idx: int, value: int) -> vFPGARegister:
    return vFPGARegister(idx, bytearray(int(value).to_bytes(8, "little")))


class LoomLocalRead(FPGATestCase):
    disable_input_timing_randomization = True

    def _read(self, byte_addr, stop, tries=30):
        val = None
        for _ in range(tries):
            val = self.read_register(byte_addr // 8, stop)
            if val is not None:
                return val
        return val

    def test_local_read_returns_memory_and_poison(self):
        self.overwrite_simulation_time(FixedSimulationTime.from_string("1ms"))
        stop = self.simulate_fpga_non_blocking()

        # Destination buffer in the sim's host-memory mock: 4 KB of
        # distinct qwords (0x1D00...0000 + index)
        data = bytearray()
        for i in range(512):
            data += int(0x1D00_0000_0000_0000 + i).to_bytes(8, "little")
        vaddr = self.get_io_writer().allocate_and_write_to_next_free_sim_memory(data)

        # Window 1 -> local route, pid 0, base = the mock buffer
        self.write_register(reg64(0, 1))
        self.write_register(reg64(1, 0b01))
        self.write_register(reg64(2, 0))
        self.write_register(reg64(3, vaddr))
        self.write_register(reg64(4, 4096))
        self.write_register(reg64(5, 1))

        # Read window 1, offset 0x18 -> qword index 3 of the buffer
        val = self._read(0x1000 + 0x18, stop)
        self.assertEqual(val, 0x1D00_0000_0000_0003, "lane-selected read data")

        # Read window 1, offset 0x40 -> qword index 8 (different line)
        val = self._read(0x1000 + 0x40, stop)
        self.assertEqual(val, 0x1D00_0000_0000_0008, "second line read")

        # Unprogrammed window 5: poison, never a hang
        val = self._read(0x5000, stop)
        self.assertIn(val, (POISON, -1), "poison on invalid window")

        # Stage cycle counters (T3): all three reads answered (poison
        # included), read-stage cycles accumulated (the two real reads
        # include the shell line-pull round trip)
        self.assertEqual(self.read_register(62, stop), 3, "stage read cnt")
        self.assertGreaterEqual(self.read_register(55, stop), 3, "stage read acc")

        self.finish_fpga_simulation()


if __name__ == "__main__":
    unittest.main()
