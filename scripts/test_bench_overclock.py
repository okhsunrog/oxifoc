import unittest
from unittest.mock import patch

import bench_overclock as bench


class ClockReadbackTests(unittest.TestCase):
    def registers(self, n=169, overclock=True):
        # F405 HSE/4 * PLLN / 2, AHB /1. Independent register fixtures.
        cfgr = 0xb80a if overclock else 0x940a
        return {
            0x40023800: [0x03037e83, (7 << 24) | (1 << 22) | (n << 6) | 4, cfgr],
            0x40023c00: [5 if n == 169 else 7],
            0x40012304: [0 if overclock else 1 << 16],
            0x40013000: [1 << 3],  # SPI /4
            0x40010028: [0, 2097 if overclock else 4195],
            0x40010020: [0],
        }

    def decode(self, regs):
        with patch.object(bench, "read", side_effect=lambda address, words=1: regs[address]):
            return bench.clocks()

    def test_normal_profile(self):
        x = self.decode(self.registers(overclock=False))
        self.assertEqual(x["cpu_hz"], 167835864)
        self.assertEqual(x["tim1_hz"], 167835864)
        self.assertEqual(x["adc_hz"], 20979483)

    def test_reduced_bus_dividers(self):
        x = self.decode(self.registers())
        self.assertEqual(x["tim1_hz"], 83917932)
        self.assertEqual(x["adc_hz"], 20979483)

    def test_upper_step(self):
        regs = self.registers(n=241)
        regs[0x40010028] = [0, 2991]
        x = self.decode(regs)
        self.assertEqual(x["cpu_hz"], 239339901)
        self.assertEqual(x["flash_ws"], 7)

    def test_maximum_experimental_step(self):
        regs = self.registers(n=261)
        regs[0x40010028] = [0, 3239]
        x = self.decode(regs)
        self.assertEqual(x["cpu_hz"], 259202134)
        self.assertEqual(x["flash_ws"], 7)
        self.assertGreater(x["flash_access_hz"], 30_000_000)

    def test_rejects_incomplete_capture(self):
        meta = dict(gaps=0, samples_lost=0, rows=10, raw_telemetry=dict(
            ended=True, error=None, host_drops=0, producer_drops=0,
            records_received=10, records_delivered=10, end_total_records=11))
        with self.assertRaises(AssertionError):
            bench.validate_capture(meta)

    def test_rejects_output_enable(self):
        regs = self.registers()
        regs[0x40010020] = [1 << 6]  # CH2 complementary enable
        with self.assertRaisesRegex(AssertionError, "phase output"):
            self.decode(regs)

    def test_rejects_wrong_flash_latency(self):
        regs = self.registers()
        regs[0x40023c00] = [0]
        with self.assertRaisesRegex(AssertionError, "FLASH wait"):
            self.decode(regs)

    def test_rejects_overclocked_apb(self):
        with self.assertRaises(AssertionError):
            self.decode(self.registers(n=241, overclock=False))


if __name__ == "__main__":
    unittest.main()
