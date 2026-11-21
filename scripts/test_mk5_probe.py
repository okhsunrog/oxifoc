import unittest
from unittest.mock import patch

import mk5_probe as bench


class ClockReadbackTests(unittest.TestCase):
    def registers(self, n=169):
        # F405 HSE/4 * PLLN / 2, AHB /1, APB1 /4, APB2 /2. Independent fixtures.
        return {
            0x40023800: [0x03037e83, (7 << 24) | (1 << 22) | (n << 6) | 4, 0x940a],
            0x40023c00: [5],
            0x40012304: [1 << 16],
            0x40013000: [1 << 3],  # SPI /4
            0x40010028: [0, 4195],
            0x40010020: [0],
        }

    def decode(self, regs):
        with patch.object(bench, "read", side_effect=lambda address, words=1: regs[address]):
            return bench.clocks()

    def test_normal_profile(self):
        x = self.decode(self.registers())
        self.assertEqual(x["cpu_hz"], 167835864)
        self.assertEqual(x["tim1_hz"], 167835864)
        self.assertEqual(x["adc_hz"], 20979483)

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

    def test_rejects_apb_above_its_limit(self):
        with self.assertRaises(AssertionError):
            self.decode(self.registers(n=241))


if __name__ == "__main__":
    unittest.main()
