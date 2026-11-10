import unittest
from unittest.mock import patch
import f405_timing as timing


class PeripheralContractTests(unittest.TestCase):
    def fixture(self):
        tim = [0xf5,0,0,0,0x1f,0,0x6868,0x6868,0x1000,0,0,4195,1,0,0,0,4112,0x803c]
        def adc(cr1, cr2, smpr1, smpr2, jsqr):
            x = [0]*15
            x[1:5] = [cr1,cr2,smpr1,smpr2];x[14]=jsqr
            return x
        return {0x40010000:tim,0x40012300:[0,1<<16],
            0x40012000:adc(0x100,0x100601,0xe00001,0x201,0x200d40),
            0x40012100:adc(0x100,0x100001,0x1008,0x8,0x20b960),
            0x40012200:adc(0x180,0x100001,0x240,0x40,0x213580)}

    def decode(self, registers):
        clocks = dict(tim1_hz=167835842, adc_hz=20979480)
        with patch.object(timing.probe, 'clocks', return_value=clocks), patch.object(
            timing.probe, 'read', side_effect=lambda addr, words=1: registers[addr]):
            return timing.snapshot()

    def test_stock_contract(self):
        x = self.decode(self.fixture())
        self.assertAlmostEqual(x['adcs'][0]['sequence_conversion_us'],3.8609155)
        self.assertEqual(x['adcs'][2]['channels'], [12,13,2])

    def test_rejects_changed_injected_rank(self):
        r = self.fixture();r[0x40012100][14] ^= 1<<5
        with self.assertRaisesRegex(AssertionError,'ranks'):
            self.decode(r)

    def test_rejects_changed_sample_time(self):
        r = self.fixture();r[0x40012000][3] ^= 3
        with self.assertRaisesRegex(AssertionError,'sample-time'):
            self.decode(r)

    def test_rejects_changed_pwm_period(self):
        r = self.fixture();r[0x40010000][11] += 1
        with self.assertRaisesRegex(AssertionError,'20 kHz'):
            self.decode(r)


if __name__ == '__main__':
    unittest.main()
