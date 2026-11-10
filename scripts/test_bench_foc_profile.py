import unittest
import bench_foc_profile as bench


class DistributionTests(unittest.TestCase):
    def fixture(self):
        h = [1, 168000000, 64, 2000, 1999, 100, 8400, 0, 0, 10, 2009, 12, 200000, 0, 1, 8400]
        body, gap = [0]*256, [0]*256
        body[1], gap[131] = 2000, 1999
        return h+body+gap

    def test_recording_loss_is_not_an_ack_failure(self):
        error = 'operational errors: {"stream_stop": String("raw capture failed: raw capture lost samples: producer=26, host=0")}'
        self.assertTrue(bench.diagnostic_loss_only(error))
        self.assertFalse(bench.diagnostic_loss_only(error[:-1] + ', "terminal": String("missed ACK")}'))
        self.assertFalse(bench.diagnostic_loss_only('operational errors: {"execution": String("missed ACK")}'))

    def test_exact_mean_and_bounded_percentiles(self):
        d = bench.decode(self.fixture())
        self.assertAlmostEqual(d['mean_us'], 100/168)
        self.assertLessEqual(d['body']['p999']['lower_us'], d['mean_us'])
        self.assertGreater(d['body']['p999']['upper_us'], d['mean_us'])

    def test_epoch_change_rejected(self):
        w = self.fixture(); w[8] = 1
        with self.assertRaisesRegex(AssertionError, 'epoch'):
            bench.decode(w)

    def test_overflow_bin_has_no_false_upper_bound(self):
        w = self.fixture(); w[17] = 0; w[16+255] = 2000
        self.assertIsNone(bench.decode(w)['body']['p999']['upper_us'])


if __name__ == '__main__':
    unittest.main()
