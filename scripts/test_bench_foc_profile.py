import unittest
import struct
import bench_foc_profile as bench


class DistributionTests(unittest.TestCase):
    def test_pcs_filters_drive_state_and_resolves_thumb_addresses(self):
        data = b''.join(struct.pack('<II', pc, flag) for pc, flag in [(0x101, 1), (0x200, 0), (0x201, 1)])
        result = bench.decode_pcs(data, '00000100 00000020 T hot_function\n')
        self.assertEqual(result['total_samples'], 3)
        self.assertEqual(result['driven_samples'], 2)
        self.assertEqual({row['name']: row['samples'] for row in result['functions']},
                         {'hot_function': 1, '<unmapped>': 1})
        with self.assertRaises(AssertionError):
            bench.decode_pcs(data[:-1], '')

    def fixture(self):
        h = [1, 168000000, 64, 2000, 1999, 100, 8400, 0, 0, 10, 2009, 12, 200000, 0, 1, 8400]
        body, gap = [0]*256, [0]*256
        body[1], gap[131] = 2000, 1999
        return h+body+gap

    def test_pwm_deadline_preserves_negative_signed_margin(self):
        words = [1, 2000, 2**32-168, 8400, 840, 1008, 3, 0, 123, 0, 0]
        d = bench.decode_deadline(words, 168000000)
        self.assertEqual(d['min_margin_us'], -1)
        self.assertEqual(d['max_entry_to_commit_us'], 50)
        self.assertEqual(d['entry_phase_us'], [5, 6])
        self.assertEqual(d['late_writes'], 3)
        self.assertEqual(d['worst_seq'], 123)
        with self.assertRaises(AssertionError):
            bench.decode_deadline([1, 0]+words[2:], 168000000)

    def test_minimal_deadline_does_not_invent_unmeasured_fields(self):
        d = bench.decode_deadline([3, 2000, 1680, 0, 0, 0, 0, 0, 145, 0, 0], 168000000)
        self.assertEqual(d['min_margin_us'], 10)
        self.assertIsNone(d['max_entry_to_commit_us'])
        self.assertIsNone(d['entry_phase_us'])
        self.assertIsNone(d['worst_seq'])
        self.assertIsNone(d['outside_adc'])
        self.assertEqual(d['worst_write_ordinal'], 145)

    def test_irq_cadence_uses_host_ack_bounds(self):
        events = [dict(i=0, cmd={"velocity": {}}, t_sent=.3, t_acked=.31),
                  dict(i=1, cmd={"stop": {}}, t_sent=5.1, t_acked=5.11)]
        lo, hi = bench.host_cadence_bounds(events, 96000)
        self.assertLess(lo, 20000)
        self.assertGreater(hi, 20000)
        self.assertLess(hi, 20100)

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

    def work_fixture(self):
        words = self.fixture()
        words[5], words[7], words[12], words[15] = 101, 1, 200001, 100
        distribution = bench.decode(words)
        header = [1, 1, 256, 5, 3, 1024, 1, 0]
        classes, records = [0] * 1280, [0] * 3072
        classes[5:10] = [2000, 101, 1, 200001, 0]
        records[:3] = [12, 101, 1 | (1 << 9)]
        return header, classes, records, distribution

    def test_work_records_preserve_full_mask_and_class_totals(self):
        result = bench.decode_work(*self.work_fixture())
        self.assertTrue(result['complete_attribution'])
        self.assertEqual(result['combinations'][0]['events'], ['velocity_pi', 'hall_velocity_wrap'])
        self.assertEqual(result['classes'][0]['n'], 2000)

    def test_work_class_bits_map_indexes_to_real_event_masks(self):
        h, classes, records, d = self.work_fixture()
        # Class index 1 | 1 << 4 with the wrap map is PI plus authority-ahead.
        classes[5:10] = [0] * 5
        classes[17 * 5:18 * 5] = [2000, 101, 1, 200001, 0]
        records[:3] = [12, 101, 1 | (1 << 14) | (1 << 12)]
        result = bench.decode_work(h, classes, records, d, [0, 1, 2, 3, 14, 15, 8, 11])
        self.assertEqual(result['classes'][0]['mask'], 1 | (1 << 14))
        self.assertEqual(result['classes'][0]['events'], ['velocity_pi', 'authority_ahead'])
        self.assertEqual(result['combinations'][0]['events'], ['velocity_pi', 'angle_wrap', 'authority_ahead'])
        with self.assertRaises(AssertionError):
            bench.decode_work(h, classes, records, d, [0, 1, 2, 3, 4, 5, 6, 6])

    def test_work_rejects_inconsistent_count_or_sum(self):
        h, classes, records, d = self.work_fixture()
        classes[5] -= 1
        with self.assertRaises(AssertionError):
            bench.decode_work(h, classes, records, d)
        h, classes, records, d = self.work_fixture()
        classes[8] -= 1
        with self.assertRaises(AssertionError):
            bench.decode_work(h, classes, records, d)

    def test_work_overflow_keeps_partial_attribution_explicit(self):
        h, classes, records, d = self.work_fixture()
        h[7], classes[7], d['over_budget'] = 1, 2, 2
        result = bench.decode_work(h, classes, records, d)
        self.assertFalse(result['complete_attribution'])
        self.assertEqual(result['dropped'], 1)

    def test_disabled_work_tags_require_zero_masks(self):
        h, classes, records, d = self.work_fixture()
        h[1] = 0
        with self.assertRaises(AssertionError):
            bench.decode_work(h, classes, records, d)

    def test_instrumentation_level_matches_the_actual_elf(self):
        base = {'FOC_DISTRIBUTION_CONTROL': 1}
        bench.validate_image_mode('minimal', base)
        with self.assertRaises(AssertionError):
            bench.validate_image_mode('work', base)
        with self.assertRaises(AssertionError):
            bench.validate_image_mode('minimal', {**base, 'FOC_WORK_HEADER': 2})

    def test_report_excludes_failed_attempts(self):
        d = bench.decode(self.fixture())
        result = bench.summarize(dict(elf_sha256='abc', detail='minimal', runs=[
            dict(name='good', case='test', accepted=True, distribution=d),
            dict(name='bad', case='test', accepted=False)]))
        self.assertEqual(result['cases']['test']['n'], 2000)
        self.assertEqual(result['rejected'], ['bad'])

    def test_section_outliers_deduplicate_nested_records(self):
        width = 10 + len(bench.SECTION_NAMES)
        record = [0] * width
        record[:4] = [12, 101, 100, 0]
        record[10 + bench.SECTION_NAMES.index('velocity_request')] = 17
        words = record * 2 + [0] * (width * 14)
        summary = [2000, 200001, 101, 1, 0, 0, 0, 0, 10, 2009, 1, 0]
        result = bench.decode_outliers(summary, words, 168000000)
        self.assertEqual(len(result['records']), 1)
        self.assertEqual(result['records'][0]['nested_section_cycles']['velocity_request'], 17)
        self.assertTrue(result['sum_u32_proven_safe'])
        summary[11] = 1
        with self.assertRaises(AssertionError):
            bench.decode_outliers(summary, words, 168000000)


if __name__ == '__main__':
    unittest.main()
