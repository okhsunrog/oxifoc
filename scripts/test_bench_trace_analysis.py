"""Offline CPU accounting cases; no device or third-party dependencies."""

import unittest

from bench_trace_analysis import Event, TraceError, analyze, decode


def events(*rows):
    return [Event(ticks, identity, kind) for ticks, kind, identity in rows]


class UsbCpuTraceTests(unittest.TestCase):
    def test_sparse_usb_keeps_validated_idle_and_adc_edges(self):
        report = analyze(events((0, 8, 0), (10, 9, 0), (20, 10, 0),
                                (30, 11, 0), (40, 8, 0), (50, 9, 0),
                                (60, 7, 1)), 100, trim=True, extend_idle_edges=True)
        self.assertEqual(report['window_ticks'], 60)
        self.assertEqual(report['adc_union_ticks'], 20)
        self.assertEqual(report['usb_exclusive_ticks'], 10)

    def test_extended_edges_still_discard_unknown_parent_and_open_tail(self):
        report = analyze(events((0, 8, 0), (5, 9, 0), (10, 11, 0),
                                (20, 8, 0), (25, 9, 0), (30, 10, 0),
                                (40, 11, 0), (50, 7, 1), (60, 10, 0),
                                (70, 8, 0), (80, 9, 0)), 100,
                         trim=True, extend_idle_edges=True)
        self.assertEqual(report['used_first_record'], 3)
        self.assertEqual(report['used_last_record'], 7)
        self.assertEqual(report['window_ticks'], 30)
        self.assertEqual(report['adc_union_ticks'], 5)
        self.assertEqual(report['usb_exclusive_ticks'], 10)

    def test_other_irq_preemption_is_not_usb_cpu(self):
        report = analyze(events(
            (0, 10, 0), (10, 19, 0), (15, 8, 0), (25, 9, 0),
            (30, 20, 0), (40, 21, 0), (50, 22, 0),
            (60, 23, 0), (70, 24, 0), (100, 11, 0),
        ), 100)
        self.assertEqual(report['usb_inclusive_ticks'], 100)
        self.assertEqual(report['adc_union_ticks'], 10)
        self.assertEqual(report['usb_excluding_adc_ticks'], 90)
        self.assertEqual(report['other_irq_preempting_usb_ticks'], 30)
        self.assertEqual(report['usb_exclusive_ticks'], 60)

    def test_unrelated_hall_is_not_a_usb_span(self):
        report = analyze(events((0, 10, 0), (10, 11, 0),
                                (20, 19, 0), (30, 20, 0),
                                (40, 10, 0), (50, 11, 0)), 100)
        self.assertEqual(report['usb_exclusive_ticks'], 20)

    def test_nested_usb_union_excludes_adc_once(self):
        report = analyze(events(
            (0, 10, 0), (10, 12, 0), (20, 8, 0), (40, 9, 0),
            (60, 13, 0), (100, 11, 0),
        ), 100, export_intervals=True)
        self.assertEqual(report["usb_inclusive_ticks"], 100)
        self.assertEqual(report["adc_preempting_usb_ticks"], 20)
        self.assertEqual(report["usb_excluding_adc_ticks"], 80)
        self.assertEqual(report["usb_irq_excluding_adc_ticks"], 30)
        self.assertEqual(report["usb_executor_only_excluding_adc_ticks"], 50)
        self.assertEqual(report["coverage_fraction"], 1)
        self.assertEqual(report["intervals"], {
            "usb_union": [[0, 100]], "adc_union": [[20, 40]],
            "usb_adc_overlap": [[20, 40]], "usb_excluding_adc": [[0, 20], [40, 100]],
        })

    def test_disjoint_frames_do_not_subtract_unrelated_adc(self):
        report = analyze(events(
            (0, 10, 0), (10, 11, 0), (20, 12, 0), (30, 13, 0),
            (40, 8, 0), (50, 9, 0), (60, 10, 0), (80, 11, 0),
        ), 100)
        self.assertEqual(report["window_ticks"], 80)
        self.assertEqual(report["usb_excluding_adc_ticks"], 40)
        self.assertEqual(report["adc_preempting_usb_ticks"], 0)

    def test_timer_wrap_with_adc_preemption(self):
        report = analyze(events(
            (0xFFFFFFF0, 10, 0), (0xFFFFFFF8, 8, 0), (4, 9, 0), (20, 11, 0),
        ), 100, export_intervals=True)
        self.assertEqual(report["window_ticks"], 36)
        self.assertEqual(report["adc_preempting_usb_ticks"], 12)
        self.assertEqual(report["usb_excluding_adc_ticks"], 24)
        self.assertEqual(report["intervals"]["usb_union"], [[0, 36]])
        self.assertEqual(report["intervals"]["adc_union"], [[8, 20]])
        self.assertEqual(report["qualified_window"]["end_timer_raw"], 20)

    def test_exported_little_endian_record_layout(self):
        self.assertEqual(decode(bytes.fromhex("78563412 efbeadde 0a000000")),
                         [Event(0x12345678, 0xDEADBEEF, 10)])

    def test_pwm_metadata_preserves_accounting_and_timestamp_validation(self):
        report = analyze(events((0, 10, 0), (4, 8, 0), (6, 14, 0x81230234),
                                (8, 9, 0), (12, 11, 0)), 100, export_intervals=True)
        self.assertEqual(report["usb_excluding_adc_ticks"], 8)
        self.assertEqual(report["intervals"]["usb_excluding_adc"], [[0, 4], [8, 12]])
        for trim in (False, True):
            with self.subTest(trim=trim), self.assertRaisesRegex(TraceError, "reordered"):
                analyze(events((0, 10, 0), (4, 8, 0), (3, 14, 0x81230234),
                               (8, 9, 0), (12, 11, 0)), 100, trim=trim)
            with self.subTest(trim=trim), self.assertRaisesRegex(TraceError, "hidden enclosing"):
                analyze(events((0, 14, 0x81230234), (12, 14, 0x23408123)), 100, trim=trim)

    def test_finite_capture_requires_explicit_trim_and_reports_coverage(self):
        trace = events(
            (0, 9, 0), (1, 6, 123), (2, 11, 0),
            (3, 8, 0), (5, 9, 0), (6, 10, 0), (8, 12, 0),
            (10, 8, 0), (14, 9, 0), (17, 13, 0), (20, 11, 0),
            (21, 10, 0), (22, 8, 0), (23, 9, 0),
        )
        with self.assertRaises(TraceError):
            analyze(trace, 100)
        report = analyze(trace, 100, trim=True)
        self.assertEqual(report["used_first_record"], 5)
        self.assertEqual(report["used_last_record"], 10)
        self.assertEqual(report["discarded_prefix_records"], 5)
        self.assertEqual(report["discarded_tail_records"], 3)
        self.assertEqual(report["discarded_prefix_ticks"], 6)
        self.assertEqual(report["discarded_tail_ticks"], 3)
        self.assertEqual(report["window_ticks"], 14)
        self.assertEqual(report["coverage_fraction"], 14 / 23)
        self.assertEqual(report["usb_excluding_adc_ticks"], 10)
        self.assertEqual(report["qualified_window"]["start_ticks"], 6)
        self.assertEqual(report["qualified_window"]["end_ticks"], 20)

    def test_initial_unknown_parent_discards_already_closed_nested_frame(self):
        report = analyze(events(
            (0, 12, 0), (2, 13, 0), (3, 11, 0),
            (4, 8, 0), (5, 9, 0), (6, 10, 0), (7, 12, 0),
            (9, 13, 0), (12, 11, 0), (13, 8, 0), (14, 9, 0),
        ), 100, trim=True)
        self.assertEqual(report["used_first_record"], 5)
        self.assertEqual(report["used_last_record"], 8)
        self.assertEqual(report["usb_excluding_adc_ticks"], 6)

    def test_closed_high_priority_frames_cannot_rule_out_hidden_executor(self):
        # This recording could be entirely inside one USART1 poll. There is no
        # executor boundary witness, despite every visible frame being closed.
        trace = events((0, 8, 0), (2, 9, 0), (5, 12, 0),
                       (7, 13, 0), (10, 8, 0), (12, 9, 0))
        for trim in (False, True):
            with self.subTest(trim=trim), self.assertRaisesRegex(TraceError, "hidden enclosing"):
                analyze(trace, 100, trim=trim)

    def test_reordered_events_rejected_even_in_discarded_prefix(self):
        for trim in (False, True):
            with self.subTest(trim=trim), self.assertRaisesRegex(TraceError, "reordered"):
                analyze(events((10, 9, 0), (9, 11, 0), (20, 10, 0), (30, 11, 0)), 100, trim=trim)

    def test_incomplete_and_impossible_frames_are_not_repaired(self):
        invalid = [
            events((0, 10, 0), (1, 8, 0), (2, 11, 0)),  # missing ADC end
            events((0, 10, 0), (1, 11, 0), (2, 9, 0)),  # interior missing begin
            events((0, 12, 0), (1, 10, 0), (2, 11, 0), (3, 13, 0)),  # priority reversal
            events((0, 10, 0), (1, 10, 0), (2, 11, 0), (3, 11, 0)),  # duplicate nesting
            events((0, 10, 1), (1, 11, 2)),  # wrong identity
        ]
        for trace in invalid:
            for trim in (False, True):
                with self.subTest(trace=trace, trim=trim), self.assertRaises(TraceError):
                    analyze(trace, 100, trim=trim)

    def test_unclosed_tail_rejected_by_default(self):
        with self.assertRaisesRegex(TraceError, "incomplete"):
            analyze(events((0, 10, 0), (1, 11, 0), (2, 10, 0), (3, 6, 10)), 100)

    def test_unusable_recordings_rejected(self):
        for trace in (
            events((0, 8, 0), (1, 9, 0)),  # no USB instrumentation/work
            events((0, 10, 0), (0, 11, 0)),  # no duration
            events((0, 10, 0), (1, 0, 0)),  # uninitialized slot
            events((0, 9, 0), (1, 11, 0)),  # only truncated prefix
        ):
            with self.subTest(trace=trace), self.assertRaises(TraceError):
                analyze(trace, 100, trim=True)
        with self.assertRaisesRegex(TraceError, "truncated"):
            decode(b"\0" * 13)


class TaskCpuTests(unittest.TestCase):
    def test_dma_wait_is_idle_and_adc_is_not_task_cpu(self):
        from bench_trace_analysis import analyze_task_cpu
        report = analyze_task_cpu(events(
            (0,1,1),(1,4,10),(3,8,0),(6,9,0),(10,5,10),(11,7,1),
            (20,25,0),(22,26,0),(30,1,1),(31,4,10),(35,5,10),(36,7,1)), 1000)
        self.assertEqual(report['cpu_ticks']['task:0x0000000a'], 10)
        self.assertEqual(report['cpu_ticks']['irq:adc'], 3)
        self.assertEqual(report['cpu_ticks']['irq:spi_rx'], 2)
        self.assertEqual(report['cpu_ticks']['idle'], 17)
        self.assertEqual(sum(report['cpu_ticks'].values()),36)

    def test_nested_usb_task_does_not_charge_the_preempted_thread(self):
        from bench_trace_analysis import analyze_task_cpu
        report = analyze_task_cpu(events(
            (0,1,1),(1,4,10),(3,10,0),(4,4,20),(5,8,0),(8,9,0),
            (10,5,20),(11,11,0),(14,5,10),(15,7,1)), 1000)
        self.assertEqual(report['cpu_ticks']['task:0x0000000a'],5)
        self.assertEqual(report['cpu_ticks']['task:0x00000014'],3)
        self.assertEqual(report['cpu_ticks']['irq:usb_executor'],2)
        self.assertEqual(report['cpu_ticks']['irq:adc'],3)

    def test_rejects_incomplete_or_overlapping_task_polls(self):
        from bench_trace_analysis import analyze_task_cpu
        for trace in [events((0,1,1),(1,4,10),(2,7,1)),
                      events((0,1,1),(1,4,10),(2,4,11),(3,5,11),(4,7,1)),
                      events((0,4,10),(1,5,10)),
                      events((3,1,1),(2,7,1))]:
            with self.subTest(trace=trace), self.assertRaises(TraceError):
                analyze_task_cpu(trace,1000)


if __name__ == "__main__":
    unittest.main()
