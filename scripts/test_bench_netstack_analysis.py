"""Offline finite-lock accounting cases, with no device or third-party packages."""

import csv
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

from bench_netstack_analysis import Event, TraceError, analyze, decode, write_csv


LOCK_A = 0x20001000
LOCK_B = 0x20002000


def events(*rows):
    return [Event(ticks, identity, kind) for ticks, kind, identity in rows]


def snapshot(counter, *, down=True, transition=False):
    return counter | (int(down) << 16) | (int(transition) << 17)


def bounded(*rows):
    return events((0, 17, snapshot(100)), *rows, (100, 17, snapshot(90)))


class NetstackTraceTests(unittest.TestCase):
    def test_nested_locks_are_lifo_by_id_and_union_is_not_summed(self):
        report = analyze(bounded(
            (10, 15, LOCK_A), (20, 15, LOCK_B), (40, 16, LOCK_B), (60, 16, LOCK_A),
        ), 1_000_000, mask="primask", export_spans=True)
        self.assertEqual(report["lock_summary"]["complete_observed_outer_groups"], 1)
        self.assertEqual(report["lock_summary"]["complete_spans"], 2)
        self.assertEqual(report["lock_summary"]["covered_union_ticks"], 50)
        self.assertEqual(report["lock_summary"]["cpu_covered_union_ticks"], 50)
        self.assertEqual(report["spans"]["lock_union"], [[10, 60]])
        self.assertEqual([row["duration_ticks"] for row in report["spans"]["locks"]], [50, 20])
        self.assertEqual([row["depth"] for row in report["spans"]["locks"]], [0, 1])
        self.assertEqual(report["locks"][0]["duration_ticks"]["us"]["max"], 50)

    def test_basepri_adc_preemption_is_independent_and_subtracted_once(self):
        report = analyze(bounded(
            (10, 15, LOCK_A), (20, 15, LOCK_B),
            (25, 17, snapshot(80)), (27, 8, 0), (37, 9, 0),
            (40, 16, LOCK_B), (50, 16, LOCK_A),
            (60, 17, snapshot(70)), (62, 8, 0), (80, 9, 0),
        ), 1_000_000, mask="basepri", export_spans=True)
        summary = report["lock_summary"]
        self.assertEqual(summary["covered_union_ticks"], 40)
        self.assertEqual(summary["recorded_adc_overlap_union_ticks"], 10)
        self.assertEqual(summary["cpu_covered_union_ticks"], 30)
        rows = report["spans"]["locks"]
        self.assertEqual([row["recorded_adc_overlap_ticks"] for row in rows], [10, 10])
        self.assertEqual([row["cpu_duration_ticks"] for row in rows], [30, 10])
        # Kind 17 at 25 precedes 8 at 27; these two probe ticks remain charged.
        self.assertIn("snapshot and entry probe before 8", summary["cpu_scope"])

    def test_adc_and_lock_spans_can_overlap_without_sharing_one_stack(self):
        report = analyze(bounded(
            (10, 8, 0), (15, 15, LOCK_A), (20, 9, 0), (40, 16, LOCK_A),
        ), 100, mask="basepri", export_spans=True)
        row = report["spans"]["locks"][0]
        self.assertEqual(row["recorded_adc_overlap_ticks"], 5)
        self.assertEqual(row["cpu_duration_ticks"], 20)

    def test_primask_rejects_adc_observations_inside_lock_even_in_discarded_edges(self):
        cases = [
            bounded((10, 15, LOCK_A), (20, 17, snapshot(5)), (30, 16, LOCK_A)),
            bounded((10, 15, LOCK_A), (20, 8, 0), (25, 9, 0), (30, 16, LOCK_A)),
            # A lock inside ADC is also impossible for this baseline.
            bounded((10, 8, 0), (20, 15, LOCK_A), (25, 16, LOCK_A), (30, 9, 0)),
            events((0, 15, LOCK_A), (10, 17, snapshot(5)), (20, 16, LOCK_A),
                   (30, 17, snapshot(6))),
        ]
        for trace in cases:
            with self.subTest(trace=trace), self.assertRaisesRegex(TraceError, "PRIMASK"):
                analyze(trace, 100, mask="primask", trim=True)

    def test_incomplete_edges_require_explicit_trim_and_report_counts(self):
        trace = events(
            (0, 9, 0), (1, 16, LOCK_B), (2, 17, snapshot(100)),
            (3, 8, 0), (5, 9, 0), (10, 15, LOCK_A), (20, 16, LOCK_A),
            (25, 17, snapshot(90)), (26, 8, 0), (27, 9, 0),
            (30, 15, LOCK_A), (31, 15, LOCK_B), (33, 16, LOCK_B),
        )
        with self.assertRaises(TraceError):
            analyze(trace, 100, mask="primask")
        report = analyze(trace, 100, mask="primask", trim=True, export_spans=True)
        discarded = report["discarded"]
        self.assertEqual(discarded["leading_lock_ends_without_begin"], 1)
        self.assertEqual(discarded["leading_adc_ends_without_begin"], 1)
        self.assertEqual(discarded["trailing_lock_begins_without_end"], 1)
        self.assertEqual(discarded["complete_lock_spans_inside_unclosed_tail_group"], 1)
        self.assertEqual(report["lock_summary"]["complete_spans"], 1)
        self.assertEqual(report["spans"]["locks"][0]["first_record"], 5)
        self.assertEqual(discarded["outside_snapshot_window_prefix_records"], 2)
        self.assertEqual(discarded["outside_snapshot_window_tail_records"], 5)

    def test_complete_basepri_group_crossing_snapshot_boundary_is_excluded_whole(self):
        # First/last ADC snapshots do not witness an empty BASEPRI lock stack.
        trace = events(
            (0, 15, LOCK_A), (5, 17, snapshot(100)),
            (10, 15, LOCK_B), (15, 16, LOCK_B), (20, 16, LOCK_A),
            (25, 15, LOCK_B), (30, 16, LOCK_B),
            (35, 15, LOCK_A), (40, 17, snapshot(90)), (45, 16, LOCK_A),
        )
        report = analyze(trace, 100, mask="basepri", export_spans=True)
        self.assertEqual(report["lock_summary"]["complete_spans"], 1)
        self.assertEqual(report["spans"]["locks"][0]["start_ticks"], 25)
        self.assertEqual(report["discarded"]["complete_outer_lock_groups_outside_snapshot_window"], 2)
        self.assertEqual(report["discarded"]["complete_lock_spans_outside_snapshot_window"], 3)

    def test_unclosed_basepri_tail_cannot_promote_its_closed_child(self):
        trace = bounded((10, 15, LOCK_A), (20, 15, LOCK_B), (30, 16, LOCK_B))
        report = analyze(trace, 100, mask="basepri", trim=True)
        self.assertEqual(report["lock_summary"]["complete_spans"], 0)
        self.assertEqual(report["discarded"]["complete_lock_spans_inside_unclosed_tail_group"], 1)
        self.assertEqual(report["adc_phase"]["snapshots"], 2)

    def test_ambiguous_prefix_after_known_begin_is_rejected(self):
        trace = bounded((10, 15, LOCK_A), (20, 16, LOCK_A), (30, 16, LOCK_B))
        for mask in ("primask", "basepri"):
            with self.subTest(mask=mask), self.assertRaisesRegex(TraceError, "initial prefix"):
                analyze(trace, 100, mask=mask, trim=True)

    def test_primask_snapshot_seals_unknown_lock_prefix_but_basepri_does_not(self):
        trace = bounded((10, 16, LOCK_A), (20, 15, LOCK_B), (30, 16, LOCK_B))
        with self.assertRaisesRegex(TraceError, "initial prefix"):
            analyze(trace, 100, mask="primask", trim=True)
        report = analyze(trace, 100, mask="basepri", trim=True)
        self.assertEqual(report["discarded"]["leading_lock_ends_without_begin"], 1)
        self.assertEqual(report["lock_summary"]["complete_spans"], 1)

    def test_internal_mismatches_are_never_repaired_by_trimming(self):
        cases = [
            bounded((10, 15, LOCK_A), (20, 15, LOCK_B), (30, 16, LOCK_A), (40, 16, LOCK_B)),
            bounded((10, 15, LOCK_A), (20, 15, LOCK_A), (30, 16, LOCK_A)),
            bounded((10, 8, 0), (20, 8, 0), (30, 9, 0)),
            bounded((10, 8, 0), (20, 9, 0), (30, 9, 0)),
            bounded((10, 8, 0), (20, 17, snapshot(1)), (30, 9, 0)),
            bounded((10, 9, 0)),  # kind 17 already witnessed a new ADC boundary
        ]
        for trace in cases:
            for trim in (False, True):
                with self.subTest(trace=trace, trim=trim), self.assertRaises(TraceError):
                    analyze(trace, 100, mask="basepri", trim=trim)

    def test_duplicate_unmatched_prefix_ends_cannot_be_hidden_by_trimming(self):
        for kind, identity in ((9, 0), (16, LOCK_A)):
            trace = events((0, kind, identity), (1, kind, identity),
                           (2, 17, snapshot(100)), (10, 17, snapshot(90)))
            with self.subTest(kind=kind), self.assertRaises(TraceError):
                analyze(trace, 100, mask="basepri", trim=True)

    def test_incomplete_adc_tail_is_reported_without_inventing_a_duration(self):
        trace = events((0, 17, snapshot(100)), (1, 8, 0), (2, 9, 0),
                       (10, 15, LOCK_A), (20, 16, LOCK_A),
                       (30, 17, snapshot(90)), (31, 8, 0))
        with self.assertRaisesRegex(TraceError, "incomplete"):
            analyze(trace, 100, mask="primask")
        report = analyze(trace, 100, mask="primask", trim=True)
        self.assertEqual(report["discarded"]["trailing_adc_begins_without_end"], 1)
        self.assertEqual(report["adc_recorded_spans"]["complete_spans"], 1)
        self.assertEqual(report["lock_summary"]["cpu_covered_union_ticks"], 10)

    def test_wrapping_timer_preserves_record_order_and_durations(self):
        trace = events(
            (0xFFFFFFF0, 17, snapshot(100)), (0xFFFFFFF8, 15, LOCK_A),
            (4, 17, snapshot(90)), (5, 8, 0), (9, 9, 0),
            (12, 16, LOCK_A), (20, 17, snapshot(80)),
        )
        report = analyze(trace, 1_000_000, mask="basepri", export_spans=True)
        self.assertEqual(report["input_span_ticks"], 36)
        self.assertEqual(report["spans"]["locks"][0]["duration_ticks"], 20)
        self.assertEqual(report["spans"]["locks"][0]["cpu_duration_ticks"], 16)
        self.assertEqual(report["adc_entry_intervals"]["ticks"]["max"], 20)

    def test_reordered_or_half_counter_gap_is_rejected_before_edge_trim(self):
        for trace in (
            events((10, 16, LOCK_A), (9, 17, snapshot(100)), (20, 17, snapshot(90))),
            events((0, 17, snapshot(100)), (0x80000000, 17, snapshot(90))),
            events((0, 17, snapshot(100)), (10, 17, snapshot(90)), (9, 15, LOCK_A)),
        ):
            with self.subTest(trace=trace), self.assertRaisesRegex(TraceError, "reordered or ambiguous"):
                analyze(trace, 100, mask="basepri", trim=True)

    def test_unknown_kind_and_bad_ids_rejected_in_discarded_edges(self):
        for kind, identity, message in (
            (0, 0, "kind"), (14, 0, "kind"), (18, 0, "kind"),
            (8, LOCK_A, "ADC span id"), (9, 1, "ADC span id"),
            (15, 0, "nonzero"), (16, 0, "nonzero"),
            (17, 1 << 18, "reserved"), (17, 4196, "CNT exceeds"),
        ):
            trace = bounded((1, kind, identity))
            for trim in (False, True):
                with self.subTest(kind=kind, identity=identity, trim=trim), self.assertRaisesRegex(TraceError, message):
                    analyze(trace, 100, mask="basepri", trim=trim)

    def test_phase_uses_direction_and_double_tim5_hz_and_flags_transition(self):
        report = analyze(events(
            (0, 17, snapshot(4190)),       # down: 4195 - 4190 = 5
            (10, 17, snapshot(2, down=False)),  # up: 4195 + 2 = 4197
            (30, 17, snapshot(4195, down=False)),  # full period -> 0
            (60, 17, snapshot(100, transition=True)),
        ), 1_000_000, mask="primask", export_spans=True)
        phase = report["adc_phase"]
        self.assertEqual(phase["snapshots"], 4)
        self.assertEqual(phase["direction_transition_ambiguous"], 1)
        self.assertEqual(phase["raw_phase_from_preceding_peak"]["count"], 3)
        self.assertEqual(phase["raw_phase_from_preceding_peak"]["ticks"]["min"], 0)
        self.assertEqual(phase["raw_phase_from_preceding_peak"]["ticks"]["p50"], 5)
        self.assertEqual(phase["raw_phase_from_preceding_peak"]["us"]["max"], 2098.5)
        self.assertFalse(report["spans"]["adc_snapshots"][-1]["phase_percentile_eligible"])
        self.assertEqual(report["adc_entry_intervals"]["ticks"]["p50"], 20)

    def test_all_ambiguous_phase_samples_have_no_percentiles(self):
        report = analyze(events((0, 17, snapshot(1, transition=True)),
                                (10, 17, snapshot(2, transition=True))), 100, mask="primask")
        self.assertEqual(report["adc_phase"]["raw_phase_from_preceding_peak"],
                         {"count": 0, "ticks": None, "us": None})

    def test_release_gap_uses_subsequent_record_and_includes_probe_tail(self):
        report = analyze(bounded(
            (10, 15, LOCK_A), (20, 16, LOCK_A),
            (20, 17, snapshot(50)),
            (30, 15, LOCK_A), (70, 16, LOCK_A),
            (80, 17, snapshot(40)),
        ), 1_000_000, mask="primask", export_spans=True)
        rows = report["spans"]["locks"]
        self.assertEqual([row["end_to_next_adc_append_ticks"] for row in rows], [0, 10])
        correlation = report["long_lock_release_correlation"]
        self.assertEqual(correlation["long_lock_spans"], 1)
        self.assertEqual(correlation["end_to_next_adc_append"]["ticks"]["max"], 10)
        self.assertIn("release tail", correlation["scope"])
        self.assertIn("causality", correlation["scope"])

    def test_report_has_no_global_mask_fraction_or_latency_claim(self):
        report = analyze(bounded((10, 15, LOCK_A), (20, 16, LOCK_A)), 100, mask="basepri")
        def visit(value):
            if isinstance(value, dict):
                for key, child in value.items():
                    self.assertNotIn("fraction", key)
                    self.assertNotIn("percent", key)
                    visit(child)
            elif isinstance(value, list):
                for child in value:
                    visit(child)
        visit(report)
        self.assertIn("unknown enclosing", report["lock_summary"]["scope"])
        self.assertIn("not absolute JEOC or IRQ latency", report["adc_phase"]["scope"])
        self.assertIn("full-period aliasing", report["adc_phase"]["scope"])

    def test_record_format_and_unusable_input(self):
        self.assertEqual(decode(bytes.fromhex("78563412 efbeadde 0f000000")),
                         [Event(0x12345678, 0xDEADBEEF, 15)])
        with self.assertRaisesRegex(TraceError, "truncated"):
            decode(b"\0" * 13)
        for trace in ([], events((0, 17, snapshot(1))),
                      events((0, 15, LOCK_A), (1, 16, LOCK_A)),
                      events((0, 17, snapshot(1)), (0, 17, snapshot(2)))):
            with self.subTest(trace=trace), self.assertRaises(TraceError):
                analyze(trace, 100, mask="primask")
        for kwargs in ({"timer_hz": 0}, {"timer_hz": True}, {"timer_hz": 1.5},
                       {"mask": "unknown"}, {"arr": 0}, {"arr": 65536}):
            options = {"timer_hz": 100, "mask": "primask", **kwargs}
            with self.subTest(options=options), self.assertRaises(TraceError):
                analyze(bounded(), **options)

    def test_csv_includes_every_qualified_span(self):
        report = analyze(bounded((10, 15, LOCK_A), (20, 16, LOCK_A)),
                         100, mask="primask", export_spans=True)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "spans.csv"
            write_csv(report, path)
            with path.open(newline="") as source:
                rows = list(csv.DictReader(source))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["id_hex"], "0x20001000")
        self.assertEqual(rows[0]["cpu_duration_ticks"], "10")

    def test_cli_honors_frozen_sidecar_and_writes_json_and_csv(self):
        trace = bounded((10, 15, LOCK_A), (20, 16, LOCK_A))
        data = b"".join(struct.pack("<III", row.timer_ticks, row.id, row.kind) for row in trace)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.bin"
            path.write_bytes(data)
            metadata = {"timer_hz": 1_000_000, "bytes": len(data), "events": len(trace),
                        "sha256": hashlib.sha256(data).hexdigest(), "state": 0}
            sidecar = Path(str(path) + ".json")
            sidecar.write_text(json.dumps(metadata))
            output, csv_path = Path(directory) / "report.json", Path(directory) / "spans.csv"
            command = ["uv", "run", "--no-project", "python", str(Path(__file__).with_name("bench_netstack_analysis.py")),
                       str(path), "--mask", "primask", "--export-spans", "--out", str(output), "--csv", str(csv_path)]
            completed = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertEqual(json.loads(output.read_text())["spans"]["locks"][0]["duration_ticks"], 10)
            self.assertTrue(csv_path.exists())
            metadata["sha256"] = "not-the-capture"
            sidecar.write_text(json.dumps(metadata))
            completed = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("sidecar sha256", completed.stderr)


if __name__ == "__main__":
    unittest.main()
