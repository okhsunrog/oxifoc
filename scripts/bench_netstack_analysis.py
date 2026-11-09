"""Analyze finite NetStack lock and ADC phase traces without third-party packages.

Run with ``uv run --no-project python scripts/bench_netstack_analysis.py TRACE
--mask primask --trim-to-complete-window --out REPORT.json --export-spans``.
TRACE consists of little-endian (TIM5 u32, payload/id u32, kind u32) records;
bench_executor_trace.py supplies its frozen-capture JSON sidecar and timer_hz.

Only kinds 8/9 (ADC begin/end, id=0), 15/16 (lock begin/end, object address),
and 17 (ADC DIR/CNT/DIR snapshot) belong to this standalone trace. All records
are validated before trimming; they are never sorted. A u32 timer gap must be
less than half the counter period. An unobserved full counter wrap is outside
what this format can establish.

Lock statistics cover fully recorded, closed outer groups inside the first to
last ADC snapshot append timestamps. They are not total interrupt-mask time:
BASEPRI permits ADC preemption, and an unobserved enclosing lock cannot be ruled
out. Nested locks are unioned for covered time. CPU duration subtracts only
recorded ADC 8/9 overlap. ADC entry snapshot/probe time before kind 8, other IRQs,
and trace overhead remain charged to the lock. Kind 17 is timestamped AFTER the
DIR/CNT/DIR read and serialization; its stamp is not the IRQ entry instant.

Phase is a raw timer observation relative to the preceding TIM1 peak, modulo
2*ARR: ARR-CNT when counting down, ARR+CNT when counting up. DIR transitions
are ambiguous and excluded from percentiles. ADC synchronization, the ISR
prologue, and full-period aliasing prevent absolute JEOC/IRQ latency claims.
"""

import argparse
from bisect import bisect_right
import csv
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import struct


class TraceError(ValueError):
    """The recording cannot support the requested qualified measurement."""


@dataclass(frozen=True)
class Event:
    timer_ticks: int
    id: int
    kind: int


@dataclass(frozen=True)
class Span:
    identity: int
    first: int
    last: int
    depth: int = 0


KINDS = {8, 9, 15, 16, 17}
U32_MAX = 0xFFFFFFFF


def decode(data: bytes) -> list[Event]:
    if len(data) % 12:
        raise TraceError("truncated 12-byte trace record")
    return [Event(*fields) for fields in struct.iter_unpack("<III", data)]


def _u32(value: int) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= U32_MAX


def _validate(events: list[Event], arr: int) -> list[int]:
    """Validate the full input before any boundary-discard decision."""
    ticks = [0]
    for index, event in enumerate(events):
        if not _u32(event.timer_ticks):
            raise TraceError(f"record {index}: timestamp is not u32")
        if not _u32(event.id):
            raise TraceError(f"record {index}: id/payload is not u32")
        if event.kind not in KINDS or not _u32(event.kind):
            raise TraceError(f"record {index}: unknown/uninitialized kind {event.kind}")
        if event.kind in (8, 9) and event.id != 0:
            raise TraceError(f"record {index}: ADC span id must be zero")
        if event.kind in (15, 16) and event.id == 0:
            raise TraceError(f"record {index}: lock id must be a nonzero object address")
        if event.kind == 17:
            if event.id >> 18:
                raise TraceError(f"record {index}: reserved ADC snapshot payload bits are set")
            if (event.id & 0xFFFF) > arr:
                raise TraceError(f"record {index}: ADC snapshot CNT exceeds ARR")
        if index:
            delta = (event.timer_ticks - events[index - 1].timer_ticks) & U32_MAX
            if delta >= 0x80000000:
                raise TraceError(f"record {index}: reordered or ambiguous timer gap")
            ticks.append(ticks[-1] + delta)
    return ticks


def _pair(events: list[Event], *, mask: str, trim: bool) -> tuple[list[list[Span]], list[Span], dict]:
    # Lock and ADC stacks are independent: ADC can preempt a BASEPRI lock.
    lock_stack = []  # (id, record, depth)
    group = []
    groups = []
    adc_start = None
    adc_spans = []
    saw_lock_begin = False
    leading_ended_lock_ids = set()
    saw_adc_boundary = False
    saw_snapshot = False
    discarded = {
        "leading_lock_ends_without_begin": 0,
        "leading_adc_ends_without_begin": 0,
        "trailing_lock_begins_without_end": 0,
        "trailing_adc_begins_without_end": 0,
        "complete_lock_spans_inside_unclosed_tail_group": 0,
    }
    for index, event in enumerate(events):
        if event.kind == 15:
            if mask == "primask" and adc_start is not None:
                raise TraceError(f"record {index}: lock begins inside an active ADC span under PRIMASK")
            if any(identity == event.id for identity, _, _ in lock_stack):
                raise TraceError(f"record {index}: recursive lock begin for active id 0x{event.id:08x}")
            lock_stack.append((event.id, index, len(lock_stack)))
            saw_lock_begin = True
        elif event.kind == 16:
            if lock_stack:
                if lock_stack[-1][0] != event.id:
                    raise TraceError(f"record {index}: mismatched lock end (LIFO id order)")
                identity, first, depth = lock_stack.pop()
                group.append(Span(identity, first, index, depth))
                if not lock_stack:
                    groups.append(sorted(group, key=lambda span: span.first))
                    group = []
            else:
                # With no priority ordering for lock IDs, a closed nested group
                # followed by an unknown parent's end is ambiguous. Reject it;
                # trimming is restricted to leading ends before ANY begin.
                if not trim or saw_lock_begin or (mask == "primask" and saw_snapshot):
                    raise TraceError(f"record {index}: lock end without begin outside initial prefix")
                if event.id in leading_ended_lock_ids:
                    raise TraceError(f"record {index}: duplicate unmatched lock end in prefix")
                leading_ended_lock_ids.add(event.id)
                discarded["leading_lock_ends_without_begin"] += 1
        elif event.kind == 8:
            if adc_start is not None:
                raise TraceError(f"record {index}: impossible same-ADC nesting")
            adc_start = index
            saw_adc_boundary = True
        elif event.kind == 9:
            if adc_start is None:
                if not trim or saw_adc_boundary:
                    raise TraceError(f"record {index}: ADC end without begin outside initial prefix")
                discarded["leading_adc_ends_without_begin"] += 1
                saw_adc_boundary = True
            else:
                adc_spans.append(Span(0, adc_start, index))
                adc_start = None
        else:  # kind 17 precedes 8 in each ADC invocation.
            if adc_start is not None:
                raise TraceError(f"record {index}: ADC entry snapshot inside an active ADC span")
            saw_snapshot = True
            saw_adc_boundary = True

        if mask == "primask" and lock_stack and event.kind in (8, 9, 17):
            raise TraceError(f"record {index}: ADC event inside a PRIMASK lock")

    if lock_stack or adc_start is not None:
        if not trim:
            raise TraceError("incomplete lock/ADC frame at capture end; use explicit edge trimming")
        discarded["trailing_lock_begins_without_end"] = len(lock_stack)
        discarded["trailing_adc_begins_without_end"] = int(adc_start is not None)
        discarded["complete_lock_spans_inside_unclosed_tail_group"] = len(group)
    return groups, adc_spans, discarded


def _union(intervals: list[tuple[int, int]]) -> list[tuple[int, int]]:
    merged = []
    # Sorting already-validated derived intervals does not change event order.
    for first, last in sorted(intervals):
        if first == last:
            continue
        if merged and first <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(last, merged[-1][1]))
        else:
            merged.append((first, last))
    return merged


def _overlap(first: int, last: int, intervals: list[tuple[int, int]]) -> int:
    return sum(max(0, min(last, end) - max(first, start)) for start, end in intervals)


def _percentile(values: list[int], percentile: float) -> float:
    position = (len(values) - 1) * percentile / 100
    lower = int(position)
    upper = min(lower + 1, len(values) - 1)
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def _stats(values: list[int], hz: int) -> dict:
    values = sorted(values)
    if not values:
        return {"count": 0, "ticks": None, "us": None}
    ticks = {"min": values[0], "p50": _percentile(values, 50),
             "p95": _percentile(values, 95), "p99": _percentile(values, 99),
             "max": values[-1]}
    return {"count": len(values), "ticks": ticks,
            "us": {key: value * 1_000_000 / hz for key, value in ticks.items()}}


def analyze(events: list[Event], timer_hz: int, *, mask: str,
            arr: int = 4195, trim: bool = False, export_spans: bool = False) -> dict:
    if not isinstance(timer_hz, int) or isinstance(timer_hz, bool) or timer_hz <= 0:
        raise TraceError("timer frequency must be a positive integer")
    if mask not in ("primask", "basepri"):
        raise TraceError("mask policy must be primask or basepri")
    if not isinstance(arr, int) or isinstance(arr, bool) or not 1 <= arr <= 0xFFFF:
        raise TraceError("ARR must be an integer from 1 to 65535")
    if len(events) < 2:
        raise TraceError("need at least two trace records")
    ticks = _validate(events, arr)
    groups, adc_spans, discarded = _pair(events, mask=mask, trim=trim)
    snapshot_indices = [index for index, event in enumerate(events) if event.kind == 17]
    if len(snapshot_indices) < 2:
        raise TraceError("need at least two ADC entry snapshots to bound the finite observation window")
    first_snapshot, last_snapshot = snapshot_indices[0], snapshot_indices[-1]
    if ticks[last_snapshot] <= ticks[first_snapshot]:
        raise TraceError("ADC snapshot window has zero duration")

    qualified = [group for group in groups
                 if group[0].first > first_snapshot and group[0].last < last_snapshot]
    excluded = [group for group in groups if group not in qualified]
    discarded.update(
        complete_outer_lock_groups_outside_snapshot_window=len(excluded),
        complete_lock_spans_outside_snapshot_window=sum(map(len, excluded)),
        outside_snapshot_window_prefix_records=first_snapshot,
        outside_snapshot_window_tail_records=len(events) - last_snapshot - 1,
        outside_snapshot_window_prefix_ticks=ticks[first_snapshot],
        outside_snapshot_window_tail_ticks=ticks[-1] - ticks[last_snapshot],
    )
    # No inferred prefix/tail duration is charged to lock CPU time. ADC spans
    # that cross a lock are clipped to it before subtraction.
    adc_union = _union([(ticks[span.first], ticks[span.last]) for span in adc_spans])
    lock_rows = []
    for group_number, group in enumerate(qualified):
        for span in group:
            first, last = ticks[span.first], ticks[span.last]
            adc_overlap = _overlap(first, last, adc_union)
            if mask == "primask" and adc_overlap:
                raise TraceError("recorded ADC span overlaps a PRIMASK lock")
            next_position = bisect_right(snapshot_indices, span.last)
            next_snapshot = snapshot_indices[next_position] if next_position < len(snapshot_indices) else None
            lock_rows.append({
                "id": span.identity, "id_hex": f"0x{span.identity:08x}",
                "outer_group": group_number, "depth": span.depth,
                "first_record": span.first, "last_record": span.last,
                "start_ticks": first, "end_ticks": last,
                "start_timer_raw": events[span.first].timer_ticks,
                "end_timer_raw": events[span.last].timer_ticks,
                "duration_ticks": last - first,
                "recorded_adc_overlap_ticks": adc_overlap,
                "cpu_duration_ticks": last - first - adc_overlap,
                "next_adc_snapshot_record": next_snapshot,
                "end_to_next_adc_append_ticks": None if next_snapshot is None else ticks[next_snapshot] - last,
            })

    lock_union = _union([(row["start_ticks"], row["end_ticks"]) for row in lock_rows])
    union_ticks = sum(last - first for first, last in lock_union)
    union_adc_ticks = sum(_overlap(first, last, adc_union) for first, last in lock_union)
    per_lock = []
    for identity in sorted({row["id"] for row in lock_rows}):
        rows = [row for row in lock_rows if row["id"] == identity]
        per_lock.append({
            "id": identity, "id_hex": f"0x{identity:08x}", "complete_spans": len(rows),
            **{key: _stats([row[key] for row in rows], timer_hz) for key in
               ("duration_ticks", "recorded_adc_overlap_ticks", "cpu_duration_ticks")},
        })

    snapshots = []
    for index in snapshot_indices:
        payload = events[index].id
        counter, down, transition = payload & 0xFFFF, bool(payload & (1 << 16)), bool(payload & (1 << 17))
        phase = (arr - counter if down else arr + counter) % (2 * arr)
        snapshots.append({
            "record": index, "append_ticks": ticks[index],
            "append_timer_raw": events[index].timer_ticks,
            "counter": counter, "direction_down": down, "direction_transition": transition,
            "phase_from_preceding_peak_tim1_ticks": phase,
            "phase_percentile_eligible": not transition,
        })
    phase_values = [row["phase_from_preceding_peak_tim1_ticks"] for row in snapshots
                    if row["phase_percentile_eligible"]]
    intervals = [ticks[last] - ticks[first] for first, last in zip(snapshot_indices, snapshot_indices[1:])]
    durations = [row["duration_ticks"] for row in lock_rows]
    long_threshold = _stats(durations, timer_hz)["ticks"]
    long_threshold = long_threshold["p95"] if long_threshold is not None else None
    long_rows = [row for row in lock_rows if row["duration_ticks"] >= long_threshold] if lock_rows else []
    release_gaps = [row["end_to_next_adc_append_ticks"] for row in long_rows
                    if row["end_to_next_adc_append_ticks"] is not None]
    report = {
        "schema_version": 1,
        "mode": "trim-to-complete-window" if trim else "strict",
        "mask_policy": mask,
        "timer_hz": timer_hz, "tim1_hz": 2 * timer_hz, "tim1_arr": arr,
        "tim1_period_ticks": 2 * arr,
        "input_records": len(events), "input_span_ticks": ticks[-1],
        "time_origin": {"timer_raw": events[0].timer_ticks,
                        "units": "unwrapped TIM5 ticks relative to first input record; half-open intervals"},
        "adc_snapshot_window": {
            "first_record": first_snapshot, "last_record": last_snapshot,
            "start_ticks": ticks[first_snapshot], "end_ticks": ticks[last_snapshot],
            "duration_ticks": ticks[last_snapshot] - ticks[first_snapshot],
            "timestamp_scope": "kind 17 append timestamp after DIR/CNT/DIR snapshot and trace serialization",
        },
        "boundary_policy": "only fully paired observed outer lock groups strictly inside the first/last ADC snapshot records; "
                           "leading unmatched ends require explicit trimming before any observed begin; "
                           "complete children of an unclosed tail group are discarded",
        "discarded": discarded,
        "lock_summary": {
            "complete_observed_outer_groups": len(qualified), "complete_spans": len(lock_rows),
            "covered_union_ticks": union_ticks,
            "recorded_adc_overlap_union_ticks": union_adc_ticks,
            "cpu_covered_union_ticks": union_ticks - union_adc_ticks,
            "duration": _stats(durations, timer_hz),
            "cpu_duration": _stats([row["cpu_duration_ticks"] for row in lock_rows], timer_hz),
            "scope": "covered time of qualified recorded groups only, unioning nested locks; "
                     "not total interrupt-mask time or a whole-window mask fraction; "
                     "unknown enclosing locks cannot be ruled out for BASEPRI",
            "cpu_scope": "subtract only recorded ADC 8/9 overlap; ADC snapshot and entry probe before 8, "
                         "untraced IRQs, release tail and trace overhead remain included",
        },
        "locks": per_lock,
        "adc_recorded_spans": {
            "complete_spans": len(adc_spans),
            "duration": _stats([ticks[span.last] - ticks[span.first] for span in adc_spans], timer_hz),
            "scope": "recorded kind 8 to kind 9 only; excludes snapshot/probe work before kind 8",
        },
        "adc_phase": {
            "snapshots": len(snapshots), "direction_transition_ambiguous": len(snapshots) - len(phase_values),
            "raw_phase_from_preceding_peak": _stats(phase_values, 2 * timer_hz),
            "tick_units": "TIM1 ticks (TIM1 clock = 2 * TIM5 clock)",
            "formula": "(ARR - CNT if DIR is down else ARR + CNT) modulo 2*ARR",
            "scope": "raw DIR/CNT/DIR phase, not absolute JEOC or IRQ latency; synchronization and ISR prologue "
                     "are unknown, full-period aliasing is not excluded; direction-transition snapshots excluded from percentiles",
        },
        "adc_entry_intervals": {
            **_stats(intervals, timer_hz),
            "scope": "between consecutive kind 17 append timestamps, not exact hardware IRQ entry instants",
        },
        "long_lock_release_correlation": {
            "long_threshold_duration_ticks_p95": long_threshold,
            "long_lock_spans": len(long_rows),
            "end_to_next_adc_append": _stats(release_gaps, timer_hz),
            "scope": "nearest subsequent kind 17 in record order after each p95-or-longer lock end; "
                     "gap includes lock release tail, ADC prologue/snapshot and probe overhead; association alone does not establish causality",
        },
    }
    if export_spans:
        report["spans"] = {
            "locks": lock_rows,
            "lock_union": [list(interval) for interval in lock_union],
            "adc": [{"first_record": span.first, "last_record": span.last,
                     "start_ticks": ticks[span.first], "end_ticks": ticks[span.last]}
                    for span in adc_spans],
            "adc_snapshots": snapshots,
        }
    return report


def write_csv(report: dict, path: Path) -> None:
    """Write every qualified lock span, including the following ADC append gap."""
    rows = report["spans"]["locks"]
    fields = ["id", "id_hex", "outer_group", "depth", "first_record", "last_record",
              "start_ticks", "end_ticks", "start_timer_raw", "end_timer_raw",
              "duration_ticks", "recorded_adc_overlap_ticks", "cpu_duration_ticks",
              "next_adc_snapshot_record", "end_to_next_adc_append_ticks"]
    with path.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--mask", choices=("primask", "basepri"), required=True,
                        help="explicit interrupt-mask policy used by this firmware")
    parser.add_argument("--timer-hz", type=int, help="TIM5 input clock if no JSON sidecar")
    parser.add_argument("--arr", type=int, default=4195, help="TIM1 ARR for the raw center-aligned phase")
    parser.add_argument("--trim-to-complete-window", action="store_true")
    parser.add_argument("--export-spans", action="store_true", help="include full qualified lock/ADC spans and snapshots in JSON")
    parser.add_argument("--csv", type=Path, help="also export every qualified lock span as CSV")
    parser.add_argument("--out", type=Path, help="write JSON here instead of stdout")
    args = parser.parse_args()
    try:
        data = args.trace.read_bytes()
        metadata_path = Path(str(args.trace) + ".json")
        metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
        if not isinstance(metadata, dict):
            raise TraceError("capture sidecar must be a JSON object")
        digest = hashlib.sha256(data).hexdigest()
        for key, actual in (("bytes", len(data)), ("sha256", digest), ("events", len(data) // 12), ("state", 0)):
            if key in metadata and metadata[key] != actual:
                raise TraceError(f"sidecar {key} does not match frozen trace")
        declared_hz = metadata.get("timer_hz")
        if args.timer_hz is not None and declared_hz is not None and args.timer_hz != declared_hz:
            raise TraceError("--timer-hz disagrees with capture sidecar")
        timer_hz = args.timer_hz if args.timer_hz is not None else declared_hz
        report = analyze(decode(data), timer_hz, mask=args.mask, arr=args.arr,
                         trim=args.trim_to_complete_window,
                         export_spans=args.export_spans or args.csv is not None)
        report.update(trace=str(args.trace.resolve()), sha256=digest, symbols=metadata.get("symbols"))
        if args.csv is not None:
            write_csv(report, args.csv)
        rendered = json.dumps(report, indent=2) + "\n"
        if args.out:
            args.out.write_text(rendered)
        else:
            print(rendered, end="")
    except (OSError, ValueError, TypeError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
