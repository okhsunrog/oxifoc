"""Measure traced USB IRQ CPU time, excluding only nested ADC time.

Run offline with `uv run --no-project python scripts/bench_trace_analysis.py TRACE`.
TRACE is the binary exported by bench_executor_trace.py; its JSON sidecar supplies
the timer frequency. Strict mode rejects incomplete IRQ frames. Finite captures
can instead explicitly discard their incomplete edges with
`--trim-to-complete-window`; the report includes the discarded coverage.
Add `--export-intervals` to include the qualified USB/ADC timeline in JSON.
Qualified windows require complete USART1 executor frames at both boundaries;
ADC/OTG-only captures cannot rule out an unobserved enclosing executor span.

Other IRQs are untraced and remain charged to an enclosing USB interval. Trace
hook overhead is included. TIM5 gaps must be shorter than half a counter period;
records are never sorted to hide preemption during trace-slot reservation.
"""

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import struct


class TraceError(ValueError):
    """The recording cannot support a qualified CPU measurement."""


@dataclass(frozen=True)
class Event:
    timer_ticks: int
    id: int
    kind: int


# NVIC priorities: ADC P0, OTG_FS P5, USART1 executor P6. Smaller preempts larger.
IRQ_EVENTS = {
    8: ("adc", 0, True),
    9: ("adc", 0, False),
    10: ("usb_executor", 6, True),
    11: ("usb_executor", 6, False),
    12: ("usb_irq", 5, True),
    13: ("usb_irq", 5, False),
}


def decode(data: bytes) -> list[Event]:
    if len(data) % 12:
        raise TraceError("truncated 12-byte trace record")
    return [Event(*fields) for fields in struct.iter_unpack("<III", data)]


def analyze(events: list[Event], timer_hz: int, *, trim: bool = False,
            export_intervals: bool = False) -> dict:
    if timer_hz <= 0:
        raise TraceError("timer frequency must be positive")
    if len(events) < 2:
        raise TraceError("need at least two trace records")

    # Validate ALL records before considering edge trimming. In particular a
    # reversed prefix is still an error, not a reason to discard more data.
    ticks = [0]
    for index, event in enumerate(events):
        if not 1 <= event.kind <= 13:
            raise TraceError(f"record {index}: unknown/uninitialized kind {event.kind}")
        if not 0 <= event.timer_ticks <= 0xFFFFFFFF:
            raise TraceError(f"record {index}: timestamp is not u32")
        if index:
            delta = (event.timer_ticks - events[index - 1].timer_ticks) & 0xFFFFFFFF
            if delta >= 0x80000000:
                raise TraceError(f"record {index}: reordered or ambiguous timer gap")
            ticks.append(ticks[-1] + delta)

    stack = []  # (context, priority, id, begin record)
    complete = []  # known outermost (begin record, end record, context)
    prefix_cut = 0
    highest_rank_seen = -1
    for index, event in enumerate(events):
        if event.kind not in IRQ_EVENTS:
            continue
        context, rank, begin = IRQ_EVENTS[event.kind]
        if begin:
            if stack and rank >= stack[-1][1]:
                raise TraceError(f"record {index}: impossible IRQ nesting")
            stack.append((context, rank, event.id, index))
        elif stack:
            if (context, rank, event.id) != stack[-1][:3]:
                raise TraceError(f"record {index}: mismatched IRQ end")
            frame = stack.pop()
            if not stack:
                complete.append((frame[3], index, context))
        else:
            # An initial unknown parent can end after known nested frames. It
            # must have lower priority than every preceding observed context.
            # Discard everything through it; never infer or charge its start.
            if not trim or rank <= highest_rank_seen:
                raise TraceError(f"record {index}: IRQ end without begin")
            prefix_cut = index + 1
        highest_rank_seen = max(highest_rank_seen, rank)

    if stack and not trim:
        raise TraceError("incomplete IRQ frame at capture end")
    tail_cut = stack[0][3] if stack else len(events)
    qualified = [(start, end) for start, end, context in complete
                 if context == "usb_executor" and start >= prefix_cut and end < tail_cut]
    if not qualified:
        raise TraceError("no complete USB executor frame; cannot exclude a hidden enclosing executor")
    # P6 USART1 cannot be nested in either higher-priority traced IRQ. These
    # boundaries witness an empty tracked stack. Without them the whole capture
    # could lie inside an executor with both boundaries unobserved; that case
    # must be rejected. Higher-priority closed frames alone are insufficient.
    start, end = qualified[0][0], qualified[-1][1]
    window_ticks = ticks[end] - ticks[start]
    if window_ticks <= 0:
        raise TraceError("complete window has zero duration")

    totals = {key: 0 for key in (
        "usb_inclusive_ticks", "adc_preempting_usb_ticks", "usb_excluding_adc_ticks",
        "usb_executor_only_excluding_adc_ticks", "usb_irq_excluding_adc_ticks",
    )}
    active = []
    previous = ticks[start]
    usb_frames = 0
    intervals = {name: [] for name in ("usb_union", "adc_union", "usb_adc_overlap", "usb_excluding_adc")}

    def span(name, begin, finish):
        if not export_intervals or begin == finish:
            return
        segments = intervals[name]
        if segments and segments[-1][1] == begin:
            segments[-1][1] = finish
        else:
            segments.append([begin, finish])

    for index in range(start, end + 1):
        event = events[index]
        delta = ticks[index] - previous
        usb = [name for name in active if name != "adc"]
        if "adc" in active:
            span("adc_union", previous, ticks[index])
        if usb:
            span("usb_union", previous, ticks[index])
            totals["usb_inclusive_ticks"] += delta
            if "adc" in active:
                span("usb_adc_overlap", previous, ticks[index])
                totals["adc_preempting_usb_ticks"] += delta
            else:
                span("usb_excluding_adc", previous, ticks[index])
                totals["usb_excluding_adc_ticks"] += delta
                bucket = "usb_irq" if usb[-1] == "usb_irq" else "usb_executor_only"
                totals[f"{bucket}_excluding_adc_ticks"] += delta
        if event.kind in IRQ_EVENTS:
            context, _, begin = IRQ_EVENTS[event.kind]
            if begin:
                active.append(context)
                usb_frames += context != "adc"
            else:
                active.pop()  # fully validated above
        previous = ticks[index]
    input_ticks = ticks[-1]
    report = {
        "mode": "trim-to-complete-window" if trim else "strict",
        "timer_hz": timer_hz,
        "input_records": len(events),
        "input_span_ticks": input_ticks,
        "used_first_record": start,
        "used_last_record": end,
        "used_records": end - start + 1,
        "qualified_window": {
            "start_ticks": ticks[start], "end_ticks": ticks[end],
            "start_timer_raw": events[start].timer_ticks,
            "end_timer_raw": events[end].timer_ticks,
            "boundary_witness": "complete lowest-priority tracked IRQ: USART1 USB executor (P6)",
        },
        "discarded_prefix_records": start,
        "discarded_tail_records": len(events) - end - 1,
        "discarded_prefix_ticks": ticks[start],
        "discarded_tail_ticks": input_ticks - ticks[end],
        "window_ticks": window_ticks,
        "window_seconds": window_ticks / timer_hz,
        "coverage_fraction": window_ticks / input_ticks,
        "complete_usb_frames": usb_frames,
        **totals,
        "usb_excluding_adc_fraction": totals["usb_excluding_adc_ticks"] / window_ticks,
        "scope": "union of USART1 executor and OTG_FS spans, excluding ADC only; "
                 "other untraced IRQs and trace overhead remain included",
    }
    if export_intervals:
        report["interval_time_origin"] = {
            "timer_raw": events[0].timer_ticks,
            "units": "unwrapped TIM5 ticks relative to first input record; half-open intervals",
        }
        report["intervals"] = intervals
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--timer-hz", type=int, help="TIM5 input clock if no JSON sidecar")
    parser.add_argument("--trim-to-complete-window", action="store_true")
    parser.add_argument("--export-intervals", action="store_true", help="include timeline intervals in JSON")
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
        if not isinstance(timer_hz, int):
            raise TraceError("provide --timer-hz or the capture JSON sidecar")
        report = analyze(decode(data), timer_hz, trim=args.trim_to_complete_window,
                         export_intervals=args.export_intervals)
        report.update(trace=str(args.trace.resolve()), sha256=digest, symbols=metadata.get("symbols"))
        rendered = json.dumps(report, indent=2) + "\n"
        if args.out:
            args.out.write_text(rendered)
        else:
            print(rendered, end="")
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
