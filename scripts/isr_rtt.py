"""Summarise F405 ISR timing from RTT logs (`probe-rs attach` output).

usage: uv run --no-project python -B scripts/isr_rtt.py [--json] LOG...

The firmware prints one report per second (docs/performance.md, "Always-on
counters"): `isr/s`, `isr-motion/s` while in active control with a nonzero
speed estimate, `telem/s` while telemetry streams, and `hall/s` last. Each
`hall/s` line closes a report, so `isr-motion/s` may come before or after
`isr/s` within it.

Windows shorter than the nominal ISR count (the attach halts the core for
50-80 ms; `isri/s short>0`) are dropped. The rest are classified:

  idle      stopped, no telemetry stream, before the first motion window
  stream    stopped, telemetry streaming, before the first motion window
  phase N   a run of full motion windows at one Hall edge rate; its first
            and last window (they can hold a speed change) are dropped, and
            so are runs with fewer than --min-windows left

Stopped figures use `isr/s`, motion phases `isr-motion/s`. For each class the
report gives the mean of the per-window averages, their spread (max - min),
the largest window maximum and the window count.
"""
import argparse
import json
import re
import statistics

ANSI = re.compile(r"\x1b\[[0-9;]*m")
PATTERNS = {
    "isr": re.compile(r"\bisr/s: n=(\d+) avg=(\d+) max=(\d+) over=(\d+)"),
    "motion": re.compile(r"\bisr-motion/s: n=(\d+) avg=(\d+) max=(\d+)"),
    "telem": re.compile(r"\btelem/s: push=(\d+)"),
    "isri": re.compile(r"\bisri/s: .*\bshort=(\d+)"),
    "hall": re.compile(r"\bhall/s: state=\w+ edges=(\d+)"),
}


def reports(lines):
    """Log lines -> list of per-second reports (dicts of parsed fields)."""
    out, cur = [], {}
    for line in lines:
        line = ANSI.sub("", line)
        for kind, rx in PATTERNS.items():
            m = rx.search(line)
            if m:
                cur[kind] = tuple(map(int, m.groups()))
                break
        else:
            continue
        if kind == "hall":
            if "isr" in cur:
                out.append(cur)
            cur = {}
    return out


def nominal_n(reps):
    return statistics.median_high(r["isr"][0] for r in reps) if reps else 0


def full(r, n):
    """Window not shortened by a probe halt or reset."""
    return r["isr"][0] >= n - 2 and r.get("isri", (0,))[0] == 0


def classify(reps, min_windows=3, tolerance=0.15):
    """-> {class: [reports]} as described in the module docstring."""
    n = nominal_n(reps)
    out, phase, seen_motion = {}, [], False

    def close():
        if len(phase) >= min_windows + 2:
            out[f"phase {sum(k.startswith('phase') for k in out) + 1}"] = phase[1:-1]

    for r in reps:
        if not full(r, n):
            continue
        if "motion" in r and r["motion"][0] >= n - 2:
            edges = r["hall"][0]
            if phase:
                ref = statistics.mean(p["hall"][0] for p in phase)
                if abs(edges - ref) > tolerance * max(ref, 1):
                    close()
                    phase = []
            phase.append(r)
            seen_motion = True
            continue
        if phase:
            close()
            phase = []
        if seen_motion or "motion" in r or r["hall"][0]:
            continue
        out.setdefault("stream" if "telem" in r else "idle", []).append(r)
    if phase:
        close()
    return out


def stats(windows, field):
    avgs = [w[field][1] for w in windows]
    return dict(windows=len(windows), avg=round(statistics.mean(avgs), 1), spread=max(avgs) - min(avgs),
                max=max(w[field][2] for w in windows),
                hall_edges=round(statistics.mean(w["hall"][0] for w in windows)))


def summarize(lines, min_windows=3):
    reps = reports(lines)
    n = nominal_n(reps)
    classes = classify(reps, min_windows)
    return dict(
        windows=len(reps), nominal_n=n,
        dropped=sum(not full(r, n) for r in reps),
        over=sum(r["isr"][3] for r in reps),
        classes={k: stats(v, "isr" if k in ("idle", "stream") else "motion") for k, v in classes.items()},
    )


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log", nargs="+")
    p.add_argument("--min-windows", type=int, default=3, help="shortest motion run kept as a phase")
    p.add_argument("--json", action="store_true")
    a = p.parse_args()
    results = {}
    for path in a.log:
        with open(path, errors="replace") as f:
            results[path] = summarize(f, a.min_windows)
    if a.json:
        print(json.dumps(results, indent=2))
        return
    for path, s in results.items():
        print(f"{path}: {s['windows']} windows (n={s['nominal_n']}), {s['dropped']} short dropped, over={s['over']}")
        for k, c in s["classes"].items():
            print(f"  {k:8} avg {c['avg']:7.1f} spread {c['spread']:3} max {c['max']:5}"
                  f"  ({c['windows']} windows, hall {c['hall_edges']}/s)")


if __name__ == "__main__":
    main()
