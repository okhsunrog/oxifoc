#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["polars>=1", "pyarrow>=16", "numpy>=1.26"]
# ///
"""benchsuite — sensorless regression suite for the B-G431B-ESC1 bench.

Runs the canonical maneuvers through oxifoc-host-cli, captures fast
telemetry + the device defmt log, and gates each run against the
thresholds agreed in docs/TODO.md ("CLEANUP SESSION", 2026-07-07). The
point is a cheap, honest PASS/FAIL before and after any estimator change:
a run that spins the motor but trips a hidden restart, leaks a fault, or
loses telemetry frames must fail loudly, not average away.

Scenarios (all thresholds live in SCENARIOS below):
  spin-punch    maneuvers/spin-punch-15-2k.json     @ 2 kHz capture
  openloop-960  maneuvers/prof-openloop-ramp960.json @ 1 kHz capture
  endurance     maneuvers/endurance-20s.json         @ 1 kHz capture

Preconditions: the canonical g431 firmware is flashed and the probe is
attached; the rotor is free to spin; no active faults. The runner checks
faults before and after every scenario and inserts a coast-down settle
between scenarios so each one starts from standstill (the cold-start
check would fail otherwise — that is intentional).

Usage:
    uv run scripts/benchsuite.py                 # full suite
    uv run scripts/benchsuite.py --scenario spin-punch
    uv run scripts/benchsuite.py --out-dir captures/bench/mytag
    uv run scripts/benchsuite.py --json          # machine-readable report

Exit code 0 = every gate passed; 1 = any gate failed or a run errored.
"""
from __future__ import annotations

import argparse
import datetime
import json
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import polars as pl
import pyarrow.parquet as pq

REPO = Path(__file__).resolve().parent.parent
CLI = REPO / "target" / "release" / "oxifoc-host-cli"

# Set from --transport; the g431 bench is RTT, the CF2/f405 bench is USB.
TRANSPORT = "rtt"
# Optional --elf/--chip overrides: oxifoc-host.toml's elf/chip keys name ONE
# board; on any other board the defmt table is wrong (device log lines
# vanish) and the RTT control-block pin never routes.
ELF: str | None = None
CHIP: str | None = None

# Seconds of coast-down between scenarios: the terminal `stop` gates off,
# the rotor freewheels from the ~7.6k erpm ceiling and needs a few seconds
# to reach standstill (see maneuvers/coast-decay.json) so the next
# scenario's deadshort probe sees a standing rotor.
SETTLE_S = 6.0

# ---------------------------------------------------------------------------
# Device-log markers (defmt lines relayed by the host as `... INFO device: ...`).
# Sources: oxifoc-core/src/foc/phase/manager.rs (startup sequencer),
# oxifoc-core/src/state.rs ("FOC step error"), foc_driver.rs voltage FAULTs,
# oxifoc-g431/src/foc.rs (COMP trip), oxifoc-g431/src/protocol.rs (isr/s).
MARK_COLD_START = "ramp cold start"
MARK_HANDOFF = (
    "handoff confirmed by probe",       # two-sided confirm passed
    "deadshort caught spinning rotor",  # pre-ramp seed from a live rotor
    "seeding observer from probe",      # confirm diverged -> fast-seed
)
MARK_RESTART = (
    "hold gave up",       # confirm hold exhausted -> observer reset + recycle
    "restart churn",      # suppressed-log counter only prints when churning
)
MARK_FAULT = (
    "FOC step error",     # dq overcurrent trip (state.rs)
    # Colon matters: "OverVoltage FAULT:", "HW overcurrent FAULT:" are
    # faults; the f405 boot line "DRV8301 nFAULT monitor started" is not.
    "FAULT:",
    # Deadman/failsafe engaging mid-run is a hard failure even though the
    # terminal Stop acknowledges the latch before the post-run fault query
    # (learned on the CF2: a deadman trip masqueraded as an estimator
    # deadlock for a whole session — captures/bench/cf2-baseline-1).
    "CommTimeout raised",
    "failsafe latched",
    "stop sequence armed",
)
RE_ISR = re.compile(r"isr/s: n=(\d+) avg=(\d+) max=(\d+) over=(\d+) load_pct=(\d+)")
RE_TRACING = re.compile(r"^\d{4}-\d{2}-\d{2}T\S+\s+(\w+)\s+(\S+?):\s?(.*)$")
RE_ANSI = re.compile(r"\x1b\[[0-9;]*m")

# ---------------------------------------------------------------------------
# Scenario table. Gates marked (TODO) are the thresholds agreed in
# docs/TODO.md; `cruise_erpm_min` is a suite addition — the 0.3 A cruise
# rides the 800 el-rad/s ceiling (~7.6k erpm, final-1 measured 7597), so a
# big sag means the governor or ceiling regressed even if the std is tiny.
SCENARIOS = {
    "spin-punch": dict(
        maneuver="maneuvers/spin-punch-15-2k.json",
        cold_starts=1,          # (TODO) exactly one deadshort->ramp cold start
        handoffs=1,             # (TODO) exactly one seed-or-confirm handoff
        isr_load_max_pct=85,    # (TODO)
        # Climb = from the 1.5 A command until the rotor first reaches 95%
        # of the measured cruise speed (~0.5 s: 1.2 s ramp is cut short by
        # an early confirm handoff + a ceiling-limited spin-up).  Medians
        # over the whole command epoch are meaningless here: the no-load
        # rotor hits the 800 el-rad/s ceiling in ~0.1 s of closed loop and
        # the per-cycle speed cut collapses iq to the ~0.1 A friction hold.
        climb=dict(ev_from=0, iq_median_min=1.4),                  # (TODO)
        cruise=dict(ev_from=1, ev_to=2, settle_s=1.0,
                    erpm_std_frac_max=0.03,                        # (TODO)
                    erpm_min=6500.0),
    ),
    "openloop-960": dict(
        maneuver="maneuvers/prof-openloop-ramp960.json",
        cold_starts=None,       # open-loop drive: no startup sequencer
        handoffs=None,
        isr_load_max_pct=85,    # (TODO) staircase loss-free, load flat
    ),
    "endurance": dict(
        maneuver="maneuvers/endurance-20s.json",
        cold_starts=1,
        handoffs=1,
        isr_load_max_pct=85,
    ),
}

# Per-board overrides (--board). CF2: the standard 0.3 A cruise scenario —
# the 2026-07-07 "0.3 A collapse" turned out to be a deadman trip + failsafe
# ramp, not a control failure (5/5 clean 0.3 A cruises at std 1.7% once the
# bench staleness config was written). Only the climb-iq gate differs:
# a 1.5 A command measures 1.33-1.46 A here (161 mA/LSB quantization +
# board gain/dead-time differences vs the g431's 1.44-1.46).
BOARD_OVERRIDES = {
    "g431": {},
    "cf2": {
        "spin-punch": dict(
            climb=dict(ev_from=0, iq_median_min=1.3),
            # 161 mA torque quantization modulates the ceiling governor:
            # cruise std measured 1.5-3.6% across runs (g431: 2.2-2.5%).
            cruise=dict(ev_from=1, ev_to=2, settle_s=1.0,
                        erpm_std_frac_max=0.05, erpm_min=6500.0),
        ),
    },
}


# ---------------------------------------------------------------------------
def run_cli(args: list[str], timeout: float) -> tuple[int, list[str], str]:
    """Run the host CLI; split stdout into tracing/device log lines and the
    non-tracing remainder (the --json document)."""
    elf_args = ["--elf", ELF] if ELF else []
    if CHIP:
        elf_args += ["--chip", CHIP]
    proc = subprocess.run(
        # Explicit transport so a serial_path in oxifoc-host.toml can't
        # route the suite to a UART the firmware doesn't serve.
        [str(CLI), "--transport", TRANSPORT, *elf_args, *args],
        cwd=REPO,  # oxifoc-host.toml (chip/elf/probe) is loaded from cwd
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    log_lines, json_buf = [], []
    for line in RE_ANSI.sub("", proc.stdout).splitlines():
        if RE_TRACING.match(line):
            log_lines.append(line)
        else:
            json_buf.append(line)
    # anyhow errors land on stderr — keep them in the log or a failed run
    # is undiagnosable from the report alone.
    log_lines.extend(f"stderr: {l}" for l in RE_ANSI.sub("", proc.stderr).splitlines())
    return proc.returncode, log_lines, "\n".join(json_buf)


def device_lines(log_lines: list[str]) -> list[str]:
    out = []
    for line in log_lines:
        m = RE_TRACING.match(line)
        if m and m.group(2) == "device":
            out.append(m.group(3))
    return out


def query_faults(timeout: float = 30.0) -> tuple[list[dict], str]:
    code, _log, body = run_cli(["--json", "faults"], timeout)
    if code != 0:
        return [], f"faults query failed (exit {code})"
    try:
        resp = json.loads(body)
    except json.JSONDecodeError:
        return [], "faults query returned unparseable output"
    return resp.get("faults", []), ""


def unwrap_seq(seq: np.ndarray, period: int = 65536) -> np.ndarray:
    s = seq.astype(np.int64)
    d = np.diff(s)
    wraps = np.cumsum(np.where(d < -period // 2, 1, 0))
    out = s.copy()
    out[1:] += wraps * period
    return out


def event_slice(df: pl.DataFrame, events: list[dict], ev_from: int, ev_to: int,
                settle_s: float = 0.0) -> pl.DataFrame:
    """Rows between two maneuver events, anchored by device seq (falling back
    to planned time only if an anchor is missing)."""
    t = df["t_s"].to_numpy()
    u = unwrap_seq(df["seq"].to_numpy())

    def row_at(ev: dict) -> int:
        anchor = ev.get("seq_after_ack") or ev.get("seq_before")
        if anchor is not None:
            # Anchor is a raw u16 seq; match it in unwrapped space near the
            # event's send time to disambiguate wraps.
            t_ev = ev["t_sent"]
            i_t = int(np.searchsorted(t, t[0] + t_ev))
            lo, hi = max(0, i_t - 4096), min(len(u), i_t + 4096)
            cand = np.where(u[lo:hi] % 65536 == anchor % 65536)[0]
            if cand.size:
                return lo + int(cand[0])
        return int(np.searchsorted(t, t[0] + ev["t_sent"]))

    lo = row_at(events[ev_from])
    hi = row_at(events[ev_to])
    if settle_s > 0.0:
        lo = int(np.searchsorted(t, t[lo] + settle_s))
    return df.slice(lo, max(0, hi - lo))


# ---------------------------------------------------------------------------
def evaluate(name: str, spec: dict, summary: dict, dev_log: list[str],
             parquet_path: Path, post_faults: list[dict]) -> list[dict]:
    """Apply every gate for one scenario; returns a list of check dicts."""
    checks: list[dict] = []

    def gate(check: str, ok: bool, detail: str):
        checks.append(dict(check=check, ok=bool(ok), detail=detail))

    # -- runner/transport integrity ----------------------------------------
    events = summary.get("events", [])
    bad_events = [e for e in events if not e.get("ok")]
    gate("events acked", not bad_events and summary.get("terminal_ok", False),
         f"{len(events)} events, {len(bad_events)} failed, "
         f"terminal_ok={summary.get('terminal_ok')}")
    rec = summary.get("record", {})
    gate("zero capture gaps", rec.get("gaps", 1) == 0 and rec.get("samples_lost", 1) == 0,
         f"gaps={rec.get('gaps')} samples_lost={rec.get('samples_lost')}")

    # -- device-log gates ----------------------------------------------------
    faults_seen = [l for l in dev_log if any(m in l for m in MARK_FAULT)]
    gate("zero faults in log", not faults_seen,
         faults_seen[0] if faults_seen else "clean")
    restarts = [l for l in dev_log if any(m in l for m in MARK_RESTART)]
    gate("zero restarts", not restarts, restarts[0] if restarts else "clean")
    gate("zero latched faults", not post_faults,
         "; ".join(f"{f.get('category')}: {f.get('details')}" for f in post_faults)
         or "clean")

    if spec.get("cold_starts") is not None:
        n = sum(MARK_COLD_START in l for l in dev_log)
        gate("cold starts", n == spec["cold_starts"],
             f"{n} (want {spec['cold_starts']})")
    if spec.get("handoffs") is not None:
        n = sum(any(m in l for m in MARK_HANDOFF) for l in dev_log)
        gate("seed-or-confirm handoffs", n == spec["handoffs"],
             f"{n} (want {spec['handoffs']})")

    isr = [RE_ISR.search(l) for l in dev_log]
    loads = [int(m.group(5)) for m in isr if m]
    overs = [int(m.group(4)) for m in isr if m]
    if spec.get("isr_load_max_pct") is not None:
        gate("ISR load", bool(loads) and max(loads) <= spec["isr_load_max_pct"],
             f"max={max(loads) if loads else '???'}% over_sum={sum(overs)} "
             f"(limit {spec['isr_load_max_pct']}%)")

    # -- capture-derived gates ------------------------------------------------
    if spec.get("climb") or spec.get("cruise"):
        df = pl.read_parquet(parquet_path)
        md = {k.decode(): v.decode()
              for k, v in (pq.ParquetFile(parquet_path).metadata.metadata or {}).items()}
        ev = json.loads(md.get("oxifoc.events", "[]"))
        cruise_mean = None
        if spec.get("cruise"):
            c = spec["cruise"]
            sl = event_slice(df, ev, c["ev_from"], c["ev_to"], settle_s=c["settle_s"])
            erpm = sl["erpm"].to_numpy()
            cruise_mean = float(np.abs(erpm.mean())) if erpm.size else 0.0
            std = float(erpm.std()) if erpm.size else 1e9
            frac = std / (cruise_mean + 1e-9)
            gate("cruise erpm std", frac <= c["erpm_std_frac_max"],
                 f"{cruise_mean:.0f}±{std:.0f} erpm = {frac * 100:.1f}% "
                 f"(max {c['erpm_std_frac_max'] * 100:.0f}%)")
            gate("cruise erpm floor", cruise_mean >= c["erpm_min"],
                 f"{cruise_mean:.0f} erpm (min {c['erpm_min']:.0f})")
        if spec.get("climb"):
            c = spec["climb"]
            t = df["t_s"].to_numpy()
            iq = df["iq_a"].to_numpy()
            erpm_abs = np.abs(df["erpm"].to_numpy())
            lo = int(np.searchsorted(t, t[0] + ev[c["ev_from"]]["t_sent"]))
            target = 0.95 * (cruise_mean or 0.0)
            hit = np.where(erpm_abs[lo:] >= target)[0] if target > 0 else np.array([])
            hi = lo + int(hit[0]) if hit.size else lo
            ok_window = hi > lo
            med = float(np.median(iq[lo:hi])) if ok_window else 0.0
            gate("climb iq median", ok_window and med >= c["iq_median_min"],
                 f"{med:.2f} A over {t[min(hi, len(t) - 1)] - t[lo]:.2f} s climb "
                 f"(min {c['iq_median_min']} A)")

    return checks


def run_scenario(name: str, spec: dict, out_dir: Path) -> dict:
    maneuver = REPO / spec["maneuver"]
    m = json.loads(maneuver.read_text())
    duration = m["timeline"][-1]["t"] + m["capture"].get("tail_s", 0) + 30.0
    out_parquet = out_dir / f"{name}.parquet"

    pre_faults, err = query_faults()
    if err or pre_faults:
        return dict(scenario=name, ok=False, error=err or
                    f"active faults before run: {pre_faults} — clear them first",
                    checks=[])

    code, log_lines, body = run_cli(
        ["--json", "maneuver", "run", str(maneuver), "--out", str(out_parquet)],
        timeout=duration,
    )
    (out_dir / f"{name}.log").write_text("\n".join(log_lines) + "\n")
    try:
        summary = json.loads(body)
    except json.JSONDecodeError:
        return dict(scenario=name, ok=False, checks=[],
                    error=f"maneuver run exit={code}, unparseable summary: {body[:400]}")

    time.sleep(SETTLE_S)
    post_faults, fault_err = query_faults()
    if fault_err:
        return dict(scenario=name, ok=False, checks=[], error=fault_err)

    checks = evaluate(name, spec, summary, device_lines(log_lines),
                      out_parquet, post_faults)
    return dict(scenario=name, ok=all(c["ok"] for c in checks), checks=checks,
                capture=str(out_parquet))


# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--scenario", choices=SCENARIOS, action="append",
                    help="run only these scenarios (default: all, in order)")
    ap.add_argument("--out-dir", default=None,
                    help="capture/report directory (default: captures/bench/<UTC>)")
    ap.add_argument("--json", action="store_true", help="JSON report on stdout")
    ap.add_argument("--transport", default="rtt", choices=["rtt", "usb", "serial"],
                    help="host transport (g431 bench: rtt; CF2/f405 bench: usb)")
    ap.add_argument("--elf", default=None,
                    help="firmware ELF for defmt decoding (required when the "
                         "bench board differs from oxifoc-host.toml's elf)")
    ap.add_argument("--chip", default=None,
                    help="probe-rs chip name for the rtt transport (required "
                         "when the bench board differs from oxifoc-host.toml)")
    ap.add_argument("--board", default="g431", choices=BOARD_OVERRIDES,
                    help="bench board profile (scenario/threshold overrides)")
    args = ap.parse_args()
    global TRANSPORT, ELF, CHIP
    TRANSPORT = args.transport
    ELF = args.elf
    CHIP = args.chip
    for name, override in BOARD_OVERRIDES[args.board].items():
        SCENARIOS[name] = {**SCENARIOS[name], **override}

    if not CLI.exists():
        print(f"{CLI} missing — build it: cargo build --release -p oxifoc-host-cli",
              file=sys.stderr)
        return 1

    stamp = datetime.datetime.now(datetime.UTC).strftime("%Y%m%dT%H%M%SZ")
    out_dir = Path(args.out_dir) if args.out_dir else REPO / "captures" / "bench" / stamp
    out_dir.mkdir(parents=True, exist_ok=True)

    names = args.scenario or list(SCENARIOS)
    results = []
    for i, name in enumerate(names):
        if i > 0:
            time.sleep(SETTLE_S)
        if not args.json:
            print(f"== {name} ({SCENARIOS[name]['maneuver']}) ==", flush=True)
        r = run_scenario(name, SCENARIOS[name], out_dir)
        results.append(r)
        if not args.json:
            if r.get("error"):
                print(f"  ERROR: {r['error']}")
            for c in r["checks"]:
                print(f"  [{'PASS' if c['ok'] else 'FAIL'}] {c['check']}: {c['detail']}")
        if r.get("error"):
            break  # a broken run poisons everything after it; stop honestly

    report = dict(when=stamp, out_dir=str(out_dir), results=results,
                  ok=all(r["ok"] for r in results))
    (out_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        verdict = "PASS" if report["ok"] else "FAIL"
        print(f"\nsuite: {verdict}  (report: {out_dir / 'report.json'})")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
