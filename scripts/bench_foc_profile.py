"""Finite SPI/Packed28 FOC profiles; all SWD setup/export is outside excitation."""
import argparse
import hashlib
import json
import math
import re
from pathlib import Path
import signal
import subprocess
import time

import bench_overclock as probe
import f405_timing


def decode(words):
    h, body, gap = words[:16], words[16:272], words[272:528]
    assert len(words) == 528 and h[0] == 1 and h[2] == 64
    assert h[3] > 1000 and sum(body) == h[3] and sum(gap) == h[4] == h[3] - 1
    assert h[8] == 0, "DWT epoch changed during measurement"
    assert (h[10] - h[9]) & 0xffffffff == h[3] - 1
    def quantiles(bins):
        result = {}
        for name, q in [("p50", .5), ("p99", .99), ("p999", .999)]:
            target, total = math.ceil(sum(bins) * q), 0
            for i, n in enumerate(bins):
                total += n
                if total >= target:
                    result[name] = dict(lower_us=i * h[2] * 1e6 / h[1],
                        upper_us=(i+1) * h[2] * 1e6 / h[1] if i < 255 else None)
                    break
        return result
    return dict(cpu_hz=h[1], n=h[3], mean_us=(h[12]+(h[13]<<32))*1e6/h[1]/h[3],
        max_us=h[5]*1e6/h[1], entry_max_us=h[6]*1e6/h[1], over_budget=h[7],
        worst_seq=h[11], first_seq=h[9], last_seq=h[10], budget_cycles=h[15],
        body=quantiles(body), entry_gap=quantiles(gap), raw_words=words)


def host_cadence_bounds(events, cycles):
    # Applied ACKs bracket the MCU mode transitions in the host's independent
    # monotonic clock. No DWT frequency/HSE estimate is used in this bound.
    start = next(e for e in events if set(e["cmd"]) & {"start", "velocity"})
    end = next(e for e in events if e["i"] > start["i"] and set(e["cmd"]) & {"coast", "stop"})
    shortest = end["t_sent"] - start["t_acked"]
    longest = end["t_acked"] - start["t_sent"]
    assert 0 < shortest <= longest
    return [cycles / longest, cycles / shortest]


def diagnostic_loss_only(log):
    # Match the complete operational-error map, excluding ACK/terminal failures.
    return re.search(r'operational errors: \{"stream_stop": String\("raw capture failed: raw capture lost samples: producer=\d+, host=\d+"\)\}', log) is not None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--detail", choices=["minimal", "sections", "sections-ram", "phase-ram"], default="minimal")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--recovery-elf", type=Path, required=True)
    parser.add_argument("--logging", choices=["timing", "quiet"], default="timing")
    parser.add_argument("--cases", default="hold,current,velocity40,velocity120")
    args = parser.parse_args()
    cases = args.cases.split(",")
    assert len(cases) == len(set(cases)) and set(cases) <= {"hold", "current", "velocity40", "velocity120"}
    if args.detail.endswith("-ram") and "current" in cases:
        parser.error("RAM section export requires >1.1 seconds active; omit the short current pulse")
    assert 1 <= args.repeats <= 5
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = dict(detail=args.detail, logging=args.logging, runs=[])
    logger = None
    try:
        with (args.output/"flash.log").open("w") as log:
            subprocess.run(["just", "flash-mk5-foc-profile", args.detail, args.logging], stdout=log,
                stderr=subprocess.STDOUT, check=True, timeout=600)
        image = args.output/"firmware.elf"
        image.write_bytes(probe.ELF.read_bytes())
        manifest["sha256"] = hashlib.sha256(image.read_bytes()).hexdigest()
        manifest["commit"] = probe.run(["git", "rev-parse", "HEAD"]).stdout.strip()
        (args.output/"source.patch").write_text(probe.run(["git", "diff", "HEAD"]).stdout)
        nm = probe.run(["rust-nm", "--defined-only", str(image)]).stdout
        (args.output/"symbols.txt").write_text(nm)
        symbols = {line.split()[-1]: int(line.split()[0],16) for line in nm.splitlines() if len(line.split()) >= 3}
        control, data = [symbols[x] for x in ["FOC_DISTRIBUTION_CONTROL", "FOC_DISTRIBUTION_DATA"]]
        manifest["peripherals"] = f405_timing.snapshot()
        manifest["clocks"] = manifest["peripherals"]["clocks"]
        for case in cases:
            maneuver = Path("maneuvers") / f"foc-profile-{case}.json"
            for repeat in range(args.repeats):
                name = f"{case}-{repeat}"
                status = json.loads(probe.run(["just", "cli-bridge", "status"]).stdout)
                assert status["motor_state"] == "Stopped" and status["fault_count"] == 0
                if args.detail in ["sections-ram", "phase-ram"]:
                    probe.run(probe.device("write", "b32", hex(symbols["FOC_SECTIONS_READY"]), "0"))
                probe.run(probe.device("write", "b32", hex(control), "1"))
                time.sleep(.1)
                assert probe.read(control)[0] == 2, "recorder not armed"
                capture = args.output / f"{name}.parquet"
                with (args.output/f"{name}.rtt.log").open("w") as log:
                    logger = subprocess.Popen(probe.device("attach", str(image.resolve())), stdout=log, stderr=subprocess.STDOUT)
                    time.sleep(1)
                    assert logger.poll() is None, "RTT monitor failed to attach"
                    try:
                        result = subprocess.run(["just", "test-bridge", str(capture), str(maneuver)],
                            capture_output=True, text=True, timeout=35)
                    finally:
                        logger.send_signal(signal.SIGINT)
                        logger.wait(timeout=5)
                        logger = None
                (args.output/f"{name}.driver.log").write_text(result.stdout+result.stderr)
                row = dict(name=name, exit=result.returncode)
                manifest["runs"].append(row)
                if result.returncode == 0:
                    report = json.loads(capture.with_suffix(".parquet.json").read_text())
                    assert report["terminal_ok"] and all(e["ok"] for e in report["events"])
                    row["record"] = report["record"]
                    probe.validate_capture(report["record"])
                    row["lossless"] = True
                else:
                    # Detailed marks can overload the producer. Retain this as
                    # failed recording evidence while still exporting timing.
                    # Other operational errors continue to abort the experiment.
                    assert args.detail in ["sections", "sections-ram", "phase-ram"] and capture.is_file()
                    capture_error = capture.with_suffix(".parquet.log").read_text()
                    assert diagnostic_loss_only(capture_error), "non-recording operational failure"
                    row["lossless"] = False
                    row["capture_error"] = capture_error
                    row["ack_verification"] = "read oxifoc.events from saved Parquet metadata"
                status = json.loads(probe.run(["just", "cli-bridge", "status"]).stdout)
                assert status["motor_state"] == "Stopped" and status["fault_count"] == 0
                assert probe.read(control)[0] == 3, "recorder did not freeze"
                row["distribution"] = decode(probe.read(data, 528))
                if row["lossless"]:
                    bounds = host_cadence_bounds(report["events"], row["distribution"]["n"])
                    assert bounds[0] <= manifest["clocks"]["pwm_hz"] <= bounds[1], "host-time IRQ cadence disagrees with TIM1"
                    row["host_cadence_bounds_hz"] = bounds
                if args.detail in ["sections-ram", "phase-ram"]:
                    assert probe.read(symbols["FOC_SECTIONS_READY"])[0] == 1
                    row["sections"] = probe.read(symbols["FOC_SECTIONS_DATA"], 25)
                row["status"] = status
                (args.output/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
                d = row["distribution"]
                print(json.dumps(dict(name=name, mean_us=d["mean_us"], max_us=d["max_us"], over=d["over_budget"], lossless=row["lossless"])), flush=True)
                time.sleep(1)
    finally:
        if logger is not None:
            logger.terminate(); logger.wait(timeout=5)
        # Always command Stop before touching SWD or restoring firmware.
        try:
            probe.run(["just", "cli-bridge", "stop"])
        finally:
            probe.run(probe.device("download", "--connect-under-reset", "--verify", str(args.recovery_elf)), timeout=90)
            probe.run(probe.device("reset"))
            manifest["restored"] = json.loads(probe.run(["target/release/oxifoc-host-cli", "--elf", str(args.recovery_elf), "--wait-secs", "12", "--json", "status"]).stdout)
            (args.output/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")


if __name__ == "__main__":
    main()
