"""Finite low-speed Hall comparison over the C5, native Packed28, no NVM writes."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time
import bench_overclock as probe
import f405_timing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--recovery-elf", required=True, type=Path)
    parser.add_argument("--repeats", default=2, type=int)
    parser.add_argument("--hall-current", type=float, default=0, help="Recalibrate Hall in RAM at this current; 0 keeps persisted calibration")
    parser.add_argument("--speed-ceiling", type=float, help="Volatile bench speed ceiling, electrical rad/s")
    parser.add_argument("--cases", default="velocity40,velocity-negative40,velocity120-to40")
    parser.add_argument("--velocity-ki", type=float, help="Volatile speed PI integral gain; other gains unchanged")
    parser.add_argument("--hall-config", type=Path, help="Replay only Hall calibration from a saved config dump, in RAM")
    parser.add_argument("--interp-min-erpm", type=float, help="Volatile Hall interpolation threshold")
    args = parser.parse_args()
    assert 0 <= args.hall_current <= 10
    assert not (args.hall_current and args.hall_config), "choose measured or replayed Hall calibration"
    cases = args.cases.split(",")
    assert len(cases) == len(set(cases))
    assert cases and set(cases) <= {"velocity40", "velocity-negative40", "velocity120-to40", "velocity40-to-iq03", "velocity40-to-iq06", "velocity40-to-iq045", "rearm-current"}
    assert 1 <= args.repeats <= 3
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = dict(runs=[])
    def save():
        (args.output/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    try:
        with (args.output/"flash.log").open("w") as log:
            subprocess.run(["just", "flash-mk5-motion"], stdout=log, stderr=subprocess.STDOUT, check=True, timeout=600)
        image = args.output/"firmware.elf"
        image.write_bytes(probe.ELF.read_bytes())
        manifest["sha256"] = hashlib.sha256(image.read_bytes()).hexdigest()
        manifest["commit"] = probe.run(["git", "rev-parse", "HEAD"]).stdout.strip()
        (args.output/"source.patch").write_text(probe.run(["git", "diff", "HEAD"]).stdout)
        manifest["peripherals"] = f405_timing.snapshot()
        config = probe.run(["just", "cli-bridge", "config", "dump"]).stdout
        (args.output/"config-persisted.json").write_text(config)
        if args.hall_current:
            resistance = json.loads(config)["motor-params"]["resistance_ohm"]
            power = 1.5 * resistance * args.hall_current**2
            with (args.output/"hall-calibration.driver.log").open("w") as log:
                subprocess.run(["just", "cli-bridge", "detect", "hall", "--max-power-w", str(power), "--record", str(args.output/"hall-calibration.parquet"), "--record-hz", "20000"], stdout=log, stderr=subprocess.STDOUT, check=True, timeout=45)
        if args.hall_config:
            hall = json.loads(args.hall_config.read_text())["hall-calibration"]
            assert hall["geometry"] is not None
            probe.run(["just", "cli-bridge", "config", "set", "hall-calibration", *[key+"="+json.dumps(value, separators=(",", ":")) for key,value in hall.items()], "--volatile"])
        if args.interp_min_erpm is not None:
            probe.run(["just", "cli-bridge", "config", "set", "hall-tuning", f"interp_min_erpm={args.interp_min_erpm}", "--volatile"])
        if args.speed_ceiling is not None:
            probe.run(["just", "cli-bridge", "config", "set", "derating", f"max_speed_erad_s={args.speed_ceiling}", "speed_start_frac=0.9", "--volatile"])
        if args.velocity_ki is not None:
            probe.run(["just", "cli-bridge", "config", "set", "velocity", f"ki={args.velocity_ki}", "--volatile"])
        (args.output/"config-before.json").write_text(probe.run(["just", "cli-bridge", "config", "dump"]).stdout)
        listing = probe.run(["rust-nm", "--defined-only", str(image)]).stdout
        symbols = {line.split()[-1]: int(line.split()[0],16) for line in listing.splitlines() if len(line.split()) >= 3}
        for case in cases:
            for repeat in range(args.repeats):
                name = f"{case}-{repeat}"
                status = json.loads(probe.run(["just", "cli-bridge", "status"]).stdout)
                assert status["motor_state"] == "Stopped" and status["fault_count"] == 0
                time.sleep(1)
                for symbol in ["BENCH_HALL_EDGE_COUNT", "BENCH_HALL_EDGE_DROPPED", "BENCH_HALL_PHASE_COUNT", "BENCH_VELOCITY_COUNT", "BENCH_FAULT_COUNT"]:
                    probe.run(probe.device("write", "b32", hex(symbols[symbol]), "0"))
                capture = args.output/f"{name}.parquet"
                result = subprocess.run(["just", "test-bridge", str(capture), f"maneuvers/foc-profile-{case}.json"], capture_output=True, text=True, timeout=35)
                (args.output/f"{name}.driver.log").write_text(result.stdout+result.stderr)
                row = dict(name=name, exit=result.returncode)
                manifest["runs"].append(row)
                # Stop before any SWD export, including on a failed capture.
                probe.run(["just", "cli-bridge", "stop"])
                row["status"] = json.loads(probe.run(["just", "cli-bridge", "status"]).stdout)
                assert row["status"]["control_mode"] == "Stopped" and row["status"]["motor_state"] in ["Stopped", "Error"]
                probe.run(["just", "dump-hall-bridge", str(args.output/f"{name}.hall.json"), str(image)], timeout=60)
                if result.returncode == 0:
                    report = json.loads(capture.with_suffix(".parquet.json").read_text())
                    probe.validate_capture(report["record"])
                    assert report["terminal_ok"] and all(e["ok"] for e in report["events"])
                    row["record"] = report["record"]
                save()
                print(json.dumps(dict(name=name, exit=result.returncode)), flush=True)
                if row["status"]["fault_count"]:
                    raise RuntimeError("fault observed; stopped evidence exported before recovery")
                if result.returncode:
                    raise RuntimeError("maneuver failed; capture and stopped Hall evidence retained")
        (args.output/"config-after.json").write_text(probe.run(["just", "cli-bridge", "config", "dump"]).stdout)
    finally:
        try:
            probe.run(["just", "cli-bridge", "stop"])
        finally:
            probe.run(probe.device("download", "--connect-under-reset", "--verify", str(args.recovery_elf)), timeout=90)
            probe.run(probe.device("reset"))
            manifest["restored"] = json.loads(probe.run(["target/release/oxifoc-host-cli", "--elf", str(args.recovery_elf), "--wait-secs", "12", "--json", "status"]).stdout)
            save()

if __name__ == "__main__":
    main()
