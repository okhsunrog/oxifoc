"""Run existing MK5 embedded-test benchmarks and restore the controller image."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

import bench_overclock as probe

ROOT = Path(__file__).resolve().parents[1]
CRATE = ROOT / "tests/stm32f405"
PORT = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_3C:DC:75:8A:FF:10-if00"
ROW = re.compile(r"BENCHV1 name=(.*?) calls=(\d+) rounds=(\d+) min=(\d+) max=(\d+) total=(\d+)")
CONTEXT = re.compile(r"SCHEDULEV1 fraction_q32=(\d+) irq_masked=true seed=(\d+)")


def decode(log):
    """Keep gross batch timing; do not turn loop-subtracted deltas into WCET."""
    rows, context = [], None
    for line in log.splitlines():
        if match := CONTEXT.search(line):
            context = {"fraction_q32": int(match[1]), "seed": int(match[2])}
        elif "wrap input: " in line:
            context = {"wrap_input": line.split("wrap input: ", 1)[1].strip()}
        elif match := ROW.search(line):
            name, calls, rounds, minimum, maximum, total = match.groups()
            calls, rounds, minimum, maximum, total = map(int, (calls, rounds, minimum, maximum, total))
            if calls <= 0 or rounds <= 0 or not 0 < minimum <= maximum:
                raise ValueError("invalid DWT batch")
            if not minimum * rounds <= total <= maximum * rounds:
                raise ValueError("inconsistent DWT total")
            rows.append(dict(context=context, name=name, calls=calls, rounds=rounds,
                batch_min_cycles=minimum, batch_max_cycles=maximum, total_cycles=total,
                mean_cycles_per_call=total / (calls * rounds)))
    if not rows:
        raise ValueError("no BENCHV1 measurements in embedded-test log")
    return rows


def save(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--recovery-elf", type=Path, required=True)
    parser.add_argument("--filter", choices=["z_bench_schedule", "z_bench_wrap_angle", "z_bench_hall_single_turn", "z_bench_brake", "deferred_logging"], default="z_bench_schedule")
    parser.add_argument("--repeats", type=int, default=3, choices=range(1, 6))
    parser.add_argument("--serial-path", default=PORT)
    parser.add_argument("--features", default="")
    parser.add_argument("--probe-speed", type=int, default=1000)
    args = parser.parse_args()
    device = probe.device
    probe.device = lambda command, *rest: device(command, "--speed", str(args.probe_speed), *rest)
    test_target = "logging" if args.filter == "deferred_logging" else "foc"
    recovery = args.recovery_elf.resolve(strict=True)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = {**os.environ, "CARGO_INCREMENTAL": "0", "CARGO_PROFILE_RELEASE_INCREMENTAL": "false",
           "CARGO_PROFILE_RELEASE_CODEGEN_UNITS": "1"}
    manifest = dict(schema=1, filter=args.filter, features=args.features, incremental=False, codegen_units=1,
        opt_level=3, lto="fat", recovery_elf=str(recovery), recovery_sha256=sha(recovery),
        commit=probe.run(["git", "rev-parse", "HEAD"]).stdout.strip(),
        dirty_files=probe.run(["git", "status", "--porcelain"]).stdout.splitlines(), runs=[])
    manifest["probe_speed_khz"] = args.probe_speed
    paths = probe.run(["git", "ls-files", "--cached", "--others", "--exclude-standard",
        "tests/stm32f405", "oxifoc-core", "vendor/rtt-target"]).stdout.splitlines()
    manifest["source_sha256"] = {name: sha(ROOT / name) for name in paths if (ROOT / name).is_file()}
    for label, command in [("rustc", ["rustc", "-Vv"]), ("probe_rs", ["probe-rs", "--version"])]:
        manifest[label] = subprocess.check_output(command, cwd=CRATE, text=True).strip()
    # Keep normal crate build/cache paths. No source copies or patch snapshots.
    with (output / "build.jsonl").open("w") as stdout, (output / "build.log").open("w") as stderr:
        subprocess.run(["cargo", "test", "-vv", "--release", "--locked", "--test", test_target,
            "--no-run", "--message-format=json", *(["--features", args.features] if args.features else [])], cwd=CRATE, env=env,
            stdout=stdout, stderr=stderr, check=True, timeout=600)
    # cargo -vv also forwards prefixed build-script output to stdout when a
    # script reruns. Only Cargo JSON records start with an opening brace.
    artifacts = [json.loads(line) for line in (output / "build.jsonl").read_text().splitlines()
                 if line.startswith("{")]
    built = next(Path(row["executable"]) for row in artifacts
                 if row.get("reason") == "compiler-artifact" and row.get("executable")
                 and row["target"]["name"] == test_target)
    image = output / "benchmark.elf"
    shutil.copyfile(built, image)
    manifest["benchmark_sha256"] = sha(image)
    (output / "symbols.txt").write_text(probe.run(["rust-nm", "--defined-only", str(image)]).stdout)

    def status():
        data = json.loads(probe.run([str(ROOT / "target/release/oxifoc-host-cli"), "--transport",
            "usb-serial-jtag", "--serial-path", args.serial_path, "--elf", str(recovery),
            "--wait-secs", "12", "--json", "status"], timeout=22).stdout)
        assert data["motor_state"] == "Stopped" and data["fault_count"] == 0, data
        assert 23000 <= data["vbus_mv"] <= 25000 and data["fet_temp_c_x10"] < 500, data
        return data

    manifest["before"] = status()
    # CLI is now idle. Flashing/running only happens within this restore guard.
    try:
        for repeat in range(args.repeats):
            log = output / f"run-{repeat}.log"
            with log.open("w") as stream:
                result = subprocess.run(probe.device("run", "--connect-under-reset", str(image),
                    args.filter, "--nocapture"), cwd=ROOT, stdout=stream,
                    stderr=subprocess.STDOUT, timeout=90)
            # embedded-test retains the latch and disables the gate driver even
            # after the test returns. Read these only after the timed windows.
            pins = dict(tim1_ccer=probe.read(0x40010020)[0],
                        gpiob_odr=probe.read(0x40020414)[0], gpioc_odr=probe.read(0x40020814)[0])
            assert pins["tim1_ccer"] & 0x555 == 0 and pins["gpiob_odr"] & (1 << 5) == 0
            assert pins["gpioc_odr"] & (1 << 5)
            registers = dict(rcc=probe.read(0x40023800, 3),
                flash_acr=probe.read(0x40023c00)[0], tim3_psc_arr=probe.read(0x40000428, 2))
            pll = registers["rcc"][1]
            assert pll & 63 == 4 and (pll >> 6) & 511 == 168 and (pll >> 16) & 3 == 0
            assert registers["flash_acr"] & 0x700 == 0x700
            assert registers["tim3_psc_arr"] == [83, 65535]
            result.check_returncode()
            rows = decode(log.read_text())
            if args.filter == "z_bench_schedule":
                assert len(rows) == 60
                assert {row["context"]["fraction_q32"] for row in rows} == {0, 4_200_000}
                assert {row["context"]["seed"] for row in rows} == {0, 0x12345600}
                assert all(row["rounds"] == 32 and row["calls"] == 64 for row in rows)
            manifest["runs"].append(dict(repeat=repeat, gate_readback=pins, registers=registers, rows=rows))
            save(output / "manifest.json", manifest)
            print(f"embedded-test repeat {repeat + 1}/{args.repeats}: {len(rows)} batches summaries", flush=True)
    finally:
        # The test image has no host command server. Recover directly under reset.
        with (output / "restore.log").open("w") as log:
            subprocess.run(probe.device("download", "--connect-under-reset", "--verify", str(recovery)),
                cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=90)
            subprocess.run(probe.device("reset"), cwd=ROOT, stdout=log,
                stderr=subprocess.STDOUT, check=True, timeout=30)
        time.sleep(1.5)
        manifest["restored"] = status()
        manifest["restored_tim1_ccer"] = probe.read(0x40010020)[0]
        assert manifest["restored_tim1_ccer"] & 0x555 == 0
        save(output / "manifest.json", manifest)
        print("controller image restored; Stopped, zero faults", flush=True)


if __name__ == "__main__":
    main()
