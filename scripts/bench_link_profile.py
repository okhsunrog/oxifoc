"""Repeat a finite MK5 maneuver and export exclusive task/IRQ CPU windows.

Run under the project dev shell after flashing the matching trace image.
The probe is used before excitation and after Stop, never inside the timed
window. Failed captures remain in the report; they are not successful repeats.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("link", choices=["bridge", "usb"])
    parser.add_argument("maneuver", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--elf", type=Path, default=Path("oxifoc-f405/target/vesc6-mk5/thumbv7em-none-eabihf/release/oxifoc-f405"))
    parser.add_argument("--source", default="ergot-raw")
    parser.add_argument("--encoding", default="packed28")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--active-delay-ms", type=int, default=1200)
    parser.add_argument("--timer-hz", type=int, default=83917920)
    args = parser.parse_args()
    if not 1 <= args.repeats <= 10 or args.active_delay_ms < 0:
        parser.error("use 1..10 repeats and nonnegative delay")
    args.out.mkdir(parents=True, exist_ok=False)
    elf = args.out / "firmware.elf"
    shutil.copyfile(args.elf, elf)
    symbols = args.out / "firmware.nm"
    symbols.write_bytes(subprocess.check_output(["rust-nm", "-n", "-S", "-C", str(elf)]))
    (args.out / "source.patch").write_bytes(subprocess.check_output(["git", "diff", "HEAD"]))
    manifest = {
        "elf_sha256": hashlib.sha256(elf.read_bytes()).hexdigest(),
        "commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "link": args.link, "source": args.source, "encoding": args.encoding,
        "maneuver": json.loads(args.maneuver.read_text()), "repeats": [],
    }
    trace = [sys.executable, "-B", "scripts/bench_executor_trace.py"]
    common = ["--symbols", str(symbols.resolve()), "--timer-hz", str(args.timer_hz)]
    for i in range(args.repeats):
        capture = args.out / f"run-{i}.parquet"
        subprocess.run(trace + ["arm", *common, "--on-active", "--active-delay-ms",
                               str(args.active_delay_ms)], check=True, timeout=15)
        # test-link has an independent terminal Stop, including failure paths.
        result = subprocess.run(["just", "test-link", args.link, str(capture),
                                 str(args.maneuver), args.source, args.encoding], timeout=90)
        binary = args.out / f"run-{i}.trace.bin"
        exported = subprocess.run(trace + ["read", *common, "--out", str(binary),
                                          "--elf", str(elf), "--capture", str(capture),
                                          "--schema", "finite-trace-v2"], timeout=15)
        cpu = args.out / f"run-{i}.cpu.json"
        analyzed = None
        if exported.returncode == 0:
            analyzed = subprocess.run([sys.executable, "-B", "scripts/bench_trace_analysis.py",
                                       str(binary), "--task-cpu", "--out", str(cpu)], timeout=15)
        manifest["repeats"].append({
            "capture": str(capture), "capture_exit": result.returncode,
            "trace_exit": exported.returncode,
            "cpu_exit": analyzed.returncode if analyzed is not None else None,
        })
        (args.out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return int(any(r["capture_exit"] or r["trace_exit"] or r["cpu_exit"]
                   for r in manifest["repeats"]))


if __name__ == "__main__":
    raise SystemExit(main())
