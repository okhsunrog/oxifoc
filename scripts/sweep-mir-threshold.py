"""Measure a firmware build with global/per-package MIR hint thresholds (uv run).

Use a fixed worktree with existing package overrides removed for a controlled
sweep. Thresholds are set via Cargo profiles; target CPU/FP flags stay intact.
"""

import argparse
import collections
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time


def output(*command, **kwargs):
    return subprocess.check_output(command, text=True, **kwargs)


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--source", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--board", choices=["g474", "cf2", "vesc6-mk5", "bridge", "remote"], required=True)
parser.add_argument("--global-threshold", type=int)
parser.add_argument("--profile-config", type=Path, help="additional native Cargo config preset")
parser.add_argument("--package", action="append", default=[], metavar="NAME=THRESHOLD")
parser.add_argument("--label", required=True)
parser.add_argument("--no-incremental", action="store_true", help="separate diagnostic: enable normal MIR inlining in local crates")
parser.add_argument("--no-incremental-package", action="append", default=[], metavar="NAME")
args = parser.parse_args()
packages = dict(x.split("=", 1) for x in args.package)
packages = {name: int(value) for name, value in packages.items()}
assert all(re.fullmatch(r"[\w-]+", name) for name in packages)
assert all(x >= 0 for x in [*packages.values(), args.global_threshold or 0])
assert re.fullmatch(r"[\w-]+", args.label)
source = args.source.resolve()
root = args.output.resolve()
case = root / "cases" / f"{args.board}-{args.label}"
case.mkdir(parents=True, exist_ok=True)
crate = {"g474": "oxifoc-g474", "cf2": "oxifoc-f405", "vesc6-mk5": "oxifoc-f405",
         "bridge": "oxifoc-bridge", "remote": "oxifoc-remote"}[args.board]
arm = args.board in ("g474", "cf2", "vesc6-mk5")
target = "thumbv7em-none-eabihf" if arm else "riscv32imac-unknown-none-elf"
identity = dict(board=args.board, global_threshold=args.global_threshold,
                packages=packages,
                source_commit=output("git", "-C", str(source), "rev-parse", "HEAD").strip(),
                source_diff_sha256=hashlib.sha256(output("git", "-C", str(source), "diff").encode()).hexdigest())
if args.profile_config:
    identity["profile_config_sha256"] = hashlib.sha256(args.profile_config.read_bytes()).hexdigest()
if args.no_incremental:
    identity["incremental"] = False
if args.no_incremental_package:
    assert all(re.fullmatch(r"[\w-]+", name) for name in args.no_incremental_package)
    identity["nonincremental_packages"] = sorted(args.no_incremental_package)
result_file = case / "result.json"
if result_file.exists():
    previous = json.loads(result_file.read_text())
    assert all(previous[k] == v for k, v in identity.items()), "case label already used for different inputs"
    print(json.dumps(previous), flush=True)
    raise SystemExit(0)
env = os.environ.copy()
for name in list(env):
    if name.startswith("CARGO_PROFILE_") or name in (
        "CARGO_ENCODED_RUSTFLAGS", "RUSTFLAGS", "RUSTUP_TOOLCHAIN", "RUSTC_WRAPPER",
        "RUSTC_WORKSPACE_WRAPPER", "RUSTC", "CARGO_INCREMENTAL"):
        env.pop(name)
env.update(TMPDIR=str(root), CARGO_BUILD_BUILD_DIR=str(root / "build"))
if args.no_incremental:
    env["CARGO_INCREMENTAL"] = "0"
command = ["cargo", "build", "--release", "--locked", "-j", "4", "-Zprofile-rustflags",
           "--target-dir", str(root / "target" / args.board)]
if args.profile_config:
    command += ["--config", str(args.profile_config.resolve())]
if args.global_threshold is not None:
    command += ["--config", f'profile.release.rustflags=["-Zinline-mir-hint-threshold={args.global_threshold}"]']
for name, threshold in sorted(packages.items()):
    command += ["--config", f'profile.release.package.{name}.rustflags=["-Zinline-mir-hint-threshold={threshold}"]']
for name in sorted(args.no_incremental_package):
    command += ["--config", f"profile.release.package.{name}.incremental=false"]
if args.board == "vesc6-mk5":
    command += ["--no-default-features", "--features", "transport-usb,transport-uart,board-vesc6-mk5"]
start = time.monotonic()
with (case / "build.log").open("w") as log:
    subprocess.run(command, cwd=source / crate, env=env, stdout=log,
                   stderr=subprocess.STDOUT, check=True)
elapsed = time.monotonic() - start
elf = case / "firmware.elf"
shutil.copyfile(root / "target" / args.board / target / "release" / crate, elf)
prefix = "arm-none-eabi-" if arm else "llvm-"
symbols = output(prefix + "nm", "-C", "-S", str(elf))
(case / "symbols.txt").write_text(symbols)
asm = output(prefix + "objdump", "-d", "-C", "--no-show-raw-insn", str(elf))
(case / "firmware.asm").write_text(asm)
sections = {}
for line in output(prefix + "size", "-A", str(elf)).splitlines():
    match = re.match(r"(\.\S+)\s+(\d+)\s+\d+", line)
    if match:
        sections[match[1]] = int(match[2])
size_values = output(prefix + "size", str(elf)).splitlines()[1].split()
image_bytes = int(size_values[0]) + int(size_values[1])
instructions = collections.Counter()
for line in asm.splitlines():
    match = re.match(r"\s*[0-9a-f]+:\s+(\S+)", line)
    if match:
        instructions[match[1]] += 1
if arm:
    assert not any(".f64" in x for x in instructions), "unsupported double-precision FP"
    assert not re.search(r"\blibm::[^\n]*\bsqrtf\b|RSQRT_TAB", symbols), "software sqrt"
    if args.board != "g474":
        assert instructions["vsqrt.f32"] > 0, "hardware sqrt missing"
critical = {}
for line in symbols.splitlines():
    match = re.match(r"([0-9a-f]+)\s+([0-9a-f]+)\s+[tT]\s+(.+)", line)
    if not match:
        continue
    name = match[3]
    if name == "ADC" or name.endswith("::__cortex_m_rt_ADC") or ("FocDriver<" in name and any(x in name for x in (
        "::step_current_control", "::step_direct_voltage", "::step_velocity_control",
        "::update_phase_with_applied_voltage"))) or ("ExclusiveDevice<" in name and "SpiDevice>::transfer" in name):
        address, size = int(match[1], 16), int(match[2], 16)
        calls = []
        for insn in asm.splitlines():
            m = re.match(r"\s*([0-9a-f]+):\s+(blx?(?:\.w)?)\s+(.+)", insn)
            if m and address <= int(m[1], 16) < address + size:
                calls.append(m[3])
        critical[name] = dict(bytes=size, calls=calls)
result = identity | dict(label=args.label, command=command, sections=sections,
    image_bytes=image_bytes, static_ram=int(size_values[1])+int(size_values[2]),
    instructions=dict(instructions), critical_symbols=critical,
    elf_sha256=hashlib.sha256(elf.read_bytes()).hexdigest(),
    rustc=output("rustc", "-vV", cwd=source / crate, env=env),
    elapsed_seconds=round(elapsed, 3))
result_file.write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps({k: result[k] for k in ("label", "board", "image_bytes", "static_ram", "elapsed_seconds")}), flush=True)
