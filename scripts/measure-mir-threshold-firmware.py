"""Compare full STM32 firmware sizes at the default/global/HAL-only MIR hint threshold.

Run with uv run. --source must be a prepared worktree with the October nightly
and vfp2sp configuration patch applied; no firmware source is changed here.
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
parser.add_argument("--variant", choices=["default", "global115", "embassy115"], required=True)
parser.add_argument("--board", choices=["g474", "cf2", "vesc6-mk5"], required=True)
args = parser.parse_args()
source = args.source.resolve()
root = args.output.resolve()
root.mkdir(parents=True, exist_ok=True)
crate = "oxifoc-g474" if args.board == "g474" else "oxifoc-f405"
label = f"{args.variant}-{args.board}"
target = root / f"target-{args.variant}" / args.board
env = os.environ.copy()
for name in ("CARGO_ENCODED_RUSTFLAGS", "RUSTC_WRAPPER", "RUSTC_WORKSPACE_WRAPPER", "RUSTC",
             "CARGO_INCREMENTAL", "CARGO_PROFILE_RELEASE_OPT_LEVEL"):
    env.pop(name, None)
env.update(TMPDIR=str(root), RUSTUP_TOOLCHAIN="nightly-2026-10-14",
           CARGO_BUILD_BUILD_DIR=str(root / "build-default"),
           RUSTFLAGS="-C target-feature=+vfp2sp")
if args.variant == "global115":
    env["RUSTFLAGS"] += " -Zinline-mir-hint-threshold=115"
command = ["cargo", "build", "--release", "--locked", "--target-dir", str(target)]
if args.variant == "embassy115":
    command += ["-Zprofile-rustflags", "--config",
                'profile.release.package.embassy-stm32.rustflags=["-Zinline-mir-hint-threshold=115"]']
if args.board == "vesc6-mk5":
    command += ["--no-default-features", "--features",
                "transport-usb,transport-uart,board-vesc6-mk5"]
start = time.monotonic()
with (root / f"{label}.log").open("w") as log:
    subprocess.run(command, cwd=source / crate, env=env,
                   stdout=log, stderr=subprocess.STDOUT, check=True)
elapsed = time.monotonic() - start
elf = root / f"{label}.elf"
shutil.copyfile(target / "thumbv7em-none-eabihf/release" / crate, elf)
sections = {}
for line in output("arm-none-eabi-size", "-A", str(elf)).splitlines():
    match = re.match(r"(\.\S+)\s+(\d+)\s+\d+", line)
    if match:
        sections[match[1]] = int(match[2])
symbols = output("arm-none-eabi-nm", "-C", "-S", str(elf))
spi_sizes = []
for line in symbols.splitlines():
    if "ExclusiveDevice<" in line and "SpiDevice>::transfer" in line:
        spi_sizes.append(int(line.split()[1], 16))
disasm = output("arm-none-eabi-objdump", "-d", "--no-show-raw-insn", str(elf))
(root / f"{label}.asm").write_text(disasm)
instructions = collections.Counter()
for line in disasm.splitlines():
    match = re.match(r"\s*[0-9a-f]+:\s+(\S+)", line)
    if match:
        instructions[match[1]] += 1
assert not any(".f64" in name for name in instructions), "unexpected double-precision instruction"
assert "RSQRT_TAB" not in symbols and not re.search(r"\blibm::[^\n]*\bsqrtf\b", symbols), "unexpected software sqrt"
if args.board != "g474":
    assert instructions["vsqrt.f32"] > 0, "hardware sqrt missing"
result = dict(variant=args.variant, board=args.board,
              source_commit=output("git", "-C", str(source), "rev-parse", "HEAD").strip(),
              source_diff_sha256=hashlib.sha256(output("git", "-C", str(source), "diff").encode()).hexdigest(),
              command=command, rustflags=env["RUSTFLAGS"], incremental="manifest default (true)",
              flash=sum(sections.get(s, 0) for s in (".vector_table", ".text", ".rodata", ".data")),
              static_ram=sum(sections.get(s, 0) for s in (".data", ".bss")),
              sections={s: sections.get(s, 0) for s in (".vector_table", ".text", ".rodata", ".data", ".bss")},
              spi_transfer_bytes=spi_sizes, hardware_sqrt_sites=instructions["vsqrt.f32"],
              elapsed_seconds=round(elapsed, 3))
(root / f"{label}.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2), flush=True)
