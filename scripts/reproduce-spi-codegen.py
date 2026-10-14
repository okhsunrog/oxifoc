"""Build the isolated SPI regression: uv run scripts/reproduce-spi-codegen.py --help."""

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess


def run(*args, **kwargs):
    return subprocess.check_output(args, text=True, **kwargs)


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("toolchain", help="e.g. nightly-2026-07-28")
parser.add_argument("--features", default="+vfp2,-fp64", help="use = for a leading minus")
parser.add_argument("--lto", choices=["fat", "thin", "off"], default="fat")
parser.add_argument("--disable-pass-in", metavar="CRATE")
parser.add_argument("--mir-hint-threshold", type=int, help="embassy_stm32 only")
parser.add_argument("--output", type=Path,
                    default=Path("/home/okhsunrog/tmp_zfs/codex_tmp/spi-codegen/reproduced"))
args = parser.parse_args()
if args.disable_pass_in and args.mir_hint_threshold is not None:
    parser.error("choose one compiler diagnostic per invocation")

repo = Path(__file__).resolve().parents[1]
example = repo / "docs/experiments/spi-codegen"
label = f"{args.toolchain}-{args.lto}"
diagnostic = None
if args.disable_pass_in:
    label += f"-disable-{args.disable_pass_in}"
    diagnostic = "-Zmir-enable-passes=-SimplifyComparisonIntegral"
if args.mir_hint_threshold is not None:
    label += f"-hint-{args.mir_hint_threshold}"
    diagnostic = f"-Zinline-mir-hint-threshold={args.mir_hint_threshold}"
out = (args.output / label).resolve()
out.mkdir(parents=True, exist_ok=True)
env = os.environ.copy()
# Override this host's global build-dir setting as well as target-dir.
env.update(TMPDIR=str(out), CARGO_BUILD_BUILD_DIR=str(out / "build"),
           RUSTFLAGS=f"-C target-feature={args.features}")
for name in ("CARGO_ENCODED_RUSTFLAGS", "RUSTC_WRAPPER", "RUSTC_WORKSPACE_WRAPPER", "RUSTC"):
    env.pop(name, None)
if diagnostic:
    env.update(RUSTC_WRAPPER=str(example / "rustc-wrapper.sh"),
               SPI_DIAGNOSTIC_CRATE=args.disable_pass_in or "embassy_stm32",
               SPI_DIAGNOSTIC_FLAG=diagnostic)
command = ["cargo", f"+{args.toolchain}", "rustc", "--locked",
           "--manifest-path", str(example / "repro/Cargo.toml"),
           "--target", "thumbv7em-none-eabihf", "--target-dir", str(out / "target"),
           "--release", "--", "--emit=asm,llvm-ir", "-C", f"lto={args.lto}"]
with (out / "build.log").open("w") as log:
    subprocess.run(command, cwd=out, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
for suffix in ("s", "ll"):
    artifact = max((out / "build/thumbv7em-none-eabihf/release").rglob(
        f"spi_regression-*.{suffix}"), key=lambda p: p.stat().st_mtime_ns)
    shutil.copyfile(artifact, out / f"spi-transfer.{suffix}")
sysroot = Path(run("rustc", f"+{args.toolchain}", "--print", "sysroot").strip())
host = re.search(r"^host: (.+)$", run("rustc", f"+{args.toolchain}", "-vV"), re.M)[1]
linker = sysroot / f"lib/rustlib/{host}/bin/rust-lld"
archive = out / "target/thumbv7em-none-eabihf/release/libspi_regression.a"
elf = out / "spi-transfer.elf"
subprocess.run([str(linker), "-flavor", "gnu", "-m", "armelf", "--gc-sections",
                "-e", "spi_transfer", "-Ttext=0x08000000", str(archive), "-o", str(elf)],
               env=env, check=True)
symbols = run("arm-none-eabi-nm", "-S", str(elf))
size = int(re.search(r"^[0-9a-f]+ ([0-9a-f]+) T spi_transfer$", symbols, re.M)[1], 16)
text, data, bss = map(int, run("arm-none-eabi-size", str(elf)).splitlines()[1].split()[:3])
result = dict(toolchain=args.toolchain, rustc=run("rustc", f"+{args.toolchain}", "-vV"),
              target="thumbv7em-none-eabihf", features=args.features, lto=args.lto,
              diagnostic_crate=env.get("SPI_DIAGNOSTIC_CRATE") if diagnostic else None,
              diagnostic_flag=diagnostic, spi_transfer_bytes=size,
              linked_text_bytes=text, linked_data_bytes=data, linked_bss_bytes=bss)
(out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
