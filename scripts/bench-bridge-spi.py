"""Run the finite, motor-inhibited STM32/C5 SPI DMA probe and check both ends."""

import argparse
import json
from pathlib import Path
import re
import signal
import struct
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
STM_PROBE = "0483:3754:002300203234511537333934"
C5_PROBE = "303a:1001:3C:DC:75:8A:FF:10"
STM_ELF = ROOT / "oxifoc-f405/target/vesc6-mk5/thumbv7em-none-eabihf/release/bridge-spi-probe"


def command(*args, **kwargs):
    return subprocess.run(args, check=True, **kwargs)


def read_words(address, count, output):
    command("probe-rs", "read", "--chip", "STM32F405RGTx", "--probe", STM_PROBE,
            "b32", hex(address), str(count), "--format", "binary", "--output", str(output),
            capture_output=True)
    return struct.unpack(f"<{count}I", output.read_bytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--c5-project", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--runs", type=int, default=3)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    c5_elf = args.c5_project / "target/riscv32imac-unknown-none-elf/release/spi_probe"
    command("just", "build-spi-probe", cwd=args.c5_project)
    symbols = {}
    for line in command("nm", str(STM_ELF), capture_output=True, text=True).stdout.splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[2].startswith("SPI_BENCH_"):
            symbols[fields[2]] = int(fields[0], 16)
    command("probe-rs", "download", "--chip", "STM32F405RGTx", "--probe", STM_PROBE,
            "--verify", str(STM_ELF))
    results = []
    for run in range(args.runs):
        prefix = args.output / f"run-{run + 1}"
        command("probe-rs", "reset", "--chip", "STM32F405RGTx", "--probe", STM_PROBE)
        log_path = prefix.with_suffix(".c5.log")
        with log_path.open("w") as log:
            process = subprocess.Popen([
                "probe-rs", "run", "--chip", "esp32c5", "--probe", C5_PROBE,
                "--preverify", "--no-location", "--no-timestamps",
                "--rtt-channel-mode", "no-block-skip", str(c5_elf)
            ], cwd=args.c5_project, stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 45
                while "SPI DMA complete:" not in log_path.read_text():
                    text = log_path.read_text()
                    if "SPI timeout" in text or process.poll() is not None:
                        raise RuntimeError(f"C5 probe failed; see {log_path}")
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f"SPI probe timed out; see {log_path}")
                    time.sleep(0.2)
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
        state = read_words(symbols["SPI_BENCH_STATE"], 1, prefix.with_suffix(".state.bin"))[0]
        words = read_words(symbols["SPI_BENCH_RESULTS"], 26 * 8, prefix.with_suffix(".stm.bin"))
        rows = []
        for case in range(26):
            hz, length, frames, errors, hal_errors, elapsed, dma_us, bad = words[case * 8:case * 8 + 8]
            if state != 3 or frames != (10000 if case == 25 else 16) or errors or hal_errors or bad:
                raise RuntimeError(f"STM32 probe failed: run={run + 1} case={case} state={state}")
            rows.append(dict(case=case, hz=hz, length=length, frames=frames, errors=errors,
                             elapsed_us=elapsed, dma_us=dma_us, includes_startup_wait=(case == 0),
                             payload_Bps=length * frames * 1e6 / elapsed if case else None))
        c5_rows = re.findall(
            r"spi: case=(\d+) len=(\d+) frames=(\d+) errors=(\d+) length_errors=(\d+) received=(\d+) first_bad=(\d+) elapsed_us=(\d+)",
            log_path.read_text())
        if len(c5_rows) != 26:
            raise RuntimeError(f"Incomplete C5 result: {log_path}")
        for case, values in enumerate(c5_rows):
            i, length, frames, errors, length_errors, received, bad, elapsed = map(int, values)
            row = rows[case]
            if (i, length, frames, errors, length_errors, received, bad) != (
                    case, row["length"], row["frames"], 0, 0, row["length"], 0):
                raise RuntimeError(f"C5 integrity check failed: {log_path} case={case}")
            row["c5_elapsed_us"] = elapsed
        prefix.with_suffix(".json").write_text(json.dumps(rows, indent=2) + "\n")
        results.append(rows)
        print(f"Run {run + 1}: 10400 full-duplex frames, no errors; "
              f"512-byte stress {rows[-1]['payload_Bps'] / 1e6:.3f} MB/s per direction", flush=True)
    args.output.joinpath("summary.json").write_text(json.dumps(results, indent=2) + "\n")


if __name__ == "__main__":
    main()
