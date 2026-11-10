"""Motor-inhibited MK5 clock sweep; always restore a known normal image.

Run via just bench-mk5-overclock. A failed stage stops the sweep. This is a
short functional screen, not a voltage/temperature-qualified operating limit.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time

PROBE = "0483:3754:002300203234511537333934"
ELF = Path("oxifoc-f405/target/vesc6-mk5/thumbv7em-none-eabihf/release/oxifoc-f405")
def device(command, *args):
    return ["probe-rs", command, "--chip", "STM32F405RGTx", "--probe", PROBE, *args]
REPORT = re.compile(r"oc: cpu=(\d+) tim1=(\d+) checks=(\d+) die_x10=(-?\d+)")
FAILURE = re.compile(r"PANIC|HardFault|oc: .*failure|oc: .*timeout|oc: .*limit")


def run(args, timeout=30):
    return subprocess.run(args, capture_output=True, text=True, timeout=timeout, check=True)


def read(address, words=1):
    output = run(device("read", "b32", hex(address), str(words))).stdout
    return [int(x, 16) for line in output.splitlines() if ":" in line
            for x in line.split(":", 1)[1].split()]


def clocks():
    deadline = time.monotonic() + 5
    while True:
        cr, pll, cfgr = read(0x40023800, 3)
        if cr & (1 << 25) and (cfgr >> 2) & 3 == 2:
            break
        assert time.monotonic() < deadline, f"PLL not locked/selected: RCC={cr:08x},{pll:08x},{cfgr:08x}"
        time.sleep(.1)
    hse = 7_944_893  # measured estimate for the UID checked by firmware
    sys = (hse // (pll & 63)) * ((pll >> 6) & 511) // (2 * (((pll >> 16) & 3) + 1))
    hpre = (cfgr >> 4) & 15
    hclk = sys // ([1] * 8 + [2, 4, 8, 16, 64, 128, 256, 512])[hpre]
    def apb(shift):
        bits = (cfgr >> shift) & 7
        return hclk // (1 if bits < 4 else 1 << (bits - 3))
    p1, p2 = apb(10), apb(13)
    flash = read(0x40023c00)[0]
    adcpre = (read(0x40012304)[0] >> 16) & 3
    adc_hz = p2 // (2 * (adcpre + 1))
    spi_hz = p2 // (1 << (((read(0x40013000)[0] >> 3) & 7) + 1))
    psc, arr = read(0x40010028, 2)
    timer = p2 * (1 if (cfgr >> 13) & 7 < 4 else 2)
    hz = timer / ((psc + 1) * 2 * arr)
    assert p1 <= 42_000_000 and p2 <= 84_000_000 and adc_hz <= 36_000_000
    assert flash & 7 == min(7, (hclk - 1) // 30_000_000), "FLASH wait state mismatch"
    assert read(0x40010020)[0] & 0x555 == 0, "phase output enabled"
    assert 19_800 <= hz <= 20_200, "PWM/ADC rate changed"
    return dict(cpu_hz=hclk, apb1_hz=p1, apb2_hz=p2, adc_hz=adc_hz,
                spi_hz=spi_hz, tim1_hz=timer, pwm_hz=hz, flash_ws=flash & 7,
                flash_access_hz=hclk / ((flash & 7) + 1), rcc=[cr, pll, cfgr])


def validate_capture(meta):
    raw = meta["raw_telemetry"]
    assert meta["gaps"] == meta["samples_lost"] == 0, "telemetry samples missing"
    assert raw["ended"] and raw["error"] is None, "incomplete telemetry session"
    assert raw["host_drops"] == raw["producer_drops"] == 0, "telemetry queue overflow"
    assert raw["records_received"] == raw["records_delivered"] == raw["end_total_records"] == meta["rows"]


def placement(image, memory, folder):
    symbols = run(["rust-nm", "--print-size", "--demangle", str(image)]).stdout
    (folder / "symbols.txt").write_text(symbols)
    result = {}
    for line in symbols.splitlines():
        fields = line.split(maxsplit=3)
        if len(fields) != 4:
            continue
        address, size, _, name = fields
        if name == "ADC" or name in ["oxifoc_f405::STATE", "oxifoc_f405::control::foc::FOC_DRIVER"] or ("run_foc_cycle::<" in name and "Thunk" not in name):
            result[name] = dict(address=int(address, 16), size=int(size, 16))
    assert len(result) == 4, "missing placement symbols"
    for name, symbol in result.items():
        if name == "ADC" or "run_foc_cycle::<" in name:
            origin = 0x20000000 if memory.startswith("sram") else 0x08000000
            length = 128 * 1024 if memory.startswith("sram") else 768 * 1024
        else:
            origin = 0x10000000 if "ccm" in memory else 0x20000000
            length = 64 * 1024 if "ccm" in memory else 128 * 1024
        assert origin <= symbol["address"] < origin + length, f"unexpected placement: {name}"
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--steps", type=int, nargs="+", default=[168, 180, 192, 204, 216, 228, 240])
    parser.add_argument("--seconds", type=int, default=20)
    parser.add_argument("--recovery-elf", type=Path, help="reuse the known normal recovery ELF from an earlier sweep")
    parser.add_argument("--load", choices=["stress", "light"], default="stress")
    parser.add_argument("--current-maneuver", type=Path, help="run two inhibited-output current maneuvers after recording")
    parser.add_argument("--memory", choices=["flash", "ccm", "sram", "sram-ccm"], default="flash")
    args = parser.parse_args()
    assert 10 <= args.seconds <= 60
    assert args.steps == sorted(set(args.steps)) and all(x in [168, 180, 192, 204, 216, 228, 240, 252, 260] for x in args.steps)
    args.output.mkdir(parents=True, exist_ok=False)
    stock = (args.output / "recovery.elf").resolve()
    shutil.copyfile(args.recovery_elf or ELF, stock)  # normal recipe builds stock first
    (args.output / "source.patch").write_text(run(["git", "diff", "HEAD"]).stdout)
    summary = dict(recovery_sha256=hashlib.sha256(stock.read_bytes()).hexdigest(), load=args.load, memory=args.memory, stages=[])
    logger = None
    try:
        clocks()  # Verify probe access/command syntax before any flash mutation.
        for mhz in args.steps:
            stage = dict(requested_mhz=mhz, passed=False)
            summary["stages"].append(stage)
            folder = args.output / str(mhz)
            folder.mkdir()
            with (folder / "flash.log").open("w") as log:
                subprocess.run(["just", "flash-mk5-overclock", str(mhz), args.load, args.memory], stdout=log,
                               stderr=subprocess.STDOUT, timeout=600, check=True)
            image = (folder / "firmware.elf").resolve()
            shutil.copyfile(ELF, image)
            stage["elf_sha256"] = hashlib.sha256(image.read_bytes()).hexdigest()
            stage["placement"] = placement(image, args.memory, folder)
            stage["clocks"] = clocks()
            info = json.loads(run(["just", "cli-bridge", "info"]).stdout)
            assert info["uuid"] == "2B002C001047343031363330" and "+outputs-inhibited" in info["sw"]
            with (folder / "rtt.log").open("w") as log:
                logger = subprocess.Popen(device("attach", str(image)), stdout=log, stderr=subprocess.STDOUT)
                # Let the initial SWD attach/DWT rearm finish before measuring.
                time.sleep(2)
                started = time.monotonic()
                command = ["just", "cli-bridge", "record", "--seconds", str(args.seconds),
                           "--fast-hz", "20000", "--fast-source", "ergot-raw", "--fast-encoding", "packed28",
                           "--out", str(folder / "capture.parquet")]
                with (folder / "capture.json").open("w") as out, (folder / "capture.log").open("w") as err:
                    capture = subprocess.Popen(command, stdout=out, stderr=err, start_new_session=True)
                    try:
                        while capture.poll() is None:
                            text = (folder / "rtt.log").read_text(errors="replace")
                            assert not FAILURE.search(text), "MCU self-test/fault report"
                            assert logger.poll() is None, "RTT monitor exited"
                            assert time.monotonic() - started < args.seconds + 20, "capture timeout"
                            time.sleep(.2)
                        assert capture.returncode == 0, "telemetry capture failed"
                    finally:
                        if capture.poll() is None:
                            os.killpg(capture.pid, signal.SIGTERM)
                            capture.wait(timeout=5)
                validate_capture(json.loads((folder / "capture.json").read_text()))
                if args.current_maneuver:
                    stage["current_runs"] = []
                    for repeat in range(2):
                        offset = len((folder / "rtt.log").read_text(errors="replace"))
                        current = subprocess.run([
                            "just", "test-bridge", str(folder / f"current-{repeat}.parquet"),
                            str(args.current_maneuver),
                        ], capture_output=True, text=True, timeout=30)
                        (folder / f"current-{repeat}.driver.log").write_text(current.stdout+current.stderr)
                        segment = (folder / "rtt.log").read_text(errors="replace")[offset:]
                        (folder / f"current-{repeat}.rtt.log").write_text(segment)
                        stage["current_runs"].append(current.returncode)
                        assert current.returncode == 0, "inhibited current maneuver failed"
                        report = json.loads((folder / f"current-{repeat}.parquet.json").read_text())
                        assert report["terminal_ok"] and all(e["ok"] for e in report["events"])
                        validate_capture(report["record"])
                        # The persistent RTT reader owns ST-Link here. Hardware
                        # enable/clock readback follows after that session closes.
                logger.send_signal(signal.SIGINT)
                logger.wait(timeout=5)
                logger = None
            text = (folder / "rtt.log").read_text(errors="replace")
            assert not FAILURE.search(text), "MCU self-test/fault report"
            reports = [tuple(map(int, r)) for r in REPORT.findall(text)]
            assert len(reports) >= args.seconds // 2, "missing stress progress"
            assert all(b[2] > a[2] for a, b in zip(reports, reports[1:])), "MCU restarted/stress stalled"
            assert all(r[0] == stage["clocks"]["cpu_hz"] for r in reports), "clock model mismatch"
            stage["die_c_range"] = [min(r[3] for r in reports)/10, max(r[3] for r in reports)/10]
            # Integrity/transport success is separate from the ISR deadline.
            timing = [tuple(map(int, r)) for r in re.findall(r"isr/s: n=(\d+) avg=(\d+) max=(\d+) over=(\d+)", text)]
            stage["isr_body_over_budget"] = sum(r[3] for r in timing)
            stage["isr_body_max_cycles"] = max((r[2] for r in timing), default=0)
            stage["stress_reports"] = len(reports)
            stage["stress_checks"] = reports[-1][2]
            stage["clocks_after"] = clocks()
            status = json.loads(run(["just", "cli-bridge", "status"]).stdout)
            assert status["fault_count"] == 0 and status["motor_state"] == "Stopped"
            stage["status"] = status
            stage["passed"] = True
            (args.output / "summary.json").write_text(json.dumps(summary, indent=2)+"\n")
            print(json.dumps(stage), flush=True)
            # Do not leave the preceding overclock running while the next
            # image is compiled. This also gives a stock-clock cooldown.
            run(device("download", "--connect-under-reset", "--verify", str(stock)), timeout=90)
            run(device("reset"))
    except Exception as error:
        summary["error"] = str(error)
        raise
    finally:
        if logger is not None and logger.poll() is None:
            logger.terminate()
            logger.wait(timeout=5)
        # Works even if the experimental PLL never starts or CPU has faulted.
        try:
            run(device("download", "--connect-under-reset", "--verify", str(stock)), timeout=90)
            run(device("reset"))
            summary["restored"] = json.loads(run([
                "target/release/oxifoc-host-cli", "--elf", str(stock),
                "--wait-secs", "12", "--json", "status",
            ]).stdout)
        except Exception as error:
            summary["restore_error"] = str(error)
            raise
        finally:
            (args.output / "summary.json").write_text(json.dumps(summary, indent=2)+"\n")


if __name__ == "__main__":
    main()
