"""Shared MK5 probe-rs helpers: RCC/TIM1/ADC clock readback and capture checks."""
import os
from pathlib import Path
import subprocess
import time

# probe-rs selector (VID:PID:serial) of the MK5 debug probe; unset, probe-rs
# uses the only attached probe.
PROBE = os.environ.get("OXIFOC_MK5_PROBE") or None
ELF = Path("oxifoc-f405/target/vesc6-mk5/thumbv7em-none-eabihf/release/oxifoc-f405")
def device(command, *args):
    return ["probe-rs", command, "--chip", "STM32F405RGTx", *probe_args(), *args]


def probe_args():
    return ["--probe", PROBE] if PROBE else []


def run(args, timeout=30):
    return subprocess.run(args, capture_output=True, text=True, timeout=timeout, check=True)


def read(address, words=1):
    output = run(device("read", "b32", hex(address), str(words))).stdout
    return [int(x, 16) for line in output.splitlines() if ":" in line
            for x in line.split(":", 1)[1].split()]


def clocks(expected_pwm_hz=20_000):
    deadline = time.monotonic() + 5
    while True:
        cr, pll, cfgr = read(0x40023800, 3)
        if cr & (1 << 25) and (cfgr >> 2) & 3 == 2:
            break
        assert time.monotonic() < deadline, f"PLL not locked/selected: RCC={cr:08x},{pll:08x},{cfgr:08x}"
        time.sleep(.1)
    hse = 8_000_000  # nominal HSE, as configured by the firmware
    sys = hse * ((pll >> 6) & 511) // ((pll & 63) * 2 * (((pll >> 16) & 3) + 1))
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
    assert abs(hz - expected_pwm_hz) <= expected_pwm_hz / 100, "PWM/ADC rate changed"
    return dict(cpu_hz=hclk, apb1_hz=p1, apb2_hz=p2, adc_hz=adc_hz,
                spi_hz=spi_hz, tim1_hz=timer, pwm_hz=hz, flash_ws=flash & 7,
                flash_access_hz=hclk / ((flash & 7) + 1), rcc=[cr, pll, cfgr])


def validate_capture(meta):
    raw = meta["raw_telemetry"]
    assert meta["gaps"] == meta["samples_lost"] == 0, "telemetry samples missing"
    assert raw["ended"] and raw["error"] is None, "incomplete telemetry session"
    assert raw["host_drops"] == raw["producer_drops"] == 0, "telemetry queue overflow"
    assert raw["records_received"] == raw["records_delivered"] == raw["end_total_records"] == meta["rows"]
