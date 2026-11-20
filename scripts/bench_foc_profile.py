"""Finite SPI/Packed28 FOC profiles; all SWD setup/export is outside excitation."""
import argparse
import bisect
import collections
import hashlib
import json
import math
import os
import resource
import shutil
import sys
import re
import select
from pathlib import Path
import signal
import subprocess
import struct
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


def decode_deadline(words, timer_hz):
    assert len(words) == 11 and words[0] in [1, 2, 3] and words[1] > 1000
    assert timer_hz > 0
    margin = words[2] if words[2] < 2**31 else words[2] - 2**32
    return dict(raw_words=words, schema=words[0], writes=words[1],
        min_margin_us=margin*1e6/timer_hz,
        max_entry_to_commit_us=words[3]*1e6/timer_hz if words[0] < 3 else None,
        entry_phase_us=[x*1e6/timer_hz for x in words[4:6]] if words[0] < 3 else None,
        late_writes=words[6], ambiguous_entries=words[7],
        worst_seq=words[8] if words[0] == 1 else None,
        worst_write_ordinal=words[8] if words[0] == 3 else None,
        outside_adc=words[9] if words[0] == 1 else None, still_masked=words[10])


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


WORK_NAMES = [
    "velocity_pi", "command", "deadman", "derating", "hall_new_edge",
    "hall_slew_clipped", "temperature", "startup", "hall_slew_wrap",
    "hall_velocity_wrap", "hall_witness_wrap", "observer_error_wrap",
    "angle_wrap", "circle_search", "authority_ahead", "authority_fallback",
]
ROOT = Path(__file__).resolve().parents[1]
PORT = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_3C:DC:75:8A:FF:10-if00"
BASE_FEATURES = "transport-spi,board-vesc6-mk5,task-priority,fast-telemetry,ccm-telemetry,bench-calibrated-clock,bench-foc-distribution"
DETAIL_FEATURES = {
    "minimal": "", "work": "bench-foc-work", "sections": "isr-profiling",
    "sections-ram": "bench-foc-sections-ram,isr-profiling",
    "phase-ram": "bench-foc-phase-ram", "deadline": "bench-foc-deadline",
    "outliers": "bench-cycle-outliers",
}
SECTION_NAMES = ["command", "protection", "step", "tail", "pwm_off", "stopped_phase",
    "gate", "control", "trig", "post", "estimator", "observer", "startup", "phase_output",
    "current_read", "phase_tracker", "velocity_request", "sensor_sampling", "phase_voltage",
    "authority_negative", "authority_positive", "dissipative_solve", "startup_confirm",
    "startup_log", "startup_seed", "observer_tail"]


def save(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def work_names(mask):
    return [name for bit, name in enumerate(WORK_NAMES) if mask & (1 << bit)]


def decode_pcs(data, symbol_listing):
    assert len(data) > 0 and len(data) % 8 == 0, "incomplete PC records"
    symbols = []
    for line in symbol_listing.splitlines():
        fields = line.split(maxsplit=3)
        if len(fields) == 4 and fields[2].lower() == "t":
            start, size = int(fields[0], 16), int(fields[1], 16)
            if size:
                symbols.append((start, start + size, fields[3]))
    symbols.sort()
    starts = [row[0] for row in symbols]
    driven = collections.Counter()
    for pc, flags in struct.iter_unpack("<II", data):
        assert flags in (0, 1), "unknown PC sampling flag"
        if flags:
            driven[pc & ~1] += 1
    functions = collections.Counter()
    for pc, count in driven.items():
        index = bisect.bisect_right(starts, pc) - 1
        name = symbols[index][2] if index >= 0 and pc < symbols[index][1] else "<unmapped>"
        functions[name] += count
    total = sum(driven.values())
    assert total > 0, "no driven PC samples"
    return dict(total_samples=len(data) // 8, driven_samples=total,
        functions=[dict(name=name, samples=count, share=count / total)
                   for name, count in functions.most_common()],
        pc_counts={hex(pc): count for pc, count in driven.items()})


def decode_outliers(summary, words, cpu_hz):
    assert len(summary) == 12 and len(words) % 16 == 0
    width = len(words) // 16
    assert width == 10 + len(SECTION_NAMES), "section schema differs from this decoder"
    assert summary[11] == 0, "DWT epoch changed"
    records = {}
    for start in range(0, len(words), width):
        record = words[start:start + width]
        if record[1]:
            records[record[0]] = dict(seq=record[0], body_us=record[1] * 1e6 / cpu_hz,
                entry_gap_us=record[2] * 1e6 / cpu_hz, flags=record[3],
                platform_cycles=record[5:10],
                nested_section_cycles=dict(zip(SECTION_NAMES, record[10:])))
    return dict(raw_summary=summary, raw_records=words, record_words=width,
        records=sorted(records.values(), key=lambda row: row["body_us"], reverse=True),
        sum_u32_proven_safe=summary[0] * summary[2] < 2**32)


def decode_work(header, classes, records, distribution, class_bits=None):
    """class_bits lists the event bit behind each class-index bit; the default is bits 0..7."""
    class_bits = list(class_bits or range(8))
    assert len(class_bits) == 8 and len(set(class_bits)) == 8 and max(class_bits) < len(WORK_NAMES)
    assert len(header) == 8 and header[0] == 1
    _, enabled, bins, width, record_width, capacity, kept, dropped = header
    assert enabled in (0, 1) and (bins, width, record_width) == (256, 5, 3)
    assert len(classes) == bins * width and len(records) == capacity * record_width
    assert 0 <= kept <= capacity
    clock = distribution["cpu_hz"]
    rows = []
    for index in range(bins):
        n, maximum, over, low, high = classes[index * width:(index + 1) * width]
        mask = sum(1 << bit for position, bit in enumerate(class_bits) if index >> position & 1)
        if n:
            assert 0 <= over <= n and low + (high << 32) <= n * maximum
            rows.append(dict(mask=mask, events=work_names(mask), n=n, over_budget=over,
                max_us=maximum * 1e6 / clock, sum_cycles=low + (high << 32),
                mean_us=(low + (high << 32)) * 1e6 / clock / n))
    assert sum(row["n"] for row in rows) == distribution["n"]
    assert sum(row["over_budget"] for row in rows) == distribution["over_budget"] == kept + dropped
    assert sum(row["sum_cycles"] for row in rows) == distribution["raw_words"][12] + (distribution["raw_words"][13] << 32)
    events, combinations = [], {}
    for i in range(kept):
        seq, cycles, mask = records[i * record_width:(i + 1) * record_width]
        assert cycles > distribution["budget_cycles"]
        assert (seq - distribution["first_seq"]) & 0xffffffff < distribution["n"]
        assert mask < 1 << len(WORK_NAMES), "unknown work bit"
        events.append(dict(seq=seq, body_cycles=cycles, body_us=cycles * 1e6 / clock,
                           mask=mask, events=work_names(mask)))
        group = combinations.setdefault(mask, dict(mask=mask, events=work_names(mask), count=0, max_us=0))
        group["count"] += 1
        group["max_us"] = max(group["max_us"], cycles * 1e6 / clock)
    if not enabled:
        assert all(row["mask"] == 0 for row in rows)
        assert all(event["mask"] == 0 for event in events)
    return dict(schema=1, tags_enabled=bool(enabled), complete_attribution=bool(enabled and not dropped),
        class_bits=class_bits, kept=kept, dropped=dropped, classes=rows, overruns=events,
        combinations=sorted(combinations.values(), key=lambda row: row["count"], reverse=True),
        raw_header=header, raw_classes=classes, raw_records=records)


def image_artifacts(image, output):
    for command, name in [(["rust-nm", "--print-size", "--defined-only", "--demangle"], "symbols.txt"),
                          (["rust-objdump", "-d", "--demangle"], "disassembly.txt"),
                          (["rust-size", "-A"], "sections.txt")]:
        with (output / name).open("w") as stream:
            subprocess.run([*command, str(image)], stdout=stream, check=True)
    return {fields[3]: int(fields[0], 16)
            for line in (output / "symbols.txt").read_text().splitlines()
            if len(fields := line.split(maxsplit=3)) == 4}


def build_image(args, manifest):
    features = ",".join(filter(None, [BASE_FEATURES, DETAIL_FEATURES[args.detail], args.extra_features]))
    env = {**os.environ, "CARGO_INCREMENTAL": "0", "CARGO_PROFILE_RELEASE_INCREMENTAL": "false",
           "CARGO_PROFILE_RELEASE_CODEGEN_UNITS": "1",
           "DEFMT_LOG": "warn" if args.logging == "quiet" else "warn,oxifoc_f405::control::foc=info"}
    env["TMPDIR"] = str(args.output / "tmp")
    Path(env["TMPDIR"]).mkdir()
    command = ["cargo", "rustc", "-vv", "--release", "--locked", "--target-dir", "target/vesc6-mk5",
               "--no-default-features", "--features", features, "--bin", "oxifoc-f405"]
    if args.link_reproduce:
        command += ["--", "-C", f"link-arg=--reproduce={args.output / 'link.tar'}"]
    with (args.output / "build.log").open("w") as stream:
        subprocess.run(command, cwd=ROOT / "oxifoc-f405", env=env, stdout=stream,
                       stderr=subprocess.STDOUT, check=True, timeout=600)
    image = args.output / "firmware.elf"
    shutil.copyfile(ROOT / probe.ELF, image)
    paths = probe.run(["git", "ls-files", "--cached", "--others", "--exclude-standard",
        "oxifoc-core", "oxifoc-f405", "vendor/rtt-target", "Cargo.lock", "Cargo.toml", ".cargo"]).stdout.splitlines()
    manifest.update(source_commit=manifest["checkout_commit"], features=features,
        body_boundary="before_all_recorders",
        command=command, build_env={key: env[key] for key in ["CARGO_INCREMENTAL", "CARGO_PROFILE_RELEASE_INCREMENTAL",
            "CARGO_PROFILE_RELEASE_CODEGEN_UNITS", "DEFMT_LOG"]},
        source_sha256={name: sha(ROOT / name) for name in paths if (ROOT / name).is_file()})
    return image


def prioritize(process):
    """Retain actual per-thread RTKit policy in each capture receipt."""
    resource.prlimit(process.pid, resource.RLIMIT_RTTIME, (200_000, 200_000))
    threads = {}
    until = time.monotonic() + 1
    while time.monotonic() < until and process.poll() is None:
        for path in Path(f"/proc/{process.pid}/task").glob("*"):
            tid = int(path.name)
            if tid in threads:
                continue
            try:
                name = (path / "comm").read_text().strip()
                worker = name == "tokio-rt-worker"
                method, signature, value = ("MakeThreadRealtimeWithPID", "ttu", "5") if worker else (
                    "MakeThreadHighPriorityWithPID", "tti", "-10")
                request = subprocess.run(["busctl", "--system", "--", "call", "org.freedesktop.RealtimeKit1",
                    "/org/freedesktop/RealtimeKit1", "org.freedesktop.RealtimeKit1", method, signature,
                    str(process.pid), str(tid), value], capture_output=True, text=True, timeout=5)
                threads[tid] = dict(tid=tid, name=name, scheduler=os.sched_getscheduler(tid),
                    nice=os.getpriority(os.PRIO_PROCESS, tid),
                    rt_priority=os.sched_getparam(tid).sched_priority,
                    rtkit_error=request.stderr.strip() if request.returncode else None)
            except (FileNotFoundError, ProcessLookupError):
                pass
        time.sleep(.02)
    return list(threads.values())


def summarize(manifest):
    groups = {}
    for run in manifest["runs"]:
        if not run["accepted"]:
            continue
        group = groups.setdefault(run["case"], dict(runs=0, n=0, sum_cycles=0, over_budget=0,
            max_us=0, entry_max_us=0, active_seconds=0, combinations={}, attribution_dropped=0))
        group["runs"] += 1
        if "distribution" in run:
            timing = run["distribution"]
            assert group.get("cpu_hz", timing["cpu_hz"]) == timing["cpu_hz"], "clock changed between repeats"
            group["n"] += timing["n"]
            group["sum_cycles"] += timing["raw_words"][12] + (timing["raw_words"][13] << 32)
            group["over_budget"] += timing["over_budget"]
            group["max_us"] = max(group["max_us"], timing["max_us"])
            group["entry_max_us"] = max(group["entry_max_us"], timing["entry_max_us"])
            group["active_seconds"] += timing["n"] * timing["budget_cycles"] / timing["cpu_hz"]
            group["cpu_hz"] = timing["cpu_hz"]
        if "work" in run:
            group["tags_enabled"] = run["work"]["tags_enabled"]
            group["attribution_dropped"] += run["work"]["dropped"]
            for row in run["work"]["combinations"]:
                combined = group["combinations"].setdefault(row["mask"],
                    dict(mask=row["mask"], events=row["events"], count=0, max_us=0))
                combined["count"] += row["count"]
                combined["max_us"] = max(combined["max_us"], row["max_us"])
        if "pwm_deadline" in run:
            deadline = run["pwm_deadline"]
            group["writes"] = group.get("writes", 0) + deadline["writes"]
            group["late_writes"] = group.get("late_writes", 0) + deadline["late_writes"]
            group["min_margin_us"] = min(group.get("min_margin_us", float("inf")), deadline["min_margin_us"])
    for group in groups.values():
        if group["n"]:
            group["mean_us"] = group["sum_cycles"] * 1e6 / group["cpu_hz"] / group["n"]
            group["over_per_second"] = group["over_budget"] / group["active_seconds"]
        group["combinations"] = sorted(group["combinations"].values(), key=lambda row: row["count"], reverse=True)
    return dict(elf_sha256=manifest["elf_sha256"], detail=manifest["detail"],
                measurement_kind=manifest.get("measurement_kind", "timing"),
                rejected=[run["name"] for run in manifest["runs"] if not run["accepted"]], cases=groups)


def validate_image_mode(detail, symbols):
    assert "FOC_DISTRIBUTION_CONTROL" in symbols, "ELF lacks the finite recorder"
    assert ("FOC_WORK_HEADER" in symbols) == (detail == "work"), "select the ELF's work instrumentation level"
    assert ("PWM_DEADLINE_DATA" in symbols) == (detail == "deadline"), "select the ELF's deadline instrumentation level"
    assert ("BENCH_CYCLE_RECORDS" in symbols) == (detail == "outliers"), "select the ELF's section-outlier instrumentation level"
    if detail == "work":
        assert all(name in symbols for name in ["ISR_WORK_ENABLED", "FOC_WORK_CLASSES", "FOC_WORK_RECORDS"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--elf", type=Path, help="Capture this exact existing ELF instead of rebuilding")
    parser.add_argument("--build-only", action="store_true")
    parser.add_argument("--link-reproduce", action="store_true", help="Retain linker inputs for address-controlled relinks")
    parser.add_argument("--report-only", action="store_true", help="Summarize an existing manifest without hardware access")
    parser.add_argument("--extra-features", default="")
    parser.add_argument("--detail", choices=DETAIL_FEATURES, default="minimal")
    parser.add_argument("--repeats", type=int, default=3, choices=range(1, 6))
    parser.add_argument("--recovery-elf", type=Path)
    parser.add_argument("--logging", choices=["timing", "quiet"], default="quiet")
    parser.add_argument("--cases", default="hold,current,velocity40,velocity120")
    parser.add_argument("--seconds", type=int, default=14, choices=range(2, 31))
    parser.add_argument("--pwm-hz", type=int, default=20000, choices=[15000, 20000])
    parser.add_argument("--capture-hz", type=int)
    parser.add_argument("--work-tags", choices=["on", "off"], default="on")
    parser.add_argument("--probe-speed", type=int, default=1000)
    parser.add_argument("--serial-path", default=PORT)
    parser.add_argument("--velocity-debug", action="store_true")
    parser.add_argument("--pc-sampler", type=Path, help="Separate PCSR diagnostic using the compiled pc_sampler example")
    args = parser.parse_args()
    if args.report_only:
        recorded = json.loads((args.output / "manifest.json").read_text())
        result = summarize(recorded)
        if recorded.get("measurement_kind") == "pcsr":
            listing = (args.output / "symbols.txt").read_text()
            result["pcsr_runs"] = [dict(name=run["name"],
                **decode_pcs((args.output / (run["name"] + ".pcs.bin")).read_bytes(), listing))
                for run in recorded["runs"] if run["accepted"]]
        save(args.output / "summary.json", result)
        print(json.dumps(result, indent=2))
        return
    cases = args.cases.split(",")
    if len(cases) != len(set(cases)) or not set(cases) <= {
        "hold", "current", "velocity40", "velocity120", "velocity450", "brake-authority",
        "transitions", "brake-transitions"}:
        parser.error("unknown or duplicate case")
    if not args.build_only and not args.recovery_elf:
        parser.error("a capture needs --recovery-elf")
    if args.build_only and args.elf:
        parser.error("--build-only and --elf are separate operations")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = dict(schema=2, detail=args.detail, logging=args.logging, runs=[],
        checkout_commit=probe.run(["git", "rev-parse", "HEAD"]).stdout.strip(),
        dirty_files=probe.run(["git", "status", "--porcelain"]).stdout.splitlines(),
        rustc=subprocess.check_output(["rustc", "-Vv"], cwd=ROOT / "oxifoc-f405", text=True),
        probe_rs=probe.run(["probe-rs", "--version"]).stdout)
    if args.elf:
        image = args.output / "firmware.elf"
        shutil.copyfile(args.elf.resolve(strict=True), image)
        metadata = args.elf.parent / "build.json"
        manifest["build_origin"] = json.loads(metadata.read_text()) if metadata.exists() else None
        if manifest["build_origin"]:
            assert manifest["build_origin"]["elf_sha256"] == sha(image), "ELF differs from its build manifest"
        manifest["source_commit"] = manifest["build_origin"].get("source_commit") if manifest["build_origin"] else None
    else:
        image = build_image(args, manifest)
    manifest["elf_sha256"] = sha(image)
    symbols = image_artifacts(image, args.output)
    validate_image_mode(args.detail, symbols)
    save(args.output / "build.json", manifest)
    if args.build_only:
        print("Built", image, manifest["elf_sha256"], flush=True)
        return
    recovery = args.recovery_elf.resolve(strict=True)
    manifest.update(recovery_elf=str(recovery), recovery_sha256=sha(recovery),
                    pwm_hz=args.pwm_hz, capture_hz=args.capture_hz or args.pwm_hz,
                    probe=probe.PROBE, probe_speed_khz=args.probe_speed, serial_path=args.serial_path)
    manifest["measurement_kind"] = "pcsr" if args.pc_sampler else "timing"
    if args.pc_sampler:
        args.pc_sampler = args.pc_sampler.resolve(strict=True)
        manifest["pc_sampler"] = dict(path=str(args.pc_sampler), sha256=sha(args.pc_sampler))
    device = probe.device
    probe.device = lambda command, *rest: device(command, "--speed", str(args.probe_speed), *rest)

    def cli(elf, *command):
        return [str(ROOT / "target/release/oxifoc-host-cli"), "--transport", "usb-serial-jtag",
                "--serial-path", args.serial_path, "--elf", str(elf), "--wait-secs", "12", "--json", *map(str, command)]

    def call(elf, *command):
        return json.loads(probe.run(cli(elf, *command), timeout=22).stdout)

    def stopped(elf):
        status = call(elf, "status")
        assert status["motor_state"] == "Stopped" and status["fault_count"] == 0, status
        assert 23000 <= status["vbus_mv"] <= 25000 and status["fet_temp_c_x10"] < 500, status
        return status

    def flash(elf, log_name):
        with (args.output / log_name).open("w") as stream:
            subprocess.run(probe.device("download", "--connect-under-reset", "--verify", str(elf)),
                           stdout=stream, stderr=subprocess.STDOUT, check=True, timeout=90)
            subprocess.run(probe.device("reset"), stdout=stream, stderr=subprocess.STDOUT, check=True, timeout=30)
        time.sleep(1.5)

    def interrupted(signum, frame):
        raise KeyboardInterrupt(signum)

    signal.signal(signal.SIGTERM, interrupted)
    manifest["before"] = stopped(recovery)
    original_pwm = call(recovery, "config", "get", "pwm-config")
    manifest["pwm_before"] = original_pwm
    save(args.output / "manifest.json", manifest)
    try:
        flash(image, "flash.log")
        stopped(image)
        if call(image, "config", "get", "pwm-config")["active_freq_hz"] != args.pwm_hz:
            assert call(image, "config", "set", "pwm-config", f"freq_hz={args.pwm_hz}")["persisted"]
        for case in cases:
            for repeat in range(args.repeats):
                name = f"{case}-{repeat}"
                row = dict(name=name, case=case, repeat=repeat, accepted=False)
                sampler = None
                manifest["runs"].append(row)
                try:
                    probe.run(probe.device("reset"))
                    time.sleep(1.5)
                    row["before"] = stopped(image)
                    row["peripherals"] = f405_timing.snapshot(args.pwm_hz)
                    row["configuration"] = call(image, "config", "dump")
                    if case in ["brake-authority", "brake-transitions"]:
                        assert isinstance(row["before"]["phase_source"], dict) and "HallToObserver" in row["before"]["phase_source"]
                        call(image, "config", "set", "current-limits", "max_dissipation=0.05", "bus_regen_max=0", "--volatile")
                    row["active_limits"] = call(image, "config", "get", "current-limits")
                    if case in ["velocity450", "brake-authority"]:
                        plan = dict(name=name, capture=dict(tail_s=.4, fast_hz=manifest["capture_hz"]), terminal="stop",
                            timeline=[dict(t=.3, cmd=dict(velocity=dict(rad_s=450))), dict(t=.3+args.seconds, cmd=dict(stop={}))])
                    elif case in ["transitions", "brake-transitions"]:
                        # Current start, both directions through zero, Coast and restart; fixed 10.6 s.
                        plan = dict(name=name, capture=dict(tail_s=.4, fast_hz=manifest["capture_hz"]), terminal="stop",
                            timeline=[dict(t=.3, cmd=dict(start=dict(iq=.2))), dict(t=.6, cmd=dict(velocity=dict(rad_s=120))),
                                dict(t=2.6, cmd=dict(velocity=dict(rad_s=450))), dict(t=4.6, cmd=dict(velocity=dict(rad_s=-120))),
                                dict(t=6.6, cmd=dict(velocity=dict(rad_s=-450))), dict(t=8.6, cmd=dict(coast={})),
                                dict(t=9.1, cmd=dict(velocity=dict(rad_s=120))), dict(t=10.6, cmd=dict(stop={}))])
                    else:
                        plan = json.loads((ROOT / "maneuvers" / f"foc-profile-{case}.json").read_text())
                        plan["capture"]["fast_hz"] = manifest["capture_hz"]
                    plan_path = args.output / f"{name}.plan.json"
                    save(plan_path, plan)
                    row["plan"] = plan
                    if args.detail == "work":
                        probe.run(probe.device("write", "b32", hex(symbols["ISR_WORK_ENABLED"]), "1" if args.work_tags == "on" else "0"))
                    if args.detail in ["sections-ram", "phase-ram"]:
                        probe.run(probe.device("write", "b32", hex(symbols["FOC_SECTIONS_READY"]), "0"))
                    control = symbols["FOC_DISTRIBUTION_CONTROL"]
                    probe.run(probe.device("write", "b32", hex(control), "1"))
                    time.sleep(.1)
                    assert probe.read(control) == [2]
                    # Last SWD disconnect can disable DWT; let its supervisor repair it while stopped.
                    time.sleep(1.3)
                    sampler = None
                    if args.pc_sampler:
                        duration = max(event["t"] for event in plan["timeline"]) + 30
                        sampler = subprocess.Popen([str(args.pc_sampler), probe.PROBE.rsplit(":", 1)[-1],
                            str(duration), "64", str(args.output / f"{name}.pcs.bin"), "24000", "controlled"],
                            stdin=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                        ready, _, _ = select.select([sampler.stderr], [], [], 10)
                        assert ready, "PC sampler attach timeout"
                        first = sampler.stderr.readline()
                        assert "attached" in first, first
                        time.sleep(1.3)
                    capture_path = args.output / f"{name}.parquet"
                    command = cli(image, *(["--velocity-debug"] if args.velocity_debug else []),
                        "--fast-source", "ergot-raw", "--fast-encoding", "packed28", "maneuver", "run", plan_path, "--out", capture_path)
                    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                        env={**os.environ, "TOKIO_WORKER_THREADS": "6", "RUST_LOG": "warn,device=info"})
                    try:
                        row["host_threads"] = prioritize(process)
                        stdout, stderr = process.communicate(timeout=max(event["t"] for event in plan["timeline"]) + 30)
                    finally:
                        if process.poll() is None:
                            process.terminate()
                            process.communicate(timeout=10)
                        call(image, "stop")
                        if sampler is not None:
                            _, rest = sampler.communicate(input="stop\n", timeout=10)
                            (args.output / f"{name}.sampler.log").write_text(first + rest)
                            assert sampler.returncode == 0, "PC sampler failed"
                    (args.output / f"{name}.driver.log").write_text(stderr)
                    (args.output / f"{name}.parquet.json").write_text(stdout)
                    report = json.loads(stdout)
                    row.update(returncode=process.returncode, record=report["record"], events=report["events"],
                               terminal_ok=report["terminal_ok"], after=call(image, "status"))
                    assert probe.read(0x40010020)[0] & 0x555 == 0
                    assert probe.read(control) == [3], "recorder not frozen"
                    if args.detail == "deadline":
                        tim5 = probe.read(0x40000c00, 12)
                        clocks = row["peripherals"]["clocks"]
                        cfgr = clocks["rcc"][2]
                        timer_hz = clocks["apb1_hz"] * (1 if (cfgr >> 10) & 7 < 4 else 2)
                        assert tim5[0] & 1 and tim5[10:12] == [0, 0xffffffff]
                        row["pwm_deadline"] = decode_deadline(probe.read(symbols["PWM_DEADLINE_DATA"], 11), 2 * timer_hz)
                    else:
                        row["distribution"] = decode(probe.read(symbols["FOC_DISTRIBUTION_DATA"], 528))
                        segments = report["record"]["pwm_timing_segments"]
                        assert len(segments) == 1
                        timing = segments[0]["control_timing"]
                        assert timing["clock_hz"] == row["distribution"]["cpu_hz"]
                        assert timing["period_ticks"] == row["distribution"]["budget_cycles"]
                    if args.detail == "work":
                        class_bits = None
                        if "FOC_WORK_CLASS_BITS" in symbols:
                            class_bits = [byte for word in probe.read(symbols["FOC_WORK_CLASS_BITS"], 2)
                                          for byte in word.to_bytes(4, "little")]
                        elif "bench-work-class-wraps" in (manifest.get("build_origin") or manifest).get("features", ""):
                            # Early wrap-map images lost the unreferenced table at link time.
                            class_bits = [0, 1, 2, 3, 14, 15, 8, 11]
                        row["work"] = decode_work(probe.read(symbols["FOC_WORK_HEADER"], 8),
                            probe.read(symbols["FOC_WORK_CLASSES"], 256 * 5),
                            probe.read(symbols["FOC_WORK_RECORDS"], 1024 * 3), row["distribution"], class_bits)
                    if args.detail in ["sections-ram", "phase-ram"]:
                        assert probe.read(symbols["FOC_SECTIONS_READY"]) == [1]
                        row["sections"] = probe.read(symbols["FOC_SECTIONS_DATA"], 25)
                    if args.detail == "outliers":
                        assert probe.read(symbols["BENCH_CYCLE_STATUS"]) == [2]
                        entry = next(line.split() for line in (args.output / "symbols.txt").read_text().splitlines()
                                     if line.endswith(" BENCH_CYCLE_RECORDS"))
                        row["section_outliers"] = decode_outliers(
                            probe.read(symbols["BENCH_CYCLE_SUMMARY"], 12),
                            probe.read(symbols["BENCH_CYCLE_RECORDS"], int(entry[1], 16) // 4),
                            row["distribution"]["cpu_hz"])
                        origin = manifest.get("build_origin") or manifest
                        if origin.get("body_boundary") == "before_all_recorders":
                            section_summary = row["section_outliers"]["raw_summary"]
                            assert section_summary[0] == row["distribution"]["n"]
                            assert section_summary[2] == row["distribution"]["raw_words"][5]
                    assert process.returncode == 0 and report["terminal_ok"] and all(event["ok"] for event in report["events"])
                    probe.validate_capture(report["record"])
                    stopped(image)
                    assert row["before"]["fault_generation"] == row["after"]["fault_generation"]
                    if args.detail == "deadline":
                        assert row["pwm_deadline"]["late_writes"] == row["pwm_deadline"]["ambiguous_entries"] == row["pwm_deadline"]["still_masked"] == 0
                    row["accepted"] = True
                    timing = row.get("distribution", row.get("pwm_deadline"))
                    print(name, {key: timing[key] for key in ["n", "mean_us", "max_us", "over_budget", "min_margin_us", "late_writes"] if key in timing}, flush=True)
                except BaseException as error:
                    row["error"] = dict(kind=type(error).__name__, message=str(error))
                    raise
                finally:
                    if sampler is not None and sampler.poll() is None:
                        try:
                            call(image, "stop")
                        finally:
                            try:
                                sampler.communicate(input="stop\n", timeout=10)
                            except subprocess.TimeoutExpired:
                                sampler.terminate()
                                sampler.communicate(timeout=5)
                    save(args.output / "manifest.json", manifest)
    finally:
        try:
            call(image, "stop")
        except Exception as error:
            manifest["stop_before_restore_error"] = str(error)
        for attempt in range(3):
            try:
                flash(recovery, f"restore-{attempt}.log")
                stopped(recovery)
                wanted = original_pwm["value"]["freq_hz"]
                if call(recovery, "config", "get", "pwm-config")["active_freq_hz"] != wanted:
                    assert call(recovery, "config", "set", "pwm-config", f"freq_hz={wanted}")["persisted"]
                    probe.run(probe.device("reset"))
                    time.sleep(1.5)
                manifest["restored"] = stopped(recovery)
                manifest["restored_pwm"] = call(recovery, "config", "get", "pwm-config")
                assert manifest["restored_pwm"]["active_freq_hz"] == wanted
                manifest["restored_ccer"] = probe.read(0x40010020)[0]
                assert manifest["restored_ccer"] & 0x555 == 0
                save(args.output / "manifest.json", manifest)
                print("Recovery restored; Stopped, zero faults, phase enables off", flush=True)
                break
            except Exception as error:
                manifest.setdefault("restore_errors", []).append(str(error))
                save(args.output / "manifest.json", manifest)
                if attempt == 2:
                    raise


if __name__ == "__main__":
    main()
