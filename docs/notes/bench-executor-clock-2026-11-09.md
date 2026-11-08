# Executor traces and independent PWM frequency check

Continues [the resistance-source study](bench-resistance-source-2026-11-09.md).
All artifacts, including failed captures, are retained under:

```text
/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-executor-clock-2026-11-09/
```

## Resistance comparison and frequency definitions

Native OxiFOC R: 30.166 / 30.213 / 30.742 mΩ, mean 30.374 mΩ. It used DC
0.6→3 A at nominal 20 kHz PWM/FOC, with filtered terminal voltage. VESC 7.00
wizard returned 30.80 mΩ at its native 5 kHz R/L stage: mean difference −1.38%.
This is agreement at the present precision, not an identical-current/temperature
calibration. Signed OxiFOC fits at other directions also gave about 30 mΩ.

LC1020E measurements are Series AC ESR **between two leads**, with rotor held at
positions found at 1 kHz. At 100 Hz the minimum-position readings were 61–63 mΩ
between leads; at 1 kHz, 65–76 mΩ across the selected positions/pairs. For the
balanced equivalent-star model, divide terminal resistance by two: 30.5–31.5 mΩ
at the 100 Hz minima, 32.5–38 mΩ at 1 kHz. AC losses and rotor/frequency dependence
prevent identifying those ESRs with exact DC winding resistance. Raw values,
method and color mapping remain in vesc-bench/docs/motor-lcr-2026-11-08.md.

20 kHz PWM is not a 20 kHz sinusoidal impedance test. Clock error does not change
DC ΔV/ΔI, and cannot explain the old 30 versus 43 mΩ voltage-definition difference.

## Default-off diagnostics

- `bench-executor-trace` enables the pinned Embassy `trace` callbacks and a
  3072-event atomic RAM buffer, plus ADC entry/exit events. No event is sent over
  USB or RTT. Exact ELF TaskStorage symbols resolve opaque task IDs.
- `bench-clock-reference` owns otherwise unused TIM5, PSC=0, no pins/IRQ/DMA. It
  compares free-running APB timer ticks, DWT and USB DSTS.FNSOF frame numbers.
- `bench-usb-thread` moves **all** USB control/RX/TX to the thread executor for A/B.
- `bench-scheduler-priority`: USB tasks priority 3, protocol 2, fast telemetry 1;
  other tasks retain zero. This is an experiment, not the production default.
- `bench-only-phase-a` permits only leg A through the normal enable paths.
  B/C remain disabled; Stop, faults and safe-off still disable A. It is mutually
  exclusive with `bench-no-phase-outputs`. Use only with motor phase A disconnected.
  HardwareInfo distinguishes the two bench builds.

`uv run scripts/bench_executor_trace.py arm --symbols EXACT.symbols` starts a
finite window after a 100 ms quiet delay. `--on-active` instead waits for active
FOC, allowing SWD to detach before the workload. Export using `read --out FILE`
after Stop and after the buffer freezes. Do not read while writers are active.
Generate symbols with `rust-nm -n -S -C EXACT.elf`. The export records its hash.

Direct SWD attach/detach can insert millisecond halts. The first immediate-arm
idle trace contains such an artifact and is excluded. DWT also freezes after
probe-rs detach clears trace enable on this setup. TIM5 timestamps avoid that
pitfall. The early flat-DWT trace files are explicitly invalid, retained as evidence.

## Executor A/B at nominal 20 kHz

PSU 24 V / 5.5 A. All three executor tests used **outputs-inhibited firmware**:
active FOC calculations and repeated zero-reference commands, with every recorded
phase-enable mask zero (MOE bit 6 alone is not a phase enable).

| Configuration | Rows | Gaps / lost | Outcome |
|---|---:|---:|---|
| P6 USB IRQ executor | 609,527 | 0 / 0 | Full 30 s command timeline and Stop succeeded |
| Thread executor, ordinary scheduler | 60,192 | 199 / 1,448 | Step 1 acknowledgement deadline, emergency Stop confirmed |
| Thread executor, priority scheduler | 66,408 | 411 / 3,122 | Step 3 acknowledgement deadline, emergency Stop confirmed |

Single A/B runs establish that neither tested thread configuration replaces the
existing IRQ executor under these conditions. They are not a general proof that
thread mode can never work, nor an EDF comparison. USB clock is also suspect;
repeat transport qualification after its independent clock issue is addressed.

In the priority trace's 38.90 ms window, fast-telemetry poll elapsed time had
median 989 µs, maximum 2200 µs; TX ready→poll maximum 2192 µs, RX 1505 µs (one
observed RX wake). IRQ-executor trace showed TX ready→poll maximum 97.6 µs.
Idle ADC entry spacing was 50.0 nominal µs; loaded IRQ trace reached 88.1 nominal
µs, priority trace 85.0 µs. These are finite instrumented windows, **not worst-case
bounds**. Poll elapsed time includes interrupts, not just task CPU time. A task
already running at window start may have no begin/end pair in that window; absence
of a pair is not evidence that it was idle. Timestamp units still use nominal
84 MHz; account for the clock scale below before treating them as physical µs.

Priority/deadline order ready tasks **within** an executor; neither preempts a
running poll or an interrupt-masking critical section. With both enabled, this
pinned implementation compares priority first, deadline second. EDF could serve
future soft scheduling budgets but does not guarantee a FOC deadline. FOC remains
an ADC interrupt. Bound producer work and critical sections before considering
removing the USB executor.

Neither Embassy tracing feature had been used before this study. `rtos-trace`
provides a backend interface (e.g. SystemView), task tracking and names; enabling
it alone does not supply a viewer/exporter. We used raw `trace` hooks to measure
this specific system with a bounded RAM buffer and Python timelines/ECDFs. No
FFT was needed for scheduling latency or fixed PWM frequency.

## Independent frequency observation

The user disconnected motor phase A and connected Hantek 6022BE CH1, physical
probe x10, to the free controller A output, ground to board input negative. B/C
were inhibited in firmware. DirectVoltage=0 generated approximately 50% PWM on A.
Motor remains connected only at B/C; this was not a motor R or rotation test.

Sigrok detected the scope and loaded its volatile FX2 firmware. Driver
`hantek-6xxx`, CH1 DC, vdiv=1 V, default 8 MHz acquisition verified in each CSV
header. A requested 800k samples returned 1,048,576; analysis uses actual row
count. First 4096 startup samples are discarded. Three independent captures each
contained 2594 accepted rising edges with periods 402–404 samples, no missing-edge
jumps. Accumulated edge fits gave **19867.1306 / 19867.1291 / 19867.1285 Hz**.

Thus the PWM observed with the scope's independent timebase is about **19.867 kHz**,
−0.6644% from nominal 20 kHz. Fine fit repeatability is not absolute frequency
accuracy: the Hantek timebase was not calibrated. It nevertheless agrees with the
separate host/sequence and USB-SOF comparisons. There is now strong evidence of
a real clock-rate discrepancy, beyond executor scheduling alone. HSE/PLL registers
select the intended external source/divisors; identifying the physical resonator
and cause still needs hardware evidence. Do not describe it as a proven bad quartz.

Scope voltage gain/offset were not calibrated: the plotted nominal probe-scaled
levels are not evidence of actual −4/28 V excursions. Frequency comes from edge
positions and is insensitive to that vertical scaling error.

The scope-associated 20 kHz telemetry capture retained 176,825 rows, zero gaps,
phase-enable masks only 0x40/0x49 (off / A high+low only). Its later command/terminal
acknowledgement failed; emergency Stop was confirmed after stream shutdown. Keep
that failure separate from the valid steady scope windows; do not call the whole
maneuver successful. Earlier scope files taken before PWM or after Stop are not
frequency evidence and are rejected by the analyzer's amplitude check.

Raw capture and analysis tools are in vesc-bench/analysis/analyze_executor_trace.py
and analyze_scope_clock.py, with versioned figures under vesc-bench/docs/assets/.

## Consequences and next steps

A scale near 0.993356 changes physical durations by about +0.669%, FFT frequency
labels and velocity estimates by the inverse scale, and timing-based L estimates
at a similar order. It barely matters to steady DC resistance. It matters more
for communication: F405 USB uses the same HSE/PLL and formally needs 48 MHz ±0.25%.
Working USB does not prove that requirement is met.

Do not merely relabel plots and assume USB was fixed. A temporary, device-specific
RCC/HSE-frequency experiment can correct timer calculations and put PLLQ back in
USB tolerance without soldering, followed by scope/SOF and load qualification.
No clock correction or per-phase calibration has been persisted in this study.
Investigate physical oscillator type/marking and temperature stability before
choosing a permanent board-calibration design or a hardware change. Resume R/L
and inverter-compensation comparisons only with all motor phases reconnected.

References: [pinned scheduler implementation](https://github.com/embassy-rs/embassy/blob/98d847be57f3ea022ce05fe9b95ab3639a1e0a93/embassy-executor/src/raw/run_queue.rs),
[trace callbacks](https://github.com/embassy-rs/embassy/blob/98d847be57f3ea022ce05fe9b95ab3639a1e0a93/embassy-executor/src/raw/trace.rs),
[rtos-trace backend interface](https://docs.rs/rtos-trace/0.2.1/rtos_trace/),
[STM32 RM0090](https://www.st.com/resource/en/reference_manual/rm0090-stm32f407-advanced-armbased-32bit-mcus-stmicroelectronics.pdf),
[sigrok Hantek 6022BE](https://sigrok.org/wiki/Hantek_6022BE).
