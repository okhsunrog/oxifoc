# Control ISR performance

How fast the F405 control interrupt runs, what decides its cost, and what to
check when changing or measuring it. The MK5 (STM32F405 at 168 MHz) is the
reference board; the CF2 shares the code and differs in board glue.

Measured history and rejected experiments are kept in
[ADC ISR optimization](notes/adc-isr-optimization.md); this page describes the
current code. Flash and RAM size are covered in [flash budget](flash-size.md).

## At a glance

| | Value |
|---|---|
| Interrupt | `ADC` (TIM1-triggered injected conversions), one control cycle per PWM period |
| PWM / FOC rate | 15 or 20 kHz (saved `PwmConfig`; default 20 kHz) |
| Cycle budget | 11 200 CPU cycles at 15 kHz, 8 400 at 20 kHz |
| MK5, 15 kHz, stopped | ~2 560 cycles average (23 % of the budget) |
| MK5, 15 kHz, velocity control at +120 / +450 electrical rad/s | ~6 094 / ~6 024 cycles average (54 %), ~7 200 maximum (64 %) |
| Same work at 20 kHz | ~73 % of the budget on average; 20 kHz has not been re-measured since the placement change |
| Placement noise with the current pins | about ±5 cycles from unrelated code edits |

Numbers are from the MK5 bench (unloaded motor, 24 V, HallToObserver) at
`1d52fe5`. Loaded, braking, direct-voltage and sensorless operation have not
been measured with the current image.

## What runs in the interrupt

`ADC()` in `oxifoc-f405/src/control/foc.rs` reads the three injected ADC
sequences, builds an `AdcSnapshot` and a Hall snapshot, then calls the shared
`run_foc_cycle` (`oxifoc-core/src/state.rs`), which runs commands, protection
and `FocDriver::step`.

| Work | Cadence |
|---|---|
| Current PI, phase estimation, observer, fast protection, PWM update | Every ISR |
| Command FIFO, Stop, accepted drive/affirmation timestamp | Every ISR |
| Velocity/position PI | About 1 kHz on a slot frame, actual elapsed `dt` |
| PI authority bounds | Prepared one native cycle before the PI, single use |
| PI postponement after fresh drive activity | At most one ISR |
| Deadman expiry poll | Once per frame (about 1 kHz); timeout 150 ms |
| Derating refresh | Every eight frames (about 8 ms) |
| Temperature conversion | Every 128 ISRs |
| Fast telemetry sample | Every `period`-th ISR while a session is armed |

The slot frame lives in `motor/foc_driver/velocity_schedule.rs`; maintenance
work is placed in slots so it never shares a cycle with the PI or its bounds.
Commands and fast protection stay native-rate. Decimated deadman polling does
not replace the independent watchdog, which the ISR feeds.

### Where the cycles go

Section averages from an `isr-profiling` image at +120 electrical rad/s
(about 6 750 cycles including roughly 560 of profiling overhead; sections are
approximate and overlap):

| Section | Cycles |
|---|---:|
| Estimator: Hall 929, observer 643, output/blend 365 | 2 279 |
| Current control | 902 |
| Remainder of `FocDriver::step` | ~820 |
| Torque gate | 495 |
| Telemetry publish | 374 |
| Command processing | 355 |
| ADC reads and conversions | 305 |
| Snapshot (ADC, PWM registers, Hall) | 293 |
| Protection | 233 |
| Cycle tail | 217 |

At +450 only the output/blend section differs (304).

## Placement decides tens of cycles

On the F405, code runs from Flash through the ART accelerator with 5 wait
states. The entry address of a hot function relative to 16-byte Flash lines
changes its cost, and which SRAM block its data lives in matters as well. A
bench diagnosis (88 runs, relinking identical objects) found:

- Identical ISR code measured 6 170–6 208 cycles at +120 across nine layouts.
  Three `nop`s in `main` or one unrelated static moved it by −15…+22.
- A single hot function's address residue (mod 16) spans up to 34 cycles.
- With the control driver in SRAM2 (the top 16 KiB of RAM) the ISR runs
  about 155 cycles slower; with only its small statics there, 10–20 slower.
  Bus-matrix or DMA contention is the presumed cause; it was not isolated.
- An instruction count is a poor proxy: an image with 148 more instructions
  in `run_foc_cycle` was 12 cycles faster at equal placement.

### What is pinned

`oxifoc-f405/foc-hot.x` (linked on the MK5) places these functions at their
measured best residue. Each is named in source with
`link_section` under the `compact-foc` feature, and the link fails if a
section goes missing.

| Section | Function | Residue |
|---|---|---:|
| `.text.foc_cycle` | `run_foc_cycle` | 8 |
| `.text.foc_phase` | `FocDriver::update_phase_with_applied_voltage` | 8 |
| `.text.foc_grant` | `FocDriver::grant_torque` | 0 |
| `.text.foc_adc` | `ADC` | 8 |
| `.text.foc_velocity_step` | `FocDriver::step_scheduled_velocity` | 4 |
| `.text.foc_current_step` | `FocController::step_with_injection` | 0 |
| `.text.foc_svpwm` | `space_vector_pwm` | 12 |
| `.text.foc_blend` | `PhaseManager::blend_with_observer` | 0 |
| `.text.foc_injection` | `PhaseManager::injection` | 4 |

With these pinned, shuffling the rest of `.text` moves the ISR by at most
about 5 cycles. The residues were swept one function at a time at 15 kHz;
re-measure them ([residue sweep](#re-measuring-a-pinned-residue)) when one of
these functions changes substantially or when switching the reference rate to
20 kHz.

### RAM placement

flip-link packs `.data`/`.bss` against the top of RAM. `foc-hot.x` places
two cold Embassy task pools (protocol servers, SPI pump) last, in
`.bss_cold`, so they fill SRAM2 and everything the ISR touches stays in
SRAM1. The SPI pump pool owns the DMA slots, which also keeps DMA traffic out
of the ISR's block. Two link-time checks guard this:

- `.bss_cold` must be at least 16 KiB (the pools are matched by name; a rename
  or a smaller pool fails the link);
- `.uninit` must stay empty: cortex-m-rt places it after `.bss`, above the
  filler.

Check the result after RAM changes with `just isr-residues <elf>`, which lists
every RAM static the ADC path references with its block and fails if one
reaches SRAM2 (`0x2001C000`), or by hand with
`rust-nm -n <elf> | grep -E 'bss_cold|FOC_DRIVER'`.

### Statics the ISR touches

LLVM's ARM global merging groups small statics used together and addresses
them from a shared base. When the set of statics changed anywhere in the
image, the grouping changed and the ISR's register allocation followed. The
statics the ISR uses are therefore grouped by owner, one static each:

| Static | Contents |
|---|---|
| `oxifoc_core::state::ISR_FLAGS` | Detection, flash, current-offset, drive-setup and motor-temperature flags |
| `oxifoc_f405::control::foc::ADC_SHARED` | Phase current samples, bus voltage, temperatures, driver-ready flag |
| `oxifoc_f405::control::foc::ISR_TIMING` | ISR cycle counters and budget |
| `oxifoc_core::runtime::streaming::FAST_PRODUCER` | Telemetry period, decimation, producer drops, attribution counters |

Add new ISR-shared atomics to the matching group instead of creating a new
static.

## Measuring

### Always-on counters

Every image times the ISR with two DWT reads and reports once per second
through defmt (RTT on the MK5 SPI image; read it with
`probe-rs attach --chip STM32F405RGTx --probe <selector> <elf>`, which does
not reset the core):

```
isr/s: n=14997 avg=2559 max=4804 over=0 load_pct=22
isr-motion/s: n=14997 avg=6094 max=7148
```

| Field | Meaning |
|---|---|
| `n` | ISRs in the window (15 000 per second at 15 kHz) |
| `avg`, `max` | Body cycles from ADC entry to the end of the timed region |
| `over` | ISRs longer than one PWM period |
| `load_pct` | Share of the window spent in the ISR |
| `isr-motion/s` | The same, only for ISRs in active current/velocity control with a nonzero speed estimate |

These lines are at `info` level: build with
`DEFMT_LOG=warn,oxifoc_f405::control::foc=info` to see them (production images
default to `warn`). The bookkeeping after the second DWT read falls into the
next interval.

### Detailed profile

`just build-mk5-profile` enables `isr-profiling`, which adds per-section
lines:

| Line | Sections |
|---|---|
| `isrp/s` | ADC reads, snapshot, `run_foc_cycle`, telemetry publish, remainder |
| `isrc/s` | Commands, protection, step, cycle tail; stopped-path split |
| `isrd/s` | Torque gate, current read, control, trig, post, estimator |
| `isre/s` | Observer, startup, output, tracker |
| `isrh/s` | Sensor sampling, phase voltage, velocity request |
| `isri/s` | Entry-to-entry intervals: average, min, max, long and short counts |

Section timers nest and overlap, and the instrumentation changes codegen and
placement. Use it to locate cost, not to compare commits.

### Other tools

| Question | Tool |
|---|---|
| Actual clocks and sampling phase | `scripts/f405_timing.py` (register readback while stopped) |
| Isolated Cortex-M4 code cost | `just bench-embedded-f405 OUTPUT RECOVERY FILTER`, see [`tests/stm32f405`](../tests/stm32f405/README.md) |
| Commands and physical motion | Host CLI maneuvers, Packed28 capture, `scripts/capreport.py` |
| Hall/observer handoff | `hall-diagnostics` (`just build-mk5-motion`), `scripts/bench_hall_edges.py` |
| Linked code and placement | `scripts/isr_placement.py` (below); `rust-nm -n`, `rust-objdump -d`, `rust-size -A` |
| Sampled execution | `probe-rs profile` (`just profile-mk5`, `just bench-mk5-foc-pc`); needs a fast host |

### Placement tools

`scripts/isr_placement.py` (run inside `nix develop`; `--help` for details)
relinks one set of compiled objects with controlled placement and reports what
the ADC path looks like in an image:

| Command | Does |
|---|---|
| `build DIR` (`just isr-capture DIR`) | Builds the MK5 image (`transport-spi,board-vesc6-mk5`, `DEFMT_LOG=warn,oxifoc_f405::control::foc=info` so `isr/s` is printed) in `oxifoc-f405/target/isr-capture` and captures its final link with `--reproduce`: `DIR/build.elf`, `DIR/build.txt` (commit, features, toolchain, hashes), `DIR/link/` |
| `relink DIR NAME [options]` | Re-runs the toolchain's `rust-lld` on the captured objects and writes `DIR/NAME.elf`; without options the result is byte-identical to `build.elf` (it says so) |
| `residues ELF...` (`just isr-residues`) | Address and residue of every `foc-hot.x` section, and the RAM statics the ADC path references, by block |
| `callgraph ELF...` | Functions reachable from `ADC` by direct calls: instructions, entry residue, call sites; with several ELFs only the rows that differ |
| `diff A B [--hot] [--function NAME]` | Per-function disassembly comparison with addresses and literal values normalised away; `--function` prints the instruction hunks |

`just isr-compare A B` runs `diff --hot` and `callgraph` for two images.

Relink options (`R` is a hex residue, repeatable where it makes sense):

| Option | Placement |
|---|---|
| `--pin LABEL=R` | Moves the `foc-hot.x` stanza `__foc_LABEL_start` (`cycle`, `phase`, `adc`, `svpwm`, …) to residue R |
| `--add-pin LABEL:GLOB@R` | Pins further input sections (e.g. `'wrap:*wrap_angle_general*@2'`) in `.foc_hot` |
| `--foc-hot FILE` | Uses an edited `foc-hot.x` instead of the captured one |
| `--res REGEX@R` | Places `.text` input sections whose name or first symbol matches at residue R; every other section keeps its residue (with `--shuffle`: natural packing) |
| `--resfrom DIR2` | Gives each `.text` section the residue of the same-named function in DIR2's link (code-vs-placement comparison); the `foc-hot.x` sections are pinned already |
| `--shuffle SEED` | Seeded random order of the unpinned `.text` sections, natural packing |
| `--allres R` | Every unpinned `.text` section at residue R |

An explicit `.text` layout is checked against the linker map after the link.
Captures, relinks and target directories live under `target/` and are not
committed.

### Running a comparison on the bench

1. Build every image from the same checkout directory. Absolute source paths
   end up in the image; the same commit built in another directory changes
   unrelated function sizes and shifts unpinned code (a 10-cycle difference
   was traced to this). `isr_placement.py build` also always uses the same
   target directory.
2. Record the ELF SHA-256, features, toolchain, PWM rate, phase source and
   supply (`build.txt` has the first three). Flash with an explicit `--probe`;
   after a failed flash, retry at `--speed 1000` before anything else.
3. Use one maneuver for every image. +120 electrical rad/s runs on the Hall
   angle and +450 on the observer (the bench blend is 150–300), so a change
   to either path shows at one of them. `maneuvers/isr-timing-short.json`
   (3 s stopped, 8 s at each speed, then Stop; 64 s per run including the
   flash) gives the averages: it matched the long maneuver within 0.3 cycles.
   Use `maneuvers/isr-timing.json` (5 s stopped, 20 s at each speed) when the
   maximum matters: the per-second maximum of one image varies by about 500
   cycles between runs, so a claim about it needs long phases and repeats.
   Keep the fast-telemetry rate identical; a slow host (Raspberry Pi 3) cannot
   keep up with native-rate capture, so both capture at 1 kHz.
4. Read the RTT log during the run. `scripts/isr_bench.sh ELF MANEUVER OUT`
   does steps 2–4 on the bench host: it flashes with `OXIFOC_MK5_PROBE` at
   4 MHz SWD (retrying at 1 MHz; flashing takes about 33 s, erase and
   programming dominate), resets, attaches `probe-rs attach` and runs
   the maneuver with the host CLI (`OXIFOC_HOST_CLI`, default
   `oxifoc-host-cli` on `PATH`; `OXIFOC_BRIDGE_SERIAL` selects the bridge),
   leaving `OUT.rtt.log`, `OUT.json` and the capture. It needs only probe-rs,
   the CLI and the two files, so it can run on a separate host. The attach
   halts the core for 50–80 ms, which shows as `n < 14997` and `short=1` in
   `isri/s`. Timing is valid from 1.3 s after the last SWD setup access (a
   probe disconnect can disable DWT until the firmware rearms it); the script
   attaches 3 s before the maneuver starts.
5. Summarise with `uv run --no-project python -B scripts/isr_rtt.py OUT.rtt.log...`.
   It drops the halted windows and reports, per log, the stopped average
   (idle and while telemetry streams) and each motion phase (a run of windows
   at one Hall edge rate, without its first and last window): mean of the
   per-second averages, their spread, the largest maximum. `--json` for
   scripts.
6. Repeat each image and interleave control and candidate. Repeats of one ELF
   agree within ±2 cycles on the average; a difference between two images is
   meaningful only when it exceeds the placement noise of the code you
   changed. One run per image is enough to find an effect of tens of cycles;
   repeat before claiming one under about 10.
7. Restore a verified production image (`just build-mk5-spi`) and confirm
   `status` reports Stopped with no faults.

DWT counts CPU cycles, so ISR timing does not depend on the oscillator's
accuracy. The MK5 assumes the nominal 8 MHz; the bench board's oscillator was
measured about 0.7 % slow, which scales times and speeds but not cycles.

For each accepted motor capture keep: ELF hash and build provenance, board,
motor, supply and PWM/clock settings, scenario and limits, every command ACK
and the terminal Stop, complete stream END totals with zero gaps and drops,
unchanged `fault_generation`, and the `isr/s` lines for the run.

### Separating code from placement

When two commits differ in ISR time, find out whether the code or only its
addresses changed:

```sh
just isr-capture target/isr/a          # at the control commit
just isr-capture target/isr/b          # candidate, checked out in the same directory
just isr-compare target/isr/a/build.elf target/isr/b/build.elf
uv run --no-project python -B scripts/isr_placement.py relink target/isr/b eq --resfrom target/isr/a
just isr-compare target/isr/a/build.elf target/isr/b/eq.elf
```

The first comparison lists the hot functions whose code changed and the
residues that moved. `eq.elf` is the candidate's code with the control's
`.text` residues; the second comparison should show only the code changes.
Bench `a/build.elf`, `b/build.elf` and `b/eq.elf`: the part of the difference
that `eq.elf` keeps is code, the rest placement. A control relink
(`relink DIR ctl0` without options) must report "identical to build.elf".

### Re-measuring a pinned residue

Sweep one pinned function at a time over the residues its alignment allows
(0, 4, 8, c for 4-byte alignment; even values for 2-byte), with everything
else fixed:

```sh
just isr-capture target/isr/sweep
for r in 0 4 8 c; do
    uv run --no-project python -B scripts/isr_placement.py relink target/isr/sweep svpwm-$r --pin svpwm=$r
done
just isr-residues target/isr/sweep/svpwm-*.elf     # check the moved entry
```

Run each image with `isr_bench.sh` (interleaved, at least two repeats) and
compare the motion-phase averages; a candidate function that is not pinned yet
can be swept the same way with `--add-pin`, then given a `link_section` and a
stanza in `oxifoc-f405/foc-hot.x`. Moving a pinned function changes the start
of the stanzas after it, which keep their residues; check with
`isr-residues` that the others did not move off theirs. Write the new residue
into `foc-hot.x` and record the measurement in the commit message.

## Rules for changing the hot path

- Compare disassembly of `ADC`, `run_foc_cycle` and the pinned functions
  against the parent (instruction count, stack accesses, FP operations, calls)
  and say what moved in the commit message, as AGENTS.md requires
  (`just isr-compare`, then `isr_placement.py diff --function NAME`). Treat
  the counts as a hint and confirm a timing claim on the bench.
- Do not call defmt in the ISR; raise an `oxifoc_core::isr_log` event and
  let its task print it. Even with the deferred logger a message costs about
  1 040 cycles in the ISR (serialization, queue grant, worker wake-up): the
  three failsafe-entry messages took that cycle from 6 930 to 10 050 cycles,
  against 7 270 with events. Repeats of an event within 10 ms print once.
- Counters only the ISR writes do not need read-modify-write atomics: a
  relaxed load and store is enough, while `fetch_*` compiles to an
  LDREX/STREX loop.
- Add ISR-shared atomics to the existing grouped statics.
- Do not put statics in `.uninit`; keep new large cold buffers in mind for the
  SRAM2 filler (`rust-nm` check above).
- After a substantial change to a pinned function, re-sweep its residue; after
  adding a new hot function, consider pinning it.
- Keep work that does not need native rate on the slot frame and move rare
  branches out of line with `#[inline(never)]`. Measure before adding
  `#[cold]`: it changes block layout around every call site. Cold on
  `isr_log::raise` made each command cycle about 450 cycles slower (seven
  call sites in `process_commands`), and marking a whole native-rate service
  cold made it slower too.
- Report flash and RAM deltas for MK5, CF2 and G474.

## Known limits and open work

- The pinned residues were measured at 15 kHz in velocity control only.
- The ISR has no complete worst-case bound: observed maxima belong to the
  exercised scenarios, and rare paths are not all covered.
- The CF2 has no pinned layout (`compact-foc` is MK5-only).
- `ISR_TIMING` still updates its counters with `fetch_add`/`fetch_max`; plain
  load/store would be cheaper (unmeasured).

## Rejected approaches

Details and numbers are in the [optimization notes](notes/adc-isr-optimization.md).

| Approach | Result |
|---|---|
| Hot code in SRAM (`ram-foc`) | Slower, with body overruns (Flash with ART is faster) |
| Whole `STATE`/driver in CCM | No useful gain |
| Overclocking | Kept on the `archive/overclock` branch; not for production |
| Global 16-byte function alignment | Moves pinned functions off their residues; gain depends on the code |
| `target-cpu=cortex-m4`, execute-only, custom frame-pointer target | Slower |
| `cold_path()` on the command service | Slower |
| Caching observer algebra or Hall decay | Extra state cost more than recomputation |
| Multirate shadow observer | Did not keep continuous capture |
