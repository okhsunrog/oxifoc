# Motor and telemetry bench journal

This journal retains the MK5 motor tuning, transport and recording evidence
from 10–13 November 2026. ADC ISR implementation, all timing optimization
results, rejected candidates and the current measurement procedure are
consolidated in [ADC ISR optimization](adc-isr-optimization.md).

Hardware: unloaded H5055, Mini V6 MK5, about 24 V. Historical settings,
restoration images and next-action statements below describe their own
sessions; use the current firmware and commissioning guide for new runs.

Artifacts:

- `/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-low-speed-2026-11-11/`
- `/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-current-2026-11-09/`

## Transport qualification, 10 November

The ST-Link V3 read-throughput investigation separated raw SWD reads from
async delivery wait. Dedicated raw telemetry and Ergot paths were compared
with complete capture/sequence/drop accounting. A larger queue absorbs
bursts but cannot fix sustained producer/consumer imbalance. The existing
native-rate recording qualification below supersedes the early lossy USB
screening runs. Exact earlier images, measurements and failed attempts are
retained in Git history and the artifact roots indexed by the ISR report.

## Motor and recording experiments, 11–13 November

### Low-speed SPI baseline and calibration (2026-11-11)

Finite `bench-mk5-motion` recipes use the production SPI/Packed28 publisher,
no ISR timing/profiling or attached RTT monitor. Keep full native 20-kHz data,
512 actual Hall edges and 128 velocity/authority snapshots at 20 Hz. Read SWD
only Stopped; clear counters before each capture. Export faults before testing
fault_count, then abort/recover on any failure. NVM is untouched.

Important configuration difference: persisted Hall has centroids only
(`geometry=null`), unlike the older volatile geometry experiments. Persisted
`velocity=null` means defaults kp=0.01, ki=0.2, accel=500, not the older
0.05/0.05 tests. Do not compare those historical experiments as identical setups.

`motion-baseline`: two repeats each +40, -40, 120→40, 4.8-s active windows.
All six command maneuvers returned success and raw streams had zero drops.
The last transition had fault_count=1 after Stop; category was not exported
because the initial harness checked fault_count too early. Preserve that gap;
no claim of a clean sixth run. Fixed export ordering before the next matrix.
+40: 12/9 reverse edges, nominal-boundary net speed 39.63/42.96 rad/s,
dq peaks 2.38/5.07 A. -40: no reverse edges, -40.11/-40.45 average speed,
but PI feedback still pulses. Last average is not instantaneous speed evidence.

`motion-fresh-hall`: recalibrate in RAM at 5 A (power class computed from stored
R), no --apply/persistence. 342,869 calibration records, zero loss. All six
comparison runs complete/lossless, no faults. +40: reverse edges 5/2,
net 37.05/39.86 rad/s, dq peaks 2.13/2.01 A. -40: 1/0 reverse edges,
net -40.60/-39.84. Both centroids and geometry changed, so do not attribute the
whole improvement to geometry alone. Regulation still oscillates.

Plots/raw summaries: each matrix's `analysis.png` and `analysis.json`; Hall
boundary counts use nominal 60° steps for robust net-travel comparison, with
200-ms windows explicitly quantized. PI feedback and actual current authority
are plotted separately. Stopped FOC/Hall angle differences are not rotor error.

Next `motion-interp100` replays the exact fresh Hall config in RAM, changes
only interpolation threshold 500→100 eRPM, leaves PI/model/limits untouched.
This tests the centroid/interpolation switching policy near the +40 target
(~382 eRPM). Keep all rejected trials, recover the normal image after each suite.


### Low-speed isolation conclusions (2026-11-11)

`motion-interp100`, same exact fresh Hall table as `motion-fresh-hall`:
6 complete/lossless runs, no hard faults. +40 reverse edges 0/3; mixed benefit,
not a standalone cure. `motion-soft-ki` changes only ki=0.2→0.02, kp=0.01,
interpolation=500 unchanged: 6 complete/lossless runs. Forward and both 120→40
runs have no reverse edges, but rest-to-40 initially has no new Hall edges
while Iq ramps to ~1.5 A over ~1.4 s, followed by ~100-rad/s feedback overshoot.
Current is actually delivered; ordinary saturation anti-windup cannot alone
identify unknown motion within a Hall sector. Do not promote this tuning as a
complete startup/regulation fix. Repeated setpoints do not reset the PI state
(driver source inspected).

`motion-hall10`: 10-A RAM-only Hall recalibration, complete native capture,
then 6 clean/lossless regulation captures. Boundaries differ from the 5-A table
by at most 4.83°, centroids at most 2.60°. +40 reverse edges 0/5; no consistent
cure from stronger alignment. All limits/PI otherwise unchanged.

The plotting analysis now uses measured switching boundaries where available,
including zero displacement at a same-boundary reversal. Legacy centroids use
inferred midpoint boundaries. Earlier nominal-60° net speeds above are coarse
historical summaries; updated `analysis.json` identifies the position source.
200-ms interval averages are not instantaneous shaft speed or encoder evidence.
Stopped FOC-versus-Hall differences are excluded from active frame metrics.

Native 20-kHz replay through the actual Rust Hall estimator (`replay-hall`)
for `motion-soft-ki/velocity40-0` matches 20-Hz MCU snapshots with 0.01344-rad/s
RMS error in the moving window. Hall-to-ADC alignment residual <=62.9 µs.
Replay and measured Iq share strong ~6–7-Hz modulation in that window; low-rate
snapshots must not be used alone for spectral claims. Native current PSD in
120→40 captures also retains Hall-frequency/harmonic components at the higher
speed. These observations support speed-linked feedback/commutation effects,
not proof that all intra-sector speed ripple is fictitious. Evidence:
`replay-alias-check.png/json`, `speed-frequency-check.json` under the new root.

Constant-torque isolation uses the same 10-A Hall table, default speed PI
only for spin-up, then CurrentControl at t=1.7 s. The initial longer 0.6-A
attempt overflowed the finite Hall prefix (not the main telemetry stream);
another ended at a host late-command refusal. Keep those failed artifacts.
Final `torque-isolation-bounded` + `torque-midpoint` shorten the plateau to
1.4 s, add explicit volatile speed rolloff 180→200 electrical rad/s and
capture 6 complete/lossless runs with no Hall-prefix drops. The normal
persisted speed limit is OFF (`derating=null`), not the 200-rad/s setting from
older RAM experiments. Do not silently assume old volatile settings survived.

At 0.3 A the motor loses Hall crossings; 0.6 A accelerates to speed derating.
Its later torque is clipped: only pre-derating data can represent fixed Iq.
At 0.45 A mean measured Iq is 0.44698/0.44527 A, RMS tracking error about
0.141/0.146 A, no post-switch reverse transitions in either repeat. Net
boundary speeds ~51.6/59.8 rad/s drift as expected without speed feedback.
The large speed/torque relaxation pattern is substantially reduced with the
speed PI removed, while current/angle ripple remains. Do not claim a perfect
current loop or a complete velocity-control fix. Expected Derating warning
and clear generations in the 0.6-A runs were captured, no hard fault.
Visually inspected `torque-isolation.png` and all comparison grids.

Four calibration/tuning matrices plus the six bounded torque captures contain
30 complete native-rate records / 3,200,414 samples, zero producer/host drops.
Earlier failed diagnostics/baseline fault are excluded from that success count.
All trials retain exact ELFs, source patches, configuration, raw Parquet,
command events and finite Hall evidence. No experimental calibration/gain was
persisted. Final full config equals initial config except automatic DC offsets;
Stopped/fault_count=0, 24.033 V, FET 29.6 °C, motor 28.8 °C.

Next code work should address (1) startup with no fresh Hall crossings and
subsequent PI handover, and (2) speed-dependent Hall feedback ripple with
explicit observability/latency bounds. Test startup and settled regulation
separately; do not mask the tradeoff with a single average-speed number.
Keep clock=168 profile, 20-kHz current/PWM and 1-kHz outer PI for this work.


Startup defect identified after inspecting the rare 6.725-A event at t=0.3159 s
in `torque-isolation-bounded/velocity40-to-iq06-1`: pre-start Hall angle had no
new crossings in the preceding 200 ms. Current dq excursion spans multiple
ADC samples. Stopped disables PWM but retains current PI integrators; Coast
resets them. `ensure_bridge_enabled` currently re-enables without clearing
that voltage memory. Next: regression for off→on with/without an intervening
Stopped/Coast cycle; reset PI in the common rearm path and preserve ordinary
running retarget state. This is a specific restart defect, not yet an accepted
explanation of all moving speed ripple.


### Accepted bridge rearm fix (2026-11-11)

A regression fails before the change: Stopped→CurrentControl retains a seeded
current PI voltage, including Stop→Start with no intervening control cycle.
`ensure_bridge_enabled` now resets the current controller before enabling the
outputs after off/Stopped/Coast. Continuous running retargets do not reset it;
that invariant is part of the same test. No per-cycle native work was added.

Matched `rearm-before` / `rearm-after`: +120 spin, Stop 2 s, rearm Iq=0.3 A,
two repeats each, all native captures complete/lossless and ACKs successful.
Peak dq in first 50 ms: 4.791/5.267 A → 0.772/0.911 A. Initial Vq:
0.360/0.392 V → 0.006/0.006 V. Exact PWM-enable edges align the comparison;
host ACK sequence snapshots are not used as physical application timestamps.
`rearm-before-after.png/json` contains the visually checked transient evidence.
The fix is in the common bridge lifecycle, not a special velocity-mode patch.

`motion-rearm-fixed`: 6 complete/lossless captures, no faults; both ±40 repeats
have zero reverse transitions, net speeds +40.03/+41.33 and -40.11/-40.57.
One 120→40 repeat still has five reverse crossings; this is not a complete
moving-regulation fix. The other has none. PI defaults remain 0.01/0.2.

`deadline-rearm-fixed`: three +120 runs after the code change, all complete and
lossless, zero late/ambiguous/masked driven commits. Minimum margins
9.938 / 10.725 / 10.498 µs. The accepted source therefore preserves the
observed 20-kHz actuation deadline in this workload. Do not label the margin
change itself an optimization gain (layout/trajectory vary).

Use the forthcoming normal fixed build in the new artifact root as recovery
for later experiments; the old `current-light-repeat/recovery.elf` lacks this
rearm fix and is only a historical A/B reference.


### Rejected small dissipative-braking pilot

`brake-pilot`: source fix 44a9790, exact 10-A Hall table, default PI, regen=0,
RAM-only max_dissipation_w=0.3, speed rolloff 180→200. One native capture,
114,680 records, complete/lossless, no fault generations. Granted Iq reaches
-0.234 A and positive Id 1.550 A: braking authority was actually exercised.
+40 has 7 reverse edges, net 36.81 rad/s, dq peak 3.486 A. This is worse than
the matched no-dissipation trials; reject rather than qualify a larger matrix.
The experiment does not establish the exact cause: coarse Hall angle plus
additional Id torque sensitivity is a plausible contributor, not measured
true rotor angle. No speed/thermal/current defaults were changed permanently.

Normal fixed SPI firmware was built and flashed, source fix 44a9790, before
this pilot; its exact build artifact is `normal-fixed.elf/json` in the latest
root and was the pilot's automatic recovery. Use that recovery going forward.
Final rebuild/flash aligns the default target ELF with the running normal
image (a bench ELF left in the target directory is not its defmt decoder).

### Speed dependence and tuning, 2026-11-11

New low-speed root above contains exact ELFs, configurations, native Packed28
Parquet, command events and complete finite Hall prefixes. Added just recipes
for long settling, RAM-only gains, speed sweep, and interpolation comparison;
new harness runs also retain the exact executed maneuver JSON. No NVM changes.

Long +40/-40/120→40, two repeats per case and tuning: default Kp/Ki=.01/.2
has 17 reverse Hall transitions across six runs; .01/.02 has zero; .015/.05
has four. In the 120→40 repeats: 4/4 → 0/0 → 2/1 respectively. Mean late
speeds remain near 40, but smoothing gains is a tradeoff: observed first full
electrical turn at ±40 takes .25–.37 s by default versus 1.10–1.44 s with
Ki=.02. Do not replace universal defaults based only on this unloaded motor.
`comparison.png/json` includes all repeats; plotted native Iq uses 5-ms means,
not the aliased 20-Hz bench snapshots. Actual Rust 20-kHz Hall replay matches
MCU velocity snapshots (RMS .017/.008 rad/s for two transition examples).

Speed sweep, default and soft Ki, two repeats each: +20,+80,+160,+320,-20,-320
electrical rad/s (7 pole pairs: ±320 is ±437 mechanical RPM). Durations shorten
at higher speeds to retain every Hall edge; ±320 has only ~0.8 s of late data,
so this is not long-duration qualification. +80 late net 79.91–80.04, +160
159.55–159.95, ±320 magnitudes 316.69–320.08. Default +80 200-ms Hall-travel
P5/P95 about 79–81.2, +160 about 159.3–160.6. At ±20 there is pronounced
rocking/stick-slip; one default +20 run has 64 reverse edges and zero net late
boundary travel. Soft Ki also fails to regulate 20 consistently. This is a
low-speed problem, not inability to run velocity control at every speed.
`speed-sweep.png/json` records currents, speed windows, voltage and all repeats.

Threshold=0 RAM pilot repeated at +20/+40/+80, two each: +20 still 29/8
reverse crossings and net 11.23/19.23; +40 2/0 and net 40.33/40.21; +80
0/0 and ~79.99. This changes both interpolation gating AND the commutation
slew floor (currently coupled to the threshold), not a pure interpolation A/B.
Reject zero threshold as a general fix; keep 500 eRPM default for now.

48 successful captures total 5,448,833 samples, no producer/host loss and no
controller faults. Separately retain THREE aborted attempts: `speed-default`
+80 and `interp0` +20 hit the host's 5-ms timeline-lateness refusal; samples
were preserved losslessly but the maneuvers are incomplete. `speed-soft` +160
also overflowed the host raw FIFO by 64 samples, with late affirm warnings and
subsequent NoRoute errors. No hardware fault; this is a real host reliability
limitation, not hidden in the successful count. Repeats succeeded. Parallel
analysis was disabled after the first failure, so it does not explain all three.
Do not silently loosen schedule tolerances or call the whole batch lossless.

### Native velocity PI evidence

`--velocity-debug` opts into the diagnostic Arm on an Ergot Packed28 native
capture. The ISR publishes exactly on velocity PI updates plus a session
origin at the first native ADC cycle. Separate 32-byte records contain actual
PI feedback/reference/target, integral, Iq request/grant/bounds, Id grant,
voltage/blockage flags, Hall pin bits/direction/edge age, full FOC sequence and
clock ticks. Speed unit 1/8 electrical rad/s; current 10 mA; age 100 µs. The
normal Packed28 frame and control behavior are unchanged. Diagnostics share
its session and block index, with their own END, count and loss checks. Data
blocks fit in the spare tail of the existing Ergot envelope, so there is no
extra send in the normal running path. Payload ~32 kB/s plus ~10 kB/s headers
while the PI runs. The host saves `<capture>.velocity.csv`, time-aligned to
the native samples including sequence wrap. No calibration or gain writes.

Final normal-image runs use the same 168/20k/1k clocks. Two +120 runs,
120→40 and velocity→Iq=.45 A: 438,598 native samples, 16,815 diagnostic
records, all 95 planned command ACKs, no drops in either stream and no faults.
Normal-image footprint change: +6048 flash bytes (.text/.rodata/.data),
+2660 static RAM bytes (.data/.bss), including the 2-KiB diagnostic queue.
Tests: 702 core, 14 host raw, 4 maneuver passed. Final stopped, 23.977 V,
fault count 0; persisted config unchanged except boot DC offsets.

Do not claim full PWM deadline qualification with diagnostics. First debug
deadline trial (separate envelopes) lost ACK and 102 producer samples; the
coalesced variant had an ACK failure with no loss, then an ACK failure and
84 producer drops without RTT. All saved short timing prefixes had zero
late/ambiguous/masked writes with minimum margin 9.55–9.62 µs. The root cause
of these instrumented-run failures is unresolved; RTT alone does not explain
them. Ordinary firmware passed the four complete diagnostic runs above.

The first transition trace (`velocity-debug-pilot`) records 5804 PI updates
at 952–1050 µs intervals. Quantized integral and granted Iq can now distinguish
speed feedback collapse from voltage saturation or rejected current. See
`normal-mux-velocity120-to40.velocity.png` for the current format. For a
focused independent review, the artifact root contains `fable-review/PROMPT.md`
and `EVIDENCE.md` with exact A/B, raw data and source entry points.

### Hall-to-observer crossover at 320 electrical rad/s

Artifacts are in `/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-hall-crossover-2026-11-11/`.
The long maneuver drives for 7.6 s, commands 320 for 5.8 s, then commands 40
for 1.8 s before Stop. The test replays the measured Hall geometry in RAM and
uses the default `hall-fallback` blend band of 150–300 electrical rad/s.
The crossover build samples the applied blend weight, selected angle and
rotor feedback, observer output and Hall health at 40 Hz. The 512-edge
prefix intentionally covers only the start of this longer maneuver.

Three complete control runs (`priority`, `half-rate` twice) held full
observer weight for about 5.3 s at 320 (~437 mechanical RPM). At weight 1
the selected rotor feedback equaled observer speed and the selected angle
matched observer phase to floating-point precision. The weight reached 0
about 0.35–0.4 s after the step to 40 and stayed at 0 until Stop. The usual
low-speed feedback oscillation returned. Hall health and angle trust were
good at every 40-Hz observation, with zero recorded faults.

The complete 20-kHz attempt had all command ACKs and 37,705 STM32 producer
drops. Shorter tests and the normal image also lost data, and a 48-KiB CCM
queue did not solve it. Raising the SPI pump task priority was a bench-only
setting used for the complete runs. With every second FOC sample exported,
two repeats recorded 170,475 samples without losses or missed ACKs. TIM1 and
the current loop remained at 20,004.27 Hz. Normal firmware was restored,
stopped in Hall mode. Continuous 20-kHz recording during this blend remains
unqualified. `half-rate/analysis.json` and the PNGs contain the aligned data.

### Low-speed cogging and velocity PI

Artifacts are in `/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-cogging-2026-11-11/`.
`sim_trace` replayed the measured Hall geometry and motor model with the
firmware controller, dry friction and a fitted 0.0174 N·m second electrical
harmonic peaking at 137°. At +20, the 200-ms travel speed P5–P95 widened from
18–22 to 1–38 electrical rad/s. At +40 it widened from 40 ± 0.2 to 20–62.
The same ripple appeared after 120→40. +80 remained steady on that time scale.
The model reproduces stop-and-go motion and overshoot. It did not reproduce
the bench's reverse Hall crossings with these plant parameters.

Six native-rate runs at ±20 exported 34,814 PI updates and 807,877 FOC samples
without loss. With regeneration capped at zero, the Iq bound in the drive
direction reached zero in 3,843 updates. The integral magnitude was at most
0.01 A in 3,175 of them. For 158 of 163 reverse Hall edges there was a
zero-bound, near-zero-integral PI observation within `[-25, +70] ms` of the
edge. This confirms the predicted clipping path while retaining the PSU limit.

`just bench-mk5-cogging` compares off, 0.25 A at 47°, and the same amplitude
with the second harmonic inverted. Two matched repeats per arm used the
same 1-kHz PI, Hall geometry and 20-kHz capture. At +40, both compensated
repeats raised late speed P5 from 5–10 to 30–31 rad/s. Inversion raised it to
26–28 with a larger P95 of 60. Across ±20, compensation reduced reverse
Hall edges from 44 to 26 and zero-drive-authority PI updates from 1,822 to
945, while substantial ripple remained. 120→40 had no repeatable gain.
+80 settled around 79–81 rad/s in all arms. The Hall prediction cannot
resolve exact position within a sector during a stall.

The final guarded build repeated the three +40 arms. Off, compensation and
inversion gave P5–P95 of 32–52, 28–57 and 39–45 rad/s, respectively. The
starting Hall states varied. Across all three +40 repeats, reverse edges
fell from 10 to 1 with compensation, but its speed-spread benefit did not
repeat consistently. The 39 total runs recorded 5,155,418 FOC samples and
221,505 PI updates without transport loss or faults. Normal firmware was
restored, stopped in Hall mode, and the stored settings matched the
pre-test dump apart from boot DC offsets.

Compensation stays bench-only. It shares the original torque bounds and is
applied only on velocity updates with the Hall source and a healthy sensor.
The experiment preserves the zero-regeneration and zero-dissipation limits.

### Motor acknowledgements under 20-kHz capture load

Evidence is in `/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-stream-reliability-2026-11-11/`.
With the measured Hall geometry, the normal STM32 build reproduced 2,150
producer drops and missed the 40-rad/s step and terminal ACK in the 320→40
Hall-to-observer maneuver. Pure Hall at 20 kHz and the same hybrid maneuver
at 10 kHz passed. SPI error counters were zero. The profiled hybrid failure
processed 126 motor requests and rejected 116 ACK sends at the STM32 route.
Its mean ISR time was 45.25 µs in the 50-µs period, versus 43.00 µs on Hall.
The instrumented run lost more samples than the normal image.

The raw SPI sender now waits for contiguous space for its block and one
reply MTU, then enqueues through the original FIFO. No extra queue RAM or
wire changes. A separate priority FIFO candidate was rejected after it lost
the route during capture setup. Packing now reads aligned telemetry grants
directly, with an unaligned fallback. Full handoff uses the observer's
normalized angle and speed directly.

With these changes the profiled hybrid build completed both 320→40 runs.
All 158/158 motor requests reached the ISR in each, with zero ACK send
errors and all timeline ACKs within 48 ms. Recording still lost 35,420
and 34,223 producer samples, respectively. The matched pure Hall run
remained lossless at 20 kHz. Mean hybrid ISR time was 44.55–44.56 µs.

The non-instrumented candidate also completed both 20-kHz hybrid runs,
with all motor requests applied, zero local ACK send errors, and a maximum
timeline ACK of 23.16 ms. It still dropped 1,729 and 1,239 producer samples
respectively. The sequence gaps concentrate around the Hall/observer
transitions, with a clean sustained +320 plateau. The 10-kHz recording
passed with zero loss while FOC still ran at 20 kHz. No C5 changes.
The previous normal image was restored and verified stopped.

### Raw telemetry queue size at native 20 kHz

On the same motor and Hall geometry, built the normal SPI release from
`efa7e8f` with the 4, 8, and 16 KiB ISR telemetry queues. Only that capacity
changed. Three repeats per capacity alternated build order and used the
same 320→40 Hall/observer maneuver. Evidence, exact build patches and the
gap plot are in `/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-queue-sizing-2026-11-11/`.

| Queue | Missing samples in the three runs | Median | Max timeline ACK |
| --- | ---: | ---: | ---: |
| 4 KiB | 1,289 / 1,707 / 1,788 | 1,707 | 19.3 ms |
| 8 KiB | 1,308 / 1,689 / 1,797 | 1,689 | 19.7 ms |
| 16 KiB | 711 / 746 / 592 | 711 | 27.5 ms |

All 27 timeline commands and nine terminal Stops were acknowledged.
STM32 producer drops matched the gaps in sequence numbers. All nine runs
had zero host drops, SPI errors, local ACK send failures and active faults.
The +320 plateau stayed clean. The 16 KiB queue reduced startup losses to
0/18/0 samples, while the return to Hall still lost 592–728 per run.
A preliminary 4 KiB run had one unclassified fault entry after the
deceleration, all ACKs and 1,828 producer drops. It is kept as `pilot-4k`.
The default remains 4 KiB. The previous bench firmware was restored,
stopped in Hall mode with zero active faults.

The 32 KiB follow-up used the same release settings, measured Hall geometry
and maneuver. Losses were 35 / 182 / 146 samples, all between 6.48 and
6.74 s during the return to Hall. Median loss was 79% lower than at 16 KiB.
All nine timeline commands and three terminal Stops were acknowledged, with
a maximum timeline ACK of 21.7 ms. SPI errors, host drops, local ACK send
failures and active faults were zero. The 32 KiB image used 93.2 KiB of main
SRAM for static data, leaving 34.8 KiB for stack and margin. Default capacity
and the restored bench firmware remain unchanged. Artifacts are in
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-queue-sizing-2026-11-11-32k/`.

With 48 KiB in main SRAM and the same release settings, three full
320→40 runs recorded 512,082 samples without gaps. The second attempt aborted
on missing raw block 956 during the +320 hold and acknowledged its terminal
Stop. STM32 producer drops and master SPI error counts were zero in all four
attempts. The block loss was not localized. All sent timeline commands and
terminal Stops were acknowledged, with a maximum timeline ACK of 15.3 ms.
One completed run recorded a Hall stale-edge warning after deceleration.
The image used 109.2 KiB of main SRAM for static data, leaving 18.8 KiB for
stack and margin. The previous bench firmware was restored and verified
stopped, with zero active faults. Artifacts are in
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-queue-sizing-2026-11-11-48k/`.

The 48 KiB CCM follow-up moved only the queue to `0x10000000`, using the
existing CCM startup copy. ELF symbols confirmed that the FOC driver,
platform state and SPI queue stayed in main SRAM. Static main SRAM fell
from 109.2 to 61.1 KiB. All three full runs captured 512,051 samples
without loss. All nine timeline commands and three terminal Stops were
acknowledged, with a 35.2 ms maximum on the timed Stop command. There
were zero SPI errors and one Hall stale-edge warning after deceleration.
The previous 4 KiB firmware was restored and verified stopped with zero
active faults. Artifacts are in
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-queue-sizing-2026-11-11-48k-ccm/`.

MK5 SPI releases now keep the 48 KiB queue in `.ccm_bss` (`NOLOAD`).
Startup enables CCM and clears it in place. The previous initialized CCM
sections still use their copy path. The queue occupies 49,172 bytes at
`0x10000000` with zero load bytes, reducing the measured flash load from
436,116 to 386,960 bytes. Main static SRAM remained at 62,608 bytes.
A marker written inside the stopped queue was cleared by a warm reset.
Three full 320→40 runs recorded 512,013 samples at 20 kHz without loss,
with nine timeline ACKs, three terminal Stops, maximum ACK 30.8 ms, and
zero faults or SPI errors. The new release is installed and stopped in
Hall mode. Artifacts are in
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-ccm-bss-2026-11-11/`.

### MK5 PWM frequency at boot

On MK5, `config set pwm-config freq_hz=15000` saves the requested rate.
Use `20000` for the default rate. Restart to apply it, then read
`config get pwm-config` to compare the saved and active rates.

On the same SPI release image, TIM1 read back 15,001.416 Hz (ARR 5594) and
20,004.272 Hz (ARR 4195). ADC1–3 retained the three injected ranks and
15-cycle acquisition. The ADC IRQ counter confirmed one interrupt per
PWM period within the SWD timing uncertainty. Stopped native-rate
captures held 30,288 and 40,485 samples without gaps or producer drops.
The MK5 was left stopped at 20 kHz. Raw evidence is under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-mk5-pwm-config/`.

### MK5 15/20 kHz drive comparison

Same deadline-instrumented image, Flipsky H5055 without load, Hall fallback,
5 kHz Packed28 recording. Two 320→40 runs and two staged +640 runs per rate.
All commands and terminal Stops acknowledged. Zero gaps across 368,909
samples and zero late, ambiguous or masked PWM commits across 1,162,367 writes.
Minimum PWM-write margin was 8.65 µs at 20 kHz and 23.65 µs at 15 kHz.
TIM1 and all three ADC register contracts were checked at both rates.

At +320, the medians of Hall travel over 200 ms were 319.91–319.97 electrical
rad/s. At +640 they were 639.42–639.81. The speed and current traces overlap
closely at the higher speeds. The residual oscillation and short current peaks
near +40 occur at both rates. The recorded FOC–Hall angle difference gives
an indirect view of the handover.
`manifest.json`, `comparison.json` and both overlays are under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-mk5-pwm-ab/`.

### Independent timing for detection

MK5 changes the carrier with the bridge off, reprimes TIM1 and the injected
ADC trigger, and updates FOC `dt`, duty limits and software dead-time correction.
The measurement lease covers restoration of drive timing before the terminal
reply. Packed28 preserves ARR across the transition. The Parquet sequence
timebase records each rate segment and excludes the short pause in TIM1.

Full Medium Outrunner detection at 5 A input / 0 A regen with 15 kHz drive
recorded 653,291 rows without gaps or producer drops. Every stage contained a
15→20→15 kHz sequence and the controller ended stopped without faults. The
measured model was 63.49 mΩ, 31.97/51.39 µH and 3.993 mWb, with all six Hall
states valid. The cancel test interrupted L while running, returned to 15 kHz
within the 18.65 ms measured cancellation round trip, and recorded 17,340 rows
without gaps. Both tests ran in diagnostic mode without `--apply`.
Artifacts are under `/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-detection-pwm/`.

The R sweep varied only the R-stage carrier. Two runs per frequency at
2.4 and 12.0 A Id, with 15 kHz drive and the same 5 A / 0 A source limits.
All 669,718 samples arrived. Firmware R in mΩ was 63.766/63.930 at 5 kHz,
63.921/63.959 at 10 kHz, 63.856/63.915 at 15 kHz and 63.780/63.793 at 20 kHz.
The total spread was 0.193 mΩ (0.30%). Fitting the two settled plateaus from
the measured phase voltages independently gave 63.675–63.888 mΩ. The 5 kHz
trace shows a similar voltage shift at both plateaus, so it largely cancels
in ΔV/ΔI. Keep the R-stage default at 20 kHz for this workflow. The sweep
compares repeatability on one H5055. The plot and plateau data are `r-sweep.png` and
`analysis.json` in the same artifact directory.

### Native-rate recording on MK5

On `fee9f1d`, repeated 320→40 and staged +640 twice per condition on the
unloaded H5055, with Hall fallback and Packed28 over the C5 bridge. Checked
TIM1 and injected ADC registers at each PWM rate on the same release image.

| PWM | Requested / actual capture | Received rows | Producer drops | Worst timeline ACK |
| --- | --- | ---: | ---: | ---: |
| 15 kHz | 15 kHz / 15,001.416 Hz | 553,656 | 0 | 9.07 ms |
| 20 kHz | 15 kHz / 20,004.272 Hz | 676,534 | 62,248 | 49.87 ms |
| 20 kHz | 10 kHz / 10,002.136 Hz | 368,985 | 0 | 11.49 ms |

`capture.fast_hz` and `record --fast-hz` request an integer stride. A 15 kHz
request uses stride 1 at either PWM setting. At 20 kHz PWM, stride 2 gives
the verified 10 kHz mode. The native 20 kHz captures lost 6–11% per run from
the STM32 producer queue, with the highest loss around slower rotation and
the return toward +40. Host drops were zero. All 54 timeline commands
were acknowledged. The motor ended stopped with zero faults at the original
20 kHz setting. Loaded comparison is pending a reproducible load fixture.

Captures, overlays and `loss-and-ack.json` are under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-mk5-native-rate-ab/`.

First light-load pilot on the H5055 at 15 kHz PWM and capture, +160
electrical rad/s, Hall fallback. The hold ran for 20.0 s with 333,453
samples and zero drops. The three main load intervals raised 100 ms mean
Iq from about 0.23 A unloaded to a maximum of 0.96 A. Hall travel over
100 ms had a median of 159.9 rad/s, a 5th–95th percentile of 156.2–165.2,
and extremes of 150.8 and 184.8 during load transitions. The bus stayed
near 24 V. The controller finished stopped, with zero faults, at 15 kHz.
The applied torque was unmeasured. Plot and capture are in
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-mk5-15k-load/`.

The 20 s repeat at +350 electrical rad/s recorded 333,457 samples without
loss. At low load it held near +350. Late in the run, 100 ms mean Iq
reached 7.8 A, versus 0.96 A at +160. Hall travel showed a dip to roughly
220 rad/s and a rebound to roughly 720 rad/s around 19.8 s, followed by
oscillation until the scheduled Stop. The FOC speed estimate also showed
the rebound. Stop was acknowledged and the controller ended fault-free.
The load was unmeasured and the release capture has no observer blend
weight or velocity PI state. See `load-350.png` and `350-statistics.json`
in the same directory.

The 20 s current-control run commanded Iq=1 A, Id=0 with Hall fallback at
15 kHz. It recorded 333,454 samples without loss and ended stopped with
zero faults. Iq averaged 1.03–1.06 A on two loaded intervals. Around
2,970 electrical rad/s, the PWM span reached the configured 95% and
Iq averaged 0.33–0.35 A on two low-load plateaus. The voltage command
was about 13.09 V at a 24 V bus, consistent with modulation saturation.
The plot and summary are `load-current-1a.png` and `current-statistics.json`
in the same directory.

Two unloaded 20 s current-control runs requested 1 A with a volatile speed
cut at 650 electrical rad/s, 15 kHz PWM and capture, and Hall fallback.
With start fractions 0.8 and 0.25, the 100 ms Hall speed peaked at 737 and
733 rad/s, then settled at medians of 649.8 and 644.3 rad/s. The 5th–95th
percentile of plateau Iq averaged over 5 ms narrowed from 0.01–0.88 A to
0.20–0.33 A. Mean Iq stayed around 0.266 A. Widening the taper reduced
steady-state current pulsing. The initial overshoot remained. Both captures
were lossless and the original derating config was restored after each run.
See `speed-cut-comparison.png` and `comparison.json` under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-mk5-speed-cut/`.

The configuration v20 follow-up recorded native Packed28 at 15,001.4 Hz
and current grants every 16 cycles on the Flipsky bench. Two unloaded
1 A runs at a 650 electrical rad/s ceiling recorded 105,413 and 105,428
samples with zero gaps. For taper starts of 80% and 25%, the 100 ms
speed peaks were 705 and 696 electrical rad/s, with steady medians of
648 and 644. Granted Iq reached zero at the first speed crossing in both
runs. On the 3–5 s plateau, its 5th–95th percentile narrowed from
0.21–0.31 A to 0.24–0.27 A. The current trace flagged only speed limiting.
See `current-speed-taper.png` and `current-speed-taper.json` under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-config-v20/`.

The same directory contains the post-reboot Hall-to-observer run at
150/300 electrical rad/s blend thresholds. The 320→40 maneuver recorded
58,900 samples with zero loss, acknowledged all 16 commands, and stopped
with zero faults. Median native speed was 320 electrical rad/s from 1.0 to
1.6 s. The return to the Hall range still showed low-speed oscillations.
`feedback-320-to40.png` shows velocity feedback and the FOC/Hall angle
difference. The capture does not carry the observer blend weight.

### Hall policy and high-speed comparison

See [Hall-to-observer tuning](hall-observer-tuning.md) for the estimator
and the next controlled comparisons.

Unloaded H5055, MK5 at 24 V, PWM and injected ADC at 15,001.4 Hz. Compared
Hall without crossover, Hall with 150–300 electrical rad/s crossover, and
the same OxiFOC algorithm with the VESC 2500–4000 eRPM thresholds
(262–419 electrical rad/s). Two complete 450→320→40 runs per mode, no
capture gaps or missed commands. One host scheduling abort was repeated.
At 320 the observer had full weight with our thresholds and about 37% with
the VESC thresholds. Both returned to Hall near the bottom of their bands.

Two matched runs per mode at 900 and 1500 electrical rad/s, and one per
mode at 2300. At 1500, mean measured dq current magnitude was 0.91 A with
the observer and 2.54 A with Hall. At 2300 it was 1.01 A vs 3.24 A at
approximately the same median speed. Each source supplied its own speed
estimate, so the narrower reported speed range with Hall does not establish
smaller mechanical speed ripple. All six captures were lossless.

Injected the invalid Hall state `111` at the Hall estimator input in a
diagnostic build. At ~454 electrical rad/s and again at ~320 inside the
VESC threshold band, the controller logged `HallError` and continued on
the observer. On the approach to 40 it withdrew torque once the angle
became untrusted. These were unloaded runs with injected loss of all three
Hall inputs. The result leaves a loaded crossover and physical or single-wire
dropout for separate tests. Keep the present lock check, recovery policy,
and thresholds pending those measurements.

Plots, native captures, register readback, and the injection patch are in
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-hall-policy.tbY1Ec/`.
The production SPI image was restored and reported Stopped with zero faults.

## Earlier Hall and velocity experiments, 10 November

### Hall handoff diagnostic follow-up (2026-11-10)

Finite RAM events now capture Hall-health changes, commutation/observer angles,
edge age and observer validity. Opt in with `just build-mk5-raw ergot hall-phase`;
normal builds exclude its 3.5 KiB buffer and ADC work. Export only after Stop:
`just dump-hall-mk5 output.json exact.elf [prebuilt-cli]`. Probe attach can halt
briefly; this is not a live, nonintrusive export. Both feature modes build, and
the decoder was exercised against device captures and its doctests.

The proposed measured-sector/observer-angle stale guard was **not accepted**:
`guarded-soft` still reported a Hall warning and lost three producer samples.
Observer phase was already about 0.85 rad ahead of Hall re-entry, so it cannot
yet serve as an independent position witness in this regime. The candidate
was removed from the working tree; its patch and all failed evidence remain
under `/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-hall-speed-2026-11-10/`.

### Hall repeatability and sweep current (2026-11-10)

Nine runs captured 4,246,674 native-rate Packed26 records through Ergot with
zero producer/host loss, sequence gaps or Hall overcaptures. At 3 A and
1 degree/5 ms, all three native calibrations failed LowConfidence: one boundary
varied by 17 degrees across turns. At 10 A it varied by at most 4 degrees;
three native 17-A calibrations passed with worst ranges of 2–3 degrees and
averaged boundaries repeatable within 0.67 degrees between runs. These are
electrical-angle repeatability figures, not absolute accuracy. Slower 10-ms
diagnostic sweeps also passed offline checks; the 17-A case covered all seven
electrical turns per mechanical revolution in each direction.

The high-current runs used a temporary 30-A dq trip, with observed peak
17.477 A. The historical 6-A bench trip is not a board/motor rating. Local
VESC 7.00 wizard source selects Hall current as detected motor limit / 3,
about 17 A for the user's 50.93-A result; its PI gains are 0.0005/1 versus
0.01/10 in these sweeps. Sixfold Iq disturbances near phase-current zero
crossings repeat at 1.666/3.289 Hz in slow/fast sweeps. PWM-command deadtime
compensation and PI tuning are candidates for the reported pulsating sound;
no synchronized audio or causal A/B has established its source yet.

Original Hall table, model, limits and board calibration were restored and
read back; Stopped, zero faults, no NVM writes. Velocity stability remains
unqualified. Detailed results, source comparison and plots:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-hall-repeat-2026-11-10/report.md`.

### Command deadtime correction A/B (2026-11-10)

`just build-mk5-raw ergot normal on|off` selects command correction independently
of the unchanged 360-ns gate deadtime. Four matched 17-A Hall sweeps, repeated
in reverse order, confirmed the sharp sixfold current disturbance comes from
command correction in this regime. Forward Iq RMS was 0.230–0.231 A with
normal PI/correction, 0.099 A without correction, 1.024 A with soft VESC-wizard
PI/correction, and 0.165 A with soft PI/no correction. Both repeats closely
agree. Normal PI/no correction also passed native Hall calibration at 17 A.

This is not acceptance of globally disabling correction. At +40 electrical
rad/s, new Hall geometry barely improved velocity RMSE (11.70 to 11.49 rad/s);
disabling correction increased it to 21.77 rad/s and reported a Hall-staleness
warning. The batch stopped before its planned repeat. No guard was weakened.
All twelve captures (3,375,368 records) were lossless, including the failed
velocity run. Every plotted Hall edge was retained in the short velocity tests.
The centroid/interpolation transition around 500 eRPM and Hall/observer health
remain diagnostic targets; stable velocity control is not established.

Both firmware variants build. Restored compensation-on firmware and original
runtime configuration, Stopped; no NVM writes. Report and inspected plots:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-hall-ab-2026-11-10/report.md`.

### Velocity PI with unavailable braking (2026-11-10)

Finite RAM snapshots exposed positive current requests during overspeed:
full back-calculation at a zero braking limit manufactured integral to cancel
the negative P term. With Kp=0.05/Ki=0.05 and a 40-erad/s target, the old code
reached 1942 erad/s feedback and a 25.4-A measured d-current peak. Its 128-edge
diagnostic prefix overflowed; full-run speed statistics from that prefix are invalid.
`ClampedPI` now uses conditional integration and bounds stored authority;
the current-loop PI remains unchanged. A regression reproduces the old failure
and passes with the corrected integration.
The same gains after the fix gave 5.74-erad/s full-revolution speed RMSE,
4.05-A dq peak and no faults, but reverse edges and ripple remain. A new
200-erad/s safety ceiling starts derating at 180; sampled feedback stayed below
101 in that run. This is not qualification of smooth velocity control.
Captures, exact ELFs and Hall replay: `oxifoc-hall-motion-2026-11-10` under
`/home/okhsunrog/tmp_zfs/codex_tmp/`. All captures so far have complete USB records.

Voltage limits now install through the stopped ISR transaction, with the same
prepared thresholds used by fault integration, UV recovery and failsafe braking.
Boot uses that validation path; writes cannot exceed the board range. Hardware
checks with PWM stopped produced OV/UV faults at deliberately crossed thresholds,
rejected 58 V, and verified persistence/reboot of the 8–26 V bench range.
668 core tests and 11 doctests pass. Evidence and exact ELF:
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-voltage-brake-2026-11-10/`.

Hall timeout now additionally requires independent observer angular travel
since the captured edge, cancelling a constant observer phase offset. Inferred
crossings after a glitch retain the conservative timeout until a real edge
arrives. Simulation regressions cover cable dropout and glitch recovery.
Physical cable removal was not tested.
Final `witness-fast-off.elf` runs at +40/-40 electrical rad/s retained
82,230/82,173 native 20-kHz samples with no losses or Hall warnings. They still
had 7/5 reverse Hall transitions: smooth velocity control is not qualified.
An earlier reverse run lost 25 producer samples; a later clean run does not
establish that intermittent loss is fixed. Command deadtime correction was
disabled in these comparisons; hardware gate deadtime remained 360 ns.

The matched 3-W dissipative-braking experiment is rejected: 18.60-A dq peak,
7,817 producer losses and repeated Hall warnings, with 42–43-us mean FOC and
peaks around 60 us. Its finite Hall buffer overflowed, so its full-run speed
statistics are invalid. The algebraic solver fast path preserves power/current
bounds in tests but is not a demonstrated cure for the runtime problem.
Next isolate the large negative Id request and Hall-frame error during braking.

The follow-up isolated Id polarity with zero requested Iq: the +2 A staircase
produced no Hall transitions, while the otherwise identical negative staircase
produced 14, including reversals. Both 20 kHz captures were complete. The heat
allocator now defaults to aligning positive Id and preserves an explicitly
requested d-axis direction; the magnitude/power solve is separate. A physical
Hall-plant regression reproduces restoring versus destabilizing behaviour.

With the same 40 rad/s profile, 3 W heat budget and PI 0.05/0.05, the old sign
gave a 17.91 A dq peak and 6507 producer drops. The final implementation's
forward/reverse peaks were 4.09/4.63 A, with no Hall warning generations, but
43/36 producer drops and remaining speed pulsation. This fixes a contributor,
not smooth-speed or lossless-stream qualification. Evidence remains in the
same artifact directory (`id-sign-results.json`, `results.json`, exact ELFs,
20 kHz Parquet, stopped Hall exports and plots). NVM dissipation stays off.

The 0.01/0.2 PI and a 100-eRPM interpolation floor did not resolve the 40-rad/s
pulsation. At -120 rad/s, the full Hall trace had no reversals and a -125.62
rad/s median revolution speed, but 49 producer samples were lost. The 120→40
step reintroduced slow-speed pulsation (2.09 A peak, 323 lost samples, no Hall
warnings). The bench edge buffer is now 512 entries to retain these runs;
the exporter also accepts the older 128-entry and seven-word velocity layouts.
Offline replay (`hall-speed-audit.json`) shows that the current edge-speed
estimate does not integrate to observed net boundary travel during rocking;
do not equate a low-pass of that signal with independent rotor-speed evidence.
Raw captures, configurations, exact images, plots and failed runs remain in the
same artifact directory; `results.json` distinguishes incomplete recordings.
Final flash configuration, verified after reset: drive 10 A, dq trip 30 A,
bus draw 5 A, regen 0 A, brake ceiling 5 A, dissipation 0 W, VBUS 8–26 V.
Controller left Stopped with no faults; motor model and Hall NVM were unchanged.

### Hall travel-consistent speed (2026-11-10)

Control samples now differentiate bounded physical boundary travel with a
20-ms filter; raw edge speed and the commutation-angle limiter remain separate.
Captured displacement is accumulated before sampling, including zero net travel
at a same-boundary reversal. This removes the unbounded integral of the previous
`sector_width / edge_age` stop estimate. Six trajectory/regression tests cover
stops, asymmetric rocking, unequal sectors, both directions and missed sample
cadence.

Replaying four complete earlier captures changes the mean-speed disagreement
against net boundary travel from +6.51/-6.95/+1.98/+6.64 rad/s to
+0.14/+0.52/+0.10/-0.67. Finite-window filter lag remains. First hardware runs at
40 rad/s with PI 0.05/0.05 and 0.01/0.2 still oscillate (13/5 reverse edges,
4.18/2.44-A dq peaks), without faults; producer losses are 404/392 samples.
These runs establish neither smooth regulation nor streaming acceptance.
Artifacts: `motion-*` in the existing `oxifoc-voltage-brake-2026-11-10` directory.

`just build-mk5-raw ergot hall-outliers off` now retains Hall evidence alongside
section costs, including the due-only velocity update. In `motion-profile-1`,
the eight longest instrumented ADC bodies were 55.6–58.9 µs: velocity work
4.35–7.86 µs, estimator work 11.34–12.84 µs, current controller 4.67–4.78 µs.
All eight coincide with the outer velocity update; six also have expensive
command processing. Detailed profiling adds overhead and lost 26,393
producer samples; it is not representative streaming acceptance.

Bench Hall images also retain 16 fault-generation/category/speed snapshots,
exported by `dump-hall-mk5`. A stopped-only temporary thermal-derating test
verified both Warning/Derating and its subsequent clear, with exact config
restoration. This closes the evidence gap where the host saw a changed registry
generation but no remaining active fault. The older `motion-forward120-2` run
had that gap (generation 2, startup feedback 190 rad/s versus a 180-rad/s
rolloff); speed derating is inferred there, not a captured fault-category fact.

The completed matrix repeats +40, -40, +120, -120 and 120→40 electrical rad/s
three times each. Net Hall-boundary speeds over the settled windows are
23.84–24.97, -26.37–-25.98, 119.64–120.12 and -120.06–-119.63 rad/s;
the step's low-speed segment averages 37.82–40.71 but still oscillates.
Every ±40 run has 11–13 reverse transitions. Removing dissipative Id in two
matched ±40 checks still gives 11/8 reversals and 4.00/3.85-A dq peaks:
braking Id alone does not explain the remaining low-speed problem.

The matrix received 1,169,105 native samples but lost 39,675 at the producer
(320–7,090 per run), with zero reported host drops and 234/234 commands
acknowledged. Minimal timing reports 41.4–43.0-µs moving means and
53.2–58.4-µs maxima; the 50-µs deadline remains unqualified. The first eight
runs use `motion-off.elf`; the remaining seven add the fault recorder in
`motion-events.elf`. On reverse120 #3 it directly captures speed Derating
at -185.04 rad/s and clearing at -175.55. Forward120 #2 remains the older
unclassified transient, not a clean run. Full Hall traces, exact images,
loss metadata and visually inspected matrix/coast/profile plots are retained
in `motion-suite-results.json`, `motion-suite.png`, `motion-coast-*.png` and
`motion-profile.png` under the same artifact directory. No NVM writes.
