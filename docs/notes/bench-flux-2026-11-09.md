# MK5 flux-linkage bring-up, 2026-11-09

Same board/wiring and calibrated clock as the R/L campaign; PSU 24 V / 5.5 A.
Persisted reference/trip limits remain 3/6 A, input 1 A, regen 0 A.
Motor parameters are not applied or persisted.

The initial 20-kHz controls returned R = 30.860864 mOhm and
Ld/Lq = 18.300083/27.557062 uH (146151/68077 consecutive samples).
The 2-A Hall calibration observed all six physical states, no Hall errors
or timer overcaptures, and retained 294432 consecutive samples. Its result
now reaches the live Hall estimator in RAM without a reboot or flash write.
`bench-hall` emits 50-Hz observations tagged with ADC sequence numbers;
the control/capture interrupts do not log.

Raw captures, exact configuration and analysis are under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-flux-2026-11-09/`.
