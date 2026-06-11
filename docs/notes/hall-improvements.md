# Hall Angle Estimation — Ideas to Borrow from VESC & MESC

> **STATUS: landed 2026-06-12, кроме §HallPll (open).** Подробности
> реализованного — в [decisions.md](../decisions.md) (конвенция
> «таблица = центроиды, якорь = граница») и коммите `9f936bb`; полный
> исходный анализ — в git-истории этого файла.

Working notes from a line-by-line comparison of the Hall pipeline against
**VESC** (`bldc`, `motor/foc_math.c`) and **MESC** (`MESC_Firmware`,
`MESC_Common/Src/MESCfoc.c` / `MESCmeasure.c`). The acquisition layer
(`oxifoc-g431/src/sensors.rs`, TIM4 XOR hardware capture, 1 MHz latched
edge timestamps) was already ahead of both references — everything here
concerned the platform-agnostic estimator/calibrator.

## Landed (детали: decisions.md + 9f936bb)

- **§1 Полусекторный лид интерполяции — подтверждён и исправлен.**
  Регрессионный тест с независимой непрерывной моделью ротора
  (`interpolation_tracks_continuous_rotor`) измерил 0.527 rad ≈ 30.2°
  систематического лида (≈13% потери момента + паразитный d-ток);
  `update()` теперь якорит базу на границу = midpoint соседних
  калиброванных центроидов (VESC-style), центроид остался для low-speed
  snap и фолбэков.
- **§2 Асимметричная установка датчиков** — поглощается midpoint'ом
  измеренных центроидов; скорость использует measured boundary-to-boundary
  ширину сектора.
- **§3 Расширение калибратора (width/boundaries)** — НЕ ПОНАДОБИЛОСЬ:
  midpoint-подход не хранит ширины (как и VESC).
- **§5 Регрессионный тест** — добавлен; урок «спаренные конвенции
  сим/эстиматор прячут смещения» зафиксирован в decisions.md (попутно
  исправлена hall-конвенция `VirtualMotor`).
- Якорь на пропущенном эдже (несмежный переход) — известная остаточная
  неточность ±30°, разбор и эскиз точного фикса в TODO.md; растворяется
  HallPll'ом.

Bench-остаток: подтвердить на железе, что d-ток на постоянной скорости
отцентрован (TODO.md → Стенд).

## [idea, OPEN] PLL-based Hall observer vs open-loop interpolation

Architectural, larger scope. Our Hall path is **open-loop interpolation**:
`base + velocity·dt` with a drift clamp and a rate limiter
(`sample_at_mut`, `hall_sensor.rs`). VESC is the same shape
(`foc_correct_hall`). MESC instead runs a **proper PLL** on the Hall edges
(`angleObserver`, `MESCfoc.c:961`):

```
FOCAngle += angle_step  −  one_on_period · hall_error
            (feed-forward)   (proportional pull toward the known boundary)
```

We already own a high-quality PLL — `BackEmfObserver` (`phase/observer.rs`)
uses exactly this structure. A "HallPll" variant would:

- track angle continuously (no per-cycle clamp/rate-limit hacks — the
  drift corrector, rate limiter and decayed-velocity bound all become
  loop dynamics with one bandwidth knob),
- naturally produce a smooth velocity estimate (no edge-to-edge
  quantization — direct cure for the hall-velocity lag limiting the
  velocity-loop bandwidth, see TODO.md),
- dissolve the skipped-edge anchor issue (innovation is taken against the
  known boundary of whatever sector we're in),
- share gain-tuning intuition with the back-EMF PLL.

Worth prototyping against the `VirtualMotor` — as an *additional*
`AngleSensor` next to the current estimator, judged by the
independent-rotor regression test; not a replacement until it wins on sim
and bench. Caveats: low speed needs gain scheduling or the same low-speed
snap (edges seconds apart); our 1 MHz hardware edge timestamps remove the
velocity-quantization pain MESC fights, so the win is the unified
boundary-anchored dynamics, not raw velocity precision. MESC's own code
carries several `// Does not work... Why??` dead ends here
(`MESCfoc.c:990,1004`); treat its gains as a starting hint, not gospel.

Triggers to actually do it: (а) стенд покажет, что velocity-круиз на
холлах не тюнится мягкими гейнами; (б) старт position control (нужен
чистый непрерывный угол/скорость).

## Reference map

**ours**
- `oxifoc-core/src/foc/hall_sensor.rs` — `update()` (boundary anchor),
  `sample_at_mut()` (drift correction + rate limiter),
  `interpolation_tracks_continuous_rotor` (the referee test)
- `oxifoc-core/src/foc/phase/observer.rs` — `BackEmfObserver` (the PLL
  skeleton to reuse)

**VESC** (`~/motor_control/bldc`)
- `motor/foc_math.c:591` `foc_correct_hall()` — boundary midpoint `:636`,
  low-speed snap `:649`, drift `×0.01` `:658`, rate-limit `×1.5` `:666`

**MESC** (`~/motor_control/MESC_Firmware`)
- `MESC_Common/Inc/MESCfoc.h:295` — `hall_table[6][4]` `{start,end,center,width}`
- `MESC_Common/Src/MESCfoc.c:922` `hallAngleEstimator()` — boundary error `:943`
- `MESC_Common/Src/MESCfoc.c:961` `angleObserver()` — Hall PLL, `angle_step`
