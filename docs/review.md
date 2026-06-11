# Ревью oxifoc — баги, проблемы, заимствования

_Дата: 2026-06-11 (обновление ревью 2026-06-10). 5 агентов + ручная
перепроверка по коду; hot-path (observers, PhaseManager, controller, PI,
SVPWM, transforms, trig, fast_math, foc_driver) прочитан вручную целиком.
Пункты, помеченные «✔ проверено», сверены с исходником в этой сессии;
«[новое]» — не было в ревью 2026-06-10._

Actionable-список ведётся в [TODO.md](TODO.md); safety-дизайн — в
[safety.md](safety.md). Этот файл — архив **анализа** (что не так, почему,
что позаимствовать), TODO — список дел.

## 0. Исправлено с 2026-06-10 (снято из списка)

Каждый снятый пункт с доказательством — чтобы было видно, что выброшено, а
не молча исчезло.

| Пункт 2026-06-10 | Где исправлено |
|---|---|
| **B1** тесты с дефолтными фичами не компилируются | тесты в `foc_driver.rs` под `#[cfg(feature = "runtime")]` |
| **B2** `HallAngleProxy` теряет rate-limit/staleness | `hall_embassy.rs:161-175` переопределяет `sample_mut`→`sample_at_mut`, `is_stale`→`is_stale_at_speed`; manager (`manager.rs:906`) их использует ✔ |
| **B3** нельзя сохранить HallCalibration/DcOffsets | `ConfigWrite` варианты (`types.rs:514`), `config_server` (`servers.rs:270`), storage worker (`storage.rs:382`), boot читает ✔ |
| **B4** `set_link_inactive` мёртв на g431/f405 | вызывается на всех трёх: `g431/protocol.rs:213`, `f405/protocol/servers.rs:183`, `g474/protocol/servers.rs:182` ✔ |
| **B5** `oxifoc-virtual --pole-pairs` мёртвый флаг | `virtual/main.rs:116` строит `MotorParams` из флага, прокидывает в sim+detect ✔ |
| **B6** CLI `--baud` затирает конфиг | теперь `Option<u32>` (`cli/main.rs:55`), применяется только при `Some` ✔ |
| **B7** framed-транспорты: handshake игнор + нет reconnect | `run_framed_with_reconnect` проверяет handshake (`host-lib/lib.rs:410`) + reconnect ✔ |
| **B8** GUI `parse().unwrap_or(0.0)` пишет 0 во flash | `parse_field` (`host-slint/lib.rs:63`) валидирует, write-пути отказывают ✔ |
| Нет watchdog / panic-хука | `safety.rs` на g431/f405: panic+HardFault чистят `BDTR.MOE` до репорта; IWDG кормится из FOC ISR (100 мс g431 / 1 с f405) ✔ |
| `CurrentLimits::default() = {0,0}` = защита выкл | теперь `from_max_current(5.0)` (`foc_driver.rs:57`); DirectVoltage/SixStep тоже ловят overcurrent ✔ |
| Блокирующая flash-запись: TOCTOU Busy-gate | `FlashPendingGuard` (RAII) + `FLASH_OP_PENDING`: сервер взводит флаг до пере-проверки, ISR отказывает в старте мотора, Stop всегда разрешён ✔ |
| Нет dq-развязки и bEMF feedforward | `controller.rs:443`: `vd_ff=−ω·Lq·iq`, `vq_ff=ω·(Ld·id+λ)` до circular-clamp; armed из stored params + `SetDecoupling` ✔ |
| Таймбаза `tick-hz-32_768` (квантование hall) | устарело: hall перешёл на аппаратный capture-таймер 1 МГц (TI1S XOR + IC), не `embassy_time` ✔ |
| HFI 150% бюджета ISR (libm sinf/cosf) | SinCos-backend (CORDIC G4 / FastSinCos F405) + `fast_math` → HFI 13.9%, см. [perf-bench](perf-bench-2026-06-11.md) ✔ |

Исправлено 2026-06-11 (вторая сессия, после этого ревью):

| Пункт | Где исправлено |
|---|---|
| **HIGH** F405 SPI в critical section | `Drv8301Spi` (шина+CS) владеется `nfault_monitor_task`; в CS-мьютексе только EN_GATE GPIO; мёртвые хелперы удалены |
| **HIGH** F405 FOC ISR без приоритета | `control/foc.rs`: NVIC priority 0 для ADC, как на G431 |
| F405 OverTemp не critical + температура мотора | `OverTemp` в critical; `BoardConfig.max_motor_temp_c` + `check_temperature_threshold` (NTC мотора PC4, 120 °C) |
| Detection: spin-down мёртв на железе | `DetectionHardware::supports_coast_telemetry` (default false) — честный пропуск с логом вместо spin-up→нули→«fallback»; реализация по фазным делителям — bench |
| Detection: `Ld ≤ Lq` зашито | знаковый Re(bin2) вместо магнитуды; quadrature-компонент копится, доминирующий Im → warn «lock far off d» (`axes_aligned`) |
| Detection: эскалация тока без power-гейта при `None` | проекция мощности по последнему известному R перед эскалацией |
| Detection: `measure_resistance` без проверки сходимости | id вне ±30% от setpoint → `UnexpectedMotion` (общий таймаут `wait_telemetry` — ещё открыт) |
| g431: нет const_assert на storage-перекрытие | портирован из f405/g474 (`FIRMWARE_END_OFFSET` уже был в build.rs) |
| Host: `HostRuntime` без `Drop` | `impl Drop { cancel() }` + GUI connect делает take()+shutdown+drop старого слота |
| Host: Stop ждёт за Detect в очереди | Detect спавнится в side-task (device всё равно отказывает детекции при работающем моторе) |
| RTT: expect() в потоке + скан 32 КБ | `ScanRegion::Ram` (по описанию чипа); все ошибки пробы → io::Error в reader — линк ломается видимо |
| CLI fire-and-forget + `--duty`=ток | `HostCommand::MotorAck` (oneshot-ответ), exit≠0 без подтверждения; `start --iq <A>` |
| GUI: молчаливый дроп телеметрии / RPM из пресета / motor_running | предупреждение по измеренному rate vs acked; pole pairs из stored MotorParams устройства; сброс на disconnect |
| Layer-2 deadman (крупнейший safety-разрыв §3) | реализован и расширен: ISR-deadman 150 мс, failsafe v2 (decel-рампа, no-progress watchdog, терминал ParkBrake), re-arm latch, ramp-into-brake; см. safety.md |
| Velocity loop не реализован (§3 Алгоритмы) | `foc/velocity.rs` + `VelocityControl` в драйвере; sensorless-деградация в coast; персист + GUI |
| Stale «Temporarily disabled» OCP-комментарий | удалён (функция живая, зовётся из main) |

---

## 1. Открытые баги и проблемы корректности

### HIGH

_Пусто — оба пункта (F405 SPI-in-CS, приоритет FOC ISR) исправлены
2026-06-11, см. §0._

### MEDIUM

- **Detection: `measure_resistance` — общий таймаут `wait_telemetry`.**
  Остаток пункта (проверка сходимости id сделана): если FOC ISR молчит,
  sample-loop ждёт вечно. Низкий риск (ISR прикрыт IWDG), но select с
  таймаутом был бы чище.

### LOW

- **`fast_math::atan2f(0,0)` возвращает π/2, а не 0.** ✔ [новое]
  `fast_math.rs:57-67`: при `x=0` всегда `r=−1` ⇒ `π/2`. Док-комментарий
  «Returns 0.0 for (0, 0)» неверен. Безвредно (confidence гейтит вырожденный
  случай в observer), но коммент врёт — поправить.
- **Anti-windup относит насыщение от feedforward+injection целиком на
  PI-интеграторы.** ✔ [новое] `controller.rs:466`: `apply_anti_windup(vd−vd_raw)`,
  где `vd_raw` включает `vd_ff`+`vd_inject`; при насыщении в основном из-за ff
  PI разматывается зря. Мелкая tuning-неоптимальность, самокорректируется.
- **Sensors: `LAST_STATE = 0` как «edge'а ещё не было».** ✔ [новое]
  `hall_embassy.rs:78`: 0 — это и реальное all-low-чтение, поэтому *первый*
  edge в состояние 0 проглатывается (estimator не узнаёт, error_count не
  растёт). Узкий случай (статический обрыв edge'ей не даёт; pull-up-обрыв
  читается как 7, не 0). Фикс: сентинел `0xFF`.
- **Sensors: незащищённый `wrapping_sub as f32` для `dt`.** ✔ [новое]
  `hall_sensor.rs:542,612`: при `now<t0` (гонка домена тиков) угол замерзает
  на цикл; рядом `dt_sample` делает `.max(1)`, `dt_from_edge` — нет.
- **Архитектура doc-rot в `icd.rs` (B9).** [агент] `icd.rs:13-16` зовёт
  `SlowTelemetry` push-топиком 10 Hz — это poll endpoint; `:135-137`
  противоречит `:147-149` про классификацию `DetectEndpoint`. Косметика.

---

## 2. Подозрение (нужен стенд)

- **F405: ADC injected-триггер срабатывает дважды за период PWM.** [агент,
  уточнение] `motor.rs:61` — `CenterAlignedBothInterrupts`;
  `hardware/peripherals.rs` триггерит injected от `TIM1_CH4` RISING_EDGE;
  CCR4 у пика. В center-aligned CNT пересекает CCR4 **дважды** за период ⇒
  два rising-edge. Сейчас, вероятно, работает: второй триггер (~0.4 мкс)
  попадает внутрь идущей injected-последовательности (~2.6 мкс) и
  игнорируется. **Уточнение к ревью 2026-06-10:** G431 на `COMPARE_OC4`→TRGO2
  **не иммунен принципиально** — OC4REF тоже взводится на обоих проходах;
  обе платы держатся на «второй триггер внутри идущей последовательности».
  Робастный фикс (обе платы, F405 первой как живая) — один детерминированный
  триггер на период (по update-событию, либо TIM→DMA→ADC). Проверить
  JEOC-rate под нагрузкой на F405.
- **Detection: pipeline-skew в `measure_inductance`.** [агент] [новое]
  `sweep.rs:336-369` пары́т ток с инжекцией *предыдущей итерации*, но команда
  идёт через `CMD_CHANNEL` (drain в ISR + ZOH PWM), и реальная латентность
  command→apply→measure на железе может быть >1 итерации; все тесты применяют
  напряжение синхронно, так что скос не ловится. Может систематически смещать
  Ld/Lq. Проверить эталонной индуктивностью + добавить тест с задержкой
  инжекции.

---

## 3. Слабые места / пробелы

### Безопасность

- ~~Link-loss failsafe / Layer-2 deadman~~ — **сделано 2026-06-11** (см. §0
  и safety.md: deadman 150 мс + failsafe v2 + защёлка + parking brake).
- **Single-sample overcurrent/voltage без persistence-фильтра.**
  `fault.rs:422-436` и `foc_driver.is_overcurrent` латчат по одному ADC-сэмплу;
  nuisance-trip на regen/EMI = опасный обрыв момента на ходу. VESC/MESC —
  интегрирующий детектор (см. §4 VESC-3, MESC-5).
- **Open-loop override при потере сенсора всегда `+52 rad/s` без знака.** ✔
  `manager.rs:559,73`: на реверсе крутит вперёд. VESC подписывает направлением
  (см. §4 VESC-8).
- **Нет graduated derating** — только пороговые fault'ы (см. §4 VESC-4).
- **Pure `Hfi` source коммутирует без гейта resolved-polarity.** ✔ [новое]
  `manager.rs:630-636` отдаёт `hfi.phase()` без `is_ready`-проверки; до probe
  угол может быть π-flipped. Дизайн-интенция (HFI с нуля валиден), но при
  старте в pure-Hfi возможен момент в неверную сторону. HfiToX и crossover'ы
  гейтят корректно — касается только прямого `Hfi`.

### Алгоритмы (разрыв с VESC/MESC)

- **Position-контур не реализован** (velocity сделан 2026-06-11 —
  `foc/velocity.rs`, каскад для position готов; нужен unwrapped-источник
  позиции).
- **Нет field weakening и MTPA.**
- **Нет автоопределения pole pairs и offset-калибровки энкодера.**
- **Нет настоящей overmodulation-стратегии:** `modulation_limit` принимает до
  1.2, но выше линейной зоны SVPWM просто клампит duty (`svpwm.rs:108-113`).
- **`apply_dq` (DirectVoltage) пропускает dead-time compensation**
  (`controller.rs:339-367`) — а это режим HFI-детекции индуктивности; искажение
  dead-time смещает измерение L. ✔
- **`has_hall()` — эвристика, ложно отказывает до первого edge**
  (`manager.rs:371`) → `set_source(Hall…)` из stored config может спорадически
  падать на холодном старте. ✔
- **Sensors: дыра staleness на низкой скорости.** ✔ [новое]
  `is_stale_at_speed` выходит при `vel<1.0` (`hall_sensor.rs:311`), а manager
  консультирует только её — fixed-timeout `is_stale()` (100 мс) без вызовов в
  control-path. Мёртвый сенсор на ~стоячем роторе не ловится staleness-путём
  (последствие на <1 rad/s минимально; at-speed покрыт тестом
  `closed_loop_hall_dropout_at_speed`). Фикс: OR'ить fixed-timeout backstop'ом.

### Архитектура / прошивки

- **Триплицированный glue g431/g474/f405 расходится:** g474 `control/foc.rs`
  закомментирован и при оживлении воспроизведёт F405-баги (нет приоритета 0,
  нет BKIN-детекта, нет IWDG-feed). Вынести ISR-скелет / state-monitor /
  ADC-IRQ-enable в core до оживления g474. [агент]
- **F405: блокирующий erase 128 КБ-сектора (~0.5 с) морозит executor.**
  TOCTOU закрыт (см. §0), мотор гарантированно остановлен Busy-гейтом во время
  записи, IWDG-маржин — бэкстоп; остаётся как известный residual.
- **Все `CMD_CHANNEL.try_send` игнорируют результат** — host не отличит
  «принято» от «дропнуто» (канал ёмкостью 8). `motor_command_server`
  (`servers.rs:103`) возвращает OK и pre-command статус даже при дропе; конфиг
  `MotorParams` шлёт 2 команды (`SetPiGains`+`SetDecoupling`) — на полном
  канале вторая молча теряется ⇒ полу-применённая конфигурация. [агент]
- **RMW статус-регистра TIM1** (`sr().modify` для bif, `g431/motor.rs:179`) —
  rc_w0, RMW может сбросить чужие pending-флаги; писать complement-mask. ✔
- **`FaultResponse` молча обрезает до 8 из 16 фолтов** (`servers.rs:154`,
  `MAX_FAULT_RESPONSE=8` vs `MAX_FAULTS=16`) без индикатора усечения. [агент]

### Host / virtual / протокол

- Виртуальное устройство тоньше модели: только CurrentControl/Stopped, нет
  fault-injection (fault-путь хоста не покрыт e2e), конфиг round-trip'ится, но
  физику не меняет (`RUNTIME_CONFIG` не доходит до `VirtualMotor`), AdcSnapshot
  нулевой. `FaultEndpoint` не использует ни один host-инструмент. [агент]
- Нет `protocol_version` в `HardwareInfo` (схема postcard без self-description;
  меняется без ошибки → молчаливый мусор при рассинхроне). [TODO]
- ~~RTT expect()/32K-скан, CLI fire-and-forget/`--duty`, GUI дроп/RPM/
  motor_running~~ — **сделано 2026-06-11**, см. §0.
- bridge/remote: пейринг по hardcoded MAC; тесты-заглушки (`assert_eq!(1+1,2)`).
- Reconnect state machine хоста не покрыт тестами; slint-wgpu-plot без тестов —
  индексная арифметика кольца (`renderer.rs:262-274`) при большом zoom-out +
  scroll-back может считать Y-auto-range по другому окну, чем рисует шейдер.
  [агент]
- Один последовательный command queue: `wait_for_active` на connect-пути без
  таймаута (`host-lib/lib.rs:404`) — полу-открытый framed-линк висит до
  `cancel` (disconnect-recovery-путь обёрнут в `RECOVERY_TIMEOUT`, connect — нет).
  [агент]

---

## 4. Что позаимствовать из каждого проекта

### VESC (`bldc`)
1. **Силовое заполнение observer'а при handover:** при openloop→sensorless и HFI-start форсировать `x1/x2 = λ·cos/sin(θ)` (`mcpwm_foc.c:4014-4108`) — убирает токовые выбросы при захвате. Прямо ложится на твой dual-slot/blend.
2. **Бесшумный HFI (v4/v5):** инжекция синхронно с PWM-последовательностью, угол в прерывании через PI + двойной интегратор на di/dt (`foc_math.c:744`, `mcpwm_foc.c:4801-4853`).
3. **Интегрирующий OV/UV-детектор** (терпит выбросы, срабатывает на устойчивое отклонение, `mc_interface.c:1881-1907`) — ответ на твой односэмпловый current fault.
4. **Graduated derating matrix:** линейный спад лимитов по T_fet/T_motor/V_bat/ERPM/duty (`update_override_limits`, `mc_interface.c:2225+`).
5. Адаптивный R-observer в фоне (температурная компенсация сопротивления, `mcpwm_foc.c:4133-4146`).
6. V0_V7_INTERPOL: повторный inverse Park + SVM в середине периода с экстраполированным углом — гладкость на высоких eRPM.
7. Реконструкция насыщенного шунта из двух других для расширения диапазона тока (`mcpwm_foc.c:3068-3083`).
8. Знаковый open-loop при потере сенсора (направление = знак последней скорости).

### MESC
1. **FW V2 — field weakening на чистой обратной связи насыщения:** экспоненциальный ramp d-тока от факта упирания вектора напряжения в круг (`MESCfoc.c:1107-1127`) — без зависимости от параметров мотора, самоограничивающийся. Лучший кандидат на твой первый FW.
2. **Deadshort-перезахват:** кратко закоротить фазы, вычислить угол из V=L·di/dt нарастающего тока, предзагрузить observer и интеграторы PI, сразу в RUN (`MESCfoc.c:1616-1698`) — перезахват крутящегося мотора без датчиков напряжения фаз.
3. **Hall-flux startup:** онлайн-обучение flux-векторов на hall-сегмент, IIR-вливание в аккумуляторы observer'а на низкой скорости (`MESCfoc.c:450-459`) — плавный zero-speed момент на дешёвых холлах.
4. **Always-on blackbox:** кольцевой лог Vbus/Iuvw/Vdq/angle в fastLoop, замораживается вокруг ошибки, стримится по CAN (`MESCfoc.c:713-732`) — бесценно для отладки fault'ов на стенде.
5. **Динамические пороги fault'ов:** Imax = 1.5× запрошенного тока, Vmax трекает Vbus + 15% (`MESCfoc.c:1295-1312`) — ловит regen-выбросы с БП.
6. Dead-time компенсация по знаку тока (`MESCpwm.c:157-179`) + режим автоизмерения dead-time и встроенный double-pulse test для bring-up железа.
7. SVM-aware выбор пары шунтов: Clarke из двух фаз с наименьшим duty (`MESCfoc.c:823-870`).

### moteus
1. **PendSV-split ISR:** prio-0 делает только критичное окно сэмплирования, остальной контур — в PendSV (prio 6); энкодерные прерывания могут вклиниваться (`bldc_servo.cc:446-469`). Станет актуально с позиционным контуром.
2. **Limit-as-fault-code телеметрия:** каждое ограничение, которое клипнуло команду, репортится кодом без останова — всегда видно, ЧТО тебя резало.
3. **Back-EMF + R·i feedforward в токовом контуре** (`bldc_servo_control.h:809-962`) — то самое недостающее W1.
4. **PLL-гейны из одного интуитивного параметра «Hz»**, автоограничение ¼ фактической частоты опроса источника (`motor_position.h:504-516`).
5. 32.32 fixed-point позиция (когда появится позиционный контур) — разрешение не деградирует с пробегом.
6. Аппаратная цепочка триггеров TIM→DMA→LPTIM→ADC (обход эрраты G4 ES0430 §2.7.11) — ноль программного джиттера.
7. GPIO-чтение состояния фазы в окне сэмплирования как аппаратное доказательство, что duty не нарушил окно (`bldc_servo.cc:891-897`).
8. Flux braking: поглощение regen в обмотки d-током по порогу Vbus — без тормозного резистора.

### ODrive
1. **Spinout-детект:** сверка электрической и механической мощности (`controller.cpp:434-441`) — дешёвая защита от срыва коммутации/слетевшей калибровки.
2. **`std::optional`-порты данных со сбросом каждый цикл** — структурная защита от stale-данных между компонентами контура (`main.cpp:371-398`).
3. **Сменный `PhaseControlLaw`:** измерение R/L — это просто другой control law с тем же arm/disarm/safety-механизмом (`motor.cpp:18-148`).
4. Компенсация фазовой задержки: Park на `phase + vel·(t_meas−t_ctrl)`, inverse Park на `phase + vel·(t_pwm−t_ctrl)` (`foc.cpp:100,163`).
5. Anticogging-калибровка с картой 3600 бинов (`controller.cpp:79-103`).
6. Документированный тайминг-бюджет на задачу (`task_times_`, MEASURE_TIME).

### ST MCSDK (B-G431B-ESC1)
1. **Staged rev-up таблица + SWITCH_OVER crossfade:** параметризованные фазы (длительность/скорость/ток) + virtual speed sensor с ramp-передачей authority observer'у (`revup_ctrl.c`, `mc_tasks_foc.c:633-686`) — прямо ложится на твой blend-crossover.
2. **Variance-gating надёжности observer'а** (`STO_PLL_IsVarianceTight`) перед доверием sensorless-скорости — дополнение к твоему readiness (flux+PLL+min speed).
3. **MC_DURATION fault:** дешёвый прямой детект перерасхода бюджета FOC-цикла (контур не уложился в период PWM → fault) (`mc_tasks_foc.c:655-658`).
4. Компенсация задержки Park/inverse-Park скоростью в dpp × фактор (фиксированная точка, `mc_tasks_foc.c:719,740`).

## 5. Приоритеты (сводно, обновлено 2026-06-11 после второй сессии)

Пункты 1–4 старого списка закрыты (см. §0). Остаток:

1. **Дешёвые code-only мелочи (§1 LOW + §3):** TIM1 SR RMW
   (complement-mask); знаковый open-loop override; pure-Hfi гейт
   resolved-polarity; hall staleness fixed-timeout backstop; `has_hall()`
   на холодном старте; `LAST_STATE` сентинел 0xFF; dt `wrapping_sub` гард;
   `FaultResponse` индикатор усечения; `CMD_CHANNEL.try_send` ack;
   `wait_for_active` таймаут на connect; anti-windup ff-вклад;
   atan2f(0,0)-коммент; icd doc-rot; `wait_telemetry` таймаут.
2. **Стенд (§2 + хвосты):** ADC double-trigger (обе платы); pipeline-skew
   индуктивности; spin-down по фазным делителям; интегрирующий
   current/voltage детектор; bench-тюнинг failsafe/velocity/brake.
3. **Алгоритмы (дёшево→дорого):** position loop → graduated derating →
   FW V2 (MESC) → MTPA.
4. **Архитектура:** вынос ISR-glue в core до оживления g474; fault-injection
   в oxifoc-virtual; protocol_version в HardwareInfo; bridge/remote.
