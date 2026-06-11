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

Исправлено 2026-06-11 (третья сессия — пакет LOW-мелочей из §5.1):

| Пункт | Где исправлено |
|---|---|
| TIM1 SR RMW (rc_w0) | `g431/motor.rs::enable` + ISR в `foc.rs` (второе место нашлось при пере-ревью — вызов был разбит переносом строки и не попал под grep); теперь все 5 мест через `oxifoc_core::clear_rc_w0!`; гонка `modify` верифицирована по asm — см. [register-access.md](register-access.md) |
| Open-loop override всегда `+52 rad/s` | `manager.rs::try_observer_fallback`: знак от последней известной скорости (`output.velocity`) |
| Pure-Hfi без гейта resolved-polarity | iq-гейт в `foc_driver::step_current_control`: при `!angle_trustworthy()` iq=0, id и коммутация остаются (инжекция/probe держат frame, гейт самоснимается на lock). Гейтить угол в менеджере нельзя — сломал бы frame-alignment probe |
| `has_hall()` ложно-отрицателен до первого edge | `AngleSensor::is_present()` (default true, `NoSensor`→false, `HallAngleProxy`→estimator создан); `has_hall`/`has_encoder` структурные, не «есть данные» |
| `LAST_STATE = 0` сентинел | `hall_embassy.rs`: сентинел `0xFF` (`NO_EDGE_YET`) — первый edge в состояние 0 больше не глотается |
| `wrapping_sub as f32` для dt | `hall_sensor.rs`: `saturating_sub` в `is_stale`/`time_since_edge`/`sample_at`/`sample_at_mut` — гонка тиковых доменов даёт dt=0, а не ~2⁶⁴ |
| `FaultResponse` молча обрезает 8 из 16 | поле `total: u8`; `total > faults.len()` ⇒ усечение (host-потребителей у FaultEndpoint пока нет) |
| `CMD_CHANNEL.try_send` игнорирует результат | `motor_command_server`/`config_server` live-apply/`phase_source_server`: `send().await` — ISR дренирует канал каждый цикл, дроп (и полу-применённый MotorParams) невозможен |
| `wait_for_active` без таймаута на connect | оба пути (framed + COBS) обёрнуты `RECOVERY_TIMEOUT` (10 с) → teardown + reconnect, как recovery-путь |
| Anti-windup относит ff+inject на PI | `controller.rs`: back-calculation `pi·(scale−1)` — круговой кламп масштабирует вектор равномерно, PI получает только свою долю; при ff=inject=0 эквивалентно прежнему `v−v_raw` |
| `atan2f(0,0)` док-коммент врёт | коммент исправлен: возвращает π/2 (1e-20 bias на \|y\|), не 0; вырожденный случай гейтится confidence |
| icd.rs doc-rot | header-таблица: SlowTelemetry — poll-endpoint (не push-топик), добавлены PhaseSource/Config/Detect(Keyed); стейл-абзац про «DetectEndpoint not classified yet» переписан (он classified `Deduplicated`) |
| `wait_telemetry` без общего таймаута | `sweep.rs::sample_vd_id`: sample-циклы `measure_resistance` под `embassy_futures::select` c дедлайном 2 с (номинал ~100 мс); на таймауте `Stopped` перед `MotorNotResponding` (ramp-down на этом пути не выполняется) |

**Решено НЕ чинить** — hall staleness fixed-timeout backstop (`is_stale_at_speed`
выходит при vel<1.0): OR с фиксированным 100 мс таймаутом **опасен** — стоячий
ротор легитимно не даёт edge'ей, backstop объявил бы холлы stale на каждой
остановке ⇒ `hall_sample=None` ⇒ fallback ⇒ open-loop override 52 rad/s у
светофора. Мёртвый-на-стоянке сенсор по edge'ам принципиально неотличим от
стоящего ротора; обрыв кабеля ловится invalid-state путём (pull-up ⇒ 0b111),
скоростно-адаптивный путь покрывает движение. Текущая реализация корректна by design.

---

## 1. Открытые баги и проблемы корректности

### HIGH

_Пусто — оба пункта (F405 SPI-in-CS, приоритет FOC ISR) исправлены
2026-06-11, см. §0._

### MEDIUM

_Пусто — `wait_telemetry`-таймаут сделан в третьей сессии, см. §0._

### LOW

_Пусто — все пункты (atan2f-коммент, anti-windup ff, LAST_STATE сентинел,
wrapping_sub dt, icd doc-rot) исправлены 2026-06-11 (третья сессия), см. §0._

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
- ~~Open-loop override всегда `+52 rad/s`~~ — **сделано 2026-06-11 (3-я
  сессия)**, знак от последней скорости, см. §0.
- **Нет graduated derating** — только пороговые fault'ы (см. §4 VESC-4).
- ~~Pure `Hfi` без гейта resolved-polarity~~ — **сделано 2026-06-11 (3-я
  сессия)**, iq-гейт по `angle_trustworthy` в `step_current_control`, см. §0.

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
- ~~`has_hall()` ложно отказывает до первого edge~~ — **сделано 2026-06-11
  (3-я сессия)**, структурный `is_present()`, см. §0.
- ~~Дыра staleness на низкой скорости~~ — **решено НЕ чинить** (3-я сессия,
  см. разбор в §0): fixed-timeout backstop ложно срабатывает на легитимной
  стоянке и ведёт к open-loop override; мёртвый сенсор на стоянке по edge'ам
  неотличим от стоящего ротора, обрыв ловится invalid-state путём.

### Архитектура / прошивки

- **Триплицированный glue g431/g474/f405 расходится:** g474 `control/foc.rs`
  закомментирован и при оживлении воспроизведёт F405-баги (нет приоритета 0,
  нет BKIN-детекта, нет IWDG-feed). Вынести ISR-скелет / state-monitor /
  ADC-IRQ-enable в core до оживления g474. [агент]
- **F405: блокирующий erase 128 КБ-сектора (~0.5 с) морозит executor.**
  TOCTOU закрыт (см. §0), мотор гарантированно остановлен Busy-гейтом во время
  записи, IWDG-маржин — бэкстоп; остаётся как известный residual.
- ~~`CMD_CHANNEL.try_send` игнорируют результат~~ — **сделано 2026-06-11
  (3-я сессия)**, `send().await` во всех серверах, см. §0.
- ~~RMW статус-регистра TIM1~~ — **сделано 2026-06-11 (3-я сессия)**,
  complement-mask, см. §0.
- ~~`FaultResponse` молча обрезает 8 из 16~~ — **сделано 2026-06-11 (3-я
  сессия)**, поле `total`, см. §0.

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
- ~~`wait_for_active` на connect-пути без таймаута~~ — **сделано 2026-06-11
  (3-я сессия)**, `RECOVERY_TIMEOUT` на обоих путях, см. §0.

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

## 5. Приоритеты (сводно, обновлено 2026-06-11 после третьей сессии)

Code-only мелочи (бывший пункт 1) закрыты в третьей сессии (см. §0; один
пункт — hall staleness backstop — аргументированный won't-fix). Остаток:

1. **Стенд (§2 + хвосты):** ADC double-trigger (обе платы); pipeline-skew
   индуктивности; spin-down по фазным делителям; интегрирующий
   current/voltage детектор; bench-тюнинг failsafe/velocity/brake.
2. **Алгоритмы (дёшево→дорого):** position loop → graduated derating →
   FW V2 (MESC) → MTPA.
3. **Архитектура:** вынос ISR-glue в core до оживления g474; fault-injection
   в oxifoc-virtual; protocol_version в HardwareInfo; bridge/remote.
