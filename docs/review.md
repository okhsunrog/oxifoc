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

---

## 1. Открытые баги и проблемы корректности

### HIGH

- **F405: SPI к DRV8301 внутри critical section маскирует FOC ISR.** ✔
  `drv8301.rs:35` — `DRV_CONFIG: CriticalSectionMutex`; `nfault_monitor_task`
  (`:244-249`) делает блокирующий `get_fault_status()` по SPI внутри `.lock()`.
  `critical-section-single-core` ⇒ PRIMASK гасит все прерывания, включая
  контурный ADC ISR, на десятки мкс — ровно в момент gate-fault'а. Подъём
  приоритета ISR не спасает (PRIMASK маскирует независимо от приоритета).
  Фикс: вынести SPI-устройство из CS-мьютекса (владеть им в задаче-мониторе
  или async SPI + non-CS mutex). _Также `get_fault_status` (`:281`),
  `reset_faults` (`:308`)._
- **F405: FOC ISR на дефолтном приоритете.** ✔ `control/foc.rs:91-95` зовёт
  `ADC::enable()` без `set_priority`. G431 (`foc.rs:109`) ставит `0`.
  Comms-ISR (USB/UART) джиттерят/вытесняют контур — а это актуатор. Фикс:
  `NVIC::set_priority(.., ADC, 0)` как на G431.

### MEDIUM

- **F405: `OverTemp` не critical + температура мотора не фолтится.** ✔
  `f405/fault.rs:94` — critical только `OverCurrent|OverVoltage|DrvFault`
  (G431/G474 включают `OverTemp`), так что перегретую плату можно перезапустить
  командой (`process_commands` блокирует старт только по `any_critical()`).
  `control/foc.rs:211` зовёт `check_temperature_fault` только для `board_temp`;
  `motor_temp_c_x10` считается (`:185`) и телеметрируется (`:227`), но не
  проверяется — перегрев мотора молча игнорируется. `BoardConfig` не имеет
  поля порога температуры мотора. Фикс: `OverTemp` в critical + порог/проверка
  для мотора.
- **Detection: spin-down-flux — мёртвый путь на железе.** ✔ [новое]
  `EmbassyDetectionHardware` (`embassy_hw.rs:64-94`) не переопределяет
  `read_coast_telemetry()`; дефолт трейта (`sweep.rs:90`) возвращает `(0,0,0)`
  ⇒ `omega_e=0 < min_omega` ⇒ `InsufficientSamples` ⇒ всегда fallback на
  driven-метод. «R-независимое» измерение λ, заявленное в коде как основное
  (`sweep.rs:895`), на плате не выполняется **никогда** (работает только в
  virtual-harness, который его переопределяет). Фикс: реализовать
  `read_coast_telemetry` (ADC делителей фазного напряжения + observer ωe) или
  перестать его рекламировать; логировать, что взят fallback.
- **Detection: `Ld ≤ Lq` зашито конструктивно.** ✔ `inductance.rs:385` —
  `amplitude = |bin2|·2/N ≥ 0`, `inv_ld = offset+amplitude ≥ inv_lq`. Использована
  только **магнитуда** бина-2 (фаза выброшена) ⇒ inverse saliency и
  перепутанные d/q (lock-angle на 90° мимо) не детектируются; downstream
  `kp_q ≥ kp_d` всегда. Фикс: комплексный бин-2 относительно фазы инжекции.
- **Detection: `find_safe_test_current` эскалирует ток без проверки мощности
  при неудачном замере.** [агент] `resistance.rs:180-193` — `test_current *= 1.5`
  выполняется и когда `quick_measure` вернул `None`, а power-guard — только в
  ветке `Some`. Флакающий замер на низком токе ⇒ безудержная эскалация к
  `current_max` без теплового гейта. (Публичный API; `run_full_detection` его
  не зовёт, но любой другой вызыватель — тепловой риск.)
- **Detection: `measure_resistance` без проверки сходимости/таймаута/движения.**
  [агент] `sweep.rs:177-251` слепо усредняет `vd`/`id` и делит `R=ΔV/ΔI`; если
  ротор не залочен (cogging, осциллирующий PI) — правдоподобно-неверный R,
  отравляющий весь downstream (inductance comp, PI tuning). Бэкстоп —
  только overcurrent-trip. Есть неиспользуемый `UnexpectedMotion`
  (`types.rs:173`). Фикс: проверять, что `id` достиг setpoint, + общий таймаут.
- **g431: нет const_assert на перекрытие storage-региона с прошивкой.** ✔
  `g431/storage.rs:22` хардкодит `0x1F000` без проверки; f405/g474 имеют
  `const _: () = assert!(STORAGE_START >= FIRMWARE_END_OFFSET)`. На 128 КБ
  single-bank рост прошивки за 124 КБ молча сотрёт свой код при первой записи
  конфига — кирпич. Фикс: портировать assert + `FIRMWARE_END_OFFSET` build.rs.
- **Host: `HostRuntime` без `Drop` → утечка tokio-рантайма+потока на каждый
  reconnect.** [агент] `host-slint/lib.rs:745` перезаписывает `Some(HostRuntime)`
  без `shutdown()`; `CancellationToken` не отменяется, старый транспорт держит
  порт. Фикс: `impl Drop { cancel_token.cancel() }` + `take()`+shutdown старого
  слота.
- **Host: `Motor(Stopped)` ждёт до ~70 с за `Detect` в одной command-queue.**
  [агент] `host-lib/lib.rs:787` — single drain, `Detect` с `deadline_ms:70_000`;
  GUI цепляет 4 detect-шага. Аварийный Stop блокируется (safety закрыта
  device-side deadman'ом, но UX неверный). Фикс: внеочередной канал для
  Motor/Stop или отмена in-flight detect.

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

- **Link-loss failsafe = 3–5 с; быстрый ISR-deadman (Layer 2) не реализован.**
  [safety.md] `icd.rs:42` `LIVENESS_TIMEOUT_MS=5000`; единственный гейт —
  async `state_monitor` по liveness, зависит от живого executor'а. Это
  задокументированный **planned** пункт (safety.md, Layer 2), не регресс, но
  крупнейший safety-разрыв слоя. Промежуточно — укоротить liveness; целево —
  `last_cmd_tick` в ISR-дрейне CMD_CHANNEL + self-contained failsafe-режим.
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

- **Velocity/Position контуры не реализованы** (`foc_driver.rs:401-408` → `Err`),
  хотя wire-типы есть и host может прислать. `ClampedPI` лежит готовый.
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
- **Stale-комментарий:** g431 `init_overcurrent_protection` помечен
  «Temporarily disabled» (`hardware.rs:234`, `config.rs:60`), но OCP реально
  **включён** (BKIN от компараторов + BKF-фильтр в `motor.rs:88-104`, зовётся
  из `main.rs:84`). Для safety-фичи протухший «disabled» — сам по себе риск. ✔
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
- RTT: `expect()` в detached-потоке (`rtt.rs:159`) = молча мёртвый транспорт
  без reconnect; скан control-block hardcoded `0x20000000..0x20008000` —
  сломается на F405/G474 (RAM > 32 КБ). [агент]
- CLI: `start`/`stop`/`source` — fire-and-forget со sleep'ами, exit code 0
  всегда; `--duty` на самом деле ток (`duty × 0.1 A`) при вводящем в
  заблуждение help'е. [агент]
- GUI: combo до 20 кГц телеметрии не влезает в 921600 baud — молчаливый дроп
  без индикации (`actual_fast_hz` есть в ack, но не сверяется); RPM делится на
  pole pairs из UI-пресета, а не из устройства (`HardwareInfo` без pole_pairs).
  `motor_running` не сбрасывается на disconnect. [агент]
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

## 5. Приоритеты (сводно)

1. **F405-glue (live-плата):** SPI вне critical section → приоритет FOC ISR 0 →
   `OverTemp` critical + проверка температуры мотора.
2. **Detection:** реализовать/убрать spin-down (`read_coast_telemetry`);
   комплексный бин-2 (snять `Ld≤Lq`); проверить pipeline-skew на стенде.
3. **Safety:** Layer-2 ISR command-staleness deadman (укоротить liveness как
   промежуточный шаг); интегрирующий current/voltage детектор вместо
   односэмплового; знаковый open-loop override.
4. **Корректность по мелочи:** g431 storage const_assert; host `HostRuntime::Drop`;
   Stop вне command-queue; снять stale «disabled» комментарий OCP.
5. **Алгоритмы (дёшево→дорого):** velocity loop → graduated derating →
   FW V2 (MESC) → MTPA.
6. **Архитектура:** вынос glue в core до оживления g474; fault-injection в
   oxifoc-virtual; protocol_version в HardwareInfo.
