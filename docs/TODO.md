# Project TODO / backlog

Единый бэклог открытой работы. Правило: **сделанное удаляется** (история —
git log и [archive/](archive/)), решения с обоснованием — в
[decisions.md](decisions.md), анализ/идеи — в [notes/](notes/), дизайн
failsafe — в [safety.md](safety.md). Карта документации — [README.md](README.md).

## Safety

- [ ] **Bench-tune regen-brake**: `brake_current_a`, `standstill_rad_s`,
  low-speed coast floor; подтвердить отсутствие OV-трипа на шине при regen;
  `BRAKE_ENTRY_MAX_E_RAD_S` (гейт входа в parking brake) + ток короткого
  замыкания обмоток на этой скорости внутри рейтингов FET.
- [ ] **Parking brake follow-ups**: кнопка в GUI + маппинг на пульт.
- [ ] **Dissipative braking near OV** (спуск с полной батареей, см.
  safety.md): когда OV-derate срезает regen-тормоз — рассеивать энергию в
  обмотках (active short / d-ток) с термоконтролем. Нужен стенд.
- [ ] **Position hold** (после position control): захват цели на engage,
  каскад position P → `VelocityLoop` → ток. `Brake` остаётся дефолтом.
- [ ] **Интегрирующий current/voltage fault-детектор** вместо
  односэмплового трипа (nuisance-trip на regen/EMI = обрыв момента на
  ходу). Референсы: VESC `mc_interface.c:1881`, MESC динамические пороги.
- [ ] G474: взвести IWDG, когда оживут motor-модули (FOC ISR).
- [ ] Bench: IWDG reset → PWM safe на железе (спровоцировать hang).
- [ ] Boot: чтение reset-reason + детект крутящегося мотора +
  flying-restart синхронизация.
- [ ] Конфигурируемая post-watchdog политика (coast / regen / hold).
- [ ] `Idempotent` marker trait + `call`/`call_once` хелперы в host-lib.

## Velocity / position

Тюнинг-ограничение (выучено в симе): hall обновляет оценку скорости только
по эджам (6/эл.оборот) — агрессивные kp/ki дают ±100 rad/s limit cycle.
Дефолты намеренно мягкие (kp 0.01, ki 0.2, accel 500 erad/s²).

- [ ] Лаг hall-скорости ограничивает полосу контура — рассмотреть менее
  лаггirovannый источник (скорость обсервера, когда доступна) до погони за
  жёсткими гейнами.
- [ ] Bench: тюнинг kp/ki/accel под Flipsky + массу борда; поведение через
  hall→observer кроссовер.
- [ ] PositionControl: position P → `omega_target` в тот же контур
  (каскад); сперва нужен unwrapped-источник позиции.

## Алгоритмы (лестница, дёшево → дорого)

- [ ] Position loop (см. выше).
- [ ] **Graduated derating** (VESC override matrix): линейный спад
  эффективных лимитов по T_fet/T_motor/V_bat/ERPM поверх bus-limits.
- [ ] **Field weakening V2** (MESC: экспоненциальный d-ток от упирания
  вектора напряжения в круг — без параметров мотора).
- [ ] MTPA.
- [ ] Настоящая overmodulation-стратегия (сейчас `modulation_limit` до 1.2
  просто клампит duty выше линейной зоны SVPWM).
- [ ] Автоопределение pole pairs; offset-калибровка энкодера.
- [ ] **Saliency-монитор для HFI** (sim теперь умеет триггерить отказ:
  `lq_sat_k` + тест `closed_loop_hfi_saliency_collapse_loses_tracking_silently`).
  Коллапс/инверсия saliency под нагрузкой НЕВИДИМЫ для confidence
  (eps≈0 читается как идеальный лок) — угол молча уезжает при здоровой
  несущей. Индустрийные ответы: dual-axis HFI45 (MESC), мониторинг
  градиента eps. До монитора — это аргумент держать HFI-моторы с запасом
  по Lq-сатурации.
- [ ] HFI-амплитуда детекции: floor от разрешения АЦП — gimbal-класс
  (риппл ~30 мА против 15 мА LSB) даёт Lq −14%, λ +8% на non-ideal
  планте; адаптация сейчас целится в долю hold-тока, не глядя на LSB.
- [ ] HallPll: PLL-вариант hall-эстиматора на базе `BackEmfObserver`-
  структуры (граничный якорь уже сделан) — прототип на VirtualMotor.
  [notes/hall-improvements.md §4]
- [ ] Hall: точный якорь на пропущенном эдже (low-prio). `crossed_boundary`
  не проверяет смежность: midpoint несмежных центроидов = центроид
  пропущенного сектора, −30° от реальной границы (старый код врал на +30°
  — не регресс; centroid-фолбэк НЕ лучше: та же ошибка зеркально + хуже
  скорость). Точный фикс: midpoint(центроид предшествующего ПО
  ПОСЛЕДОВАТЕЛЬНОСТИ сектора в текущем направлении, центроид нового) —
  точен и по базе, и по traversed-скорости для одиночных пропусков.
  Смягчающие: аппаратный капчер (OVERCAPTURES==0), error_count,
  drift-коррекция самолечит за ~5 мс. Естественно закрывается HallPll.

## Sensorless startup (см. notes/startup-and-sampling.md)

- [ ] **Align → ramp → handoff** state machine для холодного старта без
  HFI (замена фикс-52-rad/s наджа из `try_observer_fallback`).
- [ ] **Flying restart** (kick-push кейс): measure-only проход обсервера /
  HFI-проба перед моментом из Stopped/Coast; seed и сразу в closed loop.
- [ ] Current-scheduled ramp ceiling (VESC `openloop_rpm_max = map(I)`).
- [ ] Хост-тесты на VirtualMotor: cold start с произвольного угла без
  реверс-рывка; freewheel-catch.

## Sensorless tracking / BEMF (bench-blocked)

- [ ] Поднять фазные делители B-G431B-ESC1 (BEMF sense) как ADC-каналы.
- [ ] MESC-style TRACKING: gates off → измеренные v_αβ в обсервер →
  flying start с конвергированного обсервера. Hall-based уже работает;
  это sensorless-кейс. Заодно открывает spin-down flux метод на железе
  (`supports_coast_telemetry`).

## Firmware / core

- [ ] Виртуальное устройство симулирует только CurrentControl/Stopped;
  OpenLoop/DirectVoltage/SixStep/Brake принимаются и игнорируются; нет
  fault-injection (fault-путь хоста не покрыт e2e); конфиг не доходит до
  физики VirtualMotor.
- [ ] Остаток ISR-дедупликации: сборка ADC snapshot + voltage/temp fault
  checks — пер-платформенные копии. Вынести ISR-glue в core ДО оживления
  g474 (иначе он воспроизведёт уже починенные F405-баги).
- [ ] g474 motor-модули закомментированы до подключения IHM08M1;
  `control/foc.rs` синхронизируется руками без compile-check.
- [ ] **g474 + IHM08M1: чеклист перед включением мотора**
  (см. [hw/nucleo-g474re-ihm08m1.md](hw/nucleo-g474re-ihm08m1.md)):
  - config.rs BOARD содержит константы IHM07M1: для IHM08M1 — шунты
    0.010 Ω, TSV994, gain ≈5.18, offset ≈1.71 V, ≈51.8 mV/A, FS ≈ ±31 A;
    JP2 меняет feedback — проверить фактический gain на стенде.
  - CURRENT REF (PB4 PWM) опционален: BKIN-компараторы автономны
    (фикс. Vref ≈30 A); PB4 — порог отдельного U23 → CPOUT → TIM1_ETR.
  - BKIN PA6 (AF6) active-LOW + BKF; опц. PA11 = BKIN2; BKIN-флаг в FOC
    ISR + MOE re-arm (портировать с g431).
  - ADC injected по маппингу (ADC1 PA0/PA1/PC2, ADC2 PC1/PC0, TRGO2);
    удалить закомментированный internal-OPAMP план в peripherals.rs.
  - Re-enable control/motor/calibration; GPIO_BEMF (PC9) off; IWDG;
    PB15/PB14 строго Hi-Z.
  - Джамперы шилда (заводской дефолт 1-shunt/6-step!): J5/J6 → 3-Sh,
    JP1+JP2 closed, снять C3/C5/C7, JP3 closed, J9 open, Nucleo JP5 → E5V.

## VirtualMotor fidelity (анализ: notes/virtual-motor-fidelity.md)

Каждый эффект — за опциональным параметром с идеальным дефолтом
(decisions.md), каждый апгрейд — с тестом, падающим без компенсации.
Сделано 2026-06-12: sub-stepping (`substeps`), dead-time (`dead_time_v`),
квантование+шум (`adc_lsb_a`/`adc_noise_a`), Lq-сатурация (`lq_sat_k`),
duty-драйв харнесса с ARR 4250, one-cycle delay (`actuation_delay_steps` —
сразу нашёл measurement-frame баг phase advance и pipeline-skew детекции,
см. decisions.md и пункт выше). Осталось:

- [ ] **Vbus sag** (`vbus0 − i_bus·R_esr`) — UV-dip / regen-OV сценарии,
  база для динамического Vmax.
- [ ] **Coulomb friction + ω²-нагрузка** — stiction для standstill
  HFI/open-loop стартов; физичность eskate/drone строк каталога.
- [ ] Позже: coupled dynamometer (два мотора на валу), hall-глитчи
  (0/7, дребезг), cogging — только с anticogging.
- [ ] Несинусоидальная back-EMF (5/7 гармоники λ) — смещение угла
  обсервера на реальной машине.

## Size / performance

Текущие числа и правила — [flash-size.md](flash-size.md); бенчи —
[perf-bench-2026-06-11.md](perf-bench-2026-06-11.md).

- [ ] f405/g474 собираются с `opt-level = 3`, g431 с `"z"` — намеренно, но
  не измерено: что `"z"` стоил бы f405/g474 в ISR-времени.
- [ ] Живой счётчик загрузки ISR (DWT CYCCNT min/max/avg → SlowTelemetry
  раз в секунду): подтвердить shipped-"z" билд in situ; заодно закрывает
  подозрение F405 double-trigger по измеренному ISR-rate.

## Документация

- [ ] architecture.md (~1400 строк) — кандидат на doc-rot: вкомпиленные
  сниппеты сигнатур синхронизируются руками (прецедент: icd.rs).
  Перенести инвариантное в doc-comments/doctests (компилятор стережёт),
  оставить топологию, диаграммы и rationale. [Opus 4.8 review]

## Host

- [ ] `protocol_version` в `HardwareInfo` + `env!("CARGO_PKG_VERSION")`
  вместо хардкода "oxifoc-0.1.0" — обязательно до любого
  релиза/дистрибуции (postcard-схема без self-description: рассинхрон =
  молчаливый мусор).
- [ ] Reconnect state machine не покрыт тестами; slint-wgpu-plot:
  индексная арифметика кольца (`renderer.rs:262`) при большом zoom-out +
  scroll-back может считать Y-auto-range по другому окну, чем рисует шейдер.
- [ ] bridge/remote: пейринг по hardcoded MAC; тесты-заглушки. Дизайн
  пульта — [notes/remote-design.md](notes/remote-design.md).

## Стенд (ожидает железа)

- [ ] **Hall timer-capture валидация** (миграция 2026-06-10): рукой
  крутить — последовательность 1→3→2→6→4→5, скорость без 2×-скоса,
  `OVERCAPTURES == 0`, `read_hall_state_raw` читает пины в AF-режиме.
- [ ] **Hall boundary-anchor фикс (9f936bb)**: d-ток на постоянной
  скорости должен быть отцентрован (до фикса ~30°-лид давал cos-потерю
  момента + d-смещение).
- [ ] Re-run детекции — сохранённые параметры Flipsky смещены в 1.5×
  после фикса нормализации SVPWM; λ, мерянная GUI-шагом до 2026-06-12,
  мусор (q-axis метод) — перемерить. Заодно проверить L-ступень с
  реальным dead-time: до 2026-06-12 vd_hold считался как R·I и на
  Flipsky (0.3 В hold против 0.38 В дисторсии g431) она бы упала с
  MotorNotResponding — фикс (settled hold + comp в apply_dq) доказан
  симом, на железе не проверялся.
- [ ] Детекционные PI (0.01/10) — в 10× горячее VESC (0.001/1.0):
  проверить сходимость на железе.
- [ ] **F405 ADC double-trigger**: TIM1_CH4 compare стреляет дважды за
  center-aligned период; g431 `COMPARE_OC4`→TRGO2 принципиально не
  иммунен. Робастный фикс — один детерминированный триггер/период
  (update event или TIM→DMA→ADC). Проверить JEOC-rate под нагрузкой.
- [ ] **Detection pipeline-skew — ПОДТВЕРЖДЁН симом 2026-06-12, критичен**:
  `record()` парит ток с инжекцией глубиной 1 итерацию, но конвейер
  актуации добавляет цикл — эффективная латентность command→measure = 2.
  С `actuation_delay_steps: 1` в планте L-ступень разваливается
  (+1000…+6500% / FAIL — 90° фазы несущей на 5 кГц), driven-flux −46%
  (telem.vq тоже текущего цикла), pulse теряет окно (InsufficientSamples).
  Реальное железо ИМЕЕТ этот конвейер (+ async-доставка команд может
  добавить ещё) → стендовому L доверять нельзя до фикса. Фикс теперь
  разрабатывается В СИМЕ без железа: параметризовать глубину спаривания
  (или latency-probe кросс-корреляцией на старте hfi_collect), то же для
  flux (prev vq) и pulse-окна; включить `actuation_delay_steps: 1` в
  non-ideal таблицу отчёта. На стенде — только сверка эталонной
  индуктивностью.
- [ ] OCP с BKF break-фильтром под реальной нагрузкой (g431).
- [ ] Dead-time компенсация на низкой скорости.
- [ ] Hall-dropout на скорости и sensorless кроссовер.
- [ ] HFI на реальном B-G431B-ESC1: амплитуда несущей теперь решается из
  измеренной L (Flipsky: эквив. ~25 µH, saliency Lq/Ld ≈ 1.5 по LCR и
  VESC-детекции — HFI физически осмыслен); проверить целевой риппл 2 А
  на железе, polarity-probe амплитуду/длительность тюнить по месту.
  Преднагрев несущей и trust-гейт холодного демода (jam) — проверить
  downward-кроссовер на стенде.
- [ ] Source switching end-to-end через `oxifoc-host-cli source ...`.
- [ ] Качество токов на ~90% модуляции + SNR HFI-демода на одном
  V0-сэмпле (V0_V7 — только если стенд покажет проблему)
  [notes/startup-and-sampling.md, scope-решение].
