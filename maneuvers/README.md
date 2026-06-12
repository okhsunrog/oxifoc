# Манёвры — скриптуемые протоколы экспериментов

Манёвр (в смысле лётных испытаний / system identification) — плоская
временная шкала команд, исполняемая против девайса под синхронную запись
fast-телеметрии в parquet. Один и тот же файл гоняется против
`oxifoc-virtual` и против платы → A/B-записи идентичного входного
профиля для диффа сим/железо.

```sh
oxifoc-host-cli maneuver validate maneuvers/iq-step.json     # офлайн
oxifoc-host-cli --transport tcp --json maneuver run maneuvers/iq-step.json \
    --out captures/iq-step-$(date +%s).parquet
```

## Формат

```json
{
  "name": "iq-step-2A",
  "description": "...",
  "capture": { "fast_hz": 5000, "tail_s": 1.0 },
  "terminal": "stop",
  "timeline": [
    { "t": 0.5, "cmd": { "start": { "iq": 2.0 } } },
    { "t": 2.5, "cmd": { "start": { "iq": 0.0 } } }
  ]
}
```

Команды: `start{iq,id}`, `velocity{rad_s}`, `openloop{current,velocity,angle}`,
`voltage{vd,vq,angle}`, `sixstep{duty}`, `stop{}`, `coast{}`, `brake{}`.
`terminal` (`stop`/`coast`/`brake`) отправляется на ЛЮБОМ пути выхода из
таймлайна, включая ошибки. Перед стартом таймлайн проверяется против
`max_iq_a` девайса (`--force` — пропустить, девайс всё равно клампит).

## Анализ

Метаданные parquet несут сам манёвр (`oxifoc.maneuver`) и журнал событий
(`oxifoc.events`): для каждой команды — t_plan/t_sent/t_acked и
**seq-якоря** (`seq_before`, `seq_after_ack`). Эпохи режутся по seq, не
по wall-clock:

```python
import json, pyarrow.parquet as pq, polars as pl
f = pq.ParquetFile("cap.parquet")
ev = json.loads(dict(f.metadata.metadata)[b"oxifoc.events"])
df = pl.read_parquet("cap.parquet")
# окно после события 0: df.filter(pl.col("seq") >= ev[0]["seq_after_ack"])
```

Каверзы:
- расстояние «команда→отклик в данных» — это реальная латентность
  доставки+применения. На `oxifoc-virtual` она ~10 мс и зависит от
  `--batch` (команды обрабатываются на границе батча симуляции) — при
  диффе сим/железо выравнивай эпохи по фактическому фронту, а не по
  якорю, если латентности различаются;
- ненагруженный виртуальный мотор с дефолтным трением за ~100 мс
  упирается в voltage saturation — длинные токовые удержания на нём
  показывают коллапс iq (физика, не баг);
- `oxifoc-virtual` одноклиентский: только одна CLI-сессия одновременно.

## Каталог

- `iq-step.json` — переходная токового контура (PI, латентность конвейера);
- `const-speed.json` — среднее id на постоянной скорости (валидация
  boundary-anchor + advance-сплита; смещение id, растущее со скоростью =
  ошибка угла коммутации);
- `coast-decay.json` — выбег: форма распада erpm даёт трение, на железе
  с BEMF-сенсом окно coast — ground truth для λ.
