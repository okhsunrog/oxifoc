# Documentation

## Working with OxiFOC

| Page | Use it for |
|---|---|
| [Project README](../README.md) | Builds, tools and supported targets |
| [Motor commissioning](commissioning.md) | Source limits, detection, saving a setup and editing settings |
| [Architecture](architecture.md) | Control loop, feedback selection, transports and configuration |
| [Safety](safety.md) | Failsafe, command delivery and recovery design |
| [Maneuvers](../maneuvers/README.md) | Reproducible motor tests, capture format and timing |
| [Open work](TODO.md) | Tasks and bench questions still worth pursuing |

## Hardware

- [VESC 6 MK5 and C5 bridge](hw/vesc6-mk5.md)
- [Cheap FOCer 2](hw/cheap-focer2-notes.md)
- [NUCLEO-G474RE + IHM08M1](hw/nucleo-g474re-ihm08m1.md)
- [Register access](register-access.md) and [flash budget](flash-size.md)

## Bench and research

- [Hall-to-observer tuning](notes/hall-observer-tuning.md) records the current
  thresholds, their baseline and how to revisit the estimator.
- [FOC loop journal](notes/foc-loop-research.md) records MK5 timing, telemetry,
  low-speed behavior and crossover captures.
- [Commissioning design and implementation history](notes/motor-setup-wizard.md)
  and [protocol design notes](notes/protocol-versioning.md).
- [Decisions](decisions.md) records the rationale at the time of each decision.
  Later implementation can supersede an entry.
- [Research notes](notes/) and [archive](archive/) contain experiments, earlier
  board sessions and proposals. The active guides above describe the current
  code. [Backlog snapshot](archive/backlog-through-2026-11-13.md) preserves the
  detailed session logs and old review notes removed from the working backlog.

Keep user-facing commands in the guides and API details near the code that
defines them. For bench results, record the firmware revision, setup, test
conditions, result and artifact location. A local path is useful for
reproducing work on the bench machine; include the decisive numbers or a plot
in the repository so the conclusion survives cleanup of local captures.
