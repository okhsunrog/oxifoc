# Lossless fast-telemetry batches (ICD v8)

The ADC ISR produces the same 34-byte `FastTelemetry` frames as v7. No ADC,
PWM or fixed-point bits are discarded. Encoding runs in the async stream task;
host enrichment and Parquet columns use the original decoded frames.

Each `FastTelemetryBatch.data` is independent and begins with a full frame:

| Tag | Following bytes | Meaning |
| --- | --- | --- |
| 0 | 34 Pod bytes | Complete little-endian frame; also the fallback |
| 1 | 5-byte LE mask, then replacement bytes | Set mask bit i replaces byte i of the previous decoded frame; values follow increasing i |

Only bits 0–33 of the mask are valid. The encoder uses sparse replacements
only when they are smaller than a full frame. Clear/restart always begins a
new base, so a lost batch cannot corrupt any later batch. The wire includes
every sequence counter; no samples are inferred, interpolated or repeated by
the codec.

The DTO has a 480-byte payload limit. The encoder also caps sample count at 24
and reserves 35 bytes before consuming another ISR queue grant, so even a full
fallback fits. Incompressible input therefore yields shorter batches, rather
than dropped samples or oversized network messages. The conservative 32-byte
framing allowance keeps every batch within F405's 512-byte MTU. Compression
ratio and achievable throughput depend on the data; the fallback itself does
not promise that arbitrary high-entropy data will stream losslessly at 20 kHz.

The decoder validates the entire batch before exposing samples: unknown tags,
truncation, delta without a base, reserved mask bits and more than 24 samples
are rejected. Tests cover random full-width bit patterns, raw fallback,
counter/angle wraps, negative scalars, phase-ADC sentinels, PWM state changes,
MTU limits, rejected pushes, restart and malformed inputs.

F405 additionally runs USB control/RX/TX on an interrupt executor at P6, below
ADC P0, Hall P1 and USB hardware P5. It uses the otherwise unused USART1 NVIC
vector; USART1 hardware and its pins are not enabled. USB objects are built
inside that executor, then its local spawner starts separate control/RX/TX
tasks. Non-Send control handlers never cross executors. Queue waiters use
critical-section synchronization through the runtime's bbqueue configuration.

All USB tasks share one priority: splitting control into thread mode and bulk
I/O into a preempting executor exposed self-repolling during an endpoint wake
in the pinned Embassy AtomicWaker. A captured debug backtrace identified that
path. Keeping USB together avoids that preemption. The first raw-frame IRQ
experiment and compression on thread mode each failed qualification; the
combined path is checked on hardware with phase outputs inhibited.
Production stationary-current recording also passed at nominal 20 kHz; see
[qualification results and timing limits](bench-current-voltage-2026-11-08.md).

`bench-no-phase-outputs` is a default-off F405 qualification feature. Every
phase-enable path instead disables both high and low outputs, while ADC/FOC,
timer compares and USB continue. The firmware announces
`oxifoc-0.1.0+outputs-inhibited`; every saved enable mask must have bits 0–5
clear. This exercises CPU/transport load without exciting the motor. It is
not a motor-response or inverter-noise qualification.
