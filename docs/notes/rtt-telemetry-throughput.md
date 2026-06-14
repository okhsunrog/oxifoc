# Fast-telemetry throughput over ergot / RTT — analysis & chosen frame

What caps the device→host telemetry rate over the SWD/RTT debug link, every lever
that was measured (and which ones actually move the number), and the resulting
`FastTelemetry` frame design with per-field justification.

Session: 2026-06-14, branch `bench-detection-2026-06-13`.
Hardware: NUCLEO-G474RE (STM32G474, onboard **STLINK-V3E**) for the V3 numbers;
B-G431B-ESC1 (STM32G431, **STLINK-V2-1**) for the V2-1 baseline.

> Bench note: the g474 used here is an RTT-only throughput harness (single
> ergot-over-RTT interface + a synthetic telemetry generator, since the board has
> no motor shield / FOC ISR). The transport and frame findings transfer directly
> to the real boards.

---

## 1. The pipeline

```
FOC ISR / generator → FAST_TELEM_Q (bbqueue) → fast_telemetry_stream
   → postcard-encode FastTelemetryBatch<N> → COBS → OUTQ
   → RttWriter → RTT up-channel (device RAM)
        ⇡ SWD reads (probe-rs)               ← THE LINK
   → host RTT worker thread → COBS decode → ergot topic → record/parquet
```

RTT is **polled**: the host reads the target's RTT ring buffer out of RAM over
SWD. There is no DMA push from the device — every transfer is the debug probe
issuing SWD memory reads. That single fact drives most of the analysis below.

---

## 2. Raw link ceiling (no ergot, `probe-rs benchmark`)

`probe-rs benchmark` measures pure SWD memory read/write bandwidth = the hard
ceiling any RTT scheme can reach. Read is the telemetry direction.

| SWD clock | Read | Write |
|-----------|------|-------|
| 3.0 MHz *(old host default, see §4.1)* | ~125 KB/s | ~135 KB/s |
| 6.4 MHz | ~284 KB/s | ~319 KB/s |
| 9.6 MHz | ~471 KB/s | ~565 KB/s |
| **24 MHz (V3 max per spec)** | **~620 KB/s** | ~940 KB/s |
| STLINK-V2-1 (4.6 MHz cap) | ~169 KB/s | — |

Two structural facts:

- **Read is round-trip-latency bound, not clock bound.** It barely grows 9.6→24
  MHz (471→620) while write scales much further (565→940). Telemetry is the read
  direction, so ~620 KB/s is the practical V3 ceiling.
- **Big contiguous reads win.** At 24 MHz: 4-byte reads ≈ 23 KB/s, 32 B ≈ 137
  KB/s, 2 KB ≈ 629 KB/s. Per-transaction overhead dominates; you only approach
  the ceiling by reading multi-KB chunks per SWD transaction.

**V3 vs V2-1: ~3.7× on the read ceiling (620 vs 169 KB/s).**

---

## 3. Measurement method

`oxifoc-host-cli record --transport rtt --fast-hz 20000 --allow-gaps` over a fixed
3 s window; the reported rows ÷ duration = delivered samples/s. The synthetic
generator **saturates** the queue (pushes faster than the link drains), so the
delivered rate equals the throughput ceiling and the "samples lost" / gap counts
are an expected over-feed artifact, not real loss. Payload KB/s = rate × Pod
frame size.

---

## 4. Every lever, measured

### 4.1 Host SWD clock — **decisive** (was silently capped)

The host requested `set_speed(4_600)`; on the V3 probe-rs clamped that to **3.3
MHz** → ~125 KB/s, barely above a V2-1. Raising the request to the V3 ceiling
(`set_speed(24_000)`) unlocked the full ~620 KB/s. A V2-1 still clamps to its own
4.6 MHz, so a single high request value is safe for both.
Fix: `oxifoc-host-lib/src/transport/rtt.rs` (env override `OXIFOC_SWD_KHZ`).

### 4.2 Correct firmware ELF — **decisive** (was a hard bug)

The host pins the RTT control block to the firmware's `_SEGGER_RTT` symbol read
from an ELF. `resolve_elf_path` defaulted to the **g431** artifact for *any*
board. On g474 it pinned the wrong RAM address → either "RTT control block not
found" (flaky attach) or a latched/desynced block where the device never sees the
down-channel → `NoRouteToDest`, the link never routes. Both earlier "24 MHz attach
flakiness" and the routing failure were this one bug.
Fix: added a required `--elf` CLI flag; `resolve_elf_path` now **errors** instead
of guessing a board (`oxifoc-host-cli` `--elf`, `oxifoc-host-lib/src/lib.rs`).

### 4.3 Device RTT up-channel buffer — **helps to a knee**

Bigger device buffer ⇒ host reads larger chunks per SWD transaction (see §2).

| up-channel size | rate |
|-----------------|------|
| 4096 | 41.5k smp/s (8 B frame) |
| **8192** | 46.9k (+13%) |
| 16384 / 32768 | no further gain |

Knee at **8192**; beyond it the bottleneck moves upstream. (The earlier belief
that "8192 broke ergot" was actually §4.2 — the wrong ELF.)

### 4.4 Host poll cadence (busy-spin) — **no effect**

The RTT worker thread is dedicated/blocking; a capture lasts seconds. Idle sleep
swept 0 (busy-spin) → 20 ms: throughput **identical** (~46.9k). Conclusion: the
host is never the bottleneck — it keeps up trivially. Default is busy-spin
(`OXIFOC_RTT_IDLE_US=0`) for minimum latency.

### 4.5 Host defmt poll cadence — **no effect**

Reading the defmt channel every loop costs an extra SWD round-trip, but
rate-limiting it (every 64th iter) didn't move throughput → confirms host-side
SWD contention is not the limit. Kept the rate-limit anyway (free).

### 4.6 Stream cadence — **minor**

`fast_telemetry_stream` sleeps half-a-batch between drains. Shrinking that
interval: 46.9k → 50.5k (+8%), plateaus by ~51k. Small, upstream-side gain.

### 4.7 Batch N (structs per ergot packet) — **no effect**

N = samples packed into one `FastTelemetryBatch<N>` broadcast. Swept 64 / 128 /
256 at MTU 4096: **flat at ~50k smp/s / ~395 KB/s**. N changes how the same
byte-stream is *chunked into packets*, not the total bytes/s, so it does not move
a byte-rate-bound link. (N must still be ≤ the host receiver capacity, currently
`FastTelemetryTopic<256>`, and N×frame ≤ MTU.)

### 4.8 Frame size — **the real lever**, with a caveat

The path is **byte-rate bound** (~500–560 KB/s of wire after framing overhead).
Bigger frames carry more useful payload per fixed packet/transaction overhead, so
useful KB/s rises toward the link ceiling:

| frame (Pod) | rate | useful payload | % of 620 raw |
|-------------|------|----------------|--------------|
| 8 B | 51k | 400 KB/s | 65% |
| 16 B | 29.5k | 472 KB/s | 76% |
| 24 B | 20.6k | 482 KB/s | 78% |
| 32 B | 16.1k | 502 KB/s | 81% |
| 44 B (f32-heavy) | 13.1k | **563 KB/s** | 91% |

### 4.9 postcard varint — **format efficiency trap**

postcard varint-encodes integers: small values are cheap (1 B), but a `u16` ≥
16384 costs **3 B** on the wire. `f32` is fixed 4 B (no varint). So an
integer-heavy frame can *bloat* on the wire, while the f32-heavy 44 B frame was
wire-efficient — that is why 44 B beat the smaller pad frames on useful KB/s.
A measured 18 B-Pod frame with realistic values encodes to ~20 wire B (angle and
seq genuinely span the full u16 → 3 B each). To remove varint entirely and get
deterministic `frame_size × N` wire bytes, send the batch as **raw Pod bytes**
(bytemuck cast) instead of postcard — a future lever worth ~+10–15%.

---

## 5. What actually limits it (summary)

1. **Link**: SWD read bandwidth, latency-bound, ~620 KB/s at 24 MHz on V3. The
   hard ceiling; only big contiguous reads approach it.
2. **Framing overhead**: ergot header + COBS + postcard (incl. varint) → the
   ergot path tops out ~500–560 KB/s of wire, ~80–90 % of the raw link.
3. **Not the host** (busy-spin/idle/defmt cadence: no effect) and **not packet
   count** (batch N: no effect). Levers that matter: SWD clock, device buffer to
   the 8 KB knee, frame *payload density*.

---

## 6. Chosen `FastTelemetry` frame — 18 bytes

Design rule: **send only what the device measures or decides and the host cannot
recompute.** Since the host links `oxifoc-core`, anything that is a deterministic
function of the sent inputs is reconstructed host-side with the *same* math
(bit-identical), and is therefore omitted.

| field | type | bytes | encoding | why it must be sent |
|-------|------|-------|----------|---------------------|
| `ia`,`ib`,`ic` | `u16`×3 | 6 | raw ADC counts | measured. Host → amps via `BoardConfig` + `dc_offsets`; `iα/iβ/id/iq` via Clarke/Park |
| `vbus` | `u16` | 2 | **×2 mV** | measured; independent. Normalization, power, saturation, duty denorm |
| `angle` | `u16` | 2 | 0..2π full-scale | estimator output (stateful) — not reproducible; needed for Park |
| `vd`,`vq` | `i16`×2 | 4 | **×2 mV** | PI outputs (depend on integrator state) — not reproducible; needed for impedance R(f)/L(f), power, observer |
| `rpm` | `i16` | 2 | **×2 mech RPM** | filtered observer output — cleaner than host-side Δangle differentiation |
| `seq` | `u16` | 2 | FOC-cycle mod 65536 | loss / order detection |

**Total 18 B**, all 2-byte fields ⇒ `align = 2`, no padding, `bytemuck::Pod`
clean.

### What is intentionally *omitted* (reconstructed host-side)

- `id`, `iq`, `iα`, `iβ` — pure `Clarke/Park(ia,ib,ic, angle)`.
- duty cycles — `SVPWM(vd, vq) / vbus`; redundant with `vd/vq + vbus`.
- `hall_state` — not needed for current-spectrum diagnostics; meaningless on
  sensorless; available in slow telemetry (10 Hz) if ever needed.

### Encoding justifications

- **Currents as raw ADC, not amps.** 3×`u16` (6 B) vs 3×`f32` (12 B). Also *more*
  faithful: no on-device f32 conversion, host calibrates consistently. Host needs
  only a one-time config read (no per-frame cost): `adc_vref_mv`,
  `adc_max_counts`, `amp_gain`, `shunt_ohms`, `invert_current_sign` (all static
  `BoardConfig`) + per-phase zero-current `offset_counts` (from `dc_offsets`
  calibration). Formula:
  `i = (counts − offset)·(vref_mv/1000/max_counts)/(amp_gain·shunt_ohms)·sign`.
  Caveat: the DC offset drifts with temperature — fine for short (~0.2 s)
  spectral windows; for long warm-up captures, refresh the offset from slow
  telemetry or recalibrate.

- **`vbus`, `vd`, `vq` in ×2 mV, not ×1 mV.** ×1 mV (`u16`/`i16` = 65.5 V / ±32.8
  V) covers only ≤48 V boards. `vd/vq` are bounded by `vbus/√3`, so `i16` mV
  saturates at `vbus ≈ 56.7 V`. The roadmap targets VESC boards (60/75/84/100 V
  classes) → **×2 mV** gives 131 V / ±65 V at the same 2 bytes, 2 mV resolution
  (negligible). Absolute volts, no `vbus`-relative coupling. (A modulation-index
  `vd/vbus` in Q15 would never clip at any bus voltage — kept as a fallback if
  >130 V is ever needed.)

- **`rpm` mechanical in ×2 RPM.** eRPM in `i16` is hopeless (`eRPM = KV·V·pp`;
  saturates ~17 V at 7 pole-pairs). Mechanical RPM in ×1 (`i16` = ±32.8k RPM)
  covers a 270 KV/48 V motor (13k) but a high-KV drone motor (2000 KV × 24 V =
  48k) clips. **×2 RPM** → ±65.5k RPM at the same 2 bytes; 2 RPM resolution is
  irrelevant for a slow, filtered, contextual quantity in a fast frame. No need
  for `i32`. Host multiplies by pole-pairs for eRPM if wanted.

- **`seq` as `u16`, not `u32`.** At 20 kHz it wraps every ~3.28 s; loss/order
  detection uses `wrapping_sub` and is unambiguous for gaps < 32768 samples
  (1.6 s of continuous loss — never a valid capture). ergot topics over one COBS
  channel deliver in order, so `seq` is mostly for *loss* (dropped batches on
  queue overflow). For captures > 3.3 s, the host reconstructs the time axis by
  accumulating `Σ wrapping_sub`, not by absolute `seq`. Saves 2 B vs `u32`.

---

## 7. Result: 18 B frame on V3 @ 24 MHz

| frame | rate | payload | sustains 20 kHz? |
|-------|------|---------|------------------|
| 44 B engineering (main) | 13.1k smp/s | 563 KB/s | ✗ (0.66×) |
| **18 B raw (chosen)** | **27.8k smp/s** | 490 KB/s | ✅ **1.39×** |
| 8 B raw-current only | 51k smp/s | 400 KB/s | ✅ (2.5×) |

The 18 B frame streams full **20 kHz with ~40 % headroom** on V3 — carrying raw
3-phase currents + vbus + angle + vd/vq + rpm + seq. On a V2-1 (~169 KB/s) only
the ~8 B current-only frame fit 20 kHz; the rich 44 B frame managed ~2.5k smp/s.
**V3 is what makes a full diagnostic frame at full rate possible.**

Optional future lever: raw-Pod batch encoding (drop postcard varint) → ~31k
smp/s / 1.55× on the same 18 B frame.

---

## 8. Reproduce

```
# raw link ceiling
probe-rs benchmark --chip STM32G474RETx --probe <V3> --address 0x20010000 \
    --min-speed 9000 --max-speed 50000

# end-to-end ergot-over-RTT (must pass --elf for a non-g431 board)
oxifoc-host-cli --transport rtt --probe <V3> --chip STM32G474RETx \
    --elf target/thumbv7em-none-eabihf/release/oxifoc-g474 \
    record --out cap.parquet --seconds 3 --fast-hz 20000 --allow-gaps
```

Env overrides (host RTT worker): `OXIFOC_SWD_KHZ` (default 24000),
`OXIFOC_RTT_IDLE_US` (default 0 = busy-spin), `OXIFOC_RTT_DEFMT_EVERY` (default 64).
