# Fast-telemetry enrichment — shared raw→engineering decode

Status: **IMPLEMENTED** (8ee6f84 codec+enrich, 3ef160a host wiring, 8436a01
GUI; header was stale until 2026-07-06). How the CLI/GUI reconstruct engineering
units (amps, volts, id/iq) from the compact 18-byte raw `FastTelemetry`
([rtt-telemetry-throughput.md](rtt-telemetry-throughput.md)) using **one code
path in `oxifoc-core`** shared by firmware and host, so the two can't desync.

## Goal & principle

The raw frame ships ADC counts + fixed-point scalars; the host reconstructs the
rest. Because the host links `oxifoc-core`, anything that is a deterministic
function of the sent inputs is recomputed host-side with the **same** code
(bit-identical) and is therefore omitted from the frame. The only desync risk is
where firmware *encodes* and host *decodes* — fix by making those a single
paired definition in core.

## 1. Scalar codec — one LSB per field, paired enc/dec (kills desync)

The frame's scaled fields (`vbus`, `vd`, `vq`, `rpm`) are currently encoded
inline in `build_fast_telemetry` (`vbus/2`, `vd*500`, …) with **no decode
counterpart** — the one desync risk. Extract a fixed-point helper into core so
encode and decode are structurally inverse, one constant per field:

```rust
// oxifoc-core, std+no_std
pub struct Scale { lsb: f32 } // physical units per quantum
impl Scale {
    pub const fn new(lsb: f32) -> Self { Self { lsb } }
    #[inline] pub fn enc(self, v: f32) -> i32 { (v / self.lsb) as i32 } // truncates
    #[inline] pub fn dec(self, raw: i32) -> f32 { raw as f32 * self.lsb }
}
const VBUS: Scale = Scale::new(0.002); // 2 mV/LSB (volts)
const VOLT: Scale = Scale::new(0.002); // vd/vq, 2 mV/LSB
const RPM:  Scale = Scale::new(2.0);   // 2 mech RPM/LSB
```

`build_fast_telemetry` calls `pack_*` (= `Scale::enc`); `enrich` calls the
decode (`Scale::dec`). One `lsb` per field → can only change in one place →
both directions move together.

**angle** is modular (0..2π wrapping), so its own paired methods, one constant
`ANGLE_PER_LSB = TAU/65536`: `pack_angle(rad)->u16` (wrap via truncating turn
count, `rem_euclid` is std-only) and `angle_rad()->f32 = angle * ANGLE_PER_LSB`.

**Currents are NOT scaled** — they ship as raw ADC counts and decode only via
`ShuntCurrentSense` (below).

## 2. `enrich()` — the shared decode, in core

```rust
pub struct EnrichCtx { pub isense: ShuntCurrentSense, pub pole_pairs: u8 }
pub struct RichSample {
    pub ia, ib, ic, i_alpha, i_beta, id, iq: f32, // A
    pub vbus_v, vd, vq, angle_rad, mech_rpm, erpm: f32,
    pub seq: u16,
}
impl FastTelemetry {
    pub fn enrich(&self, c: &EnrichCtx) -> RichSample {
        let (ia, ib, ic) = c.isense.convert_raw(self.ia, self.ib, self.ic); // SAME fn as the FOC loop
        let (al, be) = clarke(ia, ib);                                       // SAME transforms as FOC
        let a = self.angle_rad();
        let (id, iq) = park(al, be, libm::sinf(a), libm::cosf(a));
        // vbus/vd/vq/rpm via the Scale decode; erpm = mech_rpm * pole_pairs
        ...
    }
}
```

Reused, already host-portable (pure, no_std-clean):
- `ShuntCurrentSense::convert_raw` (`foc/current_sense.rs`) — the firmware's own
  ADC→amps. Add `ShuntCurrentSense::from_calib(&BoardCalib)`.
- `clarke`/`park` (`foc/transforms.rs`).

The host (CLI and GUI both link host-lib→core) calls `frame.enrich(&ctx)`. There
is no second implementation to drift.

## 3. Where the calibration comes from

`EnrichCtx` needs static board constants + dynamic offsets + pole_pairs. Split by
mutability (see [protocol-versioning.md](protocol-versioning.md) §6):

- **`BoardCalib`** (static, compile-time): `shunt_ohms, amp_gain, adc_vref_mv,
  adc_max_counts, invert_current_sign, vbus_divider_ratio`. Make it a **sub-struct
  of `BoardConfig`** (not a duplicate) and the wire type, carried in the oxifoc
  `AppInfoEndpoint` (handshake, read once). Firmware builds its converter from
  the same `BoardConfig.calib`.
- **`dc_offsets`** (dynamic, recalibration): existing `ConfigGroupId::DcOffsets`
  read. NOT in the cached descriptor (would go stale on recalibration; the host
  triggers recalibration so it re-reads).
- **`pole_pairs`** (dynamic): existing `MotorParamsConfig.pole_pairs` read.

Host at connect: read AppInfo (`BoardCalib`) + `DcOffsets` + `MotorParams` →
`ShuntCurrentSense::from_calib(calib)` + offsets + pole_pairs → `EnrichCtx`.

## 4. Open question — raw counts vs offset-corrected delta

The frame ships **raw absolute** ADC counts (host needs offsets over the wire).
Alternative: device ships `counts − offset` (i16 delta) → host needs only static
`BoardCalib`, no dynamic offsets.

| | raw counts + offsets | offset-corrected delta |
|---|---|---|
| host needs offsets | yes (config read, stale risk) | **no** (static only) |
| rail/fault visible (4095 ≠ real current) | **yes** | no (delta masks rail) |
| MCU work | 0 | −1 subtract/phase |

**Leaning raw** (this is a diagnostic stream — rail/fault visibility matters);
offsets via the existing config read, staleness handled by host-triggered
recalibration re-reads. For long warm-up captures, a live offset could ride
`SlowTelemetry` (10 Hz) without touching the cached descriptor.

## 5. Tests (all in core, run on host CI — same code compiled into firmware)

The anti-desync guarantee: the round-trip test exercises **both** the
`build_fast_telemetry` encode and the `enrich` decode in one core test; there is
no separate host implementation that could drift.

1. **Scale round-trip**: `dec(enc(v)) ≈ v` within `±lsb` (truncation bias up to
   1 LSB) for VBUS/VOLT/RPM over a value sweep.
2. **Frame round-trip**: `RichSample → pack → FastTelemetry → enrich →
   RichSample`, every field within 1 LSB / known current tolerance.
3. **Golden**: a hand-computed raw frame + `BoardCalib`/offsets → expected
   amps/volts/id-iq (pins sign and scale).
4. **Property**: `enc` inverse of `dec` ±LSB over random inputs; `convert_raw`
   vs `BoardConfig::convert_raw_currents` at zero offset.
5. Extend the existing `transforms`/`current_sense` tests to round-trip through
   `enrich`.

## 6. Build order

(a) core: `Scale` codec + `pack_*`/decode + `enrich`/`RichSample` +
`BoardCalib`/`EnrichCtx` + tests (foundation, no hardware) → refactor
`build_fast_telemetry` onto the codec. (b) `BoardCalib` sub-struct of
`BoardConfig` + carried in `AppInfoEndpoint`. (c) host: fetch calib at connect +
`enrich` in `record`/`watch` (+ parquet amps/id/iq columns), shared by CLI & GUI.
