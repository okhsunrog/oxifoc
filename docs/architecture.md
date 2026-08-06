# Oxifoc Architecture

This document describes the FOC (Field-Oriented Control) architecture for the oxifoc project.

## Design Principles

1. **Separation of Concerns**: Each component has a single, well-defined responsibility
2. **Hardware Abstraction**: Core logic is platform-agnostic; hardware details stay in platform crates
3. **Runtime Flexibility**: Control modes and phase sources can be switched at runtime
4. **Compile-Time Safety**: Invalid configurations are caught at compile time where possible
5. **Zero Allocation**: All structures use stack allocation, suitable for `no_std` embedded
6. **Trait-Based Extensibility**: New sensors and controllers can be added without modifying core

## High-Level Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       Host / User Application                            │
│  • Set control mode (current, open-loop, direct voltage, six-step)      │
│  • Select phase source (Hall, Encoder, Observer, HFI, crossovers)       │
│  • Run calibration/detection procedures, read/write stored config       │
└─────────────────────────────────┬───────────────────────────────────────┘
                                  │ ergot endpoints → CMD_CHANNEL
                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          FocDriver<P, C, Phase, S>                       │
│  • Orchestrates control loop execution (mutated only inside the ISR)    │
│  • Manages control modes (Stopped, Current, OpenLoop, DirectVoltage,    │
│    Coast, SixStep, ...)                                                  │
│  • Enforces current limits (target clamp + measured overcurrent)        │
│  • Delegates phase estimation to PhaseProvider                          │
└───────────┬─────────────────────┬─────────────────────┬─────────────────┘
            │                     │                     │
            ▼                     ▼                     ▼
    ┌───────────────┐    ┌───────────────┐    ┌───────────────────────┐
    │   PhasePwm    │    │ CurrentSensor │    │    PhaseProvider      │
    │   (trait)     │    │   (trait)     │    │      (trait)          │
    │               │    │               │    │                       │
    │ • set_duties  │    │ • read_currents│   │ • get() → PhaseOutput │
    │ • max_duty    │    │ • is_calibrated│   │ • injection()         │
    │ • disable     │    │ • get_offsets │    │ • update(PhaseInput)  │
    └───────────────┘    └───────────────┘    │ • request_source()    │
                                              └───────────┬───────────┘
                                                          │
                                              implements  │
                                                          ▼
                              ┌────────────────────────────────────────────┐
                              │         PhaseManager<H, E>                  │
                              │  • Manages Hall sensor (H)                  │
                              │  • Manages Encoder (E)                      │
                              │  • Two concurrent estimator slots:          │
                              │    back-EMF Observer + HFI                  │
                              │  • Handles source selection & blending      │
                              └────────────────────────────────────────────┘
```

The command path from the host into the ISR-owned driver is described in
[Command Path and Communication](#command-path-and-communication).

---

## Core Components

### 1. FocDriver

The main motor driver that orchestrates FOC control.

```rust
pub struct FocDriver<P, C, Phase, S: SinCos = LibmSinCos>
where
    P: PhasePwm,
    C: CurrentSensor,
    Phase: PhaseProvider,
{
    // FOC controller (current loop, parameterized over SVPWM + trig impl)
    controller: FocController<SvpwmModulator, S>,
    // PWM output
    pwm: P,
    // Current sensor
    current_sensor: C,
    // Phase provider (manages angle sources)
    phase: Phase,
    // Current control mode
    mode: ControlMode,
    // Bus voltage (V)
    vbus: f32,
    // Control loop period in seconds (1/pwm_freq, from MotorPwmConfig::dt_s())
    dt: f32,
    // Current limiting (target clamp + measured overcurrent threshold)
    current_limits: CurrentLimits,
    // Accumulated angle for open-loop velocity mode
    open_loop_angle: f32,
}
```

**Responsibilities:**
- Execute the FOC current control loop
- Handle control mode transitions (PWM re-enable on Stopped→active,
  phase-state restore when leaving SixStep, PI gain override on OpenLoop entry)
- Coordinate with PhaseProvider for angle estimation and HFI carrier injection
- Enforce current limits: clamp commanded targets (circular, d-axis priority)
  and latch `Stopped` + disable PWM when the *measured* dq current exceeds
  `overcurrent_threshold_a` — in every mode that energizes the motor,
  including DirectVoltage and SixStep

**Key simplification:** All phase/angle complexity is delegated to
`PhaseProvider`. The fourth type parameter `S: SinCos` defaults to
`LibmSinCos`; platforms with hardware trig (G4 CORDIC) substitute their own.

### 2. PhaseProvider Trait

Abstraction for electrical phase angle provision.

```rust
/// Output from phase provider
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PhaseOutput {
    pub angle: f32,      // Electrical angle (radians, 0 to 2π)
    pub velocity: f32,   // Electrical velocity (rad/s)
}

/// Input for phase provider update
#[derive(Clone, Copy, Debug, Default)]
pub struct PhaseInput {
    pub v_alpha: f32,    // Commanded α voltage (for observer)
    pub v_beta: f32,     // Commanded β voltage (for observer)
    pub i_alpha: f32,    // Measured α current
    pub i_beta: f32,     // Measured β current
    pub dt: f32,         // Time step (seconds)
}

/// Provides electrical phase angle to FOC controller
pub trait PhaseProvider {
    /// Get current phase estimate (called at START of control step)
    fn get(&self) -> PhaseOutput;

    /// Update with latest measurements (called at END of control step)
    fn update(&mut self, input: &PhaseInput, now_ticks: u64);

    /// dq voltage to inject this cycle (HFI carrier), in the rotor frame
    /// at get()'s angle. Default: no injection.
    fn injection(&self) -> (f32, f32) {
        (0.0, 0.0)
    }

    /// Request a switch of the angle source (host command). Returns whether
    /// the request was applied. Default declines: simple providers have
    /// exactly one source.
    fn request_source(&mut self, _source: PhaseSource) -> bool {
        false
    }
}
```

**Design rationale:**
- `get()` is called first to obtain the angle for Park/Clarke transforms
- `update()` is called last with the commanded voltages (needed by observers)
- This creates a one-sample delay for observers, which is standard practice
- `injection()` must be read BETWEEN `get()` and `update()`: the HFI
  estimator demodulates the currents fed to the next `update()` against this
  exact carrier sample, and `update()` then advances the carrier.
  `FocDriver::step_current_control` feeds the result into
  `FocController::step_with_injection`
- `request_source()` is the validated entry point for host-side source
  switching; `PhaseManager` overrides it with its `set_source()`

### 3. PhaseManager

Concrete implementation of `PhaseProvider` that manages multiple angle sources with automatic fallback.

```rust
pub struct PhaseManager<H = NoSensor, E = NoSensor>
where
    H: AngleSensor,
    E: AngleSensor,
{
    // Hardware sensors
    hall: H,
    encoder: E,

    // Software estimators. Two slots run concurrently so the HfiToX
    // crossovers can hand over between them: `hfi` covers zero/low speed
    // (needs carrier injection), `observer` covers medium/high speed.
    observer: Observer,
    hfi: Option<HfiObserver>,

    // Configuration
    source: PhaseSource,

    // State
    output: PhaseOutput,
    manual_angle: f32,
    open_loop_angle: f32,
    open_loop_velocity: f32,

    // Timebase
    ticks_per_sec: u64,

    // Hall health tracking (VESC-style)
    hall_health: HallHealth,      // Ok, Stale, Invalid, NotPresent
    hall_failure_ticks: Option<u64>,

    // Open-loop override for Hall failure recovery
    open_loop_override: OpenLoopOverride,

    // Hysteresis memory for the HfiToX crossovers: true = running on the
    // high-speed source (observer/hall/encoder), false = on HFI.
    crossover_latched: bool,

    // Fault tracking
    faults: HeaplessVec<PhaseFault, 4>,
}

pub enum HallHealth {
    Ok,         // Hall working normally
    Stale,      // No edges for timeout period
    Invalid,    // Returning invalid states (0 or 7)
    NotPresent, // Hall not configured
}

pub enum PhaseFault {
    HallTimeout,
    HallInvalidState,
    ObserverNotReady,
}
```

**Responsibilities:**
- Sample hardware sensors (Hall, Encoder)
- Update **both** software estimators every cycle (always, for
  fallback/crossover readiness)
- Select/blend angle sources based on `PhaseSource`
- Track Hall sensor health and trigger fallback
- Generate the HFI carrier via `injection()` when an HFI source is
  commutating (off once a crossover is fully latched onto the fast source)
- Provide unified phase output to FocDriver

**Fallback Chain:**
When Hall fails, PhaseManager automatically falls back:
1. **Hall** → try Observer (gated on `is_ready()`, not merely configured)
2. **Observer not ready** → use OpenLoop override
3. **OpenLoop** → maintain minimum velocity until a real source takes over

With the `storage` feature, `configure_observers_from_config()` arms both
estimator slots from stored motor parameters: a `BackEmfObserver` from
(R, L_avg, λ) plus an `HfiObserver` with default carrier settings
(`HFI_DEFAULT_FREQ_HZ`, `vbus × HFI_DEFAULT_AMPLITUDE_RATIO`). The active
source is left untouched — the estimators only run; the host selects a
sensorless source explicitly when it wants one.

---

## Phase Source Selection

### PhaseSource Enum

Specifies where electrical angle comes from. `PhaseSource` is a wire type
(serde + postcard schema) — the host selects it via the
`PhaseSourceEndpoint`.

```rust
pub enum PhaseSource {
    // === Direct hardware sensor ===
    Hall,                    // Use Hall sensor
    Encoder,                 // Use encoder

    // === Software estimation ===
    Observer,                // Back-EMF observer (sensorless)
    Hfi,                     // High-frequency injection

    // === Hybrid modes ===
    HallToObserver {         // Hall at low speed, observer at high speed,
                             // automatic fallback on hall failure
        blend_low: f32,      // Start blending (electrical rad/s)
        blend_high: f32,     // Full observer (electrical rad/s)
    },
    EncoderToObserver {      // Encoder at low speed, observer at high speed
        blend_low: f32,
        blend_high: f32,
    },
    HfiToObserver {          // HFI startup, blend to observer
        min_vel: f32,        // Fully on observer at this velocity (rad/s)
        min_confidence: f32, // Minimum observer confidence (0.0-1.0)
    },
    HfiToObserverVolts {     // Like HfiToObserver, crossover on |vq − R·iq|
        toggle_v: f32,       // Drive-voltage threshold (V)
        min_confidence: f32,
    },
    HfiToHall {              // HFI startup, switch to Hall (with hysteresis)
        switch_vel: f32,
    },
    HfiToEncoder {           // HFI startup, switch to encoder (with hysteresis)
        switch_vel: f32,
    },

    // === Manual control ===
    Manual,                  // Use manually set angle (calibration)
    OpenLoop,                // Open-loop angle ramp (startup, calibration)
}
```

Every hall-consuming source (plain `Hall` included) shares the same failure
chain: hall invalid/stale → observer if `is_ready()` → open-loop recovery
override (52 rad/s from the last angle, signed by the last velocity) until a
real source returns. The hybrid sources add the velocity blend on top.

### Validation

PhaseManager validates source changes:

```rust
impl<H: AngleSensor, E: AngleSensor> PhaseManager<H, E> {
    pub fn set_source(&mut self, source: PhaseSource) -> Result<(), PhaseSourceError> {
        if source.requires_hall() && !self.has_hall() {
            return Err(PhaseSourceError::HallNotAvailable);
        }
        if source.requires_encoder() && !self.has_encoder() {
            return Err(PhaseSourceError::EncoderNotAvailable);
        }
        if source.requires_observer() && !self.observer.is_configured() {
            return Err(PhaseSourceError::ObserverNotConfigured);
        }
        // HFI sources need the estimator that actually generates a carrier
        if source.requires_hfi() && self.hfi.is_none() {
            return Err(PhaseSourceError::HfiNotConfigured);
        }

        self.source = source;
        // Crossover memory belongs to the previous source's thresholds.
        self.crossover_latched = false;
        Ok(())
    }
}
```

The trait-level `request_source()` simply wraps this:
`self.set_source(source).is_ok()`.

### HFI Crossover Semantics

The `HfiToX` sources use the `crossover_latched` hysteresis flag plus
`CROSSOVER_HYSTERESIS = 0.2`:

- **HfiToObserver** is a velocity **blend**, not a sharp switch: the output
  blends from HFI to the observer across the band
  `[min_vel·(1 − CROSSOVER_HYSTERESIS), min_vel]`. A sharp switch is not
  enough here — the HFI demod/PLL lag grows with speed, so at the crossover
  the two estimates legitimately disagree by tenths of a radian; blending
  absorbs that. The blend is only taken when the observer is ready AND its
  confidence ≥ `min_confidence`; the speed reference is the faster of the
  two velocity estimates.
- `crossover_latched` marks the fully-blended regime (`blend ≥ 1.0`). Only
  there is the carrier injection switched off (`injection()` returns zero) —
  keeping the carrier on at speed only costs losses and acoustic noise while
  the saliency response degrades anyway.
- Re-entering the band from above (or losing observer readiness) reseeds the
  HFI estimator from the last managed output via `set_phase()`/
  `set_velocity()` — its own estimate drifted while the carrier was off, and
  the trusted angle also resolves the HFI π ambiguity.
- **HfiToHall/HfiToEncoder** are sharp switches with the same hysteresis
  band: switch up at `switch_vel`, drop back to HFI only below
  `switch_vel × (1 − CROSSOVER_HYSTERESIS)` or on sensor failure (again
  reseeding HFI from the last output).

---

## Sensor Traits Hierarchy

### Base Trait: AngleSensor

Common interface for all angle sensors:

```rust
pub struct AngleSample {
    pub angle: f32,          // Electrical angle (radians)
    pub omega: f32,          // Electrical velocity (rad/s)
    pub direction: Direction,
}

pub trait AngleSensor {
    /// Sample angle at given timestamp (None = no valid sample right now)
    fn sample(&self, now_ticks: u64) -> Option<AngleSample>;

    /// Stateful snapshot for the control path. Default delegates to sample();
    /// sensors with cross-call smoothing state (e.g. the Hall estimator's
    /// VESC-style rate limiter) override this.
    fn sample_mut(&mut self, now_ticks: u64) -> Option<AngleSample> {
        self.sample(now_ticks)
    }

    /// Whether the sensor's data has implausibly stopped updating (e.g. hall
    /// edges ceased while the rotor was demonstrably spinning).
    /// Default: never stale.
    fn is_stale(&self, _now_ticks: u64) -> bool {
        false
    }

    /// Read electrical angle in radians (0..2π). Default uses sample().
    fn read_angle(&self) -> f32 { /* default via sample() */ }

    /// Read rotation direction. Default uses sample().
    fn read_direction(&self) -> Direction { /* default via sample() */ }

    /// Error count for diagnostics
    fn error_count(&self) -> u32;

    /// Reset error counter
    fn reset_errors(&mut self);
}

/// Null sensor for unused slots
pub struct NoSensor;

impl AngleSensor for NoSensor {
    fn sample(&self, _: u64) -> Option<AngleSample> { None }
    fn error_count(&self) -> u32 { 0 }
    fn reset_errors(&mut self) {}
}
```

### Extended Trait: HallSensorTrait

Hall-sensor-specific operations:

```rust
pub trait HallSensorTrait: AngleSensor {
    /// Raw 3-bit Hall state (0-7)
    fn raw_state(&self) -> u8;

    /// Logical Hall state (0-5)
    fn logical_state(&self) -> u8;

    /// Timestamp of last Hall edge
    fn last_edge_ticks(&self) -> Option<u64>;

    /// Electrical velocity from edge timing
    fn electrical_velocity(&self) -> f32;

    /// Set calibration table (angles for logical states 0-5)
    /// For backwards compatibility - prefer `set_calibration_raw`
    fn set_calibration(&mut self, table: [f32; 6]);

    /// Set calibration table using raw Hall states (8-entry table)
    /// This is the preferred method as it works with any Hall sensor wiring
    fn set_calibration_raw(&mut self, raw_table: [f32; 8]);

    /// Apply calibration result
    fn apply_calibration(&mut self, result: &HallCalibrationResult) -> bool;

    /// Set timing advance (radians)
    fn set_advance(&mut self, advance_rad: f32);

    /// Get current timing advance
    fn advance(&self) -> f32;

    /// Get interpolation diagnostics
    fn interpolation_info(&self, now_ticks: u64) -> HallInterpolationInfo;
}

pub struct HallInterpolationInfo {
    pub base_angle: f32,           // Angle from Hall state table
    pub interpolation_offset: f32, // Added by velocity extrapolation
    pub estimated_velocity: f32,   // Velocity used for interpolation
    pub time_since_edge_us: u32,   // Time since last Hall edge
}
```

### HallSensor Implementation (VESC-compatible)

The `HallSensor` struct provides VESC-style features:

```rust
pub struct HallSensor {
    // Calibration: 8-entry raw-state table (direct lookup, no logical conversion)
    calib: HallCalibration,  // raw_table: [f32; 8], valid: [bool; 8], advance_rad

    // VESC-compatible interpolation parameters
    interp_min_erpm: f32,         // Threshold below which interpolation is disabled
    max_drift_rad: f32,           // Max drift before soft correction
    drift_correction_gain: f32,   // Pull-back rate (VESC default: 0.01)
    rate_limit_factor: f32,       // Max angle step multiplier (VESC default: 1.5)

    // State tracking
    direction_reversed: bool,     // True on direction change (for velocity handling)
    timeout_ticks: u64,           // Timeout for stale detection
    // ... edge timestamps, velocity estimate, rate-limited angle, error counter
}

impl HallSensor {
    /// Timebase-aware constructor (ticks of a caller-provided clock)
    pub fn new(ticks_per_sec: u64) -> Self;

    /// Check if Hall sensor data is stale (velocity-adaptive edge timeout)
    pub fn is_stale(&self, now_ticks: u64) -> bool;

    /// Set minimum eRPM for interpolation (VESC-style)
    pub fn set_interp_min_erpm(&mut self, erpm: f32);

    /// Interpolated sample with soft drift correction and rate limiting
    /// (backs the AngleSensor::sample_mut override)
    pub fn sample_at_mut(&mut self, now_ticks: u64) -> Option<AngleSample>;
}
```

### Extended Trait: EncoderSensorTrait

Encoder-specific operations:

```rust
pub trait EncoderSensorTrait: AngleSensor {
    /// Raw encoder count
    fn counts(&self) -> i32;

    /// Set current position as zero
    fn set_zero(&mut self);

    /// Set electrical angle offset
    fn set_offset(&mut self, offset_rad: f32);

    /// Get electrical angle offset
    fn offset(&self) -> f32;

    /// Counts per electrical revolution
    fn counts_per_electrical_rev(&self) -> u32;

    /// Set counts per electrical revolution
    fn set_counts_per_electrical_rev(&mut self, cpr: u32);

    /// Check if index pulse seen
    fn index_seen(&self) -> bool { false }

    /// Reset index flag
    fn reset_index(&mut self) {}

    /// Encoder type
    fn encoder_type(&self) -> EncoderType { EncoderType::Incremental }
}

pub enum EncoderType {
    Incremental,
    IncrementalWithIndex,
    Absolute,
}
```

### Trait Hierarchy Diagram

```
                 AngleSensor
                 (base trait)
                      │
          ┌──────────┴──────────┐
          │                     │
          ▼                     ▼
   HallSensorTrait       EncoderSensorTrait
   (Hall-specific)       (Encoder-specific)
          │                     │
          ▼                     ▼
   Core/platform impls   Platform impls
   (HallSensor,          (planned)
    HallAngleProxy)
```

---

## Observer Integration

### Estimator Slots

`PhaseManager` carries **two** estimator slots that run concurrently:

```rust
pub enum Observer {
    None,
    BackEmf(BackEmfObserver),
}

impl Observer {
    pub fn update(&mut self, input: &ObserverInput);
    pub fn phase(&self) -> Option<f32>;
    pub fn velocity(&self) -> Option<f32>;
    pub fn confidence(&self) -> f32;
    /// Convergence gate: all fallback/crossover decisions use this, not
    /// phase().is_some() — a configured-but-frozen observer returns a phase.
    pub fn is_ready(&self) -> bool;
    /// Seed the estimate from a trusted external source (sensor handoff)
    pub fn seed(&mut self, angle: f32, velocity: f32);
    pub fn is_configured(&self) -> bool;
    pub fn reset(&mut self);
}

pub struct ObserverInput {
    pub v_alpha: f32,
    pub v_beta: f32,
    pub i_alpha: f32,
    pub i_beta: f32,
    pub dt: f32,
}
```

`Observer` is the back-EMF/"fast" slot — it deliberately has **no** HFI
variant. The HFI estimator lives in its own dedicated slot
(`hfi: Option<HfiObserver>`, set via `set_hfi_observer()`): it needs carrier
injection plumbed through the control loop, and the HfiToObserver crossover
requires both estimators to run concurrently. `PhaseManager::update()` feeds
the same `ObserverInput` to both slots every cycle.

### Back-EMF Observer

MXLEMMING-style flux observer (original algorithm by David Molony, MESC
project; also available in VESC as `FOC_OBSERVER_MXLEMMING`). Integrates
`(v − R·i)·dt − L·Δi` to track the rotor flux vector directly, truncates each
component to ±λ to bleed off integrator drift, then uses a PLL to extract
phase and velocity.

```rust
pub struct BackEmfObserver {
    // Flux integrator state
    x1: f32,              // α-axis rotor flux estimate (Wb)
    x2: f32,              // β-axis rotor flux estimate (Wb)

    // Previous currents for the incremental −L·Δi stator-flux removal
    i_alpha_last: f32,
    i_beta_last: f32,

    // PLL state
    phase_pll: f32,       // PLL-filtered phase
    velocity_pll: f32,    // PLL-filtered velocity

    // Motor parameters
    r: f32,               // Phase resistance (Ω)
    l: f32,               // Phase inductance (H)
    lambda: f32,          // Flux linkage (Wb)

    // Tuning
    pll_kp: f32,          // PLL proportional gain
    pll_ki: f32,          // PLL integral gain

    // State
    confidence: f32,      // Flux magnitude / λ (0-1)
    phase_err_filt: f32,  // Low-passed |PLL phase error|, for readiness
}
```

There is no explicit observer gain — the ±λ truncation *is* the MXLEMMING
correction mechanism. Readiness (`is_ready()`) requires all three of:
- confidence ≥ `READY_MIN_CONFIDENCE` (flux magnitude near λ),
- PLL locked (filtered |phase error| < `READY_MAX_PHASE_ERR_RAD`),
- |velocity| ≥ `READY_MIN_VELOCITY` (back-EMF observable at all — at
  standstill the first two can hold on pure integrator memory).

### HFI Observer

High-frequency injection estimator for zero/low speed, based on magnetic
saliency (Ld ≠ Lq). Pulsating **d-axis** injection: each cycle the control
loop reads `get_injection()` → `(A·cos θc, 0)` in the estimated rotor frame,
applies it via `FocController::step_with_injection`, and the next `update()`
synchronously demodulates the resulting carrier currents by `sin θc`. The
demodulated q channel gives an error ∝ `sin 2e · (1/Lq − 1/Ld)`; the
always-positive d channel normalizes it (gains independent of A, ωc and
absolute inductance) and provides the "is any carrier current flowing"
confidence floor. A PLL tracks the normalized error.

The saliency signal is 2θ-periodic, so the PLL lock carries a **π ambiguity**.
It is resolved by a saturation-probe state machine
(`Pending → Probing → Done`):
- After the first PLL lock (confidence crosses `HFI_READY_CONFIDENCE`) the
  carrier is suspended and palindromic ±d pulses (+,−,−,+ — cancels
  first-order bias from residual current decay) are injected, with
  zero-voltage gaps between them for current decay.
- A pulse aligned with the magnet flux saturates the iron → lower incremental
  Ld → larger current. If the −d̂ pulses consistently draw more current, the
  estimate is flipped by π. An ambiguous result (no measurable saturation,
  e.g. SPM motors) keeps the current lock.
- `is_ready()` requires confidence ≥ `HFI_READY_CONFIDENCE` **and** resolved
  polarity. `set_phase()` from a trusted source (sensor handoff, crossover
  reseed) marks polarity resolved directly — no probe needed.

---

## Control Modes

### ControlMode Enum

`ControlMode` lives in `oxifoc-core/src/types.rs` (it is a wire type) and is
re-exported from `motor::foc_driver`. There is no separate "HfiInjection"
mode — HFI is a *phase source*; runtime HFI runs in `CurrentControl` via the
`PhaseProvider::injection()` path, and detection-time injection uses
`DirectVoltage`.

```rust
pub enum ControlMode {
    /// Motor stopped, PWM disabled
    Stopped,

    /// Current control mode (torque control)
    CurrentControl {
        /// Target q-axis current (torque-producing)
        iq_target: f32,
        /// Target d-axis current (field-weakening)
        id_target: f32,
    },

    /// Velocity control mode (speed control) - NOT YET IMPLEMENTED
    VelocityControl {
        /// Target velocity in rad/s
        target_vel: f32,
    },

    /// Position control mode - NOT YET IMPLEMENTED
    PositionControl {
        /// Target position in radians
        target_pos: f32,
    },

    /// Open-loop mode — drive motor at commanded electrical angle.
    /// velocity_rad_s == 0: lock rotor at angle_rad (calibration).
    /// velocity_rad_s != 0: firmware advances the angle (open-loop spin).
    OpenLoop {
        /// Initial electrical angle (radians, 0 to 2π)
        angle_rad: f32,
        /// Current magnitude (Amps)
        current: f32,
        /// Electrical velocity (rad/s) — 0 = lock, nonzero = spin
        velocity_rad_s: f32,
        /// Optional PI gains override (kp, ki), applied on mode entry.
        /// Used by detection when motor params are unknown.
        pi_gains: Option<(f32, f32)>,
    },

    /// Direct voltage mode — apply dq voltages without PI control.
    /// Used for measurement modes (HFI inductance detection) and bringup.
    DirectVoltage {
        /// d-axis voltage (V)
        vd: f32,
        /// q-axis voltage (V)
        vq: f32,
        /// Electrical angle (radians)
        angle_rad: f32,
    },

    /// Coast mode — all FETs off (high-impedance), motor spins freely.
    /// Used during spin-down flux linkage measurement.
    Coast,

    /// Six-step (trapezoidal) commutation — voltage-mode bringup drive.
    /// Sign of duty determines direction.
    SixStep {
        /// Duty cycle (-1.0 to 1.0)
        duty: f32,
    },
}
```

All energizing modes that can read currents trip the measured-overcurrent
protection — including `DirectVoltage` (no PI loop reins the current in, so
the measured check is its only software protection) and `SixStep` (checked on
the αβ magnitude, since there is no dq frame).

### Control Loops (Planned)

Outer loop controllers for velocity and position control:

```rust
/// Outer loop controller interface (PLANNED)
pub trait OuterLoop {
    fn update(&mut self, setpoint: f32, feedback: f32, dt: f32) -> f32;
    fn reset(&mut self);
}

/// Runtime-switchable outer loop (PLANNED)
pub enum OuterLoopType {
    None,
    Pi(PiLoop),
    PiWithFeedforward(PiWithFeedforward),
}
```

When implemented, velocity/position modes will cascade through the outer
loops to generate current targets.

---

## Motor Parameter Detection

The detection module provides async functions for measuring motor parameters. These use a `DetectionHardware` trait for platform abstraction.

### DetectionHardware Trait

```rust
/// Hardware abstraction for motor detection routines.
pub trait DetectionHardware {
    /// Send a control mode command to the FOC driver.
    fn send_command(&self, mode: ControlMode);

    /// Wait for next FOC telemetry (PWM-synchronized).
    fn wait_telemetry(&mut self) -> impl Future<Output = FocOutput>;

    /// Read raw phase currents (ia, ib, ic) in Amps.
    fn read_phase_currents(&self) -> (f32, f32, f32);

    /// Read coast-down telemetry (v_alpha, v_beta, omega_e) during
    /// spin-down flux measurement. Default returns zeros (triggers
    /// fallback to driven measurement).
    fn read_coast_telemetry(&self) -> (f32, f32, f32) {
        (0.0, 0.0, 0.0)
    }
}
```

### Timer Trait

Platform-agnostic async timer for detection delays:

```rust
/// Platform-agnostic timer trait for async delays.
pub trait Timer {
    /// Delay for the specified number of milliseconds.
    fn after_millis(ms: u64) -> impl Future<Output = ()>;

    /// Delay for the specified number of microseconds.
    fn after_micros(us: u64) -> impl Future<Output = ()>;
}
```

### Detection Functions

```rust
/// Measure motor phase resistance (2-point differential, MESC-style)
pub async fn measure_resistance<H: DetectionHardware, T: Timer>(
    hw: &mut H, params: &ResistanceParams,
) -> Result<f32, DetectionError>;

/// Measure motor inductance using rotating HFI
pub async fn measure_inductance<H: DetectionHardware, T: Timer, S: SinCos>(
    hw: &mut H, params: &InductanceParams, pwm_freq_hz: f32,
) -> Result<(f32, f32), DetectionError>;  // (Ld, Lq)

/// Measure motor inductance using voltage pulses (alternative method)
pub async fn measure_inductance_pulse<H: DetectionHardware, T: Timer, S: SinCos>(
    hw: &mut H, params: &VoltagePulseParams, pwm_freq_hz: f32,
) -> Result<(f32, f32), DetectionError>;

/// Measure motor flux linkage via open-loop spinning
/// (plus measure_flux_linkage_magnitude / measure_flux_linkage_spindown variants)
pub async fn measure_flux_linkage<H: DetectionHardware, T: Timer>(
    hw: &mut H, params: &FluxLinkageParams,
) -> Result<f32, DetectionError>;

/// Run full motor parameter detection sequence
pub async fn run_full_detection<H: DetectionHardware, T: Timer, S: SinCos>(
    hw: &mut H, params: DetectionParams,
) -> Result<DetectionResult, DetectionError>;

/// Calibrate Hall sensors
pub async fn calibrate_hall<H: DetectionHardware, T: Timer, R: HallReader>(
    hw: &mut H, reader: &R, params: HallCalibrationParams,
) -> Result<HallCalibrationResult, DetectionError>;
```

---

## Control Flow

### Main Control Step

`step()` takes only `now_ticks` — the loop period `dt` is stored in the
driver (set at construction from the PWM config).

```rust
impl<P: PhasePwm, C: CurrentSensor, Phase: PhaseProvider, S: SinCos> FocDriver<P, C, Phase, S> {
    pub fn step(&mut self, now_ticks: u64) -> Result<FocOutput, &'static str> {
        let dt = self.dt;
        match self.mode {
            ControlMode::Stopped => {
                self.pwm.disable();
                self.phase.update(&PhaseInput { dt, ..Default::default() }, now_ticks);
                Ok(FocOutput::default())
            }
            ControlMode::CurrentControl { iq_target, id_target } => {
                self.step_current_control(iq_target, id_target, dt, now_ticks)
            }
            ControlMode::OpenLoop { angle_rad, current, velocity_rad_s, .. } => {
                self.step_open_loop(angle_rad, current, velocity_rad_s, dt, now_ticks)
            }
            ControlMode::DirectVoltage { vd, vq, angle_rad } => {
                self.step_direct_voltage(vd, vq, angle_rad, dt, now_ticks)
            }
            ControlMode::Coast => { /* all phases Float, reset PI, update phase */ }
            ControlMode::SixStep { duty } => self.step_six_step(duty, dt, now_ticks),
            ControlMode::VelocityControl { .. } => Err("Velocity control not implemented"),
            ControlMode::PositionControl { .. } => Err("Position control not implemented"),
        }
    }

    fn step_current_control(&mut self, iq_target: f32, id_target: f32, dt: f32, now_ticks: u64)
        -> Result<FocOutput, &'static str>
    {
        // 0. Sensor must be calibrated
        if !self.current_sensor.is_calibrated() {
            return Err("Current sensor not calibrated");
        }

        // 1. Layer 1: clamp current targets (circular, d-axis priority)
        let (id_target, iq_target) = self.current_limits.clamp_targets(id_target, iq_target);

        // 2. Get phase estimate (from previous update)
        let phase_out = self.phase.get();
        let angle_rad = phase_out.angle;

        // 3. HFI carrier for this cycle (zero for non-HFI sources). Must be
        //    read between get() and update(): the estimator demodulates the
        //    currents fed to update() against this exact carrier sample.
        let (vd_inject, vq_inject) = self.phase.injection();

        // 4. Read currents and run FOC controller (PI outputs + injection)
        let currents = self.current_sensor.read_currents();
        let max_duty = self.pwm.max_duty();
        let out = self.controller.step_with_injection(
            currents, angle_rad, id_target, iq_target, vd_inject, vq_inject, max_duty, dt,
        );

        // 5. Layer 2: measured overcurrent → disable PWM, latch Stopped
        if self.current_limits.is_overcurrent(out.id, out.iq) {
            self.pwm.disable();
            self.controller.reset();
            self.mode = ControlMode::Stopped;
            return Err("Overcurrent: measured current exceeds limit");
        }

        // 6. Set PWM duties; feed duties to the current sensor for
        //    next-cycle reconstruction
        self.pwm.set_duties(out.duties);
        self.current_sensor.update_duties(out.duties);

        // 7. Update phase provider for next iteration (feeds both estimators)
        self.phase.update(&PhaseInput {
            v_alpha: out.v_alpha,
            v_beta: out.v_beta,
            i_alpha: out.i_alpha,
            i_beta: out.i_beta,
            dt,
        }, now_ticks);

        Ok(out)
    }
}
```

### Phase Selection in PhaseManager

```rust
impl<H: AngleSensor, E: AngleSensor> PhaseManager<H, E> {
    fn compute_phase_with_fallback(
        &mut self,
        hall_sample: Option<AngleSample>,
        encoder_sample: Option<AngleSample>,
    ) -> PhaseOutput {
        match self.source {
            PhaseSource::Hall => {
                // VESC-style: Hall first, observer fallback if Hall failed
                if let Some(sample) = hall_sample {
                    PhaseOutput { angle: sample.angle, velocity: sample.omega }
                } else {
                    self.try_observer_fallback().unwrap_or(self.output)
                }
            }
            PhaseSource::Encoder => sample_to_output(encoder_sample, &self.output),
            PhaseSource::Observer => {
                // Pure sensorless: only commutate from a converged observer;
                // hold the last output (and raise ObserverNotReady) otherwise.
                /* ... gated on self.observer.is_ready() ... */
            }
            PhaseSource::Hfi => {
                // HFI estimate straight from the dedicated slot. No readiness
                // gate: HFI is valid from standstill by design.
                self.hfi_output().unwrap_or(self.output)
            }
            PhaseSource::HallToObserver { blend_low, blend_high } => {
                if hall_sample.is_none() {
                    return self.try_observer_fallback().unwrap_or(self.output);
                }
                let sensor = sample_to_output(hall_sample, &self.output);
                self.blend_with_observer(sensor, blend_low, blend_high)
            }
            PhaseSource::HfiToObserver { min_vel, min_confidence } => {
                // Velocity blend across [min_vel·(1−CROSSOVER_HYSTERESIS), min_vel]
                // with crossover_latched hysteresis + HFI reseed on the way down
                /* ... see "HFI Crossover Semantics" ... */
            }
            PhaseSource::Manual => PhaseOutput { angle: self.manual_angle, velocity: 0.0 },
            PhaseSource::OpenLoop => PhaseOutput {
                angle: self.open_loop_angle,
                velocity: self.open_loop_velocity,
            },
            // ... other variants
        }
    }
}
```

`blend_with_observer` additionally guards against the back-EMF observer's
half-turn ambiguity: while the sensor still has any blend weight, a >90°
disagreement means the observer locked onto the inverted flux vector and it
is reseeded from the sensor instead of being blended toward.

---

## Command Path and Communication

The `FocDriver` is owned by the platform's FOC ISR and mutated only there.
Every async-side request to change it goes through a single ordered channel:

```rust
/// Command for the ISR-owned FocDriver (oxifoc-core/src/state.rs)
pub enum DriverCommand {
    /// Change control mode (start/stop/targets)
    SetMode(ControlMode),
    /// Apply current limits (already clamped to the board ceiling)
    SetCurrentLimits(CurrentLimits),
    /// Apply current-loop PI gains (post-detection tune, config write)
    SetPiGains { kp: f32, ki: f32 },
    /// Switch the angle source (hall / observer / HFI / crossovers)
    SetPhaseSource(PhaseSource),
}

/// Servers send DriverCommands here, ISR receives them
pub static CMD_CHANNEL: Channel<CriticalSectionRawMutex, DriverCommand, 8> = Channel::new();
```

One channel (rather than per-purpose signals) keeps commands sequenced:
"set limits, then start" arrives exactly that way.

`state::process_commands()` drains the channel inside the ISR and:
- **Validates** each command with `DriverCommand::is_sane()` — wire input is
  arbitrary bits, so non-finite (NaN/inf) payloads are dropped at the
  boundary instead of feeding the PI loop
- Applies `SetMode` only when not in the Error latch / no critical fault is
  registered (the Error latch is exited only after the host explicitly
  cleared the fault registry)
- Applies `SetPhaseSource` via `PhaseProvider::request_source()` (the
  manager validates sensor/estimator availability) and, on success,
  **mirrors the applied source into `MotorControlState.phase_source`** so
  the host can read it back via telemetry
- Enforces a link-loss failsafe: while the link is inactive (liveness timed
  out), the driver is forced to `Stopped` regardless of the last commanded
  mode

### Shared ISR Helpers

Platform ISRs only read their ADCs and call two shared helpers from core:

```rust
// One FOC cycle of driver work (oxifoc-core/src/state.rs):
// commands → fault gating → driver.step() → measured-current fault check
pub fn run_foc_cycle<P, C, Ph, S, F>(
    state_mutex: &CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &FaultRegistry<F>,
    driver: &mut FocDriver<P, C, Ph, S>,
    vbus_v: f32,
    now_ticks: u64,
    board: &BoardConfig,
    overcurrent_fault: F,
) -> Option<FocOutput>;

// Publish one cycle's telemetry (oxifoc-core/src/runtime/streaming.rs):
// update global state (waking calibration/detection listeners) and, when
// fast streaming is enabled, push a decimated sample into the lock-free queue
pub fn publish_cycle_telemetry(state_mutex, adc, hall, foc, seq);
```

### Protocol Endpoints

Ergot endpoints (defined in `oxifoc-core/src/icd.rs`) are served by the
generic servers in `runtime/servers.rs`:

| Endpoint | Path | Request → Response |
|----------|------|--------------------|
| `HardwareInfoEndpoint` | `req/hardware_info` | `()` → `HardwareInfo` |
| `MotorEndpoint` | `cmd/motor` | `ControlMode` → `MotorStatus` |
| `PhaseSourceEndpoint` | `cmd/phase_source` | `PhaseSource` → `PhaseSourceAck` |
| `TelemetryConfigEndpoint` | `cmd/telemetry_config` | `TelemetryConfig` → `TelemetryConfigAck` |
| `SlowTelemetryEndpoint` | `req/telemetry_slow` | `()` → `SlowTelemetry` |
| `FaultEndpoint` | `cmd/fault` | `FaultRequest` → `FaultResponse` |
| `DetectEndpoint` | `cmd/detect` | `Keyed<DetectRequest>` → `DetectResponse` |
| `ConfigEndpoint` | `cmd/config` | `ConfigRequest` → `ConfigResponse` |

Notable wire-type details:
- `PhaseSource` is itself a wire type; `PhaseSourceAck.enqueued` is an
  *honest* ack — it only confirms the command was enqueued to the ISR. The
  ISR-side switch can still reject an invalid source, so the host reads the
  actually-active source back via `SlowTelemetry::phase_source`.
- `SlowTelemetry` temperatures are `i16` in 0.1 °C units (signed —
  sub-zero temperatures are representable): `fet_temp_c_x10`,
  `motor_temp_c_x10`, `board_temp_c_x10`.
- Detection is the one non-idempotent action: its request carries a `ReqId`
  and the device deduplicates on it (effectively-once retries).

### Config Pipeline

Persistent config uses `sequential-storage` (postcard map) behind a generic
worker, `storage::run_storage_worker()`: it loads all config groups at boot
(signalling `CONFIG_LOADED`), then serves `FLASH_CHANNEL` forever and signals
`FLASH_DONE` (success/failure) after every operation.

`config_server` (in `runtime/servers.rs`) handles host requests:
- **Writes are refused with `ConfigResponse::Busy` while the motor is
  Running** — internal-flash erase stalls the whole chip (up to seconds for
  an F4 sector), which would starve the FOC ISR with the motor energized.
- **Write-through ack**: `Ok` is returned only after `FLASH_DONE` confirms
  the flash write. The in-memory `RuntimeConfig` mirror is updated only after
  the persist succeeds, so it always mirrors what is actually stored.
- **Documented invariant**: `config_server` is the *only* `FLASH_CHANNEL`
  producer, so each `FLASH_DONE` pairs 1:1 with its operation.
- **Live-apply** (via `CMD_CHANNEL`, taking effect without reboot):
  - `CurrentLimits` — clamped through `CurrentLimits::from_config_clamped`:
    the config can lower limits but never raise them above the board's
    hardware ceiling; zero/negative values mean "not set" and fall back to
    board defaults (a config cannot switch protection off)
  - `PiGains` — applied verbatim
  - `MotorParams` — retunes the current loop (`calculate_current_gains` from
    R and L_avg), same as boot does
- `ResetAll` erases storage and restores board-default current limits.

---

## Platform Integration

### Constructing PhaseManager

```rust
// Hall only
let phase = PhaseManager::with_hall(hall_sensor);

// Encoder only
let phase = PhaseManager::with_encoder_only(encoder);

// Hall + Encoder
let phase = PhaseManager::with_hall(hall_sensor)
    .with_encoder(encoder);

// Sensorless: arm both estimator slots
let mut phase = PhaseManager::sensorless();
phase.set_observer(Observer::BackEmf(BackEmfObserver::new(r, l, lambda)));
phase.set_hfi_observer(HfiObserver::new(1000.0, 3.0)); // freq (Hz), amplitude (V)

// Or arm both slots from stored motor params (storage feature):
phase.configure_observers_from_config(&config, vbus);
```

### Constructing FocDriver

```rust
// dt comes from the PWM config (1/pwm_freq)
let driver = FocDriver::new(controller, pwm, current_sensor, phase, PWM_CONFIG.dt_s());
```

### Full Example (G474)

Condensed from `oxifoc-g474/src/control/foc.rs`:

```rust
// Initialize hardware
let current_sensor = G474CurrentSensor::from_board(&BOARD, &IA_SAMPLE, &IB_SAMPLE, &IC_SAMPLE);
let mut phase_manager = PhaseManager::with_hall(HallAngleProxy::new());

// Arm the sensorless estimators (back-EMF + HFI) from detected motor
// params; the angle source stays Hall until the host switches it.
phase_manager.configure_observers_from_config(config, initial_vbus_v);

// Controller from stored config (motor params → PI gains → defaults),
// hardware CORDIC for sin/cos
let foc_controller =
    FocController::<SvpwmModulator, CordicSinCos>::from_runtime_config(config, initial_vbus_v);

// Build FOC driver with dt from PWM config
let mut foc_driver = FocDriver::new(
    foc_controller,
    motor_pwm,
    current_sensor,
    phase_manager,
    PWM_CONFIG.dt_s(),
);

// Current limits: stored config (clamped to the board ceiling) or board defaults
foc_driver.set_current_limits(CurrentLimits::from_stored(
    config.current_limits.as_ref(),
    BOARD.max_phase_current_a,
));

foc_driver.current_sensor_mut().calibrate().await;
FOC_DRIVER.lock(|cell| cell.replace(Some(foc_driver)));

// In the ADC ISR: read ADCs, then call the shared cycle helpers from core
#[interrupt]
fn ADC1_2() {
    // ... read injected ADC samples (ia/ib/ic, vbus, temp), check
    //     voltage/temperature faults, build AdcSnapshot/HallSnapshot ...

    let foc_telem = FOC_DRIVER.lock(|cell| {
        cell.borrow_mut().as_mut().and_then(|driver| {
            oxifoc_core::state::run_foc_cycle(
                &STATE, &FAULT_REGISTRY, driver,
                vbus_mv as f32 / 1000.0, now_ticks, &BOARD,
                G474Fault::OverCurrent,
            )
        })
    });

    oxifoc_core::runtime::streaming::publish_cycle_telemetry(
        &STATE, adc_snapshot, hall_snapshot,
        foc_telem.unwrap_or_default(), seq,
    );
}
```

---

## Conditional Methods Pattern

PhaseManager provides additional methods when specific sensor types are available:

```rust
// Base implementation (always available)
impl<H: AngleSensor, E: AngleSensor> PhaseManager<H, E> {
    pub fn set_source(&mut self, source: PhaseSource) -> Result<(), PhaseSourceError>;
    pub fn set_observer(&mut self, observer: Observer);
    pub fn set_hfi_observer(&mut self, hfi: HfiObserver);
    pub fn hfi_observer(&self) -> Option<&HfiObserver>;
    pub fn set_manual_angle(&mut self, angle: f32);
    pub fn has_hall(&self) -> bool;
    pub fn has_encoder(&self) -> bool;
}

// Storage-backed configuration (feature = "storage")
impl<H: AngleSensor, E: AngleSensor> PhaseManager<H, E> {
    pub fn configure_observers_from_config(&mut self, config: &RuntimeConfig, vbus: f32);
}

// Hall-specific methods (only when H: HallSensorTrait)
impl<H: HallSensorTrait, E: AngleSensor> PhaseManager<H, E> {
    pub fn set_hall_calibration(&mut self, table: [f32; 6]);
    pub fn apply_hall_calibration(&mut self, result: &HallCalibrationResult) -> bool;
    pub fn hall_raw_state(&self) -> u8;
    pub fn hall_logical_state(&self) -> u8;
    pub fn set_hall_advance(&mut self, advance_rad: f32);
    pub fn hall_advance(&self) -> f32;
    pub fn hall_velocity(&self) -> f32;
}
```

---

## Module Structure

```
oxifoc-core/src/
├── lib.rs
├── timer.rs                 # Timer trait for async delays (+ EmbassyTimer)
├── fmt.rs                   # Logging macros (defmt/log/none)
├── types.rs                 # Wire types: ControlMode, telemetry, config protocol
├── icd.rs                   # Ergot endpoints/topics (MotorEndpoint, PhaseSourceEndpoint, ...)
├── state.rs                 # MotorControlState, CMD_CHANNEL, DriverCommand,
│                            #   process_commands(), run_foc_cycle()
├── storage.rs               # Config groups, FLASH_CHANNEL, run_storage_worker()
├── virtual_motor.rs         # PMSM simulation model (closed-loop tests, virtual platform)
├── delivery/                # Delivery-semantics ladder (idempotent/deduplicated)
├── runtime/
│   ├── servers.rs           # Protocol servers (motor, config, fault, phase_source, ...)
│   ├── streaming.rs         # Fast telemetry queue, publish_cycle_telemetry()
│   ├── detect.rs            # Detection server (deduplicated action)
│   └── io.rs
├── foc/                     # (inline module in lib.rs)
│   ├── config.rs            # BoardConfig
│   ├── constants.rs         # Math constants (√3, 1/√3, etc.)
│   ├── controller.rs        # FocController (step / step_with_injection / apply_dq)
│   ├── current_sense.rs     # ShuntCurrentSense helper
│   ├── current_reconstruction.rs
│   ├── fault.rs             # Fault registry + shared fault checkers
│   ├── hall_calibration.rs  # HallCalibrator, HallCalibrationResult
│   ├── hall_sensor.rs       # HallSensor struct
│   ├── hall_embassy.rs      # Embassy hall estimator + HallAngleProxy
│   ├── current_offset.rs    # ISR-owned current-offset calibration state machine
│   ├── pi_controller.rs     # PI controller with anti-windup
│   ├── pwm.rs               # PhasePwm trait, SvpwmModulator
│   ├── sensors.rs           # AngleSensor, CurrentSensor, HallSensorTrait, etc.
│   ├── svpwm.rs             # Space Vector PWM modulator
│   ├── transforms.rs        # Clarke, Park transforms
│   ├── trig.rs              # SinCos trait (LibmSinCos; platforms add CORDIC)
│   ├── phase/
│   │   ├── provider.rs      # PhaseProvider trait (get/update/injection/request_source)
│   │   ├── manager.rs       # PhaseManager (dual estimator slots, crossovers)
│   │   ├── source.rs        # PhaseSource enum
│   │   └── observer.rs      # Observer enum, BackEmfObserver, HfiObserver
│   └── detection/
│       ├── types.rs         # MotorParams, DetectionError, etc.
│       ├── sweep.rs         # DetectionHardware trait, async detection sequences
│       ├── resistance.rs    # Resistance measurement algorithms
│       ├── inductance.rs    # Inductance measurement (rotating HFI)
│       ├── voltage_pulse.rs # Pulse-based inductance measurement
│       ├── flux_linkage.rs  # Flux linkage measurement
│       ├── pi_tuning.rs     # Auto PI tuning
│       ├── embassy_hw.rs    # Shared embassy DetectionHardware impl
│       └── virtual_harness.rs
└── motor/
    ├── foc_driver.rs        # FocDriver, CurrentLimits (ControlMode re-export)
    └── six_step.rs          # Six-step commutation tables

oxifoc-g474/src/             # STM32G474 platform (Nucleo + IHM08M1)
├── main.rs
├── config.rs                # BoardConfig, PWM config, NTC
├── control/foc.rs           # ADC ISR + FOC driver init
├── cordic.rs                # CordicSinCos (hardware trig)
├── calibration.rs           # DetectionHardware glue
├── hardware/ sensors/ protocol/ transport/
├── safety.rs / motor.rs / storage.rs

oxifoc-f405/src/             # STM32F405 platform
├── main.rs
├── config.rs
├── calibration.rs
├── control/foc.rs           # ISR, FOC task
├── hardware/ sensors/ protocol/ transport/
├── fault.rs / motor.rs / storage.rs
```

Other crates in the repo:
`oxifoc-virtual` (host-side virtual motor controller), `oxifoc-host-lib` /
`oxifoc-host-cli` / `oxifoc-host-slint` (host tooling), `oxifoc-bridge` /
`oxifoc-remote` (ESP32-C6 wireless link).

---

## Summary

| Component | Responsibility | Location |
|-----------|---------------|----------|
| **FocDriver** | Control loop orchestration | `motor/foc_driver.rs` |
| **CurrentLimits** | Target clamp + overcurrent threshold | `motor/foc_driver.rs` |
| **FocController** | Current loop math | `foc/controller.rs` |
| **PhaseProvider** | Phase angle abstraction | `foc/phase/provider.rs` |
| **PhaseManager** | Sensor/estimator management | `foc/phase/manager.rs` |
| **PhaseSource** | Source selection enum (wire type) | `foc/phase/source.rs` |
| **Observer / BackEmfObserver** | Back-EMF (fast) estimator slot | `foc/phase/observer.rs` |
| **HfiObserver** | HFI (low-speed) estimator slot | `foc/phase/observer.rs` |
| **AngleSensor** | Base sensor trait | `foc/sensors.rs` |
| **HallSensorTrait** | Hall-specific interface | `foc/sensors.rs` |
| **EncoderSensorTrait** | Encoder-specific interface | `foc/sensors.rs` |
| **CurrentSensor** | Current measurement trait | `foc/sensors.rs` |
| **DriverCommand / CMD_CHANNEL** | Ordered command path into the ISR | `state.rs` |
| **run_foc_cycle / publish_cycle_telemetry** | Shared per-cycle ISR work | `state.rs` / `runtime/streaming.rs` |
| **config_server / run_storage_worker** | Persistent config pipeline | `runtime/servers.rs` / `storage.rs` |
| **DetectionHardware** | Detection abstraction | `foc/detection/sweep.rs` |
| **Timer** | Async delay abstraction | `timer.rs` |

### Key Design Decisions

1. **PhaseManager owns sensors and both estimator slots** - Single place for all angle source management
2. **FocDriver delegates phase to PhaseProvider** - Clean separation; HFI carrier flows through the same trait (`injection()`)
3. **Dual estimator slots** - Back-EMF observer and HFI run concurrently so crossovers can hand over with hysteresis and reseeding
4. **Traits extend AngleSensor** - Common base for PhaseManager, specific traits for calibration
5. **Enums for runtime flexibility** - PhaseSource, Observer, ControlMode are runtime-switchable
6. **Single ordered command channel** - All async→ISR mutation goes through CMD_CHANNEL with boundary validation (`is_sane()`)
7. **Generics for hardware** - Zero-cost abstraction for platform-specific implementations (incl. hardware trig via `SinCos`)
8. **Conditional methods** - Hall/Encoder-specific APIs only appear when relevant
9. **Async detection with traits** - DetectionHardware + Timer allow platform-agnostic detection code

### Implementation Status

| Feature | Status |
|---------|--------|
| Current control (FOC) | ✅ Implemented |
| Open-loop control (lock + spin) | ✅ Implemented |
| Direct voltage mode | ✅ Implemented |
| Coast mode | ✅ Implemented |
| Six-step (trapezoidal) mode | ✅ Implemented |
| Measured-overcurrent protection (all energizing modes) | ✅ Implemented |
| Hall sensor support | ✅ Implemented (VESC-compatible) |
| Hall 8-entry raw calibration | ✅ Implemented |
| Hall soft drift correction | ✅ Implemented |
| Hall rate limiting | ✅ Implemented |
| Hall eRPM threshold | ✅ Implemented |
| Hall health tracking | ✅ Implemented |
| Hall → Observer fallback | ✅ Implemented |
| Hall calibration | ✅ Implemented |
| Resistance detection | ✅ Implemented |
| Inductance detection (HFI + pulse) | ✅ Implemented |
| Flux linkage detection | ✅ Implemented |
| Back-EMF observer (MXLEMMING) | ✅ Implemented (closed-loop sim validated) |
| HFI angle tracking (pulsating injection + polarity probe) | ✅ Implemented (closed-loop sim validated) |
| HFI ↔ sensor/observer crossovers | ✅ Implemented |
| Persistent config storage | ✅ Implemented |
| Runtime phase source switching (host endpoint) | ✅ Implemented |
| Velocity control | 📋 Planned |
| Position control | 📋 Planned |
| Outer loop controllers | 📋 Planned |
| Encoder support | 📋 Trait defined, hardware impl planned |
| Field weakening | 📋 Planned |
| MTPA (Max Torque Per Amp) | 📋 Planned |
