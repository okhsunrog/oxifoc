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
│                            User Application                              │
│  • Set control mode (current, velocity, position)                       │
│  • Configure phase source (Hall, Encoder, Observer, Hybrid)             │
│  • Run calibration procedures                                            │
└─────────────────────────────────┬───────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                             FocDriver<P, C, Phase>                       │
│  • Orchestrates control loop execution                                   │
│  • Manages control modes (Stopped, Current, OpenLoop, HfiInjection)     │
│  • Delegates phase estimation to PhaseProvider                          │
└───────────┬─────────────────────┬─────────────────────┬─────────────────┘
            │                     │                     │
            ▼                     ▼                     ▼
    ┌───────────────┐    ┌───────────────┐    ┌───────────────────────┐
    │   PhasePwm    │    │ CurrentSensor │    │    PhaseProvider      │
    │   (trait)     │    │   (trait)     │    │      (trait)          │
    │               │    │               │    │                       │
    │ • set_duties  │    │ • read_currents│   │ • get() → PhaseOutput │
    │ • max_duty    │    │ • is_calibrated│   │ • update(PhaseInput)  │
    │ • disable     │    │ • get_offsets │    │                       │
    └───────────────┘    └───────────────┘    └───────────┬───────────┘
                                                          │
                                              implements  │
                                                          ▼
                              ┌────────────────────────────────────────────┐
                              │         PhaseManager<H, E>                  │
                              │  • Manages Hall sensor (H)                  │
                              │  • Manages Encoder (E)                      │
                              │  • Manages Observer (sensorless)            │
                              │  • Handles source selection & blending      │
                              └────────────────────────────────────────────┘
```

---

## Core Components

### 1. FocDriver

The main motor driver that orchestrates FOC control.

```rust
pub struct FocDriver<P, C, Phase>
where
    P: PhasePwm,
    C: CurrentSensor,
    Phase: PhaseProvider,
{
    // FOC controller
    controller: FocController,
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
}
```

**Responsibilities:**
- Execute the FOC current control loop
- Handle control mode transitions
- Coordinate with PhaseProvider for angle estimation

**Key simplification:** FocDriver has only 3 type parameters. All phase/angle complexity is delegated to `PhaseProvider`.

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
#[derive(Clone, Copy, Debug, Default, PartialEq)]
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
}
```

**Design rationale:**
- `get()` is called first to obtain the angle for Park/Clarke transforms
- `update()` is called last with the commanded voltages (needed by observers)
- This creates a one-sample delay for observers, which is standard practice

### 3. PhaseManager

Concrete implementation of `PhaseProvider` that manages multiple angle sources.

```rust
pub struct PhaseManager<H = NoSensor, E = NoSensor>
where
    H: AngleSensor,
    E: AngleSensor,
{
    // Hardware sensors
    hall: H,
    encoder: E,

    // Software estimator
    observer: Observer,

    // Configuration
    source: PhaseSource,

    // State
    output: PhaseOutput,
    manual_angle: f32,
    open_loop_angle: f32,
    open_loop_velocity: f32,

    // Timebase
    ticks_per_sec: u64,
}
```

**Responsibilities:**
- Sample hardware sensors (Hall, Encoder)
- Update software observer
- Select/blend angle sources based on `PhaseSource`
- Provide unified phase output to FocDriver

---

## Phase Source Selection

### PhaseSource Enum

Specifies where electrical angle comes from:

```rust
pub enum PhaseSource {
    // === Direct hardware sensor ===
    Hall,                    // Use Hall sensor
    Encoder,                 // Use encoder

    // === Software estimation ===
    Observer,                // Back-EMF observer (sensorless)
    Hfi,                     // High-frequency injection

    // === Hybrid modes ===
    HallToObserver {         // Hall at low speed, observer at high speed
        blend_low: f32,      // Start blending (electrical rad/s)
        blend_high: f32,     // Full observer (electrical rad/s)
    },
    EncoderToObserver {      // Encoder at low speed, observer at high speed
        blend_low: f32,
        blend_high: f32,
    },
    HfiToObserver {          // HFI startup, transition to observer
        min_vel: f32,
        min_confidence: f32,
    },
    HfiToHall {              // HFI startup, transition to Hall
        switch_vel: f32,
    },
    HfiToEncoder {           // HFI startup, transition to encoder
        switch_vel: f32,
    },

    // === Manual control ===
    Manual,                  // Use manually set angle (calibration)
    OpenLoop,                // Open-loop angle ramp (startup, calibration)
}
```

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
        self.source = source;
        Ok(())
    }
}
```

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
    /// Sample angle at given timestamp
    fn sample(&self, now_ticks: u64) -> Option<AngleSample>;

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
    fn set_calibration(&mut self, table: [f32; 6]);

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
   Platform impls        Platform impls
   (F405HallSensor)      (F405Encoder)
```

---

## Observer Integration

### Observer Enum

Runtime-switchable observer implementations:

```rust
pub enum Observer {
    None,
    BackEmf(BackEmfObserver),
    Hfi(HfiObserver),
}

impl Observer {
    pub fn update(&mut self, input: &ObserverInput);
    pub fn phase(&self) -> Option<f32>;
    pub fn velocity(&self) -> Option<f32>;
    pub fn confidence(&self) -> f32;
    pub fn is_configured(&self) -> bool;
}

pub struct ObserverInput {
    pub v_alpha: f32,
    pub v_beta: f32,
    pub i_alpha: f32,
    pub i_beta: f32,
    pub dt: f32,
}
```

### Back-EMF Observer

VESC-style flux observer for sensorless operation:

```rust
pub struct BackEmfObserver {
    // Observer state
    x1: f32,              // Flux linkage α component
    x2: f32,              // Flux linkage β component
    phase_pll: f32,       // PLL-filtered phase
    velocity_pll: f32,    // PLL-filtered velocity

    // Motor parameters
    r: f32,               // Phase resistance (Ω)
    l: f32,               // Phase inductance (H)
    lambda: f32,          // Flux linkage (Wb)

    // Tuning
    gain: f32,            // Observer gain
    pll_kp: f32,          // PLL proportional gain
    pll_ki: f32,          // PLL integral gain
}
```

---

## Control Modes

### ControlMode Enum

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

    /// Open-loop mode for calibration - locks rotor to specified electrical angle
    OpenLoop {
        /// Target electrical angle (radians, 0 to 2π)
        angle_rad: f32,
        /// Current magnitude (Amps) - applied as q-current to lock rotor
        current: f32,
    },

    /// HFI injection mode for inductance measurement
    HfiInjection {
        /// DC current to hold rotor in place (Amps)
        hold_current: f32,
        /// d-axis voltage to inject (V)
        vd_inject: f32,
        /// q-axis voltage to inject (V)
        vq_inject: f32,
    },
}
```

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

/// All control loops (PLANNED)
pub struct ControlLoops {
    /// Current controller (always present)
    pub current: FocController,

    /// Velocity outer loop (optional)
    pub velocity: OuterLoopType,

    /// Position outer loop (optional)
    pub position: OuterLoopType,
}
```

When implemented, FocDriver will contain `ControlLoops` and velocity/position modes will cascade through the outer loops to generate current targets.

---

## Motor Parameter Detection

The detection module provides async functions for measuring motor parameters. These use a `DetectionHardware` trait for platform abstraction.

### DetectionHardware Trait

```rust
/// Hardware abstraction for motor detection routines.
pub trait DetectionHardware {
    /// Send a control mode command to the FOC loop.
    fn send_command(&self, mode: ControlMode);

    /// Wait for next FOC telemetry (PWM-synchronized).
    async fn wait_telemetry(&mut self) -> FocTelemetry;

    /// Read raw phase currents (ia, ib, ic) in Amps.
    fn read_phase_currents(&self) -> (f32, f32, f32);
}
```

### Timer Trait

Platform-agnostic async timer for detection delays:

```rust
/// Platform-agnostic timer trait for async delays.
pub trait Timer {
    /// Delay for the specified number of milliseconds.
    async fn after_millis(ms: u64);

    /// Delay for the specified number of microseconds.
    async fn after_micros(us: u64);
}
```

### Detection Functions

```rust
/// Measure motor phase resistance
pub async fn measure_resistance<H, T>(hw: &mut H, params: &ResistanceParams)
    -> Result<f32, DetectionError>
where
    H: DetectionHardware,
    T: Timer;

/// Measure motor inductance using rotating HFI
pub async fn measure_inductance<H, T>(hw: &mut H, params: &InductanceParams, pwm_freq_hz: f32)
    -> Result<(f32, f32), DetectionError>  // (Ld, Lq)
where
    H: DetectionHardware,
    T: Timer;

/// Measure motor flux linkage via open-loop spinning
pub async fn measure_flux_linkage<H, T>(hw: &mut H, params: &FluxLinkageParams)
    -> Result<f32, DetectionError>
where
    H: DetectionHardware,
    T: Timer;

/// Run full motor parameter detection sequence
pub async fn run_full_detection<H, T>(hw: &mut H, params: DetectionParams)
    -> Result<DetectionResult, DetectionError>
where
    H: DetectionHardware,
    T: Timer;

/// Calibrate Hall sensors
pub async fn calibrate_hall<H, T, R>(hw: &mut H, reader: &R, params: HallCalibrationParams)
    -> Result<HallCalibrationResult, DetectionError>
where
    H: DetectionHardware,
    T: Timer,
    R: HallReader;
```

---

## Control Flow

### Main Control Step

```rust
impl<P: PhasePwm, C: CurrentSensor, Phase: PhaseProvider> FocDriver<P, C, Phase> {
    pub fn step(&mut self, dt: f32, now_ticks: u64) -> Result<FocTelemetry, &'static str> {
        match self.mode {
            ControlMode::Stopped => {
                self.pwm.disable();
                self.phase.update(&PhaseInput { dt, ..Default::default() }, now_ticks);
                Ok(FocTelemetry::default())
            }
            ControlMode::CurrentControl { iq_target, id_target } => {
                self.step_current_control(iq_target, id_target, dt, now_ticks)
            }
            ControlMode::OpenLoop { angle_rad, current } => {
                self.step_open_loop(angle_rad, current, dt, now_ticks)
            }
            ControlMode::HfiInjection { hold_current, vd_inject, vq_inject } => {
                self.step_hfi_injection(hold_current, vd_inject, vq_inject, dt, now_ticks)
            }
            // Velocity/Position control not yet implemented
            _ => Err("Control mode not implemented")
        }
    }

    fn step_current_control(&mut self, iq_target: f32, id_target: f32, dt: f32, now_ticks: u64)
        -> Result<FocTelemetry, &'static str>
    {
        // 1. Get phase estimate (from previous update)
        let phase_out = self.phase.get();
        let angle = phase_out.angle;

        // 2. Read phase currents
        let currents = self.current_sensor.read_currents();
        let (i_alpha, i_beta) = transforms::clarke(currents.0, currents.1);

        // 3. Run FOC controller
        let max_duty = self.pwm.max_duty();
        let telem = self.controller.step(currents, angle, id_target, iq_target, max_duty, dt);

        // 4. Set PWM duties
        self.pwm.set_duties(telem.duties);

        // 5. Update phase provider for next iteration
        self.phase.update(&PhaseInput {
            v_alpha: telem.v_alpha,
            v_beta: telem.v_beta,
            i_alpha,
            i_beta,
            dt,
        }, now_ticks);

        Ok(telem)
    }
}
```

### Phase Selection in PhaseManager

```rust
impl<H: AngleSensor, E: AngleSensor> PhaseManager<H, E> {
    fn compute_phase(&self, hall: Option<AngleSample>, enc: Option<AngleSample>) -> PhaseOutput {
        match self.source {
            PhaseSource::Hall => sample_to_output(hall, &self.output),
            PhaseSource::Encoder => sample_to_output(enc, &self.output),
            PhaseSource::Observer => {
                if let (Some(angle), Some(vel)) = (self.observer.phase(), self.observer.velocity()) {
                    PhaseOutput { angle, velocity: vel }
                } else {
                    self.output
                }
            }
            PhaseSource::HallToObserver { blend_low, blend_high } => {
                let sensor = sample_to_output(hall, &self.output);
                self.blend_with_observer(sensor, blend_low, blend_high)
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

// Sensorless only
let mut phase = PhaseManager::sensorless();
phase.set_observer(Observer::BackEmf(BackEmfObserver::new(r, l, lambda)));
```

### Constructing FocDriver

```rust
let driver = FocDriver::new(controller, pwm, current_sensor, phase);
```

### Full Example (F405)

```rust
// Initialize hardware
let pwm = MotorPwm::new(tim1, pa8, pa9, pa10, pb13, pb14, pb15);
let current = F405CurrentSensor::new();
let hall = F405HallSensor::new();

// Create phase manager with Hall + sensorless hybrid
let mut phase = PhaseManager::with_hall(hall);
phase.set_observer(Observer::BackEmf(BackEmfObserver::new(
    0.1,    // R = 100mΩ
    0.0001, // L = 100µH
    0.01,   // λ = 10mWb
)));
phase.set_source(PhaseSource::HallToObserver {
    blend_low: 300.0,   // Start blending at 300 rad/s
    blend_high: 600.0,  // Full sensorless at 600 rad/s
})?;

// Create controller and driver
let controller = FocController::new(vbus);
let mut driver = FocDriver::new(controller, pwm, current, phase);

// Start motor
driver.set_mode(ControlMode::CurrentControl { iq_target: 5.0, id_target: 0.0 });

// In ISR
loop {
    let telem = driver.step(DT, now_ticks)?;
    // ... publish telemetry
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
    pub fn set_manual_angle(&mut self, angle: f32);
    pub fn has_hall(&self) -> bool;
    pub fn has_encoder(&self) -> bool;
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
├── timer.rs                 # Timer trait for async delays
├── fmt.rs                   # Logging macros (defmt/log/none)
├── foc/
│   ├── mod.rs
│   ├── config.rs            # Board configuration
│   ├── constants.rs         # Math constants (√3, 1/√3, etc.)
│   ├── controller.rs        # FocController (current loop)
│   ├── current_sense.rs     # ShuntCurrentSense helper
│   ├── fault.rs             # Fault registry
│   ├── hall_calibration.rs  # HallCalibrator, HallCalibrationResult
│   ├── hall_sensor.rs       # HallSensor struct
│   ├── pi_controller.rs     # PI controller with anti-windup
│   ├── pwm.rs               # PhasePwm trait
│   ├── sensors.rs           # AngleSensor, CurrentSensor, HallSensorTrait, etc.
│   ├── svpwm.rs             # Space Vector PWM modulator
│   ├── transforms.rs        # Clarke, Park transforms
│   ├── phase/
│   │   ├── mod.rs
│   │   ├── provider.rs      # PhaseProvider trait
│   │   ├── manager.rs       # PhaseManager struct
│   │   ├── source.rs        # PhaseSource enum
│   │   └── observer.rs      # Observer enum, BackEmfObserver, HfiObserver
│   └── detection/
│       ├── mod.rs
│       ├── types.rs         # MotorParams, DetectionError, etc.
│       ├── sweep.rs         # Async detection (DetectionHardware, Timer traits)
│       ├── resistance.rs    # Resistance measurement algorithms
│       ├── inductance.rs    # Inductance measurement (rotating HFI)
│       ├── flux_linkage.rs  # Flux linkage measurement
│       ├── dc_offset.rs     # DC offset calibration
│       └── pi_tuning.rs     # Auto PI tuning
└── motor/
    ├── mod.rs
    └── foc_driver.rs        # FocDriver struct, ControlMode

oxifoc-f405/src/             # STM32F405 platform
├── main.rs
├── config.rs                # BoardConfig for F405
├── calibration.rs           # EmbassyTimer, F405DetectionHardware
├── hardware/                # Hardware initialization
├── sensors/                 # F405HallSensor, current sensing
├── control/
│   └── foc.rs               # ISR, FOC task
├── motor/
├── protocol/
└── transport/

oxifoc-g431/src/             # STM32G431 platform
├── main.rs
├── config.rs                # BoardConfig for G431
├── calibration.rs           # EmbassyTimer, G431DetectionHardware
├── hardware/
├── sensors/
├── control/
│   └── foc.rs
├── motor/
├── protocol/
└── transport/
```

---

## Summary

| Component | Responsibility | Location |
|-----------|---------------|----------|
| **FocDriver** | Control loop orchestration | `motor/foc_driver.rs` |
| **FocController** | Current loop math | `foc/controller.rs` |
| **PhaseProvider** | Phase angle abstraction | `foc/phase/provider.rs` |
| **PhaseManager** | Sensor/observer management | `foc/phase/manager.rs` |
| **PhaseSource** | Source selection enum | `foc/phase/source.rs` |
| **Observer** | Sensorless estimation | `foc/phase/observer.rs` |
| **AngleSensor** | Base sensor trait | `foc/sensors.rs` |
| **HallSensorTrait** | Hall-specific interface | `foc/sensors.rs` |
| **EncoderSensorTrait** | Encoder-specific interface | `foc/sensors.rs` |
| **CurrentSensor** | Current measurement trait | `foc/sensors.rs` |
| **DetectionHardware** | Detection abstraction | `foc/detection/sweep.rs` |
| **Timer** | Async delay abstraction | `timer.rs` |

### Key Design Decisions

1. **PhaseManager owns sensors and observer** - Single place for all angle source management
2. **FocDriver delegates phase to PhaseProvider** - Clean separation, only 3 type parameters
3. **Traits extend AngleSensor** - Common base for PhaseManager, specific traits for calibration
4. **Enums for runtime flexibility** - PhaseSource, Observer are runtime-switchable
5. **Generics for hardware** - Zero-cost abstraction for platform-specific implementations
6. **Conditional methods** - Hall/Encoder-specific APIs only appear when relevant
7. **Async detection with traits** - DetectionHardware + Timer allow platform-agnostic detection code

### Implementation Status

| Feature | Status |
|---------|--------|
| Current control (FOC) | Implemented |
| Open-loop control | Implemented |
| HFI injection mode | Implemented |
| Hall sensor support | Implemented |
| Hall calibration | Implemented |
| Resistance detection | Implemented |
| Inductance detection (HFI) | Implemented |
| Flux linkage detection | Implemented |
| Back-EMF observer | Implemented (untested) |
| Velocity control | Planned |
| Position control | Planned |
| Outer loop controllers | Planned |
| Encoder support | Trait defined, hardware impl planned |
| HFI angle tracking | Planned |
| Field weakening | Planned |
| MTPA (Max Torque Per Amp) | Planned |
