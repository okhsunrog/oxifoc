# FOC Implementation Plan

**Branch:** `feature/foc-implementation`
**Target Hardware:** B-G431B-ESC1 (STM32G431CB)
**Test Motor:** Flipsky 5065 270KV with Hall sensors
**Created:** 2025-01-27

---

## Project Overview

Implement Field-Oriented Control (FOC) for BLDC motors with:
- Hall sensor position feedback
- Current control loop (Id/Iq)
- Velocity control loop
- Automatic motor parameter detection
- Real-time telemetry via ergot

---

## Current Status ✅

### Completed Infrastructure
- ✅ Embassy async runtime on STM32G431 @ 170MHz
- ✅ 3-phase current sensing (OPAMP + ADC injected conversions)
- ✅ VBUS and temperature monitoring
- ✅ TIM1 PWM generation (complementary outputs with deadtime)
- ✅ Hall sensor pins initialized (PB6, PB7, PB8)
- ✅ ergot communication (Serial/RTT)
- ✅ Real-time ADC sample streaming to host
- ✅ Basic 6-step commutation (proof of concept)

### Hardware Verification Needed
- ⚠️ Current sensor shunt resistance (assumed 0.5mΩ - VERIFY!)
- ⚠️ OPAMP offset calibration (not yet implemented)
- ⚠️ Hall sensor pull-up configuration (assumed internal pull-ups OK)

---

## Implementation Phases

## **PHASE 1: Foundation (Week 1)**

### Milestone: Basic FOC Math Working

#### 1.1 Create Core FOC Math Modules
**Priority: HIGH** | **Effort: 4 hours** | **Risk: LOW**

**Tasks:**
- [ ] Create `oxifoc-device/src/motor/transforms.rs`
  - [ ] Implement Clarke transform (ABC → αβ)
  - [ ] Implement Park transform (αβ → dq)
  - [ ] Implement inverse Park (dq → αβ)
  - [ ] Implement inverse Clarke (αβ → ABC)
  - [ ] Add unit tests for round-trip accuracy
  - [ ] Use f32 (not fixed-point)

**Code template:**
```rust
// motor/transforms.rs
use core::f32::consts::SQRT_3;

pub fn clarke(ia: f32, ib: f32) -> (f32, f32) {
    let alpha = ia;
    let beta = (ia + 2.0 * ib) / SQRT_3;
    (alpha, beta)
}

pub fn park(alpha: f32, beta: f32, sin_theta: f32, cos_theta: f32) -> (f32, f32) {
    let d = cos_theta * alpha + sin_theta * beta;
    let q = cos_theta * beta - sin_theta * alpha;
    (d, q)
}

// ... inverse transforms
```

**Test cases:**
```rust
#[test]
fn test_clarke_roundtrip() {
    let (alpha, beta) = clarke(1.0, 0.5);
    let (a, b, c) = inverse_clarke(alpha, beta);
    assert!((a - 1.0).abs() < 0.001);
    assert!((b - 0.5).abs() < 0.001);
}
```

**Dependencies:**
```toml
# Add to Cargo.toml if needed
libm = { version = "0.2.15", default-features = false }  # Already have this
```

**Files to create:**
- `oxifoc-device/src/motor/transforms.rs` (~60 lines)

**Success criteria:**
- All transforms pass round-trip tests
- No allocation (verify with `cargo bloat`)

---

#### 1.2 Implement SVPWM
**Priority: HIGH** | **Effort: 3 hours** | **Risk: LOW**

**Tasks:**
- [ ] Create `oxifoc-device/src/motor/svpwm.rs`
  - [ ] Implement sector detection (6 sectors)
  - [ ] Calculate ta/tb/tc from α/β voltages
  - [ ] Convert to duty cycles (0 to max_duty)
  - [ ] Add saturation/clamping logic
  - [ ] Add visualization helper (optional, for debugging)

**Code template:**
```rust
// motor/svpwm.rs
pub fn space_vector_pwm(alpha: f32, beta: f32, max_duty: u16) -> [u16; 3] {
    use core::f32::consts::SQRT_3;

    // Convert to x/y/z
    let x = beta;
    let y = (beta + SQRT_3 * alpha) / 2.0;
    let z = (beta - SQRT_3 * alpha) / 2.0;

    // Sector detection
    let sector = match (x > 0.0, y > 0.0, z > 0.0) {
        (true, true, false) => 1,
        (_, true, true) => 2,
        (true, false, true) => 3,
        (false, false, true) => 4,
        (_, false, false) => 5,
        (false, true, false) => 6,
    };

    // Time calculations per sector
    let (ta, tb, tc) = match sector {
        1 | 4 => (x - z, x + z, -x + z),
        2 | 5 => (y - z, y + z, -y - z),
        3 | 6 => (y - x, -y + x, -y - x),
        _ => unreachable!(),
    };

    // Convert to duty cycles
    [
        to_duty(ta, max_duty),
        to_duty(tb, max_duty),
        to_duty(tc, max_duty),
    ]
}

fn to_duty(value: f32, max_duty: u16) -> u16 {
    let duty = ((value + 1.0) * (max_duty as f32 + 1.0) / 2.0) as i32;
    duty.clamp(0, max_duty as i32) as u16
}
```

**Test cases:**
```rust
#[test]
fn test_svpwm_sectors() {
    // Test all 6 sectors
    for angle_deg in [30, 90, 150, 210, 270, 330] {
        let angle_rad = (angle_deg as f32).to_radians();
        let alpha = angle_rad.cos();
        let beta = angle_rad.sin();
        let duties = space_vector_pwm(alpha, beta, 1000);
        // Verify duties are in valid range
        assert!(duties[0] <= 1000);
        assert!(duties[1] <= 1000);
        assert!(duties[2] <= 1000);
    }
}
```

**Files to create:**
- `oxifoc-device/src/motor/svpwm.rs` (~70 lines)

**Success criteria:**
- SVPWM produces valid duty cycles for all sectors
- No discontinuities at sector boundaries
- Duty cycles sum correctly

---

#### 1.3 Implement PI Controller
**Priority: HIGH** | **Effort: 2 hours** | **Risk: LOW**

**Tasks:**
- [ ] Create `oxifoc-device/src/motor/pi_controller.rs`
  - [ ] Basic PI structure with kp/ki gains
  - [ ] Integral accumulation with dt
  - [ ] Anti-windup (back-calculation method)
  - [ ] Output limits (min/max)
  - [ ] Reset function

**Code template:**
```rust
// motor/pi_controller.rs
pub struct PIController {
    kp: f32,
    ki: f32,
    integral: f32,
    output_min: f32,
    output_max: f32,
}

impl PIController {
    pub fn new(kp: f32, ki: f32) -> Self {
        Self {
            kp,
            ki,
            integral: 0.0,
            output_min: f32::NEG_INFINITY,
            output_max: f32::INFINITY,
        }
    }

    pub fn with_limits(mut self, min: f32, max: f32) -> Self {
        self.output_min = min;
        self.output_max = max;
        self
    }

    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;

        // P term
        let p_term = self.kp * error;

        // I term
        self.integral += self.ki * error * dt;

        // Output
        let output = p_term + self.integral;
        let clamped = output.clamp(self.output_min, self.output_max);

        // Anti-windup
        if output != clamped {
            self.integral = clamped - p_term;
        }

        clamped
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
    }
}
```

**Test cases:**
```rust
#[test]
fn test_pi_no_overshoot() {
    let mut pi = PIController::new(1.0, 0.1).with_limits(-10.0, 10.0);

    for _ in 0..100 {
        let output = pi.update(5.0, 0.0, 0.01); // Setpoint=5, measurement=0
        assert!(output >= -10.0 && output <= 10.0);
    }
}
```

**Files to create:**
- `oxifoc-device/src/motor/pi_controller.rs` (~60 lines)

**Success criteria:**
- PI controller converges to setpoint
- Anti-windup prevents integral overflow
- Output stays within limits

---

#### 1.4 Update Module Structure
**Priority: HIGH** | **Effort: 1 hour** | **Risk: LOW**

**Tasks:**
- [ ] Update `oxifoc-device/src/motor/mod.rs`
  - [ ] Add `pub mod transforms;`
  - [ ] Add `pub mod svpwm;`
  - [ ] Add `pub mod pi_controller;`
  - [ ] Export commonly used types

**Code:**
```rust
// motor/mod.rs
pub mod pwm;
pub mod six_step;
pub mod transforms;     // NEW
pub mod svpwm;          // NEW
pub mod pi_controller;  // NEW

// Re-exports for convenience
pub use transforms::{clarke, park, inverse_park, inverse_clarke};
pub use svpwm::space_vector_pwm;
pub use pi_controller::PIController;
```

**Files to modify:**
- `oxifoc-device/src/motor/mod.rs`

**Success criteria:**
- Project compiles without errors
- No unused code warnings

---

**PHASE 1 DELIVERABLE:**
- ✅ FOC math library complete and tested
- ✅ All transforms verified with unit tests
- ✅ SVPWM working for all sectors
- ✅ PI controller with anti-windup
- ✅ Ready to integrate with hardware

**Estimated Time:** 10 hours
**Risk Level:** LOW (pure algorithms, no hardware dependency)

---

## **PHASE 2: Hall Sensor Integration (Week 1-2)**

### Milestone: Hall Sensors Reading Motor Position

#### 2.1 Create Hall Sensor Driver
**Priority: HIGH** | **Effort: 4 hours** | **Risk: MEDIUM**

**Tasks:**
- [ ] Create `oxifoc-device/src/motor/hall.rs`
  - [ ] Implement `HallSensors` struct
  - [ ] `read_state()` → u8 (0-7, 3-bit value)
  - [ ] State validation (only 1,2,3,4,5,6 are valid)
  - [ ] Error counter for invalid states
  - [ ] Direction detection (CW vs CCW)
  - [ ] Electrical angle estimation (using lookup table)

**Code template:**
```rust
// motor/hall.rs
use embassy_stm32::gpio::Input;

pub struct HallSensors {
    h1: Input<'static>,
    h2: Input<'static>,
    h3: Input<'static>,
    error_count: u32,
}

impl HallSensors {
    pub fn new(h1: Input<'static>, h2: Input<'static>, h3: Input<'static>) -> Self {
        Self { h1, h2, h3, error_count: 0 }
    }

    /// Read Hall sensor state (0-7)
    pub fn read_state(&self) -> u8 {
        let bit_h1 = if self.h1.is_high() { 1 } else { 0 };
        let bit_h2 = if self.h2.is_high() { 1 } else { 0 };
        let bit_h3 = if self.h3.is_high() { 1 } else { 0 };

        (bit_h3 << 2) | (bit_h2 << 1) | bit_h1
    }

    /// Check if state is valid (1-6, not 0 or 7)
    pub fn is_valid_state(&mut self) -> bool {
        let state = self.read_state();
        let valid = state > 0 && state < 7;
        if !valid {
            self.error_count += 1;
        }
        valid
    }

    /// Get electrical angle from Hall state (using calibration table)
    pub fn get_angle_deg(&self, hall_table: &[f32; 8]) -> f32 {
        let state = self.read_state() as usize;
        hall_table[state]
    }
}

// Default Hall table for typical motor (before calibration)
pub const DEFAULT_HALL_TABLE: [f32; 8] = [
    0.0,   // State 0 (invalid)
    0.0,   // State 1
    120.0, // State 2
    60.0,  // State 3
    240.0, // State 4
    300.0, // State 5
    180.0, // State 6
    0.0,   // State 7 (invalid)
];
```

**Files to create:**
- `oxifoc-device/src/motor/hall.rs` (~100 lines)

**Hardware test procedure:**
```rust
// In main.rs, add test task
#[embassy_executor::task]
async fn test_hall_sensors(hall: HallSensors) {
    loop {
        let state = hall.read_state();
        defmt::info!("Hall state: {}", state);
        Timer::after(Duration::from_millis(100)).await;
    }
}
```

**Success criteria:**
- Hall state changes as motor is manually rotated
- Only states 1-6 appear (0 and 7 indicate wiring issue)
- States follow sequence: 5→1→3→2→6→4→5 (or reversed)

---

#### 2.2 Integrate Hall Sensors in Main
**Priority: HIGH** | **Effort: 2 hours** | **Risk: LOW**

**Tasks:**
- [ ] Modify `oxifoc-device/src/main.rs`
  - [ ] Create `HallSensors` instance from pins
  - [ ] Pass to motor control task
  - [ ] Add Hall state to telemetry

**Code changes:**
```rust
// In main.rs after line 416
use motor::hall::HallSensors;

let hall_sensors = HallSensors::new(hall_h1, hall_h2, hall_h3);
defmt::info!("Hall sensors initialized");

// Pass to motor task
spawner.spawn(motor_control_task(motor_ctrl, hall_sensors, motor_cmd_receiver).unwrap());
```

**Files to modify:**
- `oxifoc-device/src/main.rs`
- `oxifoc-device/src/motor/mod.rs`

**Success criteria:**
- Hall sensors readable in motor task
- No compilation errors

---

#### 2.3 Add Hall State to Protocol
**Priority: MEDIUM** | **Effort: 1 hour** | **Risk: LOW**

**Tasks:**
- [ ] Modify `protocol/src/lib.rs`
  - [ ] Add `hall_state: u8` to `AdcSample` or new `MotorTelemetry` struct
  - [ ] Add Hall error count

**Code:**
```rust
// protocol/src/lib.rs
#[derive(Clone, Schema, Serialize, Deserialize, Debug)]
pub struct MotorTelemetry {
    pub hall_state: u8,
    pub hall_errors: u32,
    pub electrical_angle_deg: f32,
    pub id_amps: f32,
    pub iq_amps: f32,
    pub vd: f32,
    pub vq: f32,
}

endpoint!(MotorTelemetryEndpoint, (), MotorTelemetry, "req/motor_telemetry");
```

**Files to modify:**
- `protocol/src/lib.rs`

**Success criteria:**
- Host can read Hall state in real-time
- GUI shows Hall state changes

---

**PHASE 2 DELIVERABLE:**
- ✅ Hall sensors working and validated
- ✅ Real-time Hall state visible in host GUI
- ✅ Error detection for faulty Hall sensors
- ✅ Ready for calibration

**Estimated Time:** 7 hours
**Risk Level:** MEDIUM (hardware-dependent, wiring issues possible)

---

## **PHASE 3: Current Sensing & Calibration (Week 2)**

### Milestone: Accurate Current Measurements

#### 3.1 Add Current Offset Calibration
**Priority: CRITICAL** | **Effort: 3 hours** | **Risk: MEDIUM**

**Tasks:**
- [ ] Create `oxifoc-device/src/motor/calibration.rs`
  - [ ] Implement `calibrate_current_offsets()` async function
  - [ ] Sample 500+ ADC readings with motor OFF
  - [ ] Calculate average offset for each phase
  - [ ] Store in global atomics

**Code template:**
```rust
// motor/calibration.rs
use embassy_time::{Duration, Timer};
use core::sync::atomic::{AtomicU16, Ordering};

static IA_OFFSET: AtomicU16 = AtomicU16::new(2048);
static IB_OFFSET: AtomicU16 = AtomicU16::new(2048);
static IC_OFFSET: AtomicU16 = AtomicU16::new(2048);

pub async fn calibrate_current_offsets() -> Result<(), CalibrationError> {
    const SAMPLES: usize = 500;

    defmt::info!("Starting current offset calibration...");

    let mut ia_sum = 0_u32;
    let mut ib_sum = 0_u32;
    let mut ic_sum = 0_u32;

    // Ensure motor is OFF
    // (PWM should already be disabled at startup)

    for i in 0..SAMPLES {
        ia_sum += crate::IA_SAMPLE.load(Ordering::Relaxed) as u32;
        ib_sum += crate::IB_SAMPLE.load(Ordering::Relaxed) as u32;
        ic_sum += crate::IC_SAMPLE.load(Ordering::Relaxed) as u32;

        Timer::after(Duration::from_millis(1)).await;

        if i % 100 == 0 {
            defmt::debug!("Calibration progress: {}/{}", i, SAMPLES);
        }
    }

    let ia_offset = (ia_sum / SAMPLES) as u16;
    let ib_offset = (ib_sum / SAMPLES) as u16;
    let ic_offset = (ic_sum / SAMPLES) as u16;

    IA_OFFSET.store(ia_offset, Ordering::Relaxed);
    IB_OFFSET.store(ib_offset, Ordering::Relaxed);
    IC_OFFSET.store(ic_offset, Ordering::Relaxed);

    defmt::info!("Current offsets: ia={}, ib={}, ic={}",
        ia_offset, ib_offset, ic_offset);

    // Sanity check: offsets should be near 2048 (mid-range)
    if (ia_offset as i32 - 2048).abs() > 200 ||
       (ib_offset as i32 - 2048).abs() > 200 ||
       (ic_offset as i32 - 2048).abs() > 200 {
        defmt::error!("Current offsets out of range!");
        return Err(CalibrationError::OffsetOutOfRange);
    }

    Ok(())
}

pub fn get_current_offsets() -> (u16, u16, u16) {
    (
        IA_OFFSET.load(Ordering::Relaxed),
        IB_OFFSET.load(Ordering::Relaxed),
        IC_OFFSET.load(Ordering::Relaxed),
    )
}

#[derive(Debug)]
pub enum CalibrationError {
    OffsetOutOfRange,
    Timeout,
}
```

**Files to create:**
- `oxifoc-device/src/motor/calibration.rs` (~100 lines)

**Integration:**
```rust
// In main.rs, before motor control task
if let Err(e) = motor::calibration::calibrate_current_offsets().await {
    defmt::error!("Current calibration failed: {:?}", e);
    // Enter safe mode or retry
}
```

**Success criteria:**
- Offsets measured near 2048 (±200 counts)
- Consistent across multiple runs
- Phase currents read near 0A with motor off

---

#### 3.2 Improve Current Conversion Function
**Priority: HIGH** | **Effort: 2 hours** | **Risk: LOW**

**Tasks:**
- [ ] Update `adc_to_amps()` in `main.rs`
  - [ ] Use calibrated offsets
  - [ ] Verify shunt resistance (CHECK YOUR BOARD!)
  - [ ] Add saturation detection

**Code:**
```rust
// In main.rs, improve adc_to_amps()
fn adc_to_amps(adc_raw: u16, offset: u16) -> f32 {
    // B-G431B-ESC1 current sensing chain:
    // - Shunt resistor: 0.5mΩ (VERIFY THIS ON YOUR BOARD!)
    // - OPAMP PGA gain: 16x
    // - ADC reference: 3.3V
    // - ADC resolution: 12-bit (0-4095)

    const SHUNT_MOHM: f32 = 0.5;  // ⚠️ CHECK SCHEMATIC!
    const PGA_GAIN: f32 = 16.0;
    const ADC_REF_V: f32 = 3.3;
    const ADC_MAX: f32 = 4095.0;

    // Center ADC reading
    let adc_centered = (adc_raw as i32) - (offset as i32);

    // Convert to voltage at ADC input
    let v_adc = (adc_centered as f32 / ADC_MAX) * ADC_REF_V;

    // Reverse OPAMP gain to get shunt voltage
    let v_shunt = v_adc / PGA_GAIN;

    // Ohm's law: I = V / R
    let i_amps = v_shunt / (SHUNT_MOHM / 1000.0);

    i_amps
}

// Use it in motor task:
let (ia_offset, ib_offset, ic_offset) = motor::calibration::get_current_offsets();
let ia = adc_to_amps(IA_SAMPLE.load(Ordering::Relaxed), ia_offset);
let ib = adc_to_amps(IB_SAMPLE.load(Ordering::Relaxed), ib_offset);
let ic = adc_to_amps(IC_SAMPLE.load(Ordering::Relaxed), ic_offset);

// Sanity check: ia + ib + ic should ≈ 0 (Kirchhoff's Current Law)
let current_sum = ia + ib + ic;
if current_sum.abs() > 0.5 {
    defmt::warn!("Current sensor mismatch: sum={:.3}A", current_sum);
}
```

**Hardware verification:**
1. With motor OFF, all currents should read ~0A
2. Apply known DC current (e.g., resistor load) to verify scaling
3. Check sign: positive current = current flowing into motor

**Files to modify:**
- `oxifoc-device/src/main.rs`

**Success criteria:**
- Phase currents read 0A ± 0.1A with motor off
- Current sum (ia+ib+ic) < 0.3A with motor running
- No NaN or Inf values

---

#### 3.3 Add Current Limits & Safety
**Priority: CRITICAL** | **Effort: 2 hours** | **Risk: MEDIUM**

**Tasks:**
- [ ] Add overcurrent detection to ADC interrupt
  - [ ] Check phase currents against limit
  - [ ] Trigger emergency stop if exceeded
  - [ ] Set fault flag

**Code:**
```rust
// In main.rs ADC interrupt (line 746)
#[interrupt]
unsafe fn ADC1_2() {
    const MAX_PHASE_CURRENT_A: f32 = 20.0;  // Conservative limit

    ADC1_INJECTED.lock(|cell| {
        if let Some(injected) = cell.borrow_mut().as_mut() {
            let samples = injected.read_injected_samples();
            IA_SAMPLE.store(samples[0], Ordering::Relaxed);

            // Overcurrent check
            let (ia_offset, _, _) = motor::calibration::get_current_offsets();
            let ia = adc_to_amps(samples[0], ia_offset);
            if ia.abs() > MAX_PHASE_CURRENT_A {
                defmt::error!("OVERCURRENT Phase A: {:.1}A", ia);
                // Set fault flag, trigger emergency stop
                MOTOR_FAULT.store(true, Ordering::Relaxed);
            }

            // ... rest of ADC handling
        }
    });

    // Similar for ADC2 (phases B and C)
}

// Add global fault flag
static MOTOR_FAULT: AtomicBool = AtomicBool::new(false);
```

**Files to modify:**
- `oxifoc-device/src/main.rs`

**Success criteria:**
- Motor stops immediately on overcurrent
- Fault logged via defmt
- Can be cleared and restarted

---

**PHASE 3 DELIVERABLE:**
- ✅ Current offsets calibrated automatically at startup
- ✅ Accurate current measurements (validated with KCL)
- ✅ Overcurrent protection working
- ✅ Current telemetry visible in host GUI

**Estimated Time:** 7 hours
**Risk Level:** MEDIUM (hardware calibration required)

---

## **PHASE 4: Basic FOC Control Loop (Week 2-3)**

### Milestone: Motor Spins with FOC (Current Control Only)

#### 4.1 Implement FOC Controller Structure
**Priority: CRITICAL** | **Effort: 6 hours** | **Risk: HIGH**

**Tasks:**
- [ ] Create `oxifoc-device/src/motor/foc_controller.rs`
  - [ ] `FocController` struct with two PI controllers (Id, Iq)
  - [ ] `update()` method: full FOC loop
  - [ ] Current control mode (Id=0, Iq=setpoint)
  - [ ] Use Hall angle for Park/inverse Park

**Code template:**
```rust
// motor/foc_controller.rs
use super::{transforms, svpwm, pi_controller::PIController, hall::HallSensors};

pub struct FocController {
    id_controller: PIController,  // Flux (d-axis)
    iq_controller: PIController,  // Torque (q-axis)
    max_duty: u16,
    vbus_voltage: f32,
}

impl FocController {
    pub fn new(max_duty: u16) -> Self {
        Self {
            // Start with conservative gains
            id_controller: PIController::new(0.5, 5.0)
                .with_limits(-12.0, 12.0),  // Voltage limits
            iq_controller: PIController::new(0.5, 5.0)
                .with_limits(-12.0, 12.0),
            max_duty,
            vbus_voltage: 24.0,  // Will be updated from VBUS_MV
        }
    }

    /// Main FOC update loop
    /// Returns: [duty_a, duty_b, duty_c]
    pub fn update(
        &mut self,
        ia: f32,
        ib: f32,
        hall_angle_deg: f32,
        id_target: f32,
        iq_target: f32,
        dt: f32,
    ) -> [u16; 3] {
        // Convert Hall angle to radians
        let theta_rad = hall_angle_deg * core::f32::consts::PI / 180.0;
        let sin_theta = libm::sinf(theta_rad);
        let cos_theta = libm::cosf(theta_rad);

        // Clarke transform: ABC → αβ
        let (i_alpha, i_beta) = transforms::clarke(ia, ib);

        // Park transform: αβ → dq (rotating frame)
        let (id, iq) = transforms::park(i_alpha, i_beta, sin_theta, cos_theta);

        // Current PI controllers
        let vd = self.id_controller.update(id_target, id, dt);
        let vq = self.iq_controller.update(iq_target, iq, dt);

        // Normalize voltages to -1..1 range
        let v_alpha_norm = vd / self.vbus_voltage;
        let v_beta_norm = vq / self.vbus_voltage;

        // Inverse Park: dq → αβ
        let (v_alpha, v_beta) = transforms::inverse_park(vd, vq, sin_theta, cos_theta);

        // SVPWM: αβ → ABC duty cycles
        svpwm::space_vector_pwm(v_alpha_norm, v_beta_norm, self.max_duty)
    }

    pub fn reset(&mut self) {
        self.id_controller.reset();
        self.iq_controller.reset();
    }

    pub fn update_vbus(&mut self, vbus_mv: u32) {
        self.vbus_voltage = vbus_mv as f32 / 1000.0;
    }
}
```

**Files to create:**
- `oxifoc-device/src/motor/foc_controller.rs` (~150 lines)

**Success criteria:**
- FOC loop compiles and runs
- No panics or overflows
- PI controllers converge (check with defmt)

---

#### 4.2 Integrate FOC in Motor Task
**Priority: CRITICAL** | **Effort: 4 hours** | **Risk: HIGH**

**Tasks:**
- [ ] Modify motor control task to use `FocController`
  - [ ] Replace 6-step commutation with FOC
  - [ ] Run at 10kHz (100µs loop time)
  - [ ] Add telemetry logging

**Code:**
```rust
// In main.rs motor_control_task
use motor::foc_controller::FocController;
use motor::calibration::get_current_offsets;

#[embassy_executor::task]
async fn motor_control_task(
    mut motor: MotorController<'static>,
    hall_sensors: HallSensors,
    cmd_receiver: /* ... */,
) {
    defmt::info!("FOC motor control task started");

    let mut foc = FocController::new(motor.get_max_duty());
    let (ia_offset, ib_offset, ic_offset) = get_current_offsets();

    // FOC loop at 10kHz
    const LOOP_PERIOD_US: u64 = 100;  // 100µs = 10kHz
    const DT_SECONDS: f32 = 0.0001;   // 100µs in seconds

    let mut id_target = 0.0_f32;
    let mut iq_target = 0.0_f32;

    loop {
        // Update VBUS voltage
        let vbus_mv = VBUS_MV.load(Ordering::Relaxed);
        foc.update_vbus(vbus_mv);

        // Read phase currents
        let ia = adc_to_amps(IA_SAMPLE.load(Ordering::Relaxed), ia_offset);
        let ib = adc_to_amps(IB_SAMPLE.load(Ordering::Relaxed), ib_offset);
        let ic = adc_to_amps(IC_SAMPLE.load(Ordering::Relaxed), ic_offset);

        // Read Hall angle
        let hall_angle_deg = hall_sensors.get_angle_deg(&motor::hall::DEFAULT_HALL_TABLE);

        // Check for commands (non-blocking)
        if let Ok(cmd) = cmd_receiver.try_receive() {
            match cmd {
                MotorCommand::Stop => {
                    id_target = 0.0;
                    iq_target = 0.0;
                    foc.reset();
                }
                MotorCommand::Start { duty } => {
                    // Map duty (0-100%) to Iq current (0-10A)
                    iq_target = (duty as f32 / 100.0) * 10.0;
                    id_target = 0.0;  // No flux weakening yet
                }
                _ => {}
            }
        }

        // Run FOC controller
        let [duty_a, duty_b, duty_c] = foc.update(
            ia, ib,
            hall_angle_deg,
            id_target,
            iq_target,
            DT_SECONDS,
        );

        // Apply PWM duties
        motor.set_pwm_duties(duty_a, duty_b, duty_c);

        // Telemetry (every 100 loops = 10ms)
        static mut LOOP_COUNTER: u32 = 0;
        unsafe {
            LOOP_COUNTER += 1;
            if LOOP_COUNTER % 100 == 0 {
                defmt::debug!("FOC: ia={:.2}, ib={:.2}, Hall={:.0}°, Iq_tgt={:.2}",
                    ia, ib, hall_angle_deg, iq_target);
            }
        }

        // Sleep for loop period
        Timer::after(Duration::from_micros(LOOP_PERIOD_US)).await;
    }
}
```

**Files to modify:**
- `oxifoc-device/src/main.rs`

**Success criteria:**
- Motor spins smoothly
- No cogging or jerking
- Current follows Iq setpoint (±10%)

---

#### 4.3 Add PWM Duty Setting to MotorPwm
**Priority: HIGH** | **Effort: 2 hours** | **Risk: LOW**

**Tasks:**
- [ ] Modify `motor/pwm.rs`
  - [ ] Add `set_pwm_duties(a, b, c)` method
  - [ ] Update TIM1 CCR registers directly

**Code:**
```rust
// In motor/pwm.rs
impl<'d> MotorPwm<'d> {
    pub fn set_pwm_duties(&mut self, duty_a: u16, duty_b: u16, duty_c: u16) {
        // Assuming you have access to TIM1 CCR registers
        // This will depend on your PWM implementation
        self.tim1.set_compare(0, duty_a);  // Phase A
        self.tim1.set_compare(1, duty_b);  // Phase B
        self.tim1.set_compare(2, duty_c);  // Phase C
    }

    pub fn get_max_duty(&self) -> u16 {
        self.config.max_duty
    }
}
```

**Files to modify:**
- `oxifoc-device/src/motor/pwm.rs`

**Success criteria:**
- PWM duties update correctly
- Complementary outputs work
- Deadtime preserved

---

**PHASE 4 DELIVERABLE:**
- ✅ Motor runs with FOC current control
- ✅ Smooth operation (no cogging)
- ✅ Current control loop stable
- ✅ Can command torque via host

**Estimated Time:** 12 hours
**Risk Level:** HIGH (first time motor runs with FOC, tuning required)

---

## **PHASE 5: Hall Calibration (Week 3)**

### Milestone: Automatic Hall Sensor Calibration

#### 5.1 Implement Hall Calibration Routine
**Priority: HIGH** | **Effort: 6 hours** | **Risk: MEDIUM**

**Tasks:**
- [ ] Add calibration routine to `motor/calibration.rs`
  - [ ] Slow open-loop rotation (360° × 3 rotations)
  - [ ] Record Hall state at each electrical angle
  - [ ] Calculate average angle for each Hall state (1-6)
  - [ ] Detect direction (CW vs CCW)

**Code template:**
```rust
// In motor/calibration.rs
pub async fn calibrate_hall_sensors(
    motor: &mut MotorController<'_>,
    hall: &HallSensors,
    duty_percent: u8,
) -> Result<[f32; 8], CalibrationError> {
    defmt::info!("Starting Hall sensor calibration...");

    const STEPS_PER_ROTATION: usize = 360;
    const ROTATIONS: usize = 3;

    // Accumulate angles for each Hall state
    let mut sin_table = [0.0_f32; 8];
    let mut cos_table = [0.0_f32; 8];
    let mut count_table = [0_u32; 8];

    // Perform rotations
    for rotation in 0..ROTATIONS {
        defmt::info!("Calibration rotation {}/{}", rotation + 1, ROTATIONS);

        for angle_deg in 0..STEPS_PER_ROTATION {
            let angle_rad = (angle_deg as f32) * core::f32::consts::PI / 180.0;

            // Apply open-loop commutation
            motor.set_electrical_angle(angle_rad, duty_percent);

            // Wait for rotor to settle
            Timer::after(Duration::from_millis(5)).await;

            // Read Hall state
            let hall_state = hall.read_state() as usize;

            // Accumulate statistics (circular mean)
            sin_table[hall_state] += libm::sinf(angle_rad);
            cos_table[hall_state] += libm::cosf(angle_rad);
            count_table[hall_state] += 1;
        }
    }

    motor.emergency_stop();

    // Calculate average angle for each state
    let mut hall_table = [0.0_f32; 8];

    defmt::info!("Hall calibration results:");
    for i in 0..8 {
        if count_table[i] > 30 {
            let angle_rad = libm::atan2f(sin_table[i], cos_table[i]);
            let mut angle_deg = angle_rad * 180.0 / core::f32::consts::PI;

            // Normalize to 0-360°
            while angle_deg < 0.0 {
                angle_deg += 360.0;
            }

            hall_table[i] = angle_deg;
            defmt::info!("  State {}: {:.1}° ({} samples)", i, angle_deg, count_table[i]);
        } else if i != 0 && i != 7 {
            defmt::warn!("  State {}: too few samples ({})", i, count_table[i]);
            return Err(CalibrationError::InsufficientSamples);
        }
    }

    Ok(hall_table)
}
```

**Files to modify:**
- `oxifoc-device/src/motor/calibration.rs`

**Success criteria:**
- Motor rotates slowly during calibration
- All 6 valid Hall states detected
- Angles are evenly distributed (~60° apart)
- Results consistent across multiple runs

---

#### 5.2 Add Calibration Command to Protocol
**Priority: MEDIUM** | **Effort: 2 hours** | **Risk: LOW**

**Tasks:**
- [ ] Add calibration endpoint to `protocol/src/lib.rs`
  - [ ] `CalibrationCommand::CalibrateHall`
  - [ ] `CalibrationResult` with status and table

**Code:**
```rust
// protocol/src/lib.rs
#[derive(Clone, Schema, Serialize, Deserialize, Debug)]
pub enum CalibrationCommand {
    CalibrateHall { duty: u8 },
    CalibrateCurrentOffsets,
}

#[derive(Clone, Schema, Serialize, Deserialize, Debug)]
pub struct CalibrationResult {
    pub success: bool,
    pub message: String<64>,
    pub hall_table: Option<[f32; 8]>,
}

endpoint!(CalibrationEndpoint, CalibrationCommand, CalibrationResult, "cmd/calibration");
```

**Files to modify:**
- `protocol/src/lib.rs`
- `oxifoc-device/src/main.rs` (add calibration server task)

**Success criteria:**
- Host can trigger calibration
- Results returned to host
- Calibration table saved

---

**PHASE 5 DELIVERABLE:**
- ✅ Hall calibration routine working
- ✅ Accurate Hall table generated
- ✅ Can be triggered from host GUI
- ✅ Motor runs smoother after calibration

**Estimated Time:** 8 hours
**Risk Level:** MEDIUM (requires motor to spin safely)

---

## **PHASE 6: Velocity Control (Week 4)**

### Milestone: Speed Control Loop Working

#### 6.1 Implement Velocity Estimation
**Priority: HIGH** | **Effort: 4 hours** | **Risk: MEDIUM**

**Tasks:**
- [ ] Add velocity estimator to `motor/foc_controller.rs`
  - [ ] Track Hall angle changes over time
  - [ ] Low-pass filter for smooth velocity
  - [ ] Convert electrical speed → mechanical RPM

**Code:**
```rust
// In foc_controller.rs
pub struct VelocityEstimator {
    last_angle: Option<f32>,
    velocity_filter: LowPassFilter,
    pole_pairs: u8,
}

impl VelocityEstimator {
    pub fn new(pole_pairs: u8) -> Self {
        Self {
            last_angle: None,
            velocity_filter: LowPassFilter::new(0.1),  // 10Hz cutoff
            pole_pairs,
        }
    }

    pub fn update(&mut self, angle_deg: f32, dt: f32) -> f32 {
        if let Some(last) = self.last_angle {
            let mut delta = angle_deg - last;

            // Handle wraparound
            if delta > 180.0 {
                delta -= 360.0;
            } else if delta < -180.0 {
                delta += 360.0;
            }

            // Angular velocity (deg/s)
            let omega_deg_s = delta / dt;

            // Filter
            let omega_filtered = self.velocity_filter.update(omega_deg_s);

            // Convert to mechanical RPM
            let rpm = (omega_filtered * 60.0) / (360.0 * self.pole_pairs as f32);

            self.last_angle = Some(angle_deg);
            rpm
        } else {
            self.last_angle = Some(angle_deg);
            0.0
        }
    }
}

struct LowPassFilter {
    alpha: f32,
    value: f32,
}

impl LowPassFilter {
    fn new(alpha: f32) -> Self {
        Self { alpha, value: 0.0 }
    }

    fn update(&mut self, input: f32) -> f32 {
        self.value = self.alpha * input + (1.0 - self.alpha) * self.value;
        self.value
    }
}
```

**Success criteria:**
- Velocity estimate is stable
- No oscillations or noise
- Mechanical RPM matches expected (verify with tachometer if available)

---

#### 6.2 Add Velocity PI Controller
**Priority: HIGH** | **Effort: 3 hours** | **Risk: LOW**

**Tasks:**
- [ ] Add velocity controller to `FocController`
  - [ ] PI controller for velocity loop
  - [ ] Output is Iq setpoint
  - [ ] Run at lower rate (1kHz vs 10kHz for current)

**Code:**
```rust
// In foc_controller.rs
pub struct FocController {
    // ... existing fields
    velocity_controller: PIController,
    velocity_estimator: VelocityEstimator,
    control_mode: ControlMode,
}

pub enum ControlMode {
    CurrentControl,
    VelocityControl,
}

impl FocController {
    pub fn update_velocity_loop(
        &mut self,
        hall_angle_deg: f32,
        velocity_target_rpm: f32,
        dt: f32,
    ) -> f32 {
        // Estimate velocity
        let velocity_rpm = self.velocity_estimator.update(hall_angle_deg, dt);

        // Velocity PI controller → Iq setpoint
        let iq_setpoint = self.velocity_controller.update(
            velocity_target_rpm,
            velocity_rpm,
            dt,
        );

        iq_setpoint
    }
}
```

**Success criteria:**
- Velocity control stable
- Motor accelerates/decelerates smoothly
- Can hold target speed ±5%

---

**PHASE 6 DELIVERABLE:**
- ✅ Velocity estimation working
- ✅ Velocity control loop stable
- ✅ Can command RPM from host
- ✅ Speed displayed in GUI

**Estimated Time:** 7 hours
**Risk Level:** MEDIUM (tuning required)

---

## **PHASE 7: Resistance Detection (Week 4-5)**

### Milestone: Automatic R Measurement

#### 7.1 Implement Resistance Measurement
**Priority: MEDIUM** | **Effort: 4 hours** | **Risk: MEDIUM**

**Tasks:**
- [ ] Add to `motor/calibration.rs`
  - [ ] Lock rotor at 0° electrical
  - [ ] Ramp current slowly to target (2-5A)
  - [ ] Measure voltage and current
  - [ ] Calculate R = V / I

**Code:**
```rust
// In motor/calibration.rs
pub async fn measure_resistance(
    motor: &mut MotorController<'_>,
    target_current_amps: f32,
) -> Result<f32, CalibrationError> {
    defmt::info!("Starting resistance measurement (I={:.1}A)...", target_current_amps);

    const RAMP_STEPS: usize = 200;
    const SAMPLES: usize = 200;

    // Lock rotor at 0°
    motor.set_electrical_angle(0.0, 0);
    motor.enable();

    // Ramp up current
    for step in 0..RAMP_STEPS {
        let current = target_current_amps * (step as f32 / RAMP_STEPS as f32);
        motor.set_id_current(current);
        Timer::after(Duration::from_millis(5)).await;
    }

    // Wait for settling
    Timer::after(Duration::from_millis(50)).await;

    // Sample
    let mut voltage_sum = 0.0_f32;
    let mut current_sum = 0.0_f32;

    for _ in 0..SAMPLES {
        let ia = get_phase_current_a();  // Helper function
        let vd = motor.get_applied_vd();

        voltage_sum += vd;
        current_sum += ia;

        Timer::after(Duration::from_micros(100)).await;
    }

    motor.emergency_stop();

    let voltage_avg = voltage_sum / SAMPLES as f32;
    let current_avg = current_sum / SAMPLES as f32;

    if current_avg < 0.1 {
        return Err(CalibrationError::NoMotorDetected);
    }

    let resistance = voltage_avg / current_avg;

    defmt::info!("Resistance: {:.3}Ω ({:.1}mΩ)", resistance, resistance * 1000.0);

    Ok(resistance)
}
```

**Success criteria:**
- Resistance measured in expected range (40-80mΩ for 5065 motor)
- Consistent across multiple measurements
- Motor doesn't spin during measurement

---

**PHASE 7 DELIVERABLE:**
- ✅ Resistance auto-detection working
- ✅ Results saved to motor parameters
- ✅ Can be triggered from host

**Estimated Time:** 4 hours
**Risk Level:** MEDIUM

---

## **Summary: Full Timeline**

| Phase | Description | Time | Risk | Completion |
|-------|-------------|------|------|------------|
| **1** | FOC Math (transforms, SVPWM, PI) | 10h | LOW | Week 1 |
| **2** | Hall Sensors | 7h | MED | Week 1-2 |
| **3** | Current Calibration | 7h | MED | Week 2 |
| **4** | FOC Current Loop | 12h | HIGH | Week 2-3 |
| **5** | Hall Calibration | 8h | MED | Week 3 |
| **6** | Velocity Control | 7h | MED | Week 4 |
| **7** | R Measurement | 4h | MED | Week 4-5 |
| **Total** | | **55h** | | **4-5 weeks** |

---

## Hardware Checklist

### Before Starting
- [ ] Verify shunt resistor value (check B-G431B-ESC1 schematic)
- [ ] Check Hall sensor wiring (PB6, PB7, PB8)
- [ ] Confirm motor is 5065 270KV with Hall sensors
- [ ] Test power supply (12-24V, 20A+ capable)
- [ ] Have emergency stop button ready
- [ ] Secure motor to prevent spinning

### Safety Limits
- [ ] Max phase current: 20A (conservative for testing)
- [ ] Max duty cycle: 80% (leave margin)
- [ ] Overcurrent protection enabled
- [ ] Timeout protection enabled

---

## Testing Strategy

### Phase 1 Testing (Math Only)
- Unit tests for each transform
- Verify SVPWM sector transitions
- Test PI controller step response

### Phase 2-3 Testing (Hardware)
1. Manual Hall sensor check (rotate by hand)
2. Current offset verification (motor OFF)
3. Current measurement validation (known load)

### Phase 4 Testing (First Spin)
1. Start with 5% duty, low Iq (1A)
2. Verify smooth rotation
3. Gradually increase current
4. Monitor temperature

### Phase 5+ Testing (Calibration & Control)
1. Run Hall calibration
2. Test velocity control at low speeds
3. Step response testing
4. Disturbance rejection (load changes)

---

## Rollback Plan

If FOC doesn't work initially:
1. Fall back to 6-step commutation (already working)
2. Debug one component at a time (transforms → SVPWM → PI)
3. Use host GUI to visualize Id, Iq, Hall angle in real-time

---

## Next Immediate Steps

1. **Commit current changes:**
   ```bash
   git add -A
   git commit -m "Add Hall sensor pin initialization"
   ```

2. **Create Phase 1 files:**
   - `motor/transforms.rs`
   - `motor/svpwm.rs`
   - `motor/pi_controller.rs`

3. **Write unit tests for transforms**

4. **Review hardware:**
   - Check shunt resistor value
   - Verify Hall sensor connections

Would you like me to start implementing Phase 1 (FOC math modules)?
