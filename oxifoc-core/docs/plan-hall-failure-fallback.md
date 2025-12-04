# Plan: VESC-style Hall Sensor Failure Fallback

## Problem

When Hall sensors fail (disconnected cable, noise, etc.), oxifoc currently:
1. Returns `None` from `HallSensor::sample_at()`
2. `PhaseManager` falls back to **last known output** (stale angle)
3. Motor likely stalls or behaves erratically

VESC handles this gracefully by:
1. Always running the observer in parallel (even in Hall mode)
2. Automatically falling back to observer angle when Hall is invalid
3. Using open-loop override at low speed if needed

## Goals

1. **Observer always runs in parallel** when using Hall sensors
2. **Automatic fallback to observer** when Hall fails
3. **Timeout detection** for stale Hall data
4. **Open-loop override** for low-speed startup recovery
5. **Fault reporting** when Hall fails

## Architecture Changes

### 1. HallSensor: Add Timeout Detection

Add to `HallSensor`:
```rust
/// Configuration
timeout_ticks: u64,           // Max time since last valid edge before "stale"

/// New methods
pub fn is_stale(&self, now_ticks: u64) -> bool
pub fn set_timeout_us(&mut self, timeout_us: u32)
pub fn time_since_edge(&self, now_ticks: u64) -> Option<u64>  // Returns ticks
```

Default timeout: ~100ms (reasonable for low-speed detection)

### 2. PhaseManager: Track Hall Health Status

Add Hall health tracking:
```rust
/// Hall sensor health status
pub enum HallHealth {
    /// Hall working normally
    Ok,
    /// Hall data is stale (no edges for timeout period)
    Stale,
    /// Hall returning invalid states (0 or 7)
    Invalid,
    /// Hall not configured
    NotPresent,
}

/// New fields in PhaseManager
hall_health: HallHealth,
hall_failure_time: Option<u64>,  // When failure was first detected
```

### 3. PhaseManager: Modify `compute_phase()` for Hall Modes

Change behavior for `PhaseSource::Hall`:

**Current:**
```rust
PhaseSource::Hall => sample_to_output(hall_sample, &self.output),
```

**New (VESC-style):**
```rust
PhaseSource::Hall => {
    match hall_sample {
        Some(sample) => {
            self.hall_health = HallHealth::Ok;
            // Use Hall angle, but still blend with observer at high speed
            // (like VESC lines 693-697)
            self.blend_hall_with_observer_by_speed(sample)
        }
        None => {
            // Hall failed - use observer if available
            self.hall_health = HallHealth::Invalid;
            self.fallback_to_observer()
        }
    }
}
```

### 4. New `PhaseSource::HallWithFallback` Mode

Create explicit mode that matches VESC behavior:
```rust
/// Hall sensor with automatic observer fallback (VESC-style)
///
/// - Uses Hall at low speed
/// - Blends to observer at high speed
/// - Falls back to observer if Hall fails
/// - Uses open-loop override if observer not ready
HallWithFallback {
    /// Start blending Hall→Observer (rad/s)
    blend_low: f32,
    /// Full observer (rad/s)
    blend_high: f32,
    /// Hall timeout before fallback (microseconds)
    timeout_us: u32,
}
```

### 5. Open-Loop Override (VESC `m_phase_observer_override`)

Add open-loop override state for startup/recovery:
```rust
/// Open-loop override state (for startup or Hall failure recovery)
struct OpenLoopOverride {
    active: bool,
    angle: f32,
    velocity: f32,
    timer: f32,  // Countdown timer
}

/// New fields in PhaseManager
open_loop_override: OpenLoopOverride,
```

When Hall fails at low speed and observer isn't ready:
1. Activate open-loop override
2. Ramp angle at minimum speed
3. Let observer sync to open-loop angle
4. Transition to observer when ready

### 6. Fault Reporting

Add fault enum and tracking:
```rust
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PhaseFault {
    /// Hall sensor timeout (no edges)
    HallTimeout,
    /// Hall sensor invalid state
    HallInvalidState,
    /// Observer not converged when needed
    ObserverNotReady,
}

/// New in PhaseManager
active_faults: heapless::Vec<PhaseFault, 4>,

/// New methods
pub fn faults(&self) -> &[PhaseFault]
pub fn clear_faults(&mut self)
pub fn has_fault(&self, fault: PhaseFault) -> bool
```

## Implementation Order

### Phase 1: Hall Timeout Detection
1. Add `timeout_ticks` and `is_stale()` to `HallSensor`
2. Add `time_since_edge()` method
3. Add tests for timeout detection

### Phase 2: Hall Health Tracking
1. Add `HallHealth` enum
2. Add health tracking to `PhaseManager`
3. Update `compute_phase()` to detect failures
4. Add `hall_health()` getter

### Phase 3: Automatic Observer Fallback
1. Modify `PhaseSource::Hall` to use observer on failure
2. Add `fallback_to_observer()` helper
3. Ensure observer always updates (even in Hall mode)
4. Add tests for fallback behavior

### Phase 4: HallWithFallback Mode
1. Add new `PhaseSource::HallWithFallback` variant
2. Implement full VESC-style logic:
   - Hall at low speed
   - Blend at medium speed
   - Observer at high speed
   - Fallback on failure
3. Add tests

### Phase 5: Open-Loop Override
1. Add `OpenLoopOverride` struct
2. Implement startup/recovery override logic
3. Add VESC-style angle correction on stuck motor
4. Add tests

### Phase 6: Fault Reporting
1. Add `PhaseFault` enum
2. Add fault tracking to `PhaseManager`
3. Set faults on failures
4. Add public API for fault access

## Testing Strategy

1. **Unit tests** in `hall_sensor.rs`:
   - `test_timeout_detection`
   - `test_is_stale_after_timeout`
   - `test_not_stale_with_recent_edge`

2. **Unit tests** in `phase/manager.rs`:
   - `test_hall_fallback_to_observer`
   - `test_hall_health_tracking`
   - `test_open_loop_override_activation`
   - `test_fault_reporting`

3. **Integration tests**:
   - Simulate Hall failure mid-run
   - Verify smooth transition to observer
   - Verify fault is reported

## Configuration Defaults (VESC-compatible)

```rust
// Hall timeout
const DEFAULT_HALL_TIMEOUT_US: u32 = 100_000;  // 100ms

// Blend thresholds (electrical rad/s)
const DEFAULT_BLEND_LOW: f32 = 300.0;   // ~2900 eRPM
const DEFAULT_BLEND_HIGH: f32 = 600.0;  // ~5700 eRPM

// Open-loop override
const DEFAULT_OPENLOOP_TIME: f32 = 0.5;      // 500ms ramp
const DEFAULT_OPENLOOP_MIN_RPM: f32 = 500.0; // Minimum RPM
```

## Files to Modify

1. `oxifoc-core/src/foc/hall_sensor.rs`
   - Add timeout detection

2. `oxifoc-core/src/foc/phase/source.rs`
   - Add `HallWithFallback` variant

3. `oxifoc-core/src/foc/phase/manager.rs`
   - Add health tracking
   - Add fallback logic
   - Add open-loop override
   - Add fault reporting

4. `oxifoc-core/src/foc/phase/mod.rs`
   - Export new types

## Compatibility Notes

- Existing `PhaseSource::Hall` behavior changes slightly (now falls back to observer)
- Existing `PhaseSource::HallToObserver` unchanged (explicit blending)
- New `PhaseSource::HallWithFallback` for full VESC-style behavior
- Observer must be configured for fallback to work (otherwise uses stale angle)
