//! Configuration constants for oxifoc-f405 (per-board `BOARD` const below)

use embassy_stm32::rcc::{Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv};
use embassy_sync::once_lock::OnceLock;
use oxifoc_core::foc::config::{BoardConfig, PhaseSense};
use oxifoc_core::foc::pwm::MotorPwmConfig;
use oxifoc_core::motor::foc_driver::PreparedBoardConfig;
use oxifoc_core::types::BoardCalib;

/// USART1 USB executor and NetStack BASEPRI ceiling share one typed priority.
/// ADC P0 and Hall P1 remain unmasked while network callbacks run.
pub const USB_EXECUTOR_PRIORITY: embassy_stm32::interrupt::Priority =
    embassy_stm32::interrupt::Priority::P6;

static MEASURED_BOARD: OnceLock<PreparedBoardConfig> = OnceLock::new();
/// Published once after TIM1 is configured, before control IRQs start. Protocol
/// tasks may wait for it while boot calibration is still in progress.
pub static CONTROL_TIMING: OnceLock<oxifoc_core::types::ControlTiming> = OnceLock::new();

/// Freeze the measured ADC reference before protocol tasks or control IRQs run.
pub fn init_measured_board(adc_vref_mv: u32) {
    let mut board = BOARD;
    board.calib.adc_vref_mv = adc_vref_mv;
    defmt::unwrap!(
        MEASURED_BOARD
            .init(PreparedBoardConfig::new(board))
            .map_err(|_| ())
    );
}

/// One immutable calibration shared by sensing, detection and the host handshake.
pub fn measured_board() -> &'static BoardConfig {
    prepared_board().board()
}

/// The same frozen calibration and precomputed voltage-protection constants.
pub fn prepared_board() -> &'static PreparedBoardConfig {
    defmt::unwrap!(MEASURED_BOARD.try_get())
}

// ============================================================================
// Board Hardware Configuration
// ============================================================================

/// Cheap FOCer 2 / Simple FOCer 2 board configuration
///
/// Hardware specs:
/// - Shunt resistors: 2x 1mΩ in parallel = 0.5mΩ effective
/// - DRV8301 amp gain: 10 V/V (for Cheap FOCer 2 v1.0, 20 V/V for v0.9)
/// - ADC: 12-bit with 3.3V reference
/// - VBUS divider: 39k / 2.2k = (39 + 2.2) / 2.2 ratio
/// - Max continuous current: ~60A (FET rating, IRFS7530)
/// - Max VBUS: 57V (VESC HW_LIM_VIN, FET Vds=60V with margin)
#[cfg(feature = "board-cf2")]
pub const BOARD: BoardConfig = BoardConfig {
    current_sense_topology: oxifoc_core::types::CurrentSenseTopology::ThreeShunt,
    calib: BoardCalib {
        shunt_ohms: 0.0005,                     // 0.5mΩ (two 1mΩ in parallel)
        amp_gain: 10.0,                         // DRV8301 10 V/V gain
        adc_vref_mv: 3300,                      // 3.3V
        adc_max_counts: 4095,                   // 12-bit
        invert_current_sign: false,             // DRV8301: standard polarity
        vbus_divider_ratio: (39.0 + 2.2) / 2.2, // ~18.73:1
    },
    max_iq_target_a: 10.0, // Max torque current
    // Fault thresholds (matched to VESC hwconf)
    max_phase_current_a: 60.0, // Peak phase current limit (FET rating)
    max_vbus_mv: 57_000,       // Overvoltage at 57V (VESC HW_LIM_VIN)
    min_vbus_mv: 6_000,        // Undervoltage at 6V (VESC HW_LIM_VIN)
    max_fet_temp_c: 100.0,     // FET overtemp at 100°C
    max_motor_temp_c: 120.0,   // motor winding NTC on PC4
    // Phase-voltage sensing on PA0/PA1/PA2 (SENS1/2/3), divider = Vbus divider
    // (39k/2.2k). No RC phase filters on this board → usable only undriven
    // (back-EMF / coasting-rotation detection), not while PWMing.
    phase_sense: Some(PhaseSense {
        divider_ratio: (39.0 + 2.2) / 2.2,
        has_filters: false,
    }),
};

/// VESC 6 MK5 (Trampa layout + clones; the bench unit is a Flipsky
/// "Mini V6 MK5") board configuration — facts from VESC hwconf
/// hw_60_core.h + the Flipsky listing, see docs/hw/vesc6-mk5.md.
///
/// Hardware specs:
/// - In-line phase shunts (HW_HAS_PHASE_SHUNTS): 0.5mΩ, standard polarity
/// - DRV8301 amp gain: 20 V/V (VESC CURRENT_AMP_GAIN, all hw60 marks)
/// - VBUS divider: 39k / 2.2k, same on SENS1-3
/// - Flipsky rating: 70A continuous / 200A instantaneous, 14-60V
///   (original Trampa hw60: ±120A, VIN 6-57V) — NB 14V operating minimum,
///   power the bench from ≥14V
#[cfg(feature = "board-vesc6-mk5")]
pub const BOARD: BoardConfig = BoardConfig {
    current_sense_topology: oxifoc_core::types::CurrentSenseTopology::ThreeShunt,
    calib: BoardCalib {
        shunt_ohms: 0.0005, // 0.5mΩ in-line phase shunts
        amp_gain: 20.0,     // DRV8301 20 V/V gain (must match drv8301.rs SHUNT_AMP_GAIN)
        adc_vref_mv: 3300,
        adc_max_counts: 4095,
        invert_current_sign: false, // hw60 does NOT define INVERTED_SHUNT_POLARITY
        vbus_divider_ratio: (39.0 + 2.2) / 2.2, // ~18.73:1
    },
    max_iq_target_a: 10.0,     // Max torque current
    max_phase_current_a: 70.0, // Flipsky continuous rating (200A "instantaneous" ignored)
    max_vbus_mv: 57_000,       // Overvoltage at 57V (VESC HW_LIM_VIN; vendor abs max 60V)
    min_vbus_mv: 6_000,        // UV fault floor (VESC HW_LIM_VIN; operating min is 14V)
    max_fet_temp_c: 100.0,     // Conservative (original hw60 cutoff: 110°C)
    max_motor_temp_c: 120.0,   // motor winding NTC on PC4
    // Phase-voltage sensing on PA0/PA1/PA2 (SENS1/2/3), divider = Vbus
    // divider. MK5 has switchable RC phase filters (enabled via PC13 at
    // boot) → usable while PWMing, unlike CF2.
    phase_sense: Some(PhaseSense {
        divider_ratio: (39.0 + 2.2) / 2.2,
        has_filters: true,
    }),
};

// ============================================================================
// PWM Configuration
// ============================================================================

/// Motor PWM configuration
///
/// Central source of truth for PWM frequency and timing.
/// Used by motor.rs for timer setup and control/foc.rs for dt calculation.
pub const PWM_CONFIG: MotorPwmConfig = MotorPwmConfig::new().with_dead_time_ns(360);
// VESC HW_DEAD_TIME_NSEC fallback: 360ns — neither CF2 nor hw60 overrides it

// Effective command-side correction is separate from gate safety timing.
// Start with the old amplitude, but blend through a provisional 1 A per-phase
// region to avoid a full voltage step at the first tiny current reference.
// This is a bench setting, not a measured calibration of the inverter.
pub const DEAD_TIME_COMP_NS: u32 = if cfg!(feature = "bench-no-deadtime-comp") {
    0
} else {
    360
};
pub const DEAD_TIME_COMP_LINEAR_CURRENT_A: f32 = 1.0;

// ============================================================================
// Timing Configuration
// ============================================================================

/// The same typed PLL settings program RCC and derive cycle-based constants.
/// No separate numerical copy of the multiplier/dividers may be maintained.
pub struct ClockProfile {
    pub hse_hz: u32,
    pub pll: Pll,
}

impl ClockProfile {
    const fn vco_hz(&self) -> u32 {
        // F405 M/N register values equal their factors. Match HAL integer order.
        (self.hse_hz / self.pll.prediv.to_bits() as u32) * self.pll.mul.to_bits() as u32
    }

    const fn sys_hz(&self) -> u32 {
        // F405 PLLP encodes Div2/4/6/8 as 0/1/2/3.
        self.vco_hz() / (2 * (self.pll.divp.expect("system PLLP required").to_bits() as u32 + 1))
    }

    const fn usb_hz(&self) -> u32 {
        self.vco_hz() / self.pll.divq.expect("USB PLLQ required").to_bits() as u32
    }
}

/// Default board clock, or the UUID-guarded bench estimate; never persisted in
/// runtime calibration. RCC must be configured before that storage is loaded.
pub const CLOCK_PROFILE: ClockProfile = ClockProfile {
    hse_hz: if cfg!(feature = "bench-calibrated-clock") {
        7_944_893
    } else {
        8_000_000
    },
    pll: Pll {
        prediv: PllPreDiv::Div4,
        mul: if cfg!(feature = "bench-calibrated-clock") {
            PllMul::Mul169
        } else {
            PllMul::Mul168
        },
        divp: Some(PllPDiv::Div2),
        divq: Some(PllQDiv::Div7),
        divr: None,
    },
};
pub const HSE_HZ: u32 = CLOCK_PROFILE.hse_hz;
/// Core clock after PLL init — MUST match hardware/peripherals.rs.
/// Single source for all cycle-based timing: the ISR cycle budget,
/// busy-wait delays (DRV8301 tWAKE, bit-bang SPI), load percentages.
pub const CPU_HZ: u32 = CLOCK_PROFILE.sys_hz();
const _: () = assert!(CPU_HZ <= 168_000_000);
const _: () = assert!(CLOCK_PROFILE.usb_hz() >= 47_880_000);
const _: () = assert!(CLOCK_PROFILE.usb_hz() <= 48_120_000);

// ============================================================================
// Protocol Configuration
// ============================================================================

/// Size of outgoing packet queue for ergot over USB (framed)
pub const USB_OUT_QUEUE_SIZE: usize = if cfg!(any(
    feature = "bench-raw-usb",
    feature = "bench-raw-rtt",
    feature = "bench-raw-ergot"
)) {
    8192
} else {
    4096
};

/// Size of outgoing packet queue for ergot over RTT (COBS stream)
pub const RTT_OUT_QUEUE_SIZE: usize = 4096;

/// Maximum packet size for ergot framing
pub const MAX_PACKET_SIZE: usize = if cfg!(any(
    feature = "bench-raw-usb",
    feature = "bench-raw-rtt",
    feature = "bench-raw-ergot"
)) {
    2304
} else {
    512
};

#[cfg(any(
    feature = "bench-raw-usb",
    feature = "bench-raw-rtt",
    feature = "bench-raw-ergot"
))]
const _: () = assert!(MAX_PACKET_SIZE >= oxifoc_core::raw_telemetry::RAW_MAX_BLOCK_BYTES + 30);

// This value is also every interface's TX MTU: a full fast-telemetry batch
// plus wire overhead must fit or full batches silently fail to broadcast.
const _: () = assert!(
    MAX_PACKET_SIZE
        >= oxifoc_core::types::FAST_BATCH_BYTES + oxifoc_core::types::FAST_BATCH_WIRE_OVERHEAD
);
