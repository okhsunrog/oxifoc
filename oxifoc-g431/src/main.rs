#![no_std]
#![no_main]

// Compile-time check: only one transport can be enabled
#[cfg(all(feature = "transport-uart", feature = "transport-rtt"))]
compile_error!(
    "Cannot enable both transport-uart and transport-rtt features simultaneously. Choose one transport."
);

#[cfg(not(any(feature = "transport-uart", feature = "transport-rtt")))]
compile_error!("Must enable either transport-uart or transport-rtt feature.");

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

// Module declarations
mod calibration;
mod config;
mod cordic;
pub mod fault;
mod foc;
mod hardware;
mod motor;
mod protocol;
// Panic/HardFault handlers (gate kill) + IWDG live here.
mod safety;
mod sensors;
mod storage;
mod transport;

use hardware::{AssignedResources, HallResources, MotorResources, StorageResources};

// Define platform state with our fault type
oxifoc_core::define_platform_state!(fault::G431Fault);
use motor::MotorPwm;
use protocol::{DeviceState, RECV_BUF, SCRATCH_BUF, get_device_state, set_device_state};

/// Global runtime config — loaded from flash at boot, read by config_server for protocol access.
pub static RUNTIME_CONFIG: critical_section::Mutex<
    core::cell::RefCell<oxifoc_core::storage::RuntimeConfig>,
> = critical_section::Mutex::new(core::cell::RefCell::new(
    oxifoc_core::storage::RuntimeConfig {
        motor_params: None,
        hall_calibration: None,
        dc_offsets: None,
        current_limits: None,
        voltage_limits: None,
        pwm_config: None,
        pi_gains: None,
        hall_tuning: None,
        failsafe: None,
    },
));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ========== STEP 1: Initialize Clock ==========
    let p = hardware::init_clock();

    // ========== STEP 2: Initialize RNG + Ergot Router Stack ==========
    let rng = embassy_stm32::rng::Rng::new(p.RNG, transport::RngIrqs);
    let stack = transport::init_stack(rng);

    // ========== STEP 3: Initialize Transport ==========
    #[cfg(feature = "transport-uart")]
    let (transport, ident) = transport::init_uart(stack, p.USART2, p.PB4, p.PB3);

    #[cfg(feature = "transport-rtt")]
    let (transport, ident) = transport::init_rtt(stack);

    // ========== STEP 4: Initialize Hardware Peripherals ==========

    // Initialize OPAMPs as PGAs for phase current shunts
    let opamp_channels = hardware::init_opamps(
        p.OPAMP1, p.OPAMP2, p.OPAMP3, p.PA1, p.PA3, p.PA7, p.PA5, p.PB0, p.PB2,
    );

    // Initialize ADC1/ADC2 with injected conversions
    let adc_handles = hardware::init_adc(p.ADC1, p.ADC2, opamp_channels, p.PA0, p.PB14);

    // Initialize hardware overcurrent protection (COMP1/2/4 + DAC3 → TIM1 BKIN).
    // The earlier false trips from switching noise are handled by the BKF
    // digital filter on the break input (set in motor.rs, same config as
    // ST's MCSDK for this board). Needs a bench re-check under load.
    hardware::init_overcurrent_protection(config::HW_OVERCURRENT_A);

    // Initialize LED
    let mut led = hardware::init_led(p.PC6);

    // ========== STEP 5: Split Resources ==========
    let r = split_resources!(p);

    // ========== STEP 6: Initialize Persistent Storage ==========
    let flash = embassy_stm32::flash::Flash::new_blocking(r.storage.flash);
    let flash = embassy_embedded_hal::adapter::BlockingAsync::new(flash);
    spawner.spawn(defmt::unwrap!(storage::storage_worker(flash)));
    let runtime_config = storage::CONFIG_LOADED.wait().await;
    // Store in static for config_server protocol access
    critical_section::with(|cs| RUNTIME_CONFIG.borrow(cs).replace(runtime_config.clone()));
    defmt::info!("Config loaded from flash");

    // ========== STEP 7: Initialize Motor PWM ==========
    defmt::info!("Initializing motor PWM...");
    let mut motor_pwm = MotorPwm::new(r.motor, config::PWM_CONFIG);
    motor_pwm.emergency_stop(); // Stop PWM triggers until FOC is ready
    defmt::info!("Motor PWM initialized, outputs disabled");

    // ========== STEP 8: Initialize Hall Sensor ==========
    defmt::info!("Initializing hall sensor...");
    sensors::init_hall(r.hall.pb6, r.hall.pb7, r.hall.pb8, p.TIM4);

    // ========== STEP 9: Initialize FOC Controller ==========
    defmt::debug!("Initializing FOC controller...");
    foc::init(
        motor_pwm,
        adc_handles.adc1,
        adc_handles.adc2,
        p.CORDIC,
        &runtime_config,
    )
    .await;
    defmt::info!("FOC init complete");

    // FOC ISR is running now — arm the watchdog it feeds.
    safety::arm_watchdog(p.IWDG);

    // ========== STEP 10: Spawn I/O and Protocol Tasks ==========

    // Spawn RX worker
    spawner.spawn(defmt::unwrap!(protocol::run_rx(
        transport.rx_worker,
        RECV_BUF.init_with(|| [0u8; config::MAX_PACKET_SIZE]),
        SCRATCH_BUF.init_with(|| [0u8; 64]),
    )));

    // Spawn TX worker (transport-specific)
    #[cfg(feature = "transport-uart")]
    spawner.spawn(defmt::unwrap!(protocol::run_tx_uart(
        transport.tx,
        stack,
        ident
    )));

    #[cfg(feature = "transport-rtt")]
    spawner.spawn(defmt::unwrap!(protocol::run_tx_rtt(
        transport.tx,
        stack,
        ident
    )));

    // Spawn protocol servers
    protocol::spawn_servers(&spawner, stack, ident);

    // Transition to "waiting for link" once tasks are up
    set_device_state(DeviceState::WaitingLink);

    defmt::info!("All tasks spawned, entering LED status loop");

    // ========== STEP 11: LED Status Loop ==========
    // Shows device state via blink patterns
    loop {
        match get_device_state() {
            DeviceState::Boot => {
                // Quick double blink
                for _ in 0..2 {
                    led.set_high();
                    Timer::after(Duration::from_millis(100)).await;
                    led.set_low();
                    Timer::after(Duration::from_millis(100)).await;
                }
                Timer::after(Duration::from_millis(600)).await;
            }
            DeviceState::WaitingLink => {
                // Slow blink (1 Hz, 10% duty)
                led.set_high();
                Timer::after(Duration::from_millis(100)).await;
                led.set_low();
                Timer::after(Duration::from_millis(900)).await;
            }
            DeviceState::Linked => {
                // Solid ON with periodic short delay to allow state changes
                led.set_high();
                Timer::after(Duration::from_millis(500)).await;
            }
            DeviceState::Error => {
                // Triple blink pattern
                for _ in 0..3 {
                    led.set_high();
                    Timer::after(Duration::from_millis(120)).await;
                    led.set_low();
                    Timer::after(Duration::from_millis(120)).await;
                }
                Timer::after(Duration::from_millis(800)).await;
            }
        }
    }
}
