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
use embassy_stm32::bind_interrupts;
use embassy_stm32::flash::InterruptHandler as FlashInterruptHandler;
use embassy_time::{Duration, Timer};

// Use panic-probe for panics
use panic_probe as _;

// Bind FLASH interrupt for async flash operations
bind_interrupts!(struct FlashIrqs {
    FLASH => FlashInterruptHandler;
});

// Module declarations
mod config;
#[allow(dead_code)]
mod cordic;
pub mod fault;
mod hardware;
mod protocol;
mod storage;
mod transport;

// Motor-related modules (commented out until IHM08M1 shield is connected)
// mod calibration;
// mod control;
// mod motor;
// mod sensors;

use hardware::{AssignedResources, HallResources, MotorResources, StorageResources};

// Motor PWM (uncomment when ready to enable motor control)
// use motor::MotorPwm;

// Define platform state with our fault type
oxifoc_core::define_platform_state!(fault::G474Fault);
use protocol::{DeviceState, RECV_BUF, SCRATCH_BUF, STACK, get_device_state, set_device_state};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ========== STEP 1: Initialize Clock ==========
    let p = hardware::peripherals::init_clock();

    defmt::info!("NUCLEO-G474RE clock initialized: 170MHz SYSCLK from 24MHz HSE");

    // ========== STEP 2: Initialize Transport ==========
    // NUCLEO-G474RE uses LPUART1 for ST-Link VCP (PA2 TX, PA3 RX)
    #[cfg(feature = "transport-uart")]
    let transport = transport::init_uart(&STACK, p.LPUART1, p.PA2, p.PA3);

    #[cfg(feature = "transport-rtt")]
    let transport = transport::init_rtt(&STACK);

    // ========== STEP 3: Initialize Hardware Peripherals ==========

    // Motor control peripherals (commented out until IHM08M1 shield is connected)
    // When the shield arrives, uncomment and configure:
    // - OPAMPs for current sensing
    // - ADCs for current/voltage/temperature measurement
    // - TIM1 for PWM generation
    // - Hall sensor inputs

    // Initialize user LED (PA5 on NUCLEO-G474RE)
    let mut led = hardware::peripherals::init_led(p.PA5);

    // ========== STEP 4: Split Resources ==========
    let r = split_resources!(p);

    // ========== STEP 5: Initialize Persistent Storage ==========
    // Use async flash for non-blocking operations on bank 2
    // Code runs from bank 1, storage is in bank 2 - no CPU stalls during flash ops
    let flash = embassy_stm32::flash::Flash::new(r.storage.flash, FlashIrqs);
    spawner.spawn(storage::storage_worker(flash).unwrap());
    let _runtime_config = storage::CONFIG_LOADED.wait().await;
    defmt::info!("Config loaded from flash");

    // ========== Motor Control Initialization (commented out) ==========
    // When IHM08M1 shield is connected, uncomment:
    //
    // // Initialize OPAMPs as PGAs for phase current shunts
    // let opamp_channels =
    //     hardware::peripherals::init_opamps(p.OPAMP1, p.OPAMP2, p.OPAMP3, ...);
    //
    // // Initialize ADC1/ADC2 with injected conversions
    // let adc_handles =
    //     hardware::peripherals::init_adc(p.ADC1, p.ADC2, opamp_channels, ...);
    //
    // // Initialize Motor PWM
    // let motor_pwm = MotorPwm::new(r.motor, config::PWM_CONFIG);
    //
    // // Initialize Hall Sensor
    // sensors::init_hall(r.hall.pb6, r.hall.pb7, r.hall.pb8, p.TIM6, config::TIMEBASE_TICKS_PER_SEC);
    //
    // // Initialize FOC Controller
    // control::init_foc(motor_pwm, adc_handles.adc1, adc_handles.adc2, p.CORDIC).await;

    // ========== STEP 6: Spawn I/O and Protocol Tasks ==========

    // Spawn RX worker
    spawner.spawn(
        protocol::servers::run_rx(
            transport.rx_worker,
            RECV_BUF.init_with(|| [0u8; config::MAX_PACKET_SIZE]),
            SCRATCH_BUF.init_with(|| [0u8; 64]),
        )
        .unwrap(),
    );

    // Spawn TX worker (transport-specific)
    #[cfg(feature = "transport-uart")]
    spawner.spawn(protocol::servers::run_tx_uart(transport.tx).unwrap());

    #[cfg(feature = "transport-rtt")]
    spawner.spawn(protocol::servers::run_tx_rtt(transport.tx).unwrap());

    // Spawn protocol servers
    protocol::servers::spawn_servers(&spawner);

    // Transition to "waiting for link" once tasks are up
    set_device_state(DeviceState::WaitingLink);

    defmt::info!("All tasks spawned, entering LED status loop");

    // ========== STEP 7: LED Status Loop ==========
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
