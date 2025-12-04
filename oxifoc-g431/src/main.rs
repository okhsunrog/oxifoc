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

// Use panic-probe for panics
use panic_probe as _;

// Module declarations
mod calibration;
mod config;
mod control;
mod hardware;
mod motor;
mod protocol;
mod sensors;
mod transport;

use hardware::{AssignedResources, HallResources, MotorResources};
use motor::MotorPwm;
use oxifoc_core::foc::pwm::MotorPwmConfig;
use protocol::{DeviceState, RECV_BUF, SCRATCH_BUF, STACK, get_device_state, set_device_state};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ========== STEP 1: Initialize Clock ==========
    let p = hardware::peripherals::init_clock();

    // ========== STEP 2: Initialize Transport ==========
    #[cfg(feature = "transport-uart")]
    let transport = transport::init_uart(&STACK, p.USART2, p.PB4, p.PB3);

    #[cfg(feature = "transport-rtt")]
    let transport = transport::init_rtt(&STACK);

    // ========== STEP 3: Initialize Hardware Peripherals ==========

    // Initialize OPAMPs as PGAs for phase current shunts
    let opamp_channels =
        hardware::peripherals::init_opamps(p.OPAMP1, p.OPAMP2, p.OPAMP3, p.PA1, p.PA7, p.PB0);

    // Initialize ADC1/ADC2 with injected conversions
    let adc_handles =
        hardware::peripherals::init_adc(p.ADC1, p.ADC2, opamp_channels, p.PA0, p.PB14);

    // Initialize LED
    let mut led = hardware::peripherals::init_led(p.PC6);

    // ========== STEP 4: Initialize Motor PWM ==========
    let r = split_resources!(p);
    let motor_pwm = MotorPwm::new(r.motor, MotorPwmConfig::default());

    // ========== STEP 5: Initialize Hall Sensor ==========
    sensors::init_hall(
        r.hall.pb6, r.hall.pb7, r.hall.pb8, p.EXTI6, p.EXTI7, p.EXTI8,
    );

    // ========== STEP 6: Initialize FOC Controller ==========
    control::init_foc(motor_pwm, adc_handles.adc1, adc_handles.adc2).await;

    // ========== STEP 7: Spawn I/O and Protocol Tasks ==========

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

    // ========== STEP 8: LED Status Loop ==========
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
