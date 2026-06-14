#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::flash::InterruptHandler as FlashInterruptHandler;
use embassy_time::{Duration, Timer};

// Bind FLASH interrupt for async flash operations
bind_interrupts!(struct FlashIrqs {
    FLASH => FlashInterruptHandler;
});

// Module declarations
mod config;
#[allow(dead_code)]
mod cordic;
mod hardware;
mod protocol;
// Panic/HardFault handlers (gate kill). IWDG arming goes here too once
// the motor modules wake up — see the module docs.
mod safety;
mod storage;
mod transport;

// Motor-related modules (commented out until IHM08M1 shield is connected).
// `sensors` is compiled to keep the TIM2 hall module from rotting.
// mod calibration;
// mod control;
// mod motor;
mod sensors;

use hardware::{AssignedResources, HallResources, MotorResources, StorageResources};

// Define platform state with our fault type
oxifoc_core::define_platform_state!(oxifoc_core::foc::fault::StandardFault);

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
        velocity: None,
        derating: None,
    },
));

use protocol::{DeviceState, get_device_state, set_device_state};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ========== STEP 1: Initialize Clock ==========
    let p = hardware::peripherals::init_clock();

    defmt::info!("NUCLEO-G474RE clock initialized: 170MHz SYSCLK from 24MHz HSE");

    // ========== STEP 2: Initialize RNG + Ergot Router Stack ==========
    let rng = embassy_stm32::rng::Rng::new(p.RNG, transport::RngIrqs);
    let stack = transport::init_stack(rng);

    // ========== STEP 3: Initialize RTT Transport (ergot + defmt) ==========
    let (rtt_transport, ident) = transport::init_rtt(stack);

    // ========== STEP 5: Initialize Hardware Peripherals ==========

    // Initialize user LED (PA5 on NUCLEO-G474RE)
    let mut led = hardware::peripherals::init_led(p.PA5);

    // ========== STEP 6: Split Resources ==========
    let r = split_resources!(p);

    // ========== STEP 7: Initialize Persistent Storage ==========
    let flash = embassy_stm32::flash::Flash::new(r.storage.flash, FlashIrqs);
    spawner.spawn(defmt::unwrap!(storage::storage_worker(flash)));
    let runtime_config = storage::CONFIG_LOADED.wait().await;
    critical_section::with(|cs| RUNTIME_CONFIG.borrow(cs).replace(runtime_config.clone()));
    defmt::info!("Config loaded from flash");

    // ========== STEP 8: Spawn Transport and Protocol Tasks ==========

    // Spawn RTT I/O workers
    spawner.spawn(defmt::unwrap!(protocol::servers::run_rx(
        rtt_transport.rx_worker,
        protocol::RECV_BUF.init_with(|| [0u8; config::MAX_PACKET_SIZE]),
        protocol::SCRATCH_BUF.init_with(|| [0u8; 64]),
    )));
    spawner.spawn(defmt::unwrap!(protocol::servers::run_tx_rtt(
        rtt_transport.tx
    )));

    // Spawn protocol servers (incl. fast telemetry + synthetic generator)
    protocol::servers::spawn_servers(&spawner, stack, ident);

    // Transition to "waiting for link" once tasks are up
    set_device_state(DeviceState::WaitingLink);

    defmt::info!("All tasks spawned, entering LED status loop");

    // ========== STEP 9: LED Status Loop ==========
    loop {
        match get_device_state() {
            DeviceState::Boot => {
                for _ in 0..2 {
                    led.set_high();
                    Timer::after(Duration::from_millis(100)).await;
                    led.set_low();
                    Timer::after(Duration::from_millis(100)).await;
                }
                Timer::after(Duration::from_millis(600)).await;
            }
            DeviceState::WaitingLink => {
                led.set_high();
                Timer::after(Duration::from_millis(100)).await;
                led.set_low();
                Timer::after(Duration::from_millis(900)).await;
            }
            DeviceState::Linked => {
                led.set_high();
                Timer::after(Duration::from_millis(500)).await;
            }
            DeviceState::Error => {
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
