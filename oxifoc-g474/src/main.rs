#![no_std]
#![no_main]

// Compile-time check: exactly one transport must be enabled
#[cfg(all(feature = "transport-uart", feature = "transport-rtt"))]
compile_error!("Cannot enable both transport-uart and transport-rtt simultaneously.");
#[cfg(all(feature = "transport-uart", feature = "transport-usb"))]
compile_error!("Cannot enable both transport-uart and transport-usb simultaneously.");
#[cfg(all(feature = "transport-rtt", feature = "transport-usb"))]
compile_error!("Cannot enable both transport-rtt and transport-usb simultaneously.");
#[cfg(not(any(
    feature = "transport-uart",
    feature = "transport-rtt",
    feature = "transport-usb"
)))]
compile_error!("Must enable exactly one of: transport-uart, transport-rtt, transport-usb.");

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

// Define platform state with our fault type
oxifoc_core::define_platform_state!(fault::G474Fault);
use protocol::{DeviceState, RECV_BUF, STACK, get_device_state, set_device_state};
#[cfg(any(feature = "transport-uart", feature = "transport-rtt"))]
use protocol::SCRATCH_BUF;
#[cfg(feature = "transport-usb")]
use protocol::OUTQ;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ========== STEP 1: Initialize Clock ==========
    let p = hardware::peripherals::init_clock();

    defmt::info!("NUCLEO-G474RE clock initialized: 170MHz SYSCLK from 24MHz HSE");

    // ========== STEP 2: Initialize Transport ==========
    #[cfg(feature = "transport-uart")]
    let transport = transport::init_uart(&STACK, p.LPUART1, p.PA2, p.PA3);

    #[cfg(feature = "transport-rtt")]
    let transport = transport::init_rtt(&STACK);

    #[cfg(feature = "transport-usb")]
    let transport = transport::init_usb(&STACK, p.USB, p.PA12, p.PA11);

    // ========== STEP 3: Initialize Hardware Peripherals ==========

    // Initialize user LED (PA5 on NUCLEO-G474RE)
    let mut led = hardware::peripherals::init_led(p.PA5);

    // ========== STEP 4: Split Resources ==========
    let r = split_resources!(p);

    // ========== STEP 5: Initialize Persistent Storage ==========
    let flash = embassy_stm32::flash::Flash::new(r.storage.flash, FlashIrqs);
    spawner.spawn(storage::storage_worker(flash).unwrap());
    let _runtime_config = storage::CONFIG_LOADED.wait().await;
    defmt::info!("Config loaded from flash");

    // ========== STEP 6: Spawn I/O and Protocol Tasks ==========

    // Spawn RX worker
    #[cfg(any(feature = "transport-uart", feature = "transport-rtt"))]
    spawner.spawn(
        protocol::servers::run_rx(
            transport.rx_worker,
            RECV_BUF.init_with(|| [0u8; config::MAX_PACKET_SIZE]),
            SCRATCH_BUF.init_with(|| [0u8; 64]),
        )
        .unwrap(),
    );

    #[cfg(feature = "transport-usb")]
    spawner.spawn(
        protocol::servers::run_rx(
            transport.rx_worker,
            RECV_BUF.init_with(|| [0u8; config::MAX_PACKET_SIZE]),
        )
        .unwrap(),
    );

    // Spawn TX worker (transport-specific)
    #[cfg(feature = "transport-uart")]
    spawner.spawn(protocol::servers::run_tx_uart(transport.tx).unwrap());

    #[cfg(feature = "transport-rtt")]
    spawner.spawn(protocol::servers::run_tx_rtt(transport.tx).unwrap());

    #[cfg(feature = "transport-usb")]
    {
        spawner.spawn(protocol::servers::usb_task(transport.usb_dev).unwrap());
        spawner.spawn(
            protocol::servers::run_tx_usb(transport.ep_in, OUTQ.framed_consumer()).unwrap(),
        );
    }

    // Spawn protocol servers
    protocol::servers::spawn_servers(&spawner);

    // Transition to "waiting for link" once tasks are up
    set_device_state(DeviceState::WaitingLink);

    defmt::info!("All tasks spawned, entering LED status loop");

    // ========== STEP 7: LED Status Loop ==========
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
