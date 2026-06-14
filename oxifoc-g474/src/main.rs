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
    // Stack first (no defmt), then the RTT/defmt sink, then the other transports
    // (which log via defmt). Idents of every registered interface are collected
    // for the state monitor.
    let rng = embassy_stm32::rng::Rng::new(p.RNG, transport::RngIrqs);
    let stack = transport::init_stack(rng);

    // ========== STEP 3: RTT (defmt sink; ergot interface when transport-rtt) ==
    #[cfg(not(feature = "transport-rtt"))]
    transport::init_defmt_rtt();
    #[cfg(feature = "transport-rtt")]
    let (rtt_transport, rtt_ident) = transport::init_rtt(stack);

    // ========== STEP 4: UART (LPUART1 VCP) + USB transports ==========
    #[cfg(feature = "transport-uart")]
    let (uart_transport, uart_ident) = transport::init_uart(stack, p.LPUART1, p.PA2, p.PA3);
    #[cfg(feature = "transport-usb")]
    let (usb_transport, usb_ident) = transport::init_usb(stack, p.USB, p.PA12, p.PA11);

    // Collect the idents of every registered interface for the state monitor.
    let mut idents: heapless::Vec<u8, 3> = heapless::Vec::new();
    #[cfg(feature = "transport-rtt")]
    let _ = idents.push(rtt_ident);
    #[cfg(feature = "transport-uart")]
    let _ = idents.push(uart_ident);
    #[cfg(feature = "transport-usb")]
    let _ = idents.push(usb_ident);

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

    // Spawn RTT I/O workers (ergot over RTT)
    #[cfg(feature = "transport-rtt")]
    {
        spawner.spawn(defmt::unwrap!(protocol::servers::run_rtt_rx(
            rtt_transport.rx_worker,
            protocol::RTT_RECV_BUF.init_with(|| [0u8; config::MAX_PACKET_SIZE]),
            protocol::RTT_SCRATCH_BUF.init_with(|| [0u8; 64]),
        )));
        spawner.spawn(defmt::unwrap!(protocol::servers::run_rtt_tx(
            rtt_transport.tx
        )));
    }

    // Spawn USB tasks
    #[cfg(feature = "transport-usb")]
    {
        spawner.spawn(defmt::unwrap!(protocol::servers::usb_task(
            usb_transport.usb_dev
        )));
        spawner.spawn(defmt::unwrap!(protocol::servers::run_usb_rx(
            usb_transport.rx_worker,
            protocol::USB_RECV_BUF.init_with(|| [0u8; config::MAX_PACKET_SIZE]),
        )));
        spawner.spawn(defmt::unwrap!(protocol::servers::run_usb_tx(
            usb_transport.ep_in,
            transport::USB_OUTQ.framed_consumer()
        )));
    }

    // Spawn UART tasks
    #[cfg(feature = "transport-uart")]
    {
        spawner.spawn(defmt::unwrap!(protocol::servers::run_uart_rx(
            uart_transport.rx_worker,
            protocol::UART_RECV_BUF.init_with(|| [0u8; config::MAX_PACKET_SIZE]),
            protocol::UART_SCRATCH_BUF.init_with(|| [0u8; 64]),
        )));
        spawner.spawn(defmt::unwrap!(protocol::servers::run_uart_tx(
            uart_transport.tx,
            stack,
            uart_ident
        )));
    }

    // Spawn protocol servers (state monitor watches every registered interface)
    protocol::servers::spawn_servers(&spawner, stack, &idents);

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
