#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::gpio::Output;
use embassy_stm32::{bind_interrupts, exti, interrupt};
use embassy_time::{Duration, Timer};

// Use panic-probe for panics
use panic_probe as _;

// Bind EXTI9_5 interrupt for nFAULT monitoring (PB7/EXTI7)
bind_interrupts!(struct ExtiIrqs {
    EXTI9_5 => exti::InterruptHandler<interrupt::typelevel::EXTI9_5>;
});

// Module declarations
mod calibration;
mod config;
mod control;
pub mod fault;
mod hardware;
mod motor;
mod protocol;
mod sensors;
mod storage;
mod transport;

#[allow(unused_imports)]
use hardware::{AssignedResources, DrvResources, HallResources, MotorResources, UartResources};
use motor::MotorPwm;

// Define platform state with our fault type
oxifoc_core::define_platform_state!(fault::F405Fault);

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
    },
));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ========== STEP 0: Initialize defmt logging (RTT + network) ==========
    let defmt_consumer = transport::init_defmt();

    // ========== STEP 1: Initialize Clock ==========
    let p = hardware::peripherals::init_clock();

    // ========== STEP 2: Initialize LEDs ==========
    let green_led = hardware::peripherals::init_green_led(p.PB0);
    let red_led = hardware::peripherals::init_red_led(p.PB1);
    spawner.spawn(heartbeat(green_led).unwrap());
    spawner.spawn(fault_led(red_led).unwrap());

    // ========== STEP 3: Initialize RNG + Ergot Router Stack ==========
    let rng = embassy_stm32::rng::Rng::new(p.RNG, transport::RngIrqs);
    let stack = transport::init_stack(rng);

    // ========== STEP 4: Split Hardware Resources ==========
    let r = split_resources!(p);

    // ========== STEP 5: Initialize USB Transport ==========
    let (usb_transport, usb_ident) = transport::init_usb(stack, p.USB_OTG_FS, p.PA12, p.PA11);

    // ========== STEP 6: Initialize UART Transport (USART3 PB10/PB11) ==========
    let (uart_transport, uart_ident) =
        transport::init_uart(stack, r.uart.usart3, r.uart.pb10, r.uart.pb11);

    // ========== STEP 7: Spawn Transport and Protocol Tasks ==========
    spawner.spawn(protocol::servers::usb_task(usb_transport.usb_dev).unwrap());
    spawner.spawn(
        protocol::servers::run_usb_rx(
            usb_transport.rx_worker,
            protocol::USB_RECV_BUF.init_with(|| [0u8; config::MAX_PACKET_SIZE]),
        )
        .unwrap(),
    );
    spawner.spawn(
        protocol::servers::run_usb_tx(usb_transport.ep_in, transport::USB_OUTQ.framed_consumer())
            .unwrap(),
    );
    spawner.spawn(
        protocol::servers::run_uart_rx(
            uart_transport.rx_worker,
            protocol::UART_RECV_BUF.init_with(|| [0u8; config::MAX_PACKET_SIZE]),
            protocol::UART_SCRATCH_BUF.init_with(|| [0u8; 64]),
        )
        .unwrap(),
    );
    spawner.spawn(protocol::servers::run_uart_tx(uart_transport.tx, stack, uart_ident).unwrap());
    protocol::servers::spawn_servers(&spawner, stack, usb_ident, uart_ident, defmt_consumer);

    // ========== STEP 8: Initialize Persistent Storage ==========
    let flash = embassy_stm32::flash::Flash::new_blocking(p.FLASH);
    let flash = embassy_embedded_hal::adapter::BlockingAsync::new(flash);
    spawner.spawn(storage::storage_worker(flash).unwrap());
    let runtime_config = storage::CONFIG_LOADED.wait().await;
    critical_section::with(|cs| RUNTIME_CONFIG.borrow(cs).replace(runtime_config.clone()));
    defmt::info!("Config loaded from flash");

    // ========== STEP 9: Initialize DRV8301 Gate Driver ==========
    let (drv_config, nfault) = hardware::drv8301::init_spi(
        r.drv.spi3,
        r.drv.pc10,
        r.drv.pc11,
        r.drv.pc12,
        r.drv.pc9,
        r.drv.pb5,
        r.drv.pb7,
        r.drv.exti7,
        ExtiIrqs,
    );

    match hardware::drv8301::configure_and_store_drv8301(drv_config) {
        Ok(()) => defmt::info!("DRV8301 ready"),
        Err(_e) => defmt::error!("DRV8301 configuration failed"),
    }

    hardware::drv8301::enable_gate_driver();
    spawner.spawn(hardware::drv8301::nfault_monitor_task(nfault).unwrap());

    // ========== STEP 10: Initialize Hall Sensor ==========
    sensors::init_hall(r.hall.pc6, r.hall.pc7, r.hall.pc8, p.TIM6);

    // ========== STEP 11: Initialize ADCs ==========
    let adc_handles = hardware::peripherals::init_adc(
        p.ADC1, p.ADC2, p.ADC3, p.PC0, p.PA3, p.PC1, p.PC4, p.PC2, p.PC3,
    );

    // ========== STEP 12: Initialize Motor PWM ==========
    let motor_pwm = MotorPwm::new(r.motor, config::PWM_CONFIG);

    // ========== STEP 13: Initialize FOC Controller ==========
    control::foc::init(motor_pwm, adc_handles, &runtime_config).await;

    defmt::info!(
        "F405 pin map: PWM PA8/PA9/PA10 + PB13/14/15, DRV8301 EN_GATE=PB5, nFAULT=PB7, \
         SPI3 CS/SCK/MISO/MOSI=PC9/PC10/PC11/PC12, halls=PC6/7/8, ADC currents PC0-2, VBUS PC3, \
         USART3 TX=PB10 RX=PB11"
    );
    defmt::info!(
        "Board config: shunt={=f32}Ω, amp_gain={=f32} V/V, vbus_ratio={=f32}:1, faults={}",
        config::BOARD.shunt_ohms,
        config::BOARD.amp_gain,
        config::BOARD.vbus_divider_ratio,
        FAULT_REGISTRY.count()
    );
}

// ========== Background Tasks ==========

/// Blink green LED so we know the scheduler is alive
#[embassy_executor::task]
async fn heartbeat(mut led: Output<'static>) {
    loop {
        led.set_low();
        Timer::after(Duration::from_millis(50)).await;
        led.set_high();
        Timer::after(Duration::from_millis(950)).await;
    }
}

/// Red LED fault indicator — on when any fault is active
#[embassy_executor::task]
async fn fault_led(mut led: Output<'static>) {
    loop {
        if FAULT_REGISTRY.any() {
            led.set_low();
        } else {
            led.set_high();
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
