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
mod transport;

#[allow(unused_imports)]
use hardware::{AssignedResources, DrvResources, HallResources, MotorResources};
use motor::MotorPwm;
use protocol::{OUTQ, RECV_BUF, STACK};

// Define platform state with our fault type
oxifoc_core::define_platform_state!(fault::F405Fault);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ========== STEP 1: Initialize Clock ==========
    let p = hardware::peripherals::init_clock();

    // ========== STEP 2: Initialize LED ==========
    let led = hardware::peripherals::init_led(p.PC13);
    spawner.spawn(heartbeat(led).unwrap());

    // ========== STEP 3: Initialize USB Transport ==========
    let transport = transport::init_usb(&STACK, p.USB_OTG_FS, p.PA12, p.PA11);

    // ========== STEP 4: Spawn USB and Protocol Tasks ==========
    spawner.spawn(protocol::servers::usb_task(transport.usb_dev).unwrap());
    spawner.spawn(
        protocol::servers::run_rx(
            transport.rx_worker,
            RECV_BUF.init_with(|| [0u8; config::MAX_PACKET_SIZE]),
        )
        .unwrap(),
    );
    spawner.spawn(protocol::servers::run_tx(transport.ep_in, OUTQ.framed_consumer()).unwrap());
    protocol::servers::spawn_servers(&spawner);

    // ========== STEP 5: Initialize Hardware Peripherals ==========
    let r = split_resources!(p);

    // ========== STEP 6: Initialize DRV8301 Gate Driver ==========
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

    // Configure DRV8301 per VESC settings (stores config globally)
    match hardware::drv8301::configure_and_store_drv8301(drv_config) {
        Ok(()) => defmt::info!("DRV8301 ready"),
        Err(_e) => defmt::error!("DRV8301 configuration failed"),
    }

    // Enable gate driver
    hardware::drv8301::enable_gate_driver();

    // Spawn nFAULT monitor task (EXTI-based fault detection with SPI readout)
    spawner.spawn(hardware::drv8301::nfault_monitor_task(nfault).unwrap());

    // ========== STEP 7: Initialize Hall Sensor ==========
    sensors::init_hall(r.hall.pc6, r.hall.pc7, r.hall.pc8, p.TIM6);

    // ========== STEP 8: Initialize Motor PWM ==========
    let motor_pwm = MotorPwm::new(r.motor, config::PWM_CONFIG);

    // ========== STEP 9: Initialize FOC Controller ==========
    // This sets up injected ADC, TIM1 trigger, and FOC driver
    control::foc::init(motor_pwm).await;

    defmt::info!(
        "F405 pin map: PWM PA8/PA9/PA10 + PB13/14/15, DRV8301 EN_GATE=PB5, nFAULT=PB7, \
         SPI3 CS/SCK/MISO/MOSI=PC9/PC10/PC11/PC12, halls=PC6/7/8, ADC currents PC0-2, VBUS PC3"
    );
    defmt::info!(
        "Board config: shunt={=f32}Ω, amp_gain={=f32} V/V, vbus_ratio={=f32}:1, faults={}",
        config::BOARD.shunt_ohms,
        config::BOARD.amp_gain,
        config::BOARD.vbus_divider_ratio,
        FAULT_REGISTRY.count()
    );

    // Main task completes, other tasks continue running
}

// ========== Background Tasks ==========

/// Blink a status LED so we know the scheduler is alive
#[embassy_executor::task]
async fn heartbeat(mut led: Output<'static>) {
    loop {
        led.set_low();
        Timer::after(Duration::from_millis(50)).await;
        led.set_high();
        Timer::after(Duration::from_millis(950)).await;
    }
}
