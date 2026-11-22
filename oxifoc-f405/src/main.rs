#![no_std]
#![no_main]

#[cfg(not(any(feature = "board-cf2", feature = "board-vesc6-mk5")))]
compile_error!("select exactly one board feature: board-cf2 or board-vesc6-mk5");
#[cfg(all(feature = "board-cf2", feature = "board-vesc6-mk5"))]
compile_error!("board-cf2 and board-vesc6-mk5 are mutually exclusive");

use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, exti, interrupt};
use embassy_time::Duration;
use oxifoc_core::state;
use oxifoc_core::stm32_rng::ErgotRng;
use oxifoc_core::storage::RuntimeConfig;

// Bind EXTI9_5 interrupt for nFAULT monitoring (PB7/EXTI7)
bind_interrupts!(struct ExtiIrqs {
    EXTI9_5 =>
        exti::InterruptHandler<embassy_stm32::peripherals::EXTI7>,
        #[cfg(feature = "transport-spi")]
        exti::InterruptHandler<embassy_stm32::peripherals::EXTI6>;
});

// USART1 hardware is unused. Reserve its vector for USB
// work below ADC, Hall and USB hardware IRQs, above thread-mode batching.
#[cfg(feature = "transport-usb")]
static USB_EXECUTOR: embassy_executor::InterruptExecutor =
    embassy_executor::InterruptExecutor::new();

#[cfg(feature = "transport-usb")]
#[embassy_stm32::interrupt]
unsafe fn USART1() {
    // SAFETY: start() initializes the executor before enabling this vector;
    // this handler is its sole poller and USART1 hardware remains disabled.
    unsafe { USB_EXECUTOR.on_interrupt() };
}

// Module declarations
mod calibration;
mod ccm;
mod config;
mod control;
#[cfg(feature = "hall-diagnostics")]
mod diagnostics;
pub mod fault;
mod hardware;
mod logging;
mod motor;
mod protocol;
// Panic/HardFault handlers (gate kill) + IWDG live here.
mod safety;
mod sensors;
mod storage;
mod transport;

#[cfg(feature = "board-vesc6-mk5")]
#[allow(unused_imports)]
use hardware::BoardCtrlResources;
#[allow(unused_imports)]
use hardware::{AssignedResources, DrvResources, HallResources, MotorResources};
use motor::MotorPwm;

// Define platform state with our fault type
oxifoc_core::define_platform_state!(fault::F405Fault);

/// Global runtime config — loaded from flash at boot, read by config_server for protocol access.
pub static RUNTIME_CONFIG: critical_section::Mutex<core::cell::RefCell<RuntimeConfig>> =
    critical_section::Mutex::new(core::cell::RefCell::new(RuntimeConfig::new()));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ========== STEP 1: Initialize Clock ==========
    let p = hardware::peripherals::init_clock();
    // MK5 ties PB10 to PA7/MOSI. Never drive the shared net from USART3.
    let _comm_tx = embassy_stm32::gpio::Input::new(p.PB10, embassy_stm32::gpio::Pull::None);
    #[cfg(not(feature = "transport-spi"))]
    let _comm_rx = embassy_stm32::gpio::Input::new(p.PB11, embassy_stm32::gpio::Pull::None);

    // Comms/housekeeping IRQs sit BELOW the control ISRs. The FOC ADC ISR
    // runs at 0 and the hall capture (TIM3) at 1, but every other vector
    // ships at the reset default 0: an in-flight handler at the SAME level
    // cannot be preempted, so a USB/RNG/EXTI/time-driver handler
    // delayed ADC entry by its full duration. Demoting them lets the FOC
    // ISR preempt immediately.
    {
        use embassy_stm32::interrupt::{InterruptExt, Priority};
        interrupt::OTG_FS.set_priority(Priority::P5); // USB transport
        interrupt::EXTI9_5.set_priority(Priority::P4); // DRV8301 nFAULT
        interrupt::RNG.set_priority(Priority::P6);
        interrupt::TIM2.set_priority(Priority::P6); // embassy time driver
    }

    // ========== STEP 2: Initialize boot epoch + Ergot profile ==========
    // Stack first (no defmt), then the RTT/defmt sink.
    let mut rng = embassy_stm32::rng::Rng::new(p.RNG, transport::RngIrqs);
    #[cfg(feature = "transport-spi")]
    let spi_epoch = rng.blocking_next_u32();
    let detection_boot =
        (u64::from(rng.blocking_next_u32()) << 32) | u64::from(rng.blocking_next_u32());
    let stack = transport::init_stack(ErgotRng::new(rng));

    // ========== STEP 3: Initialize RTT-only defmt logging ==========
    transport::init_defmt();

    spawner.spawn(defmt::unwrap!(logging::worker()));
    spawner.spawn(defmt::unwrap!(isr_log_task()));

    // ========== STEP 4: Initialize LEDs ==========
    let leds = hardware::peripherals::init_status_leds(p.TIM8, p.PB0, p.PB1);
    spawner.spawn(defmt::unwrap!(status_leds(leds)));
    #[cfg(feature = "hall-diagnostics")]
    spawner.spawn(defmt::unwrap!(diagnostics::hall_task()));

    // ========== STEP 5: Split Hardware Resources ==========
    let r = split_resources!(p);

    // MK5: latch the power button (PC5) and enable the current/phase-sense
    // filters ASAP — releasing the button before PC5 goes high powers the
    // board off. The Outputs must live for the whole run.
    #[cfg(feature = "board-vesc6-mk5")]
    let _board_ctrl = hardware::board_early_init(r.board_ctrl);

    // Calibrate VREF+ before any host handshake can publish BoardCalib and
    // before the gate driver or PWM can be enabled. ADC triggers start later.
    let adc_handles = hardware::peripherals::init_adc(
        p.ADC1, p.ADC2, p.ADC3, p.PC0, p.PA3, p.PC1, p.PC4, p.PC2, p.PC3, p.PA0, p.PA1, p.PA2,
    )
    .await;

    // ========== STEP 6: Initialize USB transport ==========
    #[cfg(feature = "transport-usb")]
    let usb_ident = {
        use embassy_stm32::interrupt::InterruptExt;
        interrupt::USART1.set_priority(config::USB_EXECUTOR_PRIORITY);
        let usb_spawner = USB_EXECUTOR.start(interrupt::USART1);
        usb_spawner.spawn(defmt::unwrap!(protocol::servers::usb_io_task(
            stack,
            p.USB_OTG_FS,
            p.PA12,
            p.PA11
        )));
        *protocol::USB_IDENT.get().await
    };
    #[cfg(feature = "transport-spi")]
    transport::spi::init(
        spawner,
        stack,
        transport::spi::Resources {
            bus: p.SPI1,
            sck: p.PA5,
            mosi: p.PA7,
            miso: p.PA6,
            cs: p.PB11,
            ready: p.PB6,
            ready_exti: p.EXTI6,
            tx: p.DMA2_CH3,
            rx: p.DMA2_CH0,
            crc: p.CRC,
        },
        spi_epoch,
    );

    // Collect the idents of every registered interface for the state monitor.
    let mut idents: heapless::Vec<transport::InterfaceIdent, 3> = heapless::Vec::new();
    #[cfg(feature = "transport-spi")]
    let _ = idents.push(());
    #[cfg(feature = "transport-usb")]
    let _ = idents.push(usb_ident);

    // ========== STEP 8: Initialize Persistent Storage ==========
    let flash = embassy_stm32::flash::Flash::new_blocking(p.FLASH, Default::default());
    let flash = embassy_embedded_hal::adapter::BlockingAsync::new(flash);
    spawner.spawn(defmt::unwrap!(storage::storage_worker(flash)));
    let mut runtime_config = storage::CONFIG_LOADED.wait().await;
    let mut pwm_config = config::PWM_CONFIG;
    if let (Some(limits), Some(stored)) = (config::PWM_CONFIG_LIMITS, &runtime_config.pwm_config) {
        if limits.accepts(stored) {
            pwm_config = pwm_config.with_pwm_freq(stored.freq_hz);
        } else {
            defmt::warn!(
                "Ignoring unsupported stored PWM config: freq={}Hz max_duty={}%",
                stored.freq_hz,
                stored.max_duty_percent
            );
            runtime_config.pwm_config = None;
        }
    }
    state::set_motor_temperature_input(runtime_config.motor_temperature.unwrap_or_default());
    critical_section::with(|cs| RUNTIME_CONFIG.borrow(cs).replace(runtime_config.clone()));
    critical_section::with(|cs| {
        let mut state = STATE.borrow(cs).borrow_mut();
        state.load_board_calibration(&runtime_config);
        state.load_record_health(&runtime_config);
        state.load_hall_calibration(&runtime_config);
        state.load_motor_feedback(&runtime_config);
    });
    defmt::info!("Config loaded from flash");
    // Publish config revisions/persistence only after the boot load is complete.
    protocol::servers::spawn_servers(
        &spawner,
        stack,
        &idents,
        detection_boot,
        pwm_config.pwm_freq_hz,
    );

    // ========== STEP 9: Initialize DRV8301 Gate Driver ==========
    let (drv_config, nfault) = hardware::drv8301::init_bus(r.drv, ExtiIrqs);

    let (drv_spi, drv_result) = hardware::drv8301::configure_drv8301(drv_config);
    match drv_result {
        Ok(()) => {
            defmt::info!("DRV8301 ready");
            hardware::drv8301::enable_gate_driver();
        }
        Err(_e) => {
            // An unconfigured DRV runs on its power-on defaults: shunt gain
            // 10 where the board calib assumes 20 (every current reads 2x
            // low, the software OC trip is effectively doubled) and no
            // VDS-OCP programmed. On the MK5's bit-bang SPI one flaky wire
            // lands exactly here. Kill-class fault + gate driver held off:
            // even after a fault clear the bridge cannot switch.
            defmt::error!("DRV8301 configuration failed: DriverFault raised, gate driver disabled");
            hardware::drv8301::disable_gate_driver();
            FAULT_REGISTRY.set(fault::F405Fault::DrvConfigFailed);
        }
    }
    spawner.spawn(defmt::unwrap!(hardware::drv8301::nfault_monitor_task(
        nfault, drv_spi
    )));

    // ========== STEP 10: Initialize Hall Sensor ==========
    sensors::init_hall(r.hall.pc6, r.hall.pc7, r.hall.pc8, p.TIM3);

    // ========== STEP 12: Initialize Motor PWM ==========
    let motor_pwm = MotorPwm::new(r.motor, pwm_config);
    let timing = motor_pwm.control_timing();
    defmt::unwrap!(config::CONTROL_TIMING.init(timing).map_err(|_| ()));
    defmt::info!(
        "Control timing: requested={}Hz clock={}Hz period={}ticks dt={}s rate={=f32}Hz",
        pwm_config.pwm_freq_hz,
        timing.clock_hz,
        timing.period_ticks,
        timing.dt(),
        timing.frequency_hz()
    );

    // ========== STEP 13: Initialize FOC Controller ==========
    #[expect(
        clippy::large_futures,
        reason = "startup owns the PWM and ADC peripherals"
    )]
    control::foc::init(motor_pwm, adc_handles, &runtime_config).await;

    // FOC ISR is running now — arm the watchdog it feeds.
    safety::arm_watchdog(p.IWDG);

    // 1 Hz ISR-cost stats (same isr/s line format as the g431).
    spawner.spawn(defmt::unwrap!(control::foc::isr_stats_task()));

    #[cfg(feature = "board-cf2")]
    defmt::info!(
        "F405 board=CF2: PWM PA8/PA9/PA10 + PB13/14/15, DRV8301 EN_GATE=PB5, nFAULT=PB7, \
         SPI3 CS/SCK/MISO/MOSI=PC9/PC10/PC11/PC12, halls=PC6/7/8, ADC currents PC0-2, VBUS PC3, \
         COMM pins reserved for bridge"
    );
    #[cfg(feature = "board-vesc6-mk5")]
    defmt::info!(
        "F405 board=VESC6_MK5: PWM PA8/PA9/PA10 + PB13/14/15, DRV8301 EN_GATE=PB5, nFAULT=PB7, \
         bit-bang SPI CS/SCK/MISO/MOSI=PC9/PC10/PB3/PB4, halls=PC6/7/8, ADC currents PC0-2 \
         (phase shunts), VBUS PC3, COMM pins reserved for bridge, latch PC5, filters PD2/PC13"
    );
    let board = config::measured_board();
    defmt::info!(
        "Board config: shunt={}Ω, amp_gain={=f32} V/V, vbus_ratio={=f32}:1, vref={}mV, faults={}",
        board.calib.shunt_ohms,
        board.calib.amp_gain,
        board.calib.vbus_divider_ratio,
        board.calib.adc_vref_mv,
        FAULT_REGISTRY.count()
    );
}

// ========== Background Tasks ==========

/// Print the log events the control ISR raised (see `oxifoc_core::isr_log`).
#[embassy_executor::task]
async fn isr_log_task() {
    oxifoc_core::isr_log::run::<oxifoc_core::timer::EmbassyTimer>().await;
}

/// Preserve the 50-ms heartbeat and fault indication, capped at 10% on duty.
#[embassy_executor::task]
async fn status_leds(
    mut leds: embassy_stm32::timer::complementary_pwm::ComplementaryPwm<
        'static,
        embassy_stm32::peripherals::TIM8,
    >,
) {
    use embassy_stm32::timer::Channel;
    let on = leds.get_max_duty() / 10;
    let mut ticker = embassy_time::Ticker::every(Duration::from_millis(50));
    let mut slot = 0;
    loop {
        leds.set_duty(Channel::Ch2, if slot == 0 { on } else { 0 });
        leds.set_duty(Channel::Ch3, if FAULT_REGISTRY.any() { on } else { 0 });
        ticker.next().await;
        slot = (slot + 1) % 20;
    }
}
