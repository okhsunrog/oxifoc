//! RTIC port experiment (see docs/notes/rtic-port-experiment.md).
//!
//! The embassy-executor thread executor is replaced by RTIC 2: the FOC ADC
//! ISR and the TIM3 hall capture become RTIC hardware tasks, and the former
//! thread-mode task soup splits into two software-task priority tiers
//! (I/O pumps above protocol/logic). Everything else stays: the
//! embassy-stm32 HAL, embassy-time (TIM2 time driver), embassy-sync
//! channels/signals, and all of oxifoc-core.

#![no_std]
#![no_main]

#[cfg(not(any(feature = "board-cf2", feature = "board-vesc6-mk5")))]
compile_error!("select exactly one board feature: board-cf2 or board-vesc6-mk5");
#[cfg(all(feature = "board-cf2", feature = "board-vesc6-mk5"))]
compile_error!("board-cf2 and board-vesc6-mk5 are mutually exclusive");
// RTIC-port limitation: rtic-macros resolves every task signature even when
// the task carries a disabled #[cfg], so tasks whose argument types only
// exist under `transport-rtt` cannot be conditionally compiled inside the
// app module. The diagnostic RTT transport is therefore not available on
// this experiment branch (USB + UART, the production interfaces, are).
#[cfg(feature = "transport-rtt")]
compile_error!("transport-rtt is not supported on the RTIC experiment branch");

use embassy_stm32::gpio::Output;
use embassy_stm32::{bind_interrupts, exti, interrupt};
use embassy_time::{Duration, Timer};

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
// Panic/HardFault handlers (gate kill) + IWDG live here.
mod safety;
mod sensors;
mod storage;
mod transport;

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
        failsafe: None,
        velocity: None,
        derating: None,
    },
));

/// Move-once wrapper for `!Send` values produced in `#[init]` and handed to
/// a software task (e.g. `embassy_usb::UsbDevice`, which holds
/// `&mut dyn Handler` references).
struct SendMove<T>(T);
// SAFETY: single-core target; the wrapped value is constructed in RTIC
// `#[init]` and moved exactly once into the spawned task, never aliased and
// never accessed from two contexts. The `Send` bound exists only because
// RTIC's spawn queue is a static, not because anything actually crosses a
// thread/core boundary.
unsafe impl<T> Send for SendMove<T> {}

// ========== Background task bodies ==========
// Plain async fns called by the RTIC software tasks in `app` below.

/// Blink green LED so we know the scheduler is alive
async fn heartbeat_loop(mut led: Output<'static>) {
    loop {
        led.set_low();
        Timer::after(Duration::from_millis(50)).await;
        led.set_high();
        Timer::after(Duration::from_millis(950)).await;
    }
}

/// Red LED fault indicator — on when any fault is active
async fn fault_led_loop(mut led: Output<'static>) {
    loop {
        if FAULT_REGISTRY.any() {
            led.set_low();
        } else {
            led.set_high();
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

/// 1 kHz scheduler/timer heartbeat: sleeps 1 ms in a loop and records how
/// late each wake was. Direct RTIC counterpart of the g431's
/// `exec_probe_task` — the point of the experiment is comparing this
/// number against the embassy cooperative executor's under load (a
/// dispatcher hog or a timer-queue stall shows up here identically).
async fn timer_probe_loop() {
    let mut late_max: u32 = 0;
    let mut last_report = embassy_time::Instant::now();
    loop {
        let before = embassy_time::Instant::now();
        Timer::after(Duration::from_micros(1000)).await;
        let late = (before.elapsed().as_micros() as u32).saturating_sub(1000);
        late_max = late_max.max(late);
        // Live marker for gross stalls; steady-state jitter is well below this.
        if late > 20_000 {
            defmt::warn!("exec stall: {}us late", late);
        }
        let now = embassy_time::Instant::now();
        if now.duration_since(last_report).as_millis() >= 1000 {
            defmt::info!("probe/s: late_max={}us", late_max);
            late_max = 0;
            last_report = now;
        }
    }
}

// ========== RTIC application ==========
//
// Priority map (RTIC logical priorities — higher preempts lower):
//
//   16  hw  ADC   FOC current loop        (NVIC 0x00 — same as pre-port)
//   15  hw  TIM3  hall edge capture       (NVIC 0x10 — same as pre-port)
//    2  sw  I/O pumps: USB device + RX/TX workers, UART RX/TX (+ RTT)
//    1  sw  everything else: protocol servers, detection, storage,
//           telemetry/fault streams, state monitor, LEDs, stats, startup
//
// The two software tiers replace the single cooperative thread executor:
// tier 2 is the experiment's answer to the g431 TX-starvation finding (a
// long-running server step can no longer starve the byte pumps). Comms
// peripheral ISRs (OTG_FS/USART3/RNG/EXTI/TIM2) stay embassy-managed at
// their pre-port NVIC priorities, above both dispatchers.
#[rtic::app(device = embassy_stm32::pac, peripherals = false, dispatchers = [UART4, UART5])]
mod app {
    use embassy_stm32::Peri;
    use embassy_stm32::peripherals::IWDG;

    use crate::hardware::DrvResources;
    use crate::hardware::peripherals::AdcHandles;
    use crate::motor::MotorPwm;
    use crate::protocol::servers;
    use crate::{config, control, hardware, protocol, safety, sensors, storage, transport};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        // DWT cycle counter drives the ISR-cost stats (ISR_CYC_* atomics in
        // control::foc) — enable before the first FOC cycle can run.
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        // ========== Clock ==========
        let p = hardware::peripherals::init_clock();

        // ========== RNG + Ergot Router Stack ==========
        // Stack first (no defmt), then the RTT/defmt sink so an ergot-over-RTT
        // interface (transport-rtt) can register on it in one rtt_init!.
        let rng = embassy_stm32::rng::Rng::new(p.RNG, transport::RngIrqs);
        let stack = transport::init_stack(rng);

        // ========== defmt logging (RTT + network) ==========
        // (transport-rtt is compile_error'd out on this branch — see top.)
        let defmt_consumer = transport::init_defmt();

        // ========== LEDs ==========
        let green_led = hardware::peripherals::init_green_led(p.PB0);
        let red_led = hardware::peripherals::init_red_led(p.PB1);
        defmt::assert!(heartbeat::spawn(green_led).is_ok());
        defmt::assert!(fault_led::spawn(red_led).is_ok());

        // ========== Split Hardware Resources ==========
        // Hand-expanded `split_resources!(p)`: the assign-resources macro is
        // a macro-expanded `#[macro_export]` macro, which Rust refuses to
        // resolve by path from inside the RTIC app module (and unqualified
        // textual scope does not reach in here either). The struct literals
        // below are exactly what the macro expands to — keep them in sync
        // with hardware/resources.rs.
        let r_motor = hardware::MotorResources {
            tim1: p.TIM1,
            pa8: p.PA8,
            pa9: p.PA9,
            pa10: p.PA10,
            pb13: p.PB13,
            pb14: p.PB14,
            pb15: p.PB15,
        };
        #[cfg(feature = "board-cf2")]
        let r_drv = DrvResources {
            spi3: p.SPI3,
            pc9: p.PC9,
            pc10: p.PC10,
            pc11: p.PC11,
            pc12: p.PC12,
            pb5: p.PB5,
            pb7: p.PB7,
            exti7: p.EXTI7,
        };
        #[cfg(feature = "board-vesc6-mk5")]
        let r_drv = hardware::DrvResources {
            pc9: p.PC9,
            pc10: p.PC10,
            pb3: p.PB3,
            pb4: p.PB4,
            pb5: p.PB5,
            pb7: p.PB7,
            exti7: p.EXTI7,
        };
        let r_hall = hardware::HallResources {
            pc6: p.PC6,
            pc7: p.PC7,
            pc8: p.PC8,
        };
        let r_uart = hardware::UartResources {
            usart3: p.USART3,
            pb10: p.PB10,
            pb11: p.PB11,
        };

        // MK5: latch the power button (PC5) and enable the current/phase-sense
        // filters ASAP — releasing the button before PC5 goes high powers the
        // board off. The Outputs must stay driven for the whole run, and RTIC
        // init locals drop when init returns — leak them deliberately.
        #[cfg(feature = "board-vesc6-mk5")]
        {
            let board_ctrl = hardware::board_early_init(hardware::BoardCtrlResources {
                pc5: p.PC5,
                pd2: p.PD2,
                pc13: p.PC13,
            });
            #[expect(
                clippy::mem_forget,
                reason = "board-control pins must stay driven forever"
            )]
            core::mem::forget(board_ctrl);
        }

        // ========== USB + UART transports ==========
        #[cfg(feature = "transport-usb")]
        let (usb_transport, usb_ident) = transport::init_usb(stack, p.USB_OTG_FS, p.PA12, p.PA11);
        #[cfg(feature = "transport-uart")]
        let (uart_transport, uart_ident) =
            transport::init_uart(stack, r_uart.usart3, r_uart.pb10, r_uart.pb11);

        // Collect the idents of every registered interface for the state monitor.
        let mut idents: heapless::Vec<u8, 3> = heapless::Vec::new();
        #[cfg(feature = "transport-uart")]
        let _ = idents.push(uart_ident);
        #[cfg(feature = "transport-usb")]
        let _ = idents.push(usb_ident);

        // ========== I/O pump tasks (priority 2) ==========
        #[cfg(feature = "transport-usb")]
        {
            defmt::assert!(usb_dev::spawn(crate::SendMove(usb_transport.usb_dev)).is_ok());
            defmt::assert!(
                usb_rx::spawn(
                    usb_transport.rx_worker,
                    protocol::USB_RECV_BUF.init_with(|| [0u8; config::MAX_PACKET_SIZE]),
                )
                .is_ok()
            );
            defmt::assert!(
                usb_tx::spawn(usb_transport.ep_in, transport::USB_OUTQ.framed_consumer()).is_ok()
            );
        }
        #[cfg(feature = "transport-uart")]
        {
            defmt::assert!(
                uart_rx::spawn(
                    uart_transport.rx_worker,
                    protocol::UART_RECV_BUF.init_with(|| [0u8; config::MAX_PACKET_SIZE]),
                    protocol::UART_SCRATCH_BUF.init_with(|| [0u8; 64]),
                )
                .is_ok()
            );
            defmt::assert!(uart_tx::spawn(uart_transport.tx, stack, uart_ident).is_ok());
        }

        // ========== Protocol/logic tasks (priority 1) ==========
        defmt::assert!(protocol_srv::spawn(stack).is_ok());
        defmt::assert!(fast_telemetry::spawn(stack).is_ok());
        defmt::assert!(fault_topic::spawn(stack).is_ok());
        defmt::assert!(state_mon::spawn(stack, idents).is_ok());
        defmt::assert!(detect_srv::spawn(stack).is_ok());
        defmt::assert!(seed_router::spawn(stack).is_ok());
        defmt::assert!(defmt_fwd::spawn(defmt_consumer, stack).is_ok());
        defmt::assert!(timer_probe::spawn().is_ok());

        // ========== Persistent Storage ==========
        let flash = embassy_stm32::flash::Flash::new_blocking(p.FLASH);
        let flash = embassy_embedded_hal::adapter::BlockingAsync::new(flash);
        defmt::assert!(storage_task::spawn(flash).is_ok());

        // ========== Hall + ADC + Motor PWM (sync hardware bring-up) ==========
        // Config-independent, so they moved from the old post-config-load
        // sequence into init; TIM3/ADC interrupts are unmasked by RTIC after
        // init returns, and the ADC has no trigger until foc::init enables it.
        sensors::init_hall(r_hall.pc6, r_hall.pc7, r_hall.pc8, p.TIM3);
        let adc_handles = hardware::peripherals::init_adc(
            p.ADC1, p.ADC2, p.ADC3, p.PC0, p.PA3, p.PC1, p.PC4, p.PC2, p.PC3,
        );
        let motor_pwm = MotorPwm::new(r_motor, config::PWM_CONFIG);

        // Async half of bring-up: config load → DRV8301 → FOC → watchdog.
        defmt::assert!(startup::spawn(r_drv, adc_handles, motor_pwm, p.IWDG).is_ok());

        (Shared {}, Local {})
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    // ========== Hardware tasks ==========

    /// FOC current loop — the ADC injected-sequence ISR. Logical priority 16
    /// maps to NVIC priority 0x00, exactly the pre-port hand-configured
    /// value: the actuator's most time-critical ISR, never preempted or
    /// jittered by comms ISRs.
    #[task(binds = ADC, priority = 16, local = [seq: u32 = 0])]
    fn adc_irq(cx: adc_irq::Context) {
        control::foc::adc_isr(cx.local.seq);
    }

    /// Hall edge capture — one logical level below the FOC ISR (NVIC 0x10,
    /// the pre-port value). Edge timestamps are latched in TIM3 hardware, so
    /// delaying this handler only delays when the estimator learns of the
    /// edge, not the timestamp.
    #[task(binds = TIM3, priority = 15)]
    fn tim3_irq(_cx: tim3_irq::Context) {
        sensors::hall::tim3_isr();
    }

    // ========== Async bring-up (continues where init left off) ==========

    /// Config-dependent half of boot, the async part of the old embassy
    /// `main`: waits for the storage worker's config load, then brings up
    /// the DRV8301 and the FOC driver (which itself awaits the first VBUS
    /// sample and the boot current-offset calibration inside the ISR).
    #[task(priority = 1)]
    async fn startup(
        _cx: startup::Context,
        drv: DrvResources,
        adc_handles: AdcHandles,
        motor_pwm: MotorPwm<'static>,
        iwdg: Peri<'static, IWDG>,
    ) {
        let runtime_config = storage::CONFIG_LOADED.wait().await;
        critical_section::with(|cs| {
            crate::RUNTIME_CONFIG
                .borrow(cs)
                .replace(runtime_config.clone());
        });
        defmt::info!("Config loaded from flash");

        // ========== DRV8301 Gate Driver ==========
        let (drv_config, nfault) = hardware::drv8301::init_bus(drv, crate::ExtiIrqs);

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
                defmt::error!(
                    "DRV8301 configuration failed: DriverFault raised, gate driver disabled"
                );
                hardware::drv8301::disable_gate_driver();
                crate::FAULT_REGISTRY.set(crate::fault::F405Fault::DrvConfigFailed);
            }
        }
        defmt::assert!(nfault_mon::spawn(nfault, drv_spi).is_ok());

        // ========== FOC Controller ==========
        control::foc::init(motor_pwm, adc_handles, &runtime_config).await;

        // FOC ISR is running now — arm the watchdog it feeds.
        safety::arm_watchdog(iwdg);

        // 1 Hz ISR-cost stats (same isr/s line format as the g431).
        defmt::assert!(isr_stats::spawn().is_ok());

        #[cfg(feature = "board-cf2")]
        defmt::info!(
            "F405 board=CF2: PWM PA8/PA9/PA10 + PB13/14/15, DRV8301 EN_GATE=PB5, nFAULT=PB7, \
             SPI3 CS/SCK/MISO/MOSI=PC9/PC10/PC11/PC12, halls=PC6/7/8, ADC currents PC0-2, VBUS PC3, \
             USART3 TX=PB10 RX=PB11"
        );
        #[cfg(feature = "board-vesc6-mk5")]
        defmt::info!(
            "F405 board=VESC6_MK5: PWM PA8/PA9/PA10 + PB13/14/15, DRV8301 EN_GATE=PB5, nFAULT=PB7, \
             bit-bang SPI CS/SCK/MISO/MOSI=PC9/PC10/PB3/PB4, halls=PC6/7/8, ADC currents PC0-2 \
             (phase shunts), VBUS PC3, USART3 TX=PB10 RX=PB11, latch PC5, filters PD2/PC13"
        );
        defmt::info!(
            "Board config: shunt={=f32}Ω, amp_gain={=f32} V/V, vbus_ratio={=f32}:1, faults={}",
            config::BOARD.calib.shunt_ohms,
            config::BOARD.calib.amp_gain,
            config::BOARD.calib.vbus_divider_ratio,
            crate::FAULT_REGISTRY.count()
        );
    }

    // ========== I/O pump tasks (priority 2) ==========

    /// USB device state machine
    #[cfg(feature = "transport-usb")]
    #[task(priority = 2)]
    async fn usb_dev(
        _cx: usb_dev::Context,
        dev: crate::SendMove<embassy_usb::UsbDevice<'static, transport::AppDriver>>,
    ) {
        servers::usb_task(dev.0).await;
    }

    /// Incoming ergot data (USB)
    #[cfg(feature = "transport-usb")]
    #[task(priority = 2)]
    async fn usb_rx(_cx: usb_rx::Context, rcvr: transport::UsbRxWorker, buf: &'static mut [u8]) {
        servers::run_usb_rx(rcvr, buf).await;
    }

    /// Outgoing ergot data (USB framed)
    #[cfg(feature = "transport-usb")]
    #[task(priority = 2)]
    async fn usb_tx(
        _cx: usb_tx::Context,
        ep_in: <transport::AppDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
        rx: ergot::exports::bbqueue::prod_cons::framed::FramedConsumer<
            &'static transport::UsbQueue,
        >,
    ) {
        servers::run_usb_tx(ep_in, rx).await;
    }

    /// Incoming ergot data (UART)
    #[cfg(feature = "transport-uart")]
    #[task(priority = 2)]
    async fn uart_rx(
        _cx: uart_rx::Context,
        rcvr: transport::UartRxWorker,
        recv_buf: &'static mut [u8],
        scratch_buf: &'static mut [u8],
    ) {
        servers::run_uart_rx(rcvr, recv_buf, scratch_buf).await;
    }

    /// Outgoing ergot data (UART COBS stream)
    #[cfg(feature = "transport-uart")]
    #[task(priority = 2)]
    async fn uart_tx(
        _cx: uart_tx::Context,
        tx: transport::UartWriter,
        stack: &'static transport::Stack,
        uart_ident: u8,
    ) {
        servers::run_uart_tx(tx, stack, uart_ident).await;
    }

    // ========== Protocol/logic tasks (priority 1) ==========

    /// All ergot endpoint servers, joined
    #[task(priority = 1)]
    async fn protocol_srv(_cx: protocol_srv::Context, stack: &'static transport::Stack) {
        // This future IS the protocol-servers task; RTIC statically
        // allocates it (same rationale as the expect inside the fn).
        #[expect(clippy::large_futures, reason = "the joined servers are the task")]
        servers::protocol_servers(stack).await;
    }

    /// Fast telemetry streaming — drains bbqueue and broadcasts batches
    #[task(priority = 1)]
    async fn fast_telemetry(_cx: fast_telemetry::Context, stack: &'static transport::Stack) {
        servers::fast_telemetry_task(stack).await;
    }

    /// Fault topic publisher
    #[task(priority = 1)]
    async fn fault_topic(_cx: fault_topic::Context, stack: &'static transport::Stack) {
        servers::fault_topic_task(stack).await;
    }

    /// Interface state monitor (link up/down → failsafe)
    #[task(priority = 1)]
    async fn state_mon(
        _cx: state_mon::Context,
        stack: &'static transport::Stack,
        idents: heapless::Vec<u8, 3>,
    ) {
        servers::state_monitor(stack, idents).await;
    }

    /// Motor detection server
    #[task(priority = 1)]
    async fn detect_srv(_cx: detect_srv::Context, stack: &'static transport::Stack) {
        servers::detect_server(stack).await;
    }

    /// Ergot well-known services (seed router + ping)
    #[task(priority = 1)]
    async fn seed_router(_cx: seed_router::Context, stack: &'static transport::Stack) {
        servers::seed_router_task(stack).await;
    }

    /// defmt → ergot topic forwarder
    #[task(priority = 1)]
    async fn defmt_fwd(
        _cx: defmt_fwd::Context,
        consumer: ergot::logging::defmt_sink::DefmtConsumer,
        stack: &'static transport::Stack,
    ) {
        servers::defmt_forwarder(consumer, stack).await;
    }

    /// Storage worker (config load at boot, then flash writes)
    #[task(priority = 1)]
    async fn storage_task(_cx: storage_task::Context, flash: storage::AsyncFlash) {
        storage::storage_worker(flash).await;
    }

    /// DRV8301 nFAULT monitor
    #[task(priority = 1)]
    async fn nfault_mon(
        _cx: nfault_mon::Context,
        nfault: hardware::drv8301::NfaultInput,
        bus: hardware::drv8301::Drv8301Spi,
    ) {
        hardware::drv8301::nfault_monitor_task(nfault, bus).await;
    }

    /// 1 Hz ISR-cost stats
    #[task(priority = 1)]
    async fn isr_stats(_cx: isr_stats::Context) {
        control::foc::isr_stats_task().await;
    }

    /// Green LED heartbeat
    #[task(priority = 1)]
    async fn heartbeat(_cx: heartbeat::Context, led: embassy_stm32::gpio::Output<'static>) {
        crate::heartbeat_loop(led).await;
    }

    /// Red LED fault indicator
    #[task(priority = 1)]
    async fn fault_led(_cx: fault_led::Context, led: embassy_stm32::gpio::Output<'static>) {
        crate::fault_led_loop(led).await;
    }

    /// 1 kHz timer-latency probe (experiment instrumentation)
    #[task(priority = 1)]
    async fn timer_probe(_cx: timer_probe::Context) {
        crate::timer_probe_loop().await;
    }
}
