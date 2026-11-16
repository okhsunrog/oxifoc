#![no_std]
#![no_main]

#[path = "support/dwt.rs"]
#[allow(dead_code)]
mod dwt;
#[path = "../../../oxifoc-f405/src/logging.rs"]
mod logging;

use core::{
    mem::ManuallyDrop,
    sync::atomic::{AtomicU32, Ordering},
};
use embassy_stm32::interrupt;

static JOB: AtomicU32 = AtomicU32::new(0);
static DONE: AtomicU32 = AtomicU32::new(0);
static CYCLES: AtomicU32 = AtomicU32::new(0);

#[interrupt]
fn ADC() {
    let start = dwt::cycles();
    if JOB.load(Ordering::Relaxed) == 2 {
        let bytes = core::hint::black_box([0x5a; 300]);
        defmt::warn!("LOGGER oversized {=[u8]}", bytes.as_slice());
    } else {
        defmt::warn!(
            "startup: handoff confirmed by probe (probe_vel={} observer_vel={})",
            core::hint::black_box(123.25_f32),
            core::hint::black_box(125.5_f32)
        );
    }
    CYCLES.store(dwt::cycles().wrapping_sub(start), Ordering::Relaxed);
    DONE.store(1, Ordering::Release);
}

#[interrupt]
fn USART2() {
    defmt::warn!("LOGGER contending interrupt");
    DONE.store(1, Ordering::Release);
}

fn trigger(irq: interrupt::Interrupt) {
    DONE.store(0, Ordering::Relaxed);
    cortex_m::peripheral::NVIC::pend(irq);
    while DONE.load(Ordering::Acquire) == 0 {
        core::hint::spin_loop();
    }
}

struct Interrupting;

impl defmt::Format for Interrupting {
    fn format(&self, fmt: defmt::Formatter) {
        trigger(interrupt::ADC);
        trigger(interrupt::USART2);
        defmt::write!(fmt, "resumed");
    }
}

struct FatalDuringFormat;

impl defmt::Format for FatalDuringFormat {
    fn format(&self, _fmt: defmt::Formatter) {
        // Model a panic while the ordinary encoder is already acquired.
        // Production never resumes the interrupted frame after entering fatal mode.
        logging::enter_fatal();
        defmt::warn!("LOGGER emergency while normal encoder occupied");
    }
}

#[embedded_test::tests(executor = embassy_executor::Executor::new())]
mod tests {
    use super::*;
    use embassy_stm32::{
        gpio::{Level, Output, Speed},
        rcc::*,
        time::Hertz,
    };
    use embassy_time::{Duration, Timer};

    struct TestState {
        _latch: ManuallyDrop<Output<'static>>,
        _gate: ManuallyDrop<Output<'static>>,
        _timer: ManuallyDrop<
            embassy_stm32::timer::low_level::Timer<'static, embassy_stm32::peripherals::TIM3>,
        >,
    }

    #[init]
    fn init() -> TestState {
        let mut config = embassy_stm32::Config::default();
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::Hse;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::Div4,
            mul: PllMul::Mul168,
            divp: Some(PllPDiv::Div2),
            divq: Some(PllQDiv::Div7),
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::Div1;
        config.rcc.apb1_pre = APBPrescaler::Div4;
        config.rcc.apb2_pre = APBPrescaler::Div2;
        config.rcc.sys = Sysclk::Pll1P;
        let p = embassy_stm32::init(config);
        let timer = embassy_stm32::timer::low_level::Timer::new(p.TIM3);
        let regs = timer.regs_gp16();
        regs.psc().write_value(83);
        regs.arr().write(|w| w.set_arr(u16::MAX));
        regs.egr().write(|w| w.set_ug(true));
        timer.start();
        let state = TestState {
            _latch: ManuallyDrop::new(Output::new(p.PC5, Level::High, Speed::Low)),
            _gate: ManuallyDrop::new(Output::new(p.PB5, Level::Low, Speed::Low)),
            _timer: ManuallyDrop::new(timer),
        };
        embassy_stm32::pac::FLASH.acr().modify(|w| {
            w.set_prften(true);
            w.set_icen(true);
            w.set_dcen(true);
        });
        let channels = rtt_target::rtt_init! { up: {
            0: { size: 4096, mode: rtt_target::ChannelMode::NoBlockSkip, name: "defmt" }
        }};
        logging::init(channels.up.0);
        dwt::enable();
        // SAFETY: both handlers are installed and TIM1/ADC remain unconfigured.
        unsafe {
            cortex_m::peripheral::NVIC::unmask(interrupt::ADC);
        }
        // SAFETY: USART2 is used only as a software-pended logging test interrupt.
        unsafe {
            cortex_m::peripheral::NVIC::unmask(interrupt::USART2);
        }
        state
    }

    #[test]
    async fn deferred_logging(_state: TestState) {
        // SAFETY: the test is polled by the executor supplied above.
        let spawner = unsafe { embassy_executor::Spawner::for_current_executor().await };
        spawner.spawn(defmt::unwrap!(logging::worker()));
        Timer::after(Duration::from_millis(1)).await;
        let mut stats = dwt::Stats::new();
        let queued = logging::LOG_STATS.adc_queued.load(Ordering::Relaxed);
        let written = logging::LOG_STATS.written.load(Ordering::Relaxed);
        for _ in 0..32 {
            trigger(interrupt::ADC);
            stats.record(CYCLES.load(Ordering::Relaxed));
        }
        defmt::assert_eq!(
            logging::LOG_STATS.adc_queued.load(Ordering::Relaxed),
            queued + 32
        );
        // No task switch has occurred, so none of the ADC frames reached RTT yet.
        defmt::assert_eq!(logging::LOG_STATS.written.load(Ordering::Relaxed), written);
        Timer::after(Duration::from_millis(20)).await;
        defmt::assert!(logging::LOG_STATS.written.load(Ordering::Relaxed) > written);
        stats.report("adc_log_waiting_consumer", 32, 1);

        let oversize = logging::LOG_STATS.oversize.load(Ordering::Relaxed);
        JOB.store(2, Ordering::Relaxed);
        trigger(interrupt::ADC);
        defmt::assert_eq!(
            logging::LOG_STATS.oversize.load(Ordering::Relaxed),
            oversize + 1
        );
        JOB.store(0, Ordering::Relaxed);
        let full = logging::LOG_STATS.queue_full.load(Ordering::Relaxed);
        for _ in 0..256 {
            trigger(interrupt::ADC);
        }
        defmt::assert!(logging::LOG_STATS.queue_full.load(Ordering::Relaxed) > full);
        Timer::after(Duration::from_millis(40)).await;

        let queued = logging::LOG_STATS.adc_queued.load(Ordering::Relaxed);
        let contention = logging::LOG_STATS.contention.load(Ordering::Relaxed);
        defmt::warn!("LOGGER outer preempted {}", Interrupting);
        defmt::assert_eq!(
            logging::LOG_STATS.adc_queued.load(Ordering::Relaxed),
            queued + 1
        );
        defmt::assert_eq!(
            logging::LOG_STATS.contention.load(Ordering::Relaxed),
            contention + 1
        );
        Timer::after(Duration::from_millis(10)).await;
        defmt::assert_eq!(cortex_m::register::primask::read_raw(), 0);
        // Fatal logging must bypass the ADC queue, including a panic in ADC.
        let queued = logging::LOG_STATS.adc_queued.load(Ordering::Relaxed);
        let written = logging::LOG_STATS.written.load(Ordering::Relaxed);
        defmt::warn!("LOGGER abandoned frame {}", FatalDuringFormat);
        defmt::assert!(logging::LOG_STATS.written.load(Ordering::Relaxed) > written);
        trigger(interrupt::ADC);
        defmt::assert_eq!(
            logging::LOG_STATS.adc_queued.load(Ordering::Relaxed),
            queued
        );
        defmt::assert!(logging::LOG_STATS.written.load(Ordering::Relaxed) > written);
        defmt::info!(
            "LOGGER PASS queued={} full={} oversize={} contention={} rtt_full={} written={}",
            logging::LOG_STATS.adc_queued.load(Ordering::Relaxed),
            logging::LOG_STATS.queue_full.load(Ordering::Relaxed),
            logging::LOG_STATS.oversize.load(Ordering::Relaxed),
            logging::LOG_STATS.contention.load(Ordering::Relaxed),
            logging::LOG_STATS.rtt_full.load(Ordering::Relaxed),
            logging::LOG_STATS.written.load(Ordering::Relaxed)
        );
    }
}
