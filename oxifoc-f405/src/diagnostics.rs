//! Default-off bench diagnostics. No per-event logging or USB traffic.

/// Low-rate observations tied to the same ADC sequence as fast telemetry.
/// No GPIO/USB logging is added to the control or Hall capture interrupts.
#[cfg(feature = "bench-hall")]
#[embassy_executor::task]
pub async fn hall_task() {
    use core::sync::atomic::Ordering;
    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_millis(20));
    loop {
        ticker.next().await;
        let snapshot = critical_section::with(|cs| {
            let state = crate::STATE.borrow(cs).borrow();
            (state.last_adc.seq, state.last_hall, state.control_mode)
        });
        if let (seq, Some(hall), mode) = snapshot
            && mode != oxifoc_core::motor::ControlMode::Stopped
        {
            defmt::info!(
                "hall: seq={} raw={} logical={} angle={} omega={} errors={} over={}",
                seq,
                crate::sensors::hall::read_hall_state_raw(),
                hall.state,
                hall.angle_rad,
                hall.velocity_rad_s,
                hall.error_count,
                crate::sensors::hall::OVERCAPTURES.load(Ordering::Relaxed),
            );
        }
    }
}

#[cfg(feature = "bench-clock-uart")]
embassy_stm32::bind_interrupts!(struct ClockUartIrqs {
    DMA1_STREAM3 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH3>;
});

/// Hardware clocks every bit; CPU/USB scheduling can only insert inter-burst gaps.
#[cfg(feature = "bench-clock-uart")]
#[embassy_executor::task]
pub async fn uart_clock_task(
    usart: embassy_stm32::Peri<'static, embassy_stm32::peripherals::USART3>,
    tx: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PB10>,
    dma: embassy_stm32::Peri<'static, embassy_stm32::peripherals::DMA1_CH3>,
) {
    use embassy_stm32::{
        interrupt::{InterruptExt, Priority},
        usart::{Config, UartTx},
    };
    embassy_stm32::interrupt::DMA1_STREAM3.set_priority(Priority::P6);
    let mut config = Config::default();
    config.baudrate = 100_000;
    let mut uart = defmt::unwrap!(UartTx::new(usart, tx, dma, ClockUartIrqs, config));
    defmt::info!(
        "uartclk: baud=100000 pclk={} brr={} hse={} pll_n={}",
        embassy_stm32::rcc::frequency::<embassy_stm32::peripherals::USART3>().0,
        embassy_stm32::pac::USART3.brr().read().0,
        crate::config::HSE_HZ,
        crate::config::PLL_N
    );
    static PATTERN: [u8; 4096] = [0x55; 4096];
    loop {
        if uart.write(&PATTERN).await.is_err() {
            defmt::error!("UART clock DMA write failed");
        }
        embassy_time::Timer::after_millis(20).await;
    }
}

#[cfg(any(feature = "bench-executor-trace", feature = "bench-pwm-commit"))]
pub mod trace {
    use core::sync::atomic::{AtomicU32, Ordering};

    // 8/9 already bracket ADC. These IRQ spans include preemption;
    // the offline analyzer unions nested USB spans and subtracts ADC.
    #[cfg(feature = "bench-executor-trace")]
    pub const USB_EXECUTOR_BEGIN: u32 = 10;
    #[cfg(feature = "bench-executor-trace")]
    pub const USB_EXECUTOR_END: u32 = 11;
    #[cfg(feature = "bench-executor-trace")]
    const USB_IRQ_BEGIN: u32 = 12;
    #[cfg(feature = "bench-executor-trace")]
    const USB_IRQ_END: u32 = 13;
    const PWM_WRITE: u32 = 14;

    /// Before in low 16 bits, after in high 16 bits; each packs CNT[14:0]
    /// and DIR at bit 15. The caller records only after completing its writes.
    #[inline]
    pub fn record_pwm_write(before: u16, after: u16) {
        record(PWM_WRITE, u32::from(before) | (u32::from(after) << 16));
    }

    #[cfg(feature = "bench-executor-trace")]
    pub struct UsbIrqBegin;
    #[cfg(feature = "bench-executor-trace")]
    pub struct UsbIrqEnd;

    #[cfg(feature = "bench-executor-trace")]
    impl embassy_stm32::interrupt::typelevel::Handler<embassy_stm32::interrupt::typelevel::OTG_FS>
        for UsbIrqBegin
    {
        unsafe fn on_interrupt() {
            record(USB_IRQ_BEGIN, 0);
        }
    }

    #[cfg(feature = "bench-executor-trace")]
    impl embassy_stm32::interrupt::typelevel::Handler<embassy_stm32::interrupt::typelevel::OTG_FS>
        for UsbIrqEnd
    {
        unsafe fn on_interrupt() {
            record(USB_IRQ_END, 0);
        }
    }

    // 33 KiB of events + 1 KiB of whole-profile task statistics. A short CS
    // serializes each timeline event, including its timestamp, so preemption
    // cannot reorder slots. This bench-only instrumentation briefly delays ADC;
    // its cost remains part of the measured workload.
    // SWD writes CONTROL=1 to request a finite capture after 100 ms. This
    // excludes attach/detach halts from its window. 2 means recording,
    // 3 means waiting, 4 means initializing, 0 means frozen. Read after frozen and
    // outstanding IRQs have returned. Reaching capacity is normal, not a wrap.
    const CAPACITY: usize = 2816;
    #[repr(C)]
    pub struct Event {
        timer_ticks: AtomicU32,
        id: AtomicU32,
        kind: AtomicU32,
    }
    impl Event {
        const fn new() -> Self {
            Self {
                timer_ticks: AtomicU32::new(0),
                id: AtomicU32::new(0),
                kind: AtomicU32::new(0),
            }
        }
    }
    #[unsafe(no_mangle)]
    pub static BENCH_TRACE_CONTROL: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static BENCH_TRACE_COUNT: AtomicU32 = AtomicU32::new(0);
    /// Additional delay after active FOC, set before arming over SWD.
    #[unsafe(no_mangle)]
    pub static BENCH_TRACE_ACTIVE_DELAY: AtomicU32 = AtomicU32::new(0);
    static DELAY_TICKS: AtomicU32 = AtomicU32::new(0);
    static START_TICKS: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static BENCH_TRACE_EVENTS: [Event; CAPACITY] = [const { Event::new() }; CAPACITY];

    // Separate from the finite timeline: count task progress throughout
    // an active profile, including low-rate background tasks that
    // need not appear in a 30 ms trace window. Freeze on the first Stop.
    // CONTROL: 0 disabled, 5 arm on active, 4 resetting, 2 recording, 3 frozen.
    #[unsafe(no_mangle)]
    pub static BENCH_TASK_STATS_CONTROL: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static BENCH_TASK_STATS_START: AtomicU32 = AtomicU32::new(0);
    #[unsafe(no_mangle)]
    pub static BENCH_TASK_STATS_END: AtomicU32 = AtomicU32::new(0);
    #[repr(C)]
    pub struct TaskStats {
        id: AtomicU32,
        polls: AtomicU32,
        readies: AtomicU32,
        first_poll_tick: AtomicU32,
        last_poll_tick: AtomicU32,
        last_ready_tick: AtomicU32,
        max_poll_gap: AtomicU32,
    }
    impl TaskStats {
        const fn new() -> Self {
            Self {
                id: AtomicU32::new(0),
                polls: AtomicU32::new(0),
                readies: AtomicU32::new(0),
                first_poll_tick: AtomicU32::new(0),
                last_poll_tick: AtomicU32::new(0),
                last_ready_tick: AtomicU32::new(0),
                max_poll_gap: AtomicU32::new(0),
            }
        }
        fn reset(&self) {
            self.polls.store(0, Ordering::Relaxed);
            self.readies.store(0, Ordering::Relaxed);
            self.first_poll_tick.store(0, Ordering::Relaxed);
            self.last_poll_tick.store(0, Ordering::Relaxed);
            self.last_ready_tick.store(0, Ordering::Relaxed);
            self.max_poll_gap.store(0, Ordering::Relaxed);
        }
    }
    #[unsafe(no_mangle)]
    pub static BENCH_TASK_STATS: [TaskStats; 32] = [const { TaskStats::new() }; 32];
    #[unsafe(no_mangle)]
    pub static BENCH_TASK_STATS_OVERFLOW: AtomicU32 = AtomicU32::new(0);

    #[cfg(feature = "bench-executor-trace")]
    fn register_task(task: u32) {
        for slot in &BENCH_TASK_STATS {
            if slot.id.load(Ordering::Relaxed) == task
                || slot
                    .id
                    .compare_exchange(0, task, Ordering::Relaxed, Ordering::Relaxed)
                    .is_ok()
            {
                return;
            }
        }
        BENCH_TASK_STATS_OVERFLOW.fetch_add(1, Ordering::Relaxed);
    }

    #[cfg(feature = "bench-executor-trace")]
    fn task_event(task: u32, kind: u32) {
        if BENCH_TASK_STATS_CONTROL.load(Ordering::Relaxed) != 2 {
            return;
        }
        let Some(slot) = BENCH_TASK_STATS
            .iter()
            .find(|s| s.id.load(Ordering::Relaxed) == task)
        else {
            return;
        };
        let now = embassy_stm32::pac::TIM5.cnt().read();
        // A Stop IRQ can preempt this hook. Discard a hook spanning that
        // boundary, instead of timestamping it after the frozen interval.
        if BENCH_TASK_STATS_CONTROL.load(Ordering::Relaxed) != 2 {
            return;
        }
        match kind {
            6 => {
                slot.last_ready_tick.store(now, Ordering::Relaxed);
                slot.readies.fetch_add(1, Ordering::Relaxed);
            }
            4 => {
                // Only exec-begin writes poll timestamps for this task. Ready
                // hooks may preempt it but update separate fields; no extra
                // critical section or interrupt masking is introduced here.
                let previous = slot.last_poll_tick.swap(now, Ordering::Relaxed);
                if slot.polls.fetch_add(1, Ordering::Relaxed) == 0 {
                    slot.first_poll_tick.store(now, Ordering::Relaxed);
                } else {
                    slot.max_poll_gap
                        .fetch_max(now.wrapping_sub(previous), Ordering::Relaxed);
                }
            }
            _ => {}
        }
    }

    pub fn trigger_stopped() {
        if BENCH_TASK_STATS_CONTROL.load(Ordering::Relaxed) == 2 {
            BENCH_TASK_STATS_END.store(embassy_stm32::pac::TIM5.cnt().read(), Ordering::Relaxed);
            BENCH_TASK_STATS_CONTROL.store(3, Ordering::Relaxed);
        }
    }

    // CONTROL=5 waits for active FOC. SWD can finish long before the workload
    // starts, avoiding debug halts in the experiment's active interval.
    pub fn trigger_active() {
        if BENCH_TASK_STATS_CONTROL
            .compare_exchange(5, 4, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
        {
            // Called in ADC IRQ. Other task hooks cannot run during reset.
            for slot in &BENCH_TASK_STATS {
                slot.reset();
            }
            BENCH_TASK_STATS_START.store(embassy_stm32::pac::TIM5.cnt().read(), Ordering::Relaxed);
            BENCH_TASK_STATS_CONTROL.store(2, Ordering::Relaxed);
        }
        if BENCH_TRACE_CONTROL
            .compare_exchange(5, 4, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
        {
            BENCH_TRACE_COUNT.store(0, Ordering::Relaxed);
            let delay = BENCH_TRACE_ACTIVE_DELAY.load(Ordering::Relaxed);
            START_TICKS.store(embassy_stm32::pac::TIM5.cnt().read(), Ordering::Relaxed);
            DELAY_TICKS.store(delay, Ordering::Relaxed);
            BENCH_TRACE_CONTROL.store(if delay == 0 { 2 } else { 3 }, Ordering::Relaxed);
        }
    }

    #[inline]
    pub fn record(kind: u32, id: u32) {
        match BENCH_TRACE_CONTROL.load(Ordering::Relaxed) {
            2 => {}
            1 => {
                if BENCH_TRACE_CONTROL
                    .compare_exchange(1, 4, Ordering::Relaxed, Ordering::Relaxed)
                    .is_ok()
                {
                    START_TICKS.store(embassy_stm32::pac::TIM5.cnt().read(), Ordering::Relaxed);
                    DELAY_TICKS.store(crate::config::CPU_HZ / 20, Ordering::Relaxed);
                    BENCH_TRACE_CONTROL.store(3, Ordering::Relaxed);
                }
                return;
            }
            3 => {
                if embassy_stm32::pac::TIM5
                    .cnt()
                    .read()
                    .wrapping_sub(START_TICKS.load(Ordering::Relaxed))
                    < DELAY_TICKS.load(Ordering::Relaxed)
                    || BENCH_TRACE_CONTROL
                        .compare_exchange(3, 4, Ordering::Relaxed, Ordering::Relaxed)
                        .is_err()
                {
                    return;
                }
                // State 4 owns initialization; nested hooks return without
                // touching the buffer. Keep this work outside the append CS.
                BENCH_TRACE_COUNT.store(0, Ordering::Relaxed);
                BENCH_TRACE_CONTROL.store(2, Ordering::Relaxed);
            }
            _ => return,
        }
        critical_section::with(|_| {
            // A higher-priority writer may have filled/frozen the trace after
            // the fast-path load. Never append to that completed generation.
            if BENCH_TRACE_CONTROL.load(Ordering::Relaxed) != 2 {
                return;
            }
            // probe-rs can freeze DWT on detach; TIM5 remains independent.
            // Timestamp, reservation and payload are one bounded operation.
            let ticks = embassy_stm32::pac::TIM5.cnt().read();
            let index = BENCH_TRACE_COUNT.load(Ordering::Relaxed);
            // All active writers share this CS; no LDREX/STREX retry is needed.
            BENCH_TRACE_COUNT.store(index.wrapping_add(1), Ordering::Relaxed);
            if let Some(event) = BENCH_TRACE_EVENTS.get(index as usize) {
                event.timer_ticks.store(ticks, Ordering::Relaxed);
                event.id.store(id, Ordering::Relaxed);
                event.kind.store(kind, Ordering::Relaxed);
            } else {
                BENCH_TRACE_CONTROL.store(0, Ordering::Relaxed);
            }
        });
    }

    // Task IDs are resolved against this exact ELF's TaskStorage/POOL symbols.
    // Executor events carry executor IDs; task events carry task IDs.
    #[cfg(feature = "bench-executor-trace")]
    #[unsafe(no_mangle)]
    fn _embassy_trace_poll_start(executor: u32) {
        record(1, executor);
    }
    #[cfg(feature = "bench-executor-trace")]
    #[unsafe(no_mangle)]
    fn _embassy_trace_task_new(_executor: u32, task: u32) {
        register_task(task);
        record(2, task);
    }
    #[cfg(feature = "bench-executor-trace")]
    #[unsafe(no_mangle)]
    fn _embassy_trace_task_end(_executor: u32, task: u32) {
        record(3, task);
    }
    #[cfg(feature = "bench-executor-trace")]
    #[unsafe(no_mangle)]
    fn _embassy_trace_task_exec_begin(_executor: u32, task: u32) {
        task_event(task, 4);
        record(4, task);
    }
    #[cfg(feature = "bench-executor-trace")]
    #[unsafe(no_mangle)]
    fn _embassy_trace_task_exec_end(_executor: u32, task: u32) {
        record(5, task);
    }
    #[cfg(feature = "bench-executor-trace")]
    #[unsafe(no_mangle)]
    fn _embassy_trace_task_ready_begin(_executor: u32, task: u32) {
        #[cfg(feature = "bench-scheduler-deadline")]
        super::deadline::ready(task);
        task_event(task, 6);
        record(6, task);
    }
    #[cfg(feature = "bench-executor-trace")]
    #[unsafe(no_mangle)]
    fn _embassy_trace_executor_idle(executor: u32) {
        record(7, executor);
    }
}

/// Experimental release-relative EDF policy. It only affects queue ordering;
/// it cannot interrupt an in-progress poll. No changes to Ergot are needed.
#[cfg(feature = "bench-scheduler-deadline")]
pub mod deadline {
    use core::{
        future::poll_fn,
        sync::atomic::{AtomicPtr, AtomicU32, Ordering},
        task::Poll,
    };
    use embassy_executor::Metadata;

    struct Slot {
        id: AtomicU32,
        metadata: AtomicPtr<Metadata>,
        budget: AtomicU32,
    }
    static SLOTS: [Slot; 5] = [const {
        Slot {
            id: AtomicU32::new(0),
            metadata: AtomicPtr::new(core::ptr::null_mut()),
            budget: AtomicU32::new(0),
        }
    }; 5];

    pub async fn register(index: usize, budget_us: u64) {
        let metadata = Metadata::for_current_task().await;
        let id = poll_fn(|cx| Poll::Ready(embassy_executor::raw::task_from_waker(cx.waker()).id()))
            .await;
        let slot = &SLOTS[index];
        slot.budget.store(
            embassy_time::Duration::from_micros(budget_us).as_ticks() as u32,
            Ordering::Relaxed,
        );
        slot.metadata
            .store(core::ptr::from_ref(metadata).cast_mut(), Ordering::Relaxed);
        slot.id.store(id, Ordering::Release);
        ready(id);
    }

    pub fn ready(task: u32) {
        for slot in &SLOTS {
            if slot.id.load(Ordering::Acquire) == task {
                // SAFETY: published once by register from Embassy's static
                // Metadata reference. Tasks in this bench never terminate.
                // The ready hook runs before enqueue; the task is not already
                // in the sorted run queue, so its ordering key is stable there.
                let metadata = unsafe { &*slot.metadata.load(Ordering::Relaxed) };
                metadata.set_deadline(
                    embassy_time::Instant::now()
                        .as_ticks()
                        .saturating_add(u64::from(slot.budget.load(Ordering::Relaxed))),
                );
                return;
            }
        }
    }
}

/// TIM5 runs freely from APB1's timer clock, including while the core sleeps.
/// It has no outputs, IRQs, DMA, or relation to the PWM/ADC triggers.
#[cfg(feature = "bench-clock-reference")]
#[embassy_executor::task]
pub async fn clock_reference_task(
    tim: embassy_stm32::Peri<'static, embassy_stm32::peripherals::TIM5>,
) {
    use embassy_stm32::{pac, timer::low_level::Timer};
    use embassy_time::{Duration, Ticker};
    let timer = Timer::new(tim);
    timer.set_max_compare_value(u32::MAX);
    timer.start();
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut previous = None;
    let mut totals = (0u64, 0u64, 0u64);
    loop {
        ticker.next().await;
        let frame = pac::USB_OTG_FS.dsts().read().fnsof() & 0x7ff;
        let Some(clock) = crate::control::foc::dwt_clock_sample() else {
            previous = None;
            totals = (0, 0, 0);
            continue;
        };
        let ticks = timer.get_counter();
        if frame != pac::USB_OTG_FS.dsts().read().fnsof() & 0x7ff {
            continue;
        }
        if let Some((prev_frame, prev_clock, prev_ticks)) = previous {
            let frames = frame.wrapping_sub(prev_frame) & 0x7ff;
            let dt = ticks.wrapping_sub(prev_ticks);
            if let Some(dc) = clock.elapsed_since(prev_clock)
                && (900..=1100).contains(&frames)
                && (70_000_000..=100_000_000).contains(&dt)
            {
                totals.0 += u64::from(frames);
                totals.1 += u64::from(dc.get());
                totals.2 += u64::from(dt);
                defmt::info!(
                    "clockref: frames={} dwt={} tim5={} total_frames={} total_dwt={} total_tim5={}",
                    frames,
                    dc.get(),
                    dt,
                    totals.0,
                    totals.1,
                    totals.2
                );
            } else {
                totals = (0, 0, 0);
            }
        }
        previous = Some((frame, clock, ticks));
    }
}
