//! Default-off bench diagnostics. No per-event logging or USB traffic.

#[cfg(feature = "bench-foc-deadline")]
pub mod pwm_deadline;

#[cfg(feature = "bench-foc-distribution")]
pub mod foc_distribution;

#[cfg(feature = "bench-overclock")]
pub mod clock_stress;

#[cfg(feature = "bench-raw-codec")]
pub mod codec_bench;

#[cfg(feature = "bench-cycle-outliers")]
pub mod cycle_outliers;

#[cfg(feature = "bench-hall")]
pub mod hall_edges;

/// Low-rate observations tied to the same ADC sequence as fast telemetry.
/// No GPIO/USB logging is added to the control or Hall capture interrupts.
#[cfg(feature = "bench-hall")]
#[embassy_executor::task]
pub async fn hall_task() {
    use core::sync::atomic::Ordering;
    loop {
        // These are fresh observations, not a history buffer. If streaming
        // delays this task, skip missed observations instead of logging a
        // burst of nearly identical snapshots while catching up.
        embassy_time::Timer::after_millis(20).await;
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

#[cfg(any(
    feature = "bench-executor-trace",
    feature = "bench-pwm-commit",
    feature = "bench-netstack-trace"
))]
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
    #[cfg(feature = "bench-netstack-trace")]
    pub const NETSTACK_LOCK_BEGIN: u32 = 15;
    #[cfg(feature = "bench-netstack-trace")]
    pub const NETSTACK_LOCK_END: u32 = 16;
    #[cfg(feature = "bench-netstack-trace")]
    pub const ADC_ENTRY_PHASE: u32 = 17;

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

    // Other IRQs that can preempt the P6 USB executor must not be charged
    // to USB. TIM2/RNG are also P6 and cannot preempt it.
    #[cfg(feature = "bench-executor-trace")]
    macro_rules! irq_pair {
        ($begin:ident, $end:ident, $irq:ident, $kind:literal) => {
            pub struct $begin;
            pub struct $end;
            impl
                embassy_stm32::interrupt::typelevel::Handler<
                    embassy_stm32::interrupt::typelevel::$irq,
                > for $begin
            {
                unsafe fn on_interrupt() {
                    record($kind, 0);
                }
            }
            impl
                embassy_stm32::interrupt::typelevel::Handler<
                    embassy_stm32::interrupt::typelevel::$irq,
                > for $end
            {
                unsafe fn on_interrupt() {
                    record($kind + 1, 0);
                }
            }
        };
    }
    #[cfg(feature = "bench-executor-trace")]
    irq_pair!(FaultIrqBegin, FaultIrqEnd, EXTI9_5, 23);
    #[cfg(all(feature = "bench-executor-trace", feature = "transport-spi"))]
    irq_pair!(SpiRxIrqBegin, SpiRxIrqEnd, DMA2_STREAM0, 25);
    #[cfg(all(feature = "bench-executor-trace", feature = "transport-spi"))]
    irq_pair!(SpiTxIrqBegin, SpiTxIrqEnd, DMA2_STREAM3, 27);

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

    /// Whether the finite trace is accepting events. ADC probes must remain
    /// inactive while armed, delayed or frozen; network events advance those
    /// states without adding MMIO work to every control cycle.
    #[cfg(feature = "bench-netstack-trace")]
    #[inline(always)]
    pub fn is_recording() -> bool {
        BENCH_TRACE_CONTROL.load(Ordering::Relaxed) == 2
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

    #[cfg(feature = "bench-executor-trace")]
    use embassy_executor::{
        ExecutorId,
        raw::{TaskRef, trace::Trace},
    };

    #[cfg(feature = "bench-executor-trace")]
    pub struct ExecutorTrace;

    #[cfg(feature = "bench-executor-trace")]
    impl Trace for ExecutorTrace {
        fn poll_start(executor: ExecutorId) {
            record(1, executor.get() as u32);
        }

        fn task_new(_: ExecutorId, task: TaskRef) {
            let id = task.id().get() as u32;
            register_task(id);
            record(2, id);
        }

        fn task_end(_: ExecutorId, task: TaskRef) {
            record(3, task.id().get() as u32);
        }

        fn task_exec_begin(_: ExecutorId, task: TaskRef) {
            let id = task.id().get() as u32;
            task_event(id, 4);
            record(4, id);
        }

        fn task_exec_end(_: ExecutorId, task: TaskRef) {
            record(5, task.id().get() as u32);
        }

        fn task_ready_begin(_: ExecutorId, task: TaskRef) {
            #[cfg(feature = "bench-scheduler-deadline")]
            super::deadline::ready(task);
            let id = task.id().get() as u32;
            task_event(id, 6);
            record(6, id);
        }

        fn executor_idle(executor: ExecutorId) {
            record(7, executor.get() as u32);
        }

        fn idle() {}
        fn task_name_set(_: TaskRef, _: &'static str) {}
        fn task_priority_set(_: TaskRef, _: u8) {}
        fn task_deadline_set(_: TaskRef, _: u64) {}
    }

    #[cfg(feature = "bench-executor-trace")]
    embassy_executor::trace_impl!(ExecutorTrace);
}

/// Experimental release-relative EDF policy. It only affects queue ordering;
/// it cannot interrupt an in-progress poll. No changes to Ergot are needed.
#[cfg(feature = "bench-scheduler-deadline")]
pub mod deadline {
    use core::{
        future::poll_fn,
        sync::atomic::{AtomicU32, Ordering},
        task::Poll,
    };
    use embassy_executor::{MetadataRef, raw::TaskRef};

    struct Slot {
        id: AtomicU32,
        budget: AtomicU32,
    }
    static SLOTS: [Slot; 5] = [const {
        Slot {
            id: AtomicU32::new(0),
            budget: AtomicU32::new(0),
        }
    }; 5];

    pub async fn register(index: usize, budget_us: u64) {
        let task =
            poll_fn(|cx| Poll::Ready(embassy_executor::raw::task_from_waker(cx.waker()))).await;
        let slot = &SLOTS[index];
        slot.budget.store(
            embassy_time::Duration::from_micros(budget_us).as_ticks() as u32,
            Ordering::Relaxed,
        );
        slot.id.store(task.id().get() as u32, Ordering::Release);
        ready(task);
    }

    pub fn ready(task: TaskRef) {
        let id = task.id().get() as u32;
        for slot in &SLOTS {
            if slot.id.load(Ordering::Acquire) == id {
                // The ready hook runs before enqueue, when its ordering key can change.
                MetadataRef::new(task).set_deadline(
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
#[cfg(any(
    feature = "bench-clock-reference",
    feature = "bench-executor-trace",
    feature = "bench-foc-deadline"
))]
pub fn start_trace_clock(
    tim: embassy_stm32::Peri<'static, embassy_stm32::peripherals::TIM5>,
) -> &'static embassy_stm32::timer::low_level::Timer<'static, embassy_stm32::peripherals::TIM5> {
    // Main finishes after spawning workers. Keep the owning driver alive so
    // its Drop cannot gate TIM5's clock when main returns.
    static CLOCK: static_cell::StaticCell<
        embassy_stm32::timer::low_level::Timer<'static, embassy_stm32::peripherals::TIM5>,
    > = static_cell::StaticCell::new();
    let timer = CLOCK.init(embassy_stm32::timer::low_level::Timer::new(tim));
    timer.set_max_compare_value(u32::MAX);
    timer.start();
    timer
}

#[cfg(feature = "bench-clock-reference")]
#[embassy_executor::task]
pub async fn clock_reference_task(
    tim: embassy_stm32::Peri<'static, embassy_stm32::peripherals::TIM5>,
) {
    use embassy_stm32::pac;
    use embassy_time::{Duration, Ticker};
    let timer = start_trace_clock(tim);
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
