//! Single-flight model application with durable ISR-side config commit.
//!
//! Slot ownership is transferred with release/acquire atomics. The producer
//! writes only Preparing; the ISR borrows only Applying; cleanup owns Reading.
//! The slot never adds a critical section around estimator payload moves.
//! Platforms may already hold their driver-ownership CS around the whole ISR;
//! the one-shot stopped install must be included in that platform timing bound.

use super::{CMD_CHANNEL, DriverCommand};
use crate::{
    motor::foc_driver::PreparedMotorModel,
    storage::{MotorParamsConfig, RuntimeConfig, VoltageFeedforwardConfig},
    types::{ConfigGroupId, ConfigResponse, ReqId},
};
use core::{
    cell::{RefCell, UnsafeCell},
    mem::MaybeUninit,
    sync::atomic::{AtomicU32, Ordering},
};
use critical_section::Mutex;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

#[cfg(feature = "storage")]
#[derive(Clone, Copy)]
pub(crate) enum CachedConfigAck {
    Applied(u32),
    Persisted(u32),
}

#[cfg(feature = "storage")]
pub(crate) struct ConfigActionCache {
    entries: [Option<(ReqId, CachedConfigAck)>; 4],
    next: usize,
}

#[cfg(feature = "storage")]
impl ConfigActionCache {
    pub(crate) const fn new() -> Self {
        Self {
            entries: [None; 4],
            next: 0,
        }
    }

    pub(crate) fn get(&self, id: ReqId) -> Option<CachedConfigAck> {
        self.entries
            .iter()
            .flatten()
            .find_map(|(cached, ack)| (*cached == id).then_some(*ack))
    }

    pub(crate) fn insert(&mut self, id: ReqId, ack: CachedConfigAck) {
        self.entries[self.next] = Some((id, ack));
        self.next = (self.next + 1) % self.entries.len();
    }
}

const EMPTY: u32 = 0;
const PREPARING: u32 = 1;
const READY: u32 = 2;
const APPLYING: u32 = 3;
const FINISHED: u32 = 4;
const READING: u32 = 5;
const APPLYING_CANCELLED: u32 = 6;

pub(crate) enum ModelConfigUpdate {
    MotorParams(MotorParamsConfig),
    VoltageFeedforward(VoltageFeedforwardConfig),
    Reset,
}

impl From<MotorParamsConfig> for ModelConfigUpdate {
    fn from(value: MotorParamsConfig) -> Self {
        Self::MotorParams(value)
    }
}

impl ModelConfigUpdate {
    pub(crate) fn apply_to_config(&self, config: &mut RuntimeConfig) {
        match self {
            Self::MotorParams(value) => config.motor_params = Some(value.clone()),
            Self::VoltageFeedforward(value) => config.voltage_feedforward = Some(*value),
            Self::Reset => *config = RuntimeConfig::default(),
        }
    }

    fn group(&self) -> ConfigGroupId {
        match self {
            Self::MotorParams(_) | Self::Reset => ConfigGroupId::MotorParams,
            Self::VoltageFeedforward(_) => ConfigGroupId::VoltageFeedforward,
        }
    }
}

struct Pending {
    model: PreparedMotorModel,
    update: ModelConfigUpdate,
    persisted: Option<&'static Mutex<RefCell<[Option<u32>; ConfigGroupId::COUNT]>>>,
    config: &'static Mutex<RefCell<RuntimeConfig>>,
    revisions: &'static Mutex<RefCell<[u32; ConfigGroupId::COUNT]>>,
    cache: &'static Mutex<RefCell<ConfigActionCache>>,
    request: ReqId,
    expected_revision: u32,
    response: Option<ConfigResponse>,
}

// Cortex-M4 has native 32-bit atomics, so tag state and generation together.
// A stale command/cleanup can never claim a replacement slot after an ABA.
const STATE_BITS: u32 = 3;
const STATE_MASK: u32 = (1 << STATE_BITS) - 1;
const GENERATION_MASK: u32 = u32::MAX >> STATE_BITS;
const fn ticket(generation: u32, state: u32) -> u32 {
    (generation << STATE_BITS) | state
}

struct ModelSlot {
    ticket: AtomicU32,
    payload: UnsafeCell<MaybeUninit<Pending>>,
    done: Signal<CriticalSectionRawMutex, u32>,
}

// SAFETY: no payload reference escapes its exclusive atomic state owner. Each
// transition publishes prior writes with Release and claims them with Acquire.
unsafe impl Sync for ModelSlot {}

static SLOT: ModelSlot = ModelSlot::new();

pub fn model_apply_pending() -> bool {
    SLOT.ticket.load(Ordering::Acquire) & STATE_MASK != EMPTY
}

impl ModelSlot {
    const fn new() -> Self {
        Self {
            ticket: AtomicU32::new(EMPTY),
            payload: UnsafeCell::new(MaybeUninit::uninit()),
            done: Signal::new(),
        }
    }

    fn reserve(&self) -> Option<u32> {
        let mut old = self.ticket.load(Ordering::Acquire);
        loop {
            if old & STATE_MASK != EMPTY {
                return None;
            }
            let generation = (((old >> STATE_BITS) + 1) & GENERATION_MASK).max(1);
            match self.ticket.compare_exchange(
                old,
                ticket(generation, PREPARING),
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => return Some(generation),
                Err(current) => old = current,
            }
        }
    }

    fn transition(&self, generation: u32, old: u32, new: u32) -> bool {
        self.ticket
            .compare_exchange(
                ticket(generation, old),
                ticket(generation, new),
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_ok()
    }

    fn claim_ready(&self, generation: u32) -> bool {
        self.transition(generation, READY, APPLYING)
    }

    fn clear_if(&self, generation: u32, previous: u32) {
        if self.transition(generation, previous, READING) {
            // SAFETY: Reading exclusively owns this initialized generation;
            // a producer cannot replace it until the final Empty publication.
            unsafe { self.payload.get().cast::<Pending>().drop_in_place() };
            self.ticket
                .store(ticket(generation, EMPTY), Ordering::Release);
        }
    }

    fn finish(&self, generation: u32) {
        if !self.transition(generation, APPLYING, FINISHED) {
            // ApplyingCancelled still owns the durable commit, then disposal.
            self.clear_if(generation, APPLYING_CANCELLED);
        }
        self.done.signal(generation);
    }

    fn cancel(&self, generation: u32) {
        let mut old = self.ticket.load(Ordering::Acquire);
        loop {
            if old >> STATE_BITS != generation {
                break;
            }
            match old & STATE_MASK {
                READY | FINISHED => {
                    match self.ticket.compare_exchange(
                        old,
                        ticket(generation, READING),
                        Ordering::AcqRel,
                        Ordering::Acquire,
                    ) {
                        Ok(_) => {
                            // SAFETY: Reading exclusively owns this generation.
                            unsafe { self.payload.get().cast::<Pending>().drop_in_place() };
                            self.ticket
                                .store(ticket(generation, EMPTY), Ordering::Release);
                            break;
                        }
                        Err(current) => old = current,
                    }
                }
                APPLYING => {
                    match self.ticket.compare_exchange(
                        old,
                        ticket(generation, APPLYING_CANCELLED),
                        Ordering::AcqRel,
                        Ordering::Acquire,
                    ) {
                        Ok(_) => break,
                        Err(current) => old = current,
                    }
                }
                _ => break, // ApplyingCancelled owns durable completion and cleanup.
            }
        }
    }
}

/// Cancellation removes an unclaimed request. A claimed request commits its
/// mirror/revision/cache before completion, even if its async waiter disappears.
pub(crate) struct ModelApplyGuard {
    generation: u32,
    active: bool,
}

impl ModelApplyGuard {
    #[allow(clippy::too_many_arguments)]
    pub fn prepare(
        model: PreparedMotorModel,
        value: impl Into<ModelConfigUpdate>,
        config: &'static Mutex<RefCell<RuntimeConfig>>,
        revisions: &'static Mutex<RefCell<[u32; ConfigGroupId::COUNT]>>,
        cache: &'static Mutex<RefCell<ConfigActionCache>>,
        request: ReqId,
        expected_revision: u32,
    ) -> Option<Self> {
        Self::prepare_inner(
            model,
            value.into(),
            None,
            config,
            revisions,
            cache,
            request,
            expected_revision,
        )
    }

    pub fn prepare_reset(
        model: PreparedMotorModel,
        config: &'static Mutex<RefCell<RuntimeConfig>>,
        revisions: &'static Mutex<RefCell<[u32; ConfigGroupId::COUNT]>>,
        persisted: &'static Mutex<RefCell<[Option<u32>; ConfigGroupId::COUNT]>>,
        cache: &'static Mutex<RefCell<ConfigActionCache>>,
        expected_revision: u32,
    ) -> Option<Self> {
        Self::prepare_inner(
            model,
            ModelConfigUpdate::Reset,
            Some(persisted),
            config,
            revisions,
            cache,
            ReqId(0),
            expected_revision,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn prepare_inner(
        model: PreparedMotorModel,
        update: ModelConfigUpdate,
        persisted: Option<&'static Mutex<RefCell<[Option<u32>; ConfigGroupId::COUNT]>>>,
        config: &'static Mutex<RefCell<RuntimeConfig>>,
        revisions: &'static Mutex<RefCell<[u32; ConfigGroupId::COUNT]>>,
        cache: &'static Mutex<RefCell<ConfigActionCache>>,
        request: ReqId,
        expected_revision: u32,
    ) -> Option<Self> {
        let generation = SLOT.reserve()?;
        SLOT.done.reset();
        // SAFETY: Preparing exclusively owns the uninitialized slot.
        unsafe {
            SLOT.payload.get().write(MaybeUninit::new(Pending {
                model,
                update,
                persisted,
                config,
                revisions,
                cache,
                request,
                expected_revision,
                response: None,
            }));
        };
        SLOT.ticket
            .store(ticket(generation, READY), Ordering::Release);
        Some(Self {
            generation,
            active: true,
        })
    }

    pub async fn apply<T: crate::timer::Timer>(mut self) -> ConfigResponse {
        let confirm = async {
            CMD_CHANNEL
                .send(DriverCommand::ApplyMotorModel(self.generation))
                .await;
            while SLOT.done.wait().await != self.generation {}
        };
        // Timeout covers both a full command channel and a stopped ADC loop.
        // A completed durable transaction wins even if the timer was also ready.
        let _ = embassy_futures::select::select(confirm, T::after_millis(1_000)).await;
        if !SLOT.transition(self.generation, FINISHED, READING) {
            return ConfigResponse::Error;
        }
        // SAFETY: Reading exclusively owns the initialized completion.
        let pending = unsafe { &mut *SLOT.payload.get().cast::<Pending>() };
        let response = pending.response.take().unwrap_or(ConfigResponse::Error);
        // SAFETY: Reading still exclusively owns the slot. Drop in place so
        // retrieving a scalar completion does not copy the estimator payload.
        unsafe { SLOT.payload.get().cast::<Pending>().drop_in_place() };
        self.active = false;
        SLOT.ticket
            .store(ticket(self.generation, EMPTY), Ordering::Release);
        response
    }
}

impl Drop for ModelApplyGuard {
    fn drop(&mut self) {
        if self.active {
            SLOT.cancel(self.generation);
        }
    }
}

/// Called only by the driver owner between complete FOC cycles. Heavy prepared
/// state moves add no extra masking; the commit CS copies only the
/// small electrical parameter record and revision/cache metadata.
pub(super) fn apply<P, C, Ph, S>(
    generation: u32,
    foc: &mut crate::motor::FocDriver<P, C, Ph, S>,
    state: &Mutex<RefCell<super::MotorControlState>>,
    allowed: bool,
) where
    P: crate::foc::pwm::PhasePwm,
    C: crate::foc::sensors::CurrentSensor,
    Ph: crate::foc::phase::PhaseProvider,
    S: crate::foc::trig::SinCos,
{
    if !SLOT.claim_ready(generation) {
        return;
    }
    // SAFETY: Applying exclusively owns the payload until Finished is released.
    let pending = unsafe { &mut *SLOT.payload.get().cast::<Pending>() };
    let group = pending.update.group();
    let current_revision =
        critical_section::with(|cs| pending.revisions.borrow(cs).borrow()[group.index()]);
    let response = if current_revision != pending.expected_revision {
        ConfigResponse::Conflict { current_revision }
    } else if !allowed {
        ConfigResponse::Busy
    } else {
        match foc.apply_motor_model(&mut pending.model) {
            Ok(source) => critical_section::with(|cs| {
                if matches!(pending.update, ModelConfigUpdate::Reset) {
                    *pending.config.borrow(cs).borrow_mut() = RuntimeConfig::default();
                    *pending.revisions.borrow(cs).borrow_mut() = [0; ConfigGroupId::COUNT];
                    if let Some(persisted) = pending.persisted {
                        *persisted.borrow(cs).borrow_mut() = [None; ConfigGroupId::COUNT];
                    }
                    *pending.cache.borrow(cs).borrow_mut() = ConfigActionCache::new();
                    let mut state = state.borrow(cs).borrow_mut();
                    state.board_calibration = crate::storage::BoardCalibrationConfig::IDENTITY;
                    state.board_calibration_valid = true;
                    state.motor_pole_pairs = pending.model.pole_pairs;
                    state.phase_source = source;
                    state.motor_model_generation = generation;
                    return ConfigResponse::Reset;
                }
                {
                    let mut config = pending.config.borrow(cs).borrow_mut();
                    pending.update.apply_to_config(&mut config);
                }
                let revision = current_revision.wrapping_add(1);
                pending.revisions.borrow(cs).borrow_mut()[group.index()] = revision;
                pending
                    .cache
                    .borrow(cs)
                    .borrow_mut()
                    .insert(pending.request, CachedConfigAck::Applied(revision));
                let mut state = state.borrow(cs).borrow_mut();
                state.phase_source = source;
                state.motor_pole_pairs = pending.model.pole_pairs;
                state.motor_model_generation = generation;
                ConfigResponse::Applied {
                    req_id: pending.request,
                    revision,
                }
            }),
            Err(crate::motor::foc_driver::MotorModelError::Consumed) => ConfigResponse::Invalid,
            Err(crate::motor::foc_driver::MotorModelError::Busy) => ConfigResponse::Busy,
            Err(crate::motor::foc_driver::MotorModelError::Unsupported) => {
                ConfigResponse::Unsupported
            }
        }
    };
    pending.response = Some(response);
    SLOT.finish(generation);
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{
        boxed::Box,
        sync::{Arc, atomic::AtomicBool},
        thread,
    };

    fn factory() -> impl Fn() -> Pending {
        let config: &'static _ =
            Box::leak(Box::new(Mutex::new(RefCell::new(RuntimeConfig::default()))));
        let revisions: &'static _ = Box::leak(Box::new(Mutex::new(RefCell::new(
            [0; ConfigGroupId::COUNT],
        ))));
        let cache: &'static _ =
            Box::leak(Box::new(Mutex::new(RefCell::new(ConfigActionCache::new()))));
        move || Pending {
            model: PreparedMotorModel::from_config(&RuntimeConfig::default(), 24.0, 3.0),
            update: ModelConfigUpdate::MotorParams(MotorParamsConfig::default()),
            persisted: None,
            config,
            revisions,
            cache,
            request: ReqId(1),
            expected_revision: 0,
            response: None,
        }
    }

    fn publish(slot: &ModelSlot, pending: Pending) -> u32 {
        let generation = slot.reserve().expect("test producer owns empty slot");
        assert!(!slot.claim_ready(generation), "Preparing cannot be claimed");
        // SAFETY: the test producer exclusively owns Preparing.
        unsafe {
            slot.payload.get().write(MaybeUninit::new(pending));
        }
        slot.ticket
            .store(ticket(generation, READY), Ordering::Release);
        generation
    }

    #[test]
    fn model_slot_tagged_cas_rejects_stale_claim_cancel_and_cleanup() {
        let slot = ModelSlot::new();
        let pending = factory();
        let first = publish(&slot, pending());
        slot.cancel(first);
        let second = publish(&slot, pending());
        assert!(!slot.claim_ready(first));
        slot.clear_if(first, FINISHED);
        slot.cancel(first);
        assert_eq!(slot.ticket.load(Ordering::Acquire), ticket(second, READY));
        assert!(slot.claim_ready(second));
        slot.cancel(second);
        slot.cancel(first); // cannot erase second's ApplyingCancelled state
        assert_eq!(
            slot.ticket.load(Ordering::Acquire),
            ticket(second, APPLYING_CANCELLED)
        );
        slot.finish(second);
        assert_eq!(slot.ticket.load(Ordering::Acquire), ticket(second, EMPTY));
        let third = publish(&slot, pending());
        assert!(slot.claim_ready(third));
        slot.finish(third);
        slot.clear_if(first, FINISHED); // old completion cannot dispose third
        assert_eq!(slot.ticket.load(Ordering::Acquire), ticket(third, FINISHED));
        slot.cancel(third);
    }

    #[test]
    fn model_slot_cancellation_and_replacement_are_safe_across_threads() {
        let slot = Arc::new(ModelSlot::new());
        let done = Arc::new(AtomicBool::new(false));
        let worker_slot = slot.clone();
        let worker_done = done.clone();
        let consumer = thread::spawn(move || {
            while !worker_done.load(Ordering::Acquire) {
                let value = worker_slot.ticket.load(Ordering::Acquire);
                if value & STATE_MASK == READY {
                    let generation = value >> STATE_BITS;
                    if worker_slot.claim_ready(generation) {
                        thread::yield_now(); // cancellation can race Applying
                        worker_slot.finish(generation);
                    }
                } else {
                    thread::yield_now();
                }
            }
        });
        let pending = factory();
        for iteration in 0..10_000 {
            let generation = publish(&slot, pending());
            if iteration % 2 == 0 {
                thread::yield_now();
            }
            slot.cancel(generation);
            loop {
                let current = slot.ticket.load(Ordering::Acquire);
                if current & STATE_MASK == EMPTY {
                    break;
                }
                thread::yield_now();
            }
            // Force the old completion/cancel after the next publication.
            let next = publish(&slot, pending());
            slot.cancel(generation);
            slot.clear_if(generation, FINISHED);
            slot.cancel(next);
            while slot.ticket.load(Ordering::Acquire) & STATE_MASK != EMPTY {
                thread::yield_now();
            }
        }
        done.store(true, Ordering::Release);
        consumer.join().unwrap();
    }

    #[test]
    fn model_slot_generation_wrap_keeps_zero_reserved() {
        let slot = ModelSlot::new();
        slot.ticket
            .store(ticket(GENERATION_MASK, EMPTY), Ordering::Release);
        assert_eq!(slot.reserve(), Some(1));
        // No payload was initialized; return Preparing directly to Empty.
        slot.ticket.store(ticket(1, EMPTY), Ordering::Release);
    }
}
