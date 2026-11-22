//! Log events raised in the control ISR, printed from a task.
//!
//! A defmt call in the ADC ISR costs about 1,200 cycles even with the F405's
//! deferred logger: serialization, the queue grant and waking the worker all
//! run in the interrupt. [`raise`] only stores the event's arguments and sets
//! its bit and wakes [`run`], which prints the pending events. Repeats of one
//! event between two polls print once, with the latest arguments, which also
//! keeps a per-cycle condition from flooding the log.
//!
//! Without defmt (host builds) [`raise`] prints at once: the `log` backend
//! there has no interrupt to protect.

#[cfg(all(feature = "defmt", feature = "runtime"))]
use core::sync::atomic::{AtomicU32, Ordering};

use crate::motor::foc_driver::StepError;

/// How long [`run`] collects events after a wake before printing them.
pub const POLL_MS: u64 = 10;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Event {
    CommTimeout {
        deadman_expired: bool,
        link_lost: bool,
    },
    FailsafeLinkLost,
    FailsafeStoppingFault,
    KillCutGate,
    StopSequenceArmed {
        policy: u8,
        terminal: u8,
        seed_iq: f32,
    },
    OverVoltage {
        vbus: f32,
    },
    UnderVoltage {
        vbus: f32,
    },
    OverTemp {
        fet_over: bool,
        motor_over: bool,
    },
    SwOvercurrent {
        ia: f32,
        ib: f32,
        ic: f32,
    },
    StepError(StepError),
    ParkBrakeReleased {
        current: f32,
        limit: f32,
    },
    NonFiniteCommand,
    RejectedBootCalibration,
    RejectedOffsetCalibration,
    RejectedFlashPending,
    BrakeRampAtSpeed,
    BrakeRejectedUncalibrated,
    RejectedFailsafeLatched,
}

#[cfg(any(all(feature = "defmt", feature = "runtime"), test))]
const KINDS: usize = 18;
#[cfg(any(all(feature = "defmt", feature = "runtime"), test))]
const _: () = assert!(KINDS <= 32, "the pending set is one u32");

/// Record an ISR event. Out of line, so a call site costs a call, not
/// registers. Deliberately not `#[cold]`: on the MK5 bench the cold hint on
/// the seven command-rejection sites made LLVM lay out `process_commands`
/// so that every command cycle ran about 450 cycles slower.
pub(crate) fn raise(event: Event) {
    #[cfg(all(feature = "defmt", feature = "runtime"))]
    {
        let (kind, words) = event.encode();
        for (slot, word) in ARGS[kind].iter().zip(words) {
            slot.store(word, Ordering::Relaxed);
        }
        PENDING.fetch_or(1 << kind, Ordering::Release);
        WAKER.wake();
    }
    #[cfg(not(all(feature = "defmt", feature = "runtime")))]
    print(event);
}

#[cfg(all(feature = "defmt", feature = "runtime"))]
static PENDING: AtomicU32 = AtomicU32::new(0);
#[cfg(all(feature = "defmt", feature = "runtime"))]
static WAKER: embassy_sync::waitqueue::AtomicWaker = embassy_sync::waitqueue::AtomicWaker::new();
// The latest arguments of each kind. A raise racing the print can mix two
// raises' words; both describe the same event kind.
#[cfg(all(feature = "defmt", feature = "runtime"))]
static ARGS: [[AtomicU32; 3]; KINDS] = [const { [const { AtomicU32::new(0) }; 3] }; KINDS];

/// Print the events raised since the last call.
#[cfg(all(feature = "defmt", feature = "runtime"))]
pub fn drain() {
    let mut pending = PENDING.swap(0, Ordering::Acquire);
    while pending != 0 {
        let kind = pending.trailing_zeros() as usize;
        pending &= pending - 1;
        let words = ARGS[kind]
            .each_ref()
            .map(|slot| slot.load(Ordering::Relaxed));
        if let Some(event) = Event::decode(kind, words) {
            print(event);
        }
    }
}

/// Print ISR events forever. The task sleeps until [`raise`] wakes it, then
/// waits [`POLL_MS`] so events of one cycle and its neighbours print together.
#[cfg(all(feature = "defmt", feature = "runtime"))]
pub async fn run<T: crate::timer::Timer>() -> ! {
    loop {
        core::future::poll_fn(|cx| {
            WAKER.register(cx.waker());
            if PENDING.load(Ordering::Acquire) != 0 {
                core::task::Poll::Ready(())
            } else {
                core::task::Poll::Pending
            }
        })
        .await;
        T::after_millis(POLL_MS).await;
        drain();
    }
}

impl Event {
    #[cfg_attr(not(all(feature = "defmt", feature = "runtime")), allow(dead_code))]
    fn encode(self) -> (usize, [u32; 3]) {
        let b = u32::from;
        let f = f32::to_bits;
        match self {
            Self::CommTimeout {
                deadman_expired,
                link_lost,
            } => (0, [b(deadman_expired), b(link_lost), 0]),
            Self::FailsafeLinkLost => (1, [0; 3]),
            Self::FailsafeStoppingFault => (2, [0; 3]),
            Self::KillCutGate => (3, [0; 3]),
            Self::StopSequenceArmed {
                policy,
                terminal,
                seed_iq,
            } => (4, [policy.into(), terminal.into(), f(seed_iq)]),
            Self::OverVoltage { vbus } => (5, [f(vbus), 0, 0]),
            Self::UnderVoltage { vbus } => (6, [f(vbus), 0, 0]),
            Self::OverTemp {
                fet_over,
                motor_over,
            } => (7, [b(fet_over), b(motor_over), 0]),
            Self::SwOvercurrent { ia, ib, ic } => (8, [f(ia), f(ib), f(ic)]),
            Self::StepError(e) => (9, [e as u32, 0, 0]),
            Self::ParkBrakeReleased { current, limit } => (10, [f(current), f(limit), 0]),
            Self::NonFiniteCommand => (11, [0; 3]),
            Self::RejectedBootCalibration => (12, [0; 3]),
            Self::RejectedOffsetCalibration => (13, [0; 3]),
            Self::RejectedFlashPending => (14, [0; 3]),
            Self::BrakeRampAtSpeed => (15, [0; 3]),
            Self::BrakeRejectedUncalibrated => (16, [0; 3]),
            Self::RejectedFailsafeLatched => (17, [0; 3]),
        }
    }

    #[cfg_attr(not(all(feature = "defmt", feature = "runtime")), allow(dead_code))]
    fn decode(kind: usize, [w0, w1, w2]: [u32; 3]) -> Option<Self> {
        let f = f32::from_bits;
        Some(match kind {
            0 => Self::CommTimeout {
                deadman_expired: w0 != 0,
                link_lost: w1 != 0,
            },
            1 => Self::FailsafeLinkLost,
            2 => Self::FailsafeStoppingFault,
            3 => Self::KillCutGate,
            4 => Self::StopSequenceArmed {
                policy: w0 as u8,
                terminal: w1 as u8,
                seed_iq: f(w2),
            },
            5 => Self::OverVoltage { vbus: f(w0) },
            6 => Self::UnderVoltage { vbus: f(w0) },
            7 => Self::OverTemp {
                fet_over: w0 != 0,
                motor_over: w1 != 0,
            },
            8 => Self::SwOvercurrent {
                ia: f(w0),
                ib: f(w1),
                ic: f(w2),
            },
            9 => Self::StepError(match w0 {
                0 => StepError::NotCalibrated,
                1 => StepError::Overcurrent,
                _ => StepError::SourceLimit,
            }),
            10 => Self::ParkBrakeReleased {
                current: f(w0),
                limit: f(w1),
            },
            11 => Self::NonFiniteCommand,
            12 => Self::RejectedBootCalibration,
            13 => Self::RejectedOffsetCalibration,
            14 => Self::RejectedFlashPending,
            15 => Self::BrakeRampAtSpeed,
            16 => Self::BrakeRejectedUncalibrated,
            17 => Self::RejectedFailsafeLatched,
            _ => return None,
        })
    }
}

// The texts are matched by scripts/benchsuite.py ("CommTimeout raised",
// "stop sequence armed", "FAULT:", "FOC step error", "failsafe latched").
fn print(event: Event) {
    match event {
        Event::CommTimeout {
            deadman_expired,
            link_lost,
        } => warn!(
            "CommTimeout raised: deadman_expired={} link_lost={}",
            deadman_expired, link_lost
        ),
        Event::FailsafeLinkLost => warn!("failsafe: host link lost while running"),
        Event::FailsafeStoppingFault => warn!("failsafe: stopping-class fault active"),
        Event::KillCutGate => error!("KILL fault active: cutting gate drive"),
        Event::StopSequenceArmed {
            policy,
            terminal,
            seed_iq,
        } => warn!(
            "stop sequence armed: policy={} terminal={} seed_iq={}",
            policy, terminal, seed_iq
        ),
        Event::OverVoltage { vbus } => error!("OverVoltage FAULT: vbus = {} V (integrated)", vbus),
        Event::UnderVoltage { vbus } => {
            error!("UnderVoltage FAULT: vbus = {} V (integrated)", vbus);
        }
        Event::OverTemp {
            fet_over,
            motor_over,
        } => error!(
            "OverTemp FAULT (fet_over={}, motor_over={})",
            fet_over, motor_over
        ),
        Event::SwOvercurrent { ia, ib, ic } => {
            error!("SW overcurrent FAULT: ia={}, ib={}, ic={}", ia, ib, ic);
        }
        Event::StepError(e) => error!("FOC step error: {:?}", e),
        Event::ParkBrakeReleased { current, limit } => warn!(
            "park brake: short current {} A past the {} A limit without an angle, releasing",
            current, limit
        ),
        Event::NonFiniteCommand => warn!("Dropping non-finite driver command"),
        Event::RejectedBootCalibration => warn!("Mode rejected: boot current calibration active"),
        Event::RejectedOffsetCalibration => {
            warn!("Mode rejected: current-offset calibration active");
        }
        Event::RejectedFlashPending => warn!("Motor start rejected: flash operation in flight"),
        Event::BrakeRampAtSpeed => info!("Brake at speed: ramping to standstill first"),
        Event::BrakeRejectedUncalibrated => {
            warn!("Brake rejected: at speed and current sensor uncalibrated");
        }
        Event::RejectedFailsafeLatched => {
            warn!("Mode rejected: failsafe latched, acknowledge with Stopped first");
        }
    }
}

#[cfg(test)]
mod round_trip {
    use super::*;

    /// Every kind survives the word encoding the ISR and the task share.
    #[test]
    fn events_round_trip_through_their_words() {
        let events = [
            Event::CommTimeout {
                deadman_expired: true,
                link_lost: false,
            },
            Event::FailsafeLinkLost,
            Event::FailsafeStoppingFault,
            Event::KillCutGate,
            Event::StopSequenceArmed {
                policy: 2,
                terminal: 1,
                seed_iq: -1.5,
            },
            Event::OverVoltage { vbus: 58.25 },
            Event::UnderVoltage { vbus: 9.5 },
            Event::OverTemp {
                fet_over: false,
                motor_over: true,
            },
            Event::SwOvercurrent {
                ia: 1.0,
                ib: -2.0,
                ic: 3.0,
            },
            Event::StepError(StepError::NotCalibrated),
            Event::StepError(StepError::Overcurrent),
            Event::StepError(StepError::SourceLimit),
            Event::ParkBrakeReleased {
                current: 12.0,
                limit: 10.0,
            },
            Event::NonFiniteCommand,
            Event::RejectedBootCalibration,
            Event::RejectedOffsetCalibration,
            Event::RejectedFlashPending,
            Event::BrakeRampAtSpeed,
            Event::BrakeRejectedUncalibrated,
            Event::RejectedFailsafeLatched,
        ];
        let mut kinds = [false; KINDS];
        for event in events {
            let (kind, words) = event.encode();
            kinds[kind] = true;
            assert_eq!(Event::decode(kind, words), Some(event));
        }
        assert!(kinds.iter().all(|&seen| seen), "a kind has no test case");
    }
}
