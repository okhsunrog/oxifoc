//! Bounded operation history for one device-issued commissioning session.
//!
//! No transport, timers or actuator access. The caller serializes transitions
//! and must disarm/restore the actuator before publishing `finish`. A session
//! identity must include a non-repeating boot epoch; this module does not mint
//! identities or claim persistence across reboots.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SessionId {
    pub boot: u64,
    pub generation: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OperationId {
    pub session: SessionId,
    pub sequence: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rejection {
    WrongSession,
    ResultExpired,
    OutOfOrder,
    KeyConflict,
    Busy,
    NotRunning,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Outcome<T, E> {
    Completed(T),
    Failed(E),
    Cancelled,
}

#[derive(Clone, Debug, PartialEq)]
pub enum State<T, E> {
    Running,
    Cancelling,
    Finished(Outcome<T, E>),
}

#[derive(Clone, Debug, PartialEq)]
pub enum Admission<T, E> {
    Start,
    Replay(State<T, E>),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cancellation {
    /// Owner must stop the actuator, then call finish to acknowledge cleanup.
    StopRequired,
    /// No actuator work remains, including cancellation before admission.
    Settled,
}

struct Entry<R, T, E> {
    sequence: u32,
    // None is a tombstone for a cancel that arrived before its start.
    request: Option<R>,
    state: State<T, E>,
}

/// Retains one result; older sequence numbers remain permanently retired.
/// No heap and no eviction that could turn an old retry into a new start.
/// A different operation cannot start until cleanup of the current one ends.
pub struct Operations<R, T, E> {
    session: SessionId,
    last: Option<Entry<R, T, E>>,
}

impl<R: PartialEq + Clone, T: Clone, E: Clone> Operations<R, T, E> {
    pub const fn new(session: SessionId) -> Self {
        Self {
            session,
            last: None,
        }
    }

    fn locate(&self, id: OperationId) -> Result<bool, Rejection> {
        if id.session != self.session {
            return Err(Rejection::WrongSession);
        }
        let last = self.last.as_ref().map_or(0, |e| e.sequence);
        if id.sequence == 0 || id.sequence < last {
            return Err(Rejection::ResultExpired);
        }
        if self.last.is_some() && id.sequence == last {
            return Ok(true);
        }
        if last.checked_add(1) != Some(id.sequence) {
            return Err(Rejection::OutOfOrder);
        }
        Ok(false)
    }

    pub fn status(&self, id: OperationId) -> Result<State<T, E>, Rejection> {
        if !self.locate(id)? {
            return Err(Rejection::NotRunning);
        }
        Ok(self
            .last
            .as_ref()
            .ok_or(Rejection::NotRunning)?
            .state
            .clone())
    }

    pub fn begin(&mut self, id: OperationId, request: R) -> Result<Admission<T, E>, Rejection> {
        if self.locate(id)? {
            let entry = self.last.as_ref().ok_or(Rejection::NotRunning)?;
            if entry.request.as_ref().is_some_and(|old| *old != request) {
                return Err(Rejection::KeyConflict);
            }
            return Ok(Admission::Replay(entry.state.clone()));
        }
        if self
            .last
            .as_ref()
            .is_some_and(|e| !matches!(e.state, State::Finished(_)))
        {
            return Err(Rejection::Busy);
        }
        self.last = Some(Entry {
            sequence: id.sequence,
            request: Some(request),
            state: State::Running,
        });
        Ok(Admission::Start)
    }

    pub fn cancel(&mut self, id: OperationId) -> Result<Cancellation, Rejection> {
        if self.locate(id)? {
            let entry = self.last.as_mut().ok_or(Rejection::NotRunning)?;
            return match entry.state {
                State::Finished(_) => Ok(Cancellation::Settled),
                _ => {
                    entry.state = State::Cancelling;
                    Ok(Cancellation::StopRequired)
                }
            };
        }
        if self
            .last
            .as_ref()
            .is_some_and(|e| !matches!(e.state, State::Finished(_)))
        {
            return Err(Rejection::Busy);
        }
        self.last = Some(Entry {
            sequence: id.sequence,
            request: None,
            state: State::Finished(Outcome::Cancelled),
        });
        Ok(Cancellation::Settled)
    }

    /// Call only AFTER actuator cleanup. Cancellation wins over a result that
    /// was concurrently becoming ready. Late completions never affect a newer
    /// operation or overwrite an already-published outcome.
    pub fn finish(&mut self, id: OperationId, outcome: Outcome<T, E>) -> Result<(), Rejection> {
        if !self.locate(id)? {
            return Err(Rejection::NotRunning);
        }
        let entry = self.last.as_mut().ok_or(Rejection::NotRunning)?;
        entry.state = match entry.state {
            State::Running => State::Finished(outcome),
            State::Cancelling => State::Finished(Outcome::Cancelled),
            State::Finished(_) => return Err(Rejection::NotRunning),
        };
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const SESSION: SessionId = SessionId {
        boot: 17,
        generation: 1,
    };
    fn id(sequence: u32) -> OperationId {
        OperationId {
            session: SESSION,
            sequence,
        }
    }
    fn journal() -> Operations<u32, u32, &'static str> {
        Operations::new(SESSION)
    }

    #[test]
    fn lost_running_and_terminal_responses_never_restart() {
        for outcome in [
            Outcome::Completed(12),
            Outcome::Failed("source limited"),
            Outcome::Cancelled,
        ] {
            let mut j = journal();
            assert_eq!(j.begin(id(1), 42), Ok(Admission::Start));
            for _ in 0..5 {
                assert_eq!(j.begin(id(1), 42), Ok(Admission::Replay(State::Running)));
            }
            j.finish(id(1), outcome.clone()).unwrap();
            for _ in 0..5 {
                assert_eq!(
                    j.begin(id(1), 42),
                    Ok(Admission::Replay(State::Finished(outcome.clone())))
                );
            }
            assert_eq!(j.begin(id(1), 43), Err(Rejection::KeyConflict));
        }
    }

    #[test]
    fn cancelled_before_start_cannot_energize() {
        let mut j = journal();
        assert_eq!(j.cancel(id(1)), Ok(Cancellation::Settled));
        assert_eq!(
            j.begin(id(1), 42),
            Ok(Admission::Replay(State::Finished(Outcome::Cancelled)))
        );
        assert_eq!(j.begin(id(2), 42), Ok(Admission::Start));
        assert_eq!(j.cancel(id(1)), Err(Rejection::ResultExpired));
        assert_eq!(j.status(id(2)), Ok(State::Running));
    }

    #[test]
    fn cancel_requires_cleanup_and_wins_completion_race() {
        let mut j = journal();
        j.begin(id(1), 42).unwrap();
        assert_eq!(j.cancel(id(1)), Ok(Cancellation::StopRequired));
        assert_eq!(j.cancel(id(1)), Ok(Cancellation::StopRequired));
        assert_eq!(j.begin(id(2), 43), Err(Rejection::Busy));
        assert_eq!(j.status(id(1)), Ok(State::Cancelling));
        j.finish(id(1), Outcome::Completed(1)).unwrap();
        assert_eq!(j.status(id(1)), Ok(State::Finished(Outcome::Cancelled)));
        assert_eq!(j.cancel(id(1)), Ok(Cancellation::Settled));
        assert_eq!(
            j.finish(id(1), Outcome::Completed(2)),
            Err(Rejection::NotRunning)
        );
    }

    #[test]
    fn evicted_result_never_becomes_new_work() {
        let mut j = journal();
        for seq in 1..100 {
            j.begin(id(seq), seq).unwrap();
            j.finish(id(seq), Outcome::Completed(seq)).unwrap();
        }
        assert_eq!(j.begin(id(1), 1), Err(Rejection::ResultExpired));
        assert_eq!(
            j.finish(id(1), Outcome::Completed(1)),
            Err(Rejection::ResultExpired)
        );
        assert_eq!(
            j.status(id(99)),
            Ok(State::Finished(Outcome::Completed(99)))
        );
    }

    #[test]
    fn identity_and_sequence_are_checked_before_mutation() {
        let mut j = journal();
        for session in [
            SessionId {
                boot: 18,
                ..SESSION
            },
            SessionId {
                generation: 2,
                ..SESSION
            },
        ] {
            let other = OperationId {
                session,
                sequence: 1,
            };
            assert_eq!(j.begin(other, 42), Err(Rejection::WrongSession));
            assert_eq!(j.cancel(other), Err(Rejection::WrongSession));
        }
        assert_eq!(j.begin(id(0), 42), Err(Rejection::ResultExpired));
        assert_eq!(j.begin(id(2), 42), Err(Rejection::OutOfOrder));
        assert_eq!(j.begin(id(1), 42), Ok(Admission::Start));
        assert_eq!(j.cancel(id(2)), Err(Rejection::Busy));
    }

    #[test]
    fn sequence_exhaustion_does_not_wrap() {
        let mut j = journal();
        j.last = Some(Entry {
            sequence: u32::MAX,
            request: Some(42),
            state: State::Finished(Outcome::Completed(1)),
        });
        assert_eq!(j.begin(id(1), 42), Err(Rejection::ResultExpired));
        assert_eq!(j.begin(id(0), 42), Err(Rejection::ResultExpired));
    }
}
