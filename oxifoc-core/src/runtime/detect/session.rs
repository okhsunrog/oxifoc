//! Protocol admission is separate from execution. This owner is polled only by
//! the detection task, including while its measurement future is suspended.
use super::operations::{Admission, Operations};
use crate::types::measurement::*;
use crate::types::{DetectError, DetectRequest, DetectResponse};

pub struct Session {
    id: SessionId,
    previous: Option<(SessionId, crate::types::ReqId)>,
    active: Option<OperationId>,
    operations: Operations<DetectRequest, DetectResponse, DetectError>,
}

pub struct Dispatch {
    pub reply: Reply,
    pub start: Option<Measurement>,
    pub cancel_active: bool,
}
impl From<Reply> for Dispatch {
    fn from(reply: Reply) -> Self {
        Self {
            reply,
            start: None,
            cancel_active: false,
        }
    }
}
impl Session {
    pub fn new(boot: u64) -> Self {
        let id = SessionId {
            boot,
            generation: 0,
        };
        Self {
            id,
            previous: None,
            active: None,
            operations: Operations::new(id),
        }
    }
    pub fn handle(&mut self, request: Request) -> Dispatch {
        match request {
            Request::Inspect => Reply::Session(self.id).into(),
            Request::Open { expected, client } => {
                if self.previous == Some((expected, client)) {
                    return Reply::Session(self.id).into();
                }
                if expected != self.id {
                    return Reply::Rejected(Rejection::WrongSession).into();
                }
                if self.active.is_some() {
                    return Reply::Rejected(Rejection::Busy).into();
                }
                let Some(generation) = self.id.generation.checked_add(1) else {
                    return Reply::Rejected(Rejection::OutOfOrder).into();
                };
                self.previous = Some((self.id, client));
                self.id.generation = generation;
                self.operations = Operations::new(self.id);
                Reply::Session(self.id).into()
            }
            Request::Start { id, request } => {
                if !Measurement::request_is_finite(&request) {
                    return Reply::Rejected(Rejection::InvalidRequest).into();
                }
                if self.id.generation == 0 {
                    return Reply::Rejected(Rejection::WrongSession).into();
                }
                match self.operations.begin(id, request) {
                    Ok(Admission::Start) => {
                        self.active = Some(id);
                        Dispatch {
                            reply: Reply::Operation(State::Running),
                            start: Some(Measurement { id, request }),
                            cancel_active: false,
                        }
                    }
                    Ok(Admission::Replay(state)) => Reply::Operation(state).into(),
                    Err(e) => Reply::Rejected(e).into(),
                }
            }
            Request::Status { id } => match self.operations.status(id) {
                Ok(state) => Reply::Operation(state).into(),
                Err(e) => Reply::Rejected(e).into(),
            },
            Request::Cancel { id } => match self.operations.cancel(id) {
                Ok(_) => Dispatch {
                    reply: match self.operations.status(id) {
                        Ok(s) => Reply::Operation(s),
                        Err(e) => Reply::Rejected(e),
                    },
                    start: None,
                    cancel_active: self.active == Some(id),
                },
                Err(e) => Reply::Rejected(e).into(),
            },
        }
    }
    /// Execution calls this only after acknowledged actuator cleanup.
    pub fn finish(&mut self, id: OperationId, outcome: Outcome<DetectResponse, DetectError>) {
        if self.operations.finish(id, outcome).is_ok() {
            self.active = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn open(s: &mut Session) -> SessionId {
        let Reply::Session(expected) = s.handle(Request::Inspect).reply else {
            panic!()
        };
        let Reply::Session(id) = s
            .handle(Request::Open {
                expected,
                client: crate::types::ReqId(1),
            })
            .reply
        else {
            panic!()
        };
        let Reply::Session(replayed) = s
            .handle(Request::Open {
                expected,
                client: crate::types::ReqId(1),
            })
            .reply
        else {
            panic!()
        };
        assert_eq!(id, replayed);
        id
    }
    #[test]
    fn malformed_requests_and_other_clients_cannot_claim_identity() {
        let mut s = Session::new(9);
        let initial = s.id;
        let issued = open(&mut s);
        assert!(matches!(
            s.handle(Request::Open {
                expected: initial,
                client: crate::types::ReqId(99)
            })
            .reply,
            Reply::Rejected(Rejection::WrongSession)
        ));
        let id = OperationId {
            session: issued,
            sequence: 1,
        };
        for value in [f32::NAN, f32::INFINITY] {
            assert!(matches!(
                s.handle(Request::Start {
                    id,
                    request: DetectRequest::MeasureResistance {
                        max_power_loss_w: value,
                        current_limit_a: None
                    }
                })
                .reply,
                Reply::Rejected(Rejection::InvalidRequest)
            ));
        }
        assert!(s.active.is_none());
    }

    #[test]
    fn boot_and_session_replacement_retire_commands() {
        let mut s = Session::new(9);
        let id = OperationId {
            session: open(&mut s),
            sequence: 1,
        };
        assert!(!s.handle(Request::Cancel { id }).cancel_active);
        open(&mut s);
        assert!(matches!(
            s.handle(Request::Cancel { id }).reply,
            Reply::Rejected(Rejection::WrongSession)
        ));
        let mut rebooted = Session::new(10);
        assert!(matches!(
            rebooted
                .handle(Request::Open {
                    expected: id.session,
                    client: crate::types::ReqId(2)
                })
                .reply,
            Reply::Rejected(Rejection::WrongSession)
        ));
        assert!(matches!(
            rebooted.handle(Request::Status { id }).reply,
            Reply::Rejected(Rejection::WrongSession)
        ));
    }
    #[test]
    fn active_session_cannot_be_replaced_and_cancel_requires_cleanup() {
        let mut s = Session::new(9);
        let id = OperationId {
            session: open(&mut s),
            sequence: 1,
        };
        let request = DetectRequest::MeasureResistance {
            max_power_loss_w: 120.,
            current_limit_a: None,
        };
        assert!(s.handle(Request::Start { id, request }).start.is_some());
        assert!(s.handle(Request::Start { id, request }).start.is_none());
        assert!(matches!(
            s.handle(Request::Open {
                expected: id.session,
                client: crate::types::ReqId(2)
            })
            .reply,
            Reply::Rejected(Rejection::Busy)
        ));
        assert!(s.handle(Request::Cancel { id }).cancel_active);
        assert!(matches!(
            s.handle(Request::Status { id }).reply,
            Reply::Operation(State::Cancelling)
        ));
        s.finish(id, Outcome::Failed(DetectError::HardwareFault));
        assert!(matches!(
            s.handle(Request::Status { id }).reply,
            Reply::Operation(State::Finished(Outcome::Cancelled))
        ));
        assert!(s.handle(Request::Start { id, request }).start.is_none());
    }
}
