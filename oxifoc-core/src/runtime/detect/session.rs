//! Protocol admission is separate from execution. This owner is polled only by
//! the detection task, including while its measurement future is suspended.
use super::operations::{Admission, Operations};
use crate::types::commissioning::CommissioningPlan;
use crate::types::measurement::*;
use crate::types::{ConfigGroupId, DetectError, DetectRequest, DetectResponse};

pub struct Session {
    id: SessionId,
    previous: Option<(SessionId, crate::types::ReqId)>,
    active: Option<OperationId>,
    plan: Option<CommissioningPlan>,
    revisions: [u32; ConfigGroupId::COUNT],
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
            plan: None,
            revisions: [0; ConfigGroupId::COUNT],
            operations: Operations::new(id),
        }
    }
    pub fn plan(&self) -> Option<CommissioningPlan> {
        self.plan
    }

    #[cfg(test)]
    fn handle(&mut self, request: Request) -> Dispatch {
        self.handle_with_revisions(request, [0; ConfigGroupId::COUNT])
    }

    pub fn handle_with_revisions(
        &mut self,
        request: Request,
        revisions: [u32; ConfigGroupId::COUNT],
    ) -> Dispatch {
        match request {
            Request::Inspect => Reply::Session(self.id).into(),
            Request::Open {
                expected,
                client,
                plan,
            } => {
                if plan.is_some_and(|p| !p.is_valid()) {
                    return Reply::Rejected(Rejection::InvalidRequest).into();
                }
                if self.previous == Some((expected, client)) {
                    return if plan == self.plan {
                        Reply::Session(self.id)
                    } else {
                        Reply::Rejected(Rejection::KeyConflict)
                    }
                    .into();
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
                self.plan = plan;
                self.revisions = revisions;
                Reply::Session(self.id).into()
            }
            Request::Start { id, request } => {
                if !Measurement::request_is_finite(&request) {
                    return Reply::Rejected(Rejection::InvalidRequest).into();
                }
                if self.id.generation == 0 || id.session != self.id {
                    return Reply::Rejected(Rejection::WrongSession).into();
                }
                if let Some(plan) = self.plan
                    && self.operations.status(id).is_err()
                {
                    if !plan.allows(request) {
                        return Reply::Rejected(Rejection::InvalidRequest).into();
                    }
                    if self.revisions != revisions {
                        return Reply::Rejected(Rejection::ConditionsChanged).into();
                    }
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
                plan: None,
            })
            .reply
        else {
            panic!()
        };
        let Reply::Session(replayed) = s
            .handle(Request::Open {
                expected,
                client: crate::types::ReqId(1),
                plan: None,
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
                client: crate::types::ReqId(99),
                plan: None
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
                    client: crate::types::ReqId(2),
                    plan: None
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
                client: crate::types::ReqId(2),
                plan: None
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

#[cfg(test)]
mod plan_tests {
    use super::*;
    use crate::types::commissioning::{MotorClass, SourceLimits};
    fn plan() -> CommissioningPlan {
        CommissioningPlan {
            source: SourceLimits {
                input_a: 5.0,
                regen_a: 0.0,
            },
            motor_class: MotorClass::MediumOutrunner,
            pole_pairs: 7,
            flux_target_erpm: 4000.,
            current_limit_a: None,
            calibrate_hall: true,
        }
    }
    #[test]
    fn session_binds_plan_and_revisions_without_losing_terminal_replay() {
        let mut s = Session::new(42);
        let initial = s.id;
        let p = plan();
        let request = Request::Open {
            expected: initial,
            client: crate::types::ReqId(1),
            plan: Some(p),
        };
        let Reply::Session(id) = s.handle(request.clone()).reply else {
            panic!()
        };
        assert!(matches!(s.handle(request).reply, Reply::Session(v) if v == id));
        let mut changed = p;
        changed.source.input_a = 7.;
        assert!(matches!(
            s.handle(Request::Open {
                expected: initial,
                client: crate::types::ReqId(1),
                plan: Some(changed)
            })
            .reply,
            Reply::Rejected(Rejection::KeyConflict)
        ));
        let first = OperationId {
            session: id,
            sequence: 1,
        };
        let r = DetectRequest::MeasureResistance {
            max_power_loss_w: 120.,
            current_limit_a: None,
        };
        assert!(
            s.handle(Request::Start {
                id: first,
                request: r
            })
            .start
            .is_some()
        );
        s.finish(
            first,
            Outcome::Completed(DetectResponse::Resistance {
                resistance_ohm: 0.06,
            }),
        );
        let mut revisions = [0; ConfigGroupId::COUNT];
        revisions[ConfigGroupId::CurrentLimits.index()] = 1;
        assert!(matches!(
            s.handle_with_revisions(
                Request::Start {
                    id: first,
                    request: r
                },
                revisions
            )
            .reply,
            Reply::Operation(State::Finished(Outcome::Completed(_)))
        ));
        let second = OperationId {
            sequence: 2,
            ..first
        };
        assert!(matches!(
            s.handle_with_revisions(
                Request::Start {
                    id: second,
                    request: r
                },
                revisions
            )
            .reply,
            Reply::Rejected(Rejection::ConditionsChanged)
        ));
        assert!(s.active.is_none());
        assert!(matches!(
            s.handle(Request::Start {
                id: second,
                request: DetectRequest::MeasureResistance {
                    max_power_loss_w: 400.,
                    current_limit_a: None
                }
            })
            .reply,
            Reply::Rejected(Rejection::InvalidRequest)
        ));
        assert!(
            s.handle(Request::Start {
                id: second,
                request: r
            })
            .start
            .is_some()
        );
    }
    #[test]
    fn invalid_source_cannot_open_a_session() {
        let mut s = Session::new(42);
        for bad in [f32::NAN, f32::INFINITY, -1., 0.] {
            let mut p = plan();
            p.source.input_a = bad;
            assert!(matches!(
                s.handle(Request::Open {
                    expected: s.id,
                    client: crate::types::ReqId(1),
                    plan: Some(p)
                })
                .reply,
                Reply::Rejected(Rejection::InvalidRequest)
            ));
        }
        assert_eq!(s.id.generation, 0);
    }
}
