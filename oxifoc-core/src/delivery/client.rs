//! The reliable client driver.
//!
//! Generic over [`Timer`], so one implementation serves both host (a tokio
//! timer) and device ([`crate::timer::EmbassyTimer`]). The per-attempt timeout
//! is a runtime-agnostic `embassy_futures::select` race; **cancellation is left
//! to the call site** (e.g. `tokio::select!` against a `CancellationToken`),
//! which keeps the driver free of any runtime dependency.
//!
//! All the decision logic lives in [`super::policy`]; this file is the thin
//! glue that owns the timer and maps ergot's `ReqRespError` onto an
//! [`Outcome`].

use core::marker::PhantomData;

use embassy_futures::select::{Either, select};
use ergot::Address;
use ergot::net_stack::{NetStackHandle, ReqRespError};
use serde::Serialize;
use serde::de::DeserializeOwned;

use crate::timer::Timer;

use super::policy::{Decision, GiveUp, Outcome, decide};
use super::{Command, Deduplicated, DeliveryError, Idempotent, RetryPolicy};

/// A reliable endpoint client: a net-stack handle bound to a [`Timer`].
///
/// Construct it with [`ReliableExt::reliable`] (`stack.reliable::<TokioTimer>()`).
/// The three methods form the delivery ladder, and each is gated on the
/// command's [`Command::Delivery`] class — so a retry the class can't support
/// does not compile (`at_least_once` of an `AtMostOnce` command is rejected).
#[derive(Clone)]
pub struct Reliable<NS, T> {
    stack: NS,
    _timer: PhantomData<fn() -> T>,
}

impl<NS, T> Reliable<NS, T>
where
    NS: NetStackHandle,
    T: Timer,
{
    /// Bind a net-stack handle to a timer type.
    pub fn new(stack: NS) -> Self {
        Self {
            stack,
            _timer: PhantomData,
        }
    }

    /// One attempt, bounded by `policy.attempt_timeout_ms`. Available for any
    /// classified command. Retries only on [`Outcome::NotSent`] (the frame
    /// never left the host, so the effect provably did not happen) — never on
    /// a timeout, because a timed-out action may have been applied.
    pub async fn at_most_once<E>(
        &self,
        dst: Address,
        req: &E::Request,
        name: Option<&str>,
        policy: &RetryPolicy,
    ) -> Result<E::Response, DeliveryError>
    where
        E: Command,
        E::Request: Serialize + Clone + DeserializeOwned + 'static,
        E::Response: Serialize + Clone + DeserializeOwned + 'static,
    {
        self.run::<E>(dst, req, name, false, policy).await
    }

    /// Retry until success or the deadline budget is spent. Requires an
    /// [`Idempotent`] command, so retrying a maybe-applied attempt is safe.
    pub async fn at_least_once<E>(
        &self,
        dst: Address,
        req: &E::Request,
        name: Option<&str>,
        policy: &RetryPolicy,
    ) -> Result<E::Response, DeliveryError>
    where
        E: Command<Delivery = Idempotent>,
        E::Request: Serialize + Clone + DeserializeOwned + 'static,
        E::Response: Serialize + Clone + DeserializeOwned + 'static,
    {
        self.run::<E>(dst, req, name, true, policy).await
    }

    /// Retry with a stable request id; the server deduplicates on it, so the
    /// effect happens at most once even though delivery is at-least-once.
    /// Requires a [`Deduplicated`] command (whose request is a
    /// [`super::Keyed`] payload). Build the `Keyed` request with a stable id
    /// once — the driver re-sends the *same* payload on every retry.
    pub async fn effectively_once<E>(
        &self,
        dst: Address,
        req: &E::Request,
        name: Option<&str>,
        policy: &RetryPolicy,
    ) -> Result<E::Response, DeliveryError>
    where
        E: Command<Delivery = Deduplicated>,
        E::Request: Serialize + Clone + DeserializeOwned + 'static,
        E::Response: Serialize + Clone + DeserializeOwned + 'static,
    {
        self.run::<E>(dst, req, name, true, policy).await
    }

    /// The shared retry loop. `retry_on_timeout` is the only difference between
    /// the three public methods, and it is set from the command's class.
    async fn run<E>(
        &self,
        dst: Address,
        req: &E::Request,
        name: Option<&str>,
        retry_on_timeout: bool,
        policy: &RetryPolicy,
    ) -> Result<E::Response, DeliveryError>
    where
        E: Command,
        E::Request: Serialize + Clone + DeserializeOwned + 'static,
        E::Response: Serialize + Clone + DeserializeOwned + 'static,
    {
        let mut attempt: u32 = 0;
        let mut elapsed_ms: u64 = 0;

        loop {
            let target = self.stack.stack();
            let req_fut = target.endpoints().request::<E>(dst, req, name);

            // Race the request against this attempt's timeout.
            let (outcome, result) =
                match select(req_fut, T::after_millis(policy.attempt_timeout_ms)).await {
                    Either::First(res) => {
                        let o = match &res {
                            Ok(_) => Outcome::Ok,
                            // Local error: the frame never left — provably not applied.
                            Err(ReqRespError::Local(_)) => Outcome::NotSent,
                            // The server answered (with an error), or it's misuse
                            // (broadcast): authoritative, do not retry.
                            Err(ReqRespError::Remote(_)) | Err(ReqRespError::NoBroadcast) => {
                                Outcome::Remote
                            }
                        };
                        (o, Some(res))
                    }
                    Either::Second(()) => {
                        elapsed_ms = elapsed_ms.saturating_add(policy.attempt_timeout_ms);
                        (Outcome::TimedOut, None)
                    }
                };

            match decide(attempt, elapsed_ms, outcome, retry_on_timeout, policy) {
                Decision::Done => {
                    // `Done` implies `outcome == Ok`, i.e. `result == Some(Ok(_))`.
                    if let Some(Ok(resp)) = result {
                        return Ok(resp);
                    }
                    unreachable!("Done implies a successful response");
                }
                Decision::GiveUp(GiveUp::Remote) => return Err(DeliveryError::Remote),
                Decision::GiveUp(GiveUp::NotRetryable) => {
                    return Err(DeliveryError::TimedOut {
                        attempts: attempt + 1,
                    });
                }
                Decision::GiveUp(GiveUp::BudgetExhausted) => {
                    return Err(match outcome {
                        Outcome::NotSent => DeliveryError::NotSent,
                        _ => DeliveryError::TimedOut {
                            attempts: attempt + 1,
                        },
                    });
                }
                Decision::Retry { after_ms } => {
                    T::after_millis(after_ms).await;
                    elapsed_ms = elapsed_ms.saturating_add(after_ms);
                    attempt += 1;
                }
            }
        }
    }
}

/// Bind any [`NetStackHandle`] to a timer: `stack.reliable::<TokioTimer>()`.
pub trait ReliableExt: NetStackHandle + Sized {
    /// Wrap this handle in a [`Reliable`] client using timer `T`.
    fn reliable<T: Timer>(self) -> Reliable<Self, T> {
        Reliable::new(self)
    }
}

impl<NS: NetStackHandle> ReliableExt for NS {}
