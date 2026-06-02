//! Delivery semantics — a typed ladder of delivery guarantees over ergot's
//! at-most-once transport.
//!
//! ergot delivers a request *now or never*: it arrives once or is dropped, with
//! no built-in retry. Reliability is therefore the application's job, and the
//! safe way to add it is to make the *kind* of guarantee a property of the
//! command, checked by the compiler.
//!
//! # The ladder
//!
//! When the host retries (because it didn't get a response), the device may end
//! up executing the command **0, 1, or 2+ times** — and the host cannot tell a
//! lost *request* (executed 0×) from a lost *response* (executed 1×). So the one
//! question that classifies every command is: *is executing it a second time
//! harmful, and if so, can the device recognise and suppress the duplicate?*
//!
//! | Class            | Repeat is…           | Device memory | Client method        |
//! |------------------|----------------------|---------------|----------------------|
//! | [`Idempotent`]   | harmless by nature   | none          | `at_least_once`      |
//! | [`Deduplicated`] | made harmless by id  | id → response  | `effectively_once`   |
//! | [`AtMostOnce`]   | harmful, can't dedup | n/a           | `at_most_once`       |
//!
//! - **Idempotent** — reads and absolute setpoints (`set mode = X`). `f(f(x)) ==
//!   f(x)`, so retrying blindly is safe; no device-side bookkeeping.
//! - **Deduplicated** — a side-effecting action tagged with a stable
//!   [`ReqId`]; the device caches `id → response` and returns the cache on a
//!   repeat (see [`crate::delivery`] `Dedup`). Retry is safe because the *effect*
//!   happens at most once even though delivery is at-least-once.
//! - **AtMostOnce** — a harmful action that cannot be deduplicated (e.g. reboot:
//!   the responder, and its dedup cache, are destroyed by the command). We do
//!   **not** retry on timeout; the effect is confirmed by *observing state*, not
//!   by the response.
//!
//! The class a command declares is the compiler-enforced ceiling on which client
//! method may be used: you cannot `at_least_once` an [`AtMostOnce`] command. The
//! marker is the author's promise that the command really has the stated effect
//! semantics (like `Send`/`Sync` discipline) — the *ladder* is type-enforced, the
//! idempotency itself is asserted.

mod client;
mod policy;

pub use client::{Reliable, ReliableExt};
pub use policy::{Decision, GiveUp, Outcome, RetryPolicy};

use ergot::traits::Endpoint;
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

mod sealed {
    pub trait Sealed {}
}

/// Where a command sits on the delivery-guarantee ladder.
///
/// Sealed: the set of classes is closed, so the client API can exhaustively
/// gate its methods on them. The name is the *best achievable guarantee*.
pub trait DeliveryClass: sealed::Sealed {}

/// Read or absolute setpoint: `f(f(x)) == f(x)`. Safe to retry blindly.
pub enum Idempotent {}
/// Side-effecting action made safe to retry by server-side dedup on a [`ReqId`].
pub enum Deduplicated {}
/// A bare action: retry re-executes it, and it cannot be deduplicated. The
/// ceiling is at-most-once.
pub enum AtMostOnce {}

impl sealed::Sealed for Idempotent {}
impl sealed::Sealed for Deduplicated {}
impl sealed::Sealed for AtMostOnce {}
impl DeliveryClass for Idempotent {}
impl DeliveryClass for Deduplicated {}
impl DeliveryClass for AtMostOnce {}

/// Every command declares its delivery semantics. This is the single source of
/// truth; the client API is generic over [`Command::Delivery`] so an unsafe
/// retry does not compile.
///
/// Declare it next to the `endpoint!` definition, e.g.
/// ```ignore
/// impl Command for MotorEndpoint { type Delivery = Idempotent; }
/// ```
pub trait Command: Endpoint {
    /// This command's place on the ladder.
    type Delivery: DeliveryClass;
    /// Tuned default retry budget for this command.
    const POLICY: RetryPolicy = RetryPolicy::DEFAULT;
}

/// A client-chosen request id, stable across retries of the *same* logical
/// request. The server dedups on it so a [`Deduplicated`] action runs at most
/// once. Mirrors an HTTP idempotency key.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct ReqId(pub u64);

/// A request payload tagged with a [`ReqId`]. The wire envelope for
/// [`Deduplicated`] commands.
#[derive(Clone, Debug, Serialize, Deserialize, Schema)]
pub struct Keyed<T> {
    /// Stable across retries; the dedup key.
    pub id: ReqId,
    /// The actual request.
    pub inner: T,
}

impl<T> Keyed<T> {
    /// Tag a request with an id.
    pub fn new(id: ReqId, inner: T) -> Self {
        Self { id, inner }
    }
}

/// The outcome of a reliable send, told in terms of *what the caller may
/// conclude about whether the effect happened* — this taxonomy is the guarantee.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DeliveryError {
    /// The frame never left the host (no route / local queue full). The effect
    /// definitely did **not** happen — safe to re-send even for [`AtMostOnce`].
    NotSent,
    /// Sent, but no response within the budget. The effect **may** have
    /// happened. `attempts` is how many sends were made.
    TimedOut { attempts: u32 },
    /// The server processed the request and replied with a protocol/application
    /// error. Authoritative — never retried.
    Remote,
    /// Aborted by the caller (cancellation / reconnect).
    Cancelled,
}
