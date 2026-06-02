//! Retry policy — the pure decision core, shared by every runtime driver.
//!
//! This module has **no async, no timer, no transport**: it is a deterministic
//! function from `(attempt, elapsed, outcome, class)` to a [`Decision`]. The
//! interesting reliability logic — the at-most/at-least/effectively-once ladder,
//! the `NotSent` vs `TimedOut` distinction, backoff math, the deadline budget —
//! all lives here so it can be unit-tested without spinning a runtime, and so
//! the tokio (host) and embassy (device) drivers can share one implementation.
//!
//! Budgeting is **deadline-based, not attempt-count-based**: a vehicle cares
//! about total uncommanded latency, not the number of tries. The driver owns
//! every sleep, so it can track *scheduled* elapsed time exactly without a clock
//! — and a request that completes faster than its timeout only makes us finish
//! early, never overshoot.

/// A retry budget. Times are in milliseconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RetryPolicy {
    /// Total wall-clock budget across all attempts. Should be `<=` the device's
    /// failsafe deadman threshold, so retries never mask a dying link.
    pub deadline_ms: u64,
    /// First backoff; doubles each attempt up to `max_backoff_ms`.
    pub base_backoff_ms: u64,
    /// Backoff ceiling.
    pub max_backoff_ms: u64,
    /// Per-attempt timeout (how long to wait for one response before giving up
    /// on that attempt).
    pub attempt_timeout_ms: u64,
}

impl RetryPolicy {
    /// A sane default for a 2 s interactive command budget.
    pub const DEFAULT: Self = Self {
        deadline_ms: 2_000,
        base_backoff_ms: 50,
        max_backoff_ms: 500,
        attempt_timeout_ms: 800,
    };

    /// Exponential backoff for the given (zero-based) attempt index, capped.
    pub fn backoff_ms(&self, attempt: u32) -> u64 {
        let factor = 1u64.checked_shl(attempt).unwrap_or(u64::MAX);
        self.base_backoff_ms
            .saturating_mul(factor)
            .min(self.max_backoff_ms)
    }
}

/// What a single attempt did, as observed by the driver. Maps from ergot's
/// `ReqRespError` plus the per-attempt timeout.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Got a response.
    Ok,
    /// Frame never left the host — provably not applied.
    NotSent,
    /// Sent, no response in time — maybe applied.
    TimedOut,
    /// Server replied with an error — authoritative.
    Remote,
}

/// Why a reliable send stopped without success.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GiveUp {
    /// The deadline budget would be exceeded by another attempt.
    BudgetExhausted,
    /// The server returned an error; do not retry.
    Remote,
    /// The attempt timed out (effect maybe applied) and the command's class
    /// does not permit retry-on-timeout.
    NotRetryable,
}

/// What the driver should do after an attempt.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    /// Success — return the response.
    Done,
    /// Sleep `after_ms` and attempt again.
    Retry { after_ms: u64 },
    /// Stop and surface this reason.
    GiveUp(GiveUp),
}

/// The pure retry decision.
///
/// * `attempt` — zero-based index of the attempt that just produced `outcome`.
/// * `scheduled_elapsed_ms` — total time the driver has *scheduled* so far
///   (sum of attempt timeouts consumed + backoffs slept). Drives the deadline.
/// * `retry_on_timeout` — whether the command's [`crate::delivery::DeliveryClass`]
///   permits retrying after a `TimedOut` (true for idempotent/deduplicated).
///   `NotSent` is always retryable regardless, because the effect provably did
///   not happen.
pub fn decide(
    attempt: u32,
    scheduled_elapsed_ms: u64,
    outcome: Outcome,
    retry_on_timeout: bool,
    policy: &RetryPolicy,
) -> Decision {
    match outcome {
        Outcome::Ok => Decision::Done,
        // The server spoke; its answer is the truth. Retrying cannot help.
        Outcome::Remote => Decision::GiveUp(GiveUp::Remote),
        // Maybe-applied: only retry if the class says it's safe.
        Outcome::TimedOut if !retry_on_timeout => Decision::GiveUp(GiveUp::NotRetryable),
        // NotSent (always) or TimedOut (when permitted): consider another go.
        Outcome::NotSent | Outcome::TimedOut => {
            let backoff = policy.backoff_ms(attempt);
            // Will the next attempt (backoff + its own timeout) fit the budget?
            let next_cost = backoff.saturating_add(policy.attempt_timeout_ms);
            if scheduled_elapsed_ms.saturating_add(next_cost) > policy.deadline_ms {
                Decision::GiveUp(GiveUp::BudgetExhausted)
            } else {
                Decision::Retry { after_ms: backoff }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const P: RetryPolicy = RetryPolicy {
        deadline_ms: 1_000,
        base_backoff_ms: 100,
        max_backoff_ms: 400,
        attempt_timeout_ms: 200,
    };

    #[test]
    fn ok_is_done() {
        assert_eq!(decide(0, 0, Outcome::Ok, true, &P), Decision::Done);
        // Even a non-retryable command is Done on success.
        assert_eq!(decide(3, 999, Outcome::Ok, false, &P), Decision::Done);
    }

    #[test]
    fn remote_never_retries() {
        assert_eq!(
            decide(0, 0, Outcome::Remote, true, &P),
            Decision::GiveUp(GiveUp::Remote)
        );
    }

    #[test]
    fn timeout_not_retryable_for_non_idempotent() {
        // The crux: a maybe-applied timeout on a non-idempotent command stops.
        assert_eq!(
            decide(0, 0, Outcome::TimedOut, false, &P),
            Decision::GiveUp(GiveUp::NotRetryable)
        );
    }

    #[test]
    fn not_sent_retries_even_when_not_retryable() {
        // NotSent is provably-not-applied, so it is safe to re-send for ANY class.
        assert_eq!(
            decide(0, 0, Outcome::NotSent, false, &P),
            Decision::Retry { after_ms: 100 }
        );
    }

    #[test]
    fn timeout_retries_when_idempotent() {
        assert_eq!(
            decide(0, 0, Outcome::TimedOut, true, &P),
            Decision::Retry { after_ms: 100 }
        );
    }

    #[test]
    fn backoff_doubles_and_caps() {
        assert_eq!(P.backoff_ms(0), 100);
        assert_eq!(P.backoff_ms(1), 200);
        assert_eq!(P.backoff_ms(2), 400);
        assert_eq!(P.backoff_ms(3), 400); // capped at max
        assert_eq!(P.backoff_ms(64), 400); // no overflow panic
    }

    #[test]
    fn budget_exhaustion_stops_retrying() {
        // Almost out of budget: next attempt (backoff 100 + timeout 200) won't fit.
        assert_eq!(
            decide(0, 800, Outcome::TimedOut, true, &P),
            Decision::GiveUp(GiveUp::BudgetExhausted)
        );
        // Exactly fits (700 + 100 + 200 == 1000) → still retries.
        assert_eq!(
            decide(0, 700, Outcome::TimedOut, true, &P),
            Decision::Retry { after_ms: 100 }
        );
    }
}
