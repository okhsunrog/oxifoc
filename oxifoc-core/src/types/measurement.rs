//! Session-scoped measurement protocol. Transport retries preserve operation identity.
use super::{DetectError, DetectRequest, DetectResponse};
#[derive(
    serde::Serialize, serde::Deserialize, postcard_schema::Schema, Clone, Copy, Debug, PartialEq, Eq,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SessionId {
    pub boot: u64,
    pub generation: u32,
}

#[derive(
    serde::Serialize, serde::Deserialize, postcard_schema::Schema, Clone, Copy, Debug, PartialEq, Eq,
)]
pub struct OperationId {
    pub session: SessionId,
    pub sequence: u32,
}

#[derive(
    serde::Serialize, serde::Deserialize, postcard_schema::Schema, Clone, Copy, Debug, PartialEq, Eq,
)]
pub enum Rejection {
    WrongSession,
    ResultExpired,
    OutOfOrder,
    KeyConflict,
    Busy,
    NotRunning,
    InvalidRequest,
    ConditionsChanged,
}

#[derive(
    serde::Serialize, serde::Deserialize, postcard_schema::Schema, Clone, Debug, PartialEq,
)]
pub enum Outcome<T, E> {
    Completed(T),
    Failed(E),
    Cancelled,
}

#[derive(
    serde::Serialize, serde::Deserialize, postcard_schema::Schema, Clone, Debug, PartialEq,
)]
pub enum State<T, E> {
    Running,
    Cancelling,
    Finished(Outcome<T, E>),
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, postcard_schema::Schema)]
pub enum Request {
    Inspect,
    /// Compare-and-swap: a retry returns the same issued session.
    Open {
        expected: SessionId,
        client: super::ReqId,
        plan: Option<super::commissioning::CommissioningPlan>,
    },
    Start {
        id: OperationId,
        request: DetectRequest,
    },
    Status {
        id: OperationId,
    },
    Cancel {
        id: OperationId,
    },
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, postcard_schema::Schema)]
pub enum Reply {
    Session(SessionId),
    Operation(State<DetectResponse, DetectError>),
    Rejected(Rejection),
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, postcard_schema::Schema)]
pub struct Measurement {
    pub id: OperationId,
    pub request: DetectRequest,
}

impl Measurement {
    /// Reject non-finite floats before identity comparison: NaN is not equal
    /// to itself and cannot participate in a replay contract based on equality.
    pub fn request_is_finite(request: &DetectRequest) -> bool {
        use DetectRequest::*;
        match *request {
            MeasureResistance {
                max_power_loss_w,
                current_limit_a,
            } => max_power_loss_w.is_finite() && current_limit_a.is_none_or(f32::is_finite),
            MeasureInductance {
                max_power_loss_w,
                current_limit_a,
                resistance_ohm,
            }
            | CalibrateHall {
                max_power_loss_w,
                current_limit_a,
                resistance_ohm,
            } => {
                max_power_loss_w.is_finite()
                    && resistance_ohm.is_finite()
                    && current_limit_a.is_none_or(f32::is_finite)
            }
            MeasureFlux {
                max_power_loss_w,
                current_limit_a,
                resistance_ohm,
                inductance_h,
                openloop_erpm,
                ..
            } => {
                [
                    max_power_loss_w,
                    resistance_ohm,
                    inductance_h,
                    openloop_erpm,
                ]
                .iter()
                .all(|v| v.is_finite())
                    && current_limit_a.is_none_or(f32::is_finite)
            }
            MeasureCurrentOffsets { .. } => true,
        }
    }
}

#[derive(
    Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HallTransitionGeometryConfig {
    pub entry_angles: [f32; 8],
    pub next_states: [u8; 8],
}

#[derive(
    Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize, postcard_schema::Schema,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HallCalibrationConfig {
    /// Electrical angle (radians) for each raw Hall state (0-7)
    pub angles: [f32; 8],
    /// Validity flags for each Hall state
    pub valid: [bool; 8],
    /// Omitted geometry retains the legacy midpoint approximation. JSON may
    /// omit this field; flash migration uses explicit frozen layouts below.
    #[serde(default)]
    pub geometry: Option<HallTransitionGeometryConfig>,
}
