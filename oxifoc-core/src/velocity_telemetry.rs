//! Fixed-size control-loop observations, correlated with the native FOC sequence.
//! Wire units: speed 1/8 electrical rad/s, current 1/100 A, Hall age 100 us.
//! Clipping and missing values are explicit flags, never plausible zero readings.

use serde::{Deserialize, Serialize};

pub const VELOCITY_RECORD_BYTES: usize = 32;
pub const ACTIVE: u16 = 1;
pub const UPDATED: u16 = 1 << 1;
pub const FEEDBACK_VALID: u16 = 1 << 2;
pub const LOWER_BOUND: u16 = 1 << 3;
pub const UPPER_BOUND: u16 = 1 << 4;
pub const VOLTAGE_POSITIVE: u16 = 1 << 5;
pub const VOLTAGE_NEGATIVE: u16 = 1 << 6;
pub const BLOCKED_POSITIVE: u16 = 1 << 7;
pub const BLOCKED_NEGATIVE: u16 = 1 << 8;
pub const GRANT_CLIPPED: u16 = 1 << 9;
pub const QUANTIZATION_CLIPPED: u16 = 1 << 10;
pub const HALL_UNAVAILABLE: u16 = 1 << 11;
pub const HALL_AGE_CLIPPED: u16 = 1 << 12;
pub const SESSION_START: u16 = 1 << 13;
/// Current-control samples reuse flags that are specific to the velocity PI.
pub const CURRENT_CONTROL: u16 = 1 << 14;
pub const STARTUP_CURRENT_SHAPING: u16 = FEEDBACK_VALID;
pub const LIMIT_SPEED: u16 = LOWER_BOUND;
pub const LIMIT_MODULATION: u16 = UPPER_BOUND;
pub const LIMIT_TEMPERATURE: u16 = VOLTAGE_POSITIVE;
pub const LIMIT_BUS_VOLTAGE: u16 = VOLTAGE_NEGATIVE;
pub const LIMIT_BUS_CURRENT: u16 = BLOCKED_POSITIVE;
pub const LIMIT_PHASE_CURRENT: u16 = BLOCKED_NEGATIVE;
pub const LIMIT_ANGLE: u16 = 1 << 15;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct VelocityDebugFrame {
    pub sequence: u32,
    /// Low 32 bits of the driver's monotonic microsecond clock.
    pub tick_us: u32,
    pub target: i16,
    pub reference: i16,
    pub feedback: i16,
    pub requested_iq: i16,
    pub granted_iq: i16,
    pub granted_id: i16,
    pub integral: i16,
    pub iq_min: i16,
    pub iq_max: i16,
    pub hall_age_100us: u16,
    pub flags: u16,
    /// Physical Hall pin bits; 0xff means unavailable, not a logical sector.
    pub hall_raw: u8,
    /// Hall direction: +1 clockwise, -1 counter-clockwise, 0 unknown/stopped.
    pub hall_direction: i8,
}

impl VelocityDebugFrame {
    pub fn encode(self) -> [u8; VELOCITY_RECORD_BYTES] {
        let mut b = [0; VELOCITY_RECORD_BYTES];
        b[..4].copy_from_slice(&self.sequence.to_le_bytes());
        b[4..8].copy_from_slice(&self.tick_us.to_le_bytes());
        for (i, value) in [
            self.target,
            self.reference,
            self.feedback,
            self.requested_iq,
            self.granted_iq,
            self.granted_id,
            self.integral,
            self.iq_min,
            self.iq_max,
        ]
        .into_iter()
        .enumerate()
        {
            b[8 + i * 2..10 + i * 2].copy_from_slice(&value.to_le_bytes());
        }
        b[26..28].copy_from_slice(&self.hall_age_100us.to_le_bytes());
        b[28..30].copy_from_slice(&self.flags.to_le_bytes());
        b[30] = self.hall_raw;
        b[31] = self.hall_direction as u8;
        b
    }

    pub fn decode(b: &[u8]) -> Option<Self> {
        if b.len() != VELOCITY_RECORD_BYTES {
            return None;
        }
        let word = |i| u16::from_le_bytes([b[i], b[i + 1]]);
        Some(Self {
            sequence: u32::from_le_bytes(b[..4].try_into().ok()?),
            tick_us: u32::from_le_bytes(b[4..8].try_into().ok()?),
            target: word(8) as i16,
            reference: word(10) as i16,
            feedback: word(12) as i16,
            requested_iq: word(14) as i16,
            granted_iq: word(16) as i16,
            granted_id: word(18) as i16,
            integral: word(20) as i16,
            iq_min: word(22) as i16,
            iq_max: word(24) as i16,
            hall_age_100us: word(26),
            flags: word(28),
            hall_raw: b[30],
            hall_direction: b[31] as i8,
        })
    }

    pub fn quantize(&mut self, value: f32, scale: f32) -> i16 {
        let scaled = value * scale;
        if !scaled.is_finite() || scaled < f32::from(i16::MIN) || scaled > f32::from(i16::MAX) {
            self.flags |= QUANTIZATION_CLIPPED;
        }
        // Truncate toward zero; error < one wire unit. Saturation is flagged.
        scaled as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn fixed_wire_preserves_signed_values_wraps_and_flags() {
        let f = VelocityDebugFrame {
            sequence: u32::MAX,
            tick_us: 3,
            target: -2560,
            feedback: i16::MIN,
            granted_iq: -32767,
            flags: ACTIVE | SESSION_START,
            hall_raw: 6,
            hall_direction: -1,
            ..Default::default()
        };
        assert_eq!(VelocityDebugFrame::decode(&f.encode()), Some(f));
        assert!(VelocityDebugFrame::decode(&[0; 31]).is_none());
        assert!(VelocityDebugFrame::decode(&[0; 33]).is_none());
    }
    #[test]
    fn quantization_reports_overflow_and_nonfinite_data() {
        let mut f = VelocityDebugFrame::default();
        assert_eq!(f.quantize(-0.453, 100.0), -45);
        assert_eq!(f.flags, 0);
        assert_eq!(f.quantize(400.0, 100.0), i16::MAX);
        assert_ne!(f.flags & QUANTIZATION_CLIPPED, 0);
        f.flags = 0;
        f.quantize(f32::NAN, 8.0);
        assert_ne!(f.flags & QUANTIZATION_CLIPPED, 0);
    }
}
