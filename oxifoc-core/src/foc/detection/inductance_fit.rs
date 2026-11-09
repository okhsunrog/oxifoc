//! Streaming identification of a stationary two-axis RL response.
//!
//! Each interval obeys di = B (u - R i_previous) + c under zero-order hold,
//! with B = (I - exp(-R dt L^-1))/R. A fitted intercept absorbs a constant
//! holding-voltage error. Carrier zeros require no division or fabricated data.

use crate::foc::detection::types::DetectionError;
use crate::foc::fast_math::sqrtf;

#[derive(Clone, Default)]
pub(super) struct RlWindow {
    n: u32,
    x: [f32; 2],
    y: [f32; 2],
    xx: [f32; 3],
    yx: [f32; 4],
}

pub(super) struct Fit {
    pub ld: f32,
    pub lq: f32,
    pub saliency: f32,
    pub quadrature: f32,
}

impl RlWindow {
    pub fn record(&mut self, x: [f32; 2], y: [f32; 2]) {
        self.n += 1;
        for j in 0..2 {
            self.x[j] += x[j];
            self.y[j] += y[j];
        }
        self.xx[0] += x[0] * x[0];
        self.xx[1] += x[0] * x[1];
        self.xx[2] += x[1] * x[1];
        self.yx[0] += y[0] * x[0];
        self.yx[1] += y[0] * x[1];
        self.yx[2] += y[1] * x[0];
        self.yx[3] += y[1] * x[1];
    }

    pub fn finish(&self, resistance: f32, dt: f32) -> Result<Fit, DetectionError> {
        if self.n < 8 || !resistance.is_finite() || resistance < 0.0 || !dt.is_finite() || dt <= 0.0
        {
            return Err(DetectionError::InsufficientSamples);
        }
        let n = self.n as f32;
        // Center both regressors and responses: c is a nuisance parameter,
        // rather than a guessed voltage correction saved to the motor config.
        let g00 = self.xx[0] - self.x[0] * self.x[0] / n;
        let g01 = self.xx[1] - self.x[0] * self.x[1] / n;
        let g11 = self.xx[2] - self.x[1] * self.x[1] / n;
        let determinant = g00 * g11 - g01 * g01;
        if !determinant.is_finite()
            || determinant <= 1e-6 * (g00 + g11) * (g00 + g11)
            || g00 + g11 <= 1e-12
        {
            return Err(DetectionError::LowConfidence);
        }
        let h00 = self.yx[0] - self.y[0] * self.x[0] / n;
        let h01 = self.yx[1] - self.y[0] * self.x[1] / n;
        let h10 = self.yx[2] - self.y[1] * self.x[0] / n;
        let h11 = self.yx[3] - self.y[1] * self.x[1] / n;
        let b00 = (h00 * g11 - h01 * g01) / determinant;
        let b01 = (h01 * g00 - h00 * g01) / determinant;
        let b10 = (h10 * g11 - h11 * g01) / determinant;
        let b11 = (h11 * g00 - h10 * g01) / determinant;
        // A stationary reciprocal RL model has symmetric B. Reject a gross
        // nonreciprocal response, e.g. wrong command/current pipeline pairing.
        if (b01 - b10).abs() > 0.5 * (b00 + b11).abs() {
            return Err(DetectionError::LowConfidence);
        }
        let cross = (b01 + b10) * 0.5;
        let difference = b00 - b11;
        let split = sqrtf(difference * difference + 4.0 * cross * cross);
        let fast = (b00 + b11 + split) * 0.5;
        let slow = (b00 + b11 - split) * 0.5;
        if !fast.is_finite() || !slow.is_finite() || slow <= 0.0 || resistance * fast >= 1.0 {
            return Err(DetectionError::LowConfidence);
        }
        let convert = |b: f32| {
            if resistance == 0.0 {
                dt / b
            } else {
                -resistance * dt / libm::log1pf(-resistance * b)
            }
        };
        let small = convert(fast);
        let large = convert(slow);
        // Keep signed d/q labeling relative to the assumed rotor lock. Do not
        // hard-code Ld < Lq: inverse saliency must remain observable.
        let (ld, lq) = if difference >= 0.0 {
            (small, large)
        } else {
            (large, small)
        };
        let inverse_split = (1.0 / small - 1.0 / large) * 0.5;
        let (saliency, quadrature) = if split > 1e-12 {
            (
                inverse_split * difference / split,
                inverse_split * 2.0 * cross / split,
            )
        } else {
            (0.0, 0.0)
        };
        Ok(Fit {
            ld,
            lq,
            saliency,
            quadrature,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_unexcited_axis_and_zero_response() {
        let mut one_axis = RlWindow::default();
        let mut no_response = RlWindow::default();
        for k in 0..32 {
            let x = [(k % 3) as f32, (k % 5) as f32];
            one_axis.record([x[0], 0.0], x);
            no_response.record(x, [0.0; 2]);
        }
        assert!(one_axis.finish(0.03, 50e-6).is_err());
        assert!(no_response.finish(0.03, 50e-6).is_err());
    }

    #[test]
    fn recovers_signed_axes_and_constant_bias_under_exact_zoh() {
        for (ld, lq) in [(18e-6, 28e-6), (28e-6, 18e-6), (25e-6, 25e-6)] {
            for r in [0.0, 0.03, 0.2] {
                let dt = 1.0 / 20_004.272;
                let b = [ld, lq].map(|l| {
                    if r == 0.0 {
                        dt / l
                    } else {
                        -libm::expm1f(-r * dt / l) / r
                    }
                });
                let mut fit = RlWindow::default();
                for k in 0..32 {
                    let x = [(k % 3) as f32 * 0.1, (k % 5) as f32 * 0.1];
                    fit.record(x, [b[0] * x[0] + 0.023, b[1] * x[1] - 0.041]);
                }
                let result = fit.finish(r, dt).unwrap();
                assert!((result.ld / ld - 1.0).abs() < 1e-4);
                assert!((result.lq / lq - 1.0).abs() < 1e-4);
            }
        }
    }
}
