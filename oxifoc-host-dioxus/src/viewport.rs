//! Horizontal sample window. The newest sample is at offset zero.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Viewport {
    pub visible: u32,
    pub offset: u32,
}

impl Viewport {
    #[cfg(test)]
    pub const fn new(visible: u32) -> Self {
        Self { visible, offset: 0 }
    }

    /// Positive native wheel motion zooms in. While paused, preserve the
    /// sample under the pointer whenever the recorded history permits it.
    pub fn zoom(self, delta: f64, anchor: f64, paused: bool, capacity: u32) -> Self {
        if !delta.is_finite() || delta == 0.0 {
            return self;
        }
        let visible = (self.visible as f64 * (-delta / 120.0).clamp(-20.0, 20.0).exp())
            .round()
            .clamp(2.0, capacity as f64) as u32;
        let offset = if paused {
            let from_right = 1.0 - anchor.clamp(0.0, 1.0);
            let shift = from_right * (self.visible as f64 - visible as f64);
            (self.offset as f64 + shift)
                .round()
                .clamp(0.0, capacity.saturating_sub(visible) as f64) as u32
        } else {
            0
        };
        Self { visible, offset }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn paused_zoom_keeps_the_sample_under_the_cursor() {
        let old = Viewport {
            visible: 2400,
            offset: 8000,
        };
        for anchor in [0.0, 0.25, 0.5, 0.9, 1.0] {
            let new = old.zoom(30.0, anchor, true, 65536);
            let point = |v: Viewport| v.offset as f64 + (1.0 - anchor) * (v.visible - 1) as f64;
            assert!((point(old) - point(new)).abs() <= 0.5);
        }
    }

    #[test]
    fn zoom_respects_history_limits_and_live_edge() {
        let old = Viewport {
            visible: 1200,
            offset: 2000,
        };
        assert_eq!(old.zoom(-1e9, 0.5, true, 65536), Viewport::new(65536));
        assert_eq!(old.zoom(1e9, 1.0, false, 65536), Viewport::new(2));
        assert_eq!(old.zoom(f64::NAN, 0.5, true, 65536), old);
    }
}
