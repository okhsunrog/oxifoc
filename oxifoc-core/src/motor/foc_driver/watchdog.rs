//! Command freshness, engage grace and startup handoff timing.

use super::{BENCH_STALENESS_TIMEOUT_US, ControlMode, STARTUP_STALENESS_TIMEOUT_US};

#[derive(Default)]
pub(super) struct CommandWatchdog {
    last_cmd_tick: Option<u64>,
    engaged_tick: Option<u64>,
    was_starting: bool,
    was_active: bool,
}

impl CommandWatchdog {
    pub(super) fn note_command(&mut self, now_ticks: u64) {
        self.last_cmd_tick = Some(now_ticks);
    }

    /// A handoff reopens the engage window: the command pump can still be
    /// starved just after startup. Never judge startup-era staleness by the
    /// tighter drive bound retroactively. Preserve startup-before-engage order.
    #[inline(always)]
    pub(super) fn note_cycle(&mut self, now_ticks: u64, mode: ControlMode, starting: bool) {
        if self.was_starting && !starting {
            self.last_cmd_tick = Some(now_ticks);
            self.engaged_tick = Some(now_ticks);
        }
        self.was_starting = starting;
        let active = !matches!(
            mode,
            ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
        );
        if active && !self.was_active {
            self.engaged_tick = Some(now_ticks);
        }
        self.was_active = active;
    }

    pub(super) fn staleness_us(&self, mode: ControlMode, now_ticks: u64) -> Option<u64> {
        match mode {
            ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake => None,
            // A backwards timer glitch is not a stale command.
            _ => self.last_cmd_tick.map(|t| now_ticks.saturating_sub(t)),
        }
    }

    pub(super) fn expired(
        &self,
        mode: ControlMode,
        starting: bool,
        now_ticks: u64,
        staleness_us: Option<u64>,
        normal_timeout_us: u64,
    ) -> bool {
        self.timeout_us(mode, starting, now_ticks, normal_timeout_us)
            .zip(staleness_us)
            .is_some_and(|(bound, stale)| stale > bound)
    }

    fn timeout_us(
        &self,
        mode: ControlMode,
        starting: bool,
        now_ticks: u64,
        normal_timeout_us: u64,
    ) -> Option<u64> {
        Some(match mode {
            ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake => return None,
            ControlMode::OpenLoop { .. } | ControlMode::DirectVoltage { .. } => {
                BENCH_STALENESS_TIMEOUT_US
            }
            // Sensorless startup or the engage-grace window: bounded
            // device-local transient whose ISR load can starve the command
            // pump (see [`STARTUP_STALENESS_TIMEOUT_US`] / `engaged_tick`).
            _ if starting
                || self.engaged_tick.is_some_and(|t| {
                    now_ticks.saturating_sub(t) < STARTUP_STALENESS_TIMEOUT_US
                }) =>
            {
                STARTUP_STALENESS_TIMEOUT_US
            }
            _ => normal_timeout_us,
        })
    }
}
