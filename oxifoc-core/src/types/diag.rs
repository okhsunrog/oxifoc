//! Controller diagnostics (`req/diag`): what an assembled board, without a
//! debug probe, can still tell about its own health.
//!
//! The ISR keeps counting as before; a once-per-second task folds the counts
//! into an [`IsrWindow`] and the running totals of a [`DiagSummary`]. Reading
//! is idempotent, so several clients can poll without disturbing each other;
//! `window` tells a poller whether it missed one.

use heapless::String;
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DiagRequest {
    /// Totals since boot and the reset cause.
    Summary,
    /// The last completed one-second window.
    IsrWindow,
    /// The last panic or HardFault record, kept across resets until cleared.
    Crash,
    /// Forget the crash record (answers `Crash(None)`).
    ClearCrash,
}

// postcard encodes the variant index — append only.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DiagResponse {
    Summary(DiagSummary),
    IsrWindow(IsrWindow),
    /// No completed window yet, or this firmware does not measure it.
    Unavailable,
    Crash(Option<CrashRecord>),
}

pub const CRASH_FILE_LEN: usize = 48;
pub const CRASH_MESSAGE_LEN: usize = 96;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CrashKind {
    Panic,
    HardFault,
}

/// What the firmware recorded when it last panicked or hard-faulted. It
/// lives in RAM that survives a reset (not a power cycle) and stays until a
/// client clears it, so a crash followed by more reboots is not lost.
///
/// The bridge records panics only: its CPU exceptions arrive as panics
/// whose message names the cause and the faulting address.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CrashRecord {
    pub kind: CrashKind,
    /// Boots since the crash (1 = the reboot right after it).
    pub boots_since: u16,
    /// Time since boot when it happened.
    pub uptime_ms: u32,
    /// Faulting PC and LR for a HardFault; 0 for a panic.
    pub pc: u32,
    pub lr: u32,
    /// Configurable and HardFault status registers (CFSR, HFSR).
    pub cfsr: u32,
    pub hfsr: u32,
    /// Panic location and message, truncated to fit.
    pub file: String<CRASH_FILE_LEN>,
    pub line: u32,
    pub message: String<CRASH_MESSAGE_LEN>,
}

/// Why the controller last came out of reset (the MCU's reset flags, read
/// once at boot and then cleared). Several can be set at once: a power-on
/// reset also reports `pin` and `brown_out` on the STM32F4.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ResetFlags {
    pub power_on: bool,
    pub brown_out: bool,
    pub pin: bool,
    pub software: bool,
    pub independent_watchdog: bool,
    pub window_watchdog: bool,
    pub low_power: bool,
}

/// Totals since boot. Counters only grow, so a client compares two reads
/// (a bench run checks before/after) and a lost read costs nothing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DiagSummary {
    /// Changes on every boot; a different value means the counters restarted.
    pub boot_epoch: u64,
    pub uptime_ms: u64,
    pub reset: ResetFlags,
    /// One-second windows completed since boot.
    pub windows: u32,
    /// Control ISRs longer than one PWM period.
    pub isr_overruns: u32,
    /// Longest control ISR body seen, CPU cycles.
    pub isr_max_cycles: u32,
    /// Entry-to-entry intervals well beyond / short of one period.
    pub isr_long_intervals: u32,
    pub isr_short_intervals: u32,
    /// Fast-telemetry samples queued and dropped by the producer.
    pub telemetry_pushed: u32,
    pub telemetry_dropped: u32,
    /// Hall edges and capture overruns; edges serve as an odometer source.
    pub hall_edges: u32,
    pub hall_overcaptures: u32,
}

/// One second of control-ISR timing, as the `isr/s` RTT line used to print.
/// Cycle counts are DWT CPU cycles; averages are `cycles / samples`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IsrWindow {
    /// Sequence number of this window (1 = the first after boot).
    pub window: u32,
    /// CPU cycles the window spanned; 0 when the cycle counter restarted
    /// inside it (a debugger attach), so loads are not computable.
    pub elapsed_cycles: u32,
    /// Cycles of one PWM period, the ISR budget.
    pub budget_cycles: u32,
    pub isr: CycleStats,
    /// Only ISRs in active current/velocity control with a nonzero speed.
    pub motion: CycleStats,
    pub overruns: u32,
    pub intervals: IntervalStats,
    pub telemetry: TelemetryStats,
    pub hall: HallStats,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CycleStats {
    pub samples: u32,
    pub sum_cycles: u32,
    pub max_cycles: u32,
}

/// ISR entry-to-entry intervals (CPU cycles).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IntervalStats {
    pub samples: u32,
    pub sum_cycles: u32,
    pub min_cycles: u32,
    pub max_cycles: u32,
    pub long: u32,
    pub short: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TelemetryStats {
    pub pushed: u32,
    pub dropped: u32,
    pub read: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HallStats {
    /// Raw sensor pins (bits 0..=2) at the end of the window.
    pub state: u8,
    pub edges: u32,
    pub overcaptures: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BridgeDiagRequest {
    /// Uptime, reset reason and the counters of every link.
    Links,
    /// The bridge's last panic record, kept across resets until cleared.
    Crash,
    /// Forget the crash record (answers `Crash(None)`).
    ClearCrash,
}

// postcard encodes the variant index — append only.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BridgeDiagResponse {
    Links(BridgeDiag),
    Crash(Option<CrashRecord>),
}

/// Bridge diagnostics (`req/bridge_diag`, served by the bridge itself): the
/// links it carries, reachable over BLE even when the controller is silent.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BridgeDiag {
    pub uptime_ms: u64,
    /// The bridge MCU's raw reset-reason code (ESP32-C5 `SocResetReason`;
    /// 0 = unknown, 0x01 power-on, 0x03 software, 0x0F brown-out). A panic
    /// and an update both end in a software reset.
    pub reset_reason: u8,
    pub spi: SpiLinkDiag,
    pub usb: UsbLinkDiag,
    pub ble: [BleLinkDiag; BRIDGE_BLE_LINKS],
    pub wifi: WifiLinkDiag,
}

pub const BRIDGE_BLE_LINKS: usize = 2;

/// SPI link to the controller. Counters only grow.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SpiLinkDiag {
    pub slots: u32,
    pub errors: u32,
    pub length_errors: u32,
    pub crc_errors: u32,
    pub header_errors: u32,
    /// Controller boots the bridge has seen (its link epoch changed),
    /// including the first one after the bridge started.
    pub peer_boots: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UsbLinkDiag {
    /// Frames the outbound queue had no room for.
    pub dropped: u32,
    pub write_errors: u32,
    /// Largest single write grant, bytes.
    pub max_grant: u32,
}

/// Ergot TCP link over Wi-Fi; all zero in an image without it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WifiLinkDiag {
    pub connected: bool,
    pub connects: u32,
    /// Frames the outbound queue had no room for.
    pub dropped: u32,
    /// Most bytes the TCP transmit buffer has held.
    pub max_backlog: u32,
}

/// One BLE central slot. Counters only grow.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BleLinkDiag {
    pub connected: bool,
    /// Encrypted with a stored bond.
    pub trusted: bool,
    pub connects: u32,
    pub disconnects: u32,
    /// HCI reason of the last disconnect (0 = none yet).
    pub last_disconnect_reason: u8,
    /// Links dropped for neither bonding nor pairing in time.
    pub untrusted_drops: u32,
    pub notify_errors: u32,
    /// Frames the link's outbound queue had no room for.
    pub dropped: u32,
}

impl CycleStats {
    pub fn average(&self) -> Option<u32> {
        self.sum_cycles.checked_div(self.samples)
    }
}

impl IntervalStats {
    pub fn average(&self) -> Option<u32> {
        self.sum_cycles.checked_div(self.samples)
    }
}

impl IsrWindow {
    /// Share of the window spent in the ISR, percent.
    pub fn load_percent(&self) -> Option<u32> {
        let elapsed = u64::from(self.elapsed_cycles);
        (elapsed != 0).then(|| (u64::from(self.isr.sum_cycles) * 100 / elapsed) as u32)
    }
}

impl DiagSummary {
    /// Fold one completed window into the totals.
    pub fn absorb(&mut self, window: &IsrWindow) {
        self.windows = self.windows.wrapping_add(1);
        self.isr_overruns = self.isr_overruns.wrapping_add(window.overruns);
        self.isr_max_cycles = self.isr_max_cycles.max(window.isr.max_cycles);
        self.isr_long_intervals = self.isr_long_intervals.wrapping_add(window.intervals.long);
        self.isr_short_intervals = self
            .isr_short_intervals
            .wrapping_add(window.intervals.short);
        self.telemetry_pushed = self.telemetry_pushed.wrapping_add(window.telemetry.pushed);
        self.telemetry_dropped = self
            .telemetry_dropped
            .wrapping_add(window.telemetry.dropped);
        self.hall_edges = self.hall_edges.wrapping_add(window.hall.edges);
        self.hall_overcaptures = self
            .hall_overcaptures
            .wrapping_add(window.hall.overcaptures);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every response must fit one BLE notification (505 bytes, of which
    /// the Ergot header takes up to 24).
    #[test]
    fn largest_responses_fit_a_ble_frame() {
        let stats = CycleStats {
            samples: u32::MAX,
            sum_cycles: u32::MAX,
            max_cycles: u32::MAX,
        };
        let window = IsrWindow {
            window: u32::MAX,
            elapsed_cycles: u32::MAX,
            budget_cycles: u32::MAX,
            isr: stats,
            motion: stats,
            overruns: u32::MAX,
            intervals: IntervalStats {
                samples: u32::MAX,
                sum_cycles: u32::MAX,
                min_cycles: u32::MAX,
                max_cycles: u32::MAX,
                long: u32::MAX,
                short: u32::MAX,
            },
            telemetry: TelemetryStats {
                pushed: u32::MAX,
                dropped: u32::MAX,
                read: u32::MAX,
            },
            hall: HallStats {
                state: u8::MAX,
                edges: u32::MAX,
                overcaptures: u32::MAX,
            },
        };
        let summary = DiagSummary {
            boot_epoch: u64::MAX,
            uptime_ms: u64::MAX,
            reset: ResetFlags {
                power_on: true,
                brown_out: true,
                pin: true,
                software: true,
                independent_watchdog: true,
                window_watchdog: true,
                low_power: true,
            },
            windows: u32::MAX,
            isr_overruns: u32::MAX,
            isr_max_cycles: u32::MAX,
            isr_long_intervals: u32::MAX,
            isr_short_intervals: u32::MAX,
            telemetry_pushed: u32::MAX,
            telemetry_dropped: u32::MAX,
            hall_edges: u32::MAX,
            hall_overcaptures: u32::MAX,
        };
        // Worst case for the strings: every character takes two UTF-8 bytes.
        let mut message = String::<CRASH_MESSAGE_LEN>::new();
        while message.push('\u{7ff}').is_ok() {}
        let mut file = String::<CRASH_FILE_LEN>::new();
        while file.push('\u{7ff}').is_ok() {}
        let crash = CrashRecord {
            kind: CrashKind::HardFault,
            boots_since: u16::MAX,
            uptime_ms: u32::MAX,
            pc: u32::MAX,
            lr: u32::MAX,
            cfsr: u32::MAX,
            hfsr: u32::MAX,
            file,
            line: u32::MAX,
            message,
        };
        let link = BleLinkDiag {
            connected: true,
            trusted: true,
            connects: u32::MAX,
            disconnects: u32::MAX,
            last_disconnect_reason: u8::MAX,
            untrusted_drops: u32::MAX,
            notify_errors: u32::MAX,
            dropped: u32::MAX,
        };
        let bridge = BridgeDiag {
            uptime_ms: u64::MAX,
            reset_reason: u8::MAX,
            spi: SpiLinkDiag {
                slots: u32::MAX,
                errors: u32::MAX,
                length_errors: u32::MAX,
                crc_errors: u32::MAX,
                header_errors: u32::MAX,
                peer_boots: u32::MAX,
            },
            usb: UsbLinkDiag {
                dropped: u32::MAX,
                write_errors: u32::MAX,
                max_grant: u32::MAX,
            },
            ble: [link; BRIDGE_BLE_LINKS],
            wifi: WifiLinkDiag {
                connected: true,
                connects: u32::MAX,
                dropped: u32::MAX,
                max_backlog: u32::MAX,
            },
        };
        let mut buf = [0u8; 512];
        for response in [
            BridgeDiagResponse::Links(bridge),
            BridgeDiagResponse::Crash(Some(crash.clone())),
        ] {
            let len = postcard::to_slice(&response, &mut buf).unwrap().len();
            assert!(len + 24 <= 505, "{response:?}: {len} bytes");
        }
        for response in [
            DiagResponse::IsrWindow(window),
            DiagResponse::Summary(summary),
            DiagResponse::Crash(Some(crash)),
        ] {
            let len = postcard::to_slice(&response, &mut buf).unwrap().len();
            assert!(len + 24 <= 505, "{response:?}: {len} bytes");
        }
    }

    #[test]
    fn summary_accumulates_windows() {
        let mut summary = DiagSummary::default();
        let window = IsrWindow {
            overruns: 2,
            isr: CycleStats {
                max_cycles: 7_000,
                ..Default::default()
            },
            hall: HallStats {
                edges: 430,
                ..Default::default()
            },
            ..Default::default()
        };
        summary.absorb(&window);
        summary.absorb(&IsrWindow {
            isr: CycleStats {
                max_cycles: 6_000,
                ..Default::default()
            },
            ..window
        });
        assert_eq!(summary.windows, 2);
        assert_eq!(summary.isr_overruns, 4);
        assert_eq!(summary.isr_max_cycles, 7_000);
        assert_eq!(summary.hall_edges, 860);
    }
}
