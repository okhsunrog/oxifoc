//! CAN frame counts, taken from ergot's own `Fragmenter` rather than from a
//! formula, so they follow ergot when its framing changes.

use ergot::interface_manager::utils::can::{Fragmenter, FrameEnd, max_message_len};

// The CAN id fields do not change the frame count or lengths.
const PRIORITY: u8 = 1;
const DST_NODE: u8 = 0x13;
const SRC_NODE: u8 = 0x01;
const TRANSFER_ID: u8 = 0;

/// Largest encoded frame the bridges accept on the CAN bus
/// (`oxifoc-bridge/src/transport/can.rs`, `MTU`).
pub const BRIDGE_CAN_MTU: usize = 512;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Bus {
    /// Classic CAN: 8-byte payloads.
    Classic,
    /// CAN FD: 64-byte payloads.
    Fd,
}

impl Bus {
    pub const fn max_payload(self) -> usize {
        match self {
            Self::Classic => 8,
            Self::Fd => 64,
        }
    }

    /// Longest message the Fragmenter accepts on this bus.
    pub const fn max_message_len(self) -> usize {
        max_message_len(self.max_payload())
    }
}

/// `None` when the Fragmenter refuses the message as too large. The count
/// depends only on the length, so a zeroed buffer stands in for the message.
pub fn frame_count(len: usize, bus: Bus) -> Option<usize> {
    let message = vec![0; len];
    Fragmenter::new(
        &message,
        bus.max_payload(),
        PRIORITY,
        DST_NODE,
        SRC_NODE,
        TRANSFER_ID,
    )
    .ok()
    .map(Iterator::count)
}

/// How one message splits into frames.
pub enum Fragments {
    /// Longer than the bus can carry in one message.
    TooLarge,
    Fit {
        frames: usize,
        /// Data bytes in the last frame.
        last_data: usize,
        /// Length of the last frame: the data length, or on CAN FD the next
        /// valid frame length when the data is padded.
        last_len: usize,
        /// Payload bytes of all frames, padding included.
        wire_bytes: usize,
        /// Fewest bytes the message must lose to need one frame fewer;
        /// `None` for a single frame.
        to_save: Option<usize>,
    },
}

impl Fragments {
    pub fn of(message: &[u8], bus: Bus) -> Self {
        let Ok(fragmenter) = Fragmenter::new(
            message,
            bus.max_payload(),
            PRIORITY,
            DST_NODE,
            SRC_NODE,
            TRANSFER_ID,
        ) else {
            return Self::TooLarge;
        };
        let frames: Vec<_> = fragmenter.collect();
        let last = frames.last().expect("the Fragmenter yields a frame");
        let count = frames.len();
        let last_data = if last.id.end() == Some(FrameEnd::LastPadded) {
            message.len() - bus.max_payload() * (count - 1)
        } else {
            usize::from(last.len)
        };
        let to_save = (count > 1).then(|| {
            (1..message.len())
                .find(|&cut| frame_count(message.len() - cut, bus).is_some_and(|n| n < count))
                .expect("an empty message is one frame")
        });
        Self::Fit {
            frames: count,
            last_data,
            last_len: usize::from(last.len),
            wire_bytes: frames.iter().map(|f| usize::from(f.len)).sum(),
            to_save,
        }
    }

    pub fn frames(&self) -> Option<usize> {
        match self {
            Self::TooLarge => None,
            Self::Fit { frames, .. } => Some(*frames),
        }
    }
}

/// Whether cutting 2, 4 or 6 bytes (what a smaller header could save)
/// removes a frame: the smallest such cut as `-k: n->m`, plus `-6 -> 1` when
/// six bytes would make the message a single frame and the first cut does
/// not already say so. Empty when no such cut removes a frame.
pub fn header_savings(len: usize, fragments: &Fragments, bus: Bus) -> String {
    let Some(frames) = fragments.frames() else {
        return String::new();
    };
    let mut flags = Vec::new();
    let first_cut = [2, 4, 6].into_iter().find_map(|cut| {
        let fewer = frame_count(len.checked_sub(cut).filter(|&l| l > 0)?, bus)?;
        (fewer < frames).then_some((cut, fewer))
    });
    if let Some((cut, fewer)) = first_cut {
        flags.push(format!("-{cut}: {frames}->{fewer}"));
    }
    let single_after_six = frames > 1 && frame_count(len.saturating_sub(6), bus) == Some(1);
    if single_after_six && first_cut.is_none_or(|(_, fewer)| fewer != 1) {
        flags.push("-6 -> 1".to_owned());
    }
    flags.join(", ")
}
