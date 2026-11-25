//! Wire sizes of the OxiFOC ICD and of ergot's control plane on ergot's CAN
//! transport. Every message is encoded with ergot's own frame encoder
//! (`wire_frames::encode_frame_ty`) and split with ergot's own `Fragmenter`
//! at 8-byte (classic CAN) and 64-byte (CAN FD) payloads, so the tables
//! follow the message definitions and ergot's framing when either changes.
//!
//! Each message is measured at the smallest and largest value its schema
//! allows (per top-level enum variant) and at hand-built representative
//! values; the raw telemetry blocks come from captured records.

mod catalog;
mod frames;
mod header;
mod messages;
mod raw_blocks;
mod report;
mod synth;

use std::{path::PathBuf, process::ExitCode};

use clap::Parser;
use serde::Serialize;

use crate::{
    messages::SESSION,
    raw_blocks::{BUNDLED_FIXTURE, RawBlocks},
    report::Fixture,
};

#[derive(Parser)]
#[command(about = "Encoded sizes and CAN frame counts of every ICD and ergot message")]
struct Args {
    /// Print every table: the full per-message columns, the fixture
    /// summary, protocol error frames, header savings near a frame boundary,
    /// larger nets and the capacities the maximum values were built with.
    #[arg(long)]
    full: bool,
    /// Concatenated Packed28 raw blocks for the `telemetry/raw_block` rows,
    /// instead of the bundled fixture.
    fixture: Option<PathBuf>,
}

/// Postcard length of `value`.
pub fn encoded_len<T: Serialize + ?Sized>(value: &T) -> usize {
    postcard::to_stdvec(value)
        .expect("postcard encodes into a Vec")
        .len()
}

fn main() -> ExitCode {
    let args = Args::parse();
    let (name, data) = match &args.fixture {
        Some(path) => match std::fs::read(path) {
            Ok(data) => (path.display().to_string(), data),
            Err(error) => {
                eprintln!("cannot read {}: {error}", path.display());
                return ExitCode::FAILURE;
            }
        },
        None => (
            "bundled fixture/packed28-blocks.bin".to_owned(),
            BUNDLED_FIXTURE.to_vec(),
        ),
    };
    let raw_blocks = match RawBlocks::from_fixture(&data, SESSION) {
        Ok(blocks) => Some(blocks),
        Err(error) => {
            eprintln!("{name}: not raw blocks ({error:?}); no representative raw blocks");
            None
        }
    };
    let catalog = messages::catalog(raw_blocks.as_ref());
    for line in &catalog.not_measured {
        eprintln!("not measured: {line}");
    }

    let mut out = String::new();
    let written = if args.full {
        let fixture = Fixture {
            name: &name,
            blocks: raw_blocks.as_ref(),
        };
        report::full(&mut out, &catalog, &fixture)
    } else {
        report::condensed(&mut out, &catalog)
    };
    written.expect("formatting into a String does not fail");
    print!("{out}");
    ExitCode::SUCCESS
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every message builds at its schema extremes and every table renders;
    /// a message type the synthesizer cannot build fails here.
    #[test]
    fn every_message_is_measured() {
        let blocks = RawBlocks::from_fixture(BUNDLED_FIXTURE, SESSION).unwrap();
        assert_eq!(blocks.packed_full.len(), 8);
        let catalog = messages::catalog(Some(&blocks));
        assert_eq!(catalog.not_measured, Vec::<String>::new());
        let fixture = Fixture {
            name: "bundled",
            blocks: Some(&blocks),
        };
        let mut out = String::new();
        report::full(&mut out, &catalog, &fixture).unwrap();
        report::condensed(&mut out, &catalog).unwrap();
    }
}
