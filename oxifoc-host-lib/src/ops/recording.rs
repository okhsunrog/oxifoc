//! A fast telemetry session recorded on the bridge: start, stop, fetch, clear.

use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use oxifoc_core::{
    raw_record::{RecordMode, RecordRequest, RecordResponse, RecordState, RecordStatus},
    raw_telemetry::RawStreamDecoder,
    types::FastTelemetry,
};

use crate::{CommandSender, HostCommand};

/// Tries of one chunk read before a fetch gives up. Reads repeat safely.
const READ_TRIES: usize = 4;

pub fn request(cmd: &CommandSender, request: RecordRequest) -> Result<RecordResponse> {
    let (tx, rx) = tokio::sync::oneshot::channel();
    cmd.send(HostCommand::RawRecord(request, tx))
        .context("send recording request")?;
    rx.blocking_recv()
        .context("backend dropped the recording request")?
        .context("recording request failed")
}

fn status_of(response: RecordResponse) -> Result<RecordStatus> {
    match response {
        RecordResponse::Status(status) => Ok(status),
        RecordResponse::Refused(refusal) => bail!("the bridge refused: {refusal:?}"),
        RecordResponse::Data(_) => bail!("the bridge answered with data to a status request"),
    }
}

pub fn status(cmd: &CommandSender) -> Result<RecordStatus> {
    status_of(request(cmd, RecordRequest::Status)?)
}

/// Start a recording at `fast_hz`, or at the controller's rate if that is
/// lower. The token and the start time come from this host's clock.
pub fn start(cmd: &CommandSender, fast_hz: u16, mode: RecordMode) -> Result<RecordStatus> {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    status_of(request(
        cmd,
        RecordRequest::Start {
            session: (now.as_nanos() as u64).max(1),
            fast_hz,
            mode,
            started_unix_ms: now.as_millis() as u64,
        },
    )?)
}

pub fn stop(cmd: &CommandSender) -> Result<RecordStatus> {
    status_of(request(cmd, RecordRequest::Stop)?)
}

pub fn clear(cmd: &CommandSender) -> Result<RecordStatus> {
    status_of(request(cmd, RecordRequest::Clear)?)
}

/// Read a stopped recording whole, `chunk` bytes a request; `progress` is
/// told the bytes read so far and the total.
pub fn fetch(
    cmd: &CommandSender,
    chunk: u16,
    mut progress: impl FnMut(usize, usize),
) -> Result<(RecordStatus, Vec<u8>)> {
    let status = status(cmd)?;
    if !matches!(status.state, RecordState::Stopped(_)) {
        bail!("no stopped recording to fetch: {:?}", status.state);
    }
    let total = status.bytes as usize;
    let mut bytes = Vec::with_capacity(total);
    loop {
        let read = RecordRequest::Read {
            offset: bytes.len() as u32,
            max_bytes: chunk,
        };
        let mut tries = 0;
        let data = loop {
            tries += 1;
            match request(cmd, read) {
                Ok(RecordResponse::Data(data)) => break data.data,
                Ok(other) => bail!("the bridge answered {other:?} to a read"),
                Err(error) if tries == READ_TRIES => {
                    return Err(error).with_context(|| format!("read at byte {}", bytes.len()));
                }
                Err(_) => {}
            }
        };
        bytes.extend_from_slice(&data);
        progress(bytes.len(), total);
        if data.len() < usize::from(chunk) {
            break;
        }
    }
    if bytes.len() != total {
        bail!("fetched {} bytes of a {total}-byte recording", bytes.len());
    }
    Ok((status, bytes))
}

/// The records of a fetched recording.
pub struct Records {
    pub samples: Vec<FastTelemetry>,
    pub blocks: u32,
    /// Blocks missing from the sequence between the first and the last.
    pub missing_blocks: u32,
}

/// Decode a recording: the blocks of `session`, one after another. A ring
/// begins at whatever block it still holds, so the first index is free.
pub fn decode(bytes: &[u8], session: u64) -> Result<Records> {
    let mut records = Records {
        samples: Vec::new(),
        blocks: 0,
        missing_blocks: 0,
    };
    let mut next = None;
    let mut foreign = false;
    RawStreamDecoder::new()
        .push(bytes, |block| {
            foreign |= block.header.session != session;
            if let Some(next) = next {
                let missing = block.header.block_index.wrapping_sub(next);
                records.missing_blocks = records.missing_blocks.wrapping_add(missing);
            }
            next = Some(block.header.block_index.wrapping_add(1));
            records.blocks += 1;
            records.samples.extend(block.samples);
        })
        .map_err(|error| anyhow::anyhow!("malformed recording: {error:?}"))?;
    if foreign {
        bail!("the recording holds blocks of another session");
    }
    Ok(records)
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxifoc_core::{
        raw_delta::compress_envelope, raw_record::Recording, raw_telemetry::RawBlockEncoder,
    };

    fn block(session: u64, index: u32) -> Vec<u8> {
        let mut encoder = RawBlockEncoder::new();
        encoder.begin(session, index, 1);
        for record in 0..64_u16 {
            let frame = FastTelemetry {
                seq: (index as u16).wrapping_mul(64).wrapping_add(record),
                angle: record * 100,
                pwm_arr: 4200,
                ..FastTelemetry::default()
            };
            assert!(encoder.push(&frame));
        }
        let plain = encoder.finish(0).to_vec();
        let mut relayed = vec![0; plain.len()];
        let len = compress_envelope(&plain, &mut relayed).unwrap();
        relayed.truncate(len);
        relayed
    }

    #[test]
    fn a_ring_decodes_from_its_oldest_block_and_counts_a_missing_one() {
        let mut memory = vec![0; 4000];
        let mut recording = Recording::new(&mut memory);
        for index in (0..200).filter(|&index| index != 190) {
            assert!(recording.push(&block(9, index), RecordMode::Ring));
        }
        let mut bytes = vec![0; recording.len()];
        assert_eq!(recording.read(0, &mut bytes), bytes.len());
        let records = decode(&bytes, 9).unwrap();
        assert!(records.blocks > 5 && records.blocks < 199);
        assert_eq!(records.samples.len(), records.blocks as usize * 64);
        assert_eq!(records.samples.last().unwrap().seq, 199_u16 * 64 + 63);
        // Block 190 is within the newest blocks a 4000-byte ring keeps.
        assert_eq!(records.missing_blocks, 1);
        assert!(decode(&bytes, 8).is_err());
        assert!(
            decode(&bytes[..bytes.len() - 1], 9).is_ok_and(|cut| cut.blocks == records.blocks - 1)
        );
    }
}
