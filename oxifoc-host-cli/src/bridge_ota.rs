//! Send an ESP-IDF application image to the C5 bridge over Ergot.

use std::{
    fs,
    path::{Path, PathBuf},
    thread,
    time::{Duration, Instant},
};

use anyhow::{Context, Result, bail, ensure};
use clap::Subcommand;
use heapless::Vec as Chunk;
use oxifoc_core::types::bridge_auth::{Key, Nonce};
use oxifoc_core::types::bridge_ota::{
    CHUNK_SIZE, OtaAction, OtaError, OtaRequest, OtaResponse, UploadRequest, is_bridge_image, sign,
};
use oxifoc_host_lib::{HostRuntime, ops::bridge_ota::Client};
use serde_json::json;
use sha2::{Digest as _, Sha256};

use crate::{bridge_key, emit};

#[derive(Subcommand)]
pub enum OtaCommand {
    /// Show the running firmware slot and available space
    Status,
    /// Upload an ESP-IDF app binary and verify the bridge boots from the new slot
    Update { image: PathBuf },
    /// Discard an interrupted upload
    Abort,
}

pub fn run(runtime: &HostRuntime, action: OtaCommand, wait: Duration, json: bool) -> Result<()> {
    let mut client = Client {
        cmd: &runtime.cmd_tx,
        target: None,
    };
    let (nonce, booted_slot, capacity) = info(&mut client, wait)?;
    match action {
        OtaCommand::Status => emit(
            json,
            json!({ "booted_slot": booted_slot, "capacity": capacity }),
            format!("Running ota_{booted_slot}. Inactive slot capacity {capacity} bytes."),
        ),
        OtaCommand::Abort => {
            let key = bridge_key()?;
            let reply = authenticated(&mut client, &key, nonce, OtaAction::Abort)?;
            ensure!(
                matches!(reply, OtaResponse::Aborted),
                "unexpected reply: {reply:?}"
            );
            emit(json, json!({"aborted": true}), "Upload discarded.".into());
        }
        OtaCommand::Update { image } => {
            update(&mut client, &image, nonce, booted_slot, capacity, json)?;
        }
    }
    Ok(())
}

fn info(client: &mut Client<'_>, wait: Duration) -> Result<(Nonce, u8, u32)> {
    let deadline = Instant::now() + wait;
    loop {
        client.target = None;
        match client.request(OtaRequest::Info) {
            Ok(OtaResponse::Info {
                nonce,
                booted_slot,
                capacity,
            }) => {
                return Ok((nonce, booted_slot, capacity));
            }
            Ok(reply) => bail!("bridge OTA status: {reply:?}"),
            Err(_) if Instant::now() < deadline => thread::sleep(Duration::from_millis(250)),
            Err(err) => return Err(err),
        }
    }
}

fn authenticated(
    client: &mut Client<'_>,
    key: &Key,
    nonce: Nonce,
    action: OtaAction,
) -> Result<OtaResponse> {
    let upload = UploadRequest { nonce, action };
    let tag = sign(key, &upload).context("encode bridge OTA request")?;
    let request = OtaRequest::Authenticated { upload, tag };
    // Repeating an identical authenticated request is safe. The bridge
    // compares previously written bytes before acknowledging a duplicate. Let
    // the backend reconnect and rediscover the endpoint if the transport drops.
    let deadline = Instant::now() + Duration::from_secs(25);
    loop {
        match client.request(request.clone()) {
            Ok(response) => return Ok(response),
            Err(error) if Instant::now() >= deadline => return Err(error),
            Err(_) => {
                client.target = None;
                thread::sleep(Duration::from_millis(250));
            }
        }
    }
}

fn update(
    client: &mut Client<'_>,
    path: &Path,
    nonce: Nonce,
    booted_slot: u8,
    capacity: u32,
    json: bool,
) -> Result<()> {
    let key = bridge_key()?;
    let image = fs::read(path).with_context(|| format!("read {}", path.display()))?;
    ensure!(
        is_bridge_image(&image),
        "expected an ESP32-C5 oxifoc-bridge app binary"
    );
    let length = u32::try_from(image.len())?;
    ensure!(
        length <= capacity && length.is_multiple_of(4),
        "image does not fit the inactive slot"
    );
    let digest = Sha256::digest(&image).into();
    let reply = authenticated(client, &key, nonce, OtaAction::Begin { length, digest })?;
    let OtaResponse::Progress { mut offset } = reply else {
        bail!("bridge refused the upload: {reply:?}");
    };
    ensure!(offset <= length, "bridge returned an invalid offset");
    if !json {
        eprintln!(
            "Uploading {length} bytes to ota_{} from byte {offset}.",
            booted_slot ^ 1
        );
    }
    let mut next_report = offset + 65_536;
    while offset < length {
        let end = (offset + CHUNK_SIZE as u32).min(length);
        let data = Chunk::from_slice(&image[offset as usize..end as usize])
            .map_err(|_| anyhow::anyhow!("invalid OTA chunk length"))?;
        let reply = authenticated(client, &key, nonce, OtaAction::Chunk { offset, data })?;
        let OtaResponse::Progress { offset: confirmed } = reply else {
            bail!("bridge rejected chunk at byte {offset}: {reply:?}");
        };
        ensure!(
            confirmed == end,
            "bridge confirmed unexpected offset {confirmed}"
        );
        offset = confirmed;
        if !json && offset >= next_report {
            eprintln!("{offset}/{length} bytes");
            next_report = offset + 65_536;
        }
    }

    let expected = booted_slot ^ 1;
    match authenticated(client, &key, nonce, OtaAction::Finish) {
        Ok(OtaResponse::Rebooting { slot }) => ensure!(slot == expected, "unexpected boot slot"),
        // The last reply can be lost as the bridge resets. Confirm the actual
        // booted slot after rediscovery before reporting success.
        Err(_) | Ok(OtaResponse::Error(OtaError::Unauthorized)) => {}
        Ok(reply) => bail!("bridge rejected activation: {reply:?}"),
    }
    let deadline = Instant::now() + Duration::from_secs(75);
    loop {
        let (challenge, slot, _) =
            info(client, deadline.saturating_duration_since(Instant::now()))?;
        if challenge != nonce {
            ensure!(
                slot == expected,
                "bridge booted ota_{slot}, expected ota_{expected}"
            );
            break;
        }
        ensure!(Instant::now() < deadline, "bridge reboot was not observed");
        thread::sleep(Duration::from_millis(500));
    }
    emit(
        json,
        json!({ "booted_slot": expected, "bytes": length }),
        format!("Bridge booted ota_{expected} after uploading {length} bytes."),
    );
    Ok(())
}
