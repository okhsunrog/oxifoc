//! Recent raw blocks, indexed by capture cookie and block number.

use core::{cell::RefCell, pin::pin};

use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use ergot::{FrameKind, socket::HeaderMessage, wire_frames::de_frame};
use oxifoc_core::{
    icd::{RawTelemetryBlock, TelemetryReplayEndpoint, UDP_MTU, raw_block_limit},
    raw_telemetry::RawBlockHeader,
    types::wifi_telemetry::TelemetryReplayRequest,
};

use crate::transport::Stack;

const BLOCKS: usize = 32;
const BYTES: usize = raw_block_limit(UDP_MTU) as usize;

#[derive(Clone, Copy)]
struct Entry {
    net: u16,
    session: u64,
    block: u32,
    len: u16,
    data: [u8; BYTES],
}

impl Entry {
    const EMPTY: Self = Self {
        net: 0,
        session: 0,
        block: 0,
        len: 0,
        data: [0; BYTES],
    };
}

static HISTORY: Mutex<CriticalSectionRawMutex, RefCell<[Entry; BLOCKS]>> =
    Mutex::new(RefCell::new([Entry::EMPTY; BLOCKS]));

pub fn remember(net: u16, packet: &[u8]) {
    let Some(frame) = de_frame(packet).filter(|frame| frame.hdr.kind == FrameKind::TOPIC_MSG)
    else {
        return;
    };
    let Some(data) = frame
        .body
        .ok()
        .and_then(|body| postcard::from_bytes::<&[u8]>(body).ok())
        .filter(|data| data.len() <= BYTES)
    else {
        return;
    };
    let Ok(header) = RawBlockHeader::parse(data) else {
        return;
    };
    HISTORY.lock(|entries| {
        let entry = &mut entries.borrow_mut()[header.block_index as usize % BLOCKS];
        entry.net = net;
        entry.session = header.session;
        entry.block = header.block_index;
        entry.len = data.len() as u16;
        entry.data[..data.len()].copy_from_slice(data);
    });
}

#[embassy_executor::task]
pub async fn serve(router: &'static Stack) {
    let server = router
        .endpoints()
        .bounded_server::<TelemetryReplayEndpoint, 2>(None);
    let server = pin!(server);
    let mut handle = server.attach();
    loop {
        let _ = handle
            .serve_full(async |msg: &HeaderMessage<TelemetryReplayRequest>| {
                HISTORY.lock(|entries| {
                    let entries = entries.borrow();
                    let entry = &entries[msg.t.block as usize % BLOCKS];
                    if entry.len == 0
                        || entry.net != msg.hdr.src.network_id
                        || entry.session != msg.t.session
                        || entry.block != msg.t.block
                    {
                        return None;
                    }
                    let data =
                        heapless::Vec::from_slice(&entry.data[..usize::from(entry.len)]).ok()?;
                    Some(RawTelemetryBlock { data })
                })
            })
            .await;
    }
}
