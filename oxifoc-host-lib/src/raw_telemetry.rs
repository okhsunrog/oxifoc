//! Connection-scoped raw ingress and capture integrity, independent of motor commands.

use crate::config::FastSource;
use anyhow::{Result, anyhow, bail};
use crossbeam_channel::Sender;
use oxifoc_core::{
    raw_telemetry::{
        DecodedRawBlock, RAW_HEADER_BYTES, RAW_MAGIC, RawBlockHeader, RawEncoding,
        RawStreamDecoder, RawTelemetryStatus,
    },
    types::FastTelemetry,
    velocity_telemetry::{SESSION_START, VelocityDebugFrame},
};
use serde::Serialize;
use std::{
    sync::{Arc, Condvar, Mutex},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

const STALL_LIMIT: Duration = Duration::from_secs(2);

fn producer_off(status: RawTelemetryStatus) -> bool {
    matches!(
        status,
        RawTelemetryStatus::Stopped | RawTelemetryStatus::Unavailable
    )
}

fn validate_stop_cookie(owned: Option<u64>, actual: u64) -> Result<()> {
    if let Some(owned) = owned
        && owned != actual
    {
        bail!("raw stop cookie mismatch: owned {owned}, device {actual}");
    }
    Ok(())
}

#[derive(Clone, Debug, Serialize)]
pub struct RawCaptureReport {
    pub source: FastSource,
    pub encoding: RawEncoding,
    pub session: u64,
    pub records_received: u64,
    pub records_delivered: u64,
    pub host_drops: u64,
    pub producer_drops: u32,
    pub blocks_received: u64,
    pub raw34_blocks: u64,
    pub packed26_blocks: u64,
    pub packed28_blocks: u64,
    pub packed26_delta_blocks: u64,
    pub fallback_blocks: u64,
    pub end_total_records: Option<u64>,
    pub ended: bool,
    pub error: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub velocity_debug: Option<VelocityDebugReport>,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct VelocityDebugReport {
    pub records: u64,
    pub blocks: u64,
    pub producer_drops: u32,
    pub end_total_records: Option<u64>,
}

impl RawCaptureReport {
    /// Delivery to the host FIFO and export to a file are separate boundaries.
    pub fn validate_export(&self, session: u64, rows: usize) -> Result<()> {
        if self.session != session {
            bail!("raw capture session changed before export");
        }
        if let Some(error) = &self.error {
            bail!("raw capture failed: {error}");
        }
        if !self.ended || self.end_total_records != Some(self.records_received) {
            bail!("raw capture has no complete END/totals");
        }
        if self.host_drops != 0
            || self.producer_drops != 0
            || self.records_delivered != self.records_received
            || rows as u64 != self.records_received
        {
            bail!(
                "raw export incomplete: {rows} rows, {} received, {} delivered, producer drops {}, host drops {}",
                self.records_received,
                self.records_delivered,
                self.producer_drops,
                self.host_drops
            );
        }
        Ok(())
    }
}

struct CaptureState {
    report: RawCaptureReport,
    next_block: u32,
    last_progress: Instant,
    velocity_frames: Vec<VelocityDebugFrame>,
}

#[derive(Default)]
struct Inner {
    ready: bool,
    transport_error: Option<String>,
    capture: Option<CaptureState>,
}

pub struct RawTelemetryState {
    source: FastSource,
    encoding: RawEncoding,
    tx: Sender<FastTelemetry>,
    inner: Mutex<Inner>,
    changed: Condvar,
    velocity_debug: bool,
}

impl RawTelemetryState {
    #[cfg(test)]
    pub(crate) fn new(
        source: FastSource,
        encoding: RawEncoding,
        tx: Sender<FastTelemetry>,
    ) -> Arc<Self> {
        Self::new_with_velocity_debug(source, encoding, tx, false)
    }

    pub(crate) fn new_with_velocity_debug(
        source: FastSource,
        encoding: RawEncoding,
        tx: Sender<FastTelemetry>,
        velocity_debug: bool,
    ) -> Arc<Self> {
        Arc::new(Self {
            source,
            encoding,
            tx,
            inner: Mutex::new(Inner::default()),
            changed: Condvar::new(),
            velocity_debug,
        })
    }

    pub fn velocity_frames(&self, session: u64) -> Result<Option<Vec<VelocityDebugFrame>>> {
        let inner = self.inner.lock().unwrap();
        let capture = inner
            .capture
            .as_ref()
            .filter(|c| c.report.session == session)
            .ok_or_else(|| anyhow!("velocity capture session changed before export"))?;
        Ok(capture
            .report
            .velocity_debug
            .as_ref()
            .map(|_| capture.velocity_frames.clone()))
    }

    pub fn source(&self) -> FastSource {
        self.source
    }
    pub fn encoding(&self) -> RawEncoding {
        self.encoding
    }

    pub(crate) fn reader_ready(&self) {
        let mut inner = self.inner.lock().unwrap();
        inner.ready = true;
        inner.transport_error = None;
        self.changed.notify_all();
    }

    pub(crate) fn control_ready(&self) {
        if self.source == FastSource::ErgotRaw {
            self.reader_ready();
        }
    }

    pub(crate) fn connection_lost(&self, error: impl ToString) {
        self.fail(error);
        self.inner.lock().unwrap().ready = false;
    }

    pub(crate) fn control_lost(&self) {
        let mut inner = self.inner.lock().unwrap();
        if let Some(capture) = &mut inner.capture
            && !capture.report.ended
        {
            capture
                .report
                .error
                .get_or_insert_with(|| "USB control connection lost during capture".into());
        }
        self.changed.notify_all();
    }

    pub(crate) fn fail(&self, error: impl ToString) {
        let error = error.to_string();
        let mut inner = self.inner.lock().unwrap();
        inner.transport_error = Some(error.clone());
        if let Some(capture) = &mut inner.capture
            && !capture.report.ended
            && capture.report.error.is_none()
        {
            capture.report.error = Some(error);
        }
        self.changed.notify_all();
    }

    pub(crate) async fn wait_ready(&self) -> Result<()> {
        let deadline = tokio::time::Instant::now() + STALL_LIMIT;
        loop {
            {
                let inner = self.inner.lock().unwrap();
                if let Some(error) = &inner.transport_error {
                    bail!("raw reader unavailable: {error}");
                }
                if inner.ready {
                    return Ok(());
                }
            }
            if tokio::time::Instant::now() >= deadline {
                bail!("raw reader did not become ready");
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }

    pub(crate) fn arm(&self) -> Result<u64> {
        use std::sync::atomic::{AtomicU64, Ordering};
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        let session = NEXT
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |last| {
                Some(last.wrapping_add(1).max(now).max(1))
            })
            .unwrap()
            .wrapping_add(1)
            .max(now)
            .max(1);
        let mut inner = self.inner.lock().unwrap();
        if !inner.ready {
            bail!("raw reader is not ready");
        }
        if let Some(error) = &inner.transport_error {
            bail!("raw reader failed: {error}");
        }
        if inner
            .capture
            .as_ref()
            .is_some_and(|c| !c.report.ended && c.report.error.is_none())
        {
            bail!("raw capture already active; stop it before arming another session");
        }
        inner.capture = Some(CaptureState {
            report: RawCaptureReport {
                source: self.source,
                encoding: self.encoding,
                session,
                records_received: 0,
                records_delivered: 0,
                host_drops: 0,
                producer_drops: 0,
                blocks_received: 0,
                raw34_blocks: 0,
                packed26_blocks: 0,
                packed28_blocks: 0,
                packed26_delta_blocks: 0,
                fallback_blocks: 0,
                end_total_records: None,
                ended: false,
                error: None,
                velocity_debug: self.velocity_debug.then(VelocityDebugReport::default),
            },
            next_block: 0,
            last_progress: Instant::now(),
            velocity_frames: Vec::new(),
        });
        Ok(session)
    }

    pub(crate) fn active_session(&self) -> Option<u64> {
        self.inner
            .lock()
            .unwrap()
            .capture
            .as_ref()
            .map(|c| c.report.session)
    }

    fn fail_capture(&self, session: u64, error: impl ToString) {
        let mut inner = self.inner.lock().unwrap();
        if let Some(capture) = &mut inner.capture
            && capture.report.session == session
            && !capture.report.ended
        {
            capture
                .report
                .error
                .get_or_insert_with(|| error.to_string());
        }
        self.changed.notify_all();
    }

    pub fn report(&self) -> Option<RawCaptureReport> {
        self.inner
            .lock()
            .unwrap()
            .capture
            .as_ref()
            .map(|c| c.report.clone())
    }

    pub fn check(&self) -> Result<()> {
        let mut inner = self.inner.lock().unwrap();
        if let Some(capture) = &mut inner.capture {
            if !capture.report.ended
                && capture.last_progress.elapsed() > STALL_LIMIT
                && capture.report.error.is_none()
            {
                capture.report.error = Some("raw telemetry stalled without END".into());
            }
            if let Some(error) = &capture.report.error {
                bail!("raw capture failed: {error}");
            }
        } else if let Some(error) = &inner.transport_error {
            bail!("raw reader failed: {error}");
        }
        Ok(())
    }

    pub fn wait_end(&self, timeout: Duration) -> Result<RawCaptureReport> {
        let session = self
            .active_session()
            .ok_or_else(|| anyhow!("no raw capture session"))?;
        self.wait_end_for(session, timeout)
    }

    pub fn wait_end_for(&self, session: u64, timeout: Duration) -> Result<RawCaptureReport> {
        let deadline = Instant::now() + timeout;
        let mut inner = self.inner.lock().unwrap();
        loop {
            let capture = inner
                .capture
                .as_ref()
                .ok_or_else(|| anyhow!("no raw capture session"))?;
            if capture.report.session != session {
                bail!("raw capture session changed while waiting for END");
            }
            if let Some(error) = &capture.report.error {
                bail!("raw capture failed: {error}");
            }
            if capture.report.ended {
                return Ok(capture.report.clone());
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                bail!("raw capture did not receive its END marker");
            }
            inner = self.changed.wait_timeout(inner, left).unwrap().0;
        }
    }

    fn accept(&self, block: DecodedRawBlock) -> Result<bool> {
        let mut inner = self.inner.lock().unwrap();
        let Some(capture) = &mut inner.capture else {
            return Ok(false);
        };
        let header = block.header;
        if header.session != capture.report.session {
            return Ok(false);
        }
        let result = (|| -> Result<()> {
            if let Some(error) = &capture.report.error {
                bail!("capture already failed: {error}");
            }
            if capture.report.ended {
                bail!("raw data arrived after END");
            }
            if header.block_index != capture.next_block {
                bail!(
                    "raw block index {}, expected {}",
                    header.block_index,
                    capture.next_block
                );
            }
            if header.encoding != capture.report.encoding
                && !(matches!(
                    capture.report.encoding,
                    RawEncoding::Packed26 | RawEncoding::Packed26Delta
                ) && header.encoding == RawEncoding::Raw34
                    && header.is_fallback())
            {
                bail!("unexpected raw encoding");
            }
            capture.next_block = capture.next_block.wrapping_add(1);
            capture.report.blocks_received += 1;
            capture.last_progress = Instant::now();
            if header.is_velocity() {
                let report = capture
                    .report
                    .velocity_debug
                    .as_mut()
                    .ok_or_else(|| anyhow!("unsolicited velocity debug block"))?;
                if report.end_total_records.is_some() {
                    bail!("velocity debug data arrived after its END");
                }
                report.blocks += 1;
                report.producer_drops = report.producer_drops.max(header.producer_drops);
                if header.is_end() {
                    report.end_total_records = Some(header.total_records);
                    if report.records == 0
                        || header.total_records != report.records
                        || report.producer_drops != 0
                    {
                        bail!(
                            "incomplete velocity debug stream: END {}, records {}, drops {}",
                            header.total_records,
                            report.records,
                            report.producer_drops
                        );
                    }
                } else {
                    // Diagnostics remain bounded even for a caller that never requests Stop.
                    if capture.velocity_frames.len() + block.velocity.len() > 1_048_576 {
                        bail!("velocity debug host capture limit exceeded");
                    }
                    for frame in block.velocity {
                        if let Some(previous) = capture.velocity_frames.last() {
                            let delta = frame.sequence.wrapping_sub(previous.sequence);
                            if frame.flags & SESSION_START != 0 || delta == 0 || delta >= 1 << 31 {
                                bail!("invalid velocity debug sequence");
                            }
                        } else if frame.flags & SESSION_START == 0 {
                            bail!("velocity debug stream has no session origin");
                        }
                        capture.velocity_frames.push(frame);
                        report.records += 1;
                    }
                }
                return Ok(());
            }
            capture.report.producer_drops =
                capture.report.producer_drops.max(header.producer_drops);
            if header.is_end() {
                if capture
                    .report
                    .velocity_debug
                    .as_ref()
                    .is_some_and(|d| d.end_total_records.is_none())
                {
                    bail!("fast END arrived before velocity debug END");
                }
                capture.report.end_total_records = Some(header.total_records);
                if header.total_records != capture.report.records_received {
                    bail!(
                        "END total {} differs from {} decoded records",
                        header.total_records,
                        capture.report.records_received
                    );
                }
                if capture.report.producer_drops != 0 || capture.report.host_drops != 0 {
                    bail!(
                        "raw capture lost samples: producer={}, host={}",
                        header.producer_drops,
                        capture.report.host_drops
                    );
                }
                capture.report.ended = true;
            } else {
                match header.encoding {
                    RawEncoding::Raw34 => capture.report.raw34_blocks += 1,
                    RawEncoding::Packed26 => capture.report.packed26_blocks += 1,
                    RawEncoding::Packed28 => capture.report.packed28_blocks += 1,
                    RawEncoding::Packed26Delta => capture.report.packed26_delta_blocks += 1,
                }
                capture.report.fallback_blocks += u64::from(header.is_fallback());
                capture.report.records_received += block.samples.len() as u64;
                for sample in block.samples {
                    if self.tx.try_send(sample).is_ok() {
                        capture.report.records_delivered += 1;
                    } else {
                        capture.report.host_drops += 1;
                    }
                }
                if capture.report.host_drops != 0 {
                    bail!("host raw telemetry channel overflow");
                }
            }
            Ok(())
        })();
        if let Err(error) = &result {
            capture
                .report
                .error
                .get_or_insert_with(|| error.to_string());
        }
        self.changed.notify_all();
        result.map(|()| true)
    }
}

/// Configure through Ergot control; raw readers and END waits never own this
/// command task. A status query makes stale-session cleanup explicit.
pub(crate) async fn configure<NS>(
    stack: &NS,
    state: &Arc<RawTelemetryState>,
    fast_hz: u16,
    address: ergot::Address,
) -> Result<oxifoc_core::types::TelemetryConfigAck>
where
    NS: ergot::net_stack::NetStackHandle + Clone + Send + Sync + 'static,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send + Sync,
{
    configure_inner(stack, state, fast_hz, address, false).await
}

pub(crate) async fn prepare_connection<NS>(
    stack: &NS,
    state: &Arc<RawTelemetryState>,
    address: ergot::Address,
) -> Result<()>
where
    NS: ergot::net_stack::NetStackHandle + Clone + Send + Sync + 'static,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send + Sync,
{
    configure_inner(stack, state, 0, address, true).await?;
    wait_stopped(stack, address).await
}

async fn configure_inner<NS>(
    stack: &NS,
    state: &Arc<RawTelemetryState>,
    fast_hz: u16,
    address: ergot::Address,
    startup_cleanup: bool,
) -> Result<oxifoc_core::types::TelemetryConfigAck>
where
    NS: ergot::net_stack::NetStackHandle + Clone + Send + Sync + 'static,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send + Sync,
{
    use oxifoc_core::{
        delivery::ReliableExt,
        icd::{RawTelemetryEndpoint, VelocityDebugArmEndpoint},
        raw_telemetry::{RawTelemetryRequest, RawTelemetryStatus, VelocityDebugArm},
    };
    let client = stack.clone().reliable::<crate::TokioTimer>();
    if fast_hz != 0
        && state
            .report()
            .is_some_and(|capture| !capture.ended && capture.error.is_none())
    {
        bail!("raw capture already active; stop it before arming another session");
    }
    let status = client
        .at_least_once::<RawTelemetryEndpoint>(
            address,
            &RawTelemetryRequest::Status,
            Some("raw_telemetry"),
            &crate::SETPOINT_POLICY,
        )
        .await
        .map_err(|error| anyhow!("raw telemetry unsupported/status failed: {error:?}"))?;
    let previous_session = status.session;
    if fast_hz == 0 && !startup_cleanup {
        validate_stop_cookie(
            state.report().map(|capture| capture.session),
            status.session,
        )?;
    }
    if fast_hz != 0 && !producer_off(status.status) {
        // Startup performs stale-session cleanup before publishing connected.
        // An ordinary command never waits for raw I/O or silently replaces a
        // running capture, so Motor Stop remains independent of reader drain.
        bail!("raw previous session is not stopped; stop it before arming: {status:?}");
    }
    let session = if fast_hz != 0 {
        state.wait_ready().await?;
        state.arm()?
    } else {
        if producer_off(status.status) && status.timing.actual_fast_hz == 0 {
            return Ok(status.timing);
        }
        if status.session == 0 {
            bail!("raw status did not identify its active session");
        }
        status.session
    };
    let request = if fast_hz == 0 {
        RawTelemetryRequest::Stop { session }
    } else {
        RawTelemetryRequest::Arm {
            session,
            previous_session,
            encoding: state.encoding(),
            fast_hz,
        }
    };
    let result = if fast_hz != 0 && state.velocity_debug {
        client
            .at_least_once::<VelocityDebugArmEndpoint>(
                address,
                &VelocityDebugArm {
                    session,
                    previous_session,
                    fast_hz,
                },
                Some("velocity_debug"),
                &crate::SETPOINT_POLICY,
            )
            .await
    } else {
        client
            .at_least_once::<RawTelemetryEndpoint>(
                address,
                &request,
                Some("raw_telemetry"),
                &crate::SETPOINT_POLICY,
            )
            .await
    }
    .map_err(|error| anyhow!("raw telemetry control failed: {error:?}"));
    let result = result.and_then(|ack| {
        if ack.session != session {
            bail!("raw acknowledgement cookie mismatch");
        }
        if fast_hz == 0 {
            if !(ack.status == RawTelemetryStatus::Stopping || producer_off(ack.status))
                || ack.timing.actual_fast_hz != 0
            {
                bail!("raw stop rejected: {ack:?}");
            }
        } else if ack.status != RawTelemetryStatus::Armed
            || ack.encoding != state.encoding()
            || ack.timing.actual_fast_hz == 0
        {
            bail!("raw arm rejected: {ack:?}");
        }
        Ok(ack.timing)
    });
    if let Err(error) = &result
        && fast_hz != 0
    {
        state.fail_capture(session, error);
    }
    result
}

/// Startup-only drain fence; callers have not published control readiness yet.
/// Ordinary telemetry Stop returns its producer-disable ACK without this wait.
pub(crate) async fn wait_stopped<NS>(stack: &NS, address: ergot::Address) -> Result<()>
where
    NS: ergot::net_stack::NetStackHandle + Clone + Send + Sync + 'static,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send + Sync,
{
    use oxifoc_core::{
        delivery::ReliableExt,
        icd::RawTelemetryEndpoint,
        raw_telemetry::{RawTelemetryRequest, RawTelemetryStatus},
    };
    let client = stack.clone().reliable::<crate::TokioTimer>();
    tokio::time::timeout(STALL_LIMIT, async {
        let mut previous = None;
        loop {
            let status = client
                .at_least_once::<RawTelemetryEndpoint>(
                    address,
                    &RawTelemetryRequest::Status,
                    Some("raw_telemetry"),
                    &crate::SETPOINT_POLICY,
                )
                .await
                .map_err(|error| anyhow!("raw cleanup status failed: {error:?}"))?;
            if previous.is_some_and(|session| session != status.session) {
                bail!("raw session changed during startup cleanup");
            }
            previous = Some(status.session);
            if producer_off(status.status) && status.timing.actual_fast_hz == 0 {
                return Ok(());
            }
            if status.status != RawTelemetryStatus::Stopping {
                bail!("raw startup cleanup failed: {status:?}");
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .map_err(|_| anyhow!("raw previous session did not drain"))?
}

/// Seek a matching block0 only before a session begins; every subsequent byte
/// is decoded strictly, including the suffix of the completion containing it.
pub(crate) struct RawIngress {
    state: Arc<RawTelemetryState>,
    decoder: RawStreamDecoder,
    session: Option<u64>,
    synchronized: bool,
    prefix: [u8; RAW_HEADER_BYTES],
    prefix_len: usize,
}

/// The Ergot carrier preserves block bytes and feeds the same strict session
/// parser. A missing topic frame is an index/END failure, never a silent skip.
pub(crate) fn spawn_ergot_subscriber<NS>(
    stack: &NS,
    state: Arc<RawTelemetryState>,
    cancel: tokio_util::sync::CancellationToken,
) where
    NS: ergot::net_stack::NetStackHandle + Clone + Send + Sync + 'static,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send,
{
    let stack = stack.clone();
    tokio::spawn(async move {
        use oxifoc_core::icd::RawTelemetryBlockTopic;
        let ns = stack.stack();
        // 32 blocks bound the additional decoded-message queue. USB itself
        // has the same 32×4096-byte receive requests as the direct raw path.
        let receiver = ns
            .topics()
            .heap_bounded_receiver::<RawTelemetryBlockTopic>(32, Some("raw_fast"));
        let mut receiver = core::pin::pin!(receiver);
        let mut handle = receiver.as_mut().subscribe_unicast();
        let mut ingress = RawIngress::new(state.clone());
        state.reader_ready();
        let mut ticker = tokio::time::interval(Duration::from_millis(100));
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            // Capture failures belong to their cookie. Keep listening for
            // the next capture on this stack.
            tokio::select! {
                _ = cancel.cancelled() => break,
                _ = ticker.tick() => { let _ = state.check(); }
                block = handle.recv() => { let _ = ingress.push(&block.t.data); }
            }
        }
    });
}

impl RawIngress {
    pub(crate) fn new(state: Arc<RawTelemetryState>) -> Self {
        Self {
            state,
            decoder: RawStreamDecoder::new(),
            session: None,
            synchronized: false,
            prefix: [0; RAW_HEADER_BYTES],
            prefix_len: 0,
        }
    }

    fn decode(&mut self, data: &[u8]) -> Result<bool> {
        let mut matched = false;
        let mut failure = None;
        self.decoder
            .push(data, |block| match self.state.accept(block) {
                Ok(current) => matched |= current,
                Err(error) => failure = Some(error),
            })
            .map_err(|error| anyhow!("malformed raw stream: {error:?}"))?;
        if let Some(error) = failure {
            return Err(error);
        }
        Ok(matched)
    }

    pub(crate) fn push(&mut self, data: &[u8]) -> Result<()> {
        let result = self.push_inner(data);
        if let Err(error) = &result
            && let Some(session) = self.session
        {
            self.state.fail_capture(session, error);
        }
        result
    }

    fn push_inner(&mut self, data: &[u8]) -> Result<()> {
        let session = self
            .state
            .inner
            .lock()
            .unwrap()
            .capture
            .as_ref()
            .filter(|capture| capture.report.error.is_none())
            .map(|capture| capture.report.session);
        if session != self.session {
            self.session = session;
            self.decoder.reset();
            self.synchronized = false;
            self.prefix_len = 0;
        }
        if session.is_none() {
            return Ok(());
        }
        let mut offset = 0;
        while !self.synchronized && offset < data.len() {
            let byte = data[offset];
            offset += 1;
            if self.prefix_len < self.prefix.len() {
                self.prefix[self.prefix_len] = byte;
                self.prefix_len += 1;
            } else {
                self.prefix.rotate_left(1);
                self.prefix[RAW_HEADER_BYTES - 1] = byte;
            }
            if self.prefix_len == RAW_HEADER_BYTES
                && self.prefix[..4] == RAW_MAGIC
                && Some(u64::from_le_bytes(self.prefix[8..16].try_into().unwrap())) == session
            {
                let header = RawBlockHeader::parse(&self.prefix)
                    .map_err(|error| anyhow!("malformed current raw header: {error:?}"))?;
                if header.block_index != 0 {
                    bail!("raw first block index {}, expected 0", header.block_index);
                }
                let prefix = self.prefix;
                self.decode(&prefix)?;
                self.synchronized = true;
            }
        }
        if offset < data.len() {
            self.decode(&data[offset..])?;
        }
        self.state.check()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crossbeam_channel::{Receiver, bounded};
    use oxifoc_core::raw_telemetry::RawBlockEncoder;

    #[test]
    fn velocity_sidecar_requires_its_own_totals_and_session_origin() {
        use oxifoc_core::velocity_telemetry::VELOCITY_RECORD_BYTES;
        for (debug_end, drops, expected_ok) in
            [(true, 0, true), (false, 0, false), (true, 1, false)]
        {
            let (tx, rx) = bounded(8);
            let state = RawTelemetryState::new_with_velocity_debug(
                FastSource::ErgotRaw,
                RawEncoding::Packed28,
                tx,
                true,
            );
            state.reader_ready();
            let session = state.arm().unwrap();
            let first = FastTelemetry {
                seq: u16::MAX,
                ..Default::default()
            };
            let mut bytes = data(session, 0, RawEncoding::Packed28, &[first], 0);
            let mut debug = [0; RAW_HEADER_BYTES + 2 * VELOCITY_RECORD_BYTES];
            RawBlockHeader::velocity(session, 1, 2, 0, None).write(&mut debug);
            let origin = VelocityDebugFrame {
                sequence: u32::MAX,
                flags: SESSION_START,
                ..Default::default()
            };
            let update = VelocityDebugFrame {
                sequence: 19,
                requested_iq: -30,
                ..Default::default()
            };
            debug[RAW_HEADER_BYTES..RAW_HEADER_BYTES + VELOCITY_RECORD_BYTES]
                .copy_from_slice(&origin.encode());
            debug[RAW_HEADER_BYTES + VELOCITY_RECORD_BYTES..].copy_from_slice(&update.encode());
            bytes.extend(debug);
            let mut next = 2;
            if debug_end {
                let mut marker = [0; RAW_HEADER_BYTES];
                RawBlockHeader::velocity(session, next, 0, drops, Some(2)).write(&mut marker);
                bytes.extend(marker);
                next += 1;
            }
            bytes.extend(end(session, next, RawEncoding::Packed28, 1, 0));
            let mut ingress = RawIngress::new(state.clone());
            let result: Result<()> = bytes.chunks(13).try_for_each(|chunk| ingress.push(chunk));
            assert_eq!(result.is_ok(), expected_ok);
            assert_eq!(rx.try_recv().unwrap(), first);
            assert_eq!(
                state.velocity_frames(session).unwrap().unwrap(),
                [origin, update]
            );
            assert_eq!(state.wait_end(Duration::ZERO).is_ok(), expected_ok);
        }
    }

    #[tokio::test]
    async fn ergot_carrier_preserves_blocks_end_and_missing_block_failure() {
        use oxifoc_core::icd::{RawTelemetryBlock, RawTelemetryBlockTopic};
        for encoding in [
            RawEncoding::Raw34,
            RawEncoding::Packed26,
            RawEncoding::Packed26Delta,
            RawEncoding::Packed28,
        ] {
            for missing in [false, true] {
                let stack = ergot::toolkits::null::new_arc_null_stack();
                let (tx, rx) = bounded(64);
                let state = RawTelemetryState::new(FastSource::ErgotRaw, encoding, tx);
                let cancel = tokio_util::sync::CancellationToken::new();
                spawn_ergot_subscriber(&stack, state.clone(), cancel.clone());
                state.wait_ready().await.unwrap();
                let session = state.arm().unwrap();
                let samples: Vec<_> = (0..64)
                    .map(|seq| FastTelemetry {
                        seq,
                        ia: 2048 + seq,
                        ib: 2048 - seq,
                        ic: 2048,
                        vphase_adc: [1800, 1900, 2000],
                        pwm_ccr: [1000, 2000, 3000],
                        pwm_arr: 4195,
                        ..FastTelemetry::default()
                    })
                    .collect();
                let publish = |bytes: Vec<u8>| {
                    let mut block = RawTelemetryBlock {
                        data: Default::default(),
                    };
                    block.data.extend_from_slice(&bytes).unwrap();
                    stack
                        .topics()
                        .unicast::<RawTelemetryBlockTopic>(
                            ergot::Address {
                                network_id: 0,
                                node_id: 0,
                                port_id: 0,
                            },
                            &block,
                        )
                        .unwrap();
                };
                for bytes in [
                    data(session.wrapping_sub(1), 0, encoding, &samples, 0),
                    data(session, 0, encoding, &samples, 0),
                    end(session, if missing { 2 } else { 1 }, encoding, 64, 0),
                ] {
                    publish(bytes);
                }
                tokio::time::timeout(Duration::from_secs(1), async {
                    loop {
                        let report = state.report().unwrap();
                        if report.ended || report.error.is_some() {
                            break;
                        }
                        tokio::task::yield_now().await;
                    }
                })
                .await
                .unwrap();
                let report = state.report().unwrap();
                assert_eq!(rx.try_iter().collect::<Vec<_>>(), samples);
                if missing {
                    assert!(report.error.as_deref().unwrap().contains("block index 2"));
                    assert!(!report.ended);
                    let next = state.arm().unwrap();
                    publish(end(session, 1, encoding, 64, 0));
                    publish(data(next, 0, encoding, &samples, 0));
                    publish(end(next, 1, encoding, 64, 0));
                    let recovered = tokio::task::spawn_blocking({
                        let state = state.clone();
                        move || state.wait_end_for(next, Duration::from_secs(1))
                    })
                    .await
                    .unwrap()
                    .unwrap();
                    recovered.validate_export(next, 64).unwrap();
                    assert_eq!(rx.try_iter().collect::<Vec<_>>(), samples);
                } else {
                    report.validate_export(session, 64).unwrap();
                    assert_eq!(report.source, FastSource::ErgotRaw);
                }
                cancel.cancel();
            }
        }
    }

    fn ready(
        capacity: usize,
        encoding: RawEncoding,
    ) -> (Arc<RawTelemetryState>, Receiver<FastTelemetry>, u64) {
        let (tx, rx) = bounded(capacity);
        let state = RawTelemetryState::new(FastSource::UsbRaw, encoding, tx);
        state.reader_ready();
        let session = state.arm().unwrap();
        (state, rx, session)
    }

    fn data(
        session: u64,
        index: u32,
        encoding: RawEncoding,
        samples: &[FastTelemetry],
        drops: u32,
    ) -> Vec<u8> {
        let mut encoder = RawBlockEncoder::new();
        encoder.begin(session, index, encoding, 1);
        for sample in samples {
            assert!(encoder.push(sample));
        }
        encoder.finish(drops).to_vec()
    }

    fn end(session: u64, index: u32, encoding: RawEncoding, total: u64, drops: u32) -> Vec<u8> {
        RawBlockEncoder::new()
            .end(session, index, encoding, total, drops)
            .to_vec()
    }

    #[test]
    fn arbitrary_chunks_empty_completions_and_old_tokens_reach_the_same_fast_channel() {
        let samples = [
            FastTelemetry {
                seq: 65535,
                ..FastTelemetry::default()
            },
            FastTelemetry {
                seq: 0,
                ..FastTelemetry::default()
            },
        ];
        for chunk in 1..100 {
            let (state, rx, session) = ready(8, RawEncoding::Packed26);
            let mut stream = b"old truncated endpoint prefix OX".to_vec();
            stream.extend(data(
                session.wrapping_sub(1),
                0,
                RawEncoding::Raw34,
                &samples,
                0,
            ));
            stream.extend(data(session, 0, RawEncoding::Packed26, &samples, 0));
            stream.extend(end(session, 1, RawEncoding::Packed26, 2, 0));
            let mut ingress = RawIngress::new(state.clone());
            for bytes in stream.chunks(chunk) {
                ingress.push(&[]).unwrap();
                ingress.push(bytes).unwrap();
            }
            let report = state.wait_end(Duration::ZERO).unwrap();
            assert_eq!(report.records_received, 2);
            assert_eq!(report.records_delivered, 2);
            assert_eq!(report.blocks_received, 2);
            assert_eq!(report.packed26_blocks, 1);
            assert_eq!(rx.try_iter().collect::<Vec<_>>(), samples);
        }
    }

    #[test]
    fn matching_first_block_requires_index_zero_and_strict_suffix_in_the_same_completion() {
        for wrong_index in [false, true] {
            let (state, _rx, session) = ready(8, RawEncoding::Raw34);
            let mut stream = data(
                session,
                u32::from(wrong_index),
                RawEncoding::Raw34,
                &[FastTelemetry::default()],
                0,
            );
            if !wrong_index {
                stream.extend([255; 32]);
            }
            assert!(RawIngress::new(state.clone()).push(&stream).is_err());
            assert!(state.check().is_err());
            assert!(state.wait_end(Duration::ZERO).is_err());
        }
    }

    #[test]
    fn no_end_mismatched_totals_gaps_and_device_drops_cannot_succeed() {
        for (index, total, drops) in [(1, 2, 0), (2, 1, 0), (1, 1, 3)] {
            let (state, _rx, session) = ready(8, RawEncoding::Raw34);
            let mut ingress = RawIngress::new(state.clone());
            ingress
                .push(&data(
                    session,
                    0,
                    RawEncoding::Raw34,
                    &[FastTelemetry::default()],
                    0,
                ))
                .unwrap();
            assert!(state.wait_end(Duration::ZERO).is_err());
            assert!(
                ingress
                    .push(&end(session, index, RawEncoding::Raw34, total, drops))
                    .is_err()
            );
            assert!(!state.report().unwrap().ended);
        }
    }

    #[test]
    fn channel_overflow_is_reported_instead_of_silently_discarded() {
        let (state, _rx, session) = ready(1, RawEncoding::Raw34);
        let samples = [FastTelemetry::default(); 2];
        assert!(
            RawIngress::new(state.clone())
                .push(&data(session, 0, RawEncoding::Raw34, &samples, 0))
                .is_err()
        );
        let report = state.report().unwrap();
        assert_eq!(report.records_received, 2);
        assert_eq!(report.records_delivered, 1);
        assert_eq!(report.host_drops, 1);
        assert!(state.wait_end(Duration::ZERO).is_err());
    }

    #[test]
    fn explicit_packed_fallback_retains_invalid_adc_bits_and_provenance() {
        let (state, rx, session) = ready(8, RawEncoding::Packed26);
        let sample = FastTelemetry {
            ia: u16::MAX,
            ..FastTelemetry::default()
        };
        let mut ingress = RawIngress::new(state.clone());
        ingress
            .push(&data(session, 0, RawEncoding::Packed26, &[sample], 0))
            .unwrap();
        ingress
            .push(&end(session, 1, RawEncoding::Packed26, 1, 0))
            .unwrap();
        assert_eq!(rx.try_recv().unwrap(), sample);
        let report = state.wait_end(Duration::ZERO).unwrap();
        assert_eq!(report.raw34_blocks, 1);
        assert_eq!(report.packed26_blocks, 0);
        assert_eq!(report.fallback_blocks, 1);
    }

    #[test]
    fn stalled_or_disconnected_capture_stays_failed_after_reader_reconnect() {
        for fault in 0..3 {
            let (state, _rx, old) = ready(8, RawEncoding::Raw34);
            match fault {
                0 => {
                    state
                        .inner
                        .lock()
                        .unwrap()
                        .capture
                        .as_mut()
                        .unwrap()
                        .last_progress = Instant::now() - STALL_LIMIT - Duration::from_millis(1);
                }
                1 => state.connection_lost("test disconnect"),
                _ => state.control_lost(),
            }
            assert!(state.check().is_err());
            state.reader_ready();
            assert!(state.wait_end(Duration::ZERO).is_err());
            assert_eq!(state.active_session(), Some(old));
            let new = state.arm().unwrap();
            assert_ne!(old, new);
            let mut ingress = RawIngress::new(state.clone());
            ingress
                .push(&end(old, 0, RawEncoding::Raw34, 0, 0))
                .unwrap();
            assert!(!state.report().unwrap().ended);
            ingress
                .push(&end(new, 0, RawEncoding::Raw34, 0, 0))
                .unwrap();
            assert!(state.wait_end(Duration::ZERO).is_ok());
        }
    }

    #[test]
    fn completion_wakes_waiter_and_late_disconnect_does_not_revoke_consumed_end() {
        let (state, _rx, session) = ready(8, RawEncoding::Raw34);
        let waiting = state.clone();
        let waiter = std::thread::spawn(move || waiting.wait_end(Duration::from_secs(1)));
        RawIngress::new(state.clone())
            .push(&end(session, 0, RawEncoding::Raw34, 0, 0))
            .unwrap();
        assert!(waiter.join().unwrap().unwrap().ended);
        state.connection_lost("unplug after completion");
        assert!(state.wait_end(Duration::ZERO).unwrap().ended);
        assert!(state.arm().is_err());
    }

    #[test]
    fn malformed_prelude_is_resynchronized_but_a_matching_bad_encoding_is_fatal() {
        let (state, _rx, session) = ready(8, RawEncoding::Raw34);
        let mut stream = b"OXTR".to_vec();
        stream.extend([255; 28]);
        stream.extend(data(
            session,
            0,
            RawEncoding::Packed26,
            &[FastTelemetry::default()],
            0,
        ));
        assert!(RawIngress::new(state.clone()).push(&stream).is_err());
        assert!(state.report().unwrap().error.unwrap().contains("encoding"));
    }

    #[test]
    fn stale_blocks_junk_and_truncated_payload_cannot_hide_current_block_zero() {
        for chunk in 1..100 {
            let (state, rx, session) = ready(8, RawEncoding::Raw34);
            let sample = FastTelemetry {
                seq: 123,
                ..FastTelemetry::default()
            };
            let old = data(
                session.wrapping_sub(1),
                0,
                RawEncoding::Raw34,
                &[sample; 8],
                0,
            );
            let mut stream = old.clone();
            stream.push(b'x');
            // Also leave a stale valid header with only half its payload.
            stream.extend(&old[..old.len() / 2]);
            stream.extend(data(session, 0, RawEncoding::Raw34, &[sample], 0));
            stream.extend(end(session, 1, RawEncoding::Raw34, 1, 0));
            let mut ingress = RawIngress::new(state.clone());
            for bytes in stream.chunks(chunk) {
                ingress.push(bytes).unwrap();
            }
            assert_eq!(rx.try_recv().unwrap(), sample);
            assert!(rx.try_recv().is_err());
            assert_eq!(state.wait_end(Duration::ZERO).unwrap().records_received, 1);
        }
    }

    #[test]
    fn unavailable_is_terminal_without_making_a_failed_capture_successful() {
        assert!(producer_off(RawTelemetryStatus::Unavailable));
        assert!(producer_off(RawTelemetryStatus::Stopped));
        for status in [
            RawTelemetryStatus::Armed,
            RawTelemetryStatus::Stopping,
            RawTelemetryStatus::Busy,
            RawTelemetryStatus::Invalid,
        ] {
            assert!(!producer_off(status));
        }
        let (state, _rx, old) = ready(8, RawEncoding::Raw34);
        state.connection_lost("sink aborted without END");
        state.reader_ready();
        assert!(state.wait_end(Duration::ZERO).is_err());
        assert_ne!(state.arm().unwrap(), old);
        assert!(!state.report().unwrap().ended);
    }

    #[test]
    fn complete_end_requires_every_exported_row_and_the_same_owned_session() {
        let (state, _rx, session) = ready(8, RawEncoding::Raw34);
        let mut ingress = RawIngress::new(state.clone());
        ingress
            .push(&data(
                session,
                0,
                RawEncoding::Raw34,
                &[FastTelemetry::default(); 2],
                0,
            ))
            .unwrap();
        ingress
            .push(&end(session, 1, RawEncoding::Raw34, 2, 0))
            .unwrap();
        let report = state.wait_end_for(session, Duration::ZERO).unwrap();
        assert!(report.validate_export(session, 2).is_ok());
        assert!(report.validate_export(session, 1).is_err()); // contiguous prefix or suffix
        assert!(report.validate_export(session, 3).is_err());
        assert!(report.validate_export(session.wrapping_add(1), 2).is_err());
        let new = state.arm().unwrap();
        assert_ne!(new, session);
        assert!(state.wait_end_for(session, Duration::ZERO).is_err());
    }

    #[test]
    fn capture_stop_cannot_stop_another_cookie_but_startup_can_clean_unknown_state() {
        assert!(validate_stop_cookie(Some(7), 7).is_ok());
        assert!(validate_stop_cookie(Some(7), 8).is_err());
        assert!(validate_stop_cookie(Some(7), 0).is_err());
        assert!(validate_stop_cookie(None, 8).is_ok());
    }
}
