//! Paired CPU microbenchmark, not a control-loop qualification. ADC priority 0
//! excludes normal IRQ preemption; no interrupt masking, I/O or fixture loads
//! are inside DWT brackets. Intermittent visits still differ from a warm task
//! draining many records, and code/ART layout can affect these measurements.

use core::{
    hint::black_box,
    sync::atomic::{Ordering, compiler_fence},
};
use ergot::{
    Address, AnyAllAppendix, FrameKind, Header,
    exports::bbqueue::nicknames::Churrasco,
    interface_manager::{InterfaceSink, LinkDst, LinkMeta, utils::framed_stream::Sink},
    traits::Topic,
};
use oxifoc_core::{
    icd::{RawTelemetryBlockRef, RawTelemetryBlockRefTopic},
    raw_telemetry::{RawBlockEncoder, RawEncoding},
    types::FastTelemetry,
};

const RECORDS: usize = 64;
const CASES: usize = 9;
const WARMUPS: u32 = 2;
const REPEATS: u32 = 8;
const EMPTY_BRACKETS: u32 = 64;
const SESSION: u64 = 0x4350_5542_454e_4348;
const FRAME_MTU: u16 = 2304;
type FrameQueue = Churrasco<8192>;
#[cfg(target_arch = "arm")]
const OP_BUDGET_CYCLES: u32 = crate::config::CPU_HZ / 20_000;
#[cfg(not(target_arch = "arm"))]
const OP_BUDGET_CYCLES: u32 = 168_000_000 / 20_000;

fn frame_header() -> Header {
    Header {
        src: Address {
            network_id: 1,
            node_id: 1,
            port_id: 0,
        },
        dst: Address {
            network_id: 1,
            node_id: 2,
            port_id: 0,
        },
        any_all: Some(AnyAllAppendix {
            key: ergot::Key(RawTelemetryBlockRefTopic::TOPIC_KEY.to_bytes()),
            nash: None,
        }),
        kind: FrameKind::TOPIC_MSG,
        class: Default::default(),
        ttl: ergot::DEFAULT_TTL - 1,
    }
}

#[derive(Clone, Copy)]
struct FramingStats([u32; 6]); // sum, min, max, committed bytes, count, op over 50 us
impl FramingStats {
    const fn new() -> Self {
        Self([0, u32::MAX, 0, 0, 0, 0])
    }
    fn record(&mut self, cycles: u32, bytes: usize) {
        let [sum, min, max, total_bytes, count, over] = &mut self.0;
        *sum += cycles;
        *min = (*min).min(cycles);
        *max = (*max).max(cycles);
        *total_bytes += bytes as u32;
        *count += 1;
        *over += u32::from(cycles > OP_BUDGET_CYCLES);
    }
}

// Exact lossless-capture frames, little-endian FastTelemetry words (no struct
// layout dependency). Sources under oxifoc-velocity-stream-2026-11-10:
// ergot-thread-packed26-velocity0.parquet, rows 30000..30064,
// SHA256 51b9b67320baa5732fb2b6ccda1c0035ec5749fe1feadf2ad5ed89f50f51b325;
// ergot-thread-packed26-velocity40-hold5s.parquet, rows 60000..60064,
// SHA256 05a530b8ba2541b37afc3aba5b730b3b6868f84dd9eef9f64516bb854009d46f.
// Combined fixture SHA256:
// 36768506dfa3b0be2c3a8d08afd7d53b5a8cafe43001388e70f1ff458ccf2062.
const FRAMES: &[u8; 2 * RECORDS * 34] = include_bytes!("codec_bench_frames.bin");

fn fixture(case: usize, index: usize) -> FastTelemetry {
    let offset = (case / 3).min(1) * RECORDS * 34 + index * 34;
    let word = |i: usize| u16::from_le_bytes([FRAMES[offset + i * 2], FRAMES[offset + i * 2 + 1]]);
    FastTelemetry {
        // Third fixture deliberately exercises lossless full-width fallback.
        ia: if case / 3 == 2 {
            word(0) | 0x8000
        } else {
            word(0)
        },
        ib: word(1),
        ic: word(2),
        vbus: word(3),
        angle: word(4),
        vd: word(5) as i16,
        vq: word(6) as i16,
        rpm: word(7) as i16,
        seq: word(8),
        vphase_adc: [word(9), word(10), word(11)],
        pwm_ccr: [word(12), word(13), word(14)],
        pwm_arr: word(15),
        pwm_enables: word(16),
    }
}

fn encoding(case: usize) -> RawEncoding {
    match case % 3 {
        0 => RawEncoding::Raw34,
        1 => RawEncoding::Packed26,
        _ => RawEncoding::Packed26Delta,
    }
}

fn bench_allowed(stopped: bool, pwm_enables: u16) -> bool {
    // Stop disables phase CH1..3 CCE/CCNE, but retains MOE and CH4 for ADC.
    // The pre-cycle snapshot intentionally delays admission until the next
    // ADC after a Stop command, rather than guessing the applied gate state.
    stopped && pwm_enables & 0x3f == 0
}

#[inline(always)]
fn measure<T>(clock: &mut impl FnMut() -> u32, work: impl FnOnce() -> T) -> (T, u32) {
    compiler_fence(Ordering::SeqCst);
    let before = clock();
    compiler_fence(Ordering::SeqCst);
    let value = black_box(work());
    compiler_fence(Ordering::SeqCst);
    let after = clock();
    compiler_fence(Ordering::SeqCst);
    (value, after.wrapping_sub(before))
}

#[derive(Clone, Copy)]
struct Stats {
    begin: u32,
    push: u32,
    finish: u32,
    min: u32,
    max: u32,
    first_push: u32,
    following_push: u32,
    bytes: u32,
    checksum: u32,
    push_max: u32,
}
impl Stats {
    const fn new() -> Self {
        Self {
            begin: 0,
            push: 0,
            finish: 0,
            min: u32::MAX,
            max: 0,
            first_push: 0,
            following_push: 0,
            bytes: 0,
            checksum: 0,
            push_max: 0,
        }
    }
    fn words(self) -> [u32; 10] {
        [
            self.begin,
            self.push,
            self.finish,
            self.min,
            self.max,
            self.first_push,
            self.following_push,
            self.bytes,
            self.checksum,
            self.push_max,
        ]
    }
}

#[derive(Clone, Copy)]
enum Stage {
    Begin,
    Push,
    Finish,
    FrameErgot,
    FrameCopy,
    Checksum,
}

/// The interrupt macro gives this static state exclusively to ADC. The task
/// sees only atomically published results after completion, never the encoder.
pub struct Bench {
    encoder: RawBlockEncoder,
    stats: [Stats; CASES],
    frame_queue: FrameQueue,
    framing: [[FramingStats; 2]; CASES],
    case: usize,
    round: u32,
    stage: Stage,
    frame: usize,
    cursor: usize,
    block: Stats,
    empty_n: u32,
    empty_sum: u32,
    empty_min: u32,
    empty_max: u32,
    divider: u8,
    generation: Option<u32>,
}
impl Bench {
    pub const fn new() -> Self {
        Self {
            encoder: RawBlockEncoder::new(),
            stats: [Stats::new(); CASES],
            frame_queue: FrameQueue::new(),
            framing: [[FramingStats::new(); 2]; CASES],
            case: 0,
            round: 0,
            stage: Stage::Begin,
            frame: 0,
            cursor: 0,
            block: Stats::new(),
            empty_n: 0,
            empty_sum: 0,
            empty_min: u32::MAX,
            empty_max: 0,
            divider: 0,
            generation: None,
        }
    }

    // One bounded visit: begin, at most four individually measured pushes,
    // finish, one whole-block framing/copy operation, or 64 checksum bytes.
    // The framing queue is private, polling-only and drained outside DWT:
    // this excludes routing, real queue contention, wakes and USB service.
    fn visit(&mut self, mut clock: impl FnMut() -> u32) -> Result<bool, ()> {
        if self.empty_n < EMPTY_BRACKETS {
            let (_, cycles) = measure(&mut clock, || black_box(()));
            self.empty_n += 1;
            self.empty_sum += cycles;
            self.empty_min = self.empty_min.min(cycles);
            self.empty_max = self.empty_max.max(cycles);
            return Ok(false);
        }
        match self.stage {
            Stage::Begin => {
                self.block = Stats::new();
                self.frame = 0;
                let choice = black_box(encoding(self.case));
                let index = black_box(self.round * CASES as u32 + self.case as u32);
                let (_, cycles) = measure(&mut clock, || {
                    black_box(&mut self.encoder).begin(SESSION, index, choice, 1);
                });
                self.block.begin = cycles;
                self.stage = Stage::Push;
            }
            Stage::Push => {
                for within_visit in 0..4 {
                    // Force the input into RAM before the timed encoder call.
                    let mut scratch = fixture(self.case, self.frame);
                    let input = black_box(&mut scratch);
                    let (accepted, cycles) = measure(&mut clock, || {
                        black_box(&mut self.encoder).push(black_box(input))
                    });
                    if !accepted {
                        return Err(());
                    }
                    self.block.push += cycles;
                    self.block.push_max = self.block.push_max.max(cycles);
                    if within_visit == 0 {
                        self.block.first_push += cycles;
                    } else {
                        self.block.following_push += cycles;
                    }
                    self.frame += 1;
                }
                if self.frame == RECORDS {
                    self.stage = Stage::Finish;
                }
            }
            Stage::Finish => {
                let (bytes, cycles) = measure(&mut clock, || {
                    black_box(black_box(&mut self.encoder).finish(0)).len()
                });
                self.block.finish = cycles;
                self.block.bytes = bytes as u32;
                self.cursor = 0;
                self.stage = Stage::FrameErgot;
            }
            Stage::FrameErgot | Stage::FrameCopy => {
                let plain = matches!(self.stage, Stage::FrameCopy);
                let bytes = black_box(self.encoder.finish(0));
                let header = frame_header();
                let link = LinkMeta {
                    src_node: 1,
                    dst: LinkDst::Node(2),
                };
                let body = RawTelemetryBlockRef { data: bytes };
                let producer = self.frame_queue.framed_producer();
                let mut sink = Sink::new(self.frame_queue.framed_producer(), FRAME_MTU);
                let (result, cycles) = if plain {
                    measure(&mut clock, || {
                        let mut grant = producer.grant(FRAME_MTU).map_err(drop)?;
                        grant[..bytes.len()].copy_from_slice(bytes);
                        grant.commit(bytes.len() as u16);
                        Ok(())
                    })
                } else {
                    measure(&mut clock, || {
                        black_box(&mut sink).send_ty(&link, &header, &body)
                    })
                };
                result?;
                let consumer = self.frame_queue.framed_consumer();
                let grant = consumer.read().map_err(drop)?;
                let len = black_box(&grant[..]).len();
                grant.release();
                if self.round >= WARMUPS {
                    self.framing[self.case][usize::from(plain)].record(cycles, len);
                }
                self.stage = if plain {
                    Stage::Checksum
                } else {
                    Stage::FrameCopy
                };
            }
            Stage::Checksum => {
                // Rewriting the same header is outside measurement and avoids
                // keeping a self-referential slice or using raw pointers.
                let bytes = black_box(self.encoder.finish(0));
                let end = (self.cursor + 64).min(bytes.len());
                for &byte in &bytes[self.cursor..end] {
                    self.block.checksum = self.block.checksum.rotate_left(5) ^ u32::from(byte);
                }
                self.cursor = end;
                if end == bytes.len() {
                    if self.round >= WARMUPS {
                        let result = &mut self.stats[self.case];
                        result.begin += self.block.begin;
                        result.push += self.block.push;
                        result.finish += self.block.finish;
                        let total = self.block.begin + self.block.push + self.block.finish;
                        result.min = result.min.min(total);
                        result.max = result.max.max(total);
                        result.first_push += self.block.first_push;
                        result.following_push += self.block.following_push;
                        result.bytes += self.block.bytes;
                        result.checksum = result.checksum.rotate_left(3) ^ self.block.checksum;
                        result.push_max = result.push_max.max(self.block.push_max);
                    }
                    self.case += 1;
                    if self.case == CASES {
                        self.case = 0;
                        self.round += 1;
                    }
                    self.stage = Stage::Begin;
                    return Ok(self.round == WARMUPS + REPEATS);
                }
            }
        }
        Ok(false)
    }

    #[cfg(target_arch = "arm")]
    pub fn on_adc(&mut self, stopped: bool, pwm_enables: u16, generation: u32) {
        use device::*;
        let stopped_and_outputs_off = bench_allowed(stopped, pwm_enables);
        SAFE.store(stopped_and_outputs_off, Ordering::Release);
        if !stopped_and_outputs_off {
            IDLE_EPOCH.fetch_add(1, Ordering::Relaxed);
        }
        if STATUS.load(Ordering::Acquire) != ARMED {
            return;
        }
        if !stopped_and_outputs_off {
            abort(1);
            return;
        }
        if !cortex_m::peripheral::DWT::cycle_counter_enabled()
            || self.generation.is_some_and(|old| old != generation)
        {
            abort(2);
            return;
        }
        self.generation = Some(generation);
        self.divider = self.divider.wrapping_add(1) & 15;
        if self.divider != 0 {
            return;
        }
        match self.visit(cortex_m::peripheral::DWT::cycle_count) {
            Ok(false) => {}
            Ok(true) => {
                for (shared, result) in RESULTS.iter().zip(self.stats) {
                    for (slot, value) in shared.iter().zip(result.words()) {
                        slot.store(value, Ordering::Relaxed);
                    }
                }
                for (slot, value) in
                    EMPTY
                        .iter()
                        .zip([self.empty_sum, self.empty_min, self.empty_max])
                {
                    slot.store(value, Ordering::Relaxed);
                }
                for (shared, results) in FRAMING.iter().zip(self.framing) {
                    for (slots, result) in shared.iter().zip(results) {
                        for (slot, value) in slots.iter().zip(result.0) {
                            slot.store(value, Ordering::Relaxed);
                        }
                    }
                }
                STATUS.store(DONE, Ordering::Release);
            }
            Err(()) => abort(3),
        }
    }
}

#[cfg(target_arch = "arm")]
mod device {
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
    pub(super) const WAITING: u32 = 0;
    pub(super) const ARMED: u32 = 1;
    pub(super) const DONE: u32 = 2;
    pub(super) const ABORTED: u32 = 3;
    pub(super) static STATUS: AtomicU32 = AtomicU32::new(WAITING);
    pub(super) static SAFE: AtomicBool = AtomicBool::new(false);
    pub(super) static IDLE_EPOCH: AtomicU32 = AtomicU32::new(0);
    pub(super) static REASON: AtomicU32 = AtomicU32::new(0);
    pub(super) static RESULTS: [[AtomicU32; 10]; super::CASES] =
        [const { [const { AtomicU32::new(0) }; 10] }; super::CASES];
    pub(super) static EMPTY: [AtomicU32; 3] = [const { AtomicU32::new(0) }; 3];
    pub(super) static FRAMING: [[[AtomicU32; 6]; 2]; super::CASES] =
        [const { [const { [const { AtomicU32::new(0) }; 6] }; 2] }; super::CASES];
    pub(super) fn abort(reason: u32) {
        REASON.store(reason, Ordering::Relaxed);
        STATUS.store(ABORTED, Ordering::Release);
    }
}

#[cfg(target_arch = "arm")]
#[embassy_executor::task]
pub async fn report_task(stack: &'static crate::transport::Stack) {
    use device::*;
    use embassy_time::{Instant, Timer};
    use ergot::interface_manager::{InterfaceState, Profile};
    let ident = *crate::protocol::USB_IDENT.get().await;
    let mut idle_since = None;
    let mut idle_epoch = IDLE_EPOCH.load(Ordering::Relaxed);
    loop {
        Timer::after_millis(20).await;
        let active = stack.manage_profile(|profile| {
            matches!(
                profile.interface_state(ident),
                Some(InterfaceState::Active { .. })
            )
        });
        let status = STATUS.load(Ordering::Acquire);
        if status == DONE {
            break;
        }
        if status == ABORTED {
            defmt::error!(
                "codecbench: aborted reason={}",
                REASON.load(Ordering::Relaxed)
            );
            return;
        }
        if status == ARMED {
            if !active {
                abort(4);
            }
            continue;
        }
        let epoch = IDLE_EPOCH.load(Ordering::Relaxed);
        if !active || !SAFE.load(Ordering::Acquire) || idle_epoch != epoch {
            idle_since = None;
            idle_epoch = epoch;
            continue;
        }
        let since = idle_since.get_or_insert_with(Instant::now);
        if since.elapsed().as_millis() >= 2000 {
            defmt::info!(
                "codecbench: start records=64 warmups={} repeats={} visit_adc=16 pushes_per_visit=4 input=ram",
                WARMUPS,
                REPEATS
            );
            STATUS.store(ARMED, Ordering::Release);
        }
    }
    defmt::info!(
        "codecbench: empty n={} sum={} min={} max={}",
        EMPTY_BRACKETS,
        EMPTY[0].load(Ordering::Relaxed),
        EMPTY[1].load(Ordering::Relaxed),
        EMPTY[2].load(Ordering::Relaxed)
    );
    Timer::after_millis(20).await;
    for (case, shared) in RESULTS.iter().enumerate() {
        let [
            begin,
            push,
            finish,
            min,
            max,
            first,
            following,
            bytes,
            checksum,
            push_max,
        ] = core::array::from_fn(|i| shared[i].load(Ordering::Relaxed));
        defmt::info!(
            "codecbench: fixture={} encoding={} blocks={} records={} begin={} push={} finish={} block_min={} block_max={} first_push={} following_push={} push_max={} bytes={} checksum={}",
            case / 3,
            encoding(case),
            REPEATS,
            REPEATS * RECORDS as u32,
            begin,
            push,
            finish,
            min,
            max,
            first,
            following,
            push_max,
            bytes,
            checksum
        );
        Timer::after_millis(20).await;
        for (plain, stats) in FRAMING[case].iter().enumerate() {
            let [sum, min, max, bytes, count, over] =
                core::array::from_fn(|i| stats[i].load(Ordering::Relaxed));
            defmt::info!(
                "framebench: fixture={} encoding={} plain={} mtu={} n={} sum={} min={} max={} bytes={} op_over_50us={}",
                case / 3,
                encoding(case),
                plain,
                FRAME_MTU,
                count,
                sum,
                min,
                max,
                bytes,
                over
            );
            Timer::after_millis(20).await;
        }
    }
    defmt::info!("codecbench: complete");
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxifoc_core::raw_telemetry::decode_raw_block;

    #[test]
    fn stopped_admission_keeps_adc_moe_but_requires_all_phase_outputs_off() {
        assert!(bench_allowed(true, 0));
        assert!(bench_allowed(true, 1 << 6));
        assert!(!bench_allowed(false, 1 << 6));
        assert!(!bench_allowed(false, 0));
        for phase_enable in 0..6 {
            assert!(!bench_allowed(true, 1 << phase_enable));
            assert!(!bench_allowed(true, (1 << 6) | (1 << phase_enable)));
        }
    }

    #[test]
    fn frozen_fixtures_roundtrip_all_encodings_including_full_width_fallback() {
        for case in 0..CASES {
            let mut encoder = RawBlockEncoder::new();
            encoder.begin(SESSION, case as u32, encoding(case), 1);
            for index in 0..RECORDS {
                assert!(encoder.push(&fixture(case, index)));
            }
            let decoded = decode_raw_block(encoder.finish(0)).unwrap();
            assert_eq!(decoded.samples.len(), RECORDS);
            assert_eq!(decoded.header.is_fallback(), case >= 6 && case % 3 != 0);
            for (index, actual) in decoded.samples.iter().enumerate() {
                assert_eq!(*actual, fixture(case, index));
            }
        }
    }

    #[test]
    fn actual_sink_matches_explicit_wire_bytes_and_plain_copy_across_queue_wraps() {
        let queue = FrameQueue::new();
        let header = frame_header();
        let link = LinkMeta {
            src_node: 1,
            dst: LinkDst::Node(2),
        };
        let mut sink = Sink::new(queue.framed_producer(), FRAME_MTU);
        let consumer = queue.framed_consumer();
        for case in (0..CASES).cycle().take(CASES * 4) {
            let mut encoder = RawBlockEncoder::new();
            encoder.begin(SESSION, case as u32, encoding(case), 1);
            for index in 0..RECORDS {
                assert!(encoder.push(&fixture(case, index)));
            }
            let bytes = encoder.finish(0);
            // Fixed post-routing addresses, metadata and absent nash. Construct
            // the body length explicitly, independently of Ergot/Postcard.
            let mut expected = std::vec![0x80, 0x82, 0x04, 0x80, 0x84, 0x04, 0xde];
            expected.extend_from_slice(&RawTelemetryBlockRefTopic::TOPIC_KEY.to_bytes());
            expected.push(0);
            assert!((128..16_384).contains(&bytes.len()));
            expected
                .extend_from_slice(&[(bytes.len() as u8 & 0x7f) | 0x80, (bytes.len() >> 7) as u8]);
            expected.extend_from_slice(bytes);
            sink.send_ty(&link, &header, &RawTelemetryBlockRef { data: bytes })
                .unwrap();
            let grant = consumer.read().unwrap();
            assert_eq!(&grant[..], &expected);
            let decoded = ergot::wire_frames::de_frame(&grant).unwrap();
            assert_eq!(decoded.hdr, header);
            let owned: oxifoc_core::icd::RawTelemetryBlock =
                postcard::from_bytes(decoded.body.unwrap()).unwrap();
            assert_eq!(&owned.data[..], bytes);
            grant.release();
            assert!(consumer.read().is_err());

            let producer = queue.framed_producer();
            let mut grant = producer.grant(FRAME_MTU).unwrap();
            grant[..bytes.len()].copy_from_slice(bytes);
            grant.commit(bytes.len() as u16);
            let grant = consumer.read().unwrap();
            assert_eq!(&grant[..], bytes);
            grant.release();
            assert!(consumer.read().is_err());

            // Failed serialization aborts its grant without publishing bytes.
            let mut short_sink = Sink::new(queue.framed_producer(), (expected.len() - 1) as u16);
            assert!(
                short_sink
                    .send_ty(&link, &header, &RawTelemetryBlockRef { data: bytes })
                    .is_err()
            );
            assert!(consumer.read().is_err());
        }
        // Exercise a full queue refusal, then verify all accepted frames and
        // an empty consumer after draining; no full-queue reset is required.
        let body = RawTelemetryBlockRef {
            data: &[0x5a; 2208],
        };
        let mut count = 0;
        while sink.send_ty(&link, &header, &body).is_ok() {
            count += 1;
            assert!(count < 8);
        }
        assert!(count > 0);
        for _ in 0..count {
            let grant = consumer.read().unwrap();
            let decoded = ergot::wire_frames::de_frame(&grant).unwrap();
            let actual: oxifoc_core::icd::RawTelemetryBlock =
                postcard::from_bytes(decoded.body.unwrap()).unwrap();
            assert_eq!(&actual.data[..], body.data);
            grant.release();
        }
        assert!(consumer.read().is_err());
    }

    #[test]
    fn finite_round_robin_excludes_warmup_and_accounts_each_timed_operation() {
        let mut bench = Bench::new();
        let mut cycles = u32::MAX - 100;
        let mut done = false;
        for _ in 0..10_000 {
            done = bench
                .visit(|| {
                    cycles = cycles.wrapping_add(10);
                    cycles
                })
                .unwrap();
            if done {
                break;
            }
        }
        assert!(done);
        assert_eq!(bench.round, WARMUPS + REPEATS);
        assert_eq!(bench.empty_n, EMPTY_BRACKETS);
        assert_eq!(bench.empty_sum, EMPTY_BRACKETS * 10);
        for result in bench.stats {
            assert_eq!(result.begin, REPEATS * 10);
            assert_eq!(result.push, REPEATS * RECORDS as u32 * 10);
            assert_eq!(result.finish, REPEATS * 10);
            assert_eq!(result.first_push, REPEATS * 16 * 10);
            assert_eq!(result.following_push, REPEATS * 48 * 10);
            assert_eq!(result.min, 660);
            assert_eq!(result.max, 660);
            assert_ne!(result.checksum, 0);
        }
        assert_eq!(bench.stats[0].bytes, REPEATS * (32 + 64 * 34));
        assert_eq!(bench.stats[1].bytes, REPEATS * (32 + 64 * 26));
        for result in &bench.stats[6..] {
            assert_eq!(result.bytes, bench.stats[0].bytes);
        }
        for (case, results) in bench.framing.iter().enumerate() {
            for (plain, result) in results.iter().enumerate() {
                let overhead = if plain == 0 { REPEATS * 18 } else { 0 };
                assert_eq!(
                    result.0,
                    [
                        REPEATS * 10,
                        10,
                        10,
                        bench.stats[case].bytes + overhead,
                        REPEATS,
                        0
                    ]
                );
            }
        }
        assert!(bench.frame_queue.framed_consumer().read().is_err());
    }
}
