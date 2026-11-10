use super::*;
use std::vec::Vec;

fn random(seed: &mut u32) -> u16 {
    *seed ^= *seed << 13;
    *seed ^= *seed >> 17;
    *seed ^= *seed << 5;
    *seed as u16
}
fn arbitrary(seed: &mut u32) -> FastTelemetry {
    frame(core::array::from_fn(|_| random(seed)))
}
fn valid(seed: &mut u32, seq: u16) -> FastTelemetry {
    let mut f = arbitrary(seed);
    f.ia &= 4095;
    f.ib &= 4095;
    f.ic &= 4095;
    f.vphase_adc = f.vphase_adc.map(|v| v & 4095);
    f.pwm_enables &= 127;
    f.pwm_arr = 65_535;
    f.seq = seq;
    f
}
#[test]
fn raw34_preserves_arbitrary_full_width_frames_and_independent_blocks() {
    let mut encoder = RawBlockEncoder::new();
    let mut seed = 0x7912d65;
    for block in 0..100 {
        encoder.begin(13, block, RawEncoding::Raw34, 1);
        let expected: Vec<_> = (0..64).map(|_| arbitrary(&mut seed)).collect();
        for f in &expected {
            assert!(encoder.push(f));
        }
        assert!(!encoder.push(&arbitrary(&mut seed)));
        let bytes = encoder.finish(7);
        assert_eq!(bytes.len(), 32 + 64 * 34);
        let decoded = decode_raw_block(bytes).unwrap();
        assert_eq!(decoded.samples.as_slice(), expected);
        assert!(!decoded.header.is_fallback());
        assert_eq!(decoded.header.producer_drops, 7);
    }
}
#[test]
fn packed26_retains_every_scalar_bit_full_ccr_absence_and_sequence_wrap() {
    let mut encoder = RawBlockEncoder::new();
    let mut seed = 0x7213e5b;
    for block in 0..100 {
        encoder.begin(19, block, RawEncoding::Packed26, 3);
        let expected: Vec<_> = (0..64_u16)
            .map(|i| {
                let mut f = valid(&mut seed, 65530_u16.wrapping_add(i.wrapping_mul(3)));
                if i % 7 == 0 {
                    f.vphase_adc = [u16::MAX; 3];
                }
                f
            })
            .collect();
        for f in &expected {
            assert!(encoder.push(f));
        }
        let bytes = encoder.finish(0);
        assert_eq!(bytes.len(), 32 + 64 * 26);
        let decoded = decode_raw_block(bytes).unwrap();
        assert_eq!(decoded.samples.as_slice(), expected);
        assert_eq!(decoded.header.encoding, RawEncoding::Packed26);
    }
}
#[test]
fn fallback_is_explicit_and_retains_invalid_adc_mixed_sentinels_and_reserved_enables() {
    let base = FastTelemetry {
        vphase_adc: [0, 4095, 2000],
        pwm_enables: 127,
        ..FastTelemetry::default()
    };
    let cases = [
        FastTelemetry { ia: 4096, ..base },
        FastTelemetry {
            ib: u16::MAX,
            ..base
        },
        FastTelemetry {
            vphase_adc: [65535, 0, 4095],
            ..base
        },
        FastTelemetry {
            vphase_adc: [4096, 0, 0],
            ..base
        },
        FastTelemetry {
            pwm_enables: 128,
            ..base
        },
        FastTelemetry {
            pwm_enables: 65535,
            ..base
        },
    ];
    let mut encoder = RawBlockEncoder::new();
    for input in cases {
        encoder.begin(21, 0, RawEncoding::Packed26, 1);
        assert!(encoder.push(&input));
        // Raw fallback can retain arbitrary following seq/ARR values without
        // one-record header storms during an irregular run.
        let next = FastTelemetry {
            seq: 35,
            pwm_arr: 53,
            ..base
        };
        assert!(encoder.push(&next));
        let decoded = decode_raw_block(encoder.finish(0)).unwrap();
        assert!(decoded.header.is_fallback());
        assert_eq!(decoded.header.encoding, RawEncoding::Raw34);
        assert_eq!(decoded.samples.as_slice(), &[input, next]);
    }
}
#[test]
fn packed_gap_arr_change_or_invalid_record_refuses_transactionally() {
    let first = FastTelemetry {
        seq: 65535,
        pwm_arr: 4195,
        ..FastTelemetry::default()
    };
    let mut encoder = RawBlockEncoder::new();
    encoder.begin(8, 0, RawEncoding::Packed26, 1);
    assert!(encoder.push(&first));
    let saved = encoder.finish(0).to_vec();
    for next in [
        FastTelemetry { seq: 1, ..first },
        FastTelemetry {
            seq: 0,
            pwm_arr: 4196,
            ..first
        },
        FastTelemetry {
            seq: 0,
            ia: 5000,
            ..first
        },
    ] {
        assert!(!encoder.push(&next));
        assert_eq!(encoder.finish(0), saved);
    }
    assert!(encoder.push(&FastTelemetry { seq: 0, ..first }));
}
#[test]
fn incremental_decoder_accepts_arbitrary_chunks_empty_completions_and_end() {
    let mut encoder = RawBlockEncoder::new();
    encoder.begin(91, 0, RawEncoding::Packed26, 1);
    let input = FastTelemetry {
        seq: 400,
        pwm_enables: 64,
        ..FastTelemetry::default()
    };
    assert!(encoder.push(&input));
    let mut stream = encoder.finish(0).to_vec();
    stream.extend_from_slice(encoder.end(91, 1, RawEncoding::Packed26, 1, 0));
    for size in 1..=stream.len() {
        let mut decoder = RawStreamDecoder::new();
        let mut blocks = Vec::new();
        for bytes in stream.chunks(size) {
            decoder.push(&[], |_| panic!()).unwrap();
            decoder.push(bytes, |block| blocks.push(block)).unwrap();
        }
        assert_eq!(blocks.len(), 2);
        assert_eq!(blocks[0].samples.as_slice(), &[input]);
        assert!(blocks[1].header.is_end());
        assert_eq!(blocks[1].header.total_records, 1);
        assert!(blocks[1].samples.is_empty());
    }
}
#[test]
fn malformed_headers_fail_closed_and_reset_does_not_replay_stale_session() {
    let mut encoder = RawBlockEncoder::new();
    encoder.begin(1, 0, RawEncoding::Raw34, 1);
    assert!(encoder.push(&FastTelemetry::default()));
    let good = encoder.finish(0).to_vec();
    for offset in [0, 4, 5, 6, 7, 30] {
        let mut bad = good.clone();
        bad[offset] = 255;
        assert!(decode_raw_block(&bad).is_err());
        let mut decoder = RawStreamDecoder::new();
        assert!(decoder.push(&bad, |_| panic!()).is_err());
        let mut count = 0;
        decoder.push(&good, |_| count += 1).unwrap();
        assert_eq!(count, 1);
    }
    let mut decoder = RawStreamDecoder::new();
    decoder.push(&good[..20], |_| panic!()).unwrap();
    decoder.reset();
    encoder.begin(2, 0, RawEncoding::Raw34, 1);
    assert!(encoder.push(&FastTelemetry {
        seq: 12,
        ..FastTelemetry::default()
    }));
    decoder
        .push(encoder.finish(0), |b| assert_eq!(b.header.session, 2))
        .unwrap();
}
