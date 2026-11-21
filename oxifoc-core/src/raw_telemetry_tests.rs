use super::*;
use crate::types::HallAngle;
use std::vec::Vec;

fn random(seed: &mut u32) -> u16 {
    *seed ^= *seed << 13;
    *seed ^= *seed >> 17;
    *seed ^= *seed << 5;
    *seed as u16
}
fn valid(seed: &mut u32, seq: u16) -> FastTelemetry {
    let mut word = || random(seed);
    FastTelemetry {
        ia: word() & 4095,
        ib: word() & 4095,
        ic: word() & 4095,
        vbus: word(),
        angle: word(),
        hall_angle: HallAngle::from_encoded(word()),
        vd: word() as i16,
        vq: word() as i16,
        rpm: word() as i16,
        seq,
        vphase_adc: [word() & 4095, word() & 4095, word() & 4095],
        pwm_ccr: [word(), word(), word()],
        pwm_arr: 65_535,
        pwm_enables: word() & 127,
    }
}

#[test]
fn packed28_preserves_every_hall_code_and_other_fields_across_sequence_wrap() {
    let mut encoder = RawBlockEncoder::new();
    let mut seed = 0x7213e5b;
    for block in 0..1024_u32 {
        encoder.begin(19, block, 3);
        let expected: Vec<_> = (0..64_u16)
            .map(|i| {
                let mut f = valid(&mut seed, 65530_u16.wrapping_add(i.wrapping_mul(3)));
                f.hall_angle = HallAngle::from_encoded((block * 64 + u32::from(i)) as u16);
                if i % 7 == 0 {
                    f.vphase_adc = [u16::MAX; 3];
                }
                f
            })
            .collect();
        for f in &expected {
            assert!(encoder.push(f));
        }
        assert!(encoder.is_full());
        assert!(!encoder.push(&valid(&mut seed, 65530_u16.wrapping_add(64 * 3))));
        let bytes = encoder.finish(7);
        assert_eq!(bytes.len(), 32 + 64 * 28);
        let decoded = decode_raw_block(bytes).unwrap();
        assert_eq!(decoded.samples.as_slice(), expected);
        assert_eq!(decoded.header.producer_drops, 7);
        assert_eq!(decoded.header.block_index, block);
    }
}

#[test]
fn unpackable_frames_are_refused_without_a_fallback_format() {
    let base = FastTelemetry {
        vphase_adc: [0, 4095, 2000],
        pwm_enables: 127,
        hall_angle: HallAngle::from_encoded(65535),
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
        encoder.begin(21, 0, 1);
        assert!(!encoder.push(&input));
        assert!(encoder.is_empty());
    }
    encoder.begin(21, 0, 1);
    assert!(encoder.push(&base));
    assert_eq!(
        decode_raw_block(encoder.finish(0)).unwrap().samples[0],
        base
    );
}

#[test]
fn gap_arr_change_or_invalid_record_refuses_transactionally_and_begin_starts_afresh() {
    let first = FastTelemetry {
        seq: 65535,
        pwm_arr: 4195,
        ..FastTelemetry::default()
    };
    let mut encoder = RawBlockEncoder::new();
    encoder.begin(8, 0, 1);
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
        FastTelemetry {
            seq: 0,
            pwm_enables: 128,
            ..first
        },
        FastTelemetry {
            seq: 0,
            vphase_adc: [65535, 0, 4095],
            ..first
        },
    ] {
        assert!(!encoder.push(&next));
        assert_eq!(encoder.finish(0), saved);
    }
    let second = FastTelemetry { seq: 0, ..first };
    assert!(encoder.push(&second));
    assert_eq!(
        decode_raw_block(encoder.finish(0))
            .unwrap()
            .samples
            .as_slice(),
        &[first, second]
    );
    // A new block takes its sequence and ARR from its own first record.
    let other = FastTelemetry {
        seq: 9,
        pwm_arr: 53,
        ..first
    };
    encoder.begin(8, 1, 1);
    assert!(encoder.push(&other));
    assert_eq!(
        &encoder.finish(0)[RAW_HEADER_BYTES..],
        reference_packed(&other)
    );
    let decoded = decode_raw_block(encoder.finish(0)).unwrap();
    assert_eq!(decoded.header.first_seq, 9);
    assert_eq!(decoded.header.pwm_arr, 53);
    assert_eq!(decoded.samples.as_slice(), &[other]);
}

#[test]
fn incremental_decoder_accepts_arbitrary_chunks_empty_completions_and_end() {
    let mut encoder = RawBlockEncoder::new();
    encoder.begin(91, 0, 1);
    let input = [
        FastTelemetry {
            seq: 65535,
            pwm_enables: 64,
            vphase_adc: [65535; 3],
            ..FastTelemetry::default()
        },
        FastTelemetry {
            seq: 0,
            pwm_enables: 64,
            vphase_adc: [65535; 3],
            ..FastTelemetry::default()
        },
    ];
    for sample in &input {
        assert!(encoder.push(sample));
    }
    let mut stream = encoder.finish(0).to_vec();
    stream.extend_from_slice(encoder.end(91, 1, 2, 0));
    for size in 1..=stream.len() {
        let mut decoder = RawStreamDecoder::new();
        let mut blocks = Vec::new();
        for bytes in stream.chunks(size) {
            decoder.push(&[], |_| panic!()).unwrap();
            decoder.push(bytes, |block| blocks.push(block)).unwrap();
        }
        assert_eq!(blocks.len(), 2);
        assert_eq!(blocks[0].samples.as_slice(), &input);
        assert!(blocks[1].header.is_end());
        assert_eq!(blocks[1].header.total_records, 2);
        assert!(blocks[1].samples.is_empty());
    }
}

#[test]
fn malformed_headers_fail_closed_and_reset_does_not_replay_stale_session() {
    let mut encoder = RawBlockEncoder::new();
    encoder.begin(1, 0, 1);
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
    encoder.begin(2, 0, 1);
    assert!(encoder.push(&FastTelemetry {
        seq: 12,
        ..FastTelemetry::default()
    }));
    decoder
        .push(encoder.finish(0), |b| assert_eq!(b.header.session, 2))
        .unwrap();
}

#[test]
fn only_the_packed28_tag_and_known_flags_parse() {
    let mut encoder = RawBlockEncoder::new();
    encoder.begin(3, 0, 1);
    assert!(encoder.push(&FastTelemetry::default()));
    let good = encoder.finish(0).to_vec();
    let end = RawBlockEncoder::new().end(3, 1, 1, 0).to_vec();
    for block in [&good, &end] {
        assert_eq!(block[5], 3);
        assert!(decode_raw_block(block).is_ok());
        for tag in (0..=255).filter(|&tag| tag != 3) {
            let mut bad = block.clone();
            bad[5] = tag;
            assert_eq!(
                RawBlockHeader::parse(&bad).unwrap_err(),
                RawDecodeError::Header
            );
        }
        // Bit 1 was the full-width fallback flag; bits 3..7 are unassigned.
        for flag in (1..8).filter(|&bit| bit != 2).map(|bit| 1_u8 << bit) {
            let mut bad = block.clone();
            bad[6] |= flag;
            assert!(RawBlockHeader::parse(&bad).is_err(), "flag {flag}");
        }
    }
}

#[test]
fn parser_rejects_mismatched_payload_lengths_and_trailing_bytes() {
    let mut encoder = RawBlockEncoder::new();
    encoder.begin(43, 0, 1);
    assert!(encoder.push(&FastTelemetry::default()));
    assert!(encoder.push(&FastTelemetry {
        seq: 1,
        ..FastTelemetry::default()
    }));
    let good = encoder.finish(0).to_vec();
    for kind in 0..5 {
        let mut bad = good.clone();
        match kind {
            0 => bad.push(0),
            1 => bad[30..32].copy_from_slice(&55_u16.to_le_bytes()),
            2 => bad[30..32].copy_from_slice(&28_u16.to_le_bytes()),
            3 => {
                bad.extend([0; 28]);
                bad[30..32].copy_from_slice(&84_u16.to_le_bytes());
            }
            _ => bad[7] = 0,
        }
        assert!(decode_raw_block(&bad).is_err(), "mutation{kind}");
        let mut decoder = RawStreamDecoder::new();
        let mut exposed = 0;
        let result = decoder.push(&bad, |_| exposed += 1);
        // A trailing byte is the start of the next header, not part of this block.
        if kind == 0 {
            assert!(result.is_ok());
            assert_eq!(exposed, 1);
            decoder.reset();
        } else {
            assert!(result.is_err());
            assert_eq!(exposed, 0, "malformed block exposed samples");
        }
        let mut recovered = 0;
        decoder.push(&good, |_| recovered += 1).unwrap();
        assert_eq!(recovered, 1);
    }
}

// Bit-at-a-time packing, deliberately independent of the production helpers.
fn reference_packed(f: &FastTelemetry) -> [u8; 28] {
    let mut out = [0; 28];
    let present = f.vphase_adc != [u16::MAX; 3];
    let phase = if present { f.vphase_adc } else { [0; 3] };
    for (i, value) in [f.ia, f.ib, f.ic, phase[0], phase[1], phase[2]]
        .iter()
        .enumerate()
    {
        for bit in 0..12 {
            out[(i * 12 + bit) / 8] |= (((value >> bit) & 1) as u8) << ((i * 12 + bit) % 8);
        }
    }
    out[9] = f.pwm_enables as u8 | if present { 128 } else { 0 };
    for (i, value) in [
        f.vbus,
        f.angle,
        f.vd as u16,
        f.vq as u16,
        f.rpm as u16,
        f.pwm_ccr[0],
        f.pwm_ccr[1],
        f.pwm_ccr[2],
        f.hall_angle.encoded(),
    ]
    .iter()
    .enumerate()
    {
        for bit in 0..16 {
            out[10 + i * 2 + bit / 8] |= (((value >> bit) & 1) as u8) << (bit % 8);
        }
    }
    out
}

#[test]
fn packed28_matches_independent_wire_oracle() {
    let mut encoder = RawBlockEncoder::new();
    let mut seed = 0x671a389;
    for block in 0..100 {
        let expected: Vec<_> = (0..64_u16)
            .map(|i| {
                let mut sample = valid(&mut seed, 65530_u16.wrapping_add(i.wrapping_mul(3)));
                if i % 7 == 0 {
                    sample.vphase_adc = [u16::MAX; 3];
                }
                sample
            })
            .collect();
        encoder.begin(31, block, 3);
        for sample in &expected {
            assert!(encoder.push(sample));
        }
        assert_eq!(
            &encoder.finish(0)[RAW_HEADER_BYTES..],
            expected
                .iter()
                .flat_map(reference_packed)
                .collect::<Vec<_>>()
        );
    }
}

#[test]
fn packed28_keeps_its_exact_header_and_payload_bytes() {
    let sample = FastTelemetry {
        ia: 0x123,
        ib: 0x456,
        ic: 0x789,
        vbus: 0xabcd,
        angle: 0x1234,
        hall_angle: HallAngle::from_encoded(0xbeef),
        vd: -2,
        vq: 32767,
        rpm: -32768,
        seq: 65535,
        vphase_adc: [0xabc, 0xdef, 0x123],
        pwm_ccr: [0, 65535, 0xabcd],
        pwm_arr: 65535,
        pwm_enables: 127,
    };
    let payload = [
        0x23, 0x61, 0x45, 0x89, 0xc7, 0xab, 0xef, 0x3d, 0x12, 0xff, 0xcd, 0xab, 0x34, 0x12, 0xfe,
        0xff, 0xff, 0x7f, 0, 0x80, 0, 0, 0xff, 0xff, 0xcd, 0xab, 0xef, 0xbe,
    ];
    let mut expected = Vec::from(*b"OXTR");
    expected.extend([1, 3, 0, 1]);
    expected.extend([
        8, 7, 6, 5, 4, 3, 2, 1, 0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55, 0xff, 0xff, 3, 0,
        0xff, 0xff, 28, 0,
    ]);
    expected.extend(payload);
    let mut encoder = RawBlockEncoder::new();
    encoder.begin(0x0102_0304_0506_0708, 0x1122_3344, 3);
    assert!(encoder.push(&sample));
    assert_eq!(encoder.finish(0x5566_7788), expected);
    assert_eq!(
        decode_raw_block(&expected).unwrap().samples.as_slice(),
        &[sample]
    );
    let mut end = Vec::from(*b"OXTR");
    end.extend([1, 3, 1, 0]);
    end.extend([
        8, 7, 6, 5, 4, 3, 2, 1, 0x45, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55, 9, 0, 0, 0, 0, 0,
        0, 0,
    ]);
    assert_eq!(
        encoder.end(0x0102_0304_0506_0708, 0x1122_3345, 9, 0x5566_7788),
        end
    );
}

#[test]
fn adversarial_header_and_payload_mutations_are_bounded_and_panic_free() {
    let mut encoder = RawBlockEncoder::new();
    let mut seed = 0x8827641;
    encoder.begin(53, 0, 1);
    for seq in 0..4 {
        assert!(encoder.push(&valid(&mut seed, seq)));
    }
    let good = encoder.finish(0).to_vec();
    for _ in 0..6144 {
        let mut bad = good.clone();
        let index = usize::from(random(&mut seed)) % bad.len();
        bad[index] = random(&mut seed) as u8;
        if random(&mut seed) & 7 == 0 {
            bad.truncate(usize::from(random(&mut seed)) % bad.len());
        }
        let _ = decode_raw_block(&bad);
        let mut decoder = RawStreamDecoder::new();
        for chunk in bad.chunks(7) {
            if decoder
                .push(chunk, |block| {
                    assert!(block.samples.len() <= RAW_MAX_RECORDS);
                })
                .is_err()
            {
                break;
            }
        }
    }
}

#[test]
fn block_limit_caps_records_and_keeps_later_blocks_at_that_size() {
    let mut encoder = RawBlockEncoder::new();
    let mut seed = 0x51ed;
    // A 478-byte limit (a 505-byte BLE frame) holds 15 Packed28 records.
    encoder.set_block_limit(478);
    for block in 0..2 {
        encoder.begin(77, block, 1);
        let first = (block * 15) as u16;
        for seq in first..first + 15 {
            assert!(encoder.push(&valid(&mut seed, seq)));
        }
        assert!(encoder.is_full());
        assert!(!encoder.push(&valid(&mut seed, first + 15)));
        let bytes = encoder.finish(0);
        assert_eq!(bytes.len(), RAW_HEADER_BYTES + 15 * PACKED28_BYTES);
        assert!(bytes.len() <= 478);
        assert_eq!(decode_raw_block(bytes).unwrap().samples.len(), 15);
    }
    encoder.set_block_limit(RAW_MIN_BLOCK_BYTES);
    encoder.begin(77, 2, 1);
    assert!(encoder.push(&valid(&mut seed, 30)));
    assert!(encoder.is_full());
    encoder.set_block_limit(usize::MAX);
    encoder.begin(77, 3, 1);
    for seq in 0..RAW_MAX_RECORDS as u16 {
        assert!(encoder.push(&valid(&mut seed, seq)));
    }
    assert!(encoder.is_full());
}
