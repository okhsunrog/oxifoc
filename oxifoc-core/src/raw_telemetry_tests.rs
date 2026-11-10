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
fn packed28_preserves_every_hall_code_and_other_fields_across_sequence_wrap() {
    let mut encoder = RawBlockEncoder::new();
    let mut seed = 0x7213e5b;
    for block in 0..1024_u32 {
        encoder.begin(19, block, RawEncoding::Packed28, 3);
        let expected: Vec<_> = (0..64_u16)
            .map(|i| {
                let mut f = valid(&mut seed, 65530_u16.wrapping_add(i.wrapping_mul(3)));
                f.hall_angle =
                    crate::types::HallAngle::from_encoded((block * 64 + u32::from(i)) as u16);
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
        assert_eq!(bytes.len(), 32 + 64 * 28);
        let decoded = decode_raw_block(bytes).unwrap();
        assert_eq!(decoded.samples.as_slice(), expected);
        assert_eq!(decoded.header.encoding, RawEncoding::Packed28);
        assert!(!decoded.header.is_fallback());
    }
}

#[test]
fn fixed_codec_never_changes_format_or_silently_discards_hall() {
    let sample = FastTelemetry {
        hall_angle: crate::types::HallAngle::from_encoded(65535),
        ..Default::default()
    };
    let mut encoder = RawBlockEncoder::new();
    for encoding in [
        RawEncoding::Raw34,
        RawEncoding::Packed26,
        RawEncoding::Packed26Delta,
    ] {
        encoder.begin(1, 0, encoding, 1);
        assert!(!encoder.push(&sample));
        assert!(encoder.is_empty());
    }
    encoder.begin(1, 0, RawEncoding::Packed28, 1);
    assert!(!encoder.push(&FastTelemetry { ia: 4096, ..sample }));
    assert!(encoder.is_empty());
    assert!(encoder.push(&sample));
    assert_eq!(
        decode_raw_block(encoder.finish(0)).unwrap().samples[0],
        sample
    );
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

// Bit-at-a-time packing and bytewise delta construction are deliberately
// independent of the production packing/unpacking helpers.
fn reference_packed(f: &FastTelemetry) -> [u8; 26] {
    let mut out = [0; 26];
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

fn reference_delta(frames: &[FastTelemetry]) -> Vec<u8> {
    let mut previous = reference_packed(&frames[0]);
    let mut out = previous.to_vec();
    for sample in &frames[1..] {
        let current = reference_packed(sample);
        let mut mask = 0_u32;
        let mut changed = Vec::new();
        for (i, (&new, &old)) in current.iter().zip(&previous).enumerate() {
            if new != old {
                mask |= 1 << i;
                changed.push(new);
            }
        }
        if 5 + changed.len() < 27 {
            out.push(1);
            out.extend(mask.to_le_bytes());
            out.extend(changed);
        } else {
            out.push(0);
            out.extend(current);
        }
        previous = current;
    }
    out
}

#[test]
fn packed_delta_matches_independent_wire_oracle_and_restores_every_bit() {
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
        encoder.begin(31, block, RawEncoding::Packed26Delta, 3);
        for sample in &expected {
            assert!(encoder.push(sample));
        }
        assert_eq!(
            &encoder.finish(0)[RAW_HEADER_BYTES..],
            reference_delta(&expected)
        );
        let decoded = decode_raw_block(encoder.finish(0)).unwrap();
        assert_eq!(decoded.header.encoding, RawEncoding::Packed26Delta);
        assert_eq!(decoded.samples.as_slice(), expected);
        assert!(encoder.finish(0).len() <= RAW_MAX_BLOCK_BYTES);
        let saved = encoder.finish(0).to_vec();
        assert!(!encoder.push(&expected[0]));
        assert_eq!(encoder.finish(0), saved);
    }
}

#[test]
fn packed_delta_handles_exact_minimum_and_worst_case_without_expanding_storage() {
    let a = FastTelemetry {
        pwm_arr: 65535,
        ..FastTelemetry::default()
    };
    let b = FastTelemetry {
        ia: 4095,
        ib: 4095,
        ic: 4095,
        vbus: 65535,
        angle: 65535,
        hall_angle: Default::default(),
        vd: -1,
        vq: -1,
        rpm: -1,
        seq: 0,
        vphase_adc: [4095; 3],
        pwm_ccr: [65535; 3],
        pwm_arr: 65535,
        pwm_enables: 127,
    };
    let mut encoder = RawBlockEncoder::new();
    for varied in [false, true] {
        encoder.begin(37, 0, RawEncoding::Packed26Delta, 1);
        let expected: Vec<_> = (0..64)
            .map(|i| FastTelemetry {
                seq: i,
                ..if varied && i % 2 != 0 { b } else { a }
            })
            .collect();
        for sample in &expected {
            assert!(encoder.push(sample));
        }
        let bytes = encoder.finish(0);
        assert_eq!(
            bytes.len(),
            RAW_HEADER_BYTES + 26 + 63 * if varied { 27 } else { 5 }
        );
        assert_eq!(
            decode_raw_block(bytes).unwrap().samples.as_slice(),
            expected
        );
    }
}

#[test]
fn packed_delta_refuses_discontinuities_transactionally_and_clears_history_on_begin() {
    let first = FastTelemetry {
        seq: 65535,
        pwm_arr: 4195,
        ..FastTelemetry::default()
    };
    let mut encoder = RawBlockEncoder::new();
    encoder.begin(41, 0, RawEncoding::Packed26Delta, 1);
    assert!(encoder.push(&first));
    let saved = encoder.finish(0).to_vec();
    for invalid in [
        FastTelemetry { seq: 1, ..first },
        FastTelemetry {
            seq: 0,
            pwm_arr: 4196,
            ..first
        },
        FastTelemetry {
            seq: 0,
            ia: 4096,
            ..first
        },
        FastTelemetry {
            seq: 0,
            pwm_enables: 128,
            ..first
        },
    ] {
        assert!(!encoder.push(&invalid));
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
    let invalid = FastTelemetry {
        ia: 65535,
        vphase_adc: [65535, 0, 4095],
        ..first
    };
    encoder.begin(41, 1, RawEncoding::Packed26Delta, 1);
    assert!(encoder.push(&invalid));
    assert!(encoder.push(&second));
    let decoded = decode_raw_block(encoder.finish(0)).unwrap();
    assert_eq!(decoded.header.encoding, RawEncoding::Raw34);
    assert!(decoded.header.is_fallback());
    assert_eq!(decoded.samples.as_slice(), &[invalid, second]);
    encoder.begin(41, 2, RawEncoding::Packed26Delta, 1);
    assert!(encoder.push(&second));
    assert_eq!(
        &encoder.finish(0)[RAW_HEADER_BYTES..],
        reference_packed(&second)
    );
    assert_eq!(
        decode_raw_block(encoder.finish(0))
            .unwrap()
            .samples
            .as_slice(),
        &[second]
    );
}

#[test]
fn packed_delta_parser_rejects_unknown_tags_masks_missing_replacements_and_trailing_bytes() {
    let mut encoder = RawBlockEncoder::new();
    encoder.begin(43, 0, RawEncoding::Packed26Delta, 1);
    assert!(encoder.push(&FastTelemetry::default()));
    assert!(encoder.push(&FastTelemetry {
        seq: 1,
        ..FastTelemetry::default()
    }));
    let good = encoder.finish(0).to_vec();
    for kind in 0..6 {
        let mut bad = good.clone();
        match kind {
            0 => bad[RAW_HEADER_BYTES + 26] = 2,
            1 => bad[RAW_HEADER_BYTES + 30] = 128,
            2 => bad[RAW_HEADER_BYTES + 27] = 1,
            3 => {
                bad.push(0);
                bad[30..32].copy_from_slice(&32_u16.to_le_bytes());
            }
            4 => bad[30..32].copy_from_slice(&25_u16.to_le_bytes()),
            _ => bad[30..32].copy_from_slice(&54_u16.to_le_bytes()),
        }
        assert!(decode_raw_block(&bad).is_err(), "mutation{kind}");
        let mut decoder = RawStreamDecoder::new();
        assert!(
            decoder
                .push(&bad, |_| panic!("malformed block exposed samples"))
                .is_err()
        );
        let mut recovered = 0;
        decoder.push(&good, |_| recovered += 1).unwrap();
        assert_eq!(recovered, 1);
    }
}

#[test]
fn packed_delta_incremental_boundaries_end_and_independent_reentry_are_exact() {
    let mut encoder = RawBlockEncoder::new();
    let sample = FastTelemetry {
        seq: 65535,
        vphase_adc: [65535; 3],
        ..FastTelemetry::default()
    };
    encoder.begin(47, 0, RawEncoding::Packed26Delta, 1);
    assert!(encoder.push(&sample));
    assert!(encoder.push(&FastTelemetry { seq: 0, ..sample }));
    let mut stream = encoder.finish(0).to_vec();
    stream.extend(encoder.end(47, 1, RawEncoding::Packed26Delta, 2, 0));
    for chunk in 1..=stream.len() {
        let mut decoder = RawStreamDecoder::new();
        let mut decoded = Vec::new();
        for bytes in stream.chunks(chunk) {
            decoder.push(&[], |_| panic!()).unwrap();
            decoder.push(bytes, |block| decoded.push(block)).unwrap();
        }
        assert_eq!(decoded.len(), 2);
        assert_eq!(
            decoded[0].samples.as_slice(),
            &[sample, FastTelemetry { seq: 0, ..sample }]
        );
        assert!(decoded[1].header.is_end());
        assert_eq!(decoded[1].header.total_records, 2);
    }
}

#[test]
fn existing_tags_keep_their_exact_header_and_payload_bytes() {
    let sample = FastTelemetry {
        ia: 0x123,
        ib: 0x456,
        ic: 0x789,
        vbus: 0xabcd,
        angle: 0x1234,
        hall_angle: Default::default(),
        vd: -2,
        vq: 32767,
        rpm: -32768,
        seq: 65535,
        vphase_adc: [0xabc, 0xdef, 0x123],
        pwm_ccr: [0, 65535, 0xabcd],
        pwm_arr: 65535,
        pwm_enables: 127,
    };
    let raw = [
        0x23, 1, 0x56, 4, 0x89, 7, 0xcd, 0xab, 0x34, 0x12, 0xfe, 0xff, 0xff, 0x7f, 0, 0x80, 0xff,
        0xff, 0xbc, 0x0a, 0xef, 0x0d, 0x23, 1, 0, 0, 0xff, 0xff, 0xcd, 0xab, 0xff, 0xff, 0x7f, 0,
    ];
    let packed = [
        0x23, 0x61, 0x45, 0x89, 0xc7, 0xab, 0xef, 0x3d, 0x12, 0xff, 0xcd, 0xab, 0x34, 0x12, 0xfe,
        0xff, 0xff, 0x7f, 0, 0x80, 0, 0, 0xff, 0xff, 0xcd, 0xab,
    ];
    for (encoding, tag, payload) in [
        (RawEncoding::Raw34, 0, raw.as_slice()),
        (RawEncoding::Packed26, 1, packed.as_slice()),
    ] {
        let mut expected = Vec::from(*b"OXTR");
        expected.extend([1, tag, 0, 1]);
        expected.extend([
            8,
            7,
            6,
            5,
            4,
            3,
            2,
            1,
            0x44,
            0x33,
            0x22,
            0x11,
            0x88,
            0x77,
            0x66,
            0x55,
            0xff,
            0xff,
            3,
            0,
            0xff,
            0xff,
            payload.len() as u8,
            0,
        ]);
        expected.extend(payload);
        let mut encoder = RawBlockEncoder::new();
        encoder.begin(0x0102_0304_0506_0708, 0x1122_3344, encoding, 3);
        assert!(encoder.push(&sample));
        assert_eq!(encoder.finish(0x5566_7788), expected);
        assert_eq!(
            decode_raw_block(&expected).unwrap().samples.as_slice(),
            &[sample]
        );
    }
}

#[test]
fn adversarial_headers_and_delta_payload_mutations_are_bounded_and_panic_free() {
    let mut encoder = RawBlockEncoder::new();
    let mut seed = 0x8827641;
    for encoding in [
        RawEncoding::Raw34,
        RawEncoding::Packed26,
        RawEncoding::Packed26Delta,
    ] {
        encoder.begin(53, 0, encoding, 1);
        for seq in 0..4 {
            assert!(encoder.push(&valid(&mut seed, seq)));
        }
        let good = encoder.finish(0).to_vec();
        for _ in 0..2048 {
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
}
