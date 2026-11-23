use super::*;
use crate::raw_telemetry::{
    RawBlockEncoder, RawBlockHeader, RawStreamDecoder, decode_raw_block, raw_block_len,
};
use crate::types::{FastTelemetry, HallAngle};
use std::vec::Vec;

fn random(seed: &mut u32) -> u16 {
    *seed ^= *seed << 13;
    *seed ^= *seed >> 17;
    *seed ^= *seed << 5;
    *seed as u16
}

/// A motor turning at `speed` angle counts a sample, with ADC noise of
/// `noise` counts.
fn turning(seed: &mut u32, index: u32, speed: u16, noise: u16) -> FastTelemetry {
    let angle = (index as u16).wrapping_mul(speed);
    let phase = f32::from(angle) * core::f32::consts::TAU / 65_536.0;
    let third = core::f32::consts::TAU / 3.0;
    let mut wave = |offset: f32, scale: f32, centre: f32| {
        let jitter = f32::from(random(seed) % (2 * noise + 1)) - f32::from(noise);
        (centre + scale * (phase + offset).sin() + jitter) as u16
    };
    let currents = [0.0, -third, third].map(|offset| wave(offset, 300.0, 2048.0));
    let voltages = [0.0, -third, third].map(|offset| wave(offset, 500.0, 1000.0));
    let duties = [0.0, -third, third].map(|offset| wave(offset, 1500.0, 2100.0));
    FastTelemetry {
        ia: currents[0],
        ib: currents[1],
        ic: currents[2],
        vbus: 30_000 + random(seed) % 8,
        angle,
        hall_angle: HallAngle::from_encoded(angle & 0xff00),
        vd: -40 + (random(seed) % 5) as i16,
        vq: 900 + (random(seed) % 9) as i16,
        rpm: speed as i16,
        seq: index as u16,
        vphase_adc: voltages,
        pwm_ccr: duties,
        pwm_arr: 4200,
        pwm_enables: 0x3f,
    }
}

fn noise(seed: &mut u32, index: u32) -> FastTelemetry {
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
        seq: index as u16,
        vphase_adc: [word() & 4095, word() & 4095, word() & 4095],
        pwm_ccr: [word(), word(), word()],
        pwm_arr: 4200,
        pwm_enables: word() & 127,
    }
}

fn block(index: u32, frames: &[FastTelemetry]) -> Vec<u8> {
    let mut encoder = RawBlockEncoder::new();
    encoder.begin(7, index, 1);
    for frame in frames {
        assert!(encoder.push(frame));
    }
    encoder.finish(3).to_vec()
}

fn turning_block(seed: &mut u32, index: u32, count: u32, speed: u16) -> Vec<u8> {
    let frames: Vec<_> = (0..count)
        .map(|i| turning(seed, index * 64 + i, speed, 3))
        .collect();
    block(index, &frames)
}

fn compressed(plain: &[u8]) -> Vec<u8> {
    let mut out = [0; PLAIN_BLOCK_BYTES];
    let len = compress(plain, &mut out).expect("the block shrinks");
    out[..len].to_vec()
}

fn expanded(delta: &[u8]) -> Result<Vec<u8>, RawDecodeError> {
    let mut out = [0; PLAIN_BLOCK_BYTES];
    expand(delta, &mut out).map(|len| out[..len].to_vec())
}

#[test]
fn a_turning_motor_shrinks_and_expands_to_the_same_bytes() {
    let mut seed = 0x51f1_5eed;
    for speed in [0, 7, 63, 400, 3000, 20_000] {
        for count in [2, 3, 17, 64] {
            let plain = turning_block(&mut seed, 5, count, speed);
            let delta = compressed(&plain);
            assert!(is_delta(&delta));
            assert_eq!(delta_block_len(&delta), Ok(delta.len()));
            assert_eq!(raw_block_len(&delta), Ok(delta.len()));
            assert_eq!(
                expanded(&delta).as_deref(),
                Ok(&plain[..]),
                "{speed} {count}"
            );
        }
    }
}

#[test]
fn a_slow_motor_at_full_rate_takes_under_eleven_bytes_a_record() {
    let mut seed = 0x0bad_cafe;
    // 63 counts a sample is 120 electrical rad/s sampled at 20 kHz.
    let plain = turning_block(&mut seed, 0, 64, 63);
    let delta = compressed(&plain);
    let per_record = (delta.len() - RAW_HEADER_BYTES) as f32 / 64.0;
    assert!(per_record < 11.0, "{per_record}");
}

#[test]
fn fields_that_wrap_cost_no_more_than_fields_that_do_not() {
    let mut seed = 0x1234_5678;
    let frames = |start: u32, seed: &mut u32| -> Vec<FastTelemetry> {
        (0..64).map(|i| turning(seed, start + i, 400, 0)).collect()
    };
    // The second block's angle passes 65535 on its way.
    let level = compressed(&block(0, &frames(0, &mut seed)));
    let wrapping = compressed(&block(0, &frames(150, &mut seed)));
    assert!(
        wrapping.len() <= level.len() + 8,
        "{} {}",
        wrapping.len(),
        level.len()
    );

    // Every measured field at once steps across its top value and back.
    let mut low = noise(&mut seed, 0);
    (low.ia, low.ib, low.ic, low.vphase_adc) = (0, 4095, 0, [4095, 0, 4095]);
    (low.vbus, low.angle, low.vd, low.vq, low.rpm) = (0, 65_535, -1, 0, i16::MIN);
    low.pwm_ccr = [65_535, 0, 65_535];
    low.hall_angle = HallAngle::from_encoded(0);
    let mut high = low;
    (high.ia, high.ib, high.ic, high.vphase_adc) = (4095, 0, 4095, [0, 4095, 0]);
    (high.vbus, high.angle, high.vd, high.vq, high.rpm) = (65_535, 0, 0, -1, i16::MAX);
    high.pwm_ccr = [0, 65_535, 0];
    high.hall_angle = HallAngle::from_encoded(65_535);
    let frames: Vec<_> = (0..64_u16)
        .map(|i| {
            let mut frame = if i % 2 == 0 { low } else { high };
            frame.seq = i;
            frame
        })
        .collect();
    let plain = block(0, &frames);
    let delta = compressed(&plain);
    // A step of one up or down takes two bits; the enable byte is constant.
    assert_eq!(
        delta.len(),
        RAW_HEADER_BYTES + 28 + 10 + (15 * 2 * 63_usize).div_ceil(8)
    );
    assert_eq!(expanded(&delta).as_deref(), Ok(&plain[..]));
}

#[test]
fn a_constant_block_is_its_first_record_and_the_widths() {
    let mut seed = 9;
    let mut frame = turning(&mut seed, 0, 0, 0);
    let frames: Vec<_> = (0..64_u16)
        .map(|seq| {
            frame.seq = seq;
            frame
        })
        .collect();
    let plain = block(2, &frames);
    let delta = compressed(&plain);
    assert_eq!(delta.len(), RAW_HEADER_BYTES + 28 + 10);
    assert_eq!(expanded(&delta).as_deref(), Ok(&plain[..]));
}

#[test]
fn blocks_that_would_not_shrink_or_are_not_plain_records_are_left_alone() {
    let mut seed = 0x7213_e5b1;
    let mut out = [0; PLAIN_BLOCK_BYTES];
    let frames: Vec<_> = (0..64).map(|i| noise(&mut seed, i)).collect();
    assert_eq!(compress(&block(0, &frames), &mut out), None);
    // One record has nothing to take a difference from.
    assert_eq!(compress(&block(0, &frames[..1]), &mut out), None);

    let mut encoder = RawBlockEncoder::new();
    assert_eq!(compress(encoder.end(7, 4, 640, 0), &mut out), None);
    let mut velocity = [0; RAW_HEADER_BYTES + 2 * crate::velocity_telemetry::VELOCITY_RECORD_BYTES];
    RawBlockHeader::velocity(7, 5, 2, 0, None).write(&mut velocity);
    assert_eq!(compress(&velocity, &mut out), None);

    let plain = turning_block(&mut seed, 0, 64, 63);
    assert_eq!(compress(&plain[..plain.len() - 1], &mut out), None);
    assert_eq!(compress(&plain, &mut out[..RAW_HEADER_BYTES + 40]), None);
    // A Delta28 block is not rewritten again.
    assert_eq!(compress(&compressed(&plain), &mut out), None);
}

#[test]
fn a_damaged_block_is_refused() {
    let mut seed = 0xfeed_f00d;
    let delta = compressed(&turning_block(&mut seed, 0, 64, 63));
    assert!(expanded(&delta).is_ok());

    assert_eq!(
        expanded(&delta[..delta.len() - 1]),
        Err(RawDecodeError::Length)
    );
    let mut longer = delta.clone();
    longer.push(0);
    assert_eq!(expanded(&longer), Err(RawDecodeError::Length));
    assert_eq!(expanded(&delta[..20]), Err(RawDecodeError::Truncated));

    let damaged = |edit: &dyn Fn(&mut Vec<u8>)| {
        let mut bytes = delta.clone();
        edit(&mut bytes);
        expanded(&bytes)
    };
    assert_eq!(damaged(&|b| b[0] ^= 1), Err(RawDecodeError::Header));
    assert_eq!(damaged(&|b| b[4] = 2), Err(RawDecodeError::Header));
    assert_eq!(damaged(&|b| b[6] = 1), Err(RawDecodeError::Header));
    assert_eq!(damaged(&|b| b[7] = 1), Err(RawDecodeError::Header));
    assert_eq!(damaged(&|b| b[7] = 65), Err(RawDecodeError::Header));
    // The stated length no longer follows from the widths.
    assert_eq!(damaged(&|b| b[7] = 63), Err(RawDecodeError::Length));
    assert_eq!(
        damaged(&|b| b[30..32].copy_from_slice(&37_u16.to_le_bytes())),
        Err(RawDecodeError::Header)
    );
    // The first field is 12 bits wide: a width of 13 cannot be.
    assert_eq!(
        damaged(&|b| b[RAW_HEADER_BYTES + 28] = (b[RAW_HEADER_BYTES + 28] & !31) | 13),
        Err(RawDecodeError::Header)
    );
    // A narrower width leaves bytes unaccounted for.
    assert_eq!(
        damaged(&|b| b[RAW_HEADER_BYTES + 28] &= !31),
        Err(RawDecodeError::Length)
    );
}

#[test]
fn padding_bits_must_be_zero() {
    let mut seed = 77;
    for count in 2..=64 {
        let delta = compressed(&turning_block(&mut seed, 0, count, 63));
        let mut widths = 0;
        for field in 0..FIELDS {
            widths += read_bits(
                &delta[RAW_HEADER_BYTES + 28..],
                field * WIDTH_BITS,
                WIDTH_BITS,
            );
        }
        let used = widths as usize * (count as usize - 1) % 8;
        if used == 0 {
            continue;
        }
        let mut bytes = delta.clone();
        *bytes.last_mut().unwrap() |= 0x80;
        assert_eq!(expanded(&bytes), Err(RawDecodeError::Length), "{count}");
    }
}

#[test]
fn an_envelope_keeps_its_blocks_and_shrinks_the_plain_ones() {
    let mut seed = 0x0dd_ba11;
    let records = turning_block(&mut seed, 3, 64, 63);
    let mut velocity =
        [0; RAW_HEADER_BYTES + 11 * crate::velocity_telemetry::VELOCITY_RECORD_BYTES];
    RawBlockHeader::velocity(7, 4, 11, 0, None).write(&mut velocity);
    let mut envelope = records.clone();
    envelope.extend_from_slice(&velocity);

    let mut out = [0; crate::raw_telemetry::RAW_MAX_BLOCK_BYTES];
    let len = compress_envelope(&envelope, &mut out).unwrap();
    let delta = compressed(&records);
    assert_eq!(out[..delta.len()], delta[..]);
    assert_eq!(out[delta.len()..len], velocity[..]);
    assert_eq!(
        crate::raw_telemetry::raw_block_header(&out[..len])
            .unwrap()
            .payload_bytes(),
        64 * 28
    );

    // What does not shrink goes through unchanged, and a cut envelope fails.
    let frames: Vec<_> = (0..64).map(|i| noise(&mut seed, i)).collect();
    let plain = block(0, &frames);
    assert_eq!(compress_envelope(&plain, &mut out), Ok(plain.len()));
    assert_eq!(out[..plain.len()], plain[..]);
    assert_eq!(
        compress_envelope(&envelope[..envelope.len() - 1], &mut out),
        Err(RawDecodeError::Truncated)
    );
    assert_eq!(
        compress_envelope(&plain, &mut out[..plain.len() - 1]),
        Err(RawDecodeError::Length)
    );
}

#[test]
fn decoders_take_both_layouts_in_one_stream() {
    let mut seed = 0xabcd_ef01;
    let mut stream = Vec::new();
    let mut expected = Vec::new();
    for index in 0..40_u32 {
        let plain = if index % 5 == 4 {
            let frames: Vec<_> = (0..64).map(|i| noise(&mut seed, index * 64 + i)).collect();
            block(index, &frames)
        } else {
            turning_block(&mut seed, index, 64, 63)
        };
        let decoded = decode_raw_block(&plain).unwrap();
        let mut out = [0; PLAIN_BLOCK_BYTES];
        match compress(&plain, &mut out) {
            Some(len) => {
                let again = decode_raw_block(&out[..len]).unwrap();
                assert_eq!(again.header, decoded.header);
                assert_eq!(again.samples, decoded.samples);
                stream.extend_from_slice(&out[..len]);
            }
            None => stream.extend_from_slice(&plain),
        }
        expected.push(decoded);
    }
    assert!(stream.len() < 40 * (RAW_HEADER_BYTES + 64 * 28) / 2);

    for step in [1, 7, 31, 500, stream.len()] {
        let mut decoder = RawStreamDecoder::new();
        let mut blocks = Vec::new();
        for part in stream.chunks(step) {
            decoder.push(part, |block| blocks.push(block)).unwrap();
        }
        assert_eq!(blocks.len(), expected.len());
        for (block, wanted) in blocks.iter().zip(&expected) {
            assert_eq!(block.header, wanted.header);
            assert_eq!(block.samples, wanted.samples);
        }
    }
}

/// The format written out field by field with the general bit helpers, as
/// the module documentation states it.
fn reference(plain: &[u8]) -> Option<Vec<u8>> {
    let count = usize::from(plain[7]);
    let records = &plain[RAW_HEADER_BYTES..];
    let field = |record: usize, field: usize| {
        let bits = usize::from(FIELD_BITS[field]);
        read_bits(records, record * RECORD_BYTES * 8 + FIELD_AT[field], bits)
    };
    let code = |record: usize, index: usize| {
        let size = 1_u32 << FIELD_BITS[index];
        let delta = (size + field(record, index) - field(record - 1, index)) % size;
        if delta < size / 2 {
            delta * 2
        } else {
            (size - 1 - delta) * 2 + 1
        }
    };
    let widths: Vec<usize> = (0..FIELDS)
        .map(|index| {
            let widest = (1..count).map(|record| code(record, index)).max().unwrap();
            (u32::BITS - widest.leading_zeros()) as usize
        })
        .collect();
    let bits: usize = widths.iter().map(|width| width * (count - 1)).sum();
    let payload = RECORD_BYTES + WIDTH_BYTES + bits.div_ceil(8);
    if payload >= count * RECORD_BYTES {
        return None;
    }
    let mut out = std::vec![0; RAW_HEADER_BYTES + payload];
    out[..RAW_HEADER_BYTES].copy_from_slice(&plain[..RAW_HEADER_BYTES]);
    out[5] = DELTA28_TAG;
    out[30..32].copy_from_slice(&(payload as u16).to_le_bytes());
    let body = &mut out[RAW_HEADER_BYTES..];
    body[..RECORD_BYTES].copy_from_slice(&records[..RECORD_BYTES]);
    let mut cursor = RECORD_BYTES * 8;
    for &width in &widths {
        write_bits(body, cursor, width as u32);
        cursor += WIDTH_BITS;
    }
    for (index, &width) in widths.iter().enumerate() {
        for record in 1..count {
            write_bits(body, cursor, code(record, index));
            cursor += width;
        }
    }
    Some(out)
}

#[test]
fn the_compressor_writes_the_documented_format_bit_for_bit() {
    let mut seed = 0x51ed_270b;
    let mut out = [0xa5; PLAIN_BLOCK_BYTES];
    let mut blocks = Vec::new();
    for speed in [0, 1, 7, 113, 2048, 20_000, 32_768, 65_535] {
        for count in [2, 3, 17, 63, 64] {
            blocks.push(turning_block(&mut seed, 5, count, speed));
        }
    }
    // Every field takes its full width, and the block still shrinks: the
    // records alternate between two random ones bar a field or two.
    for held in 0..FIELDS {
        let mut frames = [noise(&mut seed, 0), noise(&mut seed, 1)];
        frames[1].pwm_enables = frames[0].pwm_enables;
        if held < 3 {
            frames[1].vphase_adc[held] = frames[0].vphase_adc[held];
        } else {
            frames[1].pwm_ccr[held % 3] = frames[0].pwm_ccr[held % 3];
        }
        let frames: Vec<_> = (0..64)
            .map(|index| FastTelemetry {
                seq: index as u16,
                ..frames[index % 2]
            })
            .collect();
        blocks.push(block(9, &frames));
    }
    let noisy: Vec<_> = (0..64).map(|index| noise(&mut seed, index)).collect();
    blocks.push(block(9, &noisy));
    let mut shrunk = 0;
    for plain in &blocks {
        let expected = reference(plain);
        let written = compress(plain, &mut out).map(|len| out[..len].to_vec());
        assert_eq!(written, expected);
        shrunk += usize::from(written.is_some());
    }
    assert!(shrunk > blocks.len() / 2 && shrunk < blocks.len());
}
