use super::*;
use crate::raw_telemetry::{
    RAW_HEADER_BYTES, RAW_MAX_BLOCK_BYTES, RawBlockEncoder, RawBlockHeader, decode_raw_block,
};
use crate::types::HallAngle;
use crate::velocity_telemetry::VELOCITY_RECORD_BYTES;

fn roundtrip(bytes: &[u8]) {
    let mut wire = [0; 2304];
    let encoded = postcard::to_slice(&RawTelemetryBlockRef { data: bytes }, &mut wire)
        .expect("borrowed raw block fits the Ergot MTU");
    assert!(encoded.len() + ergot::wire_frames::MAX_HDR_ENCODED_SIZE <= 2304);
    let decoded: RawTelemetryBlock = postcard::from_bytes(encoded)
        .expect("borrowed and owned raw topic representations share a wire format");
    assert_eq!(decoded.data.as_slice(), bytes);
    let original = decode_raw_block(bytes).expect("encoder emits a valid raw block");
    let received = decode_raw_block(&decoded.data).expect("Ergot preserves the raw block bytes");
    assert_eq!(original.header, received.header);
    for (original, received) in original.samples.iter().zip(&received.samples) {
        assert_eq!(bytemuck::bytes_of(original), bytemuck::bytes_of(received));
    }
}

#[test]
fn raw_topic_borrowed_wire_matches_owned_and_keeps_blocks_end_and_companion() {
    use ergot::traits::Topic;
    assert_eq!(
        RawTelemetryBlockRefTopic::TOPIC_KEY,
        RawTelemetryBlockTopic::TOPIC_KEY
    );
    let mut encoder = RawBlockEncoder::new();
    encoder.begin(42, 0, 1);
    for seq in 0..64 {
        let sample = FastTelemetry {
            seq,
            ia: 1000 + seq,
            ib: 2000 - seq,
            ic: 3000,
            vphase_adc: [1800, 1900, 2000],
            vbus: 12000,
            pwm_ccr: [seq, 1200, 3000],
            pwm_arr: 4195,
            hall_angle: HallAngle::from_encoded(0x8000 + seq),
            ..FastTelemetry::default()
        };
        assert!(encoder.push(&sample));
    }
    // A full native block, then the largest velocity companion it leaves room for.
    let mut envelope = encoder.finish(0).to_vec();
    let companion =
        (RAW_MAX_BLOCK_BYTES - envelope.len() - RAW_HEADER_BYTES) / VELOCITY_RECORD_BYTES;
    assert!(companion >= 11);
    let mut block = vec![0; RAW_HEADER_BYTES + companion * VELOCITY_RECORD_BYTES];
    RawBlockHeader::velocity(42, 1, companion as u8, 0, None).write(&mut block);
    envelope.extend(block);
    roundtrip(encoder.finish(0));
    roundtrip(encoder.end(42, 2, 64, 0));
    let mut wire = [0; 2304];
    let encoded = postcard::to_slice(&RawTelemetryBlockRef { data: &envelope }, &mut wire).unwrap();
    assert!(encoded.len() + ergot::wire_frames::MAX_HDR_ENCODED_SIZE <= 2304);
    let decoded: RawTelemetryBlock = postcard::from_bytes(encoded).unwrap();
    assert_eq!(decoded.data.as_slice(), envelope);
}

#[test]
fn raw_topic_capacity_holds_a_full_envelope_and_rejects_oversize() {
    let payload = [0xa5; RAW_MAX_BLOCK_BYTES + 1];
    let mut wire = [0; 2304];
    let encoded = postcard::to_slice(
        &RawTelemetryBlockRef {
            data: &payload[..RAW_MAX_BLOCK_BYTES],
        },
        &mut wire,
    )
    .unwrap();
    assert_eq!(encoded.len(), RAW_MAX_BLOCK_BYTES + 2);
    let decoded: RawTelemetryBlock = postcard::from_bytes(encoded).unwrap();
    assert_eq!(decoded.data.len(), RAW_MAX_BLOCK_BYTES);
    let encoded = postcard::to_slice(&RawTelemetryBlockRef { data: &payload }, &mut wire).unwrap();
    assert!(postcard::from_bytes::<RawTelemetryBlock>(encoded).is_err());
}
