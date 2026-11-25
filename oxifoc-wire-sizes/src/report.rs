//! Markdown tables of the measurements: the condensed set a reader needs by
//! default, and with `--full` every table the measurement produces.

use std::{
    collections::BTreeSet,
    fmt::{Result, Write},
};

use ergot::{
    FrameKind, ProtocolError, TrafficClass, nash::NameHash, wire_frames::encode_frame_err,
};
use postcard::ser_flavors::StdVec;

use crate::{
    catalog::{Catalog, Group, Message},
    encoded_len,
    frames::{BRIDGE_CAN_MTU, Bus, Fragments, header_savings},
    header::{
        Appendix, HeaderFields, HeaderShape, NetRange, RESPONSE, breakdown_shapes, header_len,
    },
    raw_blocks::{Lengths, RawBlocks, ble_block_limit, ble_block_records},
};

/// Where the raw blocks came from, and what they measured.
pub struct Fixture<'a> {
    pub name: &'a str,
    pub blocks: Option<&'a RawBlocks>,
}

pub fn condensed(out: &mut String, catalog: &Catalog) -> Result {
    header_summary(out)?;
    writeln!(
        out,
        "Columns: `hdr` is the header shape code from the table above and its \
         length; classic is classic CAN (8-byte frames), FD is CAN FD (64-byte \
         frames), each as `frames (bytes to cut for one frame fewer)`; `FD last` \
         is the data in the last FD frame and, when padded, the frame length; \
         `known port` is the same request sent to a known port: total, classic / \
         FD frames.\n"
    )?;
    for group in Group::ALL {
        writeln!(out, "## Messages: {}\n", group.label())?;
        writeln!(
            out,
            "| message | part | variant | value | body | hdr | total | classic | FD | FD last | known port |"
        )?;
        writeln!(out, "|---|---|---|---|---|---|---|---|---|---|---|")?;
        for message in in_group(catalog, group) {
            let frame = message.encode_as_sent();
            let total = frame.len();
            let shape = &message.socket.shape;
            let classic = Fragments::of(&frame, Bus::Classic);
            let fd = Fragments::of(&frame, Bus::Fd);
            let known_port = message.socket.known_port.map_or_else(
                || "-".to_owned(),
                |shape| {
                    let frame = message.encode(&shape);
                    format!(
                        "{}: {}/{}",
                        frame.len(),
                        frames_or_too_large(&Fragments::of(&frame, Bus::Classic)),
                        frames_or_too_large(&Fragments::of(&frame, Bus::Fd))
                    )
                },
            );
            writeln!(
                out,
                "| `{}` | {} | {} | {} | {} | {}{} | {}{} | {} | {} | {} | {} |",
                message.socket.path,
                message.socket.part,
                message.variant,
                message.value,
                message.body_len,
                shape.code,
                header_len(&message.header(shape)),
                total,
                if total > BRIDGE_CAN_MTU {
                    " **>512**"
                } else {
                    ""
                },
                frames_and_save(&classic, total, Bus::Classic),
                frames_and_save(&fd, total, Bus::Fd),
                last_frame(&fd),
                known_port,
            )?;
        }
        writeln!(out)?;
    }
    over_mtu(out, catalog)?;
    if !catalog.not_measured.is_empty() {
        not_measured(out, catalog)?;
    }
    Ok(())
}

pub fn full(out: &mut String, catalog: &Catalog, fixture: &Fixture<'_>) -> Result {
    if let Some(blocks) = fixture.blocks {
        fixture_summary(out, fixture.name, blocks)?;
    }
    header_breakdown(out)?;
    name_hashes(out, catalog)?;
    for group in Group::ALL {
        messages(out, catalog, group)?;
    }
    protocol_errors(out)?;
    over_mtu(out, catalog)?;
    near_boundary(out, catalog)?;
    larger_nets(out, catalog)?;
    writeln!(
        out,
        "## Capacities the max values were built with (found by deserializing)\n"
    )?;
    for line in &catalog.capacities {
        writeln!(out, "- {line}")?;
    }
    writeln!(out)?;
    not_measured(out, catalog)
}

fn in_group(catalog: &Catalog, group: Group) -> impl Iterator<Item = &Message> {
    catalog
        .messages
        .iter()
        .filter(move |m| m.socket.group == group)
}

fn sample_header(shape: &HeaderShape) -> HeaderFields {
    HeaderFields::of(&shape.header([0xff; 8], FrameKind::ENDPOINT_REQ, TrafficClass::Normal))
}

/// One row per header shape at the bench nets, with the totals larger nets
/// would give.
fn header_summary(out: &mut String) -> Result {
    writeln!(
        out,
        "## Header shapes (bytes; each field encoded alone, the sum checked against ergot's header encoder)\n"
    )?;
    writeln!(
        out,
        "| code | case | addresses | src | dst | meta | key | name hash | total | nets 32..4095 | nets >= 4096 |"
    )?;
    writeln!(out, "|---|---|---|---|---|---|---|---|---|---|---|")?;
    for shape in breakdown_shapes() {
        let f = sample_header(&shape);
        let larger = |range| {
            if shape.is_link_local() {
                "-".to_owned()
            } else {
                sample_header(&shape.in_range(range)).total.to_string()
            }
        };
        writeln!(
            out,
            "| {} | {} | {} -> {} | {} | {} | {} | {} | {} | **{}** | {} | {} |",
            shape.code,
            shape.label,
            shape.src,
            shape.dst,
            f.src,
            f.dst,
            f.meta,
            f.key,
            f.name_hash,
            f.total,
            larger(NetRange::Mid),
            larger(NetRange::Large),
        )?;
    }
    writeln!(out)
}

fn header_breakdown(out: &mut String) -> Result {
    writeln!(
        out,
        "## Header breakdown (measured: each field encoded alone, sum checked against encode_frame_hdr)\n"
    )?;
    writeln!(
        out,
        "| case | addresses | src | dst | meta (kind/class/ttl) | key | name hash | total |"
    )?;
    writeln!(out, "|---|---|---|---|---|---|---|---|")?;
    for shape in breakdown_shapes() {
        let ranges: &[NetRange] = if shape.is_link_local() {
            &[NetRange::Small]
        } else {
            &NetRange::ALL
        };
        for &range in ranges {
            let moved = shape.in_range(range);
            let f = sample_header(&moved);
            writeln!(
                out,
                "| {} | {} -> {} ({}) | {} | {} | {} | {} | {} | **{}** |",
                moved.label,
                moved.src,
                moved.dst,
                if shape.is_link_local() {
                    "link-local"
                } else {
                    range.label()
                },
                f.src,
                f.dst,
                f.meta,
                f.key,
                f.name_hash,
                f.total,
            )?;
        }
    }
    writeln!(out)
}

/// The names the senders put in the any/all appendix.
fn name_hashes(out: &mut String, catalog: &Catalog) -> Result {
    writeln!(
        out,
        "Name hashes the senders put in the any/all appendix (measured varint size; `None` encodes as 0, 1 byte):\n"
    )?;
    writeln!(out, "| name | hash | bytes |")?;
    writeln!(out, "|---|---|---|")?;
    let mut seen = BTreeSet::new();
    for message in &catalog.messages {
        if let Appendix::Key { name: Some(name) } = message.socket.shape.appendix
            && seen.insert(name)
        {
            let hash = NameHash::new(name).to_u32();
            writeln!(out, "| `{name}` | 0x{hash:08x} | {} |", encoded_len(&hash))?;
        }
    }
    writeln!(out)
}

fn fixture_summary(out: &mut String, name: &str, blocks: &RawBlocks) -> Result {
    writeln!(out, "## Raw telemetry blocks from the fixture (measured)\n")?;
    writeln!(out, "fixture: `{name}`\n")?;
    let distinct = |lengths: &[usize]| lengths.iter().copied().collect::<BTreeSet<_>>();
    writeln!(
        out,
        "- Packed28, 64 records: {} blocks of {:?} bytes; Delta28 of them: {}",
        blocks.packed_full.len(),
        distinct(&blocks.packed_full),
        Lengths(&blocks.delta_full)
    )?;
    writeln!(
        out,
        "- Re-cut at the BLE limit ({} bytes, {} records): {} blocks, Packed28 {:?} bytes, Delta28 {}",
        ble_block_limit(),
        ble_block_records(),
        blocks.packed_ble.len(),
        distinct(&blocks.packed_ble),
        Lengths(&blocks.delta_ble)
    )?;
    writeln!(out, "- END block: {} bytes\n", blocks.end.len())
}

fn messages(out: &mut String, catalog: &Catalog, group: Group) -> Result {
    writeln!(
        out,
        "## Messages: {} (measured; header as sent on the bench topology, nets < 32)\n",
        group.label()
    )?;
    writeln!(
        out,
        "C8 = classic CAN, max_payload 8; FD = CAN FD, max_payload 64, `n (last)` or `n (data->padded)`; save = fewest bytes to cut for one frame fewer. `alt` = the same request sent to a known port (same addresses, no appendix): total bytes, C8 / FD frames.\n"
    )?;
    writeln!(
        out,
        "| message | part | class | variant | value | body | header | hdr | total | C8 frames | C8 save | FD frames | FD save | FD wire bytes | alt: total: C8 / FD |"
    )?;
    writeln!(
        out,
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"
    )?;
    for message in in_group(catalog, group) {
        let shape = &message.socket.shape;
        let frame = message.encode_as_sent();
        let total = frame.len();
        let classic = Fragments::of(&frame, Bus::Classic);
        let fd = Fragments::of(&frame, Bus::Fd);
        let alt = message.socket.known_port.map_or_else(
            || "-".to_owned(),
            |shape| {
                let frame = message.encode(&shape);
                format!(
                    "{}: {} / {}",
                    frame.len(),
                    frames_or_too_large(&Fragments::of(&frame, Bus::Classic)),
                    frames_or_too_large(&Fragments::of(&frame, Bus::Fd))
                )
            },
        );
        let wire_bytes = match fd {
            Fragments::TooLarge => 0,
            Fragments::Fit { wire_bytes, .. } => wire_bytes,
        };
        writeln!(
            out,
            "| `{}` | {} | {} | {} | {} | {} | {} | {} | {}{} | {} | {} | {} | {} | {} | {} |",
            message.socket.path,
            message.socket.part,
            class_name(message.socket.class),
            message.variant,
            message.value,
            message.body_len,
            shape.label,
            header_len(&message.header(shape)),
            total,
            if total > BRIDGE_CAN_MTU {
                " **>MTU 512**"
            } else {
                ""
            },
            frames_with_last(&classic, Bus::Classic),
            save(&classic, total, Bus::Classic),
            frames_with_last(&fd, Bus::Fd),
            save(&fd, total, Bus::Fd),
            wire_bytes,
            alt,
        )?;
    }
    writeln!(out)
}

fn protocol_errors(out: &mut String) -> Result {
    writeln!(
        out,
        "## Protocol error frames (measured, `encode_frame_err`, response-style header)\n"
    )?;
    writeln!(out, "| error | total | C8 | FD |")?;
    writeln!(out, "|---|---|---|---|")?;
    let header = RESPONSE.header([0; 8], FrameKind::PROTOCOL_ERROR, TrafficClass::Normal);
    for (name, error) in [
        ("SseNoSpace", ProtocolError::SseNoSpace),
        ("NSSE_NO_ROUTE (NsseNoRoute)", ProtocolError::NsseNoRoute),
    ] {
        let frame =
            encode_frame_err(StdVec::new(), &header, error).expect("a frame encodes into a Vec");
        writeln!(
            out,
            "| {name} | {} | {} | {} |",
            frame.len(),
            frames_or_dash(&Fragments::of(&frame, Bus::Classic)),
            frames_or_dash(&Fragments::of(&frame, Bus::Fd)),
        )?;
    }
    writeln!(out)
}

fn over_mtu(out: &mut String, catalog: &Catalog) -> Result {
    writeln!(
        out,
        "## Over {BRIDGE_CAN_MTU} bytes encoded (cannot cross classic CAN; over the bridges' CAN FD MTU of {BRIDGE_CAN_MTU})\n"
    )?;
    for message in &catalog.messages {
        let frame = message.encode_as_sent();
        if frame.len() > BRIDGE_CAN_MTU {
            writeln!(
                out,
                "- `{}` {} {} {}: {} bytes (FD {} frames)",
                message.socket.path,
                message.socket.part,
                message.variant,
                message.value,
                frame.len(),
                frames_or_dash(&Fragments::of(&frame, Bus::Fd)),
            )?;
        }
    }
    writeln!(out)
}

fn near_boundary(out: &mut String, catalog: &Catalog) -> Result {
    writeln!(
        out,
        "## Near a frame boundary (measured with the Fragmenter on a shortened message)\n"
    )?;
    writeln!(
        out,
        "Header savings of 2, 4 or 6 bytes that remove a frame; `-> 1` when the message would become a single frame.\n"
    )?;
    writeln!(
        out,
        "| message | part | variant | value | total | C8 frames | C8 save | C8 flag | FD frames | FD save | FD flag |"
    )?;
    writeln!(out, "|---|---|---|---|---|---|---|---|---|---|---|")?;
    for message in &catalog.messages {
        let frame = message.encode_as_sent();
        let total = frame.len();
        let classic = Fragments::of(&frame, Bus::Classic);
        let fd = Fragments::of(&frame, Bus::Fd);
        let classic_flag = header_savings(total, &classic, Bus::Classic);
        let fd_flag = header_savings(total, &fd, Bus::Fd);
        if classic_flag.is_empty() && fd_flag.is_empty() {
            continue;
        }
        writeln!(
            out,
            "| `{}` | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |",
            message.socket.path,
            message.socket.part,
            message.variant,
            message.value,
            total,
            frames_or_dash(&classic),
            save(&classic, total, Bus::Classic),
            classic_flag,
            frames_or_dash(&fd),
            save(&fd, total, Bus::Fd),
            fd_flag,
        )?;
    }
    writeln!(out)
}

fn larger_nets(out: &mut String, catalog: &Catalog) -> Result {
    writeln!(
        out,
        "## Larger nets (measured by re-encoding with nets >= 32 and >= 4096)\n"
    )?;
    writeln!(
        out,
        "Messages whose frame count changes against the bench header (nets < 32).\n"
    )?;
    writeln!(
        out,
        "| message | part | variant | value | total small/mid/large | C8 frames small/mid/large | FD frames small/mid/large |"
    )?;
    writeln!(out, "|---|---|---|---|---|---|---|")?;
    for message in &catalog.messages {
        let frames: Vec<Vec<u8>> = NetRange::ALL
            .iter()
            .map(|&range| message.encode(&message.socket.shape.in_range(range)))
            .collect();
        let counts = |bus| -> Vec<Option<usize>> {
            frames
                .iter()
                .map(|frame| Fragments::of(frame, bus).frames())
                .collect()
        };
        let (classic, fd) = (counts(Bus::Classic), counts(Bus::Fd));
        let constant = |counts: &[Option<usize>]| counts.windows(2).all(|w| w[0] == w[1]);
        if constant(&classic) && constant(&fd) {
            continue;
        }
        let join = |items: Vec<String>| items.join("/");
        let counts_text = |counts: &[Option<usize>]| {
            join(
                counts
                    .iter()
                    .map(|n| n.map_or_else(|| "-".to_owned(), |n| n.to_string()))
                    .collect(),
            )
        };
        writeln!(
            out,
            "| `{}` | {} | {} | {} | {} | {} | {} |",
            message.socket.path,
            message.socket.part,
            message.variant,
            message.value,
            join(frames.iter().map(|f| f.len().to_string()).collect()),
            counts_text(&classic),
            counts_text(&fd),
        )?;
    }
    writeln!(out)
}

fn not_measured(out: &mut String, catalog: &Catalog) -> Result {
    writeln!(out, "## Not measured\n")?;
    if catalog.not_measured.is_empty() {
        writeln!(out, "- (none)")?;
    }
    for line in &catalog.not_measured {
        writeln!(out, "- {line}")?;
    }
    Ok(())
}

fn class_name(class: TrafficClass) -> &'static str {
    match class {
        TrafficClass::Control => "Control",
        TrafficClass::Normal => "Normal",
        TrafficClass::Bulk => "Bulk",
        TrafficClass::Background => "Background",
    }
}

fn frames_or_dash(fragments: &Fragments) -> String {
    fragments
        .frames()
        .map_or_else(|| "-".to_owned(), |n| n.to_string())
}

fn frames_or_too_large(fragments: &Fragments) -> String {
    fragments
        .frames()
        .map_or_else(|| "TOO LARGE".to_owned(), |n| n.to_string())
}

/// `n`, on CAN FD with the last frame: `n (last)` or `n (data->padded)`.
fn frames_with_last(fragments: &Fragments, bus: Bus) -> String {
    match (fragments, bus) {
        (Fragments::TooLarge, _) => format!("TOO LARGE (>{})", bus.max_message_len()),
        (Fragments::Fit { frames, .. }, Bus::Classic) => frames.to_string(),
        (Fragments::Fit { frames, .. }, Bus::Fd) => format!("{frames} ({})", last_frame(fragments)),
    }
}

/// `data` or `data->padded`.
fn last_frame(fragments: &Fragments) -> String {
    match fragments {
        Fragments::TooLarge => "-".to_owned(),
        Fragments::Fit {
            last_data,
            last_len,
            ..
        } if last_data == last_len => last_data.to_string(),
        Fragments::Fit {
            last_data,
            last_len,
            ..
        } => format!("{last_data}->{last_len}"),
    }
}

fn save(fragments: &Fragments, total: usize, bus: Bus) -> String {
    match fragments {
        Fragments::TooLarge => format!("-{} to fit", total - bus.max_message_len()),
        Fragments::Fit { to_save: None, .. } => "1 frame".to_owned(),
        Fragments::Fit {
            to_save: Some(bytes),
            ..
        } => bytes.to_string(),
    }
}

/// `n (bytes to save)`, `1`, or how far a message is over the bus's limit.
fn frames_and_save(fragments: &Fragments, total: usize, bus: Bus) -> String {
    match fragments {
        Fragments::TooLarge => format!(
            "too large ({} over {})",
            total - bus.max_message_len(),
            bus.max_message_len()
        ),
        Fragments::Fit {
            frames,
            to_save: None,
            ..
        } => frames.to_string(),
        Fragments::Fit {
            frames,
            to_save: Some(bytes),
            ..
        } => format!("{frames} ({bytes})"),
    }
}
