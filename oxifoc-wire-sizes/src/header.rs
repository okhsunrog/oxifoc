//! The ergot headers the messages are measured with: the addresses of the
//! bench topology and whether a header carries the any/all appendix (key and
//! name hash).

use ergot::{
    Address, AnyAllAppendix, DEFAULT_TTL, FrameKind, Header, Key, TrafficClass, nash::NameHash,
    wire_frames::encode_frame_hdr,
};
use postcard::{
    Serializer,
    ser_flavors::{Flavor as _, StdVec},
};

use crate::encoded_len;

const fn addr(network_id: u16, node_id: u8, port_id: u8) -> Address {
    Address {
        network_id,
        node_id,
        port_id,
    }
}

// The bench topology: a host on the root bridge's USB segment (net 1,
// node 2), the CAN bus (net 3: root bridge node 1, second bridge node 19),
// the controller on the second bridge's SPI segment (net 8, node 2). All
// nets are below 32, so every address is the smallest ergot encodes.
const HOST: Address = addr(1, 2, 7);
const CONTROLLER_ANY: Address = addr(8, 2, 0);
const CONTROLLER_PORT: Address = addr(8, 2, 5);
const BRIDGE_ANY: Address = addr(3, 19, 0);
const BUS_BROADCAST: Address = addr(3, 255, 255);
/// The port a socket query resolves to; requests addressed to it drop the
/// appendix.
pub const KNOWN_PORT: u8 = 5;

#[derive(Clone, Copy)]
pub enum Appendix {
    None,
    /// The key, and the name hash when the sender asks for a named socket.
    Key {
        name: Option<&'static str>,
    },
}

/// How a message is addressed. `code` is the short tag the condensed table
/// puts in front of the header length; shapes of the same length share one.
#[derive(Clone, Copy)]
pub struct HeaderShape {
    pub code: char,
    pub label: &'static str,
    pub src: Address,
    pub dst: Address,
    pub appendix: Appendix,
}

impl HeaderShape {
    pub fn header(&self, key: [u8; 8], kind: FrameKind, class: TrafficClass) -> Header {
        let any_all = match self.appendix {
            Appendix::None => None,
            Appendix::Key { name } => Some(AnyAllAppendix {
                key: Key(key),
                nash: name.map(NameHash::new),
            }),
        };
        Header {
            src: self.src,
            dst: self.dst,
            any_all,
            kind,
            class,
            ttl: DEFAULT_TTL,
        }
    }

    /// The same request sent to the port a socket query found: same
    /// addresses, no appendix. `None` for a shape without an appendix.
    pub fn sent_to_known_port(self) -> Option<Self> {
        matches!(self.appendix, Appendix::Key { .. }).then_some(Self {
            code: 'K',
            label: "same, to a known port",
            dst: Address {
                port_id: KNOWN_PORT,
                ..self.dst
            },
            appendix: Appendix::None,
            ..self
        })
    }

    /// The same shape with every routed (non-zero) net moved into `range`.
    pub fn in_range(self, range: NetRange) -> Self {
        let shift = |a: Address| {
            if a.network_id == 0 {
                a
            } else {
                Address {
                    network_id: a.network_id + range.offset(),
                    ..a
                }
            }
        };
        Self {
            src: shift(self.src),
            dst: shift(self.dst),
            ..self
        }
    }

    /// Link-local: both nets 0, so moving the nets changes nothing.
    pub fn is_link_local(&self) -> bool {
        self.src.network_id == 0
    }
}

/// Net ids grow the address varint at 32 and at 4096.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum NetRange {
    /// As on the bench: below 32.
    Small,
    Mid,
    Large,
}

impl NetRange {
    pub const ALL: [Self; 3] = [Self::Small, Self::Mid, Self::Large];

    fn offset(self) -> u16 {
        match self {
            Self::Small => 0,
            Self::Mid => 32,
            Self::Large => 4096,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Small => "nets 1..31",
            Self::Mid => "nets 32..4095",
            Self::Large => "nets >= 4096",
        }
    }
}

pub fn named_request(name: &'static str) -> HeaderShape {
    HeaderShape {
        code: 'N',
        label: "req port 0 + key + name hash",
        src: HOST,
        dst: CONTROLLER_ANY,
        appendix: Appendix::Key { name: Some(name) },
    }
}

pub const ANONYMOUS_REQUEST: HeaderShape = HeaderShape {
    code: 'A',
    label: "req port 0 + key, no name",
    src: HOST,
    dst: BRIDGE_ANY,
    appendix: Appendix::Key { name: None },
};
pub const KNOWN_PORT_REQUEST: HeaderShape = HeaderShape {
    code: 'K',
    label: "req to known port",
    src: HOST,
    dst: CONTROLLER_PORT,
    appendix: Appendix::None,
};
pub const RESPONSE: HeaderShape = HeaderShape {
    code: 'R',
    label: "response",
    src: CONTROLLER_PORT,
    dst: HOST,
    appendix: Appendix::None,
};
pub const TOPIC_BROADCAST: HeaderShape = HeaderShape {
    code: 'B',
    label: "topic broadcast",
    src: CONTROLLER_ANY,
    dst: BUS_BROADCAST,
    appendix: Appendix::Key { name: None },
};
pub const TOPIC_UNICAST_ANY_PORT: HeaderShape = HeaderShape {
    code: 'U',
    label: "topic unicast port 0 + key",
    src: BRIDGE_ANY,
    dst: addr(1, 2, 0),
    appendix: Appendix::Key { name: None },
};
pub const TOPIC_UNICAST_KNOWN_PORT: HeaderShape = HeaderShape {
    code: 'P',
    label: "topic unicast to known port",
    src: CONTROLLER_ANY,
    dst: HOST,
    appendix: Appendix::None,
};
/// ergot's bus address claim, sent link-local from the candidate.
pub const CLAIM_REQUEST: HeaderShape = HeaderShape {
    code: 'L',
    label: "claim req, link-local (0.19 -> 0.1:0) + key",
    src: addr(0, 19, 9),
    dst: addr(0, 1, 0),
    appendix: Appendix::Key { name: None },
};
/// A candidate node id of 64 or more takes a second varint byte.
pub const CLAIM_REQUEST_HIGH_NODE: HeaderShape = HeaderShape {
    code: 'L',
    label: "claim req, link-local, candidate >= 64 (0.200) + key",
    src: addr(0, 200, 9),
    dst: addr(0, 1, 0),
    appendix: Appendix::Key { name: None },
};
pub const BUS_RESPONSE: HeaderShape = HeaderShape {
    code: 'R',
    label: "response on the bus (3.1 -> 3.19)",
    src: addr(3, 1, 4),
    dst: addr(3, 19, 9),
    appendix: Appendix::None,
};
pub const BUS_KNOWN_PORT_REQUEST: HeaderShape = HeaderShape {
    code: 'K',
    label: "req to known port on the bus (3.19 -> 3.1:p)",
    src: addr(3, 19, 9),
    dst: addr(3, 1, 4),
    appendix: Appendix::None,
};
pub const BUS_ANONYMOUS_REQUEST: HeaderShape = HeaderShape {
    code: 'A',
    label: "req port 0 + key on the bus (3.19 -> 3.1:0)",
    src: addr(3, 19, 9),
    dst: addr(3, 1, 0),
    appendix: Appendix::Key { name: None },
};
pub const SOCKET_QUERY: HeaderShape = HeaderShape {
    code: 'B',
    label: "topic broadcast with src port (re-flooded on the bus)",
    src: HOST,
    dst: BUS_BROADCAST,
    appendix: Appendix::Key { name: None },
};

/// Every shape, in the order of the header breakdown table.
pub fn breakdown_shapes() -> [HeaderShape; 12] {
    [
        KNOWN_PORT_REQUEST,
        ANONYMOUS_REQUEST,
        named_request("motor"),
        RESPONSE,
        TOPIC_BROADCAST,
        TOPIC_UNICAST_ANY_PORT,
        TOPIC_UNICAST_KNOWN_PORT,
        CLAIM_REQUEST,
        CLAIM_REQUEST_HIGH_NODE,
        BUS_RESPONSE,
        BUS_KNOWN_PORT_REQUEST,
        BUS_ANONYMOUS_REQUEST,
    ]
}

pub fn header_len(header: &Header) -> usize {
    let mut ser = Serializer {
        output: StdVec::new(),
    };
    encode_frame_hdr(&mut ser, header).expect("a header encodes into a Vec");
    ser.output.finalize().expect("a Vec flavor finalizes").len()
}

/// Bytes of each header field, each encoded on its own.
pub struct HeaderFields {
    pub src: usize,
    pub dst: usize,
    /// Kind, class and TTL, packed into one `u8` (not a varint).
    pub meta: usize,
    pub key: usize,
    pub name_hash: usize,
    pub total: usize,
}

impl HeaderFields {
    /// Panics if the fields do not add up to the header ergot encodes, which
    /// would mean ergot's header layout changed under this breakdown.
    pub fn of(header: &Header) -> Self {
        let (key, name_hash) = match &header.any_all {
            None => (0, 0),
            Some(a) => (
                a.key.0.len(),
                encoded_len(&a.nash.as_ref().map_or(0, NameHash::to_u32)),
            ),
        };
        let fields = Self {
            src: encoded_len(&header.src),
            dst: encoded_len(&header.dst),
            meta: 1,
            key,
            name_hash,
            total: header_len(header),
        };
        assert_eq!(
            fields.src + fields.dst + fields.meta + fields.key + fields.name_hash,
            fields.total,
            "header fields do not add up to the encoded header"
        );
        fields
    }
}
