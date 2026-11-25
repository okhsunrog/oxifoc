//! The measured messages: for each endpoint request, endpoint response and
//! topic message, the schema minimum and maximum of every top-level variant
//! and the hand-built representative values.

use std::any::type_name;

use ergot::{
    FrameKind, Header, TrafficClass,
    traits::{Endpoint, Topic},
    wire_frames::encode_frame_ty,
};
use postcard::ser_flavors::StdVec;
use postcard_schema::Schema;
use serde::{Serialize, de::DeserializeOwned};

use crate::{
    encoded_len,
    header::{HeaderShape, header_len},
    synth::{Extreme, root_variants, synthesize},
};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Group {
    Oxifoc,
    Ergot,
}

impl Group {
    pub const ALL: [Self; 2] = [Self::Oxifoc, Self::Ergot];

    pub fn label(self) -> &'static str {
        match self {
            Self::Oxifoc => "OxiFOC",
            Self::Ergot => "ergot",
        }
    }
}

/// One direction of an endpoint, or a topic, and how it is addressed.
#[derive(Clone, Copy)]
pub struct Socket {
    pub group: Group,
    pub path: &'static str,
    /// `req`, `resp` or `topic`.
    pub part: &'static str,
    pub key: [u8; 8],
    pub kind: FrameKind,
    pub class: TrafficClass,
    pub shape: HeaderShape,
    /// The same message sent to a known port, for requests that carry the
    /// any/all appendix.
    pub known_port: Option<HeaderShape>,
}

/// Encodes the message's value behind a given header. Boxed so messages of
/// every type share one list.
type Encoder = Box<dyn Fn(&Header) -> Vec<u8>>;

pub struct Message {
    pub socket: Socket,
    /// A top-level enum variant, `-` for a type that is not an enum, or
    /// `rep` for a representative value.
    pub variant: String,
    /// `min`, `max` or `fixed` (min and max encode alike), or the
    /// representative's description.
    pub value: String,
    pub body_len: usize,
    encode: Encoder,
}

impl Message {
    pub fn header(&self, shape: &HeaderShape) -> Header {
        shape.header(self.socket.key, self.socket.kind, self.socket.class)
    }

    /// The whole ergot frame as ergot encodes it, with `shape`'s header.
    pub fn encode(&self, shape: &HeaderShape) -> Vec<u8> {
        let header = self.header(shape);
        let frame = (self.encode)(&header);
        assert_eq!(
            frame.len(),
            header_len(&header) + self.body_len,
            "an ergot frame is its header and the postcard body"
        );
        frame
    }

    /// The frame with the header it is sent with.
    pub fn encode_as_sent(&self) -> Vec<u8> {
        self.encode(&self.socket.shape)
    }
}

/// A hand-built value and what it stands for.
pub type Representative<'a, T> = (&'a str, T);

#[derive(Default)]
pub struct Catalog {
    pub messages: Vec<Message>,
    /// The capacities the maximum values were built with.
    pub capacities: Vec<String>,
    /// Values that could not be built, and what synthesized values may lack.
    pub not_measured: Vec<String>,
}

impl Catalog {
    pub fn endpoint<E>(
        &mut self,
        group: Group,
        request_shape: HeaderShape,
        response_shape: HeaderShape,
        requests: Vec<Representative<'_, E::Request>>,
        responses: Vec<Representative<'_, E::Response>>,
    ) where
        E: Endpoint,
        E::Request: Schema + Serialize + DeserializeOwned + 'static,
        E::Response: Schema + Serialize + DeserializeOwned + 'static,
    {
        let request = Socket {
            group,
            path: E::PATH,
            part: "req",
            key: E::REQ_KEY.to_bytes(),
            kind: FrameKind::ENDPOINT_REQ,
            class: E::CLASS,
            shape: request_shape,
            known_port: request_shape.sent_to_known_port(),
        };
        self.add(request, requests);
        let response = Socket {
            part: "resp",
            key: E::RESP_KEY.to_bytes(),
            kind: FrameKind::ENDPOINT_RESP,
            shape: response_shape,
            known_port: None,
            ..request
        };
        self.add(response, responses);
    }

    pub fn topic<T>(
        &mut self,
        group: Group,
        shape: HeaderShape,
        representatives: Vec<Representative<'_, T::Message>>,
    ) where
        T: Topic,
        T::Message: Schema + Serialize + DeserializeOwned + 'static,
    {
        let socket = Socket {
            group,
            path: T::PATH,
            part: "topic",
            key: T::TOPIC_KEY.to_bytes(),
            kind: FrameKind::TOPIC_MSG,
            class: T::CLASS,
            shape,
            known_port: None,
        };
        self.add(socket, representatives);
    }

    /// The schema extremes of every top-level variant, then the
    /// representatives.
    fn add<T>(&mut self, socket: Socket, representatives: Vec<Representative<'_, T>>)
    where
        T: Schema + Serialize + DeserializeOwned + 'static,
    {
        let variants: Vec<(Option<usize>, &str)> = match root_variants(T::SCHEMA) {
            Some(names) => names
                .into_iter()
                .enumerate()
                .map(|(i, n)| (Some(i), n))
                .collect(),
            None => vec![(None, "-")],
        };
        for (index, variant) in variants {
            let min = synthesize::<T>(T::SCHEMA, Extreme::Min, index);
            let max = synthesize::<T>(T::SCHEMA, Extreme::Max, index);
            let (min, max) = match (min, max) {
                (Ok(min), Ok(max)) => (min, max),
                (min, max) => {
                    let error = |r: Result<_, String>| r.err().unwrap_or_else(|| "ok".into());
                    self.not_measured.push(format!(
                        "{} {} {variant}: min: {}; max: {}",
                        socket.path,
                        socket.part,
                        error(min.map(|_| ())),
                        error(max.map(|_| ()))
                    ));
                    continue;
                }
            };
            let ty = short_type_name(type_name::<T>());
            for note in min.notes.iter().chain(&max.notes) {
                self.not_measured.push(format!(
                    "{} {}: {ty} {variant}: {note}",
                    socket.path, socket.part
                ));
            }
            let capacities: Vec<String> = max
                .capacities
                .iter()
                .filter(|(_, capacity)| *capacity > 0)
                .map(|(path, capacity)| format!("{path}={capacity}"))
                .collect();
            if !capacities.is_empty() {
                self.capacities
                    .push(format!("{ty} {variant}: {}", capacities.join(", ")));
            }
            if encoded_len(&min.value) == encoded_len(&max.value) {
                self.push(socket, variant, "fixed", min.value);
            } else {
                self.push(socket, variant, "min", min.value);
                self.push(socket, variant, "max", max.value);
            }
        }
        for (description, value) in representatives {
            self.push(socket, "rep", description, value);
        }
    }

    fn push<T: Serialize + 'static>(
        &mut self,
        socket: Socket,
        variant: &str,
        value: &str,
        data: T,
    ) {
        self.messages.push(Message {
            socket,
            variant: variant.to_owned(),
            value: value.to_owned(),
            body_len: encoded_len(&data),
            encode: Box::new(move |header| {
                encode_frame_ty(StdVec::new(), header, &data).expect("a frame encodes into a Vec")
            }),
        });
    }
}

/// A type name without its module paths, generics included.
fn short_type_name(full: &str) -> String {
    let mut short = String::new();
    let mut word = String::new();
    for c in full.chars() {
        if c.is_alphanumeric() || c == '_' || c == ':' {
            word.push(c);
        } else {
            short.push_str(last_segment(&word));
            word.clear();
            short.push(c);
        }
    }
    short.push_str(last_segment(&word));
    short
}

fn last_segment(path: &str) -> &str {
    path.rsplit("::").next().unwrap_or(path)
}
