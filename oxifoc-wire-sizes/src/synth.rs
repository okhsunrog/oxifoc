//! Builds the smallest and the largest value of a type from its postcard
//! schema and proves each by deserializing it into the real Rust type, which
//! enforces the heapless capacities the schema does not state. The caller
//! re-encodes the decoded value with ergot's encoder; the bytes built here
//! are never used as a size.
//!
//! * Min: integers 1 (one varint byte; 1 rather than 0 so `NonZero*` fields
//!   decode), `false`, `None`, empty sequences and strings, and the enum
//!   variant with the shortest encoding.
//! * Max: integers at their type's extreme (longest varint), `Some`,
//!   sequences and strings at their capacity, and the enum variant with the
//!   longest encoding. Floats are fixed size either way.
//!
//! A capacity is found by searching for the longest length that still
//! deserializes, one sequence or string at a time.

use std::collections::{HashMap, HashSet};

use postcard_schema::schema::{
    DataModelType as Type, DataModelVariant as VariantType, NamedType, NamedVariant,
};
use serde::de::DeserializeOwned;

/// Capacities are searched up to this length.
const CAPACITY_SEARCH_LIMIT: usize = 1 << 16;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Extreme {
    Min,
    Max,
}

/// The names of a type's top-level variants, or `None` if it is not an enum.
pub fn root_variants(root: &NamedType) -> Option<Vec<&'static str>> {
    match root.ty {
        Type::Enum(variants) => Some(variants.iter().map(|v| v.name).collect()),
        _ => None,
    }
}

pub struct Synthesized<T> {
    pub value: T,
    /// Capacity of each sequence and string, by schema path (Max only).
    pub capacities: Vec<(String, usize)>,
    /// What the value may lack: maps left empty, nested schemas, capacities
    /// past the search limit.
    pub notes: Vec<String>,
}

/// Builds the `extreme` value of `T`, of top-level variant `root_variant`
/// when `T` is an enum. `Err` when `T`'s `Deserialize` refuses the bytes the
/// schema describes.
pub fn synthesize<T: DeserializeOwned>(
    root: &NamedType,
    extreme: Extreme,
    root_variant: Option<usize>,
) -> Result<Synthesized<T>, String> {
    let schema = Schema {
        root,
        extreme,
        root_variant,
    };
    let mut lengths = HashMap::new();
    let mut sized = HashSet::new();
    let mut notes = Vec::new();
    loop {
        let built = schema.build(&lengths, None);
        if !decodes_exactly::<T>(&built.bytes) {
            return Err(format!(
                "synthesized {extreme:?} bytes refused by Deserialize ({} bytes)",
                built.bytes.len()
            ));
        }
        // Sizing one sequence can reveal others (the elements of a
        // sequence that was empty), so repeat until no new path appears.
        let unsized_paths: Vec<String> = built
            .lengths_used
            .into_iter()
            .filter(|p| !sized.contains(p))
            .collect();
        if unsized_paths.is_empty() {
            let value = postcard::from_bytes::<T>(&built.bytes).map_err(|e| e.to_string())?;
            let mut capacities: Vec<_> = lengths.into_iter().collect();
            capacities.sort();
            notes.extend(built.notes);
            return Ok(Synthesized {
                value,
                capacities,
                notes,
            });
        }
        for path in unsized_paths {
            sized.insert(path.clone());
            let capacity = schema
                .capacity::<T>(&lengths, &path)
                .unwrap_or_else(|longest| {
                    notes.push(format!(
                        "{path}: no capacity found up to {CAPACITY_SEARCH_LIMIT}"
                    ));
                    longest
                });
            lengths.insert(path, capacity);
        }
    }
}

fn decodes_exactly<T: DeserializeOwned>(bytes: &[u8]) -> bool {
    matches!(postcard::take_from_bytes::<T>(bytes), Ok((_, rest)) if rest.is_empty())
}

#[derive(Clone, Copy)]
struct Schema<'a> {
    root: &'a NamedType,
    extreme: Extreme,
    root_variant: Option<usize>,
}

struct Built {
    bytes: Vec<u8>,
    /// Paths of the sequences and strings whose length was asked for, in
    /// the order they were met.
    lengths_used: Vec<String>,
    notes: Vec<String>,
}

impl Schema<'_> {
    /// `target`: while searching the capacity of that path, the enum
    /// variants on the way to it are chosen so the path exists.
    fn build(&self, lengths: &HashMap<String, usize>, target: Option<&str>) -> Built {
        let mut builder = Builder {
            extreme: self.extreme,
            lengths,
            target,
            root_variant: self.root_variant,
            lengths_used: Vec::new(),
            notes: Vec::new(),
        };
        let mut bytes = Vec::new();
        builder.value(self.root, "", &mut bytes);
        Built {
            bytes,
            lengths_used: builder.lengths_used,
            notes: builder.notes,
        }
    }

    /// The longest length of `path` that still deserializes, the other
    /// lengths as in `lengths`. `Err` with the longest length tried when
    /// even the search limit deserializes.
    fn capacity<T: DeserializeOwned>(
        &self,
        lengths: &HashMap<String, usize>,
        path: &str,
    ) -> Result<usize, usize> {
        let mut trial = lengths.clone();
        let mut decodes_with = |len: usize| {
            trial.insert(path.to_owned(), len);
            decodes_exactly::<T>(&self.build(&trial, Some(path)).bytes)
        };
        if !decodes_with(0) {
            return Ok(0);
        }
        let (mut fits, mut too_long) = (0, 1);
        while decodes_with(too_long) {
            fits = too_long;
            too_long *= 2;
            if too_long > CAPACITY_SEARCH_LIMIT {
                return Err(fits);
            }
        }
        while too_long - fits > 1 {
            let mid = fits.midpoint(too_long);
            if decodes_with(mid) {
                fits = mid;
            } else {
                too_long = mid;
            }
        }
        Ok(fits)
    }
}

struct Builder<'a> {
    extreme: Extreme,
    lengths: &'a HashMap<String, usize>,
    target: Option<&'a str>,
    root_variant: Option<usize>,
    lengths_used: Vec<String>,
    notes: Vec<String>,
}

impl Builder<'_> {
    fn length(&mut self, path: &str) -> usize {
        if self.extreme == Extreme::Min {
            return 0;
        }
        if !self.lengths_used.iter().any(|p| p == path) {
            self.lengths_used.push(path.to_owned());
        }
        self.lengths.get(path).copied().unwrap_or(0)
    }

    fn value(&mut self, ty: &NamedType, path: &str, out: &mut Vec<u8>) {
        let max = self.extreme == Extreme::Max;
        let unsigned = |extreme: u128| if max { extreme } else { 1 };
        let signed = |extreme: i128| zigzag(if max { extreme } else { 1 });
        match ty.ty {
            Type::Bool => out.push(u8::from(max)),
            Type::U8 => out.push(if max { u8::MAX } else { 1 }),
            Type::I8 => out.push(if max { i8::MIN.to_le_bytes()[0] } else { 1 }),
            Type::U16 => varint(out, unsigned(u16::MAX.into())),
            Type::U32 => varint(out, unsigned(u32::MAX.into())),
            Type::U64 | Type::Usize => varint(out, unsigned(u64::MAX.into())),
            Type::U128 => varint(out, unsigned(u128::MAX)),
            Type::I16 => varint(out, signed(i16::MIN.into())),
            Type::I32 => varint(out, signed(i32::MIN.into())),
            Type::I64 | Type::Isize => varint(out, signed(i64::MIN.into())),
            Type::I128 => varint(out, signed(i128::MIN)),
            Type::F32 => out.extend(1.0f32.to_le_bytes()),
            Type::F64 => out.extend(1.0f64.to_le_bytes()),
            Type::Char => {
                let c = if max { '\u{10FFFF}' } else { 'a' };
                let mut utf8 = [0; 4];
                let s = c.encode_utf8(&mut utf8);
                varint(out, s.len() as u128);
                out.extend(s.as_bytes());
            }
            Type::String | Type::ByteArray => {
                let len = self.length(path);
                varint(out, len as u128);
                out.extend(std::iter::repeat_n(b'a', len));
            }
            Type::Option(inner) => {
                out.push(u8::from(max));
                if max {
                    self.value(inner, &format!("{path}?"), out);
                }
            }
            Type::Unit | Type::UnitStruct => {}
            Type::NewtypeStruct(inner) => self.value(inner, path, out),
            Type::Seq(element) => {
                let len = self.length(path);
                varint(out, len as u128);
                for _ in 0..len {
                    self.value(element, &format!("{path}[]"), out);
                }
            }
            Type::Tuple(elements) | Type::TupleStruct(elements) => {
                for (i, element) in elements.iter().enumerate() {
                    self.value(element, &format!("{path}.{i}"), out);
                }
            }
            Type::Map { .. } => {
                self.notes.push(format!("map at {path} left empty"));
                varint(out, 0);
            }
            Type::Struct(fields) => {
                for field in fields.iter() {
                    self.value(field.ty, &format!("{path}.{}", field.name), out);
                }
            }
            Type::Enum(variants) => {
                let index = self.choose_variant(variants, path);
                varint(out, index as u128);
                self.variant(variants[index], path, out);
            }
            Type::Schema => self
                .notes
                .push(format!("schema type at {path} not synthesized")),
        }
    }

    fn variant(&mut self, variant: &NamedVariant, path: &str, out: &mut Vec<u8>) {
        let path = format!("{path}::{}", variant.name);
        match variant.ty {
            VariantType::UnitVariant => {}
            VariantType::NewtypeVariant(inner) => self.value(inner, &path, out),
            VariantType::TupleVariant(elements) => {
                for (i, element) in elements.iter().enumerate() {
                    self.value(element, &format!("{path}.{i}"), out);
                }
            }
            VariantType::StructVariant(fields) => {
                for field in fields.iter() {
                    self.value(field.ty, &format!("{path}.{}", field.name), out);
                }
            }
        }
    }

    /// The requested top-level variant; else the variant the capacity
    /// search needs; else the shortest (Min) or longest (Max) one, the first
    /// on a tie.
    fn choose_variant(&mut self, variants: &[&NamedVariant], path: &str) -> usize {
        if path.is_empty()
            && let Some(index) = self.root_variant
        {
            return index;
        }
        if let Some(target) = self.target
            && let Some(index) = variants
                .iter()
                .position(|v| lies_under(target, &format!("{path}::{}", v.name)))
        {
            return index;
        }
        let mut best: Option<(usize, usize)> = None;
        for (index, variant) in variants.iter().enumerate() {
            let mut bytes = Vec::new();
            varint(&mut bytes, index as u128);
            self.variant(variant, path, &mut bytes);
            let better = best.is_none_or(|(_, len)| match self.extreme {
                Extreme::Max => bytes.len() > len,
                Extreme::Min => bytes.len() < len,
            });
            if better {
                best = Some((index, bytes.len()));
            }
        }
        best.map_or(0, |(index, _)| index)
    }
}

/// `path` is `prefix` or lies below it, on a segment boundary.
fn lies_under(path: &str, prefix: &str) -> bool {
    path.strip_prefix(prefix)
        .is_some_and(|rest| rest.is_empty() || rest.starts_with(['.', '[', '?', ':']))
}

fn varint(out: &mut Vec<u8>, mut value: u128) {
    loop {
        let low = (value & 0x7f) as u8;
        value >>= 7;
        if value == 0 {
            out.push(low);
            return;
        }
        out.push(low | 0x80);
    }
}

fn zigzag(value: i128) -> u128 {
    ((value << 1) ^ (value >> 127)) as u128
}

#[cfg(test)]
mod tests {
    use super::*;

    fn varint_bytes(value: u128) -> Vec<u8> {
        let mut out = Vec::new();
        varint(&mut out, value);
        out
    }

    #[test]
    fn varints_match_postcard() {
        for value in [0u32, 1, 127, 128, 16_383, 16_384, u32::MAX] {
            assert_eq!(
                varint_bytes(value.into()),
                postcard::to_stdvec(&value).unwrap()
            );
        }
        for value in [0i32, 1, -1, 63, -64, 64, i32::MIN, i32::MAX] {
            assert_eq!(
                varint_bytes(zigzag(value.into())),
                postcard::to_stdvec(&value).unwrap()
            );
        }
    }

    #[test]
    fn capacities_come_from_deserialize() {
        let max = synthesize::<heapless::Vec<u8, 12>>(
            <heapless::Vec<u8, 12> as postcard_schema::Schema>::SCHEMA,
            Extreme::Max,
            None,
        )
        .unwrap();
        assert_eq!(max.value.len(), 12);
        assert_eq!(max.capacities, [(String::new(), 12)]);
    }
}
