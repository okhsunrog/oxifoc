//! Domain-separated authentication for bridge administration requests.

pub type Key = [u8; 32];
pub type Nonce = [u8; 16];
pub type Tag = [u8; 32];

#[cfg(feature = "bridge-auth")]
use hmac::{Hmac, KeyInit as _, Mac as _};
#[cfg(feature = "bridge-auth")]
use sha2::Sha256;

#[cfg(feature = "bridge-auth")]
pub fn parse_key(text: &str) -> Option<Key> {
    if !text.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return None;
    }
    let (pairs, rest) = text.as_bytes().as_chunks::<2>();
    if pairs.len() != 32 || !rest.is_empty() {
        return None;
    }
    let mut key = [0; 32];
    for (byte, pair) in key.iter_mut().zip(pairs) {
        *byte = u8::from_str_radix(core::str::from_utf8(pair).ok()?, 16).ok()?;
    }
    Some(key)
}

#[cfg(feature = "bridge-auth")]
fn mac(key: &Key, domain: &[u8], payload: &[u8]) -> Hmac<Sha256> {
    let mut mac = Hmac::<Sha256>::new_from_slice(key).expect("HMAC accepts any key length");
    mac.update(domain);
    mac.update(payload);
    mac
}

#[cfg(feature = "bridge-auth")]
pub fn sign(key: &Key, domain: &[u8], payload: &[u8]) -> Tag {
    mac(key, domain, payload).finalize().into_bytes().into()
}

#[cfg(feature = "bridge-auth")]
pub fn verify(key: &Key, domain: &[u8], payload: &[u8], tag: &Tag) -> bool {
    mac(key, domain, payload).verify_slice(tag).is_ok()
}
