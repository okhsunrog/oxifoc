//! Control of the bridge's wireless SWD probe over Ergot.

use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

#[cfg(feature = "bridge-auth")]
use super::bridge_auth::{self, Key};
use super::bridge_auth::{Nonce, Tag};

pub const MAX_ARM_MINUTES: u8 = 30;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Schema)]
pub enum ProbeRequest {
    Arm {
        minutes: u8,
    },
    Disarm,
    Challenge,
    Authenticated {
        operation: ProbeOperation,
        nonce: Nonce,
        tag: Tag,
    },
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Schema)]
pub enum ProbeOperation {
    Arm { minutes: u8 },
    Disarm,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Schema)]
pub enum ProbeResponse {
    Armed { ipv4: [u8; 4], minutes: u8 },
    Disarmed,
    UsbRequired,
    InvalidDuration,
    WifiUnavailable,
    Challenge { nonce: Nonce },
    AuthorizationUnavailable,
    Unauthorized,
}

#[cfg(feature = "bridge-auth")]
impl ProbeOperation {
    const DOMAIN: &[u8] = b"OxiFOC C5 SWD v1\0";

    fn payload(self, nonce: Nonce) -> [u8; 18] {
        let mut payload = [0; 18];
        payload[..16].copy_from_slice(&nonce);
        if let Self::Arm { minutes } = self {
            payload[16..].copy_from_slice(&[1, minutes]);
        }
        payload
    }

    pub fn sign(self, key: &Key, nonce: Nonce) -> Tag {
        bridge_auth::sign(key, Self::DOMAIN, &self.payload(nonce))
    }

    pub fn verify(self, key: &Key, nonce: Nonce, tag: &Tag) -> bool {
        bridge_auth::verify(key, Self::DOMAIN, &self.payload(nonce), tag)
    }
}

#[cfg(all(test, feature = "bridge-auth"))]
mod tests {
    use super::*;

    #[test]
    fn authorization_binds_operation_duration_and_nonce() {
        let key = [1; 32];
        let nonce = [2; 16];
        let operation = ProbeOperation::Arm { minutes: 5 };
        let tag = operation.sign(&key, nonce);
        assert!(operation.verify(&key, nonce, &tag));
        assert!(!operation.verify(&key, [3; 16], &tag));
        assert!(!ProbeOperation::Arm { minutes: 6 }.verify(&key, nonce, &tag));
        assert!(!ProbeOperation::Disarm.verify(&key, nonce, &tag));
    }
}
