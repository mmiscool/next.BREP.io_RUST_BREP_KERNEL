//! Public handles: the opaque `<kind>_<epoch>_<nonce>` strings an agent names
//! a document by (the contract's §3). The epoch is random per process, so a
//! handle from an earlier run is recognisably stale; the nonce is random per
//! object, so a handle cannot be guessed from another. Both are 128 random
//! bits written as 22 characters of unpadded base64url. The app's own
//! `Document::id` (a counter) stays the internal key and never leaves the
//! process.
use std::sync::OnceLock;

/// 128 random bits. A v4 uuid carries 122 (six are its version and variant);
/// xoring in a second one's bytes shifted by one position covers those six
/// with random bits too.
fn random_128() -> [u8; 16] {
    let a = *uuid::Uuid::new_v4().as_bytes();
    let b = *uuid::Uuid::new_v4().as_bytes();
    std::array::from_fn(|i| a[i] ^ b[(i + 1) % 16])
}

/// Unpadded base64url of 16 bytes: always 22 characters.
fn encode(bytes: &[u8; 16]) -> String {
    const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
    let mut out = String::with_capacity(22);
    let mut acc = 0u32;
    let mut bits = 0;
    for &byte in bytes {
        acc = (acc << 8) | u32::from(byte);
        bits += 8;
        while bits >= 6 {
            bits -= 6;
            out.push(ALPHABET[((acc >> bits) & 63) as usize] as char);
        }
    }
    if bits > 0 {
        out.push(ALPHABET[((acc << (6 - bits)) & 63) as usize] as char);
    }
    out
}

/// This process's epoch, fixed at first use.
pub fn epoch() -> &'static str {
    static EPOCH: OnceLock<String> = OnceLock::new();
    EPOCH.get_or_init(|| encode(&random_128()))
}

/// A fresh handle of `kind` (`doc`, `op`, `exec`, `art`).
pub fn mint(kind: &str) -> String {
    format!("{kind}_{}_{}", epoch(), encode(&random_128()))
}

/// What a handle string says about itself, checked by shape before any lookup.
#[derive(Debug, PartialEq, Eq)]
pub enum Parsed {
    /// Well-formed, of this kind and this process's epoch: look it up.
    Current,
    /// Well-formed and of this kind, but minted by another process.
    OtherEpoch,
    /// Not a handle of this kind.
    Malformed,
}

pub fn parse(kind: &str, handle: &str) -> Parsed {
    let Some(rest) = handle.strip_prefix(kind).and_then(|r| r.strip_prefix('_')) else {
        return Parsed::Malformed;
    };
    let well = |part: &str| part.len() == 22 && part.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_');
    // The epoch may itself contain '_' (base64url), so split by length.
    if rest.len() != 45 || rest.as_bytes()[22] != b'_' {
        return Parsed::Malformed;
    }
    let (epoch_part, nonce) = (&rest[..22], &rest[23..]);
    if !well(epoch_part) || !well(nonce) {
        return Parsed::Malformed;
    }
    if epoch_part == epoch() {
        Parsed::Current
    } else {
        Parsed::OtherEpoch
    }
}

