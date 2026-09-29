//! Passwords, session tokens and the random ids everything else is keyed by.
//!
//! # The verifier format
//!
//! `pbkdf2$<iterations>$<b64 salt>$<b64 hash>` — self-describing on purpose.
//! The iteration count travels WITH the hash, so raising the cost later leaves
//! every existing account verifiable at the cost it was created with, and a
//! re-hash on next login is an upgrade rather than a migration.
//!
//! The comparison is constant-time ([`subtle`]). A byte-by-byte `==` on a hash
//! leaks how many leading bytes were right, which is enough to find the rest.

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use hmac::Hmac;
use rand::RngCore;
use sha2::{Digest, Sha256};
use subtle::ConstantTimeEq;

/// The cost new passwords are hashed at. Existing hashes keep whatever cost
/// they were written with — that is what the `$<iterations>$` field is for.
pub const DEFAULT_ITERATIONS: u32 = 120_000;

const SALT_BYTES: usize = 16;
const HASH_BYTES: usize = 32;

/// `n` bytes of cryptographic randomness, URL-safe base64 with no padding —
/// the session tokens, the salts, and every record id.
pub fn random_token(bytes: usize) -> String {
    let mut buf = vec![0u8; bytes];
    rand::thread_rng().fill_bytes(&mut buf);
    URL_SAFE_NO_PAD.encode(buf)
}

/// A fresh record id. 16 bytes is 128 bits — collision is not a failure mode
/// that needs handling.
pub fn new_id() -> String {
    random_token(16)
}

/// Hash `password` at [`DEFAULT_ITERATIONS`] with a fresh salt.
pub fn hash_password(password: &str) -> String {
    let mut salt = [0u8; SALT_BYTES];
    rand::thread_rng().fill_bytes(&mut salt);
    hash_with(password, &salt, DEFAULT_ITERATIONS)
}

fn hash_with(password: &str, salt: &[u8], iterations: u32) -> String {
    let mut out = [0u8; HASH_BYTES];
    // `pbkdf2` only errors on a zero-length output buffer, which HASH_BYTES
    // rules out at compile time.
    pbkdf2::pbkdf2::<Hmac<Sha256>>(password.as_bytes(), salt, iterations, &mut out)
        .expect("HASH_BYTES is a valid PBKDF2 output length");
    format!(
        "pbkdf2${iterations}${}${}",
        URL_SAFE_NO_PAD.encode(salt),
        URL_SAFE_NO_PAD.encode(out)
    )
}

/// Whether `password` matches `verifier`. A malformed verifier is a `false`,
/// never a panic — a hand-edited database file must not take the server down.
pub fn verify_password(password: &str, verifier: &str) -> bool {
    let mut parts = verifier.split('$');
    if parts.next() != Some("pbkdf2") {
        return false;
    }
    let Some(iterations) = parts.next().and_then(|n| n.parse::<u32>().ok()) else {
        return false;
    };
    let Some(salt) = parts.next().and_then(|s| URL_SAFE_NO_PAD.decode(s).ok()) else {
        return false;
    };
    let Some(expected) = parts.next().and_then(|s| URL_SAFE_NO_PAD.decode(s).ok()) else {
        return false;
    };
    if parts.next().is_some() || expected.is_empty() || iterations == 0 {
        return false;
    }
    let mut actual = vec![0u8; expected.len()];
    if pbkdf2::pbkdf2::<Hmac<Sha256>>(password.as_bytes(), &salt, iterations, &mut actual).is_err() {
        return false;
    }
    actual.ct_eq(&expected).into()
}

/// SHA-256 of a document, hex. What a revision records as its `content_hash`
/// so a client can see drift without fetching the payload.
pub fn content_hash(body: &str) -> String {
    let digest = Sha256::digest(body.as_bytes());
    let mut hex = String::with_capacity(digest.len() * 2);
    for byte in digest {
        use std::fmt::Write;
        let _ = write!(hex, "{byte:02x}");
    }
    hex
}

