//! Admission keys (contract §7.1): stateless, authenticated, five-minute
//! first-admission validity.
//!
//! `key_<epoch>_<payload>` where payload = base64url(nonce\[16\] ‖ purpose\[1\] ‖
//! scope digest\[8\] ‖ deadline ms since process start \[8\] ‖ MAC\[16\]). The MAC is
//! HMAC-SHA256 under an epoch-local random secret, truncated to 128 bits. The
//! deadline is MONOTONIC (ms since this process's origin), so moving the wall
//! clock cannot extend it; `firstAdmissionExpiresAt` is its projection.
use super::wire::{self, ApiError};
use sha2::{Digest, Sha256};
use std::sync::OnceLock;
use std::time::{Duration, Instant};

pub const FIRST_ADMISSION: Duration = Duration::from_secs(300);
const MAX_KEY_BYTES: usize = 512;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Purpose {
    Command,
    Execution,
}

impl Purpose {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "command" => Some(Purpose::Command),
            "execution" => Some(Purpose::Execution),
            _ => None,
        }
    }
    fn byte(self) -> u8 {
        match self {
            Purpose::Command => 1,
            Purpose::Execution => 2,
        }
    }
}

fn secret() -> &'static [u8; 32] {
    static SECRET: OnceLock<[u8; 32]> = OnceLock::new();
    SECRET.get_or_init(|| {
        let (a, b) = (wire::random_128(), wire::random_128());
        std::array::from_fn(|i| if i < 16 { a[i] } else { b[i - 16] })
    })
}

fn origin() -> Instant {
    static ORIGIN: OnceLock<Instant> = OnceLock::new();
    *ORIGIN.get_or_init(Instant::now)
}

fn hmac(message: &[u8]) -> [u8; 32] {
    let key = secret();
    let mut ipad = [0x36u8; 64];
    let mut opad = [0x5cu8; 64];
    for i in 0..32 {
        ipad[i] ^= key[i];
        opad[i] ^= key[i];
    }
    let inner = Sha256::new().chain_update(ipad).chain_update(message).finalize();
    Sha256::new().chain_update(opad).chain_update(inner).finalize().into()
}

fn scope_digest(scope: &str) -> [u8; 8] {
    let d = Sha256::digest(scope.as_bytes());
    std::array::from_fn(|i| d[i])
}

/// A key's verified content.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Key {
    /// The nonce, as the binding's identity.
    pub nonce: String,
    pub purpose: Purpose,
    pub deadline: Instant,
}

impl Key {
    pub fn expired(&self) -> bool {
        Instant::now() >= self.deadline
    }
}

/// A fresh key for `scope` and `purpose` and its advertised expiry.
pub fn issue(scope: &str, purpose: Purpose) -> (String, String) {
    let nonce = wire::random_128();
    let deadline = Instant::now() + FIRST_ADMISSION;
    let ms = deadline.duration_since(origin()).as_millis() as u64;
    let mut payload = Vec::with_capacity(49);
    payload.extend_from_slice(&nonce);
    payload.push(purpose.byte());
    payload.extend_from_slice(&scope_digest(scope));
    payload.extend_from_slice(&ms.to_be_bytes());
    let mac = hmac(&payload);
    payload.extend_from_slice(&mac[..16]);
    (format!("key_{}_{}", wire::epoch(), wire::b64url(&payload)), wire::timestamp(deadline))
}

/// Verify a key's epoch, authentication, scope and purpose. Expiry is NOT
/// refused here: the caller looks up a retained binding first (§7.1), and
/// only a key with no binding is refused for being past its deadline.
pub fn verify(key: &str, scope: &str, purpose: Purpose) -> Result<Key, ApiError> {
    if key.len() > MAX_KEY_BYTES {
        return Err(ApiError::new("invalid_arguments", "validation", "idempotencyKey is longer than 512 bytes"));
    }
    let invalid = || ApiError::new("invalid_arguments", "validation", "idempotencyKey is not a key this service issued; call cad_issue_key");
    let rest = key.strip_prefix("key_").ok_or_else(invalid)?;
    let (epoch, payload) = rest.split_at_checked(22).ok_or_else(invalid)?;
    let payload = payload.strip_prefix('_').ok_or_else(invalid)?;
    if epoch != wire::epoch() {
        return Err(ApiError::new("handle_epoch_mismatch", "validation", "the key was issued by an earlier run of the server")
            .effects(wire::Effects::unknown()));
    }
    let bytes = wire::b64url_decode(payload).filter(|b| b.len() == 49).ok_or_else(invalid)?;
    let (body, mac) = bytes.split_at(33);
    if hmac(body)[..16] != *mac {
        return Err(invalid());
    }
    if body[16] != purpose.byte() {
        return Err(ApiError::new("invalid_arguments", "validation", "the key was issued for another purpose (command vs execution)"));
    }
    if body[17..25] != scope_digest(scope) {
        return Err(invalid());
    }
    let ms = u64::from_be_bytes(body[25..33].try_into().unwrap());
    Ok(Key { nonce: wire::b64url(&body[..16]), purpose, deadline: origin() + Duration::from_millis(ms) })
}

