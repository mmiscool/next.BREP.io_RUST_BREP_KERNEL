//! The common wire types (contract §3): handles, timestamps, `ApiError`,
//! `Effects` and `Response<T>`, as JSON values.
//!
//! Handles are `<kind>_<epoch>_<nonce>`: 128 random bits each, 22 characters
//! of unpadded base64url. The epoch is this server process's; document
//! handles are minted by the app with ITS process epoch
//! (`BREP_app/src/handles.rs`) — in one process both are fresh per run, which
//! is what the contract needs of them (an old handle is recognisably stale).
use serde_json::{json, Value};
use std::sync::OnceLock;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub fn random_128() -> [u8; 16] {
    // A v4 uuid carries 122 random bits; xoring a second one's bytes in,
    // shifted by one, covers its six fixed bits with random ones.
    let a = *uuid::Uuid::new_v4().as_bytes();
    let b = *uuid::Uuid::new_v4().as_bytes();
    std::array::from_fn(|i| a[i] ^ b[(i + 1) % 16])
}

const B64URL: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

pub fn b64url(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 4 / 3 + 2);
    let (mut acc, mut bits) = (0u32, 0);
    for &byte in bytes {
        acc = (acc << 8) | u32::from(byte);
        bits += 8;
        while bits >= 6 {
            bits -= 6;
            out.push(B64URL[((acc >> bits) & 63) as usize] as char);
        }
    }
    if bits > 0 {
        out.push(B64URL[((acc << (6 - bits)) & 63) as usize] as char);
    }
    out
}

pub fn b64url_decode(text: &str) -> Option<Vec<u8>> {
    let mut out = Vec::with_capacity(text.len() * 3 / 4);
    let (mut acc, mut bits) = (0u32, 0);
    for c in text.bytes() {
        let v = B64URL.iter().position(|&a| a == c)? as u32;
        acc = (acc << 6) | v;
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push((acc >> bits) as u8);
        }
    }
    Some(out)
}

/// This process's epoch.
pub fn epoch() -> &'static str {
    static EPOCH: OnceLock<String> = OnceLock::new();
    EPOCH.get_or_init(|| b64url(&random_128()))
}

/// A fresh handle of `kind` (`op`, `exec`, `art`).
pub fn mint(kind: &str) -> String {
    format!("{kind}_{}_{}", epoch(), b64url(&random_128()))
}

#[derive(Debug, PartialEq, Eq)]
pub enum HandleShape {
    Current,
    OtherEpoch,
    Malformed,
}

/// What a handle string says about itself, by shape, before any lookup.
pub fn handle_shape(kind: &str, handle: &str) -> HandleShape {
    let Some(rest) = handle.strip_prefix(kind).and_then(|r| r.strip_prefix('_')) else {
        return HandleShape::Malformed;
    };
    let well = |p: &str| p.len() == 22 && p.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_');
    if rest.len() != 45 || rest.as_bytes()[22] != b'_' || !well(&rest[..22]) || !well(&rest[23..]) {
        return HandleShape::Malformed;
    }
    if &rest[..22] == epoch() {
        HandleShape::Current
    } else {
        HandleShape::OtherEpoch
    }
}

/// A document handle's shape. Documents are minted by the app with ITS
/// epoch, so only the shape is checked here; the app checks the epoch.
pub fn handle_shape_doc(handle: &str) -> HandleShape {
    let Some(rest) = handle.strip_prefix("doc_") else { return HandleShape::Malformed };
    let well = |p: &str| p.len() == 22 && p.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_');
    if rest.len() == 45 && rest.as_bytes()[22] == b'_' && well(&rest[..22]) && well(&rest[23..]) {
        HandleShape::Current
    } else {
        HandleShape::Malformed
    }
}

/// The process's monotonic origin and the wall clock read at it: every
/// advertised timestamp is projected from monotonic time, so a wall-clock
/// step cannot lengthen a deadline.
fn origin() -> &'static (Instant, SystemTime) {
    static ORIGIN: OnceLock<(Instant, SystemTime)> = OnceLock::new();
    ORIGIN.get_or_init(|| (Instant::now(), SystemTime::now()))
}

/// UTC RFC 3339 with milliseconds for a monotonic instant.
pub fn timestamp(at: Instant) -> String {
    let (mono, wall) = origin();
    let wall = match at.checked_duration_since(*mono) {
        Some(d) => *wall + d,
        None => *wall - mono.duration_since(at),
    };
    let ms = wall.duration_since(UNIX_EPOCH).unwrap_or(Duration::ZERO).as_millis() as i64;
    let (days, rem) = (ms.div_euclid(86_400_000), ms.rem_euclid(86_400_000));
    let (year, month, day) = brep_render::sheets::frame::civil_from_days(days);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}.{:03}Z",
        rem / 3_600_000,
        rem / 60_000 % 60,
        rem / 1000 % 60,
        rem % 1000
    )
}

pub fn now() -> String {
    timestamp(Instant::now())
}

/// One document's before/after in an outcome.
pub fn doc_effect(document_id: &str, before: Option<&str>, after: Option<&str>, changed: bool) -> Value {
    json!({ "documentId": document_id, "beforeRevision": before, "afterRevision": after, "changed": changed })
}

/// `Effects` (contract §3.1).
#[derive(Debug, Clone)]
pub struct Effects {
    pub disposition: &'static str,
    pub documents: Vec<Value>,
    pub external: Vec<Value>,
}

impl Effects {
    pub fn none() -> Self {
        Effects { disposition: "none", documents: vec![], external: vec![] }
    }
    pub fn unknown() -> Self {
        Effects { disposition: "unknown", documents: vec![], external: vec![] }
    }
    pub fn to_json(&self) -> Value {
        json!({ "disposition": self.disposition, "documents": self.documents, "external": self.external })
    }
}

/// The contract's error record. `message` is bounded to 2 KiB and is the
/// caller-safe sentence, not a raw native payload.
#[derive(Debug, Clone)]
pub struct ApiError {
    pub code: &'static str,
    pub message: String,
    pub phase: &'static str,
    pub operation_id: Option<String>,
    pub execution_id: Option<String>,
    pub context: Option<Value>,
    pub effects: Effects,
    pub retry: &'static str,
}

impl ApiError {
    pub fn new(code: &'static str, phase: &'static str, message: impl Into<String>) -> Self {
        let retry = match code {
            "overloaded" | "not_ready" => "retry_same_key_within_retention",
            "revision_conflict" | "document_busy" | "document_not_active" => "inspect_then_new_submission",
            _ => "do_not_retry",
        };
        ApiError {
            code,
            message: bounded(message.into(), 2048),
            phase,
            operation_id: None,
            execution_id: None,
            context: None,
            effects: Effects::none(),
            retry,
        }
    }
    pub fn effects(mut self, effects: Effects) -> Self {
        self.effects = effects;
        self
    }
    pub fn retry(mut self, retry: &'static str) -> Self {
        self.retry = retry;
        self
    }
    pub fn op(mut self, id: &str) -> Self {
        self.operation_id = Some(id.to_string());
        self
    }
    pub fn exec(mut self, id: &str) -> Self {
        self.execution_id = Some(id.to_string());
        self
    }
    pub fn context(mut self, context: Value) -> Self {
        self.context = Some(context);
        self
    }
    pub fn to_json(&self) -> Value {
        json!({
            "code": self.code,
            "message": self.message,
            "phase": self.phase,
            "correlationId": b64url(&random_128()[..8]),
            "operationId": self.operation_id,
            "executionId": self.execution_id,
            "context": self.context,
            "effects": self.effects.to_json(),
            "retry": self.retry,
        })
    }

    /// An app reply's error string begins with the contract code when the
    /// app's context check refused it (`revision_conflict: …`); anything else
    /// is the command's own failure in `phase`.
    pub fn from_app(error: &str, phase: &'static str) -> Self {
        if let Some(message) = error.strip_prefix("overwrite_refused: ") {
            return Self::new("external_io_failed", "external_io", sanitize(message)).retry("inspect_then_new_submission");
        }
        if let Some(message) = error.strip_prefix("external_io_failed: ") {
            return Self::new("external_io_failed", "external_io", sanitize(message)).effects(Effects::unknown());
        }
        if let Some(message) = error.strip_prefix("deadline_exceeded: ") {
            return Self::new("deadline_exceeded", "running", sanitize(message)).effects(Effects::unknown()).retry("inspect_then_new_submission");
        }
        if let Some(message) = error.strip_prefix("host_deadline_expired: ") {
            return Self::new("deadline_exceeded", "queue", sanitize(message)).retry("inspect_then_new_submission");
        }
        if let Some(message) = error.strip_prefix("host_deadline_exceeded: ") {
            return Self::new("deadline_exceeded", "queue", sanitize(message)).effects(Effects::unknown()).retry("inspect_then_new_submission");
        }
        const CODES: &[&str] = &[
            "invalid_arguments",
            "document_unknown",
            "document_closed",
            "document_not_active",
            "document_busy",
            "revision_conflict",
            "handle_epoch_mismatch",
        ];
        for code in CODES {
            if let Some(rest) = error.strip_prefix(code).and_then(|r| r.strip_prefix(": ")) {
                return ApiError::new(code, if *code == "revision_conflict" { "admission" } else { "validation" }, rest);
            }
        }
        ApiError::new(if phase == "evaluation" { "evaluation_failed" } else { "internal_error" }, phase, sanitize(error))
    }
}

/// `Response<T>`: `{ok: true, value}` or `{ok: false, error}`.
pub fn ok(value: Value) -> Value {
    json!({ "ok": true, "value": value })
}

pub fn fail(error: &ApiError) -> Value {
    json!({ "ok": false, "error": error.to_json() })
}

pub fn respond(result: Result<Value, ApiError>) -> Value {
    match result {
        Ok(v) => ok(v),
        Err(e) => fail(&e),
    }
}

pub fn bounded(mut text: String, max: usize) -> String {
    if text.len() > max {
        let mut cut = max.saturating_sub(3);
        while !text.is_char_boundary(cut) {
            cut -= 1;
        }
        text.truncate(cut);
        text.push('…');
    }
    text
}

/// A native message with host paths removed: anything that looks like an
/// absolute path becomes `<path>`.
pub fn sanitize(text: &str) -> String {
    let words: Vec<String> = text
        .split(' ')
        .map(|w| if w.starts_with('/') && w.len() > 1 || w.get(1..3) == Some(":\\") { "<path>".to_string() } else { w.to_string() })
        .collect();
    bounded(words.join(" "), 2048)
}

/// A canonical unsigned decimal u64.
pub fn parse_revision(text: &str) -> Option<u64> {
    let canonical = text == "0" || (!text.is_empty() && !text.starts_with('0') && text.bytes().all(|b| b.is_ascii_digit()));
    if canonical {
        text.parse().ok()
    } else {
        None
    }
}

/// `DocumentContext` from a JSON value, validated.
pub fn document_context(v: &Value) -> Result<(String, String), ApiError> {
    let obj = v.as_object().ok_or_else(|| ApiError::new("invalid_arguments", "validation", "context must be {documentId, expectedRevision}"))?;
    if obj.keys().any(|k| k != "documentId" && k != "expectedRevision") {
        return Err(ApiError::new("invalid_arguments", "validation", "context takes only documentId and expectedRevision"));
    }
    let id = obj.get("documentId").and_then(Value::as_str).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "context.documentId must be a string"))?;
    let rev = obj.get("expectedRevision").and_then(Value::as_str).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "context.expectedRevision must be a decimal string"))?;
    if parse_revision(rev).is_none() {
        return Err(ApiError::new("invalid_arguments", "validation", "context.expectedRevision must be a canonical unsigned decimal string"));
    }
    if !id.starts_with("doc_") {
        return Err(ApiError::new("invalid_arguments", "validation", "context.documentId is not a document handle"));
    }
    Ok((id.to_string(), rev.to_string()))
}

pub fn context_json(document_id: &str, revision: &str) -> Value {
    json!({ "documentId": document_id, "expectedRevision": revision })
}

