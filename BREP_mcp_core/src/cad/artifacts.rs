//! Artifacts (contract §12): opaque, scoped, expiring byte objects — a STEP
//! export, an uploaded model. Never a path. Bounded per object (32 MiB), per
//! process (512 MiB) and in age (30 min, advertised exactly).
use super::canon::sha256_hex;
use super::wire::{self, ApiError};
use serde_json::{json, Value};
use std::collections::HashMap;
use std::sync::Mutex;
use std::time::{Duration, Instant};

pub const MAX_ARTIFACT: usize = 32 << 20;
pub const MAX_PROCESS: usize = 512 << 20;
pub const AGE: Duration = Duration::from_secs(30 * 60);
pub const DEFAULT_CHUNK: usize = 64 << 10;
pub const MAX_CHUNK: usize = 256 << 10;
pub const PER_EXECUTION_COUNT: usize = 16;
pub const PER_EXECUTION_BYTES: usize = 128 << 20;

/// Who an artifact belongs to: the authorization scope plus, for one made
/// inside a code-mode execution, that execution.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Owner {
    pub scope: String,
    pub execution: Option<String>,
}

struct Artifact {
    owner: Owner,
    media_type: String,
    bytes: Vec<u8>,
    sha256: String,
    expires: Instant,
    /// Admitted operations still reading the bytes (an import).
    leases: usize,
}

/// A chunked upload in progress ([`Artifacts::begin`]): the bytes so far,
/// against the length and digest the caller declared. Its space is reserved
/// in the process budget from `begin`.
struct Upload {
    owner: Owner,
    media_type: String,
    length: usize,
    sha256: String,
    bytes: Vec<u8>,
    expires: Instant,
}

/// The largest chunk one `append` carries (its base64 fits a 1 MiB request).
pub const MAX_APPEND: usize = 512 << 10;

#[derive(Default)]
pub struct Artifacts {
    inner: Mutex<HashMap<String, Artifact>>,
    uploads: Mutex<HashMap<String, Upload>>,
    budgets: Mutex<HashMap<String, (usize, usize, Instant)>>,
}

impl Artifacts {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_execution_budget(&self, execution: &str, count: u64, bytes: u64) {
        let mut budgets = self.budgets.lock().unwrap();
        budgets.retain(|_, (_, _, expires)| *expires > Instant::now());
        budgets.insert(execution.into(), (count as usize, bytes as usize, Instant::now() + Duration::from_secs(45 * 60)));
    }

    fn budget(&self, owner: &Owner) -> (usize, usize) {
        owner.execution.as_ref().and_then(|id| self.budgets.lock().unwrap().get(id).map(|(n,b,_)| (*n,*b)))
            .unwrap_or((PER_EXECUTION_COUNT, PER_EXECUTION_BYTES))
    }

    fn sweep(map: &mut HashMap<String, Artifact>) {
        let now = Instant::now();
        map.retain(|_, a| a.expires > now || a.leases > 0);
    }

    pub fn create(&self, owner: Owner, media_type: &str, bytes: Vec<u8>) -> Result<Value, ApiError> {
        if bytes.len() > MAX_ARTIFACT {
            return Err(ApiError::new("resource_limit", "external_io", format!("an artifact may hold at most {MAX_ARTIFACT} bytes")));
        }
        let mut map = self.inner.lock().unwrap();
        Self::sweep(&mut map);
        if owner.execution.is_some() {
            let mine: Vec<&Artifact> = map.values().filter(|a| a.owner == owner && a.media_type != "text/plain").collect();
            let (count_limit, byte_limit) = self.budget(&owner);
            let uploads = self.uploads.lock().unwrap();
            let pending: Vec<_> = uploads.values().filter(|u| u.owner == owner && u.expires > Instant::now()).collect();
            if mine.len() + pending.len() >= count_limit {
                return Err(ApiError::new("resource_limit", "external_io", format!("an execution may create at most {PER_EXECUTION_COUNT} artifacts")));
            }
            if mine.iter().map(|a| a.bytes.len()).sum::<usize>() + pending.iter().map(|u| u.length).sum::<usize>() + bytes.len() > byte_limit {
                return Err(ApiError::new("resource_limit", "external_io", "the execution's artifact bytes are used up"));
            }
        }
        let used: usize = map.values().map(|a| a.bytes.len()).sum::<usize>() + self.reserved();
        if used + bytes.len() > MAX_PROCESS {
            return Err(ApiError::new("resource_limit", "external_io", "the process's artifact storage is full"));
        }
        let id = wire::mint("art");
        let artifact = Artifact {
            owner,
            media_type: media_type.to_string(),
            sha256: sha256_hex(&bytes),
            bytes,
            expires: Instant::now() + AGE,
            leases: 0,
        };
        let r = reference(&id, &artifact);
        map.insert(id, artifact);
        Ok(r)
    }

    fn get<'a>(map: &'a mut HashMap<String, Artifact>, id: &str, owner: &Owner) -> Result<&'a mut Artifact, ApiError> {
        match wire::handle_shape("art", id) {
            wire::HandleShape::Current => {}
            wire::HandleShape::OtherEpoch => {
                return Err(ApiError::new("handle_epoch_mismatch", "lookup", "the artifact is from an earlier run of the server").effects(wire::Effects::unknown()))
            }
            wire::HandleShape::Malformed => return Err(ApiError::new("invalid_arguments", "validation", "artifactId is not an artifact handle")),
        }
        let unknown = || ApiError::new("artifact_unknown", "lookup", "no such artifact");
        let a = map.get_mut(id).ok_or_else(unknown)?;
        // Another scope's (or another execution's) artifact reads as unknown:
        // a denial must not reveal that it exists.
        if a.owner.scope != owner.scope || (owner.execution.is_some() && a.owner.execution != owner.execution) {
            return Err(unknown());
        }
        if a.expires <= Instant::now() && a.leases == 0 {
            return Err(ApiError::new("artifact_expired", "lookup", "the artifact has expired"));
        }
        Ok(a)
    }

    pub fn read(&self, owner: &Owner, id: &str, offset: usize, limit: Option<usize>) -> Result<Value, ApiError> {
        let limit = limit.unwrap_or(DEFAULT_CHUNK);
        if limit == 0 || limit > MAX_CHUNK {
            return Err(ApiError::new("invalid_arguments", "validation", format!("limit must be 1..={MAX_CHUNK}")));
        }
        let mut map = self.inner.lock().unwrap();
        let a = Self::get(&mut map, id, owner)?;
        if offset > a.bytes.len() {
            return Err(ApiError::new("invalid_arguments", "validation", "offset is past the end of the artifact"));
        }
        let end = (offset + limit).min(a.bytes.len());
        use base64::Engine as _;
        Ok(json!({
            "artifact": reference(id, a),
            "offset": offset,
            "bytesBase64": base64::engine::general_purpose::STANDARD.encode(&a.bytes[offset..end]),
            "eof": end == a.bytes.len(),
        }))
    }

    /// Bytes reserved by uploads still in progress.
    fn reserved(&self) -> usize {
        let now = Instant::now();
        let mut uploads = self.uploads.lock().unwrap();
        uploads.retain(|_, u| u.expires > now);
        uploads.values().map(|u| u.length).sum()
    }

    /// Start a chunked upload of `length` bytes whose SHA-256 is `sha256`
    /// (contract §12's streaming route for inputs above the transport
    /// limit). Answers the upload id; the space is reserved now.
    pub fn begin(&self, owner: Owner, media_type: &str, length: usize, sha256: &str) -> Result<Value, ApiError> {
        if length == 0 || length > MAX_ARTIFACT {
            return Err(ApiError::new("resource_limit", "validation", format!("byteLength must be 1..={MAX_ARTIFACT}")));
        }
        if sha256.len() != 64 || !sha256.bytes().all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase()) {
            return Err(ApiError::new("invalid_arguments", "validation", "sha256 must be 64 lowercase hex digits"));
        }
        let mut map = self.inner.lock().unwrap();
        Self::sweep(&mut map);
        let used: usize = map.values().map(|a| a.bytes.len()).sum::<usize>() + self.reserved();
        if owner.execution.is_some() {
            let (count_limit, byte_limit) = self.budget(&owner);
            let uploads = self.uploads.lock().unwrap();
            let mine: Vec<_> = map.values().filter(|a| a.owner == owner && a.media_type != "text/plain").collect();
            let pending: Vec<_> = uploads.values().filter(|u| u.owner == owner && u.expires > Instant::now()).collect();
            if mine.len() + pending.len() >= count_limit || mine.iter().map(|a| a.bytes.len()).sum::<usize>() + pending.iter().map(|u| u.length).sum::<usize>() + length > byte_limit {
                return Err(ApiError::new("resource_limit", "external_io", "execution artifact budget exhausted"));
            }
        }
        if used + length > MAX_PROCESS {
            return Err(ApiError::new("resource_limit", "external_io", "the process's artifact storage is full"));
        }
        let id = wire::mint("art");
        let expires = Instant::now() + AGE;
        self.uploads.lock().unwrap().insert(
            id.clone(),
            Upload { owner, media_type: media_type.to_string(), length, sha256: sha256.to_string(), bytes: Vec::with_capacity(length), expires },
        );
        Ok(json!({ "uploadId": id, "maxChunkBytes": MAX_APPEND, "expiresAt": wire::timestamp(expires) }))
    }

    /// Append the next chunk; `offset` must be where the upload stands.
    pub fn append(&self, owner: &Owner, id: &str, offset: usize, bytes: Vec<u8>) -> Result<Value, ApiError> {
        if bytes.is_empty() || bytes.len() > MAX_APPEND {
            return Err(ApiError::new("invalid_arguments", "validation", format!("a chunk is 1..={MAX_APPEND} bytes")));
        }
        let mut uploads = self.uploads.lock().unwrap();
        let unknown = || ApiError::new("artifact_unknown", "lookup", "no upload in progress has that id");
        let u = uploads.get_mut(id).filter(|u| u.owner == *owner && u.expires > Instant::now()).ok_or_else(unknown)?;
        if offset != u.bytes.len() {
            return Err(ApiError::new("invalid_arguments", "validation", format!("offset {offset}: the upload stands at {}", u.bytes.len())));
        }
        if u.bytes.len() + bytes.len() > u.length {
            return Err(ApiError::new("invalid_arguments", "validation", "the chunk runs past the declared byteLength"));
        }
        u.bytes.extend_from_slice(&bytes);
        Ok(json!({ "uploadId": id, "received": u.bytes.len(), "byteLength": u.length }))
    }

    /// Finish an upload: the bytes must be exactly the declared length and
    /// digest. Answers the artifact's reference (the upload id becomes the
    /// artifact id).
    pub fn commit(&self, owner: &Owner, id: &str) -> Result<Value, ApiError> {
        let mut map = self.inner.lock().unwrap();
        let u = {
            let mut uploads = self.uploads.lock().unwrap();
            match uploads.get(id) {
                Some(u) if u.owner == *owner && u.expires > Instant::now() => uploads.remove(id).unwrap(),
                _ => return Err(ApiError::new("artifact_unknown", "lookup", "no upload in progress has that id")),
            }
        };
        if u.bytes.len() != u.length {
            return Err(ApiError::new("invalid_arguments", "validation", format!("the upload holds {} of {} bytes", u.bytes.len(), u.length)));
        }
        let digest = sha256_hex(&u.bytes);
        if digest != u.sha256 {
            return Err(ApiError::new("invalid_arguments", "validation", "the bytes do not match the declared sha256"));
        }
        let artifact = Artifact { owner: u.owner, media_type: u.media_type, sha256: digest, bytes: u.bytes, expires: Instant::now() + AGE, leases: 0 };
        let r = reference(id, &artifact);
        map.insert(id.to_string(), artifact);
        Ok(r)
    }

    /// The whole of an artifact for an admitted operation, under a lease
    /// that keeps it past its advertised expiry while the operation runs.
    pub fn lease(&self, owner: &Owner, id: &str) -> Result<(Vec<u8>, String), ApiError> {
        let mut map = self.inner.lock().unwrap();
        let a = Self::get(&mut map, id, owner)?;
        a.leases += 1;
        Ok((a.bytes.clone(), a.media_type.clone()))
    }

    pub fn release(&self, id: &str) {
        if let Some(a) = self.inner.lock().unwrap().get_mut(id) {
            a.leases = a.leases.saturating_sub(1);
        }
    }

    /// The references of every artifact an execution created (its log aside).
    /// Remove every artifact one execution made (its host forgot it).
    pub fn drop_execution(&self, execution: &str) {
        self.inner.lock().unwrap().retain(|_, a| a.leases > 0 || a.owner.execution.as_deref() != Some(execution));
    }

    pub fn owned_by(&self, scope: &str, execution: &str) -> Vec<Value> {
        let map = self.inner.lock().unwrap();
        let mut refs: Vec<(&String, &Artifact)> = map
            .iter()
            .filter(|(_, a)| a.owner.scope == scope && a.owner.execution.as_deref() == Some(execution) && a.media_type != "text/plain")
            .collect();
        refs.sort_by_key(|(_, a)| a.expires);
        refs.into_iter().map(|(id, a)| reference(id, a)).collect()
    }

    pub fn reference_of(&self, owner: &Owner, id: &str) -> Result<Value, ApiError> {
        let mut map = self.inner.lock().unwrap();
        let a = Self::get(&mut map, id, owner)?;
        Ok(reference(id, a))
    }
}

fn reference(id: &str, a: &Artifact) -> Value {
    json!({
        "artifactId": id,
        "uri": format!("brep://artifact/{id}"),
        "chunkUriTemplate": format!("brep://artifact/{id}/chunks/{{offset}}"),
        "mediaType": a.media_type,
        "byteLength": a.bytes.len(),
        "sha256": a.sha256,
        "expiresAt": wire::timestamp(a.expires),
    })
}

