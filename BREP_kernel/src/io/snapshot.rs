//! Native serialized exact-BREP snapshot — the parts-library fast lane.
//!
//! Serializes a set of EVALUATED solids (full topology, exact NURBS geometry,
//! every deterministic face/edge name, and the scene-metadata records stamped
//! against those names) into one base64 string, and restores them exactly.
//! This is the `partsLibrary.snapshot` payload of the assemblies build spec
//! (§2.1/§10.1): inserting a component decodes this straight into resident
//! kernel structs — no sub-part history execution, no STEP-import-style
//! reconstruction or validation gate.
//!
//! **Format.** A byte container framed over the existing [`encode_solid`] flat
//! `f64` codec, base64-wrapped for JSON embedding. Reusing the flat codec
//! (rather than a serde-derive binary layer) keeps the dependency allowlist of
//! the published crate unchanged and pins an EXPLICIT wire layout: a struct
//! field reorder cannot silently change the format, and every `f64` round-trips
//! bit-exact through `to_le_bytes`. Layout (integers u32 little-endian):
//!
//! ```text
//! magic "BREPSNAP" | format version | solid count
//! per solid: name (len+utf8) | SolidNames JSON (len+utf8) | f64 count | f64s (LE)
//! metadata record count
//! per record: entity name (len+utf8) | record JSON (len+utf8)
//! trailer: FNV-1a 64 checksum of every preceding byte
//! ```
//!
//! The checksum makes ANY byte corruption detectable — without it a bit-flip
//! inside a control-point coordinate would decode into silently different
//! geometry instead of the clean `Err` the self-heal lane keys off.
//!
//! **Failure frame.** The snapshot is a CACHE; the embedded part document is
//! the durable source. The format does not survive kernel refactors — it fails
//! DETECTABLY instead: any unreadable, truncated, corrupted, or
//! version-mismatched payload returns a clean `Err` (never a panic), which the
//! ACOMP self-heal lane answers by re-executing the embedded document and
//! re-snapshotting.
//!
//! **…except where the payload IS the document — a DURABILITY commitment.** A
//! natively-imported part (`IMPORT3D` with `inputParams.nativeBrep`) has NO
//! parametric history behind it and does not retain the source file it was read
//! from: this container holds the only copy of that geometry. There is nothing
//! to self-heal FROM, so an unreadable payload there is LOST GEOMETRY, not a
//! slow rebuild. This container and [`crate::SOLID_CODEC_VERSION`] are therefore
//! a **durable format**, not a disposable cache: a version bump must ship a
//! READER for the previous version (or a migration that rewrites old payloads),
//! and a layout change that cannot be read forward is a breaking change to saved
//! user documents. Cheap insurance while the format is at version 1; expensive
//! to retrofit after the first field file. The clean-`Err` contract above still
//! holds — it is the detection mechanism, no longer the whole answer.
//!
//! **Metadata seam.** [`restore_solids`] RETURNS the captured metadata records
//! without stamping them into the scene-metadata store: the ACOMP lane must
//! namespace entity names (`ACOMP2:…`) before calling
//! `scene_metadata::merge_record`, and only it knows the instance prefix.

use crate::topology::BrepSolid;
use crate::{decode_solid, encode_solid, SolidNames};
use std::collections::BTreeMap;

/// Bumped whenever the container layout changes. A mismatch is always a clean
/// `Err` — the self-heal signal for a snapshot whose part document can be
/// re-executed. For a payload that IS the document (`nativeBrep`) that `Err` is
/// unrecoverable, so per the durability commitment in the module doc a bump MUST
/// ship a reader for the previous version, or a migration.
pub const SNAPSHOT_FORMAT_VERSION: u32 = 1;

const MAGIC: &[u8; 8] = b"BREPSNAP";

/// One restored solid: its scene name plus the exact topology/geometry.
#[derive(Debug, Clone)]
pub struct RestoredSolid {
    pub name: String,
    pub solid: BrepSolid,
}

/// The output of [`restore_solids`]: solids in snapshot order plus the
/// captured scene-metadata records (`entity name -> own record`), NOT stamped
/// into the store — see the metadata seam in the module doc.
#[derive(Debug, Clone)]
pub struct RestoredSnapshot {
    pub solids: Vec<RestoredSolid>,
    pub metadata: Vec<(String, serde_json::Map<String, serde_json::Value>)>,
}

// ---------------------------------------------------------------------------
// Snapshot (encode)
// ---------------------------------------------------------------------------

/// Serialize `(name, solid)` pairs into one base64 snapshot payload. Captures
/// each entity's OWN scene-metadata record (solid name + every face/edge name)
/// alongside the geometry, so a stamped part restores with its metadata. The
/// payload is byte-deterministic for identical input (solids in caller order,
/// metadata sorted by name).
pub fn snapshot_solids(solids: &[(&str, &BrepSolid)]) -> Result<String, String> {
    let mut out = Vec::with_capacity(1024);
    out.extend_from_slice(MAGIC);
    push_u32(&mut out, SNAPSHOT_FORMAT_VERSION);
    push_count(&mut out, solids.len())?;
    let mut metadata = BTreeMap::<String, String>::new();
    let mut capture = |name: &str| -> Result<(), String> {
        if let Some(record) = crate::feature_pipeline::scene_metadata::own_record(name) {
            let json = serde_json::to_string(&record)
                .map_err(|error| format!("snapshot: metadata record for '{name}': {error}"))?;
            metadata.insert(name.to_string(), json);
        }
        Ok(())
    };
    for (name, solid) in solids {
        let (data, names) = encode_solid(solid)?;
        let names_json = serde_json::to_string(&names)
            .map_err(|error| format!("snapshot: names for '{name}': {error}"))?;
        push_str(&mut out, name)?;
        push_str(&mut out, &names_json)?;
        push_count(&mut out, data.len())?;
        for value in &data {
            out.extend_from_slice(&value.to_le_bytes());
        }
        capture(name)?;
        for face_name in names.faces.values() {
            capture(face_name)?;
        }
        for edge_name in names.edges.values() {
            capture(edge_name)?;
        }
    }
    push_count(&mut out, metadata.len())?;
    for (name, record_json) in &metadata {
        push_str(&mut out, name)?;
        push_str(&mut out, record_json)?;
    }
    Ok(seal(out))
}

/// Append the container checksum and base64-wrap — the single encode exit.
fn seal(mut container: Vec<u8>) -> String {
    let digest = fnv1a(&container);
    container.extend_from_slice(&digest.to_le_bytes());
    base64_encode(&container)
}

/// FNV-1a 64 over the container bytes — cheap, dependency-free corruption
/// detection (not cryptographic; the payload is a local cache, not an input
/// trust boundary).
fn fnv1a(bytes: &[u8]) -> u64 {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    for &byte in bytes {
        hash ^= byte as u64;
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}

/// [`snapshot_solids`] over RESIDENT solids — the shape the parts-library lane
/// holds after running a sub-part history (`AddedSolid.name` + handle).
pub fn snapshot_resident_solids(named_handles: &[(String, u32)]) -> Result<String, String> {
    let mut owned = Vec::with_capacity(named_handles.len());
    for (name, handle) in named_handles {
        let solid = crate::with_registered_solid_str(*handle, |solid| Ok(solid.clone()))?;
        owned.push((name.as_str(), solid));
    }
    let borrowed: Vec<(&str, &BrepSolid)> = owned
        .iter()
        .map(|(name, solid)| (*name, solid))
        .collect();
    snapshot_solids(&borrowed)
}

// ---------------------------------------------------------------------------
// Restore (decode) — O(payload), straight into kernel structs
// ---------------------------------------------------------------------------

/// Restore a snapshot payload exactly. Every failure mode (bad base64, wrong
/// magic, version mismatch, truncation, corrupted geometry) is a clean `Err`.
pub fn restore_solids(payload: &str) -> Result<RestoredSnapshot, String> {
    let bytes = base64_decode(payload)?;
    // Checksum first: any corruption anywhere fails here with one clear error.
    if bytes.len() < MAGIC.len() + 8 {
        return Err("snapshot: payload too short".into());
    }
    let (body, trailer) = bytes.split_at(bytes.len() - 8);
    let stored = u64::from_le_bytes(trailer.try_into().expect("8-byte trailer"));
    if fnv1a(body) != stored {
        return Err("snapshot: payload checksum mismatch (corrupted)".into());
    }
    let mut reader = ByteReader {
        data: body,
        cursor: 0,
    };
    if reader.take(MAGIC.len())? != MAGIC {
        return Err("snapshot: not a BREP snapshot payload (bad magic)".into());
    }
    let version = reader.u32()?;
    if version != SNAPSHOT_FORMAT_VERSION {
        return Err(format!(
            "snapshot: unsupported format version {version} (expected {SNAPSHOT_FORMAT_VERSION})"
        ));
    }
    let solid_count = reader.u32()? as usize;
    let mut solids = Vec::with_capacity(solid_count.min(1024));
    for _ in 0..solid_count {
        let name = reader.string()?;
        let names_json = reader.string()?;
        let names: SolidNames = serde_json::from_str(&names_json)
            .map_err(|error| format!("snapshot: names for '{name}': {error}"))?;
        let data = reader.f64_vec()?;
        let solid = decode_solid(&data, &names)
            .map_err(|error| format!("snapshot: solid '{name}': {error}"))?;
        solids.push(RestoredSolid { name, solid });
    }
    let record_count = reader.u32()? as usize;
    let mut metadata = Vec::with_capacity(record_count.min(4096));
    for _ in 0..record_count {
        let name = reader.string()?;
        let record_json = reader.string()?;
        let record: serde_json::Map<String, serde_json::Value> =
            serde_json::from_str(&record_json)
                .map_err(|error| format!("snapshot: metadata record for '{name}': {error}"))?;
        metadata.push((name, record));
    }
    if reader.cursor != body.len() {
        return Err("snapshot: trailing bytes after payload".into());
    }
    Ok(RestoredSnapshot { solids, metadata })
}

// ---------------------------------------------------------------------------
// Byte framing
// ---------------------------------------------------------------------------

fn push_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn push_count(out: &mut Vec<u8>, count: usize) -> Result<(), String> {
    let value: u32 = count
        .try_into()
        .map_err(|_| format!("snapshot: count {count} exceeds u32"))?;
    push_u32(out, value);
    Ok(())
}

fn push_str(out: &mut Vec<u8>, text: &str) -> Result<(), String> {
    push_count(out, text.len())?;
    out.extend_from_slice(text.as_bytes());
    Ok(())
}

struct ByteReader<'a> {
    data: &'a [u8],
    cursor: usize,
}

impl<'a> ByteReader<'a> {
    fn take(&mut self, count: usize) -> Result<&'a [u8], String> {
        let end = self
            .cursor
            .checked_add(count)
            .ok_or("snapshot: truncated payload")?;
        let slice = self
            .data
            .get(self.cursor..end)
            .ok_or("snapshot: truncated payload")?;
        self.cursor = end;
        Ok(slice)
    }

    fn u32(&mut self) -> Result<u32, String> {
        let raw = self.take(4)?;
        Ok(u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]))
    }

    fn string(&mut self) -> Result<String, String> {
        let length = self.u32()? as usize;
        let raw = self.take(length)?;
        String::from_utf8(raw.to_vec()).map_err(|_| "snapshot: invalid UTF-8 string".into())
    }

    fn f64_vec(&mut self) -> Result<Vec<f64>, String> {
        let count = self.u32()? as usize;
        let raw = self.take(
            count
                .checked_mul(8)
                .ok_or("snapshot: truncated payload")?,
        )?;
        Ok(raw
            .chunks_exact(8)
            .map(|chunk| {
                f64::from_le_bytes([
                    chunk[0], chunk[1], chunk[2], chunk[3], chunk[4], chunk[5], chunk[6], chunk[7],
                ])
            })
            .collect())
    }
}

// ---------------------------------------------------------------------------
// Base64 (standard alphabet, padded) — kept local: the published crate's
// dependency allowlist stays unchanged, and the decoder must be strict
// (reject stray characters / bad length / misplaced padding, never panic).
// ---------------------------------------------------------------------------

const B64_ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

fn base64_encode(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let b0 = chunk[0] as u32;
        let b1 = chunk.get(1).copied().unwrap_or(0) as u32;
        let b2 = chunk.get(2).copied().unwrap_or(0) as u32;
        let triple = (b0 << 16) | (b1 << 8) | b2;
        out.push(B64_ALPHABET[(triple >> 18) as usize & 63] as char);
        out.push(B64_ALPHABET[(triple >> 12) as usize & 63] as char);
        out.push(if chunk.len() > 1 {
            B64_ALPHABET[(triple >> 6) as usize & 63] as char
        } else {
            '='
        });
        out.push(if chunk.len() > 2 {
            B64_ALPHABET[triple as usize & 63] as char
        } else {
            '='
        });
    }
    out
}

fn base64_value(byte: u8) -> Result<u32, String> {
    match byte {
        b'A'..=b'Z' => Ok((byte - b'A') as u32),
        b'a'..=b'z' => Ok((byte - b'a') as u32 + 26),
        b'0'..=b'9' => Ok((byte - b'0') as u32 + 52),
        b'+' => Ok(62),
        b'/' => Ok(63),
        _ => Err(format!(
            "snapshot: invalid base64 character 0x{byte:02x}"
        )),
    }
}

fn base64_decode(text: &str) -> Result<Vec<u8>, String> {
    let bytes = text.as_bytes();
    if bytes.len() % 4 != 0 {
        return Err("snapshot: base64 length must be a multiple of 4".into());
    }
    let mut out = Vec::with_capacity(bytes.len() / 4 * 3);
    for (index, quad) in bytes.chunks_exact(4).enumerate() {
        let is_last = (index + 1) * 4 == bytes.len();
        // Padding is legal only as the final one or two characters.
        let padding = match (quad[2], quad[3]) {
            (b'=', b'=') if is_last => 2,
            (_, b'=') if is_last && quad[2] != b'=' => 1,
            (b'=', _) => return Err("snapshot: misplaced base64 padding".into()),
            _ => 0,
        };
        if quad[0] == b'=' || quad[1] == b'=' {
            return Err("snapshot: misplaced base64 padding".into());
        }
        let mut triple = base64_value(quad[0])? << 18 | base64_value(quad[1])? << 12;
        if padding < 2 {
            triple |= base64_value(quad[2])? << 6;
        }
        if padding < 1 {
            triple |= base64_value(quad[3])?;
        }
        out.push((triple >> 16) as u8);
        if padding < 2 {
            out.push((triple >> 8) as u8);
        }
        if padding < 1 {
            out.push(triple as u8);
        }
    }
    Ok(out)
}

