//! Human-readable part and revision identities. Escaping is transport syntax,
//! never a lookup through a second identifier.

use std::path::PathBuf;

/// Encode one identifier as a URL/file segment, including dots so labels such
/// as `..` cannot escape a model directory or lose a suffix.
pub fn segment(value: &str) -> String {
    let mut out = String::new();
    for byte in value.bytes() {
        if byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_') {
            out.push(byte as char);
        } else {
            use std::fmt::Write;
            write!(&mut out, "%{byte:02X}").unwrap();
        }
    }
    out
}

pub fn unsegment(value: &str) -> Option<String> {
    let mut out = Vec::new();
    let bytes = value.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'%' {
            let hex = std::str::from_utf8(bytes.get(i + 1..i + 3)?).ok()?;
            out.push(u8::from_str_radix(hex, 16).ok()?);
            i += 3;
        } else {
            out.push(bytes[i]);
            i += 1;
        }
    }
    String::from_utf8(out).ok()
}

pub fn document_key(part: &str, revision: &str) -> String {
    format!("part/{}/rev/{}", segment(part), segment(revision))
}

pub fn split_key(key: &str) -> Option<(String, String)> {
    let bits: Vec<_> = key.split('/').collect();
    let ["part", part, "rev", revision] = bits.as_slice() else {
        return None;
    };
    let (part, revision) = (unsegment(part)?, unsegment(revision)?);
    crate::db::check_number(&part).ok()?;
    crate::db::check_label(&revision).ok()?;
    Some((part, revision))
}

pub fn model_path(part: &str, revision: &str) -> PathBuf {
    PathBuf::from("models")
        .join(segment(part))
        .join(format!("{}.json", segment(revision)))
}
