//! Human-readable part and revision identities. Escaping is transport syntax,
//! never a lookup through a second identifier.

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

/// The store route for the document `key` (`part/<p>/rev/<r>`, already
/// segmented). The server percent-decodes the path once before splitting it,
/// so each `%` of the segmented key is escaped again: `M3/A2` is segmented to
/// `M3%2FA2` and sent as `M3%252FA2`, which decodes back to the key. Every
/// `/api/store/doc` request goes through here.
pub fn doc_path(key: &str) -> String {
    format!("/api/store/doc/{}", key.trim_start_matches('/').replace('%', "%25"))
}

pub fn part_path(part: &str) -> String {
    format!("/api/parts/{}", segment(part))
}
pub fn revision_path(part: &str, revision: &str) -> String {
    format!("{}/revisions/{}", part_path(part), segment(revision))
}
