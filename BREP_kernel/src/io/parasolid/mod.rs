//! Parasolid XT (transmit file) import.
//!
//! Written from the published *Parasolid XT Format Reference*; section
//! numbers in this module are that reference's (July 2022 edition).
//!
//! * [`parse_xt_text`] reads a text part (`.x_t`, `.xmt_txt`) into its node
//!   graph: the header, the token stream, and each node laid out by the base
//!   schema (`schema`) as edited by the file's embedded schema deltas.
//!   [`parse_xt`] reads the same from bytes, text or binary (`.x_b`, neutral
//!   or typed), part or partition (the form SolidWorks embeds bodies in).
//! * [`import_parasolid`], [`import_parasolid_report`] and
//!   [`import_parasolid_bytes_report`] read its bodies
//!   into solids by translating each to Part 21 for the STEP importer
//!   (`to_step`), with names and colours.

mod check;
mod geometry;
mod import;
mod reader;
mod schema;
mod to_step;
mod to_xt;
mod writer;


pub use import::{import_parasolid, import_parasolid_bodies, import_parasolid_bytes_bodies, import_parasolid_bytes_report, import_parasolid_report, ParasolidImportReport};
pub use reader::{parse_xt, parse_xt_text, Node, NodeLayout, Value, XtFile};
pub use check::check_xt_body;
pub use to_xt::{export_parasolid_binary, export_parasolid_report, export_parasolid_text, XtExportBody, XtExportReport};
pub use writer::{write_xt_binary_part_data, write_xt_part_data};
pub use schema::{Count as XtFieldCount, FieldDef as XtFieldDef, Kind as XtFieldKind};

/// Whether `text` is a Parasolid text part: it starts with the transmit
/// header's preamble (§3.1), or, written without a header, with the part
/// data's `T<length> : TRANSMIT FILE` banner (§3.2). Only the start counts,
/// so a STEP file that mentions Parasolid in its own header is not one.
pub fn is_xt_text(text: &str) -> bool {
    let start = text.trim_start_matches(['\u{feff}', ' ', '\t', '\r', '\n']);
    if start.starts_with("**ABCDEFGHIJKLMNOPQRSTUVWXYZ") {
        return start.lines().nth(1).is_some_and(|line| line.starts_with("**PARASOLID"));
    }
    let Some(rest) = start.strip_prefix('T') else { return false };
    let digits = rest.bytes().take_while(u8::is_ascii_digit).count();
    digits > 0 && rest[digits..].trim_start().starts_with(": TRANSMIT FILE")
}

/// Whether `bytes` are a binary Parasolid file: the neutral or typed binary
/// flag (`PS\0\0`, `PS\0\1`), at the start or after the keyword header
/// (§3.1, §3.3).
pub fn is_xt_binary(bytes: &[u8]) -> bool {
    let body = match bytes.windows(15).position(|w| w == b"**END_OF_HEADER") {
        Some(end) if bytes.starts_with(b"**ABCDEFGHIJKLMNOPQRSTUVWXYZ") => {
            match bytes[end..].iter().position(|&b| b == b'\n') {
                Some(n) => &bytes[end + n + 1..],
                None => return false,
            }
        }
        _ => bytes,
    };
    matches!(body, [b'P', b'S', 0, 0 | 1, ..])
}
