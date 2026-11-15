//! Parasolid XT (transmit file) import.
//!
//! Written from the published *Parasolid XT Format Reference*; section
//! numbers in this module are that reference's (July 2022 edition).
//!
//! * [`parse_xt_text`] reads a text part (`.x_t`, `.xmt_txt`) into its node
//!   graph: the header, the token stream, and each node laid out by the base
//!   schema (`schema`) as edited by the file's embedded schema deltas.
//! * [`import_parasolid`] and [`import_parasolid_report`] read its bodies
//!   into solids by translating each to Part 21 for the STEP importer
//!   (`to_step`), with names and colours.

mod geometry;
mod import;
mod reader;
mod schema;
mod to_step;


pub use import::{import_parasolid, import_parasolid_report, ParasolidImportReport};
pub use reader::{parse_xt_text, Node, NodeLayout, Value, XtFile};
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
