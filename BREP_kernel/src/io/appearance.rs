//! Imported appearance — the display colour an exchange format carries, in the
//! kernel's own terms.
//!
//! **Where colour LIVES.** The kernel has no colour field on `BrepSolid`, and it
//! does not need one: colour is scene metadata, keyed by the face/solid NAME in
//! `feature_pipeline::scene_metadata`. This module is only the CARRIER between
//! "the importer read a colour" and "the pipeline stamped it on a name" — it is
//! deliberately not a storage layer, and nothing here is serialized.
//!
//! **The record shape (THE convention, one spelling everywhere).** A colour is
//! stamped as a single string attribute on the entity's own metadata record:
//!
//! ```text
//! { "color": "#RRGGBB" }
//! ```
//!
//! * key [`COLOR_METADATA_KEY`] — `color`, US spelling, matching
//!   `BREP_render/src/color.rs` and the caller-side colour params.
//! * value — uppercase `#RRGGBB` sRGB hex, the form the info/metadata panel shows
//!   and a human can type back. [`ImportedColor::to_hex`] is the ONE producer.
//!
//! The record rides on the FINAL, stamped name, so it is captured by
//! `io/snapshot.rs` into the native IMPORT3D payload and restored with the
//! geometry — an imported colour survives save/reload and the parts library for
//! free.
//!
//! **Precedence.** A face colour is stamped on the FACE name; a body colour on
//! the BODY name only. A body colour is never fanned out onto its faces: the
//! solid's own info window already surfaces it, and materializing it per face
//! would make the face records claim a styling the file never authored.
//!
//! **Units.** Exchange formats give components as 0..1 doubles with no gamma
//! statement; OCC, FreeCAD and every viewer that reads them treat the value as
//! sRGB and scale it straight to 8 bits, so `round(c * 255)` is the conversion —
//! no linear/sRGB transfer applied.

/// The scene-metadata key an imported colour is stamped under. ONE spelling.
pub const COLOR_METADATA_KEY: &str = "color";

/// An imported display colour: sRGB components in 0..=1, kept as read.
///
/// The float form is preserved (rather than collapsing to 8-bit at read time) so
/// a later viewport lane can use the exact authored value; the metadata record
/// carries the [`Self::to_hex`] rendering of it.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ImportedColor {
    pub r: f64,
    pub g: f64,
    pub b: f64,
}

impl ImportedColor {
    /// Clamp to the 0..=1 the format promises. A non-finite component reads as
    /// 0 rather than poisoning the hex conversion.
    pub fn new(r: f64, g: f64, b: f64) -> Self {
        Self {
            r: clamp_unit(r),
            g: clamp_unit(g),
            b: clamp_unit(b),
        }
    }

    /// Uppercase `#RRGGBB` — the ONE rendering that reaches a metadata record.
    pub fn to_hex(&self) -> String {
        format!(
            "#{:02X}{:02X}{:02X}",
            channel(self.r),
            channel(self.g),
            channel(self.b)
        )
    }
}

fn clamp_unit(value: f64) -> f64 {
    if value.is_finite() {
        value.clamp(0.0, 1.0)
    } else {
        0.0
    }
}

fn channel(value: f64) -> u8 {
    (clamp_unit(value) * 255.0).round() as u8
}

/// One imported body's appearance: an optional body-wide colour plus an optional
/// colour per FACE.
///
/// `faces` is POSITIONAL and parallel to the body's faces in shell/face order —
/// the same walk `feature_pipeline::features::import3d::stamp_imported_names`
/// uses — so index `i` is the i-th face of `solid.shells.iter().flat_map(faces)`.
/// It is either empty (nothing read) or exactly as long as that face count; a
/// producer that cannot guarantee the pairing must leave it EMPTY rather than
/// emit a shorter or speculative list.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BodyAppearance {
    /// The whole body's colour, when the file styled the solid itself.
    pub body: Option<ImportedColor>,
    /// Per-face colour in shell/face order, or empty when none was read.
    pub faces: Vec<Option<ImportedColor>>,
}

impl BodyAppearance {
    /// Nothing to stamp: no body colour and no face carries one.
    pub fn is_empty(&self) -> bool {
        self.body.is_none() && self.faces.iter().all(Option::is_none)
    }
}

