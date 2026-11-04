//! The product's recognition switches (the STL conversion's recognition
//! options and the freeform split). Lane S's `BREP_RANSAC_ADAPTIVE` was
//! removed for BREP_RANSAC 0.2.0: its census pair favoured the classical
//! policy, which keeps the adaptive policy out of the recognizer's public API.

/// Lane U row 3, promoted ON 2026-10-29: `RecognitionOptions::torus_fast` at
/// the STL conversion's recognition and the freeform split (output identical
/// on every sweep and census row, census recognition 2.7x faster).
/// `BREP_RANSAC_TORUS_FAST=0` (exactly) restores the old fit. The
/// recognizer's own default stays off, so other callers (the kernel's face
/// refit among them) are unchanged.
pub(crate) fn torus_fast() -> bool {
    parse_torus_fast(std::env::var("BREP_RANSAC_TORUS_FAST").ok().as_deref())
}

/// On unless the switch's value is exactly "0".
fn parse_torus_fast(value: Option<&str>) -> bool {
    value != Some("0")
}

