//! An edge proved clear of a carrier before the general pierce search starts
//! (`BREP_IMPRINT_PIERCE_CLEAR`, default on since its promotion; `=0` restores the search alone).
//!
//! The general search seeds a Newton at every sample chord of the edge whose box meets one of the
//! carrier's span boxes, and each Newton opens with a global projection onto the carrier. Where an
//! edge runs alongside a freeform carrier at a distance, the span boxes are loose enough to pass
//! hundreds of chords, and every Newton ends far outside the tolerance: the search returns nothing,
//! at the price of hundreds of global projections. The proof (`crate::curve_clear`) shows the
//! answer is empty without it; where it cannot, the search runs as before.

/// `BREP_IMPRINT_PIERCE_CLEAR` (default on; `=0` off): prove an edge clear of a carrier before the general
/// pierce search.
pub(super) fn pierce_clear_enabled() -> bool {
    switch!("BREP_IMPRINT_PIERCE_CLEAR", true)
}

