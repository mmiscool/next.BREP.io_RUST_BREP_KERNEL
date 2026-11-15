//! Switches of the carried-uv section march, read by pcurve, projection and imprint.


/// Trace-carried UV sections at the pcurve accuracy floor. This is independent
/// of the older, looser-budget Hermite experiment. Disable both UV switches
/// to retain fitting; reuse never changes the march budget.
pub fn carried_uv_enabled() -> bool {
    switch!("BREP_MARCH_UV_SECTIONS", true) || emission_refine_enabled()
}

/// Refine only the emission samples after clipping, never the original march.
pub fn emission_refine_enabled() -> bool {
    switch!("BREP_MARCH_UV_REFINE", true)
}

/// Decide eligibility before retaining metadata or entering the large emitter.
pub fn carried_uv_min_stations() -> usize {
    if emission_refine_enabled() { 32 } else { 2 }
}
