//! Numerical policy owned by the kernel adapter and STEP validation layer.

pub(crate) mod scalar {
    pub(crate) const GEOMETRIC_SCALE_FLOOR: f64 = 1.0e-12;
}

pub(crate) mod brep {
    pub(crate) const REVOLUTION_EQUAL_RADIUS_RELATIVE: f64 = 1.0e-9;
    pub(crate) const REVOLUTION_MIN_ABSOLUTE_HEIGHT: f64 = f64::EPSILON;
}

pub(crate) mod step_validation {
    pub(crate) const COORDINATE_ROUNDOFF_RELATIVE: f64 = 32_768.0 * f64::EPSILON;
    pub(crate) const TESSELLATED_COORDINATE_EVIDENCE_ROUNDOFF_RELATIVE: f64 =
        1_048_576.0 * f64::EPSILON;
    pub(crate) const CONE_APEX_ROUNDOFF_RELATIVE: f64 = 4.0 * f64::EPSILON;
    pub(crate) const NORMAL_EQUIVALENCE_RADIANS: f64 = 1.28e-5;
    pub(crate) const INITIAL_PERTURBATION_RELATIVE: f64 = 1.0e-3;
    pub(crate) const FALLBACK_CHORD_SOLID_RELATIVE: f64 = 1.0e-3;
    pub(crate) const FALLBACK_CHORD_MINOR_RADIUS_RELATIVE: f64 = 5.0e-2;
    pub(crate) const FALLBACK_CHORD_RELATIVE_FLOOR: f64 = 1.0e-8;
}
