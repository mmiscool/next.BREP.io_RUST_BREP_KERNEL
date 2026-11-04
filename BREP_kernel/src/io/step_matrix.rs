//! Matrix composition shared by STEP import and export.

/// A row-major 4×4 matrix used for STEP assembly placements.
pub type Mat4 = [f64; 16];

pub(crate) const MAT4_IDENTITY: Mat4 = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

pub(crate) fn mat4_mul(a: &Mat4, b: &Mat4) -> Mat4 {
    crate::transform_topology::multiply_elements(a, b)
}

