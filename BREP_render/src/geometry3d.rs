//! Shared three-dimensional vector operations.

#[inline]
pub fn norm3(v: [f64; 3]) -> [f64; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len <= 0.0 {
        return [0.0, 0.0, 1.0];
    }
    [v[0] / len, v[1] / len, v[2] / len]
}

#[inline]
pub fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[inline]
pub fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
pub fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
pub fn add3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[inline]
pub fn scale3(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

#[inline]
pub fn len3(a: [f64; 3]) -> f64 {
    dot3(a, a).sqrt()
}

/// Rotate `v` around unit `axis` by `angle` (Rodrigues).
#[inline]
pub fn rotate3(v: [f64; 3], axis: [f64; 3], angle: f64) -> [f64; 3] {
    let (sin, cos) = angle.sin_cos();
    let cross = cross3(axis, v);
    let dot = dot3(axis, v);
    [
        v[0] * cos + cross[0] * sin + axis[0] * dot * (1.0 - cos),
        v[1] * cos + cross[1] * sin + axis[1] * dot * (1.0 - cos),
        v[2] * cos + cross[2] * sin + axis[2] * dot * (1.0 - cos),
    ]
}

/// Normalize a finite vector whose length exceeds 1e-12.
#[inline]
pub fn normalize3(v: [f64; 3]) -> Option<[f64; 3]> {
    let len = len3(v);
    (len > 1e-12 && len.is_finite()).then(|| [v[0] / len, v[1] / len, v[2] / len])
}

/// Normalize or return the caller's fallback unchanged.
#[inline]
pub fn normalize3_or(v: [f64; 3], fallback: [f64; 3]) -> [f64; 3] {
    normalize3(v).unwrap_or(fallback)
}

/// Row-major matrix composition. Valid affine results use the kernel's arithmetic.
/// Keep the raw result for singular/projective/nonfinite display matrices: these
/// callers historically accepted them, unlike `AffineTransform::compose`.
pub fn mat4_mul_row_major(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    use brep_kernel::AffineTransform;
    (AffineTransform { elements: *a }).compose(&AffineTransform { elements: *b })
        .map(|transform| transform.elements)
        .unwrap_or_else(|_| std::array::from_fn(|k| {
            let (row, col) = (k / 4, k % 4);
            (0..4).map(|n| a[row * 4 + n] * b[n * 4 + col]).sum()
        }))
}

/// Transpose the rotation and invert its translation, preserving the raw result
/// even when the kernel's affine-result validator refuses the display matrix.
/// As with the kernel helper, the caller is responsible for rigidity.
pub fn mat4_rigid_inverse(m: &[f64; 16]) -> [f64; 16] {
    (brep_kernel::AffineTransform { elements: *m }).rigid_inverse()
        .map(|transform| transform.elements)
        .unwrap_or_else(|_| {
            let mut out = [0.0; 16];
            for row in 0..3 {
                for col in 0..3 {
                    out[row * 4 + col] = m[col * 4 + row];
                }
                out[row * 4 + 3] = -(0..3).map(|k| m[k * 4 + row] * m[k * 4 + 3]).sum::<f64>();
            }
            out[15] = 1.0;
            out
        })
}

