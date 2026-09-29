//! Shared three-dimensional vector operations.

pub fn norm3(v: [f64; 3]) -> [f64; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len <= 0.0 {
        return [0.0, 0.0, 1.0];
    }
    [v[0] / len, v[1] / len, v[2] / len]
}

pub fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn add3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn scale3(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub fn len3(a: [f64; 3]) -> f64 {
    dot3(a, a).sqrt()
}

/// Rotate `v` around unit `axis` by `angle` (Rodrigues).
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
