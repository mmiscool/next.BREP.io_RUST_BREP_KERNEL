//! Plane-space and screen-space distance calculations.

pub(crate) fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
}

/// Return distance and the closest segment parameter, clamped to `[0, 1]`.
/// Segments with squared length at most `1e-18` collapse to their first endpoint.
pub(crate) fn point_segment_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 <= 1e-18 {
        0.0
    } else {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0)
    };
    (distance(p, (a.0 + t * dx, a.1 + t * dy)), t)
}

